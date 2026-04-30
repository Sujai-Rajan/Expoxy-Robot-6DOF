"""
Dispensing Cell HMI — uFactory xArm 850 / DP160 Epoxy Applicator
=================================================================
SDK optimisations applied
  • Single get_cgpio_state() call per poll tick (replaces 7 individual reads).
  • SDK push-report callbacks (state / mode / error+warn / connect) update a
    thread-safe snapshot; read_* methods read the snapshot instead of making
    blocking RPC calls.
  • arm.state / arm.mode attributes (kept fresh by SDK background thread) used
    as fallback when callback data is not yet populated.
  • clean_error() + clean_warn() + motion_enable() called before every reset so
    the controller can recover from fault states cleanly.
  • Graceful disconnect on application close.

Wiring
------
CI0  E-Stop          → handled by xArm firmware (WebUI input config)
CI1  Manual Mode     → handled by xArm firmware; reflected via mode_code
CI2  Tube Change     → run tube_change recipe (operator presses to swap epoxy)
CI3  (not wired)
CI4  Start / Resume  → active high
CI5  Pause           → active high
CI6  Stop            → active high
CI7  Reset to Home   → active high
DI0  Board Present   → photoelectric sensor
DI1  Cartridge Empty → (disabled, ENABLE_CARTRIDGE_CHECK = False)
DI4  Light Curtain   → LIGHT_CURTAIN_BROKEN_VALUE = 0 (NC sensor, 0 = broken)
CO7  Dispenser Trigger
CO0-CO6 firmware-driven status outputs (configured in xArm WebUI)
"""

import sys
import time
import importlib.util
import threading
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QLinearGradient, QGuiApplication
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
    QSizePolicy,
)

BASE_DIR = Path(__file__).resolve().parent
RECIPES_DIR = BASE_DIR / "recipes"

# Locate the xArm Python SDK — check both possible install paths.
for _sdk_candidate in (
    BASE_DIR / "xArm-Python-SDK",
    BASE_DIR / "Dependencies" / "xArm-Python-SDK-master",
):
    if _sdk_candidate.exists():
        _sdk_str = str(_sdk_candidate)
        if _sdk_str not in sys.path:
            sys.path.insert(0, _sdk_str)
        break

# ---------------------------------------------------------------------------
# xArm CGPIO channel mapping
# CI0-CI7  => channels  0-7   (digital inputs, controller side)
# DI0-DI7  => channels  8-15  (digital inputs, tool / expansion side)
# CO0-CO7  => channels  0-7   (digital outputs, controller side)
# get_cgpio_state() states[2] bitmask: bit i = channel i state (0-15)
# ---------------------------------------------------------------------------

# Operator hardware buttons (active-high, momentary NO unless noted)
PHYSICAL_START_CI   = 4   # CI4  Start / Resume
PAUSE_BUTTON_CI     = 5   # CI5  Pause
STOP_BUTTON_CI      = 6   # CI6  Stop
RESET_BUTTON_CI     = 7   # CI7  Reset to home
TUBE_CHANGE_BUTTON_CI = 2 # CI2  Tube change (swap epoxy cartridge)

# Sensors
BOARD_PRESENT_DI    = 0   # DI0  Board present photoelectric sensor
CARTRIDGE_EMPTY_DI  = 1   # DI1  Cartridge empty (currently disabled)
LIGHT_CURTAIN_DI    = 4   # DI4  Safety light curtain  (NC, 0 = broken)
LIGHT_CURTAIN_BROKEN_VALUE = 0  # raw sensor value that means "beam broken"

# Output — driven by recipes and our dispenser helpers
DISPENSER_TRIGGER_CO = 7  # CO7  Epoxy dispenser solenoid

# Recipe file stems
MANUAL_DISPENSE_PROGRAM = "purge"
RESET_PROGRAM           = "reset"
TUBE_CHANGE_PROGRAM     = "tube_change"

# Misc config
ENABLE_CARTRIDGE_CHECK    = False
DEFAULT_RECIPE            = "PCB_466"
XARM_IP                   = "10.40.17.196"
RESET_ENABLE_WAIT_SECONDS = 5
RESET_HOME_TIMEOUT_SECONDS = 7
POLL_INTERVAL_MS          = 300  # QTimer period; change here if tuning


# ---------------------------------------------------------------------------
# Shared state dataclass (written from multiple threads; simple fields only)
# ---------------------------------------------------------------------------

@dataclass
class SensorState:
    board_present:        bool  = False
    cartridge_empty:      bool  = False
    manual_mode:          bool  = False
    light_curtain_broken: bool  = False
    light_curtain_paused: bool  = False   # paused specifically by curtain break
    dispenser_on:         bool  = False
    robot_running:        bool  = False
    robot_paused:         bool  = False
    robot_stopped:        bool  = False
    robot_resetting:      bool  = False
    cycle_count:          int   = 0
    last_cycle_seconds:   float = 0.0


# ---------------------------------------------------------------------------
# OutputTrackingArmProxy
# Wraps the raw XArmAPI object passed into recipes so that any CO7 write
# is reflected in the controller's output_states / state.dispenser_on.
# ---------------------------------------------------------------------------

class OutputTrackingArmProxy:
    """Thin proxy around XArmAPI; tracks CO7 (dispenser) state."""

    def __init__(self, arm, controller: "RobotController"):
        object.__setattr__(self, "_arm", arm)
        object.__setattr__(self, "_controller", controller)

    def __getattr__(self, name):
        return getattr(self._arm, name)

    def __setattr__(self, name, value):
        setattr(self._arm, name, value)

    def set_cgpio_digital(self, ionum, value, *args, **kwargs):
        code = self._arm.set_cgpio_digital(ionum, value, *args, **kwargs)
        if code == 0:
            ctrl = self._controller
            ctrl.output_states[int(ionum)] = bool(value)
            if int(ionum) == DISPENSER_TRIGGER_CO:
                ctrl.state.dispenser_on = bool(value)
        return code


# ---------------------------------------------------------------------------
# RobotController
# ---------------------------------------------------------------------------

class RobotController:
    """
    Thread-safe interface to the xArm.

    SDK push-report callbacks keep _cb_snapshot up-to-date in real time.
    A single get_cgpio_state() call per poll tick populates _gpio_cache for
    all 16 digital-input channels so individual read_ci / read_di methods
    return cached values without extra network round-trips.
    """

    def __init__(self):
        self.state = SensorState()
        self.current_recipe:        Optional[str]  = None
        self.current_recipe_module                 = None
        self.program_cycle_counts:  dict           = {}
        self.program_last_cycle_seconds: dict      = {}
        self.arm                                   = None
        self.recipe_thread:   Optional[threading.Thread] = None
        self.reset_thread:    Optional[threading.Thread] = None
        self.last_reset_error: Optional[str]       = None
        self.cycle_abort_requested: bool           = False
        self.output_states: dict = {DISPENSER_TRIGGER_CO: False}

        # SDK push-report callback cache (written from SDK thread, read from GUI thread)
        self._cb_lock     = threading.Lock()
        self._cb_snapshot: dict = {}  # keys: 'state', 'mode', 'error_code', 'warn_code', 'connected'

        # GPIO bulk-read cache (refreshed once per poll tick)
        self._gpio_cache: dict = {}   # {channel(0-15): bool}

        self.connect_robot()

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    def connect_robot(self) -> None:
        try:
            from xarm.wrapper import XArmAPI
            self.arm = XArmAPI(XARM_IP)
            self._register_callbacks()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
        except Exception as exc:
            print(f"Robot connection failed: {exc}")
            self.arm = None

    def disconnect_robot(self) -> None:
        if self.arm is not None:
            try:
                self.dispenser_off_immediate()
                self.arm.disconnect()
            except Exception as exc:
                print(f"Disconnect error: {exc}")
            self.arm = None

    # ------------------------------------------------------------------
    # SDK callbacks (called from SDK background thread)
    # ------------------------------------------------------------------

    def _register_callbacks(self) -> None:
        if self.arm is None:
            return
        self.arm.register_connect_changed_callback(self._on_connect_changed)
        self.arm.register_state_changed_callback(self._on_state_changed)
        self.arm.register_mode_changed_callback(self._on_mode_changed)
        self.arm.register_error_warn_changed_callback(self._on_error_warn_changed)

    def _on_connect_changed(self, data: dict) -> None:
        with self._cb_lock:
            self._cb_snapshot["connected"] = data.get("connected", False)
        if not data.get("connected", True):
            print("xArm disconnected (push report)")

    def _on_state_changed(self, data: dict) -> None:
        with self._cb_lock:
            self._cb_snapshot["state"] = data.get("state")

    def _on_mode_changed(self, data: dict) -> None:
        with self._cb_lock:
            self._cb_snapshot["mode"] = data.get("mode")

    def _on_error_warn_changed(self, data: dict) -> None:
        with self._cb_lock:
            self._cb_snapshot["error_code"] = data.get("error_code", 0)
            self._cb_snapshot["warn_code"]  = data.get("warn_code",  0)

    # ------------------------------------------------------------------
    # Bulk GPIO read  (one SDK call refreshes all 16 channels)
    # ------------------------------------------------------------------

    def refresh_gpio_cache(self) -> None:
        """
        Populate _gpio_cache with a single get_cgpio_state() call.
        states[2] is a 16-bit integer: bit i = state of CGPIO channel i.
        CI0-CI7 = channels 0-7, DI0-DI7 = channels 8-15.
        Call this once per poll tick before reading any CI/DI.
        """
        if self.arm is None:
            self._gpio_cache = {}
            return
        try:
            result = self.arm.get_cgpio_state()
            # SDK returns (code, states) or just states depending on wrapper version
            if isinstance(result, (list, tuple)) and len(result) >= 2:
                code, states = result[0], result[1]
            else:
                self._gpio_cache = {}
                return
            if code != 0 or not isinstance(states, (list, tuple)) or len(states) < 3:
                self._gpio_cache = {}
                return
            inp_word = int(states[2])
            self._gpio_cache = {ch: bool((inp_word >> ch) & 1) for ch in range(16)}
        except Exception as exc:
            print(f"get_cgpio_state failed: {exc}")
            self._gpio_cache = {}

    @staticmethod
    def ci_channel(index: int) -> int:
        return index           # CI0-CI7 = channels 0-7

    @staticmethod
    def di_channel(index: int) -> int:
        return 8 + index       # DI0-DI7 = channels 8-15

    def _read_cached(self, channel: int) -> bool:
        return self._gpio_cache.get(channel, False)

    def read_ci(self, index: int) -> bool:
        return self._read_cached(self.ci_channel(index))

    def read_di(self, index: int) -> bool:
        return self._read_cached(self.di_channel(index))

    # ------------------------------------------------------------------
    # Named sensor reads (all use cache after refresh_gpio_cache())
    # ------------------------------------------------------------------

    def read_board_present(self) -> bool:
        return self.read_di(BOARD_PRESENT_DI)

    def read_cartridge_empty(self) -> bool:
        return self.read_di(CARTRIDGE_EMPTY_DI)

    def read_physical_start(self) -> bool:
        return self.read_ci(PHYSICAL_START_CI)

    def read_pause_button(self) -> bool:
        return self.read_ci(PAUSE_BUTTON_CI)

    def read_stop_button(self) -> bool:
        return self.read_ci(STOP_BUTTON_CI)

    def read_reset_button(self) -> bool:
        return self.read_ci(RESET_BUTTON_CI)

    def read_tube_change_button(self) -> bool:
        return self.read_ci(TUBE_CHANGE_BUTTON_CI)

    def read_light_curtain(self) -> bool:
        """Returns True when the light curtain beam is broken."""
        raw = self.read_di(LIGHT_CURTAIN_DI)
        broken = (raw == bool(LIGHT_CURTAIN_BROKEN_VALUE))
        self.state.light_curtain_broken = broken
        return broken

    def read_manual_mode(self) -> bool:
        manual = self.read_robot_mode_code() == 2
        self.state.manual_mode = manual
        return manual

    # ------------------------------------------------------------------
    # Robot state / mode / error — read from callback snapshot (no RPC)
    # ------------------------------------------------------------------

    def _cb_get(self, key):
        with self._cb_lock:
            return self._cb_snapshot.get(key)

    def read_robot_state_code(self) -> Optional[int]:
        if self.arm is None:
            if self.state.robot_resetting or self.state.robot_stopped:
                return 4
            if self.state.robot_paused:
                return 3
            if self.state.robot_running:
                return 1
            return 2
        val = self._cb_get("state")
        if val is None:
            val = getattr(self.arm, "state", None)
        return int(val) if isinstance(val, (int, bool)) else None

    def read_robot_mode_code(self) -> Optional[int]:
        if self.arm is None:
            return 2 if self.state.manual_mode else 0
        val = self._cb_get("mode")
        if val is None:
            val = getattr(self.arm, "mode", None)
        return int(val) if isinstance(val, (int, bool)) else None

    def read_warning_code(self) -> int:
        val = self._cb_get("warn_code")
        if val is None:
            val = getattr(self.arm, "warn_code", 0) if self.arm else 0
        return int(val) if isinstance(val, (int, bool)) else 0

    def read_error_code(self) -> int:
        val = self._cb_get("error_code")
        if val is None:
            val = getattr(self.arm, "error_code", 0) if self.arm else 0
        return int(val) if isinstance(val, (int, bool)) else 0

    def is_dispenser_on(self) -> bool:
        return bool(self.output_states.get(DISPENSER_TRIGGER_CO, False))

    # ------------------------------------------------------------------
    # Output control
    # ------------------------------------------------------------------

    def set_co(self, index: int, state: bool, sync: bool = True) -> None:
        channel = index  # CO channels share indices with CGPIO channels
        if self.arm is None:
            self.output_states[index] = bool(state)
            if index == DISPENSER_TRIGGER_CO:
                self.state.dispenser_on = bool(state)
            return
        try:
            code = self.arm.set_cgpio_digital(channel, int(state), delay_sec=0, sync=sync)
            if code != 0:
                print(f"set_co CO{index} failed, code={code}")
                return
            self.output_states[index] = bool(state)
            if index == DISPENSER_TRIGGER_CO:
                self.state.dispenser_on = bool(state)
        except Exception as exc:
            print(f"set_co CO{index} exception: {exc}")

    def dispenser_off(self) -> None:
        self.set_co(DISPENSER_TRIGGER_CO, False, sync=True)

    def dispenser_off_immediate(self) -> None:
        self.set_co(DISPENSER_TRIGGER_CO, False, sync=False)

    # ------------------------------------------------------------------
    # Recipe loading and execution
    # ------------------------------------------------------------------

    def load_recipe(self, recipe_name: str, recipe_path: Path) -> None:
        spec = importlib.util.spec_from_file_location(recipe_name, recipe_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Could not load recipe: {recipe_path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.current_recipe        = recipe_name
        self.current_recipe_module = module
        self.program_cycle_counts.setdefault(recipe_name, 0)
        self.program_last_cycle_seconds.setdefault(recipe_name, 0.0)

    def get_program_cycle_count(self, recipe_name: Optional[str]) -> int:
        return self.program_cycle_counts.get(recipe_name, 0) if recipe_name else 0

    def get_program_last_cycle_seconds(self, recipe_name: Optional[str]) -> float:
        return self.program_last_cycle_seconds.get(recipe_name, 0.0) if recipe_name else 0.0

    def _run_loaded_recipe(self) -> None:
        self._execute_loaded_program(count_cycle=True)
        self.state.robot_running = False
        self.recipe_thread = None

    def _execute_loaded_program(self, count_cycle: bool = True) -> None:
        start_time = time.perf_counter()
        try:
            module      = self.current_recipe_module
            recipe_name = self.current_recipe
            if module is None:
                raise RuntimeError("No recipe module loaded.")
            if hasattr(module, "RobotMain"):
                proxy = OutputTrackingArmProxy(self.arm, self) if self.arm is not None else None
                module.RobotMain(proxy).run()
            elif hasattr(module, "run"):
                module.run(self)
            else:
                raise RuntimeError("Recipe has neither RobotMain nor run().")
            if not self.cycle_abort_requested and recipe_name:
                elapsed = time.perf_counter() - start_time
                self.state.last_cycle_seconds = elapsed
                self.program_last_cycle_seconds[recipe_name] = elapsed
                self.program_cycle_counts[recipe_name] = \
                    self.program_cycle_counts.get(recipe_name, 0) + 1
                if count_cycle:
                    self.state.cycle_count += 1
        except Exception as exc:
            print(f"Recipe run failed: {exc}")
        finally:
            self.dispenser_off()

    def _cycle_active(self) -> bool:
        return self.recipe_thread is not None and self.recipe_thread.is_alive()

    def _reset_active(self) -> bool:
        return self.reset_thread is not None and self.reset_thread.is_alive()

    def _terminate_active_cycle(self, join_timeout: float = 5.0) -> None:
        active_thread = self.recipe_thread
        if active_thread is None or not active_thread.is_alive():
            self.recipe_thread = None
            return
        if self.arm is not None:
            try:
                self.arm.set_state(0)
                time.sleep(0.05)
                self.dispenser_off_immediate()
                self.arm.set_state(4)
            except Exception as exc:
                print(f"_terminate_active_cycle arm commands failed: {exc}")
        else:
            self.dispenser_off()
        active_thread.join(timeout=join_timeout)
        if active_thread.is_alive():
            raise RuntimeError("Timed out waiting for active recipe to stop.")
        self.recipe_thread = None

    # ------------------------------------------------------------------
    # Cycle control
    # ------------------------------------------------------------------

    def start_cycle(self) -> None:
        if self.state.robot_paused and self._cycle_active():
            # Resume a paused cycle
            if self.arm is not None:
                try:
                    self.arm.set_state(0)
                except Exception as exc:
                    raise RuntimeError(f"Resume command failed: {exc}") from exc
            self.last_reset_error       = None
            self.cycle_abort_requested  = False
            self.state.robot_running    = True
            self.state.robot_paused     = False
            self.state.robot_stopped    = False
            return

        if ENABLE_CARTRIDGE_CHECK and self.read_cartridge_empty():
            raise RuntimeError("Cannot start: dispenser cartridge is empty.")
        if not self.read_board_present():
            raise RuntimeError("Cannot start: board not detected in fixture.")
        if self.current_recipe_module is None:
            raise RuntimeError("No recipe loaded. Select a program first.")
        if self._reset_active() or self.state.robot_resetting:
            raise RuntimeError("Cannot start: robot reset is in progress.")
        if self._cycle_active():
            raise RuntimeError("A recipe is already running.")

        self.last_reset_error      = None
        self.cycle_abort_requested = False
        self.state.robot_running   = True
        self.state.robot_paused    = False
        self.state.robot_stopped   = False
        self.recipe_thread = threading.Thread(
            target=self._run_loaded_recipe, daemon=True, name="recipe"
        )
        self.recipe_thread.start()

    def run_manual_dispense(self) -> None:
        if self._reset_active() or self.state.robot_resetting:
            raise RuntimeError("Robot reset is in progress.")
        if self._cycle_active():
            raise RuntimeError("Another program is already active.")
        recipe_path = RECIPES_DIR / f"{MANUAL_DISPENSE_PROGRAM}.py"
        if not recipe_path.exists():
            raise RuntimeError(f"Purge program not found: {recipe_path}")
        self.load_recipe(MANUAL_DISPENSE_PROGRAM, recipe_path)
        self._start_recipe_thread()

    def run_tube_change(self) -> None:
        if self._reset_active() or self.state.robot_resetting:
            raise RuntimeError("Robot reset is in progress.")
        if self._cycle_active():
            raise RuntimeError("Another program is already active.")
        if self.state.manual_mode:
            raise RuntimeError("Switch out of manual mode before running tube change.")
        if self.state.light_curtain_broken:
            raise RuntimeError("Light curtain is broken. Clear the area first.")
        recipe_path = RECIPES_DIR / f"{TUBE_CHANGE_PROGRAM}.py"
        if not recipe_path.exists():
            raise RuntimeError(
                f"Tube change program not found.\n\n"
                f"Create a '{TUBE_CHANGE_PROGRAM}.py' file in the recipes folder."
            )
        self.load_recipe(TUBE_CHANGE_PROGRAM, recipe_path)
        self._start_recipe_thread()

    def _start_recipe_thread(self) -> None:
        self.last_reset_error      = None
        self.cycle_abort_requested = False
        self.state.robot_running   = True
        self.state.robot_paused    = False
        self.state.robot_stopped   = False
        self.recipe_thread = threading.Thread(
            target=self._run_loaded_recipe, daemon=True, name="recipe"
        )
        self.recipe_thread.start()

    def pause_immediate(self) -> None:
        if self.state.robot_resetting:
            raise RuntimeError("Cannot pause: robot reset is in progress.")
        self.state.robot_paused  = True
        self.state.robot_running = False
        self.state.robot_stopped = False
        self.dispenser_off_immediate()
        if self.arm is not None:
            try:
                self.arm.set_state(3)
            except Exception as exc:
                print(f"Pause command failed: {exc}")

    def stop_cycle(self) -> None:
        if self.state.robot_resetting:
            raise RuntimeError("Cannot stop: robot reset is in progress.")
        self.last_reset_error      = None
        self.cycle_abort_requested = True
        self.state.robot_running   = False
        self.state.robot_paused    = False
        self.state.robot_stopped   = True
        self._terminate_active_cycle()

    def abort_for_manual_mode(self) -> None:
        """
        Called when manual mode is engaged while a cycle is running.
        The firmware stops arm motion; we clean up software state so the
        operator returns to STOPPED (not a phantom-running cycle) when
        they release manual mode.
        """
        self.cycle_abort_requested = True
        self.state.robot_running   = False
        self.state.robot_paused    = False
        self.state.robot_stopped   = True
        self.state.light_curtain_paused = False
        try:
            self.dispenser_off_immediate()
        except Exception as exc:
            print(f"Dispenser off during manual-mode abort failed: {exc}")
        # Signal the recipe thread — it will exit on its next motion attempt
        if self.recipe_thread is not None and not self.recipe_thread.is_alive():
            self.recipe_thread = None

    def reset_to_initial_position(self) -> None:
        if self._reset_active() or self.state.robot_resetting:
            raise RuntimeError("Reset is already in progress.")

        self.last_reset_error      = None
        self.cycle_abort_requested = True
        self.state.robot_running   = False
        self.state.robot_paused    = False
        self.state.robot_stopped   = False
        self.state.robot_resetting = True

        def worker():
            try:
                if self.arm is None:
                    return
                self._terminate_active_cycle()

                # Clear errors so the controller can move again after a fault
                try:
                    self.arm.clean_error()
                    self.arm.clean_warn()
                    self.arm.motion_enable(enable=True)
                    self.arm.set_mode(0)
                    self.arm.set_state(0)
                except Exception as exc:
                    print(f"Re-enable during reset failed: {exc}")

                reset_recipe = RECIPES_DIR / f"{RESET_PROGRAM}.py"
                if not reset_recipe.exists():
                    raise RuntimeError(f"Reset program not found: {reset_recipe}")
                self.load_recipe(RESET_PROGRAM, reset_recipe)
                self._execute_loaded_program(count_cycle=False)
            except Exception as exc:
                self.last_reset_error = str(exc)
                print(f"Reset failed: {exc}")
            finally:
                self.state.robot_running   = False
                self.state.robot_paused    = False
                self.state.robot_resetting = False
                self.recipe_thread = None

        self.reset_thread = threading.Thread(target=worker, daemon=True, name="reset")
        self.reset_thread.start()

    def emergency_stop(self) -> None:
        self.cycle_abort_requested = True
        self.state.robot_paused    = True
        self.state.robot_running   = False
        self.dispenser_off()
        if self.arm is not None:
            try:
                self.arm.emergency_stop()
            except Exception as exc:
                print(f"Emergency stop command failed: {exc}")


# ---------------------------------------------------------------------------
# GUI helpers
# ---------------------------------------------------------------------------

class GlowPanel(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("GlowPanel")

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.rect().adjusted(1, 1, -1, -1)
        grad = QLinearGradient(rect.topLeft(), rect.bottomRight())
        grad.setColorAt(0, QColor("#0f172a"))
        grad.setColorAt(1, QColor("#0b1220"))
        painter.setBrush(grad)
        painter.setPen(QPen(QColor("#223047"), 1))
        painter.drawRoundedRect(rect, 24, 24)
        super().paintEvent(event)


class StatusCard(QFrame):
    def __init__(self, title: str, subtitle: str = "", parent=None):
        super().__init__(parent)
        self.setObjectName("StatusCard")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 22, 22, 22)
        layout.setSpacing(10)
        layout.setAlignment(Qt.AlignCenter)

        self.dot = QLabel("●")
        self.dot.setObjectName("StatusDot")
        self.dot.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.dot)

        self.title_label = QLabel(title)
        self.title_label.setObjectName("CardTitle")
        self.title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.title_label)

        self.value_label = QLabel("UNKNOWN")
        self.value_label.setObjectName("CardValue")
        self.value_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.value_label)

        self.subtitle_label = QLabel(subtitle)
        self.subtitle_label.setObjectName("CardSubtitle")
        self.subtitle_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.subtitle_label)

    def set_status(self, text: str, ok: bool, subtitle: str = ""):
        self.value_label.setText(text)
        self.subtitle_label.setText(subtitle)
        self.value_label.setProperty("ok", ok)
        self.dot.setProperty("ok", ok)
        self.value_label.style().unpolish(self.value_label)
        self.value_label.style().polish(self.value_label)
        self.dot.style().unpolish(self.dot)
        self.dot.style().polish(self.dot)


class MetricCard(QFrame):
    def __init__(self, title: str, value: str = "0", accent: str = "blue", parent=None):
        super().__init__(parent)
        self.setObjectName(f"MetricCard_{accent}")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(22, 18, 22, 18)
        layout.setSpacing(6)
        layout.setAlignment(Qt.AlignCenter)

        self.title_label = QLabel(title)
        self.title_label.setObjectName("MetricTitle")
        self.title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.title_label)

        self.value_label = QLabel(value)
        self.value_label.setObjectName("MetricValue")
        self.value_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.value_label)

    def set_value(self, value: str):
        self.value_label.setText(value)


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.robot = RobotController()
        self.recipe_dir = RECIPES_DIR
        self.recipe_dir.mkdir(exist_ok=True)

        # Prime the GPIO cache so initial latch values are accurate
        self.robot.refresh_gpio_cache()

        # Rising-edge latches — initialise to current state so a button held
        # at startup is ignored until it is released and pressed again.
        self.start_latch       = self.robot.read_physical_start()
        self.pause_latch       = self.robot.read_pause_button()
        self.stop_latch        = self.robot.read_stop_button()
        self.reset_latch       = self.robot.read_reset_button()
        self.tube_change_latch = self.robot.read_tube_change_button()
        self.curtain_latch     = self.robot.read_light_curtain()
        self.manual_mode_latch = self.robot.read_manual_mode()

        self.status_cards_compact: Optional[bool] = None

        self.setWindowTitle("Dispensing Cell HMI")
        screen = QGuiApplication.primaryScreen()
        if screen is not None:
            avail = screen.availableGeometry()
            self.resize(
                max(1280, min(avail.width(),  1600)),
                max(720,  min(avail.height(), 900)),
            )
        else:
            self.resize(1600, 900)

        # ---------- layout ----------
        root = QWidget()
        root.setObjectName("Root")
        self.setCentralWidget(root)
        self.root_layout = QVBoxLayout(root)
        self.root_layout.setContentsMargins(28, 28, 28, 28)
        self.root_layout.setSpacing(18)

        # Hero bar
        hero = GlowPanel()
        self.hero_layout = QHBoxLayout(hero)
        self.hero_layout.setContentsMargins(28, 22, 28, 22)
        self.hero_layout.setSpacing(20)

        title_block = QVBoxLayout()
        title_block.setSpacing(4)
        eyebrow = QLabel("DISPENSING ROBOT")
        eyebrow.setObjectName("Eyebrow")
        title_block.addWidget(eyebrow)
        self.live_commentary = QLabel("Waiting for board")
        self.live_commentary.setObjectName("LiveCommentary")
        self.live_commentary.setWordWrap(True)
        title_block.addWidget(self.live_commentary)
        self.hero_layout.addLayout(title_block, 1)

        metrics_wrap = QHBoxLayout()
        metrics_wrap.setSpacing(12)
        self.cycle_count_card = MetricCard("Program Count", "0",    "blue")
        self.last_cycle_card  = MetricCard("Last Cycle Time", "--:--", "purple")
        metrics_wrap.addWidget(self.cycle_count_card)
        metrics_wrap.addWidget(self.last_cycle_card)
        self.hero_layout.addLayout(metrics_wrap)

        self.hero_state = QLabel("SYSTEM IDLE")
        self.hero_state.setObjectName("HeroState")
        self.hero_state.setAlignment(Qt.AlignCenter)
        self.hero_state.setMinimumWidth(210)
        self.hero_layout.addWidget(self.hero_state)
        self.root_layout.addWidget(hero)

        # Three-column content
        self.content_layout = QGridLayout()
        self.content_layout.setHorizontalSpacing(18)
        self.content_layout.setVerticalSpacing(18)
        self.root_layout.addLayout(self.content_layout, 1)

        # Left panel — program selection
        left_panel  = GlowPanel()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(24, 24, 24, 24)
        left_layout.setSpacing(16)

        recipe_title = QLabel("Program Setup")
        recipe_title.setObjectName("SectionTitle")
        recipe_title.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(recipe_title)

        left_layout.addWidget(QLabel(""))   # spacer label keeps height stable

        info_bar = QFrame()
        info_bar.setObjectName("InfoBar")
        info_bar_layout = QVBoxLayout(info_bar)
        info_bar_layout.setContentsMargins(18, 16, 18, 16)
        info_bar_layout.setSpacing(6)
        self.loaded_program_label = QLabel(DEFAULT_RECIPE)
        self.loaded_program_label.setObjectName("LoadedProgramTile")
        self.loaded_program_label.setAlignment(Qt.AlignCenter)
        info_bar_layout.addWidget(self.loaded_program_label)
        hint_label = QLabel("Currently loaded program")
        hint_label.setObjectName("InfoLabel")
        hint_label.setAlignment(Qt.AlignCenter)
        info_bar_layout.addWidget(hint_label)
        left_layout.addWidget(info_bar)

        self.recipe_combo = QComboBox()
        self.recipe_combo.setObjectName("RecipeCombo")
        self.recipe_combo.currentIndexChanged.connect(self.on_recipe_selected)
        left_layout.addWidget(self.recipe_combo)

        left_layout.addStretch()
        self.refresh_button = QPushButton("Refresh Programs")
        self.refresh_button.setObjectName("SecondaryButton")
        self.refresh_button.clicked.connect(self.refresh_recipes)
        left_layout.addWidget(self.refresh_button)
        self.content_layout.addWidget(left_panel, 0, 1)

        # Middle panel — status cards
        middle_panel  = GlowPanel()
        middle_layout = QVBoxLayout(middle_panel)
        middle_layout.setContentsMargins(24, 24, 24, 24)
        middle_layout.setSpacing(16)
        status_title = QLabel("Live Machine Status")
        status_title.setObjectName("SectionTitle")
        status_title.setAlignment(Qt.AlignCenter)
        middle_layout.addWidget(status_title)

        self.status_cards = QGridLayout()
        self.status_cards.setHorizontalSpacing(14)
        self.status_cards.setVerticalSpacing(14)

        self.program_card      = StatusCard("Loaded Program",  "Current recipe")
        self.board_card        = StatusCard("Board Presence",  "DI0 board sensor")
        self.dispenser_card    = StatusCard("Dispenser",       "CO7 epoxy trigger")
        self.motion_card       = StatusCard("Robot Motion",    "Controller status")
        self.run_card          = StatusCard("Robot State",     "State / mode")
        self.light_curtain_card = StatusCard("Light Curtain",  "DI4 safety beam")
        self.manual_card       = StatusCard("Manual Mode",     "Controller mode")
        self.warning_card      = StatusCard("Warning",         "warn_code")
        self.error_card        = StatusCard("Error",           "error_code")
        self.status_card_widgets = (
            self.program_card, self.board_card,  self.dispenser_card,
            self.motion_card,  self.run_card,    self.light_curtain_card,
            self.manual_card,  self.warning_card, self.error_card,
        )
        for card in self.status_card_widgets:
            card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        middle_layout.addLayout(self.status_cards)
        middle_layout.addStretch()
        self.content_layout.addWidget(middle_panel, 0, 0)

        # Right panel — operator buttons
        right_panel  = GlowPanel()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(24, 24, 24, 24)
        right_layout.setSpacing(16)
        control_title = QLabel("Operator Controls")
        control_title.setObjectName("SectionTitle")
        control_title.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(control_title)

        self.start_button         = QPushButton("Start Cycle")
        self.start_button.setObjectName("StartButton")
        self.start_button.clicked.connect(self.start_cycle)

        self.pause_button         = QPushButton("Pause")
        self.pause_button.setObjectName("PauseButton")
        self.pause_button.clicked.connect(self.pause_cycle)

        self.stop_button          = QPushButton("Stop")
        self.stop_button.setObjectName("StopButton")
        self.stop_button.clicked.connect(self.stop_cycle)

        self.reset_button         = QPushButton("Reset")
        self.reset_button.setObjectName("ResetButton")
        self.reset_button.clicked.connect(self.reset_system)

        self.manual_dispense_button = QPushButton("Purge Epoxy")
        self.manual_dispense_button.setObjectName("PurgeButton")
        self.manual_dispense_button.clicked.connect(self.run_manual_dispense)

        self.tube_change_button   = QPushButton("Tube Change")
        self.tube_change_button.setObjectName("TubeChangeButton")
        self.tube_change_button.clicked.connect(self.run_tube_change)

        self.primary_buttons = (
            self.start_button,
            self.pause_button,
            self.stop_button,
            self.reset_button,
            self.manual_dispense_button,
            self.tube_change_button,
        )
        for btn in self.primary_buttons:
            btn.setMinimumHeight(68)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            right_layout.addWidget(btn)
        right_layout.addStretch()
        self.content_layout.addWidget(right_panel, 0, 2)

        self.content_layout.setColumnStretch(0, 1)
        self.content_layout.setColumnStretch(1, 1)
        self.content_layout.setColumnStretch(2, 1)

        # Populate recipe list and select default
        self.refresh_recipes()
        idx = self.recipe_combo.findText(DEFAULT_RECIPE)
        if idx >= 0:
            self.recipe_combo.setCurrentIndex(idx)
            self.update_loaded_program_display()

        # Poll timer
        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.update_status)
        self.poll_timer.start(POLL_INTERVAL_MS)

        self.apply_styles()
        self.update_responsive_layout(force=True)
        self.update_status()

    # ------------------------------------------------------------------
    # Layout helpers
    # ------------------------------------------------------------------

    def format_cycle_time(self, seconds: float) -> str:
        if seconds <= 0:
            return "--:--"
        whole = int(round(seconds))
        return f"{whole // 60:02d}:{whole % 60:02d}"

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_responsive_layout()

    def update_responsive_layout(self, force: bool = False):
        compact = self.width() >= self.height() or self.height() < 900
        if not force and compact == self.status_cards_compact:
            return
        self.status_cards_compact = compact
        dense = self.height() <= 1080 or self.width() <= 1600

        while self.status_cards.count():
            self.status_cards.takeAt(0)
        cols = 3
        for i, card in enumerate(self.status_card_widgets):
            self.status_cards.addWidget(card, i // cols, i % cols)
        for c in range(cols):
            self.status_cards.setColumnStretch(c, 1)

        card_h = 128 if dense else 142
        for card in self.status_card_widgets:
            card.setMinimumHeight(card_h)
        for btn in self.primary_buttons:
            btn.setMinimumHeight(46 if dense else (52 if compact else 62))
        self.reset_button.setMinimumHeight(42 if dense else (46 if compact else 54))
        self.hero_state.setMinimumWidth(150 if dense else (180 if compact else 210))

        if dense:
            self.root_layout.setContentsMargins(14, 14, 14, 14)
            self.root_layout.setSpacing(10)
            self.hero_layout.setContentsMargins(16, 12, 16, 12)
            self.hero_layout.setSpacing(10)
            self.content_layout.setHorizontalSpacing(10)
            self.content_layout.setVerticalSpacing(10)
        elif compact:
            self.root_layout.setContentsMargins(20, 20, 20, 20)
            self.root_layout.setSpacing(14)
            self.hero_layout.setContentsMargins(22, 18, 22, 18)
            self.hero_layout.setSpacing(14)
            self.content_layout.setHorizontalSpacing(14)
            self.content_layout.setVerticalSpacing(14)
        else:
            self.root_layout.setContentsMargins(28, 28, 28, 28)
            self.root_layout.setSpacing(18)
            self.hero_layout.setContentsMargins(28, 22, 28, 22)
            self.hero_layout.setSpacing(20)
            self.content_layout.setHorizontalSpacing(18)
            self.content_layout.setVerticalSpacing(18)

    # ------------------------------------------------------------------
    # Stylesheet
    # ------------------------------------------------------------------

    def apply_styles(self):
        self.setStyleSheet("""
            QMainWindow, QWidget#Root {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #030712,stop:1 #111827);
            }
            QWidget {
                color: #e5eefc;
                font-family: Segoe UI, Inter, Arial, sans-serif;
                font-size: 14px;
            }
            QLabel { background: transparent; }
            QLabel#Eyebrow {
                font-size: 11px; font-weight: 700;
                color: #7dd3fc; letter-spacing: 1px;
            }
            QLabel#LiveCommentary {
                font-size: 18px; font-weight: 700;
                color: #f8fbff; max-width: 680px;
            }
            QLabel#HeroState {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #0f766e,stop:1 #0369a1);
                border: 1px solid #38bdf8; border-radius: 22px; padding: 14px;
                font-size: 18px; font-weight: 800; color: white;
            }
            QLabel#SectionTitle {
                font-size: 18px; font-weight: 700; color: #f8fbff;
            }
            QLabel#SectionSubtext {
                font-size: 12px; color: #93a7c2; min-height: 0px;
            }
            QFrame#InfoBar {
                background: rgba(15,23,42,0.88);
                border: 1px solid #27364d; border-radius: 18px;
            }
            QLabel#LoadedProgramTile {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #082f49,stop:1 #0f4c81);
                border: 1px solid #38bdf8; border-radius: 18px;
                padding: 18px 14px; font-size: 24px; font-weight: 800; color: #f8fbff;
            }
            QLabel#InfoLabel { font-size: 12px; color: #8ea0b8; }
            QFrame#StatusCard {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #0f172a,stop:1 #172235);
                border: 1px solid #25344b; border-radius: 22px;
            }
            QLabel#StatusDot { font-size: 22px; color: #475569; }
            QLabel#StatusDot[ok="true"]  { color: #4ade80; }
            QLabel#StatusDot[ok="false"] { color: #fb7185; }
            QLabel#CardTitle  { font-size: 12px; font-weight: 700; color: #e2e8f0; }
            QLabel#CardValue  { font-size: 24px; font-weight: 800; color: #f8fbff; }
            QLabel#CardValue[ok="true"]  { color: #4ade80; }
            QLabel#CardValue[ok="false"] { color: #fb7185; }
            QLabel#CardSubtitle { font-size: 11px; color: #8ca0b8; min-height: 0px; }
            QFrame#MetricCard_blue, QFrame#MetricCard_purple {
                border-radius: 20px; min-width: 150px;
            }
            QFrame#MetricCard_blue {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #082f49,stop:1 #0f4c81);
                border: 1px solid #38bdf8;
            }
            QFrame#MetricCard_purple {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #312e81,stop:1 #5b21b6);
                border: 1px solid #a78bfa;
            }
            QLabel#MetricTitle {
                font-size: 11px; font-weight: 700;
                color: rgba(255,255,255,0.78); text-transform: uppercase;
            }
            QLabel#MetricValue { font-size: 22px; font-weight: 800; color: white; }
            QComboBox#RecipeCombo {
                min-height: 44px; border-radius: 18px;
                border: 1px solid #334155; background: rgba(15,23,42,0.95);
                padding: 10px 16px; font-size: 14px; font-weight: 600; color: #f8fbff;
            }
            QComboBox#RecipeCombo::drop-down { border: none; width: 40px; }
            QPushButton {
                border: none; border-radius: 20px;
                padding: 10px 14px; font-size: 14px; font-weight: 700;
            }
            QPushButton#SecondaryButton {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #1e293b,stop:1 #334155);
                color: #e2e8f0; border: 1px solid #475569;
            }
            QPushButton#StartButton {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #10b981,stop:1 #059669);
                color: white; border: 1px solid #34d399;
            }
            QPushButton#PauseButton {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #facc15,stop:1 #eab308);
                color: #1f2937; border: 1px solid #fde047;
            }
            QPushButton#StopButton {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #dc2626,stop:1 #b91c1c);
                color: white; border: 1px solid #f87171;
            }
            QPushButton#ResetButton {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #1f2937,stop:1 #030712);
                color: white; border: 1px solid #475569;
            }
            QPushButton#PurgeButton {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #2563eb,stop:1 #1d4ed8);
                color: white; border: 1px solid #60a5fa;
            }
            QPushButton#TubeChangeButton {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #7c3aed,stop:1 #6d28d9);
                color: white; border: 1px solid #a78bfa;
            }
            QPushButton:hover   { opacity: 0.9; }
            QPushButton:pressed { padding-top: 14px; padding-bottom: 10px; }

            /* ---- Dialog styling (prevents invisible text on white background) ---- */
            QMessageBox { background-color: #0f172a; }
            QMessageBox QLabel {
                color: #f8fbff; font-size: 15px; font-weight: 600;
                padding: 10px 4px; min-width: 360px;
            }
            QMessageBox QPushButton {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #1e293b,stop:1 #334155);
                color: #f8fbff; border: 1px solid #475569; border-radius: 10px;
                padding: 8px 24px; min-width: 90px;
                font-size: 14px; font-weight: 700;
            }
            QMessageBox QPushButton:hover {
                background: qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #334155,stop:1 #475569);
            }
        """)

    # ------------------------------------------------------------------
    # Recipe list management
    # ------------------------------------------------------------------

    def refresh_recipes(self):
        self.recipe_combo.clear()
        for f in sorted(self.recipe_dir.glob("*.py")):
            self.recipe_combo.addItem(f.stem, str(f))
        if self.recipe_combo.count() == 0:
            self.recipe_combo.addItem(
                DEFAULT_RECIPE, str(self.recipe_dir / f"{DEFAULT_RECIPE}.py")
            )

    def update_loaded_program_display(self):
        name = self.robot.current_recipe or self.recipe_combo.currentText() or DEFAULT_RECIPE
        self.loaded_program_label.setText(name)

    def on_recipe_selected(self, _index: int):
        if self.robot._cycle_active() or self.robot._reset_active() or self.robot.state.robot_resetting:
            self.update_loaded_program_display()
            return
        recipe_path_raw = self.recipe_combo.currentData()
        if not recipe_path_raw:
            self.update_loaded_program_display()
            return
        try:
            self.robot.load_recipe(Path(recipe_path_raw).stem, Path(recipe_path_raw))
        except Exception as exc:
            QMessageBox.critical(self, "Load Failed", str(exc))
        self.update_loaded_program_display()

    def set_live_commentary(self, text: str):
        self.live_commentary.setText(text)

    # ------------------------------------------------------------------
    # Main poll tick
    # ------------------------------------------------------------------

    def update_status(self):
        # --- Single bulk GPIO read for this tick ---
        self.robot.refresh_gpio_cache()

        # --- Read all values (no extra SDK calls) ---
        board_present       = self.robot.read_board_present()
        light_curtain_broken = self.robot.read_light_curtain()
        physical_start      = self.robot.read_physical_start()
        pause_button        = self.robot.read_pause_button()
        stop_button         = self.robot.read_stop_button()
        reset_button        = self.robot.read_reset_button()
        tube_change_button  = self.robot.read_tube_change_button()
        manual_mode         = self.robot.read_manual_mode()   # reads from cb_snapshot
        state_code          = self.robot.read_robot_state_code()
        mode_code           = self.robot.read_robot_mode_code()
        warning_code        = self.robot.read_warning_code()
        error_code          = self.robot.read_error_code()

        # Derived flags
        emergency_stop_active = error_code in {1, 2, 3}
        manual_mode_active    = manual_mode or mode_code == 2
        robot_enabled = (
            False if self.robot.arm is None
            else (error_code == 0 and state_code is not None and state_code < 4)
        )
        robot_moving   = bool(self.robot.state.robot_running or state_code == 1)
        motion_stopped = bool(state_code in {3, 4} or (not robot_moving and robot_enabled))
        dispenser_on   = self.robot.is_dispenser_on()
        loaded_name    = self.robot.current_recipe or self.recipe_combo.currentText() or DEFAULT_RECIPE

        # ============================================================
        # Hardware button / sensor edge handling  (rising edge = True)
        # ============================================================

        # Manual mode engaged — abort any running cycle immediately
        if manual_mode_active and not self.manual_mode_latch:
            if self.robot.state.robot_running or self.robot.state.robot_paused:
                self.robot.abort_for_manual_mode()
        self.manual_mode_latch = manual_mode_active

        # Light curtain broken — auto-pause
        if light_curtain_broken and not self.curtain_latch:
            if self.robot.state.robot_running and not self.robot.state.robot_paused:
                try:
                    self.robot.pause_immediate()
                    self.robot.state.light_curtain_paused = True
                except Exception as exc:
                    print(f"Light curtain auto-pause failed: {exc}")
        self.curtain_latch = light_curtain_broken

        # CI5 — Pause
        if pause_button and not self.pause_latch:
            if self.robot.state.robot_running and not self.robot.state.robot_paused:
                self.pause_cycle()
        self.pause_latch = pause_button

        # CI6 — Stop
        if stop_button and not self.stop_latch:
            if self.robot.state.robot_running or self.robot.state.robot_paused:
                self.stop_cycle()
        self.stop_latch = stop_button

        # CI7 — Reset to home
        if reset_button and not self.reset_latch:
            if not self.robot.state.robot_resetting:
                self.reset_system()
        self.reset_latch = reset_button

        # CI2 — Tube change
        if tube_change_button and not self.tube_change_latch:
            self.run_tube_change()
        self.tube_change_latch = tube_change_button

        # CI4 — Start / Resume  (gated by light curtain)
        if physical_start and not self.start_latch:
            self.start_latch = True
            if light_curtain_broken:
                QMessageBox.warning(
                    self, "Cannot Start",
                    "Light curtain is broken.\nClear the area before starting.",
                )
            elif not self.robot.state.robot_running and not self.robot.state.robot_resetting:
                self.start_cycle()
        elif not physical_start:
            self.start_latch = False

        # ============================================================
        # Status card updates
        # ============================================================
        self.program_card.set_status(loaded_name, True)
        self.board_card.set_status(
            "DETECTED" if board_present else "NOT DETECTED", board_present)
        self.light_curtain_card.set_status(
            "BROKEN" if light_curtain_broken else "CLEAR",
            not light_curtain_broken,
            "Robot pauses when broken",
        )
        self.manual_card.set_status(
            "ENABLED" if manual_mode_active else "DISABLED",
            not manual_mode_active,
            f"Mode code {mode_code if mode_code is not None else '--'}",
        )
        self.warning_card.set_status(
            "ACTIVE" if warning_code else "CLEAR",
            warning_code == 0,
            f"warn_code={warning_code}",
        )
        self.error_card.set_status(
            "ACTIVE" if error_code else "CLEAR",
            error_code == 0,
            f"error_code={error_code}",
        )
        self.motion_card.set_status(
            "STOPPED" if motion_stopped else "MOVING", not motion_stopped)
        self.dispenser_card.set_status(
            "ON" if dispenser_on else "OFF", dispenser_on)

        # ============================================================
        # Hero / run state + live commentary
        # ============================================================
        state_sub = (
            f"State {state_code if state_code is not None else '--'} / "
            f"Mode {mode_code if mode_code is not None else '--'}"
        )

        if emergency_stop_active:
            self.run_card.set_status("E-STOP", False, state_sub)
            self.hero_state.setText("ESTOP ACTIVE")
            self.set_live_commentary(
                "Emergency stop is active. Release the E-stop on the cabinet, then press Reset.")

        elif self.robot.last_reset_error or error_code:
            self.run_card.set_status("FAULT", False, state_sub)
            self.hero_state.setText("SYSTEM FAULT")
            self.set_live_commentary(
                self.robot.last_reset_error
                or f"Robot error code {error_code}. Press Reset to recover.")

        elif manual_mode_active:
            self.run_card.set_status("MANUAL", True, state_sub)
            self.hero_state.setText("MANUAL MODE")
            self.set_live_commentary(
                "Manual mode: arm can be guided by hand. Switch to auto to run programs.")

        elif self.robot.state.robot_resetting:
            self.run_card.set_status("RESETTING", False, state_sub)
            self.hero_state.setText("SYSTEM RESETTING")
            self.set_live_commentary("Moving to home position")

        elif self.robot.state.robot_running:
            self.run_card.set_status("RUNNING", True, state_sub)
            self.hero_state.setText("SYSTEM RUNNING")
            self.set_live_commentary(f"Running program {loaded_name}")

        elif self.robot.state.robot_paused:
            self.run_card.set_status("PAUSED", False, state_sub)
            self.hero_state.setText("SYSTEM PAUSED")
            if self.robot.state.light_curtain_paused:
                if light_curtain_broken:
                    self.set_live_commentary(
                        "Paused: light curtain is broken. Clear the area, then press Start to resume.")
                else:
                    self.set_live_commentary(
                        "Paused by light curtain. Beam is clear — press Start to resume.")
            else:
                self.set_live_commentary("Paused by operator. Press Start to resume.")

        elif self.robot.state.robot_stopped:
            self.run_card.set_status("STOPPED", False, state_sub)
            self.hero_state.setText("SYSTEM STOPPED")
            self.set_live_commentary(
                "Stopped. Press Start to run the program from the beginning.")

        else:
            self.run_card.set_status("IDLE", robot_enabled, state_sub)
            self.hero_state.setText("SYSTEM IDLE")
            if light_curtain_broken:
                self.set_live_commentary(
                    "Light curtain is broken. Clear the area before starting.")
            elif warning_code:
                self.set_live_commentary(f"Robot warning code {warning_code}")
            elif not board_present:
                self.set_live_commentary("Place a board in the fixture, then press Start.")
            else:
                self.set_live_commentary(f"Ready to run program {loaded_name}. Press Start.")

        self.cycle_count_card.set_value(
            str(self.robot.get_program_cycle_count(loaded_name)))
        self.last_cycle_card.set_value(
            self.format_cycle_time(self.robot.get_program_last_cycle_seconds(loaded_name)))

    # ------------------------------------------------------------------
    # Operator button handlers
    # ------------------------------------------------------------------

    def start_cycle(self):
        if self.robot.state.light_curtain_broken:
            QMessageBox.warning(
                self, "Cannot Start",
                "Light curtain is broken.\nClear the area before starting.",
            )
            return

        if self.robot.state.robot_paused and self.robot._cycle_active():
            try:
                self.robot.start_cycle()
                self.robot.state.light_curtain_paused = False
                name = self.robot.current_recipe or self.recipe_combo.currentText() or DEFAULT_RECIPE
                self.set_live_commentary(f"Resumed program {name}")
            except Exception as exc:
                QMessageBox.critical(self, "Resume Failed", str(exc))
            return

        recipe_path_raw = self.recipe_combo.currentData()
        if not recipe_path_raw:
            QMessageBox.warning(
                self, "No Program",
                "No dispense program found. Add a recipe file to the recipes folder.")
            return

        try:
            recipe_path = Path(recipe_path_raw)
            self.robot.load_recipe(recipe_path.stem, recipe_path)
            self.update_loaded_program_display()
            self.robot.start_cycle()
            self.robot.state.light_curtain_paused = False
        except Exception as exc:
            QMessageBox.critical(self, "Start Failed", str(exc))

    def run_manual_dispense(self):
        try:
            self.robot.run_manual_dispense()
            self.update_loaded_program_display()
            self.set_live_commentary("Running epoxy purge")
        except Exception as exc:
            QMessageBox.critical(self, "Purge Failed", str(exc))

    def run_tube_change(self):
        try:
            self.robot.run_tube_change()
            self.update_loaded_program_display()
            self.set_live_commentary("Moving to tube change position")
        except Exception as exc:
            QMessageBox.critical(self, "Tube Change Failed", str(exc))

    def pause_cycle(self):
        try:
            self.robot.pause_immediate()
            self.set_live_commentary("Paused by operator")
        except Exception as exc:
            QMessageBox.critical(self, "Pause Failed", str(exc))

    def stop_cycle(self):
        try:
            self.robot.stop_cycle()
            self.set_live_commentary("Stopped by operator")
        except Exception as exc:
            QMessageBox.critical(self, "Stop Failed", str(exc))

    def reset_system(self):
        try:
            self.robot.reset_to_initial_position()
            self.set_live_commentary("Resetting — moving to home position")
        except Exception as exc:
            QMessageBox.critical(self, "Reset Failed", str(exc))

    # ------------------------------------------------------------------
    # Clean shutdown
    # ------------------------------------------------------------------

    def closeEvent(self, event):
        self.poll_timer.stop()
        self.robot.disconnect_robot()
        super().closeEvent(event)


# ---------------------------------------------------------------------------

def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
