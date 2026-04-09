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
SDK_DIR = BASE_DIR / "Dependencies" / "xArm-Python-SDK-master"

if SDK_DIR.exists():
    sdk_path = str(SDK_DIR)
    if sdk_path not in sys.path:
        sys.path.insert(0, sdk_path)


# xArm CGPIO mapping from exported WebUI programs:
# CI0-CI7 => 0-7
# DI0-DI7 => 8-15
# CO0-CO7 => 0-7
# DO0-DO7 => 8-15
BOARD_PRESENT_DI = 0          # DI0 => CGPIO channel 8
CARTRIDGE_EMPTY_DI = 1        # DI1 => CGPIO channel 9
ENABLE_ROBOT_CI = 0           # CI0 => Enable Robot
MANUAL_MODE_CI = 1            # CI1 => Manual Mode
SAFEGUARD_RESET_CI = 2        # CI2 => Safeguard Reset
STOP_MOVING_CI = 3            # CI3 => Stop Moving
PHYSICAL_START_CI = 4         # CI4 => dedicated cycle start input
ROBOT_ENABLED_CO = 0          # CO0 => Robot Enabled
MANUAL_MODE_CO = 1            # CO1 => Manual Mode
MOTION_STOPPED_CO = 2         # CO2 => Motion Stopped
EMERGENCY_STOP_CO = 3         # CO3 => Emergency Stop Pressed
WARNING_CO = 4                # CO4 => Warning
ERROR_CO = 5                  # CO5 => Error
ROBOT_MOVING_CO = 6           # CO6 => Robot Moving
DISPENSER_TRIGGER_CO = 7      # CO7 => CGPIO channel 7
MANUAL_DISPENSE_PROGRAM = "purge"
RESET_PROGRAM = "reset"
ENABLE_CARTRIDGE_CHECK = False
DEFAULT_RECIPE = "PCB_466"
XARM_IP = "192.168.1.196"  
RESET_ENABLE_WAIT_SECONDS = 15
RESET_HOME_TIMEOUT_SECONDS = 20


@dataclass
class SensorState:
    board_present: bool = False
    cartridge_empty: bool = False
    enable_robot: bool = False
    manual_mode: bool = False
    safeguard_reset: bool = False
    stop_moving: bool = False
    dispenser_on: bool = False
    robot_running: bool = False
    robot_paused: bool = False
    robot_stopped: bool = False
    robot_resetting: bool = False
    cycle_count: int = 0
    last_cycle_seconds: float = 0.0


class OutputTrackingArmProxy:
    def __init__(self, arm, controller):
        object.__setattr__(self, '_arm', arm)
        object.__setattr__(self, '_controller', controller)

    def __getattr__(self, name):
        return getattr(self._arm, name)

    def __setattr__(self, name, value):
        setattr(self._arm, name, value)

    def set_cgpio_digital(self, ionum, value, *args, **kwargs):
        code = self._arm.set_cgpio_digital(ionum, value, *args, **kwargs)
        if code == 0:
            self._controller.output_states[int(ionum)] = bool(value)
            if int(ionum) == DISPENSER_TRIGGER_CO:
                self._controller.state.dispenser_on = bool(value)
        return code


class RobotController:
    def __init__(self):
        self.state = SensorState()
        self.current_recipe: Optional[str] = None
        self.current_recipe_module = None
        self.arm = None
        self.recipe_thread = None
        self.reset_thread = None
        self.last_reset_error: Optional[str] = None
        self.cycle_abort_requested = False
        self.output_states = {
            DISPENSER_TRIGGER_CO: False,
        }
        self.connect_robot()

    def connect_robot(self) -> None:
        try:
            from xarm.wrapper import XArmAPI

            self.arm = XArmAPI(XARM_IP)
            self.arm.connect()
            self._attach_output_tracking()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
        except Exception as exc:
            print(f"Robot connection not active yet: {exc}")
            self.arm = None

    def _attach_output_tracking(self) -> None:
        if self.arm is None or getattr(self.arm, '_epoxy_output_tracking', False):
            return

        original_set_cgpio_digital = self.arm.set_cgpio_digital

        def tracked_set_cgpio_digital(ionum, value, *args, **kwargs):
            code = original_set_cgpio_digital(ionum, value, *args, **kwargs)
            if code == 0:
                self.output_states[int(ionum)] = bool(value)
                if int(ionum) == DISPENSER_TRIGGER_CO:
                    self.state.dispenser_on = bool(value)
            return code

        self.arm.set_cgpio_digital = tracked_set_cgpio_digital
        self.arm._epoxy_output_tracking = True

    def _read_cgpio_channel(self, channel: int) -> bool:
        if self.arm is None:
            if channel == self.di_channel(BOARD_PRESENT_DI):
                return self.state.board_present
            if channel == self.di_channel(CARTRIDGE_EMPTY_DI):
                return self.state.cartridge_empty
            return False
        try:
            result = self.arm.get_cgpio_digital(channel)
            if isinstance(result, (list, tuple)) and len(result) > 1:
                return bool(result[1])
        except Exception as exc:
            print(f"Failed to read CGPIO channel {channel}: {exc}")
        return False

    @staticmethod
    def ci_channel(index: int) -> int:
        return index

    @staticmethod
    def di_channel(index: int) -> int:
        return 8 + index

    @staticmethod
    def co_channel(index: int) -> int:
        return index

    @staticmethod
    def do_channel(index: int) -> int:
        return 8 + index

    def read_ci(self, index: int) -> bool:
        return self._read_cgpio_channel(self.ci_channel(index))

    def read_di(self, index: int) -> bool:
        return self._read_cgpio_channel(self.di_channel(index))

    def read_board_present(self) -> bool:
        return self.read_di(BOARD_PRESENT_DI)

    def read_cartridge_empty(self) -> bool:
        return self.read_di(CARTRIDGE_EMPTY_DI)

    def read_physical_start(self) -> bool:
        return self.read_ci(PHYSICAL_START_CI)

    @staticmethod
    def _controller_function_active(raw_state: bool) -> bool:
        return not raw_state

    def read_enable_robot(self) -> bool:
        enabled = self._controller_function_active(self.read_ci(ENABLE_ROBOT_CI))
        self.state.enable_robot = enabled
        return enabled

    def read_manual_mode(self) -> bool:
        manual_mode = self._controller_function_active(self.read_ci(MANUAL_MODE_CI))
        self.state.manual_mode = manual_mode
        return manual_mode

    def read_safeguard_reset(self) -> bool:
        safeguard_reset = self._controller_function_active(self.read_ci(SAFEGUARD_RESET_CI))
        self.state.safeguard_reset = safeguard_reset
        return safeguard_reset

    def read_stop_moving(self) -> bool:
        stop_moving = self._controller_function_active(self.read_ci(STOP_MOVING_CI))
        self.state.stop_moving = stop_moving
        return stop_moving

    @staticmethod
    def _extract_sdk_scalar(result) -> Optional[int]:
        if isinstance(result, (list, tuple)):
            if len(result) > 1 and isinstance(result[1], (int, bool)):
                return int(result[1])
            if len(result) == 1 and isinstance(result[0], (int, bool)):
                return int(result[0])
            return None
        if isinstance(result, (int, bool)):
            return int(result)
        return None

    def read_robot_state_code(self) -> Optional[int]:
        if self.arm is None:
            if self.state.robot_resetting or self.state.robot_stopped:
                return 4
            if self.state.robot_paused:
                return 3
            if self.state.robot_running:
                return 1
            return 2
        try:
            state_code = self._extract_sdk_scalar(self.arm.get_state())
            if state_code is not None:
                return state_code
        except Exception:
            pass
        state_attr = getattr(self.arm, 'state', None)
        return int(state_attr) if isinstance(state_attr, (int, bool)) else None

    def read_robot_mode_code(self) -> Optional[int]:
        if self.arm is None:
            return 2 if self.state.manual_mode else 0
        try:
            mode_code = self._extract_sdk_scalar(self.arm.get_mode())
            if mode_code is not None:
                return mode_code
        except Exception:
            pass
        mode_attr = getattr(self.arm, 'mode', None)
        return int(mode_attr) if isinstance(mode_attr, (int, bool)) else None

    def read_warning_code(self) -> int:
        warn_code = getattr(self.arm, 'warn_code', 0) if self.arm is not None else 0
        return int(warn_code) if isinstance(warn_code, (int, bool)) else 0

    def read_error_code(self) -> int:
        error_code = getattr(self.arm, 'error_code', 0) if self.arm is not None else 0
        return int(error_code) if isinstance(error_code, (int, bool)) else 0

    def is_dispenser_on(self) -> bool:
        return bool(self.output_states.get(DISPENSER_TRIGGER_CO, False))

    def set_co(self, index: int, state: bool, sync: bool = True) -> None:
        channel = self.co_channel(index)
        if self.arm is None:
            self.output_states[index] = bool(state)
            if index == DISPENSER_TRIGGER_CO:
                self.state.dispenser_on = bool(state)
            print(f"CO{index} / channel {channel} -> {'ON' if state else 'OFF'}")
            return
        try:
            code = self.arm.set_cgpio_digital(channel, int(state), delay_sec=0, sync=sync)
            if code != 0:
                print(f"Failed to set CO{index}, code={code}")
                return
            self.output_states[index] = bool(state)
            if index == DISPENSER_TRIGGER_CO:
                self.state.dispenser_on = bool(state)
        except Exception as exc:
            print(f"Failed to set CO{index}: {exc}")

    def dispenser_off(self) -> None:
        self.set_co(DISPENSER_TRIGGER_CO, False)

    def dispenser_off_immediate(self) -> None:
        self.set_co(DISPENSER_TRIGGER_CO, False, sync=False)

    def load_recipe(self, recipe_name: str, recipe_path: Path) -> None:
        spec = importlib.util.spec_from_file_location(recipe_name, recipe_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Could not load recipe: {recipe_path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.current_recipe = recipe_name
        self.current_recipe_module = module

    def _run_loaded_recipe(self) -> None:
        self._execute_loaded_program(count_cycle=True)
        self.state.robot_running = False
        self.recipe_thread = None

    def _execute_loaded_program(self, count_cycle: bool = True) -> None:
        start_time = time.perf_counter()
        try:
            module = self.current_recipe_module
            if module is None:
                raise RuntimeError("No recipe module loaded.")
            if hasattr(module, 'RobotMain'):
                recipe_arm = OutputTrackingArmProxy(self.arm, self) if self.arm is not None else None
                robot_main = module.RobotMain(recipe_arm)
                robot_main.run()
            elif hasattr(module, 'run'):
                module.run(self)
            else:
                raise RuntimeError("Recipe file has neither RobotMain nor run().")
            if count_cycle and not self.cycle_abort_requested:
                elapsed = time.perf_counter() - start_time
                self.state.last_cycle_seconds = elapsed
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
            self.arm.set_state(0)
            time.sleep(0.1)
            self.dispenser_off_immediate()
            self.arm.set_state(4)
        else:
            self.dispenser_off()

        active_thread.join(timeout=join_timeout)
        if active_thread.is_alive():
            raise RuntimeError("Timed out waiting for active recipe to stop.")

        self.recipe_thread = None

    def start_cycle(self) -> None:
        if self.state.robot_paused and self._cycle_active():
            if self.arm is not None:
                try:
                    self.arm.set_state(0)
                except Exception as exc:
                    raise RuntimeError(f"Resume command failed: {exc}") from exc
            self.last_reset_error = None
            self.cycle_abort_requested = False
            self.state.robot_running = True
            self.state.robot_paused = False
            self.state.robot_stopped = False
            return

        if ENABLE_CARTRIDGE_CHECK and self.read_cartridge_empty():
            raise RuntimeError("Cannot start cycle: dispenser cartridge is empty.")
        if not self.read_board_present():
            raise RuntimeError("Cannot start cycle: board not present.")
        if self.current_recipe_module is None:
            raise RuntimeError("No recipe loaded.")
        if self._reset_active() or self.state.robot_resetting:
            raise RuntimeError("Robot reset is in progress.")
        if self._cycle_active():
            raise RuntimeError("A recipe is already running.")

        self.last_reset_error = None
        self.cycle_abort_requested = False
        self.state.robot_running = True
        self.state.robot_paused = False
        self.state.robot_stopped = False
        self.recipe_thread = threading.Thread(target=self._run_loaded_recipe, daemon=True)
        self.recipe_thread.start()

    def run_manual_dispense(self) -> None:
        if self._reset_active() or self.state.robot_resetting:
            raise RuntimeError("Robot reset is in progress.")
        if self._cycle_active():
            raise RuntimeError("Cannot run manual dispense while another program is active.")
        manual_recipe = RECIPES_DIR / f"{MANUAL_DISPENSE_PROGRAM}.py"
        if not manual_recipe.exists():
            raise RuntimeError(f"Manual dispense program not found: {manual_recipe}")
        self.load_recipe(MANUAL_DISPENSE_PROGRAM, manual_recipe)
        self.last_reset_error = None
        self.cycle_abort_requested = False
        self.state.robot_running = True
        self.state.robot_paused = False
        self.state.robot_stopped = False
        self.recipe_thread = threading.Thread(target=self._run_loaded_recipe, daemon=True)
        self.recipe_thread.start()

    def pause_immediate(self) -> None:
        if self.state.robot_resetting:
            raise RuntimeError("Robot reset is in progress.")
        self.state.robot_paused = True
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
            raise RuntimeError("Robot reset is in progress.")

        self.last_reset_error = None
        self.cycle_abort_requested = True
        self.state.robot_running = False
        self.state.robot_paused = False
        self.state.robot_stopped = True
        self._terminate_active_cycle()

    def reset_to_initial_position(self) -> None:
        if self._reset_active() or self.state.robot_resetting:
            raise RuntimeError("Robot reset is already in progress.")

        self.last_reset_error = None
        self.cycle_abort_requested = True
        self.state.robot_running = False
        self.state.robot_paused = False
        self.state.robot_stopped = False
        self.state.robot_resetting = True

        def worker():
            try:
                if self.arm is None:
                    return

                self._terminate_active_cycle()
                reset_recipe = RECIPES_DIR / f"{RESET_PROGRAM}.py"
                if not reset_recipe.exists():
                    raise RuntimeError(f"Reset program not found: {reset_recipe}")

                self.load_recipe(RESET_PROGRAM, reset_recipe)
                self._execute_loaded_program(count_cycle=False)
            except Exception as exc:
                self.last_reset_error = str(exc)
                print(f"Reset command failed: {exc}")
            finally:
                self.state.robot_running = False
                self.state.robot_paused = False
                self.state.robot_resetting = False
                self.recipe_thread = None

        self.reset_thread = threading.Thread(target=worker, daemon=True)
        self.reset_thread.start()

    def emergency_stop(self) -> None:
        self.cycle_abort_requested = True
        self.state.robot_paused = True
        self.state.robot_running = False
        self.dispenser_off()
        if self.arm is not None:
            try:
                self.arm.emergency_stop()
            except Exception as exc:
                print(f"Emergency stop command failed: {exc}")


class GlowPanel(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("GlowPanel")

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = self.rect().adjusted(1, 1, -1, -1)

        gradient = QLinearGradient(rect.topLeft(), rect.bottomRight())
        gradient.setColorAt(0, QColor("#0f172a"))
        gradient.setColorAt(1, QColor("#0b1220"))
        painter.setBrush(gradient)
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


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.robot = RobotController()
        self.recipe_dir = RECIPES_DIR
        self.recipe_dir.mkdir(exist_ok=True)
        # Ignore any held physical inputs on startup until they are released once.
        self.start_latch = self.robot.read_physical_start()
        self.status_cards_compact: Optional[bool] = None

        self.setWindowTitle("Dispensing Cell HMI")
        screen = QGuiApplication.primaryScreen()
        if screen is not None:
            available = screen.availableGeometry()
            self.resize(max(1280, min(available.width(), 1600)), max(720, min(available.height(), 900)))
        else:
            self.resize(1600, 900)

        root = QWidget()
        root.setObjectName("Root")
        self.setCentralWidget(root)
        self.root_layout = QVBoxLayout(root)
        self.root_layout.setContentsMargins(28, 28, 28, 28)
        self.root_layout.setSpacing(18)

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
        self.cycle_count_card = MetricCard("Cycle Count", "0", "blue")
        self.last_cycle_card = MetricCard("Last Cycle Time", "--:--", "purple")
        metrics_wrap.addWidget(self.cycle_count_card)
        metrics_wrap.addWidget(self.last_cycle_card)
        self.hero_layout.addLayout(metrics_wrap)

        self.hero_state = QLabel("SYSTEM IDLE")
        self.hero_state.setObjectName("HeroState")
        self.hero_state.setAlignment(Qt.AlignCenter)
        self.hero_state.setMinimumWidth(210)
        self.hero_layout.addWidget(self.hero_state)

        self.root_layout.addWidget(hero)

        self.content_layout = QGridLayout()
        self.content_layout.setHorizontalSpacing(18)
        self.content_layout.setVerticalSpacing(18)
        self.root_layout.addLayout(self.content_layout, 1)

        left_panel = GlowPanel()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(24, 24, 24, 24)
        left_layout.setSpacing(16)

        recipe_title = QLabel("Program Setup")
        recipe_title.setObjectName("SectionTitle")
        recipe_title.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(recipe_title)

        recipe_help = QLabel("")
        recipe_help.setObjectName("SectionSubtext")
        recipe_help.setWordWrap(True)
        recipe_help.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(recipe_help)

        self.recipe_combo = QComboBox()
        self.recipe_combo.setObjectName("RecipeCombo")
        left_layout.addWidget(self.recipe_combo)

        info_bar = QFrame()
        info_bar.setObjectName("InfoBar")
        info_bar_layout = QVBoxLayout(info_bar)
        info_bar_layout.setContentsMargins(18, 14, 18, 14)
        info_bar_layout.setSpacing(5)

        self.loaded_program_label = QLabel("Loaded Program: None")
        self.loaded_program_label.setObjectName("InfoLabelStrong")
        self.loaded_program_label.setAlignment(Qt.AlignCenter)
        info_bar_layout.addWidget(self.loaded_program_label)

        hint_label = QLabel("")
        hint_label.setObjectName("InfoLabel")
        hint_label.setAlignment(Qt.AlignCenter)
        info_bar_layout.addWidget(hint_label)

        left_layout.addWidget(info_bar)

        self.refresh_button = QPushButton("Refresh Programs")
        self.refresh_button.setObjectName("SecondaryButton")
        self.refresh_button.clicked.connect(self.refresh_recipes)
        left_layout.addWidget(self.refresh_button)
        left_layout.addStretch()

        self.content_layout.addWidget(left_panel, 0, 0)

        middle_panel = GlowPanel()
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

        self.program_card = StatusCard("Loaded Program", "Current recipe")
        self.board_card = StatusCard("Board Presence", "DI0 / channel 8")
        self.dispenser_card = StatusCard("Dispenser", "CO7 epoxy trigger")
        self.motion_card = StatusCard("Robot Motion", "CO6 moving / CO2 stopped")
        self.run_card = StatusCard("Robot State", "Controller state / mode")
        self.enable_card = StatusCard("Robot Enabled", "CI0 low / CO0")
        self.manual_card = StatusCard("Manual Mode", "CI1 low / CO1")
        self.warning_card = StatusCard("Warning", "CO4 / warn_code")
        self.error_card = StatusCard("Error", "CO5 / error_code")
        self.status_card_widgets = (
            self.program_card,
            self.board_card,
            self.dispenser_card,
            self.motion_card,
            self.run_card,
            self.enable_card,
            self.manual_card,
            self.warning_card,
            self.error_card,
        )
        for card in self.status_card_widgets:
            card.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)

        middle_layout.addLayout(self.status_cards)
        middle_layout.addStretch()

        self.content_layout.addWidget(middle_panel, 0, 1)

        right_panel = GlowPanel()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(24, 24, 24, 24)
        right_layout.setSpacing(16)

        control_title = QLabel("Operator Controls")
        control_title.setObjectName("SectionTitle")
        control_title.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(control_title)

        self.start_button = QPushButton("Start Cycle")
        self.start_button.setObjectName("StartButton")
        self.start_button.clicked.connect(self.start_cycle)

        self.pause_button = QPushButton("Pause")
        self.pause_button.setObjectName("PauseButton")
        self.pause_button.clicked.connect(self.pause_cycle)

        self.stop_button = QPushButton("Stop")
        self.stop_button.setObjectName("StopButton")
        self.stop_button.clicked.connect(self.stop_cycle)

        self.reset_button = QPushButton("Reset")
        self.reset_button.setObjectName("ResetButton")
        self.reset_button.clicked.connect(self.reset_system)

        self.manual_dispense_button = QPushButton("Purge Epoxy")
        self.manual_dispense_button.setObjectName("PurgeButton")
        self.manual_dispense_button.clicked.connect(self.run_manual_dispense)

        self.primary_buttons = (
            self.start_button,
            self.pause_button,
            self.stop_button,
            self.reset_button,
            self.manual_dispense_button,
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

        self.refresh_recipes()
        initial_recipe_index = self.recipe_combo.findText(DEFAULT_RECIPE)
        if initial_recipe_index >= 0:
            self.recipe_combo.setCurrentIndex(initial_recipe_index)
            self.loaded_program_label.setText(f"Loaded Program: {DEFAULT_RECIPE}")

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.update_status)
        self.poll_timer.start(300)

        self.apply_styles()
        self.update_responsive_layout(force=True)
        self.update_status()

    def format_cycle_time(self, seconds: float) -> str:
        if seconds <= 0:
            return "--:--"
        whole = int(round(seconds))
        mins = whole // 60
        secs = whole % 60
        return f"{mins:02d}:{secs:02d}"

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

        columns = 3
        for index, card in enumerate(self.status_card_widgets):
            row = index // columns
            column = index % columns
            self.status_cards.addWidget(card, row, column)

        for column in range(columns):
            self.status_cards.setColumnStretch(column, 1)

        status_card_height = 128 if dense else 142
        for card in self.status_card_widgets:
            card.setMinimumHeight(status_card_height)

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

    def apply_styles(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget#Root {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #030712, stop:1 #111827);
            }
            QWidget {
                color: #e5eefc;
                font-family: Segoe UI, Inter, Arial, sans-serif;
                font-size: 14px;
            }
            QLabel {
                background: transparent;
            }
            QLabel#Eyebrow {
                font-size: 11px;
                font-weight: 700;
                color: #7dd3fc;
                letter-spacing: 1px;
            }
            QLabel#LiveCommentary {
                font-size: 18px;
                font-weight: 700;
                color: #f8fbff;
                max-width: 680px;
            }
            QLabel#HeroState {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #0f766e, stop:1 #0369a1);
                border: 1px solid #38bdf8;
                border-radius: 22px;
                padding: 14px;
                font-size: 18px;
                font-weight: 800;
                color: white;
            }
            QLabel#SectionTitle {
                font-size: 18px;
                font-weight: 700;
                color: #f8fbff;
            }
            QLabel#SectionSubtext {
                font-size: 12px;
                color: #93a7c2;
                min-height: 0px;
            }
            QFrame#InfoBar {
                background: rgba(15, 23, 42, 0.88);
                border: 1px solid #27364d;
                border-radius: 18px;
            }
            QLabel#InfoLabelStrong {
                font-size: 14px;
                font-weight: 700;
                color: #dbeafe;
            }
            QLabel#InfoLabel {
                font-size: 12px;
                color: #8ea0b8;
            }
            QFrame#StatusCard {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #0f172a, stop:1 #172235);
                border: 1px solid #25344b;
                border-radius: 22px;
            }
            QLabel#StatusDot {
                font-size: 22px;
                color: #475569;
            }
            QLabel#StatusDot[ok="true"] {
                color: #4ade80;
            }
            QLabel#StatusDot[ok="false"] {
                color: #fb7185;
            }
            QLabel#CardTitle {
                font-size: 12px;
                font-weight: 700;
                color: #e2e8f0;
            }
            QLabel#CardValue {
                font-size: 24px;
                font-weight: 800;
                color: #f8fbff;
            }
            QLabel#CardValue[ok="true"] {
                color: #4ade80;
            }
            QLabel#CardValue[ok="false"] {
                color: #fb7185;
            }
            QLabel#CardSubtitle {
                font-size: 11px;
                color: #8ca0b8;
                min-height: 0px;
            }
            QFrame#MetricCard_blue, QFrame#MetricCard_purple {
                border-radius: 20px;
                min-width: 150px;
            }
            QFrame#MetricCard_blue {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #082f49, stop:1 #0f4c81);
                border: 1px solid #38bdf8;
            }
            QFrame#MetricCard_purple {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #312e81, stop:1 #5b21b6);
                border: 1px solid #a78bfa;
            }
            QLabel#MetricTitle {
                font-size: 11px;
                font-weight: 700;
                color: rgba(255,255,255,0.78);
                text-transform: uppercase;
            }
            QLabel#MetricValue {
                font-size: 22px;
                font-weight: 800;
                color: white;
            }
            QComboBox#RecipeCombo {
                min-height: 44px;
                border-radius: 18px;
                border: 1px solid #334155;
                background: rgba(15, 23, 42, 0.95);
                padding: 10px 16px;
                font-size: 14px;
                font-weight: 600;
                color: #f8fbff;
            }
            QComboBox#RecipeCombo::drop-down {
                border: none;
                width: 40px;
            }
            QPushButton {
                border: none;
                border-radius: 20px;
                padding: 10px 14px;
                font-size: 14px;
                font-weight: 700;
            }
            QPushButton#SecondaryButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #1e293b, stop:1 #334155);
                color: #e2e8f0;
                border: 1px solid #475569;
            }
            QPushButton#StartButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #10b981, stop:1 #059669);
                color: white;
                border: 1px solid #34d399;
            }
            QPushButton#PauseButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #f59e0b, stop:1 #d97706);
                color: white;
                border: 1px solid #fbbf24;
            }
            QPushButton#StopButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #dc2626, stop:1 #b91c1c);
                color: white;
                border: 1px solid #f87171;
            }
            QPushButton#ResetButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #2563eb, stop:1 #1d4ed8);
                color: white;
                border: 1px solid #60a5fa;
            }
            QPushButton#PurgeButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #7c3aed, stop:1 #6d28d9);
                color: white;
                border: 1px solid #a78bfa;
            }
                        QPushButton:hover {
                opacity: 0.95;
            }
            QPushButton:pressed {
                padding-top: 16px;
                padding-bottom: 12px;
            }
            """
        )

    def refresh_recipes(self):
        self.recipe_combo.clear()
        recipe_files = sorted(self.recipe_dir.glob("*.py"))
        for file in recipe_files:
            self.recipe_combo.addItem(file.stem, str(file))
        if self.recipe_combo.count() == 0:
            self.recipe_combo.addItem(DEFAULT_RECIPE, str(self.recipe_dir / f"{DEFAULT_RECIPE}.py"))

    def set_live_commentary(self, text: str):
        self.live_commentary.setText(text)

    def update_status(self):
        board_present = self.robot.read_board_present()
        _cartridge_empty = self.robot.read_cartridge_empty()
        enable_robot = self.robot.read_enable_robot()
        manual_mode = self.robot.read_manual_mode()
        stop_moving = self.robot.read_stop_moving()
        physical_start = self.robot.read_physical_start()
        state_code = self.robot.read_robot_state_code()
        mode_code = self.robot.read_robot_mode_code()
        warning_code = self.robot.read_warning_code()
        error_code = self.robot.read_error_code()

        emergency_stop_active = error_code in {1, 2, 3}
        manual_mode_active = manual_mode or mode_code == 2
        if self.robot.arm is None:
            robot_enabled = enable_robot
        else:
            robot_enabled = error_code == 0 and state_code is not None and state_code < 4
        robot_moving = bool(self.robot.state.robot_running or state_code == 1)
        motion_stopped = bool(stop_moving or state_code in {3, 4} or (not robot_moving and robot_enabled))
        dispenser_on = self.robot.is_dispenser_on()
        loaded_name = self.robot.current_recipe or self.recipe_combo.currentText() or DEFAULT_RECIPE

        self.program_card.set_status(loaded_name, True)
        self.board_card.set_status("DETECTED" if board_present else "NOT DETECTED", board_present )
        self.enable_card.set_status("ENABLED" if robot_enabled else "DISABLED", robot_enabled)
        self.manual_card.set_status("ENABLED" if manual_mode_active else "DISABLED", manual_mode_active, f"Mode code {mode_code if mode_code is not None else '--'}")
        self.warning_card.set_status("ACTIVE" if warning_code else "CLEAR", warning_code == 0, f"warn_code={warning_code}")
        self.error_card.set_status("ACTIVE" if error_code else "CLEAR", error_code == 0, f"error_code={error_code}")
        self.motion_card.set_status("STOPPED" if motion_stopped else "MOVING", not motion_stopped )
        self.dispenser_card.set_status("ON" if dispenser_on else "OFF", dispenser_on)

        if emergency_stop_active:
            self.run_card.set_status("E-STOP", False, f"State {state_code if state_code is not None else '--'} / Mode {mode_code if mode_code is not None else '--'}")
            self.hero_state.setText("ESTOP ACTIVE")
            self.set_live_commentary("Emergency stop is active on the controller")
        elif self.robot.last_reset_error or error_code:
            self.run_card.set_status("FAULT", False, f"State {state_code if state_code is not None else '--'} / Mode {mode_code if mode_code is not None else '--'}")
            self.hero_state.setText("SYSTEM FAULT")
            self.set_live_commentary(self.robot.last_reset_error or f"Robot error code {error_code}")
        elif manual_mode_active:
            self.run_card.set_status("MANUAL", True, f"State {state_code if state_code is not None else '--'} / Mode {mode_code if mode_code is not None else '--'}")
            self.hero_state.setText("MANUAL MODE")
            self.set_live_commentary("Manual mode active: the arm can be guided by hand")
        elif self.robot.state.robot_resetting:
            self.run_card.set_status("RESETTING", False, f"State {state_code if state_code is not None else '--'} / Mode {mode_code if mode_code is not None else '--'}")
            self.hero_state.setText("SYSTEM RESETTING")
            if enable_robot:
                self.set_live_commentary("Moving to home")
            else:
                self.set_live_commentary("Waiting for Button Press")
        elif stop_moving:
            self.run_card.set_status("HOLD", False, f"State {state_code if state_code is not None else '--'} / Mode {mode_code if mode_code is not None else '--'}")
            self.hero_state.setText("MOTION HOLD")
            self.set_live_commentary("Stop Moving input is active on CI3")
        elif self.robot.state.robot_running:
            self.run_card.set_status("RUNNING", True, f"State {state_code if state_code is not None else '--'} / Mode {mode_code if mode_code is not None else '--'}")
            self.hero_state.setText("SYSTEM RUNNING")
            self.set_live_commentary(f"Running program {loaded_name}")
        elif self.robot.state.robot_paused:
            self.run_card.set_status("PAUSED", False, f"State {state_code if state_code is not None else '--'} / Mode {mode_code if mode_code is not None else '--'}")
            self.hero_state.setText("SYSTEM PAUSED")
            self.set_live_commentary("Paused by operator")
        elif self.robot.state.robot_stopped:
            self.run_card.set_status("STOPPED", False, f"State {state_code if state_code is not None else '--'} / Mode {mode_code if mode_code is not None else '--'}")
            self.hero_state.setText("SYSTEM STOPPED")
            self.set_live_commentary("Stopped by operator")
        else:
            self.run_card.set_status("IDLE", robot_enabled, f"State {state_code if state_code is not None else '--'} / Mode {mode_code if mode_code is not None else '--'}")
            self.hero_state.setText("SYSTEM IDLE")
            if warning_code:
                self.set_live_commentary(f"Robot warning code {warning_code}")
            elif not board_present:
                self.set_live_commentary("Waiting for board")
            else:
                self.set_live_commentary(f"Ready to run program {loaded_name}")

        self.cycle_count_card.set_value(str(self.robot.state.cycle_count))
        self.last_cycle_card.set_value(self.format_cycle_time(self.robot.state.last_cycle_seconds))

        if physical_start and not self.start_latch and not self.robot.state.robot_running and not self.robot.state.robot_resetting:
            self.start_latch = True
            self.start_cycle()
        elif not physical_start:
            self.start_latch = False

    def start_cycle(self):
        if self.robot.state.robot_paused and self.robot._cycle_active():
            try:
                self.robot.start_cycle()
                active_name = self.robot.current_recipe or self.recipe_combo.currentText() or DEFAULT_RECIPE
                self.set_live_commentary(f"Resumed program {active_name}")
            except Exception as exc:
                QMessageBox.critical(self, "Resume Failed", str(exc))
            return

        recipe_path_raw = self.recipe_combo.currentData()
        if not recipe_path_raw:
            QMessageBox.warning(self, "No Program", "No dispense program was found in the recipes folder.")
            return

        recipe_path = Path(recipe_path_raw)
        recipe_name = recipe_path.stem

        try:
            self.robot.load_recipe(recipe_name, recipe_path)
            self.loaded_program_label.setText(f"Loaded Program: {recipe_name}")
            self.robot.start_cycle()
        except Exception as exc:
            QMessageBox.critical(self, "Start Failed", str(exc))

    def run_manual_dispense(self):
        try:
            self.robot.run_manual_dispense()
            self.loaded_program_label.setText(f"Loaded Program: {MANUAL_DISPENSE_PROGRAM}")
            self.set_live_commentary("Running epoxy purge")
        except Exception as exc:
            QMessageBox.critical(self, "Purge Failed", str(exc))

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
            self.set_live_commentary("Re-enabling robot")
        except Exception as exc:
            QMessageBox.critical(self, "Reset Failed", str(exc))

    
def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
