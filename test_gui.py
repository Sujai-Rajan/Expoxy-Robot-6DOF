import sys
import time
import importlib.util
import threading
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QLinearGradient
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


# xArm CGPIO mapping from exported WebUI programs:
# CI0-CI7 => 0-7
# DI0-DI7 => 8-15
# CO0-CO7 => 0-7
# DO0-DO7 => 8-15
BOARD_PRESENT_DI = 0          # DI0 => CGPIO channel 8
CARTRIDGE_EMPTY_DI = 1        # DI1 => CGPIO channel 9
PHYSICAL_START_CI = 4         # CI4 => CGPIO channel 4
PHYSICAL_ESTOP_CI = 2         # CI2 => CGPIO channel 2
DISPENSER_TRIGGER_CO = 7      # CO7 => CGPIO channel 7
MANUAL_DISPENSE_PROGRAM = "DISPENSE_TEST"
ENABLE_CARTRIDGE_CHECK = False
DEFAULT_RECIPE = "PCB_466"
XARM_IP = "192.168.1.196"  # replace later


@dataclass
class SensorState:
    board_present: bool = False
    cartridge_empty: bool = False
    robot_running: bool = False
    robot_paused: bool = False
    cycle_count: int = 0
    last_cycle_seconds: float = 0.0


class RobotController:
    def __init__(self):
        self.state = SensorState()
        self.current_recipe: Optional[str] = None
        self.current_recipe_module = None
        self.arm = None
        self.recipe_thread = None
        self.connect_robot()

    def connect_robot(self) -> None:
        try:
            from xarm.wrapper import XArmAPI

            self.arm = XArmAPI(XARM_IP)
            self.arm.connect()
            self.arm.motion_enable(enable=True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
        except Exception as exc:
            print(f"Robot connection not active yet: {exc}")
            self.arm = None

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

    def read_physical_estop(self) -> bool:
        return self.read_ci(PHYSICAL_ESTOP_CI)

    def set_co(self, index: int, state: bool) -> None:
        channel = self.co_channel(index)
        if self.arm is None:
            print(f"CO{index} / channel {channel} -> {'ON' if state else 'OFF'}")
            return
        try:
            code = self.arm.set_cgpio_digital(channel, int(state), delay_sec=0, sync=True)
            if code != 0:
                print(f"Failed to set CO{index}, code={code}")
        except Exception as exc:
            print(f"Failed to set CO{index}: {exc}")

    def dispenser_off(self) -> None:
        self.set_co(DISPENSER_TRIGGER_CO, False)

    def load_recipe(self, recipe_name: str, recipe_path: Path) -> None:
        spec = importlib.util.spec_from_file_location(recipe_name, recipe_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"Could not load recipe: {recipe_path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.current_recipe = recipe_name
        self.current_recipe_module = module

    def _run_loaded_recipe(self) -> None:
        start_time = time.perf_counter()
        try:
            module = self.current_recipe_module
            if module is None:
                raise RuntimeError("No recipe module loaded.")
            if hasattr(module, 'RobotMain'):
                robot_main = module.RobotMain(self.arm)
                robot_main.run()
            elif hasattr(module, 'run'):
                module.run(self)
            else:
                raise RuntimeError("Recipe file has neither RobotMain nor run().")
            elapsed = time.perf_counter() - start_time
            self.state.last_cycle_seconds = elapsed
            self.state.cycle_count += 1
        except Exception as exc:
            print(f"Recipe run failed: {exc}")
        finally:
            self.dispenser_off()
            self.state.robot_running = False

    def start_cycle(self) -> None:
        if ENABLE_CARTRIDGE_CHECK and self.read_cartridge_empty():
            raise RuntimeError("Cannot start cycle: dispenser cartridge is empty.")
        if not self.read_board_present():
            raise RuntimeError("Cannot start cycle: board not present.")
        if self.current_recipe_module is None:
            raise RuntimeError("No recipe loaded.")
        if self.recipe_thread is not None and self.recipe_thread.is_alive():
            raise RuntimeError("A recipe is already running.")

        self.state.robot_running = True
        self.state.robot_paused = False
        self.recipe_thread = threading.Thread(target=self._run_loaded_recipe, daemon=True)
        self.recipe_thread.start()

    def run_manual_dispense(self) -> None:
        if self.recipe_thread is not None and self.recipe_thread.is_alive():
            raise RuntimeError("Cannot run manual dispense while another program is active.")
        manual_recipe = Path("./recipes") / f"{MANUAL_DISPENSE_PROGRAM}.py"
        if not manual_recipe.exists():
            raise RuntimeError(f"Manual dispense program not found: {manual_recipe}")
        self.load_recipe(MANUAL_DISPENSE_PROGRAM, manual_recipe)
        self.state.robot_running = True
        self.state.robot_paused = False
        self.recipe_thread = threading.Thread(target=self._run_loaded_recipe, daemon=True)
        self.recipe_thread.start()

    def pause_immediate(self) -> None:
        self.state.robot_paused = True
        self.state.robot_running = False
        self.dispenser_off()
        if self.arm is not None:
            try:
                self.arm.set_state(3)
            except Exception as exc:
                print(f"Pause command failed: {exc}")

    def emergency_stop(self) -> None:
        self.state.robot_paused = True
        self.state.robot_running = False
        self.dispenser_off()
        if self.arm is not None:
            try:
                self.arm.set_state(4)
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
        self.recipe_dir = Path("./recipes")
        self.recipe_dir.mkdir(exist_ok=True)
        self.start_latch = False
        self.estop_latch = False

        self.setWindowTitle("RTV Dispensing Cell HMI")
        self.resize(1460, 900)

        root = QWidget()
        self.setCentralWidget(root)
        root_layout = QVBoxLayout(root)
        root_layout.setContentsMargins(28, 28, 28, 28)
        root_layout.setSpacing(18)

        hero = GlowPanel()
        hero_layout = QHBoxLayout(hero)
        hero_layout.setContentsMargins(28, 22, 28, 22)
        hero_layout.setSpacing(20)

        title_block = QVBoxLayout()
        title_block.setSpacing(4)

        eyebrow = QLabel("UFACTORY XARM 850 · RTV DISPENSING CELL")
        eyebrow.setObjectName("Eyebrow")
        title_block.addWidget(eyebrow)

        header = QLabel("Operator Control Dashboard")
        header.setObjectName("Header")
        title_block.addWidget(header)

        sub = QLabel("Premium lightweight HMI for recipe selection, live status, and direct cycle control.")
        sub.setObjectName("HeroSubtext")
        sub.setWordWrap(True)
        title_block.addWidget(sub)

        hero_layout.addLayout(title_block, 1)

        metrics_wrap = QHBoxLayout()
        metrics_wrap.setSpacing(12)
        self.cycle_count_card = MetricCard("Cycle Count", "0", "blue")
        self.last_cycle_card = MetricCard("Last Cycle Time", "--:--", "purple")
        metrics_wrap.addWidget(self.cycle_count_card)
        metrics_wrap.addWidget(self.last_cycle_card)
        hero_layout.addLayout(metrics_wrap)

        self.hero_state = QLabel("SYSTEM IDLE")
        self.hero_state.setObjectName("HeroState")
        self.hero_state.setAlignment(Qt.AlignCenter)
        self.hero_state.setMinimumWidth(210)
        hero_layout.addWidget(self.hero_state)

        root_layout.addWidget(hero)

        content = QGridLayout()
        content.setHorizontalSpacing(18)
        content.setVerticalSpacing(18)
        root_layout.addLayout(content, 1)

        left_panel = GlowPanel()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(24, 24, 24, 24)
        left_layout.setSpacing(16)

        recipe_title = QLabel("Program Setup")
        recipe_title.setObjectName("SectionTitle")
        recipe_title.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(recipe_title)

        recipe_help = QLabel("Select the board recipe to run. Board presence is read from DI0 using CGPIO channel 8.")
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

        hint_label = QLabel("Programs are loaded from the local recipes folder.")
        hint_label.setObjectName("InfoLabel")
        hint_label.setAlignment(Qt.AlignCenter)
        info_bar_layout.addWidget(hint_label)

        left_layout.addWidget(info_bar)

        self.refresh_button = QPushButton("Refresh Programs")
        self.refresh_button.setObjectName("SecondaryButton")
        self.refresh_button.clicked.connect(self.refresh_recipes)
        left_layout.addWidget(self.refresh_button)
        left_layout.addStretch()

        content.addWidget(left_panel, 0, 0)

        middle_panel = GlowPanel()
        middle_layout = QVBoxLayout(middle_panel)
        middle_layout.setContentsMargins(24, 24, 24, 24)
        middle_layout.setSpacing(16)

        status_title = QLabel("Live Machine Status")
        status_title.setObjectName("SectionTitle")
        status_title.setAlignment(Qt.AlignCenter)
        middle_layout.addWidget(status_title)

        cards = QGridLayout()
        cards.setHorizontalSpacing(14)
        cards.setVerticalSpacing(14)

        self.board_card = StatusCard("Board Presence", "DI0 / channel 8")
        self.cart_card = StatusCard("Cartridge Status", "DI1 / channel 9")
        self.run_card = StatusCard("Robot State", "")

        cards.addWidget(self.board_card, 0, 0)
        cards.addWidget(self.cart_card, 0, 1)
        cards.addWidget(self.run_card, 1, 0, 1, 2)
        middle_layout.addLayout(cards)
        middle_layout.addStretch()

        content.addWidget(middle_panel, 0, 1)

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

        self.manual_dispense_button = QPushButton("Run Dispense Program")
        self.manual_dispense_button.setObjectName("SecondaryButton")
        self.manual_dispense_button.clicked.connect(self.run_manual_dispense)

        self.pause_button = QPushButton("Pause")
        self.pause_button.setObjectName("PauseButton")
        self.pause_button.clicked.connect(self.pause_cycle)

        self.estop_button = QPushButton("Emergency Stop")
        self.estop_button.setObjectName("EStopButton")
        self.estop_button.clicked.connect(self.emergency_stop)

        for btn in (self.start_button, self.manual_dispense_button, self.pause_button, self.estop_button):
            btn.setMinimumHeight(68)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            right_layout.addWidget(btn)

        right_layout.addStretch()
        content.addWidget(right_panel, 0, 2)

        content.setColumnStretch(0, 1)
        content.setColumnStretch(1, 1)
        content.setColumnStretch(2, 1)

        self.refresh_recipes()
        initial_recipe_index = self.recipe_combo.findText(DEFAULT_RECIPE)
        if initial_recipe_index >= 0:
            self.recipe_combo.setCurrentIndex(initial_recipe_index)
            self.loaded_program_label.setText(f"Loaded Program: {DEFAULT_RECIPE}")

        self.poll_timer = QTimer(self)
        self.poll_timer.timeout.connect(self.update_status)
        self.poll_timer.start(300)

        self.apply_styles()
        self.update_status()

    def format_cycle_time(self, seconds: float) -> str:
        if seconds <= 0:
            return "--:--"
        whole = int(round(seconds))
        mins = whole // 60
        secs = whole % 60
        return f"{mins:02d}:{secs:02d}"

    def apply_styles(self):
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #030712, stop:1 #111827);
                color: #e5eefc;
                font-family: Segoe UI, Inter, Arial, sans-serif;
                font-size: 14px;
            }
            QLabel#Eyebrow {
                font-size: 12px;
                font-weight: 700;
                color: #7dd3fc;
                letter-spacing: 1px;
            }
            QLabel#Header {
                font-size: 32px;
                font-weight: 800;
                color: #f8fbff;
            }
            QLabel#HeroSubtext {
                font-size: 14px;
                color: #9fb0c7;
                max-width: 640px;
            }
            QLabel#HeroState {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #0f766e, stop:1 #0369a1);
                border: 1px solid #38bdf8;
                border-radius: 22px;
                padding: 18px;
                font-size: 20px;
                font-weight: 800;
                color: white;
            }
            QLabel#SectionTitle {
                font-size: 21px;
                font-weight: 700;
                color: #f8fbff;
            }
            QLabel#SectionSubtext {
                font-size: 13px;
                color: #93a7c2;
            }
            QFrame#InfoBar {
                background: rgba(15, 23, 42, 0.88);
                border: 1px solid #27364d;
                border-radius: 18px;
            }
            QLabel#InfoLabelStrong {
                font-size: 16px;
                font-weight: 700;
                color: #dbeafe;
            }
            QLabel#InfoLabel {
                font-size: 13px;
                color: #8ea0b8;
            }
            QFrame#StatusCard {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #0f172a, stop:1 #172235);
                border: 1px solid #25344b;
                border-radius: 22px;
            }
            QLabel#StatusDot {
                font-size: 26px;
                color: #475569;
            }
            QLabel#StatusDot[ok="true"] {
                color: #4ade80;
            }
            QLabel#StatusDot[ok="false"] {
                color: #fb7185;
            }
            QLabel#CardTitle {
                font-size: 14px;
                font-weight: 700;
                color: #e2e8f0;
            }
            QLabel#CardValue {
                font-size: 27px;
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
                font-size: 12px;
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
                font-size: 12px;
                font-weight: 700;
                color: rgba(255,255,255,0.78);
                text-transform: uppercase;
            }
            QLabel#MetricValue {
                font-size: 28px;
                font-weight: 800;
                color: white;
            }
            QComboBox#RecipeCombo {
                min-height: 54px;
                border-radius: 18px;
                border: 1px solid #334155;
                background: rgba(15, 23, 42, 0.95);
                padding: 10px 16px;
                font-size: 16px;
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
                padding: 14px 18px;
                font-size: 16px;
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
            QPushButton#EStopButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #ef4444, stop:1 #dc2626);
                color: white;
                border: 1px solid #f87171;
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
            self.recipe_combo.addItem(DEFAULT_RECIPE, f"recipes/{DEFAULT_RECIPE}.py")

    def update_status(self):
        board_present = self.robot.read_board_present()
        cartridge_empty = self.robot.read_cartridge_empty()
        physical_start = self.robot.read_physical_start()
        physical_estop = self.robot.read_physical_estop()

        self.board_card.set_status("DETECTED" if board_present else "NOT DETECTED", board_present, "DI0")
        if ENABLE_CARTRIDGE_CHECK:
            self.cart_card.set_status("EMPTY" if cartridge_empty else "OK", not cartridge_empty, "DI1")
        else:
            self.cart_card.set_status("BYPASSED", True, "DI1")

        if self.robot.state.robot_running:
            self.run_card.set_status("RUNNING", True, "")
            self.hero_state.setText("SYSTEM RUNNING")
        elif self.robot.state.robot_paused:
            self.run_card.set_status("PAUSED", False, "")
            self.hero_state.setText("SYSTEM PAUSED")
        else:
            self.run_card.set_status("IDLE", True, "")
            self.hero_state.setText("SYSTEM IDLE")

        self.cycle_count_card.set_value(str(self.robot.state.cycle_count))
        self.last_cycle_card.set_value(self.format_cycle_time(self.robot.state.last_cycle_seconds))

        if physical_estop and not self.estop_latch:
            self.estop_latch = True
            self.robot.emergency_stop()
        elif not physical_estop:
            self.estop_latch = False

        if physical_start and not self.start_latch and not self.robot.state.robot_running and not self.robot.state.robot_paused:
            self.start_latch = True
            self.start_cycle()
        elif not physical_start:
            self.start_latch = False

    def start_cycle(self):
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
        except Exception as exc:
            QMessageBox.critical(self, "Manual Dispense Failed", str(exc))

    def pause_cycle(self):
        try:
            self.robot.pause_immediate()
        except Exception as exc:
            QMessageBox.critical(self, "Pause Failed", str(exc))

    def emergency_stop(self):
        try:
            self.robot.emergency_stop()
        except Exception as exc:
            QMessageBox.critical(self, "Emergency Stop Failed", str(exc))


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
