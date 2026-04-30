#!/usr/bin/python3
"""PyQt5 GUI for laser-camera boresight calibration.

Launches the C++ laser_boresight tool as a subprocess,
sends commands via stdin, and parses stdout for progress.
"""

import sys
import os
import re
import subprocess
import datetime
from pathlib import Path

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QProgressBar, QTextEdit, QGroupBox,
    QFrame, QSizePolicy, QMessageBox, QShortcut, QGridLayout
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QTextCursor, QColor, QPalette, QKeySequence

# ── 项目路径 ──
PROJECT_DIR = Path(__file__).resolve().parent.parent
BINARY = PROJECT_DIR / "src" / "build" / "calibrate" / "laser_boresight"
CAMERA_CFG = PROJECT_DIR / "config" / "camera.yaml"
SERIAL_PORT = "/dev/ttyUSB0"
SESSION_FILE = PROJECT_DIR / "logs" / "laser_calib_session.yaml"
SNAPSHOT_DIR = PROJECT_DIR / "logs" / "laser_calib_snapshots"

DISTANCES = [0.5, 1.0, 2.0]
REPEATS = 2
TOTAL_SCANS = len(DISTANCES) * REPEATS


class OutputReader(QThread):
    """Read subprocess stdout line-by-line in a background thread."""
    line_received = pyqtSignal(str)
    process_finished = pyqtSignal(int)

    def __init__(self, process):
        super().__init__()
        self.process = process

    def run(self):
        try:
            for line in iter(self.process.stdout.readline, ''):
                if not line:
                    break
                self.line_received.emit(line.rstrip('\n'))
        except Exception:
            pass
        rc = self.process.wait()
        self.process_finished.emit(rc)


class CalibGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.process = None
        self.reader = None
        self.current_scan = 0
        self.state = "idle"  # idle | init | prompt | scanning | done
        self._result_lines = []
        self._serial_ready = False  # C++ tool serial port opened
        self._g_sent = False        # 'g' already sent this session
        self._abort_requested = False
        self._resume_requested = False

        # Log file
        log_dir = PROJECT_DIR / "logs"
        log_dir.mkdir(exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._log_path = log_dir / f"calib_{ts}.log"
        self._log_file = None

        self.setWindowTitle("\u6fc0\u5149\u540c\u8f74\u6821\u51c6\u5de5\u5177")
        self.setMinimumSize(640, 700)
        self.setFocusPolicy(Qt.StrongFocus)
        self._build_ui()
        self._setup_shortcuts()

    # ──────────────────── UI ────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(10)
        root.setContentsMargins(16, 16, 16, 16)

        # Title
        title = QLabel("\u6fc0\u5149-\u76f8\u673a\u540c\u8f74\u6821\u51c6")
        title.setFont(QFont("", 20, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        root.addWidget(title)

        # ── Status ──
        status_box = QGroupBox("\u5f53\u524d\u72b6\u6001")
        sl = QVBoxLayout(status_box)

        self.step_label = QLabel("\u6309\u201c\u5f00\u59cb\u6807\u5b9a\u201d\u542f\u52a8")
        self.step_label.setFont(QFont("", 16, QFont.Bold))
        self.step_label.setAlignment(Qt.AlignCenter)
        self.step_label.setWordWrap(True)
        sl.addWidget(self.step_label)

        self.hint_label = QLabel("")
        self.hint_label.setFont(QFont("", 11))
        self.hint_label.setAlignment(Qt.AlignCenter)
        self.hint_label.setStyleSheet("color: #666;")
        sl.addWidget(self.hint_label)

        self.progress = QProgressBar()
        self.progress.setRange(0, TOTAL_SCANS)
        self.progress.setValue(0)
        self.progress.setTextVisible(True)
        self.progress.setFormat("%v / %m \u6b21\u6e38\u5b8c\u6210")
        self.progress.setMinimumHeight(28)
        sl.addWidget(self.progress)

        root.addWidget(status_box)

        # ── Buttons ──
        btn_row = QHBoxLayout()
        btn_row.setSpacing(10)

        self.btn_start = self._make_btn("\u5f00\u59cb\u6807\u5b9a", "#4CAF50", self._on_start)
        self.btn_space = self._make_btn("\u786e\u8ba4\u505c\u4f4d", "#2196F3", self._on_space)
        self.btn_save = self._make_btn("\u4fdd\u5b58\u7ed3\u679c", "#FF9800", self._on_save)
        self.btn_space.setEnabled(False)
        self.btn_save.setEnabled(False)

        btn_row.addWidget(self.btn_start)
        btn_row.addWidget(self.btn_space)
        btn_row.addWidget(self.btn_save)
        root.addLayout(btn_row)

        # ── Manual gimbal controls ──
        ctrl_box = QGroupBox("云台微调")
        cl = QVBoxLayout(ctrl_box)
        ctrl_hint = QLabel("可直接点按钮，或按 W/A/S/D/0")
        ctrl_hint.setAlignment(Qt.AlignCenter)
        ctrl_hint.setStyleSheet("color: #666;")
        cl.addWidget(ctrl_hint)

        grid = QGridLayout()
        grid.setSpacing(8)
        self.btn_up = self._make_pad_btn("W\n上", lambda: self._hotkey_send('w', "W: pitch 上"))
        self.btn_left = self._make_pad_btn("A\n左", lambda: self._hotkey_send('a', "A: yaw 左"))
        self.btn_front = self._make_pad_btn("0\n前方", lambda: self._hotkey_send('0', "0: 对正前方"))
        self.btn_right = self._make_pad_btn("D\n右", lambda: self._hotkey_send('d', "D: yaw 右"))
        self.btn_down = self._make_pad_btn("S\n下", lambda: self._hotkey_send('s', "S: pitch 下"))
        grid.addWidget(self.btn_up, 0, 1)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_front, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_down, 2, 1)
        cl.addLayout(grid)
        root.addWidget(ctrl_box)

        # ── Results ──
        result_box = QGroupBox("\u6807\u5b9a\u7ed3\u679c")
        rl = QVBoxLayout(result_box)
        self.result_label = QLabel("\u5c1a\u672a\u6807\u5b9a")
        self.result_label.setFont(QFont("Monospace", 11))
        self.result_label.setAlignment(Qt.AlignLeft)
        self.result_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.result_label.setMinimumHeight(120)
        rl.addWidget(self.result_label)
        root.addWidget(result_box)

        # ── Log ──
        log_box = QGroupBox("\u65e5\u5fd7")
        ll = QVBoxLayout(log_box)
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Monospace", 9))
        self.log_text.setMinimumHeight(160)
        ll.addWidget(self.log_text)
        root.addWidget(log_box)

        # ── Quit ──
        quit_btn = QPushButton("\u9000\u51fa")
        quit_btn.setMinimumHeight(36)
        quit_btn.clicked.connect(self._on_quit)
        root.addWidget(quit_btn)

    def _setup_shortcuts(self):
        self._shortcuts = []

        def bind(seq, handler):
            sc = QShortcut(QKeySequence(seq), self)
            sc.setContext(Qt.ApplicationShortcut)
            sc.activated.connect(handler)
            self._shortcuts.append(sc)

        bind("W", lambda: self._hotkey_send('w', "W: pitch 上"))
        bind("S", lambda: self._hotkey_send('s', "S: pitch 下"))
        bind("A", lambda: self._hotkey_send('a', "A: yaw 左"))
        bind("D", lambda: self._hotkey_send('d', "D: yaw 右"))
        bind("0", lambda: self._hotkey_send('0', "0: 对正前方"))
        bind("Space", self._hotkey_space)
        bind("R", self._hotkey_save)

    def _make_btn(self, text, color, slot):
        btn = QPushButton(text)
        btn.setFont(QFont("", 13, QFont.Bold))
        btn.setMinimumHeight(52)
        btn.setCursor(Qt.PointingHandCursor)
        btn.setStyleSheet(
            f"QPushButton {{ background-color: {color}; color: white;"
            f"  border-radius: 8px; padding: 8px 16px; }}"
            f"QPushButton:hover {{ background-color: {color}cc; }}"
            f"QPushButton:disabled {{ background-color: #bbb; color: #888; }}"
        )
        btn.clicked.connect(slot)
        return btn

    def _make_pad_btn(self, text, slot):
        btn = QPushButton(text)
        btn.setFont(QFont("", 12, QFont.Bold))
        btn.setMinimumSize(72, 56)
        btn.setAutoRepeat(True)
        btn.setAutoRepeatDelay(220)
        btn.setAutoRepeatInterval(80)
        btn.setStyleSheet(
            "QPushButton { background-color: #455A64; color: white;"
            " border-radius: 8px; padding: 6px 10px; }"
            "QPushButton:hover { background-color: #546E7A; }"
            "QPushButton:pressed { background-color: #37474F; }"
        )
        btn.clicked.connect(slot)
        return btn

    # ──────────────────── Actions ────────────────────
    def _on_start(self):
        if self.process and self.process.poll() is None:
            # Already running → just send 'g'
            self._send('g')
            self.btn_start.setEnabled(False)
            self.step_label.setText("\u6b63\u5728\u521d\u59cb\u5316...")
            return

        session_args = ["--fresh-session"]
        if self._has_resume_data():
            choice = self._choose_session_action()
            if choice is None:
                return
            self._resume_requested = (choice == "resume")
            session_args = ["--resume-session"] if choice == "resume" else ["--fresh-session"]

        self._reset_state()
        self._serial_ready = False
        self._g_sent = False
        self._abort_requested = False
        self._resume_requested = ("--resume-session" in session_args)
        self.btn_start.setEnabled(False)
        self.step_label.setText("\u6b63\u5728\u542f\u52a8\u76f8\u673a\u548c\u4e32\u53e3...")
        self.hint_label.setText("")

        # Open log file
        self._log_file = open(self._log_path, 'w', encoding='utf-8')
        self._log("\u542f\u52a8 laser_boresight ... \u65e5\u5fd7: " + str(self._log_path))

        self.process = subprocess.Popen(
            [str(BINARY), "--config", str(CAMERA_CFG), "--port", SERIAL_PORT,
             "--session", str(SESSION_FILE), *session_args],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            cwd=str(PROJECT_DIR),
        )

        self.reader = OutputReader(self.process)
        self.reader.line_received.connect(self._on_line)
        self.reader.process_finished.connect(self._on_proc_done)
        self.reader.start()
        # 'g' will be sent automatically when '\u4e32\u53e3:' is detected in stdout

    def _choose_session_action(self):
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Question)
        box.setWindowTitle("发现上次进度")
        box.setText("发现未完成的标定进度或历史快照。")
        box.setInformativeText("继续上次会优先恢复当前会话；只有当前会话不存在或损坏时，才回退历史快照。重新开始会清空当前进度。")
        btn_resume = box.addButton("继续上次", QMessageBox.AcceptRole)
        btn_restart = box.addButton("重新开始", QMessageBox.DestructiveRole)
        btn_cancel = box.addButton("取消", QMessageBox.RejectRole)
        box.setDefaultButton(btn_resume)
        box.exec_()
        clicked = box.clickedButton()
        if clicked == btn_resume:
            return "resume"
        if clicked == btn_restart:
            return "fresh"
        return None

    def _has_resume_data(self):
        if SESSION_FILE.exists():
            return True
        if not SNAPSHOT_DIR.exists():
            return False
        return any(SNAPSHOT_DIR.glob("*.yaml"))

    def _on_space(self):
        self._send(' ')
        self.btn_space.setEnabled(False)
        self.step_label.setText("\u6e38\u5b8c\u4e2d...")
        self.hint_label.setText("\u8bf7\u4fdd\u6301\u8bbe\u521d\u4e0d\u52a8")

    def _on_save(self):
        self._send('r')
        self.btn_save.setEnabled(False)
        self.hint_label.setText("\u5df2\u4fdd\u5b58\u5230 config/control.yaml")

    def _on_quit(self):
        self._stop_process()
        self.close()

    def _stop_process(self):
        if self.process and self.process.poll() is None:
            self._send('q')
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()

    def _hotkey_send(self, key: str, hint: str):
        if self.process and self.process.poll() is None:
            self._send(key)
            self.hint_label.setText(hint)
            self._log(f"[GUI] send {key}")

    def _hotkey_space(self):
        if self.process and self.process.poll() is None and self.btn_space.isEnabled():
            self._on_space()

    def _hotkey_save(self):
        if self.process and self.process.poll() is None and self.btn_save.isEnabled():
            self._on_save()

    def _send(self, key: str):
        if self.process and self.process.poll() is None:
            try:
                self.process.stdin.write(key)
                self.process.stdin.flush()
            except (BrokenPipeError, OSError):
                pass

    def keyPressEvent(self, event):
        if self.process and self.process.poll() is None:
            key = event.key()
            if key == Qt.Key_W:
                self._send('w')
                self.hint_label.setText("W: pitch 上")
                return
            if key == Qt.Key_S:
                self._send('s')
                self.hint_label.setText("S: pitch 下")
                return
            if key == Qt.Key_A:
                self._send('a')
                self.hint_label.setText("A: yaw 左")
                return
            if key == Qt.Key_D:
                self._send('d')
                self.hint_label.setText("D: yaw 右")
                return
            if key == Qt.Key_Space and self.btn_space.isEnabled():
                self._on_space()
                return
            if key == Qt.Key_R and self.btn_save.isEnabled():
                self._on_save()
                return
        super().keyPressEvent(event)

    # ──────────────────── Output parsing ────────────────────
    def _on_line(self, line: str):
        self._log(line)

        # Prompt: [N/M] 请移到 X.Xm 处
        m = re.match(r'\[(\d+)/(\d+)\]\s*请移到\s*([\d.]+)m', line)
        if m:
            idx, total, dist = m.group(1), m.group(2), m.group(3)
            self.state = "prompt"
            self.step_label.setText(f"[{idx}/{total}]  请移到 {dist}m 处")
            self.hint_label.setText("摆好设备后点击「确认就位」")
            self.btn_space.setEnabled(True)
            self.btn_space.setFocus()
            self.current_scan = int(idx) - 1
            self.progress.setValue(self.current_scan)
            return

        # Scan start: [N/M] ...遍开始
        m = re.match(r'\[(\d+)/(\d+)\]\s*.+遍开始', line)
        if m:
            self.state = "scanning"
            self.step_label.setText(f"[{m.group(1)}/{m.group(2)}]  扫描中...")
            self.hint_label.setText("请保持设备不动")
            self.btn_space.setEnabled(False)
            return

        # Manual capture start
        m = re.match(r'\[(\d+)/(\d+)\]\s*.+手动采集中', line)
        if m:
            self.state = "scanning"
            self.step_label.setText(f"[{m.group(1)}/{m.group(2)}]  手动采集中...")
            self.hint_label.setText("可用 WASD/方向按钮微调，0=前方，空格提前结束")
            self.btn_space.setEnabled(True)
            return

        # Auto-start second pass: [N/M] ...自动开始
        m = re.match(r'\[(\d+)/(\d+)\]\s*.+自动开始', line)
        if m:
            self.state = "scanning"
            self.step_label.setText(f"[{m.group(1)}/{m.group(2)}]  同距离第2遍...")
            self.hint_label.setText("请保持设备不动")
            return

        # Spiral progress: [螺旋] 第N圈
        m2 = re.search(r'第(\d+)圈.*?(\d+)样本', line)
        if m2 and self.state == "scanning":
            self.hint_label.setText(f"第{m2.group(1)}圈  {m2.group(2)}个样本")
            return

        # Auto-scan unstable: current pass discarded, wait for retry
        if '扫描失稳' in line:
            self.state = "prompt"
            self.step_label.setText("扫描失稳，本遍已删除")
            self.hint_label.setText("前一遍已保留，请手动调整后按「确认就位」")
            self.btn_space.setEnabled(False)
            self.btn_save.setEnabled(False)
            return

        # Link warning while still running
        if '未收到云台回传' in line:
            self.hint_label.setText("云台回传中断，请检查串口/下位机")

        # All done
        if '全部完成' in line:
            self.state = "done"
            m3 = re.search(r'(\d+)\s*个样本', line)
            cnt = m3.group(1) if m3 else "?"
            self.step_label.setText(f"标定完成!  {cnt} 个样本")
            self.hint_label.setText("检查结果后点击「保存结果」")
            self.btn_save.setEnabled(True)
            self.btn_save.setFocus()
            self.progress.setValue(TOTAL_SCANS)
            return

        # Calibration result block
        if any(k in line for k in ['laser_offset_x', 'laser_offset_y',
                                     'yaw:', 'pitch:', 'RMS_u']):
            if 'laser_offset_x' in line:
                self._result_lines.clear()
            self._result_lines.append(line.strip())
            self.result_label.setText('\n'.join(self._result_lines))

        # Saved confirmation
        if 'Saved to' in line:
            self.hint_label.setText("已保存!")

        # Serial port ready
        if '\u4e32\u53e3:' in line:
            self._serial_ready = True
            QTimer.singleShot(1500, self._send_g)

    def _send_g(self):
        if self._serial_ready and not self._g_sent and not self._resume_requested:
            self._send('g')
            self._g_sent = True

    def _on_proc_done(self, rc):
        self._log(f"[进程退出 code={rc}]")
        self.btn_start.setEnabled(True)
        self.btn_start.setText("重新开始")
        self.btn_space.setEnabled(False)
        if self.state == "aborted":
            self.hint_label.setText("已停止，点击「重新开始」")
        elif self.state != "done":
            self.step_label.setText("已结束")
            self.hint_label.setText("点击「重新开始」再次标定")

    # ──────────────────── Helpers ────────────────────
    def _log(self, text: str):
        self.log_text.append(text)
        self.log_text.moveCursor(QTextCursor.End)
        if self._log_file:
            try:
                self._log_file.write(text + '\n')
                self._log_file.flush()
            except Exception:
                pass

    def _reset_state(self):
        self.current_scan = 0
        self.state = "idle"
        self._result_lines.clear()
        self.progress.setValue(0)
        self.result_label.setText("\u5c1a\u672a\u6807\u5b9a")
        self.btn_save.setEnabled(False)
        self.btn_space.setEnabled(False)
        self.log_text.clear()
        if self._log_file:
            self._log_file.close()
            self._log_file = None

    def closeEvent(self, event):
        self._stop_process()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    gui = CalibGUI()
    gui.show()
    sys.exit(app.exec_())
