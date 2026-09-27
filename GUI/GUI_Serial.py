import sys
import struct
import zlib
import serial
import serial.tools.list_ports
from collections import deque

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QDoubleSpinBox, QSpinBox, QPushButton,
    QComboBox, QTabWidget, QGroupBox, QStatusBar
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QFont
import pyqtgraph as pg

# ==============================================================================
# STRUKTUR PACKET DAN PROTOKOL SERIAL
# ==============================================================================
# Format Packet To Send (dari Arduino ke PC):
# uint32_t tick, float current_rpm, float target_rpm_pwm, float current_mA, float bus_voltage_V
TELEMETRY_DATA_FORMAT = '<Iffff'
TELEMETRY_DATA_SIZE = struct.calcsize(TELEMETRY_DATA_FORMAT)  # 20 bytes

# Total ukuran frame: 3 (SOF) + 1 (LEN) + 20 (DATA) + 4 (CRC32) = 28 bytes
SOF_BYTES = bytes([0xAA, 0xBB, 0xCC])
FRAME_TOTAL_LEN = 3 + 1 + TELEMETRY_DATA_SIZE + 4

# Format Packet To Receive (dari PC ke Arduino):
# uint32_t tick, uint8_t mode, float target, float m, float c, float kp, float ki, float kd, float controll_period_ms
COMMAND_DATA_FORMAT = '<IBfffffff'


class SerialWorker(QThread):
    packet_received = pyqtSignal(dict)
    connection_status = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self.ser = None
        self.running = False
        self.circular_buffer = bytearray()

    def connect_port(self, port_name, baudrate=115200):
        try:
            self.ser = serial.Serial(port_name, baudrate, timeout=0.05)
            self.running = True
            self.circular_buffer.clear()
            self.start()
            self.connection_status.emit(True, f"Connected: {port_name} @ {baudrate}")
        except Exception as e:
            self.connection_status.emit(False, str(e))

    def disconnect_port(self):
        self.running = False
        self.wait(500)
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.connection_status.emit(False, "Disconnected")

    def send_command(self, tick, mode, target, m, c, kp, ki, kd, controll_period_ms):
        if not self.ser or not self.ser.is_open:
            return

        payload = struct.pack(COMMAND_DATA_FORMAT, tick, mode, target, m, c, kp, ki, kd, controll_period_ms)
        payload_len = len(payload)
        
        # Hitung CRC32 dari len + payload (IEEE 802.3)
        crc_data = bytes([payload_len]) + payload
        crc32_val = zlib.crc32(crc_data) & 0xFFFFFFFF

        frame = SOF_BYTES + bytes([payload_len]) + payload + struct.pack('<I', crc32_val)
        try:
            self.ser.write(frame)
        except Exception as e:
            print(f"Write error: {e}")

    def run(self):
        while self.running:
            if not self.ser or not self.ser.is_open:
                break

            try:
                raw = self.ser.read(self.ser.in_waiting or 1)
                if not raw:
                    continue
                self.circular_buffer.extend(raw)

                # Circular Searching / Sliding Window Frame Extraction
                while len(self.circular_buffer) >= FRAME_TOTAL_LEN:
                    # 1. Cari SOF 0xAA, 0xBB, 0xCC
                    sof_index = self.circular_buffer.find(SOF_BYTES)
                    if sof_index == -1:
                        self.circular_buffer = self.circular_buffer[-2:]
                        break

                    # Geser buffer jika index bukan 0 (buang noise byte)
                    if sof_index > 0:
                        del self.circular_buffer[:sof_index]

                    if len(self.circular_buffer) < FRAME_TOTAL_LEN:
                        break

                    # 2. Cek Payload Length
                    p_len = self.circular_buffer[3]
                    if p_len != TELEMETRY_DATA_SIZE:
                        del self.circular_buffer[0]
                        continue

                    # 3. Validasi CRC32
                    candidate_frame = self.circular_buffer[:FRAME_TOTAL_LEN]
                    crc_target = candidate_frame[3:4 + TELEMETRY_DATA_SIZE]  # len + payload
                    computed_crc = zlib.crc32(crc_target) & 0xFFFFFFFF
                    received_crc = struct.unpack('<I', candidate_frame[-4:])[0]

                    if computed_crc == received_crc:
                        payload_raw = candidate_frame[4:4 + TELEMETRY_DATA_SIZE]
                        unpacked = struct.unpack(TELEMETRY_DATA_FORMAT, payload_raw)
                        data = {
                            "tick": unpacked[0],
                            "current_rpm": unpacked[1],
                            "target_rpm_pwm": unpacked[2],
                            "current_mA": unpacked[3],
                            "bus_voltage_V": unpacked[4]
                        }
                        self.packet_received.emit(data)
                        del self.circular_buffer[:FRAME_TOTAL_LEN]
                    else:
                        del self.circular_buffer[0]

            except Exception:
                break


# ==============================================================================
# GUI UTAMA (PYQT5)
# ==============================================================================
class MotorControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DC Motor Control Studio - Telemetry & PID Tuning")
        self.resize(1280, 750)

        self.worker = SerialWorker()
        self.worker.packet_received.connect(self.on_packet_received)
        self.worker.connection_status.connect(self.on_connection_status)

        # Buffer Data Grafik RPM
        self.buffer_size = 200
        self.time_history = deque(maxlen=self.buffer_size)
        self.rpm_act_history = deque(maxlen=self.buffer_size)
        self.rpm_tgt_history = deque(maxlen=self.buffer_size)

        self.active_mode = 0  # 0: Open Loop, 1: Closed Loop

        # State Parameter Kontrol Terkini (dikirim berkala 10 Hz)
        self.cmd_target = 0.0
        self.cmd_m = 1.0
        self.cmd_c = 0.0
        self.cmd_kp = 1.20
        self.cmd_ki = 0.80
        self.cmd_kd = 0.01
        self.cmd_period = 100.0

        self.setup_ui()
        self.setup_theme()

        # Timer transmisi serial tepat 10 Hz (100 ms)
        self.tx_timer = QTimer()
        self.tx_timer.setInterval(100)
        self.tx_timer.timeout.connect(self.send_10hz_command)

        # Timer scan port berkala
        self.port_scan_timer = QTimer()
        self.port_scan_timer.timeout.connect(self.refresh_ports)
        self.port_scan_timer.start(2000)
        self.refresh_ports()

    def setup_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        main_layout.setContentsMargins(12, 12, 12, 12)
        main_layout.setSpacing(12)

        # ---------------- SISI KIRI: KONTROL PANEL ----------------
        left_panel = QVBoxLayout()
        left_panel.setSpacing(10)

        # Panel Koneksi Serial
        conn_group = QGroupBox("Port Serial")
        conn_layout = QGridLayout(conn_group)
        self.port_combo = QComboBox()
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "57600", "9600"])
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        
        conn_layout.addWidget(QLabel("Port:"), 0, 0)
        conn_layout.addWidget(self.port_combo, 0, 1)
        conn_layout.addWidget(QLabel("Baud:"), 1, 0)
        conn_layout.addWidget(self.baud_combo, 1, 1)
        conn_layout.addWidget(self.btn_connect, 2, 0, 1, 2)
        left_panel.addWidget(conn_group)

        # Telemetri Realtime Card (Teks Box Saja)
        telemetry_group = QGroupBox("Status Daya & Kecepatan")
        telemetry_layout = QGridLayout(telemetry_group)
        
        self.lbl_v_act = QLabel("0.00 V")
        self.lbl_v_act.setFont(QFont("Monospace", 14, QFont.Bold))
        self.lbl_v_act.setStyleSheet("color: #38bdf8;")

        self.lbl_i_act = QLabel("0 mA")
        self.lbl_i_act.setFont(QFont("Monospace", 14, QFont.Bold))
        self.lbl_i_act.setStyleSheet("color: #f59e0b;")

        self.lbl_rpm_act = QLabel("0.0 RPM")
        self.lbl_rpm_act.setFont(QFont("Monospace", 16, QFont.Bold))
        self.lbl_rpm_act.setStyleSheet("color: #10b981;")

        self.lbl_rpm_tgt = QLabel("0.0")
        self.lbl_rpm_tgt.setFont(QFont("Monospace", 16, QFont.Bold))
        self.lbl_rpm_tgt.setStyleSheet("color: #f43f5e;")

        telemetry_layout.addWidget(QLabel("Tegangan Bus:"), 0, 0)
        telemetry_layout.addWidget(self.lbl_v_act, 0, 1)
        telemetry_layout.addWidget(QLabel("Arus Motor:"), 1, 0)
        telemetry_layout.addWidget(self.lbl_i_act, 1, 1)
        telemetry_layout.addWidget(QLabel("Actual RPM:"), 2, 0)
        telemetry_layout.addWidget(self.lbl_rpm_act, 2, 1)
        telemetry_layout.addWidget(QLabel("Target Received:"), 3, 0)
        telemetry_layout.addWidget(self.lbl_rpm_tgt, 3, 1)
        left_panel.addWidget(telemetry_group)

        # Mode Tab: Open Loop vs Closed Loop
        self.tabs = QTabWidget()
        self.tab_open = QWidget()
        self.tab_closed = QWidget()
        self.setup_open_loop_tab()
        self.setup_closed_loop_tab()
        
        self.tabs.addTab(self.tab_open, "Open Loop (PWM)")
        self.tabs.addTab(self.tab_closed, "Closed Loop (PID)")
        self.tabs.currentChanged.connect(self.on_tab_changed)
        left_panel.addWidget(self.tabs)

        left_panel.addStretch()
        main_layout.addLayout(left_panel, stretch=1)

        # ---------------- SISI KANAN: GRAFIK TELEMETRI RPM ----------------
        right_panel = QVBoxLayout()
        
        pg.setConfigOptions(antialias=True)
        self.plot_widget_rpm = pg.PlotWidget(title="<b>Response: Actual RPM vs Target (10 Hz Tx Stream)</b>")
        self.plot_widget_rpm.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget_rpm.setLabel('left', 'RPM')
        self.plot_widget_rpm.setLabel('bottom', 'Waktu (s)')
        self.plot_widget_rpm.addLegend(offset=(-10, 10))
        self.curve_rpm_act = self.plot_widget_rpm.plot(pen=pg.mkPen('#10b981', width=2), name="Actual RPM")
        self.curve_rpm_tgt = self.plot_widget_rpm.plot(pen=pg.mkPen('#f43f5e', width=2, style=Qt.DashLine), name="Target RPM/PWM")

        right_panel.addWidget(self.plot_widget_rpm)
        main_layout.addLayout(right_panel, stretch=3)

        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)

    def setup_open_loop_tab(self):
        layout = QVBoxLayout(self.tab_open)
        grid = QGridLayout()
        
        self.spin_pwm = QSpinBox()
        self.spin_pwm.setRange(-255, 255)
        self.spin_pwm.setValue(0)
        self.spin_pwm.setSingleStep(5)

        self.spin_open_period = QDoubleSpinBox()
        self.spin_open_period.setRange(5.0, 1000.0)
        self.spin_open_period.setValue(100.0)
        self.spin_open_period.setSuffix(" ms")

        btn_send_pwm = QPushButton("Update Output PWM")
        btn_send_pwm.clicked.connect(self.apply_open_loop)

        btn_stop = QPushButton("EMERGENCY STOP")
        btn_stop.setStyleSheet("background-color: #ef4444; color: white; font-weight: bold;")
        btn_stop.clicked.connect(self.emergency_stop)

        grid.addWidget(QLabel("Direct PWM (-255 s.d 255):"), 0, 0)
        grid.addWidget(self.spin_pwm, 0, 1)
        grid.addWidget(QLabel("Control Period:"), 1, 0)
        grid.addWidget(self.spin_open_period, 1, 1)
        grid.addWidget(btn_send_pwm, 2, 0, 1, 2)
        grid.addWidget(btn_stop, 3, 0, 1, 2)
        
        layout.addLayout(grid)
        layout.addStretch()

    def setup_closed_loop_tab(self):
        layout = QVBoxLayout(self.tab_closed)
        grid = QGridLayout()

        self.spin_target_rpm = QDoubleSpinBox()
        self.spin_target_rpm.setRange(-500.0, 500.0)
        self.spin_target_rpm.setValue(30.0)

        self.spin_closed_period = QDoubleSpinBox()
        self.spin_closed_period.setRange(5.0, 1000.0)
        self.spin_closed_period.setValue(100.0)
        self.spin_closed_period.setSuffix(" ms")

        self.spin_kp = QDoubleSpinBox()
        self.spin_kp.setDecimals(3)
        self.spin_kp.setRange(0.0, 50.0)
        self.spin_kp.setValue(1.20)

        self.spin_ki = QDoubleSpinBox()
        self.spin_ki.setDecimals(3)
        self.spin_ki.setRange(0.0, 50.0)
        self.spin_ki.setValue(0.80)

        self.spin_kd = QDoubleSpinBox()
        self.spin_kd.setDecimals(4)
        self.spin_kd.setRange(0.0, 10.0)
        self.spin_kd.setValue(0.01)

        self.spin_ff_m = QDoubleSpinBox()
        self.spin_ff_m.setDecimals(3)
        self.spin_ff_m.setRange(0.001, 100.0)
        self.spin_ff_m.setValue(1.0)

        self.spin_ff_c = QDoubleSpinBox()
        self.spin_ff_c.setDecimals(2)
        self.spin_ff_c.setRange(-255.0, 255.0)
        self.spin_ff_c.setValue(0.0)

        btn_send_pid = QPushButton("Update Parameter & Target")
        btn_send_pid.clicked.connect(self.apply_closed_loop)

        grid.addWidget(QLabel("Target RPM:"), 0, 0)
        grid.addWidget(self.spin_target_rpm, 0, 1)
        grid.addWidget(QLabel("Control Period:"), 1, 0)
        grid.addWidget(self.spin_closed_period, 1, 1)
        grid.addWidget(QLabel("Kp:"), 2, 0)
        grid.addWidget(self.spin_kp, 2, 1)
        grid.addWidget(QLabel("Ki:"), 3, 0)
        grid.addWidget(self.spin_ki, 3, 1)
        grid.addWidget(QLabel("Kd:"), 4, 0)
        grid.addWidget(self.spin_kd, 4, 1)
        grid.addWidget(QLabel("FF (m):"), 5, 0)
        grid.addWidget(self.spin_ff_m, 5, 1)
        grid.addWidget(QLabel("FF (c):"), 6, 0)
        grid.addWidget(self.spin_ff_c, 6, 1)
        grid.addWidget(btn_send_pid, 7, 0, 1, 2)

        layout.addLayout(grid)
        layout.addStretch()

    def refresh_ports(self):
        current = self.port_combo.currentText()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports != [self.port_combo.itemText(i) for i in range(self.port_combo.count())]:
            self.port_combo.clear()
            self.port_combo.addItems(ports)
            idx = self.port_combo.findText(current)
            if idx >= 0:
                self.port_combo.setCurrentIndex(idx)

    def toggle_connection(self):
        if self.worker.running:
            self.tx_timer.stop()
            self.worker.disconnect_port()
            self.btn_connect.setText("Connect")
            self.btn_connect.setStyleSheet("")
        else:
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            if port:
                self.worker.connect_port(port, baud)

    def on_connection_status(self, is_connected, message):
        self.status_bar.showMessage(message)
        if is_connected:
            self.btn_connect.setText("Disconnect")
            self.btn_connect.setStyleSheet("background-color: #10b981; color: white;")
            # Terapkan konfigurasi awal sesuai tab dan mulai timer transmisi 10 Hz
            if self.active_mode == 0:
                self.apply_open_loop()
            else:
                self.apply_closed_loop()
            self.tx_timer.start()
        else:
            self.tx_timer.stop()
            self.btn_connect.setText("Connect")
            self.btn_connect.setStyleSheet("")

    def on_tab_changed(self, index):
        self.active_mode = index
        if index == 0:
            self.apply_open_loop()
        else:
            self.apply_closed_loop()

    def emergency_stop(self):
        self.spin_pwm.setValue(0)
        self.spin_target_rpm.setValue(0.0)
        self.cmd_target = 0.0
        # Langsung bypass kirim paket stop instan
        self.worker.send_command(0, 0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, self.cmd_period)
        self.status_bar.showMessage("EMERGENCY STOP DIKIRIM!", 2500)

    def apply_open_loop(self):
        self.cmd_target = float(self.spin_pwm.value())
        self.cmd_period = float(self.spin_open_period.value())
        self.status_bar.showMessage(f"Open Loop diatur: PWM = {self.cmd_target}", 2000)

    def apply_closed_loop(self):
        self.cmd_target = float(self.spin_target_rpm.value())
        self.cmd_period = float(self.spin_closed_period.value())
        self.cmd_m = float(self.spin_ff_m.value())
        self.cmd_c = float(self.spin_ff_c.value())
        self.cmd_kp = float(self.spin_kp.value())
        self.cmd_ki = float(self.spin_ki.value())
        self.cmd_kd = float(self.spin_kd.value())
        self.status_bar.showMessage(f"Closed Loop diatur: Target = {self.cmd_target} RPM", 2000)

    def send_10hz_command(self):
        """Transmisi serial periodik tepat 10 Hz (100 ms)."""
        if not self.worker.running:
            return

        if self.active_mode == 0:
            self.worker.send_command(
                tick=0,
                mode=0,
                target=self.cmd_target,
                m=1.0, c=0.0, kp=0.0, ki=0.0, kd=0.0,
                controll_period_ms=self.cmd_period
            )
        else:
            self.worker.send_command(
                tick=0,
                mode=1,
                target=self.cmd_target,
                m=self.cmd_m,
                c=self.cmd_c,
                kp=self.cmd_kp,
                ki=self.cmd_ki,
                kd=self.cmd_kd,
                controll_period_ms=self.cmd_period
            )

    def on_packet_received(self, data):
        # Update Numerik Label (Teks Box)
        self.lbl_v_act.setText(f"{data['bus_voltage_V']:.2f} V")
        self.lbl_i_act.setText(f"{data['current_mA']:.0f} mA")
        self.lbl_rpm_act.setText(f"{data['current_rpm']:.1f} RPM")
        self.lbl_rpm_tgt.setText(f"{data['target_rpm_pwm']:.1f}")

        # Update History Buffer RPM
        t = data['tick'] / 1000.0
        self.time_history.append(t)
        self.rpm_act_history.append(data['current_rpm'])
        self.rpm_tgt_history.append(data['target_rpm_pwm'])

        # Update Grafik Response RPM
        time_list = list(self.time_history)
        self.curve_rpm_act.setData(time_list, list(self.rpm_act_history))
        self.curve_rpm_tgt.setData(time_list, list(self.rpm_tgt_history))

    def setup_theme(self):
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #1e1e2e;
                color: #cdd6f4;
                font-family: 'Segoe UI', Arial, sans-serif;
                font-size: 12px;
            }
            QGroupBox {
                border: 1px solid #45475a;
                border-radius: 6px;
                margin-top: 10px;
                font-weight: bold;
                color: #89b4fa;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 5px;
            }
            QTabWidget::pane {
                border: 1px solid #45475a;
                border-radius: 4px;
                top: -1px;
            }
            QTabBar::tab {
                background: #313244;
                color: #a6adc8;
                padding: 8px 16px;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }
            QTabBar::tab:selected {
                background: #45475a;
                color: #cdd6f4;
                font-weight: bold;
            }
            QPushButton {
                background-color: #45475a;
                border: 1px solid #585b70;
                padding: 6px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #585b70;
            }
            QDoubleSpinBox, QSpinBox, QComboBox {
                background-color: #313244;
                border: 1px solid #45475a;
                border-radius: 4px;
                padding: 4px;
                color: #cdd6f4;
            }
            QStatusBar {
                background-color: #181825;
                color: #a6adc8;
            }
        """)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MotorControlGUI()
    window.show()
    sys.exit(app.exec_())