import sys
import struct
import zlib
import time
import serial
import serial.tools.list_ports
from collections import deque
import numpy as np

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QGroupBox, QLabel, QPushButton, QComboBox,
    QDoubleSpinBox, QFrame, QCheckBox
)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt
import pyqtgraph as pg

# Penanda Paket Protokol Biner (Arduino)[cite: 10]
MARKER_SOF = bytes([0xAA, 0xBB, 0xCC])
MARKER_EOF = bytes([0xCC, 0xBB, 0xAA])

# Format Paket Tx Arduino: uint8_t sof[3], 4x float, uint32_t crc, uint8_t eof[3] -> 26 Byte[cite: 10]
TX_PACKET_FORMAT = "<3sffffI3s"
TX_PACKET_SIZE = struct.calcsize(TX_PACKET_FORMAT)

# Format Payload Rx Arduino: uint8_t sof[3], 7x float[cite: 10]
RX_PAYLOAD_FORMAT = "<3sfffffff"


def calculate_crc32(payload: bytes) -> int:
    return zlib.crc32(payload) & 0xFFFFFFFF


class SerialWorker(QThread):
    packet_received = pyqtSignal(float, float, float, float)
    connection_status = pyqtSignal(bool, str)

    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.is_running = False
        self.buffer = bytearray()

        # Parameter aktif yang dikirim terus-menerus (streaming)[cite: 10]
        self.active_params = {
            "target_rpm": 100.0,
            "ff_m": 1.0,
            "ff_c": 0.0,
            "kp": 3.0,
            "ki": 0.1,
            "kd": 0.0,
            "sample_time": 0.01
        }

    def connect_serial(self, port_name, baudrate=115200):
        try:
            self.serial_port = serial.Serial(port_name, baudrate, timeout=0.05)
            self.is_running = True
            self.start()
            self.connection_status.emit(True, f"Terhubung: {port_name}")
        except Exception as e:
            self.connection_status.emit(False, str(e))

    def disconnect_serial(self):
        self.is_running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.wait()
        self.connection_status.emit(False, "Terputus")

    def update_active_parameters(self, target_rpm, ff_m, ff_c, kp, ki, kd, sample_time):
        self.active_params["target_rpm"] = float(target_rpm)
        self.active_params["ff_m"] = float(ff_m)
        self.active_params["ff_c"] = float(ff_c)
        self.active_params["kp"] = float(kp)
        self.active_params["ki"] = float(ki)
        self.active_params["kd"] = float(kd)
        self.active_params["sample_time"] = float(sample_time)

    def send_packet(self):
        if not (self.serial_port and self.serial_port.is_open):
            return

        payload = struct.pack(
            RX_PAYLOAD_FORMAT,
            MARKER_SOF,
            self.active_params["target_rpm"],
            self.active_params["ff_m"],
            self.active_params["ff_c"],
            self.active_params["kp"],
            self.active_params["ki"],
            self.active_params["kd"],
            self.active_params["sample_time"]
        )

        crc = calculate_crc32(payload)
        packet = payload + struct.pack("<I3s", crc, MARKER_EOF)

        try:
            self.serial_port.write(packet)
        except Exception as e:
            print(f"UART Tx Error: {e}")

    def run(self):
        while self.is_running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    bytes_to_read = self.serial_port.in_waiting
                    if bytes_to_read > 0:
                        raw = self.serial_port.read(bytes_to_read)
                        self.buffer.extend(raw)

                        while len(self.buffer) >= TX_PACKET_SIZE:
                            sof_idx = self.buffer.find(MARKER_SOF)
                            if sof_idx == -1:
                                self.buffer.clear()
                                break

                            if sof_idx > 0:
                                del self.buffer[:sof_idx]

                            if len(self.buffer) < TX_PACKET_SIZE:
                                break

                            candidate = self.buffer[:TX_PACKET_SIZE]
                            sof, rpm, pwm, volt, curr, crc, eof = struct.unpack(TX_PACKET_FORMAT, candidate)

                            if eof == MARKER_EOF:
                                payload_to_crc = candidate[:struct.calcsize("<3sffff")]
                                if calculate_crc32(payload_to_crc) == crc:
                                    self.packet_received.emit(rpm, pwm, volt, curr)
                                    del self.buffer[:TX_PACKET_SIZE]
                                    continue

                            del self.buffer[0]
                except Exception as e:
                    print(f"UART Rx Error: {e}")
            time.sleep(0.001)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Monitoring & Kendali Motor DC - Dashboard")
        self.resize(1250, 800)

        self.worker = SerialWorker()
        self.worker.packet_received.connect(self.update_telemetry)
        self.worker.connection_status.connect(self.handle_connection_status)

        # Buffer data untuk kurva
        self.start_time = None
        self.time_data = deque()
        self.rpm_data = deque()
        self.pwm_data = deque()
        self.target_data = deque()
        self.current_time = 0.0

        self.apply_dark_theme()
        self.init_ui()

        # Timer streaming parameter Tx ke mikrokontroler (50 ms / 20 Hz)[cite: 10]
        self.stream_timer = QTimer(self)
        self.stream_timer.setInterval(50)
        self.stream_timer.timeout.connect(self.worker.send_packet)

        # Timer rendering visual grafik (30 FPS = 33 ms)[cite: 10]
        self.render_timer = QTimer(self)
        self.render_timer.setInterval(33)
        self.render_timer.timeout.connect(self.render_plot)

    def apply_dark_theme(self):
        self.setStyleSheet("""
            QWidget {
                background-color: #18191f;
                color: #e0e0e0;
                font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
                font-size: 13px;
            }
            QGroupBox {
                border: 1px solid #2d303e;
                border-radius: 8px;
                margin-top: 12px;
                padding-top: 12px;
                font-weight: bold;
                color: #00adb5;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                padding: 0 6px;
                left: 10px;
            }
            QPushButton {
                background-color: #2b2e4a;
                border: 1px solid #3f446b;
                border-radius: 6px;
                padding: 6px 12px;
                color: #ffffff;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #00adb5;
                color: #18191f;
                border: 1px solid #00adb5;
            }
            QPushButton:pressed {
                background-color: #00828a;
            }
            QComboBox, QDoubleSpinBox {
                background-color: #222430;
                border: 1px solid #373a4d;
                border-radius: 5px;
                padding: 4px 8px;
                color: #ffffff;
                selection-background-color: #00adb5;
            }
            QComboBox::drop-down {
                border: none;
            }
            QDoubleSpinBox:focus {
                border: 1px solid #00adb5;
            }
            QCheckBox {
                spacing: 8px;
                font-weight: bold;
            }
            QCheckBox::indicator {
                width: 16px;
                height: 16px;
                border-radius: 3px;
                border: 1px solid #3f446b;
                background-color: #222430;
            }
            QCheckBox::indicator:checked {
                background-color: #00adb5;
                border: 1px solid #00adb5;
            }
        """)

    def init_ui(self):
        main_layout = QHBoxLayout()
        left_panel = QVBoxLayout()
        right_panel = QVBoxLayout()

        # 1. Panel Port Serial[cite: 10]
        conn_group = QGroupBox("Koneksi Port Serial")
        conn_layout = QGridLayout()
        self.port_combo = QComboBox()
        self.refresh_ports()
        self.btn_refresh = QPushButton("Segarkan")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect = QPushButton("Hubungkan")
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.lbl_status = QLabel("Status: Terputus")
        self.lbl_status.setStyleSheet("color: #ff5252; font-weight: bold;")

        conn_layout.addWidget(self.port_combo, 0, 0)
        conn_layout.addWidget(self.btn_refresh, 0, 1)
        conn_layout.addWidget(self.btn_connect, 1, 0, 1, 2)
        conn_layout.addWidget(self.lbl_status, 2, 0, 1, 2)
        conn_group.setLayout(conn_layout)
        left_panel.addWidget(conn_group)

        # 2. Panel Parameter Kendali[cite: 10]
        param_group = QGroupBox("Pengaturan Parameter (Tekan Enter/Kirim)")
        param_layout = QGridLayout()

        self.spin_target_rpm = self.create_spinbox(-600.0, 600.0, 0.0, 1)
        self.spin_ff_m = self.create_spinbox(-100.0, 100.0, 0.0, 4)
        self.spin_ff_c = self.create_spinbox(-255.0, 255.0, 0.0, 2)
        self.spin_kp = self.create_spinbox(0.0, 100.0, 0.0, 3)
        self.spin_ki = self.create_spinbox(0.0, 100.0, 0.0, 3)
        self.spin_kd = self.create_spinbox(0.0, 100.0, 0.0, 4)
        self.spin_ts = self.create_spinbox(0.001, 1.0, 0.01, 3)

        fields = [
            ("Target RPM", self.spin_target_rpm),
            ("Feedforward m (Slope)", self.spin_ff_m),
            ("Feedforward c (Offset)", self.spin_ff_c),
            ("Kp", self.spin_kp),
            ("Ki", self.spin_ki),
            ("Kd", self.spin_kd),
            ("Waktu Sampling (s)", self.spin_ts)
        ]

        for i, (label, widget) in enumerate(fields):
            param_layout.addWidget(QLabel(label), i, 0)
            param_layout.addWidget(widget, i, 1)

        self.btn_apply = QPushButton("Kirim Parameter")
        self.btn_apply.setStyleSheet("background-color: #00adb5; color: #18191f;")
        self.btn_apply.clicked.connect(self.commit_parameters)
        param_layout.addWidget(self.btn_apply, len(fields), 0, 1, 2)

        param_group.setLayout(param_layout)
        left_panel.addWidget(param_group)

        # 3. Panel Numerik Besar[cite: 10]
        metric_group = QGroupBox("Pemantauan Daya & Status")
        metric_layout = QGridLayout()

        lbl_v_title = QLabel("TEGANGAN BUS")
        lbl_v_title.setStyleSheet("font-size: 11px; color: #a0a5b5; font-weight: bold;")
        self.val_volt = QLabel("0.00 V")
        self.val_volt.setStyleSheet("font-size: 26px; font-weight: bold; color: #f9d342;")

        lbl_i_title = QLabel("KONSUMSI ARUS")
        lbl_i_title.setStyleSheet("font-size: 11px; color: #a0a5b5; font-weight: bold;")
        self.val_curr = QLabel("0.00 mA")
        self.val_curr.setStyleSheet("font-size: 26px; font-weight: bold; color: #ff3366;")

        self.val_rpm = QLabel("0.00 RPM")
        self.val_rpm.setStyleSheet("font-size: 16px; font-weight: bold; color: #00ff88;")
        self.val_pwm = QLabel("0.00")
        self.val_pwm.setStyleSheet("font-size: 16px; font-weight: bold; color: #ff9900;")

        metric_layout.addWidget(lbl_v_title, 0, 0)
        metric_layout.addWidget(self.val_volt, 1, 0)
        metric_layout.addWidget(lbl_i_title, 2, 0)
        metric_layout.addWidget(self.val_curr, 3, 0)

        div = QFrame()
        div.setFrameShape(QFrame.HLine)
        div.setStyleSheet("color: #2d303e;")
        metric_layout.addWidget(div, 4, 0, 1, 2)

        metric_layout.addWidget(QLabel("Kecepatan Aktual:"), 5, 0)
        metric_layout.addWidget(self.val_rpm, 5, 1)
        metric_layout.addWidget(QLabel("Sinyal PWM:"), 6, 0)
        metric_layout.addWidget(self.val_pwm, 6, 1)

        metric_group.setLayout(metric_layout)
        left_panel.addWidget(metric_group)
        left_panel.addStretch()

        # ================= KANAN: GRAFIK & CHECKBOX VISIBILITAS =================
        # Baris Checkbox untuk memilih elemen kurva yang ditampilkan
        view_ctrl_layout = QHBoxLayout()
        view_ctrl_label = QLabel("Tampilkan Kurva:")
        view_ctrl_label.setStyleSheet("font-weight: bold; color: #00adb5;")

        self.chk_rpm = QCheckBox("RPM Aktual")
        self.chk_rpm.setChecked(True)
        self.chk_rpm.setStyleSheet("color: #00ff88;")
        self.chk_rpm.stateChanged.connect(self.update_curve_visibility)

        self.chk_target = QCheckBox("Setpoint RPM")
        self.chk_target.setChecked(True)
        self.chk_target.setStyleSheet("color: #ff3366;")
        self.chk_target.stateChanged.connect(self.update_curve_visibility)

        self.chk_pwm = QCheckBox("Sinyal PWM")
        self.chk_pwm.setChecked(True)
        self.chk_pwm.setStyleSheet("color: #ff9900;")
        self.chk_pwm.stateChanged.connect(self.update_curve_visibility)

        view_ctrl_layout.addWidget(view_ctrl_label)
        view_ctrl_layout.addWidget(self.chk_rpm)
        view_ctrl_layout.addWidget(self.chk_target)
        view_ctrl_layout.addWidget(self.chk_pwm)
        view_ctrl_layout.addStretch()

        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.PlotWidget(title="Respon Motor: Kecepatan (RPM) & Sinyal Kontrol (PWM)")
        self.plot_widget.setBackground('#1e2029')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.25)
        
        # Rentang RPM: -600.0 hingga 600.0, Waktu: 30 detik[cite: 10]
        self.plot_widget.setYRange(-600.0, 600.0, padding=0.0)
        self.plot_widget.setXRange(0.0, 30.0, padding=0.0)
        self.plot_widget.setMouseEnabled(x=False, y=False)
        self.plot_widget.setLabel('bottom', "Waktu", units='s')
        self.plot_widget.setLabel('left', "Nilai", units='RPM / PWM')
        self.plot_widget.addLegend(offset=(10, 10))

        # Kurva Setpoint Target (Garis putus-putus merah/magenta)
        pen_target = pg.mkPen(color='#ff3366', width=1.2, style=Qt.DashLine)
        self.curve_target = self.plot_widget.plot(pen=pen_target, name="Setpoint RPM")

        # Kurva RPM Aktual (Hijau Neon tipis)[cite: 10]
        self.curve_rpm = self.plot_widget.plot(
            pen=pg.mkPen(color='#00ff88', width=1.0), 
            name="RPM Aktual"
        )
        
        # Kurva Sinyal PWM (Oranye tipis)[cite: 10]
        self.curve_pwm = self.plot_widget.plot(
            pen=pg.mkPen(color='#ff9900', width=1.0), 
            name="Sinyal PWM"
        )

        right_panel.addLayout(view_ctrl_layout)
        right_panel.addWidget(self.plot_widget)

        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_panel, 3)

        central_widget = QWidget()
        central_widget.setLayout(main_layout)
        self.setCentralWidget(central_widget)

    def update_curve_visibility(self):
        self.curve_rpm.setVisible(self.chk_rpm.isChecked())
        self.curve_target.setVisible(self.chk_target.isChecked())
        self.curve_pwm.setVisible(self.chk_pwm.isChecked())

    def create_spinbox(self, min_val, max_val, default_val, decimals):
        sb = QDoubleSpinBox()
        sb.setRange(min_val, max_val)
        sb.setValue(default_val)
        sb.setDecimals(decimals)
        sb.setSingleStep(10 ** (-decimals))
        sb.editingFinished.connect(self.commit_parameters)
        return sb

    def commit_parameters(self):
        self.worker.update_active_parameters(
            target_rpm=self.spin_target_rpm.value(),
            ff_m=self.spin_ff_m.value(),
            ff_c=self.spin_ff_c.value(),
            kp=self.spin_kp.value(),
            ki=self.spin_ki.value(),
            kd=self.spin_kd.value(),
            sample_time=self.spin_ts.value()
        )

    def refresh_ports(self):
        self.port_combo.clear()
        for p in serial.tools.list_ports.comports():
            self.port_combo.addItem(p.device)

    def toggle_connection(self):
        if self.worker.is_running:
            self.stream_timer.stop()
            self.render_timer.stop()
            self.worker.disconnect_serial()
            self.btn_connect.setText("Hubungkan")
        else:
            port = self.port_combo.currentText()
            if port:
                self.commit_parameters()
                self.worker.connect_serial(port)
                self.btn_connect.setText("Putuskan")

    def handle_connection_status(self, is_connected, message):
        if is_connected:
            self.lbl_status.setText(f"Status: {message}")
            self.lbl_status.setStyleSheet("color: #00ff88; font-weight: bold;")
            self.start_time = time.time()
            self.time_data.clear()
            self.rpm_data.clear()
            self.pwm_data.clear()
            self.target_data.clear()
            self.stream_timer.start()
            self.render_timer.start()
        else:
            self.lbl_status.setText("Status: Terputus")
            self.lbl_status.setStyleSheet("color: #ff5252; font-weight: bold;")
            self.stream_timer.stop()
            self.render_timer.stop()

    def update_telemetry(self, rpm, pwm, volt, curr):
        self.current_time = time.time() - self.start_time

        self.val_rpm.setText(f"{rpm:.2f} RPM")
        self.val_pwm.setText(f"{pwm:.2f}")
        self.val_volt.setText(f"{volt:.2f} V")
        self.val_curr.setText(f"{curr:.2f} mA")

        self.time_data.append(self.current_time)
        self.rpm_data.append(rpm)
        self.pwm_data.append(pwm)
        self.target_data.append(self.worker.active_params["target_rpm"])

        # Buang data yang lebih tua dari 30 detik[cite: 10]
        cutoff = self.current_time - 30.0
        while self.time_data and self.time_data[0] < cutoff:
            self.time_data.popleft()
            self.rpm_data.popleft()
            self.pwm_data.popleft()
            self.target_data.popleft()

    def render_plot(self):
        if not self.time_data:
            return

        t_arr = np.fromiter(self.time_data, dtype=np.float32)

        if self.chk_rpm.isChecked():
            rpm_arr = np.fromiter(self.rpm_data, dtype=np.float32)
            self.curve_rpm.setData(t_arr, rpm_arr)

        if self.chk_target.isChecked():
            target_arr = np.fromiter(self.target_data, dtype=np.float32)
            self.curve_target.setData(t_arr, target_arr)

        if self.chk_pwm.isChecked():
            pwm_arr = np.fromiter(self.pwm_data, dtype=np.float32)
            self.curve_pwm.setData(t_arr, pwm_arr)

        if self.current_time <= 30.0:
            self.plot_widget.setXRange(0.0, 30.0, padding=0.0)
        else:
            self.plot_widget.setXRange(self.current_time - 30.0, self.current_time, padding=0.0)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())