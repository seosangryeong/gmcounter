import sys
import os
import webbrowser
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, QWidget, QPushButton, QTextEdit, QLineEdit, QLabel
from PyQt5.QtCore import QThread, pyqtSignal
import folium
from folium.plugins import HeatMap
import socket
import colorsys
import datetime

class DataReceiver(QThread):
    data_received = pyqtSignal(float, float, float)

    def __init__(self, ip, port):
        super().__init__()
        self.ip = ip
        self.port = port

    def run(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.ip, self.port))
        server_socket.listen(1)

        print(f"데이터 수신 대기 중... (IP: {self.ip}, Port: {self.port})")
        conn, addr = server_socket.accept()
        print(f"연결됨: {addr}")

        while True:
            data = conn.recv(1024).decode().strip()
            if not data:
                break

            try:
                cpm = float(data[data.index('cpm')+3:data.index('lat')])
                lat = float(data[data.index('lat')+3:data.index('lon')])
                lon = float(data[data.index('lon')+3:])

                self.data_received.emit(cpm, lat, lon)
            except ValueError as e:
                print(f"데이터 파싱 오류: {e}")
            except Exception as e:
                print(f"예외 발생: {e}")

        conn.close()

class RadiationMap:
    def __init__(self):
        self.map = None
        self.data_points = []
        self.min_cpm = float('inf')
        self.max_cpm = float('-inf')

    def initialize_map(self, lat, lon):
        self.map = folium.Map(location=[lat, lon], zoom_start=12)

    def add_point(self, lat, lon, cpm):
        if lat == 0.0 and lon == 0.0:
            # print(f"오류: 위도와 경도가 0입니다. CPM: {cpm}")
            return
        
        if self.map is None:
            self.initialize_map(lat, lon)
        
        self.min_cpm = min(self.min_cpm, cpm)
        self.max_cpm = max(self.max_cpm, cpm)
        
        if len(self.data_points) > 0:
            color = self.get_color(cpm)
            folium.CircleMarker(
                location=[lat, lon],
                radius=10,
                popup=f"CPM: {cpm}, Lat: {lat}, Lon: {lon}",
                color=color,
                fill=True,
                fillColor=color,
                fillOpacity=0.7
            ).add_to(self.map)
    
        self.data_points.append([lat, lon, cpm])

    # 낮은 cpm : 파랑 , 높은 pm : 빨강   ppt에서 색상 보면 hue확인가능
    def get_color(self, cpm):
        # CPM 값을 0-1 범위로 정규화 (100을 최대값으로 설정)
        normalized = min(cpm / 100, 1)
        
        # HSV 색상 모델을 사용. 파랑 : 250 빨강 : 360
        hue = 120 * normalized + 240
        # hue = 120 * normalized
        
        # HSV to RGB 변환
        rgb = colorsys.hsv_to_rgb(hue / 360, 1, 1)
        
        # RGB 값을 16진수 문자열로 변환
        return '#{:02x}{:02x}{:02x}'.format(int(rgb[0] * 255), int(rgb[1] * 255), int(rgb[2] * 255))

    def save_map(self):
        if self.map and self.data_points:
            bounds = [
                [min(point[0] for point in self.data_points), min(point[1] for point in self.data_points)],
                [max(point[0] for point in self.data_points), max(point[1] for point in self.data_points)]
            ]
            self.map.fit_bounds(bounds)
            #1초마다 맵 새로고침
        #     self.map.get_root().header.add_child(folium.Element("""
        #     <meta http-equiv="refresh" content="1">
        # """))
            self.filename = 'radiation_map.html'  # 여기로 이동
            self.map.save(self.filename)
            print(f"지도가 {self.filename}으로 업데이트되었습니다.")
        else:
            print("저장할 지도가 아직 생성되지 않았습니다.")
        return self.filename



class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.radiation_map = RadiationMap()
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Radiation Map')
        self.setGeometry(100, 100, 400, 350)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        layout = QVBoxLayout()
        central_widget.setLayout(layout)

        # IP and Port input
        ip_port_layout = QHBoxLayout()
        
        self.ip_label = QLabel('IP:')
        self.ip_input = QLineEdit('192.168.50.100')  # 기본 IP
        ip_port_layout.addWidget(self.ip_label)
        ip_port_layout.addWidget(self.ip_input)

        self.port_label = QLabel('Port:')
        self.port_input = QLineEdit('6000')  # 기본 포트
        ip_port_layout.addWidget(self.port_label)
        ip_port_layout.addWidget(self.port_input)

        layout.addLayout(ip_port_layout)

        # Controls
        controls_layout = QHBoxLayout()
        
        self.connect_button = QPushButton('Connect')
        self.connect_button.clicked.connect(self.start_receiver)
        controls_layout.addWidget(self.connect_button)

        self.show_map_button = QPushButton('Show Map')
        self.show_map_button.clicked.connect(self.show_map)
        controls_layout.addWidget(self.show_map_button)

        layout.addLayout(controls_layout)

        # Data display
        data_layout = QHBoxLayout()
        
        self.cpm_label = QLabel('CPM:')
        self.cpm_value = QLineEdit()
        self.cpm_value.setReadOnly(True)
        data_layout.addWidget(self.cpm_label)
        data_layout.addWidget(self.cpm_value)

        self.lat_label = QLabel('Latitude:')
        self.lat_value = QLineEdit()
        self.lat_value.setReadOnly(True)
        data_layout.addWidget(self.lat_label)
        data_layout.addWidget(self.lat_value)

        self.lon_label = QLabel('Longitude:')
        self.lon_value = QLineEdit()
        self.lon_value.setReadOnly(True)
        data_layout.addWidget(self.lon_label)
        data_layout.addWidget(self.lon_value)

        layout.addLayout(data_layout)

        # Log display
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)

    def start_receiver(self):
        ip = self.ip_input.text()
        port = int(self.port_input.text())
        self.receiver = DataReceiver(ip, port)
        self.receiver.data_received.connect(self.update_data)
        self.receiver.start()
        self.log_text.append(f"연결 시작... (IP: {ip}, Port: {port})")

    def update_data(self, cpm, lat, lon):
        self.cpm_value.setText(str(cpm))
        self.lat_value.setText(str(lat))
        self.lon_value.setText(str(lon))
        
        if lat != 0.0 or lon != 0.0:
            self.radiation_map.add_point(lat, lon, cpm)
            self.radiation_map.save_map()
            # self.log_text.append(f"CPM={cpm}, Lat={lat}, Lon={lon}")

    def show_map(self):
        filename = self.radiation_map.save_map()
        if filename:
            webbrowser.open('file://' + os.path.realpath(filename))

    def closeEvent(self, event):
        if hasattr(self, 'receiver'):
            self.receiver.quit()
            self.receiver.wait()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())