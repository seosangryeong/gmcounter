import socket
import math
import random
from isaacsim import SimulationApp
import numpy as np
import threading
import queue

# launch omniverse app
CONFIG = {
    "width": 1280, 
    "height": 720, 
    "sync_loads": True, 
    "headless": False, 
    "renderer": "RayTracedLighting"
}

app = SimulationApp(launch_config=CONFIG)

import carb
import omni
from omni.isaac.core.utils.stage import is_stage_loading
from omni.isaac.core.objects.sphere import VisualSphere
from omni.isaac.core.materials import PreviewSurface
from pxr import Sdf

# 구 생성 요청을 저장할 큐
sphere_queue = queue.Queue()

def measure(lat1, lon1, lat2, lon2):
    """두 지리적 좌표 간의 거리 계산 (미터 단위)"""
    R = 6378.137  # 지구의 반지름 (km)
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    
    a = math.sin(dLat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dLon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    d = R * c  # 거리 (km 단위)
    return d * 1000  # 거리 (미터 단위)

def geodetic_to_local_meters(lat, lon, origin_lat, origin_lon):
    """위도/경도를 원점으로부터의 미터 단위 거리로 변환"""
    # 동서 방향 거리
    x = measure(origin_lat, origin_lon, origin_lat, lon)
    if lon < origin_lon:
        x = -x
    
    # 남북 방향 거리
    y = measure(origin_lat, origin_lon, lat, origin_lon)
    if lat < origin_lat:
        y = -y
    
    return x, y

def cpm_to_color(cpm):
    """cpm 값에 따라 색상을 결정 (파란색 -> 빨간색)"""
    ratio = (cpm - 40) / (80 - 40)
    ratio = max(0, min(1, ratio))  # 0~1 사이로 제한
    r = int(255 * ratio)
    g = 0
    b = int(255 * (1 - ratio))
    return r, g, b

def create_sphere(x, y, z, cpm, radius=1.0):
    """구를 생성하고 위치, 색상, 크기를 설정"""
    try:
        r, g, b = cpm_to_color(cpm)
        color = np.array([r/255.0, g/255.0, b/255.0])
        
        material = PreviewSurface(
            prim_path=f"/World/Materials/Sphere_{random.randint(0, 1e6)}", 
            color=color
        )
        shader = material.shaders_list[0]
        shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.5)
        
        prim = VisualSphere(
            prim_path=f"/World/Sphere_{random.randint(0, 1e6)}", 
            position=(x, y, z), 
            radius=1.0,
            visual_material=material
        )
        return prim
    except Exception as e:
        print(f"구 생성 중 에러 발생: {e}")
        return None

def start_socket_server(origin_lat, origin_lon):
    """소켓 서버 시작"""
    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind(('0.0.0.0', 5050))
        server_socket.listen(1)
        print("Socket server listening on port 5050...")
        conn, addr = server_socket.accept()
        print(f"Connected by {addr}")

        while True:
            try:
                data = conn.recv(1024).decode()
                if not data:
                    break

                cpm = float(data.split("cpm")[1].split("lat")[0])
                lat = float(data.split("lat")[1].split("lon")[0])
                lon = float(data.split("lon")[1].strip())
                alt = random.uniform(10, 30)
                
                x, y = geodetic_to_local_meters(lat, lon, origin_lat, origin_lon)
                z = alt

                # 구 생성 요청을 큐에 추가
                sphere_queue.put((x, y, z, cpm))
                carb.log_info(f"구 생성 요청 추가: ({x}, {y}, {z}) with CPM {cpm}")

            except Exception as e:
                print(f"데이터 처리 중 에러 발생: {e}")

    except Exception as e:
        print(f"소켓 서버 에러: {e}")
    finally:
        conn.close()
        server_socket.close()

def main():
    # 기준점 설정 (연구실 위치)
    origin_lat = 36.424846
    origin_lon = 127.370299

    # USD 파일 경로 설정
    usd_path = "/home/nuc/Downloads/lab.usd"

    # USD 파일 열기
    omni.usd.get_context().open_stage(usd_path)

    # Wait two frames so that stage starts loading
    app.update()
    app.update()

    print("Loading stage...")
    while is_stage_loading():
        app.update()
    print("Loading Complete")

    # 소켓 서버를 별도 스레드로 실행
    socket_thread = threading.Thread(target=start_socket_server, args=(origin_lat, origin_lon))
    socket_thread.daemon = True
    socket_thread.start()

    # 시뮬레이션 루프
    while app.is_running():
        # 큐에 있는 구 생성 요청 처리
        try:
            while not sphere_queue.empty():
                x, y, z, cpm = sphere_queue.get_nowait()
                sphere = create_sphere(x, y, z, cpm)
                if sphere:
                    carb.log_info(f"구 생성 완료: ({x}, {y}, {z}) with CPM {cpm}")
        except queue.Empty:
            pass
        
        app.update()

if __name__ == "__main__":
    main()
    app.close()
