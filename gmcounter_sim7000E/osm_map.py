import osmium
import folium
from shapely.geometry import Point

class OSMHandler(osmium.SimpleHandler):
    def __init__(self):
        super(OSMHandler, self).__init__()
        self.nodes = []

    def node(self, n):
        if len(self.nodes) < 1000:  # 처리할 노드 수 제한
            self.nodes.append(Point(n.location.lon, n.location.lat))

def visualize_osm(osm_file):
    # OSM 파일에서 데이터 읽기
    handler = OSMHandler()
    handler.apply_file(osm_file)

    if not handler.nodes:
        print("노드를 찾을 수 없습니다.")
        return

    # 중심점 계산
    center_lat = sum(point.y for point in handler.nodes) / len(handler.nodes)
    center_lon = sum(point.x for point in handler.nodes) / len(handler.nodes)

    # 지도 생성
    m = folium.Map(location=[center_lat, center_lon], zoom_start=12)

    # 사용자로부터 위도와 경도 입력 받기
    try:
        user_lat = float(input("위도를 입력하세요: "))
        user_lon = float(input("경도를 입력하세요: "))
        
        # 사용자 입력 위치에 마커 추가
        folium.Marker(
            location=[user_lat, user_lon],
            popup="입력한 위치",
            icon=folium.Icon(color='red', icon='info-sign')
        ).add_to(m)
        
        print(f"입력한 위치 (위도: {user_lat}, 경도: {user_lon})에 마커를 추가했습니다.")
    except ValueError:
        print("올바른 숫자 형식으로 위도와 경도를 입력해주세요.")

    # HTML 파일로 저장
    m.save("osm_visualization.html")
    print("지도가 'osm_visualization.html' 파일로 저장되었습니다.")

if __name__ == '__main__':
    osm_file_path = "/home/nuc/osm/map.osm"
    visualize_osm(osm_file_path)