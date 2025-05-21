import socket
import random
import time

#랜덤하게 CPM값, 위경도 보내는 코드

def generate_random_data():
    cpm = random.uniform(40, 80)
    lat = random.uniform(36.424571, 36.425644)  # 36.424846 ± 0.01
    lon = random.uniform(127.366938, 127.372926)  # 127.370299 ± 0.01
    return cpm, lat, lon

def main():
    # 서버 정보
    # host = '223.171.53.193'  # 서버 IP 주소
    host = '192.168.50.100'
    port = 6000  # 서버 포트 번호

    # 소켓 생성
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    try:
        # 서버에 연결
        client_socket.connect((host, port))
        print(f"서버에 연결되었습니다: {host}:{port}")

        while True:
            # 랜덤 데이터 생성
            cpm, lat, lon = generate_random_data()

            # 데이터 포맷팅
            data = f"cpm{cpm:.2f}lat{lat:.6f}lon{lon:.6f}\n"

            # 데이터 전송
            client_socket.send(data.encode())
            print(f"전송된 데이터: {data.strip()}")

            # 5초 대기
            time.sleep(1)

    except KeyboardInterrupt:
        print("프로그램을 종료합니다.")
    except Exception as e:
        print(f"에러 발생: {e}")
    finally:
        # 소켓 닫기
        client_socket.close()
        print("소켓이 닫혔습니다.")

if __name__ == "__main__":
    main()
