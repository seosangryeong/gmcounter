### 🟩 Gmcounter ROS Package
#### 환경
rasberrypi4 model b , ubuntu20.04, ros1 
,디지털핀 -> GPIO17번
#### 설치 순서
1. cakin_ws/src에 git clone
2. cakin_make
3. source devel/setup.bash
4. roscore 
5. sudo bash -lc "\
  source /opt/ros/noetic/setup.bash; \
  source /home/raspi/catkin_ws/devel/setup.bash; \
  rosrun gmcounter gmcounter.py\
"  
alias 설정을 해서 실행하면 편함
#### 참고
gpio 권한설정이 필요하기 때문에 sudo 명령어로 실행해야 함.  <br><br><br><br>
####  


### 🟩 Gmcounter + Isaac sim (TCP 통신)


**위경도 m 변환**

**ecef 변환**

| 좌표계 | 설명 | 단위 |
| --- | --- | --- |
| Geodetic | 위도, 경도 ,고도 | 도 |
| ECEF | 지구 중심 기준의 3D 직교좌표(earth-centered, earth-fixed) | m |
| ENU(로컬) | 특정 기준점에서 동쪽-북쪽-상향으로 정렬된 로컬 좌표 | m |

전체 변환 과정은 위도,경도, 고도 → ecef로 변환 → enu로 변환이다.(고도(m)는 그냥 사용)

1. 위/경도 → ecef
    1. 위도/경도를 라디안으로 변환
    2. 타원체의 중심에서부터 위도 방향 곡률 반경 N 계산
        
        ```python
        N = a / sqrt(1 - e² * sin²(lat))
        
        # a = 지구 적도 반지름 = 6,378,137.0 m
        # e^2 = 이심률 제곱 = 0.006694
        # lat = 위도
        ```
        
    3. X, Y, Z를 공식으로 계산
        
        ```python
        x = (N + alt) * cos(lat) * cos(lon)
        y = (N + alt) * cos(lat) * sin(lon)
        z = (N*(1 - e²) + alt) * sin(lat)
        
        #x,y,z는 지구 중심 기준 좌표
        ```
        
2. ECEF → ENU
    1. 기준점의 ECEF 좌표 계산
    2. 기준점에서 이동한 벡터 (dx, dy, dz) 계산
    3. 위/경도 기반 회전행렬로 동(E), 북(N), 상(U) 방향 로컬 좌표 변환
