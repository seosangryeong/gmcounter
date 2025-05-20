### Gmcounter
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
#### 참고
gpio 권한설정이 안되어 sudo 명령어로 실행해야 함.
