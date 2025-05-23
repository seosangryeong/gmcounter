#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32
import RPi.GPIO as GPIO
import time

"""

ros로 cpm 값 퍼블리시 하는 코드.
gpio 권한 문제가 있어 아래 명령어로 실행해야 함.

sudo bash -lc "\
  source /opt/ros/noetic/setup.bash; \
  source /home/raspi/catkin_ws/devel/setup.bash; \
  rosrun gmcounter gmcounter.py\
"

"""

WINDOWING       = 60      
MOVING_WINDOW   = 1.0     # sec
GM_PIN          = 17      

counts   = 0
readings = [0] * WINDOWING
total    = 0
readIndex= 0
prevTime = time.time()

# 인터럽트 콜백
def TUBE_IMPULSE1(channel):
    global counts
    counts += 1

def main():
    global counts, total, readIndex, prevTime

    rospy.init_node('gmcounter')
    pub = rospy.Publisher('/gmcounter/cpm', Int32, queue_size=10)

    # GPIO 설정
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
    GPIO.add_event_detect(GM_PIN, GPIO.FALLING,
                          callback=TUBE_IMPULSE1,
                          bouncetime=10)

    rate = rospy.Rate(100)  
    while not rospy.is_shutdown():
        now = time.time()
        if now - prevTime >= MOVING_WINDOW:
            prevTime = now

            c = counts
            counts = 0

            total -= readings[readIndex]
            readings[readIndex] = c
            total += c
            readIndex = (readIndex + 1) % WINDOWING

            pub.publish(total)

        rate.sleep()

    GPIO.cleanup()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
