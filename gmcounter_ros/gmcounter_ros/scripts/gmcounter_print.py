import RPi.GPIO as GPIO
import time

"""
rasberry pi4 model b , gmcounter 사용
인터럽트, cpm 출력하는 코드

GPIO를 사용하기 위해서는 루트 권한으로 실행해야함.
터미널에서 sudo python3 gmcounter_print.py
"""

WINDOWING = 60
MOVING_WINDOWING = 1.0  # sec
NUMBER_OF_COUNTER = 1



GM_PIN = 17  #GPIO 17

counts = [0 for _ in range(NUMBER_OF_COUNTER)]
readings = [[0 for _ in range(WINDOWING)] for _ in range(NUMBER_OF_COUNTER)]
total = [0 for _ in range(NUMBER_OF_COUNTER)]
readIndex = 0
prevTime = time.time()

impulseDetected = False
dataToSendFlag = False
lastImpulseCount = 0

# 인터럽트 콜백 함수
def TUBE_IMPULSE1(channel):
    global impulseDetected, lastImpulseCount
    counts[0] += 1
    impulseDetected = True
    lastImpulseCount = counts[0]
    print("interrupt")

# GPIO 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(GM_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(GM_PIN, GPIO.FALLING, callback=TUBE_IMPULSE1, bouncetime=10)

try:

    while True:
        # 1초마다 CPM 계산
        curTime = time.time()
        if curTime - prevTime >= MOVING_WINDOWING:
            prevTime = curTime

            for i in range(NUMBER_OF_COUNTER):
                total[i] -= readings[i][readIndex]
                readings[i][readIndex] = counts[i]
                counts[i] = 0
                total[i] += readings[i][readIndex]

            readIndex = (readIndex + 1) % WINDOWING

            print(f"CPM: {total[0]}")

        time.sleep(0.005)

except KeyboardInterrupt:
    print()

finally:
    GPIO.cleanup()
