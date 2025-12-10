from djitellopy import Tello
import KeyPressModule as kp   # 키 입력 모듈 (강의 자료 제공)
import numpy as np
from time import sleep
import cv2
import math

############### 파라미터 설정 ###############
fSpeed = 117 / 10    # 전진 속도 cm/s   ≈ 11.7cm/s
aSpeed = 360 / 10    # 회전 속도 deg/s  ≈ 36deg/s
interval = 0.25      # 제어 주기 (초)

dInterval = fSpeed * interval   # 한 프레임당 전진 거리 (cm)
aInterval = aSpeed * interval   # 한 프레임당 회전 각도 (deg)
##############################################

# 초기 위치 및 자세 변수
x, y = 500, 500      # 1000×1000 이미지 중앙에서 시작
a = 0                # 현재 방향 각도 (드론 헤딩, 0° = 오른쪽)
yaw = 0              # yaw 속도 누적용

kp.init()            # 키보드 입력 모듈 초기화
me = Tello()
me.connect()
print("배터리 잔량:", me.get_battery(), "%")

# 그려질 비행 경로 점들 (시작점 2개)
points = [(0, 0), (0, 0)]

def getKeyboardInput():
    lr, fb, ud, yv = 0, 0, 0, 0   # 좌우, 앞뒤, 상하, yaw
    global x, y, a, yaw

    # 1. 키 입력에 따른 속도 설정
    if kp.getKey("LEFT"):   lr = -15
    if kp.getKey("RIGHT"):  lr =  15
    if kp.getKey("UP"):     fb =  15
    if kp.getKey("DOWN"):   fb = -15
    if kp.getKey("w"):      ud =  15
    if kp.getKey("s"):      ud = -15
    if kp.getKey("a"):      yv = -50
    if kp.getKey("d"):      yv =  50

    # 2. 이륙 / 착륙
    if kp.getKey("e"):      
        me.takeoff()
        sleep(2)
    if kp.getKey("q"):      
        me.land()
        sleep(2)

    # 3. Dead Reckoning – 위치 추정 계산
    a += yaw                           # yaw 누적으로 현재 방향 갱신
    if fb != 0 or lr != 0:             # 전진/후진/좌우 이동이 있을 때만
        # 이동 방향 결정 (드론 기준)
        if lr == -15:   direction = -180
        elif lr == 15:  direction = 0
        elif fb == 15:  direction = 270   # 앞 = 270도 (Tello 기준)
        elif fb == -15: direction = 90
        else: direction = a

        # 삼각함수로 x, y 이동량 계산
        x += int(dInterval * math.cos(math.radians(direction + a)))
        y += int(dInterval * math.sin(math.radians(direction + a)))

    # yaw 속도 누적 (a 키, d 키)
    yaw = yv * interval / 10   # 실제 회전량으로 변환

    return [lr, fb, ud, yv, x, y]

def drawPoints(img, points):
    for point in points:
        cv2.circle(img, point, 5, (0, 0, 255), cv2.FILLED)
    # 현재 위치는 초록색으로 크게 표시
    cv2.circle(img, points[-1], 8, (0, 255, 0), cv2.FILLED)
    # 좌표를 미터 단위로 표시 (100px = 1m)
    cx, cy = points[-1]
    cv2.putText(img, f'({(cx-500)/100:.2f}, {(500-cy)/100:.2f})m',
                (cx + 10, cy + 30), cv2.FONT_HERSHEY_PLAIN, 1,
                (255, 0, 255), 1)

# 메인 루프
while True:
    vals = getKeyboardInput()
    me.send_rc_control(vals[0], vals[1], vals[2], vals[3])   # 실제 드론 제어

    # 2D 맵 그리기
    img = np.zeros((1000, 1000, 3), np.uint8)
    img[:] = (30, 30, 30)   # 어두운 배경

    # 새로운 위치가 생기면 경로에 추가
    if points[-1] != (vals[4], vals[5]):
        points.append((vals[4], vals[5]))

    drawPoints(img, points)

    # 좌표축 표시 (선택사항)
    cv2.line(img, (0, 500), (1000, 500), (255, 255, 255), 1)   # X축
    cv2.line(img, (500, 0), (500, 1000), (255, 255, 255), 1)   # Y축

    cv2.imshow("Tello Flight Path", img)
    if cv2.waitKey(1) & 0xFF == ord('c'):   # c 키 누르면 경로 초기화
        points = [(500, 500)]
    
    if cv2.waitKey(1) == 27:   # ESC 누르면 종료
        me.land()
        break

cv2.destroyAllWindows()
