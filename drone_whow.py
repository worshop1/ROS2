from djitellopy import Tello
import time

class AcrobaticDrone:
    def __init__(self, is_edu=False):
        """
        드론 객체 초기화
        - Tello 클래스 생성
        - 이동 속도, 회전 속도 설정
        - EDU 여부 저장
        """
        self.tello = Tello()
        self.is_edu = is_edu
        self.speed = 15       # RC 이동 속도 (cm/s)
        self.aspeed = 50      # RC 회전 속도 (deg/s)

    def connect(self):
        """
        드론 연결
        - WiFi 연결 시도
        - 배터리 확인
        """
        print("🔌 드론 연결 중...")
        self.tello.connect()
        print("✅ 연결 완료!")
        print(f"📶 배터리: {self.tello.get_battery()}%")

        model = "Tello EDU" if self.is_edu else "일반 Tello"
        print(f"🤖 모델: {model}")

    def check_battery(self):
        """
        배터리 확인 (20% 미만이면 비행 금지)
        """
        battery = self.tello.get_battery()
        if battery < 20:
            print(f"⚠️ 배터리 부족: {battery}%")
            return False
        return True

    def takeoff(self):
        """
        이륙 후 안정화 대기
        """
        print("\n🚁 이륙 시작...")
        self.tello.takeoff()
        time.sleep(3)
        print("✅ 이륙 완료!")

    def land(self):
        """
        착륙 후 안정화 대기
        """
        print("\n🛬 착륙 시작...")
        self.tello.land()
        time.sleep(3)
        print("✅ 착륙 완료!")

    def stop(self):
        """
        RC 명령 0 → 드론 정지
        """
        self.tello.send_rc_control(0, 0, 0, 0)

    # ===========================
    #      제비돌기(Flip)
    # ===========================

    def _prepare_for_flip(self):
        """
        플립 수행 전 안전 고도 확보
        약 60cm 상승 후 정지
        """
        print("   ⬆️ 안전 높이 확보 중...")
        self.tello.send_rc_control(0, 0, self.speed, 0)
        time.sleep(4)
        self.stop()
        time.sleep(1)

    def flip_forward(self):
        """앞으로 제비돌기"""
        print("🤸 앞으로 제비돌기!")
        self._prepare_for_flip()
        self.tello.flip_forward()
        time.sleep(3)

    def flip_back(self):
        """뒤로 제비돌기"""
        print("🤸 뒤로 제비돌기!")
        self._prepare_for_flip()
        self.tello.flip_back()
        time.sleep(3)

    def flip_left(self):
        """왼쪽 제비돌기"""
        print("🤸 왼쪽 제비돌기!")
        self._prepare_for_flip()
        self.tello.flip_left()
        time.sleep(3)

    def flip_right(self):
        """오른쪽 제비돌기"""
        print("🤸 오른쪽 제비돌기!")
        self._prepare_for_flip()
        self.tello.flip_right()
        time.sleep(3)


    # ===========================
    #      RC 기반 이동 제어
    # ===========================

    def move_forward_rc(self, duration=1):
        """앞으로 이동"""
        self.tello.send_rc_control(0, self.speed, 0, 0)
        time.sleep(duration)
        self.stop()

    def move_back_rc(self, duration=1):
        """뒤로 이동"""
        self.tello.send_rc_control(0, -self.speed, 0, 0)
        time.sleep(duration)
        self.stop()

    def move_left_rc(self, duration=1):
        """왼쪽 이동"""
        self.tello.send_rc_control(-self.speed, 0, 0, 0)
        time.sleep(duration)
        self.stop()

    def move_right_rc(self, duration=1):
        """오른쪽 이동"""
        self.tello.send_rc_control(self.speed, 0, 0, 0)
        time.sleep(duration)
        self.stop()

    def move_up_rc(self, duration=1):
        """상승"""
        self.tello.send_rc_control(0, 0, self.speed, 0)
        time.sleep(duration)
        self.stop()

    def move_down_rc(self, duration=1):
        """하강"""
        self.tello.send_rc_control(0, 0, -self.speed, 0)
        time.sleep(duration)
        self.stop()

    def rotate_clockwise_rc(self, duration=1):
        """시계 방향 회전"""
        self.tello.send_rc_control(0, 0, 0, self.aspeed)
        time.sleep(duration)
        self.stop()

    def rotate_counter_clockwise_rc(self, duration=1):
        """반시계 방향 회전"""
        self.tello.send_rc_control(0, 0, 0, -self.aspeed)
        time.sleep(duration)
        self.stop()


    # ===========================
    #        회전 함수
    # ===========================

    def rotate_360_clockwise(self):
        """시계 방향 360도 회전"""
        print("🔄 시계방향 360도 회전!")
        self.rotate_clockwise_rc(7.2)  # 360 / 50deg/s
        print("✅ 회전 완료!")

    def rotate_360_counter_clockwise(self):
        """반시계 방향 360도 회전"""
        print("🔄 반시계방향 360도 회전!")
        self.rotate_counter_clockwise_rc(7.2)
        print("✅ 회전 완료!")


    # ===========================
    #       패턴 비행 알고리즘
    # ===========================

    def square_pattern(self, duration=2):
        """사각형 패턴 비행"""
        print("⬜ 사각형 패턴 비행")
        for i in range(4):
            self.move_forward_rc(duration)
            self.rotate_clockwise_rc(1.8)  # 약 90도

    def triangle_pattern(self, duration=2):
        """삼각형 패턴 비행"""
        print("🔺 삼각형 패턴 비행")
        for i in range(3):
            self.move_forward_rc(duration)
            self.rotate_clockwise_rc(2.4)  # 약 120도

    def circle_pattern(self, duration=10):
        """원형 궤적 비행"""
        print("⭕ 원형 비행")
        self.tello.send_rc_control(0, self.speed, 0, 30)
        time.sleep(duration)
        self.stop()

    def figure_eight(self, duration=6):
        """8자 비행"""
        print("8️⃣ 8자 패턴 비행")
        # 첫 번째 원
        self.tello.send_rc_control(0, self.speed, 0, 35)
        time.sleep(duration)
        self.stop()
        # 두 번째 원 (반대 방향)
        self.tello.send_rc_control(0, self.speed, 0, -35)
        time.sleep(duration)
        self.stop()

    def zigzag_pattern(self, duration=2):
        """지그재그 비행"""
        self.move_forward_rc(duration)
        self.move_right_rc(duration)
        self.move_forward_rc(duration)
        self.move_left_rc(duration)
        self.move_forward_rc(duration)

    def up_down_dance(self):
        """상하 댄스"""
        for _ in range(3):
            self.move_up_rc(2)
            self.move_down_rc(2)

    def spiral_up(self):
        """나선형 상승"""
        print("🌀 나선형 상승")
        for _ in range(8):
            self.tello.send_rc_control(0, 0, self.speed, 25)
            time.sleep(1)
            self.stop()

    def spiral_down(self):
        """나선형 하강"""
        print("🌀 나선형 하강")
        for _ in range(8):
            self.tello.send_rc_control(0, 0, -self.speed, 25)
            time.sleep(1)
            self.stop()


    # ===========================
    #      복합 퍼포먼스
    # ===========================

    def dance_routine(self):
        """댄스 루틴 시퀀스"""
        print("💃 드론 댄스 루틴 시작")

        self.move_up_rc(3)                    # 상승
        self.rotate_360_clockwise()           # 360 회전
        self.flip_forward()                   # 플립
        self.rotate_clockwise_rc(3.6)         # 180도 회전
        self.flip_back()
        self.rotate_360_counter_clockwise()
        self.flip_left()
        self.flip_right()
        self.move_down_rc(3)                  # 하강

    def performance_show(self):
        """전체 퍼포먼스 쇼"""
        print("🎪 전체 쇼 시작")

        self.takeoff()
        self.square_pattern()
        self.rotate_360_clockwise()
        self.flip_forward()
        self.flip_right()
        self.flip_back()
        self.flip_left()
        self.triangle_pattern()
        self.circle_pattern()
        self.up_down_dance()
        self.spiral_up()
        self.figure_eight()
        self.spiral_down()
        self.rotate_360_counter_clockwise()
        self.land()


# ===========================
#           MAIN
# ===========================

def main():
    """프로그램 메인 메뉴"""

    print("🎮 드론 자율 곡예 비행 프로그램")

    drone = AcrobaticDrone()

    try:
        drone.connect()

        if not drone.check_battery():
            print("❌ 배터리 부족으로 종료")
            return

        print("1. 퍼포먼스 쇼")
        print("2. 댄스 루틴")
        print("3. 사각형 패턴")
        print("4. 삼각형 패턴")
        print("5. 원형 비행")
        print("6. 8자 비행")
        print("7. 지그재그")
        print("8. 상하 댄스")
        print("9. 나선형 비행")
        print("10. 제비돌기")
        print("11. 360도 회전")

        choice = input("선택 (1-11): ").strip()

        if choice == "1":
            drone.performance_show()
        elif choice == "2":
            drone.takeoff()
            drone.dance_routine()
            drone.land()
        elif choice == "3":
            drone.takeoff()
            drone.square_pattern()
            drone.land()
        elif choice == "4":
            drone.takeoff()
            drone.triangle_pattern()
            drone.land()
        elif choice == "5":
            drone.takeoff()
            drone.circle_pattern()
            drone.land()
        elif choice == "6":
            drone.takeoff()
            drone.figure_eight()
            drone.land()
        elif choice == "7":
            drone.takeoff()
            drone.zigzag_pattern()
            drone.land()
        elif choice == "8":
            drone.takeoff()
            drone.up_down_dance()
            drone.land()
        elif choice == "9":
            drone.takeoff()
            drone.spiral_up()
            drone.spiral_down()
            drone.land()
        elif choice == "10":
            drone.takeoff()
            drone.flip_forward()
            drone.flip_back()
            drone.flip_left()
            drone.flip_right()
            drone.land()
        elif choice == "11":
            drone.takeoff()
            drone.rotate_360_clockwise()
            drone.rotate_360_counter_clockwise()
            drone.land()
        else:
            print("❌ 잘못된 선택")

    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        try:
            drone.land()
        except:
            pass

    finally:
        print("👋 프로그램 종료")


if __name__ == "__main__":
    main()
