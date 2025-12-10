from djitellopy import Tello
import time

class AcrobaticDrone:
    def __init__(self):
        self.tello = Tello()
        
    def connect(self):
        """드론 연결"""
        print("🔌 드론 연결 중...")
        self.tello.connect()
        print(f"✅ 연결 완료!")
        print(f"📶 배터리: {self.tello.get_battery()}%")
        
    def check_battery(self):
        """배터리 확인"""
        battery = self.tello.get_battery()
        if battery < 20:
            print(f"⚠️ 배터리 부족: {battery}%")
            return False
        return True
    
    def takeoff(self):
        """이륙"""
        print("\n🚁 이륙 시작...")
        self.tello.takeoff()
        time.sleep(2)
        print("✅ 이륙 완료!")
        
    def land(self):
        """착륙"""
        print("\n🛬 착륙 시작...")
        self.tello.land()
        print("✅ 착륙 완료!")
        
    def flip_forward(self):
        """앞으로 제비돌기"""
        print("🤸 앞으로 제비돌기!")
        self.tello.flip_forward()
        time.sleep(2)
        
    def flip_back(self):
        """뒤로 제비돌기"""
        print("🤸 뒤로 제비돌기!")
        self.tello.flip_back()
        time.sleep(2)
        
    def flip_left(self):
        """왼쪽 제비돌기"""
        print("🤸 왼쪽 제비돌기!")
        self.tello.flip_left()
        time.sleep(2)
        
    def flip_right(self):
        """오른쪽 제비돌기"""
        print("🤸 오른쪽 제비돌기!")
        self.tello.flip_right()
        time.sleep(2)
        
    def rotate_360_clockwise(self):
        """시계방향 360도 회전"""
        print("🔄 시계방향 360도 회전!")
        self.tello.rotate_clockwise(360)
        time.sleep(3)
        
    def rotate_360_counter_clockwise(self):
        """반시계방향 360도 회전"""
        print("🔄 반시계방향 360도 회전!")
        self.tello.rotate_counter_clockwise(360)
        time.sleep(3)
        
    def square_pattern(self, distance=50):
        """사각형 패턴 비행"""
        print(f"⬜ 사각형 패턴 비행 (한 변: {distance}cm)")
        for i in range(4):
            self.tello.move_forward(distance)
            time.sleep(1)
            self.tello.rotate_clockwise(90)
            time.sleep(1)
        print("✅ 사각형 완료!")
        
    def circle_pattern(self):
        """원형 패턴 비행"""
        print("⭕ 원형 패턴 비행!")
        for _ in range(36):
            self.tello.move_forward(10)
            self.tello.rotate_clockwise(10)
            time.sleep(0.5)
        print("✅ 원형 완료!")
        
    def spiral_up(self):
        """나선형 상승"""
        print("🌀 나선형 상승!")
        for _ in range(8):
            self.tello.move_up(20)
            self.tello.rotate_clockwise(45)
            time.sleep(1)
        print("✅ 나선 상승 완료!")
        
    def spiral_down(self):
        """나선형 하강"""
        print("🌀 나선형 하강!")
        for _ in range(8):
            self.tello.move_down(20)
            self.tello.rotate_clockwise(45)
            time.sleep(1)
        print("✅ 나선 하강 완료!")
        
    def figure_eight(self):
        """8자 비행"""
        print("8️⃣ 8자 패턴 비행!")
        # 첫 번째 원 (시계방향)
        for _ in range(18):
            self.tello.move_forward(15)
            self.tello.rotate_clockwise(20)
            time.sleep(0.5)
        # 두 번째 원 (반시계방향)
        for _ in range(18):
            self.tello.move_forward(15)
            self.tello.rotate_counter_clockwise(20)
            time.sleep(0.5)
        print("✅ 8자 완료!")
        
    def dance_routine(self):
        """댄스 루틴"""
        print("\n💃 드론 댄스 루틴 시작!")
        
        # 상승
        self.tello.move_up(50)
        time.sleep(1)
        
        # 360도 회전
        self.rotate_360_clockwise()
        
        # 앞으로 제비돌기
        self.flip_forward()
        
        # 180도 회전
        self.tello.rotate_clockwise(180)
        time.sleep(2)
        
        # 뒤로 제비돌기
        self.flip_back()
        
        # 반시계방향 360도 회전
        self.rotate_360_counter_clockwise()
        
        # 왼쪽 제비돌기
        self.flip_left()
        
        # 오른쪽 제비돌기
        self.flip_right()
        
        # 하강
        self.tello.move_down(50)
        time.sleep(1)
        
        print("✅ 댄스 루틴 완료!")
        
    def performance_show(self):
        """전체 퍼포먼스 쇼"""
        print("\n🎪 드론 퍼포먼스 쇼 시작!")
        print("=" * 50)
        
        # 이륙
        self.takeoff()
        time.sleep(2)
        
        # 1. 사각형 패턴
        self.square_pattern(60)
        time.sleep(1)
        
        # 2. 360도 회전
        self.rotate_360_clockwise()
        time.sleep(1)
        
        # 3. 제비돌기 4방향
        print("\n🎯 4방향 제비돌기!")
        self.flip_forward()
        self.flip_right()
        self.flip_back()
        self.flip_left()
        time.sleep(1)
        
        # 4. 나선형 상승
        self.spiral_up()
        time.sleep(1)
        
        # 5. 8자 비행
        self.figure_eight()
        time.sleep(1)
        
        # 6. 나선형 하강
        self.spiral_down()
        time.sleep(1)
        
        # 7. 마지막 360도 회전
        self.rotate_360_counter_clockwise()
        
        # 착륙
        self.land()
        
        print("\n" + "=" * 50)
        print("🎉 퍼포먼스 쇼 완료!")
        print(f"📶 최종 배터리: {self.tello.get_battery()}%")


def main():
    """메인 실행 함수"""
    drone = AcrobaticDrone()
    
    try:
        # 드론 연결
        drone.connect()
        
        # 배터리 확인
        if not drone.check_battery():
            print("❌ 배터리를 충전해주세요!")
            return
        
        print("\n" + "=" * 50)
        print("🎮 드론 자율 곡예 비행 프로그램")
        print("=" * 50)
        print("\n원하는 모드를 선택하세요:")
        print("1. 전체 퍼포먼스 쇼 (추천!)")
        print("2. 댄스 루틴")
        print("3. 사각형 패턴")
        print("4. 원형 패턴")
        print("5. 8자 비행")
        print("6. 나선형 비행")
        print("7. 제비돌기만")
        print("8. 360도 회전만")
        
        choice = input("\n선택 (1-8): ").strip()
        
        if choice == "1":
            drone.performance_show()
        elif choice == "2":
            drone.takeoff()
            drone.dance_routine()
            drone.land()
        elif choice == "3":
            drone.takeoff()
            drone.square_pattern(60)
            drone.land()
        elif choice == "4":
            drone.takeoff()
            drone.circle_pattern()
            drone.land()
        elif choice == "5":
            drone.takeoff()
            drone.figure_eight()
            drone.land()
        elif choice == "6":
            drone.takeoff()
            drone.spiral_up()
            time.sleep(1)
            drone.spiral_down()
            drone.land()
        elif choice == "7":
            drone.takeoff()
            time.sleep(2)
            drone.flip_forward()
            drone.flip_back()
            drone.flip_left()
            drone.flip_right()
            drone.land()
        elif choice == "8":
            drone.takeoff()
            time.sleep(2)
            drone.rotate_360_clockwise()
            time.sleep(1)
            drone.rotate_360_counter_clockwise()
            drone.land()
        else:
            print("❌ 잘못된 선택입니다.")
            
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        try:
            drone.land()
        except:
            pass
    finally:
        print("\n👋 프로그램 종료")


if __name__ == "__main__":
    main()
