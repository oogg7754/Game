from DrivingInterface.drive_controller import DrivingController

class DrivingClient(DrivingController):
    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = False # 문제가 생겼는가

        self.track_type = 99

        self.is_accident = False # 사고가 났는가
        self.recovery_count = 0 # 복구 횟수
        self.accident_count = 0 # 사고 횟수

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #

        if self.is_debug: # 문제 생겼을때 차량 정보 보여주기
            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))

            print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))

            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("=========================================================")

        ###########################################################################

        ## 도로의 실제 폭의 1/2 로 계산됨
        half_load_width = self.half_road_limit - 1.25 # 도로폭 : 10m, 차량 전폭 : 2m

        ## 차량 핸들 조정을 위해 참고할 전방의 커브 값 가져오기
        angle_num = int(sensing_info.speed / 45) # 0~45km : 0번째 각도 참조, 45~89km : 1번째 각도 참조, 90~134km : 2번째 각도 참조, 135~179km : 4번째 각도 참조
        ref_angle = sensing_info.track_forward_angles[angle_num]

        ## 차량의 차선 중앙 정렬을 위한 미세 조정 값 계산
        middle_add = (sensing_info.to_middle / 80) * -1 # to_middle : -6 ~ +6 (트랙을 완전히 벗어나지 않았을때) > middle_add : -0.075 ~ +0.075 (to_middle과 부호 반대)

        ## 전방의 커브 각도에 따라 throttle 값을 조절하여 속도를 제어함
        throttle_factor = 0.6 / (abs(ref_angle) + 0.1) # 커브 각이 클수록 엑셀을 살살 밟음, 커브 각이 클수록 엑셀을 세게 밟음
        if throttle_factor > 0.11: throttle_factor = 0.11  ## throttle 값을 최대 0.81 로 설정
        set_throttle = 0.7 + throttle_factor
        if sensing_info.speed < 60: set_throttle = 0.9  ## 속도가 60Km/h 이하인 경우 0.9 로 설정

        ## 차량의 Speed 에 따라서 핸들을 돌리는 값을 조정함
        steer_factor = sensing_info.speed * 1.5 # 70km 이하 (50km이면 steer_factor는 75)
        if sensing_info.speed > 70: steer_factor = sensing_info.speed * 0.85 # 70~100km (80km이면 steer_factor는 68)
        if sensing_info.speed > 100: steer_factor = sensing_info.speed * 0.7 # 100km 이상 (120km이면 steer_factor는 84)

        ## (참고할 전방의 커브 - 내 차량의 주행 각도) / (계산된 steer factor) 값으로 steering 값을 계산
        set_steering = (ref_angle - sensing_info.moving_angle) / (steer_factor + 0.001)
        # 커브가 왼쪽(-)으로 꺾여있고, 주행 각도가 오른쪽(+)이면 왼쪽으로 핸들 돌리기(-)
        # 커브가 오른쪽(+)으로 꺾여있고, 주행 각도가 왼쪽(-)이면 오른쪽으로 핸들 돌리기(+)
        # (참고할 전방의 커브 - 내 차량의 주행 각도)는 일반적인 경우 -60도에서 +60까지 일것으로 예상
        

        ## 차선 중앙정렬 값을 추가로 고려함
        set_steering += middle_add # middle_add : -0.075 ~ +0.075
        
        ## 긴급 및 예외 상황 처리(초기화) ########################################################################################
        full_throttle = True # 풀악셀 밟을지
        emergency_brake = False # 비상 브레이크 밟을지

        ## 전방 커브의 각도가 큰 경우 속도를 제어함
        ## 차량 핸들 조정을 위해 참고하는 커브 보다 조금 더 멀리 참고하여 미리 속도를 줄임
        road_range = int(sensing_info.speed / 30)
        for i in range(0, road_range):
            fwd_angle = abs(sensing_info.track_forward_angles[i])
            if fwd_angle > 45:  ## 커브가 45도 이상인 경우 brake, throttle 을 제어
                full_throttle = False
            if fwd_angle > 80:  ## 커브가 80도 이상인 경우 steering 까지 추가로 제어
                emergency_brake = True
                break

        ## brake, throttle 제어
        set_brake = 0.0
        if full_throttle == False:
            if sensing_info.speed > 100:
                set_brake = 0.3
            if sensing_info.speed > 120:
                set_throttle = 0.7
                set_brake = 0.7
            if sensing_info.speed > 130:
                set_throttle = 0.5
                set_brake = 1.0

        ## steering 까지 추가로 제어
        if emergency_brake:
            if set_steering > 0:
                set_steering += 0.3
            else:
                set_steering -= 0.3

        ## 충돌 상황 감지 후 회피 하기 (1~5 단계)
        ## 1. 30Km/h 이상의 속도로 달리는 경우 정상 적인 상황으로 간주
        if sensing_info.speed > 30.0:
            self.is_accident = False
            self.recovery_count = 0
            self.accident_count = 0

        ## 2. 레이싱 시작 후 Speed 1km/h 이하가 된 경우 상황 체크
        if sensing_info.lap_progress > 0.5 and self.is_accident == False and \
           (sensing_info.speed < 1.0 and sensing_info.speed > -1.0):
            self.accident_count += 1

        ## 3. Speed 1km/h 이하인 상태가 지속된 경우 충돌로 인해 멈준 것으로 간주
        if self.accident_count > 6:
            self.is_accident = True

        ## 4. 충돌로 멈춘 경우 후진 시작
        if self.is_accident == True:
            set_steering = 0.02
            set_brake = 0.0
            set_throttle = -1
            self.recovery_count += 1

        ## 5. 어느 정도 후진이 되었을 때 충돌을 회피 했다고 간주 후 정상 주행 상태로 돌림
        if self.recovery_count > 20:
            self.is_accident = False
            self.recovery_count = 0
            self.accident_count = 0
            set_steering = 0
            set_brake = 0
            set_throttle = 0
        ################################################################################################################

        # Moving straight forward
        car_controls.steering = set_steering
        car_controls.throttle = set_throttle
        car_controls.brake = set_brake
        
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}"\
                  .format(car_controls.steering, car_controls.throttle, car_controls.brake))

        #
        # Editing area ends
        # ==========================================================#
        return car_controls


    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name


if __name__ == '__main__':
    print("[MyCar] Start Bot!")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot!")

    exit(return_code)
