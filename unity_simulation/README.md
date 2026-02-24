# Unity Simulation – ROS2 Wheel Control & Odometry

Unity 물리 엔진을 사용하여 ROS2 `/cmd_rpm` 명령을 수신하고, 가상 차량을 제어하며, 엔코더 기반 Odometry를 `/odom`으로 발행합니다.

## 파일 구조

```
unity_simulation/
└── Assets/
    └── Scripts/
        ├── Messages/
        │   └── WheelRPM.cs          # custom_msgs/WheelRPM C# 바인딩
        ├── WheelController.cs       # 메인 제어 스크립트 (구독/발행/물리)
        ├── SimulationBootstrap.cs   # ROS2 컨텍스트 초기화
        ├── VehicleSetupHelper.cs    # WheelCollider 초기 설정 헬퍼
        └── ScenarioTester.cs        # 기본 동작 시나리오 자동 테스트
```

## 의존성

| 패키지 | 버전 |
|--------|------|
| [ros2-for-unity](https://github.com/RobotecAI/ros2-for-unity) | ≥ 1.3 |
| ROS2 (Humble / Iron / Jazzy) | - |
| custom_msgs (이 저장소의 `ros2_ws/src/custom_msgs`) | 0.1.0 |

## Unity 씬 설정

1. **ROS2Manager** 빈 오브젝트 생성
   - `ROS2UnityComponent` 추가
   - `SimulationBootstrap` 추가

2. **차량 루트 오브젝트**에 다음 추가:
   - `Rigidbody` (질량: 15 kg)
   - `WheelController` → Inspector에서 WheelCollider 4개 연결
   - `VehicleSetupHelper` → 컨텍스트 메뉴 "Configure Vehicle" 실행

3. WheelCollider 4개를 각 바퀴 위치에 추가 (FL/FR/RL/RR)

## ROS2 토픽

| 방향 | 토픽 | 타입 |
|------|------|------|
| 수신 | `/cmd_rpm` | `custom_msgs/WheelRPM` |
| 수신 | `/estop` | `std_msgs/Bool` |
| 발행 | `/odom` | `nav_msgs/Odometry` |

## custom_msgs 빌드

```bash
cd ros2_ws
colcon build --packages-select custom_msgs
source install/setup.bash
```

## 시나리오 테스트

`ScenarioTester` 컴포넌트의 **Run On Start** 체크박스를 활성화하면 Unity 플레이 시 자동으로 다음 시나리오를 순차 실행합니다:

1. 직진 (driveRpm, driveRpm)
2. 제자리 좌회전 (-driveRpm, +driveRpm)
3. 제자리 우회전 (+driveRpm, -driveRpm)
4. 저속 전진 (creepRpm, creepRpm)
5. 완전 정지 (0, 0)

## ESTOP

`/estop` 토픽에 `std_msgs/Bool(data=true)` 를 발행하면 즉시 모든 바퀴의 토크를 차단하고 최대 브레이크를 적용합니다.

```bash
ros2 topic pub /estop std_msgs/msg/Bool "data: true" --once
```
