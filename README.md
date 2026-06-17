# HereLink RC Reader

HereLink 조종기 입력을 `HereLink Air Unit -> S.Bus -> Cube/Mini Carrier Board -> USB -> Python/ROS2` 경로로 읽고, ROS2 `/cmd_vel`로 변환하는 도구입니다.

Camera autoview 관련 파일은 `camera_autoview/` 아래에 있습니다.

## 연결 구조

```text
HereLink Controller
        |
        v
HereLink Air Unit S.Bus1
        |
        |  Signal + GND
        v
Mini Carrier Board RC IN
        |
        v
CubeOrange+
        |
        |  micro USB
        v
PC /dev/ttyACM0
```

## 배선 체크

```text
HereLink Air Unit S.Bus1     Mini Carrier Board RC IN
Signal / S        -------->  Signal / S / PPM-SBUS
GND / -           -------->  GND / -
5V / +            -------->  연결하지 않아도 됨
```

- Air Unit은 별도 전원이 필요합니다.
- `RC IN`에는 보통 `Signal + GND`만 연결합니다.
- Mini Carrier Board 기본 `RC IN`은 `PPM/S.BUS`입니다.
- Mini Carrier Board가 `SPKT` 모드로 개조되어 있으면 HereLink S.Bus 입력이 안 들어올 수 있습니다.

## 실행 전 확인

Cube가 PC에 정상 인식되는지 먼저 확인합니다.

```bash
lsusb | grep -i cube
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -ls
```

정상 예시:

```text
CubePilot CubeOrange+
/dev/ttyACM0
```

`CubeOrange+-BL`은 bootloader 모드입니다. 잠깐 보였다가 `CubeOrange+`로 바뀌는 것은 괜찮지만, 계속 `BL`이거나 USB disconnect가 반복되면 정상 RC 읽기 상태가 아닙니다.

## Python 유틸리티

필요 패키지:

```bash
sudo apt install python3-serial
```

채널 1-4 확인:

```bash
python3 utils/read_rc_from_cube.py --port /dev/ttyACM0 --first4 --timeout 0
```

출력 예시:

```text
t=179627ms rssi=40 | ch1=1514 | ch2=1514 | ch3=1514 | ch4=1514
```

전체 RC 채널 JSON:

```bash
python3 utils/read_rc_from_cube.py --port /dev/ttyACM0 --timeout 0
```

Python 코드에서 사용:

```python
from utils.read_rc_from_cube import iter_rc_messages

for message in iter_rc_messages("/dev/ttyACM0", timeout=0):
    channels = message["channels"]
    print(channels.get("ch1"), channels.get("ch2"), channels.get("ch3"), channels.get("ch4"))
```

MAVLink 진단:

```bash
python3 utils/check_mavlink_uart.py --port /dev/ttyACM0 --baud 115200 --seconds 5 --summary-only --frame-limit 20
```

정상적으로 RC가 들어오면 `RC_CHANNELS msgid=65`, `RC_CHANNELS_RAW msgid=35`, `MANUAL_CONTROL msgid=69` 중 하나 이상이 보입니다.

## ROS2 `/cmd_vel` 노드

ROS2 패키지는 `herelink_sbus_cmd_vel/`입니다. 이 패키지의 `sbus_cmd_vel_node`는 Cube MAVLink `RC_CHANNELS`를 읽고 `geometry_msgs/msg/Twist`를 `/cmd_vel`로 publish합니다.

확인된 S.Bus1 조이스틱 매핑:

```text
left stick  앞: ch2=1102
left stick  뒤: ch2=1927
left stick  좌: ch4=1102
left stick  우: ch4=1927

right stick 앞: ch3=1102
right stick 뒤: ch3=1927
right stick 좌: ch1=1102
right stick 우: ch1=1927

center: 1524
```

기본 `/cmd_vel` 변환:

```text
ch2 -> linear.x
ch4 -> linear.y
ch1 -> angular.z
```

기본 방향:

- left stick 앞: `linear.x` 양수
- left stick 뒤: `linear.x` 음수
- left stick 좌: `linear.y` 양수
- left stick 우: `linear.y` 음수
- right stick 좌: `angular.z` 양수
- right stick 우: `angular.z` 음수
- right stick 앞/뒤: 기본 미사용

## ROS2 빌드와 실행

이 레포는 ROS2 워크스페이스의 `src` 아래에 둔 뒤, 워크스페이스 루트에서 빌드합니다.

```bash
cd ~/ros2_ws
colcon build --packages-select herelink_sbus_cmd_vel
source install/setup.bash
```

빌드 중 `catkin_pkg` 관련 Python 에러가 나면 ROS2가 쓰는 Python을 명시합니다.

```bash
cd ~/ros2_ws
colcon build --packages-select herelink_sbus_cmd_vel --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

Launch 실행:

```bash
ros2 launch herelink_sbus_cmd_vel sbus_cmd_vel.launch.py
```

직접 실행:

```bash
ros2 run herelink_sbus_cmd_vel sbus_cmd_vel_node --ros-args -p port:=/dev/ttyACM0
```

디버그 로그:

```bash
ros2 run herelink_sbus_cmd_vel sbus_cmd_vel_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p debug_output:=true
```

출력 확인:

```bash
ros2 topic echo /cmd_vel
```

## ROS 토픽

이 패키지가 publish하는 주 토픽은 `/cmd_vel`입니다.

- `/cmd_vel`: `herelink_sbus_cmd_vel` 노드가 publish하는 `geometry_msgs/msg/Twist`
- `/parameter_events`: ROS2 파라미터 변경 이벤트 기본 토픽
- `/rosout`: ROS2 로그 기본 토픽
- `/robot_path_requests`: `rmf_fleet_msgs/msg/PathRequest` 타입의 RMF 계열 경로 요청 토픽
- `/robot_state`: `rmf_fleet_msgs/msg/RobotState` 타입의 RMF 계열 로봇 상태 토픽

`/robot_path_requests`와 `/robot_state`는 이 패키지의 조종 노드가 만든 토픽이 아닙니다. 기존 RMF/fleet adapter 쪽 프로세스나 DDS discovery 정보 때문에 보일 수 있습니다.

## 주요 파라미터

- `port`: 시리얼 포트, 기본 `/dev/ttyACM0`
- `baud`: 시리얼 baud rate, 기본 `115200`
- `linear_channel`: 전후진 채널, 기본 `2`
- `lateral_channel`: 좌우 이동 채널, 기본 `4`
- `angular_channel`: 회전 채널, 기본 `1`
- `center_pwm`: 중앙 PWM, 기본 `1524`
- `min_pwm`: 최소 PWM, 기본 `1102`
- `max_pwm`: 최대 PWM, 기본 `1927`
- `deadband_pwm`: 중앙 deadband, 기본 `30`
- `max_linear`: 최대 `linear.x`, 기본 `1.0`
- `max_lateral`: 최대 `linear.y`, 기본 `1.0`
- `max_angular`: 최대 `angular.z`, 기본 `1.0`
- `linear_direction`: 전후진 방향 보정, 기본 `-1.0`
- `lateral_direction`: 좌우 이동 방향 보정, 기본 `-1.0`
- `angular_direction`: 회전 방향 보정, 기본 `-1.0`
- `rc_timeout_sec`: RC 입력 timeout, 기본 `0.5`
- `publish_zero_on_timeout`: timeout 시 정지 명령 publish, 기본 `true`
- `debug_output`: RC 입력과 `/cmd_vel` 변환 로그 출력, 기본 `false`

속도 제한 예시:

```bash
ros2 run herelink_sbus_cmd_vel sbus_cmd_vel_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p max_linear:=0.5 \
  -p max_lateral:=0.5 \
  -p max_angular:=1.2
```

방향이 반대로 느껴지면 부호 파라미터만 바꿉니다.

```bash
ros2 run herelink_sbus_cmd_vel sbus_cmd_vel_node --ros-args \
  -p port:=/dev/ttyACM0 \
  -p linear_direction:=1.0
```

## 트러블슈팅

### `/dev/ttyACM0`가 없을 때

```text
Failed to open /dev/ttyACM0: No such file or directory
```

확인:

```bash
lsusb | grep -i cube
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -ls
```

원인 후보:

- Cube USB 케이블이 빠짐
- 충전 전용 micro USB 케이블 사용
- USB 포트/허브 접촉 불량
- Mini Carrier Board micro USB 포트 접촉 불량
- Cube 또는 Air Unit 전원 불안정

### MAVLink는 나오는데 RC가 안 보일 때

```text
MAVLink-like frames found: ...
No RC_CHANNELS, RC_CHANNELS_RAW, or MANUAL_CONTROL frames were seen during this capture.
```

원인 후보:

- Air Unit 전원이 안 들어옴
- Air Unit과 조종기가 페어링되지 않음
- S.Bus1의 Signal/GND가 RC IN에 반대로 연결됨
- Mini Carrier Board가 `SPKT` 모드로 개조됨
- Air Unit S.Bus 포트가 다른 포트와 헷갈림

### Permission denied가 날 때

임시 실행:

```bash
sudo python3 utils/read_rc_from_cube.py --port /dev/ttyACM0 --first4 --once
```

영구 해결:

```bash
sudo usermod -aG dialout $USER
```

그 다음 로그아웃 후 다시 로그인합니다.

## 파일 설명

- `herelink_sbus_cmd_vel/`: `/cmd_vel`을 publish하는 C++ ROS2 패키지
- `utils/read_rc_from_cube.py`: Cube USB MAVLink에서 RC 채널을 읽는 Python 스크립트
- `utils/check_mavlink_uart.py`: MAVLink 프레임과 메시지 종류를 확인하는 진단 스크립트
- `camera_autoview/`: 카메라 자동 실행 관련 파일
- `docs/`: Rover metadata 문서와 예시
