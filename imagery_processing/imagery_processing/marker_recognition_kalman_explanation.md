# Marker Recognition 노드 설명서

이 문서는 `marker_recognition` Python 노드를 처음 보는 사람이 전체 흐름을 이해할 수 있도록 정리한 설명서입니다.  
이 노드는 **SIYI A8 mini 카메라 영상에서 ArUco 마커를 찾고**, 마커 중심과 화면 중심의 차이를 **미터 단위 오차 `x_m`, `y_m`**로 변환한 뒤, `/landing/coordinates` 토픽으로 발행합니다. 또한 Lidar 고도값을 읽어 PX4에 `DistanceSensor`로 보내고, 카메라 줌과 짐벌 pitch 제어도 수행합니다.

---

## 1. 전체 목적

이 노드의 목적은 다음과 같습니다.

```text
카메라 영상 수신
→ ArUco 마커 검출
→ 화면 중심과 마커 중심의 픽셀 오차 계산
→ 카메라 내부 파라미터와 고도를 이용해 m 단위 위치 오차 계산
→ Kalman Filter로 x, y 오차 안정화
→ /landing/coordinates로 발행
→ 랜딩 제어 노드가 이 값을 받아 드론을 마커 중심으로 유도
```

즉, 이 노드는 **비전 기반 착륙에서 “마커가 드론 기준으로 어디에 있는지”를 계산해주는 노드**입니다.

---

## 2. 주요 ROS 토픽

### Subscribe

| 토픽 | 메시지 타입 | 역할 |
|---|---|---|
| `/fmu/out/vehicle_attitude` | `px4_msgs/msg/VehicleAttitude` | 드론 roll/pitch를 받아 Lidar 값 신뢰성 판단 |
| `/mission_mode` | `std_msgs/msg/String` | 현재 미션 상태 수신 |
| 카메라 RTSP | GStreamer pipeline | SIYI A8 mini 영상 입력 |

### Publish

| 토픽 | 메시지 타입 | 역할 |
|---|---|---|
| `/landing/coordinates` | `geometry_msgs/msg/PointStamped` | 랜딩 제어에 사용할 x, y, z 발행 |
| `/landing/video/compressed` | `sensor_msgs/msg/CompressedImage` | 디버그용 HUD 영상 발행 |
| `/fmu/in/distance_sensor` | `px4_msgs/msg/DistanceSensor` | PX4에 Lidar 거리값 전달 |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | 짐벌 pitch/yaw, 줌 관련 명령 전달 |

---

## 3. `/landing/coordinates`의 의미

이 노드가 가장 중요하게 발행하는 값은 다음입니다.

```python
msg.point.x = self.x_m
msg.point.y = self.y_m
msg.point.z = self._altitude
```

의미는 다음과 같습니다.

| 값 | 의미 | 단위 |
|---|---|---|
| `point.x` | 화면 중심 기준 마커의 좌우 오차 | m |
| `point.y` | 화면 중심 기준 마커의 전후 오차 | m |
| `point.z` | Lidar 기반 고도 | m |

현재 코드에서 `x_m`, `y_m`은 raw 값이 아니라 **Kalman Filter를 통과한 filtered 또는 predicted 값**입니다.

---

## 4. 영상 입력: GStreamer Pipeline

카메라 영상은 `build_gst_pipeline()` 함수에서 만든 GStreamer pipeline으로 들어옵니다.

```python
rtspsrc location=rtsp://192.168.0.20:8554/main.264 ...
```

이 부분은 SIYI A8 mini의 RTSP 영상을 OpenCV에서 읽기 위한 설정입니다.

전체 흐름은 다음과 같습니다.

```text
SIYI A8 mini RTSP stream
→ GStreamer decode
→ OpenCV VideoCapture
→ frame 단위 처리
```

---

## 5. SIYI A8 mini UDP 제어 클래스

`SiyiA8MiniUDP` 클래스는 카메라에 직접 UDP 명령을 보내기 위한 클래스입니다.

주로 자동 줌 기능에서 사용됩니다.

### `build_packet()`

```python
def build_packet(self, cmd_id, payload, need_ack=True)
```

이 함수는 SIYI SDK 형식에 맞춰 UDP 패킷을 만듭니다.

패킷 구성은 대략 다음과 같습니다.

```text
STX
CTRL
DATA_LENGTH
SEQ
CMD_ID
PAYLOAD
CRC16
```

각 항목의 의미는 다음과 같습니다.

| 항목 | 의미 |
|---|---|
| `0x55 0x66` | 패킷 시작 바이트 |
| `ctrl` | ACK 필요 여부 |
| `data_len` | payload 길이 |
| `seq` | 패킷 시퀀스 번호 |
| `cmd` | 명령 ID |
| `payload` | 실제 명령 데이터 |
| `crc` | 오류 검출용 CRC16 |

### `send_packet()`

```python
def send_packet(self, packet)
```

생성된 UDP 패킷을 카메라 IP와 port로 전송하고, ACK 응답을 기다립니다.

ACK가 오면 응답 데이터를 반환하고, timeout이면 `None`을 반환합니다.

### `absolute_zoom()`

```python
def absolute_zoom(self, zoom)
```

카메라 줌 배율을 직접 지정합니다.

예:

```text
1.0 → 1배 줌
2.0 → 2배 줌
4.0 → 4배 줌
```

---

## 6. Lidar 처리

Lidar는 I2C로 읽습니다.

주요 상수는 다음과 같습니다.

```python
I2C_BUS = 7
LIDAR_ADDR = 0x62
```

### 처리 흐름

```text
Lidar 측정 시작
→ distance_cm 읽기
→ m 단위 변환
→ 센서 장착 높이 보정
→ roll/pitch gate 검사
→ altitude 업데이트
→ PX4 DistanceSensor로 publish
```

### 고도 계산

```python
distance_m = distance_cm / 100.0
raw_altitude = distance_m - self._lidar_altitude
```

여기서:

| 값 | 의미 |
|---|---|
| `distance_m` | Lidar가 직접 측정한 거리 |
| `self._lidar_altitude` | Lidar가 기체에서 떨어진 장착 높이 |
| `raw_altitude` | 실제 기체 하단 또는 기준점 기준 고도 |

---

## 7. Lidar attitude gate

드론이 많이 기울면 Lidar가 바닥을 제대로 보지 못할 수 있습니다.  
그래서 roll/pitch가 너무 크면 Lidar 값을 바로 사용하지 않고 이전 값을 유지합니다.

```python
if abs(roll_deg) > gate or abs(pitch_deg) > gate:
    return False
```

기본 gate 값은 다음입니다.

```python
lidar_attitude_gate_deg = 20.0
```

즉 roll 또는 pitch가 20도보다 커지면 Lidar 값을 신뢰하지 않습니다.

---

## 8. ArUco 마커 검출

마커 검출은 `_camera_timer_cb()`에서 매 프레임 수행됩니다.

### 처리 흐름

```text
카메라 frame 읽기
→ grayscale 변환
→ raw / CLAHE / gamma 전처리 후보 생성
→ 각 후보 이미지에서 ArUco 검출
→ 가장 큰 마커 선택
→ 마커 중심 cx, cy 계산
```

### 전처리 후보

코드는 3가지 이미지를 순서대로 시도합니다.

```python
candidates = [
    ("raw", gray),
    ("clahe", gray_clahe),
    ("gamma", gray_gamma),
]
```

| 이름 | 의미 |
|---|---|
| `raw` | 원본 grayscale |
| `clahe` | 대비 향상 |
| `gamma` | gamma correction 적용 |

이 중 하나에서 마커가 검출되면 그 결과를 사용합니다.

---

## 9. 픽셀 오차에서 m 단위 오차로 변환

마커 중심이 검출되면 다음 값을 계산합니다.

```python
dx_px = cx - cx0
dy_px = cy0 - cy
```

여기서:

| 값 | 의미 |
|---|---|
| `cx, cy` | 검출된 마커 중심 pixel 좌표 |
| `cx0, cy0` | 화면 중심 pixel 좌표 |
| `dx_px` | 화면 중심 기준 좌우 pixel 오차 |
| `dy_px` | 화면 중심 기준 상하 pixel 오차 |

그 다음 카메라 내부 파라미터와 고도 `z`를 이용해 m 단위로 변환합니다.

```python
raw_x_m = dx_px / fx * z
raw_y_m = dy_px / fy * z
```

여기서:

| 값 | 의미 |
|---|---|
| `fx`, `fy` | 카메라 focal length |
| `z` | 현재 고도 |
| `raw_x_m` | m 단위 좌우 오차 |
| `raw_y_m` | m 단위 전후 오차 |

---

## 10. Kalman Filter를 쓰는 이유

카메라 검출값은 프레임마다 튈 수 있습니다.

예를 들어 raw 값이 다음처럼 들어올 수 있습니다.

```text
0.80 → 0.75 → 0.78 → 1.10 → 0.72
```

여기서 `1.10`은 실제 움직임이 아니라 검출 노이즈일 수 있습니다.

Kalman Filter는 이런 raw 값을 바로 쓰지 않고, 이전 상태와 현재 측정값을 조합해서 더 안정적인 값을 만듭니다.

---

## 11. Kalman Filter 상태 정의

코드의 `TargetKalman2D` 클래스는 다음 상태를 사용합니다.

```text
x = [x_m, y_m, vx_mps, vy_mps]^T
```

| 상태 | 의미 |
|---|---|
| `x_m` | 마커 좌우 위치 오차 |
| `y_m` | 마커 전후 위치 오차 |
| `vx_mps` | x 오차 변화율 |
| `vy_mps` | y 오차 변화율 |

측정값은 다음입니다.

```text
z = [raw_x_m, raw_y_m]^T
```

즉 카메라가 직접 주는 값은 위치 오차 `x_m`, `y_m`뿐이고, 속도 `vx`, `vy`는 Kalman Filter 내부에서 추정합니다.

---

## 12. filtered 값과 predicted 값의 차이

### filtered 값

마커가 검출되었을 때 사용합니다.

```text
예측
→ 실제 raw 측정값으로 보정
→ filtered 값
```

즉 검출 성공 시:

```python
self.x_m, self.y_m = self._target_kf.update(raw_x_m, raw_y_m)
```

이 값은 raw 값 그대로가 아니라, Kalman Filter로 안정화된 값입니다.

### predicted 값

마커가 검출되지 않았을 때 사용합니다.

```text
이전 상태만 이용해서 현재 위치 예측
→ predicted 값
```

즉 검출 실패 시:

```python
self.x_m, self.y_m = self._target_kf.predict_only()
```

이 값은 카메라 측정 없이 이전 움직임만으로 예측한 값입니다.

---

## 13. 검출 성공/실패 시 동작

### 검출 성공

```text
ArUco 검출 성공
→ raw_x_m, raw_y_m 계산
→ Kalman update
→ filtered x_m, y_m publish
→ HUD에는 초록 십자가 표시
```

이때 `/landing/coordinates`에는 filtered 값이 나갑니다.

### 검출 실패 후 10초 이내

```text
ArUco 검출 실패
→ Kalman predict_only
→ predicted x_m, y_m publish
→ HUD에는 빨간 십자가 표시
```

이때도 `/landing/coordinates`에는 NaN이 아니라 predicted 값이 나갑니다.

### 검출 실패 후 10초 초과

```text
10초 이상 검출 실패
→ Kalman Filter reset
→ x_m, y_m = NaN
→ HUD 십자가 없음
→ /landing/coordinates에 NaN publish
```

---

## 14. HUD 표시 의미

디버그 영상 `/landing/video/compressed`에는 다음 정보가 표시됩니다.

| 표시 | 의미 |
|---|---|
| 초록 십자가 | 실제 카메라가 검출한 ArUco 중심 |
| 파란 십자가 | 화면 중심 |
| 초록 선 | 화면 중심과 실제 마커 중심을 연결 |
| 빨간 십자가 | 마커 lost 시 Kalman prediction 위치 |
| `raw:` | 실제 검출 기반 raw m 단위 오차 |
| `pub:` | 실제 publish되는 filtered/predicted m 단위 오차 |
| `ALT:` | 현재 Lidar 고도 |
| `ZOOM/CALIB` | 현재 줌 배율과 사용 중인 카메라 보정값 |

---

## 15. Kalman Filter 파라미터

다음 ROS parameter를 사용합니다.

```python
target_kf_process_var = 0.01
target_kf_measurement_var = 0.08
target_predict_timeout = 10.0
```

### `target_kf_process_var`

Kalman Filter의 process noise, 즉 모델 불확실성입니다.

```text
값을 키우면:
  움직임 변화에 더 빠르게 반응
  하지만 덜 부드러움

값을 줄이면:
  더 부드러움
  하지만 반응이 느림
```

### `target_kf_measurement_var`

Kalman Filter의 measurement noise, 즉 raw 카메라 측정값의 불확실성입니다.

```text
값을 키우면:
  raw 측정값을 덜 믿음
  더 부드러움
  하지만 늦게 따라감

값을 줄이면:
  raw 측정값을 더 믿음
  반응 빠름
  하지만 노이즈도 더 따라감
```

### `target_predict_timeout`

마커를 잃어버린 후 Kalman prediction 값을 계속 발행할 시간입니다.

```text
10.0이면:
  검출 실패 후 10초 동안 predicted x,y 발행
  10초 초과 시 NaN 발행
```

---

## 16. 실행 중 parameter 변경

다음 명령으로 실행 중 값을 바꿀 수 있습니다.

```bash
ros2 param set /marker_recognition target_kf_measurement_var 0.12
ros2 param set /marker_recognition target_kf_process_var 0.02
ros2 param set /marker_recognition target_predict_timeout 3.0
```

### 주의

`target_kf_measurement_var` 또는 `target_kf_process_var`를 바꾸면 Kalman Filter 객체가 새로 만들어집니다.

즉, 기존 추정 상태는 리셋되고 다음 검출부터 다시 초기화됩니다.

---

## 17. 추천 시작값

초기 테스트는 다음 값을 추천합니다.

```bash
-p target_kf_process_var:=0.01
-p target_kf_measurement_var:=0.08
-p target_predict_timeout:=3.0
```

실기체에서는 `target_predict_timeout=10.0`이 너무 길 수 있습니다.  
처음에는 2~3초로 시작하고, 시스템이 안정적인지 확인한 뒤 늘리는 것이 안전합니다.

---

## 18. 튜닝 방법

### raw 좌표가 너무 많이 튈 때

```bash
ros2 param set /marker_recognition target_kf_measurement_var 0.12
```

또는 더 부드럽게:

```bash
ros2 param set /marker_recognition target_kf_measurement_var 0.15
```

### 반응이 너무 느릴 때

```bash
ros2 param set /marker_recognition target_kf_measurement_var 0.04
```

또는 모델 반응성을 올리려면:

```bash
ros2 param set /marker_recognition target_kf_process_var 0.02
```

### 마커 lost 후 prediction이 너무 오래 유지될 때

```bash
ros2 param set /marker_recognition target_predict_timeout 2.0
```

---

## 19. 랜딩 제어 노드와의 관계

랜딩 제어 노드는 `/landing/coordinates`만 보면 됩니다.

```text
검출 성공:
  filtered x,y 수신

검출 실패 10초 이내:
  predicted x,y 수신

검출 실패 10초 초과:
  NaN 수신
```

따라서 C++ 랜딩 제어 노드에서는 기존처럼:

```cpp
valid_xy = isfinite(desired_x_) && isfinite(desired_y_)
```

로 valid 여부를 판단하면 됩니다.

---

## 20. 전체 요약

이 노드는 다음 기능을 수행합니다.

```text
1. SIYI A8 mini 영상 수신
2. ArUco 마커 검출
3. Lidar 고도 측정
4. 픽셀 오차를 m 단위 오차로 변환
5. Kalman Filter로 x,y 안정화
6. 검출 실패 시 일정 시간 prediction 유지
7. 오래 실패하면 NaN 발행
8. HUD 영상 발행
9. PX4에 DistanceSensor 및 짐벌 명령 발행
10. 고도에 따른 자동 줌 제어
```

핵심은 다음입니다.

```text
초록 십자가 = 실제 detect 위치
빨간 십자가 = Kalman prediction 위치
raw = 카메라 기반 원시 m 오차
pub = 실제 랜딩 제어 노드로 나가는 값
```

