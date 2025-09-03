# Aerial Image-to-Position (YOLO) — `yolo` branch

다중 드론에서 수집한 항공 영상을 **YOLO 탐지 → 3D 위치(NED 좌표)** 로 변환하는 ROS 2 패키지입니다.  
이 브랜치는 카메라 기반 탐지와 위치 산출 파이프라인에 초점이 맞춰져 있습니다. 저장소는 「제23회 임베디드 소프트웨어 경진대회 찾아줘 드론」 소스의 **YOLO 파트**를 분리·정리한 형태입니다.

---

## 핵심 아이디어: Aerial Image → Position

1. **지면 카메라 영상 수집 & 사람 중심 픽셀 탐지**  
   - YOLO(v8 권장)로 사람/관심 객체의 2D 바운딩 박스를 검출하고 **중심 픽셀 (u, v)** 을 얻습니다.

2. **카메라 원점에서 사람 방향 벡터 계산**  
   - `sensor_msgs/CameraInfo`(K, D)로 정규화 좌표 `(x, y, 1)`를 만들고 **카메라 좌표계 광선** `r_cam` 을 계산합니다.

3. **카메라 → 드론 좌표 변환(외부파라미터/마운트 각 반영)**  
   - 고정 보정행렬 **`R_b_cam`** 으로 `r_body = R_b_cam * r_cam` 을 계산합니다.

4. **드론 자세(RPY/특히 Yaw) 반영하여 NED 기준 방향 벡터 계산**  
   - 드론 자세로부터 **`R_n_b`** 를 만들고 `r_ned = R_n_b * r_body` 를 구합니다.

5. **광선–지면 교차로 최종 NED 좌표 산출**  
   - 드론 위치 `C_ned` 과 지면 고도(또는 DEM)를 사용해 `P_ned = C_ned + t * r_ned` 의 **교점**을 구합니다.  
   - `t` 는 광선이 **지면 높이**에 도달할 때의 스칼라입니다.

---

## 구성 요소(주요 노드)

### `yolov8_ros2_pt.py`
Ultralytics YOLOv8 추론 노드. 카메라 이미지를 구독해 `yolov8_msgs/msg/Yolov8Inference` 를 퍼블리시합니다.  

### `box2pos_delay.py`
픽셀 중심점과 CameraInfo, 드론 자세/위치를 받아 **픽셀 → 지면 3D 점(NED)** 으로 변환하고, Marker/Pose를 퍼블리시합니다.  

### `compressed_to_raw.py`
Compressed Image를 받아 Image Raw로 변환하여 퍼블리시합니다.

---

## ROS 2 주요 Topic(기본값 예시)

**Subscriptions**
### `yolov8_ros2_pt.py`
- `/recon_1/camera/image_raw` — 이미지 

### `box2pos_delay.py`
- `/drone1/fmu/out/monitoring` — 드론 정보
- `/Yolov8_Inference` — YOLO Bounding Box 정보

### `compressed_to_raw.py`
- `/compressed_image_topic` — 압축된 이미지

**Publications**
### `yolov8_ros2_pt.py`
- `/Yolov8_Inference` — YOLO Bounding Box 정보
- `/inference_result_1` — YOLO Bounding Box가 포함된 이미지

### `box2pos_delay.py`
- `/detections_posearray` — 이미지내 검출된 객체들의 계산된 NED 좌표

### `compressed_to_raw.py`
- `/recon_1/camera/image_raw` — 이미지 

---

## 실행 예시
~~~bash
# 픽셀 → NED 변환/시각화 노드
ros2 run yolo compressed_to_raw
~~~

~~~bash
# YOLO 추론 노드 (GPU, 사람 클래스만)
ros2 run yolobot_recognition yolov8_ros2_pt.py
~~~

~~~bash
# 픽셀 → NED 변환/시각화 노드
ros2 run box2pos box2pos_delay
~~~


---

## 주요 파라미터

**YOLO (`yolov8_ros2_pt.py`)**
- `conf` *(float)* — 탐지 신뢰도 임계값

**변환/시각화 (`box2pos_delay.py`)**
- `detection_delay_sec` *(float)* — 드론 정보와 YOLO 이미지 간 오차 Delay  
- `fx, fy` *(float)* — 카메라 초점
- `cx, cy` *(float)* — 카메라 주점
---
