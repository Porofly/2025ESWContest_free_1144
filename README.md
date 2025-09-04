# Camera (YOLO)

## 소개
* 드론에서 촬영한 영상에서 YOLO로 사람을 검출한 뒤, 사람의 위치(NED 좌표)를 계산하는 데 사용됩니다.

### 주요 특징

* 드론 카메라 영상에서 사람 검출
* 사람의 위치 좌표 계산 (픽셀 → NED)

## How to Run

~~~bash
# YOLO
ros2 run yolobot_recognition yolov8_ros2_pt.py
~~~

~~~bash
# 좌표 생성(이미지 픽셀 → NED 변환)
ros2 run box2pos box2pos_delay
~~~
