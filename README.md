# Digital Twin Visualization

현장 상황을 **직관적으로 모니터링**하고 **명령 전달/이해**를 돕기 위한 Digital Twin 시각화 모듈입니다.  
다수 드론과 사람(탐지 대상·실종자)을 3D 공간에 표현하고, **좌표계 정합(NED↔시뮬레이터 좌표계)**, **탐지/신호 융합**, **시각화** 를 제공합니다.

이 브랜치는 **시각화 중심 구성**입니다.

---

## 핵심 기능

- **단일 좌표계 정합**
  - RTK-GPS/기준점으로 모든 드론의 **NED 좌표계**를 공통 원점에 일치시켜 시뮬레이터(UE)와 일관되게 매핑
  - 시뮬레이터 요구 좌표계(대개 ENU)로 **NED → 시뮬레이터 좌표 변환** 제공

- **실종자 신호 융합 강조 표시**
  - 탐지된 사람 후보와 실제 조난/신호(UWB-Tag) 위치를 비교하여 **근접 후보 강조(색상/라벨)**

- **멀티 드론/멀티 타깃 시각화**
  - 드론 정보, 타겟 위치, 감지된 객체 위치 등을 3D로 가시화

---

## 시스템 구성

~~~text
[드론 (NED, 센서, 카메라)]
         │ + 스폰 객체 정보 및 드론 세부 정보 (YAML)
         ▼
[변환·추론 노드]
 - NED ↔ ENU 좌표 변환
 - 후보-신호 근접도 계산
         │ Position/Poses (소켓 통신)
         ▼
[시뮬레이터 뷰어(UE)]
 - UE 스폰/이동
 - 색상/라벨 강조
~~~ 

## 실행 예시

~~~bash
ros2 launch realgazebo uwb_realgazebo
~~~

---

## ROS 2 인터페이스 (초기값)

**Subscriptions**
- `/detections_posearray` — 탐지된 객체의 NED 좌표 
- `/drone/manager/out/actuator_motors` — 드론 모터 속도
- `/drone/manager/out/monitoring` — 드론 정보
- `/drone/jfi/in/target` — UWB Tag 위치 추정 정보

---

## 주요 파라미터 (초기값)

- `position_tolerance` *(double)*: UWB 태그 기준 객체 거리
- `unreal_ip, unreal_port` *(int)*: Unreal Engine IP 및 Port

---