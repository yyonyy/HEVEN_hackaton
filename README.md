# HEVEN_hackaton

### 2024 HEVEN 자율주행 해커톤
- 일정 : 2024-11-22(금) 17:00 ~ 2024-11-22(토) 10:00
- 장소 : 성균관대학교 산학협력관 러닝팩토리 (85133호)
- 인원 : 약 30명 내외 (5~6인 1팀 구성)
- 내용 : Robot, Simulation상에서의 자율주행 알고리즘 구현

## 로봇 launch ( 파란색 라이다의 경우 실행하지 않아도 됨.)
```bash
roslaunch omo_r1mini_bringup omo_r1mini_bringup.launch
```

### 카메라

```bash
roslaunch jetson_camera jetson_camera.launch
```

### Lidar 및 모터

-검은색 Lidar

```bash
roslaunch omo_r1mini_bringup omo_r1mini_robot_x4pro.launch
```

-파란색 Lidar 

```bash
roslaunch omo_r1mini_bringup omo_r1mini_robot.launch
```


## PC launch

### map

```bash
roslaunch omo_hackathon start.launch
```

### 자율주행코드

```bash
rosrun omo_hackathon omo.py
```

## 가상환경 Setting
### 1. Docker를 이용한 개발 환경 구성 (Recommended)

Windows 환경에서도 다음의 링크를 참고하여 편하게 개발할 수 있습니다.
[Docker 환경 설치하기](https://github.com/yyonyy/HEVEN_hackaton/blob/main/Simulation/InstallDocker.md)

### 2. Ubuntu에 패키지 직접 설치

* Ubuntu 20.04 멀티부팅 설치

   https://carrido-hobbies-well-being.tistory.com/84

* ROS 설치 (상단의 *"noetic"* 클릭 후 진행)

   http://wiki.ros.org/Installation/Ubuntu

* Dependencies 설치

    ```
    sudo apt-get install ros-noetic-tf2-geometry-msgs ros-noetic-ackermann-msgs ros-noetic-joy ros-noetic-map-server
    ```

* ROS용 워크스페이스 생성

    ```
    mkdir catkin_ws && cd catkin_ws
    mkdir src && cd src
    ```
    
* 레포지토리 복제

    ```
    git clone https://github.com/yyonyy/HEVEN_hackaton.git
    ```

* 패키지 빌드

    ```
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## 최신 버전 업데이트

* 아래 명령어를 실행하여, "Already up to date." 라는 문구가 떠야 최신 버전임
    ```
    cd ~/catkin_ws/src/HEVEN_hackaton/
    git pull
    ```

## 실행

* 시뮬레이터 실행
    ```
    cd ~/catkin_ws/src/HEVEN_hackaton/Simulation
    bash simulate.sh <주차 구역 number>
    ```
    
* 자율주행 알고리즘 (brain) 실행
    ```
    cd ~/catkin_ws/src/HEVEN_hackaton/Simulation
    bash brain.sh
    ```

* 수동 조작 노드 실행 (시뮬레이터 실행 후)
    ```
    cd ~/catkin_ws/src/HEVEN_hackaton/Simulation
    bash joystick.sh
    ```
    
