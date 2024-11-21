# HEVEN_hackaton

## 로봇 launch

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
