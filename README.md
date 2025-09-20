# DYROS Robot Controller Package
 ![mecanum_video](images/mecanum_motion.gif)


이 패키지는 여러 형태의 로봇 구동에 필요한 제어 알고리즘을 구현한 것이다. 
지원되는 로봇 형태:
 - 바퀴 기반 모바일 로봇: Differential Wheel, Mecanum Wheel, Active Caster Wheel
 - 단일 매니퓰레이터 로봇: 위치, 속도, 토크 제어 모드 기반의 매니퓰레이터
 - 바퀴 기반의 모바일 단일 매니퓰레이터

각 로봇 제어기에 대한 설명
robotdata, robotcontroller 함수 설명
필요한 파일? URDF, SRDF 같은거
cpp, python 둘 다 지원
사용 예시


# Requirement
- [ROS2 humble](https://docs.ros.org/en/humble/index.html)
- [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)
- [OSQP](https://osqp.org/docs/get_started/sources.html)
- [OSQP-Eigen](https://github.com/robotology/osqp-eigen)

 # Installation
 ```bash
cd ros2_ws
git clone https://github.com/JunHeonYoon/dyros_robot_controller.git src
colcon build --symlink-install
source install/setup.bash
```



# Usage
In [macro](wheel_code_gen.py) code, there are some parameters that can be customized according to user settings.
  1. link_name: link name of wheel
  2. n_roller: number of rollers of wheel
  3. pos: position of wheel wrt body frame
  4. mass: mass of a wheel
  5. diag_inertia: diagonal inertia of a wheel [ixx, iyy, izz]
  6. size: size of a wheel [radius, height]
  7. type: type of a wheel (1 or 2)

```bash
python3 wheel_code_gen.py --link_name mecanum 
```

After running this code, you can get `wheel.xml`. Using this, you can implement collision model of mecanum wheel.


# Example
SUMMIT XL STEEL model was used to implement mecanum wheel.

In this [code](robots/summit_xl_description/assets/summit_xls.urdf.xml), you can see how to use.

And you can simmulate this model.
```
python3 summit_test.py
```


# Notice
For using this model, you can not use velocity actuator like below.
```
<actuator>
  <velocity name="mecanum_wheel_joint" .../>
</actuator>
```
Rather than using velocity, use force based actuator.
```
<actuator>
  <motor name="mecanum_wheel_joint" .../>
</actuator>
```
And in controller code, you can control wheel by PID controller like this [code](summit_test.py).

# License
`mujoco mecanum` is releasd under MIT license. Please see the [LICENSE](LICENSE) file for more information.
