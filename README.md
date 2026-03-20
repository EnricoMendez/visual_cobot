# visual_cobot

Paquete ROS 2 en Python para reconocimiento de gestos de mano con MediaPipe y control de un UFACTORY Lite6.

## Contenido

- `visual_cobot/gesture_recognition.py`: detecta gestos de mano desde una imagen y publica resultados en ROS 2.
- `visual_cobot/visual_control.py`: recibe gestos y ejecuta acciones sobre el gripper y el robot.
- `visual_cobot/visual_control_sim.py`: recibe gestos y se conecta al Lite6 simulado en Gazebo.
- `launch/gesture_recognition.launch.py`: launch para el nodo de reconocimiento.
- `config/gesture_recognizer.task`: modelo de MediaPipe usado por el reconocedor.

## Dependencias

- ROS 2
- `rclpy`
- `cv_bridge`
- `sensor_msgs`
- `std_msgs`
- `control_msgs`
- `trajectory_msgs`
- `ament_index_python`
- `opencv-python`
- `mediapipe`
- `xarm_msgs`

## Build

Desde tu workspace:

```bash
colcon build --packages-select visual_cobot
source install/setup.bash
```

## Ejecucion

Lanzar reconocimiento de gestos:

```bash
ros2 launch visual_cobot gesture_recognition.launch.py
```

Ejecutar control del robot:

```bash
ros2 run visual_cobot visual_control
```

Ejecutar control para simulacion en Gazebo:

```bash
ros2 launch xarm_gazebo lite6_beside_table_gazebo.launch.py
ros2 run visual_cobot visual_control_sim --ros-args -p use_sim_time:=true
```

O con un solo launch:

```bash
ros2 launch visual_cobot visual_control_sim.launch.py
```

Nota:

- `visual_control_sim` usa `/lite6_traj_controller/follow_joint_trajectory`.
- En `xarm_ros2`, el gripper del `lite6` no queda expuesto en Gazebo, asi que los gestos de abrir/cerrar se registran como `no-op`.

## Topics

- Publica gesto en `/hand/gesture`
- Publica imagen anotada en `/hand/annotated_image`
- Publica posicion de mano en `/hand/x`, `/hand/y` y `/hand/position`

## Licencia

Apache-2.0
