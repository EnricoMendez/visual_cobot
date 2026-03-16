# visual_cobot

Paquete ROS 2 en Python para reconocimiento de gestos de mano con MediaPipe y control de un UFACTORY Lite6.

## Contenido

- `visual_cobot/gesture_recognition.py`: detecta gestos de mano desde una imagen y publica resultados en ROS 2.
- `visual_cobot/visual_control.py`: recibe gestos y ejecuta acciones sobre el gripper y el robot.
- `launch/gesture_recognition.launch.py`: launch para el nodo de reconocimiento.
- `config/gesture_recognizer.task`: modelo de MediaPipe usado por el reconocedor.

## Dependencias

- ROS 2
- `rclpy`
- `cv_bridge`
- `sensor_msgs`
- `std_msgs`
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

## Topics

- Publica gesto en `/hand/gesture`
- Publica imagen anotada en `/hand/annotated_image`
- Publica posicion de mano en `/hand/x`, `/hand/y` y `/hand/position`

## Licencia

Apache-2.0
