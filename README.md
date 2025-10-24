# ROS2 Package for Bridging ROS2 and PPM Signal Encoder

The node subscribes to command topics (aileron/elevator/throttle/rudder/mode), packs them into PPM channel values, and streams them over serial to a microcontroller that generates the PPM waveform (verified with Arduino Nano ESP32). This repository contains source codes for Arduino Nano ESP32, the ROS2 node, and the associated launch file.

## Architecture


| Stage | Component                          | Interface              | Purpose                              |
|------:|------------------------------------|------------------------|--------------------------------------|
| 1     | ROS 2 topic `/uav/auto_joy` or `/uav/joy`      | Joy Msg  | AETR+Mode commands as a 5-elem joy msg |
| 2     | `ppm_bridge_node`                  | ROS 2 ↔︎ serial        | Map command indices → PPM μs values    |
| 3     | Serial link `/uav/joy_serial_status`                       | `/dev/ttyACM0` 115200  | Send packed channel bytes            |
| 4     | Arduino Nano ESP32 (PPM encoder)   | PPM output pin         | Generate PPM frame at 22ms frame length                  |
| 5     | External TX (Lemon RX DSM module)  | Trainer/PPM input      | RF to receiver on UAV           |


## Requirements

This has been tested with the following setup and system requirements:
* ROS2 Jazzy and Ubuntu 24.04
* rclcpp (C++) and standard message types (std_msgs)
* Arduino Nano ESP32
* USB serial link between autopilot computer and Arduino Nano ESP32
* USB serial link between autopilot computer and external joystick

## Running the Bridge
To Build this in your ROS2 Workspace
```
cd ~/ros2_ws/src
git clone https://github.com/wsribunma/ppm_bridge.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ppm_bridge
source install/setup.bash
```

Running the code
```
ros2 launch ppm_bridge cub_vehicle_bridge.launch.py
```

## Topics & Message Format

| Topic name        | Message type               | Array size | Element order (index)                      | Accepted ranges / format                                                                 | Notes                                                                                              |
|-------------------|----------------------------|------------|--------------------------------------------|-------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------|
| `/uav/auto_joy`   | `sensor_msgs/msg/Joy` | 5          | `[0]=Aileron, [1]=Elevator, [2]=Throttle, [3]=Rudder, [4]=Mode` | Aileron/Elevator/Rudder: `[-1.0, +1.0]` • Throttle: `[0.0, 1.0]` • Mode: `{-1.0, 1.0}` | Default PPM order **AETR + Mode**. Values are linearly mapped to PPM μs (see index mapping below). |
| `/uav/joy`   | `sensor_msgs/msg/Joy` | 5          | `[0]=Aileron, [1]=Elevator, [2]=Throttle, [3]=Rudder, [4]=Mode` | Aileron/Elevator/Rudder: `[-1.0, +1.0]` • Throttle: `[0.0, 1.0]` • Mode: `{-1.0, 1.0}` | Default PPM order **AETR + Mode**. Values are linearly mapped to PPM μs (see index mapping below). |
| `/uav/joy_serial_status`   | `std_msgs/Float32MultiArray` | 5          | `[Aileron, Elevator, Throttle, Rudder, Mode]` | Aileron/Elevator/Throttle/Rudder: `[1000,2000]` | Set of 5 PWM signals to be parsed through serial port to Arduino|


## Index → PPM Mapping (AETR + Mode)

| Index | Channel   | Description                                        | ROS value → PPM (μs) mapping                            | Notes                                                                                 |
|------:|-----------|----------------------------------------------------|----------------------------------------------------------|---------------------------------------------------------------------------------------|
| 0     | Aileron   | Roll command                                       | `-1.0 → 1000`, `0.0 → 1500`, `+1.0 → 2000`               | Positive Joy (>1500 μs) = roll left; Negative Joy (<1500 μs>)= roll right                                           |
| 1     | Elevator  | Pitch command                                      | `-1.0 → 2000`, `0.0 → 1500`, `+1.0 → 1000`               | Positive Joy (<1500 μs) = Elevator up (pitch down); Negative Joy (>1500 μs) = Elevator down (pitch up) |
| 2     | Throttle  | Throttle                                           | `0.0 → 1000`, `1.0 → 2000`                               | Zero Joy (1000 μs) = Throttle idle; Joy 1.0 (2000 μs) = Full Throttle                                                             |
| 3     | Rudder    | Yaw command                                        | `-1.0 → 1000`, `0.0 → 1500`, `+1.0 → 2000`               | Positive Joy (>1500 μs)= yaw left; Nagative Joy (<1500 μs) = yaw right                                                                 |
| 4     | Mode      | Flight mode (discrete)                             | `-1.0 → 1000` (Manual), `1.0 → 2000` (Stabilize)          | Values are treated as discrete states: 1000 = Manual, 2000 = Stabilized|

* Note: The inputs from AETR + Mode are remapped and serialized to TAER + Mode format for Lemon RX DSM Transmitter
