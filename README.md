# ClickMove-Robotics


This project integrates a graphical user interface (GUI) with an inertial measurement unit (IMU) sensor to provide dual-mode control of a robotic arm. Users can choose to control the robotic arm by clicking on cursor buttons in the GUI or by using the IMU sensor to move the cursor for button selection. 

## Key features:
- **Dual Control Modes**: Users can operate the robotic arm using either cursor buttons in the GUI or by physically moving an IMU sensor, offering flexibility and ease of use.
- **Intuitive GUI**: A user-friendly interface with clearly labeled buttons for different robotic arm actions, enabling quick and straightforward control.
- **Real-Time Feedback**: The GUI provides real-time visual feedback on the robotic arm's status and position, allowing users to see the effects of their inputs immediately.
- **IMU Integration**: The IMU sensor accurately detects movements, translating them into cursor movements for intuitive control without the need for manual clicking.

## Demo
![Description of GIF](https://github.com/ali-rabiee/ClickMove-Robotics/blob/main/demo/demo.gif?raw=true)

```bash
git clone https://github.com/ali-rabiee/ClickMove-Robotics.git
cd ClickMove-Robotics

```
## Control via cursor
```bash
python3 jaco3D_1.py
```
## Control via IMU
First, upload the IMU_Mouse_Arduino_Control_Position_Sys_V6.ino into an Arduino board with an IMU sensor and then:
```bash
python3 v6.py
```
