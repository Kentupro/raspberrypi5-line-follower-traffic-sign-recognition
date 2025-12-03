# Line Follower Robot with Traffic Sign Recognition (Raspberry Pi 5)

This repository contains the source code and documentation for my bachelor diploma project:  
**"Sistem mecatronic de tip line follower cu recunoaștere a semnelor de circulație bazat pe platforma Raspberry Pi"**.

The system is a small autonomous robot that follows a line using QTR reflectance sensors and interprets traffic signs using a YOLOv8 neural network running on a Raspberry Pi 5.

Features

- Autonomous **line following** using QTR sensors + PID control  
- Real-time **YOLOv8 traffic sign recognition** (71 classes)  
- **Raspberry Pi 5** + Pi Camera V2  
- Flask Web Interface for live streaming  
- Custom 3D-printed PLA chassis  
- 7.4V 3000 mAh Li-Po battery providing ~1 hour of autonomy

  System Architecture

 Hardware
- Raspberry Pi 5 (4GB RAM + 128GB SSD)  
- QTR reflectance sensor array  
- DC motors + L298N driver  
- Pi Camera V2  
- Step-down regulator  
- Li-Po 7.4V battery  
- Custom 3D printed chassis  

 Software
- Python 3  
- PID control for steering  
- QTR sensor processing  
- YOLOv8 inference  
- Flask Web UI  
- Image acquisition pipeline
