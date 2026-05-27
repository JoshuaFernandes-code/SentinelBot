# SentinelBot
Autonomous AI face tracking robot built on Raspberry Pi 4.
Real-time face detection with pan-tilt camera tracking,
live web dashboard, and manual drive control.
Python |  OpenCV | Flask | RPi.GPIO 



An autonomous ground robot that patrols indoor 
environments, detects human faces in real time using 
computer vision, and physically tracks detected faces 
using a pan-tilt servo mechanism. Live annotated video 
streams to a web dashboard accessible from any phone.


# Features
- Real-time face detection using OpenCV Haar Cascade
- Pan-tilt camera physically tracks detected faces
- Autonomous patrol mode with left/right sweep
- Live MJPEG video stream via Flask web server
- Manual drive control from mobile browser
- 100% offline — no internet or cloud required
- Dual mode: PATROL and LOCKED ON


# Tech Stack
Python | OpenCV | Flask | Picamera2 | RPi.GPIO | 
Threading | HTML/CSS/JavaScript

# System Architecture
Pi Camera → OpenCV Detection → Proportional Controller 
→ Pan-Tilt Servos + Motor Control + Flask Dashboard

# How It Works
1. Robot powers on and begins autonomous patrol
2. Camera sweeps left/right continuously
3. OpenCV detects human faces each frame
4. Proportional controller calculates angular error
5. Pan/tilt servos correct to center the face
6. Live annotated stream sent to phone dashboard
7. Resumes patrol when face disappears

# Installation
```bash
git clone https://github.com/[yourname]/sentinelbot
cd sentinelbot
pip install -r requirements.txt
sudo python3 sentinel.py
```

# Results
- Real-time detection on embedded ARM hardware
- Pan-tilt tracking response under 100ms
- Live stream accessible on any device via hotspot
- Stable autonomous operation

# Future Work
- YOLOv8 upgrade for full body detection
- Face recognition for identity logging
- SLAM for autonomous mapping and navigation
- Obstacle avoidance integration
- WhatsApp/email alert on detection

# Built By
Joshua Joel Fernandes
Electronics and Computer Engineering
AIEM Goa | 2024-2028
