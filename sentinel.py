from flask import Flask, Response, jsonify
from picamera2 import Picamera2
import cv2
import RPi.GPIO as GPIO
import time
import threading
import logging
import socket
import sys
import os

# ─── LOGGING ──────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s: %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("/home/nishad/sentinelbot.log")
    ]
)
log = logging.getLogger("SentinelBot")

app = Flask(__name__)
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# ─── MOTORS ───────────────────────────────────
IN1, IN2, IN3, IN4 = 17, 18, 27, 22
ENA, ENB = 12, 13

try:
    GPIO.setup([IN1, IN2, IN3, IN4, ENA, ENB], GPIO.OUT)
    pA = GPIO.PWM(ENA, 100)
    pB = GPIO.PWM(ENB, 100)
    pA.start(70)
    pB.start(70)
    log.info("Motors OK")
except Exception as e:
    log.error("Motor init failed: " + str(e))
    log.error("FIX: Check L298N wiring and ENA/ENB jumpers removed")
    sys.exit(1)

def forward():
    GPIO.output(IN1, 1); GPIO.output(IN2, 0)
    GPIO.output(IN3, 1); GPIO.output(IN4, 0)

def backward():
    GPIO.output(IN1, 0); GPIO.output(IN2, 1)
    GPIO.output(IN3, 0); GPIO.output(IN4, 1)

def left():
    GPIO.output(IN1, 0); GPIO.output(IN2, 1)
    GPIO.output(IN3, 1); GPIO.output(IN4, 0)

def right():
    GPIO.output(IN1, 1); GPIO.output(IN2, 0)
    GPIO.output(IN3, 0); GPIO.output(IN4, 1)

def stop():
    GPIO.output(IN1, 0); GPIO.output(IN2, 0)
    GPIO.output(IN3, 0); GPIO.output(IN4, 0)

# ─── SERVOS ───────────────────────────────────
PAN_PIN  = 23
TILT_PIN = 24
servo_lock = threading.Lock()

try:
    GPIO.setup(PAN_PIN,  GPIO.OUT)
    GPIO.setup(TILT_PIN, GPIO.OUT)
    pan_pwm  = GPIO.PWM(PAN_PIN,  50)
    tilt_pwm = GPIO.PWM(TILT_PIN, 50)
    pan_pwm.start(7.5)
    tilt_pwm.start(7.5)
    time.sleep(1)
    pan_pwm.ChangeDutyCycle(0)
    tilt_pwm.ChangeDutyCycle(0)
    log.info("Servos OK")
except Exception as e:
    log.error("Servo init failed: " + str(e))
    log.error("FIX: Check GPIO 23 and GPIO 24 signal wires")
    sys.exit(1)

# ─── SERVO SMOOTHING ──────────────────────────
pan_angle    = 90.0
tilt_angle   = 90.0
pan_target   = 90.0
tilt_target  = 90.0
PAN_MIN      = 45
PAN_MAX      = 135
TILT_MIN     = 65
TILT_MAX     = 115
SMOOTH       = 0.15   # 0.0 = no smoothing, 1.0 = instant
KP           = 0.04

def set_servo_raw(pwm, angle):
    angle = max(0, min(180, angle))
    duty = 2.5 + (angle / 180.0) * 10.0
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.03)
    pwm.ChangeDutyCycle(0)

def smooth_servo_thread():
    global pan_angle, tilt_angle
    while True:
        try:
            with servo_lock:
                # Lerp toward target smoothly
                pan_angle  += (pan_target  - pan_angle)  * SMOOTH
                tilt_angle += (tilt_target - tilt_angle) * SMOOTH
                pan_clamped  = max(PAN_MIN,  min(PAN_MAX,  pan_angle))
                tilt_clamped = max(TILT_MIN, min(TILT_MAX, tilt_angle))
                set_servo_raw(pan_pwm,  pan_clamped)
                set_servo_raw(tilt_pwm, tilt_clamped)
        except Exception as e:
            log.error("Smooth servo error: " + str(e))
        time.sleep(0.04)

threading.Thread(target=smooth_servo_thread, daemon=True).start()

# ─── STATE ────────────────────────────────────
face_detected  = False
patrol_active  = True
state = {
    "faces":  0,
    "mode":   "PATROLLING",
    "status": "MOVING",
    "patrol": True,
    "error":  ""
}

# ─── PATROL THREAD ────────────────────────────
def patrol_loop():
    global pan_target, face_detected, patrol_active
    direction = 1
    log.info("Patrol started")
    while True:
        try:
            if patrol_active and not face_detected:
                pan_target += direction * 1.5
                if pan_target >= PAN_MAX:
                    pan_target = PAN_MAX
                    direction  = -1
                    right(); time.sleep(0.3); stop()
                elif pan_target <= PAN_MIN:
                    pan_target = PAN_MIN
                    direction  = 1
                    left();  time.sleep(0.3); stop()
                else:
                    forward()
                    state["status"] = "PATROLLING"
                time.sleep(0.06)
            elif not patrol_active and not face_detected:
                stop()
                state["status"] = "STANDBY"
                time.sleep(0.1)
            else:
                stop()
                state["status"] = "LOCKED ON"
                time.sleep(0.1)
        except Exception as e:
            log.error("Patrol error: " + str(e))
            time.sleep(0.5)

threading.Thread(target=patrol_loop, daemon=True).start()

# ─── FACE DETECTION ───────────────────────────
CASCADE_PATHS = [
    cv2.data.haarcascades + "haarcascade_frontalface_default.xml",
    "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml",
    "/usr/share/opencv/haarcascades/haarcascade_frontalface_default.xml",
]

cascade = None
for path in CASCADE_PATHS:
    if os.path.exists(path):
        cascade = cv2.CascadeClassifier(path)
        log.info("Cascade loaded: " + path)
        break

if cascade is None:
    log.error("Cascade XML not found!")
    log.error("FIX: run -> find / -name haarcascade_frontalface_default.xml 2>/dev/null")
    log.error("Add that path to CASCADE_PATHS in the code")
    sys.exit(1)

# ─── CAMERA ───────────────────────────────────
try:
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
    picam2.start()
    time.sleep(2)
    log.info("Camera OK")
except Exception as e:
    log.error("Camera failed: " + str(e))
    log.error("FIX: Reseat ribbon cable — blue side faces HDMI ports")
    sys.exit(1)

# ─── VIDEO STREAM ─────────────────────────────
def generate_frames():
    global pan_target, tilt_target, face_detected
    consecutive_errors = 0

    while True:
        try:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # ── FLIP CAMERA (upside down mount) ──
            frame = cv2.flip(frame, -1)  # -1 = flip both axes

            gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = cascade.detectMultiScale(gray, 1.1, 5, minSize=(50, 50))
            state["faces"] = len(faces)
            consecutive_errors = 0

            if len(faces) > 0:
                face_detected = True
                state["mode"] = "LOCKED ON"

                x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
                fx = x + w // 2
                fy = y + h // 2

                err_x = fx - 320
                err_y = fy - 240

                with servo_lock:
                    if abs(err_x) > 12:
                        pan_target -= err_x * KP
                        pan_target  = max(PAN_MIN, min(PAN_MAX, pan_target))
                    if abs(err_y) > 12:
                        tilt_target += err_y * KP
                        tilt_target  = max(TILT_MIN, min(TILT_MAX, tilt_target))

                # Draw tracking UI
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 80), 2)
                cv2.circle(frame, (fx, fy), 5, (0, 255, 80), -1)
                cv2.line(frame, (320, 240), (fx, fy), (0, 220, 255), 1)

                # Corner brackets instead of full rectangle
                br = 18
                for cx, cy, sx, sy in [(x,y,1,1),(x+w,y,-1,1),(x,y+h,1,-1),(x+w,y+h,-1,-1)]:
                    cv2.line(frame,(cx,cy),(cx+sx*br,cy),(0,255,80),2)
                    cv2.line(frame,(cx,cy),(cx,cy+sy*br),(0,255,80),2)

            else:
                face_detected = False
                state["mode"] = "PATROLLING" if patrol_active else "STANDBY"

            # ── HUD OVERLAY ──
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (640, 100), (0, 0, 0), -1)
            frame = cv2.addWeighted(overlay, 0.4, frame, 0.6, 0)

            # Crosshair
            cv2.line(frame, (310, 240), (330, 240), (255, 255, 255), 1)
            cv2.line(frame, (320, 230), (320, 250), (255, 255, 255), 1)
            cv2.circle(frame, (320, 240), 20, (255, 255, 255), 1)

            mode_color = (0, 255, 80) if face_detected else (0, 160, 255)
            cv2.putText(frame, state["mode"], (12, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, mode_color, 2)
            cv2.putText(frame, "FACES: " + str(len(faces)), (12, 62),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 220, 0), 1)
            cv2.putText(frame, "PAN:"  + str(round(pan_angle,  1)), (510, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 255), 1)
            cv2.putText(frame, "TILT:" + str(round(tilt_angle, 1)), (510, 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (100, 100, 255), 1)

            # Status dot
            dot_color = (0, 255, 80) if face_detected else (0, 160, 255)
            cv2.circle(frame, (620, 20), 7, dot_color, -1)

            ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ret:
                continue
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n")

        except Exception as e:
            consecutive_errors += 1
            log.error("Frame error: " + str(e))
            if consecutive_errors > 10:
                state["error"] = "Camera error — reseat ribbon cable"
                log.error("FIX: Power off Pi, reseat camera ribbon cable, restart")
            time.sleep(0.1)

# ─── WIFI CHECK ───────────────────────────────
def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except:
        return None

# ─── WEB DASHBOARD ────────────────────────────
PAGE = """<!DOCTYPE html>
<html>
<head>
<title>SentinelBot</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
@import url('https://fonts.googleapis.com/css2?family=Rajdhani:wght@400;600;700&family=Share+Tech+Mono&display=swap');
*{box-sizing:border-box;margin:0;padding:0}
:root{
  --green:#00ff88;--orange:#ff8c00;--blue:#00aaff;
  --red:#ff3344;--bg:#080c10;--card:#0d1117;--border:#1a2030
}
body{background:var(--bg);color:white;font-family:'Rajdhani',sans-serif;
     text-align:center;min-height:100vh;overflow-x:hidden}

/* Header */
.header{padding:14px 10px 6px;position:relative}
.header h1{font-size:24px;font-weight:700;letter-spacing:4px;color:var(--green);
           text-shadow:0 0 20px rgba(0,255,136,0.4)}
.header p{font-size:10px;letter-spacing:3px;color:#445;margin-top:2px}
.live-pill{display:inline-flex;align-items:center;gap:5px;background:#1a0010;
           border:1px solid var(--red);color:var(--red);font-size:10px;
           padding:3px 10px;border-radius:20px;letter-spacing:2px;margin-top:6px}
.live-dot{width:6px;height:6px;background:var(--red);border-radius:50%;
          animation:blink 1s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:0.2}}

/* Video */
.video-wrap{position:relative;display:inline-block;width:100%;max-width:640px;
            margin-top:10px}
#stream{width:100%;display:block;border:1px solid #1a2030;border-radius:4px}
.scan-line{position:absolute;top:0;left:0;right:0;height:2px;
           background:linear-gradient(90deg,transparent,var(--green),transparent);
           animation:scan 3s linear infinite;pointer-events:none}
@keyframes scan{0%{top:0}100%{top:100%}}
.corner{position:absolute;width:16px;height:16px;border-color:var(--green);
        border-style:solid;pointer-events:none}
.tl{top:0;left:0;border-width:2px 0 0 2px}
.tr{top:0;right:0;border-width:2px 2px 0 0}
.bl{bottom:0;left:0;border-width:0 0 2px 2px}
.br{bottom:0;right:0;border-width:0 2px 2px 0}

/* Stats */
.stats{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;
       padding:10px 12px;max-width:640px;margin:0 auto}
.stat{background:var(--card);border:1px solid var(--border);
      border-radius:8px;padding:10px 8px}
.stat-label{font-size:9px;letter-spacing:2px;color:#445}
.stat-val{font-size:22px;font-weight:700;font-family:'Share Tech Mono',monospace;
          margin-top:2px;transition:color 0.3s}

/* Patrol toggle */
.patrol-wrap{padding:0 12px 8px;max-width:640px;margin:0 auto}
.patrol-btn{width:100%;padding:11px;font-family:'Rajdhani',sans-serif;
            font-size:14px;font-weight:700;letter-spacing:2px;border-radius:8px;
            cursor:pointer;transition:all 0.2s;border:1px solid}
.patrol-on{background:rgba(255,140,0,0.1);border-color:var(--orange);
           color:var(--orange)}
.patrol-on:active{background:var(--orange);color:#000}
.patrol-off{background:rgba(0,255,136,0.1);border-color:var(--green);
            color:var(--green)}
.patrol-off:active{background:var(--green);color:#000}

/* Controls */
.controls{padding:0 12px 20px;max-width:260px;margin:0 auto}
.row{display:flex;justify-content:center;gap:8px;margin:5px 0}
.btn{background:var(--card);color:var(--green);border:1px solid #1e2a1e;
     padding:16px;font-size:18px;border-radius:10px;cursor:pointer;
     min-width:62px;transition:all 0.15s;-webkit-tap-highlight-color:transparent;
     user-select:none;font-family:'Share Tech Mono',monospace}
.btn:active{background:var(--green);color:#000;border-color:var(--green);
            transform:scale(0.93)}
.stop-btn{background:rgba(255,51,68,0.1);border-color:#2a1015;
          color:var(--red);padding:16px 30px}
.stop-btn:active{background:var(--red);color:white;border-color:var(--red)}

/* Error */
.err{background:rgba(255,51,68,0.1);border:1px solid var(--red);
     color:#ff8899;font-size:12px;padding:8px 12px;margin:6px 12px;
     border-radius:6px;display:none;letter-spacing:1px}

/* Divider */
.div{border:none;border-top:1px solid var(--border);margin:8px 12px}
</style>
</head>
<body>
<div class="header">
  <h1>SENTINELBOT</h1>
  <p>AI FACE TRACKING SURVEILLANCE SYSTEM</p>
  <div class="live-pill"><span class="live-dot"></span>LIVE</div>
</div>

<div id="err" class="err"></div>

<div style="display:flex;justify-content:center;padding:0 12px">
  <div class="video-wrap">
    <img id="stream" src="/video" onerror="streamErr()">
    <div class="scan-line"></div>
    <div class="corner tl"></div>
    <div class="corner tr"></div>
    <div class="corner bl"></div>
    <div class="corner br"></div>
  </div>
</div>

<div class="stats">
  <div class="stat">
    <div class="stat-label">FACES</div>
    <div class="stat-val" id="f" style="color:var(--green)">0</div>
  </div>
  <div class="stat">
    <div class="stat-label">MODE</div>
    <div class="stat-val" id="m" style="color:var(--orange);font-size:13px;padding-top:5px">PATROLLING</div>
  </div>
  <div class="stat">
    <div class="stat-label">DRIVE</div>
    <div class="stat-val" id="s" style="color:var(--blue);font-size:13px;padding-top:5px">MOVING</div>
  </div>
</div>

<hr class="div">

<div class="patrol-wrap">
  <button id="pbtn" class="patrol-btn patrol-on" onclick="togglePatrol()">
    ■ STOP PATROL
  </button>
</div>

<div class="controls">
  <div class="row">
    <button class="btn" ontouchstart="cmd('forward')" ontouchend="cmd('stop')"
            onmousedown="cmd('forward')" onmouseup="cmd('stop')">▲</button>
  </div>
  <div class="row">
    <button class="btn" ontouchstart="cmd('left')" ontouchend="cmd('stop')"
            onmousedown="cmd('left')" onmouseup="cmd('stop')">◀</button>
    <button class="btn stop-btn" onclick="cmd('stop')">■</button>
    <button class="btn" ontouchstart="cmd('right')" ontouchend="cmd('stop')"
            onmousedown="cmd('right')" onmouseup="cmd('stop')">▶</button>
  </div>
  <div class="row">
    <button class="btn" ontouchstart="cmd('backward')" ontouchend="cmd('stop')"
            onmousedown="cmd('backward')" onmouseup="cmd('stop')">▼</button>
  </div>
</div>

<script>
var patrolOn=true;

function cmd(a){
  fetch('/cmd/'+a).catch(function(){showErr('Connection lost — check WiFi')});
  document.getElementById('s').innerText=a.toUpperCase();
}

function togglePatrol(){
  patrolOn=!patrolOn;
  fetch('/cmd/'+(patrolOn?'patrol_on':'patrol_off'))
    .catch(function(){showErr('Connection lost — check WiFi')});
  var b=document.getElementById('pbtn');
  if(patrolOn){
    b.innerText='■ STOP PATROL';
    b.className='patrol-btn patrol-on';
  } else {
    b.innerText='▶ START PATROL';
    b.className='patrol-btn patrol-off';
  }
}

function streamErr(){
  document.getElementById('stream').style.display='none';
  showErr('Stream lost — refresh page or check WiFi connection');
}

function showErr(msg){
  var e=document.getElementById('err');
  e.innerText='⚠  '+msg;
  e.style.display='block';
  clearTimeout(window._et);
  window._et=setTimeout(function(){e.style.display='none'},5000);
}

setInterval(function(){
  fetch('/stats').then(function(r){return r.json()}).then(function(d){
    document.getElementById('f').innerText=d.faces;
    var m=document.getElementById('m');
    m.innerText=d.mode;
    m.style.color=d.mode==='LOCKED ON'?'var(--green)':'var(--orange)';
    document.getElementById('s').innerText=d.status;
    if(d.error)showErr(d.error);
  }).catch(function(){showErr('Lost connection — are you on the same WiFi?')});
},900);
</script>
</body>
</html>"""

# ─── ROUTES ───────────────────────────────────
@app.route("/")
def index():
    return PAGE

@app.route("/video")
def video():
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/cmd/<action>")
def command(action):
    global patrol_active
    if   action == "forward":    forward();  state["status"] = "FORWARD"
    elif action == "backward":   backward(); state["status"] = "BACKWARD"
    elif action == "left":       left();     state["status"] = "LEFT"
    elif action == "right":      right();    state["status"] = "RIGHT"
    elif action == "stop":
        patrol_active = False
        stop()
        state["status"]  = "STOPPED"
        state["patrol"]  = False
    elif action == "patrol_on":
        patrol_active    = True
        state["patrol"]  = True
        state["status"]  = "PATROLLING"
    elif action == "patrol_off":
        patrol_active    = False
        stop()
        state["patrol"]  = False
        state["status"]  = "STANDBY"
    return jsonify({"ok": True})

@app.route("/stats")
def stats():
    return jsonify(state)

# ─── MAIN ─────────────────────────────────────
if __name__ == "__main__":
    ip = get_ip()
    if ip is None:
        log.warning("No WiFi found!")
        log.warning("FIX: sudo raspi-config -> System -> Wireless LAN")
    else:
        log.info("WiFi OK — IP: " + ip)
        log.info("Dashboard: http://" + ip + ":5000")

    log.info("SentinelBot starting — patrol begins automatically")
    stop()
    time.sleep(0.5)

    try:
        app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
    except OSError as e:
        if "Address already in use" in str(e):
            log.error("Port 5000 busy — FIX: sudo fuser -k 5000/tcp")
        else:
            log.error("Server error: " + str(e))
    except KeyboardInterrupt:
        log.info("Stopped by user")
    finally:
        stop()
        pA.stop(); pB.stop()
        pan_pwm.stop(); tilt_pwm.stop()
        GPIO.cleanup()
        log.info("Shutdown complete")
