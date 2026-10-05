import os
import cv2
import urllib.request
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from fastapi import FastAPI
from fastapi.responses import StreamingResponse, FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn
import threading

app = FastAPI()
WEB_DIR = os.path.dirname(os.path.abspath(__file__))
app.mount("/static", StaticFiles(directory=WEB_DIR), name="static")

class TeleopBridge(Node):
    def __init__(self):
        super().__init__('web_teleop_bridge')
        self.drive_pub = self.create_publisher(String, '/bulldozer/drive_cmd', 10)
        self.arm_pub = self.create_publisher(String, '/bulldozer/arm_cmd', 10)

    def send_drive(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.drive_pub.publish(msg)

    def send_arm(self, cmd: str):
        msg = String()
        msg.data = cmd
        self.arm_pub.publish(msg)

rclpy.init()
bridge = TeleopBridge()
threading.Thread(target=lambda: rclpy.spin(bridge), daemon=True).start()

PHONE_STREAM_URL = "http://10.42.0.50:8081/video"
ESP_CAM_URL      = "http://10.42.0.60/"

def stream_mjpeg_safe(url, rotate_ccw=False):
    """Safely reads MJPEG HTTP bytes without FFmpeg double-free crashes."""
    while True:
        try:
            stream = urllib.request.urlopen(url, timeout=3)
            bytes_data = b''
            while True:
                bytes_data += stream.read(4096)
                a = bytes_data.find(b'\xff\xd8') # JPEG Start
                b = bytes_data.find(b'\xff\xd9') # JPEG End
                if a != -1 and b != -1:
                    jpg = bytes_data[a:b+2]
                    bytes_data = bytes_data[b+2:]
                    
                    if rotate_ccw:
                        frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        if frame is not None:
                            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
                            _, encoded = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                            jpg = encoded.tobytes()

                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')
        except Exception:
            # Reconnect automatically without killing the process
            continue

@app.get('/video_feed')
def video_feed():
    return StreamingResponse(stream_mjpeg_safe(PHONE_STREAM_URL, rotate_ccw=True), 
                             media_type='multipart/x-mixed-replace; boundary=frame')

@app.get('/cam2_feed')
def cam2_feed():
    return StreamingResponse(stream_mjpeg_safe(ESP_CAM_URL, rotate_ccw=False), 
                             media_type='multipart/x-mixed-replace; boundary=frame')

@app.get('/cmd/drive/{cmd}')
def drive_cmd(cmd: str):
    bridge.send_drive(cmd)
    return {"status": "ok", "cmd": cmd}

@app.get('/cmd/arm/{cmd}')
def arm_cmd(cmd: str):
    bridge.send_arm(cmd)
    return {"status": "ok", "cmd": cmd}

@app.get('/')
def serve_index():
    return FileResponse(os.path.join(WEB_DIR, "index.html"))

if __name__ == '__main__':
    uvicorn.run(app, host="0.0.0.0", port=5000, log_level="warning")