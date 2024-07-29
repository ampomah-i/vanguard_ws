import sys
import os
from flask import Flask, Response
import threading
import rclpy

# Add the src directory to the Python path
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))

from vision.camera_subscriber import CameraSubscriber


app = Flask(__name__)

# Initialize ROS 2 and the subscriber node
rclpy.init(args=None)
node = CameraSubscriber()

@app.route('/camera')
def stream():
    def generate():
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            frame = node.get_merged_image()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n\r\n')
            else:
                yield (b'--frame\r\n'
                       b'Content-Type: text/plain\r\n\r\nNo merged image available\r\n\r\n')
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=True)
