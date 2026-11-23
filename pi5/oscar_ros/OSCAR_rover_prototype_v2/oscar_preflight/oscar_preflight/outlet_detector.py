"""Pi camera or video-file outlet detections; optional CV dependencies.

Publishes JSON detections with bounding boxes, no assumed physical distances.
Install OpenCV + Ultralytics separately; no motor movement is produced here.
"""
import json
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OutletDetector(Node):
    def __init__(self):
        super().__init__('oscar_outlet_detector')
        self.declare_parameter('model_path', '')
        self.declare_parameter('video_source', '0')
        self.declare_parameter('confidence', 0.55)
        self.declare_parameter('process_hz', 2.0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('allow_camera', False)
        if not self.get_parameter('allow_camera').value:
            raise RuntimeError('Camera disabled by default; enable only after selecting the correct camera and model')
        try:
            import cv2
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError('Camera inference requires cv2 and ultralytics in the active Python environment') from exc
        model_path = self.get_parameter('model_path').value
        if not model_path:
            raise RuntimeError('Set model_path to the checkpoint on THIS machine')
        self.cv2 = cv2
        self.model = YOLO(model_path)
        source = self.get_parameter('video_source').value
        self.cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open video_source={source}')
        self.pub = self.create_publisher(String, '/outlets/detections', 10)
        self.frame_seq = 0
        self.timer = self.create_timer(1.0 / max(0.2, float(self.get_parameter('process_hz').value)), self.run_frame)

    def run_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warn('Camera frame unavailable')
            return
        width, height = frame.shape[1], frame.shape[0]
        conf = float(self.get_parameter('confidence').value)
        results = self.model.predict(source=frame, conf=conf, imgsz=int(self.get_parameter('image_width').value), verbose=False)
        detections = []
        if results:
            result = results[0]
            if result.boxes is not None:
                for b in result.boxes:
                    x1, y1, x2, y2 = [float(i) for i in b.xyxy[0].tolist()]
                    detections.append({'class_id': int(b.cls.item()), 'confidence': float(b.conf.item()),
                                       'xyxy': [x1,y1,x2,y2], 'center_x_norm': ((x1+x2)*0.5)/width,
                                       'center_y_norm': ((y1+y2)*0.5)/height})
        msg = String()
        msg.data = json.dumps({'version': 1, 'timestamp_unix': time.time(), 'frame_sequence': self.frame_seq,
                               'image_width': width, 'image_height': height, 'detections': detections})
        self.frame_seq += 1
        self.pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap'): self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = OutletDetector()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc: print(exc)
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
