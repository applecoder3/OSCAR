import json
import time
import numpy as np
import onnxruntime as ort
from PIL import Image
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

modelPath = "/workspace/OSCAR_rover_prototype_v2/models/exp.onnx"
imagePath = "/workspace/camera_tests/live.jpg"

class OutletDetector(Node):
    def __init__(self):
        super().__init__("oscar_onnx_detector")

        self.publisher = self.create_publisher(
            String, "/outlets/detections", 10
        )

        self.session = ort.InferenceSession(
            modelPath, providers=["CPUExecutionProvider"]
        )

        self.frameNumber = 0
        self.timer = self.create_timer(2.0, self.detect)

        self.get_logger().info("ONNX outlet detector started")

    def detect(self):
        try:
            image = Image.open(imagePath).convert("RGB")
        except Exception as error:
            self.get_logger().error(str(error))
            return

        width, height = image.size
        scale = min(640 / width, 640 / height)
        newWidth = round(width * scale)
        newHeight = round(height * scale)

        resized = image.resize((newWidth, newHeight))
        canvas = Image.new("RGB", (640, 640), (114, 114, 114))

        padX = (640 - newWidth) // 2
        padY = (640 - newHeight) // 2
        canvas.paste(resized, (padX, padY))

        inputData = np.asarray(canvas, dtype=np.float32) / 255.0
        inputData = np.transpose(inputData, (2, 0, 1))[None, ...]

        predictions = self.session.run(
            None, {"images": inputData}
        )[0][0].T

        candidates = []

        for cx, cy, w, h, confidence in predictions:
            confidence = float(confidence)

            if confidence < 0.55:
                continue

            x1 = max(0.0, float(cx - w / 2 - padX) / scale)
            y1 = max(0.0, float(cy - h / 2 - padY) / scale)
            x2 = min(float(width), float(cx + w / 2 - padX) / scale)
            y2 = min(float(height), float(cy + h / 2 - padY) / scale)

            if x2 <= x1 or y2 <= y1:
                continue

            candidates.append({
                "class_id": 0,
                "confidence": confidence,
                "xyxy": [x1, y1, x2, y2],
                "center_x_norm": ((x1 + x2) / 2) / width,
                "center_y_norm": ((y1 + y2) / 2) / height
            })

        candidates.sort(
            key=lambda detection: detection["confidence"],
            reverse=True
        )

        def iou(a, b):
            ax1, ay1, ax2, ay2 = a["xyxy"]
            bx1, by1, bx2, by2 = b["xyxy"]

            intersection = max(0, min(ax2, bx2) - max(ax1, bx1)) * \
                           max(0, min(ay2, by2) - max(ay1, by1))
            areaA = (ax2 - ax1) * (ay2 - ay1)
            areaB = (bx2 - bx1) * (by2 - by1)

            return intersection / max(areaA + areaB - intersection, 1e-6)

        detections = []
        for candidate in candidates:
            if all(iou(candidate, existing) < 0.45 for existing in detections):
                detections.append(candidate)
            if len(detections) >= 20:
                break

        message = String()
        message.data = json.dumps({
            "version": 1,
            "timestamp_unix": time.time(),
            "frame_sequence": self.frameNumber,
            "image_width": width,
            "image_height": height,
            "detections": detections
        })

        self.publisher.publish(message)
        self.frameNumber += 1

        self.get_logger().info(
            f"Frame {self.frameNumber}: {len(detections)} outlets detected"
        )

def main():
    rclpy.init()
    node = OutletDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
