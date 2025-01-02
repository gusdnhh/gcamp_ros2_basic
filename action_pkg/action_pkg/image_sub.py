import cv2
import rclpy
import os
from datetime import datetime
from ultralytics import YOLO

from rclpy.node import Node
from cv_bridge import (
    CvBridge,
    CvBridgeError
)
from sensor_msgs.msg import Image

class ImageSubscriber(Node):

    def __init__(self):

        super().__init__("image_subscriber")

        self.sub_period = 10
        self.subscription = self.create_subscription(
            Image,
            "/diffbot/camera_sensor/image_raw",
            self.listener_callback,
            self.sub_period
        )

        self.cv_bridge = CvBridge()

        self.model = YOLO('/home/gusdnhh/gcamp_ros2_ws/src/gcamp_ros2_basic/weights/best.pt')
        self.detections = None

        self.output_dir = "/home/gusdnhh/gcamp_ros2_ws/src/gcamp_ros2_basic/imgs"

    def listener_callback(self, data):

        try:
            current_frame_bgr = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
            current_frame_hsv = cv2.cvtColor(current_frame_bgr, cv2.COLOR_BGR2HSV)
        except CvBridgeError as e:
            self.get_logger().info(e)
            return

        results = self.model.predict(current_frame_bgr, conf = 0.5)

        for box in results[0].boxes:
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            self.detections = results[0].names[class_id]


            x1, y1, x2, y2 = box.xyxy[0].tolist()
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            detection = {
                        'class_id': class_id,
                        'class_name': self.detections,
                        'confidence': confidence,
                        'bbox': [x1, y1, x2, y2]
                    }

            cv2.rectangle(current_frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label = f"{self.detections} {confidence:.2f}"
            (text_width, text_height), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

            cv2.rectangle(current_frame_bgr, (x1, y1 - text_height - 10),
                                  (x1 + text_width, y1), (0, 255, 0), -1)

            cv2.putText(current_frame_bgr, label, (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        cv2.imshow("camera", current_frame_bgr)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            time_str = datetime.now().strftime("%Y%m%d_%H%M%S%f")
            file_name = f"frame_{time_str}.jpg"
            file_path = os.path.join(self.output_dir, file_name)
            cv2.imwrite(file_path, current_frame_bgr)
            self.get_logger().info(f"이미지가 저장되었습니다: {file_path}")

        self.center_pixel = current_frame_hsv[400, 400]

def main(args=None):
    rclpy.init(args=args)
    image_subsrciber = ImageSubscriber()
    rclpy.spin(image_subsrciber)
    image_subsrciber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()