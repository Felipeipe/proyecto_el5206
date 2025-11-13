#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose, BoundingBox2D
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        # Cargar modelo YOLO (fine-tuned)
        self.model = YOLO('/path/to/best.pt')
        self.bridge = CvBridge()

        # Suscripción al tópico de la cámara
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',   # ajusta según tu cámara
            self.image_callback,
            10
        )

        # Publicador de detecciones
        self.publisher = self.create_publisher(Detection2DArray, '/detections', 10)

        self.get_logger().info("YOLO Detector Node iniciado.")

    def image_callback(self, msg):
        # Convertir ROS Image -> OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Inferencia YOLO
        results = self.model(frame, verbose=False)[0]

        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for box in results.boxes:
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            conf = float(box.conf[0])
            x1, y1, x2, y2 = map(float, box.xyxy[0])

            detection = Detection2D()
            detection.header = msg.header

            # Bounding Box
            bbox = BoundingBox2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1
            detection.bbox = bbox

            # Hipótesis (clase y confianza)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = cls_name
            hypothesis.hypothesis.score = conf
            detection.results.append(hypothesis)

            detections_msg.detections.append(detection)

        self.publisher.publish(detections_msg)
        self.get_logger().info(f"Publicadas {len(detections_msg.detections)} detecciones.")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
