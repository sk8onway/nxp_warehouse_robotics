import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage

def camera_image_callback(self, message):
    """
    Processes incoming compressed camera images and detects QR codes.

    - Decodes ROS2 CompressedImage into OpenCV format
    - Uses OpenCV QRCodeDetector
    - Draws bounding box around detected QR
    - Publishes debug image for visualization (Foxglove / RViz)

    Args:
        message (sensor_msgs.msg.CompressedImage): Camera image message
    """

    # Decode compressed image to OpenCV format
    np_arr = np.frombuffer(message.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    # Initialize QR detector
    qr_detector = cv2.QRCodeDetector()

    # Detect and decode QR
    qr_data, bbox, _ = qr_detector.detectAndDecode(image)

    if qr_data:
        self.qr_code_str = qr_data
        self.get_logger().info(f"QR Code Detected: {qr_data}")

        # Draw bounding box if detected
        if bbox is not None and len(bbox) > 0:
            bbox = bbox.astype(int)
            for i in range(len(bbox[0])):
                pt1 = tuple(bbox[0][i])
                pt2 = tuple(bbox[0][(i + 1) % len(bbox[0])])
                cv2.line(image, pt1, pt2, (0, 255, 0), 2)

    # Publish debug image for visualization
    self.publish_debug_image(self.publisher_qr_decode, image)
