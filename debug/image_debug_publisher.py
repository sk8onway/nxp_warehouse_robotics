"""
image_debug_publisher.py

Utility module for publishing debug images in ROS2.

Purpose:
- Convert OpenCV images (NumPy arrays) into ROS2 CompressedImage messages
- Publish them on a debug topic for visualization (RViz / Foxglove)

This module does NOT create a ROS node.
It is meant to be used by other ROS2 nodes for debugging purposes.
"""

import cv2
from sensor_msgs.msg import CompressedImage


def publish_debug_image(publisher, image):
    """
    Publish an OpenCV image as a ROS2 CompressedImage message.

    This function is typically used to visualize intermediate
    perception results such as:
    - QR detection bounding boxes
    - Object detection outputs
    - Shelf detection overlays

    Args:
        publisher:
            A ROS2 publisher created with:
            create_publisher(sensor_msgs.msg.CompressedImage, topic_name, qos)

        image:
            OpenCV image as a NumPy array (BGR format)

    Returns:
        None
    """

    # Ensure the image is valid and non-empty
    if image is None or image.size == 0:
        return

    # Encode the OpenCV image into JPEG format
    success, encoded_data = cv2.imencode('.jpg', image)
    if not success:
        return

    # Create CompressedImage ROS message
    message = CompressedImage()
    message.format = "jpeg"
    message.data = encoded_data.tobytes()

    # Publish the debug image
    publisher.publish(message)
