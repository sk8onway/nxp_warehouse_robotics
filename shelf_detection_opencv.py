"""
Shelf Detection using OpenCV on a SLAM Occupancy Grid

This module detects rectangular shelf-like structures from a binary
SLAM map using classical computer vision techniques.

Key ideas:
- Morphological filtering to clean noisy SLAM maps
- Contour detection to find connected obstacle regions
- Geometric filtering using area and aspect ratio
- Orientation estimation using minimum-area bounding rectangles

This file is intentionally ROS-independent.
Map-to-world conversion and publishing should be handled elsewhere.
"""

import cv2
import numpy as np


def detect_shelves(slam_map: np.ndarray):
    """
    Detect shelf-like structures from a grayscale SLAM map.

    Args:
        slam_map (np.ndarray):
            Grayscale map where free space is white (255)
            and obstacles are black (0).

    Returns:
        List[dict]: Each dict contains:
            - map_center (x, y) : shelf center in map pixel coordinates
            - width              : shelf width (pixels)
            - height             : shelf height (pixels)
            - yaw                : shelf orientation in degrees
            - area               : bounding box area (pixels^2)
    """

    # ------------------------------------------------------------------
    # 1. Threshold the map to ensure binary representation
    # ------------------------------------------------------------------
    _, binary_map = cv2.threshold(
        slam_map, 50, 255, cv2.THRESH_BINARY
    )

    # ------------------------------------------------------------------
    # 2. Morphological preprocessing
    #    - Close small gaps inside shelves
    #    - Remove isolated noise
    #    - Slightly expand connected components
    # ------------------------------------------------------------------
    def preprocess(binary):
        kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        kernel_dilate = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

        closed = cv2.morphologyEx(binary, cv2.MORPH_CLOSE,
                                  kernel_close, iterations=4)
        opened = cv2.morphologyEx(closed, cv2.MORPH_OPEN,
                                  kernel_open, iterations=2)
        dilated = cv2.dilate(opened, kernel_dilate, iterations=1)

        return dilated

    processed_map = preprocess(binary_map)

    # ------------------------------------------------------------------
    # 3. Extract contours from the processed map
    # ------------------------------------------------------------------
    contours, _ = cv2.findContours(
        processed_map, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )

    if not contours:
        return []

    # ------------------------------------------------------------------
    # 4. Identify and ignore the warehouse boundary
    #    (largest contour by area)
    # ------------------------------------------------------------------
    warehouse_contour = max(contours, key=cv2.contourArea)

    # Shelf geometry constraints (empirically tuned)
    MIN_AREA = 700
    MAX_AREA = 2800
    ASPECT_RATIO_RANGE = (0.35, 0.55)

    shelves = []

    # ------------------------------------------------------------------
    # 5. Analyze each contour
    # ------------------------------------------------------------------
    for cnt in contours:
        # Skip the warehouse boundary
        if np.array_equal(cnt, warehouse_contour):
            continue

        # Convex hull stabilizes noisy contours
        hull = cv2.convexHull(cnt)

        # Polygon approximation reduces jitter
        epsilon = 0.01 * cv2.arcLength(hull, True)
        approx = cv2.approxPolyDP(hull, epsilon, True)

        # Minimum-area bounding rectangle
        (cx, cy), (w, h), angle = cv2.minAreaRect(approx)

        area = w * h
        if area < MIN_AREA or area > MAX_AREA:
            continue

        aspect_ratio = min(w, h) / max(w, h)
        if not (ASPECT_RATIO_RANGE[0] <= aspect_ratio <= ASPECT_RATIO_RANGE[1]):
            continue

        # Normalize orientation:
        # OpenCV angle convention is tricky; normalize to shelf direction
        yaw = angle if w < h else angle + 90

        shelves.append({
            "map_center": (cx, cy),  # pixel coordinates
            "width": w,
            "height": h,
            "yaw": yaw,              # degrees
            "area": area
        })

    return shelves
