"""Thin dt_apriltags wrapper shared by the calibration runner and any ROS node.

Kept separate from capture so an image callback (ROS) and a VideoCapture loop
(CLI) can run identical detection code.

Detector settings mirror the working configuration in April_PNP_Live.py with one
deliberate change: quad_decimate is 1.0 rather than 2. Decimation is a realtime
speed hack that costs corner-localization accuracy, and extrinsic calibration is
an offline, one-shot job where accuracy is the only thing that matters.
"""

from __future__ import annotations

import cv2
import dt_apriltags
import numpy as np

DEFAULT_FAMILY = "tag36h11"
DEFAULT_MIN_DECISION_MARGIN = 30.0


def make_detector(
    *,
    families: str = DEFAULT_FAMILY,
    quad_decimate: float = 1.0,
    quad_sigma: float = 0.0,
    nthreads: int = 6,
    decode_sharpening: float = 0.0,
) -> dt_apriltags.Detector:
    """Build a detector tuned for accuracy rather than frame rate."""
    return dt_apriltags.Detector(
        searchpath=["apriltags"],
        families=families,
        nthreads=nthreads,
        quad_decimate=quad_decimate,
        quad_sigma=quad_sigma,
        refine_edges=1,
        decode_sharpening=decode_sharpening,
        debug=0,
    )


def to_gray(image: np.ndarray) -> np.ndarray:
    """Grayscale view of a BGR or already-gray frame."""
    if image.ndim == 2:
        return image
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


def detect(
    detector: dt_apriltags.Detector,
    image: np.ndarray,
    *,
    min_decision_margin: float = DEFAULT_MIN_DECISION_MARGIN,
    max_hamming: int = 0,
) -> list:
    """Detect tags and drop low-confidence or error-corrected hits.

    ``max_hamming=0`` keeps only detections that decoded with zero bit errors.
    A corrected detection is usually still the right id, but calibration has no
    reason to accept any doubt about identity.
    """
    gray = to_gray(image)
    dets = detector.detect(gray)
    return [
        d
        for d in dets
        if d.decision_margin >= min_decision_margin and d.hamming <= max_hamming
    ]


def detections_to_corners(dets) -> dict[int, np.ndarray]:
    """{tag_id: (4, 2) float64 pixel corners}, as dt_apriltags orders them.

    Duplicate ids in one frame mean two physical tags share an id, which would
    silently corrupt the solve, so they are dropped with a warning.
    """
    seen: dict[int, np.ndarray] = {}
    duplicates: set[int] = set()
    for d in dets:
        tag_id = int(d.tag_id)
        if tag_id in seen:
            duplicates.add(tag_id)
            continue
        seen[tag_id] = np.asarray(d.corners, dtype=np.float64).reshape(4, 2)
    for tag_id in sorted(duplicates):
        print(f"  [warn] tag id {tag_id} detected more than once; dropping it")
        seen.pop(tag_id, None)
    return seen
