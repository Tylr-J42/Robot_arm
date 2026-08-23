"""Camera opening and burst capture, shared by every CLI that grabs frames.

Separated so the calibration tool, the object locator and the TF publisher all
open the camera the same way -- in particular, all of them get the resolution
assert, which is the cheapest safeguard in this pipeline.
"""

from __future__ import annotations

import cv2
import numpy as np

import constants


# Settings that change the IMAGE GEOMETRY. If any of these differ between
# calibration and runtime, the intrinsics are invalid -- silently, with no
# symptom beyond slightly-wrong poses. Everything else on the list below
# affects image quality only.
GEOMETRY_CRITICAL = ("frame_width", "frame_height", "autofocus", "focus", "zoom")

# One place for the whole capture configuration. Override per camera by
# defining CAMERA_SETTINGS = {...} in constants.py; anything absent there
# falls back to these. A value of None means "leave the driver's default".
DEFAULT_SETTINGS = {
    "fourcc": "MJPG",
    "frame_width": None,      # None -> constants.camera_res
    "frame_height": None,
    "fps": 30.0,
    # Autofocus MUST be off. A refocus changes the focal length, which changes
    # the intrinsics, which invalidates every pose derived from them. On a
    # fixed-focus module these are simply ignored.
    "autofocus": 0,
    "focus": None,
    "zoom": None,             # digital zoom crops and rescales -> new intrinsics
    "auto_exposure": 1,       # 1 = manual on V4L2
    "exposure": 30,
    "auto_wb": 0,
    "gain": None,
    # Sharpness is edge enhancement. It does not change geometry, but it biases
    # subpixel corner localization, so keep it at 0 for both calibration and
    # runtime.
    "sharpness": 0,
    # None = leave the driver's own default. These are per-camera image-quality
    # controls whose legal ranges differ wildly between devices, and an
    # out-of-range value is silently CLAMPED -- often to the worst setting the
    # camera has. (The Rocketfish clamps contrast to 60, its minimum, when asked
    # for anything lower; its default is 136.) Set these per camera in
    # constants.CAMERA_SETTINGS, not here.
    "brightness": None,
    "contrast": None,
    "gamma": None,
}

_PROPS = {
    "frame_width": cv2.CAP_PROP_FRAME_WIDTH,
    "frame_height": cv2.CAP_PROP_FRAME_HEIGHT,
    "fps": cv2.CAP_PROP_FPS,
    "autofocus": cv2.CAP_PROP_AUTOFOCUS,
    "focus": cv2.CAP_PROP_FOCUS,
    "zoom": cv2.CAP_PROP_ZOOM,
    "auto_exposure": cv2.CAP_PROP_AUTO_EXPOSURE,
    "exposure": cv2.CAP_PROP_EXPOSURE,
    "auto_wb": cv2.CAP_PROP_AUTO_WB,
    "gain": cv2.CAP_PROP_GAIN,
    "sharpness": cv2.CAP_PROP_SHARPNESS,
    "brightness": cv2.CAP_PROP_BRIGHTNESS,
    "contrast": cv2.CAP_PROP_CONTRAST,
    "gamma": cv2.CAP_PROP_GAMMA,
}

# Controls whose readback is not expected to echo what we wrote (drivers
# normalise or reinterpret these), so a mismatch is not worth reporting.
_NO_VERIFY = {"fourcc", "fps", "auto_exposure", "auto_wb", "autofocus"}


class Camera:
    """Thin wrapper around cv2.VideoCapture that also carries its settings.

    cv2.VideoCapture is a C extension type and rejects new attributes, so the
    resolved configuration cannot simply be attached to it. Every other
    attribute is delegated, so this behaves like a VideoCapture everywhere
    else -- read(), set(), get(), release() all work unchanged.
    """

    def __init__(self, cap, settings: dict):
        self._cap = cap
        self.settings = settings

    def __getattr__(self, name):
        # Only reached for attributes this wrapper does not define itself.
        return getattr(object.__getattribute__(self, "_cap"), name)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.release()
        return False


def get_settings(**overrides) -> dict:
    """DEFAULT_SETTINGS <- constants.CAMERA_SETTINGS <- explicit overrides."""
    s = dict(DEFAULT_SETTINGS)
    s.update(getattr(constants, "CAMERA_SETTINGS", {}) or {})
    s.update({k: v for k, v in overrides.items() if v is not None})
    if s["frame_width"] is None or s["frame_height"] is None:
        s["frame_width"], s["frame_height"] = constants.camera_res
    return s


def readback(cam: cv2.VideoCapture) -> dict:
    """What the driver actually accepted, as opposed to what we asked for."""
    return {name: float(cam.get(prop)) for name, prop in _PROPS.items()}


def open_camera(device: int = 0, exposure: int | None = None, **overrides):
    """Open the camera, apply the configured settings, and verify resolution.

    A driver that silently falls back to 640x480 invalidates the intrinsics in
    constants.py, and every number downstream is then wrong with no visible
    symptom beyond a slightly-off answer. That check is the cheapest safeguard
    in this pipeline, so it is an assert rather than a warning.

    Returns the VideoCapture; the settings actually applied are available via
    ``cam.settings`` (requested) and ``readback(cam)`` (accepted).
    """
    settings = get_settings(exposure=exposure, **overrides)
    cam = cv2.VideoCapture(device)
    if not cam.isOpened():
        raise SystemExit(f"could not open camera device {device}")

    if settings.get("fourcc"):
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*settings["fourcc"]))
    # Order matters: autofocus and auto-exposure must go off before the manual
    # values are written, or the driver overwrites them again.
    for name in ("autofocus", "auto_exposure", "auto_wb", "frame_width",
                 "frame_height", "fps", "focus", "zoom", "exposure", "gain",
                 "sharpness", "brightness", "contrast"):
        value = settings.get(name)
        if value is not None:
            cam.set(_PROPS[name], float(value))

    # Verify each control actually took. Drivers silently CLAMP out-of-range
    # values to the nearest legal one -- which is often the worst possible
    # setting -- and report success either way. This check is what turns that
    # into a visible warning instead of a mystery detection failure.
    for name, want in settings.items():
        if want is None or name in _NO_VERIFY:
            continue
        got = cam.get(_PROPS[name])
        if abs(float(got) - float(want)) > max(1.0, abs(float(want)) * 0.02):
            print(f"  [WARN] {name}: asked for {want}, driver reports {got:g} "
                  f"-- clamped or unsupported on this camera")

    shown = " ".join(
        f"{k}={settings[k]}" for k in
        ("fourcc", "exposure", "brightness", "contrast", "gamma", "sharpness")
        if settings.get(k) is not None)
    print(f"  camera: {int(settings['frame_width'])}x{int(settings['frame_height'])} "
          f"{shown}  (autofocus off)")

    got_w = int(round(cam.get(cv2.CAP_PROP_FRAME_WIDTH)))
    got_h = int(round(cam.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    want_w, want_h = int(settings["frame_width"]), int(settings["frame_height"])
    if (got_w, got_h) != (want_w, want_h):
        cam.release()
        raise SystemExit(
            f"camera delivered {got_w}x{got_h}, but the configured resolution is "
            f"{want_w}x{want_h}. The intrinsics were calibrated at "
            f"{want_w}x{want_h} and are invalid at any other resolution."
        )

    return Camera(cam, settings)


def check_against_calibration(settings: dict, intrinsics_doc: dict) -> list[str]:
    """Warn about geometry-critical settings that differ from calibration time.

    Returns a list of human-readable complaints; empty means consistent. Only
    GEOMETRY_CRITICAL keys are compared -- exposure and contrast may freely
    differ, they change how the image looks but not where things project to.
    """
    recorded = (intrinsics_doc or {}).get("capture_settings")
    problems = []
    if not recorded:
        return ["calibration file records no capture settings; cannot verify"]
    for key in GEOMETRY_CRITICAL:
        want, got = recorded.get(key), settings.get(key)
        if want is None and got is None:
            continue
        if want != got:
            problems.append(
                f"{key}: calibrated with {want!r}, now {got!r} -- this changes "
                "the image geometry, so the intrinsics no longer apply"
            )
    return problems


def settle(cam: cv2.VideoCapture, frames: int = 10) -> None:
    """Discard the first few frames while auto-exposure/gain stabilises."""
    for _ in range(frames):
        cam.read()


def capture_burst(
    cam: cv2.VideoCapture, n: int = 20, settle_frames: int = 10
) -> list[np.ndarray]:
    """Grab a short burst of frames of a static scene.

    A "still image" here is really a burst. Nothing in the scene is moving, so
    every frame observes the same geometry; averaging the detected corners
    across the burst cuts detector noise by sqrt(N) at the cost of a fraction of
    a second. That matters more now that the camera pose is re-solved from each
    capture rather than averaged over a long calibration run.
    """
    settle(cam, settle_frames)
    frames = []
    attempts = 0
    while len(frames) < n and attempts < n * 10:
        attempts += 1
        ok, image = cam.read()
        if ok:
            frames.append(image)
    if not frames:
        raise SystemExit("camera returned no frames")
    return frames


def warn_if_settings_drifted(cam, intrinsics_path="camera_intrinsics.yaml") -> None:
    """Print a warning if the live settings differ from the calibrated ones.

    Non-fatal and best-effort: if no calibration file is present this says
    nothing. It exists to catch the specific silent failure where someone
    changes resolution, focus or zoom and the poses quietly go wrong.
    """
    import yaml
    from pathlib import Path

    path = Path(intrinsics_path)
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / path
    if not path.is_file():
        return
    try:
        doc = yaml.safe_load(path.read_text())
    except Exception:
        return
    for problem in check_against_calibration(getattr(cam, "settings", {}), doc):
        print(f"  [WARN] {problem}")
