#!/usr/bin/env python3
"""Webots Supervisor controller for a 4-camera multi-angle showcase rig tracking WEBOTS_VEHICLE0."""

import math
import os
from controller import Supervisor

_DEFAULT_CAMS = "cam_chase,cam_top,cam_front,cam_side"
CAM_NAMES = tuple(
    n.strip()
    for n in os.environ.get("AVIS_SHOWCASE_CAM_NAMES", _DEFAULT_CAMS).split(",")
    if n.strip()
)
OUT_DIR = os.environ.get(
    "AVIS_SHOWCASE_CAM_DIR",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), ".frames"),
)
CAM_REFRESH_MS = max(10, int(os.environ.get("AVIS_SHOWCASE_REFRESH_MS", "30")))
JPEG_QUALITY = max(50, min(100, int(os.environ.get("AVIS_SHOWCASE_QUALITY", "85"))))


def _wrap_angle(ang: float) -> float:
    return (ang + math.pi) % (2.0 * math.pi) - math.pi


def main() -> None:
    os.makedirs(OUT_DIR, exist_ok=True)
    robot = Supervisor()
    time_step = int(robot.getBasicTimeStep())

    self_node = robot.getSelf()
    trans_field = self_node.getField("translation") if self_node is not None else None
    rot_field = self_node.getField("rotation") if self_node is not None else None
    vehicle_node = robot.getFromDef("WEBOTS_VEHICLE0")

    cameras = {}
    for name in CAM_NAMES:
        cam = robot.getDevice(name)
        if cam is not None:
            cam.enable(CAM_REFRESH_MS)
            cameras[name] = cam

    smooth_pos = None
    smooth_yaw = None
    step_idx = 0
    cam_stride = max(1, CAM_REFRESH_MS // max(1, time_step))

    while robot.step(time_step) != -1:
        if vehicle_node is not None and trans_field is not None and rot_field is not None:
            vpos = vehicle_node.getPosition()
            vrot = vehicle_node.getOrientation()
            raw_yaw = math.atan2(float(vrot[3]), float(vrot[0]))
            if smooth_pos is None or smooth_yaw is None:
                smooth_pos = [float(vpos[0]), float(vpos[1]), 0.0]
                smooth_yaw = raw_yaw
            else:
                alpha_p = 0.40
                alpha_y = 0.16
                smooth_pos[0] += alpha_p * (float(vpos[0]) - smooth_pos[0])
                smooth_pos[1] += alpha_p * (float(vpos[1]) - smooth_pos[1])
                smooth_yaw = _wrap_angle(smooth_yaw + alpha_y * _wrap_angle(raw_yaw - smooth_yaw))

            trans_field.setSFVec3f(smooth_pos)
            rot_field.setSFRotation([0.0, 0.0, 1.0, smooth_yaw])

        if step_idx % cam_stride == 0:
            for name, cam in cameras.items():
                tmp_path = os.path.join(OUT_DIR, f".tmp_{name}.jpg")
                dst_path = os.path.join(OUT_DIR, f"{name}.jpg")
                cam.saveImage(tmp_path, JPEG_QUALITY)
                if os.path.isfile(tmp_path):
                    os.replace(tmp_path, dst_path)

        step_idx += 1


if __name__ == "__main__":
    main()
