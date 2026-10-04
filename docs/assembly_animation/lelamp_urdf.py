"""Read the official LeLamp URDF and place every visual mesh at the zero pose.

Coordinates are returned in millimetres in a frame where +Z is up (base on the floor).
"""
from __future__ import annotations

import os
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

SIM = Path(os.environ.get("LELAMP_SIMULATION_DIR", Path(__file__).resolve().parents[2] / "simulation"))
URDF = SIM / "robot.urdf"


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def origin_matrix(el: ET.Element | None) -> np.ndarray:
    t = np.eye(4)
    if el is None:
        return t
    xyz = [float(v) for v in el.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in el.get("rpy", "0 0 0").split()]
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def read_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if 84 + count * 50 != len(data):
        raise ValueError(f"{path.name}: not a binary STL")
    rec = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=count, offset=84)
    return rec["v"].astype(np.float64)


@dataclass
class Visual:
    link: str
    part: str
    mesh: str
    tris: np.ndarray  # (n, 3, 3) millimetres, world frame, Z up


def _axis_rotation(axis: np.ndarray, q: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    t = np.eye(4)
    t[:3, :3] = np.eye(3) + np.sin(q) * k + (1 - np.cos(q)) * k @ k
    return t


def load_visuals(joint_angles: dict[str, float] | None = None) -> list[Visual]:
    """joint_angles maps URDF joint names ("1".."5") to radians; missing joints stay at 0."""
    joint_angles = joint_angles or {}
    root = ET.parse(URDF).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    parent_of = {}
    for j in root.findall("joint"):
        t = origin_matrix(j.find("origin"))
        if j.get("type") == "revolute":
            axis = np.array([float(v) for v in j.find("axis").get("xyz").split()])
            t = t @ _axis_rotation(axis, joint_angles.get(j.get("name"), 0.0))
        parent_of[j.find("child").get("link")] = (j.find("parent").get("link"), t)

    world: dict[str, np.ndarray] = {}

    def link_world(name: str) -> np.ndarray:
        if name not in world:
            if name in parent_of:
                parent, t = parent_of[name]
                world[name] = link_world(parent) @ t
            else:
                world[name] = np.eye(4)
        return world[name]

    # Re-express everything in the base link frame, then make +Z point from base to head.
    base_inv = np.linalg.inv(link_world("scs215_v5"))
    visuals: list[Visual] = []
    for name, link in links.items():
        if name == "imu_site":
            continue
        lw = base_inv @ link_world(name)
        comments = [c for c in link.iter() if c.tag is ET.Comment]
        for vis in link.findall("visual"):
            mesh = Path(vis.find("geometry/mesh").get("filename")).name
            t = lw @ origin_matrix(vis.find("origin"))
            tris = read_stl(SIM / "assets" / mesh)
            pts = tris.reshape(-1, 3) @ t[:3, :3].T + t[:3, 3]
            mat = vis.find("material")
            part = mat.get("name").removesuffix("_material") if mat is not None else mesh
            visuals.append(Visual(name, part, mesh, pts.reshape(-1, 3, 3) * 1000.0))

    base = np.concatenate([v.tris.reshape(-1, 3) for v in visuals if v.link == "scs215_v5" and v.mesh == "lamp_base.stl"])
    head = np.concatenate([v.tris.reshape(-1, 3) for v in visuals if v.link == "diffuser"])
    up = head.mean(0) - base.mean(0)
    up /= np.linalg.norm(up)
    # Snap "up" to the nearest base axis so the base sits flat.
    axis = np.zeros(3)
    k = int(np.argmax(np.abs(up)))
    axis[k] = np.sign(up[k])
    rot = _rotation_to_z(axis)
    for v in visuals:
        v.tris = v.tris @ rot.T
    floor = min(v.tris[..., 2].min() for v in visuals if v.link == "scs215_v5")
    for v in visuals:
        v.tris[..., 2] -= floor
    return visuals


def _rotation_to_z(a: np.ndarray) -> np.ndarray:
    z = np.array([0.0, 0.0, 1.0])
    if np.allclose(a, z):
        return np.eye(3)
    if np.allclose(a, -z):
        return np.diag([1.0, -1.0, -1.0])
    v = np.cross(a, z)
    s, c = np.linalg.norm(v), float(a @ z)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s**2)


if __name__ == "__main__":
    vs = load_visuals()
    for v in vs:
        c = v.tris.reshape(-1, 3).mean(0)
        print(f"{v.link:24s} {v.part:28s} tris={len(v.tris):6d} centroid={c.round(1)}")
