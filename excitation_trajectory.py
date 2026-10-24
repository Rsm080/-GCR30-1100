#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""五阶傅立叶级数动力学参数识别激励轨迹设计器

本脚本为 GCR30-1100（也可用于任意串联机器人 URDF）设计用于**动力学参数识别**的
激励轨迹，并导出给示教器（teach_pendant.py）显示/回放。

1. 轨迹形式（Swevers 等人的标准傅立叶级数，N = 5 阶）
   q_i(t)   = q0_i + sum_{k=1..N} [ a_ik/(k*wf) * sin(k*wf*t)
                                    - b_ik/(k*wf) * cos(k*wf*t) ]
   qd_i(t)  =        sum_{k=1..N} [ a_ik * cos(k*wf*t) + b_ik * sin(k*wf*t) ]
   qdd_i(t) =        sum_{k=1..N} [ -a_ik*k*wf * sin(k*wf*t)
                                    + b_ik*k*wf * cos(k*wf*t) ]
   其中 wf = 2*pi*f 为基频，a_ik、b_ik 为待设计系数（量纲 rad/s）。

2. 设计目标
   * 满足位置 / 速度 / 加速度约束（URDF 只给出位置与速度，加速度上限可配置）
   * 满足关节力矩上限（用刚体逆动力学校验）
   * 检查轨迹全程的连杆网格离地间隙，自动给出底座抬高/垫台高度
   * 最小化回归矩阵 W 的条件数（列归一化后，只在可辨识的基参数子空间内计算），
     使最小二乘辨识对噪声更鲁棒

3. 动力学回归矩阵
   对每个连杆使用 10 个惯性参数
       pi_i = [ m, m*cx, m*cy, m*cz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz ]
   其中惯量是相对**连杆坐标系原点**的惯量。用递归牛顿-欧拉法构造
   tau = W(q, qd, qdd) @ pi，本实现已与 PyBullet 逆动力学逐点对比，
   随机状态下的最大误差约 1e-8 N·m。

4. 用法
   python3 excitation_trajectory.py                     # 设计并保存到 trajectories/
   python3 excitation_trajectory.py --harmonics 5 --freq 0.1 --candidates 150 --refine 200
   python3 excitation_trajectory.py --csv traj.csv --npz traj.npz
   python3 excitation_trajectory.py --urdf 其他机器人.urdf
   设计完成后：
   python3 teach_pendant.py                             # 示教器中按 e 播放激励轨迹

注意：识别实验时请把 q/qd/qdd 采样数据与力矩一起保存，用本脚本导出的
NPZ（含 W 与参数名）即可直接做加权最小二乘 / 基参数辨识。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

DEG = 180.0 / math.pi
RAD = math.pi / 180.0

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_URDF = REPO_ROOT / "src" / "duco_support" / "urdf" / "duco_gcr30_1100.urdf"
DEFAULT_OUTPUT = REPO_ROOT / "trajectories" / "gcr30_1100_fourier5.json"


# --------------------------------------------------------------------------- #
# 基础数学工具
# --------------------------------------------------------------------------- #
def rpy_matrix(rpy: Sequence[float]) -> np.ndarray:
    """URDF 的 fixed-axis roll-pitch-yaw -> 旋转矩阵 (Rz @ Ry @ Rx)。"""
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def axis_angle(axis: Sequence[float], angle: float) -> np.ndarray:
    axis = np.asarray(axis, float)
    axis = axis / np.linalg.norm(axis)
    k = skew(axis)
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def skew(v: Sequence[float]) -> np.ndarray:
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]], dtype=float)


# --------------------------------------------------------------------------- #
# URDF 解析
# --------------------------------------------------------------------------- #
def parse_urdf(path: Path) -> Tuple[Dict[str, Optional[Dict]], List[Dict]]:
    """返回 (links, joints)。惯性参数统一换算到连杆原点坐标系。"""
    root = ET.parse(path).getroot()

    links: Dict[str, Optional[Dict]] = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            links[link.get("name")] = None
            continue
        origin = inertial.find("origin")
        xyz = np.array([float(v) for v in (origin.get("xyz", "0 0 0")).split()])
        rpy = np.array([float(v) for v in (origin.get("rpy", "0 0 0")).split()])
        mass = float(inertial.find("mass").get("value"))
        attrs = inertial.find("inertia").attrib
        inertia_com = np.array([
            [float(attrs["ixx"]), float(attrs["ixy"]), float(attrs["ixz"])],
            [float(attrs["ixy"]), float(attrs["iyy"]), float(attrs["iyz"])],
            [float(attrs["ixz"]), float(attrs["iyz"]), float(attrs["izz"])],
        ])
        rot = rpy_matrix(rpy)
        inertia_link_com = rot @ inertia_com @ rot.T
        # 平行轴定理：相对连杆原点的惯量
        inertia_origin = inertia_link_com + mass * (
            xyz.dot(xyz) * np.eye(3) - np.outer(xyz, xyz))
        links[link.get("name")] = {
            "mass": mass, "com": xyz, "inertia": inertia_origin,
        }

    joints: List[Dict] = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()] \
            if origin is not None else [0.0, 0.0, 0.0]
        rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()] \
            if origin is not None else [0.0, 0.0, 0.0]
        axis_el = joint.find("axis")
        axis = [float(v) for v in axis_el.get("xyz", "0 0 1").split()] \
            if axis_el is not None else [0.0, 0.0, 1.0]
        limit = joint.find("limit")
        joints.append({
            "name": joint.get("name"),
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "xyz": np.array(xyz, float),
            "rpy": np.array(rpy, float),
            "axis": np.array(axis, float),
            "lower": float(limit.get("lower")) if limit is not None else -math.pi,
            "upper": float(limit.get("upper")) if limit is not None else math.pi,
            "effort": float(limit.get("effort")) if limit is not None else 100.0,
            "velocity": float(limit.get("velocity")) if limit is not None else 1.0,
        })
    return links, joints


def build_chain(links: Dict[str, Optional[Dict]], joints: List[Dict]):
    """返回 (基座名, 拓扑序连杆名列表, child->joint 映射)。"""
    child_joint = {j["child"]: j for j in joints}
    parent_of = {j["child"]: j["parent"] for j in joints}
    roots = [name for name in links if name not in parent_of]
    if len(roots) != 1:
        raise SystemExit("仅支持单基座机器人，检测到基座: %s" % roots)
    base = roots[0]
    order: List[str] = []
    frontier = [base]
    while frontier:
        link = frontier.pop(0)
        order.append(link)
        for joint in joints:
            if joint["parent"] == link:
                frontier.append(joint["child"])
    return base, order, child_joint


# --------------------------------------------------------------------------- #
# 动力学回归矩阵（递归牛顿-欧拉，10 参数/连杆）
# --------------------------------------------------------------------------- #
def regressor(links, joints, order, child_joint, q, qd, qdd,
              gravity=(0.0, 0.0, -9.81), return_states: bool = False):
    """返回 tau = Y(q, qd, qdd) @ pi 的回归矩阵 Y (n_joints x 10*n_links)。

    pi_i = [m, m*cx, m*cy, m*cz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]（惯量相对连杆原点）。
    """
    g = np.array(gravity, float)
    moving = [name for name in order if name in child_joint]
    joint_index = {j["name"]: i for i, j in enumerate(joints)}
    Y = np.zeros((len(joints), 10 * len(moving)))

    states = {
        order[0]: {"p": np.zeros(3), "R": np.eye(3),
                   "w": np.zeros(3), "alpha": np.zeros(3), "a": np.zeros(3)}
    }

    for name in order:
        if name not in child_joint:
            continue
        joint = child_joint[name]
        parent_state = states[joint["parent"]]
        qi = q[joint_index[joint["name"]]]
        qdi = qd[joint_index[joint["name"]]]
        qddi = qdd[joint_index[joint["name"]]]

        r_parent = parent_state["R"] @ joint["xyz"]
        origin_pos = parent_state["p"] + r_parent
        R_origin = parent_state["R"] @ rpy_matrix(joint["rpy"])
        z = R_origin @ (joint["axis"] / np.linalg.norm(joint["axis"]))

        if joint["type"] in ("revolute", "continuous"):
            R_child = R_origin @ axis_angle(joint["axis"], qi)
            child_pos = origin_pos
            w = parent_state["w"] + z * qdi
            alpha = (parent_state["alpha"] + z * qddi
                     + np.cross(parent_state["w"], z) * qdi)
            a = (parent_state["a"]
                 + np.cross(parent_state["alpha"], r_parent)
                 + np.cross(parent_state["w"],
                            np.cross(parent_state["w"], r_parent)))
        elif joint["type"] == "prismatic":
            R_child = R_origin
            child_pos = origin_pos + z * qi
            v_rel = z * qdi
            a_rel = z * qddi + np.cross(parent_state["w"], z) * qdi
            w = parent_state["w"]
            alpha = parent_state["alpha"]
            a = (parent_state["a"]
                 + np.cross(parent_state["alpha"], r_parent)
                 + np.cross(parent_state["w"],
                            np.cross(parent_state["w"], r_parent))
                 + 2 * np.cross(parent_state["w"], v_rel) + a_rel)
        else:  # fixed
            R_child = R_origin
            child_pos = origin_pos
            w = parent_state["w"]
            alpha = parent_state["alpha"]
            a = (parent_state["a"]
                 + np.cross(parent_state["alpha"], r_parent)
                 + np.cross(parent_state["w"],
                            np.cross(parent_state["w"], r_parent)))

        states[name] = {"p": child_pos, "R": R_child, "w": w,
                        "alpha": alpha, "a": a}

    # 每个连杆的 6x10 参数基 wrench（世界系、位于连杆原点）
    wrenches = {}
    for name in moving:
        st = states[name]
        R = st["R"]
        w_local = R.T @ st["w"]
        alpha_local = R.T @ st["alpha"]
        v_local = R.T @ (st["a"] - g)

        # 力：f = m*v + (alpha x) h + omega x (omega x h)
        M_h = skew(alpha_local) + skew(w_local) @ skew(w_local)
        A_f = np.zeros((3, 10))
        A_n = np.zeros((3, 10))
        A_f[:, 0] = v_local
        A_f[:, 1:4] = M_h
        # 力矩中的 h 项：n_h = h x v = -v x h
        A_n[:, 1:4] = -skew(v_local)

        # 力矩中的惯量项：n_I = I*alpha + omega x (I*omega)
        def inertia_map(vector):
            L = np.zeros((3, 6))
            vx, vy, vz = vector
            L[0] = [vx, vy, vz, 0, 0, 0]
            L[1] = [0, vx, 0, vy, vz, 0]
            L[2] = [0, 0, vx, 0, vy, vz]
            return L

        A_n[:, 4:10] = (inertia_map(alpha_local)
                        + skew(w_local) @ inertia_map(w_local))
        wrenches[name] = (st["p"], R @ A_f, R @ A_n)

    for j_index, joint in enumerate(joints):
        child = joint["child"]
        st = states[child]
        p_joint = st["p"]
        z = st["R"] @ (joint["axis"] / np.linalg.norm(joint["axis"]))
        for name in moving:
            cursor = name
            in_subtree = False
            while cursor in child_joint:
                if child_joint[cursor]["name"] == joint["name"]:
                    in_subtree = True
                    break
                cursor = child_joint[cursor]["parent"]
            if not in_subtree:
                continue
            p_link, A_f, A_n = wrenches[name]
            col = moving.index(name)
            wrench = A_n + skew(p_link - p_joint) @ A_f
            Y[j_index, 10 * col:10 * col + 10] += (
                z @ wrench if joint["type"] != "prismatic" else z @ A_f)

    if return_states:
        return Y, states
    return Y


def true_parameters(links, joints, order, child_joint) -> np.ndarray:
    """从 URDF 惯量参数组装 10 参数向量 pi。"""
    params: List[float] = []
    for name in [n for n in order if n in child_joint]:
        link = links[name]
        if link is None:
            params.extend([0.0] * 10)
            continue
        mass = link["mass"]
        com = link["com"]
        inertia = link["inertia"]
        params.extend([mass, *(mass * com),
                       inertia[0, 0], inertia[0, 1], inertia[0, 2],
                       inertia[1, 1], inertia[1, 2], inertia[2, 2]])
    return np.array(params, float)


def parameter_names(links, joints, order, child_joint) -> List[str]:
    names: List[str] = []
    for name in [n for n in order if n in child_joint]:
        for suffix in ("m", "mx", "my", "mz", "Ixx", "Ixy", "Ixz",
                       "Iyy", "Iyz", "Izz"):
            names.append("%s.%s" % (name, suffix))
    return names


def tcp_positions(links, joints, order, child_joint,
                  q_samples: Sequence[Sequence[float]]) -> np.ndarray:
    """用本模块的正运动学求最后一个连杆原点（TCP 参考点）轨迹。"""
    points = []
    zeros = np.zeros(len(joints))
    last = [n for n in order if n in child_joint][-1]
    for q in q_samples:
        _, states = regressor(links, joints, order, child_joint,
                              np.asarray(q, float), zeros, zeros,
                              return_states=True)
        points.append(states[last]["p"].copy())
    return np.array(points)


# --------------------------------------------------------------------------- #
# 地面间隙检查（用连杆网格凸包）
# --------------------------------------------------------------------------- #
def resolve_mesh_path(filename: str, urdf_path: Path) -> Optional[Path]:
    if filename.startswith("package://"):
        package, _, relative = filename[len("package://"):].partition("/")
        for directory in (urdf_path.parent, *urdf_path.parents):
            package_xml = directory / "package.xml"
            if package_xml.is_file():
                try:
                    name = (ET.parse(package_xml).getroot().findtext("name") or "").strip()
                except ET.ParseError:
                    name = ""
                if name == package:
                    return directory / relative
        return None
    path = Path(filename)
    if not path.is_absolute():
        path = urdf_path.parent / path
    return path


def read_stl_vertices(path: Path) -> Optional[np.ndarray]:
    """读取二进制 STL 顶点（去重）。失败返回 None。"""
    try:
        raw = np.fromfile(path, dtype=np.uint8)
        if raw.size < 84:
            return None
        count = int(np.frombuffer(raw[80:84], dtype="<u4")[0])
        if raw.size != 84 + count * 50:
            return None
        record = np.frombuffer(
            raw[84:84 + count * 50],
            dtype=[("normal", "<f4", 3), ("v", "<f4", (3, 3)),
                   ("attr", "<u2")])
        vertices = record["v"].reshape(-1, 3).astype(float)
        return np.unique(vertices, axis=0)
    except Exception:                                # noqa: BLE001
        return None


def load_link_geometry(urdf_path: Path, links, joints, order,
                       child_joint) -> Dict[str, Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """读取每个连杆 visual 网格在连杆系下的凸包顶点。"""
    root = ET.parse(urdf_path).getroot()
    geometry = {}
    for link_element in root.findall("link"):
        name = link_element.get("name")
        if name not in order:
            continue
        visual = link_element.find("visual")
        if visual is None:
            continue
        mesh_element = visual.find("geometry/mesh")
        if mesh_element is None:
            continue
        mesh_path = resolve_mesh_path(mesh_element.get("filename", ""),
                                      urdf_path)
        if mesh_path is None or not mesh_path.is_file():
            continue
        vertices = read_stl_vertices(mesh_path)
        if vertices is None or len(vertices) < 4:
            continue
        try:
            from scipy.spatial import ConvexHull
            points = vertices[ConvexHull(vertices).vertices]
        except Exception:                            # noqa: BLE001
            points = vertices
        origin = visual.find("origin")
        xyz = (np.array([float(v) for v in origin.get("xyz", "0 0 0").split()])
               if origin is not None else np.zeros(3))
        rpy = ([float(v) for v in origin.get("rpy", "0 0 0").split()]
               if origin is not None else [0.0, 0.0, 0.0])
        geometry[name] = (xyz, rpy_matrix(rpy), points)
    return geometry


def ground_clearance(links, joints, order, child_joint, geometry,
                     q_samples: Sequence[Sequence[float]]):
    """轨迹全程各连杆网格相对基座坐标系的最低 z。

    返回 (全局最低 z, 每连杆最低 z 字典, 最低点采样序号)。
    """
    if not geometry:
        return None, {}, None
    zeros = np.zeros(len(joints))
    per_link = {name: float("inf") for name in geometry}
    lowest = float("inf")
    lowest_index = None
    for index, q in enumerate(q_samples):
        _, states = regressor(links, joints, order, child_joint,
                              np.asarray(q, float), zeros, zeros,
                              return_states=True)
        for name, (xyz, rot, points) in geometry.items():
            state = states[name]
            local = points @ rot.T + xyz
            world = local @ state["R"].T + state["p"]
            low = float(world[:, 2].min())
            per_link[name] = min(per_link[name], low)
            if low < lowest:
                lowest = low
                lowest_index = index
    return lowest, per_link, lowest_index


# --------------------------------------------------------------------------- #
# 傅立叶轨迹
# --------------------------------------------------------------------------- #
class FourierTrajectory:
    """N 阶傅立叶级数关节轨迹（解析给出 q / qd / qdd）。"""

    def __init__(self, q0, a, b, omega, joint_names: Optional[Sequence[str]] = None):
        self.q0 = np.asarray(q0, float)
        self.a = np.asarray(a, float)          # (n, K), rad/s
        self.b = np.asarray(b, float)          # (n, K), rad/s
        self.omega = float(omega)
        if self.a.shape != self.b.shape or self.a.shape[0] != self.q0.size:
            raise ValueError("Fourier 系数维度不一致")
        self.n_joints, self.harmonics = self.a.shape
        self.joint_names = list(joint_names) if joint_names else \
            ["J%d" % (i + 1) for i in range(self.n_joints)]

    @property
    def period(self) -> float:
        return 2.0 * math.pi / self.omega

    @property
    def frequency_hz(self) -> float:
        return self.omega / (2.0 * math.pi)

    def evaluate(self, t):
        """返回 (q, qd, qdd)，形状均为 (len(t), n_joints)。"""
        t = np.atleast_1d(np.asarray(t, float))
        q = np.tile(self.q0, (t.size, 1))
        qd = np.zeros_like(q)
        qdd = np.zeros_like(q)
        for k in range(1, self.harmonics + 1):
            w = k * self.omega
            s = np.sin(w * t)[:, None]
            c = np.cos(w * t)[:, None]
            ak = self.a[:, k - 1][None, :]
            bk = self.b[:, k - 1][None, :]
            q += ak / w * s - bk / w * c
            qd += ak * c + bk * s
            qdd += -ak * w * s + bk * w * c
        return q, qd, qdd

    def amplitude_bounds(self):
        """解析界：最大 |q-q0|、|qd|、|qdd| 的上界。"""
        k = np.arange(1, self.harmonics + 1)[None, :]
        mag = np.hypot(self.a, self.b)
        pos = np.sum(mag / (k * self.omega), axis=1)
        vel = np.sum(mag, axis=1)
        acc = np.sum(mag * k * self.omega, axis=1)
        return pos, vel, acc

    def to_payload(self, **extra) -> Dict:
        payload = {
            "type": "fourier_excitation",
            "version": 1,
            "harmonics": self.harmonics,
            "frequency_hz": self.frequency_hz,
            "omega_rad_s": self.omega,
            "period_s": self.period,
            "joint_names": self.joint_names,
            "q0_deg": (self.q0 * DEG).tolist(),
            "a": self.a.tolist(),
            "b": self.b.tolist(),
        }
        payload.update(extra)
        return payload

    @classmethod
    def from_payload(cls, payload: Dict) -> "FourierTrajectory":
        q0 = np.asarray(payload["q0_deg"], float) * RAD
        omega = float(payload.get("omega_rad_s",
                                  2.0 * math.pi * float(payload["frequency_hz"])))
        return cls(q0, payload["a"], payload["b"], omega,
                   payload.get("joint_names"))


# --------------------------------------------------------------------------- #
# 激励轨迹设计器
# --------------------------------------------------------------------------- #
def resolve_urdf_for_pybullet(urdf_path: Path) -> Tuple[Path, Optional[Path]]:
    """把 package:// 网格改写为绝对路径，返回 (可用 URDF, 临时目录)。"""
    tree = ET.parse(urdf_path)
    changed = False
    for mesh in tree.getroot().iter("mesh"):
        filename = mesh.get("filename", "")
        if not filename.startswith("package://"):
            continue
        package, _, relative = filename[len("package://"):].partition("/")
        package_dir = None
        for directory in (urdf_path.parent, *urdf_path.parents):
            package_xml = directory / "package.xml"
            if package_xml.is_file():
                try:
                    name = (ET.parse(package_xml).getroot().findtext("name") or "").strip()
                except ET.ParseError:
                    name = ""
                if name == package:
                    package_dir = directory
                    break
        if package_dir is None:
            return urdf_path, None
        mesh.set("filename", str(package_dir / relative))
        changed = True
    if not changed:
        return urdf_path, None
    temp_dir = Path(tempfile.mkdtemp(prefix="excitation_urdf_"))
    output = temp_dir / urdf_path.name
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output, temp_dir


@dataclass
class DesignResult:
    trajectory: FourierTrajectory
    condition_number: float
    rank: int
    sigma_max: float
    sigma_min_base: float
    torque_utilization: np.ndarray
    max_q_deg: np.ndarray
    max_qd: np.ndarray
    max_qdd: np.ndarray
    min_ground_clearance_m: float = float("nan")
    base_height_m: float = 0.0
    pedestal_height_m: float = 0.0
    per_link_clearance: Dict[str, float] = field(default_factory=dict)


class ExcitationDesigner:
    """随机搜索 + 局部精修，最小化回归矩阵条件数。"""

    def __init__(self, links, joints, order, child_joint, args):
        self.links = links
        self.joints = joints
        self.order = order
        self.child_joint = child_joint
        self.args = args
        self.n = len(joints)
        self.K = args.harmonics
        self.omega = 2.0 * math.pi * args.freq

        lower = np.array([j["lower"] for j in joints], float)
        upper = np.array([j["upper"] for j in joints], float)
        margin = 0.2 * (upper - lower)
        q0 = np.clip(np.zeros(self.n), lower + margin, upper - margin)
        if args.q0:
            values = [float(v) for v in args.q0.split(",")]
            if len(values) != self.n:
                raise SystemExit("--q0 需要 %d 个角度值（deg）" % self.n)
            q0 = np.clip(np.array(values, float) * RAD, lower, upper)
        self.q0 = q0
        self.lower, self.upper = lower, upper
        self.pos_margin = 0.85 * np.minimum(q0 - lower, upper - q0)
        self.vmax = np.array([j["velocity"] for j in joints], float)
        self.amax = (np.full(self.n, args.amax) if args.amax
                     else np.minimum(2.5 * self.vmax, args.amax_cap))
        self.effort = np.array([j["effort"] for j in joints], float)
        self.pi_true = true_parameters(links, joints, order, child_joint)
        self.joint_names = [j["name"] for j in joints]
        self.utilization = min(max(float(args.utilization), 0.1), 1.0)
        self.geometry = None

    def _ensure_geometry(self):
        if self.geometry is None:
            try:
                self.geometry = load_link_geometry(
                    Path(self.args.urdf), self.links, self.joints,
                    self.order, self.child_joint)
            except Exception as exc:                 # noqa: BLE001
                print("  读取连杆网格失败，跳过地面间隙检查: %s" % exc)
                self.geometry = {}
        return self.geometry

    # ------------------------------------------------------------------ #
    # 系数生成与限幅
    # ------------------------------------------------------------------ #
    def scale(self, a: np.ndarray, b: np.ndarray,
              utilization: Optional[float] = None):
        """按位置/速度/加速度约束整体缩放每个关节的系数。"""
        if utilization is None:
            utilization = self.utilization
        k = np.arange(1, self.K + 1)[None, :]
        mag = np.hypot(a, b)
        pos_bound = np.sum(mag / (k * self.omega), axis=1)
        vel_bound = np.sum(mag, axis=1)
        acc_bound = np.sum(mag * k * self.omega, axis=1)
        tiny = 1e-12
        ratio = np.minimum.reduce([
            self.pos_margin / np.maximum(pos_bound, tiny),
            utilization * self.vmax / np.maximum(vel_bound, tiny),
            utilization * self.amax / np.maximum(acc_bound, tiny),
        ])
        ratio = np.where(np.isfinite(ratio), ratio, 0.0)
        return a * ratio[:, None], b * ratio[:, None]

    def random_coefficients(self, rng: np.random.Generator,
                            utilization: Optional[float] = None):
        k = np.arange(1, self.K + 1)[None, :]
        magnitude = rng.uniform(0.35, 1.0, (self.n, self.K)) / k
        phase = rng.uniform(0.0, 2.0 * math.pi, (self.n, self.K))
        a = magnitude * np.cos(phase)
        b = magnitude * np.sin(phase)
        return self.scale(a, b, utilization)

    def make_trajectory(self, a: np.ndarray, b: np.ndarray) -> FourierTrajectory:
        return FourierTrajectory(self.q0, a, b, self.omega, self.joint_names)

    # ------------------------------------------------------------------ #
    # 回归矩阵与条件数
    # ------------------------------------------------------------------ #
    def observation_matrix(self, trajectory: FourierTrajectory, samples: int):
        t = np.arange(samples) / float(samples) * trajectory.period
        q, qd, qdd = trajectory.evaluate(t)
        rows = [regressor(self.links, self.joints, self.order, self.child_joint,
                          q[i], qd[i], qdd[i])
                for i in range(samples)]
        return np.vstack(rows), t, q, qd, qdd

    @staticmethod
    def spectral_metrics(W: np.ndarray):
        """列归一化后，在可辨识子空间内计算条件数。"""
        norms = np.linalg.norm(W, axis=0)
        if norms.size == 0 or norms.max() <= 0.0:
            return math.inf, 0, 0.0, 0.0
        keep = norms > 1e-9 * norms.max()
        normalized = W[:, keep] / norms[keep]
        singular = np.linalg.svd(normalized, compute_uv=False)
        rank = int(np.sum(singular > singular[0] * 1e-7))
        if rank == 0:
            return math.inf, 0, float(singular[0]), 0.0
        sigma_max = float(singular[0])
        sigma_min = float(singular[rank - 1])
        return sigma_max / sigma_min, rank, sigma_max, sigma_min

    def score(self, a, b, samples):
        trajectory = self.make_trajectory(a, b)
        W, _, _, _, _ = self.observation_matrix(trajectory, samples)
        return self.spectral_metrics(W)

    # ------------------------------------------------------------------ #
    # 随机搜索 + 局部精修
    # ------------------------------------------------------------------ #
    def design(self) -> DesignResult:
        args = self.args
        rng = np.random.default_rng(args.seed)
        start = time.perf_counter()

        best_cond = math.inf
        best_a = best_b = None
        best_rank = 0
        best_smax = best_smin = 0.0

        for index in range(args.candidates):
            a, b = self.random_coefficients(rng)
            cond, rank, smax, smin = self.score(a, b, args.search_samples)
            if cond < best_cond:
                best_cond, best_rank = cond, rank
                best_smax, best_smin = smax, smin
                best_a, best_b = a.copy(), b.copy()
            if args.verbose and (index + 1) % max(1, args.candidates // 5) == 0:
                print("  随机搜索 %d/%d: 最佳 cond=%.1f (rank=%d)"
                      % (index + 1, args.candidates, best_cond, best_rank))
        if best_a is None:
            raise SystemExit("未找到可行轨迹，请放宽约束")
        print("  随机搜索完成: cond=%.1f, 用时 %.1f s"
              % (best_cond, time.perf_counter() - start))

        step = 0.6
        for iteration in range(args.refine):
            joint = int(rng.integers(self.n))
            cand_a, cand_b = best_a.copy(), best_b.copy()
            magnitude = float(np.linalg.norm(
                np.r_[cand_a[joint], cand_b[joint]])) + 1e-9
            sigma = step * magnitude / math.sqrt(2.0 * self.K)
            cand_a[joint] += rng.normal(0.0, sigma, self.K)
            cand_b[joint] += rng.normal(0.0, sigma, self.K)
            cand_a, cand_b = self.scale(cand_a, cand_b)
            cond, rank, smax, smin = self.score(
                cand_a, cand_b, args.search_samples)
            if cond < best_cond:
                best_cond, best_rank = cond, rank
                best_smax, best_smin = smax, smin
                best_a, best_b = cand_a, cand_b
            step = max(0.03, step * 0.995)
            if args.verbose and (iteration + 1) % max(1, args.refine // 5) == 0:
                print("  局部精修 %d/%d: 最佳 cond=%.1f"
                      % (iteration + 1, args.refine, best_cond))
        print("  精修完成: cond=%.1f, 总用时 %.1f s"
              % (best_cond, time.perf_counter() - start))

        trajectory = self.make_trajectory(best_a, best_b)

        # 力矩校验：超限则整体缩放（均匀缩放不改变列归一化后的条件数）
        for _ in range(6):
            W, _, _, _, _ = self.observation_matrix(trajectory, args.samples)
            tau = (W @ self.pi_true).reshape(args.samples, self.n)
            utilization = np.max(np.abs(tau), axis=0) / np.maximum(self.effort, 1e-9)
            peak = float(np.max(utilization))
            if peak <= args.torque_limit:
                break
            factor = args.torque_limit / peak * 0.98
            best_a *= factor
            best_b *= factor
            trajectory = self.make_trajectory(best_a, best_b)
            print("  力矩峰值 %.0f%% 超限，整体缩放 x%.3f" % (peak * 100, factor))

        # 最终评估（采样更密）
        W, t, q, qd, qdd = self.observation_matrix(trajectory, args.samples)
        cond, rank, smax, smin = self.spectral_metrics(W)
        tau = (W @ self.pi_true).reshape(args.samples, self.n)
        utilization = np.max(np.abs(tau), axis=0) / np.maximum(self.effort, 1e-9)

        dense_t = np.linspace(0.0, trajectory.period,
                              args.check_samples, endpoint=False)
        q_dense, qd_dense, qdd_dense = trajectory.evaluate(dense_t)
        max_q = np.max(np.abs(q_dense - self.q0[None, :]), axis=0) * DEG
        max_qd = np.max(np.abs(qd_dense), axis=0)
        max_qdd = np.max(np.abs(qdd_dense), axis=0)

        for k in range(self.n):
            if max_qd[k] > self.vmax[k] * 1.02:
                print("警告: J%d 速度 %.3f 超过限值 %.3f rad/s"
                      % (k + 1, max_qd[k], self.vmax[k]))
            if max_qdd[k] > self.amax[k] * 1.02:
                print("警告: J%d 加速度 %.3f 超过设定 %.3f rad/s^2"
                      % (k + 1, max_qdd[k], self.amax[k]))

        # 地面间隙检查与底座抬高
        geometry = self._ensure_geometry()
        step = max(1, len(q_dense) // 200)
        clearance, per_link, worst_index = ground_clearance(
            self.links, self.joints, self.order, self.child_joint,
            geometry, q_dense[::step])
        base_height = 0.0
        pedestal_height = 0.0
        effective = float("nan")
        if clearance is not None:
            if args.base_height >= 0.0:
                base_height = float(args.base_height)
            else:
                base_height = max(0.0, args.ground_margin - clearance)
            effective = clearance + base_height
            base_low = per_link.get(self.order[0], 0.0)
            pedestal_height = max(0.0, base_height + base_low)
            worst_time = (dense_t[worst_index * step]
                          if worst_index is not None else 0.0)
            print("  地面间隙: 最低 %+.3f m @ t=%.2f s；底座抬高 %.3f m，"
                  "垫台高 %.3f m，抬高后最小间隙 %+.3f m"
                  % (clearance, worst_time, base_height,
                     pedestal_height, effective))
        else:
            print("  地面间隙: 无网格数据，未自动抬高底座")

        return DesignResult(
            trajectory=trajectory,
            condition_number=cond,
            rank=rank,
            sigma_max=smax,
            sigma_min_base=smin,
            torque_utilization=utilization,
            max_q_deg=max_q,
            max_qd=max_qd,
            max_qdd=max_qdd,
            min_ground_clearance_m=effective,
            base_height_m=base_height,
            pedestal_height_m=pedestal_height,
            per_link_clearance={name: value + base_height
                                for name, value in per_link.items()},
        )

    # ------------------------------------------------------------------ #
    # PyBullet 交叉校验（刚体逆动力学）
    # ------------------------------------------------------------------ #
    def verify_pybullet(self, trajectory: FourierTrajectory,
                        samples: int = 12) -> Optional[float]:
        try:
            import pybullet as p
        except Exception as exc:                     # noqa: BLE001
            print("  PyBullet 未安装，跳过交叉校验 (%s)" % exc)
            return None

        temp_dir: Optional[Path] = None
        cid = -1
        try:
            load_path, temp_dir = resolve_urdf_for_pybullet(
                Path(self.args.urdf))
            cid = p.connect(p.DIRECT)
            p.setGravity(0.0, 0.0, -9.81)
            body = p.loadURDF(str(load_path), useFixedBase=True,
                              flags=p.URDF_USE_INERTIA_FROM_FILE)
            if p.getNumJoints(body) != self.n:
                print("  PyBullet 关节数与 URDF 不一致，跳过交叉校验")
                return None
            t = np.linspace(0.0, trajectory.period, samples, endpoint=False)
            q, qd, qdd = trajectory.evaluate(t)
            worst = 0.0
            for i in range(samples):
                tau_pb = np.array(p.calculateInverseDynamics(
                    body, q[i].tolist(), qd[i].tolist(), qdd[i].tolist()))
                Y = regressor(self.links, self.joints, self.order,
                              self.child_joint, q[i], qd[i], qdd[i])
                worst = max(worst, float(np.max(np.abs(Y @ self.pi_true - tau_pb))))
            return worst
        except Exception as exc:                     # noqa: BLE001
            print("  PyBullet 交叉校验跳过: %s" % exc)
            return None
        finally:
            if cid >= 0:
                try:
                    p.disconnect(cid)
                except Exception:                     # noqa: BLE001
                    pass
            if temp_dir is not None:
                shutil.rmtree(temp_dir, ignore_errors=True)

    # ------------------------------------------------------------------ #
    # 导出
    # ------------------------------------------------------------------ #
    def build_payload(self, result: DesignResult) -> Dict:
        trajectory = result.trajectory
        _, t, q, qd, qdd = self.observation_matrix(
            trajectory, self.args.samples)
        tcp = tcp_positions(self.links, self.joints, self.order,
                            self.child_joint, q)
        limits = {
            "position_deg": (np.column_stack([self.lower, self.upper])
                             * DEG).tolist(),
            "velocity_rad_s": self.vmax.tolist(),
            "acceleration_rad_s2": self.amax.tolist(),
            "effort_nm": self.effort.tolist(),
        }
        metrics = {
            "condition_number": float(result.condition_number),
            "rank": int(result.rank),
            "sigma_max": float(result.sigma_max),
            "sigma_min_base": float(result.sigma_min_base),
            "torque_utilization": result.torque_utilization.tolist(),
            "max_q_deg": result.max_q_deg.tolist(),
            "max_qd_rad_s": result.max_qd.tolist(),
            "max_qdd_rad_s2": result.max_qdd.tolist(),
            "min_ground_clearance_m": float(result.min_ground_clearance_m),
            "ground_margin_m": float(self.args.ground_margin),
            "per_link_clearance_m": result.per_link_clearance,
        }
        return trajectory.to_payload(
            robot=self.args.urdf.stem,
            urdf=str(Path(self.args.urdf).resolve()),
            base_height_m=float(result.base_height_m),
            pedestal_height_m=float(result.pedestal_height_m),
            generated=datetime.now().isoformat(timespec="seconds"),
            design={
                "harmonics": self.K,
                "seed": self.args.seed,
                "candidates": self.args.candidates,
                "refine_iterations": self.args.refine,
                "search_samples": self.args.search_samples,
                "samples": self.args.samples,
            },
            limits=limits,
            metrics=metrics,
            samples={
                "count": int(len(t)),
                "t": t.tolist(),
                "q_deg": (q * DEG).tolist(),
                "qd_rad_s": qd.tolist(),
                "qdd_rad_s2": qdd.tolist(),
                "tcp_xyz": tcp.tolist(),
            },
        )

    def save_json(self, payload: Dict, path: Path) -> None:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2, ensure_ascii=False),
                        encoding="utf-8")
        print("已保存轨迹 JSON: %s" % path)

    def save_csv(self, trajectory: FourierTrajectory, path: Path) -> None:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        count = self.args.csv_samples
        t = np.linspace(0.0, trajectory.period, count, endpoint=False)
        q, qd, qdd = trajectory.evaluate(t)
        header = (["t_s"]
                  + ["q%d_rad" % (i + 1) for i in range(self.n)]
                  + ["qd%d_rad_s" % (i + 1) for i in range(self.n)]
                  + ["qdd%d_rad_s2" % (i + 1) for i in range(self.n)])
        with path.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            for index in range(count):
                writer.writerow([t[index], *q[index], *qd[index], *qdd[index]])
        print("已保存轨迹 CSV : %s (%d 行)" % (path, count))

    def save_npz(self, trajectory: FourierTrajectory, path: Path,
                 result: DesignResult) -> None:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        W, t, q, qd, qdd = self.observation_matrix(
            trajectory, self.args.samples)
        np.savez_compressed(
            path,
            t=t, q=q, qd=qd, qdd=qdd, W=W,
            pi_true=self.pi_true,
            parameter_names=np.array(parameter_names(
                self.links, self.joints, self.order, self.child_joint)),
            a=trajectory.a, b=trajectory.b, q0=trajectory.q0,
            omega=trajectory.omega,
            condition_number=result.condition_number,
            rank=result.rank,
        )
        print("已保存回归数据 NPZ: %s (W: %s)" % (path, W.shape))

    def save_plot(self, trajectory: FourierTrajectory, path: Path,
                  result: Optional[DesignResult] = None) -> None:
        """导出关节位置/速度/加速度与 TCP 轨迹曲线（可选）。"""
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:                     # noqa: BLE001
            print("matplotlib 不可用，跳过绘图: %s" % exc)
            return

        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        count = self.args.csv_samples
        t = np.linspace(0.0, trajectory.period, count)
        q, qd, qdd = trajectory.evaluate(t)
        tcp = tcp_positions(self.links, self.joints, self.order,
                            self.child_joint, q)

        figure, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
        for k in range(self.n):
            label = "J%d" % (k + 1)
            axes[0].plot(t, (q[:, k] - trajectory.q0[k]) * DEG, label=label)
            axes[1].plot(t, qd[:, k], label=label)
            axes[2].plot(t, qdd[:, k], label=label)
        axes[0].set_ylabel("q - q0 (deg)")
        axes[1].set_ylabel("qd (rad/s)")
        axes[2].set_ylabel("qdd (rad/s^2)")
        axes[3].plot(t, tcp[:, 0], label="x")
        axes[3].plot(t, tcp[:, 1], label="y")
        axes[3].plot(t, tcp[:, 2], label="z")
        axes[3].set_ylabel("TCP (m)")
        axes[3].set_xlabel("t (s)")
        for axis in axes:
            axis.grid(True, alpha=0.3)
            axis.legend(ncol=3, fontsize=8, loc="upper right")
        title = ("GCR30-1100  %d-order Fourier excitation  (T = %.1f s"
                 % (trajectory.harmonics, trajectory.period))
        if result is not None:
            title += ", cond = %.1f, rank = %d" % (
                result.condition_number, result.rank)
        figure.suptitle(title + ")")
        figure.tight_layout()
        figure.savefig(path, dpi=140)
        plt.close(figure)
        print("已保存轨迹图 PNG: %s" % path)

    # ------------------------------------------------------------------ #
    # 报告
    # ------------------------------------------------------------------ #
    def report(self, result: DesignResult,
               pybullet_error: Optional[float] = None) -> None:
        trajectory = result.trajectory
        print("\n" + "=" * 82)
        print("五阶傅立叶级数动力学参数识别激励轨迹")
        print("=" * 82)
        print("机器人      : %s" % self.args.urdf.stem)
        print("谐波数      : %d     基频: %.4f Hz     周期: %.3f s"
              % (self.K, trajectory.frequency_hz, trajectory.period))
        print("回归矩阵    : 列归一化后基参数秩 %d / %d 个惯性参数"
              % (result.rank, 10 * self.n))
        print("条件数      : %.2f   (sigma_max=%.2f, sigma_min=%.3e)"
              % (result.condition_number, result.sigma_max,
                 result.sigma_min_base))
        if pybullet_error is not None:
            print("逆动力学校验: 与 PyBullet 最大偏差 %.2e N·m"
                  % pybullet_error)
        if not math.isnan(result.min_ground_clearance_m):
            print("地面间隙    : 底座 +%.3f m（垫台 %.3f m），抬高后最小 %+.3f m"
                  % (result.base_height_m, result.pedestal_height_m,
                     result.min_ground_clearance_m))

        print("\n关节            q0(deg)  位置幅值  速度峰值/限值      加速度峰值/限值     力矩峰值/限值")
        print("-" * 82)
        for k, joint in enumerate(self.joints):
            print("%-12s  %+7.2f  %7.2f   %5.2f/%5.2f rad/s   %5.2f/%5.2f rad/s²   %3.0f%%"
                  % (joint["name"], self.q0[k] * DEG, result.max_q_deg[k],
                     result.max_qd[k], self.vmax[k],
                     result.max_qdd[k], self.amax[k],
                     result.torque_utilization[k] * 100))

        print("\nFourier 系数 a (rad/s)，列 = 谐波 1..%d:" % self.K)
        for k, name in enumerate(self.joint_names):
            print("  %-14s " % name
                  + " ".join("%+9.4f" % value for value in trajectory.a[k]))
        print("Fourier 系数 b (rad/s):")
        for k, name in enumerate(self.joint_names):
            print("  %-14s " % name
                  + " ".join("%+9.4f" % value for value in trajectory.b[k]))
        print("\n轨迹公式: q_i(t) = q0_i + sum_k [a_ik/(k*wf)*sin(k*wf*t)"
              " - b_ik/(k*wf)*cos(k*wf*t)]")
        print("=" * 82 + "\n")


# --------------------------------------------------------------------------- #
# 命令行入口
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="五阶傅立叶级数动力学参数识别激励轨迹设计器",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--urdf", default=str(DEFAULT_URDF),
                        help="机器人 URDF 路径")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT),
                        help="输出轨迹 JSON（示教器读取）")
    parser.add_argument("--harmonics", type=int, default=5,
                        help="傅立叶谐波阶数")
    parser.add_argument("--freq", type=float, default=0.1,
                        help="基频 (Hz)，周期 = 1/freq")
    parser.add_argument("--q0", default="",
                        help="轨迹中心角，逗号分隔的度数；默认自动取 0（在安全范围内）")
    parser.add_argument("--amax", type=float, default=0.0,
                        help="统一加速度上限 rad/s^2；0 表示自动取 2.5*velocity")
    parser.add_argument("--amax-cap", type=float, default=10.0,
                        help="自动加速度上限的天花板 rad/s^2")
    parser.add_argument("--torque-limit", type=float, default=0.85,
                        help="允许的最大力矩利用率（0~1）")
    parser.add_argument("--utilization", type=float, default=0.95,
                        help="位置/速度/加速度限值的利用率（0.1~1.0，越小越温和）")
    parser.add_argument("--ground-margin", type=float, default=0.02,
                        help="抬高底座后要求的最小离地间隙 (m)")
    parser.add_argument("--base-height", type=float, default=-1.0,
                        help="底座抬高量 (m)；<0 表示按轨迹自动计算")
    parser.add_argument("--samples", type=int, default=300,
                        help="导出与最终评估的采样点数")
    parser.add_argument("--search-samples", type=int, default=40,
                        help="搜索/精修阶段每个候选的采样点数（越小越快）")
    parser.add_argument("--check-samples", type=int, default=2000,
                        help="约束检查的密采样点数")
    parser.add_argument("--candidates", type=int, default=150,
                        help="随机候选轨迹数量")
    parser.add_argument("--refine", type=int, default=200,
                        help="局部精修迭代次数")
    parser.add_argument("--seed", type=int, default=0,
                        help="随机种子，便于复现")
    parser.add_argument("--csv", default="",
                        help="可选：导出 q/qd/qdd 的 CSV 文件")
    parser.add_argument("--csv-samples", type=int, default=1000,
                        help="CSV 采样点数")
    parser.add_argument("--npz", default="",
                        help="可选：导出含回归矩阵 W 的 NPZ（用于参数辨识）")
    parser.add_argument("--plot", default="",
                        help="可选：保存关节/TCP 轨迹曲线 PNG")
    parser.add_argument("--no-pybullet-check", action="store_true",
                        help="跳过与 PyBullet 逆动力学的交叉校验")
    parser.add_argument("--quiet", action="store_true",
                        help="只输出最终报告")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    args.verbose = not args.quiet
    args.urdf = Path(args.urdf).expanduser().resolve()
    if not args.urdf.is_file():
        raise SystemExit("找不到 URDF: %s" % args.urdf)

    print("读取 URDF: %s" % args.urdf)
    links, joints = parse_urdf(args.urdf)
    base, order, child_joint = build_chain(links, joints)
    print("基座: %s | 运动链: %s" % (base, " -> ".join(order)))

    designer = ExcitationDesigner(links, joints, order, child_joint, args)
    print("关节数 %d | 谐波 %d 阶 | 基频 %.4f Hz | 周期 %.2f s"
          % (designer.n, args.harmonics, args.freq, 1.0 / args.freq))
    print("速度上限: %s rad/s" % np.array2string(designer.vmax, precision=2))
    print("加速度上限: %s rad/s^2" % np.array2string(designer.amax, precision=2))

    result = designer.design()
    pybullet_error = None
    if not args.no_pybullet_check:
        pybullet_error = designer.verify_pybullet(result.trajectory)
    designer.report(result, pybullet_error)

    payload = designer.build_payload(result)
    designer.save_json(payload, Path(args.output))
    if args.csv:
        designer.save_csv(result.trajectory, Path(args.csv))
    if args.npz:
        designer.save_npz(result.trajectory, Path(args.npz), result)
    if args.plot:
        designer.save_plot(result.trajectory, Path(args.plot), result)

    print("完成。在示教器中查看/回放：")
    print("  python3 teach_pendant.py")
    print("  （示教器按 e 播放激励轨迹，或启动时用 --trajectory %s）"
          % args.output)
    return 0


if __name__ == "__main__":
    sys.exit(main())
