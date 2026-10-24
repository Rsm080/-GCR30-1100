#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GCR30-1100 PyBullet 示教器 (Teach Pendant)

一个用于查看/示教 GCR30-1100（DUCO / 新松-杜科 GCR 系列）模型的轻量示教器：

  * 关节滑条（单位：度）+ 速度倍率滑条，带软件限速，运动平滑
  * 实时显示 6 个关节角、TCP 位置 (x/y/z) 与姿态 (rpy)
  * 示教点记录、回放（Smoothstep 缓动）、清空
  * 示教点保存/读取 JSON；TCP 轨迹线与点序号显示
  * 五阶傅立叶激励轨迹（动力学参数识别）加载/播放/暂停，实时显示进度与跟踪误差
  * 按轨迹自动抬高底座（圆柱垫台），避免激励轨迹与地面干涉
  * 关节轴显示、地面网格、视角复位
  * 自动解析 URDF 中的 package:// 网格路径（PyBullet 原生不支持）

用法：
    python3 teach_pendant.py                     # 打开 GUI 示教器
    python3 teach_pendant.py --direct            # 无窗口自检（用于 CI/验证）
    python3 teach_pendant.py --urdf 其他模型.urdf # 查看其它机器人
    python3 teach_pendant.py --waypoints my.json # 指定示教点文件

键盘快捷键（需先点击 PyBullet 窗口获得焦点）：
    1 ~ 6      选中关节 J1 ~ J6
    , / .      当前关节 -1° / +1°
    < / >      当前关节 -5° / +5°（Shift + , / .）
    0          全零位
    h          演示位（肘部弯折的安全姿态）
    r          随机姿态
    空格       记录当前示教点
    Enter      回放全部示教点（回放中再按一次可取消）
    c          清空示教点
    s          保存示教点 JSON
    l          读取示教点 JSON
    e          播放/暂停五阶傅立叶激励轨迹
    E          停止激励轨迹并复位到 t=0
    p          在终端打印当前状态
    x          显示/隐藏关节轴
    t          显示/隐藏 TCP 轨迹
    v          复位相机视角
    q / Esc    退出

作者说明：模型来自 yizhongzhang1989/duco_ros2_driver（duco_support 包），
GCR30_1100 的 STL 网格补齐自 shenyixsm/duco_raw。网格路径会在运行时
改写到一个临时 URDF 中，不会修改仓库内的原始文件。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import shutil
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import pybullet as p

try:
    from excitation_trajectory import FourierTrajectory
except Exception:                                   # noqa: BLE001
    FourierTrajectory = None                        # type: ignore

# --------------------------------------------------------------------------- #
# 常量与路径
# --------------------------------------------------------------------------- #
DEG = math.pi / 180.0
RAD = 180.0 / math.pi

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_URDF = REPO_ROOT / "src" / "duco_support" / "urdf" / "duco_gcr30_1100.urdf"
DEFAULT_WAYPOINT_FILE = REPO_ROOT / "waypoints" / "gcr30_1100_waypoints.json"
DEFAULT_TRAJECTORY = REPO_ROOT / "trajectories" / "gcr30_1100_fourier5.json"

# 演示位：肘部弯折、末端朝前的安全姿态（度）
DEMO_POSE_DEG = [0.0, -25.0, 55.0, 0.0, 30.0, 0.0]

FRAME_DT = 1.0 / 60.0      # 主循环 60 Hz
SUBSTEPS = 4               # 每帧物理子步数 -> 物理步长 1/240 s
JOYSTICK_STEP_DEG = 1.0    # 键盘点动步长
RANDOM_LIMIT_DEG = 60.0    # 随机姿态最大偏角

AXIS_LENGTH = 0.10         # 关节坐标轴长度 (m)
AXIS_COLORS = ([1.0, 0.15, 0.15], [0.15, 1.0, 0.15], [0.25, 0.45, 1.0])


# --------------------------------------------------------------------------- #
# URDF / 资源路径处理
# --------------------------------------------------------------------------- #
def find_package_dir(package: str, start: Path) -> Optional[Path]:
    """从 URDF 所在目录向上查找名为 package 的 ROS 包根目录。"""
    start = start.resolve()
    for directory in (start, *start.parents):
        package_xml = directory / "package.xml"
        if package_xml.is_file():
            try:
                name = (ET.parse(package_xml).getroot().findtext("name") or "").strip()
            except ET.ParseError:
                name = ""
            if name == package:
                return directory
        if directory.name == package and (directory / "meshes").is_dir():
            return directory

    # 兼容已经 colcon/ament 安装到 workspace 的情况
    for env_name in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        for prefix in os.environ.get(env_name, "").split(os.pathsep):
            if not prefix:
                continue
            candidate = Path(prefix) / "share" / package
            if (candidate / "package.xml").is_file():
                return candidate
    return None


def materialize_urdf(urdf_path: Path) -> Tuple[Path, Optional[Path]]:
    """把 URDF 中的 package:// 网格路径改写为绝对路径。

    PyBullet 无法解析 ROS 的 package:// URI，这里生成一个临时 URDF 供加载，
    返回 (可加载的 URDF 路径, 临时目录)；若无需改写则临时目录为 None。
    """
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    cache: Dict[str, Path] = {}
    changed = False
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if not filename.startswith("package://"):
            continue
        remainder = filename[len("package://"):]
        package, _, relative = remainder.partition("/")
        if package not in cache:
            package_dir = find_package_dir(package, urdf_path.parent)
            if package_dir is None:
                raise SystemExit(
                    "无法定位 ROS 包 '%s'（URDF: %s）。\n"
                    "请确认网格目录存在，或在 ROS 环境中 source 工作空间后重试。"
                    % (package, urdf_path)
                )
            cache[package] = package_dir
        mesh.set("filename", str(cache[package] / relative))
        changed = True

    if not changed:
        return urdf_path, None

    temp_dir = Path(tempfile.mkdtemp(prefix="teach_pendant_"))
    output = temp_dir / urdf_path.name
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output, temp_dir


# --------------------------------------------------------------------------- #
# 示教点数据结构
# --------------------------------------------------------------------------- #
@dataclass
class Waypoint:
    """一个示教点：关节角（度）与 TCP 位姿（m / 度）。"""

    joints_deg: List[float] = field(default_factory=list)
    tcp_xyz: List[float] = field(default_factory=list)
    tcp_rpy: List[float] = field(default_factory=list)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def smoothstep(t: float) -> float:
    t = clamp(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


# --------------------------------------------------------------------------- #
# 示教器主体
# --------------------------------------------------------------------------- #
class TeachPendant:
    """基于 PyBullet 的关节空间示教器。"""

    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.gui = not args.direct
        self.urdf_source = Path(args.urdf).expanduser().resolve()
        if not self.urdf_source.is_file():
            raise SystemExit("找不到 URDF 文件: %s" % self.urdf_source)

        self.urdf_path, self._temp_dir = materialize_urdf(self.urdf_source)
        self.waypoint_file = Path(args.waypoints).expanduser()

        self.cid = -1
        self.robot = -1
        self.joints: List[Dict] = []
        self.joint_ids: List[int] = []
        self.tcp_index = -1

        # 底座抬高（离地），优先使用命令行参数，否则读取轨迹 JSON 的建议值
        suggested_base, suggested_pedestal = self._peek_placement(args.trajectory)
        if args.base_height is not None:
            self.base_height = max(0.0, float(args.base_height))
            offset = suggested_pedestal - suggested_base
            self.pedestal_height = max(0.0, self.base_height + offset)
        elif args.no_trajectory:
            self.base_height = 0.0
            self.pedestal_height = 0.0
        else:
            self.base_height = suggested_base
            self.pedestal_height = suggested_pedestal

        # 关节目标 / 实际下发的指令 / 滑条上次读数（均为弧度）
        self.targets: List[float] = []
        self.commands: List[float] = []
        self.last_slider: List[float] = []

        self.waypoints: List[Waypoint] = []
        self.play_queue: List[Tuple[List[float], List[float], float]] = []
        self.play_active = False
        self.play_elapsed = 0.0

        # 傅立叶激励轨迹（动力学参数识别）
        self.excitation = None
        self.excitation_payload: Dict = {}
        self.exc_playing = False
        self.exc_time = 0.0
        self.exc_cycle = 0
        self.exc_desired: Optional[List[float]] = None
        self.exc_path_ids: List[int] = []
        self.exc_marker = -1

        self.active_joint = 0
        self.speed_scale = 1.0
        self.axes_on = bool(args.show_axes)
        self.trail_on = True
        self.axes_ids: List[int] = []
        self.trail_ids: List[int] = []
        self.readout_id = -1
        self.status = "Ready"
        self.running = False

        self.sliders: List[int] = []
        self.speed_slider = -1
        self.buttons: Dict[str, int] = {}
        self.button_values: Dict[str, float] = {}

    @staticmethod
    def _peek_placement(path) -> Tuple[float, float]:
        """读取轨迹 JSON 建议的底座/垫台高度（不修改任何状态）。"""
        try:
            payload = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
            base = max(0.0, float(payload.get("base_height_m", 0.0)))
            pedestal = max(0.0, float(payload.get("pedestal_height_m", base)))
            return base, pedestal
        except Exception:                            # noqa: BLE001
            return 0.0, 0.0

    # ------------------------------------------------------------------ #
    # 初始化
    # ------------------------------------------------------------------ #
    def setup(self) -> None:
        self.cid = p.connect(p.GUI if self.gui else p.DIRECT)
        if self.cid < 0:
            raise SystemExit(
                "无法连接 PyBullet %s 模式。\n"
                "如果是无显示器环境，请使用 --direct 自检，或配置 DISPLAY 后重试。"
                % ("GUI" if self.gui else "DIRECT")
            )

        p.setTimeStep(FRAME_DT / SUBSTEPS, physicsClientId=self.cid)
        p.setGravity(0.0, 0.0, 0.0 if self.args.zero_gravity else -9.81,
                     physicsClientId=self.cid)
        p.setAdditionalSearchPath(str(REPO_ROOT), physicsClientId=self.cid)

        if self.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1, physicsClientId=self.cid)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1, physicsClientId=self.cid)
            p.configureDebugVisualizer(p.COV_ENABLE_KEYBOARD_SHORTCUTS, 0,
                                       physicsClientId=self.cid)

        self._build_scene()
        self._load_robot()
        self._init_motors()

        if self.gui:
            self.reset_camera()
            self._build_ui()
            self._draw_axes()
            self._refresh_trail()

        if not self.args.no_trajectory:
            self.load_excitation(self.args.trajectory)

    def _build_scene(self) -> None:
        """地面 + 网格。"""
        collision = p.createCollisionShape(p.GEOM_PLANE, physicsClientId=self.cid)
        # PyBullet 的 GEOM_PLANE 没有默认可视几何，这里用薄盒做可见地面
        visual = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[2.5, 2.5, 0.005],
            rgbaColor=[0.30, 0.32, 0.35, 1.0], physicsClientId=self.cid,
        )
        p.createMultiBody(0, collision, visual, basePosition=[0.0, 0.0, -0.005],
                          physicsClientId=self.cid)

        # 底座垫台：把机器人整体抬高，避免激励轨迹与地面干涉
        # 用细圆柱（r=0.16 m，底座网格最大半径约 0.131 m），
        # 保证低处摆动的连杆（最小径向距离约 0.178 m）不会碰到垫台侧面
        if self.pedestal_height > 1e-4:
            pedestal_radius = 0.16
            pedestal_collision = p.createCollisionShape(
                p.GEOM_CYLINDER, radius=pedestal_radius,
                height=self.pedestal_height, physicsClientId=self.cid)
            pedestal_visual = p.createVisualShape(
                p.GEOM_CYLINDER, radius=pedestal_radius,
                length=self.pedestal_height,
                rgbaColor=[0.20, 0.22, 0.25, 1.0], physicsClientId=self.cid)
            p.createMultiBody(0, pedestal_collision, pedestal_visual,
                              basePosition=[0.0, 0.0, 0.5 * self.pedestal_height],
                              physicsClientId=self.cid)

        if not self.gui:
            return
        grid_color = [0.42, 0.44, 0.48]
        half, step = 2.0, 0.5
        count = int(half / step)
        for i in range(-count, count + 1):
            offset = i * step
            p.addUserDebugLine([-half, offset, 0.001], [half, offset, 0.001],
                               grid_color, lineWidth=1, physicsClientId=self.cid)
            p.addUserDebugLine([offset, -half, 0.001], [offset, half, 0.001],
                               grid_color, lineWidth=1, physicsClientId=self.cid)

    def _load_robot(self) -> None:
        try:
            self.robot = p.loadURDF(
                str(self.urdf_path),
                basePosition=[0.0, 0.0, self.base_height],
                baseOrientation=[0.0, 0.0, 0.0, 1.0],
                useFixedBase=True,
                flags=p.URDF_USE_INERTIA_FROM_FILE,
                physicsClientId=self.cid,
            )
        except p.error as exc:
            raise SystemExit("加载 URDF 失败: %s\n%s" % (self.urdf_path, exc))

        self.joints = []
        for index in range(p.getNumJoints(self.robot, physicsClientId=self.cid)):
            info = p.getJointInfo(self.robot, index, physicsClientId=self.cid)
            if info[2] not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                continue
            lower, upper = float(info[8]), float(info[9])
            if math.isclose(lower, upper):          # continuous joint
                lower, upper = -math.pi, math.pi
            self.joints.append({
                "index": index,
                "name": info[1].decode(),
                "type": info[2],
                "lower": lower,
                "upper": upper,
                "force": max(float(info[10]), 1.0),
                "max_vel": max(float(info[11]), 0.1),
                "link": info[12].decode(),
            })

        if not self.joints:
            raise SystemExit("URDF 中没有可动关节: %s" % self.urdf_source)

        self.joint_ids = [joint["index"] for joint in self.joints]
        self.tcp_index = self.joint_ids[-1]
        count = len(self.joints)
        self.targets = [0.0] * count
        self.commands = [0.0] * count
        self.last_slider = [0.0] * count

        print("已加载模型: %s" % self.urdf_source.name)
        print("TCP 参考坐标系: %s (joint index %d)"
              % (self.joints[-1]["link"], self.tcp_index))
        for k, joint in enumerate(self.joints, start=1):
            print("  J%d  %-14s 限位 %+7.2f° ~ %+7.2f°  力矩 %6.1f N.m  速度 %.2f rad/s"
                  % (k, joint["name"], joint["lower"] * RAD, joint["upper"] * RAD,
                     joint["force"], joint["max_vel"]))

    def _init_motors(self) -> None:
        for joint in self.joints:
            p.resetJointState(self.robot, joint["index"], 0.0, 0.0,
                              physicsClientId=self.cid)
            p.setJointMotorControl2(
                self.robot, joint["index"], p.POSITION_CONTROL,
                targetPosition=0.0, force=joint["force"],
                positionGain=0.6, velocityGain=1.0,
                maxVelocity=joint["max_vel"],
                physicsClientId=self.cid,
            )

    def _build_ui(self) -> None:
        """创建滑条与按钮。

        PyBullet 内置调试字体只支持 ASCII，所以界面文字统一用英文。
        """
        for k, joint in enumerate(self.joints, start=1):
            self.sliders.append(p.addUserDebugParameter(
                "J%d (deg)" % k,
                joint["lower"] * RAD, joint["upper"] * RAD, 0.0,
                physicsClientId=self.cid,
            ))

        self.speed_slider = p.addUserDebugParameter(
            "Speed x", 0.1, 2.0, 1.0, physicsClientId=self.cid)

        labels = [
            ("zero", "Zero (0)"),
            ("demo", "Demo (h)"),
            ("random", "Random (r)"),
            ("record", "Record (Space)"),
            ("play", "Play (Enter)"),
            ("clear", "Clear (c)"),
            ("save", "Save (s)"),
            ("load", "Load (l)"),
            ("excitation", "Excitation (e)"),
        ]
        for key, label in labels:
            self.buttons[key] = p.addUserDebugParameter(
                label, 1, 0, 0, physicsClientId=self.cid)

        self.last_slider = [
            p.readUserDebugParameter(sid, physicsClientId=self.cid)
            for sid in self.sliders
        ]
        self.button_values = {key: 0.0 for key in self.buttons}

        print("\n示教器已就绪，请点击 PyBullet 窗口后使用滑条或键盘操作。")
        print("快捷键: 1-6 选关节 | , ./ 微调 ±1° | < > ±5° | 0 全零 | h 演示位")
        print("        r 随机 | 空格 记录 | Enter 回放 | c 清空 | s 保存 | l 读取")
        print("        p 打印 | x 关节轴 | t 轨迹 | v 视角 | e 激励轨迹 | q/Esc 退出\n")

    # ------------------------------------------------------------------ #
    # 状态读取 / 目标设置
    # ------------------------------------------------------------------ #
    def get_joint_positions(self) -> List[float]:
        states = p.getJointStates(self.robot, self.joint_ids,
                                  physicsClientId=self.cid)
        return [float(state[0]) for state in states]

    def tcp_pose(self) -> Tuple[List[float], List[float], List[float]]:
        state = p.getLinkState(self.robot, self.tcp_index,
                               computeForwardKinematics=True,
                               physicsClientId=self.cid)
        position = [float(v) for v in state[4]]
        quaternion = [float(v) for v in state[5]]
        rpy = [float(v) for v in p.getEulerFromQuaternion(quaternion)]
        return position, rpy, quaternion

    def set_targets_deg(self, degrees: Sequence[float], status: str = "") -> None:
        for k, (joint, value) in enumerate(zip(self.joints, degrees)):
            self.targets[k] = clamp(float(value) * DEG, joint["lower"], joint["upper"])
        if self.gui and self.sliders:
            # 滑条不可写，只能记录当前读数，避免预设姿态被旧滑条值立刻覆盖
            self.last_slider = [
                p.readUserDebugParameter(sid, physicsClientId=self.cid)
                for sid in self.sliders
            ]
        if status:
            self.status = status

    def jog_joint(self, delta_deg: float) -> None:
        self.stop_excitation()
        joint = self.joints[self.active_joint]
        value = clamp(self.targets[self.active_joint] + delta_deg * DEG,
                      joint["lower"], joint["upper"])
        self.targets[self.active_joint] = value
        if self.gui and self.sliders:
            self.last_slider[self.active_joint] = p.readUserDebugParameter(
                self.sliders[self.active_joint], physicsClientId=self.cid)
        self.status = "J%d -> %+.1f deg" % (self.active_joint + 1, value * RAD)

    def update_from_sliders(self) -> None:
        if (not self.gui or not self.sliders
                or self.play_active or self.exc_playing):
            return
        for k, slider in enumerate(self.sliders):
            value = p.readUserDebugParameter(slider, physicsClientId=self.cid)
            if abs(value - self.last_slider[k]) > 1e-9:
                self.last_slider[k] = value
                joint = self.joints[k]
                self.targets[k] = clamp(value * DEG, joint["lower"], joint["upper"])
                self.status = "J%d slider -> %+.1f deg" % (k + 1, value)
        speed = p.readUserDebugParameter(self.speed_slider, physicsClientId=self.cid)
        self.speed_scale = clamp(float(speed), 0.1, 2.0)

    def update_from_buttons(self) -> None:
        if not self.gui:
            return
        for key, param in self.buttons.items():
            value = p.readUserDebugParameter(param, physicsClientId=self.cid)
            if value != self.button_values.get(key, 0.0):
                self.button_values[key] = value
                self.on_button(key)

    def on_button(self, key: str) -> None:
        actions = {
            "zero": self.zero_pose,
            "demo": self.demo_pose,
            "random": self.random_pose,
            "record": self.record_waypoint,
            "play": self.toggle_playback,
            "clear": self.clear_waypoints,
            "save": self.save_waypoints,
            "load": self.load_waypoints,
            "excitation": self.toggle_excitation,
        }
        action = actions.get(key)
        if action is not None:
            action()

    # ------------------------------------------------------------------ #
    # 姿态动作
    # ------------------------------------------------------------------ #
    def zero_pose(self) -> None:
        self.stop_excitation()
        self.set_targets_deg([0.0] * len(self.joints), "Zero pose")

    def demo_pose(self) -> None:
        self.stop_excitation()
        pose = DEMO_POSE_DEG[:len(self.joints)]
        self.set_targets_deg(pose, "Demo pose")

    def random_pose(self) -> None:
        self.stop_excitation()
        pose = []
        for joint in self.joints:
            lower = max(joint["lower"] * RAD, -RANDOM_LIMIT_DEG)
            upper = min(joint["upper"] * RAD, RANDOM_LIMIT_DEG)
            pose.append(random.uniform(lower, upper) if upper > lower else 0.0)
        self.set_targets_deg(pose, "Random pose")

    def reset_camera(self) -> None:
        if not self.gui:
            return
        p.resetDebugVisualizerCamera(
            cameraDistance=2.4 + 0.6 * self.base_height,
            cameraYaw=55.0, cameraPitch=-15.0,
            cameraTargetPosition=[0.30, 0.0, 0.70 + 0.75 * self.base_height],
            physicsClientId=self.cid)

    # ------------------------------------------------------------------ #
    # 示教点记录 / 回放 / 存取
    # ------------------------------------------------------------------ #
    def record_waypoint(self) -> None:
        positions = self.get_joint_positions()
        tcp_position, tcp_rpy, _ = self.tcp_pose()
        waypoint = Waypoint(
            joints_deg=[round(v * RAD, 3) for v in positions],
            tcp_xyz=[round(v, 4) for v in tcp_position],
            tcp_rpy=[round(v * RAD, 2) for v in tcp_rpy],
        )
        self.waypoints.append(waypoint)
        self._refresh_trail()
        self.status = "Recorded waypoint #%d" % len(self.waypoints)
        print("记录示教点 #%d  关节(deg)=%s  TCP=(%.3f, %.3f, %.3f)"
              % (len(self.waypoints), waypoint.joints_deg, *waypoint.tcp_xyz))

    def _segment_duration(self, start: Sequence[float],
                          goal: Sequence[float]) -> float:
        """按各关节最大速度估算段时长（留 1.5 倍裕量）。"""
        longest = 0.0
        for k, joint in enumerate(self.joints):
            delta = abs(goal[k] - start[k])
            longest = max(longest, delta / (joint["max_vel"] * 1.5))
        return clamp(longest, 0.6, 4.0)

    def toggle_playback(self) -> None:
        self.stop_excitation()
        if self.play_active:
            self.play_queue.clear()
            self.play_active = False
            self.status = "Playback cancelled"
            print("回放已取消")
            return
        if not self.waypoints:
            self.status = "No waypoints"
            print("没有示教点可回放，请先按空格记录")
            return

        previous = self.get_joint_positions()
        queue: List[Tuple[List[float], List[float], float]] = []
        for waypoint in self.waypoints:
            goal = [clamp(value * DEG, joint["lower"], joint["upper"])
                    for value, joint in zip(waypoint.joints_deg, self.joints)]
            queue.append((list(previous), goal,
                          self._segment_duration(previous, goal)))
            previous = goal

        self.play_queue = queue
        self.play_elapsed = 0.0
        self.play_active = True
        self.status = "Playing %d waypoints" % len(self.waypoints)
        print("开始回放 %d 个示教点" % len(self.waypoints))

    def update_playback(self, dt: float) -> None:
        if not self.play_active:
            return
        if not self.play_queue:
            self.play_active = False
            return
        start, goal, duration = self.play_queue[0]
        self.play_elapsed += dt
        ratio = smoothstep(self.play_elapsed / max(duration, 1e-6))
        self.targets = [a + (b - a) * ratio for a, b in zip(start, goal)]
        if self.play_elapsed < duration:
            return

        self.play_queue.pop(0)
        self.play_elapsed = 0.0
        if self.play_queue:
            return
        self.play_active = False
        self.status = "Playback finished"
        if self.gui and self.sliders:
            self.last_slider = [
                p.readUserDebugParameter(sid, physicsClientId=self.cid)
                for sid in self.sliders
            ]
        print("回放完成")

    def clear_waypoints(self) -> None:
        self.waypoints.clear()
        self.play_queue.clear()
        self.play_active = False
        self._refresh_trail()
        self.status = "Waypoints cleared"
        print("已清空示教点")

    def save_waypoints(self) -> None:
        path = self.waypoint_file
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                "robot": self.urdf_source.stem,
                "urdf": str(self.urdf_source),
                "units": {"joints": "deg", "tcp_xyz": "m", "tcp_rpy": "deg"},
                "created": datetime.now().isoformat(timespec="seconds"),
                "waypoints": [asdict(waypoint) for waypoint in self.waypoints],
            }
            path.write_text(json.dumps(payload, indent=2, ensure_ascii=False),
                            encoding="utf-8")
            self.status = "Saved %d waypoints" % len(self.waypoints)
            print("已保存 %d 个示教点到 %s" % (len(self.waypoints), path))
        except OSError as exc:
            self.status = "Save failed"
            print("保存示教点失败: %s" % exc)

    def load_waypoints(self) -> None:
        path = self.waypoint_file
        if not path.is_file():
            self.status = "No waypoint file"
            print("示教点文件不存在: %s" % path)
            return
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
            items = payload.get("waypoints", []) if isinstance(payload, dict) else payload
            loaded: List[Waypoint] = []
            for item in items:
                joints = [float(value) for value in item["joints_deg"]]
                if len(joints) != len(self.joints):
                    raise ValueError("关节数量不匹配: 文件 %d 个, 模型 %d 个"
                                     % (len(joints), len(self.joints)))
                loaded.append(Waypoint(
                    joints_deg=joints,
                    tcp_xyz=[float(v) for v in item.get("tcp_xyz", [])],
                    tcp_rpy=[float(v) for v in item.get("tcp_rpy", [])],
                ))
        except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
            self.status = "Load failed"
            print("读取示教点失败: %s" % exc)
            return

        self.waypoints = loaded
        self.play_queue.clear()
        self.play_active = False
        self._refresh_trail()
        self.status = "Loaded %d waypoints" % len(loaded)
        print("已读取 %d 个示教点: %s" % (len(loaded), path))

    # ------------------------------------------------------------------ #
    # 傅立叶激励轨迹（动力学参数识别）
    # ------------------------------------------------------------------ #
    def load_excitation(self, path) -> bool:
        """加载 excitation_trajectory.py 导出的 JSON。"""
        if FourierTrajectory is None:
            print("未找到 excitation_trajectory.py（或缺少 numpy），激励轨迹功能不可用")
            return False
        if not path:
            return False
        file_path = Path(path).expanduser()
        if not file_path.is_file():
            print("未找到激励轨迹 %s" % file_path)
            print("可先运行: python3 excitation_trajectory.py")
            return False
        try:
            payload = json.loads(file_path.read_text(encoding="utf-8"))
            trajectory = FourierTrajectory.from_payload(payload)
        except Exception as exc:                     # noqa: BLE001
            print("读取激励轨迹失败: %s" % exc)
            return False

        if trajectory.n_joints != len(self.joints):
            print("轨迹关节数 %d 与模型关节数 %d 不一致，已忽略"
                  % (trajectory.n_joints, len(self.joints)))
            return False
        names = payload.get("joint_names")
        if names and list(names) != [j["name"] for j in self.joints]:
            print("警告: 轨迹关节名与模型不一致，仍按关节顺序加载")
        payload_height = float(payload.get("base_height_m", 0.0))
        if abs(payload_height - self.base_height) > 1e-6:
            print("提示: 轨迹建议底座高度 %.3f m，当前使用 %.3f m"
                  % (payload_height, self.base_height))

        self.excitation = trajectory
        self.excitation_payload = payload
        self.exc_time = 0.0
        self.exc_cycle = 0
        self.exc_playing = False
        self.exc_desired = None
        self._ensure_exc_marker()
        self._refresh_excitation_path()

        metrics = payload.get("metrics", {})
        condition = metrics.get("condition_number")
        print("已加载激励轨迹: %s | %d 阶 | 周期 %.1f s | 条件数 %s"
              % (file_path.name, trajectory.harmonics, trajectory.period,
                 ("%.1f" % condition)
                 if isinstance(condition, (int, float)) else "n/a"))
        self.status = "Excitation loaded"
        return True

    def _ensure_exc_marker(self) -> None:
        if not self.gui or self.exc_marker >= 0:
            return
        visual = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.02, rgbaColor=[0.1, 0.9, 1.0, 0.9],
            physicsClientId=self.cid)
        self.exc_marker = p.createMultiBody(
            0, -1, visual, physicsClientId=self.cid)

    def _refresh_excitation_path(self) -> None:
        self._remove_debug_items(self.exc_path_ids)
        if not self.gui or not self.trail_on or not self.excitation_payload:
            return
        points = self.excitation_payload.get("samples", {}).get("tcp_xyz", [])
        if len(points) < 2:
            return
        shift = self.base_height
        points = [[point[0], point[1], point[2] + shift] for point in points]
        closed = list(points) + [points[0]]
        for start, end in zip(closed, closed[1:]):
            self.exc_path_ids.append(p.addUserDebugLine(
                start, end, [0.10, 0.80, 1.00], lineWidth=2.0, lifeTime=0,
                physicsClientId=self.cid))

    def toggle_excitation(self) -> None:
        if self.excitation is None:
            self.status = "No excitation loaded"
            print("没有加载激励轨迹。请先运行: python3 excitation_trajectory.py")
            return
        if self.exc_playing:
            self.exc_playing = False
            self._sync_sliders()
            self.status = "Excitation paused"
            print("激励轨迹已暂停（e 继续，Shift+E 停止并复位）")
            return
        self.play_queue.clear()
        self.play_active = False
        self.exc_playing = True
        self.status = "Excitation playing"
        print("开始播放激励轨迹：%d 阶傅立叶，周期 %.1f s"
              % (self.excitation.harmonics, self.excitation.period))

    def stop_excitation(self, reset: bool = True) -> None:
        was_active = self.exc_playing
        self.exc_playing = False
        if reset:
            self.exc_time = 0.0
            self.exc_cycle = 0
        self._sync_sliders()
        if was_active:
            self.status = "Excitation stopped"

    def _sync_sliders(self) -> None:
        """让滑条读数与当前状态对齐，避免下次拖动时突变。"""
        if self.gui and self.sliders:
            self.last_slider = [
                p.readUserDebugParameter(sid, physicsClientId=self.cid)
                for sid in self.sliders
            ]

    def update_excitation(self, dt: float) -> None:
        if not self.exc_playing or self.excitation is None:
            return
        period = self.excitation.period
        self.exc_time += dt * self.speed_scale
        if self.exc_time >= period:
            self.exc_time %= period
            self.exc_cycle += 1
        q, _, _ = self.excitation.evaluate([self.exc_time])
        desired = [float(value) for value in q[0]]
        self.exc_desired = desired
        for k, joint in enumerate(self.joints):
            self.targets[k] = clamp(desired[k], joint["lower"], joint["upper"])
        self._move_exc_marker()

    def _move_exc_marker(self) -> None:
        if not self.gui or self.exc_marker < 0 or not self.excitation_payload:
            return
        points = self.excitation_payload.get("samples", {}).get("tcp_xyz", [])
        if not points:
            return
        count = len(points)
        position = self.exc_time / self.excitation.period * count
        index = int(position) % count
        nxt = (index + 1) % count
        ratio = position - int(position)
        xyz = [a + (b - a) * ratio
               for a, b in zip(points[index], points[nxt])]
        xyz[2] += self.base_height
        p.resetBasePositionAndOrientation(
            self.exc_marker, xyz, [0.0, 0.0, 0.0, 1.0], physicsClientId=self.cid)

    def print_status(self) -> None:
        positions = [value * RAD for value in self.get_joint_positions()]
        position, rpy, _ = self.tcp_pose()
        print("-" * 66)
        print("关节角 (deg): " + "  ".join(
            "J%d=%+7.2f" % (k + 1, value) for k, value in enumerate(positions)))
        print("TCP (m): x=%+.4f  y=%+.4f  z=%+.4f" % tuple(position))
        print("TCP RPY (deg): r=%+.2f  p=%+.2f  y=%+.2f"
              % tuple(value * RAD for value in rpy))
        print("示教点: %d  当前关节: J%d  速度倍率: x%.2f"
              % (len(self.waypoints), self.active_joint + 1, self.speed_scale))
        print("-" * 66)

    # ------------------------------------------------------------------ #
    # 可视化辅助
    # ------------------------------------------------------------------ #
    def _remove_debug_items(self, ids: List[int]) -> None:
        for item in ids:
            try:
                p.removeUserDebugItem(item, physicsClientId=self.cid)
            except p.error:
                pass
        ids.clear()

    def toggle_axes(self) -> None:
        self.axes_on = not self.axes_on
        self._draw_axes()
        self.status = "Joint axes ON" if self.axes_on else "Joint axes OFF"

    def _draw_axes(self) -> None:
        self._remove_debug_items(self.axes_ids)
        if not self.gui or not self.axes_on:
            return
        for joint in self.joints:
            targets = ([AXIS_LENGTH, 0.0, 0.0],
                       [0.0, AXIS_LENGTH, 0.0],
                       [0.0, 0.0, AXIS_LENGTH])
            for direction, color in zip(targets, AXIS_COLORS):
                self.axes_ids.append(p.addUserDebugLine(
                    [0.0, 0.0, 0.0], direction, color,
                    lineWidth=2.5, lifeTime=0,
                    parentObjectUniqueId=self.robot,
                    parentLinkIndex=joint["index"],
                    physicsClientId=self.cid,
                ))

    def toggle_trail(self) -> None:
        self.trail_on = not self.trail_on
        self._refresh_trail()
        self._refresh_excitation_path()
        self.status = "TCP trail ON" if self.trail_on else "TCP trail OFF"

    def _refresh_trail(self) -> None:
        self._remove_debug_items(self.trail_ids)
        if not self.gui or not self.trail_on:
            return
        points = [list(wp.tcp_xyz) for wp in self.waypoints
                  if len(wp.tcp_xyz) == 3]
        if not points:
            return
        for start, end in zip(points, points[1:]):
            self.trail_ids.append(p.addUserDebugLine(
                start, end, [1.0, 0.55, 0.10], lineWidth=3, lifeTime=0,
                physicsClientId=self.cid))
        for index, point in enumerate(points, start=1):
            label_position = [point[0], point[1], point[2] + 0.04]
            self.trail_ids.append(p.addUserDebugText(
                str(index), label_position, [1.0, 0.85, 0.20],
                textSize=0.9, lifeTime=0, physicsClientId=self.cid))

    def _update_readout(self) -> None:
        if not self.gui:
            return
        positions = [value * RAD for value in self.get_joint_positions()]
        position, rpy, _ = self.tcp_pose()

        half = (len(positions) + 1) // 2
        row1 = "  ".join("J%d %+7.2f" % (i + 1, value)
                         for i, value in enumerate(positions[:half]))
        row2 = "  ".join("J%d %+7.2f" % (i + half + 1, value)
                         for i, value in enumerate(positions[half:]))
        lines = [
            "GCR30-1100  TEACH PENDANT",
            row1,
            row2,
            "TCP  x%+.3f  y%+.3f  z%+.3f m" % tuple(position),
            "RPY  r%+6.1f  p%+6.1f  y%+6.1f deg"
            % tuple(value * RAD for value in rpy),
            "J%d selected | points %d | speed x%.1f%s"
            % (self.active_joint + 1, len(self.waypoints), self.speed_scale,
               "  | PLAYING" if self.play_active else ""),
            self.status,
        ]
        if self.excitation is not None:
            line = ("EXC %5.2f/%4.1fs cyc%d %s"
                    % (self.exc_time, self.excitation.period,
                       self.exc_cycle + 1,
                       "PLAY" if self.exc_playing else "PAUSE"))
            if self.base_height > 1e-4:
                line += " | base +%.2fm" % self.base_height
            if self.exc_desired is not None:
                actual = self.get_joint_positions()
                error = max(abs(a - b) for a, b in
                            zip(actual, self.exc_desired)) * RAD
                line += " | err %.1fdeg" % error
            lines.insert(-1, line)
        self.readout_id = p.addUserDebugText(
            "\n".join(lines), [0.0, 0.0, 1.62],
            textColorRGB=[1.0, 1.0, 1.0], textSize=1.1, lifeTime=0,
            replaceItemUniqueId=self.readout_id,
            physicsClientId=self.cid)

    # ------------------------------------------------------------------ #
    # 键盘 / 主循环
    # ------------------------------------------------------------------ #
    def handle_keys(self) -> None:
        if not self.gui:
            return
        events = p.getKeyboardEvents(physicsClientId=self.cid)
        for code, state in events.items():
            if not (state & p.KEY_WAS_TRIGGERED):
                continue
            if code in (ord("q"), 27):                       # q / Esc
                self.running = False
            elif ord("1") <= code <= ord("9"):
                index = code - ord("1")
                if index < len(self.joints):
                    self.active_joint = index
                    self.status = "J%d selected (%s)" % (
                        index + 1, self.joints[index]["name"])
            elif code == ord(","):
                self.jog_joint(-JOYSTICK_STEP_DEG)
            elif code == ord("."):
                self.jog_joint(+JOYSTICK_STEP_DEG)
            elif code == ord("<"):
                self.jog_joint(-5.0 * JOYSTICK_STEP_DEG)
            elif code == ord(">"):
                self.jog_joint(+5.0 * JOYSTICK_STEP_DEG)
            elif code == ord("0"):
                self.zero_pose()
            elif code in (ord("h"), ord("H")):
                self.demo_pose()
            elif code in (ord("r"), ord("R")):
                self.random_pose()
            elif code == ord(" "):
                self.record_waypoint()
            elif code in (13, 10):                            # Enter
                self.toggle_playback()
            elif code in (ord("e"), ord("E")):
                if code == ord("E"):
                    self.stop_excitation()
                    print("激励轨迹已停止并复位到 t=0")
                else:
                    self.toggle_excitation()
            elif code in (ord("c"), ord("C")):
                self.clear_waypoints()
            elif code in (ord("s"), ord("S")):
                self.save_waypoints()
            elif code in (ord("l"), ord("L")):
                self.load_waypoints()
            elif code in (ord("p"), ord("P")):
                self.print_status()
            elif code in (ord("x"), ord("X")):
                self.toggle_axes()
            elif code in (ord("t"), ord("T")):
                self.toggle_trail()
            elif code in (ord("v"), ord("V")):
                self.reset_camera()

    def command_motors(self, dt: float) -> None:
        """按关节最大速度做指令限速，再交给位置控制器跟踪。"""
        for k, joint in enumerate(self.joints):
            max_step = joint["max_vel"] * self.speed_scale * dt
            delta = self.targets[k] - self.commands[k]
            self.commands[k] += clamp(delta, -max_step, max_step)
            p.setJointMotorControl2(
                self.robot, joint["index"], p.POSITION_CONTROL,
                targetPosition=self.commands[k], force=joint["force"],
                positionGain=0.6, velocityGain=1.0,
                maxVelocity=joint["max_vel"] * self.speed_scale,
                physicsClientId=self.cid,
            )

    def step(self) -> None:
        self.update_from_sliders()
        self.update_from_buttons()
        self.update_playback(FRAME_DT)
        self.update_excitation(FRAME_DT)
        self.command_motors(FRAME_DT)
        for _ in range(SUBSTEPS):
            p.stepSimulation(physicsClientId=self.cid)
        self._update_readout()

    def run(self) -> None:
        self.running = True
        frame = 0
        next_time = time.perf_counter()
        while self.running:
            try:
                if not p.isConnected(self.cid):
                    break
                self.handle_keys()
                self.step()
            except p.error as exc:
                print("PyBullet 运行结束: %s" % exc)
                break
            frame += 1
            if self.args.frames and frame >= self.args.frames:
                print("已运行 %d 帧，自动退出" % frame)
                break
            next_time += FRAME_DT
            delay = next_time - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_time = time.perf_counter()

    def close(self) -> None:
        try:
            if self.cid >= 0 and p.isConnected(self.cid):
                p.disconnect(physicsClientId=self.cid)
        except p.error:
            pass
        self.cid = -1
        if self._temp_dir is not None:
            shutil.rmtree(self._temp_dir, ignore_errors=True)
            self._temp_dir = None

    # ------------------------------------------------------------------ #
    # 无窗口自检（--direct）
    # ------------------------------------------------------------------ #
    def selftest(self) -> bool:
        print("\n===== DIRECT 模式自检 =====")
        failures: List[str] = []

        def check(name: str, condition: bool, detail: str = "") -> None:
            print("  [%s] %s%s" % ("PASS" if condition else "FAIL", name,
                                   ("  (%s)" % detail) if detail else ""))
            if not condition:
                failures.append(name)

        check("模型加载", self.robot >= 0, "body id %d" % self.robot)
        base_z = p.getBasePositionAndOrientation(
            self.robot, physicsClientId=self.cid)[0][2]
        com_offset = p.getDynamicsInfo(
            self.robot, -1, physicsClientId=self.cid)[3][2]
        check("底座抬高", abs(base_z - (self.base_height + com_offset)) < 1e-6,
              "安装面 z=%.3f m，质心 z=%.3f m" % (self.base_height, base_z))
        check("可动关节", len(self.joints) > 0,
              "%d 个: %s" % (len(self.joints),
                             ", ".join(j["name"] for j in self.joints)))

        mesh_refs = [mesh.get("filename", "")
                     for mesh in ET.parse(self.urdf_path).getroot().iter("mesh")]
        missing = [name for name in mesh_refs
                   if name.startswith("/") and not Path(name).is_file()]
        check("网格文件齐全", not missing,
              "%d 个引用, %d 个缺失" % (len(mesh_refs), len(missing)))

        count = len(self.joints)
        target = [10.0, -20.0, 30.0, -15.0, 25.0, 40.0][:count]
        self.set_targets_deg(target, "selftest")
        for _ in range(int(3.0 / FRAME_DT)):
            self.step()
        actual = [value * RAD for value in self.get_joint_positions()]
        error = max(abs(a - b) for a, b in zip(actual, target))
        check("关节跟随误差 < 0.5°", error < 0.5, "最大 %.3f°" % error)

        position, rpy, _ = self.tcp_pose()
        valid = all(math.isfinite(v) for v in list(position) + list(rpy))
        check("TCP 位姿有效", valid,
              "xyz=(%.3f, %.3f, %.3f) rpy=(%.1f, %.1f, %.1f)°"
              % (position[0], position[1], position[2],
                 rpy[0] * RAD, rpy[1] * RAD, rpy[2] * RAD))

        self.clear_waypoints()
        self.set_targets_deg([0.0] * count)
        for _ in range(int(2.0 / FRAME_DT)):
            self.step()
        self.record_waypoint()
        second = [0.0, -20.0, 35.0, 0.0, 15.0, 30.0][:count]
        self.set_targets_deg(second)
        for _ in range(int(3.0 / FRAME_DT)):
            self.step()
        self.record_waypoint()
        check("记录示教点", len(self.waypoints) == 2,
              "%d 个点" % len(self.waypoints))

        self.toggle_playback()
        frames = 0
        limit = int(20.0 / FRAME_DT)
        while self.play_active and frames < limit:
            self.step()
            frames += 1
        check("回放结束", not self.play_active,
              "耗时 %.1f s" % (frames * FRAME_DT))

        for _ in range(int(1.0 / FRAME_DT)):
            self.step()
        final = [value * RAD for value in self.get_joint_positions()]
        error = max(abs(a - b) for a, b in
                    zip(final, self.waypoints[-1].joints_deg))
        check("回放终点误差 < 0.5°", error < 0.5, "最大 %.3f°" % error)

        original = self.waypoint_file
        temp_dir = Path(tempfile.mkdtemp(prefix="teach_wp_"))
        self.waypoint_file = temp_dir / "waypoints.json"
        self.save_waypoints()
        saved = len(self.waypoints)
        self.waypoints = []
        self.load_waypoints()
        check("JSON 保存/读取", len(self.waypoints) == saved and saved > 0,
              "往返 %d 个点" % len(self.waypoints))
        self.waypoint_file = original
        shutil.rmtree(temp_dir, ignore_errors=True)

        if self.excitation is not None:
            period = self.excitation.period
            t_check = [period * i / 200.0 for i in range(200)]
            q_check, qd_check, _ = self.excitation.evaluate(t_check)
            inside = all(
                joint["lower"] - 1e-6 <= float(value) <= joint["upper"] + 1e-6
                for k, joint in enumerate(self.joints)
                for value in q_check[:, k])
            speed_ok = all(
                abs(float(value)) <= joint["max_vel"] * 1.02
                for k, joint in enumerate(self.joints)
                for value in qd_check[:, k])
            check("激励轨迹满足关节限位", inside)
            check("激励轨迹满足速度限位", speed_ok)

            self.stop_excitation()
            self.toggle_excitation()
            for _ in range(int(4.0 / FRAME_DT)):
                self.step()
            if self.exc_desired is not None:
                actual = self.get_joint_positions()
                error = max(abs(a - b) for a, b in
                            zip(actual, self.exc_desired)) * RAD
                check("激励轨迹跟踪 < 1.5°", error < 1.5,
                      "最大 %.2f°" % error)
            self.stop_excitation()
        else:
            print("  [SKIP] 未加载傅立叶激励轨迹")

        print("===== 自检结果: %s =====\n"
              % ("全部通过" if not failures else "失败 %d 项" % len(failures)))
        return not failures


# --------------------------------------------------------------------------- #
# 命令行入口
# --------------------------------------------------------------------------- #
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="GCR30-1100 PyBullet 示教器（关节滑条 / 示教点记录回放 / TCP 位姿显示）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--urdf", default=str(DEFAULT_URDF),
                        help="URDF 文件路径")
    parser.add_argument("--waypoints", default=str(DEFAULT_WAYPOINT_FILE),
                        help="示教点 JSON 文件路径")
    parser.add_argument("--trajectory", default=str(DEFAULT_TRAJECTORY),
                        help="傅立叶激励轨迹 JSON（启动时自动加载）")
    parser.add_argument("--no-trajectory", action="store_true",
                        help="启动时不加载激励轨迹")
    parser.add_argument("--base-height", type=float, default=None,
                        help="底座抬高量 (m)；默认读取轨迹 JSON 的建议值，0 表示不抬高")
    parser.add_argument("--direct", action="store_true",
                        help="无窗口 DIRECT 模式，运行自检后退出")
    parser.add_argument("--zero-gravity", action="store_true",
                        help="关闭重力（纯运动学查看模型）")
    parser.add_argument("--show-axes", action="store_true",
                        help="启动时显示关节坐标轴")
    parser.add_argument("--frames", type=int, default=0,
                        help="GUI 运行指定帧数后自动退出（0 = 不限制），便于冒烟测试")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    pendant = TeachPendant(args)
    try:
        pendant.setup()
        if args.direct:
            return 0 if pendant.selftest() else 1
        pendant.run()
        return 0
    except KeyboardInterrupt:
        print("\n已退出示教器")
        return 0
    finally:
        pendant.close()


if __name__ == "__main__":
    sys.exit(main())
