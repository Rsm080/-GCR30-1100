#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""为每个关节生成“关节角度—时间”曲线（共 6 张 PNG）。

数据来源是 excitation_trajectory.py 导出的傅立叶激励轨迹 JSON；
优先用傅立叶系数解析求值，缺少系数时退回 JSON 中的采样数据。

用法：
    python3 plot_joint_motion.py
    python3 plot_joint_motion.py --json trajectories/gcr30_1100_fourier5.json \
        --outdir trajectories/joint_plots
    python3 plot_joint_motion.py --with-velocity      # 同时在副轴画关节速度
    python3 plot_joint_motion.py --samples 2000 --dpi 200

输出（默认）：
    trajectories/joint_plots/gcr30_1100_fourier5_joint1.png
    ...                                           _joint2.png ... _joint6.png
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent
DEFAULT_JSON = REPO_ROOT / "trajectories" / "gcr30_1100_fourier5.json"
DEFAULT_OUTDIR = REPO_ROOT / "trajectories" / "joint_plots"

RAD = 180.0 / math.pi


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="按关节生成关节角度—时间曲线（6 张图）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--json", default=str(DEFAULT_JSON),
                        help="傅立叶激励轨迹 JSON")
    parser.add_argument("--outdir", default=str(DEFAULT_OUTDIR),
                        help="输出图片目录")
    parser.add_argument("--prefix", default="",
                        help="输出文件名前缀，默认取 JSON 文件名")
    parser.add_argument("--samples", type=int, default=1000,
                        help="每条曲线的采样点数")
    parser.add_argument("--dpi", type=int, default=150,
                        help="图片分辨率")
    parser.add_argument("--with-velocity", action="store_true",
                        help="在右侧副轴同时绘制角速度 (deg/s)")
    return parser


def _payload_samples(payload: dict, count: int):
    """从 JSON 采样数据回退求值（无傅立叶系数时使用）。"""
    samples = payload["samples"]
    t = np.asarray(samples["t"], float)
    q = np.asarray(samples["q_deg"], float)
    qd = np.asarray(samples.get("qd_rad_s", np.zeros_like(q)), float)
    if len(t) != count:
        grid = np.linspace(t[0], t[-1], count)
        q = np.vstack([np.interp(grid, t, q[:, k])
                       for k in range(q.shape[1])]).T
        qd = np.vstack([np.interp(grid, t, qd[:, k])
                        for k in range(qd.shape[1])]).T
        t = grid
    return t, q, qd


def load_curves(payload: dict, count: int):
    """返回 (t, q_deg, qd_deg_s, trajectory_or_None)。"""
    try:
        from excitation_trajectory import FourierTrajectory
        trajectory = FourierTrajectory.from_payload(payload)
        t = np.linspace(0.0, trajectory.period, count)
        q, qd, _ = trajectory.evaluate(t)
        return t, q * RAD, qd * RAD, trajectory
    except Exception as exc:                         # noqa: BLE001
        print("未能解析傅立叶系数（%s），改用 JSON 采样数据" % exc)
        t, q, qd = _payload_samples(payload, count)
        return t, q, qd * RAD, None


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    json_path = Path(args.json).expanduser()
    if not json_path.is_file():
        raise SystemExit("找不到轨迹 JSON: %s" % json_path)

    payload = json.loads(json_path.read_text(encoding="utf-8"))
    count = max(int(args.samples), 10)
    t, q_deg, qd_deg, trajectory = load_curves(payload, count)

    names = payload.get("joint_names") or \
        ["J%d" % (k + 1) for k in range(q_deg.shape[1])]
    limits = payload.get("limits", {}).get("position_deg", [])
    q0 = np.asarray(payload.get("q0_deg", np.zeros(q_deg.shape[1])), float)
    if trajectory is not None:
        period = trajectory.period
    else:
        period = float(payload.get("period_s",
                                   t[-1] - t[0] if len(t) > 1 else 0.0))

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as exc:                         # noqa: BLE001
        raise SystemExit("需要 matplotlib 才能绘图: %s" % exc)

    outdir = Path(args.outdir).expanduser()
    outdir.mkdir(parents=True, exist_ok=True)
    prefix = args.prefix or json_path.stem

    print("关节角度曲线：%s" % json_path)
    print("采样 %d 点 | 周期 %.2f s | 输出目录 %s"
          % (len(t), period, outdir))

    outputs = []
    for k in range(q_deg.shape[1]):
        figure, axis = plt.subplots(figsize=(8.6, 4.6))
        axis.plot(t, q_deg[:, k], color="#1f77b4", linewidth=1.8,
                  label="q(t)")
        axis.axhline(q0[k], color="#2ca02c", linestyle=":", linewidth=1.0,
                     alpha=0.8, label="q0")

        if k < len(limits):
            lower, upper = float(limits[k][0]), float(limits[k][1])
            axis.axhline(lower, color="#d62728", linestyle="--",
                         linewidth=1.0, alpha=0.7, label="joint limit")
            axis.axhline(upper, color="#d62728", linestyle="--",
                         linewidth=1.0, alpha=0.7)

        axis.set_xlabel("time (s)")
        axis.set_ylabel("joint angle (deg)")
        title = "Joint %d  (%s)  -  angle vs time" % (k + 1, names[k])
        if period > 0.0:
            title += "   [period %.1f s]" % period
        axis.set_title(title, fontsize=11)
        axis.grid(True, alpha=0.3)
        axis.set_xlim(t[0], t[-1])

        if args.with_velocity:
            twin = axis.twinx()
            twin.plot(t, qd_deg[:, k], color="#ff7f0e", linewidth=1.0,
                      alpha=0.75, label="qd(t)")
            twin.set_ylabel("joint velocity (deg/s)")
            handles, labels = axis.get_legend_handles_labels()
            handles2, labels2 = twin.get_legend_handles_labels()
            axis.legend(handles + handles2, labels + labels2,
                        loc="upper right", fontsize=8)
        else:
            axis.legend(loc="upper right", fontsize=8)

        figure.tight_layout()
        path = outdir / ("%s_joint%d.png" % (prefix, k + 1))
        figure.savefig(path, dpi=args.dpi)
        plt.close(figure)
        outputs.append(path)

        print("  J%d %-14s q ∈ [%+7.2f, %+7.2f] deg  ->  %s"
              % (k + 1, names[k], float(q_deg[:, k].min()),
                 float(q_deg[:, k].max()), path.name))

    print("已生成 %d 张关节角度图。" % len(outputs))
    return 0


if __name__ == "__main__":
    sys.exit(main())
