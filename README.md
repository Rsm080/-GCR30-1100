# GCR30-1100 机器人仿真与动力学参数辨识工具

[**简体中文**](README.md) | [English](README.en.md)

基于 **DUCO / 新松-杜科 GCR30-1100** 六轴协作机器人 URDF 的 PyBullet 示教器、五阶傅立叶激励轨迹设计与动力学参数辨识工具集。

- 机器人：GCR30-1100（30 kg 负载，6 个旋转关节，TCP 参考坐标系 `link_6`）
- 仿真：PyBullet
- 功能：模型查看、关节示教、轨迹回放、激励轨迹生成、回归矩阵/条件数优化、离地间隙检查
- 原 ROS2 驱动说明：[README.en.md](README.en.md)

## 目录结构

```text
GCR30-1100/
├── teach_pendant.py                 # PyBullet 示教器（GUI / DIRECT 自检）
├── excitation_trajectory.py         # 五阶傅立叶激励轨迹设计器
├── plot_joint_motion.py             # 生成 6 张单关节角度—时间图
├── trajectories/
│   ├── gcr30_1100_fourier5.json     # 轨迹（示教器读取）
│   ├── gcr30_1100_fourier5.csv      # q / qd / qdd 采样
│   ├── gcr30_1100_fourier5.npz      # 回归矩阵 W、pi_true、参数名
│   ├── gcr30_1100_fourier5.png      # 关节/TCP 轨迹总览
│   └── joint_plots/                 # 6 张单关节角度图
├── src/duco_support/
│   ├── urdf/duco_gcr30_1100.urdf    # GCR30-1100 URDF
│   ├── meshes/GCR30_1100/           # 7 个 visual STL
│   ├── collision/GCR30_1100/        # 7 个 collision STL
│   └── ...
├── README.md                        # 简体中文（本文件）
└── README.en.md                     # 英文说明（原 ROS2 驱动文档）
```

## 环境要求

- Python 3.10 及以上
- `pybullet`、`numpy`、`scipy`、`matplotlib`

```bash
pip install pybullet numpy scipy matplotlib
```

- 运行 GUI 示教器需要显示器（`DISPLAY`）；
- 无显示器时可用 `--direct` 运行自检，或用离屏渲染。

## 快速开始

```bash
# 仓库名以 "-" 开头，克隆时建议显式指定目录名，避免 cd 参数歧义
git clone git@github.com:Rsm080/-GCR30-1100.git GCR30-1100
cd GCR30-1100

# 无窗口自检（检查模型、网格、示教点、激励轨迹、限位与跟踪）
python3 teach_pendant.py --direct

# 打开 GUI 示教器
python3 teach_pendant.py
```

正常启动后会看到：地面网格、GCR30-1100 模型、可选的圆柱垫台、关节滑条与示教按钮；终端会打印模型信息与快捷键。
