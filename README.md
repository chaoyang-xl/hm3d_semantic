# HM3D RGB-D 语义重建数据生成工具

基于 [Habitat-Sim](https://github.com/facebookresearch/habitat-sim) 的 HM3D 室内场景数据采集工具。项目可以在 Habitat 可导航区域内自动运动、键盘遥控或回放已有轨迹，并同步导出 RGB、米制深度、语义实例 ID 和相机真值位姿。

本仓库只负责**仿真数据生成与数据集校验**，不直接依赖 ROS、SLAM、YOLO、SAM 或对象跟踪框架。生成的数据兼容 Replica 风格的离线 RGB-D 处理流程，可交给外部 `semantic_map_offline` 包完成 YOLO-World、MobileSAM、点云投影、对象关联和语义地图生成。

## 目录

- [主要功能](#主要功能)
- [处理流程](#处理流程)
- [项目结构](#项目结构)
- [环境要求](#环境要求)
- [安装](#安装)
- [准备 HM3D 数据](#准备-hm3d-数据)
- [快速开始](#快速开始)
- [三种轨迹模式](#三种轨迹模式)
- [导出参数](#导出参数)
- [输出数据格式](#输出数据格式)
- [坐标系约定](#坐标系约定)
- [数据校验](#数据校验)
- [语义点云重建](#语义点云重建)
- [测试](#测试)
- [常见问题](#常见问题)
- [Git 提交说明](#git-提交说明)
- [参考资料](#参考资料)

## 主要功能

- 同步生成 RGB、深度、语义实例图和相机真值位姿。
- 默认输出分辨率为 `640 x 480`，可通过命令行修改。
- 深度以 `uint16` 毫米保存，零值表示无效或超出配置范围。
- 支持 `waypoint`、`interactive` 和 `replay` 三种轨迹来源。
- Waypoint 模式先原地对准运动方向，再沿 NavMesh 平移，避免侧移。
- 交互模式提供大尺寸实时窗口、完整键位提示和当前状态信息。
- Waypoint 和 Interactive 的平移通过 Habitat `PathFinder.try_step()` 约束，不穿墙。
- 导出过程使用 `.partial` 临时目录，校验成功后再发布正式数据。
- 保留 Habitat Y-up 原始 GT，同时生成下游建图使用的 Z-up `traj.txt`。
- 提供独立校验器，检查帧数、分辨率、深度类型、位姿和语义 ID。
- 可选生成 RGB、深度、语义预览图和轨迹俯视图。

## 处理流程

```text
HM3D 场景与 NavMesh
        |
        v
Habitat-Sim 同步传感器
 RGB + Depth + Semantic
        |
        v
Waypoint / Keyboard / Replay
        |
        v
OpenCV 光学坐标相机位姿
        |
        v
事务式数据集导出
        |
        v
严格校验与预览
        |
        +------------------------------+
                                       v
                         YOLO-World + MobileSAM
                                       |
                                       v
                         语义对象 PLY / NPZ / JSON
```

## 项目结构

```text
.
├── hm3d_reconstruction/
│   ├── config.py          # 参数定义和合法性检查
│   ├── coordinate.py      # OpenCV、Habitat 和 Z-up 坐标转换
│   ├── dataset.py         # RGB-D、语义图、轨迹文件读写
│   ├── exporter.py        # 事务式数据集导出
│   ├── simulator.py       # Habitat-Sim、运动逻辑和交互窗口
│   ├── trajectory.py      # 航向角、四元数和轨迹回放
│   ├── validator.py       # 独立数据集校验
│   └── visualization.py   # 预览图和轨迹图
├── scripts/
│   ├── inspect_scene.py
│   ├── export_hm3d_dataset.py
│   └── validate_dataset.py
├── tests/                 # 单元测试与可选真实场景集成测试
├── environment.yml
└── pyproject.toml
```

## 环境要求

推荐配置：

- Linux。
- Conda 或 Miniconda。
- Python 3.9。
- Habitat-Sim 0.3.3。
- NumPy 1.23 及以上。
- Pillow 9 及以上。
- 带显示器或桌面会话的环境，用于 `interactive` 模式。

本项目已在以下组合验证：

```text
Python       3.9.25
Habitat-Sim  0.3.3
NumPy        1.26.4
Pillow       11.3.0
```

### 图形版和 Headless 版

交互式键盘采集需要 Tk 窗口和可用的 `DISPLAY`，因此应安装**非 headless** Habitat-Sim。Waypoint 和 Replay 可在服务器上使用 headless 版本，但不能打开交互窗口。

Habitat-Sim 官方 Conda 包支持 `withbullet`、`headless` 等构建特性。不要在计划使用键盘控制的环境中加入 `headless`。

## 安装

### 方法一：使用仓库环境文件

```bash
conda env create -f environment.yml
conda activate hm3d_reconstruction
```

如果环境已经存在：

```bash
conda env update -n hm3d_reconstruction -f environment.yml --prune
conda activate hm3d_reconstruction
```

### 方法二：手动创建图形环境

```bash
conda create -n hm3d_reconstruction python=3.9 -y
conda activate hm3d_reconstruction

conda install habitat-sim=0.3.3 withbullet \
  -c conda-forge -c aihabitat

python -m pip install -e ".[test]"
```

### Headless 环境

仅用于 Waypoint 或 Replay：

```bash
conda create -n hm3d_reconstruction_headless python=3.9 -y
conda activate hm3d_reconstruction_headless

conda install habitat-sim=0.3.3 withbullet headless \
  -c conda-forge -c aihabitat

python -m pip install -e ".[test]"
```

### 验证安装

```bash
python -c "import habitat_sim; print(\"Habitat-Sim import OK\")"
python -m pytest -q
```

## 准备 HM3D 数据

HM3D 场景通常按以下方式组织：

```text
HM3D_ROOT/
├── hm3d-minival-habitat-v0.2/
│   ├── 00804-BHXhpBwSMLh/
│   │   ├── BHXhpBwSMLh.basis.glb
│   │   ├── BHXhpBwSMLh.basis.navmesh
│   │   ├── BHXhpBwSMLh.semantic.glb
│   │   └── BHXhpBwSMLh.semantic.txt
│   └── ...
└── hm3d-minival-semantic-configs-v0.2/
    ├── hm3d_annotated_basis.scene_dataset_config.json
    └── hm3d_annotated_minival_basis.scene_dataset_config.json
```

至少需要：

- `.basis.glb`：场景渲染资产。
- `.basis.navmesh`：机器人可导航区域。
- `.scene_dataset_config.json`：Habitat 场景数据集配置。
- `.semantic.glb` 和 `.semantic.txt`：仅在保存 Habitat GT 语义时需要。

HM3D 大文件已被 `.gitignore` 排除，不要提交到 Git 仓库。

### 设置示例路径

以下命令以本机的 `00804-BHXhpBwSMLh` 为例：

```bash
export HM3D_ROOT=/home/weiyu/hm3d_semantic_dataset

export HM3D_SCENE=$HM3D_ROOT/hm3d-minival-habitat-v0.2/00804-BHXhpBwSMLh/BHXhpBwSMLh.basis.glb

export HM3D_CONFIG=$HM3D_ROOT/hm3d-minival-semantic-configs-v0.2/hm3d_annotated_basis.scene_dataset_config.json
```

换场景时只需要修改 `HM3D_SCENE`。如果使用不同数据划分，也要切换到与之匹配的 scene dataset 配置。

## 快速开始

### 1. 检查场景

```bash
python scripts/inspect_scene.py \
  --scene "$HM3D_SCENE" \
  --scene-dataset-config "$HM3D_CONFIG" \
  --output outputs/00804_scene_check \
  --no-save-semantic
```

检查成功后会输出：

- NavMesh 是否加载成功。
- 可导航面积。
- 语义实例数量；关闭语义时为 `0`。
- RGB、深度和可选语义传感器尺寸。

如果场景具备完整语义资产，可去掉 `--no-save-semantic`。场景没有语义标注时，场景检查和数据导出都必须保留该参数。

### 2. 自动采集一份数据

```bash
python scripts/export_hm3d_dataset.py \
  --scene "$HM3D_SCENE" \
  --scene-dataset-config "$HM3D_CONFIG" \
  --output outputs/00804_waypoint \
  --trajectory-mode waypoint \
  --frames 500 \
  --width 640 \
  --height 480 \
  --forward-step 0.10 \
  --turn-angle-deg 5 \
  --alignment-tolerance-deg 10 \
  --seed 42 \
  --no-save-semantic \
  --preview
```

### 3. 校验结果

```bash
python scripts/validate_dataset.py \
  --data-root outputs/00804_waypoint \
  --sample-count 50 \
  --strict \
  --write-preview
```

程序返回码为 `0` 表示校验通过；返回码为 `1` 表示存在错误。

## 三种轨迹模式

三种模式保存相同的数据结构，只是机器人位姿的来源不同。

| 模式 | 轨迹来源 | `--frames` 含义 | 适用场景 |
| --- | --- | --- | --- |
| `waypoint` | Habitat 随机起点和最短路径 | 必须输出的准确帧数 | 自动批量采集 |
| `interactive` | 键盘遥控 | 最大记录帧数 | 人工覆盖指定房间 |
| `replay` | JSON 位置和四元数 | 从轨迹中读取的准确帧数 | 重复实验、参数对比 |

### Waypoint 自动模式

```bash
python scripts/export_hm3d_dataset.py \
  --scene "$HM3D_SCENE" \
  --scene-dataset-config "$HM3D_CONFIG" \
  --output outputs/00804_waypoint \
  --trajectory-mode waypoint \
  --frames 1000 \
  --width 640 --height 480 \
  --forward-step 0.10 \
  --turn-angle-deg 5 \
  --alignment-tolerance-deg 10 \
  --seed 42 \
  --no-save-semantic \
  --preview
```

运动过程：

1. 从 NavMesh 采样一个可导航起点。
2. 采样可到达目标点并计算 Habitat 最短路径。
3. 按 `forward-step` 对路径进行加密。
4. 航向误差大于容差时原地旋转。
5. 朝向基本对齐后通过 `try_step()` 向前平移。
6. 到达目标后继续采样新目标，直到满足帧数。

转弯帧也计入 `--frames`，因此同样的总帧数不代表同样的平移距离。

### Interactive 键盘模式

```bash
python scripts/export_hm3d_dataset.py \
  --scene "$HM3D_SCENE" \
  --scene-dataset-config "$HM3D_CONFIG" \
  --output outputs/00804_manual \
  --trajectory-mode interactive \
  --frames 5000 \
  --width 640 --height 480 \
  --display-scale 5.0 \
  --ui-scale 0 \
  --forward-step 0.10 \
  --turn-angle-deg 5 \
  --no-save-semantic \
  --preview
```

窗口启动后默认处于暂停状态：

| 按键 | 操作 |
| --- | --- |
| `W` | 沿当前朝向前进 |
| `S` | 沿当前朝向后退 |
| `A` | 原地左转 |
| `D` | 原地右转 |
| `R` | 开始或继续记录 |
| `P` | 暂停记录 |
| `Q` | 校验并保存为正式输出目录 |
| `Esc` | 停止并保留完整 `.partial` 数据 |

注意：

- 按下第一次 `R` 前可以自由寻找起点，这些预览移动不会写入数据。
- 暂停后仍可移动，恢复记录后从当前位置继续。
- 达到 `--frames` 上限时自动结束并保存。
- `Q` 只有在至少记录一帧后才生效。
- 关闭窗口等价于 `Esc`。
- `--display-scale` 只改变窗口中的图像大小，不改变保存分辨率。
- `--ui-scale 0` 根据屏幕高度自动缩放字体；看不清时可用 `--ui-scale 2.5`。

### Replay 回放模式

最简单的方式是回放一次已有导出的 `trajectory.json`：

```bash
python scripts/export_hm3d_dataset.py \
  --scene "$HM3D_SCENE" \
  --scene-dataset-config "$HM3D_CONFIG" \
  --output outputs/00804_replay \
  --trajectory-mode replay \
  --trajectory-file outputs/00804_manual/trajectory.json \
  --frames 422 \
  --width 640 --height 480 \
  --no-save-semantic \
  --preview
```

回放 JSON 支持以下结构：

```json
{
  "frames": [
    {
      "agent_position": [6.64, 0.15, 4.74],
      "agent_rotation_xyzw": [0.0, 0.0, 0.0, 1.0]
    }
  ]
}
```

也支持字段别名：

```json
[
  {
    "position": [6.64, 0.15, 4.74],
    "rotation_xyzw": [0.0, 0.0, 0.0, 1.0]
  }
]
```

要求：

- 位置和四元数必须为有限数值。
- 四元数顺序为 `x, y, z, w`。
- 每个位姿都必须位于当前场景 NavMesh 上。
- 轨迹状态数不能少于 `--frames`。
- 当前移动模型只使用四元数中的 yaw 航向。

## 导出参数

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `--scene` | 必填 | HM3D `.basis.glb` 文件 |
| `--scene-dataset-config` | 必填 | Habitat scene dataset 配置 |
| `--output` | 必填 | 最终输出目录，没有隐式默认路径 |
| `--frames` | `100` | Waypoint/Replay 的准确帧数；Interactive 的最大帧数 |
| `--width` | `640` | 保存图像宽度 |
| `--height` | `480` | 保存图像高度 |
| `--hfov-deg` | `79` | 相机水平视场角，范围 `(0, 180)` |
| `--sensor-height` | `0.88` | 相机相对机器人底部高度，单位米 |
| `--display-scale` | `5.0` | 交互预览图缩放，范围 `[0.5, 6.0]` |
| `--ui-scale` | `0` | UI 自动缩放；手动范围 `[0.8, 3.0]` |
| `--min-depth-m` | `0.05` | 最小有效深度，单位米 |
| `--max-depth-m` | `10.0` | 最大有效深度，单位米 |
| `--trajectory-mode` | `waypoint` | `waypoint`、`interactive` 或 `replay` |
| `--trajectory-file` | 无 | Replay 必需的 JSON 轨迹 |
| `--forward-step` | `0.10` | 单次前进或后退距离，单位米 |
| `--turn-angle-deg` | `5` | 单次旋转角度 |
| `--alignment-tolerance-deg` | `10` | Waypoint 平移前允许的航向误差 |
| `--seed` | `42` | Habitat、起点和路径随机种子 |
| `--save-semantic` | 开启 | 保存 Habitat GT 语义实例图 |
| `--no-save-semantic` | - | 关闭 Habitat GT 语义传感器 |
| `--preview` | 关闭 | 生成图像拼图和轨迹预览 |
| `--overwrite` | 关闭 | 事务式替换已有非空输出 |

查看命令行内置说明：

```bash
python scripts/export_hm3d_dataset.py --help
```

## 输出数据格式

```text
OUTPUT/
├── results/
│   ├── frame000000.jpg
│   ├── depth000000.png
│   └── ...
├── semantic/
│   ├── semantic000000.png
│   └── ...
├── pose_gt/
│   ├── 000000.txt
│   └── ...
├── preview/
│   ├── rgb_samples.jpg
│   ├── depth_samples.jpg
│   ├── semantic_samples.jpg
│   └── trajectory_topdown.png
├── cam_params.json
├── metadata.json
├── semantic_metadata.json
├── trajectory.json
├── traj_gt.txt
├── traj.txt
└── export_report.json
```

未启用语义或预览时，相应目录可能为空或不存在有效文件。

### RGB

- 路径：`results/frameXXXXXX.jpg`。
- 分辨率：由 `--width` 和 `--height` 决定。
- 像素布局：RGB。

### 深度

- 路径：`results/depthXXXXXX.png`。
- 类型：`uint16`。
- 单位：毫米。
- 换算：`depth_m = depth_uint16 / cam_params.camera.scale`。
- 默认有效范围：`0.05 m` 到 `10 m`。
- 无效、非有限或超范围值保存为 `0`。

### 语义实例

- 路径：`semantic/semanticXXXXXX.png`。
- 内容：Habitat 原始语义实例 ID，不是类别训练 ID。
- 映射：`semantic_metadata.json`。

使用 `--no-save-semantic` 时不会生成有效语义帧，后续可使用 YOLO-World 和 MobileSAM 预测语义。

### 相机内参

`cam_params.json` 示例：

```json
{
  "camera": {
    "cx": 319.5,
    "cy": 239.5,
    "fx": 388.191,
    "fy": 388.191,
    "h": 480,
    "scale": 1000.0,
    "w": 640
  }
}
```

### 位姿和轨迹

- `pose_gt/XXXXXX.txt`：单帧 Habitat Y-up 相机真值矩阵。
- `traj_gt.txt`：按每 4 行一个矩阵堆叠的 Habitat Y-up 真值轨迹。
- `traj.txt`：转换为 Z-up 地图坐标的下游轨迹。
- `trajectory.json`：机器人、传感器位姿、动作和碰撞状态。

所有矩阵都是相机到世界的刚体变换：

```text
P_world = T_world_camera P_camera
```

### 导出报告

`export_report.json` 包含：

- 实际和请求帧数。
- 无效深度比例。
- 语义元数据覆盖率。
- 碰撞次数。
- NavMesh 可导航面积。
- 轨迹总长度、平均步长和最大步长。
- 前进、后退、原地旋转帧数。
- 采集耗时。

采集模式、场景、帧数、坐标约定和停止原因保存在 `metadata.json`。

## 坐标系约定

### 相机局部坐标

保存的深度和相机局部点采用 OpenCV 光学坐标：

```text
X：向右
Y：向下
Z：向前
```

### Habitat GT 世界坐标

Habitat 世界坐标是右手系，`Y` 向上。`traj_gt.txt` 和 `pose_gt/` 保留该坐标系。

```text
T_habitat_camera =
T_habitat_sensor * diag(1, -1, -1, 1)
```

### 下游 Z-up 地图坐标

`semantic_map_offline` 的俯视图使用 `X-Y` 作为地面、`Z` 作为高度，因此 `traj.txt` 使用：

```text
x_map =  x_habitat
y_map = -z_habitat
z_map =  y_habitat
```

对应齐次变换：

```text
T_map_habitat =
  [1  0  0  0]
  [0  0 -1  0]
  [0  1  0  0]
  [0  0  0  1]
```

该矩阵行列式为 `+1`，是右手系刚体旋转，不是镜像变换。

不要直接把 Y-up `traj_gt.txt` 交给固定使用 XY 俯视投影的程序，否则会把高度轴当成地面轴，生成狭长的侧视图。

## 数据校验

```bash
python scripts/validate_dataset.py \
  --data-root outputs/00804_manual \
  --sample-count 50 \
  --strict \
  --write-preview
```

校验内容包括：

- RGB、深度、位姿和语义帧编号连续。
- 各模态帧数一致。
- 图像分辨率与内参一致。
- 深度图类型为 `uint16`。
- 位姿旋转正交、行列式为 1、齐次矩阵末行为 `[0, 0, 0, 1]`。
- `traj.txt` 是 `traj_gt.txt` 的正确 Z-up 变换。
- 单帧 `pose_gt` 与 `traj_gt.txt` 一致。
- 反投影结果为有限值。
- 相邻帧平移没有异常跳变。
- 语义实例 ID 可以在元数据中找到。

推荐在交给任何点云或学习管线前使用 `--strict`。

## 事务保存与 `.partial`

采集首先写入：

```text
OUTPUT.partial/
```

正常完成或 Interactive 中按 `Q` 后：

1. 重新打开并严格校验数据。
2. 校验通过后将 `.partial` 原子重命名为正式输出。
3. 如果使用 `--overwrite`，旧目录会在发布期间暂存为 `.previous`。

Interactive 已记录至少一帧后按 `Esc`：

- 数据仍会经过完整校验。
- 目录保留 `.partial` 后缀。
- 文件内容可用于检查和恢复，但不会冒充正式完成结果。

发生异常时，`.partial/failure_report.json` 会记录错误类型、消息、已写帧数、配置和 traceback。

## 语义点云重建

该步骤由外部 `semantic_map_offline` 包完成，不是本仓库的安装依赖。下面使用已经验证的 YOLO-World + MobileSAM 流程。

### 准备路径

```bash
export HM3D_EXPORT=/home/weiyu/vscode_workspace/hd3d_semantic/outputs/00804_manual

export SEMANTIC_MAP_OFFLINE=/home/weiyu/vscode_workspace/ros2_wp/src/semantic_map_offline

export YOLO_WORLD_MODEL=/home/weiyu/vscode_workspace/models/yolov8s-world.pt
export CLIP_MODEL=/home/weiyu/vscode_workspace/models/clip/ViT-B-32.pt
```

### 完整重建

```bash
cd "$SEMANTIC_MAP_OFFLINE"

conda run --no-capture-output -n opi_yolo_eval \
python scripts/evaluate_sam_projection_tracking.py \
  --data-root "$HM3D_EXPORT" \
  --model "$YOLO_WORLD_MODEL" \
  --clip-model "$CLIP_MODEL" \
  --classes-path config/class_list/gpt_indoor_general.txt \
  --output /home/weiyu/vscode_workspace/hd3d_semantic/outputs/00804_manual_reconstruction_sam \
  --start 0 \
  --frames 0 \
  --frame-step 1 \
  --confidence 0.5 \
  --device 0 \
  --pose-convention replica \
  --pixel-stride 2 \
  --voxel-size 0.02 \
  --overlap-radius 0.04 \
  --min-confirmed-observations 8 \
  --sam-checkpoint MobileSAM/weights/mobile_sam.pt \
  --sam-source MobileSAM \
  --sam-device cuda \
  --mask-erode-px 2 \
  --progress-every 50
```

关键参数：

- `--frames 0`：处理所有连续可用帧。
- `--frame-step 1`：每一帧都处理。
- `--confidence 0.5`：YOLO-World 置信度阈值。
- `--pixel-stride 2`：SAM mask 内隔像素投影。
- `--voxel-size 0.02`：2 cm 对象融合体素。
- `--min-confirmed-observations 8`：至少观测 8 次才保留对象。
- `--pose-convention replica`：读取本项目生成的 Z-up `traj.txt`。

输出包括：

```text
RECONSTRUCTION_OUTPUT/
├── objects/
│   ├── object_XXXX_<class>.ply
│   └── object_XXXX_<class>.npz
├── associations.json
├── semantic_objects.json
├── summary.json
└── timing.json
```

### 二维语义俯视图

```bash
conda run --no-capture-output -n opi_yolo_eval \
python scripts/view_objects_2d.py \
  --objects-dir /home/weiyu/vscode_workspace/hd3d_semantic/outputs/00804_manual_reconstruction_sam/objects \
  --output /home/weiyu/vscode_workspace/hd3d_semantic/outputs/00804_manual_reconstruction_sam/semantic_objects_xy.png \
  --json-output /home/weiyu/vscode_workspace/hd3d_semantic/outputs/00804_manual_reconstruction_sam/semantic_objects_xy.json \
  --min-observations 8 \
  --color-mode semantic \
  --point-radius 1
```

### 三维对象点云

```bash
conda run --no-capture-output -n opi_yolo_eval \
python scripts/view_tracked_objects_3d.py \
  --objects-dir /home/weiyu/vscode_workspace/hd3d_semantic/outputs/00804_manual_reconstruction_sam/objects \
  --min-observations 8 \
  --color-mode semantic \
  --show-boxes \
  --show-origin \
  --background dark
```

外部重建包输出的是**语义对象点云地图**。默认类别列表会排除墙、地板、天花板等类别，因此它不是包含全部建筑表面的稠密场景网格。

## 测试

运行不依赖真实 HM3D 数据的测试：

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
python -m pytest -q
```

真实场景集成测试被标记为 `integration`，需要 Habitat-Sim 和本地 HM3D 资产：

```bash
python -m pytest -m integration -q
```

## 常见问题

### `scene not found`

检查 `--scene` 是否直接指向 `.basis.glb`，不要只传场景目录。

```bash
test -f "$HM3D_SCENE" && echo OK
```

### `scene dataset config not found`

配置文件通常位于单独的 semantic configs 目录，不一定和 `.basis.glb` 在同一层。

```bash
find "$HM3D_ROOT" -name "*.scene_dataset_config.json"
```

### `NavMesh failed to load`

确认 `.basis.navmesh` 与 `.basis.glb` 配套，并保持数据集原始相对目录结构。

### `semantic scene contains no instance metadata`

当前场景没有正确加载语义资产。只采 RGB-D 时使用：

```bash
--no-save-semantic
```

需要 Habitat GT 语义时，检查 `.semantic.glb`、`.semantic.txt` 和 annotated scene dataset 配置。

### Interactive 没有窗口

- 确认安装的不是 headless Habitat-Sim。
- 确认当前会话存在 `DISPLAY`。
- 确认 Tk 可用：

```bash
python -c "import tkinter; tkinter.Tk().destroy()"
```

SSH 环境需要 X11 转发或远程桌面；纯终端服务器应使用 Waypoint 或 Replay。

### 窗口或字体太小

```bash
--display-scale 5.0 --ui-scale 2.5
```

保存图像仍然使用 `--width` 和 `--height`，不会因 UI 放大而改变。

### 按 `Esc` 后没有正式输出目录

这是预期行为。`Esc` 保留 `OUTPUT.partial`，`Q` 才会发布为 `OUTPUT`。如果在第一次 `R` 前退出，则没有录制帧，只会留下失败报告。

### 输出目录已存在

默认拒绝覆盖非空目录。推荐为每次实验使用新目录；确认需要替换时显式添加：

```bash
--overwrite
```

### Replay 报 `off navmesh`

回放轨迹必须来自同一场景，且每个机器人位置都要落在当前 NavMesh 上。不要把 `traj.txt` 中的相机 Z-up 矩阵位置直接当作 Habitat agent Y-up 位置；优先使用导出的 `trajectory.json`。

### `semantic_objects_xy.png` 被压成狭长图

这通常表示把 Habitat Y-up `traj_gt.txt` 误当成 Z-up 地图轨迹。当前版本应向 `semantic_map_offline` 提供 `traj.txt`。先运行严格校验：

```bash
python scripts/validate_dataset.py \
  --data-root /path/to/dataset \
  --sample-count 50 \
  --strict
```

### 深度图看起来全黑

深度 PNG 是 `uint16` 毫米数据，普通图片查看器会按 8 位图显示。读取时应使用 unchanged 模式，再除以 `scale`：

```python
import cv2
import json

root = "outputs/00804_manual"
depth = cv2.imread(f"{root}/results/depth000000.png", cv2.IMREAD_UNCHANGED)
scale = json.load(open(f"{root}/cam_params.json", encoding="utf-8"))["camera"]["scale"]
depth_m = depth.astype("float32") / scale
```

## Git 提交说明

`.gitignore` 已排除以下本地或大体积内容：

- HM3D `.glb`、`.navmesh`、压缩包。
- Conda、venv 和 Python 缓存。
- `outputs/`、`datasets/`、`runs/` 等生成数据。
- `.partial/` 和 `.previous/` 事务目录。
- 本地重建产物和任务文档。

推送前建议执行：

```bash
python -m pytest -q
git diff --check
git status --short
```

确认 `git status` 中只包含代码、测试和文档，不包含 HM3D 数据、模型或重建点云。

## 参考资料

- [Habitat-Sim 官方仓库](https://github.com/facebookresearch/habitat-sim)
- [Habitat-Sim 数据集说明](https://github.com/facebookresearch/habitat-sim/blob/main/DATASETS.md)
- [Habitat-Sim SceneDataset JSON 文档](https://aihabitat.org/docs/habitat-sim/attributesJSON)
- [Habitat-Lab 官方仓库](https://github.com/facebookresearch/habitat-lab)
- [Habitat-Lab 数据集目录约定](https://github.com/facebookresearch/habitat-lab/blob/main/DATASETS.md)
- [HM3D 官方仓库](https://github.com/facebookresearch/habitat-matterport3d-dataset)
