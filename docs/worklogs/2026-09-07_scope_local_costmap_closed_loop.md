# M1 SCOPE 接入 Local Costmap：实现、验证与原生 GPU 闭环记录

日期：2026-09-07

## 结论与阅读路径

本日工作将原本仅作为在线观测器运行的 SCOPE，接入为 Nav2 local costmap 的可选预测增量层，并完成配置、测试、文档和原生 GPU LiDAR 闭环验证。最终状态需要分成两部分理解：

- **工程接入完成**：新增的 `m1_scope_costmap_layer` 能严格配对预测与不确定性栅格，将 `p+1σ` 风险保守投影到 local costmap，并在数据过期、禁用、滚动窗口移动和新帧切换时撤销自身贡献。插件、local costmap、MPPI 与现有速度安全链已经形成可运行闭环。
- **本次导航验收未通过**：在固定 seed、固定起点和固定目标下，`scope_enabled:=true` 的目标被接受，但 60 s 内未成功，随后以 `Goal failed` 结束；相同条件下 `scope_enabled:=false` 的基线在 6.855 s 内成功。该单次结果不能证明总体性能优劣，但明确说明当前风险映射还不能被表述为已验证的导航提升或安全增强。

本文依次记录实施边界、插件设计、Nav2 接线、测试驱动修正、构建回归、原生 GPU 闭环结果和后续问题。2026-09-06 的离线评估与在线观测器记录保持不变，本日志只记录 2026-09-07 新增的闭环入口工作。

## 1. 工作目标与边界

本次工作的目标不是修改 SCOPE 模型，而是在保持官方前向栅格坐标契约不变的前提下，把已有 `/scope/prediction` 和 `/scope/uncertainty` 输出融合为 local costmap 增量代价。

保持不变的边界包括：

- 不重新训练或修改官方 SCOPE 模型、随包冻结的 `model.py`/`convlstm.py` 或外部 checkpoint。
- 输入仍是 10 帧、10 Hz、`[N,10,1,64,64]`，分辨率仍为 0.1 m。
- 模型坐标仍为前向 `x=[0,6.4)` m、横向 `y=[-3.2,3.2)` m，在线默认预测时域仍为 5 步，即 0.5 s。
- 只修改 local costmap，不向 global costmap 添加 SCOPE layer。
- SCOPE layer 不发布速度，不绕过 MPPI、velocity smoother、collision monitor 或 watchdog。
- `scope_enabled` 仍默认关闭；关闭时不启动 predictor，local SCOPE layer 也禁用。
- 保留工作树中已有的 2026-09-06 worklog 修改和未跟踪 PDF，不覆盖、不清理、不纳入本次改动。

MPPI 只做温和的朝向偏好调整：`PathAngleCritic.cost_weight` 从 2.0 调为 4.0，`PreferForwardCritic.cost_weight` 从 1.0 调为 2.0；`GoalAngleCritic=3.0`、`TwirlingCritic=10.0` 和 `motion_model=Omni` 保持不变。

## 2. 新增 `m1_scope_costmap_layer` 包

### 2.1 包结构与插件导出

新增 C++ ament 包 `src/m1_scope_costmap_layer`，主要文件为：

- `include/m1_scope_costmap_layer/scope_layer.hpp`
- `src/scope_layer.cpp`
- `test/test_scope_layer.cpp`
- `scope_layer.xml`
- `CMakeLists.txt`
- `package.xml`

插件类为 `m1_scope_costmap_layer::ScopeLayer`，继承 `nav2_costmap_2d::Layer`，通过 pluginlib 导出。插件 XML、CMake 安装规则和 `package.xml` 的 `nav2_costmap_2d` export 均经过动态加载测试。

### 2.2 输入配对与验证

插件分别订阅 `/scope/prediction` 和 `/scope/uncertainty`，每个 topic 只保留最多 4 条待配对消息，避免队列无界增长。只有下列字段完全一致时才形成快照：

- `header.stamp`
- `header.frame_id`
- `width`、`height`
- `resolution`
- origin 的完整位置和四元数

此外，输入 frame 必须等于 layered costmap 的 global frame；当前 local costmap 使用 `odom`。插件不执行 TF 查询，因此不会为了预测未来时间戳进行 TF 外推。

输入校验拒绝：

- 负时间戳或非法 nanosecond；
- 空 frame、不同 frame；
- 零尺寸、数据长度不符；
- 非有限、非正或会导致派生尺寸/角点溢出的 resolution；
- 非有限 origin、非归一化或非平面四元数；
- 不在 OccupancyGrid 合法编码 `-1` 或 `0..100` 内的数值。

其中 `-1` 是唯一合法 unknown 值。prediction 或 uncertainty 任一输入为 unknown 时，该源单元透明，不向 master costmap 写入任何代价。

### 2.3 风险融合与代价映射

对已知单元，插件按下式恢复概率和标准差：

```text
prediction = prediction_value / 100
std        = uncertainty_encoding_scale * uncertainty_value / 100
risk       = min(1, prediction + uncertainty_gain * std)
```

默认 `uncertainty_encoding_scale=0.5`、`uncertainty_gain=1.0`。这与 predictor 当前“标准差除以 0.5 后映射到 `0..100`”的传输编码一致，因此默认风险为 `min(1,prediction+std)`。

默认映射为：

| 风险区间 | SCOPE layer 行为 |
|---|---|
| `risk < 0.35` | 透明，不写入 |
| `0.35 <= risk < 0.60` | 提议代价 200 |
| `risk >= 0.60` | 提议 `LETHAL_OBSTACLE=254` |

合并始终采用 max 语义。插件只有在 SCOPE 代价高于当前 master 代价，或 master 为 `NO_INFORMATION` 时才提高该格代价；从不写 `FREE_SPACE`，也不清除 obstacle layer 已写入的实测障碍。

### 2.4 旋转栅格投影与覆盖裁剪

每个 SCOPE 源单元使用 OccupancyGrid origin 的平移和 yaw 变换到 `odom` 世界坐标。源单元是旋转四边形，master 单元是轴对齐矩形；二者只有存在严格正面积交集时才认为覆盖。

候选范围先在世界坐标中裁剪到当前 master map 和本轮 update window，然后才转换为整数栅格索引，避免极端坐标的浮点到整数未定义行为。实现覆盖：

- 0° origin；
- 90° origin；
- 任意 yaw；
- 0.10 m 源栅格到 0.05 m master 栅格；
- local costmap 边界裁剪；
- 滚动窗口原点变化；
- 极小但可表示的正面积交集；
- 精确零面积共边不写入。

这里没有把 `float32` 的 0.1 m 强行归一化成十进制精确 0.1。代码严格使用消息中实际编码的 resolution；只要几何上存在可表示的正面积交集，就保守覆盖。这样避免用固定 epsilon 或 ULP 吸收真实窄交集。

### 2.5 过期回退、旧边界撤销与 currentness

freshness 以完整消息对的本地接收时间计算，而不是以预测目标时间戳计算。默认超过 0.5 s 没有新的完整消息对时，插件停止贡献新代价，同时把上一次已经渲染的 bounds 加入下一轮更新范围，使 Nav2 先重置该窗口，再按 obstacle、SCOPE、inflation 顺序重新组合。

该机制用于以下变化：

- 新预测帧替换旧预测帧；
- rolling local costmap 原点移动；
- SCOPE 数据由 fresh 变为 stale；
- layer 从 enabled 切换为 disabled；
- topic 参数热切换；
- ROS `/clock` 回拨。

关键约束是：可选 SCOPE 数据过期时，`ScopeLayer` 仍保持 `current_=true`。否则 `LayeredCostmap::isCurrent()` 会把整个 local costmap 判为 stale，进而让 controller 因一个可选预测源失效。当前实现只撤销本层，不让 local costmap 或 MPPI 进入 stale 状态。

### 2.6 线程、参数热切换与运行时统计

订阅回调、参数更新和 costmap update 之间通过 mutex 和不可变的 cycle snapshot 隔离。prediction/uncertainty topic 热切换时：

1. 先验证 topic 名并创建新订阅；
2. 为新订阅分配新的 generation；
3. 原子替换配置和订阅；
4. 清空旧 pending/snapshot，但保留已渲染 bounds 供下一轮撤销；
5. 忽略仍在途的旧 generation 回调。

公开参数为：`enabled`、`prediction_topic`、`uncertainty_topic`、`low_threshold`、`lethal_threshold`、`uncertainty_gain`、`uncertainty_encoding_scale`、`medium_cost` 和 `stale_timeout`。

每次有效 render 会更新以下统计，并以最多 1 Hz 的 INFO 日志输出：

```text
SCOPE raster stats valid=<n> clipped=<n> medium=<n> lethal=<n>
```

`valid` 是 prediction 与 uncertainty 均已知的源格数量；`clipped` 是未完全落在当前更新窗口内的有效源格数量；`medium` 和 `lethal` 是本轮实际提高 master cost 的目标格数量。

## 3. Nav2 配置与 launch 接线

### 3.1 Local costmap 插件顺序

`nav2_params.yaml` 的 local costmap 插件顺序改为：

```yaml
plugins: [obstacle_layer, scope_layer, inflation_layer]
```

SCOPE 参数使用计划中的默认值。inflation layer 仍位于最后，`inflation_radius` 保持 0.4 m，因此 SCOPE 写入的致命障碍会继续使用现有 inflation。global costmap 的 `[static_layer, obstacle_layer, inflation_layer]` 保持不变。

### 3.2 `scope_enabled` 联动

`nav2_m1_gazebo.launch.py` 继续只公开一个 `scope_enabled` 参数，默认 `false`。它一方面通过 `IfCondition` 控制 predictor include，另一方面通过 Humble `RewrittenYaml` 的完全限定路径：

```text
local_costmap.local_costmap.ros__parameters.scope_layer.enabled
```

只改写 local SCOPE layer 的 `enabled`。没有使用模糊的叶节点键重写，因此不会误改 obstacle layer、其他 Nav2 插件或节点中的同名 `enabled` 参数。

`m1_nav2_bringup/package.xml` 新增 `m1_scope_costmap_layer` 运行时依赖。模型节点和最终速度发布链没有改动。

## 4. 测试驱动实施与关键返工

本次功能按 RED-GREEN-REFACTOR 方式实施。最初先建立插件测试框架，在头文件不存在时捕获预期构建失败，再实现最小插件。随后通过规格审查和代码质量审查逐轮增加失败用例并修正实现。

### 4.1 第一轮功能测试

第一轮覆盖风险融合、阈值边界、严格配对、旋转投影、裁剪、透明/max 合并、stale 撤销、enabled 切换、旧/新 bounds 和 pluginlib 加载。初始实现可以构建，第一轮 14 个 gtest 全部通过。

### 4.2 规格审查发现的问题

第一次规格审查没有接受初始实现，发现：

- stale 时错误地把 `current_` 设为 false；
- 错误接受 `-2..-128` 为 unknown；
- `/clock` 回拨后旧 epoch snapshot 可能阻塞新消息；
- 固定面积容差会漏掉真实的极小正面积交集；
- 测试只手工 reset master，没有经过真实 `LayeredCostmap::updateMap()`；
- 统计名称与实际计数口径不完全一致。

针对这些问题新增 12 个验收场景，其中 8 个准确复现失败。修正后加入真实 rolling/non-rolling `LayeredCostmap` 集成夹具，验证启动无数据、stale、disabled、新帧和 rolling origin 时，基础层能够重新组合且 costmap 保持 current。

第二次严格审查又给出了大坐标反例：使用 4 ULP 距离归零会在坐标约 1000 m 时吞掉宽度约 `1.14e-13 m` 的可表示正交集。随后删除所有会侵蚀正交集的 ULP/epsilon snapping，并增加大坐标、精确共边、微小 yaw 和候选边界舍入测试。

### 4.3 代码质量审查发现的问题

代码质量审查进一步发现：

- topic 动态切换不是 generation-safe，旧订阅在途回调可能污染新队列；
- 负时间戳可能在回调中构造 `rclcpp::Time` 时抛异常；
- 极端 resolution 可能使派生角点成为 Inf/NaN；
- 过大的 `stale_timeout` 可能溢出 int64 纳秒；
- 初始多边形裁剪使用大量 `vector` 和哈希表，密集 64×64 更新测得 72,719 次 heap allocation。

新增的 5 个质量回归测试最初为 0/5。最终实现增加 generation token、异常安全订阅交换、输入/Duration 上界验证，并把热循环改成固定容量栈内多边形裁剪和单块连续 scratch buffer。相同密集用例的分配约束降到不超过 4 次；Release-like 运行约 10–18 ms，满足 10 Hz 更新周期的单次计算预算，但这不是全系统实时性证明。

### 4.4 Nav2 配置回归

`m1_nav2_bringup` 测试实际生成并解析 `scope_enabled=false/true` 两份 RewrittenYaml，分别求值 predictor 的 `IfCondition`，再比较完整 YAML，确认除 `use_sim_time` 和唯一的 local SCOPE enabled 路径外没有其他参数被误改。

配置测试还固定：

- MPPI `PathAngleCritic=4.0`、`PreferForwardCritic=2.0`；
- `GoalAngleCritic=3.0`、`TwirlingCritic=10.0`、`Omni` 不变；
- local 插件顺序正确；
- global costmap 不含 SCOPE；
- 0.4 m inflation 不变；
- 九个公开参数及默认值正确；
- `m1_nav2_bringup` 声明新插件运行时依赖。

## 5. 构建与自动测试结果

为避免当前工作区旧 build cache 指向相邻 `/home/xinlei/Data/ROS/...` checkout，最终验证使用独立 `/tmp` build/install/log 目录。环境为 ROS 2 Humble 和仓库 `.venv-scope-runtime`。

第一次只选择四个目标包时，`m1_nav2_bringup` 因本地依赖 `m1_nav2_support`、`yahboomcar_description` 尚未在隔离 install 中构建而失败。将这两个必要工作区依赖加入同一隔离构建后，六包全部成功。该失败属于隔离构建依赖不完整，不是源码编译失败。

最终四个目标包测试结果为：

| 包 | 通过 | 失败 |
|---|---:|---:|
| `m1_scope_bridge` | 19 | 0 |
| `m1_scope_predictor` | 24 | 0 |
| `m1_scope_costmap_layer` | 36 | 0 |
| `m1_nav2_bringup` | 26 | 0 |
| **合计** | **105** | **0** |

`colcon test-result` 共读取 106 条记录，0 error、0 failure、0 skipped；比上表多的一条是 costmap gtest 外层的 CTest wrapper。costmap 测试在受限沙箱内出现 DDS `getifaddrs`/UDP socket permission 警告，但所有测试均完成并通过。

predictor runtime contract 单独重跑 7/7 通过，额外的 predictor/bridge 坐标相关测试 15/15 通过。确认：

- 历史长度仍为 10 帧、间隔 0.1 s；
- batched 模型输入仍为 `[N,10,1,64,64]`；
- 采样输出为 `[N,1,64,64]`，聚合输出为 `[1,64,64]`；
- 分辨率、模型坐标范围和 0.5 s 时域未改变；
- 零激光位姿时 ROS grid origin 仍为 `(0,-3.2,0)`；
- NumPy x/y 轴到 OccupancyGrid row-major 的转换未改变。

## 6. 原生 GPU LiDAR 闭环测试

### 6.1 测试条件

两次运行均使用：

- ROS 2 Humble；
- Python 3.10、PyTorch 2.5.1+cu124；
- NVIDIA GeForce RTX 4060 Laptop GPU，驱动 580.173.02；
- 官方 checkpoint，SHA-256 为 `0eb7d348530670e0547c24e91df583540930c89668937723a4e7161d6fab7d92`；
- `software_lidar:=false`；
- `dynamic_obstacles:=true`；
- `dynamic_seed:=20260814`；
- `gui:=false`、`rviz:=false`；
- 初始位姿 `(-2.5,-1.5,0)`；
- 目标 `(-2.5,1.5,π/2)`。

没有使用软件 LiDAR 兜底。所有运行产物保存在 `/tmp/m1-native-scope-acceptance.KeHpyo/`，未复制进仓库。

### 6.2 原生传感器质量门

SCOPE-on 预检 20.05 s 内收到 241 帧 `/scan`，频率 12.042 Hz；baseline 预检为 12.053 Hz。每帧固定 667 beams，两次均没有整帧 `-Inf` 或 NaN。SCOPE-on 预检有限值比例均值为 0.99850，最小有限距离 1.156 m。

因此，本次没有触发“原生 GPU `/scan` 异常”的传感器后端阻塞，也没有用软件 LiDAR 结果代替原生验收。

### 6.3 SCOPE 输出与配对

目标窗口内收到 491 条 prediction 和 491 条 uncertainty；491 对消息在 stamp、frame、宽高、resolution 和 origin 上全部严格匹配，没有 unmatched pair。SCOPE 使用 CUDA，末次诊断为：

- 已分配 CUDA 内存：3,666,944 B；
- 推理延迟：151.917 ms；
- 输出频率：5.347 Hz；
- prediction age：335 ms；
- submitted/dropped job 累计：6636/415。

保存的最后 20 个诊断样本中，推理延迟 mean/p95 为 134.834/168.233 ms，输出频率 mean/p95 为 6.887/7.462 Hz。预检短窗口曾达到约 9.998 Hz；目标运行中随着系统负载上升，实际持续输出低于该值。latest-only worker 丢弃旧任务而不积压。

### 6.4 SCOPE-on 闭环结果

目标被 Nav2 接受，goal ID 为 `d5de8e6aed2041d2ba4a97adcbe9ff0f`。59.11 s 时最后一条 CLI feedback 为：

- 位姿约 `(-2.853,0.865,-0.009 rad)`；
- 剩余距离 0.791 m；
- recovery 次数 20。

目标没有在 60 s 内成功，launch 最终在约 62.650 s 记录 `Goal failed`。目标期间出现 14 次 MPPI `Optimizer fail to compute path`/FollowPath abort。没有发现 costmap stale 或 not-current 中止，因此失败不是由可选 SCOPE 数据过期把整个 costmap 拖成 stale 引起。

local costmap 更新频率为 10.001 Hz。目标期间采集 60 条 SCOPE raster 日志：

| 统计 | 最小 | 最大 | 平均 |
|---|---:|---:|---:|
| valid 源格 | 4096 | 4096 | 4096 |
| clipped 源格 | 2871 | 2968 | 2899.25 |
| medium master 格 | 0 | 105 | 21.67 |
| lethal master 格 | 23 | 401 | 243.83 |

大量 clipped 是因为 SCOPE 前向 6.4×6.4 m 栅格比 5×5 m rolling local costmap 大，并且会随预测 origin/yaw 旋转；插件只写二者交集。当前风险映射在目标运行中平均实际提高约 244 个 master 格为 lethal，这与 MPPI 路径求解困难同时出现，但单次测试不能证明二者之间的充分因果关系。

速度链所有阶段均观测到非零指令：

| topic | 非零样本数 |
|---|---:|
| `/cmd_vel_nav` | 367 |
| `/cmd_vel_smoothed` | 406 |
| `/m1/cmd_vel_raw` | 153 |
| watchdog `/cmd_vel` | 152 |

最终 `/cmd_vel` 在完整监测窗口约 19.94 Hz，说明 `/cmd_vel_nav -> velocity smoother -> collision monitor -> watchdog -> /cmd_vel` 链路持续存在。collision monitor 对输出有大量抑制，但 watchdog 仍是最终 `/cmd_vel` 发布者。

SCOPE-on 的最小有限扫描距离为 0.172 m。基于 odom 的粗略诊断为：净位移 2.154 m；车头相对净位移方向误差 p50/p95 为 1.015/1.720 rad；车体系横向/前向速度绝对比 p50/p95 为 1.221/3.537。该方向误差不是逐点相对 global path tangent 的误差，且前向速度接近零时横移比会不稳定，只作为本次运行观察值。

### 6.5 `scope_enabled:=false` 基线

基线使用相同原生传感器、seed、起点和目标。运行时确认：

- predictor 进程不存在；
- `/scope/prediction` 和 `/scope/uncertainty` 均为 0 publisher；
- `scope_layer.enabled=false`；
- 原生 `/scan` 预检 12.053 Hz、667 beams、无整帧异常。

目标 ID 为 `c8163473456d4a8fa9273ee8abcfd79e`，最终 `SUCCEEDED`，navigation time 6.855 s，0 recovery，最终 feedback 位姿约 `(-2.492,1.542,1.558 rad)`。

基线各速度阶段也均有非零消息，说明关闭 SCOPE 后原有链路没有被接线改动破坏。基线最小有限扫描距离为 0.170 m，local costmap 更新频率约 10.003 Hz。

### 6.6 接入 SCOPE 前后的测试效果对比

下面把两次同 seed、同起点、同目标的运行放在一起。这里的“接入效果”既包括系统是否按预期工作，也包括本次导航行为发生了什么变化。

| 指标 | SCOPE-on | SCOPE-off baseline | 本次观察 |
|---|---:|---:|---|
| 目标结果 | `Goal failed` | `SUCCEEDED` | 本次接入后未完成目标 |
| 导航时间 | 60 s 内未成功，约 62.650 s 后失败 | 6.855 s | SCOPE-on 明显滞留 |
| recovery 次数 | 20 | 0 | SCOPE-on 频繁进入恢复 |
| MPPI optimizer/FollowPath failure | 14 | 0 | SCOPE-on 局部轨迹多次不可求解 |
| local costmap 更新频率 | 10.001 Hz | 约 10.003 Hz | 接入没有拖垮 costmap 更新循环 |
| costmap stale/not-current | 0 | 0 | 可选层没有令 Nav2 stale |
| 原生 `/scan` 预检 | 12.042 Hz | 12.053 Hz | 传感器输入基本一致且正常 |
| 最小有限扫描距离 | 0.172 m | 0.170 m | 单次最小距离近似，不能据此认定更安全 |
| 动态障碍检测比例 | 0.9762 | 0.8815 | SCOPE-on costmap 更常覆盖动态障碍附近，但不等于成功避障 |
| 动态障碍最长未检测时间 | 0.834 s | 3.429 s | SCOPE-on 的占据覆盖更持续 |
| 净位移 | 2.154 m | 3.197 m | SCOPE-on 前进受限、未到达目标 |
| 车头—净路径方向误差 p50/p95 | 1.015/1.720 rad | 0.225/0.683 rad | 本次 MPPI 朝向跟随反而更差 |
| 横向/前向速度比 p50/p95 | 1.221/3.537 | 0.464/4.644 | SCOPE-on 典型横移占比更高；p95 受前向速度接近零影响 |

从工程效果看，接入是成功的：SCOPE 使用 CUDA 持续推理，491 对 prediction/uncertainty 全部严格匹配；local costmap 保持约 10 Hz；速度命令经过完整安全链；没有进程在目标执行期间崩溃，也没有因为 SCOPE 输出间歇而触发 costmap stale。`scope_enabled=false` 时 predictor 和 layer 同时退出闭环，baseline 正常工作。这说明新增入口具备可运行、可禁用和不破坏默认路径的基本工程属性。

从本次导航效果看，结果是负面的。SCOPE-on 每次统计平均实际增加约 21.67 个 medium 格和 243.83 个 lethal 格，最多达到 105/401 个。随后 MPPI 多次无法生成可执行轨迹，机器人产生 20 次 recovery，最终未到达目标。相比之下，关闭 SCOPE 后相同目标很快成功。当前 `risk>=0.60` 直接写 lethal、再进行 0.4 m inflation 的组合，对冻结模型的密集概率输出可能过于保守；uncertainty 加到 prediction 后也可能扩大高风险区域。这些是与运行现象一致的待验证解释，不是已经确认的唯一根因。

`dynamic_obstacle_detected_ratio` 从 0.8815 增至 0.9762、最长 miss 从 3.429 s 降至 0.834 s，表明接入后的 costmap 在本次轨迹上更持续地标记了动态障碍附近区域。然而“标得更多”没有转化为成功导航：过多或持续的 lethal/inflated 区域也可能封闭 MPPI 的局部可行空间。最小扫描距离两组近似，因此不能用这两个单次最小值主张 SCOPE 提高了安全裕量。

MPPI 权重调整希望产生温和的车头朝向偏好，但本次 odom 粗略指标没有显示预期改善：SCOPE-on 的车头—净路径方向误差中位数为 1.015 rad，高于 baseline 的 0.225 rad，横向/前向速度比中位数也从 0.464 增至 1.221。这个结果同时受到局部路径反复失败、恢复行为和净位移定义影响，不能单独归因于 `PathAngleCritic`/`PreferForwardCritic` 权重；它至少说明当前“MPPI 调权 + SCOPE 风险层”的整体组合尚未达到期望运动效果。

推理负载也是效果的一部分。短预检窗口输出接近 10 Hz，但目标期间最后 20 个诊断样本的输出频率均值只有 6.887 Hz，推理延迟均值/p95 为 134.834/168.233 ms，末次 prediction age 为 335 ms。数据仍低于 0.5 s stale timeout，latest-only 调度也防止积压，但在高负载闭环中可供 costmap 使用的新预测频率明显低于扫描和 costmap 频率。后续参数实验应同时记录预测年龄，避免只比较风险阈值而忽略推理时序。

### 6.7 结论与证据边界

本次运行支持以下结论：

1. 原生 GPU LiDAR 正常时，SCOPE predictor、严格配对 layer、local costmap、MPPI 和速度安全链能够共同运行。
2. 运行期间没有出现 local costmap 或 controller stale；stale 撤销和基础层重组语义由真实 `LayeredCostmap` 自动化测试直接覆盖，本次运行没有单独制造超过 0.5 s 的 SCOPE 断流事件。
3. `scope_enabled=false` 能同时移除 predictor 和 local layer 贡献，原始 Nav2 链路仍可成功导航。
4. 当前一次 SCOPE-on 运行没有通过目标结果，且与大量 lethal 预测格和 MPPI optimizer failure 同时出现。

不能从这一对运行推出 SCOPE 总体上降低导航性能，也不能把 baseline 的单次成功当作统计对照。两次运行虽使用相同 seed 和目标，但动态场景、调度、GPU 推理时序和机器人实际轨迹仍可能不同。后续需要冻结更严格的场景时序并增加重复试验，才能判断风险阈值、uncertainty gain、lethal 映射或 local window 几何是否需要调整。

## 7. 运行限制与异常记录

- 监测器在 SCOPE 启动后才附着，没有精确记录第 10 帧预热完成的冷启动时间；附着后 READY 状态和连续输出正常。
- 两个 70 s costmap 监测窗口都没有出现可用于计算 ghost-clear latency 的事件；ghost 撤销主要由自动化真实 `LayeredCostmap` 集成测试覆盖。
- 目标期间进程均保持存活。协调 SIGINT 退出时，SCOPE predictor 出现重复 `rclpy.shutdown()`，dual-laser merger 在 SCOPE-on 退出时报告 `-11`，Ignition wrapper 因 SIGINT 报告 `-2`。这些发生在停止阶段，不是目标运行期间崩溃；最终检查没有残留 Gazebo、Nav2、SCOPE 或监测进程。
- 原生闭环产物位于 `/tmp`，属于当前机器上的临时证据，不是仓库内长期归档。

## 8. 可复现命令

### 8.1 构建

```bash
source /opt/ros/humble/setup.bash
source .venv-scope-runtime/bin/activate
/usr/bin/colcon build --packages-up-to \
  m1_scope_bridge m1_scope_predictor m1_scope_costmap_layer \
  m1_nav2_bringup --symlink-install
source install/setup.bash
```

最终隔离验证实际把 build、install 和 log base 指向 `/tmp/m1-four-package-validation.y0SGYi/`，并显式包含 `m1_nav2_support`、`yahboomcar_description` 两个本地依赖，以避开工作区旧 cache。

### 8.2 SCOPE-on 原生闭环

```bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py \
  gui:=false rviz:=false \
  software_lidar:=false dynamic_obstacles:=true \
  dynamic_seed:=20260814 scope_enabled:=true
```

### 8.3 SCOPE-off 原生基线

```bash
ros2 launch m1_nav2_bringup nav2_m1_gazebo.launch.py \
  gui:=false rviz:=false \
  software_lidar:=false dynamic_obstacles:=true \
  dynamic_seed:=20260814 scope_enabled:=false
```

两次运行均向 `NavigateToPose` 发送 `(-2.5,1.5,π/2)`，并在发送前显式发布初始位姿 `(-2.5,-1.5,0)`。

## 9. 文件与证据索引

仓库内本日主要文件：

- `src/m1_scope_costmap_layer/`：新 Nav2 costmap 插件、导出描述和测试。
- `src/m1_nav2_bringup/config/nav2_params.yaml`：MPPI 权重和 local SCOPE layer 参数。
- `src/m1_nav2_bringup/launch/nav2_m1_gazebo.launch.py`：`scope_enabled` 联动。
- `src/m1_nav2_bringup/package.xml`：新插件运行时依赖。
- `src/m1_nav2_bringup/test/test_nav2_params.py`：配置与 launch 回归。
- `docs/scope_online.md`：更新后的在线使用、风险映射和失效回退说明。

当前机器上的临时原生运行证据：

- `/tmp/m1-native-scope-acceptance.KeHpyo/summary.md`
- `/tmp/m1-native-scope-acceptance.KeHpyo/summary.json`
- `/tmp/m1-native-scope-acceptance.KeHpyo/commands.txt`
- `/tmp/m1-native-scope-acceptance.KeHpyo/scope/launch.log`
- `/tmp/m1-native-scope-acceptance.KeHpyo/scope/action_goal.log`
- `/tmp/m1-native-scope-acceptance.KeHpyo/baseline/launch.log`
- `/tmp/m1-native-scope-acceptance.KeHpyo/baseline/action_goal.log`

## 10. 后续建议

下一阶段不应直接宣称闭环收益，而应先解释本次 SCOPE-on 失败：

1. 固定完整仿真时钟和障碍物轨迹，重复多 seed、多次 A/B，分离偶然轨迹差异。
2. 统计每帧 lethal/medium 空间分布与 global path、实测 scan、动态障碍真值的关系，判断 lethal 格是否主要来自墙体、动态障碍或不确定性扩张。
3. 做参数消融：分别测试 `uncertainty_gain`、`low_threshold`、`lethal_threshold` 和 medium/lethal 映射，但每次只改变一个因素。
4. 保持 SCOPE layer 为可选、默认关闭；在重复闭环证据形成前，不将它描述为避障性能或实机安全增强。
5. 单独修复协调退出时 predictor 重复 shutdown 和 SCOPE-on merger `-11`，避免停止阶段噪声掩盖真正运行时崩溃。

本日工作完成了一个可运行、可禁用、可失效回退且有测试覆盖的 local costmap 预测入口；同时，原生闭环试验给出了清晰的负面结果，说明工程接入完成不等于导航策略已经调优完成。
