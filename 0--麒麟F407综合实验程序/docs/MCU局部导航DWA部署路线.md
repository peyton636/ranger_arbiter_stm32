# MCU 侧 DWA 局部路径规划 — 内存评估与部署路线图

| 元数据 | 值 |
|--------|-----|
| **文档版本** | v0.1.1 |
| **基线固件** | `mcu` @ `2df47c6`（改代码rtos逻辑之前的） |
| **分工** | Jetson：全局规划 + sub-goal 下发；MCU：DWA 局部规划 + 安全仲裁执行 |
| **相关文档** | [rtos移植.md](./rtos移植.md) §2.1、[Jetson_BLOB协议_v2.md](./Jetson_BLOB协议_v2.md)、[硬件连接与通信协议.md](./硬件连接与通信协议.md) |

---

## 1. 当前内存使用情况（基线 `2df47c6`）

### 1.1 物理 RAM 三块

STM32F407 本工程使用 **片内主 SRAM 128KB** + **CCM 64KB** + **外扩 SRAM 1MB**（FSMC @ `0x68000000`）。  
LCD 标注「192KB」= 128KB + 64KB CCM。

```text
Flash 1MB @ 0x08000000     程序 .text / .rodata

0x20000000  片内主 SRAM 128KB
0x10000000  CCM 64KB（仅 CPU，不可 DMA）
0x68000000  外扩 SRAM 1MB
```

链接脚本：`Obj/Template.sct` 仅映射主 SRAM；CCM / 外扩地址由 `__attribute__((at(...)))` 固定。

### 1.2 片内主 SRAM（`0x20000000`）

| 块 | 大小 | 说明 |
|----|------|------|
| `.data` / `.bss` | ~14~20KB | `arb_state`、测距滤波、`jetson_eth` 队列、GPS/CAN 缓冲等 |
| **mem1 池** `mem1base[]` | **32KB** | `mymalloc(SRAMIN, …)` |
| **mem1 管理表** | **2KB** | `MEM1_MAX_SIZE/32 × 2` 字节 |
| **余量（估）** | **~70KB+** | 链接后未固定占用部分 |

**mem1 运行时主要占用**（`UI_TEST_MODE=1` + `ETH_LWIP_ENABLE=1`）：

| 调用方 | 约占用 | 说明 |
|--------|--------|------|
| `FATFS_Init()` | ~3KB | 常驻 |
| `ETH_Mem_Malloc()` | ~15KB+ | 以太网 DMA Rx/Tx（**必须片内**，DMA 不可访问 CCM） |
| 其他 `mymalloc(SRAMIN)` | 很少 | — |

上电串口：`[MEM] pool IN=…% EX=…% CCM=…% (mem1=32KB mem2=960KB heap=32KB)`（`User/main.c`）。

**约定**：小对象、DMA → `SRAMIN`；大块/WiFi/日志 → `SRAMEX`。

配置：`Malloc/malloc.h` → `MEM1_MAX_SIZE = 32KB`。

### 1.3 CCM（`0x10000000`，64KB）

| 地址（约） | 块 | 大小 | 说明 |
|------------|-----|------|------|
| `0x10000000` | `mem3base[]` | **16KB** | `mymalloc(SRAMCCM, …)`，arbiter 路径基本不用 |
| `0x10004000` | `mem3mapbase[]` | **1KB** | mem3 位图 |
| `0x10004400` | `ucHeap[]` | **32KB** | **FreeRTOS 内核堆**（任务栈、TCB、Mutex） |
| **未用** | — | **~15KB** | 可增大 `FREERTOS_HEAP_BYTES` |

布局与溢出检查：`APP/freertos/freertos_heap.c`。  
堆大小：`User/FreeRTOSConfig.h` → `FREERTOS_HEAP_BYTES = 32KB`。

**ucHeap 当前占用（8 个 RTOS 任务，`ETH_LWIP_ENABLE=1`）**：

| 任务 | 栈 (words) | 约字节 |
|------|------------|--------|
| Motion | 512 | 2KB |
| Can | 384 | 1.5KB |
| Sensor | 384 | 1.5KB |
| Key | 256 | 1KB |
| **Jetson** | **1024** | **4KB** |
| Gps | 768 | 3KB |
| Net | 512 | 2KB |
| Ui | 768 | 3KB |
| Idle 等 | ~128 | ~0.5KB |
| **栈合计** | **~4736 w** | **~18.5KB** |
| TCB + Mutex 等 | — | **~1~2KB** |
| **合计 / 32KB 堆** | — | **~20KB / 32KB** |

栈余量监控：`APP/freertos/rtos_debug.c` → `RTOS_PrintTaskStackWatermarks()`（UiTask 可每 5s 打印）。

### 1.4 外扩 SRAM（`0x68000000`，1MB）

| 块 | 大小 | 说明 |
|----|------|------|
| `mem2base[]` | **960KB** | `mymalloc(SRAMEX, …)` |
| `mem2mapbase[]` | **60KB** | 管理表 |

**lwIP（以太网启用时）** 从 mem2 分配（`LwIP/lwip_app/lwip_comm/lwip_comm.c`）：

| 项 | 配置 | 约占用 |
|----|------|--------|
| `memp_memory` | `memp_get_memorysize()` | 数 KB |
| `ram_heap` | `MEM_SIZE=16000`（`lwipopts.h`） | ~16KB |
| pbuf 池 | `PBUF_POOL_SIZE=20`，`BUFSIZE=512` | ~10KB |

`UI_TEST_MODE=1` 且无 legacy 相机时，mem2 除 lwIP 外几乎空闲。

### 1.5 模块 ↔ 内存对照

| 模块 | 存放位置 |
|------|----------|
| FreeRTOS 任务栈、TCB、`xArbMutex` | CCM `ucHeap` |
| `arb_state`、测距、`jetson_eth` 队列 | 主 SRAM `.bss` |
| FatFS、以太网 DMA | mem1（片内） |
| lwIP | mem2（外扩） |
| BLOB 组包临时缓冲 | 多在任务栈上局部数组 |

### 1.6 加 DWA 后的内存预估

| 区域 | 增量 | 是否需调配置 |
|------|------|--------------|
| CCM `ucHeap` | +新任务栈 **512~768w（2~3KB）** | **建议** `FREERTOS_HEAP_BYTES` **32KB → 40KB** |
| 主 SRAM `.bss` | DWA 静态 **~1.5~2.5KB**（定点、单条轨迹） | 一般不用改 linker |
| mem1 / mem2 | 不增加（DWA 禁止大块 `mymalloc`） | 否 |

**DWA 实现约束（省 RAM）**：

- 速度采样 **8×8 或 10×10**，禁止教程式全轨迹矩阵（>60KB）
- **int32 mm + int16 0.001rad**，少用 `double`
- 只保留**最优一条**预测轨迹，循环内临时计算
- `predict_time=2s`，`dt=0.1s`

---

## 2. 系统架构（Jetson 全局 / MCU 局部）

```text
Jetson（全局）                         MCU（局部）
  AMCL / 地图 / A* 或 Nav2
       │
       ▼
  取路径 sub-goal（0.5~1.5m）
  变换到车体坐标系 body frame
       │
       │  BLOB 0x11 local_goal
       ▼
  vJetsonTask 解析 ──► LocalGoal_Write
       │
       ▼
  vLocalPlannerTask（20ms，DWA）
    ├─ mcu_odom：(x,y,θ,v,ω)
    ├─ DistSnapshot：四路超声 → dist 代价
    └─ 输出 local_cmd (v, ω)
       │
       ▼
  vMotionTask → Arbiter（超声/急停/心跳）→ CAN 0x111
       │
       │  BLOB 0x12 nav_feedback
       ▼
  Jetson 切换 sub-goal / 重规划
```

**原则**：

- MCU **odom** 为上电归零的航迹推算，**不是** GPS/SLAM 全局位姿
- DWA 输出 **必须过 Arbiter**，不可绕过安全层直连 CAN
- 导航量产时：`ARBITER_IGNORE_DIST_SENSOR` 改 **0**

---

## 3. BLOB 协议扩展（相对现有 `0x01~0x10`）

> **权威定义**：[Jetson_BLOB协议_v2.md](./Jetson_BLOB协议_v2.md) **§5**（v2.0-draft.5.12）。下文为摘要。

### 3.1 可复用（不必新建）

| MSG | 用途 |
|-----|------|
| `0x01` | 直接控车 + **链路心跳**（所有模式均须 ≤20ms 发） |
| `0x04` | 四路超声距离 + stamp → DWA `dist()` |
| `0x02/0x03` | 速度、仲裁状态监控 |
| `0x08` | 四轮里程（odom 辅助校正） |
| `0x107/0x108` | 时间同步（与导航并行） |

### 3.2 新增（2 个 MSG，阶段 0）

#### 下行 `0x11` — `local_goal_t`（**22 B**，Jetson → MCU）

| 字段 | 类型 | 说明 |
|------|------|------|
| timestamp_ms | u32 BE | 发帧时刻 |
| goal_x_mm / goal_y_mm | s32 BE | **body frame**（前/左，mm） |
| goal_theta_mrad | s16 BE | 目标朝向 0.001 rad |
| nav_mode | u8 | 0=DIRECT 1=LOCAL_NAV 2=PAUSE 3=ESTOP |
| seq | u8 | 目标序号（独立于 0x01 SEQ） |
| max_v_mm_s / max_omega_mrad_s | s16 BE | 限速，0=默认 |
| hold | u8 | 保活：timeout_ms = hold×50，0→500ms |
| flags | u8 | bit0=GOAL_THETA_VALID |

**DIRECT 时**：运动仍来自 `0x01`；**LOCAL_NAV 时**：0x01 的 v/ω 忽略，但仍须发 0x01 心跳。

#### 上行 `0x12` — `nav_feedback_t`（**28 B**，MCU → Jetson）

| 字段 | 说明 |
|------|------|
| pose_x/y_mm, pose_theta_mrad | MCU odom |
| cmd_v, cmd_omega | 规划器输出（仲裁前） |
| fb_v, fb_omega | 底盘反馈 |
| goal_dist_mm | 到目标距离（u16，无效 0xFFFF） |
| plan_status | 0=IDLE 1=TRACKING 2=REACHED 3=BLOCKED 4=FAIL |
| nav_mode_fb, seq_echo, flags | 回显 |

---

## 4. MCU 代码结构（规划）

```text
APP/local_nav/
├── local_nav.h / local_nav.c       # 模式、目标缓存、API
├── local_nav_shared.h / .c         # LocalGoal / NavFeedback 快照（仿 DistSnapshot）
├── mcu_odom.h / mcu_odom.c         # 位姿积分
├── motion_model.h / motion_model.c # 差速预测 x+=v·cos(θ)·dt
├── dwa_planner.h / dwa_planner.c   # 采样 + 评价
└── dwa_config.h                    # v_max、αβγ、采样步长

改动现有：
├── agv_blob_wire.h / agv_blob_pack.c   # 0x11 下行、0x12 上行
├── arbiter.c                           # LOCAL_NAV 时指令源 = local_cmd
├── rtos_tasks.h / rtos_tasks.c        # vLocalPlannerTask
├── freertos_app.c                      # 创建任务
└── User/FreeRTOSConfig.h               # 堆 32→40KB（阶段 0）
```

---

## 5. DWA 算法步骤（与教程对应）

| 步骤 | 教程 | MCU 实现要点 |
|------|------|--------------|
| 1 速度采样 | Vm ∩ Vd ∩ Va | `dwa_calc_dynamic_window()`；Va 用四路超声最近距离 |
| 2 轨迹预测 | KinematicModel | `motion_model_predict()`；dt=0.1s，预测 2s |
| 3 轨迹评价 | heading + dist + velocity | `G = α·heading + β·dist + γ·velocity`；选最大 G |
| 输出 | control_opt [v,ω] | 写入 `local_cmd` → Arbiter |

**与 Ranger 底盘**：第一阶段按 **质心 (v, ω)** 控制（与现有 `0x111` / `jetson_cmd` 一致）；自旋/阿克曼模式切换后续再细化。

---

## 6. Jetson 侧（概要，不在本仓库）

| 模块 | 职责 |
|------|------|
| 全局规划 | Nav2 / A* → `/plan` |
| `local_goal_bridge` | sub-goal → body frame → BLOB `0x11` @ 20Hz |
| `eth_gateway` | 解析 `0x12` → `/jetson_eth/nav_feedback` |
| **不部署** | DWA（在 MCU） |

---

## 7. 部署路线图与待办清单

### 阶段 0 — 协议与骨架（不动算法）

**目标**：能收发 `0x11/0x12`，模式切换生效，车仍可由 `0x01` 直接控。

| # | 待办 | 负责侧 | 状态 |
|---|------|--------|------|
| 0.1 | `Jetson_BLOB协议_v2.md` 增加 `0x11`/`0x12` 定义 | 文档 | ✅ |
| 0.2 | `agv_blob_wire.h` 增加 `BLOB_MSG_LOCAL_GOAL(0x11)`、`BLOB_MSG_NAV_FB(0x12)` 及 payload 长度 | MCU | ⬜ |
| 0.3 | `BlobPack_HandleDownlink` 解析 `0x11` → `LocalGoal_Write()` | MCU | ⬜ |
| 0.4 | `BlobPack_UplinkTick` 周期发送 `0x12`（可先填 IDLE） | MCU | ⬜ |
| 0.5 | 新建 `APP/local_nav/`，实现 `local_nav_shared` 快照读写 | MCU | ⬜ |
| 0.6 | `arbiter.c`：`nav_mode` 枚举；`DIRECT` 仍用 `jetson_cmd` | MCU | ⬜ |
| 0.7 | `FREERTOS_HEAP_BYTES` 32KB → **40KB**；编译通过 `freertos_heap.c` 溢出检查 | MCU | ⬜ |
| 0.8 | Jetson `local_goal_bridge` 发假 goal；gateway 解析 `0x12` | Jetson | ⬜ |
| 0.9 | 验证：`0x01` 直接控车不受影响；`offset_valid` 时间同步仍正常 | 联调 | ⬜ |

**验收**：`nav_mode=DIRECT` 下车可动；`0x12` topic 有数据。

---

### 阶段 1 — MCU odom + Pure Pursuit（先不通 DWA）

**目标**：`LOCAL_NAV` 下能追到 body frame 目标点（开阔场地直线）。

| # | 待办 | 负责侧 | 状态 |
|---|------|--------|------|
| 1.1 | `mcu_odom.c`：CAN `0x221` 线速度 + 自旋速度积分 (x,y,θ) | MCU | ⬜ |
| 1.2 | `vLocalPlannerTask` 20ms；栈 **512w**，优先级介于 Sensor 与 Motion | MCU | ⬜ |
| 1.3 | Pure Pursuit / carrot follower（不用 DWA）输出 `(v,ω)` | MCU | ⬜ |
| 1.4 | `arbiter`：`LOCAL_NAV` 时 `output` 来自 `local_cmd` | MCU | ⬜ |
| 1.5 | `0x12` 填真实 pose、`plan_status=TRACKING/REACHED` | MCU | ⬜ |
| 1.6 | Jetson 发 body frame sub-goal（前方 1m） | Jetson | ⬜ |
| 1.7 | `RTOS_PrintTaskStackWatermarks`：LocNav 栈余量 >100w | MCU | ⬜ |

**验收**：sub-goal 在前方 1m，车能到达；`plan_status` 变 REACHED。

---

### 阶段 2 — 超声接入 + 简单避障

**目标**：有障碍时减速/绕开，`BLOCKED` 上报。

| # | 待办 | 负责侧 | 状态 |
|---|------|--------|------|
| 2.1 | DWA/追踪器代价中加入四路 `DistSnapshot` → `dist()` | MCU | ⬜ |
| 2.2 | `plan_status=BLOCKED` 当无可行速度 | MCU | ⬜ |
| 2.3 | `ARBITER_IGNORE_DIST_SENSOR` 改 **0** | MCU | ⬜ |
| 2.4 | 挡板测试：前方障碍时减速或停 | 联调 | ⬜ |
| 2.5 | Jetson 收到 BLOCKED 后停车或重发 sub-goal | Jetson | ⬜ |

**验收**：挡板前停车或绕行；Arbiter 进入限速/避障仍正常。

---

### 阶段 3 — 完整 DWA

**目标**：替换 Pure Pursuit 为动态窗口法。

| # | 待办 | 负责侧 | 状态 |
|---|------|--------|------|
| 3.1 | `dwa_planner.c`：Vm、Vd、Va 采样窗口 | MCU | ⬜ |
| 3.2 | 轨迹预测 + heading/dist/velocity 评价（定点） | MCU | ⬜ |
| 3.3 | 采样 **8×8 或 10×10**；单次规划 <15ms | MCU | ⬜ |
| 3.4 | `dwa_config.h` 参数：v_max 与 `ARBITER_MAX_SPEED_MM_S` 对齐 | MCU | ⬜ |
| 3.5 | 栈水位不足则栈 →640/768w 或堆 →44KB | MCU | ⬜ |
| 3.6 | 动静态障碍场景联调 | 联调 | ⬜ |

**验收**：绕障平滑；无栈溢出；`[RTOS] LocNav stack free` 健康。

---

### 阶段 4 — Jetson 全局规划闭环

**目标**：AMCL + 全局路径 + 连续 sub-goal。

| # | 待办 | 负责侧 | 状态 |
|---|------|--------|------|
| 4.1 | Nav2 / A* 发布 `/plan` | Jetson | ⬜ |
| 4.2 | `local_goal_bridge` 20Hz，到达阈值切换下一 sub-goal | Jetson | ⬜ |
| 4.3 | `nav_feedback` 与 `/odom` 时间轴对齐（time_sync offset） | Jetson | ⬜ |
| 4.4 | 全流程：定位 → 规划 → 走完全路径 | 联调 | ⬜ |
| 4.5 | 可选：1h soak + bag 记录 | 联调 | ⬜ |

**验收**：室内/开阔场地完成多点导航；心跳丢失时 MCU 降级停车。

---

## 8. 风险与检查项

| 风险 | 缓解 |
|------|------|
| ucHeap 不足 | 阶段 0 堆增至 40KB；监控栈水位 |
| mem1 以太网占满 | DWA 不用 `mymalloc(SRAMIN)`；勿增大 ETH_RXBUFNB |
| 教程式全轨迹存储爆 RAM | 仅保留最优轨迹；定点粗采样 |
| 绕过 Arbiter 不安全 | 强制 `local_cmd` → `Arbiter_Process` |
| goal 用 map 坐标难实现 | **统一 body frame** 下发 |
| Jetson 断连 | `hold_ms` 超时 + 心跳丢失 → 停车 |

---

## 9. 调优入口速查

| 需求 | 文件 / 宏 |
|------|-----------|
| FreeRTOS 堆 | `User/FreeRTOSConfig.h` → `FREERTOS_HEAP_BYTES` |
| 新任务栈 | `APP/freertos/rtos_tasks.h` → `LOCAL_PLANNER_TASK_STACK_SIZE` |
| 片内池 | `Malloc/malloc.h` → `MEM1_MAX_SIZE` |
| DWA 参数 | `APP/local_nav/dwa_config.h`（待建） |
| 栈监控 | `RTOS_VERBOSE_STACK_LOG=1`，`rtos_debug.c` |
| 三池使用率 | `my_mem_perused(SRAMIN/SRAMEX/SRAMCCM)` |

---

## 10. 附录：变更记录

| 版本 | 日期 | 内容 |
|------|------|------|
| v0.1 | 2026-06-22 | 初版：内存基线 + 协议扩展 + 四阶段待办 |
| v0.1.1 | 2026-06-22 | 重命名为 `MCU局部导航DWA部署路线.md`；修复 UTF-8 编码 |
