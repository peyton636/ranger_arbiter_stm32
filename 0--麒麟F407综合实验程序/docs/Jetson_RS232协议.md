# Jetson 与 STM32B RS232（USART2）通信协议

| 元数据 | 值 |
|--------|-----|
| **协议版本** | **v1.4** |
| **文档日期** | 2026-06-15 |
| **状态** | 已实现（`JETSON_LINK_CAN=0`）；**v1.5 时间同步 RS232 联调通过（2026-06-15）** |

本文档定义 **Jetson / 上位机 SoC** 经 **RS232/TTL 串口** 与 STM32B（麒麟 F407 仲裁板）通信的协议。

硬件接线与 CAN/串口切换见 [硬件连接与通信协议.md](./硬件连接与通信协议.md) **§2.2**。

> **与 CAN 的关系**：应用层 V3 帧（type 0x01/0x02/0x03）字段定义与 [Jetson_CAN协议.md](./Jetson_CAN协议.md) **§4~6 完全一致**；8 字节服务载荷也与 CAN 相同，仅传输封装不同（见 **§3**）。

> **联调问题、RTT/offset 说明**：见 [Jetson时间同步联调记录.md](./Jetson时间同步联调记录.md)。

---

## 1. 物理层

| 项目 | 约定 |
|------|------|
| MCU 外设 | **USART2**（代码函数名 `USART3_*`） |
| 引脚 | **PA2 = TX**，**PA3 = RX**（F407 板） |
| 电平 | **3.3V TTL** |
| 波特率 | **115200** 8N1 |
| 编译开关 | `APP/jetson_can/jetson_can.h` → `#define JETSON_LINK_CAN 0` |

---

## 2. 启用步骤

| 步骤 | 说明 |
|------|------|
| 1 | `jetson_can.h` 中 `JETSON_LINK_CAN` 设为 **0** |
| 2 | 重新编译烧录 |
| 3 | 接线：**PA2 → Jetson RX**，**PA3 → Jetson TX**，共 GND |

| 模块 | 作用 |
|------|------|
| `usart3.c` | V3 编解码；服务帧 `0xA5` 封装 |
| `jetson_can.c` | GPS/故障/时间同步/状态查询 |
| `rtos_tasks.c` | `JetsonTask` / `GpsTask` |
| `app_boot.c` | `USART3_Init()` |

---

## 3. 传输层总览

RS232 字节流 **混传** 两类帧，靠首字节魔数区分：

| 魔数 | 长度 | 用途 | 等价 CAN |
|:----:|:----:|------|----------|
| **0xAA** | **24 B** | V3 应用帧（控制/状态/扩展） | 0x101 / 0x102 / 0x103 |
| **0xA5** | **11 B** | 服务帧（GPS/时间/故障/状态等） | 0x104~0x10B |

**V3 校验**：`byte23 = byte0 ⊕ byte1 ⊕ … ⊕ byte22`

**V3 帧类型（byte1）**：

| 取值 | 方向 | 含义 |
|:--:|:----:|------|
| **0x01** | Jetson → STM32 | 下行控制 |
| **0x02** | STM32 → Jetson | 上行状态 |
| **0x03** | STM32 → Jetson | 上行扩展 |

---

## 4. 通信周期

| 数据 | 帧格式 | 周期 |
|------|--------|------|
| Jetson → STM32 控制 | V3 type=0x01 | 建议 **≤20 ms**，`seq` 递增 |
| STM32 → Jetson 状态 | V3 type=0x02 | **~20 ms**（与 0x03 交替） |
| STM32 → Jetson 扩展 | V3 type=0x03 | **~40 ms**（与 0x02 交替） |
| STM32 → Jetson GPS | 0xA5 + 0x104/105/106 | **~100 ms** |
| STM32 → Jetson 故障 | 0xA5 + 0x109 | 事件触发 + 有故障时 **1 Hz** |
| Jetson → STM32 时间同步 | 0xA5 + 0x107 | 建议 **10 s** 或事件触发 |
| Jetson → STM32 状态查询 | 0xA5 + 0x10A | 按需 |
| 心跳超时 | — | **300 ms** 无新 0x01 seq → `DEGRADED` |

---

## 5. V3 下行控制帧（FRAME_TYPE = 0x01）

等价 CAN ID **0x101**。字段与 [Jetson_CAN协议.md §4](./Jetson_CAN协议.md) 相同。

| | 0 | 1 | 2 | 3 | 4~5 | 6~7 | 8~9 | 10 | 11 | 12 | 13 | 14~22 | 23 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | HEADER | FRAME_TYPE | SEQ | MODE_REQ | V | OMEGA | STEER | MOTION_MODEL | LIGHT_ENABLE | LIGHT_MODE | CLEAR_ERROR | RSV | XOR_CHK |
| **说明** | 固定 0xAA | 固定 0x01 | 序号 | 控制模式 | 线速度 mm/s，大端 s16 | 角速度，×1000 rad/s | 转角，×1000 rad | 运动模型 | 灯开关 | 灯模式 | 清错码 | 填 0 | XOR(0~22) |

> byte4~9 经 **运动仲裁** 后写入底盘 CAN **0x111**。

#### HEADER（byte0）

| 取值 | 含义 |
|:--:|------|
| **0xAA** | V3 帧头；收到后连续收满 24 字节 |

#### FRAME_TYPE（byte1）

| 取值 | 含义 |
|:--:|------|
| **0x01** | 下行运动控制（本帧） |

#### SEQ（byte2）

| 取值 | 含义 |
|:--:|------|
| 0~255 | 每发一帧递增；**变化即刷新心跳**；300 ms 无新 seq → DEGRADED |

#### MODE_REQ（byte3）

| 取值 | 含义 | STM32 动作 |
|:--:|------|------------|
| **0x00** | 待机 | CAN **0x421 = 0x00** |
| **0x01** | CAN 指令控制 | CAN **0x421 = 0x01** |
| **0x02** | 遥控请求 | 当前按待机处理 |

#### V / OMEGA / STEER（byte4~9）

| 字段 | 字节 | 类型 | 含义 |
|------|:--:|------|------|
| V | 4~5 | s16 大端 | 目标线速度，**mm/s** |
| OMEGA | 6~7 | s16 大端 | 目标角速度，**0.001 rad/s** |
| STEER | 8~9 | s16 大端 | 目标转角，**0.001 rad** |

#### MOTION_MODEL（byte10）

| 取值 | 含义 | 底盘 CAN 0x141 |
|:--:|------|:--:|
| **0x00** | 前后阿克曼 | 0x00 |
| **0x01** | 斜移 | 0x01 |
| **0x02** | 自旋 | 0x02 |
| **0x03** | 驻车 | 0x03 |

#### LIGHT_ENABLE（byte11）

| 取值 | 含义 |
|:--:|------|
| **0** | 关 |
| **1** | 开 |

#### LIGHT_MODE（byte12）

| 取值 | 含义 |
|:--:|------|
| **0** | 模式 0 |
| **1** | 模式 1 |

#### CLEAR_ERROR（byte13）

| 取值 | 含义 |
|:--:|------|
| **0x00** | 清除全部驱动器相关故障 |
| **0x01~0x08** | 清除 1~8 号电机驱动通讯故障 |
| **0x09** | 电池欠压 |
| **0x0A** | 遥控丢失 |
| **0x0B~0x0E** | 5~8 号转向校准 |
| **0x0F** | 过流 |
| **0x10** | 过温 |

非 0 时触发一次清错，发底盘 CAN **0x441**。

#### RSV（byte14~22）

| 取值 | 含义 |
|:--:|------|
| **0x00** | 保留，填 0 |

#### XOR_CHK（byte23）

| 取值 | 含义 |
|:--:|------|
| 计算值 | 前 23 字节逐字节异或 |

---

## 6. V3 上行状态帧（FRAME_TYPE = 0x02）

等价 CAN ID **0x102**。STM32 → Jetson，周期 ~20 ms（与 0x03 交替）。

| | 0 | 1 | 2 | 3 | 4 | 5 | 6~7 | 8~9 | 10~11 | 12~13 | 14~15 | 16~17 | 18~19 | 20~21 | 22 | 23 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | HEADER | FRAME_TYPE | SEQ | SAFETY_STATE | LINK_STATE | LIMIT_FACTOR | FB_V | FB_OMEGA | FB_STEER | SONAR_F | SONAR_B | SONAR_L | SONAR_R | BAT_V | SOC | XOR_CHK |
| **说明** | 0xAA | 0x02 | TX序号 | 安全模式 | 链路状态 | 限速0~100 | 线速度 mm/s | 角速度 | 转角 | 前距 mm | 后距 mm | 左距 mm | 右距 mm | 电池0.1V | 电量% | XOR |

#### HEADER / FRAME_TYPE（byte0~1）

| 字节 | 取值 | 含义 |
|:--:|:--:|------|
| 0 | **0xAA** | V3 帧头 |
| 1 | **0x02** | 上行状态 |

#### SEQ（byte2）

| 取值 | 含义 |
|:--:|------|
| 0~255 | STM32 发送序号，递增 |

#### SAFETY_STATE（byte3）

| 取值 | 模式 |
|:--:|------|
| **0x01** | NORMAL |
| **0x02** | SPEED_LIMIT |
| **0x03** | DEGRADED / RECOVERING |
| **0x04** | EMERGENCY |

#### LINK_STATE（byte4，位域）

| Bit | 名称 | =1 含义 |
|:--:|------|---------|
| [0] | JETSON_HB_LOST | 心跳 >300 ms |
| [1] | CHASSIS_FAULT | 底盘 system_status=0x02 |
| [2] | BEEP_ACTIVE | 测距报警蜂鸣器激活 |
| [7:3] | RSV | 0 |

#### LIMIT_FACTOR（byte5）

| 取值 | 含义 |
|:--:|------|
| 0~100 | 非 SPEED_LIMIT 时为 **100**；SPEED_LIMIT 时按最近障碍 60~180 mm 线性映射 |

#### FB_V / FB_OMEGA / FB_STEER（byte6~11）

| 字段 | 字节 | 类型 | 含义 |
|------|:--:|------|------|
| FB_V | 6~7 | s16 大端 | 底盘反馈线速度，**mm/s** |
| FB_OMEGA | 8~9 | s16 大端 | 反馈角速度，**0.001 rad/s** |
| FB_STEER | 10~11 | s16 大端 | 反馈转角，**0.001 rad** |

#### SONAR_F / B / L / R（byte12~19）

| 字段 | 字节 | 类型 | 含义 |
|------|:--:|------|------|
| SONAR_* | 各 2 字节 | u16 大端 | 四向测距，**mm**；无效 **0xFFFF**；有效 0~60000 |

#### BAT_V / SOC（byte20~22）

| 字段 | 字节 | 类型 | 含义 |
|------|:--:|------|------|
| BAT_V | 20~21 | u16 大端 | 电池电压，**0.1 V** |
| SOC | 22 | u8 | 电量 **%** |

#### XOR_CHK（byte23）

| 取值 | 含义 |
|:--:|------|
| 计算值 | XOR(byte0..22) |

---

## 7. V3 上行扩展帧（FRAME_TYPE = 0x03）

等价 CAN ID **0x103**。STM32 → Jetson，周期 ~40 ms（与 0x02 交替）。

| | 0 | 1 | 2 | 3~4 | 5~6 | 7~8 | 9~10 | 11~12 | 13~14 | 15~16 | 17~18 | 19 | 20 | 21~22 | 23 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | HEADER | FRAME_TYPE | SEQ | WHEEL_RF | WHEEL_RR | WHEEL_LR | WHEEL_LF | STEER_RF | STEER_RR | STEER_LR | STEER_LF | MOTOR_TEMP_MAX | DRIVER_STATE_OR | RSV | XOR_CHK |
| **说明** | 0xAA | 0x03 | TX序号 | 右前轮速 | 右后轮速 | 左后轮速 | 左前轮速 | 右前转角 | 右后转角 | 左后转角 | 左前转角 | 最高温℃ | 驱动状态OR | 填0 | XOR |

> 四轮速单位 **mm/s**（s16 大端）；四轮转角单位 **0.001 rad**（s16 大端）。

#### HEADER / FRAME_TYPE（byte0~1）

| 字节 | 取值 | 含义 |
|:--:|:--:|------|
| 0 | **0xAA** | V3 帧头 |
| 1 | **0x03** | 上行扩展 |

#### WHEEL_RF / RR / LR / LF（byte3~10）

| 字段 | 字节 | 含义 |
|------|:--:|------|
| WHEEL_RF | 3~4 | 右前轮速 mm/s |
| WHEEL_RR | 5~6 | 右后轮速 mm/s |
| WHEEL_LR | 7~8 | 左后轮速 mm/s |
| WHEEL_LF | 9~10 | 左前轮速 mm/s |

#### STEER_RF / RR / LR / LF（byte11~18）

| 字段 | 字节 | 含义 |
|------|:--:|------|
| STEER_RF | 11~12 | 右前转角 |
| STEER_RR | 13~14 | 右后转角 |
| STEER_LR | 15~16 | 左后转角 |
| STEER_LF | 17~18 | 左前转角 |

#### MOTOR_TEMP_MAX（byte19）

| 类型 | 含义 |
|:--:|------|
| s8 | 8 路电机最高温度，**℃** |

#### DRIVER_STATE_OR（byte20，位域 OR 汇总）

| Bit | 名称 | =1 含义 |
|:--:|------|---------|
| [0] | VLOW | 电源电压过低 |
| [1] | MOT_OT | 电机过温 |
| [2] | DRV_OC | 驱动器过流 |
| [3] | DRV_OT | 驱动器过温 |
| [4] | SENSOR | 传感器异常 |
| [5] | ERR | 驱动器错误 |
| [6] | EN | 驱动器使能 |

#### RSV / XOR_CHK（byte21~23）

| 字节 | 取值 | 含义 |
|:--:|:--:|------|
| 21~22 | **0x00** | 保留 |
| 23 | 计算值 | XOR(byte0..22) |

---

## 8. SVC 服务帧（魔数 0xA5，11 字节）

用于承载与 CAN **相同的 8 字节载荷**，外加 CAN ID 标识。见 `0xA5` 后连续收满 **11 字节**。

| | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | MAGIC | ID_H | ID_L | P0 | P1 | P2 | P3 | P4 | P5 | P6 | P7 |
| **说明** | 固定 0xA5 | CAN ID 高字节 | CAN ID 低字节 | 载荷 byte0 | 载荷 byte1 | … | … | … | … | … | 载荷 byte7 |

**CAN ID** = `(byte1 << 8) | byte2`；**byte3~10** 即 PAYLOAD，按 ID 查 **§9~§14**。

#### MAGIC（byte0）

| 取值 | 含义 |
|:--:|------|
| **0xA5** | 服务帧头；收到后连续收满 11 字节 |

#### ID_H + ID_L（byte1~2）— 逻辑 CAN ID

| ID | 方向 | 含义 |
|:--:|:----:|------|
| **0x104** | STM32 → Jetson | GPS 帧 A |
| **0x105** | STM32 → Jetson | GPS 帧 B |
| **0x106** | STM32 → Jetson | GPS 帧 C |
| **0x107** | Jetson → STM32 | 时间同步请求 |
| **0x108** | STM32 → Jetson | 时间同步响应 |
| **0x109** | STM32 → Jetson | 故障上报 |
| **0x10A** | Jetson → STM32 | 状态查询请求 |
| **0x10B** | STM32 → Jetson | 状态查询响应 |

> GPS 三帧发送顺序：`0x104 → 0x105 → 0x106`，帧间间隔约 **2 ms**。

**示例**：

```text
GPS-A：  A5 01 04 [8B GPS-A 载荷]
时间 QUERY：A5 01 07 01 00 00 00 00 00 00 00
状态查询：  A5 01 0A 01 00 00 00 00 00 00 00
```

---

## 9. GPS 服务帧（ID = 0x104 / 0x105 / 0x106）

PAYLOAD = SVC 帧 byte3~10（P0~P7）。三帧独立 ID，不做组帧重组。

### 9.1 GPS 帧 A（ID = 0x104）

| | P0 | P1 | P2 | P3 | P4~P5 | P6~P7 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | MAGIC | FRAG_IDX | FLAGS | NUM_SV | HDOP_X100 | SPEED_CMS |
| **说明** | 0xA4 | 固定 0 | 定位标志 | 卫星数 | HDOP×100 | 速度 cm/s |

#### MAGIC / FRAG_IDX（P0~P1）

| 取值 | 含义 |
|:--:|------|
| P0 = **0xA4** | GPS 分片魔数 |
| P1 = **0x00** | 分片序号 0 |

#### FLAGS（P2，位域）

| Bit | 名称 | =1 含义 |
|:--:|------|---------|
| [0] | POS_VALID | 位置有效 |
| [1] | VEL_VALID | 速度有效 |
| [2] | HEADING_VALID | 航向有效 |
| [3] | FIX_3D_LIKE | 定位质量满足有效 fix |
| [4] | USEFULL | NMEA `A` 有效标志 |
| [7:5] | RSV | 0 |

#### HDOP_X100 / SPEED_CMS（P4~P7）

| 字段 | 类型 | 说明 |
|------|------|------|
| HDOP_X100 | u16 大端 | 无效 **0xFFFF** |
| SPEED_CMS | u16 大端 | 速度 **cm/s**，0~65535 |

### 9.2 GPS 帧 B（ID = 0x105）

| | P0 | P1 | P2~P5 | P6~P7 |
|:---:|:---:|:---:|:---:|:---:|
| **字段** | MAGIC | FRAG_IDX | LAT_E7 | HEADING_X100 |
| **说明** | 0xA4 | 固定 1 | 纬度×1e7，s32 大端 | 航向×100，s16 大端 |

### 9.3 GPS 帧 C（ID = 0x106）

| | P0 | P1 | P2~P5 | P6~P7 |
|:---:|:---:|:---:|:---:|:---:|
| **字段** | MAGIC | FRAG_IDX | LON_E7 | ALT_DM |
| **说明** | 0xA4 | 固定 2 | 经度×1e7，s32 大端 | 海拔×10 dm，s16 大端 |

#### ALT_DM（P6~P7）

| 取值 | 含义 |
|:--:|------|
| 有效 | 海拔(m) = ALT_DM / 10.0，范围 -1000~+1000 m |
| **0x7FFF** | 无效 / 无 fix |

---

## 10. 时间同步（ID = 0x107 / 0x108）

### 10.1 时间请求（ID = 0x107，Jetson → STM32）

P0 = **CMD** 选择子，决定后续字节布局。

#### CMD 取值（P0）

| 取值 | 名称 | 有效字节 | 含义 |
|:--:|------|:--------:|------|
| **0x01** | QUERY | 1 | v1.4 单次查 MCU tick + UTC |
| **0x02** | START | 8 | v1.5 开启对时会话 |
| **0x03** | PING | 8 | v1.5 会话心跳 / 测 RTT |
| **0x04** | STOP | 2 | v1.5 结束会话 |

#### QUERY（P0 = 0x01）

| | P0 | P1~P7 |
|:---:|:---:|:---:|
| **字段** | CMD | RSV |
| **说明** | 0x01 | 填 0 |

#### START（P0 = 0x02）

| | P0 | P1 | P2~P5 | P6~P7 |
|:---:|:---:|:---:|:---:|:---:|
| **字段** | CMD | SESSION_ID | JETSON_MONO_MS | RSV |
| **说明** | 0x02 | 会话 ID | Jetson 单调 ms，u32 大端 | 填 0 |

#### PING（P0 = 0x03）

| | P0 | P1 | P2~P5 | P6~P7 |
|:---:|:---:|:---:|:---:|:---:|
| **字段** | CMD | PING_SEQ | JETSON_MONO_MS | RSV |
| **说明** | 0x03 | 递增序号 | Jetson 单调 ms，u32 大端 | 填 0 |

#### STOP（P0 = 0x04）

| | P0 | P1 | P2~P7 |
|:---:|:---:|:---:|:---:|
| **字段** | CMD | SESSION_ID | RSV |
| **说明** | 0x04 | 须与 START 一致 | 忽略 |

### 10.2 时间响应（ID = 0x108，STM32 → Jetson）

**两种布局互斥**，按请求 CMD 解析。

#### QUERY 应答（响应 CMD=0x01）

| | P0~P3 | P4~P7 |
|:---:|:---:|:---:|
| **字段** | SYSTEM_TICK_MS | UTC_TIME_SEC |
| **说明** | MCU 单调 ms，u32 大端 | Unix 秒，u32 大端；无 GPS=0 |

#### START / PING 应答

| | P0 | P1 | P2~P5 | P6 | P7 |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | CMD_ECHO | ECHO_BYTE | MCU_TICK_RX_MS | TIME_FLAGS | MCU_PROC_100US |
| **说明** | 回显 0x02/0x03 | 回显 SEQ/SESSION | 收到瞬间 MCU tick | 时间标志 | 处理耗时×100μs |

#### TIME_FLAGS（P6，位域）

| Bit | 名称 | =1 含义 |
|:--:|------|---------|
| [0] | GPS_UTC_VALID | GPS fix 且 UTC 有效 |
| [1] | RTC_VALID | RTC 有效（预留） |
| [2] | RTC_SYNCED_FROM_GPS | RTC 已由 GPS 同步（预留） |
| [7:3] | RSV | 0 |

> RS232 模式下 **禁止在 USART2 中断内发送 0x108**；应在 JetsonTask 任务上下文回复。详见 [Jetson时间同步联调记录.md §2.5](./Jetson时间同步联调记录.md)。

---

## 11. 故障上报（ID = 0x109，STM32 → Jetson）

事件触发 + 有故障时 1 Hz 保活。

| | P0 | P1 | P2 | P3 | P4~P5 | P6 | P7 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | FAULT_1 | FAULT_2 | FAULT_3 | FAULT_4 | FAULT_TS | CHASSIS_SYS | CHASSIS_FAULT |
| **说明** | 主故障码 | 次故障码 | 第三码 | 第四码 | tick 低16位 | 底盘0x211状态 | fault低字节 |

#### FAULT_1 常用取值（P0）

| 取值 | 含义 |
|:--:|------|
| **0x00** | 无故障 |
| **0x02** | GPS 无 fix |
| **0x03** | 四路测距均无效 |
| **0x10** | Jetson 心跳丢失 |
| **0x20** | 底盘 system_status=0x02 |
| **0x21** | 底盘 fault_info 非 0 |
| **0x30** | 紧急停车 / 四向危险 |
| **0x40** | 最近距未知 |
| **0x50** | 时间同步会话丢失 |

完整错误码见 [Jetson_CAN协议.md §9](./Jetson_CAN协议.md)。

---

## 12. 状态查询（ID = 0x10A / 0x10B）

### 12.1 状态查询请求（ID = 0x10A，Jetson → STM32）

| | P0 | P1~P7 |
|:---:|:---:|:---:|
| **字段** | QUERY | RSV |
| **说明** | 0x01 | 填 0 |

#### QUERY（P0）

| 取值 | 含义 |
|:--:|------|
| **0x01** | 查询关键状态快照 |

### 12.2 状态查询响应（ID = 0x10B，STM32 → Jetson）

收到 0x10A 后 **50 ms 内** 回复。不等同完整 0x02/0x03，仅为 8B 快速快照。

| | P0 | P1 | P2~P3 | P4~P5 | P6 | P7 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | SAFETY_STATE | LINK_STATE | FB_V | NEAREST_MM | LIMIT_FACTOR | FLAGS |
| **说明** | 同 0x02 byte3 | 同 0x02 byte4 | 线速度 mm/s | 最近距离 mm | 限速 0~100 | 标志位 |

#### SAFETY_STATE / LINK_STATE（P0~P1）

取值同 **§6** 上行状态帧 byte3~4。

#### NEAREST_MM（P4~P5）

| 取值 | 含义 |
|:--:|------|
| 0~60000 | 最近障碍距离 mm |
| **0xFFFF** | 无效 |

#### FLAGS（P7，位域）

| Bit | 名称 | =1 含义 |
|:--:|------|---------|
| [0] | GPS_FIX | `GPS_HasFix()` |
| [1] | CHASSIS_CAN_ACTIVE | mode_control == 0x01 |
| [2] | DIST_VALID | 至少一路测距有效 |
| [7:3] | RSV | 0 |

---

## 13. V3 与底盘 CAN 映射

| V3 字段 | 底盘 CAN |
|---------|----------|
| MODE_REQ | 0x421 |
| MOTION_MODEL | 0x141 |
| LIGHT_ENABLE / LIGHT_MODE | 0x121 |
| CLEAR_ERROR | 0x441 |
| V / STEER / OMEGA | 0x111（经仲裁） |

---

## 14. Jetson 侧对接建议

### 14.1 串口读帧状态机

```text
IDLE:
  读 1 字节 b
  b==0xAA → 再读 23 字节 → V3 帧（校验 XOR，看 byte1 类型）
  b==0xA5 → 再读 10 字节 → 服务帧（解析 ID + 8B payload）
  其他   → 丢弃，回 IDLE
```

### 14.2 发送

1. 控制：每 **≤20 ms** 发 V3 **0x01**，**seq 递增**；
2. 时间同步：发 `0xA5` + ID **0x0107** + 8B 载荷（QUERY/PING 等）；
3. 状态查询：发 `0xA5` + ID **0x010A** + `[0x01,0,0,0,0,0,0,0]`；
4. 同时解析 **V3 0x02/0x03** 与 **0xA5 服务帧**。

### 14.3 Python 解析示例

```python
def read_v3_frame(ser):
    b = ser.read(24)
    if len(b) != 24 or b[0] != 0xAA:
        return None
    chk = 0
    for i in range(23):
        chk ^= b[i]
    if chk != b[23]:
        return None
    return b

def read_service_frame(ser):
    b = ser.read(11)
    if len(b) != 11 or b[0] != 0xA5:
        return None
    can_id = (b[1] << 8) | b[2]
    return can_id, b[3:11]

def send_time_sync_req(ser):
    ser.write(bytes([0xA5, 0x01, 0x07, 0x01, 0, 0, 0, 0, 0, 0, 0]))

def parse_time_rsp_query(payload):  # 0x108, QUERY 应答
    tick = (payload[0] << 24) | (payload[1] << 16) | (payload[2] << 8) | payload[3]
    utc  = (payload[4] << 24) | (payload[5] << 16) | (payload[6] << 8) | payload[7]
    return tick, utc
```

### 14.4 单位约定

| 量 | 单位 |
|----|------|
| 速度 | **mm/s** |
| 角度 | **×1000**（0.001 rad） |
| 测距 u16 | **0xFFFF** = 无效 |

---

## 附录 A：CLEAR_ERROR 码（0x441）

V3 下行 byte13 取值，详见 **§5**。

| 码 | 含义 |
|:--:|------|
| 0x00 | 清除全部驱动器相关故障 |
| 0x01~0x08 | 清除 1~8 号电机驱动通讯故障 |
| 0x09 | 电池欠压 |
| 0x0A | 遥控丢失 |
| 0x0B~0x0E | 5~8 号转向校准 |
| 0x0F | 过流 |
| 0x10 | 过温 |

---

## 附录 B：源码索引

| 功能 | 函数 |
|------|------|
| 下行解析 | `Arbiter_ParseJetsonCmd()` |
| 上行 0x02 | `USART3_SendV3StatusFrame()` |
| 上行 0x03 | `USART3_SendV3DetailFrame()` |
| 服务帧发送 | `USART3_SendServiceFrame()` / `JetsonLink_Deliver8()` |
| 服务帧接收 | `USART3_ProcessRxByte()` → `USART3_GetServiceRequest()` |
| GPS 打包 | `JetsonCAN_SendGps()` |
| 故障上报 | `JetsonCAN_ServiceFault()` |
| 时间/查询响应 | `JetsonCAN_HandleServiceRequest()` |
| 串口 RX 中断 | `USART2_IRQHandler` |

常量：`APP/can/can2.h` → `JETSON_RS232_SVC_MAGIC`（0xA5）、`JETSON_RS232_SVC_LEN`（11）。

---

## 附录 C：变更记录

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| v1.0 | 2026-05-28 | 初版：V3 24B（0x01/0x02/0x03） |
| v1.1 | 2026-05-28 | 新增 **0xA5 服务帧**；支持 Jetson 发 0x107/0x10A |
| v1.2 | 2026-05-28 | 交叉引用 CAN v1.5 时间同步 |
| v1.3 | 2026-06-15 | RS232 时间同步联调通过 |
| **v1.4** | **2026-06-15** | **全部帧格式改为横排字节表**；合并 SVC 载荷定义入本文 |

---

## 相关文档

- [硬件连接与通信协议.md](./硬件连接与通信协议.md)
- [Jetson_CAN协议.md](./Jetson_CAN协议.md)
- [Jetson时间同步联调记录.md](./Jetson时间同步联调记录.md)
- [rtos移植.md](./rtos移植.md)
