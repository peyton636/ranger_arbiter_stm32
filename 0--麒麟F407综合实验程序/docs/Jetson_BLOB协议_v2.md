# Jetson 与 MCU BLOB 二进制协议（v2.0 草案）

| 元数据 | 值 |
|--------|-----|
| **协议版本** | **v2.0-draft.5.7** |
| **文档日期** | 2026-06-16 |
| **物理层** | RS232 115200 8N1（USART2）或 CAN2 500 kbps（编译期二选一） |
| **编码** | 多字节整数 **大端 BE**；`#pragma pack(1)` |

> **横排表约定**：与 [Jetson_RS232协议.md](./Jetson_RS232协议.md) 相同——首列留空；多字节范围写 `4-5`（ASCII 连字符，勿用 `~` 防删除线）；**分隔行 `|:---:` 个数必须与列数一致**。

> **与 V3 的关系**：BLOB **PAYLOAD（struct）** 为 v2 新定义；**传输层**沿用 V3 思路——**同一条线格式**，RS232 整帧发送，CAN 按 ID 8 字节顺序分片。时间同步等 **0xA5 / 0x107~0x108 服务帧保持不变**，与 BLOB 混传。

硬件切换见 [硬件连接与通信协议.md](./硬件连接与通信协议.md) **§2.2**（`JETSON_LINK_CAN` 1=CAN2，0=USART2）。

---

## 0. 传输封装（所有 BLOB 业务帧共用）

### 0.1 线格式（Wire Image）

RS232 与 CAN **共用同一字节序列**：先 9 字节头，再 `LEN` 字节 PAYLOAD。

```text
[0xAB][VER=0x01][MSG_ID][SEQ][LEN_H][LEN_L][FRAG_IDX][FRAG_CNT][FLAGS][PAYLOAD...]
```

| | 0 | 1 | 2 | 3 | 4-5 | 6 | 7 | 8 | 9+ |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | MAGIC | VER | MSG_ID | SEQ | LEN | FRAG_IDX | FRAG_CNT | FLAGS | PAYLOAD |
| **类型** | u8 | u8 | u8 | u8 | u16 BE | u8 | u8 | u8 | struct |
| **说明** | 0xAB | 0x01 | 消息类型 | 0-255 | PAYLOAD字节数 | 固定0 | CAN分片数 | 填0 | 见各章 |

- **线长** = `9 + LEN`（例：agv_control_t → 9+14=**23 B**）
- **LEN** 仅计 PAYLOAD，不含 9 字节头
- RS232 单帧：`FRAG_IDX=0`，`FRAG_CNT=1`
- CAN 多片：发送前设 `FRAG_CNT = ceil(线长 / 8)`，`FRAG_IDX` 仍为 0（分片顺序靠 CAN 帧到达次序，同 V3）

### 0.2 RS232 模式（`JETSON_LINK_CAN=0`）

字节流 **混传**，首字节魔数分支（与现有 V3 并存）：

| 魔数 | 线长 | 用途 | 文档 |
|:----:|:----:|------|------|
| **0xAA** | 24 B | 旧 V3 应用帧 | [Jetson_RS232协议.md](./Jetson_RS232协议.md) |
| **0xA5** | 11 B | 服务帧（时间同步/GPS/故障等） | 同上 §8 |
| **0xAB** | **9+LEN** | **BLOB v2**（本协议） | 本文 |

> **MCU 收包优先级（`JETSON_USE_BLOB_V2=1`）**：IDLE 下 **0xAB → 0xA5**，**忽略 0xAA**（防 BLOB payload 误同步进 V3）。对比 V3 下行需 Jetson `use_blob_v2:=false` 且 MCU 启用 V3 分支。详见 **附录 C §C.9**。

**0xAB 收包状态机**（与 V3 收 0xAA 类似）：

1. 读到 **0xAB** → 再收 **8** 字节（凑齐 9 字节头）
2. 从 byte4-5 解析 **LEN**
3. 再收 **LEN** 字节 PAYLOAD
4. 按 **MSG_ID** 解 struct

| 项目 | 约定 |
|------|------|
| 物理 | USART2，PA2=TX / PA3=RX，115200 8N1 |
| 发送 | 整段 `[9B头][PAYLOAD]` **一次写出**，无额外 XOR |
| 周期 | 见 **§0.4** |

### 0.3 CAN 模式（`JETSON_LINK_CAN=1`）

与 V3 的 **3×8 分片** 相同规则，只是线长可变：

1. 内存组装完整线格式 `wire[0 .. 9+LEN-1]`
2. 设 `wire[7] = FRAG_CNT = ceil((9+LEN) / 8)`
3. 选 **CAN ID** = `0x180 + MSG_ID`（见下表）
4. 按顺序发 `ceil(线长/8)` 帧，每帧 **8 B**；末帧不足补 **0x00**
5. 接收端同 ID 顺序拼接后，再按 **§0.1** 解析

| MSG_ID | CAN ID | 方向 | struct | PAYLOAD | 线长 | CAN帧数 |
|:--:|:--:|:----:|--------|:--:|:--:|:--:|
| **0x01** | **0x181** | Jetson->MCU | agv_control_t | 14 B | 23 B | 3 |
| **0x02** | **0x182** | MCU->Jetson | agv_motion_t | 40 B | 49 B | 7 |
| **0x03** | **0x183** | MCU->Jetson | mcu_status_t | 42 B | 51 B | 7 |
| **0x04** | **0x184** | MCU->Jetson | sensor_blob_t | 28 B | 37 B | 5 |
| **0x05** | **0x185** | MCU->Jetson | gps_compact_t | 32 B | 41 B | 6 |
| **0x06** | **0x186** | MCU->Jetson | agv_motor04_t | 44 B | 53 B | 7 |
| **0x07** | **0x187** | MCU->Jetson | agv_motor58_t | 44 B | 53 B | 7 |
| **0x08** | **0x188** | MCU->Jetson | agv_energy_t | 41 B | 50 B | 7 |
| **0x0B** | **0x18B** | MCU->Jetson | agv_motor_pos_t | 36 B | 45 B | 6 |
| **0x10** | **0x190** | Jetson->MCU | sensor_cfg_t | 8 B | 17 B | 3 |

> **0x107~0x10B**（时间同步/故障/状态查询）仍走 [Jetson_CAN协议.md](./Jetson_CAN协议.md) 原定义，**不**包在 0xAB 头里。

**CAN 分片示例**（MSG 0x01，线长 23 B → 3 帧，ID=0x181）：

| CAN序号 | byte0-7（线格式偏移） |
|:--:|------|
| 0 | `[0xAB][0x01][0x01][SEQ][0x00][0x0E][0x00][0x03]` |
| 1 | `[0x00][PAYLOAD 0-6]` |
| 2 | `[PAYLOAD 7-13][0x00][0x00]`（末帧补 0） |

### 0.4 建议通信周期

| 数据 | MSG | 周期 |
|------|:---:|:--:|
| Jetson 控制 | 0x01 | **≤20 ms** |
| 运动摘要 | 0x02 | **20 ms** |
| 电机 0-3 / 4-7 | 0x06 / 0x07 | 各 **40 ms**（交替） |
| 能源/里程 | 0x08 | **100 ms** |
| GPS | 0x05 | **100 ms** |
| 四路超声 | 0x04 | **20 ms** |
| MCU 状态 | 0x03 | **50 ms** |
| 全脉冲 | 0x0B | **1 Hz**（可选） |
| 时间同步 | 0xA5/0x107 | **10 s**（不变） |

---

## MSG_ID 总览

| MSG_ID | 类别 | 方向 | struct | PAYLOAD |
|:--:|------|:----:|--------|:--:|
| **0x01** | 底盘 | Jetson->MCU | agv_control_t | 14 B |
| **0x02** | 底盘 | MCU->Jetson | agv_motion_t | 40 B |
| **0x06** | 底盘 | MCU->Jetson | agv_motor04_t | 44 B |
| **0x07** | 底盘 | MCU->Jetson | agv_motor58_t | 44 B |
| **0x08** | 底盘 | MCU->Jetson | agv_energy_t | 41 B |
| **0x0B** | 底盘 | MCU->Jetson | agv_motor_pos_t | 36 B |
| **0x05** | GPS | MCU->Jetson | gps_compact_t | 32 B |
| **0x04** | 传感器 | MCU->Jetson | sensor_blob_t | 28 B |
| **0x10** | 传感器 | Jetson->MCU | sensor_cfg_t | 8 B |
| **0x03** | MCU | MCU->Jetson | mcu_status_t | 42 B |

**帧 stamp**：每个 struct **byte 0-3 = timestamp_ms（u32 BE）**，必须有。

---

## 1. 底盘

### 1.1 控制 agv_control_t（MSG 0x01，14 B）

| | 0-3 | 4-5 | 6-7 | 8-9 | 10 | 11 | 12 | 13 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | timestamp_ms | linear_vel | angular_vel | steer_angle | control_mode | motion_drive_info | clear_fault | light_info |
| **类型** | u32 BE | s16 BE | s16 BE | s16 BE | u8 | u8 | u8 | u8 |
| **说明** | 帧时刻 | 线速度mm/s | 角速度0.001rad/s | 转角0.001rad | 0待机1CAN | 运动+驱动模式 | 清错码 | 灯光 |

### 1.2 运动 agv_motion_t（MSG 0x02，40 B）

| | 0-3 | 4 | 5 | 6 | 7-10 | 11-12 | 13-18 | 19-26 | 27-34 | 35-39 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | timestamp_ms | system_info | motion_info | light_pack | fault_code | bat_v | vel_x3 | wheel_angle_x4 | wheel_speed_x4 | rsv |
| **类型** | u32 BE | u8 | u8 | u8 | u32 BE | u16 BE | s16x3 BE | s16x4 BE | s16x4 BE | u8x5 |
| **说明** | 帧时刻 | 系统状态 | 运动模式 | 灯光+计数 | 0x211故障 | 电压0.1V | 线角转 | 四轮角 | 四轮速 | 填0 |

### 1.3 电机 agv_motor04_t / agv_motor58_t（MSG 0x06/0x07，44 B）

| | 0-3 | 4-13 | 14-23 | 24-33 | 34-43 |
|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | timestamp_ms | motor0 | motor1 | motor2 | motor3 |
| **说明** | 帧时刻 | 10B紧凑体 | 10B | 10B | 10B |

motor 10B：`speed s16 | current s16 | voltage u16 | temp i8 | driver_status u8 | position_lo u16`

### 1.4 能源 agv_energy_t（MSG 0x08，41 B）

| | 0-3 | 4-19 | 20-33 | 34-40 |
|:---:|:---:|:---:|:---:|:---:|
| **字段** | timestamp_ms | odom_x4 | bms | remote_x7 |
| **说明** | 帧时刻 | 四轮里程s32 | 14B BMS | 7B遥控 |

---

## 2. GPS

### 2.1 gps_compact_t（MSG 0x05，32 B）

| | 0-3 | 4 | 5 | 6-7 | 8-9 | 10-13 | 14-17 | 18-19 | 20-21 | 22-25 | 26-31 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | timestamp_ms | flags | num_sv | hdop_x100 | speed_cms | lat_e7 | lon_e7 | heading_x100 | alt_dm | utc_sec | rsv |
| **类型** | u32 BE | u8 | u8 | u16 BE | u16 BE | s32 BE | s32 BE | s16 BE | s16 BE | u32 BE | u8x6 |
| **说明** | 帧时刻 | 定位标志 | 卫星数 | HDOPx100 | cm/s | 纬度e7 | 经度e7 | 航向x100 | 海拔dm | UTC秒 | 填0 |

#### flags（byte 4）

| Bit | 名称 | =1 |
|:--:|------|-----|
| 0 | POS_VALID | 位置有效 |
| 1 | VEL_VALID | 速度有效 |
| 2 | HEADING_VALID | 航向有效 |
| 3 | FIX_3D_LIKE | fix有效 |
| 4 | USEFULL | NMEA有效 |
| 5-7 | RSV | 0 |

| 字段 | 无效值 |
|------|--------|
| hdop_x100 | 0xFFFF |
| alt_dm | 0x7FFF |
| utc_sec | 无GPS填0 |

---

## 3. 传感器

### 3.1 sensor_blob_t（MSG 0x04，28 B）

| | 0-3 | 4-5 | 6-7 | 8-9 | 10-11 | 12-15 | 16-19 | 20-23 | 24-27 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | timestamp_ms | dist_f | dist_b | dist_l | dist_r | stamp_f | stamp_b | stamp_l | stamp_r |
| **类型** | u32 BE | u16 BE | u16 BE | u16 BE | u16 BE | u32 BE | u32 BE | u32 BE | u32 BE |
| **说明** | 帧时刻 | 前mm | 后mm | 左mm | 右mm | 前stamp | 后stamp | 左stamp | 右stamp |

dist 无效 **0xFFFF**；有效 0-60000 mm。stamp = 该路 stable_mm 最后变化 tick。

### 3.2 sensor_cfg_t（MSG 0x10，8 B）

| | 0-3 | 4-5 | 6 | 7 |
|:---:|:---:|:---:|:---:|:---:|
| **字段** | timestamp_ms | threshold_mm | enable_mask | rsv |
| **类型** | u32 BE | u16 BE | u8 | u8 |
| **说明** | 帧时刻 | 避障阈值mm | bit0-3四路使能 | 0 |

---

## 4. MCU

### 4.1 mcu_status_t（MSG 0x03，42 B）

| | 0-3 | 4 | 5 | 6 | 7 | 8-13 | 14-21 | 22-37 | 38-39 | 40 | 41 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | timestamp_ms | seq | safety | link | limit | arb_vwd | sonar_x4 | stamp_x4 | near | jetson_seq | rsv |
| **类型** | u32 BE | u8 | u8 | u8 | u8 | s16x3 BE | u16x4 BE | u32x4 BE | u16 BE | u8 | u8 |
| **说明** | 帧时刻 | 序号 | 安全态 | 链路 | 限速% | 仲裁速度 | 四向mm | 四向stamp | 最近mm | 控制seq | 0 |

safety：0x01正常 0x02限速 0x03降级 0x04紧急。sonar 同 0x04，Jetson 只订 0x04 即可。

---

## 附录 A：C 结构体

```c
#pragma pack(push, 1)

/* 9B BLOB 头（RS232/CAN 共用） */
typedef struct {
    uint8_t  magic;      /* 0xAB */
    uint8_t  ver;        /* 0x01 */
    uint8_t  msg_id;
    uint8_t  seq;
    uint16_t len_be;     /* PAYLOAD 长度，大端 */
    uint8_t  frag_idx;   /* 固定 0 */
    uint8_t  frag_cnt;   /* RS232=1；CAN=ceil((9+len)/8) */
    uint8_t  flags;      /* 0 */
} blob_hdr_t;

typedef struct { /* 0x01, 14B */
    uint32_t timestamp_ms;
    int16_t  linear_vel, angular_vel, steer_angle;
    uint8_t  control_mode, motion_drive_info, clear_fault, light_info;
} agv_control_t;

typedef struct { /* 10B */
    int16_t  speed_rpm, current;
    uint16_t voltage;
    int8_t   temperature;
    uint8_t  driver_status;
    uint16_t position_lo;
} motor_compact_t;

typedef struct { /* 0x02, 40B */
    uint32_t timestamp_ms;
    uint8_t  system_info, motion_info, light_pack;
    uint32_t fault_code;
    uint16_t battery_voltage;
    int16_t  linear_velocity, angular_velocity, steering_angle;
    int16_t  wheel_angle[4], wheel_speed[4];
    uint8_t  rsv[5];
} agv_motion_t;

typedef struct { /* 0x06/0x07, 44B */
    uint32_t timestamp_ms;
    motor_compact_t motor[4];
} agv_motor04_t, agv_motor58_t;

typedef struct { /* 0x08, 41B */
    uint32_t timestamp_ms;
    int32_t  odom[4];
    struct {
        uint8_t bms_soc, bms_soh;
        uint16_t bms_voltage;
        int16_t  bms_current, bms_temperature;
        uint8_t  bms_alarm1, bms_alarm2, bms_warning1, bms_warning2;
    } bms;
    uint8_t remote[7];
} agv_energy_t;

typedef struct { /* 0x05, 32B */
    uint32_t timestamp_ms;
    uint8_t  flags, num_sv;
    uint16_t hdop_x100, speed_cms;
    int32_t  lat_e7, lon_e7;
    int16_t  heading_x100, alt_dm;
    uint32_t utc_sec;
    uint8_t  rsv[6];
} gps_compact_t;

typedef struct { /* 0x04, 28B */
    uint32_t timestamp_ms;
    uint16_t dist_mm[4];
    uint32_t stamp_ms[4];
} sensor_blob_t;

typedef struct { /* 0x10, 8B */
    uint32_t timestamp_ms;
    uint16_t threshold_mm;
    uint8_t  enable_mask, rsv;
} sensor_cfg_t;

typedef struct { /* 0x03, 42B */
    uint32_t timestamp_ms;
    uint8_t  seq, safety, link_flags, limit_factor;
    int16_t  arb_v, arb_w, arb_steer;
    uint16_t sonar_mm[4];
    uint32_t sonar_stamp_ms[4];
    uint16_t nearest_mm;
    uint8_t  jetson_seq, rsv;
} mcu_status_t;

#pragma pack(pop)
```

---

## 附录 C：Jetson 侧工作与 RS232 联调清单

### C.1 Jetson 侧要干什么（总览）

| 序号 | 工作项 | 说明 |
|:--:|--------|------|
| 1 | **确认串口设备** | 找到接 F407 的 USB-TTL（常见 Prolific `/dev/ttyUSB*`），**不要**与 4G/GPS/CH340 调试口混淆 |
| 2 | **打开 115200 8N1 原始字节流** | 无 XOR、无行协议；`pyserial` 等一次 `write` 整帧 |
| 3 | **实现 BLOB 收包状态机** | 见 **§0.2**：`0xAB` → 收满 9B 头 → 读 LEN → 再收 LEN 字节 PAYLOAD |
| 4 | **混传解析** | 同一线路可能还有 **`0xA5` 11B 服务帧**；按首字节分支（**不要**把 0xAB 当 0xAA 24B V3） |
| 5 | **周期发送 MSG 0x01 控制** | **≤20 ms** 一帧；**`seq` 每帧递增**（心跳靠 seq 变化，**300 ms** 无新 seq → MCU `DEGRADED`） |
| 6 | **解析 MCU 上行 BLOB** | 至少订 **0x02/0x03/0x04**；可选 0x05 GPS、0x06/0x07 电机、0x08 能源 |
| 7 | **（可选）时间同步** | 仍用 **0xA5 + CAN ID 0x107**，与 BLOB 无关；见 [Jetson_RS232协议.md §8](./Jetson_RS232协议.md) |
| 8 | **与 MCU 固件对齐** | MCU 需 `JETSON_LINK_CAN=0`、`JETSON_USE_BLOB_V2=1`、**`ETH_LWIP_ENABLE=0`**（释放 PA2 给 USART2_TX） |

### C.2 Jetson 必须发送：MSG 0x01 控制帧

**线格式**（23 字节，一次写出）：

```text
AB 01 01 SEQ 00 0E 00 01 00  [14B agv_control_t PAYLOAD]
```

**PAYLOAD（14 B，大端）**

| 字节 | 字段 | 类型 | Jetson 填法 |
|:--:|------|------|-------------|
| 0-3 | timestamp_ms | u32 BE | `int(time.time()*1000) & 0xFFFFFFFF` 或单调 ms |
| 4-5 | linear_vel | s16 BE | 线速度 **mm/s**（联调可先 `100`） |
| 6-7 | angular_vel | s16 BE | 角速度 **0.001 rad/s** |
| 8-9 | steer_angle | s16 BE | 转角 **0.001 rad** |
| 10 | control_mode | u8 | **`0x01`** = CAN 控制运行；`0x00` = 待机 |
| 11 | motion_drive_info | u8 | 低 4 bit = 运动模式(0 阿克曼)；高 4 bit = 驱动模式 |
| 12 | clear_fault | u8 | 平时 **0**；清错时按 [Jetson_RS232协议.md §5](./Jetson_RS232协议.md) |
| 13 | light_info | u8 | bit0=灯开；bit1=灯模式 |

**心跳**：`SEQ` **每发一帧 +1**（0~255 回绕）；停止发送或 seq 不变超过 **300 ms**，MCU 进 **`DEGRADED`** 并可能仍按旧速度惯性输出（需实测）。

**Python 组帧示例**：

```python
import struct
import time

def build_control_blob(seq: int, v_mm_s: int = 100) -> bytes:
    ts = int(time.time() * 1000) & 0xFFFFFFFF
    payload = struct.pack(">IhhhBBBB",
        ts, v_mm_s, 0, 0,   # linear, angular, steer
        0x01,               # control_mode = CAN 控制
        0x00,               # motion_drive_info
        0x00,               # clear_fault
        0x00)               # light_info
    hdr = bytes([0xAB, 0x01, 0x01, seq & 0xFF, 0x00, 0x0E, 0x00, 0x01, 0x00])
    return hdr + payload

def blob_rx_feed(buf: bytearray, byte: int):
    """简化：返回完整 wire 或 None；生产环境用状态机。"""
    buf.append(byte)
    if len(buf) < 9:
        return None
    if buf[0] != 0xAB:
        buf.clear()
        return None
    ln = (buf[4] << 8) | buf[5]
    need = 9 + ln
    if len(buf) < need:
        return None
    frame = bytes(buf[:need])
    del buf[:need]
    return frame
```

### C.3 Jetson 会收到什么（MCU 当前固件）

MCU `vJetsonTask` 周期 **20 ms**，BLOB 上行大致为：

| MSG_ID | struct | 线长 | 频率（约） |
|:--:|--------|:--:|:--:|
| 0x02 | agv_motion_t | 49 B | 每 20 ms |
| 0x04 | sensor_blob_t | 37 B | 每 20 ms |
| 0x06/0x07 | agv_motor04/58_t | 53 B | 每 40 ms 交替 |
| 0x03 | mcu_status_t | 51 B | 每 60 ms |
| 0x08 | agv_energy_t | 50 B | 每 100 ms |
| 0x05 | gps_compact_t | 41 B | ~100 ms（GpsTask） |
| 0x0B | agv_motor_pos_t | 45 B | ~1 Hz |
| 0xA5 | 服务帧 | 11 B | 故障/GPS 分片/时间应答等 |

**联调第一阶段**只订：**0x02**（运动）、**0x03**（MCU 状态含 `safety`/`jetson_seq`）、**0x01 回环验证 seq**。

**mcu_status_t 关键字段**（PAYLOAD byte 5 = `safety`）：

| safety | 含义 |
|:--:|------|
| 0x01 | NORMAL |
| 0x02 | SPEED_LIMIT |
| 0x03 | DEGRADED |
| 0x04 | EMERGENCY |

byte 40 = `jetson_seq`：应等于 Jetson 最近下发的 **0x01 头里 SEQ**。

### C.4 Jetson 推荐最小程序结构

```text
1. 打开 serial(port, 115200, timeout=0.05)
2. 启动 RX 线程：读字节 → 0xAB 状态机 / 0xA5 服务帧分支
3. 主循环 20ms：
     seq = (seq + 1) & 0xFF
     ser.write(build_control_blob(seq, v=100))
4. RX 解析到 MSG 0x03：打印 safety、jetson_seq、limit_factor
5. （可选）每 10s 发 0xA5 时间同步 0x107
```

脚本建议放：`~/catkin_ws/cangyirobot/tools/jetson_blob_test.py`（与现有 `jetson_time_ping_test.py` 并列）。

### C.5 硬件与固件前置条件

| 检查项 | Jetson | F407 MCU |
|--------|--------|----------|
| 链路 | USB-TTL 3.3V | `JETSON_LINK_CAN=0` |
| 协议 | 发/收 **0xAB BLOB** | `JETSON_USE_BLOB_V2=1` |
| 以太网 | 无关 | **`ETH_LWIP_ENABLE=0`**（PA2 给 USART2_TX） |
| 波特率 | 115200 8N1 | 115200 8N1 |
| 接线 | **RX←PA2，TX→PA3，GND** | PA2=TX→Jetson RX；PA3=RX←Jetson TX |
| 仲裁 | — | 若 `ARBITER_IGNORE_DIST_SENSOR=1`，测距不参与避障 |

### C.6 联调清单（按顺序打勾）

> **重要（2026-06 联调教训，2026-06-16 修订）**
>
> 1. **`[JETSON CMD]` 仅 V3（0xAA）打印**；BLOB 看 **`[JETSON BLOB CMD]`** / **`[JETSON BLOB RX] first 0x01`**。
> 2. **`rs232_gateway` 会发 0x01 + 0xA5**：默认 `use_blob_v2=true` 时 **50Hz BLOB 0x01** + 时间同步 **0xA5**；日志里只见 `TimeSync PING` 是因为 **0x01 默认不打日志**，不代表没发。
> 3. **`ros2 launch agv_base_driver jetson_rs232_bringup.launch.py`** 会起 `rs232_gateway` + `agv_base_driver`；**写串口的是 gateway**，driver 只发 ROS topic。
> 4. BLOB 固件（`JETSON_USE_BLOB_V2=1`）在 IDLE **不再收 0xAA V3 下行**（防 payload 误同步）；对比 V3 需 **`use_blob_v2:=false`** 且 MCU 改回双解析或旧固件。
> 5. 若见 **`[BLOB RX] hdr reject`** → Jetson 线头与 MCU 不一致（常见：`FRAG_IDX≠0`、`FLAGS≠0`、`LEN≠14`）。
> 6. 联调前 **`pkill -f rs232_gateway; pkill -f agv_base_driver`**，避免多进程抢同一 `/dev/ttyUSB*`。

#### 阶段 0 — 环境与接线

- [ ] MCU 已烧录：`ETH_LWIP_ENABLE=0`、`JETSON_USE_BLOB_V2=1`、`JETSON_LINK_CAN=0`
- [ ] 串口日志**无** `[ETH] lwIP OK`（以太网已关）
- [ ] 确认 Jetson 串口设备：`ls -l /dev/serial/by-id/` 或 `dmesg | tail` 插拔识别
- [ ] 接线：F407 **PA2→Jetson RX**，**PA3←Jetson TX**，**GND 共地**
- [ ] 电平：**3.3V TTL**（勿接 RS232 ±12V 直连）
- [ ] Jetson 侧：`stty -F /dev/ttyUSBx 115200 cs8 -cstopb -parenb raw -echo`

#### 阶段 1 — 单向 RX（只收 MCU）

- [ ] Jetson **不发**数据，只读串口 5 s
- [ ] 能扫到 **`0xAB 0x01`** 且 byte2 为 **0x02/0x03/0x04** 等
- [ ] 若完全无数据：查 TX 线是否接反、是否开错 `/dev/ttyUSB*`
- [ ] 若只有乱码：查波特率 115200、GND

#### 阶段 2 — 下行 0x01（Jetson→MCU）

- [ ] Jetson 以 **20 ms** 周期发 **MSG 0x01**，`control_mode=0x01`，`seq` 递增
- [ ] MCU 串口（USART1 调试口）`ARB` 由 **DEGRADED** 变 **NORMAL**（或 `safety=0x01` 出现在 0x03）
- [ ] `mcu_status_t.jetson_seq` 与下发 `seq` 一致
- [ ] 停发 0x01 超过 **300 ms**，MCU 回 **DEGRADED**（`safety=0x03`）

#### 阶段 3 — 业务验证

- [ ] 改 `linear_vel`（如 0 / 100 / -50），MCU `[MOTION] CMD=` 跟随变化
- [ ] 收 **0x02** 解析 `linear_velocity` 与底盘反馈一致
- [ ] 收 **0x04** 四向测距与 MCU `[DS]` 日志一致（无效=0xFFFF）
- [ ] （可选）**0xA5/0x107** 时间同步仍通（见时间同步联调记录）

#### 阶段 4 — 压力与排错

- [ ] 连续运行 10 min 无 seq 卡死、无 DEGRADED 误触发
- [ ] Jetson RX 线程不阻塞 20 ms 发送
- [ ] 记录：首包延迟、0x01→0x03 往返 ms（应用层 RTT）

### C.7 常见现象对照

| 现象 | 可能原因 |
|------|----------|
| Jetson 完全收不到字节 | TX/RX 反接、错 port、MCU 未跑 BLOB 分支 |
| 收到 0xAA 24B 而非 0xAB | Jetson `use_blob_v2:=false` 或旧 install；MCU 已 BLOB 模式 |
| 一直 DEGRADED | 未发 0x01、seq 不变、线格式错、或 **MCU 收包失步**（见 **§C.9**） |
| MCU 有 `[JETSON CMD]` 但不动 | `control_mode=0x00` 待机；需 **0x01** |
| **BLOB 模式无 `[JETSON CMD]`** | **正常**：V3 才打 `[JETSON CMD]`；BLOB 应看 **`[JETSON BLOB CMD]`** |
| **0xA5 TimeSync 通但 DEGRADED** | **0xA5 ≠ 0x01 心跳**；TimeSync 通只说明服务层双向 OK |
| `[JETSON RX] bytes=0` | PA3 未收到任何字节（下行物理断/未发/错 port） |
| `[JETSON RX] bytes>0 ab=0` | 有字节但未在 IDLE 同步 **0xAB**（V3 误同步或失步，见 **§C.9.4**） |
| `[JETSON RX] hdr_rej>0` | 线头字段与 MCU 校验不一致 |
| Jetson 发 BLOB 仍 DEGRADED | 下行未到 PA3，或 **`ETH_LWIP_ENABLE=1` 占 PA2** |
| 混线解析错帧 | 0xAB / 0xA5 / 0xAA 须独立状态机；BLOB 模式勿在 IDLE 收 0xAA |

### C.9 RS232 BLOB 联调实录：问题与解决方案

本节记录 **2026-06 F407 ↔ Jetson（`rs232_gateway` / `link_test`）** 联调 BLOB v2 时遇到的典型问题、日志特征与已落地修复。以太网 ping 问题见 [以太网与WiFi接入方案.md](./以太网与WiFi接入方案.md) §13（RS232 阶段与以太网 **独立**）。

#### C.9.1 三层诊断模型（先分层再定责）

联调时不要把「串口通」「TimeSync 通」「仲裁 NORMAL」混为一谈：

| 层级 | 判据 | 通过标志 | 失败时含义 |
|:--:|------|----------|------------|
| **L1 物理 RX** | MCU `[JETSON RX] bytes` 递增 | `bytes>0` | Jetson→PA3 无字节 |
| **L2 服务 0xA5** | gateway `TimeSync PING rtt=…ms` | RTT 稳定 | 仅说明 0xA5 双向 OK |
| **L3 BLOB 0x01 心跳** | `[JETSON BLOB RX/CMD]`、`ARB=NORMAL` | `safety_state→1` | **控制心跳未进仲裁** |

**经验**：L2 通、L3 不通 最常见——**不是线没接好**，而是 **0x01 未 parse 或未当心跳**。

#### C.9.2 问题 1：上行通、下行 `bytes=0`

| 项目 | 内容 |
|------|------|
| **现象** | Jetson 能收 0x02/0x03；MCU `[JETSON RX] bytes=0 ab=0`；无 `[JETSON BLOB RX]` |
| **根因** | USART2 **PA3 未收到** Jetson TX 字节（接线、错 port、或测试窗口内未发下行） |
| **排查** | 确认 **Jetson TX→PA3**，**Jetson RX←PA2**，GND；`lsof /dev/ttyUSB*` 独占 |
| **MCU 侧** | 启动行应有 `[JETSON] USART2 PA2/PA3, 115200, BLOB v2 …`；无 `[ETH]` |
| **Jetson 侧** | `link_test --blob-v2 --time 10` 或 `ros2 launch … jetson_rs232_bringup` |

#### C.9.3 问题 2：`bytes>0` 但 `ab=0`、`blob_ctrl=0`、一直 DEGRADED

| 项目 | 内容 |
|------|------|
| **现象** | `[JETSON RX] bytes≈6000/5s`（≈1200 B/s）；`ab=0`、`hdr_rej=0`；0xA5 TimeSync 正常；`safety_state=3` |
| **根因（主）** | **MCU 收包状态机在 BLOB 模式下被 V3（0xAA）误同步**：失步后 IDLE 遇到 payload 里的 `0xAA` 进入 V3，吞 24 字节，真正的 `0xAB` 帧头被错过 → `ab_idle` 永不增加 |
| **次要** | Jetson 实际发 **V3 0xAA**（`use_blob_v2:=false` 或旧脚本无 `--blob-v2`）；带宽 24×50≈1200 B/s 与 BLOB 23×50≈1150 B/s 接近，不能单靠带宽区分 |
| **日志误导** | gateway 日志 **只有 TimeSync** 不代表没发 0x01；**0x01 无 RTT 日志** |
| **解决方案（MCU 固件，已合入）** | 见 **§C.9.6** |

#### C.9.4 问题 3：误把 TimeSync 当心跳

| 项目 | 内容 |
|------|------|
| **现象** | `TimeSync PING rtt=20~40ms` 正常，但 `link_state=1`、`ARB=DEGRADED` |
| **根因** | **0x107/0x108 时间同步不走 `Arbiter_UpdateHeartbeat()`**；仲裁只认 **0x01 控制帧 seq 变化**（BLOB 或 V3） |
| **解决** | 确认 **50Hz 0x01** 在发：`ros2 param get /rs232_gateway use_blob_v2` → `True`；或单独 `link_test --blob-v2` |

#### C.9.5 问题 4：PA2 与以太网 MDIO 冲突

| 项目 | 内容 |
|------|------|
| **现象** | 上行偶通、下行异常；或日志有 `[ETH] lwIP OK` |
| **根因** | **PA2** 复用 **USART2_TX** 与 **ETH_MDIO**；`ETH_LWIP_ENABLE=1` 时以太网 init 在 `USART3_Init()` 之后占 PA2 |
| **解决** | RS232 BLOB 联调期 **`ETH_LWIP_ENABLE=0`**（`APP/freertos/app_boot.h`），重烧后再测 |

#### C.9.6 问题 5：USART2 ORE 静默丢 RX 字节

| 项目 | 内容 |
|------|------|
| **现象** | 高负载时下行 0x01 丢失；`ore` 计数增加（新固件） |
| **根因** | 旧 `USART2_IRQHandler` 在 **ORE** 时读 DR 后直接 `return`，**不调用** `USART3_ProcessRxByte`；MCU 上行 BLOB 刷屏时 115200 易过载 |
| **解决** | ORE 时仍 `ProcessRxByte`；心跳丢失时 **降低上行帧率**（`BlobPack_UplinkTick`）；见 `APP/usart3/usart3.c` |

#### C.9.7 问题 6：串口多进程混跑

| 项目 | 内容 |
|------|------|
| **现象** | 数据时有时无；parse failed；统计异常 |
| **根因** | `rs232_gateway`、`link_test`、`jetson_bridge` 同时打开 `/dev/ttyUSB*` |
| **解决** | 测试前：`pkill -f rs232_gateway; pkill -f agv_base_driver; pkill -f jetson_bridge; sleep 0.5` |

#### C.9.8 问题 7：Jetson 测试脚本协议不一致

| 项目 | 内容 |
|------|------|
| **现象** | `link_test` 无 `--blob-v2` 时 MCU `ab=0` |
| **根因** | 旧版 `tools/jetson_rs232_link_test.py` 默认发 **V3 0xAA 24B**；MCU BLOB 固件不认 V3 心跳（新固件 IDLE 已忽略 0xAA） |
| **解决** | BLOB 联调必须 **`--blob-v2`**；或 ROS **`use_blob_v2:=true`**（gateway 默认已是 true） |

#### C.9.9 MCU 固件修复清单（2026-06-16）

| 文件 | 改动 | 针对问题 |
|------|------|----------|
| `APP/usart3/usart3.c` | ORE 仍喂解析；IDLE **0xAB→0xA5**，BLOB 模式 **忽略 0xAA**；magic 统计 | §C.9.5、§C.9.3 |
| `APP/agv_blob/agv_blob_rs232.c` | 收包超长 `BLOB_MAX_WIRE` 强制 reset；`hdr reject` 日志 | 失步恢复 |
| `APP/agv_blob/agv_blob_pack.c` | 心跳丢失时减上行；`[JETSON BLOB RX/CMD]` 日志 | 带宽、可观测性 |
| `APP/freertos/rtos_tasks.c` | BLOB 模式关闭 V3 下行解析；`[JETSON RX]` 周期统计 | §C.9.3 |
| `APP/freertos/app_boot.h` | `ETH_LWIP_ENABLE=0` | §C.9.5 |
| `tools/jetson_rs232_link_test.py` | 增加 **`--blob-v2`** | §C.9.8 |

#### C.9.10 F407 调试口日志对照

| 日志 | 含义 |
|------|------|
| `[JETSON] … BLOB v2 (0xAB) down/up + 0xA5 svc` | BLOB 固件已生效 |
| `[JETSON BLOB RX] first 0x01 seq=…` | 首帧 0x01 进 BLOB 层 |
| `[JETSON BLOB CMD] seq=… mode=1` | 进仲裁，`control_mode=0x01` |
| `[Arbiter] DEGRADED -> NORMAL` | 心跳恢复 |
| `[BLOB RX] hdr reject …` | 线头校验失败（贴整行给 Jetson 对 hex） |
| `[JETSON CMD] parse failed` | V3 误同步或 Jetson 发 0xAA（BLOB 模式应减少） |
| `[JETSON RX] +5s: bytes=… ab_idle=… \| magic AB/A5/AA=…` | 5 s 窗口 RX 诊断 |

**`[JETSON RX]` 字段解读**：

| 字段 | 正常（BLOB launch） | 异常 |
|------|---------------------|------|
| `bytes` | 数百~数千 / 5s | `0` → L1 断 |
| `ab_idle` | 与 0x01 帧率同量级 | `0` 且 `magic AB>0` → 失步 |
| `ctrl` | 递增 | `0` → 未进 `BlobPack_HandleDownlink` |
| `magic AB/A5/AA` | AB 高、A5 低、AA≈0 | AA 高 → Jetson 发 V3 或误同步 |

#### C.9.11 Jetson 侧命令速查

```bash
# 环境
source /opt/ros/humble/setup.bash
source ~/catkin_ws/cangyirobot/install/setup.bash
export SERIAL=/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0

# 清串口
pkill -f rs232_gateway; pkill -f agv_base_driver; pkill -f jetson_bridge; sleep 0.5

# 专测 BLOB 心跳（推荐先跑）
python3 tools/jetson_rs232_link_test.py --port "$SERIAL" --blob-v2 --time 10

# 完整 ROS
ros2 launch agv_base_driver jetson_rs232_bringup.launch.py serial_port:="$SERIAL"
ros2 param get /rs232_gateway use_blob_v2    # 期望 True
ros2 topic echo /jetson_rs232/v3_status --field safety_state   # 期望 3→1
```

**0x01 下发帧参考**（23 B）：

```text
ab 01 01 SEQ 00 0e 00 01 00  + 14B agv_control_t
示例：ab010101000e00010001954e5400000000000001000000
```

#### C.9.12 联调通过标准

- [ ] F407：`[JETSON BLOB RX] first 0x01` → `[JETSON BLOB CMD]` → `ARB=NORMAL`
- [ ] Jetson：`safety_state` **3（DEGRADED）→ 1（NORMAL）**；`link_state` **1→0**
- [ ] 停发 0x01 **>300 ms** 后回 DEGRADED（心跳超时验证）
- [ ] 0xA5 TimeSync 仍可 PING（与心跳独立）

### C.8 相关文档

| 文档 | 用途 |
|------|------|
| 本文 **§0~§4** | BLOB 线格式与 struct |
| [Jetson_RS232协议.md](./Jetson_RS232协议.md) | 0xA5 服务帧、旧 V3 对照、Python 示例 |
| [Jetson时间同步联调记录.md](./Jetson时间同步联调记录.md) | 串口设备名、probe、时间同步 |
| [硬件连接与通信协议.md](./硬件连接与通信协议.md) | PA2/PA3 接线总图 |
| [以太网与WiFi接入方案.md](./以太网与WiFi接入方案.md) | 以太网暂停说明、PA2 冲突 |

---

## 附录 B：变更记录

| 版本 | 日期 | 内容 |
|------|------|------|
| v2.0-draft.5.7 | 2026-06-16 | 新增 **§C.9** RS232 BLOB 联调实录（问题/解决方案）；修正 gateway 发 0x01 说明 |
| v2.0-draft.5.6 | 2026-06-16 | 新增 **附录 C**：Jetson 侧工作与 RS232 联调清单 |
| v2.0-draft.5.5 | 2026-06-15 | 新增 §0.2 RS232 / §0.3 CAN 双传输映射与 CAN ID 表 |
| v2.0-draft.5.4 | 2026-06-15 | 修复横排表分隔行列数不一致导致无法渲染 |
