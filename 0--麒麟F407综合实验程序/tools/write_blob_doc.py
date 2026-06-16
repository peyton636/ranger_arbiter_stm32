# -*- coding: utf-8 -*-
"""Regenerate Jetson_BLOB协议_v2.md as UTF-8 (no BOM)."""
from pathlib import Path

DOC = r"""# Jetson 与 MCU BLOB 二进制协议（v2.0 草案）

| 元数据 | 值 |
|--------|-----|
| **协议版本** | **v2.0-draft.5.1** |
| **文档日期** | 2026-06-15 |
| **物理层** | RS232 115200 8N1（USART2）；CAN 模式复用相同 PAYLOAD |
| **编码** | 多字节整数 **大端 BE**；`#pragma pack(1)` |

---

## 0. 传输封装（所有业务帧共用）

```text
[0xAB][VER=0x01][MSG_ID][SEQ][LEN_H][LEN_L][FRAG_IDX=0][FRAG_CNT=1][FLAGS=0][PAYLOAD...]
```

| 字节 | 字段 | 取值 |
|:--:|------|------|
| 0 | MAGIC | 固定 **0xAB** |
| 1 | VER | 固定 **0x01** |
| 2 | MSG_ID | 见下表 |
| 3 | SEQ | 0~255 递增 |
| 4~5 | LEN | **= sizeof(struct)**，u16 BE |
| 6~8 | FRAG/CRC | 默认不分片、无 CRC |
| 9~ | PAYLOAD | packed struct 原样字节 |

**与时间同步混传**：`0xA5` 服务帧（0x107/0x108）不变，见 [Jetson_RS232协议.md](./Jetson_RS232协议.md)。

### 帧级 timestamp_ms（强制）

每一个 wire struct **byte0~3 = timestamp_ms（u32 BE）**，表示发送方组包完成、即将发出的时刻。

| 方向 | timestamp_ms |
|------|--------------|
| Jetson→MCU（0x01、0x10） | Jetson 单调毫秒 |
| MCU→Jetson（其余 MSG） | MCU tick ms |

配合 PING/offset：`t_jetson = mcu_timestamp_ms - offset_ms`。

| 名称 | 含义 |
|------|------|
| **帧 stamp** | struct 首 4B，整帧快照时刻（**必须有**） |
| **路 stamp** | 如 sensor 的 stamp_f~r，某路数据最后变化时刻 |

### MSG_ID 总览

| MSG_ID | 类别 | 方向 | struct | 长度 |
|:--:|------|:----:|--------|:--:|
| **0x01** | ① 底盘 | Jetson→MCU | `agv_control_t` | 14 B |
| **0x02** | ① 底盘 | MCU→Jetson | `agv_motion_t` | 40 B |
| **0x06** | ① 底盘 | MCU→Jetson | `agv_motor04_t` | 44 B |
| **0x07** | ① 底盘 | MCU→Jetson | `agv_motor58_t` | 44 B |
| **0x08** | ① 底盘 | MCU→Jetson | `agv_energy_t` | 41 B |
| **0x0B** | ① 底盘 | MCU→Jetson | `agv_motor_pos_t` | 36 B（可选 1 Hz） |
| **0x05** | ② GPS | MCU→Jetson | `gps_compact_t` | 32 B |
| **0x04** | ③ 传感器 | MCU→Jetson | `sensor_blob_t` | 28 B |
| **0x10** | ③ 传感器 | Jetson→MCU | `sensor_cfg_t` | 8 B |
| **0x03** | ④ MCU | MCU→Jetson | `mcu_status_t` | 42 B |

---

## ① 底盘

MCU 聚合底盘 CAN → **4 条上行帧（0x02/06/07/08，各 ~40B）**；Jetson 下发 **0x01** → 仲裁后转 CAN。

### 1.1 控制 `agv_control_t`（MSG 0x01，14 B）

| | 0~3 | 4~5 | 6~7 | 8~9 | 10 | 11 | 12 | 13 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | timestamp_ms | linear_vel | angular_vel | steer_angle | control_mode | motion_drive_info | clear_fault | light_info |

- **linear_vel / angular_vel / steer_angle**：s16 BE，mm/s、0.001 rad/s、0.001 rad
- **control_mode**：0x00 待机，0x01 CAN 指令
- **motion_drive_info**：bit[1:0] 运动模式，bit2 电流/电压驱动
- **clear_fault**：非 0 触发底盘 0x441 清错
- **light_info**：bit0 使能，bit1 灯模式

### 1.2 为何不用 201 B 单帧？

| 块 | 字节 | 说明 |
|----|:--:|------|
| motor[8]×16B | 128 | 占 64%，主因 |
| odom+remote+bms | 37 | |
| 运动+系统 | 33 | |

**对策**：拆 4 帧；电机改 10B 紧凑体；`light_info+light_count` 合并为 `light_pack`（1B）。

### 1.3 运动 `agv_motion_t`（MSG 0x02，40 B）

| | 0~3 | 4 | 5 | 6 | 7~10 | 11~12 | 13~18 | 19~26 | 27~34 | 35~39 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | ts | system_info | motion_info | light_pack | fault_code | bat_v | vel×3 | wheel_angle[4] | wheel_speed[4] | rsv |

- **system_info**：bit0 异常，bit[2:1] 待机/CAN/遥控
- **motion_info**：bit[1:0] 阿克曼/斜移/自旋/驻车，bit2 切换中，bit3 驱动模式
- **light_pack**：bit0~1 灯，bit[7:2] 计数 0~63
- **fault_code**：u32 BE，0x211 原样
- **bat_v**：u16，×0.1 V

### 1.4 电机（MSG 0x06 / 0x07，各 44 B）

| MSG | 电机 | CAN |
|:---:|:--:|-----|
| 0x06 | 0~3 驱动 | 0x251~254 |
| 0x07 | 4~7 转向 | 0x255~258 |

**motor_compact_t（10 B/台）**：speed s16，current s16，voltage u16，temp i8，driver_status u8 FLAG，position_lo u16。

**driver_status** 同底盘 0x261 byte5：VLOW/MOT_OT/DRV_OC/DRV_OT/SENSOR/ERR/EN。

### 1.5 能源 `agv_energy_t`（MSG 0x08，41 B）

ts(4) + odom[4] s32(16) + bms(14) + remote[7]。周期建议 50 ms。

### 1.6 可选全脉冲 `agv_motor_pos_t`（MSG 0x0B，36 B，1 Hz）

ts + position_pulse[8] u32。

### 1.7 发送节奏

每 20 ms：0x02 → 0x06 → 0x07；每 50 ms：0x08；每 100 ms：0x05 GPS。

---

## ② GPS

### 2.1 `gps_compact_t`（MSG 0x05，32 B）

| | 0~3 | 4 | 5 | 6~7 | 8~9 | 10~13 | 14~17 | 18~19 | 20~21 | 22~25 | 26~31 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | ts | flags | num_sv | hdop | speed | lat | lon | heading | alt | utc | rsv |

**flags**：bit0 POS，bit1 VEL，bit2 HEADING，bit3 FIX，bit4 USEFULL。

- hdop×100，无效 0xFFFF；alt_dm 无效 0x7FFF；utc 无 GPS 填 0。

---

## ③ 传感器

### 3.1 `sensor_blob_t`（MSG 0x04，28 B）

| | 0~3 | 4~11 | 12~27 |
|:---:|:---:|:---:|
| **字段** | ts | dist_f/b/l/r u16 | stamp_f/b/l/r u32 |

| 取值 | 含义 |
|:--:|------|
| 0~60000 | 有效距离 mm |
| **0xFFFF** | 无效/超时/未知 |

### 3.2 `sensor_cfg_t`（MSG 0x10，8 B）

ts + threshold_mm u16 + enable_mask u8（bit0~3 前/后/左/右使能）。

---

## ④ MCU

### 4.1 `mcu_status_t`（MSG 0x03，42 B）

| | 0~3 | 4 | 5 | 6 | 7 | 8~13 | 14~21 | 22~37 | 38~39 | 40 | 41 |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| **字段** | ts | seq | safety | link | limit | arb_v/w/steer | sonar[4] | stamp[4] | near | jetson_seq | rsv |

- **safety**：0x01 正常，0x02 限速，0x03 降级，0x04 紧急
- **link_flags**：bit0 心跳丢失，bit1 底盘故障，bit2 蜂鸣，bit3 CAN 有效，bit4 UART 有效

#### sonar[4]（byte14~21）

与 **§3.1 dist_*** 相同：前/后/左/右 **mm**；无效 **0xFFFF**。

> **为何 0x03 里还有 sonar？** 与 0x04 内容相同，是 **MCU 仲裁线程本地快照**（`DistSnapshot`），限速/紧急停车在 20 ms 环里读，不必等 Jetson 侧或组 0x04 帧。Jetson **订阅避障数据只订 0x04 即可**；0x03 里的 sonar 可视为调试/冗余字段，后续版本可删以把 0x03 缩到 22 B。

#### stamp[4]（byte22~37）

与 **§3.1 stamp_*** 相同：每路 u32 BE，stable_mm 最后变化时刻。

#### nearest_mm（byte38~39）

四向最小有效距离 mm；全无效 **0xFFFF**。

---

## 附录 A：C 结构体

```c
#pragma pack(push, 1)

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

typedef struct { /* 0x0B, 36B */
    uint32_t timestamp_ms;
    uint32_t position_pulse[8];
} agv_motor_pos_t;

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

## 附录 B：发送周期

| MSG | 周期 |
|:---:|:--:|
| 0x01 | 20 ms |
| 0x02/0x06/0x07 | 20 ms |
| 0x08 | 50 ms |
| 0x03/0x04 | 20 ms |
| 0x05 | 100 ms |

---

## 附录 C：变更记录

| 版本 | 日期 | 内容 |
|------|------|------|
| v2.0-draft.5 | 2026-06-15 | 201B 拆为 0x02/06/07/08；电机 10B |
| v2.0-draft.5.1 | 2026-06-15 | 修复 GBK/UTF-8 乱码；澄清 0x03 sonar 冗余 |
"""

def main():
    root = Path(__file__).resolve().parents[1]
    for name in ("Jetson_BLOB协议_v2.md", "Jetson_BLOB_protocol_v2.md"):
        p = root / "docs" / name
        p.write_text(DOC, encoding="utf-8", newline="\n")
        print("wrote", p, "lines", DOC.count("\n") + 1)

if __name__ == "__main__":
    main()
