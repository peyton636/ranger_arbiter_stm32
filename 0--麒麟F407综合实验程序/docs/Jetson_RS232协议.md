# Jetson 与 STM32 B RS232（USART2）通信协议

| 元数据 | 值 |
|--------|-----|
| **协议版本** | v1.0 |
| **文档日期** | 2026-05-28 |
| **状态** | 已实现（`JETSON_LINK_CAN=0` 时启用） |

本文档描述 **Jetson / 上位机 SoC** 经 **RS232/TTL 串口** 与 STM32 B（麒麟 F407）通信的协议。  
硬件接线与 CAN/串口切换见 [硬件连接与通信协议.md](./硬件连接与通信协议.md) **§2.2**。

> **与 CAN 的关系**：应用层均为 **V3 24 字节帧**（type 0x01/0x02/0x03），字段语义与 [Jetson_CAN协议.md](./Jetson_CAN协议.md) §4~6 一致；**仅物理层不同**（整帧串口收发，无 CAN ID、无 GPS/0x107 等扩展 CAN 帧）。

---

## 1. 物理层

| 项目 | 约定 |
|------|------|
| MCU 外设 | **USART2**（代码函数名 `USART3_*`） |
| 引脚 | **PA2 = TX**，**PA3 = RX**（F407 侧） |
| 电平 | **3.3V TTL**；接 Jetson 需 USB-串口或电平匹配 |
| 波特率 | **115200** 8N1 |
| 帧边界 | **固定 24 字节**；无额外帧头长度字段 |
| 编译开关 | `jetson_can.h` 中 `#define JETSON_LINK_CAN 0` |

---

## 2. 启用与源码

| 步骤 | 说明 |
|------|------|
| 1 | `APP/jetson_can/jetson_can.h`：`JETSON_LINK_CAN` 改为 **0** |
| 2 | 重新编译烧录 |
| 3 | 硬件接 **PA2?Jetson RX，PA3?Jetson TX**，共 GND |

| 模块 | 作用 |
|------|------|
| `usart3.c` | V3 编解码；`USART3_DeliverV3Frame()` 走 `USART3_SendData()` |
| `rtos_tasks.c` | `JetsonTask`：`USART3_GetJetsonFrame()` + `USART2_IRQHandler` |
| `app_boot.c` | `USART3_Init()`，不初始化 CAN2 Jetson |

**RS232 模式不提供**：GPS CAN（0x104~0x106）、时间同步（0x107/0x108）、故障上报（0x109）、状态查询（0x10A/0x10B）。GPS 仅在 F407 本地使用（USART6）。

---

## 3. 通信周期

| 方向 | 帧 type | 周期 |
|------|---------|------|
| Jetson → STM32 | 0x01 下行 | 建议 **≥20 ms**；`seq` 递增保活 |
| STM32 → Jetson | 0x02 状态 | **~20 ms**（与 0x03 交替） |
| STM32 → Jetson | 0x03 扩展 | **~40 ms**（与 0x02 交替） |
| 心跳超时 | — | **300 ms** 无新 seq → `DEGRADED` |

---

## 4. 通用约定

| 项目 | 约定 |
|------|------|
| 帧长 | **24 字节** |
| 帧头 | byte0 = **0xAA** |
| 校验 | byte23 = XOR(byte0..22) |

```text
chk = 0
for i in 0..22: chk ^= frame[i]
frame[23] = chk
```

| type (byte1) | 方向 | 名称 |
|:--:|------|------|
| **0x01** | Jetson → STM32 | 下行控制 |
| **0x02** | STM32 → Jetson | 上行状态 |
| **0x03** | STM32 → Jetson | 上行扩展 |

---

## 5. 下行控制帧 type=0x01

| 字节 | 字段 | 类型 | 说明 |
|:--:|------|------|------|
| 0 | HEADER | u8 | 固定 **0xAA** |
| 1 | FRAME_TYPE | u8 | 固定 **0x01** |
| 2 | SEQ | u8 | 0~255；**变化刷新心跳** |
| 3 | MODE_REQ | u8 | 见表 5.1 |
| 4~5 | V | s16 BE | 线速度 mm/s |
| 6~7 | OMEGA | s16 BE | 角速度 0.001 rad/s |
| 8~9 | STEER | s16 BE | 转角 0.001 rad |
| 10 | MOTION_MODEL | u8 | 见表 5.2 |
| 11 | LIGHT_EN | u8 | 0/1 |
| 12 | LIGHT_MODE | u8 | 0/1 |
| 13 | CLEAR_ERROR | u8 | 见附录 A |
| 14~22 | RSV | u8 | 0 |
| 23 | XOR_CHK | u8 | XOR |

### 5.1 MODE_REQ

| 值 | 含义 | STM32 动作 |
|:--:|------|------------|
| 0x00 | 待机 | CAN 0x421 = 0x00 |
| 0x01 | CAN 指令控制 | CAN 0x421 = 0x01 |
| 0x02 | 遥控请求 | 按待机处理 |

### 5.2 MOTION_MODEL

| 值 | 含义 | CAN 0x141 |
|:--:|------|-----------|
| 0x00 | 前后阿克曼 | 0x00 |
| 0x01 | 斜移 | 0x01 |
| 0x02 | 自旋 | 0x02 |
| 0x03 | 驻车 | 0x03 |

> byte4~9 经 **避障仲裁** 后写入 CAN 0x111。

---

## 6. 上行状态帧 type=0x02

| 字节 | 字段 | 说明 |
|:--:|------|------|
| 0 | HEADER | 0xAA |
| 1 | FRAME_TYPE | 0x02 |
| 2 | SEQ | TX 序号 |
| 3 | SAFETY_STATE | 见表 6.1 |
| 4 | LINK_STATE | 见表 6.2 |
| 5 | LIMIT_FACTOR | 0~100 |
| 6~7 | FB_V | s16 BE，mm/s |
| 8~9 | FB_OMEGA | s16 BE，0.001 rad/s |
| 10~11 | FB_STEER | s16 BE，0.001 rad |
| 12~13 | SONAR_F | u16 BE，mm |
| 14~15 | SONAR_B | u16 BE |
| 16~17 | SONAR_L | u16 BE |
| 18~19 | SONAR_R | u16 BE |
| 20~21 | BAT_V | u16 BE，0.1 V |
| 22 | SOC | u8，% |
| 23 | XOR_CHK | XOR |

**SONAR_***：单位 **mm**；无效值 **0xFFFF**；有效范围 **0~60000 mm**。

### 6.1 SAFETY_STATE

| 值 | 模式 |
|:--:|------|
| 0x01 | NORMAL |
| 0x02 | SPEED_LIMIT |
| 0x03 | DEGRADED / RECOVERING |
| 0x04 | EMERGENCY |

### 6.2 LINK_STATE

| Bit | 名称 | =1 |
|:---:|------|-----|
| 0 | JETSON_HB_LOST | 心跳 >300 ms |
| 1 | CHASSIS_FAULT | 底盘 system_status=0x02 |
| 2 | BEEP_ACTIVE | 测距报警蜂鸣响 |
| 3~7 | RSV | 0 |

### 6.3 LIMIT_FACTOR

非 SPEED_LIMIT 时为 100；SPEED_LIMIT 时按最近距 60~180 mm 线性映射 0~100。

---

## 7. 上行扩展帧 type=0x03

| 字节 | 字段 | 说明 |
|:--:|------|------|
| 0~2 | HEADER/TYPE/SEQ | 同前 |
| 3~10 | WHEEL_RF/RR/LR/LF | 四轮速 s16 BE，mm/s |
| 11~18 | STEER_RF/RR/LR/LF | 四轮转角 s16 BE |
| 19 | MOTOR_TEMP_MAX | s8，℃ |
| 20 | DRIVER_STATE_OR | u8 |
| 21~22 | RSV | 0 |
| 23 | XOR_CHK | XOR |

轮速/转角来源见 [Jetson_CAN协议.md](./Jetson_CAN协议.md) §6 轮速映射表。

---

## 8. V3 → 底盘 CAN 映射

| V3 字段 | 底盘 CAN |
|---------|----------|
| mode_req | 0x421 |
| motion_model | 0x141 |
| light_en / light_mode | 0x121 |
| clear_error | 0x441 |
| v / steer / omega | 0x111（经仲裁） |

---

## 9. Jetson 侧对接建议

1. 下行 **≥20 ms** 发 0x01，**seq 递增**。
2. 上行需同时解析 **0x02 + 0x03** 才得完整状态。
3. 速度 mm/s；角度 ×1000（0.001 rad）。
4. 测距 u16，**0xFFFF** = 无效。
5. 需要 GPS / 时间同步 / 故障 CAN 时，请改用 **CAN 模式**（见 [Jetson_CAN协议.md](./Jetson_CAN协议.md)）。

---

## 附录 A：CLEAR_ERROR 码（0x441）

| 码 | 含义 |
|:--:|------|
| 0x00 | 清除全部非严重故障 |
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
| 串口 RX | `USART2_IRQHandler` → `USART3_ProcessRxByte()` |

---

## 相关文档

- [硬件连接与通信协议.md](./硬件连接与通信协议.md)
- [Jetson_CAN协议.md](./Jetson_CAN协议.md)
- [rtos移植.md](./rtos移植.md)
