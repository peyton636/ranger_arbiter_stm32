# Jetson 与 MCU 时间同步联调记录


| 元数据      | 值                                                                                       |
| -------- | --------------------------------------------------------------------------------------- |
| **文档版本** | v1.3.1                                                                                  |
| **最后更新** | **2026-06-15**                                                                          |
| **链路**   | RS232 / USART2（`JETSON_LINK_CAN=0`）                                                     |
| **协议**   | [Jetson_CAN协议.md §8.1](./Jetson_CAN协议.md) / [Jetson_RS232协议.md §10](./Jetson_RS232协议.md) |
| **联调结论** | **单调时钟对时已通过**（2026-06-15）；**四路测距 stamp 待实现**（见 §5）；**协议与代码流程见 §3** |


---

## 1. 硬件与串口分工（必读）


| MCU 口      | 引脚       | 连接                       | 用途                               |
| ---------- | -------- | ------------------------ | -------------------------------- |
| **USART1** | PA9/PA10 | Windows 写程序电脑            | `[MOTION]`、`[BOOT]` 等 **调试 log** |
| **USART2** | PA2/PA3  | Jetson USB-TTL（Prolific） | **V3 控制 + 0xA5 服务帧（时间同步）**       |


> 在 Windows 上能看到 `[MOTION]` **不代表** Jetson 串口正常；时间同步测试必须接 **USART2 那路**。

Jetson 上当前设备（2026-06-15 实测）：


| 设备              | 端口                                      | 用途            |
| --------------- | --------------------------------------- | ------------- |
| STM32B Prolific | `/dev/ttyUSB7` 或 `by-id/...Prolific...` | **时间同步 / V3** |
| CH340           | `/dev/ttyUSB0`                          | 非 Jetson 链路   |
| Quectel 4G      | `/dev/ttyUSB1~5`                        | 非 Jetson 链路   |


建议始终使用 **by-id** 路径，避免插拔后编号变化：

```bash
/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
```

接线（TX/RX 交叉，共地）：

```text
Jetson USB-TTL TX  ->  STM32 PA3 (USART2_RX)
Jetson USB-TTL RX  <-  STM32 PA2 (USART2_TX)
GND                --  GND
115200 8N1
```

---

## 2. 联调过程中出现的问题与解决方案

### 2.1 问题总表


| #   | 现象                                   | 根因                                   | 解决方案                                 | 验证方法                                      |
| --- | ------------------------------------ | ------------------------------------ | ------------------------------------ | ----------------------------------------- |
| 1   | `--probe` 报 `could not open port ''` | 未传 `--port` 或 `$PORT` 为空             | 直接写路径：`--port /dev/ttyUSB7`          | 脚本报错提示非空 port                             |
| 2   | QUERY/START/PING 全超时；probe **0 字节**  | 接错口（USART1 调试口）或线未接 PA2/PA3          | 换 Prolific 口；先 `--probe` 看 `0xAA>0`  | `jetson_rs232_link_test.py --listen-only` |
| 3   | `--probe` 正常（约 1900 帧/5s），但时间同步超时    | 超时太短 + 混流难找 0x108                    | `--timeout 1.5`；按 seq 匹配 PING        | `--ping-only --count 10`                  |
| 4   | START 成功，PING 全超时；V3 上行停止            | PING 在 USART2 中断里发 0x108，与 V3 抢 UART | 取消 ISR 内发 PING；服务请求在锁外处理             | Windows 出现 `cmd=0x03`                     |
| 5   | Windows 只有 `cmd=0x02` 无 `cmd=0x03`   | 同 #4 或固件未更新                          | 重烧固件                                 | `--ping-only`                             |
| 6   | RTT 约 1505 ms                        | 脚本用超时时刻记 t4                          | 收到 0x108 瞬间记 t_recv；真实 RTT 约 5~30 ms | `--timeout 0.5`                           |
| 7   | offset 约 -11000000 ms                | mcu_tick 与 jetson_mono 尺度不同          | `jetson_mono = mcu_tick - offset`    | 对比相邻 PING                                 |
| 8   | `proc=0.00 ms`                       | tick 1 ms 粒度                         | 可接受                                  | —                                         |
| 9   | `utc_unix=0`                         | 无 GPS fix                            | 正常                                   | 户外测 GPS                                   |
| 10  | 串口打不开                                | gateway 占用                           | `fuser` / `pkill rs232_gateway`      | —                                         |


### 2.2 MCU 固件侧最终做法（2026-06-15）


| 项                  | 实现                                                            |
| ------------------ | ------------------------------------------------------------- |
| **0x107 CMD**      | `0x01` QUERY、`0x02` START、`0x03` PING、`0x04` STOP             |
| **0x108 格式**       | QUERY：tick+UTC；PING/START：CMD_ECHO+seq+mcu_tick_rx+flags+proc |
| **RX 路径**          | `USART2_IRQHandler` 只收字节、组 `0xA5` 帧，置 `usart3_svc_ready`      |
| **TX 路径**          | **全部在 JetsonTask 任务里** 调 `USART3_SendServiceFrame` / V3 发送    |
| **任务顺序**           | 服务请求处理在 `**App_ArbiterLock()` 之外**（见 **§2.5**）                |
| **USART2 IRQ 优先级** | 6（须 >= FreeRTOS `configMAX_SYSCALL_INTERRUPT_PRIORITY`）       |
| **调试**             | `JETSON_TIME_SVC_DEBUG=0`（`jetson_can.c` 可改 1 联调）             |


相关源码（流程详解见 **§3**）：

- `APP/jetson_can/jetson_can.c` — `HandleTimeRequest()`、`SendTimeSync()`、`SendTimeSessionRsp()`
- `APP/usart3/usart3.c` — `USART3_ProcessRxByte()`、`USART3_SendServiceFrame()`
- `APP/freertos/rtos_tasks.c` — `vJetsonTask()`

> **最重要的一条**：RS232 模式下 **USART2 只有一根发送线**，V3 与服务帧 **不能在中断里抢发**。细节必读 **§2.5**。

### 2.3 Jetson 测试工具

- `tools/jetson_time_ping_test.py` — START / PING / QUERY / probe
- `tools/jetson_rs232_link_test.py` — V3 上行 listen-only、下行 heartbeat

推荐命令见 §2.5 末尾。

### 2.4 2026-06-15 联调通过（已达成）


| 项目                                    | 结果                |
| ------------------------------------- | ----------------- |
| `--ping-only` 10/10                   | seq 对齐，0x108 格式正确 |
| START + PING + QUERY                  | 全通                |
| Windows `[JETSON TIME] cmd=0x02/0x03` | 与 Jetson 收包一致     |
| PING 后 V3 上行                          | **不中断**（修复前会停）    |


---

### 2.5 【重点】USART2 单通道冲突：为什么禁止 ISR 发串口、为什么服务要在锁外处理

本节说明联调中 **最严重的一类故障**：START 正常、PING 全挂、V3 上行消失。根因不是协议错，而是 **RTOS 下 USART2 发送权冲突**。

#### 2.5.1 硬件与软件共用关系

```text
                    USART2 (PA2 TX / PA3 RX)  —— 物理上只有 1 路 TTL
                              |
        +---------------------+---------------------+
        |                                           |
   RX: USART2_IRQHandler                      TX: 谁都可以调 USART2
   (每字节中断)                                    |
        |                          +----------------+----------------+
        |                          |                                 |
   组 0xAA V3 下行 (24B)         0xA5 服务帧 0x108 (11B)        0xAA V3 上行 (24B)
   组 0xA5 服务帧 0x107           PING/START/QUERY 回复          JetsonTask 周期 20ms
```


| 资源            | 说明                                       |
| ------------- | ---------------------------------------- |
| **USART1**    | 仅 printf 调试，与 Jetson 链路无关                |
| **USART2 TX** | V3 状态/扩展帧 + `0xA5` 封装的服务响应 **共用同一发送函数栈** |
| **USART2 RX** | 在中断里逐字节解析，**不能**在 ISR 里长时间阻塞             |


`USART3_SendByte()` 实现为 **轮询等待 TXE**（忙等），发 11 字节服务帧约 **1 ms**，发 24 字节 V3 约 **2 ms**：

```c
// APP/usart3/usart3.c
USART_SendData(USART2, data);
while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);  // 阻塞直到发完
```

#### 2.5.2 错误设计（v1.5 初版，已废弃）

曾为实现「PING 快路径」，在 **收满 11 字节 0x107 PING** 后于 **中断上下文** 直接：

```text
USART2_IRQHandler
  -> USART3_ProcessRxByte()
       -> JetsonCAN_TryFastServiceReply()   // CMD=0x03
            -> JetsonLink_Deliver8(0x108)
                 -> USART3_SendServiceFrame()  // 在 ISR 里阻塞发 11 字节
```

**设计动机（当时）**：避免 JetsonTask 20 ms 周期导致 PING RTT 偏大。

**实际后果（实测）**：


| 现象           | Jetson 侧                          | Windows 调试口                                |
| ------------ | --------------------------------- | ------------------------------------------ |
| START 成功     | 收到 0x108 echo=0x02                | `[JETSON TIME] cmd=0x02`                   |
| PING 全超时     | 等不到 0x108                         | **无** `cmd=0x03` 或偶发一次                     |
| START 后 V3 停 | probe 从 ~~9600 B/s 降到 **0~~37 B** | `[MOTION]` 仍打印（任务未全死，但 **Jetson 链路 TX 停**） |


#### 2.5.3 根因分析（三条机制叠加）

**机制 1：TX 重入 — 两路发送交错**

```text
时间轴 ->
JetsonTask:  [==== 发 V3 24B，USART3_SendData 循环中 ====]
ISR:              [== 发 0xA5 11B PING 回复 ==]
物理 TX 线:   ... AA 02 ... A5 01 08 ...   （帧边界破坏，Jetson 解析失败）
```

`USART3_SendData` **无互斥锁**。任务发到一半被 RX 中断抢占，ISR 再发服务帧，字节 **交织** 成非法流。

**机制 2：ISR 中长时间阻塞**

- ISR 内 `while(TXE)` 忙等 ~1 ms，此期间若 RX 继续进字节，中断 **嵌套或积压**。
- 115200 下约 87 us/字节，1 ms 内可再进十几次 RX；状态机 `usart3_rx_mode` 与 **发送** 交叉，易出现 **收包丢失 / 不再置 svc_ready**。

**机制 3：与 `App_ArbiterLock()` 叠加（加剧 START 后异常）**

旧代码把 `HandleServiceRequest` 放在 **持锁区内**，且持锁期间发 V3：

```text
App_ArbiterLock();
  GetServiceRequest -> HandleTimeRequest(START);  // 发 0x108，尚可
  USART3_SendV3StatusFrame();                     // 发 24B，耗时长
App_ArbiterUnlock();
```

PING 若在 ISR 抢 TX，持锁任务又占 TX，**链路进入不可预测状态**。表现即：**不是单纯慢，而是 TX 管线被打乱后 V3 周期上行也停**。

> **为何 START 常成功、PING 常失败？**  
> START 走 **JetsonTask 任务路径**（与 V3 顺序执行，虽慢但互斥）。PING 走 **ISR 快路径**，与 V3 **并发抢 TX** — 冲突在 PING 上爆发。

#### 2.5.4 正确设计（当前固件，2026-06-15）

**原则 1：ISR 只做 RX，不做 USART2 TX**

```text
USART2_IRQHandler:
  收字节 -> 状态机组帧
  收满 0xA5 -> usart3_svc_can_id / usart3_svc_payload / usart3_svc_ready = 1
  （不调用任何 Send*）
```

**原则 2：JetsonTask 统一负责 TX（服务响应 + V3）**

```text
vJetsonTask 每 20ms:
  1) DistSnapshot_Read(...)
  2) 【锁外】USART3_GetServiceRequest -> JetsonCAN_HandleServiceRequest
       -> HandleTimeRequest(QUERY/START/PING/STOP)
       -> JetsonLink_Deliver8(0x108)   // 与下一步 V3 顺序执行，无并发 TX
  3) App_ArbiterLock()
  4) 解析 V3 下行、ServiceFault、USART3_SendV3StatusFrame / DetailFrame
  5) App_ArbiterUnlock()
```

对应 `APP/freertos/rtos_tasks.c` 中 **148~177 行** 结构。

**原则 3：服务请求在锁外 — 不是为了“更快”，而是为了**


| 目的                        | 说明                            |
| ------------------------- | ----------------------------- |
| **缩短持锁时间**                | 仲裁器锁不应覆盖 **毫秒级** 串口发送         |
| **避免与 Motion/Can 任务争锁过久** | 发 0x108 + 发 V3 若全在锁内，实时性变差    |
| **逻辑清晰**                  | **TX 串行化** 在单任务内完成，ISR 不参与 TX |


服务处理 **不修改 arbiter 内部状态**（仅时间同步），放锁外 **安全**；`0x10A` 状态查询读 `arb_state` 为只读快照，亦可锁外。

#### 2.5.5 时序示意（修复后 PING 一次）

```text
Jetson                          MCU JetsonTask (20ms)              MCU ISR
  |                                      |                            |
  |-- 0xA5 0x107 PING -------------------|--------------------------->| 收满 11B
  |                                      |                            | svc_ready=1
  |                                      |<- 下一周期 wake ------------|
  |                                      | GetServiceRequest          |
  |                                      | HandleTimeRequest(PING)    |
  |                                      | SendServiceFrame(0x108)    |
  |<-- 0xA5 0x108 ----------------------|                            |
  |                                      | SendV3StatusFrame (0xAA)   |
  |<-- V3 0x02 -------------------------|                            |
```

- PING 额外延迟：**0~20 ms**（等下一 JetsonTask 周期），实测 RTT **约 5~30 ms**（含 RS232 传输），**可接受**。
- 相对 20 ms 控制环，用 1 Hz PING 维持 offset 足够。

#### 2.5.6 延迟与“快路径”的权衡


| 方案                    | PING 额外延迟 | 风险             |
| --------------------- | --------- | -------------- |
| ISR 内回 0x108（已废弃）     | 理论 <1 ms  | **TX 冲突、链路停摆** |
| JetsonTask 锁外回（当前）    | 0~20 ms   | 低，已 10/10 验证   |
| 独立高优先级 TX 任务 + 队列（未做） | 可 <5 ms   | 实现复杂，当前不需要     |


文档 §8.1.3 所述 **sub-ms 延迟** 对本项目 **20 ms 控制周期** 非必须；**链路可靠优先于 PING 快 1 ms**。

#### 2.5.7 开发自查清单（后续改 Jetson 链路必读）

- [ ] **ISR 内是否调用了 `USART3_Send`* / `JetsonLink_Deliver8`？** — RS232 必须为 **否**
- [x] **新增服务响应是否都在 JetsonTask（或单一 TX 任务）内发送？**
- [ ] **是否在持锁期间做 >1 ms 的阻塞 IO？** — 应避免
- [ ] **USART2 IRQ 优先级是否 >= 5（FreeRTOS syscall 阈值）？** — 当前为 6
- [ ] **Jetson 测试**：PING 后 `--probe` 是否仍有 ~25 帧/s V3 上行？
- [ ] **Windows 调试**：连续 PING 是否 **每条** 都有 `cmd=0x03`（`JETSON_TIME_SVC_DEBUG=1`）？

#### 2.5.8 CAN 模式（`JETSON_LINK_CAN=1`）说明

CAN 无「单线 TX 重入」问题，但 `JetsonCAN_ProcessRx()` 仍在 **JetsonTask 锁外** 调用 `HandleTimeRequest`，与 RS232 **同一原则**：**在中断/驱动回调里只收包，回复在任务里发**。

---

## 3. 时间同步协议与代码流程（Jetson / MCU）

本节说明 **0x107 请求 / 0x108 响应** 在两侧如何组帧、传参、处理。帧字段横排表见 [Jetson_RS232协议.md §10](./Jetson_RS232协议.md)。

### 3.1 总览

```text
Jetson                                      MCU (STM32)
  │                                           │
  │  发 SVC: A5 01 07 [8B TIME_REQ]           │
  │ ────────────────────────────────────────> │ USART2 中断收字节
  │                                           │   ProcessRxByte → svc_ready=1
  │                                           │ JetsonTask (~20ms，锁外)
  │                                           │   GetServiceRequest
  │                                           │   → HandleTimeRequest
  │                                           │   → 组 0x108 → SendServiceFrame
  │  收 SVC: A5 01 08 [8B TIME_RSP]           │
  │ <──────────────────────────────────────── │
```

| 侧 | 职责 |
|----|------|
| **Jetson** | 组 `0xA5+0x107+8B` 发出；记录 `t0/t1/t4`；收 `0x108` 算 RTT / offset / UTC |
| **MCU RX** | 中断只收字节、置 `usart3_svc_ready`，**不发串口** |
| **MCU TX** | `vJetsonTask` 任务里读 `buf[0]` 分支，**现采** `NowMs()` 填应答后发送 |

MCU 时间基准：`JetsonCAN_NowMs() = xTaskGetTickCount() * portTICK_PERIOD_MS`。

### 3.2 四种命令

| CMD | 值 | Jetson 发 | MCU 回 0x108 | 说明 |
|:---:|:--:|-----------|:------------:|------|
| QUERY | 0x01 | 拉 tick + UTC | 是（tick+UTC 格式） | v1.4 单次查询 |
| START | 0x02 | 开对时会话 | 是（session 格式） | v1.5 会话开始 |
| PING | 0x03 | 测 RTT | 是（seq 格式） | v1.5 周期心跳 |
| STOP | 0x04 | 关会话 | **否** | 仅清 MCU 会话标志 |

逻辑 ID：**0x107** 请求，**0x108** 响应。

### 3.3 四种命令交互时序（Jetson ↔ MCU）

以下时序中的 **8B 载荷** 经 RS232 封装为 `A5 01 07 [8B]` 发出；应答为 `A5 01 08 [8B]`。数值为便于理解的示例。

#### 3.3.1 QUERY（查时刻）

```text
Jetson                                    MCU
   │                                        │
   │  ────── QUERY ──────────────────────>  │
   │  [0x01, 0,0,0,0,0,0,0]                │
   │                                        │
   │                                    ① 中断收满 11B，svc_ready=1
   │                                    ② JetsonTask 取请求
   │                                    ③ 读当前 tick = 1000
   │                                    ④ 读当前 utc = 123456（无 GPS→0）
   │                                        │
   │  <────── 0x108 应答 ─────────────────  │
   │  [tick=1000, utc=123456]              │
   │                                        │
   │  ⑤ 记录 mcu_tick、mcu_utc              │
   │  ⑥ 与本地时刻对比，建立 UTC 参考         │
```

| 侧 | 处理动作 |
|----|----------|
| **MCU** | 读 `NowMs()` → 读 `GPS_GetUtcUnixSec()` → 打包 0x108 发回 |
| **Jetson** | 记录 `mcu_tick`、`mcu_utc`；无 GPS 时 utc=0 属正常 |

#### 3.3.2 START（开启会话）

```text
Jetson                                    MCU
   │                                        │
   │  ────── START ──────────────────────>  │
   │  [0x02, session=1, jetson_t0=5000]     │
   │                                        │
   │                                    ① 中断收满，任务取请求
   │                                    ② 保存 session_id = 1
   │                                    ③ s_time_session_active = 1
   │                                    ④ 读 tick_rx = 8000
   │                                        │
   │  <────── 0x108 应答 ─────────────────  │
   │  [0x02, 1, mcu_tick_rx=8000, proc, fl] │
   │                                        │
   │  ⑤ 记录 mcu_t0 = 8000                  │
   │  ⑥ 粗算 offset ≈ 8000 - 5000 = 3000   │
```

| 侧 | 处理动作 |
|----|----------|
| **MCU** | 保存 `session_id` → 标记激活 → 读 tick → 回复 0x108 |
| **Jetson** | 记录 `mcu_t0`；粗算 `offset`；进入会话状态 |

> `jetson_t0` 在载荷 P2~P5 中发出，**当前 MCU 不读**；offset 粗算用的是 Jetson 本地 `t0` 与应答里的 `mcu_tick_rx`。

#### 3.3.3 PING（测延迟）

```text
Jetson                                    MCU
   │                                        │
   │  ────── PING ───────────────────────>  │
   │  [0x03, seq=1, jetson_t1=6000]         │
   │                                        │
   │                                    ① 中断收满，任务取请求
   │                                    ② 读 tick_rx = 9000
   │                                    ③ 组 0x108（回显 seq，填 proc）
   │                                        │
   │  <────── 0x108 应答 ─────────────────  │
   │  [0x03, 1, mcu_tick_rx=9000, proc=2]  │
   │                                        │
   │  ④ 记录 t4 = 收到瞬间                   │
   │  ⑤ rtt = t4 - t1                      │
   │  ⑥ one_way = (rtt - proc) / 2         │
   │  ⑦ 精修 offset（指数平滑）               │
```

| 侧 | 处理动作 |
|----|----------|
| **MCU** | 读 tick → 回复（回显 seq + `MCU_PROC_100US`） |
| **Jetson** | 算 RTT → 算单程延迟 → 更新 offset |

> PING 额外等待 **0~20 ms**（下一 JetsonTask 周期）；实测 RTT 约 **5~30 ms**（见 §2.5）。

#### 3.3.4 STOP（结束会话）

```text
Jetson                                    MCU
   │                                        │
   │  ────── STOP ───────────────────────>  │
   │  [0x04, session=1]                    │
   │                                        │
   │                                    ① 中断收满，任务取请求
   │                                    ② 检查 session_id 匹配
   │                                    ③ s_time_session_active = 0
   │                                        │
   │  （无 0x108 应答）                       │
   │                                        │
   │  ④ Jetson 不等应答，直接结束会话           │
```

| 侧 | 处理动作 |
|----|----------|
| **MCU** | 匹配 session 后清 `active` 标志 |
| **Jetson** | 发完即结束，**不等待** 0x108 |

#### 3.3.5 完整会话时间轴

```text
Jetson                                                         MCU
  │                                                              │
  │── START(s=1) ──────────────────────────────────────────────>│ active=1
  │<─ 0x108 echo START ──────────────────────────────────────────│
  │                                                              │
  │── PING #1 ─────────────────────────────────────────────────>│
  │<─ 0x108 echo seq=1 ────────────────────────────────────────│
  │── PING #2 ─────────────────────────────────────────────────>│
  │<─ 0x108 echo seq=2 ────────────────────────────────────────│
  │   ... 1 Hz 或启动阶段 10 Hz × N 次 ...                        │
  │                                                              │
  │── STOP(s=1) ─────────────────────────────────────────────>│ active=0
  │                                                              │
  │── QUERY（可选，任意时刻）────────────────────────────────────>│
  │<─ 0x108 tick + utc ────────────────────────────────────────│
```

### 3.4 MCU 侧代码路径

```text
USART2_IRQHandler
  └─ USART3_ProcessRxByte(byte)
       见 0xA5 → 收满 11B → usart3_svc_can_id=0x107, payload[8], svc_ready=1

vJetsonTask (JETSON_TASK_CYCLE_MS ≈ 20ms)
  1) DistSnapshot_Read(&f,&b,&l,&r,&n)
  2) 【锁外】USART3_GetServiceRequest(&svc_id, svc_buf)
       └─ JetsonCAN_HandleServiceRequest(svc_id, svc_buf, 8, &arb_state, n)
            └─ can_id==0x107 → JetsonCAN_HandleTimeRequest(buf, dlc)
  3) App_ArbiterLock() → V3 下行 / 故障 / V3 上行
  4) App_ArbiterUnlock()
```

| 函数 | 文件 | 作用 |
|------|------|------|
| `USART3_ProcessRxByte` | `usart3.c` | 中断组 SVC 帧 |
| `USART3_GetServiceRequest` | `usart3.c` | 任务取走 0x107 载荷 |
| `JetsonCAN_HandleServiceRequest` | `jetson_can.c` | 按 ID 分发 |
| `JetsonCAN_HandleTimeRequest` | `jetson_can.c` | 按 CMD 分支 |
| `JetsonCAN_SendTimeSync` | `jetson_can.c` | QUERY 应答 |
| `JetsonCAN_SendTimeSessionRsp` | `jetson_can.c` | START/PING 应答 |
| `JetsonLink_Deliver8` → `USART3_SendServiceFrame` | `jetson_can.c` / `usart3.c` | 发 `A5 01 08 [8B]` |

> 时间处理 **不使用** `arb_state` / `nearest_mm` 参数；二者仅用于 0x10A 状态查询。

### 3.5 各命令：帧格式、参数传递、两侧动作

#### 3.5.1 QUERY（0x01）

**Jetson 发（11B 线上）**：

| 0 | 1 | 2 | 3 | 4~10 |
|:-:|:-:|:-:|:-:|------|
| A5 | 01 | 07 | **01** | 0…0 |

**MCU 动作**：

```c
JetsonCAN_SendTimeSync();
  frame[0..3] = JetsonCAN_NowMs();      // 处理瞬间
  frame[4..7] = GPS_GetUtcUnixSec();    // 无 GPS fix → 0
  发 0x108
```

**MCU 回（8B 载荷）**：

| P0~P3 | P4~P7 |
|-------|-------|
| SYSTEM_TICK_MS（u32 BE） | UTC_TIME_SEC（u32 BE） |

**Jetson 动作**（`jetson_time_ping_test.py` → `send_query()`）：

```python
payload = [0x01] + 7×0
ser.write(build_service(0x107, payload))
# 等 0x108 → tick=u32be(payload,0), utc=u32be(payload,4)
```

MCU **只读** `buf[0]`，不解析请求里其余字节。

#### 3.5.2 START（0x02）

**Jetson 发（8B 载荷）**：

| P0 | P1 | P2~P5 | P6~P7 |
|:--:|:--:|:-----:|:-----:|
| 0x02 | session_id | JETSON_MONO_MS（u32 BE） | RSV=0 |

```python
t0 = mono_ms()
payload = struct.pack(">BBIHH", 0x02, session_id, int(t0), 0, 0)
```

**MCU 动作**：

```c
s_time_session_id = buf[1];
s_time_session_active = 1;
tick_rx = JetsonCAN_NowMs();   // 任务处理瞬间，非 ISR
JetsonCAN_SendTimeSessionRsp(0x02, buf[1], tick_rx);
```

**MCU 回（8B 载荷）**：

| P0 | P1 | P2~P5 | P6 | P7 |
|:--:|:--:|:-----:|:--:|:--:|
| 0x02 | session_id | MCU_TICK_RX_MS | TIME_FLAGS | MCU_PROC_100US |

`MCU_PROC_100US = (tick_tx - tick_rx) × 10`，上限 255（25.5 ms）。`TIME_FLAGS` bit0 = GPS UTC 有效。

**Jetson 动作**：

```python
t0 = mono_ms(); 发 START; t_rsp = 收到瞬间
mcu_t0 = u32be(payload, 2)
rtt = t_rsp - t0
offset ≈ mcu_t0 - t0
```

> Jetson 在 P2~P5 携带 `JETSON_MONO_MS`，**当前 MCU 不读**；MCU tick 在任务里现采。

#### 3.5.3 PING（0x03）

**Jetson 发**：

| P0 | P1 | P2~P5 | P6~P7 |
|:--:|:--:|:-----:|:-----:|
| 0x03 | seq | JETSON_MONO_MS | RSV=0 |

**MCU 动作**：

```c
tick_rx = JetsonCAN_NowMs();
JetsonCAN_SendTimeSessionRsp(0x03, buf[1], tick_rx);  // 回显 seq
```

**Jetson 动作**（offset 精修）：

```python
t1 = mono_ms(); 发 PING(seq); t4 = 收到瞬间
mcu_rx = u32be(payload, 2)
proc_ms = payload[7] * 0.1
rtt = t4 - t1
one_way ≈ (rtt - proc_ms) / 2
offset = mcu_rx - t1 - one_way
offset = 0.8 * old_offset + 0.2 * offset   # 指数平滑
```

#### 3.5.4 STOP（0x04）

**Jetson 发**：P0=0x04，P1=session_id，其余填 0。

**MCU 动作**：

```c
if (buf[1] == s_time_session_id) s_time_session_active = 0;
// 不回 0x108
```

### 3.6 两种 0x108 应答格式（不可混解析）

| 触发 | P0~P7 布局 |
|------|------------|
| **QUERY** | `[tick_ms u32 BE][utc_sec u32 BE]` |
| **START / PING** | `[CMD_ECHO][ECHO][mcu_tick_rx u32 BE][flags][proc_100us]` |

### 3.7 推荐会话流程（Jetson 侧）

```text
1. START(session=1)      → 收 0x108，粗算 offset
2. 循环 PING(seq=1..N)   → 收 0x108，精修 offset / 监测 RTT
3. STOP(session=1)       → MCU 清 s_time_session_active
4. 可选 QUERY(0x01)      → 拿 UTC（需 GPS fix）
```

测试工具：

```bash
python3 tools/jetson_time_ping_test.py --port /dev/ttyUSB7 --count 10
python3 tools/jetson_time_ping_test.py --port /dev/ttyUSB7 --query-only
python3 tools/jetson_time_ping_test.py --port /dev/ttyUSB7 --probe
```

### 3.8 MCU 会话静态变量

| 变量 | 写入 | 作用 |
|------|------|------|
| `s_time_session_id` | START 时 `buf[1]` | STOP 匹配用 |
| `s_time_session_active` | START=1，STOP 匹配后=0 | 会话标志（PING 不检查） |

### 3.9 数值示例：一次 PING

假设 Jetson `t1=100000 ms` 发送，MCU 任务里 `tick_rx=85000 ms`，发前 `tick_tx=85002 ms`，Jetson `t4=100015 ms` 收到：

**Jetson 发**：`A5 01 07 03 05 [t1 四字节] 00 00`

**MCU 回**：`A5 01 08 03 05 00 01 4E 88 01 14`

| 字段 | 值 | 来源 |
|------|:--:|------|
| mcu_rx | 85000 (0x00014E88) | MCU 处理 PING 时 `NowMs()` |
| flags | 0x01 | GPS UTC 有效 |
| proc | 0x14 (=20) | (85002−85000)×10 |

Jetson 计算：RTT=15 ms，proc=2 ms，one_way≈6.5 ms，offset≈mcu_rx−t1−one_way。

### 3.10 传输延迟与 offset（原理）

往返无法消除，目标是 **测量 RTT** 并建立 `mcu_tick ≈ jetson_mono + offset`。符号定义与公式详见 [Jetson_CAN协议.md §8.1.3~8.1.4](./Jetson_CAN协议.md)。Jetson 侧时钟用 `time.monotonic()`。

| 符号 | 含义 |
|------|------|
| t1 | Jetson 发 PING 前 `mono_ms()` |
| t2 | MCU 收到时 `MCU_TICK_RX_MS`（0x108 P2~P5） |
| t3 | MCU 发 0x108 前 tick；`proc ≈ (t3−t2)` |
| t4 | Jetson 收到 0x108 瞬间 `mono_ms()` |
| RTT | t4 − t1 |
| one_way | ≈ (RTT − proc) / 2 |
| offset | ≈ t2 − t1 − one_way（多次 PING 平滑） |

---

## 4. 下一步工作（2026-06-15 起）

### 4.1 Jetson 侧 — 单调时钟对时


| 项                                           | 状态       |
| ------------------------------------------- | -------- |
| rs232_gateway + `/jetson_rs232/time_sync`   | ✅ 实车验证通过 |
| 1 Hz PING + 10 s QUERY                      | ✅        |
| RTT ~40 ms、offset 稳定                        | ✅        |
| `gps_rs232_to_fix` 用 offset 修正 `/fix` stamp | ⬜ 待做     |
| 1 h soak（offset 漂移 / rtt_ema）               | ⬜ 待做     |


### 4.2 MCU / 协议 — 四路测距时间戳


| 项                                       | 状态              |
| --------------------------------------- | --------------- |
| `SonarSnapshot_t` + 每路滤波更新打戳            | ⬜ 见 **§5**      |
| 新 V3 type=0x04 或 0xA5 双服务帧上送 4×u32 tick | ⬜ 协议已定稿方向，代码未实现 |
| GPS → RTC                               | ⬜ 可选            |


### 4.3 整车

- 日志时间轴统一（PING offset + 各传感器 stamp）
- 户外 GPS fix 后复验 QUERY `utc_unix`

---

## 5. 四路测距时间戳扩展（需求与协议草案）

> **与时间同步的关系**：**§3** 的 PING/offset 解决「Jetson 钟 ↔ MCU 钟」换算；本节解决「四路 sonar **各自何时更新**」——两者叠加后，Jetson 才能把每路距离放到统一时间轴上。

### 5.1 需求定稿（与算法侧对齐）


| 项               | 定稿                                                                       |
| --------------- | ------------------------------------------------------------------------ |
| **对象**          | 四路 E08 测距：前 / 后 / 左 / 右（IF1~IF4）                                         |
| **每路 stamp 含义** | **该路滤波输出 `stable_mm` 最后一次变化** 时的 MCU 时刻（非整帧 UART 到达时刻）                   |
| **tick 格式**     | `**u32` 全量 ms**，与 `0x108 SYSTEM_TICK_MS` / PING 同一 `Log_GetUptimeMs()` 域 |
| **MCU 内存组织**    | 用 **结构体** 把「距离 + stamp」绑在一起，避免只更新 mm 忘记 tick                             |
| **上送**          | 必须经 **JetsonTask 任务内 TX**（遵守 §2.5，禁止 ISR 发串口）                            |


### 5.2 MCU 侧数据结构（规划）

替换当前 `DistSnapshot_Write(f,b,l,r,n)` 仅传 `u16` 的做法：

```c
typedef struct {
    u16 dist_mm;           /* 滤波后距离；无效用 DS_DIST_UNKNOWN */
    u32 sample_tick_ms;    /* 该路 stable_mm 最后更新时刻 */
    u8  valid;             /* 1=本路当前 dist 可信 */
} SonarLane_t;

typedef struct {
    SonarLane_t front;     /* DS_IDX_FRONT / IF1 */
    SonarLane_t back;
    SonarLane_t left;
    SonarLane_t right;
    u16         nearest_mm;
    u32         snap_tick_ms;  /* JetsonTask 读快照时刻（可选，调试） */
} SonarSnapshot_t;
```

**打戳位置（规划）**：

```text
DistanceSensor_FilterOnNewFrame()
  -> 某路 stable_mm 相对上次发生变化
       -> SonarLane.sample_tick_ms = Log_GetUptimeMs()
       -> SonarLane.dist_mm = stable_mm
SensorTask -> SonarSnapshot_Write(&snap)
JetsonTask -> SonarSnapshot_Read(&snap) -> 组 V3 / 服务帧发送
```

相关现有代码：`APP/distance_sensor/distance_sensor.c`（滤波）、`APP/freertos/motion_ui_shared.c`（快照）、`APP/freertos/rtos_tasks.c`（SensorTask / JetsonTask）。

### 5.3 现有 V3 type=0x02 为何装不下 4×u32

当前 **24 B** 状态帧（`frame_type=0x02`）字节已占满：


| 字节     | 字段         | 占用     |
| ------ | ---------- | ------ |
| 0~11   | 头、模式、速度、转角 | 12     |
| 12~13  | SONAR_F    | 2      |
| 14~15  | SONAR_B    | 2      |
| 16~17  | SONAR_L    | 2      |
| 18~19  | SONAR_R    | 2      |
| 20~21  | BAT_V      | 2      |
| 22     | SOC        | 1      |
| 23     | XOR        | 1      |
| **合计** |            | **24** |


四路 **u32 全量 tick** 需要 **4×4 = 16 B**，帧内 **无空闲 16 B**。

#### 5.3.1 曾考虑的「塞保留字节」方案 — 不可行

旧 Phase2 草案曾设想 V3 **byte14~17** 填单个 `TX_TICK_MS`（见 [Jetson_CAN协议.md §8.1.7](./Jetson_CAN协议.md)），但 **现行固件** 已用 byte14~~19 承载 **SONAR_B/L/R**，byte20~~22 为电池与 SOC。

即便强行把 byte14~~22 当作「保留区」，也只有 **9 B**（14~~22），仍 **不够 16 B**；且会与距离、BMS 字段冲突。


| 方案                     | 可用字节   | 4×u32 需要 | 结论              |
| ---------------------- | ------ | -------- | --------------- |
| 0x02 byte14~22 塞 stamp | 9      | 16       | ❌ 不够            |
| 0x02 去掉 SONAR 改发 tick  | —      | —        | ❌ Jetson 避障仍要距离 |
| **新帧专门发 tick**         | 24（整帧） | 16       | ✅ 见 §5.5        |


### 5.4 上送方案对比


| 方案                    | 做法                                                          | 优点                             | 缺点                                   | 结论       |
| --------------------- | ----------------------------------------------------------- | ------------------------------ | ------------------------------------ | -------- |
| **A. 新 V3 type=0x04** | 0x02 仍发 4×dist；另增 **0x04 测距 stamp 帧** 发 4×u32               | 与现有 0x02 兼容；单帧 24B 刚好装下 4 tick | 带宽 +1 帧/周期；Jetson 需关联 0x02 与 0x04    | **推荐**   |
| **B. 0xA5 双服务帧**      | 两帧 × 8B payload，例如 **0x111**（F/B tick）+ **0x112**（L/R tick） | 不动 V3 布局；与时间同步同为 0xA5 混流       | 每周期 +22B×2；需分配新 CAN ID；gateway 多两路解析 | 备选       |
| **C. 0xA5 单帧 8B**     | 只发 2 路 tick                                                 | —                              | **8B 装不下 4×u32**                     | ❌        |
| **D. CAN FD DLC=16**  | 一条 16B 服务响应                                                 | 一帧搞定                           | 当前 RS232/CAN2 均为 **Classic 8B**      | ❌ 现硬件不支持 |
| **E. tick 压缩 u16 增量** | 相对 `snap_tick_ms` 的 delta                                   | 省字节                            | 与「u32 全量 ms」需求不符                     | ❌ 不采用    |


### 5.5 推荐定稿：V3 type=0x04（测距 stamp 专帧）

**设计原则**：**距离与时间戳分帧** — 0x02 保持现状；新增 **0x04** 仅承载四路 `sample_tick_ms`，Jetson 用 **SEQ**（或与 0x02 相同 TX 序号规则）关联同一周期快照。

#### 5.5.1 帧布局（24 B，规划中）


| 字节    | 字段         | 类型     | 说明                            |
| ----- | ---------- | ------ | ----------------------------- |
| 0     | HEADER     | u8     | `0xAA`                        |
| 1     | FRAME_TYPE | u8     | `**0x04`**                    |
| 2     | SEQ        | u8     | 与本轮 0x02 同序或独立递增（gateway 需约定） |
| 3     | RSV        | u8     | `0`                           |
| 4~7   | STAMP_F    | u32 BE | 前向 `sample_tick_ms`           |
| 8~11  | STAMP_B    | u32 BE | 后向                            |
| 12~15 | STAMP_L    | u32 BE | 左向                            |
| 16~19 | STAMP_R    | u32 BE | 右向                            |
| 20~22 | META       | u8×3   | 预留：如 `nearest_mm` 高/低 + flags |
| 23    | XOR_CHK    | u8     | XOR(byte0~22)                 |


**MCU 发送顺序（JetsonTask 内，持锁区或与 0x02 相邻）**：

```text
... 处理服务帧（锁外）...
App_ArbiterLock();
  解析 V3 下行
  USART3_SendV3StatusFrame(...)     /* 0x02：dist + 运动状态 */
  USART3_SendV3SonarStampFrame(...) /* 0x04：4×u32 tick，新增 */
  /* 或交替 0x03 detail */
App_ArbiterUnlock();
```

**Jetson 换算**（沿用已有 offset）：

```text
lane_time_on_jetson ≈ STAMP_x - offset_ms    /* 单调时钟 */
绝对 UTC（有 GPS QUERY 时）≈ utc_unix + (STAMP_x - query_tick_ms) / 1000
```

#### 5.5.2 带宽粗算

115200 8N1 下 24 B V3 ≈ **2 ms**/帧。每 20 ms 多一帧 0x04 ≈ **+50 帧/s** → 可接受（仍远低于 probe 上限）。

### 5.6 备选：0xA5 双服务帧（8 B × 2）

若坚持 **不动 V3 type 枚举**，可用 **两帧 0xA5 服务帧**（Classic CAN 语义，RS232 上各 11 B）：


| CAN ID    | 方向           | payload 8B                       | 内容        |
| --------- | ------------ | -------------------------------- | --------- |
| **0x111** | STM32→Jetson | `[F_tick u32 BE][B_tick u32 BE]` | 前/后 stamp |
| **0x112** | STM32→Jetson | `[L_tick u32 BE][R_tick u32 BE]` | 左/右 stamp |


```text
0xA5 帧线格式（与 0x108 相同）：
[0xA5][ID_H][ID_L][8B payload...]
```

**约束**：

- 单帧 payload **固定 8 B** → 一路 u32 最多 2 路/帧 → **必须 2 帧** 才能凑齐 4 路。
- **不是 CAN FD**；RS232 上无 DLC=16。
- 两帧须在同一 JetsonTask 周期 **连续发送**，避免 TX 重入（§2.5）；gateway 以 **相同 `snap_tick_ms` 或连续 SEQ** 组包。

> **ID 说明**：0x111/0x112 为测距 stamp **预留 ID**（需同步修订 [Jetson_CAN协议.md](./Jetson_CAN协议.md) 附录 ID 表；勿与 0x10C OTA / 0x10F IMU 冲突）。

### 5.7 实现检查清单

**MCU**

- [ ] 定义 `SonarSnapshot_t`，替换 `DistSnapshot_Write/Read`
- [ ] 在滤波 `stable_mm` 变化处写 `sample_tick_ms`（**每路独立**）
- [ ] 实现 `USART3_SendV3SonarStampFrame()`（0x04）或 `JetsonLink_Deliver8`×2（0x111/0x112）
- [ ] **仅在 JetsonTask 发 TX**；新增帧不得进 ISR
- [ ] Windows `JETSON_TIME_SVC_DEBUG` / 串口 log 可打印四路 tick 抽样

**Jetson / gateway**

- [ ] 解析 0x04（或 0x111/0x112）
- [ ] 与 0x02 SONAR 按 SEQ/时间窗对齐
- [ ] 发布 ROS：`/jetson_rs232/sonar_stamped`（或 4 字段 msg）+ 沿用 `/jetson_rs232/time_sync` 的 `offset_ms`
- [ ] 文档化：QUERY 帧 `cmd_echo=0` 不是 PING 失败（见联调 FAQ）

**验证**

- [ ] 静止：四路 stamp 随滤波更新而变，步进合理（≤ 280 ms 触发周期量级）
- [ ] 运动：四路 stamp 可不同（符合「每路滤波更新时刻」）
- [ ] PING 1 Hz 下 offset 稳定；发 0x04 后 V3 上行不中断（§2.5 回归）
- [ ] bag 中 `STAMP_F` 换算到 Jetson mono 后，与 gateway 收帧时刻差 ≈ RTT/2 量级

### 5.8 与 §2.5 / §3 的衔接

```text
        PING/QUERY (0x107/0x108)  ← §3
              |
              v
         offset_ms  ——>  Jetson 全局时钟换算
              ^
              |
   0x02 dist + 0x04 stamp (或 0x111/0x112)
              |
              v
    每路: (dist_mm @ sample_tick_ms) -> 统一时间轴上的障碍观测
```

**禁止**：为降低延迟在 USART2 RX ISR 内发送 0x04/0x111（会复现 §2.5 PING 灾难）。

---

## 6. 相关文档

- [Jetson_CAN协议.md](./Jetson_CAN协议.md)
- [Jetson_RS232协议.md](./Jetson_RS232协议.md)
- [硬件连接与通信协议.md](./硬件连接与通信协议.md)

---

## 附录：变更记录


| 版本     | 日期         | 内容                                              |
| ------ | ---------- | ----------------------------------------------- |
| v1.0   | 2026-06-15 | 初版                                              |
| v1.0.1 | 2026-06-15 | 修复 UTF-8 乱码                                     |
| v1.1   | 2026-06-15 | 新增 §2.5：ISR/锁外处理专题（USART2 TX 冲突）                |
| v1.2   | 2026-05-28 | 新增 §5：四路测距 u32 stamp、V3 0x04 / 0xA5 双帧；更新 §4 状态 |
| v1.2.1 | 2026-05-28 | 修复 StrReplace 导致的 UTF-8 乱码（PowerShell 全文重写）     |
| **v1.3** | **2026-06-15** | **新增 §3：时间同步协议与代码流程（Jetson/MCU 帧格式、参数传递、源码路径）** |
| **v1.3.1** | **2026-06-15** | **§3.3 新增 QUERY/START/PING/STOP 交互时序图与完整会话时间轴** |


