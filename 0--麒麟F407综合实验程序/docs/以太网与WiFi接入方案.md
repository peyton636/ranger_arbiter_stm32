# 以太网与 WiFi 接入方案

| 元数据 | 值 |
|--------|-----|
| **文档版本** | v1.6 |
| **文档日期** | 2026-06-17 |
| **状态** | **L3 ping 已通**；**UDP+BLOB v2 上行已通**；**UDP 0xA5 TimeSync 已合入**；测距供电见 **§15.11** |
| **适用固件** | 麒麟 F407 综合实验程序，`UI_TEST_MODE=1` 仲裁模式 |

> **资源冲突（引脚/串口/中断/网络）必读：[§15](#15-资源冲突与解决办法必读)**

本文档描述 **上位机 ↔ F407** 经 **网线直连** 与 **WiFi + 网页** 两种网络控制的**整体流程、架构分层与实施步骤**。

| 相关文档 | 说明 |
|----------|------|
| [硬件连接与通信协议.md](./硬件连接与通信协议.md) | CAN / RS232 接线与系统拓扑 |
| [Jetson_CAN协议.md](./Jetson_CAN协议.md) | V3 应用层帧定义（以太网/WiFi 建议复用） |
| [Jetson_RS232协议.md](./Jetson_RS232协议.md) | RS232 传输封装（与 CAN 应用层一致） |
| [Jetson_BLOB协议_v2.md](./Jetson_BLOB协议_v2.md) | BLOB v2 二进制协议（RS232/CAN/以太网 UDP 共用应用层） |
| [测距传感器数据流.md](./测距传感器数据流.md) | USART3 测距触发/收帧（与以太网无引脚冲突，见 §15.4） |
| [rtos移植.md](./rtos移植.md) | 内存规划（含后续 WiFi 缓冲约定） |

---

## 1. 你要做的两件事（总览） 检查 上位机是否有网线接口驱动！！！！！！

领导要求的方向可以归纳为 **两条独立但可复用应用层协议的控制路径**：

| 功能 | 一句话 | 典型场景 |
|------|--------|----------|
| **① 网线直连控制** | 用 RJ45 物理网线，上位机/Jetson 与 F407 直接收发控制指令 | 实验室联调、低延迟、和 CAN/RS232 做延时对比 |
| **② WiFi + 网页控制** | 手机/电脑连 WiFi，浏览器打开网页下发速度、模式等 | 现场演示、远程调试、非 Jetson 人员操作 |

```text
                    +-------------------------------------+
                    |         F407 仲裁板（本工程）          |
                    |  Arbiter + V3 应用层（24B 控制帧）   |
                    +-----------+-------------+-----------+
                                |             |
              ① 网线直连         |             |  ② WiFi / 网页
                                |             |
                    +-----------v---+   +-----v------------------+
                    | Jetson / PC   |   | 路由器 WiFi             |
                    | UDP/TCP 二进制 |   | 浏览器 -> HTTP 网页控制   |
                    +---------------+   +------------------------+
```

**重要原则**：无论走网线、WiFi 还是已有的 CAN/RS232，**应用层尽量统一用 V3 帧**（`0xAA` + type + seq + payload + XOR），只换「传输封装」，避免协议碎片化。

---

## 2. 当前工程里已经有什么？

### 2.1 以太网（网线）— 硬件 + 演示已有，控制链路未接

| 项目 | 现状 | 代码位置 |
|------|------|----------|
| PHY 芯片 | **LAN8720**，板载 RJ45 | `APP/lan8720/` |
| TCP/IP 协议栈 | **LwIP 1.4.1**（TCP/UDP/ICMP） | `LwIP/` |
| 默认 IP | 静态 **F407 `192.168.10.30`**，上位机 **`192.168.10.201`**（`remoteip`） | `LwIP/lwip_app/lwip_comm/lwip_comm.c` |
| 仲裁模式以太网 | **已接入**：`ETH_LWIP_ENABLE=1`，`App_EthInit()` + `vNetTask` | `APP/freertos/app_boot.c` |
| RX 收包模式 | **FreeRTOS 轮询**：`ETH_RX_POLL_ONLY=1`，Net 任务 drain DMA（不用 ETH IRQ） | `app_boot.h`、`app_boot.c` |
| 诊断计数 | 启动 gratuitous ARP + 周期打印 `rx/tx_ok/rbus/desc_cpu` | `lan8720.c`、`rtos_tasks.c` |
| lwIP 内存 | 协议池/heap 在 **外扩 SRAM（SRAMEX）**；DMA 描述符/RxTx 缓冲在 mem1 | `lwip_comm.c` |
| PHY 链路检测 | 串口打印 `PHY link UP/DOWN` | `LAN8720_LinkUp()` |
| 演示应用 | HTTP 网页控 LED/蜂鸣器/ADC（Legacy GUI，仲裁模式未启用） | `GUI_APP/eth_app.c` |
| 与 RS232 冲突 | **`PA2` 复用**：`ETH_MDIO` vs `USART2_TX`；以太网 init 在 RS232 init **之后**，会占住 PA2 | 见 §12.10 |

### 2.2 WiFi — 仅规划，代码未实现

| 项目 | 现状 |
|------|------|
| WiFi 模组驱动 | **无**（`malloc.h` 注释预留了 AT 指令缓冲空间） |
| 网页控制 | 以太网演示里 **HTTP Server 可复用**，需改成运动控制页面 |
| 内存 | mem1 32KB（小缓冲）、mem2 960KB（大 payload），见 `rtos移植.md` |

### 2.3 已有上位机链路（对比用）

| 链路 | 宏 | 状态 |
|------|-----|------|
| CAN2 | `JETSON_LINK_CAN = 1` | 已实现 |
| RS232 | `JETSON_LINK_CAN = 0` | 已实现（当前默认） |
| 以太网 | `ETH_LWIP_ENABLE=1` | lwIP 已启、**ping 已通**；UDP/BLOB 控制链路待接 `jetson_eth` |
| RS232 BLOB v2 | `JETSON_USE_BLOB_V2=1` | 代码已接；与以太网 **编译期二选一**（PA2 冲突，见 §12.10） |

---

## 3. 系统分层（所有网络方案共用）

无论网线还是 WiFi，建议按四层理解，上层不变、下层可换：

```text
+---------------------------------------------------------+
| 应用层    V3 帧（0x01 控制 / 0x02 状态 / 0x03 扩展）      |  <- 复用 Jetson_CAN协议.md
+---------------------------------------------------------+
| 会话层    UDP 端口 / HTTP CGI / WebSocket（按场景选）     |
+---------------------------------------------------------+
| 传输层    UDP（实时控制）或 TCP（网页/OTA）               |
+---------------------------------------------------------+
| 网络层    IPv4 + ICMP(ping)                              |
+---------------------------------------------------------+
| 链路层    以太网 100M（LAN8720）或 WiFi 模组             |
+---------------------------------------------------------+
```

**F407 内部数据流**（与 CAN/RS232 相同终点）：

```text
网络收包 -> 解析 V3 -> Arbiter_ParseJetsonCmd() -> 避障仲裁 -> CAN1 0x111 -> 底盘
                ^
状态/GPS <- USART3_SendV3StatusFrame() 等 <- arb_state
                |
网络发包 -> 回传 0x02/0x03/服务帧
```

---

## 4. 功能一：网线直连控制（整体流程）

### 4.1 物理连接

```text
方案 A — 直连（推荐联调）
  Jetson eno1 --网线-- F407 RJ45
  Jetson 静态 IP：192.168.10.200/24
  F407 固定：192.168.10.30/24（写死在 lwip_comm_default_ip_set()，非 Flash 配置）

方案 B — 经交换机/路由器
  Jetson、F407、PC 接同一局域网（须同一广播域）
  可给 F407 配静态 IP 或后续开 DHCP（当前 LWIP_DHCP=0）
```

> 上电初始化 lwIP 时**建议插好网线**，否则 `LAN8720_Init()` 可能失败（演示代码 `eth_app.c` 已有此提示）。

### 4.2 推荐协议：UDP 透传 V3

**为什么先用 UDP 而不是 MQTT/HTTP？**

- 与现有 V3 24 字节帧天然匹配，改动最小
- 延迟低，适合和 CAN/RS232 做对比测试
- F407 RAM 紧张，UDP 比 MQTT 客户端省资源

**端口规划（建议）**

| 方向 | 协议 | 端口 | 载荷 |
|------|------|------|------|
| 上位机 -> F407 | UDP | **50001** | BLOB v2 下行 `MSG=0x01`（线长 23B）或 V3 `0x01`（24B，旧） |
| F407 -> 上位机 | UDP | **50002** | BLOB `0x02/0x03/…` 或 V3 `0x02/0x03` |
| 服务帧（可选） | UDP | **50003** | 0xA5 时间同步等（11B） |
| 连通性测试 | ICMP | — | `ping <F407_IP>` |

**单帧交互时序**

```text
上位机                              F407
  |  UDP:50001  [24B V3 0x01 seq=N]  -->  收包 -> 校验 XOR -> Arbiter_ParseJetsonCmd
  |  <-- UDP:50002  [24B V3 0x02]     |  vJetsonTask 周期上报（20ms）
  |  <-- UDP:50002  [24B V3 0x03]     |  交替发 status/detail
```

### 4.3 MCU 侧实施流程（分步）

> **完整对照表与验收标准见 [§14](#14-网线-udp-发消息完整实现流程ping-通过后)**。以下为摘要。

```text
步骤 1 — 编译开关（三选一）
  JETSON_LINK_CAN=1          → CAN2
  JETSON_LINK_CAN=0 + ETH=0  → RS232 BLOB v2（现有默认）
  JETSON_LINK_CAN=0 + ETH=1  → 以太网 UDP BLOB v2（新建 JETSON_LINK_ETH）

步骤 2 — lwIP 已就绪（ETH_LWIP_ENABLE=1）
  App_EthInit() + vNetTask 周期 App_EthPoll()  ← 工程已有

步骤 3 — 新建 APP/jetson_eth/（待实现）
  JetsonEth_Init()：UDP bind 50001，注册 recv 回调
  收：UDP payload → blob_rx_frame_t 环形队列 → BlobPack_HandleDownlink()
  发：BlobEth_Send() 替代 BlobRs232_Send()，UDP 发往 remoteip:50002

步骤 4 — 改 vJetsonTask / agv_blob_pack.c
  以太网分支复用现有 BLOB 编解码，仅换物理发送函数

步骤 5 — 上位机 Python 脚本验证（见 §14.5）
```

### 4.4 上位机侧（Jetson / PC）

```python
# BLOB v2 over UDP（推荐，与 RS232 应用层一致）
# 见 §14.5 完整脚本
sock_tx.sendto(blob_0x01_wire, ("192.168.10.30", 50001))
data, _ = sock_rx.recvfrom(2048)   # 期望 0xAB 头 + PAYLOAD
```

### 4.5 延时测量（网线）

| 层级 | 方法 | 说明 |
|------|------|------|
| 网络层 | `ping -I 192.168.10.200 192.168.10.30` | ICMP 往返，通常 < 2 ms，**不等于**控制延时 |
| 应用层 | V3 echo：发 0x01，测到 0x02 的 RTT | 与 CAN/RS232 对比的主指标 |
| 系统层 | 含 `JETSON_TASK_CYCLE_MS=20` | 状态上报最快约 50 Hz，需在报告中注明 |

---

## 5. 功能二：WiFi + 网页控制（整体流程）

「上传到网页然后控制」可以拆成 **网页从哪来** 和 **指令怎么到 F407** 两个问题。

### 5.1 三种常见架构（选一种做主方案）

#### 架构 A — F407 以太网接路由器，网页跑在板子里（推荐先做）

```text
  手机/笔记本 --WiFi-- 路由器 --网线-- F407（192.168.1.30）
                              |
                         浏览器打开 http://192.168.1.30/
                              |
                         F407 内置 HTTP Server（已有 httpd 演示）
                              |
                         CGI：/cmd?v=100&steer=0 -> 组 V3 帧 -> Arbiter
```

| 优点 | 缺点 |
|------|------|
| 复用现有 LwIP + httpd，无需 WiFi 模组 | F407 必须网线接到路由器 |
| 手机连同一 WiFi 即可控 | HTTP 不适合高频实时（适合演示/点动） |

**「上传网页」的含义**：把 `.shtml` / `.html` 页面编译进固件文件系统（`makefsdata` 工具），或后续从 SD 卡加载。

#### 架构 B — F407 挂 WiFi 模组（ESP8266/ESP32），网页仍在板子或模组里

```text
  手机 --WiFi-- ESP8266 AP/STA --UART AT-- F407
                    |
               模组内置配网 + 简易 Web（或透传 UDP 到 F407）
```

| 优点 | 缺点 |
|------|------|
| 真正无线，车上不用拉网线 | 需新写 AT 驱动、占 UART、增加模块成本 |
| 适合户外演示 | 文档标注为「后续」，当前工程 **无实现** |

#### 架构 C — Jetson 当网关：Jetson 开网页，经网线/CAN 转发给 F407

```text
  手机 --WiFi-- Jetson（跑 Flask/ROS Web）
                    |
              网线 UDP 或 CAN --> F407
```

| 优点 | 缺点 |
|------|------|
| F407 改动最小 | 依赖 Jetson 常开；不是「板子独立网页」 |

**建议实施顺序**：**A（网线+路由器+网页）-> ① 网线 UDP 联调稳定后 -> B 或 C 按硬件条件选**。

### 5.2 网页控制流程（以架构 A 为例）

```text
+----------+    GET /              +-------------+
| 浏览器    | ------------------> | httpd       | 返回 index.shtml（控制面板）
+----------+                       +-------------+
      |
      |  GET /motion.cgi?v=100&steer=0&mode=1
      v
+-------------+   组 V3 0x01 帧    +-------------+
| CGI 处理函数 | ----------------> | Arbiter      |（与 UDP 路径汇合）
+-------------+                   +-------------+
      |
      |  SSI 刷新：<!--#status--> 嵌入当前速度/距离
      v
  浏览器显示实时状态（轮询或定时刷新，如 200ms）
```

**网页上建议提供的控件**（映射 V3 字段，见 `Jetson_CAN协议.md` 4）：

| 网页控件 | V3 字段 | 说明 |
|----------|---------|------|
| 线速度滑条 | `V_H/V_L` | mm/s，有符号 |
| 转角滑条 | `STEER_H/L` | 0.1 度 |
| 运动模式 | `MOTION_MODEL` | 阿克曼/原地转/横移 |
| 待机/运行 | `MODE_REQ` | 0=待机 1=运行 |
| 清错按钮 | `CLEAR_ERROR` | 一次性触发 |
| 状态区 | 读 `arb_state` | 安全态、四向测距、GPS |

### 5.3 从演示网页到控制网页（改造要点）

现有演示：`httpd_cgi_ssi.c` 中 `/leds.cgi` 控制 LED。

改造步骤：

```text
1. 复制 fs/ 下网页模板，新建 motion.shtml（速度、模式表单）
2. 新增 /motion.cgi -> Motion_CGI_Handler()：
     解析 pcParam/pcValue -> 填 V3 24B -> Arbiter_ParseJetsonCmd()
3. 新增 SSI handler：注入 nearest_dist、current_mode 等
4. makefsdata 重新生成 fs.c 并编入工程
5. 在 vNetTask 或 vJetsonTask 中调用 httpd_init() + lwip_periodic_handle()
```

**安全注意**：网页控制务必走 **Arbiter 仲裁**（测距避障、心跳丢失停车），不要绕过 `Arbiter_Execute` 直接发 CAN。

### 5.4 WiFi 模组方案（架构 B，后续）

若板子需加 ESP8266 等模组：

```text
硬件：模组 UART 接 F407 空闲串口（勿与 Jetson USART2、测距 USART3、GPS USART6 冲突）
软件：
  1. AT 指令初始化：STA 连路由器或 AP 模式
  2. 透传模式：模组 TCP/UDP <-> 串口字节流
  3. 应用层仍用 V3 或把 HTTP 请求由模组转成串口行协议
内存：AT 缓冲放 mem1（<=2KB），大 payload 放 mem2（SRAMEX）
```

---

## 6. 三种链路如何与 CAN/RS232 共存？

| 策略 | 说明 | 推荐 |
|------|------|------|
| **编译期三选一** | `JETSON_LINK_TYPE` = CAN / RS232 / ETH，与现有一致 | 推荐先做 |
| **以太网 + CAN 并存** | 网线仅供调试监控，Jetson 控制仍走 CAN | 车上可长期保留 |
| **运行时热切换** | 复杂，易双发冲突 | 不建议初期做 |

延时对比测试时，**每次只开一条控制链路**，记录同一场景下的 RTT 表格。

---

## 7. 实施路线图（建议排期）

```text
阶段 0 — 基线（1~2 天）
  [ ] CAN、RS232 联调通过（已有固件）
  [ ] 记录现有链路 RTT 基线

阶段 1 — 网线 UDP 控制（1~2 周）
  [x] lwip_comm_init 接入仲裁模式
  [x] ping 连通（PC 或 Jetson eno1）
  [ ] jetson_eth 模块 + vJetsonTask 分支
  [ ] 上位机 UDP BLOB 收发脚本 / eth_gateway
  [ ] BLOB RTT 测试报告（对比 RS232）

阶段 2 — 网线 HTTP 网页（1 周）
  [ ] motion.shtml + motion.cgi
  [ ] 路由器组网，手机浏览器验证
  [ ] 点动、待机、状态刷新

阶段 3 — WiFi（按需，2~3 周）
  [ ] 选定架构 A/B/C
  [ ] 若 B：ESP8266 驱动 + 透传
  [ ] 与阶段 2 网页复用同一 CGI 逻辑

阶段 4 — 交付（2~3 天）
  [ ] 《三链路 + 网络延时对比报告》
  [ ] 接线说明、IP 配置、网页使用说明
```

---

## 8. 关键源码索引

| 模块 | 路径 | 作用 |
|------|------|------|
| 链路选择宏 | `APP/jetson_can/jetson_can.h` | 当前 `JETSON_LINK_CAN` 0/1 |
| Jetson 任务 | `APP/freertos/rtos_tasks.c` | `vJetsonTask`，20 ms 周期 |
| 应用层解析 | `APP/arbiter/arbiter.c` | `Arbiter_ParseJetsonCmd()` |
| CAN 传输 | `APP/jetson_can/jetson_can.c` | `JetsonLink_Deliver8()` |
| RS232 传输 | `APP/usart3/usart3.c` | V3 + 0xA5 服务帧 |
| 以太网 PHY | `APP/lan8720/lan8720.c` | 硬件初始化 |
| LwIP 配置 | `LwIP/lwip_app/lwip_comm/` | IP、init、周期轮询 |
| HTTP 演示 | `GUI_APP/eth_app.c` | 演示入口（Legacy GUI） |
| Web 服务 | `LwIP/lwip_app/web_server_demo/httpd.c` | HTTP Server |
| CGI 示例 | `LwIP/lwip_app/web_server_demo/httpd_cgi_ssi.c` | LED CGI，可仿造 motion.cgi |

**待新建**

| 模块 | 建议路径 |
|------|----------|
| 以太网 Jetson 链路 | `APP/jetson_eth/jetson_eth.c` |
| 网络任务（可选） | `APP/freertos/rtos_tasks.c` -> `vNetTask` |
| 运动控制网页 | `LwIP/lwip_app/web_server_demo/makefsdata/fs/motion.shtml` |

---

## 9. 常见问题

**Q：网线控制和 CAN 控制能同时连吗？**  
A：物理上可以，但同一时刻应只有 **一条链路写 Arbiter 心跳**，否则 seq 冲突。建议网线做监控、CAN 做主控，或编译期切换。

**Q：网页控制和 Jetson 自动导航会抢控制权吗？**  
A：会。需定义优先级（例如：心跳丢失则停车；网页仅 MANUAL 模式可用；或显式「接管」开关）。

**Q：MQTT 还要做吗？**  
A：不是第一阶段必需。UDP 透传 V3 完成并联调后，若领导要求多机订阅再上 MQTT（Broker 放 Jetson）。

**Q：ICMP / ping 和「测延时」是一回事吗？**  
A：不是。ping 测网络连通；应用层 V3 RTT 才是控制延时。两个都要写进报告。

**Q：为什么网页不适合 50 Hz 闭环？**  
A：HTTP 请求-响应开销大，浏览器轮询有抖动。实时闭环用 **UDP 网线** 或 **CAN**；网页适合点动、参数设置、状态查看。

**Q：Windows 能 ping 通 F407 吗？**  
A：取决于接线。若 Windows **只有 USB 串口**、F407 网线只接 Jetson，则 **Windows 不能 ping**（正常）。只有 PC 网线也接入同一二层网络时才可以。

**Q：Jetson `systemctl status ssh` 显示 active，但 SSH 仍超时？**  
A：多半是 **路由/网卡问题**，不是 SSH 服务坏了。常见原因：`eno1` 上挂了 `192.168.1.x` 但没插网线，把局域网流量劫持到死网口。见下文 **§11**。

**Q：有线和 WiFi 能同时用吗？**  
A：**可以**，推荐 **不同网段分工**（WiFi 做 SSH/开发，有线做设备直连），不要两块网卡配同一网段且其中一块没插线。

---

## 10. 总结

| 功能 | 物理介质 | 传输协议 | 应用层 | 工程状态 |
|------|----------|----------|--------|----------|
| 网线直连 | RJ45 + LAN8720 | **UDP**（主）+ ICMP（ping） | **BLOB v2**（推荐）/ V3 24B | **ping 已通**；UDP 控制待 `jetson_eth` |
| RS232 直连 | USART2 PA2/PA3 | BLOB v2 / V3 帧 | BLOB / V3 | 已实现；与以太网 **不可同时**（PA2） |
| 网页控制 | 网线->路由器->WiFi | **HTTP + CGI** | V3 24B | 有演示，待改运动页 |
| WiFi 免网线 | WiFi 模组（后续） | AT 透传 / 模组 Web | V3 24B | 未实现 |

**一句话**：以太网 **ping 已通**，下一步按 **§14** 接 **UDP + BLOB v2**；RS232 与以太网因 **PA2** 冲突须 **编译期二选一**。

---

## 11. 联调实录：实际接线、ping 与 Jetson 双网卡（2026-06）

本节汇总实验室真实接线、已踩坑与推荐配置，便于后续复现。

### 11.1 当前实验室接线（实测）

```text
  Windows (192.168.1.160) 有线 ──┐
                                 ├── 路由器 (192.168.1.1)
  Jetson WiFi (192.168.1.249) ───┘   网卡 wlP1p1s0

  Jetson 板载有线 eno1 (192.168.10.200) ──网线── F407 RJ45 (192.168.10.30)
         ↑ 与 MCU 点对点/直连，不经过路由器

  Windows ──USB 串口── F407          （仅调试打印、烧录，无以太网）
```

| 路径 | 能否 ping `192.168.10.30` | 说明 |
|------|---------------------------|------|
| Windows → F407 | **不能** | PC 与 MCU 无网线，属正常 |
| Jetson WiFi → F407 | **不能** | 不同网段，且不经过 eno1 |
| Jetson **eno1** → F407 | **可以** | `ping -I 192.168.10.200 192.168.10.30` |

### 11.2 Jetson 网卡对照（`ip link show`）

| 接口名 | 类型 | 用途 |
|--------|------|------|
| **`eno1`** | 板载有线 | 接 F407 / 相机 / 机械臂网线 |
| **`wlP1p1s0`** | WiFi | SSH、开发、上网（如 `192.168.1.249`） |
| `enxfacaff2c7aec` | USB 网卡 | 未插线时 `NO-CARRIER`，可忽略 |

> Jetson 上**没有 `eth0`**，有线口一般是 **`eno1`**。

### 11.3 F407 侧：串口应看到什么

成功启动以太网时（`ETH_LWIP_ENABLE=1`）：

```text
[ETH] Init lwIP (plug cable before power-on if possible)
[ETH] mem1 used 10% before lwIP
[ETH] lwIP OK  MCU IP 192.168.10.30  mask 255.255.255.0  gw 192.168.10.200
[ETH] mem1=58% mem2=3% after lwIP
[ETH] PHY link UP, speed code=...        ← 网线物理连通
[ETH] MAC 02:00:00:xx:xx:xx
```

| 串口现象 | 含义 | 处理 |
|----------|------|------|
| `lwIP init failed code=1` | 片内 mem1 不够 | 已修复：lwIP 池改 **SRAMEX** |
| `lwIP init failed code=2` | PHY/网线 | 插好网线再上电或重试 |
| `PHY link DOWN` | 链路未建立 | 查网线、Jetson `eno1` 是否 up |
| `PHY link UP` 但 ping 不通 | Jetson 路由/源地址 | 见 §11.5 |

**引脚注意**：`PA2` 同时用于 **USART2（Jetson RS232 TX）** 与 **ETH_MDIO**。以太网 init 在 `USART3_Init()` **之后**执行，启用以太网时 **RS232 发 TX 可能不可用**（测 ping 无影响）。

### 11.4 常见 ping 错误

| 现象 | 原因 |
|------|------|
| `Destination Host Unreachable` | ARP 失败，局域网找不到 `192.168.1.30` |
| Windows「无法访问目标主机」 | PC 未与 F407 网线同网 |
| `ping -c 192.168.1.1` 报 invalid argument | **写法错误**：`-c` 后是次数，不是 IP |
| ping 显示 `from 192.168.1.249` 但走 `eno1` | **eno1 未配上 IPv4**，源地址借用 WiFi |
| `ip addr show eno1` 无 `inet` | NetworkManager 冲掉手动 IP，或 `add` 未成功 |

**正确 ping 网关示例**：

```bash
ping -c 3 192.168.1.1
```

### 11.5 Jetson 通过 eno1 ping F407（临时，同网段 192.168.1.x）

仅用于 **F407 固件仍为 `192.168.1.30`** 的当前阶段：

```bash
# 可选：避免 NM 干扰
sudo nmcli device set eno1 managed no

sudo ip addr flush dev eno1
sudo ip addr add 192.168.1.200/24 dev eno1
sudo ip link set eno1 up

# 确认必须有 inet
ip -4 addr show eno1

# 主机路由 + 指定源地址（避免与 WiFi 抢路由）
sudo ip route add 192.168.1.30/32 dev eno1 src 192.168.1.200
ping -I 192.168.1.200 -c 4 192.168.1.30

# 或 ARP 探测
sudo arping -I eno1 -c 3 192.168.1.30
```

测完后若只需 WiFi SSH，建议删掉 eno1 上冲突 IP：

```bash
sudo ip addr del 192.168.1.200/24 dev eno1
sudo nmcli device set eno1 managed yes
```

### 11.6 SSH 超时：服务正常但连不上

`systemctl status ssh` 为 **active (running)**、`listening on 0.0.0.0:22` 只说明 **SSH 进程正常**。

`Connection timed out` = **网络层到不了 Jetson**，常见根因：

**`eno1` 配置了 `192.168.1.200` 但没插网线（NO-CARRIER）**，路由表出现：

```text
192.168.1.0/24 dev eno1  src 192.168.1.200  linkdown
```

则访问 `192.168.1.x`（含 SSH 回包、ping 网关）会优先走 **死网口**，表现为：

- 远程 SSH 超时
- `ping 192.168.1.1` 从 `192.168.1.200` 发出失败
- 强制 `ping -I wlP1p1s0 192.168.1.1` 则正常

**临时修复（只用 WiFi 时）**：

```bash
sudo ip addr del 192.168.1.200/24 dev eno1
ssh rxp@192.168.1.249    # 从 Windows，走 WiFi 地址
```

**不要用** `ssh rxp@192.168.1.200`，除非 eno1 已插网线且链路 UP。

重启 SSH 服务（一般不必，除非确认服务挂了）：

```bash
sudo systemctl restart ssh
# 或
sudo systemctl restart sshd
```

### 11.7 推荐：WiFi + 有线长期共存（分网段）

**有线和 WiFi 可以同时存在**，不是二选一。推荐分工：

| 接口 | IP（推荐） | 用途 | 网关 |
|------|------------|------|------|
| **WiFi** `wlP1p1s0` | `192.168.1.249`（DHCP） | SSH、开发、上网 | `192.168.1.1` |
| **有线** `eno1` | `192.168.10.200/24`（静态，插线生效） | F407 / 相机 / 机械臂直连 | **不设**（`never-default`） |

```text
WiFi:   192.168.1.249/24  →  路由器  →  ssh rxp@192.168.1.249
有线:   192.168.10.200/24 →  仅直连设备（F407 建议改为 192.168.10.30）
```

**永久配置示例（NetworkManager，在 Jetson 执行）**：

```bash
# 1. 去掉冲突的旧 IP
sudo ip addr del 192.168.1.200/24 dev eno1 2>/dev/null || true

# 2. 删除旧的通用有线配置（如 "Profile 1"，按 nmcli connection show 为准）
sudo nmcli connection delete "Profile 1" 2>/dev/null || true

# 3. eno1 专用：独立网段、无默认网关、插线自动连接
sudo nmcli connection add \
  type ethernet \
  con-name "eno1-robot" \
  ifname eno1 \
  ipv4.method manual \
  ipv4.addresses 192.168.10.200/24 \
  ipv4.never-default yes \
  ipv4.gateway "" \
  connection.autoconnect yes \
  connection.autoconnect-priority 10

# 4. 验证
ip route show
ping -c 3 192.168.1.1
```

采用 `192.168.10.0/24` 后，F407 固件默认 IP 为 **`192.168.10.30`**（`lwip_comm_default_ip_set()`，v1.1 起）。

**验证（插网线、烧录新固件后）**：

```bash
ip -4 addr show eno1                    # 192.168.10.200
ping -I 192.168.10.200 -c 4 192.168.10.30
ping -c 3 192.168.1.1                   # WiFi 仍应正常
```

### 11.8 联调检查清单

- [ ] F407 串口：`[ETH] lwIP OK` + `PHY link UP`
- [ ] Jetson `eno1`：`LOWER_UP`（插线后）
- [ ] 勿在 **未插线** 的 eno1 上保留 `192.168.1.x` 地址
- [ ] ping F407 走 **eno1**，并指定源 IP 或使用 `192.168.10.x` 分网段
- [ ] SSH 始终用 **WiFi**：`ssh rxp@192.168.1.249`
- [ ] Windows 仅串口时 **不要** 以 ping F407 作为判据

---

## 12. 固件侧以太网代码逻辑（网线直连）

本节记录 **当前工程已实现** 的以太网软件路径，便于后续恢复 ping / 接 UDP 时对照，与 [§11 联调实录](#11-联调实录实际接线ping-与-jetson-双网卡2026-06) 互补。

### 12.1 编译开关与 IP 来源

| 宏 | 文件 | 默认值 | 作用 |
|----|------|--------|------|
| `ETH_LWIP_ENABLE` | `APP/freertos/app_boot.h` | `1` | `1`=仲裁模式启动 lwIP；`0`=完全关闭以太网，**不影响 CAN/RS232** |
| `ETH_RX_POLL_ONLY` | `APP/freertos/app_boot.h` | `1` | `1`=Net 任务轮询收包，**关闭 ETH DMA 中断**（避免 ISR 内调 lwIP）；`0`=走 `ETH_IRQHandler`（裸机演示方式，FreeRTOS 下不推荐） |
| `NET_TASK_STACK_SIZE` | `APP/freertos/rtos_tasks.h` | `512` | Net 任务栈（words）；gratuitous ARP 调用链较深，原 256 易栈紧张 |
| `NET_TASK_CYCLE_MS` | `APP/freertos/rtos_tasks.h` | `5` | Net 任务周期，驱动 `App_EthPoll()` |

**IP / MAC 不是 Flash/EEPROM 配置**，在 `LwIP/lwip_app/lwip_comm/lwip_comm.c` 的 `lwip_comm_default_ip_set()` 写死：

| 项 | 值 |
|----|-----|
| F407 IP | `192.168.10.30/24` |
| 上位机 / `remoteip` | `192.168.10.201` |
| MAC | `02:00:00:` + STM32 UID 低 24 bit（例：`02:00:00:1D:00:3B`） |

### 12.2 启动顺序（`App_MotionHwInit()`）

```text
JetsonCAN_Init() / USART3_Init()     ← Jetson 链路（RS232 实际用 USART2 PA2/PA3）
GPS / 测距 / CAN1 / Arbiter ...
#if ETH_LWIP_ENABLE
  App_EthInit()                      ← 必须最后：PA2 改作 ETH_MDIO
#endif
```

> 模块名 `USART3_*` 是历史命名，Jetson RS232 物理接 **USART2**（`usart3.c` 内 `USART_Init(USART2, ...)`）。

### 12.3 初始化调用链

```text
App_EthInit()                          [app_boot.c]
  TIM3_Init(999, 839)                  ← 10ms 节拍，ISR 内 lwip_localtime += 10
  lwip_comm_init()                     [lwip_comm.c] 最多重试 5 次
    ETH_Mem_Malloc()                   ← DMA 描述符 + Rx/Tx buffer（mem1）
    lwip_comm_mem_malloc()             ← memp/heap（SRAMEX）
    LAN8720_Init()                     ← RMII GPIO、PHY 复位、ETH_MACDMA_Config
    lwip_init()
    lwip_comm_default_ip_set()
    netif_add(..., ethernetif_init, ethernet_input)
    netif_set_up()
  打印 link / MAC / 对端 IP
  ETH_RecoverRxDma(); DMARPDR=0
  lwip_comm_gratuitous_arp()           ← 主动 ARP 公告
  打印 tx_ok=...
  RTOS_AppStart() -> vNetTask
```

**`lwip_comm_init()` 返回值**

| code | 含义 |
|------|------|
| 0 | 成功 |
| 1 | mem1/SRAMEX 分配失败 |
| 2 | LAN8720 / PHY 失败 |
| 3 | `netif_add` 失败 |

### 12.4 FreeRTOS 网络任务

```text
vNetTask (prio=2, 周期 5ms)            [rtos_tasks.c]
  App_EthPoll()
  每 1000 周期（约 5s）打印：
    [ETH] link=UP rx=... tx_ok=... rbus=... desc_cpu=...
```

### 12.5 收包路径（轮询模式）

```text
App_EthPoll()                          [app_boot.c]
  while (有 Rx 帧)
    __disable_irq() / lwip_pkt_handle() / __enable_irq()   ← 每帧短临界区
  __disable_irq()
  ETH_RecoverRxDma()
  lwip_periodic_handle()
  __enable_irq()
```

> **勿**在 `while` 收包循环外长时间 `__disable_irq()`，否则会丢 **USART3 测距** RX 字节（见 **§15.4**）。

**`ETH_GetRxPktSize`**（`stm32f4x7_eth.c`）：当前 Rx 描述符 `OWN=0` 且 `LS=1` 且无 `ES` 时返回帧长，否则 0。

**`ETH_Rx_Packet`**（`lan8720.c`）：取 buffer → 推进 `DMARxDescToGet` → 归还描述符 `OWN` → 若 `RBUS` 则恢复 DMA。

### 12.6 发包路径

```text
lwip_comm_gratuitous_arp()
  etharp_gratuitous → etharp_request → low_level_output
    ETH_GetCurrentTxBuffer + memcpy
    ETH_Tx_Packet(len)                 ← 成功则 s_eth_tx_ok++
```

ICMP ping 回复、后续 UDP 发送均走同一 `low_level_output` → `ETH_Tx_Packet`。

### 12.7 硬件与驱动要点

| 项目 | 说明 |
|------|------|
| PHY | LAN8720，RMII，`APP/lan8720/lan8720.c` |
| 引脚 | MDIO PA2、MDC PC1、REF_CLK PA1、CRS_DV PA7、RXD0/1 PC4/5、TX_EN PG11、TXD0/1 PG13/14、RST PD3 |
| ETH IRQ | 优先级 6（≥ FreeRTOS syscall 5）；`ETH_RX_POLL_ONLY=1` 时不使能 RX 中断 |
| 链路 UP | `LAN8720_LinkUp()` 读 PHY BMSR bit2，**只表示 PHY 协商成功，不等于 L3 ping 通** |

### 12.8 串口诊断字段含义

```text
[ETH] gratuitous ARP sent, tx_ok=2
[ETH] link=UP rx=0 tx_ok=2 rbus=0 desc_cpu=0 (rx=0=no L2 frames)
```

| 字段 | 含义 | 典型解读 |
|------|------|----------|
| `link` | PHY BMSR 链路 | UP = 网线对端有协商 |
| `rx` | Net 任务累计收帧数 | **0 = DMA 从未收到完整以太网帧** |
| `tx_ok` | `ETH_Tx_Packet` 成功次数 | ≥1 表示 MCU **提交发送** 成功（含 gratuitous ARP） |
| `rbus` | DMASR Receive Buffer Unavailable | 1 = RX DMA 曾饿死，需 `ETH_RecoverRxDma` |
| `desc_cpu` | OWN=0 的 Rx 描述符个数 | 0 = 全在 DMA 手里等包；>0 且 rx=0 可能是轮询/描述符异常 |

### 12.9 应用层接入点（待实现 → 见 §14）

以太网 **UDP/BLOB 控制** 待新建 `APP/jetson_eth/`（见 **§14.4**）。当前 `vJetsonTask` 仍只走 `BlobRs232_*` / `JetsonCAN_*`；`vNetTask` 仅负责 LwIP 轮询与诊断。

### 12.10 与 RS232 的引脚冲突（重要）

| 引脚 | 以太网 | Jetson RS232 |
|------|--------|--------------|
| **PA2** | ETH_MDIO | **USART2_TX**（MCU → Jetson） |
| PA3 | — | USART2_RX（Jetson → MCU） |

`ETH_LWIP_ENABLE=1` 时，`App_EthInit()` 在 `USART3_Init()` **之后**执行，**PA2 被 MDIO 占用**，串口日志会提示：

```text
[ETH] PA2=ETH_MDIO after init; Jetson RS232 TX(PA2) disabled while ETH on
```

**专搞 RS232 联调时**：在 `app_boot.h` 设 **`ETH_LWIP_ENABLE=0`**，重新编译烧录，PA2 恢复为 USART2_TX。

### 12.11 关键源码索引（以太网）

| 模块 | 路径 |
|------|------|
| 开关 / Init / Poll | `APP/freertos/app_boot.h`, `app_boot.c` |
| Net 任务 | `APP/freertos/rtos_tasks.c` → `vNetTask` |
| PHY + DMA + 诊断 | `APP/lan8720/lan8720.c`, `lan8720.h` |
| lwIP 入口 | `LwIP/lwip_app/lwip_comm/lwip_comm.c` |
| netif 底层 | `LwIP/lwip-1.4.1/src/netif/ethernetif.c` |
| lwIP 时间基准 | `APP/time/time.c` → `TIM3_IRQHandler` |
| Legacy HTTP 演示 | `GUI_APP/eth_app.c`（`UI_TEST_MODE=0` 图标菜单进入） |

---

## 13. 联调疑难与诊断（2026-06 实测）

### 13.1 已确认现象

实验室 **Jetson eno1 ↔ F407 RJ45 单网线直连**，配置 `192.168.10.200` / `192.168.10.30`：

| 侧 | 现象 |
|----|------|
| F407 串口 | `[ETH] lwIP OK`、`PHY link UP`、`tx_ok=2`、**`rx=0` 持续不变** |
| Jetson | `ping -I 192.168.10.200 192.168.10.30` → `Destination Host Unreachable` |
| Jetson | `ip neigh show dev eno1` → `192.168.10.30 FAILED` |

**结论**：MCU **能发**（DMA TX 成功），**收不到任何以太网帧**（含 Jetson ARP 广播）。问题在 **二层帧未双向互通**，不是 IP 写错或 lwIP 未初始化。

### 13.2 Link UP ≠ 数据通

两端均可 `PHY link UP` / `eno1 LOWER_UP`，但仍可能 ping 不通。常见原因：

- ** damaged 网线**：100M 链路可能只靠一对线协商 UP，数据对不通
- **Jetson 源地址/路由**：须 `ping -I 192.168.10.200`；WiFi 与 eno1 分网段（见 §11.7）
- **F407 RX 硬件通路**（RMII RX、变压器）：TX 正常但 RX 无帧时 `rx=0`

### 13.3 Jetson 侧排查命令

```bash
# 安装抓包（若无 tcpdump）
sudo apt install -y tcpdump iputils-arping

# 确认 eno1 地址与路由
ip -br addr show eno1
ip route get 192.168.10.30

# MCU 复位后看 Jetson 是否收到 gratuitous ARP
ip -s link show eno1          # 对比 RX packets 是否增加
ip neigh show dev eno1        # 期望出现 02:00:00:1d:00:3b

# 抓包 + ping
sudo tcpdump -i eno1 -n -e arp or icmp
ping -I 192.168.10.200 -c 4 192.168.10.30

# 拔线验端口：拔 F407 端网线，eno1 应 DOWN
watch -n1 'ip link show eno1 | grep state'

# 绕过 ARP 测 ICMP
sudo ip neigh replace 192.168.10.30 lladdr 02:00:00:1d:00:3b dev eno1 nud permanent
ping -I 192.168.10.200 -c 4 192.168.10.30
```

### 13.4 判定表

| tx_ok | rx | Jetson RX 计数 | 倾向 |
|-------|-----|----------------|------|
| ≥1 | 0 | 不增加 | **物理/L2**：线、口、或 F407 RX |
| ≥1 | 0 | 增加 | Jetson 能收 MCU 包；查 Jetson→F407 方向 |
| 0 | 0 | — | MCU TX 未发出，查 DMA/PHY TX |
| ≥1 | >0 | — | L2 已通，再查 ICMP/防火墙 |

### 13.5 当前决策（2026-06-17 更新）

- **L3 ping 已通过**（PC `192.168.1.106` ↔ F407 `192.168.1.122`，或 `192.168.10.x` 网段）；§13.1~13.4 保留作历史排查参考。
- **下一步**：按 **[§14](#14-网线-udp-发消息完整实现流程ping-通过后)** 实现 **UDP 透传 BLOB v2**，上位机收 `0x02/0x03` 上行。
- **链路选择**：走以太网时 `ETH_LWIP_ENABLE=1` 且 **关闭 RS232 控制**（PA2=MDIO）；走 RS232 时 `ETH_LWIP_ENABLE=0`。

### 13.6 RS232 BLOB v2 联调问题（与以太网独立）

以太网 ping 与 **Jetson↔F407 RS232** 是两条链路。RS232 阶段典型问题已整理至（**v2.0-draft.5.8**）：

**[Jetson_BLOB协议_v2.md §C.9 RS232 BLOB 联调实录：问题与解决方案](./Jetson_BLOB协议_v2.md#c9-rs232-blob-联调实录问题与解决方案)**

| 阶段 | 现象摘要 | 文档章节 |
|------|----------|----------|
| 下行 L3 | `ab_idle=0` / 一直 DEGRADED | §C.9.3、**§C.9.9**（include 未编译 BLOB 收包） |
| 下行 L3 已通过 | F407 `ARB=NORMAL`，`[JETSON BLOB CMD]` | §C.9.14 阶段 A |
| 上行 L4 | topic 空、gateway「上行超时」；`listen-only` 能收 0x02/0x03 | **§C.9.10**（gateway 混流解析，**MCU 无需再改**） |

---

## 14. 网线 UDP 发消息：完整实现流程（ping 通过后）

本节是 **ping 已通之后** 的落地指南：经网线向上位机（PC 或 Jetson）收发控制与状态消息。**MCU 与上位机分工明确**，按阶段验收。

### 14.1 目标与原则

| 项目 | 约定 |
|------|------|
| **物理** | RJ45 网线直连 PC 或 Jetson `eno1` |
| **网络层** | IPv4 静态 IP，已 `ping` 通 |
| **传输层** | **UDP**（一帧一报文，低延迟） |
| **应用层** | **BLOB v2**（`0xAB` 线格式，与 RS232 相同 struct） |
| **仲裁** | 下行 `0x01` 仍走 `BlobPack_HandleDownlink()` → `Arbiter` → CAN1 |
| **与 RS232 关系** | **编译期二选一**；以太网模式下 PA2=MDIO，RS232 TX 不可用 |

> 旧版 **V3 24B（0xAA）** 仍可作为最小联调载荷，但综合工程默认 **BLOB v2**，上位机脚本应与 RS232 共用同一套 struct 定义（见 [Jetson_BLOB协议_v2.md](./Jetson_BLOB协议_v2.md)）。

### 14.2 端到端数据流

```text
┌─────────────────────────────────────────────────────────────────────────┐
│ 上位机（PC / Jetson）                                                    │
│  eth_link_test.py  或  eth_gateway（ROS2，后续）                          │
│    UDP:50001 发 BLOB 0x01 (23B)  ───────────────────────────────┐       │
│    UDP:50002 收 BLOB 0x02/0x03/...  <───────────────────────────┼───┐   │
└───────────────────────────────────────────────────────────────────┼───┼───┘
                                                                    │   │
                         网线 L2/L3（已 ping 通）                    │   │
                                                                    ▼   │
┌─────────────────────────────────────────────────────────────────────────┐
│ F407（综合实验程序）                                                     │
│  vNetTask (5ms)                                                         │
│    App_EthPoll() → lwIP 收 UDP → jetson_eth 回调入队                     │
│  vJetsonTask (20ms)                                                     │
│    出队 BLOB → BlobPack_HandleDownlink() → Arbiter → 底盘 CAN1          │
│    BlobPack_UplinkTick() → BlobEth_Send() → UDP 发往 remoteip:50002     │
└─────────────────────────────────────────────────────────────────────────┘
```

**单帧时序（20 ms 周期）**

```text
上位机                              F407
  |  UDP → :50001  [0xAB… MSG=0x01]  -->  recv 回调 → 环形队列
  |                                   |  vJetsonTask → Arbiter 解析控制
  |  <-- UDP ← :50002  [0xAB… 0x02]   |  BlobPack 组 0x02 运动摘要
  |  <-- UDP ← :50002  [0xAB… 0x03]   |  交替发 MCU 状态
```

### 14.3 IP 与端口（综合工程默认）

| 设备 | IP | 掩码 | 说明 |
|------|-----|------|------|
| **F407（MCU）** | **192.168.10.30** | 255.255.255.0 | `lwip_comm_default_ip_set()` |
| **上位机（PC）** | **192.168.10.201** | 255.255.255.0 | 网关留空；MCU `remoteip` 指向此地址 |
| F407 网关 | 无（0.0.0.0） | — | 网线直连不需网关 |

> 普中 52 例程对照 IP 为 `192.168.1.122` / `192.168.1.106`，与本综合工程不同。Jetson `eno1` 若用 `192.168.10.200`，须同步改 `lwip_comm.c` 中 `remoteip`。

#### UDP 端口

| 方向 | 本地绑定 | 对端地址 | 载荷 |
|------|----------|----------|------|
| 上位机 → F407 | 任意源端口 | `F407_IP:50001` | BLOB 线格式（例 0x01 = 23 B） |
| F407 → 上位机 | F407 任意源端口 | `上位机_IP:50002` | BLOB 0x02/0x03/… |
| 连通性 | ICMP | — | `ping F407_IP`（已完成） |

**防火墙**：Windows 须放行 **入站 UDP 50002**（或测试时临时关闭防火墙）。

### 14.4 MCU 侧：要做什么（按顺序）

#### 阶段 0 — 前置（你已完成）

- [x] `ETH_LWIP_ENABLE=1`，串口见 `[ETH] lwIP OK`、`PHY link UP`
- [x] 上位机 `ping F407_IP` 成功
- [x] 串口 `[ETH] link=UP rx>0`（有 ICMP/ARP 时 rx 会增长）

#### 阶段 1 — 编译与宏

| 宏 | 文件 | 以太网模式建议值 |
|----|------|------------------|
| `ETH_LWIP_ENABLE` | `APP/freertos/app_boot.h` | `1` |
| `JETSON_LINK_CAN` | `APP/jetson_can/jetson_can.h` | `0` |
| `JETSON_USE_BLOB_V2` | `APP/agv_blob/agv_blob_wire.h` | `1` |
| `JETSON_LINK_ETH`（**待新增**） | `jetson_can.h` 或 `jetson_eth.h` | `1` |

```text
#if JETSON_LINK_CAN
  → CAN2 链路（与以太网无关）
#elif JETSON_LINK_ETH
  → UDP BLOB（本节）
#else
  → RS232 BLOB（现有默认）
#endif
```

**注意**：`JETSON_LINK_ETH=1` 时不要再依赖 `USART2` 发 Jetson 数据；`BlobRs232_*` 分支改为 `BlobEth_*`。

#### 阶段 2 — 新建 `APP/jetson_eth/`

建议文件：`jetson_eth.c`、`jetson_eth.h`。

| 函数 | 职责 |
|------|------|
| `JetsonEth_Init()` | `udp_new()` → `udp_bind(IP_ADDR_ANY, 50001)` → `udp_recv(callback)`；在 `App_EthInit()` 成功之后调用 |
| `JetsonEth_UdpRxCallback()` | lwIP 回调：**不可**调 `printf`/阻塞；把 `p->payload` 拷入 **FreeRTOS 环形队列**（按完整 BLOB 线长） |
| `JetsonEth_GetFrame(blob_rx_frame_t *)` | 供 `vJetsonTask` 消费，接口对齐 `BlobRs232_GetFrame()` |
| `BlobEth_Send(msg_id, seq, payload, len)` | 组 9B 头 + payload → `udp_sendto(remoteip, 50002)` |

**实现要点**

1. **线程安全**：UDP 回调与 `vJetsonTask` 之间只用队列传递，不在回调里调 `BlobPack_HandleDownlink()`。
2. **remote IP**：读 `lwipdev.remoteip[]`（已在 `lwip_comm_default_ip_set()` 写好对端 IP）。
3. **内存**：`pbuf` 在回调里 `pbuf_free(p)`；发送用 `PBUF_RAM` 或栈缓冲 + `udp_sendto`。
4. **周期维护**：`vNetTask` 已有 `App_EthPoll()`，**无需**再开 ETH 中断收包。

#### 阶段 3 — 改 `agv_blob_pack.c`

将 `BlobRs232_Send()` 抽象为传输层函数指针，或：

```c
#if JETSON_LINK_ETH
  #define BlobTransport_Send  BlobEth_Send
#else
  #define BlobTransport_Send  BlobRs232_Send
#endif
```

`BlobPack_UplinkTick()`、`BlobPack_SendGps()` 等统一走 `BlobTransport_Send`，**业务逻辑零改动**。

#### 阶段 4 — 改 `vJetsonTask`（`rtos_tasks.c`）

```text
#if JETSON_LINK_ETH
  while (JetsonEth_GetFrame(&blob_frame))
    BlobPack_HandleDownlink(&blob_frame);
  BlobPack_UplinkTick(...);          // 内部已走 BlobEth_Send
#elif !JETSON_LINK_CAN && JETSON_USE_BLOB_V2
  while (BlobRs232_GetFrame(&blob_frame)) ...
#else
  ... RS232 V3 / CAN 分支
#endif
```

#### 阶段 5 — Keil 工程

- 把 `jetson_eth.c` 加入工程分组 `APP`
- 确认 `lwipopts.h` 中 `LWIP_UDP=1`（默认已开）

#### 阶段 6 — MCU 验收标准

| 级别 | 操作 | F407 串口期望 |
|------|------|---------------|
| L3 | 上位机发 1 帧 0x01 | `[JETSON BLOB CMD]`、`ARB=NORMAL`（与 RS232 相同） |
| L4 | 上位机只监听 50002 | 持续收到 0x02/0x03，`[ETH] rx` 递增 |
| L5 | 发控制改速度 | 底盘 CAN 有输出 / LCD 状态变化 |

### 14.5 上位机侧：要做什么（PC 先行，Jetson 后续）

#### 阶段 0 — 网络（你已完成）

**PC（默认）**

```cmd
ipconfig
ping 192.168.10.30
```

#### 阶段 1 — 最小 UDP 回环测试（MCU 未改代码前也可做）

用 **netcat** 或 Python 确认端口可达（需 F407 侧先 bind 50001，或暂用 52 例程 UDP 演示端口 8089 做 LwIP 冒烟）。

#### 阶段 2 — BLOB 0x01 下行 + 0x02 上行（核心）

在仓库 `tools/` 下建议新增 `eth_link_test.py`（逻辑对齐 `jetson_rs232_link_test.py --blob-v2`）：

```python
#!/usr/bin/env python3
"""F407 以太网 BLOB v2 联调：发 0x01，收 0x02/0x03"""
import socket, struct, time

F407_IP = "192.168.10.30"
PORT_DOWN = 50001
PORT_UP = 50002
MAGIC, VER = 0xAB, 0x01

def build_control(seq: int, v_mm_s: int = 0) -> bytes:
    """agv_control_t 14B + 9B 头 = 23B，字段定义见 Jetson_BLOB协议_v2.md"""
    payload = struct.pack(">hHhHBBBB",
        v_mm_s, 0, 0, 0,          # v, steer, omega, accel（示例填 0）
        1, 1, 0, 0, 0)             # mode_req, motion_model, clear_error, flags, reserved
    hdr = struct.pack(">BBBBHBBB", MAGIC, VER, 0x01, seq & 0xFF, 14, 0, 1, 0)
    return hdr + payload

def parse_blob(data: bytes):
    if len(data) < 9 or data[0] != MAGIC:
        return None
    msg_id, seq = data[2], data[3]
    plen = (data[4] << 8) | data[5]
    if len(data) < 9 + plen:
        return None
    return msg_id, seq, data[9:9+plen]

def main():
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("0.0.0.0", PORT_UP))
    rx.settimeout(2.0)
    seq = 0
    print(f"TX -> {F407_IP}:{PORT_DOWN}  RX bind :{PORT_UP}")
    while True:
        wire = build_control(seq)
        t0 = time.time()
        tx.sendto(wire, (F407_IP, PORT_DOWN))
        try:
            data, addr = rx.recvfrom(2048)
            dt = (time.time() - t0) * 1000
            p = parse_blob(data)
            print(f"seq={seq} recv {len(data)}B from {addr} msg=0x{p[0]:02X} rtt={dt:.1f}ms" if p else f"bad frame {data[:16].hex()}")
        except socket.timeout:
            print(f"seq={seq} uplink timeout")
        seq = (seq + 1) & 0xFF
        time.sleep(0.02)   # 50 Hz，与 JETSON_TASK_CYCLE_MS=20 对齐

if __name__ == "__main__":
    main()
```

**PC 运行**

```cmd
python tools\eth_link_test.py
```

**Jetson 运行**（方案 B，改脚本内 `F407_IP`）

```bash
python3 tools/eth_link_test.py
```

#### 阶段 3 — 抓包定责（可选）

```bash
# Jetson
sudo tcpdump -i eno1 -n udp port 50001 or port 50002

# Windows（需 Npcap）
# Wireshark 过滤器：udp.port == 50001 || udp.port == 50002
```

| 现象 | 倾向 |
|------|------|
| tcpdump 见 50001，无 50002 | MCU 未实现发送或 `remoteip` 错误 |
| 50002 有包但脚本解析失败 | 载荷长度/字节序与 BLOB 文档不一致 |
| 双向都有包，无 `[JETSON BLOB CMD]` | MCU 未接 `jetson_eth` 或队列未喂给 `BlobPack` |

#### 阶段 4 — ROS2 `eth_gateway`（后续，对齐 RS232）

参考 `rs232_gateway` 架构，后续在 Jetson 侧实现：

```text
ROS topic (/cmd_vel 等)
    → eth_gateway 组 BLOB 0x01 → UDP :50001
    ← UDP :50002 解析 0x02/0x03 → 发布 /jetson_eth/motion 等
```

PC 联调阶段 **不必等 ROS**，先用 `eth_link_test.py` 打通即可。

### 14.6 分工总表

| 步骤 | MCU（F407） | 上位机（PC / Jetson） |
|------|-------------|------------------------|
| 1 | `ETH_LWIP_ENABLE=1`，确认 ping | 设静态 IP，`ping F407` |
| 2 | 新建 `jetson_eth`，bind **50001** | bind **50002**，防火墙放行 |
| 3 | UDP 收 → 队列 → `BlobPack_HandleDownlink` | `eth_link_test.py` 发 **0x01** 50Hz |
| 4 | `BlobEth_Send` 发 0x02/0x03 到 `remoteip:50002` | 脚本打印 `msg=0x02/0x03`、RTT |
| 5 | 串口确认 `ARB=NORMAL` | 对比 RS232 同指令的行为 |
| 6 | Keil 合入工程、烧录 | 保存日志，写延时对比表 |
| 7（可选） | `remoteip` 与实际上位机 IP 一致 | 部署 `eth_gateway` 接 ROS2 |

### 14.7 与 RS232 / CAN 切换

| 场景 | `ETH_LWIP_ENABLE` | `JETSON_LINK_ETH` | Jetson 物理连接 |
|------|-------------------|-------------------|-----------------|
| 以太网控制 | `1` | `1` | 网线 |
| RS232 BLOB | `0` | `0` | USB-TTL / USART2 |
| CAN 控制 | `0` | `0` | CAN2 |

**不要**在同一固件里同时打开以太网 + RS232 控制：PA2 冲突，且双链路会抢 Arbiter 心跳。

### 14.8 常见问题（UDP 阶段）

**Q：ping 通但 UDP 无响应？**  
A：查 F407 是否已 `udp_bind(50001)`；Windows 防火墙是否拦 **入站 50002**；`remoteip` 是否等于 PC/Jetson 实际 IP。

**Q：能收 0x02 但 `ARB=DEGRADED`？**  
A：与 RS232 相同——须 **持续 50Hz 发 0x01** 作心跳；见 [Jetson_BLOB协议_v2.md §C.9](./Jetson_BLOB协议_v2.md)。

**Q：是否还要发 0xA5 时间同步？**  
A：以太网阶段 **可选**；BLOB 控制不依赖 0xA5。需要时可另开 UDP **50003** 透传 11B 服务帧。

**Q：载荷用 V3 还是 BLOB？**  
A：综合工程默认 **BLOB v2**；仅做最小演示时可发 24B `0xAA`，但须关闭 `JETSON_USE_BLOB_V2` 并走 V3 分支（不推荐与当前固件混用）。

---

## 15. 资源冲突与解决办法（必读）

本节汇总 **实验室已踩坑** 与 **全工程代码审计** 结论：引脚复用、串口/链路互斥、中断与 timing、上位机网络等。新增功能前先查此表。

### 15.1 总览：推荐编译组合

| 场景 | `ETH_LWIP_ENABLE` | `JETSON_LINK_CAN` | Jetson 物理链路 | 测距 LCD |
|------|-------------------|-------------------|-----------------|----------|
| **以太网 UDP 控制（当前）** | `1` | `0` | 网线 UDP | 可用（须 §15.4 修复已合入） |
| **RS232 BLOB 联调** | **`0`** | `0` | PA2/PA3 RS232 | 可用 |
| **CAN 控制** | `0` | `1` | CAN2 PB5/PB6 | 可用 |
| ~~以太网+RS232 同时~~ | — | — | **禁止** | PA2 冲突 |

控制链路 **编译期三选一**：CAN / RS232 / 以太网（`JETSON_LINK_ETH` 随 `ETH_LWIP_ENABLE` 自动为 1）。

### 15.2 引脚 / 外设冲突

| ID | 冲突资源 | 涉及功能 A | 涉及功能 B | 现象 | 解决办法 | 代码/文档 |
|:--:|----------|------------|------------|------|----------|-----------|
| **P1** | **PA2** | `ETH_MDIO`（LAN8720） | `USART2_TX`（Jetson RS232 发） | 开以太网后 MCU **发不出** RS232/BLOB 上行；PA2 被 MDIO 占 | **`ETH_LWIP_ENABLE=1` 时勿用 RS232 控制**；RS232 联调设 `ETH_LWIP_ENABLE=0` | `lan8720.c`、`app_boot.c` §12.10 |
| **P2** | PA3 | — | `USART2_RX` | 以太网模式下 PA3 仍可收字节，但 **无 TX 应答**；易误导调试 | 以太网模式 **跳过 `USART3_Init()`**（不 init USART2）；控制走 UDP | `app_boot.c`（v1.4+） |
| **P3** | 命名 | `USART3_*` 函数名 | 实际硬件 **USART2** | 查文档/接线易搞错 | 记：**Jetson=USART2 PA2/PA3**；**测距=USART3 PB10/PB11** | `usart3.c`、`distance_sensor.c` |
| **P4** | PB10/PB11 | **USART3** 测距 9600 | 以太网 RMII | **无引脚冲突**；RMII 用 PA/PC/PG | 测距与以太网 **可同时开** | `distance_sensor.c`、`lan8720.c` |
| **P8** | **E08 供电** | 四路测距转接模组 | F407 5V/3.3V 负载 | **`rx=0` / 偶发 `rx=12` 后停**；与以太网/Jetson **无关** | **独立稳定 5V** 供 E08；**GND 与 F407 共地**；勿仅靠杜邦线从开发板 5V 拖大电流 | 见 **§15.11** |
| **P5** | PA11/12 | CAN1 底盘 | — | 与以太网无重叠 | 可同时用 | `can.c` |
| **P6** | PC6/7 | USART6 GPS | — | 与以太网无重叠；GPS 启动时会 **9600→115200 切波特率** | 勿接其他设备到 USART6；等 `[GPS][CFG] done` 后再依赖 GPS | `gps.c` |
| **P7** | PA2/PA3 | `RS485`/`rs232.c` 演示 | Jetson `usart3.c` | 若 `RS485_ENABLE` 且同时链入，**重复 `USART2_IRQHandler`** | 综合工程 **勿定义 `RS485_ENABLE`**；演示 RS485 勿与 Jetson 同固件 | `rs485.c` `#if RS485_ENABLE` |

### 15.3 控制链路 / 应用层冲突

| ID | 冲突 | 现象 | 解决办法 |
|:--:|------|------|----------|
| **L1** | 以太网 + RS232 **同时**写 Arbiter | 双心跳、seq 乱、安全态跳变 | 编译期只开一条：`JETSON_LINK_ETH` 或 RS232 |
| **L2** | 以太网 + CAN **同时**主控 | 同上 | 默认 CAN 底盘保留；Jetson 控制三选一 |
| **L3** | 上位机未发 **0x01 心跳** | `ARB=DEGRADED/RECOVERING`；曾误停发 **0x04 测距 BLOB** | 上位机 **50Hz 发 0x01**；MCU 已在心跳丢失时仍发 0x04（v1.4+） |
| **L4** | Jetson **多进程**抢串口 | `TimeSync` 有、BLOB 无；`ore` 风暴 | `pkill rs232_gateway`；以太网模式无此问题 |
| **L5** | `BlobRs232_Send` + `JetsonEth_Send` 同时 | 理论上双发；当前 **宏互斥** 只链一路 | 保持 `BlobLink_Send` 单一定义 |

### 15.4 中断 / 时序冲突（MCU）

| ID | 冲突点 | 现象 | 解决办法 | 状态 |
|:--:|--------|------|----------|:--:|
| **T1** | `App_EthPoll()` **长时间** `__disable_irq()` | 测距 `valid=0`、`[DS] RX timeout`、65535 | **每帧** eth 收包单独关中断；已修复 | ✅ 已修 |
| **T2** | `JetsonEth_Send()` 内 `__disable_irq()` | 极短；偶发丢测距字节 | 发送前临界区尽量短；测距 ISR 优先级 **1** 高于 Net 任务 | 已知/可接受 |
| **T3** | **TIM4 20ms** 测距触发 vs **vNetTask 5ms** eth 轮询 | 触发本身在 ISR；收包靠 USART3 ISR | T1 修复后正常 | ✅ |
| **T4** | **TIM3 10ms** lwIP 时钟 vs **USART3** 测距 | 同抢占优先级 **1**，可能互堵数 µs | 可接受；若仍丢字节可将测距升到 **0** | 低风险 |
| **T5** | **TIM2 100ms** 拍照 ISR（`KEY_Scan+delay_ms`） | 与 `KeyTask` 冲突、卡死 | 仲裁模式 **`TIM2_Cmd(DISABLE)`** | ✅ `app_boot.c` |
| **T6** | 任务内 **大量 `printf`**（`[JETSON BLOB CMD]`） | 占 USART1、拖慢任务，间接影响实时性 | 联调用 `RTOS_VERBOSE_*`；量产关日志 | 建议 |
| **T7** | `vJetsonTask` 20ms vs 测距整轮 **~280ms** | 无直接冲突；DistSnapshot Readers 用临界区 | 保持 `DistSnapshot_Read/Write` | ✅ |
| **T8** | FreeRTOS：`configMAX_SYSCALL_INTERRUPT_PRIORITY=5` | 优先级 **≥6** 的 ISR 不能调 FreeRTOS API | ETH IRQ=6 且 **轮询模式关闭 ETH IRQ**；USART2 DMA=6 | ✅ 设计如此 |

**测距触发说明（非高电平问题）**：PB10 平时 UART 空闲高；触发时切 GPIO **拉低 ~20ms** 再切回 UART（**低电平启动**，见 `DistanceSensor_Process()`）。与以太网 MDIO **无关**。

### 15.5 内存 / lwIP 冲突

| ID | 问题 | 现象 | 解决办法 |
|:--:|------|------|----------|
| **M1** | lwIP 池占 **片内 SRAM** | `lwIP init failed code=1` | 池改 **SRAMEX**（外扩） | ✅ 已做 |
| **M2** | ETH DMA 描述符 | 需 mem1 | 保持 `ETH_Mem_Malloc()` 在 mem1 | ✅ |
| **M3** | `FREERTOS_HEAP_BYTES=32KB` | 任务多栈紧张 | 见 `rtos移植.md`；`vNetTask` 栈 512 words | 监控 `[RTOS] stack` |

### 15.6 上位机 / 网络层冲突（Jetson / PC）

| ID | 问题 | 现象 | 解决办法 |
|:--:|------|------|----------|
| **N1** | Jetson **eno1 + WiFi 同网段** `192.168.1.x` | SSH 超时、ping 走错网卡 | **分网段**：WiFi `192.168.1.x`，有线 `192.168.10.x`；eno1 `never-default` | §11.7 |
| **N2** | eno1 **未插线** 却配 `192.168.1.200` | 路由进黑洞 | 未插线 **删掉** eno1 上 `192.168.1.x` | §11.6 |
| **N3** | PC 防火墙拦 **UDP 50002** | 只 ping 通、收不到 BLOB | 放行入站 50002 或临时关防火墙 | §14.3 |
| **N4** | PC IP 与 F407 **不同网段** | ping 不通 | F407 `192.168.10.30`，PC `192.168.10.201` | `lwip_comm.c` |

### 15.7 串口分配一览（避免再接错线）

|  USART | 引脚 | 波特率 | 用途 | 与以太网 |
|:------:|------|--------|------|----------|
| **USART1** | PA9/10 | 115200 | **调试 printf**（USB 转串口） | 独立，可常开 |
| **USART2** | PA2/PA3 | 115200 | Jetson RS232 BLOB（`ETH=0` 时） | **PA2 与 MDIO 冲突** |
| **USART3** | PB10/11 | 9600 | **四路测距** E08 | **无冲突** |
| **USART6** | PC6/7 | 9600→115200 | GPS | 无冲突 |

### 15.8 新增功能前检查清单

- [ ] 是否占用 **PA2**？若开 `ETH_LWIP_ENABLE`，PA2 只能给 MDIO
- [ ] 是否再开一条 **Jetson 控制链路**？只能 CAN / RS232 / ETH 三选一
- [ ] ISR 里是否 **长时间关全局中断**？会伤测距 USART3
- [ ] 是否在 ISR 里 **printf**？禁止；日志放任务
- [ ] lwIP 路径是否在 **持锁** 时阻塞？`NO_SYS=1` 下注意与 FreeRTOS 任务并发
- [ ] 上位机是否 **持续 0x01 心跳**（50Hz）？
- [ ] Jetson 是否 **双网卡同网段**？
- [ ] E08 转接板是否 **独立 5V 供电稳定**、与 F407 **共地**？（见 **§15.11**）

### 15.9 代码审计结论（2026-06-17）

| 区域 | 结论 |
|------|------|
| `App_EthPoll` | T1 已改为逐帧关中断；`lwip_periodic` 仍短临界区，可接受 |
| `jetson_eth.c` | `udp_sendto` 前关中断，范围小 |
| `app_boot.c` | 以太网模式 **跳过 USART2 init**（v1.4+）；`App_EthInit` 在测距 **之后** |
| `distance_sensor.c` | 独占 `USART3_IRQHandler`；与 `usart3.c` **不同 USART 硬件** |
| `app_boot.h` | `APP_SENSOR_TEST_ONLY=1` 可关 ETH/GPS/CAN 做 **纯测距联调** |
| `rs485.c` / `rs232.c` | `USART2_IRQHandler` 仅在 `RS485_ENABLE` 等宏下编译；默认不链入 |
| `vJetsonTask` | 以太网模式仍调 `USART3_GetServiceRequest` 无害（无 init 则无数据） |
| `agv_blob_link.h` | RS232 / ETH 发送 **互斥**，不会双发 |
| 遗留风险 | TIM3 与测距 USART3 同优先级 **1**；日志过多 **T6**；若仍测距丢包可升测距 IRQ 到 **0** |

### 15.10 快速排障对照

| 你的现象 | 优先查 |
|----------|--------|
| ping 通，测距全 `---` / 65535 | **T1**（固件是否含 App_EthPoll 修复）；**§15.11 供电**；PB10/11 接线 |
| RS232 有收无发 | **P1** `ETH_LWIP_ENABLE=1` |
| `ARB=DEGRADED` | **L3** 上位机未发 0x01 |
| SSH / ping 网关超时 | **N1/N2** Jetson 双网卡 |
| 抓不到 UDP 50002 | **N3** 防火墙 |
| `[DS] RX timeout` 且 `rx>0` | **T1** 收帧中丢字节 |
| `rx=0, skip=0, raw=(none)`，`trig` 在涨 | **§15.11 供电/GND**；PB10 触发线；非软件/以太网问题 |
| `rx>0, skip≈rx, FF=0` | 波特率错（应 **9600**）或 TX/RX 接反 |
| `FF≥1, chk>0, ok=0` | 有帧头但校验失败：线材干扰、接触不良 |
| 同一固件有时 `rx=12` 有时 `rx=0` | **供电不稳或杜邦线接触不良**（实验室已确认） |

### 15.11 测距无数据：E08 供电问题（2026-06 实验室确认）

本节记录 **SENSOR_TEST_ONLY 独占测距联调** 中已定位的根因：**E08 四路测距转接模组供电不足或不稳定**。与以太网 lwIP、Jetson BLOB、PA2 MDIO **无因果关系**。

#### 现象与日志特征

固件开启 `APP_SENSOR_TEST_ONLY=1`（`app_boot.h`）后，串口应出现：

```text
[BOOT] SENSOR_TEST_ONLY: ETH/GPS/CAN/Jetson/Arbiter OFF
[DS] HW PB10 MOD=2 AF10=7 | PB11 MOD=2 AF11=7
[DS TEST] ... proc=... trig=... rx=... skip=... FF=... chk=... raw=...
```

| 日志组合 | 软件侧结论 | 硬件侧优先查 |
|----------|------------|--------------|
| `trig` 持续增长，`rx=0`，`skip=0`，`raw=(none)` | TIM4 触发正常，**PB11 未收到任何字节** | **E08 5V 供电、GND 共地、模组是否上电** |
| 同一固件偶发 `rx=12` 后长期 `rx=0` | 曾收到乱码/不完整数据后模组停发 | **供电跌落、杜邦线松动** |
| `[DS] HW ... MOD=2 AF=7`，`UE=1` | MCU 引脚与 USART3 配置正确 | 排除“改以太网代码改坏引脚” |

> **勿误判为以太网冲突**：RMII 不使用 PB10/PB11；独占测距模式下 ETH 已编译关闭，仍 `rx=0` 时应查 **模组供电**，而非 lwIP/UDP。

#### 根因说明

- E08 转接板需 **稳定 5V** 为四路探头 + UART 电路供电。
- 若仅通过开发板 **5V 排针 + 细杜邦线** 长距离供电，或与其他大电流负载（电机、以太网 PHY、WiFi）共用，会出现：
  - 上电瞬间模组无响应 → `rx=0`；
  - 偶发回几个字节后停发 → 与实验室观测 `rx=12` 后归零一致；
  - 压降导致 UART 波形异常 → `skip>0` 但 `FF=0`（乱码，非帧头 `0xFF`）。

#### 推荐接线与供电

| 项目 | 要求 |
|------|------|
| E08 **VCC** | **独立 5V 电源**（≥500mA 余量）或专用 DC 模块，勿与电机驱动共用一个弱 5V |
| **GND** | E08 GND 与 **F407 开发板 GND 必须共地** |
| **UART** | 模组 TX → **PB11**（MCU RX）；触发/UART → **PB10**；**9600 8N1** |
| 杜邦线 | 插紧；`rx` 时有时无优先重插 **VCC/GND/PB10/PB11** |

#### 固件侧独占测距开关（便于定责）

`APP/freertos/app_boot.h`：

```c
#define APP_SENSOR_TEST_ONLY  1   /* 1=仅测距；自动 ETH_LWIP_ENABLE=0 */
```

- `1`：只跑 TIM4 + USART3 + LCD + `[DS TEST]` 诊断，排除 Jetson/以太网干扰。
- 测距恢复正常后改回 `0`，Rebuild，再开以太网 BLOB 联调。

#### 供电修复后的验收

1. `[DS TEST]` 中 `rx` 持续增加，`FF≥1`，`ok≥1`（或 LCD 四向出现 `xxx mm`）。
2. 恢复 `APP_SENSOR_TEST_ONLY=0`，确认 `[DS ETH]` 或 `[DS]` 在以太网模式下仍正常（需 **T1** `App_EthPoll` 修复已合入）。

### 15.12 以太网 TimeSync（0xA5 服务帧，2026-06 已合入）

Jetson `eth_gateway` 经 **UDP :50001** 发 **11B** `[0xA5][ID][8B]`；MCU 在 `JetsonEth_OnUdpRx` 按首字节分流：

| `payload[0]` | 处理 |
|:--:|------|
| `0xAB` | 现有 BLOB v2 入队（不变） |
| `0xA5` | 服务帧入队 → `vJetsonTask` → `JetsonCAN_HandleServiceRequest(0x107, …)` |
| 其它 | `svc_reject++` |

**0x108 响应**：`JetsonLink_Deliver8()` 在 `JETSON_LINK_ETH=1` 时走 `JetsonEth_SendServiceFrame()` → **UDP Peer :50002**（11B，非 BLOB）。

**边界**：0x107/0x108 **不**调用 `Arbiter_UpdateHeartbeat()`；心跳仍只看 BLOB **MSG 0x01** seq。

**联调日志**：

```text
[ETH SVC] first 0x107 cmd=0x02 START
[ETH SVC] tx 0x108 cmd=0x02
[JETSON LINK] +5s: ... svc=... 107=... tx108=... svc_rej=...
```

**代码**：`APP/jetson_eth/jetson_eth.c`、`jetson_can.c`（`JetsonLink_Deliver8`）、`rtos_tasks.c`（`vJetsonTask` 服务帧循环）。

---

*文档维护：接入实现后请更新「状态」列，并在 [硬件连接与通信协议.md](./硬件连接与通信协议.md) 总表增加以太网一行。*

