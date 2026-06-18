# 以太网与 WiFi 接入方案

| 元数据 | 值 |
|--------|-----|
| **文档版本** | v1.2 |
| **文档日期** | 2026-06-16 |
| **状态** | lwIP 已接入、PHY/DMA 初始化正常；**ping 联调未通（L2 层）**，应用层 UDP/BLOB 未接；**当前优先 RS232** |
| **适用固件** | 麒麟 F407 综合实验程序，`UI_TEST_MODE=1` 仲裁模式 |

本文档描述 **上位机 ↔ F407** 经 **网线直连** 与 **WiFi + 网页** 两种网络控制的**整体流程、架构分层与实施步骤**。

| 相关文档 | 说明 |
|----------|------|
| [硬件连接与通信协议.md](./硬件连接与通信协议.md) | CAN / RS232 接线与系统拓扑 |
| [Jetson_CAN协议.md](./Jetson_CAN协议.md) | V3 应用层帧定义（以太网/WiFi 建议复用） |
| [Jetson_RS232协议.md](./Jetson_RS232协议.md) | RS232 传输封装（与 CAN 应用层一致） |
| [Jetson_BLOB协议_v2.md](./Jetson_BLOB协议_v2.md) | BLOB v2 二进制协议（RS232/CAN 双传输，当前 RS232 联调主文档） |
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
| 默认 IP | 静态 **`192.168.10.30`**，网关 `192.168.10.200`（Jetson eno1） | `LwIP/lwip_app/lwip_comm/lwip_comm.c` |
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
| 以太网 | `ETH_LWIP_ENABLE=1` | lwIP 已启；**L2 ping 未通，联调暂停** |
| RS232 BLOB v2 | `JETSON_USE_BLOB_V2=1` | 代码已接，**当前联调重点** |

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
| 上位机 -> F407 | UDP | **50001** | V3 下行 `type=0x01`（24B） |
| F407 -> 上位机 | UDP | **50002** | V3 上行 `type=0x02/0x03`（24B） |
| 服务帧（可选） | UDP | **50003** | 时间同步/故障/GPS（8B 或 24B 封装） |
| 连通性测试 | ICMP | — | `ping -I 192.168.10.200 192.168.10.30` |

**单帧交互时序**

```text
上位机                              F407
  |  UDP:50001  [24B V3 0x01 seq=N]  -->  收包 -> 校验 XOR -> Arbiter_ParseJetsonCmd
  |  <-- UDP:50002  [24B V3 0x02]     |  vJetsonTask 周期上报（20ms）
  |  <-- UDP:50002  [24B V3 0x03]     |  交替发 status/detail
```

### 4.3 MCU 侧实施流程（分步）

```text
步骤 1 — 编译开关
  新增 APP/jetson_can/jetson_can.h：
    #define JETSON_LINK_TYPE  0   /* 0=RS232  1=CAN  2=ETH */
  或保留 JETSON_LINK_CAN，另增 JETSON_LINK_ETH

步骤 2 — 启动 lwIP（仲裁模式）
  在 App_MotionHwInit() 或独立 vNetTask 中：
    lwip_comm_init()
    创建 UDP PCB，绑定本地 50001，注册接收回调
  主循环 / 专用任务中周期性调用：
    lwip_periodic_handle()

步骤 3 — 收发模块（新建 APP/jetson_eth/）
  jetson_eth.c：
    - JetsonEth_Init()
    - JetsonEth_OnUdpRx() -> 写入环形缓冲
    - JetsonEth_GetFrame() -> 供 vJetsonTask 读取（接口对齐 JetsonCAN_GetFrame）
    - JetsonEth_SendV3Frame() -> UDP 发往 192.168.1.106:50002

步骤 4 — 接入现有任务
  修改 APP/freertos/rtos_tasks.c 中 vJetsonTask：
    #if JETSON_LINK_ETH
      JetsonEth_ProcessRx();
      if(JetsonEth_GetFrame(jetson_frame)) ...
      JetsonEth_SendV3StatusFrame(...);
    #elif JETSON_LINK_CAN
      ...（现有逻辑）
    #else
      ...（RS232）

步骤 5 — Jetson/PC 测试脚本
  Python socket：发 0x01、收 0x02，统计 RTT 延时
```

### 4.4 上位机侧（Jetson / PC）

```python
# 伪代码示意
sock_tx = UDP -> ("192.168.1.30", 50001)
sock_rx = UDP bind local 50002
send_v3_cmd(seq, v, steer, omega)   # 24 字节，定义见 Jetson_CAN协议.md 4
status = recv_v3_status()           # type=0x02
rtt_ms = t_recv - t_send
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
  [ ] lwip_comm_init 接入仲裁模式
  [ ] jetson_eth 模块 + vJetsonTask 分支
  [ ] Jetson Python 收发脚本
  [ ] ping + V3 RTT 测试报告

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
| 网线直连 | RJ45 + LAN8720 | **UDP**（主）+ ICMP（ping） | V3 24B | lwIP 已启，**L2 未通，暂停** |
| RS232 直连 | USART2 PA2/PA3 | BLOB v2 / V3 帧 | BLOB / V3 | **当前联调重点** |
| 网页控制 | 网线->路由器->WiFi | **HTTP + CGI** | V3 24B | 有演示，待改运动页 |
| WiFi 免网线 | WiFi 模组（后续） | AT 透传 / 模组 Web | V3 24B | 未实现 |

**一句话**：以太网 **软件栈已就绪**，当前卡在 **L2 物理/链路**（见 §13）；**RS232 BLOB v2** 为现阶段主链路。若专搞 RS232，建议 `ETH_LWIP_ENABLE=0` 释放 **PA2** 给 `USART2_TX`。

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
| 网关 / Jetson 对端 | `192.168.10.200` |
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
  __disable_irq()                      ← 避免与 lwip_periodic 重入
  while (ETH_GetRxPktSize(DMARxDescToGet) != 0)
    lwip_pkt_handle()                  → ethernetif_input → ETH_Rx_Packet
    s_eth_rx_frames++
  ETH_RecoverRxDma()                   ← 清 DMASR.RBUS，DMARPDR=0
  lwip_periodic_handle()               ← ARP/TCP/DHCP 定时器
  __enable_irq()
```

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

### 12.9 应用层接入点（尚未实现）

以太网 **UDP/BLOB 控制** 待新建 `APP/jetson_eth/`，在 `vJetsonTask` 中与 RS232/CAN 分支并列（见 §4.3）。当前 `vJetsonTask` 仍只走 `USART3_*` / `JetsonCAN_*`。

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

### 13.5 当前决策

- **以太网联调暂停**；软件栈与诊断日志保留，后续换线/换口/笔记本交叉验证后再续。
- **控制链路改 RS232 BLOB v2**（见 [Jetson_BLOB协议_v2.md](./Jetson_BLOB协议_v2.md)、[Jetson_RS232协议.md](./Jetson_RS232协议.md)）。
- RS232 联调前建议 **`ETH_LWIP_ENABLE=0`**，避免 PA2 被 MDIO 占用导致 **MCU→Jetson 发不出**。

### 13.6 RS232 BLOB v2 联调问题（与以太网独立）

以太网 ping 与 **Jetson↔F407 RS232** 是两条链路。RS232 阶段典型问题已整理至（**v2.0-draft.5.8**）：

**[Jetson_BLOB协议_v2.md §C.9 RS232 BLOB 联调实录：问题与解决方案](./Jetson_BLOB协议_v2.md#c9-rs232-blob-联调实录问题与解决方案)**

| 阶段 | 现象摘要 | 文档章节 |
|------|----------|----------|
| 下行 L3 | `ab_idle=0` / 一直 DEGRADED | §C.9.3、**§C.9.9**（include 未编译 BLOB 收包） |
| 下行 L3 已通过 | F407 `ARB=NORMAL`，`[JETSON BLOB CMD]` | §C.9.14 阶段 A |
| 上行 L4 | topic 空、gateway「上行超时」；`listen-only` 能收 0x02/0x03 | **§C.9.10**（gateway 混流解析，**MCU 无需再改**） |

---

*文档维护：接入实现后请更新「状态」列，并在 [硬件连接与通信协议.md](./硬件连接与通信协议.md) 总表增加以太网一行。*
