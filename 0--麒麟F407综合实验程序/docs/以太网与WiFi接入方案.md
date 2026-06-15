# 以太网与 WiFi 接入方案

| 元数据 | 值 |
|--------|-----|
| **文档版本** | v1.0 |
| **文档日期** | 2026-06-15 |
| **状态** | 规划中（硬件/协议栈已有基础，未接入仲裁控制链路） |
| **适用固件** | 麒麟 F407 综合实验程序，`UI_TEST_MODE=1` 仲裁模式 |

本文档描述 **上位机 ↔ F407** 经 **网线直连** 与 **WiFi + 网页** 两种网络控制的**整体流程、架构分层与实施步骤**。

| 相关文档 | 说明 |
|----------|------|
| [硬件连接与通信协议.md](./硬件连接与通信协议.md) | CAN / RS232 接线与系统拓扑 |
| [Jetson_CAN协议.md](./Jetson_CAN协议.md) | V3 应用层帧定义（以太网/WiFi 建议复用） |
| [Jetson_RS232协议.md](./Jetson_RS232协议.md) | RS232 传输封装（与 CAN 应用层一致） |
| [rtos移植.md](./rtos移植.md) | 内存规划（含后续 WiFi 缓冲约定） |

---

## 1. 你要做的两件事（总览）

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
| 默认 IP | 静态 `192.168.1.30`，网关 `192.168.1.1` | `LwIP/lwip_app/lwip_comm/lwip_comm.c` |
| 演示应用 | HTTP 网页控 LED/蜂鸣器/ADC | `GUI_APP/eth_app.c`、`LwIP/lwip_app/web_server_demo/` |
| 仲裁模式 | **未启动** lwIP（`UI_TEST_MODE=1` 不跑以太网） | `docs/rtos移植.md` |

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
| 以太网 | 无宏，待新增 | 未接入 |

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
  Jetson/PC 网口 --网线-- F407 RJ45
  PC 设静态 IP：192.168.1.106（与 lwip_comm.c 中 remoteip 一致）
  F407 固定：192.168.1.30

方案 B — 经交换机/路由器
  Jetson、F407、PC 接同一局域网
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
| 连通性测试 | ICMP | — | `ping 192.168.1.30` |

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
| 网络层 | `ping 192.168.1.30` | ICMP 往返，通常 < 2 ms，**不等于**控制延时 |
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

---

## 10. 总结

| 功能 | 物理介质 | 传输协议 | 应用层 | 工程状态 |
|------|----------|----------|--------|----------|
| 网线直连 | RJ45 + LAN8720 | **UDP**（主）+ ICMP（ping） | V3 24B | 待开发接入 |
| 网页控制 | 网线->路由器->WiFi | **HTTP + CGI** | V3 24B | 有演示，待改运动页 |
| WiFi 免网线 | WiFi 模组（后续） | AT 透传 / 模组 Web | V3 24B | 未实现 |

**一句话**：先把 **网线 UDP + V3** 打通并测延时，再把 **httpd 网页** 改成运动控制面板；WiFi 是在此基础上的「最后一跳无线化」，可接路由器或外挂模组，**应用层和 Arbiter 逻辑不用重写**。

---

*文档维护：接入实现后请更新「状态」列，并在 [硬件连接与通信协议.md](./硬件连接与通信协议.md) 总表增加以太网一行。*
