# SBDP 项目上下文 — 给新会话快速理解

## 我们在做什么
设计 SBDP (Satellite Backhaul Discovery Protocol)——卫星网络中让用户被动持续知道自己回传瓶颈带宽的协议。

## 协议核心
- 寄生在 IP 层 (IPv6 Hop-by-Hop Option)
- 16B/104B 报头，含 backhaul_bw (float32) 字段
- 每跳执行 `min(出口带宽, 传入值)` 更新瓶颈
- 卫星主动推送 (Push) 给用户，用户无需探测
- CRC-16-CCITT 每跳重算

## 关键文件
| 文件 | 说明 |
|------|------|
| `sat-bottleneck-demo/sbdp.py` | Python 协议实现 (16B+TLV) |
| `sat-bottleneck-demo/orbit.py` | Keplerian 轨道 + 链路预算 |
| `sat-bottleneck-demo/experiment-report.html` | 实验报告 (重点看这个) |
| `ns-allinone-3.42/ns-3.42/contrib/sbdp/` | NS-3 协议模块 (Header 104B) |
| `ns-allinone-3.42/ns-3.42/scratch/sbdp-*.cc` | 6 个 NS-3 仿真脚本 |
| `~/Desktop/切换代码v.1.1/` | 用户的 36 星 Walker 星座 Python 项目 |

## NS-3 6 个实验
1. sbdp-proof — PCAP 逐字节证明 backhaul INF→150M
2. sbdp-handover — 多用户接入切换
3. sbdp-real-orbit — 36 星轨道事件驱动
4. sbdp-constellation — 星座拓扑波动
5. sbdp-optimized — 心跳 + seq 连续性
6. sbdp-visibility — 可见星回传表切换

## 当前要解决
用户发了两张图（架构图），我看不了。需要能识图的模型来比对：
- 图里是不是卫星+gNB+UE 的 5G NTN 架构
- 和我报告里画的卫星架构是否一致
- 用户桌面有 "截屏2026-06-14 09.55.20.png" 和 "09.55.24.png"

## 运行
```bash
# NS-3 实验
cd ns-allinone-3.42/ns-3.42
./ns3 run sbdp-proof

# 查看 PCAP
tcpdump -r sbdp-proof-*.pcap -X

# HTML 报告
open sat-bottleneck-demo/experiment-report.html
```
