# sat-bottleneck-demo — SBDP 卫星回传协议项目

## 项目结构

```
sat-bottleneck-demo/
  advisor-report.html      ← 实验报告前端 (白色界面, SbdpHeader格式)
  experiment-report.html   ← 旧版暗色报告 (过时, 勿用)
  sbdp.py                  ← Python 协议原型
  demo.py                  ← Python 仿真 demo
  *.pcap                   ← NS-3 实验抓包 (1,928,587包=feed链路)

ns-allinone-3.42/ns-3.42/
  contrib/sbdp/model/
    sbdp-header.h          ← 标准协议模块 (16B+TLV+CRC, 6消息类型)
    sbdp-header.cc         ← 实现: BuildAdv/BuildIslPush/BuildIslReq/BuildIslReply/BuildN2Ack
  scratch/
    sbdp-aware.cc          ← 主实验: ISL+N2+ADV 三层面 (已改用SbdpHeader)
    sbdp-proof.cc          ← 协议二进制证明实验
    sbdp-handover.cc       ← 多星切换实验
    sbdp-orbit.cc          ← 12星Walker轨道动态实验
    sbdp-n3-handover.cc    ← N3星间协同切换实验
    ...                    ← 其他实验脚本

~/Downloads/switch/        ← 切换算法代码 (DQN/DRQN/Transformer)
~/Downloads/LEO_handover_acceptance_*/ ← 论文复现项目 (RS/MSTS/MGCS/CAHS对比)
```

## 协议架构 (当前状态)

```
ISL (星间):  SBDP_LINK_PROBE / SBDP_CAPACITY_REQ / SBDP_CAPACITY_ACK
N2 (星内):   SBDP_CAPACITY_REQ / SBDP_CAPACITY_ACK
ADV (gNB→UE): SBDP_CAPACITY_ADV (BuildAdv)
N3 (协同):   SBDP_CAPACITY_MIGRATE / SBDP_CAPACITY_CONFIRM
```

全部使用 SbdpHeader (16B固定头 + TLV链 + CRC-16-CCITT)，通过 `AddHeader()/RemoveHeader()` 序列化。

## 运行实验

```bash
# 本地 NS-3 构建与运行
cd ~/ns-allinone-3.42/ns-3.42
export PATH="$HOME/cmake-3.30.2-macos-universal/CMake.app/Contents/bin:$PATH"
./ns3 build sbdp-aware
./ns3 run sbdp-aware

# 打开实验报告
open ~/sat-bottleneck-demo/advisor-report.html
```

## 远端 GPU 服务器 (TF4 实验)

```bash
# AutoDL, RTX 5090, Ubuntu 22.04
ssh -p 15562 root@connect.westc.seetacloud.com  # ZZx8lPfgSh9l
```

项目路径: `/root/leo_handover/`
- TF4 训练日志: `log/RL/DRQN_tf4_fix_cuda_per_ep.csv` (200ep)
- TF4 模型: `log/model/drqn_C_u200_tf4_fix_cuda_ep*.pkl` (ep50/100/150/200)
- TF3 日志: `log/RL/DRQN_tf3_*.csv`
- MGCS baselines: `baselines.py`, `run_mgcs_baseline.py`
- 论文复现: `results/final_calibrated_*.csv`

## 实验报告对应关系

HTML报告中的四个实验对应 NS-3 scratch 脚本:
| 报告章节 | NS-3 脚本 | 关键数据 |
|---|---|---|
| 关键实验 (U1/U2切换) | sbdp-aware.cc | B2=138→244M, U1=369MB, U2=556MB |
| 辅助实验一 (150M瓶颈) | sbdp-proof.cc | min(150,600,350)=150M |
| 辅助实验二 (多星多跳) | sbdp-handover.cc | SAT-B=282M, 多事件切换 |
| 辅助实验三 (12星Walker) | sbdp-orbit.cc | Python轨道→C++, Split Horizon 31条/100s |
| 辅助实验四 (TF4 RL对比) | 远端GPU训练 | Transformer vs MGCS/MRVT |

## 常用命令

```bash
# PCAP 分析
tcpdump -r sbdp-aware-feed-SAT-A-1.pcap -nn | wc -l  # 包数
capinfos sbdp-aware-*.pcap  # 统计

# NS-3 构建
./ns3 build <target>  # 构建单个目标
./ns3 run <target>    # 构建并运行
```
