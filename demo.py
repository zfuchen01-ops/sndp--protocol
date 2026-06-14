#!/usr/bin/env python3
"""
卫星网络 — 回传容量主动推送协议 Demo
======================================

场景：
  用户通过卫星上网，卫星基站持续监控回传链路容量，
  一旦容量变化超过阈值，立即向下游用户推送 CAPACITY_ADV 报文。
  用户在整个业务会话期间，始终知道自己当前的"回传瓶颈"。

协议 — 回传容量通告报文 (Capacity Advertisement)：
┌──────────────────────────────────────────────────────────────┐
│ 字段              │ 类型    │ 含义                            │
├───────────────────┼─────────┼─────────────────────────────────┤
│ src               │ str     │ 发送方 (卫星/地面站)            │
│ dst               │ str     │ 接收方 (用户) — 单播或广播      │
│ seq_num           │ int     │ 单调递增序号，检测丢失/过期      │
│ backhaul_bw       │ float   │ 当前回传瓶颈 (Mbps)             │
│ bottleneck_link   │ str     │ 瓶颈链路描述                    │
│ path              │ list    │ 回传路径                       │
│ timestamp         │ float   │ 发送时间                       │
│ valid_for         │ float   │ 有效期 (时间内未收到更新则告警)  │
├──────────────────────────────────────────────────────────────┤
│ PAYLOAD           │ str     │ 可选：建议发送速率等             │
└──────────────────────────────────────────────────────────────┘

工作流程：
  ┌─────────┐     ┌─────────┐     ┌─────────┐     ┌──────────┐
  │ User-1  │     │ SAT-A   │     │ SAT-B   │     │  GS (回传)│
  │         │◄────│         │◄────│         │◄────│ 1000Mbps │
  │         │ ADY │ 600Mbps │ ADY │ 350Mbps │     │ 互联网   │
  └─────────┘     └────┬────┘     └────┬────┘     └──────────┘
                       │               │
                       │            User-2
                       │            (直接连SAT-B)

  1. GS 连接到互联网，回传带宽 1000M
  2. SAT-B 检测 GS→SAT-B 链路 = 350M → 推送 "回传=350M" 给 User-2
  3. SAT-A 检测 SAT-B→SAT-A 链路 = 600M
     SAT-A 收到 SAT-B 的通告(350M) → min(600, 350) = 350M
     SAT-A 推送 "回传=350M" 给 User-1
  4. 链路波动时，星上监控触发新的推送
  5. 用户在整个会话期间持续收到更新 → "一直知道回传容量"
"""

import math
import random
from dataclasses import dataclass, field
from collections import defaultdict
from typing import Dict, List, Optional, Tuple

# ============================================================
#  配置
# ============================================================

SIMULATION_STEPS = 80          # 业务会话持续 80 个时间步
MONITOR_INTERVAL = 2           # 卫星每隔 2 步检测一次链路
PUSH_THRESHOLD = 0.03          # 容量变化超过 3% 就推送更新
TOPOLOGY_UPDATE_INTERVAL = 8   # 每 8 步拓扑波动一次
CAPACITY_VALID_FOR = 10        # 通告有效期（步），超时未更新视为过期

# ============================================================
#  协议数据结构
# ============================================================

@dataclass
class CapacityAdvisory:
    """
    回传容量通告 — 这就是卫星推给用户的"通知"。

    不是用户问、网络答；而是网络主动推、用户被动收。
    """
    src: str                        # 谁发的
    dst: str                        # 推给谁
    seq_num: int                    # 序号 (每颗卫星独立编号)
    backhaul_bw: float              # 当前回传瓶颈 (Mbps)
    bottleneck_link: str            # 瓶颈在哪
    path: List[str] = field(default_factory=list)  # 回传路径
    timestamp: float = 0.0          # 何时发送
    valid_for: float = CAPACITY_VALID_FOR  # 有效期

    def display(self) -> str:
        bw = f"{self.backhaul_bw:.1f} Mbps" if self.backhaul_bw != float('inf') else "?"
        path_str = ' → '.join(self.path) if self.path else '(待填充)'
        return (
            "┌─────────────────────────────────────────────────────────┐\n"
            "│  CAPACITY_ADV 回传容量通告 (Push)                        │\n"
            "├──────────────────┬──────────────────────────────────────┤\n"
            f"│ src              │ {self.src:<36} │\n"
            f"│ dst              │ {self.dst:<36} │\n"
            f"│ seq_num          │ #{self.seq_num:<35} │\n"
            f"│ backhaul_bw      │ {bw:<36} │\n"
            f"│ bottleneck_link  │ {self.bottleneck_link or '(无)':<36} │\n"
            f"│ path             │ {path_str:<36} │\n"
            f"│ timestamp        │ {self.timestamp:<36.1f} │\n"
            f"│ valid_for        │ {self.valid_for:<36.0f} 步 │\n"
            "└──────────────────┴──────────────────────────────────────┘"
        )

    def summary(self) -> str:
        return (f"[#{self.seq_num}] {self.src}→{self.dst}: "
                f"回传={self.backhaul_bw:.0f}Mbps @ {self.bottleneck_link}")


# ============================================================
#  节点
# ============================================================

class Node:
    def __init__(self, node_id: str, node_type: str):
        self.node_id = node_id
        self.node_type = node_type  # "ground_station" | "satellite" | "user"
        self.neighbors: Dict[str, float] = {}  # neighbor_id → bw_mbps
        self.routing: Dict[str, str] = {}      # dst → next_hop


class GroundStation(Node):
    """地面站 — 回传的起点，连互联网"""

    def __init__(self, node_id: str, internet_bw: float):
        super().__init__(node_id, "ground_station")
        self.internet_bw = internet_bw  # 到互联网的回传总带宽
        self.seq = 0

    def push_capacity(self, dst: str, timestamp: float, network: "Network") -> Optional[CapacityAdvisory]:
        """
        地面站生成回传容量通告，沿下游路径推给目标用户。
        路径上每颗卫星都会更新瓶颈字段。
        """
        self.seq += 1
        adv = CapacityAdvisory(
            src=self.node_id,
            dst=dst,
            seq_num=self.seq,
            backhaul_bw=self.internet_bw,
            bottleneck_link=f"{self.node_id} → 互联网 ({self.internet_bw:.0f} Mbps)",
            path=[self.node_id],
            timestamp=timestamp,
        )
        return network.push_advisory(adv)


class Satellite(Node):
    """卫星 — 核心：监控链路 + 转发通告 + 向下游推送"""

    def __init__(self, node_id: str):
        super().__init__(node_id, "satellite")
        self.seq = 0
        self.last_advertised_bw: Dict[str, float] = {}  # dst → 上次推送的容量
        self.downstream_users: List[str] = []  # 直接连接的用户
        self.downstream_sats: List[str] = []   # 下游卫星（远离地面站的方向）

    def monitor_and_push(self, ts: float, network: "Network") -> List[CapacityAdvisory]:
        """
        卫星周期性监控：
        1. 检查自己的回传链路（到上游卫星/地面站的出口带宽）
        2. 结合上游推送过来的回传容量，算出自己的回传瓶颈
        3. 如果变化超过阈值，向下游用户和卫星推送更新
        """
        advisories = []

        # 计算到地面站的回传瓶颈
        # 回传方向 = 卫星 → 地面站 → 互联网
        gs_nodes = [n for n in network.nodes.values() if n.node_type == "ground_station"]
        if not gs_nodes:
            return advisories

        # 找到最近的地面站
        for gs in gs_nodes:
            backhaul_bw = self._calc_backhaul_to(gs.node_id, network)
            if backhaul_bw is None:
                continue

            # 推给直连用户
            for user_id in self.downstream_users:
                prev = self.last_advertised_bw.get(user_id, 0)
                if prev == 0 or abs(backhaul_bw - prev) / max(prev, 1) > PUSH_THRESHOLD:
                    self.seq += 1
                    adv = CapacityAdvisory(
                        src=self.node_id,
                        dst=user_id,
                        seq_num=self.seq,
                        backhaul_bw=backhaul_bw,
                        bottleneck_link=self._bottleneck_desc(network),
                        path=self._backhaul_path_to(gs.node_id, network),
                        timestamp=ts,
                    )
                    self.last_advertised_bw[user_id] = backhaul_bw
                    advisories.append(adv)

            # 推给下游卫星（它们再推给自己的用户）
            for dsat_id in self.downstream_sats:
                prev = self.last_advertised_bw.get(dsat_id, 0)
                if prev == 0 or abs(backhaul_bw - prev) / max(prev, 1) > PUSH_THRESHOLD:
                    self.seq += 1
                    adv = CapacityAdvisory(
                        src=self.node_id,
                        dst=dsat_id,
                        seq_num=self.seq,
                        backhaul_bw=backhaul_bw,
                        bottleneck_link=self._bottleneck_desc(network),
                        path=self._backhaul_path_to(gs.node_id, network),
                        timestamp=ts,
                    )
                    self.last_advertised_bw[dsat_id] = backhaul_bw
                    advisories.append(adv)

        return advisories

    def _calc_backhaul_to(self, gs_id: str, network: "Network") -> Optional[float]:
        """计算自己经过回传路径到地面站的瓶颈带宽"""
        # 从自己到地面站的最短路径
        path = network.shortest_path(self.node_id, gs_id)
        if not path or len(path) < 2:
            return None

        # 沿路取 min 带宽
        bottleneck = float('inf')
        for i in range(len(path) - 1):
            a, b = path[i], path[i+1]
            bw = self.neighbors.get(b, 0) if a == self.node_id else network.nodes[a].neighbors.get(b, 0)
            if bw == 0:
                # 查找网络中该链路的带宽
                node_a = network.nodes.get(a)
                if node_a:
                    bw = node_a.neighbors.get(b, 0)
            if bw > 0:
                bottleneck = min(bottleneck, bw)
        return bottleneck if bottleneck != float('inf') else None

    def _bottleneck_desc(self, network: "Network") -> str:
        """描述回传瓶颈"""
        gs = [n for n in network.nodes.values() if n.node_type == "ground_station"]
        if not gs:
            return "无地面站"
        path = network.shortest_path(self.node_id, gs[0].node_id)
        if not path or len(path) < 2:
            return "无路径"
        # 找最小带宽链路
        min_bw, min_link = float('inf'), ""
        for i in range(len(path) - 1):
            a, b = path[i], path[i+1]
            na = network.nodes.get(a)
            bw = na.neighbors.get(b, 0) if na else 0
            if 0 < bw < min_bw:
                min_bw = bw
                min_link = f"{a} → {b} ({bw:.0f} Mbps)"
        return min_link

    def _backhaul_path_to(self, gs_id: str, network: "Network") -> List[str]:
        """回传路径（从自己到地面站）"""
        path = network.shortest_path(self.node_id, gs_id)
        return path if path else [self.node_id]


class User(Node):
    """用户 — 被动接收卫星推送的回传容量通知"""

    def __init__(self, node_id: str):
        super().__init__(node_id, "user")
        self.backhaul_capacity: float = 0.0       # 当前已知回传容量
        self.backhaul_bottleneck: str = ""        # 当前瓶颈链路
        self.backhaul_path: List[str] = []        # 回传路径
        self.last_update: float = -999            # 上次收到更新的时间
        self.update_history: List[Tuple[float, int, float, float, float, str]] = []  # (time, seq, backhaul_bw, total_bw, access_bw, link)
        self.stale_warnings: int = 0              # 过期告警次数

    def receive_advisory(self, adv: CapacityAdvisory, timestamp: float):
        """接收卫星推送的回传容量通告"""
        self.backhaul_capacity = adv.backhaul_bw
        self.backhaul_bottleneck = adv.bottleneck_link
        self.backhaul_path = adv.path
        self.last_update = timestamp
        # 总瓶颈 = min(回传瓶颈, 接入链路带宽)
        access_bw = self._get_access_bw()
        total = min(adv.backhaul_bw, access_bw) if access_bw > 0 else adv.backhaul_bw
        self.update_history.append((timestamp, adv.seq_num, adv.backhaul_bw, total, access_bw, adv.bottleneck_link))

    def _get_access_bw(self) -> float:
        """获取自己到上游卫星的接入链路带宽"""
        for nb_id, bw in self.neighbors.items():
            return bw
        return 0.0

    def check_stale(self, timestamp: float) -> bool:
        """检查回传容量信息是否过期"""
        if self.last_update < 0:
            return True
        return (timestamp - self.last_update) > CAPACITY_VALID_FOR

    def get_session_report(self) -> str:
        """业务会话结束后，输出用户看到的回传容量变化报告"""
        if not self.update_history:
            return f"  {self.node_id}: 整个会话期间未收到任何回传容量通知 ⚠"

        lines = [
            f"\n  {'─'*65}",
            f"  {self.node_id} 业务会话 — 回传容量追踪 (Push 模式)",
            f"  {'─'*65}",
            f"  收到更新: {len(self.update_history)} 次",
            f"  接入链路: {self.update_history[0][4]:.0f} Mbps (本用户到卫星)",
            f"  初始回传: {self.update_history[0][2]:.0f} Mbps → 总瓶颈: {self.update_history[0][3]:.0f} Mbps",
            f"  最终回传: {self.update_history[-1][2]:.0f} Mbps → 总瓶颈: {self.update_history[-1][3]:.0f} Mbps",
            f"  最低回传: {min(h[2] for h in self.update_history):.0f} Mbps",
            f"  最高回传: {max(h[2] for h in self.update_history):.0f} Mbps",
            f"  过期告警: {self.stale_warnings} 次",
            f"",
            f"  {'时间':<8} {'序号':<8} {'回传容量':>10} {'总瓶颈':>10} {'瓶颈链路'}",
            f"  {'-'*8} {'-'*8} {'-'*10} {'-'*10} {'-'*35}",
        ]
        for t, seq, backhaul, total, access, link in self.update_history:
            lines.append(f"  t={t:<5.0f}  #{seq:<6} {backhaul:>8.0f} Mbps {total:>8.0f} Mbps   {link}")
        return "\n".join(lines)


# ============================================================
#  网络
# ============================================================

class Network:
    """
    卫星回传网络。

    拓扑 (回传视角：右侧是地面站/互联网)：

      User-1                User-2
         │                     │
      SAT-A ────600M──── SAT-B ────350M──── GS ────1000M─── 互联网
         │                                    │
      User-3                              (回传出口)
    """

    def __init__(self):
        self.nodes: Dict[str, Node] = {}
        self.time: float = 0.0

    def get_node(self, node_id: str) -> Optional[Node]:
        return self.nodes.get(node_id)

    def build_topology(self):
        """构建拓扑"""
        # 地面站（互联网回传出口）
        gs = GroundStation("GS", internet_bw=1000)
        self.nodes["GS"] = gs

        # 卫星
        sat_b = Satellite("SAT-B")
        sat_b.downstream_sats = ["SAT-A"]  # SAT-A 在 SAT-B 下游（远离 GS）
        sat_b.downstream_users = ["User-2"]
        self.nodes["SAT-B"] = sat_b

        sat_a = Satellite("SAT-A")
        sat_a.downstream_users = ["User-1", "User-3"]  # 两颗直连用户星
        self.nodes["SAT-A"] = sat_a

        # 用户
        for uid in ["User-1", "User-2", "User-3"]:
            self.nodes[uid] = User(uid)

        # 链路 (回传方向 = 用户→卫星→卫星→地面站→互联网)
        self._add_link("GS",    "SAT-B", 350)   # 星地链路
        self._add_link("SAT-B", "SAT-A", 600)   # 星间链路
        self._add_link("SAT-A", "User-1", 150)  # 用户链路
        self._add_link("SAT-A", "User-3", 200)  # 用户链路
        self._add_link("SAT-B", "User-2", 400)  # 用户链路

        # 计算初始路由
        self.update_routing()

    def _add_link(self, a: str, b: str, bw: float):
        self.nodes[a].neighbors[b] = bw
        self.nodes[b].neighbors[a] = bw

    def update_routing(self):
        """所有节点计算到所有目标的最短路径"""
        for src_id in self.nodes:
            self.nodes[src_id].routing = self._dijkstra(src_id)

    def _dijkstra(self, src: str) -> Dict[str, str]:
        dist = {n: float('inf') for n in self.nodes}
        prev = {n: None for n in self.nodes}
        dist[src] = 0
        unvisited = set(self.nodes.keys())
        while unvisited:
            u = min(unvisited, key=lambda n: dist[n])
            if dist[u] == float('inf'):
                break
            unvisited.remove(u)
            for v in self.nodes[u].neighbors:
                if v in unvisited:
                    alt = dist[u] + 1
                    if alt < dist[v]:
                        dist[v] = alt
                        prev[v] = u
        routing = {}
        for dst in self.nodes:
            if dst == src:
                continue
            curr = dst
            while prev.get(curr) and prev[curr] != src:
                curr = prev[curr]
            if prev.get(curr) == src:
                routing[dst] = curr
        return routing

    def shortest_path(self, src: str, dst: str) -> List[str]:
        """返回 src→dst 的节点路径"""
        if src == dst:
            return [src]
        path = [src]
        current = src
        visited = set()
        while current != dst and current not in visited:
            visited.add(current)
            nxt = self.nodes[current].routing.get(dst)
            if nxt is None:
                return []
            path.append(nxt)
            current = nxt
        return path if current == dst else []

    def fluctuate_topology(self, step: int):
        """
        模拟链路带宽波动（卫星运动、雨衰等）。
        关键：双向链路必须对称更新，使用相同的随机因子。
        """
        seen_links = set()  # 防止同一条链路两端用不同随机值

        for nid, node in self.nodes.items():
            for neighbor_id in list(node.neighbors.keys()):
                link_key = tuple(sorted([nid, neighbor_id]))
                if link_key in seen_links:
                    continue
                seen_links.add(link_key)

                nb = self.nodes[neighbor_id]

                if node.node_type == "satellite" and nb.node_type == "satellite":
                    factor = 1.0 + random.uniform(-0.25, 0.25)
                    floor = 80
                elif "SAT" in nid and "GS" in neighbor_id or "GS" in nid and "SAT" in neighbor_id:
                    factor = 1.0 + random.uniform(-0.35, 0.35)
                    floor = 50
                elif "User" in nid or "User" in neighbor_id:
                    factor = 1.0 + random.uniform(-0.10, 0.10)
                    floor = 50
                else:
                    floor = 50
                    factor = 1.0

                new_bw = round(max(floor, node.neighbors[neighbor_id] * factor), 1)
                node.neighbors[neighbor_id] = new_bw
                nb.neighbors[nid] = new_bw  # 对称！

        self.update_routing()
        # 拓扑变化 → 强制卫星推送（清零记忆）
        for nid, node in self.nodes.items():
            if isinstance(node, Satellite):
                node.last_advertised_bw.clear()

    def push_advisory(self, adv: CapacityAdvisory) -> Optional[CapacityAdvisory]:
        """
        沿路径推送回传容量通告。
        只在回传链路（卫星↔卫星、卫星↔地面站）上更新瓶颈；
        用户接入链路不算回传，不更新 backhaul_bw ——
        这样用户收到的就是卫星的真实回传容量，不含最后一跳。
        """
        current_id = adv.src
        max_hops = 20

        while max_hops > 0:
            max_hops -= 1
            node = self.nodes.get(current_id)
            if node is None:
                return None

            if current_id == adv.dst:
                # 到达目标用户
                adv.path.append(current_id)
                if isinstance(node, User):
                    node.receive_advisory(adv, self.time)
                return adv

            # 转发：找下一跳
            next_hop = node.routing.get(adv.dst)
            if next_hop is None:
                return None

            out_bw = node.neighbors.get(next_hop, 0)
            if out_bw == 0:
                return None

            next_node = self.nodes.get(next_hop)
            is_user_link = (isinstance(next_node, User) or isinstance(node, User))

            # ★ 核心：只在回传链路上更新瓶颈，跳过用户接入链路 ★
            if not is_user_link and out_bw < adv.backhaul_bw:
                adv.backhaul_bw = out_bw
                adv.bottleneck_link = f"{current_id} → {next_hop} ({out_bw:.0f} Mbps)"

            adv.path.append(current_id)
            current_id = next_hop

        return None

    def run(self):
        """仿真主循环"""
        print("=" * 65)
        print("  卫星回传容量主动推送协议 — 业务会话仿真")
        print("=" * 65)
        print(f"\n  场景: 3个用户通过 2颗卫星 + 1个地面站上网")
        print(f"  协议: 卫星主动推送回传容量给用户 (Push, 非 Probe)")
        print(f"  会话时长: {SIMULATION_STEPS} 步")
        print(f"  监控间隔: 每 {MONITOR_INTERVAL} 步")
        print(f"  推送阈值: 变化 > {PUSH_THRESHOLD*100:.0f}%")
        print(f"  通告有效期: {CAPACITY_VALID_FOR} 步\n")

        self._print_topology()

        # --- 业务会话开始 ---
        print(f"\n{'='*65}")
        print(f"  ▸ 业务会话开始")
        print(f"{'='*65}")

        for step in range(SIMULATION_STEPS):
            self.time = step

            # 定期拓扑波动
            if step > 0 and step % TOPOLOGY_UPDATE_INTERVAL == 0:
                self.fluctuate_topology(step)
                print(f"\n  ── t={step}: 拓扑发生变化 ──")
                self._print_topology()

            # 卫星周期性监控 + 推送
            if step % MONITOR_INTERVAL == 0:
                for nid, node in self.nodes.items():
                    if isinstance(node, Satellite):
                        advisories = node.monitor_and_push(step, self)
                        for adv in advisories:
                            print(f"\n  [t={step}] {node.node_id} 推送 → {adv.dst}")
                            print(f"  {adv.summary()}")
                            self.push_advisory(adv)

                    elif isinstance(node, GroundStation):
                        # 地面站也周期性推给直连卫星
                        for sat_id in node.neighbors:
                            sat = self.nodes.get(sat_id)
                            if isinstance(sat, Satellite):
                                adv = node.push_capacity(sat_id, step, self)
                                if adv:
                                    print(f"\n  [t={step}] GS 推送 → {sat_id}")
                                    print(f"  {adv.summary()}")

            # 用户检查是否过期
            for nid, node in self.nodes.items():
                if isinstance(node, User) and node.check_stale(step):
                    if node.last_update >= 0:
                        node.stale_warnings += 1
                        print(f"\n  ⚠ [t={step}] {nid}: 回传容量信息已过期! "
                              f"(上次更新: t={node.last_update:.0f})")

        # --- 业务会话结束 ---
        print(f"\n\n{'='*65}")
        print(f"  ▸ 业务会话结束 (t={SIMULATION_STEPS})")
        print(f"{'='*65}")

        for uid in ["User-1", "User-2", "User-3"]:
            user = self.nodes[uid]
            print(user.get_session_report())

        self._plot()


    def _print_topology(self):
        print(f"  ── 当前拓扑 (t={self.time:.0f}) ──")
        seen = set()
        for nid, node in sorted(self.nodes.items()):
            for nb, bw in sorted(node.neighbors.items()):
                key = tuple(sorted([nid, nb]))
                if key in seen:
                    continue
                seen.add(key)
                tag = ""
                if node.node_type == "satellite" and self.nodes[nb].node_type == "satellite":
                    tag = "[星间]"
                elif "GS" in nid or "GS" in nb:
                    tag = "[星地]"
                elif "User" in nid or "User" in nb:
                    tag = "[用户]"
                print(f"    {nid} ↔ {nb} {tag:<8} {bw:>8.0f} Mbps")

    def _plot(self):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            print("\n  (pip install matplotlib 可获取趋势图)")
            return

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        colors = {"User-1": "#2196F3", "User-2": "#4CAF50", "User-3": "#FF9800"}

        for uid in ["User-1", "User-2", "User-3"]:
            user = self.nodes[uid]
            if not user.update_history:
                continue
            times = [h[0] for h in user.update_history]
            # 画回传容量 (h[2] = backhaul_bw)
            bws = [h[2] for h in user.update_history]
            ax1.step(times, bws, where='post', color=colors[uid], label=uid,
                     linewidth=2, alpha=0.85)
            ax1.scatter(times, bws, color=colors[uid], s=20, zorder=5)
            # 虚线画总瓶颈 (h[3] = total_bw)
            totals = [h[3] for h in user.update_history]
            ax1.step(times, totals, where='post', color=colors[uid],
                     linewidth=1, alpha=0.4, linestyle='--')

        ax1.set_xlabel("Time (step)", fontsize=12)
        ax1.set_ylabel("Capacity (Mbps)", fontsize=12)
        ax1.set_title("Backhaul (solid) & Total Bottleneck (dashed)", fontsize=13)
        ax1.legend(fontsize=11)
        ax1.grid(True, alpha=0.3)
        ax1.set_ylim(bottom=0)

        for uid in ["User-1", "User-2", "User-3"]:
            user = self.nodes[uid]
            if not user.update_history:
                continue
            # h[3] = total_bw (min of backhaul + access)
            bws = [h[3] for h in user.update_history]
            ax2.hist(bws, bins=15, alpha=0.5, color=colors[uid], label=uid,
                     edgecolor='black', linewidth=0.5)

        ax2.set_xlabel("Backhaul Bottleneck (Mbps)", fontsize=12)
        ax2.set_ylabel("Frequency", fontsize=12)
        ax2.set_title("Backhaul Capacity Distribution (Full Session)", fontsize=13)
        ax2.legend(fontsize=11)
        ax2.grid(True, alpha=0.3)

        plt.suptitle("Satellite Backhaul Push Protocol — Demo Results", fontsize=15, fontweight='bold')
        plt.tight_layout()
        plt.savefig("/Users/cowboy/sat-bottleneck-demo/backhaul_push.png", dpi=120)
        print(f"\n  📊 趋势图: backhaul_push.png")
        plt.close()


if __name__ == "__main__":
    random.seed(42)
    net = Network()
    net.build_topology()
    net.run()
