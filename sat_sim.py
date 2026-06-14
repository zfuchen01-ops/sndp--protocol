#!/usr/bin/env python3
"""
SBDP over LEO Constellation — Physics-Driven Demo
===================================================
离散事件仿真 + 轨道力学 + SBDP 协议。

场景:
  8 颗 LEO 卫星同轨道面 (550km, 倾角 53°)
  1 个地面站: GS-Main (互联网出口)
  3 个用户: User-1/2 连 SAT-03/SAT-05, User-3 直连 GS-Main

事件驱动流程:
  LINK_DISCOVERY (每 10s) → 计算卫星位置 → 链路预算 → 更新拓扑
  TOPOLOGY_CHANGE → SBDP_PUSH → 用户更新回传视图
  RECORD (每 60s) → 记录瓶颈历史

时间加速: 30× (1 真实秒 = 30 仿真秒)
"""

import sys
import os
import math
import random
import time as _time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from collections import defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from orbit import (
    Satellite, OrbitalElements, walker_constellation,
    orbital_position, distance_km, discover_links,
    ground_station_ecef, isl_bandwidth_mbps, sgl_bandwidth_mbps,
    LinkBudget, EARTH_RADIUS_KM,
)
from event_sim import EventSim
from sbdp import SBDPEngine

# ════════════════════════════════════════════════
#  仿真配置
# ════════════════════════════════════════════════

SIM_DURATION_SEC = 6000.0     # 仿真时长 (秒) ≈ 1 个 LEO 轨道周期
LINK_CHECK_INTERVAL = 10.0    # 链路发现间隔 (仿真秒)
RECORD_INTERVAL = 60.0        # 记录间隔
REALTIME_FACTOR = 30.0        # 时间加速因子 (0 = 尽快)
TOPOLOGY_CHANGE_THRESHOLD = 5.0  # 带宽变化超过 5 Mbps 才触发推送

# 星座参数
N_PLANES = 1
SATS_PER_PLANE = 8        # 8 颗/面 → 45° 间距 → ~5300km ISL 距离
ALTITUDE_KM = 550.0
INCLINATION_DEG = 53.0

# 地面站
# GS-Main: 互联网出口 (赤道附近)
# GS-Remote: 用户侧 (中纬度)
GS_MAIN = ("GS-Main", ground_station_ecef(0.0, 10.0))      # 0°N, 10°E


# ════════════════════════════════════════════════
#  SBDP 卫星节点 (仿真实体)
# ════════════════════════════════════════════════

class SbdpSatNode:
    """卫星上的 SBDP 协议实体"""
    def __init__(self, sat_id: str):
        self.sat_id = sat_id
        self.engine = SBDPEngine(sat_id)
        self.topology: dict = {}
        self.routing: dict = {}
        self.neighbors: dict = {}
        self.downstream: set = set()
        self.last_advertised: Dict[str, float] = {}

    def update_topology(self, full_topo: dict, gs_main_id: str = "GS-Main"):
        """从全局拓扑更新本节点视图"""
        self.topology = full_topo
        self.neighbors = full_topo.get(self.sat_id, {})
        self.routing = _compute_routing(self.sat_id, full_topo)
        self.downstream = _compute_downstream(self.sat_id, full_topo, gs_main_id)

    def compute_backhaul(self, gs_main_id: str = "GS-Main") -> Tuple[Optional[float], str]:
        """计算到 GS-Main 的回传瓶颈"""
        return _compute_backhaul(self.sat_id, self.topology, gs_main_id)

    def should_push(self, dst: str, backhaul: float) -> bool:
        prev = self.last_advertised.get(dst, -1)
        return prev < 0 or abs(backhaul - prev) > TOPOLOGY_CHANGE_THRESHOLD


class SbdpUserNode:
    """用户侧的 SBDP 协议实体"""
    def __init__(self, user_id: str):
        self.user_id = user_id
        self.engine = SBDPEngine(user_id)
        self.backhaul_history: List[Tuple[float, float, str]] = []  # (t, bw, link)
        self.topology: dict = {}
        self.routing: dict = {}
        self.neighbors: dict = {}

    def update_topology(self, full_topo: dict):
        self.topology = full_topo
        self.neighbors = full_topo.get(self.user_id, {})
        self.routing = _compute_routing(self.user_id, full_topo)

    def receive_advisory(self, raw_bytes: bytes, sim_time: float = 0):
        result = self.engine.process_incoming(raw_bytes, self.routing, self.neighbors)
        if result is None and self.engine.backhaul_view != float('inf'):
            self.backhaul_history.append((
                sim_time,
                self.engine.backhaul_view,
                self.engine.bottleneck_link,
            ))


# ════════════════════════════════════════════════
#  路由工具函数 (与 node.py 相同逻辑)
# ════════════════════════════════════════════════

def _compute_routing(node_id: str, topo: dict) -> dict:
    all_nodes = set(topo.keys())
    dist = {n: float('inf') for n in all_nodes}
    prev = {n: None for n in all_nodes}
    dist[node_id] = 0
    unvisited = set(all_nodes)
    while unvisited:
        u = min(unvisited, key=lambda n: dist[n])
        if dist[u] == float('inf'):
            break
        unvisited.remove(u)
        for v in topo.get(u, {}):
            if v in unvisited and dist[u] + 1 < dist[v]:
                dist[v] = dist[u] + 1
                prev[v] = u
    routing = {}
    for dst in all_nodes:
        if dst == node_id:
            continue
        curr = dst
        while prev.get(curr) and prev[curr] != node_id:
            curr = prev[curr]
        if prev.get(curr) == node_id:
            routing[dst] = curr
    return routing


def _compute_downstream(node_id: str, topo: dict, gs_id: str) -> set:
    def hops(nid):
        curr, cnt = nid, 0
        visited = set()
        while curr != gs_id and curr not in visited and cnt < 20:
            visited.add(curr)
            rt = _compute_routing(curr, topo)
            nxt = rt.get(gs_id)
            if nxt is None:
                return 999
            curr = nxt
            cnt += 1
        return cnt if curr == gs_id else 999

    my_hops = hops(node_id)
    downstream = set()
    for nb in topo.get(node_id, {}):
        if nb == gs_id:
            continue
        if 'User' in nb or 'GS' in nb:
            downstream.add(nb)
        elif hops(nb) > my_hops:
            downstream.add(nb)
    return downstream


def _compute_backhaul(node_id: str, topo: dict, gs_id: str) -> Tuple[Optional[float], str]:
    current = node_id
    visited = set()
    bottleneck = float('inf')
    bottleneck_link = ""
    while current != gs_id and current not in visited and len(visited) < 20:
        visited.add(current)
        rt = _compute_routing(current, topo)
        nxt = rt.get(gs_id)
        if nxt is None:
            return (None, "")
        bw = topo.get(current, {}).get(nxt, 0)
        if bw > 0 and bw < bottleneck:
            bottleneck = bw
            bottleneck_link = f"{current}→{nxt} ({bw:.0f}Mbps)"
        current = nxt
    if current != gs_id:
        return (None, "")
    return (bottleneck, bottleneck_link)


# ════════════════════════════════════════════════
#  主仿真
# ════════════════════════════════════════════════

class SatSim:
    def __init__(self):
        self.sim = EventSim()
        self.satellites: List[Satellite] = []
        self.sat_nodes: Dict[str, SbdpSatNode] = {}
        self.user_nodes: Dict[str, SbdpUserNode] = {}
        self.ground_stations: List[Tuple[str, any]] = []
        self.current_topology: dict = {}
        self.user_links: dict = {}  # user → (sat, bw)
        self.push_count: int = 0
        self.topo_change_count: int = 0

    def build_scenario(self):
        """构建星座 + 地面站 + 用户"""
        print("=" * 60)
        print("  SBDP + LEO Constellation — Physics-Driven Demo")
        print("=" * 60)
        print(f"  Constellation: {N_PLANES}p × {SATS_PER_PLANE}s @ {ALTITUDE_KM}km, {INCLINATION_DEG}° inc")
        print(f"  Ground stations: GS-Main (internet gateway only)")
        print(f"  Sim duration: {SIM_DURATION_SEC}s (≈ 1 orbit)")
        print(f"  Time factor: {REALTIME_FACTOR}×")
        print()

        # 星座
        self.satellites = walker_constellation(
            n_planes=N_PLANES,
            sats_per_plane=SATS_PER_PLANE,
            altitude_km=ALTITUDE_KM,
            inclination_deg=INCLINATION_DEG,
            phasing=1,
            prefix="SAT",
        )
        for sat in self.satellites:
            self.sat_nodes[sat.sat_id] = SbdpSatNode(sat.sat_id)

        # 地面站 (只有互联网出口)
        self.ground_stations = [GS_MAIN]

        # 用户 (固定连接到卫星的接入链路)
        # User-1 → SAT-03, User-2 → SAT-05 (中纬度用户, 通过卫星回传)
        # User-3 → GS-Main (直连互联网出口)
        self.user_nodes["User-1"] = SbdpUserNode("User-1")
        self.user_nodes["User-2"] = SbdpUserNode("User-2")
        self.user_nodes["User-3"] = SbdpUserNode("User-3")
        self.user_links = {
            "User-1": ("SAT-03", 200.0),
            "User-2": ("SAT-05", 200.0),
            "User-3": ("GS-Main", 600.0),
        }

        # 打印初始状态
        print(f"  Satellites: {[s.sat_id for s in self.satellites]}")
        print(f"  GS-Main:    internet gateway")
        print(f"  User-1→SAT-03, User-2→SAT-05 (access links)")
        print(f"  User-3:     direct to GS-Main")
        print()

    def link_discovery_callback(self):
        """周期性链路发现 (事件回调)"""
        t = self.sim.now
        gs_list = [(gs_id, pos) for gs_id, pos in self.ground_stations]
        # 放宽 ISL 最大距离 (演示用)
        isl_budget = LinkBudget(max_range_km=8000.0)
        sgl_budget = LinkBudget(max_range_km=3000.0)
        topo = discover_links(self.satellites, gs_list, t,
                              isl_budget=isl_budget, sgl_budget=sgl_budget)

        # 添加用户链路 (固定)
        for user_id, (gs_id, bw) in self.user_links.items():
            if gs_id not in topo:
                topo[gs_id] = {}
            topo[gs_id][user_id] = bw
            if user_id not in topo:
                topo[user_id] = {}
            topo[user_id][gs_id] = bw

        # 检查是否有显著变化
        changed = self._topology_changed(topo)

        if changed:
            self.topo_change_count += 1
            self.current_topology = topo

            # 更新所有节点
            for sat_id, node in self.sat_nodes.items():
                node.update_topology(topo)
            for user_id, node in self.user_nodes.items():
                node.update_topology(topo)

            # 触发 SBDP 推送
            self._sbdp_push(t)

    def _topology_changed(self, new_topo: dict) -> bool:
        """检查拓扑是否显著变化"""
        if not self.current_topology:
            return True  # 首次

        old_nodes = set(self.current_topology.keys())
        new_nodes = set(new_topo.keys())
        if old_nodes != new_nodes:
            return True

        for nid in new_nodes:
            old_nbs = self.current_topology.get(nid, {})
            new_nbs = new_topo.get(nid, {})
            if set(old_nbs.keys()) != set(new_nbs.keys()):
                return True
            for nb, new_bw in new_nbs.items():
                old_bw = old_nbs.get(nb, 0)
                if abs(new_bw - old_bw) > TOPOLOGY_CHANGE_THRESHOLD:
                    return True
        return False

    def _sbdp_push(self, t: float):
        """所有卫星执行 SBDP 推送"""
        for sat_id, node in self.sat_nodes.items():
            backhaul, bottleneck = node.compute_backhaul("GS-Main")
            if backhaul is None or backhaul == float('inf'):
                continue

            for dst in sorted(node.downstream):
                if not node.should_push(dst, backhaul):
                    continue

                # 构建 SBDP 报文
                msg = node.engine.build_advertisement(dst, backhaul, bottleneck, t)
                wire = msg.pack()
                node.last_advertised[dst] = backhaul
                self.push_count += 1

                # 投递给目标用户
                if dst in self.user_nodes:
                    user = self.user_nodes[dst]
                    user.receive_advisory(wire, t)

    def record_callback(self):
        """周期性记录"""
        pass  # 用户已经在 receive_advisory 中记录了

    def run(self):
        self.build_scenario()

        # 注册事件
        self.sim.schedule(0.0, self.link_discovery_callback)
        self.sim.schedule_recurring(LINK_CHECK_INTERVAL, self.link_discovery_callback,
                                     offset=LINK_CHECK_INTERVAL)

        print(f"  Running simulation for {SIM_DURATION_SEC}s...")
        print(f"  (time factor {REALTIME_FACTOR}× → ~{SIM_DURATION_SEC/REALTIME_FACTOR:.0f}s wall clock)")
        print()

        t0 = _time.time()
        self.sim.run(SIM_DURATION_SEC, realtime_factor=REALTIME_FACTOR)
        elapsed = _time.time() - t0

        print(f"\n  Done. Wall clock: {elapsed:.1f}s, "
              f"Sim time: {SIM_DURATION_SEC}s "
              f"(effective {SIM_DURATION_SEC/elapsed:.0f}×)")

        self.print_report()
        self.plot()

    def print_report(self):
        print(f"\n{'='*60}")
        print(f"  Simulation Report")
        print(f"{'='*60}")
        print(f"  Topology changes: {self.topo_change_count}")
        print(f"  SBDP push events: {self.push_count}")
        print()

        for user_id in ["User-1", "User-2", "User-3"]:
            node = self.user_nodes[user_id]
            hist = node.backhaul_history
            if not hist:
                print(f"  {user_id}: no backhaul data")
                continue
            bws = [h[1] for h in hist]
            first_bw = hist[0][1]
            last_bw = hist[-1][1]
            min_bw = min(bws)
            max_bw = max(bws)
            updates = len(hist)
            print(f"  {user_id}: {updates} updates, "
                  f"first={first_bw:.0f} last={last_bw:.0f} "
                  f"min={min_bw:.0f} max={max_bw:.0f} Mbps")
            print(f"    bottleneck: {hist[-1][2]}")

    def plot(self):
        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError:
            print("\n  (pip install matplotlib for plot)")
            return

        fig, axes = plt.subplots(2, 1, figsize=(14, 9))
        colors = {"User-1": "#2196F3", "User-2": "#4CAF50", "User-3": "#FF9800"}

        # 图1: 瓶颈带宽时间序列
        ax1 = axes[0]
        for uid in ["User-1", "User-2", "User-3"]:
            node = self.user_nodes[uid]
            hist = node.backhaul_history
            if not hist:
                continue
            times = [h[0] / 60 for h in hist]  # sim seconds → minutes
            bws = [h[1] for h in hist]
            ax1.step(times, bws, where='post', color=colors[uid], label=uid,
                     linewidth=2)
            ax1.scatter(times, bws, color=colors[uid], s=15, zorder=5)

        ax1.set_xlabel("Simulation Time (minutes)", fontsize=12)
        ax1.set_ylabel("Backhaul Bottleneck (Mbps)", fontsize=12)
        ax1.set_title("SBDP-Tracked Backhaul Bottleneck over LEO Orbit",
                      fontsize=13)
        ax1.legend(fontsize=11)
        ax1.grid(True, alpha=0.3)

        # 图2: 卫星位置 (2D 投影)
        ax2 = axes[1]
        # 采样几个时刻
        sample_times = [0, SIM_DURATION_SEC * 0.25, SIM_DURATION_SEC * 0.5,
                        SIM_DURATION_SEC * 0.75]

        for frac, marker in zip([0, 0.25, 0.5, 0.75], ['o', 's', '^', 'D']):
            t_sample = SIM_DURATION_SEC * frac
            label = f"t={frac*100:.0f}% orbit" if frac > 0 else "t=0"
            xs, ys = [], []
            for sat in self.satellites:
                pos = orbital_position(sat.oe, t_sample)
                xs.append(pos[0])
                ys.append(pos[1])
            ax2.scatter(xs, ys, marker=marker, s=60, label=label, zorder=5)
            for i, sat in enumerate(self.satellites):
                ax2.annotate(sat.sat_id.replace("SAT-0", "S"),
                            (xs[i], ys[i]), fontsize=8,
                            textcoords="offset points", xytext=(5, -5))

        # 地面站
        for gs_id, gs_pos in self.ground_stations:
            ax2.scatter([gs_pos[0]], [gs_pos[1]], marker='*', s=200,
                       c='red' if 'Main' in gs_id else 'orange',
                       label=gs_id if gs_id == "GS-Main" else None,
                       zorder=10, edgecolors='black')
            ax2.annotate(gs_id, (gs_pos[0], gs_pos[1]), fontsize=9,
                        textcoords="offset points", xytext=(5, 5))

        ax2.set_xlabel("X ECI (km)", fontsize=12)
        ax2.set_ylabel("Y ECI (km)", fontsize=12)
        ax2.set_title(f"Constellation: {N_PLANES}p×{SATS_PER_PLANE}s @ {ALTITUDE_KM}km, "
                      f"{INCLINATION_DEG}deg inc", fontsize=13)
        ax2.legend(fontsize=9, loc='upper right')
        ax2.grid(True, alpha=0.3)
        ax2.set_aspect('equal')

        plt.suptitle("SBDP + LEO Constellation — Physics-Driven Backhaul Discovery",
                     fontsize=15, fontweight='bold')
        plt.tight_layout()
        plt.savefig("/Users/cowboy/sat-bottleneck-demo/orbit_sbdp.png", dpi=120)
        print(f"\n  Plot saved: orbit_sbdp.png")
        plt.close()


if __name__ == "__main__":
    random.seed(42)
    sim = SatSim()
    sim.run()
