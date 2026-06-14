#!/usr/bin/env python3
"""
SBDP Controller
===============
编排多进程 SBDP 仿真：启动节点 → 下发拓扑 → 周期性波动 → 触发推送 → 收集报表。

用法:
  python controller.py                     # 自动启动所有子进程
  python controller.py --no-spawn          # 手动在各终端启动 node.py
  python controller.py --steps 30 --interval 4 --topo-interval 10
"""

import sys
import os
import json
import socket
import time
import random
import argparse
import subprocess
import signal
from collections import defaultdict

# ════════════════════════════════════════════════
#  配置
# ════════════════════════════════════════════════

HOST = "127.0.0.1"
CTRL_PORT = 11000  # controller 自己的控制端口

NODES = {
    "GS":     9001,
    "SAT-B":  9002,
    "SAT-A":  9003,
    "User-1": 9011,
    "User-2": 9012,
    "User-3": 9013,
}

INITIAL_TOPOLOGY = {
    "GS":      {"SAT-B": 350},
    "SAT-B":   {"GS": 350, "SAT-A": 600, "User-2": 400},
    "SAT-A":   {"SAT-B": 600, "User-1": 150, "User-3": 200},
    "User-1":  {"SAT-A": 150},
    "User-2":  {"SAT-B": 400},
    "User-3":  {"SAT-A": 200},
}


# ════════════════════════════════════════════════
#  链路波动
# ════════════════════════════════════════════════

def fluctuate(topology: dict, links: dict) -> dict:
    """
    对称波动链路带宽。
    links: {(a,b): bw} 是当前所有链路的展平视图。
    返回 {"A↔B": new_bw, ...}
    """
    updates = {}
    seen = set()
    for (a, b), old in sorted(links.items()):
        key = tuple(sorted([a, b]))
        if key in seen:
            continue
        seen.add(key)

        a_sat = 'SAT' in a
        b_sat = 'SAT' in b
        if a_sat and b_sat:
            factor = 1.0 + random.uniform(-0.25, 0.25)
            floor = 80
        elif ('GS' in a) != ('GS' in b):
            factor = 1.0 + random.uniform(-0.35, 0.35)
            floor = 50
        else:
            factor = 1.0 + random.uniform(-0.10, 0.10)
            floor = 50

        new_bw = round(max(floor, old * factor), 1)
        updates[f"{a}↔{b}"] = new_bw

    return updates


def flatten_links(topology: dict) -> dict:
    """展平拓扑为 {(a,b): bw}"""
    links = {}
    for a, nbs in topology.items():
        for b, bw in nbs.items():
            key = tuple(sorted([a, b]))
            if key not in links:
                links[key] = bw
    return links


# ════════════════════════════════════════════════
#  Controller
# ════════════════════════════════════════════════

class Controller:
    def __init__(self, steps=30, interval=4, topo_interval=10, auto_spawn=True):
        self.steps = steps
        self.interval = interval
        self.topo_interval = topo_interval
        self.auto_spawn = auto_spawn
        self.topology = {k: dict(v) for k, v in INITIAL_TOPOLOGY.items()}
        self.links = flatten_links(self.topology)
        self.epoch = 0
        self.procs = []
        self.reports = []

        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctrl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.ctrl_sock.bind((HOST, CTRL_PORT))
        self.ctrl_sock.settimeout(1.0)

    def start_nodes(self):
        """启动所有节点进程"""
        script_dir = os.path.dirname(os.path.abspath(__file__))
        node_script = os.path.join(script_dir, "node.py")

        for node_id, port in NODES.items():
            cmd = [sys.executable, node_script, "--id", node_id, "--port", str(port)]
            if self.auto_spawn:
                proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                )
                self.procs.append((node_id, proc))
                print(f"  launched {node_id} (pid={proc.pid}) on :{port}")
            else:
                print(f"  manual: {' '.join(cmd)}")

        if self.auto_spawn:
            time.sleep(1.0)  # 等所有进程绑定 socket

    def send_control(self, node_id: str, msg: dict):
        """发送控制消息到指定节点"""
        port = NODES[node_id] + 1000
        data = json.dumps(msg).encode('utf-8')
        self.ctrl_sock.sendto(data, (HOST, port))

    def broadcast_control(self, msg: dict):
        """广播控制消息到所有节点"""
        for node_id in NODES:
            self.send_control(node_id, msg)

    def send_init(self):
        """向所有节点发送初始拓扑，等待全部就绪"""
        neighbor_addrs = {}
        for node_id in NODES:
            neighbor_addrs[node_id] = {
                nb: (HOST, NODES[nb])
                for nb in self.topology.get(node_id, {})
            }

        for node_id in NODES:
            msg = {
                "type": "init",
                "topology": self.topology,
                "neighbor_addrs": neighbor_addrs.get(node_id, {}),
            }
            self.send_control(node_id, msg)

        print(f"\n  init sent to {len(NODES)} nodes, waiting for ready...")

        # 等待所有节点就绪
        ready = set()
        deadline = time.time() + 5.0
        while len(ready) < len(NODES) and time.time() < deadline:
            try:
                raw, _ = self.ctrl_sock.recvfrom(65535)
                r = json.loads(raw.decode('utf-8'))
                if r.get("type") == "ready":
                    ready.add(r["node"])
                    print(f"  ✓ {r['node']} ready")
            except socket.timeout:
                continue
            except json.JSONDecodeError:
                continue

        if len(ready) < len(NODES):
            print(f"  ⚠ only {len(ready)}/{len(NODES)} nodes ready: {sorted(ready)}")
        else:
            print(f"  all {len(NODES)} nodes ready")

    def collect_reports(self) -> dict:
        """收集所有节点的报表"""
        self.broadcast_control({"type": "report_request"})
        reports = {}
        deadline = time.time() + 3.0
        while time.time() < deadline and len(reports) < len(NODES):
            try:
                raw, addr = self.ctrl_sock.recvfrom(65535)
                r = json.loads(raw.decode('utf-8'))
                reports[r["node"]] = r
            except socket.timeout:
                break
            except json.JSONDecodeError:
                continue
        return reports

    def run(self):
        print("=" * 65)
        print("  SBDP Multi-Process Demo — Controller")
        print("=" * 65)
        print(f"  steps={self.steps} interval={self.interval} topo_interval={self.topo_interval}")
        print(f"  nodes: {len(NODES)} ({', '.join(NODES)})")
        print()

        self.start_nodes()
        time.sleep(0.5)
        self.send_init()
        time.sleep(0.2)

        for step in range(self.steps):
            self.epoch = step

            # 拓扑波动
            if step > 0 and step % self.topo_interval == 0:
                updates = fluctuate(self.topology, self.links)
                for link_key, bw in updates.items():
                    a, b = link_key.split("↔")
                    self.topology[a][b] = bw
                    self.topology[b][a] = bw
                self.links = flatten_links(self.topology)
                self.broadcast_control({
                    "type": "topology_update",
                    "links": updates,
                    "epoch": step,
                })
                print(f"\n  ══ t={step}: topology changed ({len(updates)} links) ══")
                for lk, bw in sorted(updates.items()):
                    print(f"    {lk}: {bw:.0f} Mbps")

            # 推送触发
            if step % self.interval == 0:
                for node_id in NODES:
                    if 'SAT' in node_id or 'GS' in node_id:
                        self.send_control(node_id, {
                            "type": "push_trigger",
                            "epoch": step,
                        })
                print(f"  [t={step}] push_trigger")

            time.sleep(0.8)  # 给节点充足时间处理

        # 收集报表
        print(f"\n  collecting reports...")
        time.sleep(0.5)
        reports = self.collect_reports()

        # 关闭
        self.broadcast_control({"type": "shutdown"})
        time.sleep(1.0)

        # 等待子进程退出
        if self.auto_spawn:
            for node_id, proc in self.procs:
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()

        # 打印报表
        self.print_report(reports)

    def print_report(self, reports: dict):
        print(f"\n{'='*65}")
        print(f"  Session Report")
        print(f"{'='*65}")

        for node_id in ["GS", "SAT-B", "SAT-A", "User-1", "User-2", "User-3"]:
            r = reports.get(node_id, {})
            if 'User' in node_id:
                print(f"  {node_id}: processed={r.get('processed','?')} "
                      f"backhaul_view={r.get('backhaul_view','?')} Mbps "
                      f"bottleneck={r.get('bottleneck','?')}")
            elif 'SAT' in node_id:
                print(f"  {node_id}: processed={r.get('processed','?')} "
                      f"sent={r.get('sent','?')}")
            elif 'GS' in node_id:
                print(f"  {node_id}: processed={r.get('processed','?')} "
                      f"sent={r.get('sent','?')}")

        # 展示子进程输出
        if self.auto_spawn:
            print(f"\n{'='*65}")
            print(f"  Node Process Output")
            print(f"{'='*65}")
            for node_id, proc in self.procs:
                try:
                    out = proc.stdout.read()
                    if out:
                        print(f"\n── {node_id} ──")
                        for line in out.split('\n'):
                            stripped = line.strip()
                            if stripped:
                                print(f"  {stripped}")
                except Exception:
                    pass


def main():
    parser = argparse.ArgumentParser(description="SBDP Controller")
    parser.add_argument("--steps", type=int, default=30)
    parser.add_argument("--interval", type=int, default=4)
    parser.add_argument("--topo-interval", type=int, default=10)
    parser.add_argument("--no-spawn", action="store_true",
                        help="Don't auto-spawn nodes; user launches them manually")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    random.seed(args.seed)
    ctrl = Controller(
        steps=args.steps,
        interval=args.interval,
        topo_interval=args.topo_interval,
        auto_spawn=not args.no_spawn,
    )
    ctrl.run()


if __name__ == "__main__":
    main()
