#!/usr/bin/env python3
"""
SBDP Node Process
=================
每个节点独立进程，通过 UDP socket 运行 SBDP 协议。

用法:
  python node.py --id GS      --port 9001
  python node.py --id SAT-B   --port 9002
  python node.py --id SAT-A   --port 9003
  python node.py --id User-1  --port 9011
  python node.py --id User-2  --port 9012
  python node.py --id User-3  --port 9013

架构:
  - data socket (port):     收发 SBDP 二进制报文
  - control socket (port+1000): 收发 JSON 控制消息
  - select() 同时监听两个 socket
"""

import sys
import os
import json
import socket
import select
import argparse
import time

# 确保能找到 sbdp.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sbdp import SBDPEngine

# ════════════════════════════════════════════════
#  ANSI 颜色
# ════════════════════════════════════════════════

COLORS = {
    "GS":     "\033[35m",  # 品红
    "SAT-B":  "\033[34m",  # 蓝
    "SAT-A":  "\033[36m",  # 青
    "User-1": "\033[32m",  # 绿
    "User-2": "\033[33m",  # 黄
    "User-3": "\033[91m",  # 亮红
}
RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"

def cprint(node_id: str, msg: str):
    """带颜色前缀打印"""
    color = COLORS.get(node_id, "")
    print(f"{color}[{node_id}]{RESET} {msg}", flush=True)


# ════════════════════════════════════════════════
#  Dijkstra 路由计算
# ════════════════════════════════════════════════

def compute_routing(node_id: str, topology: dict) -> dict:
    """
    从完整拓扑计算本节点的路由表。

    topology 格式: {"GS": {"SAT-B": 350}, "SAT-B": {"GS": 350, "SAT-A": 600}, ...}

    返回: {dst: next_hop}
    """
    all_nodes = set(topology.keys())
    dist = {n: float('inf') for n in all_nodes}
    prev = {n: None for n in all_nodes}
    dist[node_id] = 0
    unvisited = set(all_nodes)

    while unvisited:
        u = min(unvisited, key=lambda n: dist[n])
        if dist[u] == float('inf'):
            break
        unvisited.remove(u)
        for v in topology.get(u, {}):
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


def compute_downstream(node_id: str, topology: dict, gs_id: str = "GS") -> set:
    """找出本节点的下游邻居（离 GS 比本节点更远的邻居）"""
    routing = compute_routing(node_id, topology)

    def hops_to_gs(nid):
        curr, cnt = nid, 0
        visited = set()
        while curr != gs_id and curr not in visited and cnt < 20:
            visited.add(curr)
            rt = compute_routing(curr, topology)  # ★ 每跳用当前节点的路由表
            nxt = rt.get(gs_id)
            if nxt is None:
                return 999
            curr = nxt
            cnt += 1
        return cnt if curr == gs_id else 999

    my_hops = hops_to_gs(node_id)
    downstream = set()
    for nb in topology.get(node_id, {}):
        if nb == gs_id:
            continue
        if 'User' in nb:
            downstream.add(nb)  # 用户总是下游
        elif hops_to_gs(nb) > my_hops:
            downstream.add(nb)
    return downstream


def compute_backhaul(node_id: str, topology: dict, gs_id: str = "GS") -> tuple:
    """
    计算本节点到地面站的回传瓶颈。
    每跳使用对应节点的路由表。
    返回 (bottleneck_bw_mbps, bottleneck_description)
    """
    current = node_id
    visited = set()
    bottleneck = float('inf')
    bottleneck_link = ""

    while current != gs_id and current not in visited and len(visited) < 20:
        visited.add(current)
        # ★ 每跳重新计算路由表
        rt = compute_routing(current, topology)
        nxt = rt.get(gs_id)
        if nxt is None:
            return (None, "")
        bw = topology.get(current, {}).get(nxt, 0)
        if bw > 0 and bw < bottleneck:
            bottleneck = bw
            bottleneck_link = f"{current} → {nxt} ({bw:.0f} Mbps)"
        current = nxt

    if current != gs_id:
        return (None, "")
    return (bottleneck, bottleneck_link)


# ════════════════════════════════════════════════
#  节点主逻辑
# ════════════════════════════════════════════════

class SBDPNode:
    def __init__(self, node_id: str, data_port: int, host: str = "127.0.0.1"):
        self.node_id = node_id
        self.host = host
        self.data_port = data_port
        self.ctrl_port = data_port + 1000
        self.engine = SBDPEngine(node_id)
        self.topology: dict = {}          # {node: {neighbor: bw}}
        self.routing: dict = {}           # {dst: next_hop}
        self.neighbors: dict = {}         # {neighbor: bw} (本节点)
        self.downstream: set = set()      # 下游邻居
        self.neighbor_addrs: dict = {}    # {neighbor: (host, data_port)}
        self.running = True

        self.data_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.data_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.data_sock.bind((host, data_port))

        self.ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.ctrl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.ctrl_sock.bind((host, self.ctrl_port))

        cprint(node_id, f"data=:{data_port} ctrl=:{self.ctrl_port}")
        cprint(node_id, f"waiting for controller init...")

    def handle_control(self, raw: bytes):
        """处理控制面 JSON 消息"""
        try:
            msg = json.loads(raw.decode('utf-8'))
        except json.JSONDecodeError:
            return

        mtype = msg.get("type")

        if mtype == "init":
            self._handle_init(msg)
        elif mtype == "topology_update":
            self._handle_topology_update(msg)
        elif mtype == "push_trigger":
            self._handle_push_trigger(msg)
        elif mtype == "report_request":
            self._handle_report_request(msg)
        elif mtype == "shutdown":
            self._handle_shutdown(msg)

    def _handle_init(self, msg: dict):
        """初始化：接收完整拓扑"""
        self.topology = msg["topology"]
        # JSON 只有 list，转为 socket 需要的 tuple
        raw_addrs = msg.get("neighbor_addrs", {})
        self.neighbor_addrs = {
            k: tuple(v) for k, v in raw_addrs.items()
        }
        self._recompute_state()
        cprint(self.node_id, f"INIT: {len(self.topology)} nodes, "
               f"neighbors={list(self.neighbors.keys())}"
               f" downstream={sorted(self.downstream)}")
        # 回复 ready
        self.ctrl_sock.sendto(
            json.dumps({"type": "ready", "node": self.node_id}).encode(),
            (self.host, 11000)
        )

    def _handle_topology_update(self, msg: dict):
        """拓扑更新：合并新的链路带宽"""
        links = msg.get("links", {})
        epoch = msg.get("epoch", 0)
        for link_key, bw in links.items():
            a, b = link_key.split("↔")
            if a in self.topology:
                self.topology[a][b] = bw
            elif a not in self.topology:
                self.topology[a] = {}
            if b in self.topology:
                self.topology[b][a] = bw
            elif b not in self.topology:
                self.topology[b] = {}
            self.topology[a][b] = bw
            self.topology[b][a] = bw
        self._recompute_state()
        cprint(self.node_id, f"TOPO epoch={epoch}: {len(links)} links updated, "
               f"neighbors={self.neighbors}")

    def _handle_push_trigger(self, msg: dict):
        """推送触发：卫星发送 CAPACITY_ADV 给下游"""
        epoch = msg.get("epoch", 0)
        if 'SAT' not in self.node_id and 'GS' not in self.node_id:
            return

        try:
            backhaul, bottleneck = compute_backhaul(self.node_id, self.topology)
        except Exception as e:
            cprint(self.node_id, f"ERROR compute_backhaul: {e}")
            return

        if backhaul is None or backhaul == float('inf'):
            cprint(self.node_id, f"PUSH skip: no backhaul path")
            return

        cprint(self.node_id,
               f"PUSH epoch={epoch} backhaul={backhaul:.0f}M downstream={sorted(self.downstream)}")
        for dst in sorted(self.downstream):
            addr = self.neighbor_addrs.get(dst, (self.host, 0))
            if addr[1] == 0:
                cprint(self.node_id, f"  skip {dst}: no address")
                continue
            sbdp_msg = self.engine.build_advertisement(dst, backhaul, bottleneck, float(epoch))
            wire = sbdp_msg.pack()
            self.data_sock.sendto(wire, addr)
            cprint(self.node_id,
                   f"  ADV→{dst} {len(wire)}B backhaul={backhaul:.0f}M")

    def _handle_report_request(self, msg: dict):
        """上报统计"""
        report = {
            "type": "report",
            "node": self.node_id,
            "processed": self.engine.processed_count,
            "sent": self.engine.sent_count,
            "backhaul_view": round(self.engine.backhaul_view, 1),
            "bottleneck": self.engine.bottleneck_link,
        }
        # 回复给 controller (ctrl port of controller = 11000)
        self.ctrl_sock.sendto(
            json.dumps(report).encode(),
            (self.host, 11000)
        )

    def _handle_shutdown(self, msg: dict):
        cprint(self.node_id, f"SHUTDOWN (processed={self.engine.processed_count}, "
               f"backhaul_view={self.engine.backhaul_view:.1f}M)")
        self.running = False

    def _recompute_state(self):
        """从 topology 重新计算路由、邻居带宽、下游节点"""
        self.neighbors = self.topology.get(self.node_id, {})
        try:
            self.routing = compute_routing(self.node_id, self.topology)
        except Exception as e:
            cprint(self.node_id, f"ERROR routing: {e}")
            self.routing = {}
        try:
            if 'GS' not in self.node_id and 'User' not in self.node_id:
                self.downstream = compute_downstream(self.node_id, self.topology)
            elif 'GS' in self.node_id:
                self.downstream = {n for n in self.neighbors if 'SAT' in n}
            else:
                self.downstream = set()
        except Exception as e:
            cprint(self.node_id, f"ERROR downstream: {e}")
            self.downstream = set()

    def handle_data(self, raw: bytes, addr: tuple):
        """处理数据面 SBDP 二进制报文"""
        result = self.engine.process_incoming(raw, self.routing, self.neighbors)
        if result is None:
            # 到达目的地或丢弃 — 检查是否到达目的地
            from sbdp import SBDPMessage
            try:
                msg = SBDPMessage.unpack(raw)
                hdr = msg.header
                cprint(self.node_id,
                       f"RECV {len(raw)}B seq=#{hdr.seq_num} "
                       f"backhaul={hdr.backhaul_capacity:.1f}M "
                       f"crc=0x{hdr.checksum:04x} ✓ (delivered)")
            except:
                cprint(self.node_id, f"RECV {len(raw)}B [corrupt/dropped]")
        else:
            next_hop, out_bytes, log = result
            nxt_addr = self.neighbor_addrs.get(next_hop, (self.host, 0))
            if nxt_addr[1] > 0:
                self.data_sock.sendto(out_bytes, nxt_addr)
            cprint(self.node_id, f"FWD {len(raw)}B→{len(out_bytes)}B {log}")

    def run(self):
        """主循环"""
        while self.running:
            try:
                readable, _, _ = select.select(
                    [self.data_sock, self.ctrl_sock], [], [], 0.5
                )
                for sock in readable:
                    raw, addr = sock.recvfrom(65535)
                    if sock is self.ctrl_sock:
                        self.handle_control(raw)
                    elif sock is self.data_sock:
                        self.handle_data(raw, addr)
            except KeyboardInterrupt:
                break
            except Exception as e:
                cprint(self.node_id, f"ERROR: {e}")

        self.data_sock.close()
        self.ctrl_sock.close()


def main():
    parser = argparse.ArgumentParser(description="SBDP Node")
    parser.add_argument("--id", required=True, help="Node ID (GS, SAT-B, SAT-A, User-1, User-2, User-3)")
    parser.add_argument("--port", type=int, required=True, help="UDP data port")
    parser.add_argument("--host", default="127.0.0.1", help="Bind host")
    args = parser.parse_args()

    node = SBDPNode(args.id, args.port, args.host)
    node.run()


if __name__ == "__main__":
    main()
