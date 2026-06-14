#!/usr/bin/env python3
"""
SBDP — Satellite Backhaul Discovery Protocol
=============================================
逐跳独立处理的协议实现。

每个节点是一个独立的协议引擎：
  收到 bytes → unpack → 验证 CRC → 状态机决策 → 更新字段 → repack → 转发

不做全局"transmit"函数，每个中间节点只看到自己的输入和输出。
"""

import struct
import random
import math
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Dict, List, Optional, Tuple

# ════════════════════════════════════════════════
#  协议常量
# ════════════════════════════════════════════════

SBDP_VERSION = 0x1
SBDP_HEADER_SIZE = 16
SBDP_TLV_HEADER_SIZE = 4


class MsgType(IntEnum):
    CAPACITY_ADV = 0x1
    CAPACITY_REQ = 0x2
    CAPACITY_ACK = 0x3
    LINK_PROBE   = 0x4


class TlvType(IntEnum):
    SOURCE_ID       = 0x0001
    DEST_ID         = 0x0002
    BOTTLENECK_LINK = 0x0003
    PATH_TRACE      = 0x0004
    TIMESTAMP       = 0x0005
    VALIDITY_PERIOD = 0x0006
    ADVERTISER_ID   = 0x0007


class Flag(IntEnum):
    NONE             = 0x00
    REQUEST_ACK      = 0x01
    STALE_ALLOWED    = 0x02
    CONGESTION_MARK  = 0x04
    PATH_CHANGE      = 0x08
    EMERGENCY_UPDATE = 0x10


# ════════════════════════════════════════════════
#  CRC-16-CCITT
# ════════════════════════════════════════════════

_CRC_TABLE = None

def _make_crc_table():
    global _CRC_TABLE
    if _CRC_TABLE is None:
        _CRC_TABLE = []
        for i in range(256):
            crc = i << 8
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            _CRC_TABLE.append(crc & 0xFFFF)
    return _CRC_TABLE

def crc16(data: bytes) -> int:
    table = _make_crc_table()
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) ^ table[((crc >> 8) ^ b) & 0xFF]) & 0xFFFF
    return crc


# ════════════════════════════════════════════════
#  TLV 编解码
# ════════════════════════════════════════════════

@dataclass
class TLV:
    type_: int
    value: bytes

    def pack(self) -> bytes:
        return struct.pack('!HH', self.type_, len(self.value)) + self.value

    @staticmethod
    def unpack(data: bytes, offset: int) -> Tuple['TLV', int]:
        t, l = struct.unpack_from('!HH', data, offset)
        v = data[offset + 4 : offset + 4 + l]
        return TLV(type_=t, value=v), 4 + l

    def value_str(self) -> str:
        return self.value.decode('utf-8')

    @staticmethod
    def str_tlv(t: TlvType, s: str) -> 'TLV':
        return TLV(t.value, s.encode('utf-8'))

    @staticmethod
    def f64_tlv(t: TlvType, v: float) -> 'TLV':
        return TLV(t.value, struct.pack('!d', v))

    @staticmethod
    def f32_tlv(t: TlvType, v: float) -> 'TLV':
        return TLV(t.value, struct.pack('!f', v))


# ════════════════════════════════════════════════
#  SBDP 报头 (16 字节，二进制格式)
# ════════════════════════════════════════════════

@dataclass
class SBDPHeader:
    """
    固定报头 16 字节:

     0       1       2       3       4       5       6       7
    +-------+-------+-------+-------+-------+-------+-------+-------+
    | Ver(4)|Type(4)|  Flags(8)  |     Total Length (16)           |
    +-------+-------+-------+-------+-------+-------+-------+-------+
    |    Sequence Number (16)     |  TTL(8)   |  Hop Count(8)      |
    +-------+-------+-------+-------+-------+-------+-------+-------+
    |              Backhaul Capacity (float32, 32 bits)             |
    +-------+-------+-------+-------+-------+-------+-------+-------+
    |      Checksum (16)          |        Reserved (16)           |
    +-------+-------+-------+-------+-------+-------+-------+-------+
    """
    msg_type: int = MsgType.CAPACITY_ADV
    flags: int = Flag.NONE
    total_length: int = SBDP_HEADER_SIZE
    seq_num: int = 0
    ttl: int = 20
    hop_count: int = 0
    backhaul_capacity: float = float('inf')
    checksum: int = 0
    _reserved: int = 0

    def pack(self) -> bytes:
        byte0 = (SBDP_VERSION << 4) | (self.msg_type & 0x0F)
        pre = struct.pack('!BBHHBBf',
            byte0, self.flags, self.total_length,
            self.seq_num, self.ttl, self.hop_count,
            self.backhaul_capacity)
        self.checksum = crc16(pre)
        return struct.pack('!BBHHBBfHH',
            byte0, self.flags, self.total_length,
            self.seq_num, self.ttl, self.hop_count,
            self.backhaul_capacity, self.checksum, self._reserved)

    @staticmethod
    def unpack(data: bytes) -> 'SBDPHeader':
        byte0, flags, total_len, seq, ttl, hop, cap, csum, reserved = \
            struct.unpack('!BBHHBBfHH', data[:SBDP_HEADER_SIZE])
        version = (byte0 >> 4) & 0x0F
        if version != SBDP_VERSION:
            raise ValueError(f"Bad version {version}")
        pre = struct.pack('!BBHHBBf', byte0, flags, total_len, seq, ttl, hop, cap)
        if crc16(pre) != csum:
            raise ValueError(f"CRC mismatch: got 0x{csum:04x}")
        return SBDPHeader(msg_type=byte0 & 0x0F, flags=flags, total_length=total_len,
                          seq_num=seq, ttl=ttl, hop_count=hop,
                          backhaul_capacity=cap, checksum=csum, _reserved=reserved)

    def summary(self) -> str:
        names = {1: "ADV", 2: "REQ", 3: "ACK", 4: "PROBE"}
        bw = f"{self.backhaul_capacity:.1f}" if self.backhaul_capacity != float('inf') else "INF"
        return (f"type={names.get(self.msg_type, '?')} seq=#{self.seq_num} "
                f"ttl={self.ttl} hop={self.hop_count} "
                f"backhaul={bw}Mbps crc=0x{self.checksum:04x}")


# ════════════════════════════════════════════════
#  SBDP 报文 = 报头 + TLV 链
# ════════════════════════════════════════════════

@dataclass
class SBDPMessage:
    header: SBDPHeader
    tlvs: List[TLV] = field(default_factory=list)

    def pack(self) -> bytes:
        tlv_bytes = b''.join(t.pack() for t in self.tlvs)
        self.header.total_length = SBDP_HEADER_SIZE + len(tlv_bytes)
        return self.header.pack() + tlv_bytes

    @staticmethod
    def unpack(data: bytes) -> 'SBDPMessage':
        hdr = SBDPHeader.unpack(data[:SBDP_HEADER_SIZE])
        tlvs = []
        off = SBDP_HEADER_SIZE
        end = hdr.total_length
        while off < end:
            tlv, consumed = TLV.unpack(data, off)
            tlvs.append(tlv)
            off += consumed
        return SBDPMessage(header=hdr, tlvs=tlvs)

    def get_tlv(self, t: TlvType) -> Optional[TLV]:
        for tlv in self.tlvs:
            if tlv.type_ == t.value:
                return tlv
        return None

    def set_tlv(self, tlv: TLV):
        self.tlvs = [x for x in self.tlvs if x.type_ != tlv.type_]
        self.tlvs.append(tlv)

    def hexdump(self) -> str:
        raw = self.pack()
        lines = [
            f"┌── SBDP Message wire bytes ({len(raw)}B) ".ljust(67) + "┐",
        ]
        for i in range(0, len(raw), 16):
            chunk = raw[i:i+16]
            hx = ' '.join(f'{b:02x}' for b in chunk)
            asc = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
            lines.append(f"│ {i:04x}  {hx:<48} {asc:<14} │")
        lines.append("└" + "─" * 65 + "┘")
        return '\n'.join(lines)

    def field_diff(self, other: 'SBDPMessage') -> List[str]:
        """对比两个报文的差异（用于展示逐跳变化）"""
        diffs = []
        h1, h2 = self.header, other.header
        if h1.backhaul_capacity != h2.backhaul_capacity:
            diffs.append(f"  backhaul_capacity: {h1.backhaul_capacity:.1f} → {h2.backhaul_capacity:.1f} Mbps")
        if h1.hop_count != h2.hop_count:
            diffs.append(f"  hop_count:         {h1.hop_count} → {h2.hop_count}")
        if h1.ttl != h2.ttl:
            diffs.append(f"  ttl:               {h1.ttl} → {h2.ttl}")
        if h1.checksum != h2.checksum:
            diffs.append(f"  checksum:          0x{h1.checksum:04x} → 0x{h2.checksum:04x}")

        old_link = self.get_tlv(TlvType.BOTTLENECK_LINK)
        new_link = other.get_tlv(TlvType.BOTTLENECK_LINK)
        if old_link and new_link and old_link.value_str() != new_link.value_str():
            diffs.append(f"  bottleneck_link:   \"{old_link.value_str()}\" → \"{new_link.value_str()}\"")

        old_path = self.get_tlv(TlvType.PATH_TRACE)
        new_path = other.get_tlv(TlvType.PATH_TRACE)
        if old_path and new_path and old_path.value_str() != new_path.value_str():
            diffs.append(f"  path_trace:        \"{old_path.value_str()}\" → \"{new_path.value_str()}\"")
        return diffs


# ════════════════════════════════════════════════
#  协议引擎 — 每个节点独立运行
# ════════════════════════════════════════════════

class SBDPEngine:
    """
    SBDP 协议引擎。

    每个网络节点持有一个 Engine 实例。
    节点收到的只有原始字节流，引擎负责：
      1. 解包 → 2. 校验 → 3. 决策 → 4. 修改 → 5. 打包 → 6. 转发
    """

    def __init__(self, node_id: str):
        self.node_id = node_id
        self.seq = 0
        self.neighbors: Dict[str, float] = {}    # neighbor → bw
        self.routing: Dict[str, str] = {}        # dst → next_hop
        self.backhaul_view: float = float('inf')
        self.bottleneck_link: str = ""
        self.last_advertised: Dict[str, float] = {}
        self.processed_count: int = 0

    def next_seq(self) -> int:
        self.seq += 1
        return self.seq

    # ── 作为 ADVERTISER：构造报文 ──

    def build_advertisement(self, dst: str, backhaul_bw: float,
                            bottleneck: str, ts: float) -> SBDPMessage:
        """卫星构造一个 CAPACITY_ADV 报文"""
        hdr = SBDPHeader(
            msg_type=MsgType.CAPACITY_ADV,
            seq_num=self.next_seq(),
            backhaul_capacity=backhaul_bw,
            ttl=20,
        )
        msg = SBDPMessage(header=hdr, tlvs=[
            TLV.str_tlv(TlvType.SOURCE_ID, self.node_id),
            TLV.str_tlv(TlvType.DEST_ID, dst),
            TLV.str_tlv(TlvType.BOTTLENECK_LINK, bottleneck),
            TLV.str_tlv(TlvType.ADVERTISER_ID, self.node_id),
            TLV.str_tlv(TlvType.PATH_TRACE, self.node_id),
            TLV.f64_tlv(TlvType.TIMESTAMP, ts),
            TLV.f32_tlv(TlvType.VALIDITY_PERIOD, 10.0),
        ])
        return SBDPMessage.unpack(msg.pack())  # 强制计算 checksum

    # ── 作为中间节点：处理收到的报文 ──

    def process_incoming(self, raw_bytes: bytes) -> Optional[Tuple[str, 'SBDPMessage']]:
        """
        协议核心：接收原始字节流，处理后决定转发到哪里。

        返回 (next_hop_id, outbound_message) 或 None（到达目的地/丢弃）。
        """
        self.processed_count += 1

        # Step 1: 解包 & 校验
        try:
            msg = SBDPMessage.unpack(raw_bytes)
        except ValueError as e:
            print(f"  ✗ [{self.node_id}] unpack/CRC failed: {e}")
            return None

        in_msg = msg  # 保存入站报文用于对比

        # Step 2: TTL 检查
        if msg.header.ttl <= 0:
            print(f"  ✗ [{self.node_id}] TTL expired, dropping")
            return None

        # Step 3: 是否到达目的地？
        dst = msg.get_tlv(TlvType.DEST_ID)
        dst_str = dst.value_str() if dst else ""
        if dst_str == self.node_id:
            # 到达目的地，更新本地视图
            self.backhaul_view = msg.header.backhaul_capacity
            self.bottleneck_link = (msg.get_tlv(TlvType.BOTTLENECK_LINK) or
                                    TLV.str_tlv(TlvType.BOTTLENECK_LINK, "?")).value_str()
            return None  # 不转发

        # Step 4: 查找下一跳
        next_hop = self.routing.get(dst_str)
        if next_hop is None:
            print(f"  ✗ [{self.node_id}] no route to {dst_str}")
            return None

        out_bw = self.neighbors.get(next_hop, 0)
        if out_bw == 0:
            print(f"  ✗ [{self.node_id}] no link to {next_hop}")
            return None

        # Step 5: 判断是否为回传链路，决定是否更新瓶颈
        is_user_hop = 'User' in self.node_id or 'User' in next_hop

        if not is_user_hop and out_bw < msg.header.backhaul_capacity:
            # 回传链路，且比当前瓶颈更窄 → 更新
            msg.header.backhaul_capacity = out_bw
            new_bottleneck = f"{self.node_id} → {next_hop} ({out_bw:.0f} Mbps)"
            msg.set_tlv(TLV.str_tlv(TlvType.BOTTLENECK_LINK, new_bottleneck))

        # Step 6: 追加路径追踪
        path_tlv = msg.get_tlv(TlvType.PATH_TRACE)
        old_path = path_tlv.value_str() if path_tlv else ""
        msg.set_tlv(TLV.str_tlv(TlvType.PATH_TRACE, f"{old_path}→{next_hop}" if old_path else f"{self.node_id}→{next_hop}"))

        # Step 7: 更新逐跳计数器
        msg.header.ttl -= 1
        msg.header.hop_count += 1

        # Step 8: 重新序列化（计算新 CRC）
        out_msg = SBDPMessage.unpack(msg.pack())

        # 打印每跳变化
        self._log_hop(in_msg, out_msg, next_hop, out_bw, is_user_hop)

        return (next_hop, out_msg)

    def _log_hop(self, in_msg: SBDPMessage, out_msg: SBDPMessage,
                 next_hop: str, out_bw: float, is_user_hop: bool):
        """逐跳日志：展示报头如何变化"""
        tag = "[ACCESS]" if is_user_hop else "[BACKHAUL]"
        print(f"  ┌─ [{self.node_id}] ─────────────────────────────")
        print(f"  │ in:  {in_msg.header.summary()}")
        print(f"  │ out: {out_msg.header.summary()}")
        diffs = in_msg.field_diff(out_msg)
        if diffs:
            for d in diffs:
                print(f"  │ Δ {d}")
            if is_user_hop:
                print(f"  │   (user access link {out_bw:.0f}Mbps — backhaul NOT updated)")
            else:
                print(f"  │   (backhaul link {out_bw:.0f}Mbps)")
        else:
            print(f"  │   (forwarding {out_bw:.0f}Mbps {tag} — no backhaul change)")
        print(f"  └→ {next_hop}")


# ════════════════════════════════════════════════
#  网络拓扑 & 仿真
# ════════════════════════════════════════════════

class Network:
    """
    卫星网络 — 每步以逐跳方式传递 SBDP 报文。
    没有全局 transmit 函数，每个节点独立处理收到的字节流。
    """

    def __init__(self):
        self.nodes: Dict[str, SBDPEngine] = {}
        self.time: float = 0.0

    def add_node(self, nid: str) -> SBDPEngine:
        engine = SBDPEngine(nid)
        self.nodes[nid] = engine
        return engine

    def add_link(self, a: str, b: str, bw: float):
        self.nodes[a].neighbors[b] = bw
        self.nodes[b].neighbors[a] = bw

    def recompute_routing(self):
        """Dijkstra：所有节点独立计算路由表"""
        all_ids = list(self.nodes.keys())
        for src in all_ids:
            dist = {n: float('inf') for n in all_ids}
            prev = {n: None for n in all_ids}
            dist[src] = 0
            unvisited = set(all_ids)
            while unvisited:
                u = min(unvisited, key=lambda n: dist[n])
                if dist[u] == float('inf'):
                    break
                unvisited.remove(u)
                for v in self.nodes[u].neighbors:
                    if v in unvisited and dist[u] + 1 < dist[v]:
                        dist[v] = dist[u] + 1
                        prev[v] = u
            rt = {}
            for dst in all_ids:
                if dst == src:
                    continue
                curr = dst
                while prev.get(curr) and prev[curr] != src:
                    curr = prev[curr]
                if prev.get(curr) == src:
                    rt[dst] = curr
            self.nodes[src].routing = rt

    def deliver_hop_by_hop(self, msg: SBDPMessage, src: str, dst: str) -> bool:
        """
        逐跳投递。每个节点独立处理收到的字节流。
        包括目的地节点——它也要 unpack、CRC 验证、更新本地视图。
        """
        current_id = src
        wire_bytes = msg.pack()
        max_hops = 20

        for _ in range(max_hops):
            engine = self.nodes.get(current_id)
            if engine is None:
                return False

            # ★ 每个节点（包含目的地）都调用 process_incoming
            result = engine.process_incoming(wire_bytes)

            if current_id == dst:
                # 目的地已处理，投递成功
                return result is None  # None 表示"不转发"=正常终止

            if result is None:
                return False  # 中间节点丢弃了包

            next_hop, out_msg = result
            wire_bytes = out_msg.pack()
            current_id = next_hop

        return False

    def fluctuate(self):
        """对称链路波动"""
        seen = set()
        for nid, engine in self.nodes.items():
            for nb, old in list(engine.neighbors.items()):
                key = tuple(sorted([nid, nb]))
                if key in seen:
                    continue
                seen.add(key)

                a_sat, b_sat = 'SAT' in nid, 'SAT' in nb
                if a_sat and b_sat:
                    factor = 1.0 + random.uniform(-0.25, 0.25)
                    floor = 80
                elif ('GS' in nid) != ('GS' in nb):
                    factor = 1.0 + random.uniform(-0.35, 0.35)
                    floor = 50
                else:
                    factor = 1.0 + random.uniform(-0.10, 0.10)
                    floor = 50

                new_bw = round(max(floor, old * factor), 1)
                self.nodes[nid].neighbors[nb] = new_bw
                self.nodes[nb].neighbors[nid] = new_bw

        self.recompute_routing()
        # 清零广告记忆，强制下次推送
        for eng in self.nodes.values():
            if 'SAT' in eng.node_id:
                eng.last_advertised.clear()

    def run(self, steps=40, monitor_interval=4, topo_interval=10):
        print("=" * 70)
        print("  SBDP 逐跳独立处理 — Per-Hop Protocol Engine Demo")
        print("=" * 70)
        print(f"  每个节点独立: unpack → CRC → 决策 → 修改 → repack → forward")
        print()

        self._print_topo()
        self.recompute_routing()

        for step in range(steps):
            self.time = step

            if step > 0 and step % topo_interval == 0:
                self.fluctuate()
                print(f"\n  {'═'*60}")
                print(f"  ══ t={step}: topology changed ══")
                self._print_topo()

            if step % monitor_interval == 0:
                self._advertise_and_deliver(step)

        self._report()

    def _advertise_and_deliver(self, step: int):
        """每颗卫星构造 SBDP 报文，然后逐跳传递"""
        gs = [nid for nid in self.nodes if 'GS' in nid][0]

        for nid, engine in self.nodes.items():
            if 'SAT' not in nid:
                continue

            # 计算本卫星到地面站的回传瓶颈
            backhaul, bottleneck = self._calc_backhaul(nid, gs)
            if backhaul is None:
                continue

            # 推给下游卫星和用户
            targets = [nb for nb in engine.neighbors
                       if ('SAT' in nb and self._is_downstream(nid, nb, gs))
                       or 'User' in nb]

            for dst in targets:
                prev = engine.last_advertised.get(dst, -1)
                if prev < 0 or abs(backhaul - prev) / max(prev, 1) > 0.03:
                    msg = engine.build_advertisement(dst, backhaul, bottleneck, step)
                    engine.last_advertised[dst] = backhaul

                    print(f"\n  ══ [{nid}] builds CAPACITY_ADV → {dst} ══")
                    print(f"  {msg.header.summary()}")
                    if step == 0 and dst.startswith('User'):
                        print(msg.hexdump())

                    self.deliver_hop_by_hop(msg, nid, dst)

    def _calc_backhaul(self, sat_id: str, gs_id: str) -> Tuple[Optional[float], str]:
        """计算卫星到地面站路径的回传瓶颈 — 每跳用各节点的路由表"""
        current = sat_id
        path = [current]
        visited = set()
        while current != gs_id and current not in visited and len(path) < 20:
            visited.add(current)
            nxt = self.nodes[current].routing.get(gs_id)  # ★ 用当前节点的路由表
            if nxt is None:
                return None, ""
            path.append(nxt)
            current = nxt

        if current != gs_id:
            return None, ""

        bottleneck = float('inf')
        bottleneck_link = ""
        for i in range(len(path) - 1):
            bw = self.nodes[path[i]].neighbors.get(path[i+1], 0)
            if 0 < bw < bottleneck:
                bottleneck = bw
                bottleneck_link = f"{path[i]} → {path[i+1]} ({bw:.0f} Mbps)"

        return (bottleneck if bottleneck != float('inf') else None), bottleneck_link

    def _is_downstream(self, sat: str, nb: str, gs: str) -> bool:
        """nb 是否在 sat 的下游（离 GS 更远）"""
        def hops_to_gs(nid):
            curr, cnt = nid, 0
            visited = set()
            while curr != gs and curr not in visited and cnt < 20:
                visited.add(curr)
                nxt = self.nodes[curr].routing.get(gs)  # ★ 逐个节点查路由表
                if nxt is None:
                    return 999
                curr = nxt
                cnt += 1
            return cnt if curr == gs else 999
        return hops_to_gs(nb) > hops_to_gs(sat)

    def _print_topo(self):
        print(f"  ── Topology (t={self.time:.0f}) ──")
        seen = set()
        for nid, engine in sorted(self.nodes.items()):
            for nb, bw in sorted(engine.neighbors.items()):
                key = tuple(sorted([nid, nb]))
                if key in seen:
                    continue
                seen.add(key)
                tag = "[ISL]" if 'SAT' in nid and 'SAT' in nb else \
                      "[SGL]" if 'GS' in nid or 'GS' in nb else "[ACC]"
                print(f"    {nid} ↔ {nb} {tag} {bw:>8.0f} Mbps")

    def _report(self):
        print(f"\n\n{'='*70}")
        print(f"  Session Report — Per-Hop SBDP Engine Trace")
        print(f"{'='*70}")
        for nid in sorted(self.nodes):
            engine = self.nodes[nid]
            if 'User' in nid:
                print(f"  {nid}: backhaul_view = {engine.backhaul_view:.1f} Mbps")
                print(f"         bottleneck    = {engine.bottleneck_link}")
                print(f"         packets_processed = {engine.processed_count}")
            elif 'SAT' in nid:
                print(f"  {nid}: packets_processed = {engine.processed_count}, "
                      f"advertisements_sent = {engine.seq}")


if __name__ == "__main__":
    random.seed(42)

    net = Network()
    for nid in ["GS", "SAT-B", "SAT-A", "User-1", "User-2", "User-3"]:
        net.add_node(nid)

    net.add_link("GS",    "SAT-B", 350)
    net.add_link("SAT-B", "SAT-A", 600)
    net.add_link("SAT-A", "User-1", 150)
    net.add_link("SAT-A", "User-3", 200)
    net.add_link("SAT-B", "User-2", 400)

    net.run(steps=40, monitor_interval=4, topo_interval=10)
