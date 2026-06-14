#!/usr/bin/env python3
"""
SBDP — Satellite Backhaul Discovery Protocol
=============================================
纯协议模块：二进制报头、TLV 编解码、CRC 校验、协议引擎。

与网络拓扑无关——引擎通过参数接收 routing 和 neighbors，
可嵌入任何传输层（UDP、QUIC、TCP option）。
"""

import struct
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
    CAPACITY_ADV      = 0x1   # gNB→UE: 回传通告
    CAPACITY_REQ      = 0x2   # gNB→路由器: N2 查询请求
    CAPACITY_ACK      = 0x3   # 路由器→gNB: N2 查询应答
    LINK_PROBE        = 0x4   # 链路探测
    CAPACITY_MIGRATE  = 0x5   # gNB→gNB: N3 切换迁移请求
    CAPACITY_CONFIRM  = 0x6   # gNB→gNB: N3 迁移确认/拒绝


class TlvType(IntEnum):
    SOURCE_ID       = 0x0001
    DEST_ID         = 0x0002
    BOTTLENECK_LINK = 0x0003
    PATH_TRACE      = 0x0004
    TIMESTAMP       = 0x0005
    VALIDITY_PERIOD = 0x0006
    ADVERTISER_ID   = 0x0007
    # ── N2 ④ 链路 QoS ──
    LINK_LATENCY    = 0x000A   # float32, ms (逐跳累加)
    LINK_JITTER     = 0x000B   # float32, ms (逐跳取max)
    LINK_PKT_LOSS   = 0x000C   # float32, 比率 (逐跳累积: 1-∏(1-pi))
    # ── N2 ⑤ 用户/业务属性 ──
    TRAFFIC_CLASS   = 0x000D   # string: "realtime"/"bulk"/"best-effort"
    USER_PRIORITY   = 0x000E   # uint8, 0=best-effort ~ 7=network-control
    AVAIL_CAPACITY  = 0x000F   # float32, Mbps, 瓶颈链路剩余可用容量 (待: 需负载追踪)
    # ── N3 gNB 间协同 ──
    GNB_LOAD        = 0x0011   # float32, 0.0~1.0, gNB 当前负载比例
    CHANNEL_QUALITY = 0x0012   # float32, dB, 空口 SINR
    MIGRATE_TARGET  = 0x0013   # string, N3 迁移推荐目标星名
    MIGRATE_REASON  = 0x0014   # string, 迁移原因 "overload"/"signal_decay"/"backhaul_degrade"
    UE_LIST         = 0x0015   # string, 逗号分隔的 UE ID 列表
    RESERVED_BW     = 0x0016   # float32, Mbps, 为目标 UE 预留的带宽


class Flag(IntEnum):
    NONE             = 0x00
    REQUEST_ACK      = 0x01
    STALE_ALLOWED    = 0x02
    CONGESTION_MARK  = 0x04
    PATH_CHANGE      = 0x08
    EMERGENCY_UPDATE = 0x10


# ════════════════════════════════════════════════
#  LinkQoS — 单跳链路质量 (N2 ④⑤)
# ════════════════════════════════════════════════

@dataclass
class LinkQoS:
    """单条链路的服务质量参数"""
    latency_ms: float = 0.0       # 链路时延 (ms)
    jitter_ms: float = 0.0        # 链路抖动 (ms)
    pkt_loss: float = 0.0         # 丢包率 (0.0 ~ 1.0)

    def pack(self) -> bytes:
        return struct.pack('!fff', self.latency_ms, self.jitter_ms, self.pkt_loss)

    @staticmethod
    def unpack(data: bytes) -> 'LinkQoS':
        lat, jit, loss = struct.unpack('!fff', data[:12])
        return LinkQoS(latency_ms=lat, jitter_ms=jit, pkt_loss=loss)


# ════════════════════════════════════════════════
#  GnBState — gNB 接入侧状态 (N3 共享)
# ════════════════════════════════════════════════

@dataclass
class GnBState:
    """一颗卫星 gNB 的接入侧状态，通过 N3 在 gNB 间共享"""
    load_pct: float = 0.0             # 负载比例 0.0~1.0
    channel_sinr_db: float = 30.0     # 空口 SINR (dB)
    ue_count: int = 0                 # 当前接入用户数
    avail_access_bw: float = 0.0      # 空口剩余带宽 (Mbps)

    def is_overloaded(self, threshold: float = 0.85) -> bool:
        return self.load_pct > threshold

    def summary(self) -> str:
        return (f"load={self.load_pct*100:.0f}% sinr={self.channel_sinr_db:.0f}dB "
                f"ues={self.ue_count} access_bw={self.avail_access_bw:.0f}M")


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

    def value_f32(self) -> float:
        return struct.unpack('!f', self.value)[0]

    def value_f64(self) -> float:
        return struct.unpack('!d', self.value)[0]

    def value_u8(self) -> int:
        return struct.unpack('!B', self.value)[0]

    @staticmethod
    def str_tlv(t: TlvType, s: str) -> 'TLV':
        return TLV(t.value, s.encode('utf-8'))

    @staticmethod
    def f64_tlv(t: TlvType, v: float) -> 'TLV':
        return TLV(t.value, struct.pack('!d', v))

    @staticmethod
    def f32_tlv(t: TlvType, v: float) -> 'TLV':
        return TLV(t.value, struct.pack('!f', v))

    @staticmethod
    def u8_tlv(t: TlvType, v: int) -> 'TLV':
        return TLV(t.value, struct.pack('!B', v))

    @staticmethod
    def qos_tlv(t: TlvType, qos: 'LinkQoS') -> 'TLV':
        return TLV(t.value, qos.pack())


# ════════════════════════════════════════════════
#  SBDP 报头 (16 字节固定)
# ════════════════════════════════════════════════

@dataclass
class SBDPHeader:
    """
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
        names = {1: "ADV", 2: "REQ", 3: "ACK", 4: "PROBE", 5: "MIGRATE", 6: "CONFIRM"}
        bw = f"{self.backhaul_capacity:.1f}" if self.backhaul_capacity != float('inf') else "INF"
        return (f"type={names.get(self.msg_type, '?')} seq=#{self.seq_num} "
                f"ttl={self.ttl} hop={self.hop_count} "
                f"backhaul={bw}Mbps crc=0x{self.checksum:04x}")


# ════════════════════════════════════════════════
#  SBDP 完整报文
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
        lines = [f"┌── SBDP {len(raw)}B " + "─" * (60 - len(f"┌── SBDP {len(raw)}B ")) + "┐"]
        for i in range(0, len(raw), 16):
            chunk = raw[i:i+16]
            hx = ' '.join(f'{b:02x}' for b in chunk)
            asc = ''.join(chr(b) if 32 <= b < 127 else '.' for b in chunk)
            lines.append(f"│ {i:04x}  {hx:<48} {asc:<14} │")
        lines.append("└" + "─" * 65 + "┘")
        return '\n'.join(lines)

    def field_diff(self, other: 'SBDPMessage') -> List[str]:
        diffs = []
        h1, h2 = self.header, other.header
        if h1.backhaul_capacity != h2.backhaul_capacity:
            diffs.append(f"backhaul: {h1.backhaul_capacity:.1f}→{h2.backhaul_capacity:.1f}Mbps")
        if h1.hop_count != h2.hop_count:
            diffs.append(f"hop: {h1.hop_count}→{h2.hop_count}")
        if h1.ttl != h2.ttl:
            diffs.append(f"ttl: {h1.ttl}→{h2.ttl}")
        if h1.checksum != h2.checksum:
            diffs.append(f"crc: 0x{h1.checksum:04x}→0x{h2.checksum:04x}")
        old_link = self.get_tlv(TlvType.BOTTLENECK_LINK)
        new_link = other.get_tlv(TlvType.BOTTLENECK_LINK)
        if old_link and new_link and old_link.value_str() != new_link.value_str():
            diffs.append(f"bottleneck: \"{old_link.value_str()}\"→\"{new_link.value_str()}\"")
        # N2 ④: QoS changes
        for t, label in [(TlvType.LINK_LATENCY, "lat"),
                         (TlvType.LINK_JITTER, "jit"),
                         (TlvType.LINK_PKT_LOSS, "loss")]:
            old_v = self.get_tlv(t)
            new_v = other.get_tlv(t)
            if old_v and new_v and abs(old_v.value_f32() - new_v.value_f32()) > 0.001:
                diffs.append(f"{label}: {old_v.value_f32():.1f}→{new_v.value_f32():.1f}")
# N2 ⑤: available capacity
        old_a = self.get_tlv(TlvType.AVAIL_CAPACITY)
        new_a = other.get_tlv(TlvType.AVAIL_CAPACITY)
        if old_a and new_a and abs(old_a.value_f32() - new_a.value_f32()) > 0.1:
            diffs.append(f"avail: {old_a.value_f32():.0f}→{new_a.value_f32():.0f}Mbps")
        return diffs


# ════════════════════════════════════════════════
#  协议引擎 (与拓扑解耦)
# ════════════════════════════════════════════════

class SBDPEngine:
    """
    SBDP 协议引擎。不持有拓扑——routing 和 neighbors 由调用者注入。
    可以在 UDP socket、QUIC stream、或内核模块中使用。
    """

    def __init__(self, node_id: str):
        self.node_id = node_id
        self.seq = 0
        self.backhaul_view: float = float('inf')
        self.bottleneck_link: str = ""
        self.processed_count: int = 0
        self.sent_count: int = 0

    def next_seq(self) -> int:
        self.seq += 1
        return self.seq

    # ── 构造报文 ──

    def build_advertisement(self, dst: str, backhaul_bw: float,
                            bottleneck: str, ts: float,
                            out_link_qos: 'LinkQoS' = None,
                            traffic_class: str = "",
                            user_priority: int = 0,
                            gnb_state: 'GnBState' = None) -> SBDPMessage:
        """卫星构造 CAPACITY_ADV 报文（含 N2+N3 完整信息）

        N2 五项:
          ① backhaul_bw   — 回传瓶颈带宽 (Header 固定字段)
          ② bottleneck    — 瓶颈链路定位 (BOTTLENECK_LINK TLV)
          ③ path trace    — 整条路径 (PATH_TRACE TLV)
          ④ out_link_qos  — 本星出口链路 QoS (LINK_LATENCY/JITTER/PKT_LOSS TLV)
          ⑤ traffic_class — 业务类别 / user_priority — 用户优先级
        N3 捎带:
          gnb_state       — gNB 接入侧状态 (GNB_LOAD, CHANNEL_QUALITY)
                            UE 收到后可直接综合回传+接入评分
        """
        hdr = SBDPHeader(
            msg_type=MsgType.CAPACITY_ADV,
            seq_num=self.next_seq(),
            backhaul_capacity=backhaul_bw,
            ttl=20,
        )
        tlvs = [
            TLV.str_tlv(TlvType.SOURCE_ID, self.node_id),
            TLV.str_tlv(TlvType.DEST_ID, dst),
            TLV.str_tlv(TlvType.BOTTLENECK_LINK, bottleneck),
            TLV.str_tlv(TlvType.ADVERTISER_ID, self.node_id),
            TLV.str_tlv(TlvType.PATH_TRACE, self.node_id),
            TLV.f64_tlv(TlvType.TIMESTAMP, ts),
            TLV.f32_tlv(TlvType.VALIDITY_PERIOD, 10.0),
        ]
        # ── N2 ④: 出口链路 QoS ──
        if out_link_qos is not None:
            tlvs.append(TLV.f32_tlv(TlvType.LINK_LATENCY, out_link_qos.latency_ms))
            tlvs.append(TLV.f32_tlv(TlvType.LINK_JITTER, out_link_qos.jitter_ms))
            tlvs.append(TLV.f32_tlv(TlvType.LINK_PKT_LOSS, out_link_qos.pkt_loss))
        # ── N2 ⑤: 业务类别 + 用户优先级 ──
        if traffic_class:
            tlvs.append(TLV.str_tlv(TlvType.TRAFFIC_CLASS, traffic_class))
        tlvs.append(TLV.u8_tlv(TlvType.USER_PRIORITY, user_priority))
        # ── N3: gNB 接入侧状态 (捎带在 ADV 里, UE 综合评分用) ──
        if gnb_state is not None:
            tlvs.append(TLV.f32_tlv(TlvType.GNB_LOAD, gnb_state.load_pct))
            tlvs.append(TLV.f32_tlv(TlvType.CHANNEL_QUALITY, gnb_state.channel_sinr_db))

        msg = SBDPMessage(header=hdr, tlvs=tlvs)
        self.sent_count += 1
        return SBDPMessage.unpack(msg.pack())

    # ── 处理收到的报文 ──

    def process_incoming(self, raw_bytes: bytes,
                         routing: Dict[str, str],
                         neighbors: Dict[str, float],
                         link_qos: Dict[str, 'LinkQoS'] = None) -> Optional[Tuple[str, bytes, str]]:
        """
        协议核心：收到原始字节流，处理后返回转发信息。

        参数:
          raw_bytes: 收到的 SBDP 二进制报文
          routing:   当前节点的路由表 {dst → next_hop}
          neighbors: 当前节点的邻居带宽 {neighbor_id → bw_mbps}
          link_qos:  当前节点的出口链路 QoS {neighbor_id → LinkQoS} (N2 ④)

        返回:
          None = 到达目的地或丢弃
          (next_hop_id, outbound_bytes, log_line) = 需要转发
        """
        self.processed_count += 1

        # Step 1: 解包 & CRC 校验
        try:
            msg = SBDPMessage.unpack(raw_bytes)
        except ValueError as e:
            return None  # 丢弃损坏的包

        in_msg = msg

        # Step 2: TTL 检查
        if msg.header.ttl <= 0:
            return None

        # Step 3: 到达目的地？
        dst_tlv = msg.get_tlv(TlvType.DEST_ID)
        dst_str = dst_tlv.value_str() if dst_tlv else ""
        if dst_str == self.node_id:
            self.backhaul_view = msg.header.backhaul_capacity
            bl = msg.get_tlv(TlvType.BOTTLENECK_LINK)
            self.bottleneck_link = bl.value_str() if bl else "?"
            return None  # 终止

        # Step 4: 查找下一跳
        next_hop = routing.get(dst_str)
        if next_hop is None:
            return None

        out_bw = neighbors.get(next_hop, 0)
        if out_bw == 0:
            return None

        # Step 5: 回传链路 → 更新瓶颈 (N2 ①)
        is_user_hop = 'User' in self.node_id or 'User' in next_hop

        if not is_user_hop and out_bw < msg.header.backhaul_capacity:
            msg.header.backhaul_capacity = out_bw
            new_bl = f"{self.node_id} → {next_hop} ({out_bw:.0f} Mbps)"
            msg.set_tlv(TLV.str_tlv(TlvType.BOTTLENECK_LINK, new_bl))

        # Step 5b: N2 ④ — 逐跳更新链路 QoS
        out_qos = link_qos.get(next_hop) if link_qos else None
        qos_log = ""
        if not is_user_hop and out_qos is not None:
            # 时延: 累加
            lat_tlv = msg.get_tlv(TlvType.LINK_LATENCY)
            prev_lat = lat_tlv.value_f32() if lat_tlv else 0.0
            new_lat = prev_lat + out_qos.latency_ms
            msg.set_tlv(TLV.f32_tlv(TlvType.LINK_LATENCY, new_lat))

            # 抖动: 取 max
            jit_tlv = msg.get_tlv(TlvType.LINK_JITTER)
            prev_jit = jit_tlv.value_f32() if jit_tlv else 0.0
            new_jit = max(prev_jit, out_qos.jitter_ms)
            msg.set_tlv(TLV.f32_tlv(TlvType.LINK_JITTER, new_jit))

            # 丢包率: 累积 1-∏(1-pi)
            loss_tlv = msg.get_tlv(TlvType.LINK_PKT_LOSS)
            prev_loss = loss_tlv.value_f32() if loss_tlv else 0.0
            new_loss = 1.0 - (1.0 - prev_loss) * (1.0 - out_qos.pkt_loss)
            msg.set_tlv(TLV.f32_tlv(TlvType.LINK_PKT_LOSS, new_loss))

            qos_log = (f" lat={new_lat:.1f}ms jit={new_jit:.1f}ms "
                       f"loss={new_loss*100:.2f}%")

        # Step 5c: N2 ⑤ — 可用容量 (当前=瓶颈带宽, 加负载追踪后 = backhaul - 已占用)

        # Step 6: 追加路径
        path_tlv = msg.get_tlv(TlvType.PATH_TRACE)
        old_path = path_tlv.value_str() if path_tlv else ""
        new_path = f"{old_path}→{next_hop}" if old_path else f"{self.node_id}→{next_hop}"
        msg.set_tlv(TLV.str_tlv(TlvType.PATH_TRACE, new_path))

        # Step 7: 逐跳计数
        msg.header.ttl -= 1
        msg.header.hop_count += 1

        # Step 8: 重打包
        out_msg = SBDPMessage.unpack(msg.pack())

        # Step 9: 生成日志
        diffs = in_msg.field_diff(out_msg)
        tag = "[ACC]" if is_user_hop else "[BHL]"
        bw_diff = " | ".join(diffs) if diffs else "no change"
        log = (f"  {self.node_id} {tag} →{next_hop} bw={out_bw:.0f}M "
               f"backhaul={out_msg.header.backhaul_capacity:.1f}M "
               f"crc=0x{out_msg.header.checksum:04x}"
               f"{qos_log}"
               f" [{bw_diff}]")

        return (next_hop, out_msg.pack(), log)

    # ── N2 完整响应 ──

    def build_n2_response(self, dst: str, backhaul_bw: float,
                          bottleneck: str, ts: float,
                          out_qos: 'LinkQoS' = None,
                          traffic_class: str = "",
                          user_priority: int = 0,
                          n2_req_seq: int = 0) -> SBDPMessage:
        """
        构造完整的 N2 接口响应 — gNB 查询路由器"回传怎么样？"

        对应论文表1 N2 接口全部五项:
          ① backhaul_bw   → backhaul_capacity (瓶颈带宽)
          ② bottleneck    → BOTTLENECK_LINK TLV (瓶颈位置)
          ③ PATH_TRACE    → 整条端到端路径
          ④ out_qos       → LINK_LATENCY / LINK_JITTER / LINK_PKT_LOSS (链路QoS)
          ⑤ traffic_class → TRAFFIC_CLASS / USER_PRIORITY / AVAIL_CAPACITY (业务属性)

        这是 CAPACITY_ACK 类型的报文, 响应对应的 CAPACITY_REQ。
        """
        hdr = SBDPHeader(
            msg_type=MsgType.CAPACITY_ACK,
            seq_num=n2_req_seq if n2_req_seq else self.next_seq(),
            backhaul_capacity=backhaul_bw,
            ttl=1,  # 单跳: gNB↔路由器在同一颗星上
        )
        tlvs = [
            TLV.str_tlv(TlvType.SOURCE_ID, self.node_id),
            TLV.str_tlv(TlvType.DEST_ID, dst),
            TLV.str_tlv(TlvType.BOTTLENECK_LINK, bottleneck),
            TLV.str_tlv(TlvType.PATH_TRACE, self.node_id),
            TLV.f64_tlv(TlvType.TIMESTAMP, ts),
            TLV.f32_tlv(TlvType.VALIDITY_PERIOD, 10.0),
        ]
        if out_qos is not None:
            tlvs.append(TLV.f32_tlv(TlvType.LINK_LATENCY, out_qos.latency_ms))
            tlvs.append(TLV.f32_tlv(TlvType.LINK_JITTER, out_qos.jitter_ms))
            tlvs.append(TLV.f32_tlv(TlvType.LINK_PKT_LOSS, out_qos.pkt_loss))
        if traffic_class:
            tlvs.append(TLV.str_tlv(TlvType.TRAFFIC_CLASS, traffic_class))
        tlvs.append(TLV.u8_tlv(TlvType.USER_PRIORITY, user_priority))
        # AVAIL_CAPACITY 待加: 需要链路负载追踪后才能区分 "瓶颈带宽" vs "可用容量"

        msg = SBDPMessage(header=hdr, tlvs=tlvs)
        self.sent_count += 1
        return SBDPMessage.unpack(msg.pack())

    # ── N3: gNB 间协同 ──

    def build_n3_migrate_request(self, target_gNB: str,
                                 ue_ids: List[str],
                                 required_bw: float,
                                 reason: str = "overload",
                                 ts: float = 0.0) -> SBDPMessage:
        """N3 迁移请求: 源gNB→目标gNB "这些UE要切过来,能接吗?"""
        hdr = SBDPHeader(
            msg_type=MsgType.CAPACITY_MIGRATE,
            seq_num=self.next_seq(),
            backhaul_capacity=required_bw,
            ttl=1,
        )
        tlvs = [
            TLV.str_tlv(TlvType.SOURCE_ID, self.node_id),
            TLV.str_tlv(TlvType.DEST_ID, target_gNB),
            TLV.str_tlv(TlvType.UE_LIST, ",".join(ue_ids)),
            TLV.f32_tlv(TlvType.RESERVED_BW, required_bw),
            TLV.str_tlv(TlvType.MIGRATE_REASON, reason),
            TLV.f64_tlv(TlvType.TIMESTAMP, ts or 0.0),
        ]
        msg = SBDPMessage(header=hdr, tlvs=tlvs)
        self.sent_count += 1
        return SBDPMessage.unpack(msg.pack())

    def build_n3_migrate_confirm(self, target_gNB: str,
                                 accepted_ues: List[str],
                                 reserved_bw: float,
                                 rejected_ues: List[str] = None,
                                 alt_suggestion: str = "",
                                 req_seq: int = 0) -> SBDPMessage:
        """N3 迁移确认: 目标gNB→源gNB "可以接这些UE" 或 "满了,试试别的\""""
        hdr = SBDPHeader(
            msg_type=MsgType.CAPACITY_CONFIRM,
            seq_num=req_seq if req_seq else self.next_seq(),
            backhaul_capacity=reserved_bw,
            ttl=1,
        )
        tlvs = [
            TLV.str_tlv(TlvType.SOURCE_ID, self.node_id),
            TLV.str_tlv(TlvType.DEST_ID, target_gNB),
            TLV.str_tlv(TlvType.UE_LIST, ",".join(accepted_ues)),
            TLV.f32_tlv(TlvType.RESERVED_BW, reserved_bw),
        ]
        if rejected_ues:
            tlvs.append(TLV.str_tlv(TlvType.MIGRATE_REASON,
                f"rejected:{','.join(rejected_ues)}"))
        if alt_suggestion:
            tlvs.append(TLV.str_tlv(TlvType.MIGRATE_TARGET, alt_suggestion))
        msg = SBDPMessage(header=hdr, tlvs=tlvs)
        self.sent_count += 1
        return SBDPMessage.unpack(msg.pack())

    def build_n3_state_adv(self, target_gNB: str,
                           gnb_state: 'GnBState') -> SBDPMessage:
        """N3 状态通告: gNB→gNB 分享接入侧状态"""
        hdr = SBDPHeader(
            msg_type=MsgType.CAPACITY_ADV,
            seq_num=self.next_seq(),
            backhaul_capacity=gnb_state.avail_access_bw,
            ttl=1,
        )
        tlvs = [
            TLV.str_tlv(TlvType.SOURCE_ID, self.node_id),
            TLV.str_tlv(TlvType.DEST_ID, target_gNB),
            TLV.f32_tlv(TlvType.GNB_LOAD, gnb_state.load_pct),
            TLV.f32_tlv(TlvType.CHANNEL_QUALITY, gnb_state.channel_sinr_db),
        ]
        msg = SBDPMessage(header=hdr, tlvs=tlvs)
        self.sent_count += 1
        return SBDPMessage.unpack(msg.pack())


# ════════════════════════════════════════════════
#  HandoverCoordinator — 群切换协同调度 (N3)
# ════════════════════════════════════════════════

@dataclass
class NeighborInfo:
    """本地缓存的邻居 gNB 状态"""
    gnb_id: str
    state: GnBState = field(default_factory=GnBState)
    backhaul_bw: float = 0.0
    last_update: float = 0.0

    def score(self) -> float:
        """综合评分: 回传 × (1-负载) × SINR加权"""
        sinr_weight = min(1.0, max(0.1, self.state.channel_sinr_db / 30.0))
        return self.backhaul_bw * (1.0 - self.state.load_pct) * sinr_weight


class HandoverCoordinator:
    """
    N3 群切换协同调度器 — 论文 "多基站协同低开销接入迁移" 的 Demo 实现。

    场景: gNB 过载或大批 UE 同时切换时, 不让每个 UE 盲抢,
          而是源 gNB 统一分配, 向目标星发 MIGRATE 请求确认。
    """

    def __init__(self, my_gnb_id: str):
        self.my_id = my_gnb_id
        self.engine = SBDPEngine(my_gnb_id)
        self.neighbors: Dict[str, NeighborInfo] = {}

    # ── 邻居表维护 ──

    def update_neighbor(self, gnb_id: str, backhaul_bw: float,
                        gnb_state: 'GnBState' = None, ts: float = 0.0):
        """收到邻居 ADV 后更新本地邻居表"""
        if gnb_id not in self.neighbors:
            self.neighbors[gnb_id] = NeighborInfo(gnb_id=gnb_id)
        nb = self.neighbors[gnb_id]
        nb.backhaul_bw = backhaul_bw
        nb.last_update = ts
        if gnb_state is not None:
            nb.state = gnb_state

    def update_neighbor_from_adv(self, msg: SBDPMessage, ts: float = 0.0):
        """从 SBDP ADV 报文中提取 N2+N3 信息并更新邻居表"""
        src_tlv = msg.get_tlv(TlvType.SOURCE_ID)
        if not src_tlv:
            return
        gnb_id = src_tlv.value_str()
        bw = msg.header.backhaul_capacity
        load_tlv = msg.get_tlv(TlvType.GNB_LOAD)
        sinr_tlv = msg.get_tlv(TlvType.CHANNEL_QUALITY)
        state = GnBState(
            load_pct=load_tlv.value_f32() if load_tlv else 0.0,
            channel_sinr_db=sinr_tlv.value_f32() if sinr_tlv else 30.0,
        )
        self.update_neighbor(gnb_id, bw, state, ts)

    def rank_candidates(self, exclude: set = None) -> List[Tuple[str, float]]:
        """返回 (gnb_id, score) 按评分降序"""
        exclude = exclude or set()
        ranked = [(nb_id, nb.score()) for nb_id, nb in self.neighbors.items()
                  if nb_id != self.my_id and nb_id not in exclude]
        ranked.sort(key=lambda x: x[1], reverse=True)
        return ranked

    def decide_migration(self, ue_ids: List[str],
                         required_bw_per_ue: float,
                         reason: str = "overload") -> List[Tuple[str, str, float]]:
        """
        为一批 UE 做迁移分配决策。
        返回: [(target_gnb, ue_id, reserved_bw), ...]
        """
        assignments = []
        remaining = list(ue_ids)
        tried: set = set()

        while remaining:
            ranked = self.rank_candidates(exclude=tried)
            if not ranked:
                break
            best_gnb, _ = ranked[0]
            batch_size = max(1, len(remaining) // max(1, len(ranked)))
            for ue in remaining[:batch_size]:
                assignments.append((best_gnb, ue, required_bw_per_ue))
            remaining = remaining[batch_size:]
            tried.add(best_gnb)

        return assignments

    def process_migrate_request(self, msg: SBDPMessage,
                                my_state: GnBState) -> SBDPMessage:
        """
        处理收到的 MIGRATE 请求 → 判断能否接 → 返回 CONFIRM
        """
        src_tlv = msg.get_tlv(TlvType.SOURCE_ID)
        ue_tlv = msg.get_tlv(TlvType.UE_LIST)
        bw_tlv = msg.get_tlv(TlvType.RESERVED_BW)
        src = src_tlv.value_str() if src_tlv else "?"
        ue_list = ue_tlv.value_str().split(",") if ue_tlv else []
        req_bw = bw_tlv.value_f32() if bw_tlv else 0.0

        accepted, rejected = [], []
        for ue_id in ue_list:
            if (my_state.load_pct < 0.85 and
                my_state.avail_access_bw >= req_bw):
                accepted.append(ue_id)
                my_state.avail_access_bw -= req_bw
                my_state.ue_count += 1
                my_state.load_pct = min(1.0, my_state.load_pct + 0.05)
            else:
                rejected.append(ue_id)

        alt = ""
        if rejected:
            ranked = self.rank_candidates(exclude={self.my_id, src})
            if ranked:
                alt = ranked[0][0]

        return self.engine.build_n3_migrate_confirm(
            target_gNB=src,
            accepted_ues=accepted,
            reserved_bw=req_bw,
            rejected_ues=rejected if rejected else None,
            alt_suggestion=alt,
            req_seq=msg.header.seq_num,
        )
