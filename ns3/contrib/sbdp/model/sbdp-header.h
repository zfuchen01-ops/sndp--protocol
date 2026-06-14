/* SBDP Header — 16B fixed + TLV chain.  Binary format matches Python sbdp.py.
 *
 * 0       1       2       3       4       5       6       7
 * +-------+-------+-------+-------+-------+-------+-------+-------+
 * |Ver(4)|Type(4)|  Flags(8)  |     Total Length (16)            |
 * +-------+-------+-------+-------+-------+-------+-------+-------+
 * |    Sequence Number (16)    |  TTL(8)   |  Hop Count(8)       |
 * +-------+-------+-------+-------+-------+-------+-------+-------+
 * |            Backhaul Capacity (float32, 32 bits)               |
 * +-------+-------+-------+-------+-------+-------+-------+-------+
 * |      Checksum (16)         |        Reserved (16)            |
 * +-------+-------+-------+-------+-------+-------+-------+-------+
 * |                    TLV Options (variable)...                  |
 * +-------+-------+-------+-------+-------+-------+-------+-------+
 */

#ifndef SBDP_HEADER_H
#define SBDP_HEADER_H

#include "ns3/header.h"
#include "ns3/type-id.h"
#include "ns3/buffer.h"
#include <string>
#include <vector>
#include <map>
#include <cstdint>

namespace ns3 {

// ── Message types (matches Python MsgType) ──
enum SbdpMsgType : uint8_t {
  SBDP_CAPACITY_ADV      = 0x1,   // gNB→UE: backhaul advisory
  SBDP_CAPACITY_REQ      = 0x2,   // gNB→router: N2 query
  SBDP_CAPACITY_ACK      = 0x3,   // router→gNB: N2 response
  SBDP_LINK_PROBE        = 0x4,   // link probe
  SBDP_CAPACITY_MIGRATE  = 0x5,   // gNB→gNB: N3 handover request
  SBDP_CAPACITY_CONFIRM  = 0x6,   // gNB→gNB: N3 confirm/reject
};

// ── Flags ──
enum SbdpFlags : uint8_t {
  SBDP_FLAG_NONE             = 0x00,
  SBDP_FLAG_REQUEST_ACK      = 0x01,
  SBDP_FLAG_STALE_ALLOWED    = 0x02,
  SBDP_FLAG_CONGESTION_MARK  = 0x04,
  SBDP_FLAG_PATH_CHANGE      = 0x08,
  SBDP_FLAG_EMERGENCY_UPDATE = 0x10,
};

// ── TLV types (matches Python TlvType) ──
enum SbdpTlvType : uint16_t {
  SBDP_TLV_SOURCE_ID       = 0x0001,
  SBDP_TLV_DEST_ID         = 0x0002,
  SBDP_TLV_BOTTLENECK_LINK = 0x0003,
  SBDP_TLV_PATH_TRACE      = 0x0004,
  SBDP_TLV_TIMESTAMP       = 0x0005,
  SBDP_TLV_VALIDITY_PERIOD = 0x0006,
  SBDP_TLV_ADVERTISER_ID   = 0x0007,
  // N2 QoS
  SBDP_TLV_LINK_LATENCY    = 0x000A,
  SBDP_TLV_LINK_JITTER     = 0x000B,
  SBDP_TLV_LINK_PKT_LOSS   = 0x000C,
  // N2 service class
  SBDP_TLV_TRAFFIC_CLASS   = 0x000D,
  SBDP_TLV_USER_PRIORITY   = 0x000E,
  SBDP_TLV_AVAIL_CAPACITY  = 0x000F,
  // N3 gNB coordination
  SBDP_TLV_GNB_LOAD        = 0x0011,
  SBDP_TLV_CHANNEL_QUALITY = 0x0012,
  SBDP_TLV_MIGRATE_TARGET  = 0x0013,
  SBDP_TLV_MIGRATE_REASON  = 0x0014,
  SBDP_TLV_UE_LIST         = 0x0015,
  SBDP_TLV_RESERVED_BW     = 0x0016,
};

// ── TLV entry ──
struct SbdpTlv {
  uint16_t type;
  std::string value;  // raw bytes (string for convenience, can hold binary)

  SbdpTlv () : type (0) {}
  SbdpTlv (uint16_t t, const std::string &v) : type (t), value (v) {}

  // ── Typed value helpers ──
  float    AsFloat () const;   // !f unpack
  double   AsDouble () const;  // !d unpack
  uint8_t  AsU8 () const;      // !B unpack
  std::string AsStr () const { return value; }

  static SbdpTlv Str (uint16_t type, const std::string &s);
  static SbdpTlv F32 (uint16_t type, float v);
  static SbdpTlv F64 (uint16_t type, double v);
  static SbdpTlv U8  (uint16_t type, uint8_t v);

  uint32_t Size () const { return 4 + value.size (); }  // 2B type + 2B len + value
};


class SbdpHeader : public Header
{
public:
  static TypeId GetTypeId ();
  virtual TypeId GetInstanceTypeId () const override;
  virtual uint32_t GetSerializedSize () const override;
  virtual void Serialize (Buffer::Iterator start) const override;
  virtual uint32_t Deserialize (Buffer::Iterator start) override;
  virtual void Print (std::ostream &os) const override;

  static const uint32_t SBDP_FIXED_SIZE = 16;  // 16B fixed header

  SbdpHeader ();

  // ── Fixed header fields ──
  void   SetVersion (uint8_t v)   { m_version = v & 0x0F; }
  uint8_t GetVersion () const     { return m_version; }

  void   SetMsgType (uint8_t t)   { m_msgType = t & 0x0F; }
  uint8_t GetMsgType () const     { return m_msgType; }
  std::string GetMsgTypeName () const;

  void   SetFlags (uint8_t f)     { m_flags = f; }
  uint8_t GetFlags () const       { return m_flags; }

  void   SetTotalLength (uint16_t l) { m_totalLen = l; }
  uint16_t GetTotalLength () const   { return m_totalLen; }

  void   SetSeqNum (uint16_t s)   { m_seqNum = s; }
  uint16_t GetSeqNum () const     { return m_seqNum; }

  void   SetTtl (uint8_t t)       { m_ttl = t; }
  uint8_t  GetTtl () const        { return m_ttl; }

  void   SetHopCount (uint8_t h)  { m_hopCount = h; }
  uint8_t  GetHopCount () const   { return m_hopCount; }

  void   SetBackhaulBw (float b)  { m_backhaulBw = b; }
  float  GetBackhaulBw () const   { return m_backhaulBw; }

  void   SetChecksum (uint16_t c) { m_checksum = c; }
  uint16_t GetChecksum () const   { return m_checksum; }

  uint16_t ComputeChecksum () const;  // CRC-16-CCITT over pre-CRC bytes

  // ── TLV manipulation ──
  void   AddTlv (const SbdpTlv &tlv);
  void   SetTlv (const SbdpTlv &tlv);      // replace if exists, else add
  const SbdpTlv* GetTlv (uint16_t type) const;
  bool   HasTlv (uint16_t type) const;
  void   RemoveTlv (uint16_t type);
  const std::vector<SbdpTlv>& GetTlvs () const { return m_tlvs; }
  void   ClearTlvs () { m_tlvs.clear (); }

  // ── Backward-compat convenience (maps to TLVs) ──
  void   SetBottleneckLink (const std::string &link);
  std::string GetBottleneckLink () const;

  void   SetSrcNode (const std::string &src);
  std::string GetSrcNode () const;

  void   SetDestNode (const std::string &dst);
  std::string GetDestNode () const;

  void   SetPathTrace (const std::string &path);
  std::string GetPathTrace () const;

  // ── N2 QoS ──
  void   SetLinkLatency (float ms);
  float  GetLinkLatency () const;

  void   SetLinkJitter (float ms);
  float  GetLinkJitter () const;

  void   SetLinkPktLoss (float ratio);
  float  GetLinkPktLoss () const;

  void   SetTrafficClass (const std::string &tc);
  std::string GetTrafficClass () const;

  void   SetUserPriority (uint8_t pri);
  uint8_t  GetUserPriority () const;

  // ── N3 ──
  void   SetGnbLoad (float load);
  float  GetGnbLoad () const;

  void   SetChannelQuality (float sinrDb);
  float  GetChannelQuality () const;

  void   SetUeList (const std::string &list);
  std::string GetUeList () const;

  void   SetReservedBw (float bw);
  float  GetReservedBw () const;

  void   SetMigrateReason (const std::string &reason);
  std::string GetMigrateReason () const;

  void   SetMigrateTarget (const std::string &target);
  std::string GetMigrateTarget () const;

  // ── Build helpers ──
  static SbdpHeader BuildAdv (
      const std::string &src, const std::string &dst,
      float backhaulBw, const std::string &bottleneck,
      uint16_t seq = 0, float latencyMs = 0, float jitterMs = 0,
      float pktLoss = 0, const std::string &trafficClass = "",
      uint8_t priority = 0, float gnbLoad = -1, float sinrDb = -1);

  static SbdpHeader BuildMigrate (
      const std::string &src, const std::string &dst,
      const std::string &ueList, float requiredBw,
      const std::string &reason = "overload", uint16_t seq = 0);

  static SbdpHeader BuildConfirm (
      const std::string &src, const std::string &dst,
      const std::string &acceptedUes, float reservedBw,
      const std::string &rejectedUes = "",
      const std::string &altSuggestion = "", uint16_t reqSeq = 0);

private:
  void RebuildTotalLength ();

  uint8_t  m_version;     // 4 bits
  uint8_t  m_msgType;     // 4 bits
  uint8_t  m_flags;
  uint16_t m_totalLen;
  uint16_t m_seqNum;
  uint8_t  m_ttl;
  uint8_t  m_hopCount;
  float    m_backhaulBw;
  uint16_t m_checksum;
  uint16_t m_reserved;

  std::vector<SbdpTlv> m_tlvs;
};

} // namespace ns3

#endif /* SBDP_HEADER_H */
