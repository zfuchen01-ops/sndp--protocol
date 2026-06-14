#include "sbdp-header.h"
#include "ns3/log.h"
#include <cstring>
#include <cstdio>
#include <sstream>
#include <cmath>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SbdpHeader");
NS_OBJECT_ENSURE_REGISTERED (SbdpHeader);

// ═══════════════════════════════════
//  CRC-16-CCITT (matches Python sbdp.py)
// ═══════════════════════════════════

static uint16_t g_crcTable[256];
static bool g_crcInit = false;

static void InitCrc ()
{
  if (g_crcInit) return;
  for (int i = 0; i < 256; i++) {
    uint16_t crc = i << 8;
    for (int j = 0; j < 8; j++)
      crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
    g_crcTable[i] = crc & 0xFFFF;
  }
  g_crcInit = true;
}

static uint16_t Crc16 (const uint8_t *data, uint32_t len)
{
  InitCrc ();
  uint16_t crc = 0xFFFF;
  for (uint32_t i = 0; i < len; i++)
    crc = ((crc << 8) ^ g_crcTable[((crc >> 8) ^ data[i]) & 0xFF]) & 0xFFFF;
  return crc;
}


// ═══════════════════════════════════
//  SbdpTlv
// ═══════════════════════════════════

float SbdpTlv::AsFloat () const {
  if (value.size () < 4) return 0;
  uint32_t bits = ((uint8_t)value[0] << 24) | ((uint8_t)value[1] << 16)
                | ((uint8_t)value[2] << 8)  | (uint8_t)value[3];
  float f; std::memcpy (&f, &bits, sizeof (f)); return f;
}

double SbdpTlv::AsDouble () const {
  if (value.size () < 8) return 0;
  uint64_t bits = 0;
  for (int i = 0; i < 8; i++) bits = (bits << 8) | (uint8_t)value[i];
  double d; std::memcpy (&d, &bits, sizeof (d)); return d;
}

uint8_t SbdpTlv::AsU8 () const {
  return value.empty () ? 0 : (uint8_t)value[0];
}

SbdpTlv SbdpTlv::Str (uint16_t type, const std::string &s) {
  return SbdpTlv (type, s);
}

SbdpTlv SbdpTlv::F32 (uint16_t type, float v) {
  uint32_t bits; std::memcpy (&bits, &v, sizeof (bits));
  std::string val (4, '\0');
  val[0] = (bits >> 24) & 0xFF; val[1] = (bits >> 16) & 0xFF;
  val[2] = (bits >> 8)  & 0xFF; val[3] = bits & 0xFF;
  return SbdpTlv (type, val);
}

SbdpTlv SbdpTlv::F64 (uint16_t type, double v) {
  uint64_t bits; std::memcpy (&bits, &v, sizeof (bits));
  std::string val (8, '\0');
  for (int i = 7; i >= 0; i--) { val[7-i] = (bits >> (i*8)) & 0xFF; }
  return SbdpTlv (type, val);
}

SbdpTlv SbdpTlv::U8 (uint16_t type, uint8_t v) {
  return SbdpTlv (type, std::string (1, (char)v));
}


// ═══════════════════════════════════
//  SbdpHeader
// ═══════════════════════════════════

TypeId SbdpHeader::GetTypeId () {
  static TypeId tid = TypeId ("ns3::SbdpHeader")
    .SetParent<Header> ().SetGroupName ("Sbdp")
    .AddConstructor<SbdpHeader> ();
  return tid;
}

TypeId SbdpHeader::GetInstanceTypeId () const { return GetTypeId (); }

SbdpHeader::SbdpHeader ()
  : m_version (1), m_msgType (SBDP_CAPACITY_ADV), m_flags (0),
    m_totalLen (SBDP_FIXED_SIZE), m_seqNum (0), m_ttl (20), m_hopCount (0),
    m_backhaulBw (std::numeric_limits<float>::infinity ()),
    m_checksum (0), m_reserved (0)
{}

std::string SbdpHeader::GetMsgTypeName () const {
  switch (m_msgType) {
    case SBDP_CAPACITY_ADV:     return "ADV";
    case SBDP_CAPACITY_REQ:     return "REQ";
    case SBDP_CAPACITY_ACK:     return "ACK";
    case SBDP_LINK_PROBE:       return "PROBE";
    case SBDP_CAPACITY_MIGRATE: return "MIGRATE";
    case SBDP_CAPACITY_CONFIRM: return "CONFIRM";
    default: return "?";
  }
}

// ── TLV ops ──

void SbdpHeader::AddTlv (const SbdpTlv &tlv) {
  m_tlvs.push_back (tlv);
  RebuildTotalLength ();
}

void SbdpHeader::SetTlv (const SbdpTlv &tlv) {
  for (auto &t : m_tlvs) {
    if (t.type == tlv.type) { t.value = tlv.value; RebuildTotalLength (); return; }
  }
  AddTlv (tlv);
}

const SbdpTlv* SbdpHeader::GetTlv (uint16_t type) const {
  for (auto &t : m_tlvs) if (t.type == type) return &t;
  return nullptr;
}

bool SbdpHeader::HasTlv (uint16_t type) const {
  return GetTlv (type) != nullptr;
}

void SbdpHeader::RemoveTlv (uint16_t type) {
  m_tlvs.erase (std::remove_if (m_tlvs.begin (), m_tlvs.end (),
    [type](const SbdpTlv &t) { return t.type == type; }), m_tlvs.end ());
  RebuildTotalLength ();
}

void SbdpHeader::RebuildTotalLength () {
  m_totalLen = SBDP_FIXED_SIZE;
  for (auto &t : m_tlvs) m_totalLen += t.Size ();
}

// ── CRC ──

uint16_t SbdpHeader::ComputeChecksum () const {
  // CRC over: version|type, flags, totalLen, seqNum, ttl, hopCount, backhaulBw
  // (the 14 bytes before the checksum field)
  uint8_t buf[14];
  buf[0] = (m_version << 4) | (m_msgType & 0x0F);
  buf[1] = m_flags;
  buf[2] = (m_totalLen >> 8) & 0xFF;
  buf[3] = m_totalLen & 0xFF;
  buf[4] = (m_seqNum >> 8) & 0xFF;
  buf[5] = m_seqNum & 0xFF;
  buf[6] = m_ttl;
  buf[7] = m_hopCount;
  uint32_t bwBits; std::memcpy (&bwBits, &m_backhaulBw, sizeof (bwBits));
  buf[8]  = (bwBits >> 24) & 0xFF;
  buf[9]  = (bwBits >> 16) & 0xFF;
  buf[10] = (bwBits >> 8)  & 0xFF;
  buf[11] = bwBits & 0xFF;
  buf[12] = 0; buf[13] = 0;  // reserved
  return Crc16 (buf, 14);
}

// ── Serialization ──

uint32_t SbdpHeader::GetSerializedSize () const {
  return m_totalLen;
}

void SbdpHeader::Serialize (Buffer::Iterator start) const {
  // Compute CRC
  uint16_t csum = const_cast<SbdpHeader*>(this)->m_checksum;
  const_cast<SbdpHeader*>(this)->m_checksum = ComputeChecksum ();
  uint16_t actualCsum = m_checksum;
  const_cast<SbdpHeader*>(this)->m_checksum = csum;  // restore

  start.WriteU8 ((m_version << 4) | (m_msgType & 0x0F));
  start.WriteU8 (m_flags);
  start.WriteU16 (m_totalLen);
  start.WriteU16 (m_seqNum);
  start.WriteU8 (m_ttl);
  start.WriteU8 (m_hopCount);

  uint32_t bwBits; std::memcpy (&bwBits, &m_backhaulBw, sizeof (bwBits));
  start.WriteU32 (bwBits);
  start.WriteU16 (actualCsum);
  start.WriteU16 (m_reserved);

  // TLV chain
  for (auto &t : m_tlvs) {
    start.WriteU16 (t.type);
    start.WriteU16 ((uint16_t)t.value.size ());
    start.Write ((const uint8_t*)t.value.data (), t.value.size ());
  }
}

uint32_t SbdpHeader::Deserialize (Buffer::Iterator start) {
  uint8_t byte0 = start.ReadU8 ();
  m_version = (byte0 >> 4) & 0x0F;
  m_msgType = byte0 & 0x0F;
  m_flags = start.ReadU8 ();
  m_totalLen = start.ReadU16 ();
  m_seqNum = start.ReadU16 ();
  m_ttl = start.ReadU8 ();
  m_hopCount = start.ReadU8 ();

  uint32_t bwBits = start.ReadU32 ();
  std::memcpy (&m_backhaulBw, &bwBits, sizeof (m_backhaulBw));
  m_checksum = start.ReadU16 ();
  m_reserved = start.ReadU16 ();

  // Verify CRC
  uint16_t calcCsum = ComputeChecksum ();
  if (calcCsum != m_checksum) {
    NS_LOG_WARN ("SBDP CRC mismatch: got 0x" << std::hex << m_checksum
                 << " expected 0x" << calcCsum << std::dec);
  }

  // Read TLVs
  m_tlvs.clear ();
  uint32_t consumed = SBDP_FIXED_SIZE;
  while (consumed < m_totalLen) {
    uint16_t tlvType = start.ReadU16 ();
    uint16_t tlvLen  = start.ReadU16 ();
    std::string val (tlvLen, '\0');
    start.Read ((uint8_t*)&val[0], tlvLen);
    m_tlvs.push_back (SbdpTlv (tlvType, val));
    consumed += 4 + tlvLen;
  }

  return consumed;
}

void SbdpHeader::Print (std::ostream &os) const {
  os << "SBDP[" << GetMsgTypeName () << " seq=" << m_seqNum
     << " backhaul=";
  if (m_backhaulBw == std::numeric_limits<float>::infinity ())
    os << "INF";
  else
    os << m_backhaulBw;
  os << "M hop=" << (int)m_hopCount << " ttl=" << (int)m_ttl;

  auto *bl = GetTlv (SBDP_TLV_BOTTLENECK_LINK);
  if (bl) os << " bl=" << bl->AsStr ();

  auto *load = GetTlv (SBDP_TLV_GNB_LOAD);
  if (load) os << " load=" << (int)(load->AsFloat()*100) << "%";

  auto *sinr = GetTlv (SBDP_TLV_CHANNEL_QUALITY);
  if (sinr) os << " sinr=" << (int)sinr->AsFloat() << "dB";

  os << " crc=0x" << std::hex << m_checksum << std::dec
     << " tlvs=" << m_tlvs.size () << "]";
}


// ═══════════════════════════════════
//  Backward-compat convenience
// ═══════════════════════════════════

void SbdpHeader::SetBottleneckLink (const std::string &link) {
  SetTlv (SbdpTlv::Str (SBDP_TLV_BOTTLENECK_LINK, link));
}
std::string SbdpHeader::GetBottleneckLink () const {
  auto *t = GetTlv (SBDP_TLV_BOTTLENECK_LINK); return t ? t->AsStr () : "";
}

void SbdpHeader::SetSrcNode (const std::string &src) {
  SetTlv (SbdpTlv::Str (SBDP_TLV_SOURCE_ID, src));
}
std::string SbdpHeader::GetSrcNode () const {
  auto *t = GetTlv (SBDP_TLV_SOURCE_ID); return t ? t->AsStr () : "";
}

void SbdpHeader::SetDestNode (const std::string &dst) {
  SetTlv (SbdpTlv::Str (SBDP_TLV_DEST_ID, dst));
}
std::string SbdpHeader::GetDestNode () const {
  auto *t = GetTlv (SBDP_TLV_DEST_ID); return t ? t->AsStr () : "";
}

void SbdpHeader::SetPathTrace (const std::string &path) {
  SetTlv (SbdpTlv::Str (SBDP_TLV_PATH_TRACE, path));
}
std::string SbdpHeader::GetPathTrace () const {
  auto *t = GetTlv (SBDP_TLV_PATH_TRACE); return t ? t->AsStr () : "";
}

// ── N2 QoS ──

void SbdpHeader::SetLinkLatency (float ms) {
  SetTlv (SbdpTlv::F32 (SBDP_TLV_LINK_LATENCY, ms));
}
float SbdpHeader::GetLinkLatency () const {
  auto *t = GetTlv (SBDP_TLV_LINK_LATENCY); return t ? t->AsFloat () : 0;
}

void SbdpHeader::SetLinkJitter (float ms) {
  SetTlv (SbdpTlv::F32 (SBDP_TLV_LINK_JITTER, ms));
}
float SbdpHeader::GetLinkJitter () const {
  auto *t = GetTlv (SBDP_TLV_LINK_JITTER); return t ? t->AsFloat () : 0;
}

void SbdpHeader::SetLinkPktLoss (float ratio) {
  SetTlv (SbdpTlv::F32 (SBDP_TLV_LINK_PKT_LOSS, ratio));
}
float SbdpHeader::GetLinkPktLoss () const {
  auto *t = GetTlv (SBDP_TLV_LINK_PKT_LOSS); return t ? t->AsFloat () : 0;
}

void SbdpHeader::SetTrafficClass (const std::string &tc) {
  SetTlv (SbdpTlv::Str (SBDP_TLV_TRAFFIC_CLASS, tc));
}
std::string SbdpHeader::GetTrafficClass () const {
  auto *t = GetTlv (SBDP_TLV_TRAFFIC_CLASS); return t ? t->AsStr () : "";
}

void SbdpHeader::SetUserPriority (uint8_t pri) {
  SetTlv (SbdpTlv::U8 (SBDP_TLV_USER_PRIORITY, pri));
}
uint8_t SbdpHeader::GetUserPriority () const {
  auto *t = GetTlv (SBDP_TLV_USER_PRIORITY); return t ? t->AsU8 () : 0;
}

// ── N3 ──

void SbdpHeader::SetGnbLoad (float load) {
  SetTlv (SbdpTlv::F32 (SBDP_TLV_GNB_LOAD, load));
}
float SbdpHeader::GetGnbLoad () const {
  auto *t = GetTlv (SBDP_TLV_GNB_LOAD); return t ? t->AsFloat () : -1;
}

void SbdpHeader::SetChannelQuality (float sinrDb) {
  SetTlv (SbdpTlv::F32 (SBDP_TLV_CHANNEL_QUALITY, sinrDb));
}
float SbdpHeader::GetChannelQuality () const {
  auto *t = GetTlv (SBDP_TLV_CHANNEL_QUALITY); return t ? t->AsFloat () : -1;
}

void SbdpHeader::SetUeList (const std::string &list) {
  SetTlv (SbdpTlv::Str (SBDP_TLV_UE_LIST, list));
}
std::string SbdpHeader::GetUeList () const {
  auto *t = GetTlv (SBDP_TLV_UE_LIST); return t ? t->AsStr () : "";
}

void SbdpHeader::SetReservedBw (float bw) {
  SetTlv (SbdpTlv::F32 (SBDP_TLV_RESERVED_BW, bw));
}
float SbdpHeader::GetReservedBw () const {
  auto *t = GetTlv (SBDP_TLV_RESERVED_BW); return t ? t->AsFloat () : 0;
}

void SbdpHeader::SetMigrateReason (const std::string &reason) {
  SetTlv (SbdpTlv::Str (SBDP_TLV_MIGRATE_REASON, reason));
}
std::string SbdpHeader::GetMigrateReason () const {
  auto *t = GetTlv (SBDP_TLV_MIGRATE_REASON); return t ? t->AsStr () : "";
}

void SbdpHeader::SetMigrateTarget (const std::string &target) {
  SetTlv (SbdpTlv::Str (SBDP_TLV_MIGRATE_TARGET, target));
}
std::string SbdpHeader::GetMigrateTarget () const {
  auto *t = GetTlv (SBDP_TLV_MIGRATE_TARGET); return t ? t->AsStr () : "";
}


// ═══════════════════════════════════
//  Static builders
// ═══════════════════════════════════

SbdpHeader SbdpHeader::BuildAdv (
    const std::string &src, const std::string &dst,
    float backhaulBw, const std::string &bottleneck,
    uint16_t seq, float latencyMs, float jitterMs,
    float pktLoss, const std::string &trafficClass,
    uint8_t priority, float gnbLoad, float sinrDb)
{
  SbdpHeader hdr;
  hdr.SetMsgType (SBDP_CAPACITY_ADV);
  hdr.SetSeqNum (seq);
  hdr.SetBackhaulBw (backhaulBw);
  hdr.SetSrcNode (src);
  hdr.SetDestNode (dst);
  hdr.SetBottleneckLink (bottleneck);
  hdr.SetPathTrace (src);

  if (latencyMs > 0 || jitterMs > 0 || pktLoss > 0) {
    hdr.SetLinkLatency (latencyMs);
    hdr.SetLinkJitter (jitterMs);
    hdr.SetLinkPktLoss (pktLoss);
  }
  if (!trafficClass.empty ())
    hdr.SetTrafficClass (trafficClass);
  hdr.SetUserPriority (priority);

  if (gnbLoad >= 0) {
    hdr.SetGnbLoad (gnbLoad);
    hdr.SetChannelQuality (sinrDb >= 0 ? sinrDb : 30.0f);
  }

  return hdr;
}

SbdpHeader SbdpHeader::BuildMigrate (
    const std::string &src, const std::string &dst,
    const std::string &ueList, float requiredBw,
    const std::string &reason, uint16_t seq)
{
  SbdpHeader hdr;
  hdr.SetMsgType (SBDP_CAPACITY_MIGRATE);
  hdr.SetSeqNum (seq);
  hdr.SetBackhaulBw (requiredBw);
  hdr.SetTtl (1);
  hdr.SetSrcNode (src);
  hdr.SetDestNode (dst);
  hdr.SetUeList (ueList);
  hdr.SetReservedBw (requiredBw);
  hdr.SetMigrateReason (reason);
  return hdr;
}

SbdpHeader SbdpHeader::BuildConfirm (
    const std::string &src, const std::string &dst,
    const std::string &acceptedUes, float reservedBw,
    const std::string &rejectedUes,
    const std::string &altSuggestion, uint16_t reqSeq)
{
  SbdpHeader hdr;
  hdr.SetMsgType (SBDP_CAPACITY_CONFIRM);
  hdr.SetSeqNum (reqSeq);
  hdr.SetBackhaulBw (reservedBw);
  hdr.SetTtl (1);
  hdr.SetSrcNode (src);
  hdr.SetDestNode (dst);
  hdr.SetUeList (acceptedUes);
  hdr.SetReservedBw (reservedBw);
  if (!rejectedUes.empty ())
    hdr.SetMigrateReason ("rejected:" + rejectedUes);
  if (!altSuggestion.empty ())
    hdr.SetMigrateTarget (altSuggestion);
  return hdr;
}

} // namespace ns3
