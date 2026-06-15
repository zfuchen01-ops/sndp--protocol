/* SBDP Exp 3 — Real P2P ISL Links + B2 Pull+Push + Throughput Verification
 *
 * All ISL/GS links are real NS-3 P2P links. Topology events call SetDataRate()
 * to enable/disable links, then recompute global routing. B2 nexthop is looked
 * up from the real NS-3 routing table. Data flows use the same ISL links as B2.
 *
 * Throughput verification: at key events, compare B2 bottleneck vs actual UDP throughput.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-route.h"
#include "ns3/sbdp-header.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include <map>
#include <vector>
#include <set>
#include <cmath>
#include <sstream>
#include <cstring>
using namespace ns3;
NS_LOG_COMPONENT_DEFINE("SbdpOrbit");

// ═══════════════════ SatRouter B2 — Pull + Push + Port Monitoring ═══════════════════

struct NbInfo { Ipv4Address ip; double bw; };

class SatRouter : public Application {
public:
  static TypeId GetTypeId();
  SatRouter() {}

  void AddNb(std::string n, Ipv4Address ip, double bw) { m_nb[n] = {ip, bw}; }
  void AddGs(std::string g, double bw)           { m_gs[g] = bw; m_nexthop[g] = ""; }  // "" = direct
  void SetRoute(std::string gs, std::string nb)   { m_nexthop[gs] = nb; }
  Ipv4Address GetNbIp(const std::string &n) const { auto it=m_nb.find(n); return it!=m_nb.end()?it->second.ip:Ipv4Address::GetAny(); }
  void UpdateBest() {
    m_bestBw = 0;
    for (auto &p : m_gs) if (p.second > m_bestBw) { m_bestBw = p.second; m_bestGs = p.first; }
  }
  std::map<std::string, NbInfo> m_nb;
  std::map<std::string, double> m_gs;
  std::map<std::string, std::string> m_nexthop;  // gs_name → routing next-hop ("" = direct)
  double m_bestBw = 0;
  std::string m_bestGs;
  double GetBest() const { return m_bestBw > 0 ? m_bestBw : 200; }

private:
  virtual void StartApplication() override;
  virtual void StopApplication() override;
  void RecvEx(Ptr<Socket> s);
  void SendPush();
  void SendRequest();
  void SendReply(const std::string &target);
  void CheckPortChange();

  Ptr<Socket> m_sk;
  EventId m_checkTimer;
  bool m_initialized = false;
  uint16_t m_seq = 0;  // B2 message sequence number
  std::map<std::string, double> m_nbSnapshot;  // last known local link states
};

NS_OBJECT_ENSURE_REGISTERED(SatRouter);
TypeId SatRouter::GetTypeId() {
  static TypeId tid = TypeId("ns3::SatRouter").SetParent<Application>()
    .SetGroupName("Sbdp").AddConstructor<SatRouter>();
  return tid;
}

void SatRouter::StartApplication() {
  m_sk = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9997));
  m_sk->SetRecvCallback(MakeCallback(&SatRouter::RecvEx, this));

  // Initial pull at 0.05s
  Simulator::Schedule(Seconds(0.05), &SatRouter::SendRequest, this);
  // Periodic port check every 200ms
  m_checkTimer = Simulator::Schedule(Seconds(0.20), &SatRouter::CheckPortChange, this);
}

void SatRouter::StopApplication() {
  Simulator::Cancel(m_checkTimer);
  if (m_sk) m_sk->Close();
}

// ── Send PUSH (type=0) — unsolicited, port changed ──
// Wire format:
//   [Magic:0x42,0x32][Version:1][Type:0][TotalLen:2B][Seq:2B]
//   [NameLen:1B][Name][N:1B][for each: GsLen:1B,GsName,Bw:4B]
void SatRouter::SendPush() {
  if (m_gs.empty()) return;

  std::string my = Names::FindName(GetNode());
  uint8_t buf[512]; int p = 0;
  // Fixed header (8 bytes)
  buf[p++] = 0x42; buf[p++] = 0x32;  // Magic "B2"
  buf[p++] = 1;                        // Version
  buf[p++] = 0;                        // Type = PUSH
  p += 2;                              // TotalLen placeholder (filled below)
  uint16_t seq = m_seq++;
  memcpy(buf + p, &seq, 2); p += 2;   // Sequence Number
  // Name
  buf[p++] = (uint8_t)my.size();
  memcpy(buf + p, my.c_str(), my.size()); p += my.size();
  // GS entries
  int n = 0; for (auto &x : m_gs) n++;
  buf[p++] = (uint8_t)n;
  for (auto &x : m_gs) {
    uint8_t l = x.first.size(); buf[p++] = l;
    memcpy(buf + p, x.first.c_str(), l); p += l;
    int32_t bw = (int32_t)x.second;
    memcpy(buf + p, &bw, 4); p += 4;
  }
  // Fill Total Length
  uint16_t total = (uint16_t)p;
  memcpy(buf + 4, &total, 2);

  Ptr<Packet> pkt = Create<Packet>(buf, p);
  for (auto &nb : m_nb)
    m_sk->SendTo(pkt, 0, InetSocketAddress(nb.second.ip, 9997));

  NS_LOG_UNCOND("  [PUSH " << my << "] seq=" << seq << " → " << m_nb.size()
    << " nb, best=" << m_bestGs << ":" << (int)m_bestBw << "M (len=" << total << "B)");
}

// ── Send REQUEST (type=1) — initial pull ──
void SatRouter::SendRequest() {
  std::string my = Names::FindName(GetNode());
  uint8_t buf[256]; int p = 0;
  // Fixed header
  buf[p++] = 0x42; buf[p++] = 0x32;  // Magic "B2"
  buf[p++] = 1;                        // Version
  buf[p++] = 1;                        // Type = REQUEST
  p += 2;                              // TotalLen placeholder
  uint16_t seq = m_seq++;
  memcpy(buf + p, &seq, 2); p += 2;   // Sequence Number
  // Name (no payload for REQUEST)
  buf[p++] = (uint8_t)my.size();
  memcpy(buf + p, my.c_str(), my.size()); p += my.size();
  // Fill Total Length
  uint16_t total = (uint16_t)p;
  memcpy(buf + 4, &total, 2);

  Ptr<Packet> pkt = Create<Packet>(buf, p);
  for (auto &nb : m_nb)
    m_sk->SendTo(pkt, 0, InetSocketAddress(nb.second.ip, 9997));
  NS_LOG_UNCOND("  [REQ " << my << "] seq=" << seq << " → " << m_nb.size()
    << " nb (len=" << total << "B)");
}

// ── Send REPLY (type=2) — response to REQUEST ──
void SatRouter::SendReply(const std::string &target) {
  if (m_gs.empty() || !m_nb.count(target)) return;

  std::string my = Names::FindName(GetNode());
  uint8_t buf[512]; int p = 0;
  // Fixed header
  buf[p++] = 0x42; buf[p++] = 0x32;  // Magic "B2"
  buf[p++] = 1;                        // Version
  buf[p++] = 2;                        // Type = REPLY
  p += 2;                              // TotalLen placeholder
  uint16_t seq = m_seq++;
  memcpy(buf + p, &seq, 2); p += 2;   // Sequence Number
  // Name
  buf[p++] = (uint8_t)my.size();
  memcpy(buf + p, my.c_str(), my.size()); p += my.size();
  // GS entries
  int n = 0; for (auto &x : m_gs) n++;
  buf[p++] = (uint8_t)n;
  for (auto &x : m_gs) {
    uint8_t l = x.first.size(); buf[p++] = l;
    memcpy(buf + p, x.first.c_str(), l); p += l;
    int32_t bw = (int32_t)x.second;
    memcpy(buf + p, &bw, 4); p += 4;
  }
  // Fill Total Length
  uint16_t total = (uint16_t)p;
  memcpy(buf + 4, &total, 2);

  Ptr<Packet> pkt = Create<Packet>(buf, p);
  m_sk->SendTo(pkt, 0, InetSocketAddress(m_nb[target].ip, 9997));
  NS_LOG_UNCOND("  [REPLY " << my << "] seq=" << seq
    << " → " << target << " (len=" << total << "B)");
}

// ── Receive (all types) ──
// Fixed header: Magic(2) + Version(1) + Type(1) + TotalLen(2) + Seq(2) = 8 bytes
// On PUSH/REPLY: update m_gs, then propagate to our neighbors (one-hop flood)
void SatRouter::RecvEx(Ptr<Socket> s) {
  Ptr<Packet> pkt; Address from;
  bool propagated = false;
  while ((pkt = s->RecvFrom(from))) {
    uint8_t buf[512]; pkt->CopyData(buf, pkt->GetSize());
    if (pkt->GetSize() < 9) continue;  // Min: fixed header(8) + name_len(1)

    // Parse fixed header
    if (buf[0] != 0x42 || buf[1] != 0x32) continue;  // Wrong magic
    if (buf[2] != 1) continue;                         // Unknown version
    uint8_t type = buf[3];
    uint16_t totalLen; memcpy(&totalLen, buf + 4, 2);
    uint16_t seq; memcpy(&seq, buf + 6, 2);
    uint8_t nl = buf[8];
    std::string nbName((char*)buf + 9, nl);
    int pos = 9 + nl;

    double nbLink = m_nb.count(nbName) ? m_nb[nbName].bw : 1e9;

    if (type == 1) {
      // ── REQUEST: reply with our GS info ──
      NS_LOG_UNCOND("  [B2 " << Names::FindName(GetNode()) << "] ←" << nbName
        << " REQUEST seq=" << seq);
      SendReply(nbName);
    } else if (type == 0 || type == 2) {
      // ── PUSH or REPLY: update from neighbor ──
      if (pkt->GetSize() < (uint32_t)pos + 1) continue;
      int n = buf[pos++]; bool chg = false;

      for (int i = 0; i < n && pos < (int)pkt->GetSize(); i++) {
        uint8_t gl = buf[pos++];
        std::string gs((char*)buf + pos, gl); pos += gl;
        int32_t nbBw; memcpy(&nbBw, buf + pos, 4); pos += 4;
        // Only accept GS info from our routing next-hop for that GS
        if (!m_nexthop.count(gs)) continue;
        if (m_nexthop[gs] == "") continue;     // direct → authoritative
        if (m_nexthop[gs] != nbName) continue; // not our next-hop
        double newBw = std::min(nbLink, (double)nbBw);
        if (!m_gs.count(gs) || newBw != m_gs[gs]) { m_gs[gs] = newBw; chg = true; }
      }
      if (chg) {
        UpdateBest();
        NS_LOG_UNCOND("  [B2 " << Names::FindName(GetNode()) << "] ←" << nbName
          << " " << (type == 0 ? "PUSH" : "REPLY") << " seq=" << seq
          << " best=" << m_bestGs << ":" << (int)m_bestBw << "M via "
          << nbName << " link=" << (int)nbLink << "M");
        propagated = true;
      }
    }
  }
  // Propagate: if we learned new info, push to our neighbors (once per batch)
  if (propagated) {
    SendPush();
  }
}

// ── Port change detection (local links only) ──
// Only checks m_nb (local ISL links). m_gs propagation is handled by RecvEx → SendPush.
void SatRouter::CheckPortChange() {
  std::string my = Names::FindName(GetNode());

  if (!m_initialized && !m_gs.empty()) {
    // Initial snapshot after startup exchange
    m_initialized = true;
    m_nbSnapshot.clear();
    for (auto &nb : m_nb) m_nbSnapshot[nb.first] = nb.second.bw;
  } else if (m_initialized) {
    bool changed = false;

    // Only check neighbor links (local port changes)
    if (m_nb.size() != m_nbSnapshot.size()) {
      changed = true;
    } else {
      for (auto &nb : m_nb) {
        if (!m_nbSnapshot.count(nb.first) ||
            std::abs(m_nbSnapshot[nb.first] - nb.second.bw) > 0.5) {
          changed = true; break;
        }
      }
    }

    if (changed) {
      // Invalidate GS entries routed through neighbors whose links changed
      for (auto &nb : m_nb) {
        double oldBw = m_nbSnapshot.count(nb.first) ? m_nbSnapshot[nb.first] : 0;
        if (std::abs(nb.second.bw - oldBw) > 0.5 || oldBw == 0) {
          for (auto &gs : m_nexthop) {
            if (gs.second == nb.first) {
              NS_LOG_UNCOND("  [MON " << my << "] re-route " << gs.first
                << " via " << nb.first << " (link " << (int)oldBw << "→" << (int)nb.second.bw << "M)");
              m_gs[gs.first] = 0;
            }
          }
        }
      }
      UpdateBest();
      SendPush();
      // Update snapshot after push
      m_nbSnapshot.clear();
      for (auto &nb : m_nb) m_nbSnapshot[nb.first] = nb.second.bw;
      NS_LOG_UNCOND("  [MON " << my << "] local port change → PUSH (best=" << m_bestGs << ":" << (int)m_bestBw << "M)");
    }
  }

  m_checkTimer = Simulator::Schedule(Seconds(0.20), &SatRouter::CheckPortChange, this);
}


// ═══════════════════ gNB ═══════════════════

class GnbApp : public Application {
public:
  static TypeId GetTypeId(); GnbApp() {}
  void AddCov(Ipv4Address ua) { m_cov.push_back(ua); }
  void SetR(Ptr<SatRouter> r) { m_r = r; }
  void Push();
private:
  virtual void StartApplication() override {
    m_sk = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9999));
  }
  virtual void StopApplication() override { if (m_sk) m_sk->Close(); }
  Ptr<Socket> m_sk;
  std::vector<Ipv4Address> m_cov;
  Ptr<SatRouter> m_r;
  uint16_t m_seq = 0;
};
NS_OBJECT_ENSURE_REGISTERED(GnbApp);
TypeId GnbApp::GetTypeId() {
  static TypeId tid = TypeId("ns3::GnbApp").SetParent<Application>()
    .SetGroupName("Sbdp").AddConstructor<GnbApp>();
  return tid;
}

void GnbApp::Push() {
  if (!m_r) return;
  double bw = m_r->GetBest();
  std::string gs = m_r->m_bestGs;
  m_seq++;
  for (auto &ua : m_cov) {
    SbdpHeader h = SbdpHeader::BuildAdv(
      Names::FindName(GetNode()), "UE", bw,
      Names::FindName(GetNode()) + "→" + gs, m_seq);
    Ptr<Packet> p = Create<Packet>(0);
    p->AddHeader(h);
    m_sk->SendTo(p, 0, InetSocketAddress(ua, 8888));
  }
  NS_LOG_UNCOND("  [ADV " << Names::FindName(GetNode()) << "] → " << m_cov.size()
    << " UEs: " << (int)bw << "M to " << gs);
}

// ═══════════════════ UsrApp ═══════════════════

class UsrApp : public Application {
public:
  static TypeId GetTypeId(); UsrApp() {}
  struct E { double t; float bw; uint16_t seq; };
  std::vector<E> m_h;
  uint32_t m_gaps = 0, m_missed = 0;
  uint16_t m_lastSeq = 0;
  double m_lastRecv = 0;
  bool m_stale = false;
private:
  virtual void StartApplication() override {
    m_sk = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(), 8888));
    m_sk->SetRecvCallback(MakeCallback(&UsrApp::Recv, this));
  }
  virtual void StopApplication() override { if (m_sk) m_sk->Close(); }
  void Recv(Ptr<Socket> s) {
    Ptr<Packet> pkt; Address from;
    while ((pkt = s->RecvFrom(from))) {
      SbdpHeader h;
      if (pkt->GetSize() >= SbdpHeader::SBDP_FIXED_SIZE) {
        pkt->RemoveHeader(h);
        double now = Simulator::Now().GetSeconds();
        if (m_lastSeq > 0 && h.GetSeqNum() != m_lastSeq + 1) {
          m_gaps++;
          m_missed += h.GetSeqNum() > m_lastSeq ? h.GetSeqNum() - m_lastSeq - 1 : 0;
        }
        m_lastSeq = h.GetSeqNum();
        m_lastRecv = now;
        m_h.push_back({now, h.GetBackhaulBw(), h.GetSeqNum()});
        NS_LOG_UNCOND("  [U@" << (int)now << "] bw=" << (int)h.GetBackhaulBw() << "M");
      }
    }
  }
  Ptr<Socket> m_sk;
};
NS_OBJECT_ENSURE_REGISTERED(UsrApp);
TypeId UsrApp::GetTypeId() {
  static TypeId tid = TypeId("ns3::UsrApp").SetParent<Application>()
    .SetGroupName("Sbdp").AddConstructor<UsrApp>();
  return tid;
}

// ═══════════════════ main ═══════════════════

int main(int argc, char *argv[]) {
  NodeContainer n; n.Create(12);
  Names::Add("A1", n.Get(0)); Names::Add("A2", n.Get(1)); Names::Add("A3", n.Get(2));
  Names::Add("B1", n.Get(3)); Names::Add("B2", n.Get(4)); Names::Add("B3", n.Get(5));
  Names::Add("GS-E", n.Get(6)); Names::Add("GS-W", n.Get(7));
  Names::Add("U1", n.Get(8)); Names::Add("U2", n.Get(9)); Names::Add("U3", n.Get(10));
  Names::Add("Srv", n.Get(11));

  InternetStackHelper inet; inet.Install(n);
  PointToPointHelper p2p;
  p2p.SetQueue("ns3::DropTailQueue");
  Ipv4AddressHelper ipv4;
  uint32_t subnet = 0;

  // ── Pre-build ALL possible P2P links (ISL + GS + User + Server) ──
  struct Edge { int a, b; double bw; };
  // All unique edges from 17 events: 7 ISL + 4 GS pairs
  std::vector<std::pair<int,int>> allPairs = {
    {0,1},{1,2},{3,4},{4,5},  // intra-orbit ISL
    {0,3},{1,4},{2,5},        // inter-orbit ISL
    {0,6},{5,7},{3,6},{2,7},  // GS links
    {8,0},{9,3},{10,4},       // user links
    {6,11}                     // GS-E → Srv
  };
  std::map<std::pair<int,int>, NetDeviceContainer> linkDevs;
  std::map<std::pair<int,int>, std::pair<Ipv4Address,Ipv4Address>> linkIps;

  auto buildLink = [&](int a, int b, double bwMbps = 1000, double delayMs = 5) {
    auto key = a < b ? std::make_pair(a,b) : std::make_pair(b,a);
    if (linkDevs.count(key)) return;
    p2p.SetDeviceAttribute("DataRate", StringValue(std::to_string((int)bwMbps) + "Mbps"));
    p2p.SetChannelAttribute("Delay", StringValue(std::to_string(delayMs) + "ms"));
    auto dev = p2p.Install(n.Get(a), n.Get(b));
    linkDevs[key] = dev;
    char base[32]; snprintf(base, 32, "10.%u.0.0", subnet++);
    ipv4.SetBase(base, "255.255.255.0");
    auto ifaces = ipv4.Assign(dev);
    linkIps[key] = {ifaces.GetAddress(0), ifaces.GetAddress(1)};
  };

  // Build all links at default 1Gbps
  for (auto &p : allPairs) buildLink(p.first, p.second);
  // GS-E↔GS-W: 1Gbps link for inter-GS routing
  buildLink(6, 7, 1000, 5);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  // Helper: get IP of node on a specific link
  auto ipOnLink = [&](int nodeIdx, int otherIdx) -> Ipv4Address {
    auto k = nodeIdx < otherIdx ? std::make_pair(nodeIdx, otherIdx) : std::make_pair(otherIdx, nodeIdx);
    auto &ips = linkIps[k];
    return (nodeIdx < otherIdx) ? ips.first : ips.second;
  };

  // ── Create SatRouters and gNBs ──
  struct Sat { Ptr<SatRouter> r; Ptr<GnbApp> g; };
  std::vector<Sat> sats(6);
  for (int i = 0; i < 6; i++) {
    sats[i].r = CreateObject<SatRouter>();
    sats[i].g = CreateObject<GnbApp>();
    sats[i].g->SetR(sats[i].r);
    n.Get(i)->AddApplication(sats[i].r); sats[i].r->SetStartTime(Seconds(0));
    n.Get(i)->AddApplication(sats[i].g); sats[i].g->SetStartTime(Seconds(0.05));
  }

  // ── Compute nexthop from NS-3 real routing table ──
  std::map<int, std::string> gsNodes = {{6,"GS-E"},{7,"GS-W"}};
  auto computeRoutes = [&]() {
    for (int i = 0; i < 6; i++) {
      Ptr<Ipv4RoutingProtocol> rp = n.Get(i)->GetObject<Ipv4>()->GetRoutingProtocol();
      for (auto &gs : gsNodes) {
        Ipv4Address gsIp = ipOnLink(gs.first, gs.first == 6 ? 11 : 11);
        // Try to get GS IP from a direct link (GS to connected sat)
        for (auto &p : allPairs) {
          if (p.first == gs.first && p.second < 6) gsIp = ipOnLink(gs.first, p.second);
          if (p.second == gs.first && p.first < 6) gsIp = ipOnLink(gs.first, p.first);
        }
        Ipv4Address dst = ipOnLink(6, 11);  // Use GS-E's server-side IP as destination
        if (gs.first == 7) dst = ipOnLink(7, 6);  // Use GS-W's IP
        Ptr<Packet> pkt = Create<Packet>();
        Ipv4Header hdr; hdr.SetDestination(dst); hdr.SetProtocol(17);
        Socket::SocketErrno sockerr;
        Ptr<Ipv4Route> route = rp->RouteOutput(pkt, hdr, Ptr<NetDevice>(), sockerr);
        if (route) {
          Ipv4Address gw = route->GetGateway();
          std::string nh = "";
          for (auto &nb : sats[i].r->m_nb) {
            if (sats[i].r->GetNbIp(nb.first) == gw) { nh = nb.first; break; }
          }
          if (gw == Ipv4Address::GetZero()) nh = "";  // direct
          sats[i].r->SetRoute(gs.second, nh);
        }
      }
    }
  };

  // ── setTopoP2P: modify real P2P links + update SatRouter state ──
  auto setTopoP2P = [&](const std::vector<Edge> &e) {
    // 1. Disable all ISL and GS links
    for (auto &kv : linkDevs) {
      int a = kv.first.first, b = kv.first.second;
      bool isISL = (a < 6 && b < 6);
      bool isGS  = ((a == 6 || a == 7) && b < 6) || ((b == 6 || b == 7) && a < 6);
      if (isISL || isGS) {
        DynamicCast<PointToPointNetDevice>(kv.second.Get(0))->SetDataRate(DataRate(0));
        DynamicCast<PointToPointNetDevice>(kv.second.Get(1))->SetDataRate(DataRate(0));
      }
    }
    // 2. Enable edges from event
    for (auto &ed : e) {
      int a = ed.a - 1, b = ed.b - 1;  // convert 1-based to 0-based
      auto key = a < b ? std::make_pair(a,b) : std::make_pair(b,a);
      if (linkDevs.count(key)) {
        double rate = ed.bw * 1e6;  // Mbps → bps
        DynamicCast<PointToPointNetDevice>(linkDevs[key].Get(0))->SetDataRate(DataRate(rate));
        DynamicCast<PointToPointNetDevice>(linkDevs[key].Get(1))->SetDataRate(rate);
      }
    }
    // 3. Recompute routing
    Ipv4GlobalRoutingHelper::RecomputeRoutingTables();
    // 4. Update SatRouter state
    for (int i = 0; i < 6; i++) { sats[i].r->m_nb.clear(); sats[i].r->m_gs.clear(); sats[i].r->m_nexthop.clear(); }
    for (auto &ed : e) {
      if (ed.a >= 1 && ed.a <= 6 && ed.b >= 1 && ed.b <= 6) {
        std::string na = Names::FindName(n.Get(ed.a - 1)),
                    nb = Names::FindName(n.Get(ed.b - 1));
        sats[ed.a - 1].r->AddNb(nb, ipOnLink(ed.b - 1, ed.a - 1), ed.bw);
        sats[ed.b - 1].r->AddNb(na, ipOnLink(ed.a - 1, ed.b - 1), ed.bw);
      }
      if ((ed.a == 7 || ed.a == 8) && ed.b >= 1 && ed.b <= 6)
        sats[ed.b - 1].r->AddGs(Names::FindName(n.Get(ed.a - 1)), ed.bw);
      if ((ed.b == 7 || ed.b == 8) && ed.a >= 1 && ed.a <= 6)
        sats[ed.a - 1].r->AddGs(Names::FindName(n.Get(ed.b - 1)), ed.bw);
    }
    // 5. Compute nexthop from NS-3 routing
    computeRoutes();
    // 6. Update best
    for (int i = 0; i < 6; i++) sats[i].r->UpdateBest();
  };

  // Initial topology
  setTopoP2P({{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},
              {1,7,350},{6,8,300},{4,7,250},{3,8,250}});

  // Coverage: all gNBs cover all 3 UEs (ADV via IP routing)
  for (int i = 0; i < 6; i++) {
    sats[i].g->AddCov(ipOnLink(8, 0)); sats[i].g->AddCov(ipOnLink(9, 3));
    sats[i].g->AddCov(ipOnLink(10, 4));
  }

  // ── Users ──
  Ptr<UsrApp> u1 = CreateObject<UsrApp>(); n.Get(8)->AddApplication(u1); u1->SetStartTime(Seconds(0.05));
  Ptr<UsrApp> u2 = CreateObject<UsrApp>(); n.Get(9)->AddApplication(u2); u2->SetStartTime(Seconds(0.05));
  Ptr<UsrApp> u3 = CreateObject<UsrApp>(); n.Get(10)->AddApplication(u3); u3->SetStartTime(Seconds(0.05));
  std::vector<Ptr<UsrApp>> users = {u1, u2, u3};

  // ── Data flows: continuous UDP from each user to Srv (via GS-E) ──
  Ipv4Address srvIp = ipOnLink(11, 6);
  std::vector<Ptr<PacketSink>> sinks;
  for (int i = 0; i < 3; i++) {
    PacketSinkHelper sk("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 5001 + i));
    auto a = sk.Install(n.Get(11)); a.Start(Seconds(0.3)); a.Stop(Seconds(885));
    sinks.push_back(DynamicCast<PacketSink>(a.Get(0)));
    OnOffHelper oo("ns3::UdpSocketFactory", InetSocketAddress(srvIp, 5001 + i));
    oo.SetAttribute("DataRate", DataRateValue(DataRate("50Mbps")));
    oo.SetAttribute("PacketSize", UintegerValue(1472));
    oo.SetConstantRate(DataRate("50Mbps"));
    auto c = oo.Install(n.Get(8 + i)); c.Start(Seconds(0.5)); c.Stop(Seconds(885));
  }

  // ── Throughput verification: short burst at key events ──
  auto verifyThroughput = [&](double t, int ueNode, const std::string &satName, double b2Bw) {
    Simulator::Schedule(Seconds(t), [=]() {
      uint16_t port = 6000 + (int)t;
      PacketSinkHelper sk("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), port));
      auto sinkApp = sk.Install(n.Get(11)); sinkApp.Start(Seconds(t)); sinkApp.Stop(Seconds(t + 0.6));
      OnOffHelper oo("ns3::UdpSocketFactory", InetSocketAddress(srvIp, port));
      oo.SetAttribute("DataRate", DataRateValue(DataRate("200Mbps")));
      oo.SetAttribute("PacketSize", UintegerValue(1472));
      oo.SetConstantRate(DataRate("200Mbps"));
      auto srcApp = oo.Install(n.Get(ueNode)); srcApp.Start(Seconds(t)); srcApp.Stop(Seconds(t + 0.5));
      Simulator::Schedule(Seconds(t + 0.55), [=]() {
        double rxBytes = DynamicCast<PacketSink>(sinkApp.Get(0))->GetTotalRx();
        double actualMbps = rxBytes * 8.0 / 0.5 / 1e6;
        NS_LOG_UNCOND("  [VERIFY t=" << (int)t << "s] " << satName << " B2=" << (int)b2Bw
          << "M | 200M burst→ actual=" << (int)actualMbps << "M | "
          << (actualMbps <= b2Bw + 15 ? "✓ bottleneck rules" : "✗ exceeds B2 claim"));
      });
    });
  };

  // ── Events ──
  auto ev = [&](double t, const std::vector<Edge> &e, const char *desc) {
    Simulator::Schedule(Seconds(t), [=]() {
      NS_LOG_UNCOND("\n═══ t=" << (int)t << "s " << desc << " ═══");
      setTopoP2P(e);
      Simulator::Schedule(Seconds(0.5), [sats]() {
        for (int i = 0; i < 6; i++) sats[i].g->Push();
      });
    });
  };

  ev(2,  {{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}}, "INITIAL");
  // Throughput verification at t=3 (after B2 settles): SAT-A claims GS-E:350M
  verifyThroughput(3, 8, "SAT-A→GS-E", 350);

  ev(50, {{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}}, "FAULT: ISL A2↔B2");
  ev(80, {{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,80},{6,8,300},{4,7,250},{3,8,250}}, "RAIN: GS-E 350→80M");
  // Throughput verification at t=82: GS-E degraded to 80M
  verifyThroughput(82, 8, "SAT-A→GS-E", 80);

  ev(120,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}}, "RECOVER");
  ev(160,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,0},{4,7,250},{3,8,250}}, "GS-W down");
  ev(190,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,0},{4,7,250},{3,8,250}}, "POLAR BLACKOUT");
  ev(230,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}}, "RECOVER");
  ev(280,{{1,2,200},{2,3,200},{4,5,200},{5,6,0},{1,4,0},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}}, "DOUBLE FAULT");
  // Throughput verification at t=282: double fault
  verifyThroughput(282, 9, "SAT-B1→GS-E", 250);

  ev(330,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}}, "RECOVER");
  ev(380,{{1,2,200},{2,3,200},{4,5,0},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,0},{6,8,300},{4,7,0},{3,8,250}}, "CASCADE: B1 isolated");
  ev(430,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}}, "RECOVER");
  ev(480,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,100},{6,8,100},{4,7,250},{3,8,250}}, "DEGRADE: both GS→100M");
  ev(870,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,0},{3,8,350},{4,7,250}}, "POLAR END");

  // ── PCAP on key links ──
  if (linkDevs.count({0,1})) p2p.EnablePcap("sbdp-orbit-A1A2", linkDevs[{0,1}].Get(0), true);
  if (linkDevs.count({0,6})) p2p.EnablePcap("sbdp-orbit-A1GSE", linkDevs[{0,6}].Get(0), true);

  // ── Report ──
  Simulator::Schedule(Seconds(875), [&]() {
    NS_LOG_UNCOND("\n\n═══ Exp 3 B2 Report ═══");
    for (int i = 0; i < 3; i++) {
      auto &h = users[i]->m_h;
      if (h.empty()) continue;
      double mb = sinks[i]->GetTotalRx() / 1e6;
      NS_LOG_UNCOND("  U" << (i + 1) << ": " << h.size() << " ADVs | data:" << mb
        << "MB | gaps=" << users[i]->m_gaps
        << " stale=" << (users[i]->m_stale ? "YES" : "no"));
    }
    NS_LOG_UNCOND("  ─────────────────────────────");
    NS_LOG_UNCOND("  B2 mode: Pull-init + Push-on-port-change");
    NS_LOG_UNCOND("  Links: real P2P ISL, SetDataRate toggle");
    NS_LOG_UNCOND("  Routing: NS-3 Ipv4GlobalRouting");
    NS_LOG_UNCOND("  Stable period: zero B2 traffic");
  });

  NS_LOG_UNCOND("\n═══ Exp 3: Real ISL + B2 Pull+Push + Throughput Verification ═══\n");
  Simulator::Stop(Seconds(885));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
