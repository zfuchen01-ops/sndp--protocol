/* SBDP Exp 2 — B2 Pull+Push with Port Monitoring + N2 + ADV (A)
 *
 * B2: Pull-init + Push-on-change (port monitoring).
 *     Satellites monitor their own neighbor/GS state vs snapshot.
 *     On change → instant PUSH to all neighbors.
 *     On REQUEST → REPLY with current GS info.
 *     Stable period → zero B2 traffic.
 *
 * N2: gNB queries local Router → gets best e2e bottleneck.
 * A:  gNB broadcasts ADV to covered UEs.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/sbdp-header.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/ipv4-route.h"
#include <map>
#include <vector>
#include <sstream>
#include <cstring>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("SbdpHandover");

// ═══════════════════ SatRouter: B2 Pull+Push + Port Monitoring ═══════════════════

class SatRouter : public Application {
public:
  static TypeId GetTypeId(); SatRouter();
  void SetBw(double in, double out) { m_in = in; m_out = out; }
  void AddNeighbor(std::string n, Ipv4Address ip, double bw) { m_nb[n] = {ip, bw}; }
  void AddDirectGs(std::string g, double bw) { m_gsDirect[g]=bw; m_gs[g]=bw; m_bestGs=g; m_bestBw=bw; m_nexthop[g]=""; }
  void InstallMonitor(std::string nbName, Ptr<NetDevice> dev);
  void SetRoute(std::string gs, std::string nb) { m_nexthop[gs] = nb; }
  Ipv4Address GetNbIp(const std::string &n) const { auto it=m_nb.find(n); return it!=m_nb.end()?it->second.ip:Ipv4Address::GetAny(); }
  double GetBestE2e() const { return m_bestBw > 0 ? m_bestBw : GetLocalBw(); }
  double GetLocalBw() const { double b = m_out; if (m_in > 0 && m_in < b) b = m_in; return b; }

private:
  virtual void StartApplication() override;
  virtual void StopApplication() override;
  void RecvN2(Ptr<Socket> s);
  void RecvEx(Ptr<Socket> s);
  void SendPush();
  void SendRequest();
  void SendReply(const std::string &target);
  void CheckPortChange();

  Ptr<Socket> m_n2sk, m_sk;
  double m_in = 0, m_out = 0, m_bestBw = 0;
  std::string m_bestGs;
  struct Nb { Ipv4Address ip; double bw; };
  std::map<std::string, Nb> m_nb;
  std::map<std::string, double> m_gs;
  std::map<std::string, double> m_gsDirect, m_nbSnapshot;
  std::map<std::string, std::string> m_nexthop;
  struct NbMon { Ptr<DropTailQueue<Packet>> queue; uint64_t lastBytes=0; double availBw=0; };
  std::map<std::string, NbMon> m_nbMon;
  double m_lastMonTime=0;
  EventId m_checkTimer;
  bool m_initialized = false;
  uint16_t m_seq = 0;  // B2 message sequence number
};

NS_OBJECT_ENSURE_REGISTERED(SatRouter);
TypeId SatRouter::GetTypeId() {
  static TypeId tid = TypeId("ns3::SatRouter").SetParent<Application>()
    .SetGroupName("Sbdp").AddConstructor<SatRouter>();
  return tid;
}
SatRouter::SatRouter() {}
void SatRouter::InstallMonitor(std::string nbName, Ptr<NetDevice> dev) {
  auto p2p = DynamicCast<PointToPointNetDevice>(dev);
  auto& m = m_nbMon[nbName];
  m.queue = DynamicCast<DropTailQueue<Packet>>(p2p->GetQueue());
  m.lastBytes = m.queue ? m.queue->GetTotalReceivedBytes() : 0;
  NS_LOG_UNCOND("  [MON-INIT " << Names::FindName(GetNode()) << "->" << nbName << " Q=" << (m.queue?"OK":"NULL"));
}

void SatRouter::StartApplication() {
  m_n2sk = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  m_n2sk->Bind(InetSocketAddress(Ipv4Address("127.0.0.1"), 9998));
  m_n2sk->SetRecvCallback(MakeCallback(&SatRouter::RecvN2, this));

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
  if (m_n2sk) m_n2sk->Close();
  if (m_sk) m_sk->Close();
}

// ── N2: gNB queries Router ──
void SatRouter::RecvN2(Ptr<Socket> s) {
  Ptr<Packet> pkt; Address from;
  while ((pkt = s->RecvFrom(from))) {
    // Send ALL GS entries to gNB
    uint8_t buf[512]; int pos = 0;
    buf[pos++] = (uint8_t)m_gs.size();
    for (auto &p : m_gs) {
      int32_t bw = (int32_t)p.second; memcpy(buf + pos, &bw, 4); pos += 4;
      uint32_t nl = p.first.size(); memcpy(buf + pos, &nl, 4); pos += 4;
      memcpy(buf + pos, p.first.c_str(), nl); pos += nl;
      std::string bl = Names::FindName(GetNode()) + "→" + p.first + "(" + std::to_string((int)p.second) + "M)";
      uint32_t blen = bl.size(); memcpy(buf + pos, &blen, 4); pos += 4;
      memcpy(buf + pos, bl.c_str(), blen); pos += blen;
    }
    s->SendTo(Create<Packet>(buf, pos), 0, from);
    NS_LOG_UNCOND("  [R " << Names::FindName(GetNode()) << "] N2→gNB: " << m_gs.size() << " GS, best=" << m_bestGs << ":" << (int)m_bestBw << "M");
  }
}

// ── B2 Send PUSH (type=0) — unsolicited, port changed ──
// Wire format: [Magic:0x42,0x32][Ver:1][Type:0][TotalLen:2B][Seq:2B][NameLen][Name][N][entries...]
void SatRouter::SendPush() {
  if (m_gs.empty()) return;

  std::string myName = Names::FindName(GetNode());
  uint8_t buf[512]; int pos = 0;
  // Fixed header (8 bytes)
  buf[pos++] = 0x42; buf[pos++] = 0x32;  // Magic "B2"
  buf[pos++] = 1;                          // Version
  buf[pos++] = 0;                          // Type = PUSH
  pos += 2;                                // TotalLen placeholder
  uint16_t seq = m_seq++;
  memcpy(buf + pos, &seq, 2); pos += 2;   // Sequence Number
  // Name
  buf[pos++] = (uint8_t)myName.size();
  memcpy(buf + pos, myName.c_str(), myName.size()); pos += myName.size();
  // GS entries
  int n = 0; for (auto &p : m_gs) n++;
  buf[pos++] = (uint8_t)n;
  for (auto &p : m_gs) {
    uint8_t nl = p.first.size(); buf[pos++] = nl;
    memcpy(buf + pos, p.first.c_str(), nl); pos += nl;
    int32_t bw = (int32_t)p.second;
    memcpy(buf + pos, &bw, 4); pos += 4;
  }
  // Fill Total Length
  uint16_t total = (uint16_t)pos;
  memcpy(buf + 4, &total, 2);

  Ptr<Packet> pkt = Create<Packet>(buf, pos);
  for (auto &nb : m_nb)
    m_sk->SendTo(pkt, 0, InetSocketAddress(nb.second.ip, 9997));

  // Update snapshots
  m_nbSnapshot.clear();
  for (auto &nb : m_nb) m_nbSnapshot[nb.first] = nb.second.bw;
  

  NS_LOG_UNCOND("  [PUSH " << myName << "] seq=" << seq << " → " << m_nb.size()
    << " nb, best=" << m_bestGs << ":" << (int)m_bestBw << "M (" << n << " GS, len=" << total << "B)");
}

// ── B2 Send REQUEST (type=1) — initial pull ──
void SatRouter::SendRequest() {
  std::string myName = Names::FindName(GetNode());
  uint8_t buf[256]; int pos = 0;
  // Fixed header
  buf[pos++] = 0x42; buf[pos++] = 0x32;  // Magic "B2"
  buf[pos++] = 1;                          // Version
  buf[pos++] = 1;                          // Type = REQUEST
  pos += 2;                                // TotalLen placeholder
  uint16_t seq = m_seq++;
  memcpy(buf + pos, &seq, 2); pos += 2;   // Sequence Number
  // Name
  buf[pos++] = (uint8_t)myName.size();
  memcpy(buf + pos, myName.c_str(), myName.size()); pos += myName.size();
  // Fill Total Length
  uint16_t total = (uint16_t)pos;
  memcpy(buf + 4, &total, 2);

  Ptr<Packet> pkt = Create<Packet>(buf, pos);
  for (auto &nb : m_nb)
    m_sk->SendTo(pkt, 0, InetSocketAddress(nb.second.ip, 9997));
  NS_LOG_UNCOND("  [REQ " << myName << "] seq=" << seq << " → " << m_nb.size()
    << " nb (len=" << total << "B)");
}

// ── B2 Send REPLY (type=2) — response to REQUEST ──
void SatRouter::SendReply(const std::string &target) {
  if (m_gs.empty() || !m_nb.count(target)) return;

  std::string myName = Names::FindName(GetNode());
  uint8_t buf[512]; int pos = 0;
  // Fixed header
  buf[pos++] = 0x42; buf[pos++] = 0x32;  // Magic "B2"
  buf[pos++] = 1;                          // Version
  buf[pos++] = 2;                          // Type = REPLY
  pos += 2;                                // TotalLen placeholder
  uint16_t seq = m_seq++;
  memcpy(buf + pos, &seq, 2); pos += 2;   // Sequence Number
  // Name
  buf[pos++] = (uint8_t)myName.size();
  memcpy(buf + pos, myName.c_str(), myName.size()); pos += myName.size();
  // GS entries
  int n = 0; for (auto &p : m_gs) n++;
  buf[pos++] = (uint8_t)n;
  for (auto &p : m_gs) {
    uint8_t nl = p.first.size(); buf[pos++] = nl;
    memcpy(buf + pos, p.first.c_str(), nl); pos += nl;
    int32_t bw = (int32_t)p.second;
    memcpy(buf + pos, &bw, 4); pos += 4;
  }
  // Fill Total Length
  uint16_t total = (uint16_t)pos;
  memcpy(buf + 4, &total, 2);

  Ptr<Packet> pkt = Create<Packet>(buf, pos);
  m_sk->SendTo(pkt, 0, InetSocketAddress(m_nb[target].ip, 9997));
  NS_LOG_UNCOND("  [REPLY " << myName << "] seq=" << seq
    << " → " << target << " (len=" << total << "B)");
}

// ── B2 Receive (all types) ──
// Fixed header: Magic(2) + Version(1) + Type(1) + TotalLen(2) + Seq(2) = 8 bytes
// On PUSH/REPLY: update m_gs, track source, then propagate to our neighbors
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
    uint8_t nameLen = buf[8];
    std::string nbName((char*)buf + 9, nameLen);
    int pos = 9 + nameLen;

    double nbLink = 1e9;
    if (m_nbMon.count(nbName)) { nbLink = m_nbMon[nbName].availBw; }
    else if (m_nb.count(nbName)) nbLink = m_nb[nbName].bw;  // fallback to capacity

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
        uint8_t nl = buf[pos++];
        std::string gs((char*)buf + pos, nl); pos += nl;
        int32_t nbBw; memcpy(&nbBw, buf + pos, 4); pos += 4;
        // Only accept GS info from our routing next-hop for that GS
        if (!m_nexthop.count(gs)) continue;   // no route → ignore
        if (m_nexthop[gs] == "") continue;     // direct → authoritative, don't override
        if (m_nexthop[gs] != nbName) continue; // not our next-hop → ignore
        double newBw = std::min(nbLink, (double)nbBw);
        if (!m_gs.count(gs) || newBw != m_gs[gs]) { m_gs[gs] = newBw; chg = true; }
      }
      if (chg) {
        m_bestBw = 0;
        for (auto &p : m_gs) if (p.second > m_bestBw) { m_bestBw = p.second; m_bestGs = p.first; }
        NS_LOG_UNCOND("  [B2 " << Names::FindName(GetNode()) << "] ←" << nbName
          << " " << (type == 0 ? "PUSH" : "REPLY") << " seq=" << seq
          << " update: best=" << m_bestGs << ":" << (int)m_bestBw
          << "M (via " << nbName << " link=" << (int)nbLink << "M)");
        propagated = true;
      }
    }
  }
  // Propagate: if we learned new info, push to our neighbors
  if (propagated) {
    SendPush();
  }
}

// ── Port change detection ──
void SatRouter::CheckPortChange() {
  std::string my = Names::FindName(GetNode());
  double now = Simulator::Now().GetSeconds();

  // Real per-neighbor monitoring: read queue counters
  double dt = (m_lastMonTime > 0) ? (now - m_lastMonTime) : 0.2;
  if (dt < 0.01) dt = 0.2; m_lastMonTime = now;
  for (auto &kv : m_nbMon) {
    if(now < 3.0 && !kv.second.queue) NS_LOG_UNCOND("  [DBG "<<my<<"→"<<kv.first<<" NULL Q]");
    if(now < 3.0 && kv.second.queue) { uint64_t c=kv.second.queue->GetTotalReceivedBytes(); NS_LOG_UNCOND("  [DBG "<<my<<"→"<<kv.first<<" q="<<c<<" last="<<kv.second.lastBytes<<" cap="<<(m_nb.count(kv.first)?(int)m_nb[kv.first].bw:0)<<"M]"); }
    if (!kv.second.queue) continue;
    uint64_t cur = kv.second.queue->GetTotalReceivedBytes();
    uint64_t delta = cur - kv.second.lastBytes;
    double tputMbps = (delta * 8.0 / 1e6) / dt;
    double cap = m_gsDirect.count(kv.first) ? m_gsDirect[kv.first] : (m_nb.count(kv.first) ? m_nb[kv.first].bw : 1e9);
    kv.second.availBw = cap - tputMbps;
    if(now < 3.0) NS_LOG_UNCOND("  [MON-LOOP " << my << "->" << kv.first << " delta=" << delta << " tput=" << tputMbps << " cap=" << cap << " avail=" << kv.second.availBw);
    if (kv.second.availBw < 0) kv.second.availBw = 0;
    kv.second.lastBytes = cur;
  }
  for (auto &gs : m_gsDirect) {
    if (m_nbMon.count(gs.first)) m_gs[gs.first] = m_nbMon[gs.first].availBw;
  }

  if (!m_initialized && !m_gs.empty()) {
    m_initialized = true;
    m_nbSnapshot.clear();
    for (auto &nb : m_nb) m_nbSnapshot[nb.first] = nb.second.bw;
    SendPush();
  } else if (m_initialized) {
    bool changed = false;
    if (m_nb.size() != m_nbSnapshot.size()) changed = true;
    else for (auto &nb : m_nb) if (!m_nbSnapshot.count(nb.first) || std::abs(m_nbSnapshot[nb.first] - nb.second.bw) > 0.5) { changed = true; break; }
    if (changed) {
      for (auto &nb : m_nb) {
        double old = m_nbSnapshot.count(nb.first) ? m_nbSnapshot[nb.first] : 0;
        if (std::abs(nb.second.bw - old) > 0.5 || old == 0)
          for (auto &gs : m_nexthop) if (gs.second == nb.first) m_gs[gs.first] = 0;
      }
      SendPush();
      m_nbSnapshot.clear();
      for (auto &nb : m_nb) m_nbSnapshot[nb.first] = nb.second.bw;
      NS_LOG_UNCOND("  [MON " << my << "] port change → PUSH (best=" << m_bestGs << ":" << m_bestBw << "M)");
    }
  }
  m_bestBw = 0;
  for (auto &p : m_gs) if (p.second > m_bestBw) { m_bestBw = p.second; m_bestGs = p.first; }
  m_checkTimer = Simulator::Schedule(Seconds(0.20), &SatRouter::CheckPortChange, this);
}


// ═══════════════════ gNB ═══════════════════

class GnbApp : public Application {
public:
  static TypeId GetTypeId(); GnbApp();
  void AddCoverage(Ipv4Address ua) { m_cov.push_back(ua); }
  void SendN2Query(); void Push();
private:
  virtual void StartApplication() override {
    m_sk = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9999));
    m_sk->SetRecvCallback(MakeCallback(&GnbApp::RecvN2, this));
  }
  virtual void StopApplication() override { if (m_sk) m_sk->Close(); }
  void RecvN2(Ptr<Socket> s);
  Ptr<Socket> m_sk;
  std::vector<Ipv4Address> m_cov;
  uint16_t m_seq = 0;
  double m_bw = 0;
  std::string m_bl;
  bool m_wait = false;
};
NS_OBJECT_ENSURE_REGISTERED(GnbApp);
TypeId GnbApp::GetTypeId() {
  static TypeId tid = TypeId("ns3::GnbApp").SetParent<Application>()
    .SetGroupName("Sbdp").AddConstructor<GnbApp>();
  return tid;
}
GnbApp::GnbApp() {}

void GnbApp::SendN2Query() {
  uint8_t q = 0;
  m_sk->SendTo(Create<Packet>(&q, 1), 0,
    InetSocketAddress(Ipv4Address("127.0.0.1"), 9998));
}
void GnbApp::RecvN2(Ptr<Socket> s) {
  Ptr<Packet> pkt; Address from;
  while ((pkt = s->RecvFrom(from))) {
    uint8_t buf[512]; pkt->CopyData(buf, pkt->GetSize());
    if (pkt->GetSize() < 1) continue;
    int n = buf[0], pos = 1;
    for (int i = 0; i < n && pos < (int)pkt->GetSize(); i++) {
      int32_t bw; memcpy(&bw, buf + pos, 4); pos += 4;
      uint32_t nl; memcpy(&nl, buf + pos, 4); pos += 4;
      std::string gs((char*)buf + pos, nl); pos += nl;
      uint32_t blen; memcpy(&blen, buf + pos, 4); pos += 4;
      std::string bl((char*)buf + pos, blen); pos += blen;
      NS_LOG_UNCOND("  [gNB " << Names::FindName(GetNode())
        << "] N2: " << gs << "=" << bw << "M");
      if (m_wait) {
        m_seq++;
        for (auto &ua : m_cov) {
          SbdpHeader h = SbdpHeader::BuildAdv(
            Names::FindName(GetNode()), "UE", (double)bw, gs + ":" + bl, m_seq);
          Ptr<Packet> p = Create<Packet>(0); p->AddHeader(h);
          m_sk->SendTo(p, 0, InetSocketAddress(ua, 8888));
        }
      }
    }
    if (m_wait) m_wait = false;
  }
}
void GnbApp::Push() {
  // Always re-query N2 for latest bottleneck
  m_wait = true; SendN2Query();
}


// ═══════════════════ UsrApp ═══════════════════

class UsrApp : public Application {
public:
  static TypeId GetTypeId(); UsrApp();
  std::map<std::string, float> m_map;
  std::string m_cur;
  int m_ho = 0;
  float m_thr = 200.0;
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
        std::string sn = h.GetSrcNode();
        m_map[sn] = h.GetBackhaulBw();
        if (m_cur.empty()) m_cur = sn;
        NS_LOG_UNCOND("  [" << Names::FindName(GetNode()) << " @"
          << Simulator::Now().GetSeconds() << "s] ←" << sn
          << " bw=" << (int)h.GetBackhaulBw() << "M map=" << ([&]() {
            std::ostringstream ss;
            for (auto &p : m_map) {
              if (&p != &*m_map.begin()) ss << ",";
              ss << p.first << ":" << (int)p.second;
            }
            return ss.str();
          })());
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
UsrApp::UsrApp() {}


// ═══════════════════ main ═══════════════════


// ═══════ Exp 3 — 6-sat orbital ISL dynamics ═══════
int main(int argc, char *argv[]) {
  double simTime = 100;
  NodeContainer n; n.Create(10);
  Names::Add("U1",n.Get(0)); Names::Add("U2",n.Get(1));
  Names::Add("A1",n.Get(2)); Names::Add("A2",n.Get(3)); Names::Add("A3",n.Get(4));
  Names::Add("B1",n.Get(5)); Names::Add("B2",n.Get(6)); Names::Add("B3",n.Get(7));
  Names::Add("GS-E",n.Get(8)); Names::Add("GS-W",n.Get(9));
  InternetStackHelper inet; inet.Install(n);
  PointToPointHelper p2p; p2p.SetQueue("ns3::DropTailQueue"); Ipv4AddressHelper ipv4; uint32_t sn=0;
  auto L=[&](int a,int b,double bw,double d=5){
    p2p.SetDeviceAttribute("DataRate",StringValue(std::to_string((int)bw)+"Mbps"));
    p2p.SetChannelAttribute("Delay",StringValue(std::to_string(d)+"ms"));
    auto dev=p2p.Install(n.Get(a),n.Get(b));
    char base[32]; snprintf(base,32,"10.%u.0.0",sn++); ipv4.SetBase(base,"255.255.255.0"); ipv4.Assign(dev); return dev;
  };
  auto setL=[&](NetDeviceContainer &dev,double bw){DynamicCast<PointToPointNetDevice>(dev.Get(0))->SetDataRate(DataRate(bw*1e6));DynamicCast<PointToPointNetDevice>(dev.Get(1))->SetDataRate(DataRate(bw*1e6));};
  // Intra-plane ISL rings
  auto dA12=L(2,3,200); auto dA23=L(3,4,200);
  auto dB12=L(5,6,200); auto dB23=L(6,7,200);
  // Inter-plane ISL
  auto dAB1=L(2,5,180); auto dAB2=L(3,6,180);
  // Feeder links
  auto dGsE=L(2,8,350); auto dGsW=L(7,9,300);
  // User access + inter-GS
  L(0,2,300,2); L(1,2,300,2); L(8,9,1000,2);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();
  auto ip=[&](int i)->Ipv4Address{return n.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();};

  // ── SatRouters + gNBs ──
  struct Sat { Ptr<SatRouter> r; Ptr<GnbApp> g; };
  std::vector<Sat> sats(6);
  for (int i = 0; i < 6; i++) {
    sats[i].r = CreateObject<SatRouter>();
    sats[i].g = CreateObject<GnbApp>();
    n.Get(2 + i)->AddApplication(sats[i].r); sats[i].r->SetStartTime(Seconds(0));
    n.Get(2 + i)->AddApplication(sats[i].g); sats[i].g->SetStartTime(Seconds(0.05));
  }

  // ── Topology ──
  sats[0].r->SetBw(300,600); // A1
  sats[0].r->AddNeighbor("A2",ip(3),200); sats[0].r->AddNeighbor("B1",ip(5),180);
  sats[0].r->AddDirectGs("GS-E",350); sats[0].r->SetRoute("GS-E","");

  sats[1].r->SetBw(300,600); // A2
  sats[1].r->AddNeighbor("A1",ip(2),200); sats[1].r->AddNeighbor("A3",ip(4),200);
  sats[1].r->AddNeighbor("B2",ip(6),180); sats[1].r->SetRoute("GS-E","A1");

  sats[2].r->SetBw(300,600); // A3
  sats[2].r->AddNeighbor("A2",ip(3),200); sats[2].r->SetRoute("GS-E","A1");

  sats[3].r->SetBw(300,600); // B1
  sats[3].r->AddNeighbor("B2",ip(6),200); sats[3].r->AddNeighbor("A1",ip(2),180);
  sats[3].r->SetRoute("GS-W","B3"); sats[3].r->SetRoute("GS-E","A1");

  sats[4].r->SetBw(300,600); // B2
  sats[4].r->AddNeighbor("B1",ip(5),200); sats[4].r->AddNeighbor("B3",ip(7),200);
  sats[4].r->AddNeighbor("A2",ip(3),180);
  sats[4].r->SetRoute("GS-W","B3"); sats[4].r->SetRoute("GS-E","A2");

  sats[5].r->SetBw(300,600); // B3
  sats[5].r->AddNeighbor("B2",ip(6),200); sats[5].r->AddDirectGs("GS-W",300);
  sats[5].r->SetRoute("GS-W","");

  // Coverage: all sats cover both UEs
  for (int i = 0; i < 6; i++) { sats[i].g->AddCoverage(ip(0)); sats[i].g->AddCoverage(ip(1)); }

  // ── Install real per-neighbor queue monitors ──
  sats[0].r->InstallMonitor("A2", dA12.Get(0)); sats[1].r->InstallMonitor("A1", dA12.Get(1));
  sats[1].r->InstallMonitor("A3", dA23.Get(0)); sats[2].r->InstallMonitor("A2", dA23.Get(1));
  sats[3].r->InstallMonitor("B2", dB12.Get(0)); sats[4].r->InstallMonitor("B1", dB12.Get(1));
  sats[4].r->InstallMonitor("B3", dB23.Get(0)); sats[5].r->InstallMonitor("B2", dB23.Get(1));
  sats[0].r->InstallMonitor("B1", dAB1.Get(0)); sats[3].r->InstallMonitor("A1", dAB1.Get(1));
  sats[1].r->InstallMonitor("B2", dAB2.Get(0)); sats[4].r->InstallMonitor("A2", dAB2.Get(1));
  sats[0].r->InstallMonitor("GS-E", dGsE.Get(0));
  sats[5].r->InstallMonitor("GS-W", dGsW.Get(0));

  // ── Users ──
  Ptr<UsrApp> u1 = CreateObject<UsrApp>(); n.Get(0)->AddApplication(u1); u1->SetStartTime(Seconds(0.05));
  Ptr<UsrApp> u2 = CreateObject<UsrApp>(); n.Get(1)->AddApplication(u2); u2->SetStartTime(Seconds(0.05));
  std::vector<Ptr<UsrApp>> users = {u1, u2};

  // ── Data flows: both users send 100Mbps to GS-E ──
  Ipv4Address gsIp = ip(8);
  std::vector<Ptr<PacketSink>> sinks;
  for (int i = 0; i < 2; i++) {
    PacketSinkHelper sk("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 5001 + i));
    auto a = sk.Install(n.Get(8)); a.Start(Seconds(0.3)); a.Stop(Seconds(simTime));
    sinks.push_back(DynamicCast<PacketSink>(a.Get(0)));
    OnOffHelper oo("ns3::UdpSocketFactory", InetSocketAddress(gsIp, 5001 + i));
    oo.SetAttribute("DataRate", DataRateValue(DataRate("100Mbps")));
    oo.SetAttribute("PacketSize", UintegerValue(1472));
    oo.SetConstantRate(DataRate("100Mbps"));
    auto c = oo.Install(n.Get(i)); c.Start(Seconds(0.5)); c.Stop(Seconds(simTime));
  }

  // ── gNB push helper ──
  auto pushAll = [&]() { for (auto &s : sats) { s.g->Push(); } };

  // ═══ Orbital Events ═══
  Simulator::Schedule(Seconds(20), [&]() {
    NS_LOG_UNCOND("\n═══ t=20s A2↔B2 inter-plane ISL lost ═══");
    setL(dAB2, 0);
    sats[1].r->AddNeighbor("B2", ip(6), 0);
    sats[4].r->AddNeighbor("A2", ip(3), 0);
    pushAll();
  });

  Simulator::Schedule(Seconds(40), [&]() {
    NS_LOG_UNCOND("\n═══ t=40s GS-E rain fade 350→80M ═══");
    setL(dGsE, 80);
    sats[0].r->AddDirectGs("GS-E", 80);
    pushAll();
  });

  Simulator::Schedule(Seconds(60), [&]() {
    NS_LOG_UNCOND("\n═══ t=60s A2↔B2 ISL restored ═══");
    setL(dAB2, 180);
    sats[1].r->AddNeighbor("B2", ip(6), 180);
    sats[4].r->AddNeighbor("A2", ip(3), 180);
    pushAll();
  });

  Simulator::Schedule(Seconds(80), [&]() {
    NS_LOG_UNCOND("\n═══ t=80s GS-E recovered 80→350M ═══");
    setL(dGsE, 350);
    sats[0].r->AddDirectGs("GS-E", 350);
    pushAll();
  });

  // ── PCAP ──
  p2p.EnablePcap("sbdp-orbit", n.Get(2)->GetDevice(0), true);

  // ── Report ──
  Simulator::Schedule(Seconds(simTime - 2), [&]() {
    NS_LOG_UNCOND("\n═══ Exp 3 Report ═══");
    for (int i = 0; i < 2; i++) {
      double mb = sinks[i]->GetTotalRx() / 1e6;
      NS_LOG_UNCOND("  U" << (i + 1) << ": data=" << mb << "MB cur=" << users[i]->m_cur);
    }
    for (int i = 0; i < 6; i++)
      NS_LOG_UNCOND("  " << Names::FindName(n.Get(2 + i)) << " B2=" << sats[i].r->GetBestE2e() << "M");
    NS_LOG_UNCOND("  Events: A2↔B2 ISL lost/restored + GS-E rain/recovered");
  });

  NS_LOG_UNCOND("\n═══ Exp 3: Orbital ISL Dynamics ═══\n");
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
