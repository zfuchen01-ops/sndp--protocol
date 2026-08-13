/* SBDP Exp 2 — SbdpHeader Pull+Push + Port Monitoring + N2 + ADV (A)
 *
 * ISL: Pull-init + Push-on-change (port monitoring).
 *      Satellites use SbdpHeader (SBDP_LINK_PROBE / SBDP_CAPACITY_REQ / SBDP_CAPACITY_ACK).
 *      On change → instant PROBE to all neighbors (one per GS).
 *      On REQUEST → ACK with current GS info.
 *      Stable period → zero ISL traffic.
 *
 * N2: gNB queries local Router via SBDP_CAPACITY_REQ → gets GS table via SBDP_CAPACITY_ACK.
 * ADV: gNB broadcasts SbdpHeader::BuildAdv to covered UEs.
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

// ═══════════════════ SatRouter: SbdpHeader Pull+Push + Port Monitoring ═══════════════════

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
  uint16_t m_seq = 0;  // SbdpHeader message sequence number
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

// ── N2: gNB queries Router (SBDP_CAPACITY_REQ → SBDP_CAPACITY_ACK per GS) ──
void SatRouter::RecvN2(Ptr<Socket> s) {
  Ptr<Packet> pkt; Address from;
  while ((pkt = s->RecvFrom(from))) {
    // Send one SBDP_CAPACITY_ACK per GS entry
    std::string myName = Names::FindName(GetNode());
    for (auto &p : m_gs) {
      SbdpHeader h = SbdpHeader::BuildN2Ack(myName, "gNB", p.first, (float)p.second, m_seq++);
      Ptr<Packet> resp = Create<Packet>(0); resp->AddHeader(h);
      s->SendTo(resp, 0, from);
    }
    NS_LOG_UNCOND("  [R " << myName << "] N2→gNB: " << m_gs.size() << " GS (SbdpHeader ACK), best=" << m_bestGs << ":" << (int)m_bestBw << "M");
  }
}

// ── Send ISL push — SBDP_LINK_PROBE per GS ──
void SatRouter::SendPush() {
  if (m_gs.empty()) return;

  std::string myName = Names::FindName(GetNode());
  int n = 0;
  for (auto &p : m_gs) {
    SbdpHeader h = SbdpHeader::BuildIslPush(myName, p.first, (float)p.second, m_seq++);
    Ptr<Packet> pkt = Create<Packet>(0); pkt->AddHeader(h);
    for (auto &nb : m_nb)
      m_sk->SendTo(pkt, 0, InetSocketAddress(nb.second.ip, 9997));
    n++;
  }

  // Update snapshots
  m_nbSnapshot.clear();
  for (auto &nb : m_nb) m_nbSnapshot[nb.first] = nb.second.bw;

  NS_LOG_UNCOND("  [PROBE " << myName << "] seq=" << (m_seq-1) << " → " << m_nb.size()
    << " nb, best=" << m_bestGs << ":" << (int)m_bestBw << "M (" << n << " GS×SbdpHeader PROBE)");
}

// ── Send ISL request — SBDP_CAPACITY_REQ (16B) ──
void SatRouter::SendRequest() {
  std::string myName = Names::FindName(GetNode());
  SbdpHeader h = SbdpHeader::BuildIslReq(myName, m_seq++);
  Ptr<Packet> pkt = Create<Packet>(0); pkt->AddHeader(h);
  for (auto &nb : m_nb)
    m_sk->SendTo(pkt, 0, InetSocketAddress(nb.second.ip, 9997));
  NS_LOG_UNCOND("  [REQ " << myName << "] seq=" << (m_seq-1) << " → " << m_nb.size()
    << " nb (SbdpHeader REQ)");
}

// ── Send ISL reply — SBDP_CAPACITY_ACK per GS ──
void SatRouter::SendReply(const std::string &target) {
  if (m_gs.empty() || !m_nb.count(target)) return;

  std::string myName = Names::FindName(GetNode());
  int n = 0;
  for (auto &p : m_gs) {
    SbdpHeader h = SbdpHeader::BuildIslReply(myName, target, p.first, (float)p.second, m_seq++);
    Ptr<Packet> pkt = Create<Packet>(0); pkt->AddHeader(h);
    m_sk->SendTo(pkt, 0, InetSocketAddress(m_nb[target].ip, 9997));
    n++;
  }
  NS_LOG_UNCOND("  [REPLY " << myName << "] seq=" << (m_seq-1)
    << " → " << target << " (" << n << " GS×SbdpHeader ACK)");
}

// ── ISL Receive — SbdpHeader deserialize + TLV parse ──
void SatRouter::RecvEx(Ptr<Socket> s) {
  Ptr<Packet> pkt; Address from;
  bool propagated = false;
  while ((pkt = s->RecvFrom(from))) {
    if (pkt->GetSize() < SbdpHeader::SBDP_FIXED_SIZE) continue;

    SbdpHeader h;
    pkt->RemoveHeader(h);

    uint8_t type = h.GetMsgType();
    std::string nbName = h.GetSrcNode();
    uint16_t seq = h.GetSeqNum();

    double nbLink = 1e9;
    if (m_nbMon.count(nbName)) { nbLink = m_nbMon[nbName].availBw; }
    else if (m_nb.count(nbName)) nbLink = m_nb[nbName].bw;

    if (type == SBDP_CAPACITY_REQ) {
      // ── REQUEST: reply with our GS info ──
      NS_LOG_UNCOND("  [SBDP " << Names::FindName(GetNode()) << "] ←" << nbName
        << " REQ seq=" << seq);
      SendReply(nbName);
    } else if (type == SBDP_LINK_PROBE || type == SBDP_CAPACITY_ACK) {
      // ── PROBE or ACK: update single GS from neighbor ──
      std::string gs = h.GetBottleneckLink();
      float nbBw = h.GetBackhaulBw();
      if (gs.empty()) continue;

      // Only accept GS info from our routing next-hop for that GS
      if (!m_nexthop.count(gs)) continue;
      if (m_nexthop[gs] == "") continue;     // direct → authoritative
      if (m_nexthop[gs] != nbName) continue; // not our next-hop → ignore
      double newBw = std::min(nbLink, (double)nbBw);
      if (!m_gs.count(gs) || newBw != m_gs[gs]) {
        m_gs[gs] = newBw;
        m_bestBw = 0;
        for (auto &p : m_gs) if (p.second > m_bestBw) { m_bestBw = p.second; m_bestGs = p.first; }
        NS_LOG_UNCOND("  [SBDP " << Names::FindName(GetNode()) << "] ←" << nbName
          << " " << (type == SBDP_LINK_PROBE ? "PROBE" : "ACK") << " seq=" << seq
          << " gs=" << gs << " bw=" << (int)nbBw << "M newBw=" << (int)newBw
          << "M best=" << m_bestGs << ":" << (int)m_bestBw << "M");
        propagated = true;
      }
    }
  }
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
    // Parse SBDP_CAPACITY_ACK from Router
    if (pkt->GetSize() < SbdpHeader::SBDP_FIXED_SIZE) continue;
    SbdpHeader h;
    pkt->RemoveHeader(h);
    if (h.GetMsgType() != SBDP_CAPACITY_ACK) continue;

    std::string gs = h.GetBottleneckLink();
    float bw = h.GetBackhaulBw();
    std::string src = h.GetSrcNode();

    NS_LOG_UNCOND("  [gNB " << Names::FindName(GetNode())
      << "] N2: " << gs << "=" << (int)bw << "M (SbdpHeader ACK from " << src << ")");
    if (m_wait) {
      m_seq++;
      for (auto &ua : m_cov) {
        SbdpHeader adv = SbdpHeader::BuildAdv(
          Names::FindName(GetNode()), "UE", (double)bw, gs, m_seq);
        Ptr<Packet> p = Create<Packet>(0); p->AddHeader(adv);
        m_sk->SendTo(p, 0, InetSocketAddress(ua, 8888));
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


int main(){
  NodeContainer n;n.Create(4);
  Names::Add("U1",n.Get(0));Names::Add("U2",n.Get(1));Names::Add("SAT-A",n.Get(2));Names::Add("GS",n.Get(3));
  InternetStackHelper inet;inet.Install(n);
  PointToPointHelper p2p;p2p.SetQueue("ns3::DropTailQueue");Ipv4AddressHelper ipv4;
  p2p.SetDeviceAttribute("DataRate",StringValue("350Mbps"));p2p.SetChannelAttribute("Delay",StringValue("5ms"));
  auto dFeed=p2p.Install(n.Get(2),n.Get(3));ipv4.SetBase("10.0.0.0","255.255.255.0");ipv4.Assign(dFeed);
  p2p.SetDeviceAttribute("DataRate",StringValue("300Mbps"));ipv4.SetBase("10.0.1.0","255.255.255.0");auto du1=p2p.Install(n.Get(0),n.Get(2));ipv4.Assign(du1);
  ipv4.SetBase("10.0.2.0","255.255.255.0");auto du2=p2p.Install(n.Get(1),n.Get(2));ipv4.Assign(du2);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();
  auto ip=[&](int i)->Ipv4Address{return n.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();};

  Ptr<SatRouter> r=CreateObject<SatRouter>();Ptr<GnbApp> g=CreateObject<GnbApp>();
  n.Get(2)->AddApplication(r);r->SetStartTime(Seconds(0));n.Get(2)->AddApplication(g);g->SetStartTime(Seconds(0.05));
  r->SetBw(300,350);r->AddDirectGs("GS",350);r->SetRoute("GS","");
  r->InstallMonitor("GS",DynamicCast<PointToPointNetDevice>(dFeed.Get(0)));

  g->AddCoverage(ip(0));g->AddCoverage(ip(1));
  Ptr<UsrApp> u1=CreateObject<UsrApp>();n.Get(0)->AddApplication(u1);u1->SetStartTime(Seconds(0.05));
  Ptr<UsrApp> u2=CreateObject<UsrApp>();n.Get(1)->AddApplication(u2);u2->SetStartTime(Seconds(0.05));

  Ipv4Address gsIp=ip(3);
  PacketSinkHelper sk("ns3::UdpSocketFactory",InetSocketAddress(Ipv4Address::GetAny(),5001));
  auto sink1=sk.Install(n.Get(3));sink1.Start(Seconds(0.3));sink1.Stop(Seconds(50));
  OnOffHelper oo1("ns3::UdpSocketFactory",InetSocketAddress(gsIp,5001));oo1.SetAttribute("DataRate",DataRateValue(DataRate("100Mbps")));oo1.SetAttribute("PacketSize",UintegerValue(1472));oo1.SetConstantRate(DataRate("100Mbps"));
  auto src1=oo1.Install(n.Get(0));src1.Start(Seconds(0.5));src1.Stop(Seconds(50));

  PacketSinkHelper sk2("ns3::UdpSocketFactory",InetSocketAddress(Ipv4Address::GetAny(),5002));
  auto sink2=sk2.Install(n.Get(3));sink2.Start(Seconds(0.3));sink2.Stop(Seconds(50));
  OnOffHelper oo2("ns3::UdpSocketFactory",InetSocketAddress(gsIp,5002));oo2.SetAttribute("DataRate",DataRateValue(DataRate("100Mbps")));oo2.SetAttribute("PacketSize",UintegerValue(1472));oo2.SetConstantRate(DataRate("100Mbps"));
  auto src2=oo2.Install(n.Get(1));src2.Start(Seconds(0.5));src2.Stop(Seconds(50));

  // t=30: User-1 handover (flow stops)
  Simulator::Schedule(Seconds(30),[&](){NS_LOG_UNCOND("\n═══ t=30s User-1 handover ═══");src1.Get(0)->Dispose();});

  // Checkpoints
  Simulator::Schedule(Seconds(20),[=](){double r1=DynamicCast<PacketSink>(sink1.Get(0))->GetTotalRx()/1e6,r2=DynamicCast<PacketSink>(sink2.Get(0))->GetTotalRx()/1e6;NS_LOG_UNCOND("═══ @20s: U1="<<r1<<"MB U2="<<r2<<"MB B2="<<r->GetBestE2e()<<"M ═══");});
  Simulator::Schedule(Seconds(45),[=](){double r1=DynamicCast<PacketSink>(sink1.Get(0))->GetTotalRx()/1e6,r2=DynamicCast<PacketSink>(sink2.Get(0))->GetTotalRx()/1e6;NS_LOG_UNCOND("═══ @45s: U1="<<r1<<"MB U2="<<r2<<"MB B2="<<r->GetBestE2e()<<"M ═══");});

  NS_LOG_UNCOND("\n═══ Aware: Handover → B2 Change ═══\n");
  p2p.EnablePcap("sbdp-aware-feed",dFeed.Get(0),true);p2p.EnablePcap("sbdp-aware-user1",du1.Get(0),true);Simulator::Stop(Seconds(50));Simulator::Run();Simulator::Destroy();return 0;
}
