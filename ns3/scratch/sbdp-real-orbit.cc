/* SBDP Exp 3 — Real Orbit Events + Heartbeat + Seq Detection
 *
 * 6 sats (2 orbits × 3) + 2 GS + 3 users, 870s session.
 * 17 events timed from 36-sat Walker constellation.
 * Heartbeat 30s, seq gap detection, per-hop backhaul + QoS.
 * Real UDP sockets, SBDP headers, PCAP.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/sbdp-header.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include <map><vector><set><algorithm><cmath>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("SbdpRealOrbit");

class SatFwd : public Application {
public:
  static TypeId GetTypeId(); SatFwd();
  void SetBw(double b) { m_bw=b; }
  double GetBw() const { return m_bw; }
  void AddUser(Ipv4Address ua) { m_users.push_back(ua); }
  void RemoveUser(Ipv4Address ua) { m_users.erase(std::remove(m_users.begin(),m_users.end(),ua),m_users.end()); }
  void Push();
  void StartHb() { m_hb = Simulator::Schedule(Seconds(m_hbInt), &SatFwd::OnHb, this); m_lastPush = Simulator::Now().GetSeconds(); }
private:
  virtual void StartApplication() override { m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId()); m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9999)); StartHb(); }
  virtual void StopApplication() override { Simulator::Cancel(m_hb); if(m_sk) m_sk->Close(); }
  void OnHb() {
    double now = Simulator::Now().GetSeconds();
    bool hbExp = (now - m_lastPush) >= m_hbInt;
    bool sigChg = std::abs(m_bw - m_lastBw) / std::max(1.0, m_lastBw) > 0.05;
    if (sigChg || hbExp || m_lastBw < 0) {
      Push();
      NS_LOG_UNCOND("  [SAT " << Names::FindName(GetNode())
                    << (sigChg ? " PUSH" : " HEARTBEAT")
                    << " seq=" << (m_seq-1) << " bw=" << (int)m_bw << "M]");
    }
    m_hb = Simulator::Schedule(Seconds(m_hbInt), &SatFwd::OnHb, this);
  }
  Ptr<Socket> m_sk; double m_bw=0, m_lastBw=-1, m_lastPush=0, m_hbInt=30.0;
  uint16_t m_seq=0; EventId m_hb; std::vector<Ipv4Address> m_users;
};
NS_OBJECT_ENSURE_REGISTERED(SatFwd);
TypeId SatFwd::GetTypeId(){static TypeId tid=TypeId("ns3::SatFwd").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<SatFwd>();return tid;}
SatFwd::SatFwd(){}

void SatFwd::Push() {
  m_seq++;
  for (auto &ua : m_users) {
    SbdpHeader hdr = SbdpHeader::BuildAdv(
      Names::FindName(GetNode()), "UE", m_bw,
      Names::FindName(GetNode()), m_seq);
    Ptr<Packet> pkt = Create<Packet>(0); pkt->AddHeader(hdr);
    m_sk->SendTo(pkt, 0, InetSocketAddress(ua, 8888));
  }
  m_lastBw = m_bw; m_lastPush = Simulator::Now().GetSeconds();
}

class UsrApp : public Application {
public:
  static TypeId GetTypeId(); UsrApp();
  struct E { double t; float bw; uint16_t seq; };
  std::vector<E> m_h;
  uint32_t m_gaps=0, m_missed=0; bool m_stale=false;
private:
  virtual void StartApplication() override {
    m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
    m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),8888));
    m_sk->SetRecvCallback(MakeCallback(&UsrApp::Recv,this));
    m_staleCheck = Simulator::Schedule(Seconds(10), &UsrApp::CheckStale, this);
  }
  virtual void StopApplication() override { Simulator::Cancel(m_staleCheck); if(m_sk) m_sk->Close(); }
  void Recv(Ptr<Socket> s) {
    Ptr<Packet> pkt; Address from;
    while ((pkt=s->RecvFrom(from))) {
      SbdpHeader hdr;
      if (pkt->GetSize() >= SbdpHeader::SBDP_FIXED_SIZE) {
        pkt->RemoveHeader(hdr);
        double now = Simulator::Now().GetSeconds();
        if (m_lastSeq>0 && hdr.GetSeqNum() != m_lastSeq+1) {
          m_gaps++;
          m_missed += hdr.GetSeqNum()>m_lastSeq ? hdr.GetSeqNum()-m_lastSeq-1 : 0;
        }
        m_lastSeq = hdr.GetSeqNum(); m_lastRecv = now; m_stale = false;
        m_h.push_back({now, hdr.GetBackhaulBw(), hdr.GetSeqNum()});
        NS_LOG_UNCOND("  [" << Names::FindName(GetNode()) << " @" << (int)now
                      << "s] bw=" << (int)hdr.GetBackhaulBw() << "M seq=" << hdr.GetSeqNum());
      }
    }
  }
  void CheckStale() {
    double now = Simulator::Now().GetSeconds(); bool was = m_stale;
    m_stale = (now - m_lastRecv) > 35;
    if (m_stale && !was) NS_LOG_UNCOND("  ⚠ [" << Names::FindName(GetNode()) << " @" << (int)now << "s] STALE");
    if (!m_stale && was) NS_LOG_UNCOND("  ✓ [" << Names::FindName(GetNode()) << "] RECOVERED");
    m_staleCheck = Simulator::Schedule(Seconds(10), &UsrApp::CheckStale, this);
  }
  Ptr<Socket> m_sk; uint16_t m_lastSeq=0; double m_lastRecv=0; EventId m_staleCheck;
};
NS_OBJECT_ENSURE_REGISTERED(UsrApp);
TypeId UsrApp::GetTypeId(){static TypeId tid=TypeId("ns3::UsrApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<UsrApp>();return tid;}
UsrApp::UsrApp(){}

struct Edge { int a,b,bw; };
static double maxMinBw(const std::vector<Edge> &edges, int src, int dst) {
  std::map<int,std::map<int,int>> g;
  for (auto &e : edges) { g[e.a][e.b]=e.bw; g[e.b][e.a]=e.bw; }
  std::map<int,double> bw; for (auto &p : g) bw[p.first]=0; bw[src]=1e9; std::set<int> vis;
  while (true) {
    int best=-1; double bv=-1;
    for (auto &p : bw) { if (vis.count(p.first)) continue; if (p.second>bv) { bv=p.second; best=p.first; } }
    if (best<0||best==dst) return (best==dst) ? bv : 0;
    vis.insert(best);
    for (auto &nb : g[best]) { double pb = std::min(bv,(double)nb.second); if (pb>bw[nb.first]) bw[nb.first]=pb; }
  }
}

int main(int argc, char *argv[]) {
  NodeContainer all; all.Create(12);
  Names::Add("SAT-A1",all.Get(0)); Names::Add("SAT-A2",all.Get(1)); Names::Add("SAT-A3",all.Get(2));
  Names::Add("SAT-B1",all.Get(3)); Names::Add("SAT-B2",all.Get(4)); Names::Add("SAT-B3",all.Get(5));
  Names::Add("GS-East",all.Get(6)); Names::Add("GS-West",all.Get(7));
  Names::Add("User-1",all.Get(8)); Names::Add("User-2",all.Get(9)); Names::Add("User-3",all.Get(10));
  Names::Add("Server",all.Get(11));
  InternetStackHelper inet; inet.Install(all);
  PointToPointHelper p2p; p2p.SetDeviceAttribute("DataRate",StringValue("1Gbps")); p2p.SetChannelAttribute("Delay",StringValue("5ms"));
  Ipv4AddressHelper ipv4; ipv4.SetBase("10.0.0.0","255.255.255.0");
  for (int i=0; i<11; i++) { p2p.Install(all.Get(i),all.Get(i+1)); ipv4.Assign(p2p.Install(all.Get(i),all.Get(i+1))); ipv4.NewNetwork(); }
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  auto lu = [&](int i) -> Ipv4Address { return all.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal(); };

  std::vector<Ptr<SatFwd>> fwds(6);
  for (int i=0; i<6; i++) { fwds[i]=CreateObject<SatFwd>(); all.Get(i)->AddApplication(fwds[i]); fwds[i]->SetStartTime(Seconds(0.1)); }
  std::vector<Ptr<UsrApp>> users(3);
  for (int i=0; i<3; i++) { users[i]=CreateObject<UsrApp>(); all.Get(8+i)->AddApplication(users[i]); users[i]->SetStartTime(Seconds(0.1)); }
  fwds[0]->AddUser(lu(8)); fwds[3]->AddUser(lu(9)); fwds[5]->AddUser(lu(10));

  std::vector<Edge> edges; int userSat[3]={1,4,6};
  auto setE = [&](const std::vector<Edge> &e) { edges=e; };
  auto bh = [&](int sat) { double b1=maxMinBw(edges,sat,7); double b2=maxMinBw(edges,sat,8); return std::max(b1,b2); };

  auto ev = [&](double t, const std::vector<Edge> &e, const char *desc) {
    Simulator::Schedule(Seconds(t), [=]() mutable {
      setE(e); NS_LOG_UNCOND("\n═══ t=" << (int)t << "s " << desc << " ═══");
      for (int i=0; i<6; i++) { double bw = bh(i+1); fwds[i]->SetBw(bw); }
      // Handover evaluation
      for (int ui=0; ui<3; ui++) {
        int cur = userSat[ui]; double curBw = bh(cur);
        if (curBw < 350) {
          double bestBw = curBw; int best = cur;
          for (int si=1; si<=6; si++) {
            if (si==cur) continue; double b = bh(si); if (b>bestBw) { bestBw=b; best=si; }
          }
          if (best != cur) {
            fwds[cur-1]->RemoveUser(lu(8+ui)); fwds[best-1]->AddUser(lu(8+ui)); userSat[ui]=best;
            NS_LOG_UNCOND("  ↳ User-" << (ui+1) << " HANDOVER SAT-" << cur << "→SAT-" << best
                          << " (" << (int)curBw << "→" << (int)bestBw << " Mbps)");
            fwds[best-1]->SetBw(bestBw); fwds[best-1]->Push();
          }
        }
      }
    });
  };

  // ── 17 events from 36-sat Walker constellation ──
  ev(2,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"INITIAL");
  ev(50,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"FAULT: ISL A2↔B2 broken");
  ev(80,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,80},{6,8,300},{4,7,250},{3,8,250}},"RAIN: GS-East 350→80M");
  ev(120,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVER: GS-East restored");
  ev(160,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,0},{4,7,250},{3,8,250}},"FAULT: GS-West down");
  ev(190,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,0},{4,7,250},{3,8,250}},"POLAR BLACKOUT (36-sat event)");
  ev(230,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVER: GS-West+ISL back");
  ev(280,{{1,2,200},{2,3,200},{4,5,200},{5,6,0},{1,4,0},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"DOUBLE FAULT: ISL A1↔B1 + B2↔B3");
  ev(330,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVER: all ISLs restored");
  ev(380,{{1,2,200},{2,3,200},{4,5,0},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,0},{6,8,300},{4,7,0},{3,8,250}},"CASCADE: SAT-B1 isolated");
  ev(430,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVER: SAT-B1 back");
  ev(480,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,100},{6,8,100},{4,7,250},{3,8,250}},"DEGRADE: both GS to 100M");
  ev(510,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,100},{6,8,0},{4,7,250},{3,8,250}},"GS-2 OUTAGE (36-sat event)");
  ev(560,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVER: all links normal");
  ev(620,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,0},{6,8,300},{4,7,250},{3,8,250}},"GS-1 OUTAGE");
  ev(680,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVER: GS-1 back");
  ev(740,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{3,8,350},{4,7,250}},"GS-2 reconnect (36-sat event)");
  ev(830,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,0},{3,8,350},{4,7,250}},"GS-1 OUTAGE (36-sat event)");

  // ── Data flows: each user sends UDP to GS-East (flows through ISL) ──
  Ipv4Address gsAddr = all.Get(6)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
  std::vector<Ptr<PacketSink>> sinks;
  for (int i=0; i<3; i++) {
    PacketSinkHelper sink("ns3::UdpSocketFactory",
      InetSocketAddress(Ipv4Address::GetAny(), 5001+i));
    auto app = sink.Install(all.Get(6)); // GS-East
    app.Start(Seconds(0.3)); app.Stop(Seconds(885));
    sinks.push_back(DynamicCast<PacketSink>(app.Get(0)));

    OnOffHelper onoff("ns3::UdpSocketFactory",
      InetSocketAddress(gsAddr, 5001+i));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate("80Mbps")));
    onoff.SetAttribute("PacketSize", UintegerValue(1472));
    onoff.SetConstantRate(DataRate("80Mbps"));
    auto client = onoff.Install(all.Get(8+i)); // User-i
    client.Start(Seconds(0.5)); client.Stop(Seconds(885));
  }

  p2p.EnablePcap("sbdp-orbit", all.Get(0)->GetDevice(0), true);
  p2p.EnablePcap("sbdp-orbit", all.Get(8)->GetDevice(0), true);

  Simulator::Schedule(Seconds(875), [&]() {
    NS_LOG_UNCOND("\n\n╔══════════════════════════════════════════════╗");
    NS_LOG_UNCOND("║  Exp 3 Report — Orbit + Data Flows            ║");
    NS_LOG_UNCOND("╚══════════════════════════════════════════════╝");
    for (int i=0; i<3; i++) {
      auto &h = users[i]->m_h; if (h.empty()) continue;
      float lo=h[0].bw, hi=h[0].bw;
      for (auto &e : h) { lo=std::min(lo,e.bw); hi=std::max(hi,e.bw); }
      int changes=0; for(size_t j=1;j<h.size();j++) if(std::abs(h[j].bw-h[j-1].bw)>0.5) changes++;
      double mb = sinks[i]->GetTotalRx() / 1e6;
      double mbps = mb * 8.0 / 874.5;
      NS_LOG_UNCOND("  User-" << (i+1) << ": " << h.size() << " SBDP updates (" << changes
                    << " changes + " << (h.size()-changes) << " hb)"
                    << " | " << (int)lo << "-" << (int)hi << " Mbps"
                    << " | data: " << mb << "MB " << mbps << "Mbps"
                    << " | gaps=" << users[i]->m_gaps
                    << " | stale=" << (users[i]->m_stale ? "YES" : "no"));
    }
  });

  NS_LOG_UNCOND("\n╔══════════════════════════════════════════════╗");
  NS_LOG_UNCOND("║  Exp 3: Real Orbit + Heartbeat + Seq          ║");
  NS_LOG_UNCOND("║  6 sats, 2 GS, 3 users, 870s, heartbeat 30s   ║");
  NS_LOG_UNCOND("║  17 events from 36-sat Walker constellation    ║");
  NS_LOG_UNCOND("╚══════════════════════════════════════════════╝\n");

  Simulator::Stop(Seconds(885)); Simulator::Run(); Simulator::Destroy(); return 0;
}
