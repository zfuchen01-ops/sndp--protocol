/* SBDP — Long-Duration Backhaul Tracking over Dynamic Satellite Links
 *
 *    User-1 ──150M── SAT-A ──600M── SAT-B ──350M── GS ──1G── Internet
 *
 * Throughout the session, link bandwidths change (simulating satellite
 * movement, weather, handovers). Satellites continuously push updated
 * backhaul capacity to users. Users track the bottleneck over time.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/sbdp-header.h"

#include <map>
#include <vector>
#include <cmath>
#include <limits>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("SbdpLongSession");

// ════════════════════════════════════════════════════
//  SBDP Forwarder
// ════════════════════════════════════════════════════

class SbdpForwarder : public Application
{
public:
  static TypeId GetTypeId ();
  SbdpForwarder ();
  void SetInBw (double bw);
  void SetOutBw (double bw);
  double GetInBw () const  { return m_inBw; }
  double GetOutBw () const { return m_outBw; }
  void SetNextHop (Ipv4Address addr, uint16_t port)
    { m_nextHopAddr = addr; m_nextHopPort = port; }
  void AddPushTarget (const std::string &id, Ipv4Address addr, uint16_t port);
  void TriggerPush (double now);  // 主动推送

private:
  virtual void StartApplication () override;
  virtual void StopApplication () override;
  void RecvPacket (Ptr<Socket> sock);
  void DoPush (double now, const std::string &dstId, Ipv4Address dstAddr, uint16_t dstPort);

  Ptr<Socket>   m_sock;
  uint16_t      m_listenPort;
  double        m_inBw;
  double        m_outBw;
  Ipv4Address   m_nextHopAddr;
  uint16_t      m_nextHopPort;
  uint16_t      m_seq;
  struct PushTarget { std::string id; Ipv4Address addr; uint16_t port; };
  std::vector<PushTarget> m_pushTargets;
};

NS_OBJECT_ENSURE_REGISTERED (SbdpForwarder);

TypeId SbdpForwarder::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::SbdpForwarder")
    .SetParent<Application> ()
    .SetGroupName ("Sbdp")
    .AddConstructor<SbdpForwarder> ();
  return tid;
}

SbdpForwarder::SbdpForwarder ()
  : m_listenPort (9999), m_inBw (0), m_outBw (0),
    m_nextHopPort (9999), m_seq (0)
{}

void SbdpForwarder::SetInBw (double bw)  { m_inBw = bw; }
void SbdpForwarder::SetOutBw (double bw) { m_outBw = bw; }

void SbdpForwarder::StartApplication ()
{
  m_sock = Socket::CreateSocket (GetNode (),
      UdpSocketFactory::GetTypeId ());
  m_sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), m_listenPort));
  m_sock->SetRecvCallback (MakeCallback (&SbdpForwarder::RecvPacket, this));
}

void SbdpForwarder::StopApplication ()
{
  if (m_sock) m_sock->Close ();
}

void SbdpForwarder::RecvPacket (Ptr<Socket> sock)
{
  Ptr<Packet> packet;
  Address from;
  while ((packet = sock->RecvFrom (from)))
    {
      SbdpHeader hdr;
      bool hasHdr = packet->PeekHeader (hdr);
      if (hasHdr) packet->RemoveHeader (hdr);

      if (!hasHdr)
        {
          hdr.SetBackhaulBw (std::numeric_limits<float>::infinity ());
          hdr.SetSeqNum (m_seq++);
          hdr.SetTtl (20);
          hdr.SetHopCount (0);
          hdr.SetBottleneckLink ("(src)");
        }

      float prevBw = hdr.GetBackhaulBw ();
      double checkBw = m_outBw;
      if (m_inBw > 0 && m_inBw < checkBw) checkBw = m_inBw;
      bool updated = false;

      if (checkBw > 0 && checkBw < prevBw)
        {
          hdr.SetBackhaulBw (checkBw);
          std::string nn = Names::FindName (GetNode ());
          if (nn.empty ()) nn = "node";
          hdr.SetBottleneckLink (nn + "(in=" +
              std::to_string ((int)m_inBw) + "M out=" +
              std::to_string ((int)m_outBw) + "M)");
          updated = true;
        }
      hdr.SetHopCount (hdr.GetHopCount () + 1);
      hdr.SetTtl (hdr.GetTtl () - 1);
      packet->AddHeader (hdr);

      NS_LOG_INFO ("  [SBDP] " << Names::FindName (GetNode ())
                   << " bw=" << checkBw << "M "
                   << prevBw << "→" << hdr.GetBackhaulBw () << "M"
                   << (updated ? " ★" : ""));

      if (hdr.GetTtl () > 0)
        m_sock->SendTo (packet, 0,
            InetSocketAddress (m_nextHopAddr, m_nextHopPort));
    }
}

void SbdpForwarder::DoPush (double now, const std::string &dstId,
                             Ipv4Address dstAddr, uint16_t dstPort)
{
  if (!m_sock) return;  // app not started yet
  Ptr<Packet> packet = Create<Packet> (64);
  SbdpHeader hdr;
  hdr.SetBackhaulBw (std::numeric_limits<float>::infinity ());
  hdr.SetSeqNum (m_seq++);
  hdr.SetTtl (20);
  hdr.SetHopCount (0);

  double checkBw = m_outBw;
  if (m_inBw > 0 && m_inBw < checkBw) checkBw = m_inBw;
  hdr.SetBackhaulBw (checkBw);

  std::string nn = Names::FindName (GetNode ());
  if (nn.empty ()) nn = "node";
  hdr.SetBottleneckLink (nn + "(in=" +
      std::to_string ((int)m_inBw) + "M out=" +
      std::to_string ((int)m_outBw) + "M)");
  hdr.SetSrcNode (nn);
  packet->AddHeader (hdr);

  m_sock->SendTo (packet, 0, InetSocketAddress (dstAddr, dstPort));
}

void SbdpForwarder::TriggerPush (double now)
{
  // 推送给所有直连邻居 (通过 known-push-targets 列表)
  // targets 在 main 中通过 AddPushTarget 设置
  for (auto &t : m_pushTargets)
    DoPush (now, t.id, t.addr, t.port);
}

void SbdpForwarder::AddPushTarget (const std::string &id, Ipv4Address addr, uint16_t port)
{
  m_pushTargets.push_back ({id, addr, port});
}


// ════════════════════════════════════════════════════
//  User — sends probes + tracks history
// ════════════════════════════════════════════════════

struct BwRecord { double time; float backhaul; std::string link; };

class SbdpUserApp : public Application
{
public:
  static TypeId GetTypeId ();
  SbdpUserApp ();
  void SetTarget (Ipv4Address addr, uint16_t port)
    { m_targetAddr = addr; m_targetPort = port; }
  std::vector<BwRecord> GetHistory () const { return m_history; }

private:
  virtual void StartApplication () override;
  virtual void StopApplication () override;
  void RecvPacket (Ptr<Socket> sock);
  void SendProbe ();

  Ptr<Socket>   m_sock;
  uint16_t      m_listenPort;
  Ipv4Address   m_targetAddr;
  uint16_t      m_targetPort;
  uint16_t      m_seq;
  EventId       m_probeEv;
  std::vector<BwRecord> m_history;
};

NS_OBJECT_ENSURE_REGISTERED (SbdpUserApp);

TypeId SbdpUserApp::GetTypeId ()
{
  static TypeId tid = TypeId ("ns3::SbdpUserApp")
    .SetParent<Application> ()
    .SetGroupName ("Sbdp")
    .AddConstructor<SbdpUserApp> ();
  return tid;
}

SbdpUserApp::SbdpUserApp ()
  : m_listenPort (8888), m_targetPort (9999), m_seq (0)
{}

void SbdpUserApp::StartApplication ()
{
  m_sock = Socket::CreateSocket (GetNode (),
      UdpSocketFactory::GetTypeId ());
  m_sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), m_listenPort));
  m_sock->SetRecvCallback (MakeCallback (&SbdpUserApp::RecvPacket, this));
  m_probeEv = Simulator::Schedule (Seconds (1.0),
      &SbdpUserApp::SendProbe, this);
}

void SbdpUserApp::StopApplication ()
{
  Simulator::Cancel (m_probeEv);
  if (m_sock) m_sock->Close ();
}

void SbdpUserApp::SendProbe ()
{
  Ptr<Packet> packet = Create<Packet> (64);
  SbdpHeader hdr;
  hdr.SetBackhaulBw (std::numeric_limits<float>::infinity ());
  hdr.SetSeqNum (m_seq++);
  hdr.SetTtl (20);
  hdr.SetHopCount (0);
  hdr.SetSrcNode ("User-1");
  hdr.SetBottleneckLink ("(start)");
  packet->AddHeader (hdr);
  m_sock->SendTo (packet, 0,
      InetSocketAddress (m_targetAddr, m_targetPort));
  m_probeEv = Simulator::Schedule (Seconds (5.0),
      &SbdpUserApp::SendProbe, this);
}

void SbdpUserApp::RecvPacket (Ptr<Socket> sock)
{
  Ptr<Packet> packet;
  Address from;
  while ((packet = sock->RecvFrom (from)))
    {
      SbdpHeader hdr;
      if (packet->PeekHeader (hdr))
        {
          m_history.push_back ({
              Simulator::Now ().GetSeconds (),
              hdr.GetBackhaulBw (),
              hdr.GetBottleneckLink ()
          });
          NS_LOG_UNCOND ("  [USER@" << Simulator::Now ().GetSeconds ()
                         << "s] backhaul=" << hdr.GetBackhaulBw () << "Mbps "
                         << hdr.GetBottleneckLink ());
        }
    }
}


// ════════════════════════════════════════════════════
//  Topology Controller — periodic link changes
// ════════════════════════════════════════════════════

class TopologyController
{
public:
  void AddForwarder (Ptr<SbdpForwarder> fwd) { m_fwds.push_back (fwd); }
  void AddUser (Ptr<SbdpUserApp> u) { m_users.push_back (u); }
  void ScheduleChanges (double simEnd);
  void TriggerAllPushes ();

  // 接入链路变化: User-1 ↔ SAT-A
  void SetAccessBw (double bw);
  // ISL 变化: SAT-A ↔ SAT-B
  void SetIslBw (double bw);
  // SGL 变化: SAT-B ↔ Server
  void SetSglBw (double bw);

  void PrintReport ();

private:
  std::vector<Ptr<SbdpForwarder>> m_fwds;
  std::vector<Ptr<SbdpUserApp>>   m_users;
  // 索引: 0=SAT-A, 1=SAT-B, 2=Server
};

void TopologyController::SetAccessBw (double bw)
{
  if (m_fwds.size () > 0) m_fwds[0]->SetInBw (bw);
}

void TopologyController::SetIslBw (double bw)
{
  if (m_fwds.size () > 0) m_fwds[0]->SetOutBw (bw);
  if (m_fwds.size () > 1) m_fwds[1]->SetInBw (bw);
}

void TopologyController::SetSglBw (double bw)
{
  if (m_fwds.size () > 1) m_fwds[1]->SetOutBw (bw);
  if (m_fwds.size () > 2) m_fwds[2]->SetInBw (bw);
}

void TopologyController::TriggerAllPushes ()
{
  double now = Simulator::Now ().GetSeconds ();
  // SAT-A 推给 User-1 (回复到 user 的接收端口)
  // SAT-B 推给 SAT-A (再由 SAT-A 转发...简化: 直接推)
  for (auto &fwd : m_fwds)
    fwd->TriggerPush (now);
}

void TopologyController::ScheduleChanges (double simEnd)
{
  // 模拟卫星运动导致的链路带宽变化
  // t=0:   150 / 600 / 350 Mbps  (初始)
  // t=20:  雨衰, SGL 降到 120M  → 瓶颈变为 120
  // t=40:  恢复, SGL 回到 400M  → 瓶颈回到 150
  // t=60:  用户移动到更差位置, 接入链路降到 80M → 瓶颈 80
  // t=80:  ISL 拥塞降到 100M, SGL 350  → 瓶颈 80 (接入限制)
  // t=100: 接入恢复到 200, ISL 100       → 瓶颈 100 (ISL限制)
  // t=120: ISL 恢复 600, SGL 200         → 瓶颈 150 (接入)
  // t=140: 全部恢复

  struct Change { double t; double access; double isl; double sgl; const char *desc; };
  std::vector<Change> schedule = {
    {  2, 150, 600, 350, "initial (warmup)"},
    { 20, 150, 600, 120, "rain fade on SGL"},
    { 40, 150, 600, 400, "SGL recovery"},
    { 60,  80, 600, 400, "user moves (worse access)"},
    { 80,  80, 100, 350, "ISL congestion"},
    {100, 200, 100, 350, "access improves, ISL still congested"},
    {120, 150, 600, 200, "ISL recovery, SGL partial"},
    {140, 150, 600, 350, "full recovery"},
  };

  for (auto &ch : schedule)
    {
      if (ch.t >= simEnd) break;
      Simulator::Schedule (Seconds (ch.t),
          [this, ch] () {
            SetAccessBw (ch.access);
            SetIslBw (ch.isl);
            SetSglBw (ch.sgl);
            double minBw = std::min ({ch.access, ch.isl, ch.sgl, 1000.0});
            NS_LOG_UNCOND ("\n═══ t=" << ch.t << "s " << ch.desc
                           << " ═══\n  access=" << ch.access
                           << "M ISL=" << ch.isl
                           << "M SGL=" << ch.sgl
                           << "M → expected bottleneck=" << minBw << "M");
            TriggerAllPushes ();
          });
    }
}

void TopologyController::PrintReport ()
{
  NS_LOG_UNCOND ("\n\n╔══════════════════════════════════════════════╗");
  NS_LOG_UNCOND ("║  Session Report — Long-Duration Tracking      ║");
  NS_LOG_UNCOND ("╚══════════════════════════════════════════════╝");

  for (size_t ui = 0; ui < m_users.size (); ui++)
    {
      auto hist = m_users[ui]->GetHistory ();
      if (hist.empty ()) continue;

      float minBw = hist[0].backhaul, maxBw = hist[0].backhaul;
      for (auto &r : hist)
        { minBw = std::min (minBw, r.backhaul); maxBw = std::max (maxBw, r.backhaul); }

      NS_LOG_UNCOND ("\n  User-" << (ui+1) << ": " << hist.size () << " updates");
      NS_LOG_UNCOND ("  Range:  " << minBw << " – " << maxBw << " Mbps");
      NS_LOG_UNCOND ("  Initial: " << hist[0].backhaul << " Mbps @ " << hist[0].link);
      NS_LOG_UNCOND ("  Final:   " << hist.back ().backhaul << " Mbps @ " << hist.back ().link);
      NS_LOG_UNCOND ("  Timeline:");
      for (auto &r : hist)
        NS_LOG_UNCOND ("    t=" << r.time << "s  backhaul=" << r.backhaul << "Mbps  " << r.link);
    }
}


// ════════════════════════════════════════════════════
//  main
// ════════════════════════════════════════════════════

int main (int argc, char *argv[])
{
  double simTime = 160.0;
  CommandLine cmd;
  cmd.AddValue ("simTime", "Session duration (seconds)", simTime);
  cmd.Parse (argc, argv);

  // ── Nodes ──
  NodeContainer nodes;
  nodes.Create (4);
  Names::Add ("User-1", nodes.Get (0));
  Names::Add ("SAT-A",  nodes.Get (1));
  Names::Add ("SAT-B",  nodes.Get (2));
  Names::Add ("Server", nodes.Get (3));

  InternetStackHelper internet;
  internet.Install (nodes);

  // ── Links ──
  PointToPointHelper p2p;
  p2p.SetQueue ("ns3::DropTailQueue");

  p2p.SetDeviceAttribute ("DataRate", StringValue ("150Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("5ms"));
  NetDeviceContainer d01 = p2p.Install (nodes.Get (0), nodes.Get (1));

  p2p.SetDeviceAttribute ("DataRate", StringValue ("600Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("10ms"));
  NetDeviceContainer d12 = p2p.Install (nodes.Get (1), nodes.Get (2));

  p2p.SetDeviceAttribute ("DataRate", StringValue ("350Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue ("10ms"));
  NetDeviceContainer d23 = p2p.Install (nodes.Get (2), nodes.Get (3));

  // ── IP ──
  Ipv4AddressHelper ipv4;
  ipv4.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer if01 = ipv4.Assign (d01);
  ipv4.SetBase ("10.0.1.0", "255.255.255.0");
  Ipv4InterfaceContainer if12 = ipv4.Assign (d12);
  ipv4.SetBase ("10.0.2.0", "255.255.255.0");
  Ipv4InterfaceContainer if23 = ipv4.Assign (d23);

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // ── SBDP Forwarders ──
  Ptr<SbdpForwarder> fwdA = CreateObject<SbdpForwarder> ();
  fwdA->SetInBw (150);  fwdA->SetOutBw (600);
  fwdA->SetNextHop (if12.GetAddress (1), 9999);
  fwdA->AddPushTarget ("User-1", if01.GetAddress (0), 8888); // → User-1
  fwdA->AddPushTarget ("SAT-B",  if12.GetAddress (1), 9999); // → SAT-B
  nodes.Get (1)->AddApplication (fwdA);
  fwdA->SetStartTime (Seconds (0.0));

  Ptr<SbdpForwarder> fwdB = CreateObject<SbdpForwarder> ();
  fwdB->SetInBw (600);  fwdB->SetOutBw (350);
  fwdB->SetNextHop (if23.GetAddress (1), 9999);
  fwdB->AddPushTarget ("SAT-A",  if12.GetAddress (0), 9999); // notify upstream
  nodes.Get (2)->AddApplication (fwdB);
  fwdB->SetStartTime (Seconds (0.0));

  Ptr<SbdpForwarder> fwdSrv = CreateObject<SbdpForwarder> ();
  fwdSrv->SetInBw (350); fwdSrv->SetOutBw (1000);
  fwdSrv->SetNextHop (Ipv4Address ("0.0.0.0"), 0);
  nodes.Get (3)->AddApplication (fwdSrv);
  fwdSrv->SetStartTime (Seconds (0.0));

  // ── User ──
  Ptr<SbdpUserApp> user = CreateObject<SbdpUserApp> ();
  user->SetTarget (if01.GetAddress (1), 9999);
  nodes.Get (0)->AddApplication (user);
  user->SetStartTime (Seconds (0.0));

  // ── Topology Controller ──
  TopologyController topo;
  topo.AddForwarder (fwdA);
  topo.AddForwarder (fwdB);
  topo.AddForwarder (fwdSrv);
  topo.AddUser (user);
  topo.ScheduleChanges (simTime);

  // ── PCAP ──
  p2p.EnablePcapAll ("sbdp-session");

  NS_LOG_UNCOND ("\n╔══════════════════════════════════════════════╗");
  NS_LOG_UNCOND ("║  SBDP Long-Duration Backhaul Tracking        ║");
  NS_LOG_UNCOND ("╠══════════════════════════════════════════════╣");
  NS_LOG_UNCOND ("║  User-1(150M)→SAT-A(600M)→SAT-B(350M)→Server ║");
  NS_LOG_UNCOND ("║  Session: " << simTime << "s, 7 topology changes     ║");
  NS_LOG_UNCOND ("╚══════════════════════════════════════════════╝\n");

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();

  topo.PrintReport ();

  NS_LOG_UNCOND ("\nPCAP: sbdp-session-*.pcap");
  return 0;
}
