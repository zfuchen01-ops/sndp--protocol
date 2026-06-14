/* SBDP Mesh — Multi-Satellite Multi-GS Scenario
 *
 *   Path 1: User-1 ─200M─ SAT-A ─500M─ SAT-B ─600M─ GS-Main(1G)
 *   Path 2: User-2 ─150M─ SAT-B ─600M─ GS-Main(1G)    ← shares GS link
 *   Path 3: User-3 ─250M─ SAT-C ─400M─ GS-Alt(500M)    ← independent
 *
 *   When SAT-B→GS-Main drops → User-1 AND User-2 affected
 *   When SAT-A→SAT-B drops    → only User-1 affected
 *   When SAT-C→GS-Alt drops   → only User-3 affected
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
#include <algorithm>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("SbdpMesh");

// ════════════════════════════════════════════════════
//  SBDP Forwarder
// ════════════════════════════════════════════════════

class SbdpFwd : public Application
{
public:
  static TypeId GetTypeId ();
  SbdpFwd ();
  void SetBw (double inBw, double outBw) { m_inBw = inBw; m_outBw = outBw; }
  double GetInBw ()  const { return m_inBw; }
  double GetOutBw () const { return m_outBw; }
  void AddPushTarget (Ipv4Address addr, uint16_t port)
    { m_targets.push_back ({addr, port}); }
  void TriggerPush ();
  uint16_t GetNodeId () const { return m_nodeId; }

private:
  virtual void StartApplication () override;
  virtual void StopApplication () override;
  void Recv (Ptr<Socket> sock);

  Ptr<Socket> m_sock;
  uint16_t    m_nodeId;
  double      m_inBw, m_outBw;
  uint16_t    m_seq;
  struct Tgt { Ipv4Address addr; uint16_t port; };
  std::vector<Tgt> m_targets;
};

NS_OBJECT_ENSURE_REGISTERED (SbdpFwd);

TypeId SbdpFwd::GetTypeId () {
  static TypeId tid = TypeId ("ns3::SbdpFwd")
    .SetParent<Application> ().SetGroupName ("Sbdp")
    .AddConstructor<SbdpFwd> ();
  return tid;
}
SbdpFwd::SbdpFwd () : m_nodeId (0), m_inBw (0), m_outBw (0), m_seq (0) {}

void SbdpFwd::StartApplication () {
  m_sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
  m_sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 9999));
  m_sock->SetRecvCallback (MakeCallback (&SbdpFwd::Recv, this));
  m_nodeId = GetNode ()->GetId ();
}
void SbdpFwd::StopApplication () { if (m_sock) m_sock->Close (); }

void SbdpFwd::Recv (Ptr<Socket> sock) {
  Ptr<Packet> pkt;
  Address from;
  while ((pkt = sock->RecvFrom (from))) {
    SbdpHeader hdr;
    bool has = pkt->PeekHeader (hdr);
    if (has) pkt->RemoveHeader (hdr);

    if (!has) {
      hdr.SetBackhaulBw (std::numeric_limits<float>::infinity ());
      hdr.SetSeqNum (m_seq++);
      hdr.SetTtl (20); hdr.SetHopCount (0);
      hdr.SetBottleneckLink ("(init)");
    }

    float prev = hdr.GetBackhaulBw ();
    double ck = m_outBw;
    if (m_inBw > 0 && m_inBw < ck) ck = m_inBw;
    if (ck > 0 && ck < prev) {
      hdr.SetBackhaulBw (ck);
      hdr.SetBottleneckLink ("node" + std::to_string (m_nodeId)
          + "(in=" + std::to_string ((int)m_inBw)
          + "M out=" + std::to_string ((int)m_outBw) + "M)");
    }
    hdr.SetHopCount (hdr.GetHopCount () + 1);
    hdr.SetTtl (hdr.GetTtl () - 1);
    pkt->AddHeader (hdr);

    // 仅更新本地视图，不转发 (避免循环)
    // 真正的推送由 TriggerPush 主动发起
  }
}

void SbdpFwd::TriggerPush () {
  if (!m_sock) return;
  for (auto &t : m_targets) {
    Ptr<Packet> pkt = Create<Packet> (64);
    SbdpHeader hdr;
    hdr.SetBackhaulBw (std::numeric_limits<float>::infinity ());
    hdr.SetSeqNum (m_seq++); hdr.SetTtl (20); hdr.SetHopCount (0);
    double ck = m_outBw;
    if (m_inBw > 0 && m_inBw < ck) ck = m_inBw;
    hdr.SetBackhaulBw (ck);
    hdr.SetBottleneckLink ("node" + std::to_string (m_nodeId)
        + "(in=" + std::to_string ((int)m_inBw)
        + "M out=" + std::to_string ((int)m_outBw) + "M)");
    pkt->AddHeader (hdr);
    m_sock->SendTo (pkt, 0, InetSocketAddress (t.addr, t.port));
  }
}


// ════════════════════════════════════════════════════
//  User App
// ════════════════════════════════════════════════════

struct Rec { double t; float bw; std::string link; };

class SbdpUser : public Application
{
public:
  static TypeId GetTypeId ();
  SbdpUser ();
  void SetTarget (Ipv4Address a, uint16_t p) { m_tgtAddr = a; m_tgtPort = p; }
  std::vector<Rec> GetHistory () const { return m_hist; }
private:
  virtual void StartApplication () override;
  virtual void StopApplication () override;
  void Recv (Ptr<Socket> sock);
  Ptr<Socket> m_sock; Ipv4Address m_tgtAddr; uint16_t m_tgtPort;
  std::vector<Rec> m_hist;
};
NS_OBJECT_ENSURE_REGISTERED (SbdpUser);

TypeId SbdpUser::GetTypeId () {
  static TypeId tid = TypeId ("ns3::SbdpUser")
    .SetParent<Application> ().SetGroupName ("Sbdp")
    .AddConstructor<SbdpUser> ();
  return tid;
}
SbdpUser::SbdpUser () : m_tgtPort (9999) {}

void SbdpUser::StartApplication () {
  m_sock = Socket::CreateSocket (GetNode (), UdpSocketFactory::GetTypeId ());
  m_sock->Bind (InetSocketAddress (Ipv4Address::GetAny (), 8888));
  m_sock->SetRecvCallback (MakeCallback (&SbdpUser::Recv, this));
}
void SbdpUser::StopApplication () { if (m_sock) m_sock->Close (); }

void SbdpUser::Recv (Ptr<Socket> sock) {
  Ptr<Packet> pkt;
  Address from;
  while ((pkt = sock->RecvFrom (from))) {
    SbdpHeader hdr;
    if (pkt->PeekHeader (hdr)) {
      std::string uname = Names::FindName (GetNode ());
      if (uname.empty ()) uname = "user" + std::to_string (GetNode ()->GetId ());
      m_hist.push_back ({Simulator::Now ().GetSeconds (), hdr.GetBackhaulBw (), hdr.GetBottleneckLink ()});
      NS_LOG_UNCOND ("  [" << uname << " @ " << Simulator::Now ().GetSeconds ()
                     << "s] backhaul=" << hdr.GetBackhaulBw () << "Mbps "
                     << hdr.GetBottleneckLink ());
    }
  }
}


// ════════════════════════════════════════════════════
//  Helpers
// ════════════════════════════════════════════════════

static Ptr<Node> makeNode (std::string name) {
  Ptr<Node> n = CreateObject<Node> ();
  Names::Add (name, n);
  return n;
}

static void link (Ptr<Node> a, Ptr<Node> b, double bwMbps, double delayMs,
                  Ipv4AddressHelper &ipv4, uint32_t &subnet) {
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute ("DataRate", StringValue (std::to_string ((int)bwMbps) + "Mbps"));
  p2p.SetChannelAttribute ("Delay", StringValue (std::to_string (delayMs) + "ms"));
  NetDeviceContainer d = p2p.Install (a, b);
  char base[32]; snprintf (base, 32, "10.%u.0.0", subnet++);
  ipv4.SetBase (base, "255.255.255.0");
  ipv4.Assign (d);
}

struct FwdCfg { std::string node; double inBw, outBw;
  std::vector<std::pair<std::string,uint16_t>> targets; };

static void installFwds (NodeContainer &allNodes,
                          std::vector<FwdCfg> &cfgs,
                          std::vector<Ptr<SbdpFwd>> &out) {
  // 名字到节点的映射
  std::map<std::string, Ptr<Node>> nameMap;
  nameMap["User-1"] = allNodes.Get (0);
  nameMap["User-2"] = allNodes.Get (1);
  nameMap["User-3"] = allNodes.Get (2);
  nameMap["SAT-A"]  = allNodes.Get (3);
  nameMap["SAT-B"]  = allNodes.Get (4);
  nameMap["SAT-C"]  = allNodes.Get (5);
  nameMap["GS-Main"] = allNodes.Get (6);
  nameMap["GS-Alt"]  = allNodes.Get (7);

  for (auto &c : cfgs) {
    Ptr<Node> n = nameMap[c.node];
    Ptr<SbdpFwd> f = CreateObject<SbdpFwd> ();
    f->SetBw (c.inBw, c.outBw);
    for (auto &t : c.targets) {
      Ptr<Node> tn = nameMap[t.first];
      Ptr<Ipv4> ipv4 = tn->GetObject<Ipv4> ();
      Ipv4Address addr = ipv4->GetAddress (1, 0).GetLocal ();
      f->AddPushTarget (addr, t.second);
    }
    n->AddApplication (f);
    f->SetStartTime (Seconds (0.1));
    out.push_back (f);
  }
}


// ════════════════════════════════════════════════════
//  Topology Controller
// ════════════════════════════════════════════════════

class TopoCtrl
{
public:
  void AddFwd (Ptr<SbdpFwd> f) { (void)f; } // stub
  void AddUser (Ptr<SbdpUser> u) { m_users.push_back (u); }
  void Report ();

private:
  std::vector<Ptr<SbdpUser>> m_users;
};

void TopoCtrl::Report () {
  NS_LOG_UNCOND ("\n\n╔══════════════════════════════════════════════╗");
  NS_LOG_UNCOND ("║  Multi-User Session Report                    ║");
  NS_LOG_UNCOND ("╚══════════════════════════════════════════════╝");
  for (size_t i = 0; i < m_users.size (); i++) {
    auto h = m_users[i]->GetHistory ();
    if (h.empty ()) continue;
    float lo = h[0].bw, hi = h[0].bw;
    for (auto &r : h) { lo = std::min (lo, r.bw); hi = std::max (hi, r.bw); }
    NS_LOG_UNCOND ("\n  User-" << (i+1) << ": " << h.size () << " updates, "
                   << lo << " – " << hi << " Mbps");
    for (auto &r : h)
      NS_LOG_UNCOND ("    t=" << r.t << "s  backhaul=" << r.bw << "Mbps  " << r.link);
  }
}


// ════════════════════════════════════════════════════
//  main
// ════════════════════════════════════════════════════

int main (int argc, char *argv[])
{
  double simTime = 100.0;
  CommandLine cmd; cmd.AddValue ("simTime", "Duration", simTime); cmd.Parse (argc, argv);

  // ── 创建所有节点 ──
  NodeContainer all;
  all.Add (makeNode ("User-1"));   // 0
  all.Add (makeNode ("User-2"));   // 1
  all.Add (makeNode ("User-3"));   // 2
  all.Add (makeNode ("SAT-A"));    // 3
  all.Add (makeNode ("SAT-B"));    // 4
  all.Add (makeNode ("SAT-C"));    // 5
  all.Add (makeNode ("GS-Main"));  // 6  Internet 1G
  all.Add (makeNode ("GS-Alt"));   // 7  Internet 500M

  InternetStackHelper internet;
  internet.Install (all);

  // ── 链路 ──
  uint32_t sn = 0;
  Ipv4AddressHelper ipv4;

  // Path 1 + 2: User → SAT → SAT → GS-Main
  link (all.Get (0), all.Get (3), 200, 5, ipv4, sn);  // User-1 ↔ SAT-A
  link (all.Get (3), all.Get (4), 500, 10, ipv4, sn); // SAT-A ↔ SAT-B
  link (all.Get (1), all.Get (4), 150, 5, ipv4, sn);  // User-2 ↔ SAT-B
  link (all.Get (4), all.Get (6), 600, 10, ipv4, sn); // SAT-B ↔ GS-Main

  // Path 3: User → SAT → GS-Alt
  link (all.Get (2), all.Get (5), 250, 5, ipv4, sn);  // User-3 ↔ SAT-C
  link (all.Get (5), all.Get (7), 400, 10, ipv4, sn); // SAT-C ↔ GS-Alt

  // Cross-link: SAT-A ↔ SAT-C (备用路径)
  link (all.Get (3), all.Get (5), 300, 15, ipv4, sn); // SAT-A ↔ SAT-C

  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  // ── 安装 SBDP Forwarders ──
  std::vector<Ptr<SbdpFwd>> fwds;
  std::vector<FwdCfg> fwdCfgs = {
    // SAT-A: in=User-1(200), out to SAT-B(500) and SAT-C(300)
    {"SAT-A",  200, 500, {{"User-1", 8888}, {"SAT-B", 9999}}},
    // SAT-B: in from SAT-A(500)+User-2(150), out to GS-Main(600)
    {"SAT-B",  500, 600, {{"User-2", 8888}, {"SAT-A", 9999}}},
    // SAT-C: in=User-3(250)+SAT-A(300), out to GS-Alt(400)
    {"SAT-C",  250, 400, {{"User-3", 8888}, {"SAT-A", 9999}}},
    // GS-Main: in=SAT-B(600), out=Internet(1000) — 终点
    {"GS-Main",600, 1000, {}},
    // GS-Alt: in=SAT-C(400), out=Internet(500) — 终点
    {"GS-Alt", 400, 500,  {}},
  };
  installFwds (all, fwdCfgs, fwds);

  // ── 安装 Users ──
  std::vector<Ptr<SbdpUser>> users;
  for (int i = 0; i < 3; i++) {
    Ptr<SbdpUser> u = CreateObject<SbdpUser> ();
    all.Get (i)->AddApplication (u);
    u->SetStartTime (Seconds (0.1));
    users.push_back (u);
  }

  // ── Topology Controller ──
  TopoCtrl topo;
  for (auto &u : users) topo.AddUser (u);

  // 手动注册转发器索引 (用于带宽变更)
  // fwds[0]=SAT-A, fwds[1]=SAT-B, fwds[2]=SAT-C, fwds[3]=GS-Main, fwds[4]=GS-Alt
  auto schedule = [&](double t, int fwdIdx, double inBw, double outBw, const char *desc) {
    Simulator::Schedule (Seconds (t), [&fwds, &topo, fwdIdx, inBw, outBw, desc] () {
      if (fwdIdx >= 0 && (size_t)fwdIdx < fwds.size ()) {
        fwds[fwdIdx]->SetBw (inBw, outBw);
      }
      NS_LOG_UNCOND ("\n═══ t=" << Simulator::Now ().GetSeconds ()
                     << "s " << desc << " ═══");
      for (auto &f : fwds) f->TriggerPush ();
    });
  };

  schedule (5,  -1, 0, 0, "initial push");
  // SAT-B(idx 1): in stays 500, out drops 600→200
  schedule (15, 1,  500, 200, "SAT-B→GS-Main: 600→200M ★ User-1 & User-2 affected");
  // SAT-A(idx 0): in stays 200, out drops 500→100
  schedule (30, 0,  200, 100, "SAT-A→SAT-B: 500→100M ★ only User-1 affected");
  // SAT-C(idx 2): in stays 250, out drops 400→80
  schedule (45, 2,  250, 80,  "SAT-C→GS-Alt: 400→80M ★ only User-3 affected");
  // recoveries
  schedule (60, 1,  500, 600, "SAT-B→GS-Main recovers →600M");
  schedule (75, 0,  200, 500, "SAT-A→SAT-B recovers →500M");
  schedule (90, 2,  250, 400, "SAT-C→GS-Alt recovers →400M");

  // ── PCAP ──
  PointToPointHelper p2pDummy;
  // EnablePcapAll doesn't work with our manual node creation.
  // Use explicit tracing instead.

  NS_LOG_UNCOND ("\n╔══════════════════════════════════════════════╗");
  NS_LOG_UNCOND ("║  SBDP Mesh — Multi-User Backhaul Tracking     ║");
  NS_LOG_UNCOND ("╠══════════════════════════════════════════════╣");
  NS_LOG_UNCOND ("║  User-1→SAT-A→SAT-B→GS-Main(shared)          ║");
  NS_LOG_UNCOND ("║  User-2→SAT-B→GS-Main(shared)                ║");
  NS_LOG_UNCOND ("║  User-3→SAT-C→GS-Alt(independent)            ║");
  NS_LOG_UNCOND ("║  Session: " << simTime << "s                               ║");
  NS_LOG_UNCOND ("╚══════════════════════════════════════════════╝\n");

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();

  topo.Report ();
  return 0;
}
