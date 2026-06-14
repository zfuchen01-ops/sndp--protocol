/* N3 gNB Coordination — Real network gNB-to-gNB messaging over ISL
 *
 * Proves: gNBs coordinate handover by exchanging messages over the network.
 *         NOT an SBDP protocol test — this is application-layer coordination.
 *
 * Topology: 4 satellites (SAT-A/C/D/E) in a ring, SAT-B as source.
 *           SAT-B has 12 UEs, becomes overloaded, coordinates handover.
 *
 * Message format (simple struct, not SBDP):  type | ueCount | reservedBw | ...
 * Type 0x01 = MIGRATE request, 0x02 = CONFIRM response.
 *
 * Usage: ./ns3 run sbdp-n3-handover
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"

#include <map>
#include <vector>
#include <algorithm>
#include <cmath>
#include <sstream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("N3Handover");

// ═══════════════════════════════════════════
//  N3 message (simple struct, not SBDP)
// ═══════════════════════════════════════════

struct N3Msg {
  uint8_t  type;        // 1=MIGRATE, 2=CONFIRM
  uint8_t  ueCount;     // number of UEs in this message
  uint16_t seqNum;      // sequence for matching request/response
  float    reservedBw;  // Mbps per UE
  char     payload[64]; // UE list or reason string

  N3Msg () : type(0), ueCount(0), seqNum(0), reservedBw(0) { memset(payload,0,64); }

  static N3Msg Migrate (uint16_t seq, uint8_t nUe, float bw, const std::string &ueList) {
    N3Msg m; m.type=1; m.seqNum=seq; m.ueCount=nUe; m.reservedBw=bw;
    strncpy(m.payload, ueList.c_str(), 63);
    return m;
  }
  static N3Msg Confirm (uint16_t seq, uint8_t nUe, float bw, bool accept, const std::string &alt) {
    N3Msg m; m.type=2; m.seqNum=seq; m.ueCount=nUe; m.reservedBw=bw;
    std::string p = accept ? "ACCEPT" : ("REJECT:"+alt);
    strncpy(m.payload, p.c_str(), 63);
    return m;
  }
  std::string Str() const {
    std::ostringstream ss;
    ss << (type==1?"MIGRATE":"CONFIRM") << " seq=" << seqNum
       << " ues=" << (int)ueCount << " bw=" << (int)reservedBw << "M " << payload;
    return ss.str();
  }
} __attribute__((packed));


// ═══════════════════════════════════════════
//  Satellite state
// ═══════════════════════════════════════════

struct SatInfo {
  std::string name;
  double backhaulBw, loadPct, sinrDb;
  int maxUes, currentUes;

  double Score() const {
    double w = std::min(1.0, std::max(0.1, sinrDb/30.0));
    return backhaulBw * (1.0 - loadPct) * w;
  }
  bool CanAccept(int n=1) const { return (currentUes+n)<=maxUes && loadPct<0.85; }
  void Accept(int n=1) { currentUes+=n; loadPct=std::min(1.0,(double)currentUes/maxUes); }
};


// ═══════════════════════════════════════════
//  GnbApp — runs on each satellite
// ═══════════════════════════════════════════

class GnbApp : public Application
{
public:
  static TypeId GetTypeId();
  GnbApp();
  void Setup (const std::string &name, const SatInfo &info,
              std::map<std::string, Ipv4Address> &peers);
  void TriggerCoordination (int nUe, double bwPerUe);

  // Neighbor states (set before simulation)
  std::map<std::string, SatInfo> m_neighbors;

  // Stats
  int  m_acceptedUes = 0;
  int  m_sentMsgs = 0;
  int  m_recvMsgs = 0;
  std::vector<std::string> m_log;

private:
  virtual void StartApplication() override;
  virtual void StopApplication() override;
  void RecvPacket (Ptr<Socket> sock);
  void SendTo (const std::string &target, const N3Msg &msg);
  void ProcessMigrate (const N3Msg &msg, const Ipv4Address &from);
  void RunCoordinator (int nUe, double bwPerUe);

  std::string m_name;
  SatInfo m_info;
  Ptr<Socket> m_sock;
  std::map<std::string, Ipv4Address> m_peers;
  uint16_t m_seq = 100;

  // Pending coordination
  bool m_coordPending = false;
  int  m_coordUeCount = 0;
  double m_coordBw = 0;
};

NS_OBJECT_ENSURE_REGISTERED (GnbApp);

TypeId GnbApp::GetTypeId() {
  static TypeId tid = TypeId("ns3::GnbApp").SetParent<Application>()
    .SetGroupName("N3").AddConstructor<GnbApp>();
  return tid;
}
GnbApp::GnbApp() {}

void GnbApp::Setup (const std::string &name, const SatInfo &info,
                    std::map<std::string, Ipv4Address> &peers) {
  m_name = name;
  m_info = info;
  m_peers = peers;
}

void GnbApp::StartApplication() {
  m_sock = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
  m_sock->Bind(InetSocketAddress(Ipv4Address::GetAny(), 9999));
  m_sock->SetRecvCallback(MakeCallback(&GnbApp::RecvPacket, this));
}

void GnbApp::StopApplication() { if (m_sock) m_sock->Close(); }

void GnbApp::SendTo (const std::string &target, const N3Msg &msg) {
  auto it = m_peers.find(target);
  if (it == m_peers.end()) return;
  Ptr<Packet> pkt = Create<Packet>((uint8_t*)&msg, sizeof(N3Msg));
  m_sock->SendTo(pkt, 0, InetSocketAddress(it->second, 9999));
  m_sentMsgs++;

  std::ostringstream ss;
  ss << "  [" << m_name << "] →" << target << " " << msg.Str()
     << " [pkt=" << pkt->GetSize() << "B]";
  NS_LOG_UNCOND(ss.str());
  m_log.push_back(ss.str());
}

void GnbApp::RecvPacket (Ptr<Socket> sock) {
  Ptr<Packet> pkt; Address from;
  while ((pkt = sock->RecvFrom(from))) {
    N3Msg msg;
    if (pkt->GetSize() >= (uint32_t)sizeof(N3Msg)) {
      pkt->CopyData((uint8_t*)&msg, sizeof(N3Msg));
      m_recvMsgs++;

      InetSocketAddress addr = InetSocketAddress::ConvertFrom(from);
      std::string sender = "?";
      for (auto &p : m_peers) if (p.second == addr.GetIpv4()) { sender = p.first; break; }

      std::ostringstream ss;
      ss << "  [" << m_name << "] ←" << sender << " " << msg.Str();
      NS_LOG_UNCOND(ss.str());
      m_log.push_back(ss.str());

      if (msg.type == 1) ProcessMigrate(msg, addr.GetIpv4());
    }
  }
}

void GnbApp::ProcessMigrate (const N3Msg &msg, const Ipv4Address &from) {
  // Check if we can accept
  int canTake = std::min((int)msg.ueCount,
      std::max(0, m_info.maxUes - m_info.currentUes));
  if (!m_info.CanAccept(1)) canTake = 0;
  canTake = std::min(canTake, (int)msg.ueCount);

  std::string alt = canTake < msg.ueCount ? "SAT-A" : "";
  N3Msg resp = N3Msg::Confirm(msg.seqNum, canTake, msg.reservedBw,
                               canTake > 0, alt);

  // Find sender to reply
  std::string sender = "?";
  for (auto &p : m_peers) if (p.second == from) { sender = p.first; break; }

  m_info.Accept(canTake);
  m_acceptedUes += canTake;
  SendTo(sender, resp);
}

void GnbApp::TriggerCoordination (int nUe, double bwPerUe) {
  m_coordPending = true;
  m_coordUeCount = nUe;
  m_coordBw = bwPerUe;
  Simulator::Schedule(Seconds(0.0), &GnbApp::RunCoordinator, this, nUe, bwPerUe);
}

void GnbApp::RunCoordinator (int nUe, double bwPerUe) {
  NS_LOG_UNCOND("\n  ═══ " << m_name << " N3 Coordinator: " << nUe
                << " UEs need handover ═══");

  // Rank remote satellites by score
  std::vector<std::pair<std::string, double>> ranked;
  for (auto &nb : m_neighbors) {
    if (nb.first == m_name) continue;
    if (!nb.second.CanAccept(1)) continue;
    ranked.push_back({nb.first, nb.second.Score()});
  }
  std::sort(ranked.begin(), ranked.end(),
    [](auto &a, auto &b) { return a.second > b.second; });

  NS_LOG_UNCOND("  Candidate ranking:");
  for (auto &r : ranked)
    NS_LOG_UNCOND("    " << r.first << " score=" << (int)r.second);

  // Distribute UEs proportionally by score
  double totalScore = 0;
  for (auto &r : ranked) totalScore += r.second;

  std::vector<std::string> ueNames;
  for (int i=0; i<nUe; i++) { std::ostringstream ss; ss<<"UE-"<<(i+1); ueNames.push_back(ss.str()); }

  int assigned = 0;
  for (auto &r : ranked) {
    if (assigned >= nUe) break;
    int alloc = totalScore > 0 ? std::max(1, (int)std::round(nUe * r.second / totalScore)) : 1;
    alloc = std::min({alloc, nUe - assigned,
                      m_neighbors[r.first].maxUes - m_neighbors[r.first].currentUes});
    if (alloc <= 0) continue;

    std::string ueList;
    for (int i=0; i<alloc && (assigned+i)<(int)ueNames.size(); i++)
      ueList += (i?",":"") + ueNames[assigned+i];

    // ★ Send MIGRATE over the network
    N3Msg migrate = N3Msg::Migrate(m_seq++, alloc, bwPerUe, ueList);
    SendTo(r.first, migrate);
    assigned += alloc;
  }

  NS_LOG_UNCOND("  " << assigned << "/" << nUe << " UEs assigned via N3 coordination");
}


// ═══════════════════════════════════════════
//  main
// ═══════════════════════════════════════════

int main (int argc, char *argv[])
{
  int nUe = 12;
  CommandLine cmd; cmd.AddValue("ueCount", "UEs to handover", nUe); cmd.Parse(argc, argv);

  NS_LOG_UNCOND("\n╔══════════════════════════════════════════════════╗");
  NS_LOG_UNCOND("║  N3 gNB Coordination — Real Network Messaging    ║");
  NS_LOG_UNCOND("║  SAT-B overloaded → coordinates with SAT-A/C/D/E ║");
  NS_LOG_UNCOND("║  Messages sent over UDP/ISL, captured in PCAP    ║");
  NS_LOG_UNCOND("╚══════════════════════════════════════════════════╝\n");

  // ── Create nodes: 5 satellites ──
  NodeContainer nodes;
  nodes.Create(5);
  Names::Add("SAT-A", nodes.Get(0));
  Names::Add("SAT-B", nodes.Get(1));  // source (overloaded)
  Names::Add("SAT-C", nodes.Get(2));
  Names::Add("SAT-D", nodes.Get(3));
  Names::Add("SAT-E", nodes.Get(4));

  InternetStackHelper internet;
  internet.Install(nodes);

  // ── ISL links: SAT-B connected to all others (star, for simplicity) ──
  PointToPointHelper p2p;
  p2p.SetDeviceAttribute("DataRate", StringValue("1Gbps"));
  p2p.SetChannelAttribute("Delay", StringValue("5ms"));  // ISL latency

  Ipv4AddressHelper ipv4;
  std::map<std::string, Ipv4Address> addrs;

  // Each satellite gets its own peer map (SAT-B has different IP per link)
  std::map<std::string, std::map<std::string, Ipv4Address>> peerAddrs;

  auto link = [&](int a, int b, const char *base) {
    ipv4.SetBase(base, "255.255.255.0");
    auto dev = p2p.Install(nodes.Get(a), nodes.Get(b));
    auto ifaces = ipv4.Assign(dev);
    std::string na = Names::FindName(nodes.Get(a));
    std::string nb = Names::FindName(nodes.Get(b));
    peerAddrs[na][nb] = ifaces.GetAddress(1);  // na reaches nb at nb's IP
    peerAddrs[nb][na] = ifaces.GetAddress(0);  // nb reaches na at na's IP
    return dev;
  };

  auto dBA = link(1, 0, "10.0.1.0");  // SAT-B ↔ SAT-A
  auto dBC = link(1, 2, "10.0.2.0");  // SAT-B ↔ SAT-C
  auto dBD = link(1, 3, "10.0.3.0");  // SAT-B ↔ SAT-D
  auto dBE = link(1, 4, "10.0.4.0");  // SAT-B ↔ SAT-E

  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  // ── Satellite states ──
  std::map<std::string, SatInfo> allInfo = {
    {"SAT-A", {"SAT-A",500,0.60,15.0,8,3}},
    {"SAT-B", {"SAT-B",400,0.90,10.0,8,12}},  // overloaded!
    {"SAT-C", {"SAT-C",600,0.40,12.0,8,2}},
    {"SAT-D", {"SAT-D",400,0.85,10.0,8,7}},    // nearly full
    {"SAT-E", {"SAT-E",350,0.30,18.0,6,1}},
  };

  NS_LOG_UNCOND("── Satellite states ──");
  for (auto &p : allInfo)
    NS_LOG_UNCOND("  " << p.second.name << " bw=" << (int)p.second.backhaulBw
                  << "M load=" << (int)(p.second.loadPct*100)
                  << "% sinr=" << (int)p.second.sinrDb
                  << "dB ues=" << p.second.currentUes << "/" << p.second.maxUes);

  // ── Install GnbApp on each satellite ──
  std::map<std::string, Ptr<GnbApp>> apps;
  for (int i = 0; i < 5; i++) {
    auto app = CreateObject<GnbApp>();
    std::string name = Names::FindName(nodes.Get(i));
    app->Setup(name, allInfo[name], peerAddrs[name]);
    // Give each gNB knowledge of all other satellites' states
    // (in real system, this comes from SBDP ADV + N3 state sharing)
    for (auto &p : allInfo) app->m_neighbors[p.first] = p.second;
    nodes.Get(i)->AddApplication(app);
    app->SetStartTime(Seconds(0.1));
    apps[name] = app;
  }

  // ── Background data: SAT-B sends heavy UDP to all peers (simulates overload) ──
  std::vector<Ptr<PacketSink>> n3Sinks;
  for (int i : {0,2,3,4}) { // SAT-A,C,D,E
    PacketSinkHelper sink("ns3::UdpSocketFactory",
      InetSocketAddress(Ipv4Address::GetAny(), 7000));
    auto app = sink.Install(nodes.Get(i)); app.Start(Seconds(0.1)); app.Stop(Seconds(3));
    n3Sinks.push_back(DynamicCast<PacketSink>(app.Get(0)));
  }
  for (auto &tgt : std::vector<std::string>{"SAT-A","SAT-C","SAT-D","SAT-E"}) {
    OnOffHelper onoff("ns3::UdpSocketFactory",
      InetSocketAddress(peerAddrs["SAT-B"][tgt], 7000));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate("200Mbps")));
    onoff.SetAttribute("PacketSize", UintegerValue(1472));
    onoff.SetConstantRate(DataRate("200Mbps"));
    auto client = onoff.Install(nodes.Get(1)); // SAT-B
    client.Start(Seconds(0.2)); client.Stop(Seconds(2.5));
  }

  // ── PCAP ──
  p2p.EnablePcap("n3-satb-sata", dBA.Get(0), true);
  p2p.EnablePcap("n3-satb-satc", dBC.Get(0), true);
  p2p.EnablePcap("n3-satb-satd", dBD.Get(0), true);
  p2p.EnablePcap("n3-satb-sate", dBE.Get(0), true);

  // ── Trigger: SAT-B overloaded, coordinate handover ──
  Simulator::Schedule(Seconds(1.0), [&]() {
    NS_LOG_UNCOND("\n═══ t=1.0s SAT-B overloaded (90%), " << nUe << " UEs need handover ═══");
    apps["SAT-B"]->TriggerCoordination(nUe, 50.0);  // 50 Mbps per UE
  });

  // ── Report ──
  Simulator::Schedule(Seconds(2.0), [&]() {
    NS_LOG_UNCOND("\n╔══════════════════════════════════════════════════╗");
    NS_LOG_UNCOND("║  N3 Coordination Report                           ║");
    NS_LOG_UNCOND("╠══════════════════════════════════════════════════╣");
    int totalSent = 0, totalRecv = 0, totalAccepted = 0;
    for (auto &p : apps) {
      auto a = p.second;
      NS_LOG_UNCOND("  " << p.first << ": sent=" << a->m_sentMsgs
                    << " recv=" << a->m_recvMsgs
                    << " accepted=" << a->m_acceptedUes << " UEs");
      totalSent += a->m_sentMsgs;
      totalRecv += a->m_recvMsgs;
      totalAccepted += a->m_acceptedUes;
    }
    NS_LOG_UNCOND("  ─────────────────────────────────");
    double totalDataMb = 0;
    for (auto &s : n3Sinks) totalDataMb += s->GetTotalRx() / 1e6;
    NS_LOG_UNCOND("  Total: " << totalSent << " N3 msgs sent, "
                  << totalRecv << " received, "
                  << totalAccepted << " UEs accepted");
    NS_LOG_UNCOND("  Background data: " << totalDataMb << " MB transferred");
    NS_LOG_UNCOND("╚══════════════════════════════════════════════════╝");
    NS_LOG_UNCOND("\n  PCAP files: n3-satb-*.pcap");
    NS_LOG_UNCOND("  tcpdump -r n3-satb-satc-0-0.pcap -X  # see N3 messages on wire\n");
  });

  Simulator::Stop(Seconds(3));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
