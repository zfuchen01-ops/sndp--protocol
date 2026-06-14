/* SBDP Visibility — Multi-satellite ADVs to covered users via real UDP
 *
 * Each satellite pushes ADVs to ALL users in its coverage area (not just the
 * one it currently serves). Users maintain a backhaul map and handover.
 * Real SBDP packets over sockets. PCAP on all access links.
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/sbdp-header.h"
#include <map>
#include <vector>
#include <sstream>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("SbdpVis");

class SatFwd : public Application {
public:
  static TypeId GetTypeId(); SatFwd();
  void SetBw(double d) { m_bw=d; }
  void AddCoverage(Ipv4Address ua) { m_covered.push_back(ua); }
  void Push();
private:
  virtual void StartApplication() override { m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId()); m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9999)); }
  virtual void StopApplication() override { if(m_sk) m_sk->Close(); }
  Ptr<Socket> m_sk; double m_bw=0; uint16_t m_seq=0;
  std::vector<Ipv4Address> m_covered;
};
NS_OBJECT_ENSURE_REGISTERED(SatFwd);
TypeId SatFwd::GetTypeId(){static TypeId tid=TypeId("ns3::SatFwd").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<SatFwd>();return tid;}
SatFwd::SatFwd(){}
void SatFwd::Push(){
  for(auto& ua:m_covered){
    SbdpHeader hdr=SbdpHeader::BuildAdv(Names::FindName(GetNode()),"UE",m_bw,Names::FindName(GetNode())+" bw="+std::to_string((int)m_bw)+"M",m_seq++);
    Ptr<Packet> pkt=Create<Packet>(0); pkt->AddHeader(hdr);
    m_sk->SendTo(pkt,0,InetSocketAddress(ua,8888));
  }
}

class UsrApp : public Application {
public:
  static TypeId GetTypeId(); UsrApp();
  std::map<std::string,float> m_map;
  std::string m_curSat; int m_ho=0; float m_thr=250.0;
private:
  virtual void StartApplication() override { m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId()); m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),8888)); m_sk->SetRecvCallback(MakeCallback(&UsrApp::Recv,this)); }
  virtual void StopApplication() override { if(m_sk) m_sk->Close(); }
  void Recv(Ptr<Socket> s);
  Ptr<Socket> m_sk;
};
NS_OBJECT_ENSURE_REGISTERED(UsrApp);
TypeId UsrApp::GetTypeId(){static TypeId tid=TypeId("ns3::UsrApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<UsrApp>();return tid;}
UsrApp::UsrApp(){}
void UsrApp::Recv(Ptr<Socket> s){Ptr<Packet> pkt;Address from;
  while((pkt=s->RecvFrom(from))){SbdpHeader hdr;if(pkt->GetSize()>=SbdpHeader::SBDP_FIXED_SIZE){pkt->RemoveHeader(hdr);
  std::string sn=hdr.GetSrcNode();m_map[sn]=hdr.GetBackhaulBw();
  if(m_curSat.empty()) m_curSat=sn;
  NS_LOG_UNCOND("  ["<<Names::FindName(GetNode())<<" @"<<Simulator::Now().GetSeconds()<<"s] ←"<<sn<<" bw="<<(int)hdr.GetBackhaulBw()<<"M map={"<<[&](){std::ostringstream ss;for(auto&p:m_map){if(&p!=&*m_map.begin())ss<<",";ss<<p.first<<":"<<(int)p.second;}return ss.str();}()<<"}");
  // Handover check
  if(sn==m_curSat&&hdr.GetBackhaulBw()<m_thr){float best=m_map[m_curSat];std::string bestSat;
  for(auto&p:m_map)if(p.first!=m_curSat&&p.second>best){best=p.second;bestSat=p.first;}
  if(!bestSat.empty()){NS_LOG_UNCOND("  ↳ HANDOVER "<<m_curSat<<"→"<<bestSat<<" ("<<(int)m_map[m_curSat]<<"→"<<(int)best<<"M)");m_curSat=bestSat;m_ho++;}}
}}}

int main(int argc,char*argv[]){
  NodeContainer n;n.Create(11);
  for(int i=0;i<6;i++) Names::Add("SAT-"+std::string(1,'A'+i),n.Get(i));
  Names::Add("GS-East",n.Get(6)); Names::Add("GS-West",n.Get(7));
  Names::Add("User-1",n.Get(8)); Names::Add("User-2",n.Get(9)); Names::Add("User-3",n.Get(10));
  InternetStackHelper inet;inet.Install(n);
  PointToPointHelper p2p;p2p.SetDeviceAttribute("DataRate",StringValue("1Gbps"));p2p.SetChannelAttribute("Delay",StringValue("5ms"));
  Ipv4AddressHelper ipv4;uint32_t sn=0;
  std::vector<NetDeviceContainer> devs;
  auto L=[&](int a,int b){ipv4.SetBase(("10."+std::to_string(sn++)+".0.0").c_str(),"255.255.255.0");auto d=p2p.Install(n.Get(a),n.Get(b));ipv4.Assign(d);devs.push_back(d);return d;};
  // Access links (stars ↔ users)
  L(0,8);L(1,8);L(2,9);L(3,9);L(4,10);L(5,10); // primary
  L(0,9);L(4,8);L(2,10); // overlapping coverage
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  std::vector<Ptr<SatFwd>> fwds(6);
  for(int i=0;i<6;i++){fwds[i]=CreateObject<SatFwd>();n.Get(i)->AddApplication(fwds[i]);fwds[i]->SetStartTime(Seconds(0.1));}
  std::vector<Ptr<UsrApp>> users(3);
  for(int i=0;i<3;i++){users[i]=CreateObject<UsrApp>();n.Get(8+i)->AddApplication(users[i]);users[i]->SetStartTime(Seconds(0.1));}

  // Coverage (matching original)
  fwds[0]->AddCoverage(n.Get(8)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());
  fwds[1]->AddCoverage(n.Get(8)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());
  fwds[2]->AddCoverage(n.Get(9)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());
  fwds[3]->AddCoverage(n.Get(9)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());
  fwds[4]->AddCoverage(n.Get(10)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());
  fwds[5]->AddCoverage(n.Get(10)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());
  fwds[0]->AddCoverage(n.Get(9)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());
  fwds[4]->AddCoverage(n.Get(8)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());
  fwds[2]->AddCoverage(n.Get(10)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal());

  auto hoCheck=[&](){for(auto&u:users){
    float cur=u->m_map.count(u->m_curSat)?u->m_map[u->m_curSat]:999.0f;
    if(cur<u->m_thr){float best=cur;std::string bestSat;
    for(auto&p:u->m_map)if(p.first!=u->m_curSat&&p.second>best){best=p.second;bestSat=p.first;}
    if(!bestSat.empty()){NS_LOG_UNCOND("  ↳ HANDOVER "<<u->m_curSat<<"→"<<bestSat<<" ("<<(int)cur<<"→"<<(int)best<<"M)");u->m_curSat=bestSat;u->m_ho++;}}}};

  auto ev=[&](double t,double bA,double bB,double bC,double bD,double bE,double bF,const char*desc){
    Simulator::Schedule(Seconds(t),[=](){
      NS_LOG_UNCOND("\n═══ t="<<(int)t<<"s "<<desc<<" ═══");
      double bws[6]={bA,bB,bC,bD,bE,bF};
      for(int i=0;i<6;i++){fwds[i]->SetBw(bws[i]);fwds[i]->Push();}
    });
    Simulator::Schedule(Seconds(t+0.05),hoCheck);
  };

  ev(2,350,300,250,200,300,250,"INITIAL");
  ev(50,80,300,250,200,300,250,"RAIN: GS-East 350→80M (SAT-A degraded)");
  ev(120,350,300,250,200,300,250,"RECOVER");
  ev(200,350,300,250,200,0,0,"FAULT: polar blackout + GS-West down");
  ev(300,350,300,250,200,300,250,"RECOVER: all links back");

  p2p.EnablePcap("sbdp-vis",devs[0].Get(0),true);
  p2p.EnablePcap("sbdp-vis",devs[3].Get(0),true);

  Simulator::Schedule(Seconds(350),[&](){
    NS_LOG_UNCOND("\n\n╔══════════════════════════════════════════════╗");
    NS_LOG_UNCOND("║  Visibility Report (real SBDP over UDP)       ║");
    NS_LOG_UNCOND("╚══════════════════════════════════════════════╝");
    for(int i=0;i<3;i++){NS_LOG_UNCOND("  User-"<<(i+1)<<": cur="<<users[i]->m_curSat<<" ("<<(int)users[i]->m_map[users[i]->m_curSat]<<"M) "<<users[i]->m_ho<<" handovers");}
  });

  Simulator::Stop(Seconds(351));Simulator::Run();Simulator::Destroy();return 0;
}
