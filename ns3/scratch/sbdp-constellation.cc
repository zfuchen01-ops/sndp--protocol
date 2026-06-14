/* SBDP Constellation — Ring topology + random fluctuation + real UDP ADVs */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/sbdp-header.h"
#include <map>
#include <vector>
#include <set>
#include <algorithm>
#include <random>
using namespace ns3;NS_LOG_COMPONENT_DEFINE("SbdpConst");

class SatFwd:public Application{public:static TypeId GetTypeId();SatFwd();void SetBw(double b){m_bw=b;}void AddUser(Ipv4Address ua){m_users.push_back(ua);}void Push();private:virtual void StartApplication()override{m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9999));}virtual void StopApplication()override{if(m_sk)m_sk->Close();}Ptr<Socket> m_sk;double m_bw=0;uint16_t m_seq=0;std::vector<Ipv4Address> m_users;};
NS_OBJECT_ENSURE_REGISTERED(SatFwd);TypeId SatFwd::GetTypeId(){static TypeId tid=TypeId("ns3::SatFwd").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<SatFwd>();return tid;}SatFwd::SatFwd(){}
void SatFwd::Push(){for(auto&ua:m_users){SbdpHeader hdr=SbdpHeader::BuildAdv(Names::FindName(GetNode()),"UE",m_bw,Names::FindName(GetNode()),m_seq++);Ptr<Packet> pkt=Create<Packet>(0);pkt->AddHeader(hdr);m_sk->SendTo(pkt,0,InetSocketAddress(ua,8888));}}

class UsrApp:public Application{public:static TypeId GetTypeId();UsrApp();struct E{double t;float bw;};std::vector<E> m_h;private:virtual void StartApplication()override{m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),8888));m_sk->SetRecvCallback(MakeCallback(&UsrApp::Recv,this));}virtual void StopApplication()override{if(m_sk)m_sk->Close();}void Recv(Ptr<Socket> s){Ptr<Packet> pkt;Address from;while((pkt=s->RecvFrom(from))){SbdpHeader hdr;if(pkt->GetSize()>=SbdpHeader::SBDP_FIXED_SIZE){pkt->RemoveHeader(hdr);m_h.push_back({Simulator::Now().GetSeconds(),hdr.GetBackhaulBw()});NS_LOG_UNCOND("  ["<<Names::FindName(GetNode())<<" @"<<Simulator::Now().GetSeconds()<<"s] backhaul="<<(int)hdr.GetBackhaulBw()<<"Mbps");}}}Ptr<Socket> m_sk;};
NS_OBJECT_ENSURE_REGISTERED(UsrApp);TypeId UsrApp::GetTypeId(){static TypeId tid=TypeId("ns3::UsrApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<UsrApp>();return tid;}UsrApp::UsrApp(){}

struct Link{std::string a,b;double base,cur;};
int main(int argc,char*argv[]){double simTime=210;int interval=15;CommandLine cmd;cmd.AddValue("simTime","s",simTime);cmd.AddValue("interval","s",interval);cmd.Parse(argc,argv);
  std::mt19937 rng(42);std::uniform_real_distribution<> d(-0.45,0.45);
  NodeContainer all;all.Create(9);
  Names::Add("User-A",all.Get(0));Names::Add("User-B",all.Get(1));Names::Add("User-C",all.Get(2));
  Names::Add("SAT-A",all.Get(3));Names::Add("SAT-B",all.Get(4));Names::Add("SAT-C",all.Get(5));Names::Add("SAT-D",all.Get(6));
  Names::Add("GS-East",all.Get(7));Names::Add("GS-West",all.Get(8));
  InternetStackHelper inet;inet.Install(all);
  PointToPointHelper p2p;p2p.SetDeviceAttribute("DataRate",StringValue("1Gbps"));p2p.SetChannelAttribute("Delay",StringValue("5ms"));
  Ipv4AddressHelper ipv4;ipv4.SetBase("10.0.0.0","255.255.255.0");
  ipv4.SetBase("10.0.0.0","255.255.255.0");
  ipv4.Assign(p2p.Install(all.Get(0),all.Get(3))); ipv4.NewNetwork();
  ipv4.Assign(p2p.Install(all.Get(1),all.Get(4))); ipv4.NewNetwork();
  ipv4.Assign(p2p.Install(all.Get(2),all.Get(5))); ipv4.NewNetwork();
  ipv4.Assign(p2p.Install(all.Get(3),all.Get(4))); ipv4.NewNetwork();
  ipv4.Assign(p2p.Install(all.Get(4),all.Get(5))); ipv4.NewNetwork();
  ipv4.Assign(p2p.Install(all.Get(5),all.Get(6))); ipv4.NewNetwork();
  ipv4.Assign(p2p.Install(all.Get(6),all.Get(3))); ipv4.NewNetwork();
  ipv4.Assign(p2p.Install(all.Get(3),all.Get(7))); ipv4.NewNetwork();
  ipv4.Assign(p2p.Install(all.Get(5),all.Get(8)));

  std::vector<Link> links={{"SAT-A","SAT-B",250,250},{"SAT-B","SAT-C",300,300},{"SAT-C","SAT-D",200,200},{"SAT-D","SAT-A",280,280},{"SAT-A","GS-East",500,500},{"SAT-C","GS-West",350,350}};
  auto mkFwd=[&](int i){auto f=CreateObject<SatFwd>();all.Get(i)->AddApplication(f);f->SetStartTime(Seconds(0.1));return f;};
  auto fA=mkFwd(3),fB=mkFwd(4),fC=mkFwd(5),fD=mkFwd(6);
  auto mkUsr=[&](int i){auto u=CreateObject<UsrApp>();all.Get(i)->AddApplication(u);u->SetStartTime(Seconds(0.1));return u;};
  auto uA=mkUsr(0),uB=mkUsr(1),uC=mkUsr(2);
  auto lu=[&](int i)->Ipv4Address{return all.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();};
  fA->AddUser(lu(0));fB->AddUser(lu(1));fC->AddUser(lu(2));
  std::vector<Ptr<UsrApp>> users={uA,uB,uC};

  auto computeBw=[&](const std::string& sat){
    std::map<std::string,std::map<std::string,double>> g;
    for(auto&l:links){g[l.a][l.b]=l.cur;g[l.b][l.a]=l.cur;}
    std::vector<std::string> gsList={"GS-East","GS-West"};
    std::vector<std::string> sats={"SAT-A","SAT-B","SAT-C","SAT-D"};
    double best=0;
    for(auto& gsName : gsList){
      std::map<std::string,double> bw;for(auto&s:sats)bw[s]=0;for(auto&g2:gsList)bw[g2]=0;bw[sat]=1e9;
      std::set<std::string> vis;
      while(true){
        std::string bestN;double bv=-1;
        for(auto&p:bw){if(vis.count(p.first))continue;if(p.second>bv){bv=p.second;bestN=p.first;}}
        if(bestN.empty()||bestN==gsName)break;vis.insert(bestN);
        for(auto&nb:g[bestN]){double pb=std::min(bv,nb.second);if(pb>bw[nb.first])bw[nb.first]=pb;}
      }
      if(bw[gsName]>best)best=bw[gsName];
    }
    return best;
  };

  for(int t=1;t<simTime;t+=interval){
    Simulator::Schedule(Seconds(t),[&,t]()mutable{
      for(auto&l:links){bool isSgl=l.a.find("GS")!=std::string::npos||l.b.find("GS")!=std::string::npos;double f=isSgl?(1.0+d(rng)*1.5):(1.0+d(rng));l.cur=std::max(60.0,std::min(l.base*1.5,l.base*f));}
      NS_LOG_UNCOND("\n═══ t="<<t<<"s Fluctuated ═══");
      for(auto&l:links)NS_LOG_UNCOND("  "<<l.a<<"↔"<<l.b<<" "<<(int)l.cur<<"M");
      fA->SetBw(computeBw("SAT-A"));fA->Push();fB->SetBw(computeBw("SAT-B"));fB->Push();
      fC->SetBw(computeBw("SAT-C"));fC->Push();fD->SetBw(computeBw("SAT-D"));fD->Push();
    });
  }
  Simulator::Schedule(Seconds(simTime-2),[&](){NS_LOG_UNCOND("\n\n╔══════════════════════════════════════════════╗");NS_LOG_UNCOND("║  Constellation Report (real UDP ADVs)         ║");NS_LOG_UNCOND("╚══════════════════════════════════════════════╝");for(int i=0;i<3;i++){auto&h=users[i]->m_h;if(h.empty())continue;float lo=h[0].bw,hi=h[0].bw;for(auto&e:h){lo=std::min(lo,e.bw);hi=std::max(hi,e.bw);}NS_LOG_UNCOND("  User-"<<(char)('A'+i)<<": "<<h.size()<<" updates, "<<(int)lo<<"-"<<(int)hi<<" Mbps");}});
  Simulator::Stop(Seconds(simTime));Simulator::Run();Simulator::Destroy();return 0;
}
