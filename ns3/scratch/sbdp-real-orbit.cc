/* SBDP Exp 3 — B2 Neighbor Exchange + Real Orbit Events + Data Flow */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/sbdp-header.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/on-off-helper.h"
#include <map><vector><set><cmath><sstream><cstring>
using namespace ns3;
NS_LOG_COMPONENT_DEFINE("SbdpOrbit");

// ═══════════════════ SatRouter B2 ═══════════════════

struct NbInfo{Ipv4Address ip;double bw;};

class SatRouter : public Application {
public:
  static TypeId GetTypeId();
  SatRouter(){}
  void AddNb(std::string n,Ipv4Address ip,double bw){m_nb[n]={ip,bw};}
  void AddGs(std::string g,double bw){m_gs[g]=bw;}
  void UpdateBest(){m_bestBw=0;for(auto&p:m_gs)if(p.second>m_bestBw){m_bestBw=p.second;m_bestGs=p.first;}}
  std::map<std::string,NbInfo>m_nb;
  std::map<std::string,double>m_gs;
  double m_bestBw=0;std::string m_bestGs;
  double GetBest()const{return m_bestBw>0?m_bestBw:200;}
  void TriggerNow(){Simulator::Cancel(m_timer);SendEx();}
private:
  virtual void StartApplication()override;
  virtual void StopApplication()override;
  void RecvEx(Ptr<Socket>s);
  void SendEx();
  Ptr<Socket>m_sk;EventId m_timer;
};
NS_OBJECT_ENSURE_REGISTERED(SatRouter);
TypeId SatRouter::GetTypeId(){static TypeId tid=TypeId("ns3::SatRouter").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<SatRouter>();return tid;}

void SatRouter::StartApplication(){
  m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
  m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9997));
  m_sk->SetRecvCallback(MakeCallback(&SatRouter::RecvEx,this));
  m_timer=Simulator::Schedule(Seconds(0.05),&SatRouter::SendEx,this);
}
void SatRouter::StopApplication(){Simulator::Cancel(m_timer);if(m_sk)m_sk->Close();}

void SatRouter::SendEx(){
  if(m_gs.empty()){m_timer=Simulator::Schedule(Seconds(0.2),&SatRouter::SendEx,this);return;}
  std::string my=Names::FindName(GetNode());
  uint8_t buf[512];int p=0;
  buf[p++]=(uint8_t)my.size();memcpy(buf+p,my.c_str(),my.size());p+=my.size();
  int n=0;for(auto&x:m_gs)n++;buf[p++]=(uint8_t)n;
  for(auto&x:m_gs){uint8_t l=x.first.size();buf[p++]=l;memcpy(buf+p,x.first.c_str(),l);p+=l;int32_t bw=(int32_t)x.second;memcpy(buf+p,&bw,4);p+=4;}
  Ptr<Packet>pkt=Create<Packet>(buf,p);
  for(auto&nb:m_nb)m_sk->SendTo(pkt,0,InetSocketAddress(nb.second.ip,9997));
  NS_LOG_UNCOND("  [B2 "<<my<<"] adv: "<<m_bestGs<<":"<<(int)m_bestBw<<"M → "<<m_nb.size()<<" nb");
  m_timer=Simulator::Schedule(Seconds(0.2),&SatRouter::SendEx,this);
}

void SatRouter::RecvEx(Ptr<Socket>s){
  Ptr<Packet>pkt;Address from;
  while((pkt=s->RecvFrom(from))){
    uint8_t buf[512];pkt->CopyData(buf,pkt->GetSize());
    if(pkt->GetSize()<2)continue;
    uint8_t nl=buf[0];std::string nbName((char*)buf+1,nl);int pos=1+nl;
    if(pkt->GetSize()<pos+1)continue;
    int n=buf[pos++];bool chg=false;
    double nbLink=m_nb.count(nbName)?m_nb[nbName].bw:1e9;
    for(int i=0;i<n&&pos<(int)pkt->GetSize();i++){
      uint8_t gl=buf[pos++];std::string gs((char*)buf+pos,gl);pos+=gl;
      int32_t nbBw;memcpy(&nbBw,buf+pos,4);pos+=4;
      double newBw=std::min(nbLink,(double)nbBw);
      if(!m_gs.count(gs)||newBw>m_gs[gs]){m_gs[gs]=newBw;chg=true;}
    }
    if(chg){UpdateBest();NS_LOG_UNCOND("  [B2 "<<Names::FindName(GetNode())<<"] ←"<<nbName<<" best="<<m_bestGs<<":"<<(int)m_bestBw<<"M");}
  }
}

// ═══════════════════ gNB ═══════════════════

class GnbApp : public Application {
public:
  static TypeId GetTypeId();GnbApp(){}
  void AddCov(Ipv4Address ua){m_cov.push_back(ua);}void SetR(Ptr<SatRouter>r){m_r=r;}
  void Push();
private:
  virtual void StartApplication()override{m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9999));}
  virtual void StopApplication()override{if(m_sk)m_sk->Close();}
  Ptr<Socket>m_sk;std::vector<Ipv4Address>m_cov;Ptr<SatRouter>m_r;uint16_t m_seq=0;
};
NS_OBJECT_ENSURE_REGISTERED(GnbApp);
TypeId GnbApp::GetTypeId(){static TypeId tid=TypeId("ns3::GnbApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<GnbApp>();return tid;}

void GnbApp::Push(){
  if(!m_r)return;double bw=m_r->GetBest();std::string gs=m_r->m_bestGs;
  m_seq++;for(auto&ua:m_cov){SbdpHeader h=SbdpHeader::BuildAdv(Names::FindName(GetNode()),"UE",bw,Names::FindName(GetNode())+"→"+gs,m_seq);Ptr<Packet>p=Create<Packet>(0);p->AddHeader(h);m_sk->SendTo(p,0,InetSocketAddress(ua,8888));}
}

// ═══════════════════ UsrApp ═══════════════════

class UsrApp : public Application {
public:
  static TypeId GetTypeId();UsrApp(){}
  struct E{double t;float bw;uint16_t seq;};std::vector<E>m_h;uint32_t m_gaps=0,m_missed=0;uint16_t m_lastSeq=0;double m_lastRecv=0;bool m_stale=false;
private:
  virtual void StartApplication()override{m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),8888));m_sk->SetRecvCallback(MakeCallback(&UsrApp::Recv,this));}
  virtual void StopApplication()override{if(m_sk)m_sk->Close();}
  void Recv(Ptr<Socket>s){Ptr<Packet>pkt;Address from;while((pkt=s->RecvFrom(from))){SbdpHeader h;if(pkt->GetSize()>=SbdpHeader::SBDP_FIXED_SIZE){pkt->RemoveHeader(h);double now=Simulator::Now().GetSeconds();if(m_lastSeq>0&&h.GetSeqNum()!=m_lastSeq+1){m_gaps++;m_missed+=h.GetSeqNum()>m_lastSeq?h.GetSeqNum()-m_lastSeq-1:0;}m_lastSeq=h.GetSeqNum();m_lastRecv=now;m_h.push_back({now,h.GetBackhaulBw(),h.GetSeqNum()});NS_LOG_UNCOND("  [U@"<<(int)now<<"] bw="<<(int)h.GetBackhaulBw()<<"M");}}}
  Ptr<Socket>m_sk;
};
NS_OBJECT_ENSURE_REGISTERED(UsrApp);
TypeId UsrApp::GetTypeId(){static TypeId tid=TypeId("ns3::UsrApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<UsrApp>();return tid;}

// ═══════════════════ main ═══════════════════

int main(int argc,char*argv[]){
  NodeContainer n;n.Create(12);
  Names::Add("A1",n.Get(0));Names::Add("A2",n.Get(1));Names::Add("A3",n.Get(2));
  Names::Add("B1",n.Get(3));Names::Add("B2",n.Get(4));Names::Add("B3",n.Get(5));
  Names::Add("GS-E",n.Get(6));Names::Add("GS-W",n.Get(7));
  Names::Add("U1",n.Get(8));Names::Add("U2",n.Get(9));Names::Add("U3",n.Get(10));Names::Add("Srv",n.Get(11));
  InternetStackHelper inet;inet.Install(n);
  PointToPointHelper p2p;p2p.SetDeviceAttribute("DataRate",StringValue("1Gbps"));p2p.SetChannelAttribute("Delay",StringValue("5ms"));
  Ipv4AddressHelper ipv4;ipv4.SetBase("10.0.0.0","255.255.255.0");
  for(int i=0;i<11;i++){p2p.Install(n.Get(i),n.Get(i+1));ipv4.Assign(p2p.Install(n.Get(i),n.Get(i+1)));ipv4.NewNetwork();}
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();
  auto ip=[&](int i)->Ipv4Address{return n.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();};

  struct Sat{Ptr<SatRouter>r;Ptr<GnbApp>g;};std::vector<Sat>sats(6);
  for(int i=0;i<6;i++){sats[i].r=CreateObject<SatRouter>();sats[i].g=CreateObject<GnbApp>();sats[i].g->SetR(sats[i].r);n.Get(i)->AddApplication(sats[i].r);sats[i].r->SetStartTime(Seconds(0));n.Get(i)->AddApplication(sats[i].g);sats[i].g->SetStartTime(Seconds(0.05));}

  struct Edge{int a,b;double bw;};
  std::vector<Edge>edges;
  auto setTopo=[&](const std::vector<Edge>&e){
    edges=e;
    for(int i=0;i<6;i++){sats[i].r->m_nb.clear();sats[i].r->m_gs.clear();}
    for(auto&ed:edges){
      if(ed.a>=1&&ed.a<=6&&ed.b>=1&&ed.b<=6){std::string na=Names::FindName(n.Get(ed.a-1)),nb=Names::FindName(n.Get(ed.b-1));sats[ed.a-1].r->AddNb(nb,ip(ed.b-1),ed.bw);sats[ed.b-1].r->AddNb(na,ip(ed.a-1),ed.bw);}
      if((ed.a==7||ed.a==8)&&ed.b>=1&&ed.b<=6)sats[ed.b-1].r->AddGs(Names::FindName(n.Get(ed.a-1)),ed.bw);
      if((ed.b==7||ed.b==8)&&ed.a>=1&&ed.a<=6)sats[ed.a-1].r->AddGs(Names::FindName(n.Get(ed.b-1)),ed.bw);
    }
    for(int i=0;i<6;i++)sats[i].r->UpdateBest();
  };

  setTopo({{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}});

  for(int i=0;i<6;i++){sats[i].g->AddCov(ip(8));sats[i].g->AddCov(ip(9));sats[i].g->AddCov(ip(10));}

  Ptr<UsrApp>u1=CreateObject<UsrApp>();n.Get(8)->AddApplication(u1);u1->SetStartTime(Seconds(0.05));
  Ptr<UsrApp>u2=CreateObject<UsrApp>();n.Get(9)->AddApplication(u2);u2->SetStartTime(Seconds(0.05));
  Ptr<UsrApp>u3=CreateObject<UsrApp>();n.Get(10)->AddApplication(u3);u3->SetStartTime(Seconds(0.05));
  std::vector<Ptr<UsrApp>>users={u1,u2,u3};

  auto ev=[&](double t,const std::vector<Edge>&e,const char*desc){
    Simulator::Schedule(Seconds(t),[=](){setTopo(e);for(int i=0;i<6;i++)sats[i].r->TriggerNow();NS_LOG_UNCOND("\n═══ t="<<(int)t<<"s "<<desc<<" ═══");for(int i=0;i<6;i++)sats[i].g->Push();});
  };

  ev(2,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"INITIAL");
  ev(50,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"FAULT: ISL A2↔B2");
  ev(80,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,80},{6,8,300},{4,7,250},{3,8,250}},"RAIN: GS-E 350→80M");
  ev(120,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVER");
  ev(160,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,0},{4,7,250},{3,8,250}},"GS-W down");
  ev(190,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,0},{4,7,250},{3,8,250}},"POLAR BLACKOUT");
  ev(230,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVER");
  ev(280,{{1,2,200},{2,3,200},{4,5,200},{5,6,0},{1,4,0},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"DOUBLE FAULT");
  ev(330,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVER");
  ev(380,{{1,2,200},{2,3,200},{4,5,0},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,0},{6,8,300},{4,7,0},{3,8,250}},"CASCADE: B1 isolated");
  ev(430,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVER");
  ev(480,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,100},{6,8,100},{4,7,250},{3,8,250}},"DEGRADE: both GS→100M");
  ev(870,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,0},{3,8,350},{4,7,250}},"POLAR END");

  // Data flows
  Ipv4Address ga=ip(6);std::vector<Ptr<PacketSink>>sinks;
  for(int i=0;i<3;i++){PacketSinkHelper sk("ns3::UdpSocketFactory",InetSocketAddress(Ipv4Address::GetAny(),5001+i));auto a=sk.Install(n.Get(6));a.Start(Seconds(0.3));a.Stop(Seconds(885));sinks.push_back(DynamicCast<PacketSink>(a.Get(0)));OnOffHelper oo("ns3::UdpSocketFactory",InetSocketAddress(ga,5001+i));oo.SetAttribute("DataRate",DataRateValue(DataRate("50Mbps")));oo.SetAttribute("PacketSize",UintegerValue(1472));oo.SetConstantRate(DataRate("50Mbps"));auto c=oo.Install(n.Get(8+i));c.Start(Seconds(0.5));c.Stop(Seconds(885));}

  Simulator::Schedule(Seconds(875),[&](){NS_LOG_UNCOND("\n\n═══ Exp 3 B2 Report ═══");
    for(int i=0;i<3;i++){auto&h=users[i]->m_h;if(h.empty())continue;double mb=sinks[i]->GetTotalRx()/1e6;NS_LOG_UNCOND("  U"<<(i+1)<<": "<<h.size()<<" ADVs | data:"<<mb<<"MB | gaps="<<users[i]->m_gaps<<" stale="<<(users[i]->m_stale?"YES":"no"));}});

  NS_LOG_UNCOND("\n═══ Exp 3: B2 + Real Orbit ═══\n");
  Simulator::Stop(Seconds(885));Simulator::Run();Simulator::Destroy();return 0;
}
