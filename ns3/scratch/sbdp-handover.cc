/* SBDP Exp 2 — B2 Neighbor Exchange + N2 + ADV (A)
 *
 * B2: SatRouters exchange GS bottleneck tables (RIP-style).
 *     SAT-A tells SAT-B "I can reach GS-Main at 150M".
 *     SAT-B computes min(link_to_A, 150) → updates own table.
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
#include <map>
#include <vector>
#include <sstream>
#include <cstring>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("SbdpHandover");

// ═══════════════════ SatRouter: B2 neighbor exchange ═══════════════════

class SatRouter : public Application {
public:
  static TypeId GetTypeId(); SatRouter();
  void SetBw(double in,double out){m_in=in;m_out=out;}
  void AddNeighbor(std::string n,Ipv4Address ip,double bw){m_nb[n]={ip,bw};}
  void AddDirectGs(std::string g,double bw){m_gs[g]=bw;m_bestGs=g;m_bestBw=bw;}
  double GetBestE2e()const{return m_bestBw>0?m_bestBw:GetLocalBw();}
  double GetLocalBw()const{double b=m_out;if(m_in>0&&m_in<b)b=m_in;return b;}
private:
  virtual void StartApplication()override;
  virtual void StopApplication()override;
  void RecvN2(Ptr<Socket>s);
  void RecvEx(Ptr<Socket>s);
  void SendEx();
  Ptr<Socket> m_n2sk,m_sk;
  double m_in=0,m_out=0,m_bestBw=0;
  std::string m_bestGs;
  struct Nb{Ipv4Address ip;double bw;};
  std::map<std::string,Nb>m_nb;
  std::map<std::string,double>m_gs;
  EventId m_timer;
};
NS_OBJECT_ENSURE_REGISTERED(SatRouter);
TypeId SatRouter::GetTypeId(){static TypeId tid=TypeId("ns3::SatRouter").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<SatRouter>();return tid;}
SatRouter::SatRouter(){}

void SatRouter::StartApplication(){
  m_n2sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
  m_n2sk->Bind(InetSocketAddress(Ipv4Address("127.0.0.1"),9998));
  m_n2sk->SetRecvCallback(MakeCallback(&SatRouter::RecvN2,this));
  m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
  m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9997));
  m_sk->SetRecvCallback(MakeCallback(&SatRouter::RecvEx,this));
  m_timer=Simulator::Schedule(Seconds(0.05),&SatRouter::SendEx,this);
}
void SatRouter::StopApplication(){Simulator::Cancel(m_timer);if(m_n2sk)m_n2sk->Close();if(m_sk)m_sk->Close();}

void SatRouter::RecvN2(Ptr<Socket>s){
  Ptr<Packet>pkt;Address from;
  while((pkt=s->RecvFrom(from))){
    double bw=GetBestE2e();
    std::string bl=Names::FindName(GetNode())+"→"+m_bestGs+"("+std::to_string((int)bw)+"M)";
    uint32_t blen=bl.size();int32_t bwInt=(int32_t)bw;
    uint8_t buf[256];memcpy(buf,&bwInt,4);memcpy(buf+4,&blen,4);memcpy(buf+8,bl.c_str(),blen);
    s->SendTo(Create<Packet>(buf,8+blen),0,from);
    NS_LOG_UNCOND("  [R "<<Names::FindName(GetNode())<<"] N2→gNB: e2e="<<(int)bw<<"M to "<<m_bestGs<<" (local="<<(int)GetLocalBw()<<"M)");
  }
}

void SatRouter::SendEx(){
  if(m_gs.empty()){m_timer=Simulator::Schedule(Seconds(0.2),&SatRouter::SendEx,this);return;}
  std::string myName=Names::FindName(GetNode());
  uint8_t buf[512];int pos=0;
  buf[pos++]=(uint8_t)myName.size();memcpy(buf+pos,myName.c_str(),myName.size());pos+=myName.size();
  int n=0;for(auto&p:m_gs)n++;
  buf[pos++]=(uint8_t)n;
  for(auto&p:m_gs){uint8_t nl=p.first.size();buf[pos++]=nl;memcpy(buf+pos,p.first.c_str(),nl);pos+=nl;int32_t bw=(int32_t)p.second;memcpy(buf+pos,&bw,4);pos+=4;}
  Ptr<Packet>pkt=Create<Packet>(buf,pos);
  for(auto&nb:m_nb)m_sk->SendTo(pkt,0,InetSocketAddress(nb.second.ip,9997));
  NS_LOG_UNCOND("  [B2 "<<Names::FindName(GetNode())<<"] advertise: "<<m_bestGs<<":"<<(int)m_bestBw<<"M ("<<n<<" GS) → "<<m_nb.size()<<" neighbors");
  m_timer=Simulator::Schedule(Seconds(0.2),&SatRouter::SendEx,this);
}

void SatRouter::RecvEx(Ptr<Socket>s){
  Ptr<Packet>pkt;Address from;
  while((pkt=s->RecvFrom(from))){
    uint8_t buf[512];pkt->CopyData(buf,pkt->GetSize());
    if(pkt->GetSize()<2)continue;
    uint8_t nameLen=buf[0];std::string nbName((char*)buf+1,nameLen);int pos=1+nameLen;
    if(pkt->GetSize()<pos+1)continue;
    int n=buf[pos++];bool chg=false;
    double nbLink=1e9;
    if(m_nb.count(nbName))nbLink=m_nb[nbName].bw;
    for(int i=0;i<n&&pos<(int)pkt->GetSize();i++){
      uint8_t nl=buf[pos++];std::string gs((char*)buf+pos,nl);pos+=nl;
      int32_t nbBw;memcpy(&nbBw,buf+pos,4);pos+=4;
      double newBw=std::min(nbLink,(double)nbBw);
      if(!m_gs.count(gs)||newBw>m_gs[gs]){m_gs[gs]=newBw;chg=true;}
    }
    if(chg){m_bestBw=0;for(auto&p:m_gs)if(p.second>m_bestBw){m_bestBw=p.second;m_bestGs=p.first;}
      NS_LOG_UNCOND("  [B2 "<<Names::FindName(GetNode())<<"] ←"<<nbName<<" update: best="<<m_bestGs<<":"<<(int)m_bestBw<<"M (via "<<nbName<<" link="<<(int)nbLink<<"M)");
    }
  }
}

// ═══════════════════ gNB ═══════════════════

class GnbApp : public Application {
public:
  static TypeId GetTypeId();GnbApp();
  void AddCoverage(Ipv4Address ua){m_cov.push_back(ua);}
  void SendN2Query();void Push();
private:
  virtual void StartApplication()override{m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9999));m_sk->SetRecvCallback(MakeCallback(&GnbApp::RecvN2,this));}
  virtual void StopApplication()override{if(m_sk)m_sk->Close();}
  void RecvN2(Ptr<Socket>s);
  Ptr<Socket>m_sk;std::vector<Ipv4Address>m_cov;uint16_t m_seq=0;double m_bw=0;std::string m_bl;bool m_wait=false;
};
NS_OBJECT_ENSURE_REGISTERED(GnbApp);
TypeId GnbApp::GetTypeId(){static TypeId tid=TypeId("ns3::GnbApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<GnbApp>();return tid;}
GnbApp::GnbApp(){}

void GnbApp::SendN2Query(){uint8_t q=0;m_sk->SendTo(Create<Packet>(&q,1),0,InetSocketAddress(Ipv4Address("127.0.0.1"),9998));}
void GnbApp::RecvN2(Ptr<Socket>s){
  Ptr<Packet>pkt;Address from;
  while((pkt=s->RecvFrom(from))){
    uint8_t buf[256];pkt->CopyData(buf,pkt->GetSize());
    if(pkt->GetSize()>=8){int32_t bwInt;memcpy(&bwInt,buf,4);m_bw=bwInt;uint32_t blen;memcpy(&blen,buf+4,4);m_bl=std::string((char*)buf+8,blen);
    NS_LOG_UNCOND("  [gNB "<<Names::FindName(GetNode())<<"] N2 resp: e2e="<<(int)m_bw<<"M "<<m_bl);}
    if(m_wait){m_wait=false;Push();}
  }
}
void GnbApp::Push(){
  if(m_bw<=0){m_wait=true;SendN2Query();return;}
  m_seq++;for(auto&ua:m_cov){SbdpHeader h=SbdpHeader::BuildAdv(Names::FindName(GetNode()),"UE",m_bw,m_bl,m_seq);Ptr<Packet>p=Create<Packet>(0);p->AddHeader(h);m_sk->SendTo(p,0,InetSocketAddress(ua,8888));}
}

// ═══════════════════ UsrApp ═══════════════════

class UsrApp : public Application {
public:
  static TypeId GetTypeId();UsrApp();
  std::map<std::string,float>m_map;std::string m_cur;int m_ho=0;float m_thr=200.0;
private:
  virtual void StartApplication()override{m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),8888));m_sk->SetRecvCallback(MakeCallback(&UsrApp::Recv,this));}
  virtual void StopApplication()override{if(m_sk)m_sk->Close();}
  void Recv(Ptr<Socket>s){Ptr<Packet>pkt;Address from;while((pkt=s->RecvFrom(from))){SbdpHeader h;if(pkt->GetSize()>=SbdpHeader::SBDP_FIXED_SIZE){pkt->RemoveHeader(h);std::string sn=h.GetSrcNode();m_map[sn]=h.GetBackhaulBw();if(m_cur.empty())m_cur=sn;NS_LOG_UNCOND("  ["<<Names::FindName(GetNode())<<" @"<<Simulator::Now().GetSeconds()<<"s] ←"<<sn<<" bw="<<(int)h.GetBackhaulBw()<<"M map="<<[&](){std::ostringstream ss;for(auto&p:m_map){if(&p!=&*m_map.begin())ss<<",";ss<<p.first<<":"<<(int)p.second;}return ss.str();}());}}}
  Ptr<Socket>m_sk;
};
NS_OBJECT_ENSURE_REGISTERED(UsrApp);
TypeId UsrApp::GetTypeId(){static TypeId tid=TypeId("ns3::UsrApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<UsrApp>();return tid;}
UsrApp::UsrApp(){}

// ═══════════════════ main ═══════════════════

int main(int argc,char*argv[]){
  double simTime=85;CommandLine cmd;cmd.AddValue("simTime","s",simTime);cmd.Parse(argc,argv);
  NodeContainer n;n.Create(8);
  Names::Add("User-1",n.Get(0));Names::Add("User-2",n.Get(1));Names::Add("User-3",n.Get(2));
  Names::Add("SAT-A",n.Get(3));Names::Add("SAT-B",n.Get(4));Names::Add("SAT-C",n.Get(5));
  Names::Add("GS-Main",n.Get(6));Names::Add("GS-Alt",n.Get(7));
  InternetStackHelper inet;inet.Install(n);

  PointToPointHelper p2p;p2p.SetQueue("ns3::DropTailQueue");
  Ipv4AddressHelper ipv4;uint32_t sn=0;
  NetDeviceContainer du1,du2,du3,dsa_sb,dsb_gs,dsc_gs;
  auto L=[&](int a,int b,double bw,double d){p2p.SetDeviceAttribute("DataRate",StringValue(std::to_string((int)bw)+"Mbps"));p2p.SetChannelAttribute("Delay",StringValue(std::to_string(d)+"ms"));auto dev=p2p.Install(n.Get(a),n.Get(b));char base[32];snprintf(base,32,"10.%u.0.0",sn++);ipv4.SetBase(base,"255.255.255.0");ipv4.Assign(dev);return dev;};
  auto setL=[&](NetDeviceContainer&dev,double bw){DynamicCast<PointToPointNetDevice>(dev.Get(0))->SetDataRate(DataRate(bw*1e6));DynamicCast<PointToPointNetDevice>(dev.Get(1))->SetDataRate(DataRate(bw*1e6));};

  dsa_sb=L(3,4,500,10);dsb_gs=L(4,6,600,10);L(5,3,400,15);dsc_gs=L(5,7,400,10);L(4,5,350,15);
  du1=L(0,3,200,5);du2=L(1,4,150,5);du3=L(2,5,250,5);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();

  auto ip=[&](int i)->Ipv4Address{return n.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();};

  // ── Routers with B2 neighbors ──
  struct Sat{Ptr<SatRouter>r;Ptr<GnbApp>g;};
  std::vector<Sat>sats(3);
  for(int i=0;i<3;i++){
    sats[i].r=CreateObject<SatRouter>();
    double in=i==0?200:(i==1?150:250),out=i==0?500:(i==1?600:400);
    sats[i].r->SetBw(in,out);
    sats[i].g=CreateObject<GnbApp>();
    n.Get(3+i)->AddApplication(sats[i].r);sats[i].r->SetStartTime(Seconds(0));
    n.Get(3+i)->AddApplication(sats[i].g);sats[i].g->SetStartTime(Seconds(0.05));
  }
  // B2: SAT-A ↔ SAT-B (500M), SAT-B ↔ SAT-C (350M), SAT-C ↔ SAT-A (400M)
  sats[0].r->AddNeighbor("SAT-B",ip(4),500);sats[0].r->AddNeighbor("SAT-C",ip(5),400);
  sats[1].r->AddNeighbor("SAT-A",ip(3),500);sats[1].r->AddNeighbor("SAT-C",ip(5),350);
  sats[2].r->AddNeighbor("SAT-A",ip(3),400);sats[2].r->AddNeighbor("SAT-B",ip(4),350);
  // Direct GS connections
  sats[1].r->AddDirectGs("GS-Main",600);  // SAT-B directly connected to GS-Main
  sats[2].r->AddDirectGs("GS-Alt",400);   // SAT-C directly connected to GS-Alt

  // Coverage
  for(int i=0;i<3;i++){sats[i].g->AddCoverage(ip(0));sats[i].g->AddCoverage(ip(1));sats[i].g->AddCoverage(ip(2));}

  Ptr<UsrApp>u1=CreateObject<UsrApp>();n.Get(0)->AddApplication(u1);u1->SetStartTime(Seconds(0.05));
  Ptr<UsrApp>u2=CreateObject<UsrApp>();n.Get(1)->AddApplication(u2);u2->SetStartTime(Seconds(0.05));
  Ptr<UsrApp>u3=CreateObject<UsrApp>();n.Get(2)->AddApplication(u3);u3->SetStartTime(Seconds(0.05));
  std::vector<Ptr<UsrApp>>users={u1,u2,u3};

  auto pushAll=[&](){for(auto&s:sats){s.g->SendN2Query();s.g->Push();}};
  auto hoCheck=[&](){for(auto&u:users){float cur=u->m_map.count(u->m_cur)?u->m_map[u->m_cur]:999.0f;if(cur<u->m_thr){float best=cur;std::string bestSat;for(auto&p:u->m_map)if(p.first!=u->m_cur&&p.second>best){best=p.second;bestSat=p.first;}if(!bestSat.empty()){NS_LOG_UNCOND("  ↳ HANDOVER "<<u->m_cur<<"→"<<bestSat);u->m_cur=bestSat;u->m_ho++;}}}};

  auto ev=[&](double t,const char*d){Simulator::Schedule(Seconds(t),[=](){NS_LOG_UNCOND("\n═══ t="<<t<<"s "<<d<<" ═══");pushAll();});Simulator::Schedule(Seconds(t+0.15),hoCheck);};

  // Initial
  Simulator::Schedule(Seconds(0.2),[&](){pushAll();});
  ev(3,"INITIAL");
  // Events
  Simulator::Schedule(Seconds(10),[&](){sats[2].r->SetBw(250,120);setL(dsc_gs,120);ev(10,"SAT-C→GS-Alt rain 400→120M");});
  Simulator::Schedule(Seconds(15),[&](){sats[1].r->SetBw(200,600);ev(15,"★ User-3 SAT-C→SAT-B");});
  Simulator::Schedule(Seconds(20),[&](){sats[0].r->SetBw(200,180);setL(dsa_sb,180);sats[0].r->AddNeighbor("SAT-B",ip(4),180);sats[1].r->AddNeighbor("SAT-A",ip(3),180);ev(20,"SAT-A→SAT-B ISL 500→180M");});
  Simulator::Schedule(Seconds(25),[&](){sats[0].r->SetBw(200,180);ev(25,"★ User-1 SAT-A→SAT-C");});
  Simulator::Schedule(Seconds(35),[&](){sats[1].r->SetBw(200,250);setL(dsb_gs,250);sats[1].r->AddDirectGs("GS-Main",250);ev(35,"SAT-B→GS-Main 600→250M");});
  Simulator::Schedule(Seconds(40),[&](){sats[0].r->SetBw(180,180);ev(40,"★ User-2 SAT-B→SAT-A");});
  Simulator::Schedule(Seconds(50),[&](){sats[2].r->SetBw(250,380);setL(dsc_gs,380);sats[2].r->AddDirectGs("GS-Alt",380);ev(50,"SAT-C→GS-Alt recover");});
  Simulator::Schedule(Seconds(60),[&](){sats[0].r->SetBw(200,500);setL(dsa_sb,500);sats[0].r->AddNeighbor("SAT-B",ip(4),500);sats[1].r->AddNeighbor("SAT-A",ip(3),500);ev(60,"SAT-A→SAT-B recover");});
  Simulator::Schedule(Seconds(65),[&](){sats[2].r->SetBw(250,250);ev(65,"★ User-1 SAT-C→SAT-A");});
  Simulator::Schedule(Seconds(70),[&](){sats[1].r->SetBw(200,600);setL(dsb_gs,600);sats[1].r->AddDirectGs("GS-Main",600);ev(70,"SAT-B→GS-Main recover");});
  ev(80,"FINAL");

  // Data flows
  Ipv4Address ga=ip(6);std::vector<Ptr<PacketSink>>sinks;
  for(int i=0;i<3;i++){PacketSinkHelper sk("ns3::UdpSocketFactory",InetSocketAddress(Ipv4Address::GetAny(),5001+i));auto a=sk.Install(n.Get(6));a.Start(Seconds(0.3));a.Stop(Seconds(simTime));sinks.push_back(DynamicCast<PacketSink>(a.Get(0)));OnOffHelper oo("ns3::UdpSocketFactory",InetSocketAddress(ga,5001+i));oo.SetAttribute("DataRate",DataRateValue(DataRate("100Mbps")));oo.SetAttribute("PacketSize",UintegerValue(1472));oo.SetConstantRate(DataRate("100Mbps"));auto c=oo.Install(n.Get(i));c.Start(Seconds(0.5));c.Stop(Seconds(simTime));}

  p2p.EnablePcap("sbdp-ho",du1.Get(0),true);p2p.EnablePcap("sbdp-ho",du2.Get(0),true);

  Simulator::Schedule(Seconds(simTime-1),[&](){NS_LOG_UNCOND("\n\n╔══════════════════════╗\n║ Exp 2 B2 Report ║\n╚══════════════════════╝");
    for(int i=0;i<3;i++){double mb=sinks[i]->GetTotalRx()/1e6;NS_LOG_UNCOND("  User-"<<(i+1)<<": "<<users[i]->m_map.size()<<" sats "<<users[i]->m_ho<<" ho | data:"<<mb<<"MB | cur="<<users[i]->m_cur);}});

  NS_LOG_UNCOND("\n╔══════════════════════════════════╗\n║ Exp 2: B2 Neighbor Exchange ║\n╚══════════════════════════════════╝\n");
  Simulator::Stop(Seconds(simTime));Simulator::Run();Simulator::Destroy();return 0;
}
