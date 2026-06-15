/* Exp 3 — User handover → B2 available bw change for remaining users
 * 2 users on SAT-A. ISL degrades → both see B2 drop. User-1 switches →
 * SAT-A load drops → User-2 sees B2 recover slightly.
 * Uses SetNbUsed (protocol logic verified; queue monitoring needs NS-3 compat fix).
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
#include <map><vector><sstream><cstring>
using namespace ns3;
NS_LOG_COMPONENT_DEFINE("SbdpHoUser");

class SatRouter : public Application {
public:
  static TypeId GetTypeId();SatRouter(){}
  void SetBw(double in,double out){m_in=in;m_out=out;}
  void AddNb(std::string n,Ipv4Address ip,double bw){m_nb[n]={ip,bw};}
  void SetNbUsed(std::string n,double u){m_nbUsed[n]=u;}
  void AddDirectGs(std::string g,double bw,double used=0){m_gs[g]=bw-used;m_bestGs=g;m_bestBw=bw-used;m_nexthop[g]="";}
  void SetRoute(std::string gs,std::string nb){m_nexthop[gs]=nb;}
  Ipv4Address GetNbIp(const std::string&n)const{auto it=m_nb.find(n);return it!=m_nb.end()?it->second.ip:Ipv4Address::GetAny();}
  double GetBestE2e()const{return m_bestBw>0?m_bestBw:GetLocalBw();}
  std::string GetBestGs()const{return m_bestGs;}
  double GetLocalBw()const{double b=m_out;if(m_in>0&&m_in<b)b=m_in;return b;}
  struct Nb{Ipv4Address ip;double bw;};
  std::map<std::string,Nb>m_nb;
private:
  virtual void StartApplication()override;virtual void StopApplication()override;
  void RecvN2(Ptr<Socket>s);void RecvEx(Ptr<Socket>s);
  void SendPush();void SendRequest();void SendReply(const std::string&);void CheckPortChange();
  Ptr<Socket>m_n2sk,m_sk;double m_in=0,m_out=0,m_bestBw=0;std::string m_bestGs;
  std::map<std::string,double>m_gs,m_nbUsed,m_nbSnapshot;
  std::map<std::string,std::string>m_nexthop;
  EventId m_ct;bool m_init=false;uint16_t m_seq=0;
};
NS_OBJECT_ENSURE_REGISTERED(SatRouter);
TypeId SatRouter::GetTypeId(){static TypeId tid=TypeId("ns3::SatRouter").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<SatRouter>();return tid;}

void SatRouter::StartApplication(){
  m_n2sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
  m_n2sk->Bind(InetSocketAddress(Ipv4Address("127.0.0.1"),9998));
  m_n2sk->SetRecvCallback(MakeCallback(&SatRouter::RecvN2,this));
  m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
  m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9997));
  m_sk->SetRecvCallback(MakeCallback(&SatRouter::RecvEx,this));
  Simulator::Schedule(Seconds(0.05),&SatRouter::SendRequest,this);
  m_ct=Simulator::Schedule(Seconds(0.20),&SatRouter::CheckPortChange,this);
}
void SatRouter::StopApplication(){Simulator::Cancel(m_ct);if(m_n2sk)m_n2sk->Close();if(m_sk)m_sk->Close();}
void SatRouter::RecvN2(Ptr<Socket>s){
  Ptr<Packet>pkt;Address from;
  while((pkt=s->RecvFrom(from))){double bw=GetBestE2e();std::string bl=Names::FindName(GetNode())+"→"+m_bestGs+"("+std::to_string((int)bw)+"M)";uint32_t blen=bl.size();int32_t bwInt=(int32_t)bw;uint8_t buf[256];memcpy(buf,&bwInt,4);memcpy(buf+4,&blen,4);memcpy(buf+8,bl.c_str(),blen);s->SendTo(Create<Packet>(buf,8+blen),0,from);}
}
void SatRouter::SendPush(){
  if(m_gs.empty())return;std::string my=Names::FindName(GetNode());uint8_t buf[512];int p=0;
  buf[p++]=0x42;buf[p++]=0x32;buf[p++]=1;buf[p++]=0;p+=2;uint16_t seq=m_seq++;memcpy(buf+p,&seq,2);p+=2;
  buf[p++]=(uint8_t)my.size();memcpy(buf+p,my.c_str(),my.size());p+=my.size();
  int n=0;for(auto&x:m_gs)n++;buf[p++]=(uint8_t)n;
  for(auto&x:m_gs){uint8_t l=x.first.size();buf[p++]=l;memcpy(buf+p,x.first.c_str(),l);p+=l;int32_t bw=(int32_t)x.second;memcpy(buf+p,&bw,4);p+=4;}
  uint16_t total=(uint16_t)p;memcpy(buf+4,&total,2);Ptr<Packet>pkt=Create<Packet>(buf,p);
  for(auto&nb:m_nb)m_sk->SendTo(pkt,0,InetSocketAddress(nb.second.ip,9997));
  m_nbSnapshot.clear();for(auto&nb:m_nb)m_nbSnapshot[nb.first]=nb.second.bw;
}
void SatRouter::SendRequest(){
  std::string my=Names::FindName(GetNode());uint8_t buf[256];int p=0;
  buf[p++]=0x42;buf[p++]=0x32;buf[p++]=1;buf[p++]=1;p+=2;uint16_t seq=m_seq++;memcpy(buf+p,&seq,2);p+=2;
  buf[p++]=(uint8_t)my.size();memcpy(buf+p,my.c_str(),my.size());p+=my.size();
  uint16_t total=(uint16_t)p;memcpy(buf+4,&total,2);Ptr<Packet>pkt=Create<Packet>(buf,p);
  for(auto&nb:m_nb)m_sk->SendTo(pkt,0,InetSocketAddress(nb.second.ip,9997));
}
void SatRouter::SendReply(const std::string&tgt){
  if(m_gs.empty()||!m_nb.count(tgt))return;std::string my=Names::FindName(GetNode());uint8_t buf[512];int p=0;
  buf[p++]=0x42;buf[p++]=0x32;buf[p++]=1;buf[p++]=2;p+=2;uint16_t seq=m_seq++;memcpy(buf+p,&seq,2);p+=2;
  buf[p++]=(uint8_t)my.size();memcpy(buf+p,my.c_str(),my.size());p+=my.size();
  int n=0;for(auto&x:m_gs)n++;buf[p++]=(uint8_t)n;
  for(auto&x:m_gs){uint8_t l=x.first.size();buf[p++]=l;memcpy(buf+p,x.first.c_str(),l);p+=l;int32_t bw=(int32_t)x.second;memcpy(buf+p,&bw,4);p+=4;}
  uint16_t total=(uint16_t)p;memcpy(buf+4,&total,2);Ptr<Packet>pkt=Create<Packet>(buf,p);m_sk->SendTo(pkt,0,InetSocketAddress(m_nb[tgt].ip,9997));
}
void SatRouter::RecvEx(Ptr<Socket>s){
  Ptr<Packet>pkt;Address from;bool prop=false;
  while((pkt=s->RecvFrom(from))){uint8_t buf[512];pkt->CopyData(buf,pkt->GetSize());if(pkt->GetSize()<9)continue;if(buf[0]!=0x42||buf[1]!=0x32)continue;if(buf[2]!=1)continue;uint8_t type=buf[3];uint16_t seq;memcpy(&seq,buf+6,2);uint8_t nl=buf[8];std::string nb((char*)buf+9,nl);int pos=9+nl;double nbLink=m_nb.count(nb)?m_nb[nb].bw-(m_nbUsed.count(nb)?m_nbUsed[nb]:0):1e9;if(nbLink<0)nbLink=0;if(type==1){SendReply(nb);}else if(type==0||type==2){if(pkt->GetSize()<(uint32_t)pos+1)continue;int n=buf[pos++];bool chg=false;for(int i=0;i<n&&pos<(int)pkt->GetSize();i++){uint8_t gl=buf[pos++];std::string gs((char*)buf+pos,gl);pos+=gl;int32_t nbBw;memcpy(&nbBw,buf+pos,4);pos+=4;if(!m_nexthop.count(gs))continue;if(m_nexthop[gs]=="")continue;if(m_nexthop[gs]!=nb)continue;double newBw=std::min(nbLink,(double)nbBw);if(!m_gs.count(gs)||newBw>m_gs[gs]){m_gs[gs]=newBw;chg=true;}}if(chg){m_bestBw=0;for(auto&p:m_gs)if(p.second>m_bestBw){m_bestBw=p.second;m_bestGs=p.first;}prop=true;}}}if(prop)SendPush();
}
void SatRouter::CheckPortChange(){
  std::string my=Names::FindName(GetNode());
  if(!m_init&&!m_gs.empty()){m_init=true;m_nbSnapshot.clear();for(auto&nb:m_nb)m_nbSnapshot[nb.first]=nb.second.bw;}
  else if(m_init){bool chg=false;if(m_nb.size()!=m_nbSnapshot.size())chg=true;else{for(auto&nb:m_nb)if(!m_nbSnapshot.count(nb.first)||std::abs(m_nbSnapshot[nb.first]-nb.second.bw)>0.5){chg=true;break;}}if(chg){for(auto&nb:m_nb){double old=m_nbSnapshot.count(nb.first)?m_nbSnapshot[nb.first]:0;if(std::abs(nb.second.bw-old)>0.5||old==0){for(auto&gs:m_nexthop)if(gs.second==nb.first)m_gs[gs.first]=0;}}SendPush();m_nbSnapshot.clear();for(auto&nb:m_nb)m_nbSnapshot[nb.first]=nb.second.bw;}}
  m_bestBw=0;for(auto&p:m_gs)if(p.second>m_bestBw){m_bestBw=p.second;m_bestGs=p.first;}
  m_ct=Simulator::Schedule(Seconds(0.20),&SatRouter::CheckPortChange,this);
}

class GnbApp:public Application{public:
  static TypeId GetTypeId();GnbApp(){}void AddCov(Ipv4Address ua){m_cov.push_back(ua);}void SetR(Ptr<SatRouter>r){m_r=r;}void Push();
private:
  virtual void StartApplication()override;virtual void StopApplication()override;void AutoPush();
  Ptr<Socket>m_sk;std::vector<Ipv4Address>m_cov;Ptr<SatRouter>m_r;uint16_t m_seq=0;EventId m_ap;
};
NS_OBJECT_ENSURE_REGISTERED(GnbApp);
TypeId GnbApp::GetTypeId(){static TypeId tid=TypeId("ns3::GnbApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<GnbApp>();return tid;}
void GnbApp::StartApplication(){m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9999));m_ap=Simulator::Schedule(Seconds(0.3),&GnbApp::AutoPush,this);}
void GnbApp::StopApplication(){Simulator::Cancel(m_ap);if(m_sk)m_sk->Close();}
void GnbApp::AutoPush(){Push();m_ap=Simulator::Schedule(Seconds(1.0),&GnbApp::AutoPush,this);}
void GnbApp::Push(){if(!m_r)return;double bw=m_r->GetBestE2e();std::string gs=m_r->GetBestGs();m_seq++;for(auto&ua:m_cov){SbdpHeader h=SbdpHeader::BuildAdv(Names::FindName(GetNode()),"UE",bw,Names::FindName(GetNode())+"→"+gs,m_seq);Ptr<Packet>p=Create<Packet>(0);p->AddHeader(h);m_sk->SendTo(p,0,InetSocketAddress(ua,8888));}NS_LOG_UNCOND("  [ADV "<<Names::FindName(GetNode())<<"@"<<Simulator::Now().GetSeconds()<<"s] bw="<<bw<<"M to "<<gs);}

class UsrApp:public Application{public:
  static TypeId GetTypeId();UsrApp(){}std::map<std::string,float>m_map;std::string m_cur;int m_ho=0;float m_thr=200;
private:
  virtual void StartApplication()override{m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),8888));m_sk->SetRecvCallback(MakeCallback(&UsrApp::Recv,this));}
  virtual void StopApplication()override{if(m_sk)m_sk->Close();}
  void Recv(Ptr<Socket>s){Ptr<Packet>pkt;Address from;while((pkt=s->RecvFrom(from))){SbdpHeader h;if(pkt->GetSize()>=SbdpHeader::SBDP_FIXED_SIZE){pkt->RemoveHeader(h);std::string sn=h.GetSrcNode();float bw=h.GetBackhaulBw();m_map[sn]=bw;if(m_cur.empty())m_cur=sn;NS_LOG_UNCOND("  ["<<Names::FindName(GetNode())<<"@"<<Simulator::Now().GetSeconds()<<"s] ←"<<sn<<" bw="<<bw<<"M");}}}
  Ptr<Socket>m_sk;
};
NS_OBJECT_ENSURE_REGISTERED(UsrApp);
TypeId UsrApp::GetTypeId(){static TypeId tid=TypeId("ns3::UsrApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<UsrApp>();return tid;}

int main(int argc,char*argv[]){
  NodeContainer n;n.Create(6);
  Names::Add("U1",n.Get(0));Names::Add("U2",n.Get(1));Names::Add("SAT-A",n.Get(2));Names::Add("SAT-B",n.Get(3));Names::Add("SAT-C",n.Get(4));Names::Add("GS",n.Get(5));
  InternetStackHelper inet;inet.Install(n);
  PointToPointHelper p2p;p2p.SetQueue("ns3::DropTailQueue");Ipv4AddressHelper ipv4;uint32_t sn=0;
  auto L=[&](int a,int b,double bw,double d){p2p.SetDeviceAttribute("DataRate",StringValue(std::to_string((int)bw)+"Mbps"));p2p.SetChannelAttribute("Delay",StringValue(std::to_string(d)+"ms"));auto dev=p2p.Install(n.Get(a),n.Get(b));char base[32];snprintf(base,32,"10.%u.0.0",sn++);ipv4.SetBase(base,"255.255.255.0");ipv4.Assign(dev);return dev;};
  auto setL=[&](NetDeviceContainer&dev,double bw){DynamicCast<PointToPointNetDevice>(dev.Get(0))->SetDataRate(DataRate(bw*1e6));DynamicCast<PointToPointNetDevice>(dev.Get(1))->SetDataRate(DataRate(bw*1e6));};
  L(0,2,300,5);L(1,2,300,5);auto dAB=L(2,3,600,5);auto dBG=L(3,5,600,5);L(2,4,400,5);L(4,3,350,5);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();
  auto ip=[&](int i)->Ipv4Address{return n.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();};

  Ptr<SatRouter> rA=CreateObject<SatRouter>(),rB=CreateObject<SatRouter>(),rC=CreateObject<SatRouter>();
  Ptr<GnbApp> gA=CreateObject<GnbApp>(),gB=CreateObject<GnbApp>(),gC=CreateObject<GnbApp>();
  gA->SetR(rA);gB->SetR(rB);gC->SetR(rC);
  n.Get(2)->AddApplication(rA);rA->SetStartTime(Seconds(0));n.Get(2)->AddApplication(gA);gA->SetStartTime(Seconds(0.05));
  n.Get(3)->AddApplication(rB);rB->SetStartTime(Seconds(0));n.Get(3)->AddApplication(gB);gB->SetStartTime(Seconds(0.05));
  n.Get(4)->AddApplication(rC);rC->SetStartTime(Seconds(0));n.Get(4)->AddApplication(gC);gC->SetStartTime(Seconds(0.05));

  rA->SetBw(300,600);rB->SetBw(600,600);rC->SetBw(400,350);
  rA->AddNb("SAT-B",ip(3),600);rA->AddNb("SAT-C",ip(4),400);rA->SetRoute("GS","SAT-B");rA->SetNbUsed("SAT-B",200);
  rB->AddNb("SAT-A",ip(2),600);rB->AddNb("SAT-C",ip(4),350);rB->AddDirectGs("GS",600,200);
  rC->AddNb("SAT-A",ip(2),400);rC->AddNb("SAT-B",ip(3),350);rC->SetRoute("GS","SAT-B");

  // compute routes
  Ptr<Ipv4RoutingProtocol> rpA=n.Get(2)->GetObject<Ipv4>()->GetRoutingProtocol();
  for(auto&gs:std::map<std::string,int>{{"GS",5}}){Ipv4Header hdr;hdr.SetDestination(ip(gs.second));hdr.SetProtocol(17);Socket::SocketErrno se;Ptr<Ipv4Route> rt=rpA->RouteOutput(Create<Packet>(),hdr,Ptr<NetDevice>(),se);if(rt){std::string nh="";Ipv4Address gw=rt->GetGateway();for(auto&nb:rA->m_nb)if(nb.second.ip==gw)nh=nb.first;rA->SetRoute(gs.first,nh);}}

  gA->AddCov(ip(0));gA->AddCov(ip(1));gB->AddCov(ip(0));gB->AddCov(ip(1));gC->AddCov(ip(0));gC->AddCov(ip(1));
  Ptr<UsrApp> u1=CreateObject<UsrApp>();n.Get(0)->AddApplication(u1);u1->SetStartTime(Seconds(0.05));
  Ptr<UsrApp> u2=CreateObject<UsrApp>();n.Get(1)->AddApplication(u2);u2->SetStartTime(Seconds(0.05));

  Ipv4Address gsIp=ip(5);std::vector<Ptr<PacketSink>>sinks;
  for(int i=0;i<2;i++){PacketSinkHelper sk("ns3::UdpSocketFactory",InetSocketAddress(Ipv4Address::GetAny(),5001+i));auto a=sk.Install(n.Get(5));a.Start(Seconds(0.3));a.Stop(Seconds(50));sinks.push_back(DynamicCast<PacketSink>(a.Get(0)));OnOffHelper oo("ns3::UdpSocketFactory",InetSocketAddress(gsIp,5001+i));oo.SetAttribute("DataRate",DataRateValue(DataRate("100Mbps")));oo.SetAttribute("PacketSize",UintegerValue(1472));oo.SetConstantRate(DataRate("100Mbps"));auto c=oo.Install(n.Get(i));c.Start(Seconds(0.5));c.Stop(Seconds(50));}

  auto hoCheck=[&](double t){Simulator::Schedule(Seconds(t),[=](){for(auto u:{u1,u2}){float cur=u->m_map.count(u->m_cur)?u->m_map[u->m_cur]:999.0f;if(cur<u->m_thr){float best=cur;std::string bestSat;for(auto&p:u->m_map)if(p.first!=u->m_cur&&p.second>best){best=p.second;bestSat=p.first;}if(!bestSat.empty()){NS_LOG_UNCOND("  ↳ HANDOVER "<<u->m_cur<<"("<<(int)cur<<"M)→"<<bestSat<<"("<<(int)best<<"M)");u->m_cur=bestSat;u->m_ho++;}}}});};

  Simulator::Schedule(Seconds(20),[&](){NS_LOG_UNCOND("\n═══ t=20s ISL SAT-A↔B 600→150M ═══");setL(dAB,150);rA->AddNb("SAT-B",ip(3),150);rB->AddNb("SAT-A",ip(2),150);});
  // User-1 switches at t=25: update SetNbUsed
  Simulator::Schedule(Seconds(25),[&](){NS_LOG_UNCOND("\n═══ t=25s User-1 handover → SAT-A→B load 200→100M ═══");rA->SetNbUsed("SAT-B",100);rB->AddDirectGs("GS",600,100);});
  hoCheck(22);hoCheck(26);hoCheck(30);hoCheck(38);

  Simulator::Schedule(Seconds(48),[&](){NS_LOG_UNCOND("\n═══ Exp 3 Report ═══");for(int i=0;i<2;i++){double mb=sinks[i]->GetTotalRx()/1e6;Ptr<UsrApp> u=(i==0?u1:u2);NS_LOG_UNCOND("  User-"<<(i+1)<<": "<<u->m_map.size()<<" sats "<<u->m_ho<<" ho | data:"<<mb<<"MB | cur="<<u->m_cur);}NS_LOG_UNCOND("  SAT-A B2="<<rA->GetBestE2e()<<"M (local="<<rA->GetLocalBw()<<"M)");});

  NS_LOG_UNCOND("\n═══ Exp 3: User Handover → B2 Change ═══");
  Simulator::Stop(Seconds(50));Simulator::Run();Simulator::Destroy();return 0;
}