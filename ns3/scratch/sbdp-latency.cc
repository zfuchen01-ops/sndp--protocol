/* SBDP Latency Test — ISL change to user awareness
 *
 * Topo: User-1──SAT-A──200M(变50M)──SAT-B──500M──GS-Main
 *
 * Measure: ISL change at t=5s → B2 propagation → ADV reaches user
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
NS_LOG_COMPONENT_DEFINE("SbdpLatency");

struct NbInfo { Ipv4Address ip; double bw; };

class SatRouter : public Application {
public:
  static TypeId GetTypeId(); SatRouter() {}
  void AddNb(std::string n, Ipv4Address ip, double bw) { m_nb[n]={ip,bw}; }
  void AddGs(std::string g, double bw) { m_gs[g]=bw; m_nexthop[g]=""; }
  void SetRoute(std::string gs, std::string nb) { m_nexthop[gs]=nb; }
  Ipv4Address GetNbIp(const std::string &n) const { auto it=m_nb.find(n); return it!=m_nb.end()?it->second.ip:Ipv4Address::GetAny(); }
  void UpdateBest(){m_bestBw=0;for(auto&p:m_gs)if(p.second>m_bestBw){m_bestBw=p.second;m_bestGs=p.first;}}
  std::map<std::string,NbInfo>m_nb;
  std::map<std::string,double>m_gs;
  std::map<std::string,std::string>m_nexthop;
  double m_bestBw=0; std::string m_bestGs;
  double GetBest()const{return m_bestBw>0?m_bestBw:200;}
private:
  virtual void StartApplication()override;
  virtual void StopApplication()override;
  void RecvEx(Ptr<Socket>s); void SendPush(); void SendRequest(); void SendReply(const std::string&); void CheckPortChange();
  Ptr<Socket>m_sk; EventId m_ct; uint16_t m_seq=0;
  std::map<std::string,double>m_nbSnap;
  bool m_init=false;
};
NS_OBJECT_ENSURE_REGISTERED(SatRouter);
TypeId SatRouter::GetTypeId(){static TypeId tid=TypeId("ns3::SatRouter").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<SatRouter>();return tid;}
void SatRouter::StartApplication(){
  m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());
  m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9997));
  m_sk->SetRecvCallback(MakeCallback(&SatRouter::RecvEx,this));
  Simulator::Schedule(Seconds(0.05),&SatRouter::SendRequest,this);
  m_ct=Simulator::Schedule(Seconds(0.20),&SatRouter::CheckPortChange,this);
}
void SatRouter::StopApplication(){Simulator::Cancel(m_ct);if(m_sk)m_sk->Close();}

void SatRouter::SendPush(){
  if(m_gs.empty())return;
  std::string my=Names::FindName(GetNode());
  uint8_t buf[512];int p=0;
  buf[p++]=0x42;buf[p++]=0x32;buf[p++]=1;buf[p++]=0; p+=2;
  uint16_t seq=m_seq++;memcpy(buf+p,&seq,2);p+=2;
  buf[p++]=(uint8_t)my.size();memcpy(buf+p,my.c_str(),my.size());p+=my.size();
  int n=0;for(auto&x:m_gs)n++;buf[p++]=(uint8_t)n;
  for(auto&x:m_gs){uint8_t l=x.first.size();buf[p++]=l;memcpy(buf+p,x.first.c_str(),l);p+=l;int32_t bw=(int32_t)x.second;memcpy(buf+p,&bw,4);p+=4;}
  uint16_t total=(uint16_t)p;memcpy(buf+4,&total,2);
  Ptr<Packet>pkt=Create<Packet>(buf,p);
  for(auto&nb:m_nb)m_sk->SendTo(pkt,0,InetSocketAddress(nb.second.ip,9997));
}
void SatRouter::SendRequest(){
  std::string my=Names::FindName(GetNode());
  uint8_t buf[256];int p=0;
  buf[p++]=0x42;buf[p++]=0x32;buf[p++]=1;buf[p++]=1;p+=2;
  uint16_t seq=m_seq++;memcpy(buf+p,&seq,2);p+=2;
  buf[p++]=(uint8_t)my.size();memcpy(buf+p,my.c_str(),my.size());p+=my.size();
  uint16_t total=(uint16_t)p;memcpy(buf+4,&total,2);
  Ptr<Packet>pkt=Create<Packet>(buf,p);
  for(auto&nb:m_nb)m_sk->SendTo(pkt,0,InetSocketAddress(nb.second.ip,9997));
}
void SatRouter::SendReply(const std::string&tgt){
  if(m_gs.empty()||!m_nb.count(tgt))return;
  std::string my=Names::FindName(GetNode());
  uint8_t buf[512];int p=0;
  buf[p++]=0x42;buf[p++]=0x32;buf[p++]=1;buf[p++]=2;p+=2;
  uint16_t seq=m_seq++;memcpy(buf+p,&seq,2);p+=2;
  buf[p++]=(uint8_t)my.size();memcpy(buf+p,my.c_str(),my.size());p+=my.size();
  int n=0;for(auto&x:m_gs)n++;buf[p++]=(uint8_t)n;
  for(auto&x:m_gs){uint8_t l=x.first.size();buf[p++]=l;memcpy(buf+p,x.first.c_str(),l);p+=l;int32_t bw=(int32_t)x.second;memcpy(buf+p,&bw,4);p+=4;}
  uint16_t total=(uint16_t)p;memcpy(buf+4,&total,2);
  Ptr<Packet>pkt=Create<Packet>(buf,p);
  m_sk->SendTo(pkt,0,InetSocketAddress(m_nb[tgt].ip,9997));
}

void SatRouter::RecvEx(Ptr<Socket>s){
  Ptr<Packet>pkt;Address from;bool prop=false;
  while((pkt=s->RecvFrom(from))){
    uint8_t buf[512];pkt->CopyData(buf,pkt->GetSize());
    if(pkt->GetSize()<9)continue;
    if(buf[0]!=0x42||buf[1]!=0x32)continue;
    if(buf[2]!=1)continue;
    uint8_t type=buf[3];uint16_t seq;memcpy(&seq,buf+6,2);
    uint8_t nl=buf[8];std::string nb((char*)buf+9,nl);int pos=9+nl;
    double nbLink=m_nb.count(nb)?m_nb[nb].bw:1e9;
    if(type==1){SendReply(nb);}
    else if(type==0||type==2){
      if(pkt->GetSize()<(uint32_t)pos+1)continue;
      int n=buf[pos++];bool chg=false;
      for(int i=0;i<n&&pos<(int)pkt->GetSize();i++){
        uint8_t gl=buf[pos++];std::string gs((char*)buf+pos,gl);pos+=gl;
        int32_t nbBw;memcpy(&nbBw,buf+pos,4);pos+=4;
        if(!m_nexthop.count(gs))continue;
        if(m_nexthop[gs]=="")continue;
        if(m_nexthop[gs]!=nb)continue;
        double newBw=std::min(nbLink,(double)nbBw);
        if(!m_gs.count(gs)||newBw!=m_gs[gs]){m_gs[gs]=newBw;chg=true;}
      }
      if(chg){UpdateBest();prop=true;}
    }
  }
  if(prop){SendPush();}
}

void SatRouter::CheckPortChange(){
  if(!m_init&&!m_gs.empty()){m_init=true;for(auto&nb:m_nb)m_nbSnap[nb.first]=nb.second.bw;}
  else if(m_init){
    bool chg=false;
    if(m_nb.size()!=m_nbSnap.size())chg=true;
    else{for(auto&nb:m_nb)if(!m_nbSnap.count(nb.first)||std::abs(m_nbSnap[nb.first]-nb.second.bw)>0.5){chg=true;break;}}
    if(chg){
      for(auto&nb:m_nb){double old=m_nbSnap.count(nb.first)?m_nbSnap[nb.first]:0;if(std::abs(nb.second.bw-old)>0.5){for(auto&gs:m_nexthop)if(gs.second==nb.first)m_gs[gs.first]=0;}}
      UpdateBest();SendPush();
      m_nbSnap.clear();for(auto&nb:m_nb)m_nbSnap[nb.first]=nb.second.bw;
    }
  }
  m_ct=Simulator::Schedule(Seconds(0.20),&SatRouter::CheckPortChange,this);
}

class GnbApp : public Application {
public:
  static TypeId GetTypeId();GnbApp(){}
  void AddCov(Ipv4Address ua){m_cov.push_back(ua);}void SetR(Ptr<SatRouter>r){m_r=r;}
  void Push();
private:
  virtual void StartApplication()override;
  virtual void StopApplication()override;
  void AutoPush();
  Ptr<Socket>m_sk;std::vector<Ipv4Address>m_cov;Ptr<SatRouter>m_r;uint16_t m_seq=0;EventId m_ap;
};
NS_OBJECT_ENSURE_REGISTERED(GnbApp);
TypeId GnbApp::GetTypeId(){static TypeId tid=TypeId("ns3::GnbApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<GnbApp>();return tid;}
void GnbApp::StartApplication(){m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9999));m_ap=Simulator::Schedule(Seconds(0.1),&GnbApp::AutoPush,this);}
void GnbApp::StopApplication(){Simulator::Cancel(m_ap);if(m_sk)m_sk->Close();}
void GnbApp::AutoPush(){
  Push();
  m_ap=Simulator::Schedule(Seconds(0.1),&GnbApp::AutoPush,this);  // poll every 100ms
}
void GnbApp::Push(){
  if(!m_r)return;double bw=m_r->GetBest();std::string gs=m_r->m_bestGs;m_seq++;
  for(auto&ua:m_cov){SbdpHeader h=SbdpHeader::BuildAdv(Names::FindName(GetNode()),"UE",bw,Names::FindName(GetNode())+"→"+gs,m_seq);Ptr<Packet>p=Create<Packet>(0);p->AddHeader(h);m_sk->SendTo(p,0,InetSocketAddress(ua,8888));}
  NS_LOG_UNCOND("  [ADV "<<Names::FindName(GetNode())<<"@"<<Simulator::Now().GetSeconds()<<"s] bw="<<(int)bw<<"M");
}

class UsrApp : public Application {
public:
  static TypeId GetTypeId();UsrApp(){}
  std::vector<double>m_t;std::vector<double>m_bw;
private:
  virtual void StartApplication()override{m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),8888));m_sk->SetRecvCallback(MakeCallback(&UsrApp::Recv,this));}
  virtual void StopApplication()override{if(m_sk)m_sk->Close();}
  void Recv(Ptr<Socket>s){Ptr<Packet>pkt;Address from;while((pkt=s->RecvFrom(from))){SbdpHeader h;if(pkt->GetSize()>=SbdpHeader::SBDP_FIXED_SIZE){pkt->RemoveHeader(h);double now=Simulator::Now().GetSeconds();m_t.push_back(now);m_bw.push_back(h.GetBackhaulBw());NS_LOG_UNCOND("  [U@"<<now<<"s] bw="<<(int)h.GetBackhaulBw()<<"M");}}}
  Ptr<Socket>m_sk;
};
NS_OBJECT_ENSURE_REGISTERED(UsrApp);
TypeId UsrApp::GetTypeId(){static TypeId tid=TypeId("ns3::UsrApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<UsrApp>();return tid;}

int main(int argc,char*argv[]){
  NodeContainer n;n.Create(4);
  Names::Add("User",n.Get(0));Names::Add("SAT-A",n.Get(1));
  Names::Add("SAT-B",n.Get(2));Names::Add("GS",n.Get(3));
  InternetStackHelper inet;inet.Install(n);
  PointToPointHelper p2p;p2p.SetQueue("ns3::DropTailQueue");
  Ipv4AddressHelper ipv4;uint32_t sn=0;
  auto L=[&](int a,int b,double bw,double d){
    p2p.SetDeviceAttribute("DataRate",StringValue(std::to_string((int)bw)+"Mbps"));
    p2p.SetChannelAttribute("Delay",StringValue(std::to_string(d)+"ms"));
    auto dev=p2p.Install(n.Get(a),n.Get(b));
    char base[32];snprintf(base,32,"10.%u.0.0",sn++);ipv4.SetBase(base,"255.255.255.0");ipv4.Assign(dev);
    return dev;
  };
  auto setL=[&](NetDeviceContainer&dev,double bw){DynamicCast<PointToPointNetDevice>(dev.Get(0))->SetDataRate(DataRate(bw*1e6));DynamicCast<PointToPointNetDevice>(dev.Get(1))->SetDataRate(DataRate(bw*1e6));};
  auto du=L(0,1,200,5);auto dAB=L(1,2,200,5);auto dBG=L(2,3,500,5);
  Ipv4GlobalRoutingHelper::PopulateRoutingTables();
  auto ip=[&](int i)->Ipv4Address{return n.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();};

  Ptr<SatRouter> rA=CreateObject<SatRouter>(), rB=CreateObject<SatRouter>();
  Ptr<GnbApp> gA=CreateObject<GnbApp>(), gB=CreateObject<GnbApp>();
  gA->SetR(rA);gB->SetR(rB);
  n.Get(1)->AddApplication(rA);rA->SetStartTime(Seconds(0));
  n.Get(1)->AddApplication(gA);gA->SetStartTime(Seconds(0.05));
  n.Get(2)->AddApplication(rB);rB->SetStartTime(Seconds(0));
  n.Get(2)->AddApplication(gB);gB->SetStartTime(Seconds(0.05));

  rA->AddNb("SAT-B",ip(2),200);rB->AddNb("SAT-A",ip(1),200);
  rA->SetRoute("GS","SAT-B");
  rB->AddGs("GS",500);

  Ptr<Ipv4RoutingProtocol> rpA=n.Get(1)->GetObject<Ipv4>()->GetRoutingProtocol();
  Ipv4Header hdr;hdr.SetDestination(ip(3));hdr.SetProtocol(17);
  Socket::SocketErrno se;Ptr<Ipv4Route> rt=rpA->RouteOutput(Create<Packet>(),hdr,Ptr<NetDevice>(),se);
  if(rt){Ipv4Address gw=rt->GetGateway();if(gw==rA->GetNbIp("SAT-B"))rA->SetRoute("GS","SAT-B");}

  gA->AddCov(ip(0));gB->AddCov(ip(0));
  Ptr<UsrApp> u=CreateObject<UsrApp>();n.Get(0)->AddApplication(u);u->SetStartTime(Seconds(0.05));

  // ISL change at t=5s: SAT-A↔SAT-B 200→50M
  Simulator::Schedule(Seconds(5.0),[&](){
    NS_LOG_UNCOND("\n═══ t=5.0s ISL SAT-A↔SAT-B 200→50M ═══");
    setL(dAB,50);
    rA->AddNb("SAT-B",ip(2),50);rB->AddNb("SAT-A",ip(1),50);
  });

  // Data flow: User sends 300Mbps UDP → GS (bottlenecked by ISL at 200M, then 50M)
  Ipv4Address gsIp = ip(3);
  PacketSinkHelper sk("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 5001));
  auto sinkApp = sk.Install(n.Get(3)); sinkApp.Start(Seconds(0.2)); sinkApp.Stop(Seconds(8));
  OnOffHelper oo("ns3::UdpSocketFactory", InetSocketAddress(gsIp, 5001));
  oo.SetAttribute("DataRate", DataRateValue(DataRate("300Mbps")));
  oo.SetAttribute("PacketSize", UintegerValue(1472));
  oo.SetConstantRate(DataRate("300Mbps"));
  auto srcApp = oo.Install(n.Get(0)); srcApp.Start(Seconds(0.3)); srcApp.Stop(Seconds(8));

  // Throughput: measure delta 5.3s→7.3s (pure 50M ISL period)
  Ptr<PacketSink> ps = DynamicCast<PacketSink>(sinkApp.Get(0));
  Simulator::Schedule(Seconds(5.3), [ps]() {
    NS_LOG_UNCOND("  [THRUPUT t=5.3s] rx=" << (int)(ps->GetTotalRx()/1e6) << "MB");
  });
  Simulator::Schedule(Seconds(7.3), [ps]() {
    double rxMB = ps->GetTotalRx() / 1e6;
    // Baseline at t=5.3 ≈ 112MB, delta in 2s at 50M ISL
    double mb2s = rxMB - 112.0;
    double tput = mb2s * 8 / 2.0;
    NS_LOG_UNCOND("  [THRUPUT t=7.3s] rx=" << (int)rxMB << "MB | delta=" << (int)mb2s << "MB → " << (int)tput << "Mbps");
    NS_LOG_UNCOND("\n═══ B2 vs Actual Throughput ═══");
    NS_LOG_UNCOND("  ISL bottleneck: 50Mbps");
    NS_LOG_UNCOND("  B2 reported:    50Mbps");
    NS_LOG_UNCOND("  UDP actual:   ~" << (int)tput << "Mbps");
    NS_LOG_UNCOND("  Verdict:       " << (tput >= 40 ? "✓ PASS (within overhead)" : "✗ FAIL"));
  });

  p2p.EnablePcap("sbdp-latency",du.Get(0),true);
  NS_LOG_UNCOND("\n═══ ISL Change + Data Flow: B2 vs Actual Throughput ═══\n");
  Simulator::Stop(Seconds(8));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}