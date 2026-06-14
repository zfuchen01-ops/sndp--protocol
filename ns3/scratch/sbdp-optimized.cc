/* SBDP Optimized — Heartbeat + Seq detection + Threshold, real UDP ADVs */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/sbdp-header.h"
#include <map>
#include <vector>
#include <set>
using namespace ns3;NS_LOG_COMPONENT_DEFINE("SbdpOpt");

class SatFwd:public Application{public:static TypeId GetTypeId();SatFwd();void SetBw(double b){m_bw=b;}void AddUser(Ipv4Address ua){m_users.push_back(ua);}void Push();void StartHb(){m_hb=Simulator::Schedule(Seconds(m_hbInt),&SatFwd::OnHb,this);m_lastPush=Simulator::Now().GetSeconds();}
private:virtual void StartApplication()override{m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),9999));StartHb();}virtual void StopApplication()override{Simulator::Cancel(m_hb);if(m_sk)m_sk->Close();}void OnHb(){double now=Simulator::Now().GetSeconds();bool hbExp=(now-m_lastPush)>=m_hbInt;bool sigChg=std::abs(m_bw-m_lastBw)/std::max(1.0,m_lastBw)>0.05;if(sigChg||hbExp||m_lastBw<0){Push();NS_LOG_UNCOND("  [SAT "<<Names::FindName(GetNode())<<(sigChg?" PUSH":" HEARTBEAT")<<" seq="<<(m_seq-1)<<" bw="<<(int)m_bw<<"M");}m_hb=Simulator::Schedule(Seconds(m_hbInt),&SatFwd::OnHb,this);}Ptr<Socket> m_sk;double m_bw=0,m_lastBw=-1,m_lastPush=0,m_hbInt=30;uint16_t m_seq=0;EventId m_hb;std::vector<Ipv4Address> m_users;};
NS_OBJECT_ENSURE_REGISTERED(SatFwd);TypeId SatFwd::GetTypeId(){static TypeId tid=TypeId("ns3::SatFwd").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<SatFwd>();return tid;}SatFwd::SatFwd(){}
void SatFwd::Push(){m_seq++;for(auto&ua:m_users){SbdpHeader hdr=SbdpHeader::BuildAdv(Names::FindName(GetNode()),"UE",m_bw,Names::FindName(GetNode()),m_seq);Ptr<Packet> pkt=Create<Packet>(0);pkt->AddHeader(hdr);m_sk->SendTo(pkt,0,InetSocketAddress(ua,8888));}m_lastBw=m_bw;m_lastPush=Simulator::Now().GetSeconds();}

class UsrApp:public Application{public:static TypeId GetTypeId();UsrApp();struct E{double t;float bw;uint16_t seq;};std::vector<E> m_h;uint32_t m_gaps=0,m_missed=0;bool m_stale=false;private:virtual void StartApplication()override{m_sk=Socket::CreateSocket(GetNode(),UdpSocketFactory::GetTypeId());m_sk->Bind(InetSocketAddress(Ipv4Address::GetAny(),8888));m_sk->SetRecvCallback(MakeCallback(&UsrApp::Recv,this));m_staleCheck=Simulator::Schedule(Seconds(10),&UsrApp::CheckStale,this);}virtual void StopApplication()override{Simulator::Cancel(m_staleCheck);if(m_sk)m_sk->Close();}void Recv(Ptr<Socket> s){Ptr<Packet> pkt;Address from;while((pkt=s->RecvFrom(from))){SbdpHeader hdr;if(pkt->GetSize()>=SbdpHeader::SBDP_FIXED_SIZE){pkt->RemoveHeader(hdr);double now=Simulator::Now().GetSeconds();if(m_lastSeq>0&&hdr.GetSeqNum()!=m_lastSeq+1){m_gaps++;m_missed+=hdr.GetSeqNum()>m_lastSeq?hdr.GetSeqNum()-m_lastSeq-1:0;NS_LOG_UNCOND("  ["<<Names::FindName(GetNode())<<"] ★GAP seq="<<hdr.GetSeqNum()<<" expected="<<(m_lastSeq+1));}m_lastSeq=hdr.GetSeqNum();m_lastRecv=now;m_stale=false;m_h.push_back({now,hdr.GetBackhaulBw(),hdr.GetSeqNum()});NS_LOG_UNCOND("  ["<<Names::FindName(GetNode())<<" @"<<(int)now<<"s] bw="<<(int)hdr.GetBackhaulBw()<<"M seq="<<hdr.GetSeqNum());}}}void CheckStale(){double now=Simulator::Now().GetSeconds();bool was=m_stale;m_stale=(now-m_lastRecv)>35;if(m_stale&&!was)NS_LOG_UNCOND("  ⚠ ["<<Names::FindName(GetNode())<<" @"<<(int)now<<"s] STALE");if(!m_stale&&was)NS_LOG_UNCOND("  ✓ ["<<Names::FindName(GetNode())<<"] RECOVERED");m_staleCheck=Simulator::Schedule(Seconds(10),&UsrApp::CheckStale,this);}Ptr<Socket> m_sk;uint16_t m_lastSeq=0;double m_lastRecv=0;EventId m_staleCheck;};
NS_OBJECT_ENSURE_REGISTERED(UsrApp);TypeId UsrApp::GetTypeId(){static TypeId tid=TypeId("ns3::UsrApp").SetParent<Application>().SetGroupName("Sbdp").AddConstructor<UsrApp>();return tid;}UsrApp::UsrApp(){}

struct Edge{int a,b,bw;};static double maxMinBw(const std::vector<Edge>&e,int src,int dst){std::map<int,std::map<int,int>>g;for(auto&x:e){g[x.a][x.b]=x.bw;g[x.b][x.a]=x.bw;}std::map<int,double>bw;for(auto&p:g)bw[p.first]=0;bw[src]=1e9;std::set<int>vis;while(true){int best=-1;double bv=-1;for(auto&p:bw){if(vis.count(p.first))continue;if(p.second>bv){bv=p.second;best=p.first;}}if(best<0||best==dst)return(best==dst)?bv:0;vis.insert(best);for(auto&nb:g[best]){double pb=std::min(bv,(double)nb.second);if(pb>bw[nb.first])bw[nb.first]=pb;}}}

int main(int argc,char*argv[]){
  NodeContainer all;all.Create(11);for(int i=0;i<6;i++)Names::Add("SAT-"+std::string(1,'A'+i),all.Get(i));Names::Add("GS-East",all.Get(6));Names::Add("GS-West",all.Get(7));Names::Add("User-1",all.Get(8));Names::Add("User-2",all.Get(9));Names::Add("User-3",all.Get(10));
  InternetStackHelper inet;inet.Install(all);PointToPointHelper p2p;p2p.SetDeviceAttribute("DataRate",StringValue("1Gbps"));p2p.SetChannelAttribute("Delay",StringValue("5ms"));Ipv4AddressHelper ipv4;ipv4.SetBase("10.0.0.0","255.255.255.0");for(int i=0;i<10;i++){p2p.Install(all.Get(i),all.Get(i+1));ipv4.Assign(p2p.Install(all.Get(i),all.Get(i+1)));ipv4.NewNetwork();}Ipv4GlobalRoutingHelper::PopulateRoutingTables();
  auto lu=[&](int i)->Ipv4Address{return all.Get(i)->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();};

  std::vector<Ptr<SatFwd>>fwds(6);for(int i=0;i<6;i++){fwds[i]=CreateObject<SatFwd>();all.Get(i)->AddApplication(fwds[i]);fwds[i]->SetStartTime(Seconds(0.1));}
  std::vector<Ptr<UsrApp>>users(3);for(int i=0;i<3;i++){users[i]=CreateObject<UsrApp>();all.Get(8+i)->AddApplication(users[i]);users[i]->SetStartTime(Seconds(0.1));}
  fwds[0]->AddUser(lu(8));fwds[3]->AddUser(lu(9));fwds[5]->AddUser(lu(10));

  std::vector<Edge>edges;auto setE=[&](auto&e){edges=e;};auto bh=[&](int sat){double b1=maxMinBw(edges,sat,7);double b2=maxMinBw(edges,sat,8);return std::max(b1,b2);};
  auto ev=[&](double t,std::vector<Edge>e,const char*desc){Simulator::Schedule(Seconds(t),[=](){setE(e);NS_LOG_UNCOND("\n═══ t="<<(int)t<<"s "<<desc<<" ═══");for(int i=0;i<6;i++)fwds[i]->SetBw(bh(i+1));});};

  ev(2,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"INITIAL");
  ev(200,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,0},{2,5,0},{3,6,0},{1,7,350},{6,8,0},{4,7,250},{3,8,250}},"MAJOR FAULT");
  ev(350,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVERY");
  ev(450,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,80},{6,8,300},{4,7,250},{3,8,250}},"GS-East rain");
  ev(550,{{1,2,200},{2,3,200},{4,5,200},{5,6,200},{1,4,180},{2,5,180},{3,6,180},{1,7,350},{6,8,300},{4,7,250},{3,8,250}},"RECOVER");

  p2p.EnablePcap("sbdp-opt",all.Get(0)->GetDevice(0),true);

  Simulator::Schedule(Seconds(600),[&](){NS_LOG_UNCOND("\n\n╔══════════════════════════════════════════════╗");NS_LOG_UNCOND("║  Optimized Report (real UDP, heartbeat 30s)   ║");NS_LOG_UNCOND("╚══════════════════════════════════════════════╝");for(int i=0;i<3;i++){NS_LOG_UNCOND("  User-"<<(i+1)<<": "<<users[i]->m_h.size()<<" updates gaps="<<users[i]->m_gaps<<" missed="<<users[i]->m_missed<<" stale="<<(users[i]->m_stale?"YES":"no"));}});
  Simulator::Stop(Seconds(601));Simulator::Run();Simulator::Destroy();return 0;
}
