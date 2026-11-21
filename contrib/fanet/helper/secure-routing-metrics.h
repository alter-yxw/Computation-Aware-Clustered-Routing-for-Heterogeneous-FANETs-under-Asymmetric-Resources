/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef FANET_SECURE_ROUTING_METRICS_H
#define FANET_SECURE_ROUTING_METRICS_H

#include "ns3/abort.h"
#include "ns3/basic-energy-source.h"
#include "ns3/energy-source-container.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/llc-snap-header.h"
#include "ns3/mobility-model.h"
#include "ns3/net-device-container.h"
#include "ns3/node-container.h"
#include "ns3/seq-ts-header.h"
#include "ns3/tag.h"
#include "ns3/udp-client.h"
#include "ns3/udp-header.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy-state-helper.h"
#include "ns3/txop.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <iomanip>
#include <numeric>
#include <tuple>
#include <vector>

namespace ns3 { namespace secure_benchmark {

// Evaluation-only tags. Protocols neither inspect these tags nor receive truth
// labels. PacketTags consume no on-wire bytes and do not change UDP sequences.
class SecureAttemptTag : public Tag
{
public:
  uint32_t flow{0}, attempt{0};
  static TypeId GetTypeId() { static TypeId t=TypeId("ns3::SecureAttemptTag").SetParent<Tag>().AddConstructor<SecureAttemptTag>(); return t; }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 8; }
  void Serialize(TagBuffer b) const override { b.WriteU32(flow); b.WriteU32(attempt); }
  void Deserialize(TagBuffer b) override { flow=b.ReadU32(); attempt=b.ReadU32(); }
  void Print(std::ostream& o) const override { o << flow << ':' << attempt; }
};
class SecureSourceTtlTag : public Tag
{
public:
  uint8_t ttl{0};
  static TypeId GetTypeId() { static TypeId t=TypeId("ns3::SecureSourceTtlTag").SetParent<Tag>().AddConstructor<SecureSourceTtlTag>(); return t; }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 1; }
  void Serialize(TagBuffer b) const override { b.WriteU8(ttl); }
  void Deserialize(TagBuffer b) override { ttl=b.ReadU8(); }
  void Print(std::ostream& o) const override { o << unsigned(ttl); }
};
class SecureDeliveredHopsTag : public Tag
{
public:
  uint16_t hops{0};
  static TypeId GetTypeId() { static TypeId t=TypeId("ns3::SecureDeliveredHopsTag").SetParent<Tag>().AddConstructor<SecureDeliveredHopsTag>(); return t; }
  TypeId GetInstanceTypeId() const override { return GetTypeId(); }
  uint32_t GetSerializedSize() const override { return 2; }
  void Serialize(TagBuffer b) const override { b.WriteU16(hops); }
  void Deserialize(TagBuffer b) override { hops=b.ReadU16(); }
  void Print(std::ostream& o) const override { o << hops; }
};

class SecureRoutingMetrics
{
public:
  using Pair=std::pair<uint32_t,uint32_t>;
  using Attempt=std::pair<uint32_t,uint32_t>;
  struct Flow
  {
    Ptr<UdpClient> client;
    uint32_t source{0}, destination{0};
    Ipv4Address sourceIp, destinationIp;
    uint16_t port{0};
    uint64_t attempts{0}, submitted{0}, received{0}, duplicates{0}, invalid{0};
    uint64_t postAttempts{0}, postUnique{0}, uniqueBytes{0}, hopsSum{0}, hopsSamples{0};
    std::set<uint32_t> sequences;
    std::map<uint32_t,Time> attemptTimes;
    std::vector<double> delaysMs;
  };
  struct PairState
  {
    bool registered{false}, evaluated{false}, flagged{false}, everFlagged{false}, exposed{false};
    bool refused{false}, reopened{false}, resumed{false};
    uint64_t evaluations{0}, refusals{0}, opportunities{0}, drops{0}, handoffRequests{0};
    double firstOpportunity{-1}, firstDrop{-1}, firstFlag{-1}, firstRefusal{-1}, firstReopen{-1};
    double lastBlockS{-1}, firstReopenDelayS{-1};
  };
  struct Cost { uint64_t packets{0}, mpduBytes{0}, udpPayloadBytes{0}; double airtimeS{0}; };
  std::vector<Flow> flows;
  std::map<Pair,PairState> pairs; // Population is ALL ordered N*(N-1) pairs,
                               // including implicit unobserved negative pairs.
  std::vector<bool> dropTruth;
  uint32_t nodes{0};
  double trafficStart{15}, stop{240}, attackStart{30}, range{500};
  std::string algorithm;
  bool detectionAvailable{false}, restrictionAvailable{false}, sbcrRisk{false};
  bool relayTruthAvailable{true};
  Cost total, control, security, sharedSecurity;
  uint64_t unknownControl{0}, ambiguousPpdus{0}, invalidTrace{0};
  uint64_t relayOpportunities{0}, maliciousOpportunities{0}, attackDrops{0}, forwardSubmissions{0};
  uint64_t nonBusinessHooks{0}, queueExpired{0}, queueOverflow{0};
  std::set<Attempt> maliciousPackets, droppedPackets, deliveredPackets;
  double consumedJ{0};
  uint64_t completedRangeLinks{0}, leftCensoredLinks{0}, rightCensoredLinks{0};
  double completedRangeLinkSeconds{0};

  static std::string Number(double x)
  {
    if (!std::isfinite(x)) return "N/A";
    std::ostringstream s; s << std::setprecision(17) << x; return s.str();
  }
  static double Ratio(double a, double b, double scale=1) { return b>0 ? scale*a/b : NAN; }
  static double Percentile(std::vector<double> values, double p)
  {
    if (values.empty()) return NAN;
    std::sort(values.begin(), values.end());
    return values[static_cast<size_t>(std::ceil(p*values.size()))-1];
  }
  static void AttemptTx(SecureRoutingMetrics* self, uint32_t f, Ptr<const Packet> packet)
  {
    auto& flow=self->flows.at(f);
    SecureAttemptTag tag; tag.flow=f; tag.attempt=flow.attempts++;
    flow.attemptTimes[tag.attempt]=Simulator::Now();
    packet->AddPacketTag(tag);
    if (Simulator::Now().GetSeconds()>=self->attackStart) ++flow.postAttempts;
  }
  static void Outgoing(SecureRoutingMetrics* self, uint32_t node, const Ipv4Header& ip,
                       Ptr<const Packet> packet, uint32_t interface)
  {
    // DSCR/QSR defer source routing through loopback. That virtual traversal
    // consumes IPv4 TTL but is not a wireless path hop. Capture source TTL
    // only at the first real output, using SendOutgoing OR UnicastForward.
    if (interface==0) return;
    uint32_t controlBytes=0;
    self->ClassifyIp(ip,packet,controlBytes);
    SecureAttemptTag attempt;
    if (!packet->PeekPacketTag(attempt) || attempt.flow>=self->flows.size()) return;
    const auto& flow=self->flows[attempt.flow];
    // Source TTL is captured from the real outgoing IP header, not assumed 64.
    if (node!=flow.source || ip.GetSource()!=flow.sourceIp || ip.GetDestination()!=flow.destinationIp || ip.GetProtocol()!=17) return;
    SecureSourceTtlTag existing;
    if (packet->PeekPacketTag(existing)) return;
    SecureSourceTtlTag tag; tag.ttl=ip.GetTtl(); packet->AddPacketTag(tag);
  }
  static void LocalDeliver(SecureRoutingMetrics* self, uint32_t node, const Ipv4Header& ip,
                           Ptr<const Packet> packet, uint32_t)
  {
    SecureAttemptTag attempt; SecureSourceTtlTag source;
    if (!packet->PeekPacketTag(attempt) || attempt.flow>=self->flows.size()) return;
    const auto& flow=self->flows[attempt.flow];
    if (node!=flow.destination || ip.GetSource()!=flow.sourceIp || ip.GetDestination()!=flow.destinationIp || ip.GetProtocol()!=17) return;
    if (!packet->PeekPacketTag(source) || source.ttl<ip.GetTtl()) { ++self->invalidTrace; return; }
    // A direct source->destination delivery is ONE hop; loops consume TTL and
    // are retained. Only a subsequent accepted UDP Rx contributes to AvgHops.
    SecureDeliveredHopsTag tag; tag.hops=uint16_t(source.ttl)-ip.GetTtl()+1;
    packet->AddPacketTag(tag);
  }
  static void Receive(SecureRoutingMetrics* self, uint32_t f, Ptr<const Packet> packet)
  {
    auto& flow=self->flows.at(f); ++flow.received;
    SecureAttemptTag tag; SeqTsHeader header;
    if (packet->GetSize()!=100 || !packet->PeekPacketTag(tag) || tag.flow!=f || !flow.attemptTimes.count(tag.attempt)) { ++flow.invalid; return; }
    packet->PeekHeader(header);
    if (header.GetTs()!=flow.attemptTimes.at(tag.attempt) || header.GetTs()>Simulator::Now()) { ++flow.invalid; return; }
    if (!flow.sequences.insert(header.GetSeq()).second) { ++flow.duplicates; return; }
    flow.uniqueBytes+=packet->GetSize();
    flow.delaysMs.push_back((Simulator::Now()-header.GetTs()).GetSeconds()*1000);
    if (header.GetTs().GetSeconds()>=self->attackStart) ++flow.postUnique;
    SecureDeliveredHopsTag hops;
    if (packet->PeekPacketTag(hops) && hops.hops>0) { flow.hopsSum+=hops.hops; ++flow.hopsSamples; }
    else ++self->invalidTrace;
    self->deliveredPackets.insert({f,tag.attempt});
  }
  static void PacketEvent(SecureRoutingMetrics* self, uint32_t emitter, uint8_t event,
                          uint32_t target, Ptr<const Packet> packet)
  {
    SecureAttemptTag tag;
    if (!packet->PeekPacketTag(tag)) { ++self->nonBusinessHooks; return; }
    if (tag.flow>=self->flows.size() || emitter>=self->nodes) { ++self->invalidTrace; return; }
    const Attempt id{tag.flow,tag.attempt};
    // A last-hop destination has no forwarding obligation.
    const auto& flow=self->flows[tag.flow];
    if (event==2 && target<self->nodes && target!=emitter && target!=flow.destination)
    {
      self->m_upstream[{id,target}]=emitter;
      if (Simulator::Now().GetSeconds()>=self->attackStart)
        ++self->pairs[{emitter,target}].handoffRequests;
    }
    if (Simulator::Now().GetSeconds()<self->attackStart) return;
    if (event==2) ++self->forwardSubmissions;
    if (event!=0 && event!=1 && event!=2) return;
    auto from=self->m_upstream.find({id,emitter});
    if (from==self->m_upstream.end()) return;
    auto& pair=self->pairs[{from->second,emitter}];
    if (event==2 && pair.reopened && !pair.flagged) pair.resumed=true; // forwarding SUBMISSION only
    if (event==0)
    {
      ++self->relayOpportunities; ++pair.opportunities;
      if (self->dropTruth.at(emitter))
      {
        ++self->maliciousOpportunities; self->maliciousPackets.insert(id); pair.exposed=true;
        if (pair.firstOpportunity<0) pair.firstOpportunity=Simulator::Now().GetSeconds();
      }
    }
    else if (event==1)
    {
      if (!self->dropTruth.at(emitter)) { ++self->invalidTrace; return; }
      ++self->attackDrops; ++pair.drops; self->droppedPackets.insert(id);
      if (pair.firstDrop<0) pair.firstDrop=Simulator::Now().GetSeconds();
    }
  }
  // SBCR uses event 0 for statistical F-role risk, not an identity classifier.
  // DSCR event 0 is its existing IsMalicious(observer,target) zero-R decision.
  static void SecurityEvent(SecureRoutingMetrics* self, uint32_t observer, uint32_t target,
                            uint8_t event, bool flagged)
  {
    if (observer>=self->nodes || target>=self->nodes || observer==target) return;
    auto& pair=self->pairs[{observer,target}]; const double now=Simulator::Now().GetSeconds();
    if (event==0 || event==2) pair.registered=true;
    if (event==0)
    {
      if (now>=self->attackStart && pair.flagged) pair.everFlagged=true;
      if (flagged && !pair.flagged) pair.lastBlockS=now;
      pair.evaluated=true; ++pair.evaluations; pair.flagged=flagged;
      if (flagged && pair.firstFlag<0) pair.firstFlag=now;
      if (now>=self->attackStart && flagged) pair.everFlagged=true;
    }
    if (now<self->attackStart) return;
    if (event==1 || event==3)
    {
      pair.refused=true; ++pair.refusals;
      if(pair.firstRefusal<0) pair.firstRefusal=now;
    }
    if (event==4) // SBCR reopening, separate from DSCR observation event 2
    {
      pair.reopened=true;
      if(pair.firstReopen<0)
      {
        pair.firstReopen=now;
        if(pair.lastBlockS>=0) pair.firstReopenDelayS=now-pair.lastBlockS;
      }
    }
  }
  static void SbcrSecurityEvent(SecureRoutingMetrics* self, uint32_t observer,
                                uint32_t target, uint8_t event, bool flagged)
  { SecurityEvent(self,observer,target,event==2 ? 4 : event,flagged); }

  // Real MAC data MPDUs, parsed at PhyTxBegin, including retransmissions. No
  // all-PHY-byte substitution. Dedicated DSCR reputation reports are kind 4.
  // QSR beacons serve discovery AND M1 security; their full cost is shared,
  // and cannot be claimed as the incremental cost of security alone.
  unsigned Classify(Ptr<const Packet> packet, uint32_t& udpBytes)
  {
    auto p=packet->Copy(); WifiMacHeader mac;
    if(p->GetSize()<10) return 0;
    p->RemoveHeader(mac);
    if(!mac.IsData() || p->GetSize()<8) return 0;
    LlcSnapHeader llc; p->RemoveHeader(llc);
    if(llc.GetType()!=0x0800 || p->GetSize()<20) return 0;
    Ipv4Header ip; p->RemoveHeader(ip);
    return ClassifyIp(ip,p,udpBytes);
  }
  unsigned ClassifyIp(const Ipv4Header& ip, Ptr<const Packet> payload, uint32_t& udpBytes)
  {
    if(ip.GetProtocol()!=17) return 0;
    const auto key=std::make_tuple(ip.GetSource().Get(),ip.GetDestination().Get(),ip.GetIdentification());
    if(ip.GetFragmentOffset()!=0)
    {
      udpBytes=ip.GetPayloadSize();
      auto it=m_fragmentClass.find(key);
      return it==m_fragmentClass.end()?0:it->second;
    }
    auto p=payload->Copy();
    if(p->GetSize()<8 || ip.GetPayloadSize()<8) return 0;
    UdpHeader udp; p->RemoveHeader(udp); const uint16_t port=udp.GetDestinationPort();
    udpBytes=std::min(p->GetSize(),uint32_t(ip.GetPayloadSize()-8)); // Excludes Wi-Fi FCS.
    uint8_t bytes[73]{}; p->CopyData(bytes,std::min(uint32_t(73),udpBytes));
    auto remember=[&](unsigned mask){m_fragmentClass[key]=mask;return mask;};
    if(algorithm=="sbcr" && port==889)
    {
      if(udpBytes>=1 && bytes[0]>=1 && bytes[0]<=7) return remember(1);
      // Wire adapter additions only: keep the existing units and CSV schema.
      // Probes and link reports serve discovery and observation together;
      // audit requests/receipts are dedicated security control.
      const bool version=udpBytes>=9 && bytes[1]==0x53 && bytes[2]==0x42 &&
                         bytes[5]==1 && bytes[7]==0 && bytes[8]==0;
      if(version && bytes[0]==8 && bytes[3]==0x46 && bytes[4]==0x41 && udpBytes>=169)
      {
        if(bytes[6]==1)return remember(5);
        if((bytes[6]==2 || bytes[6]==3) && udpBytes==169)return remember(3);
      }
      if(version && bytes[0]==9 && bytes[3]==0x4c && bytes[4]==0x52 &&
         bytes[6]<=1 && udpBytes==49)return remember(5);
      if(udpBytes>=105 && bytes[0]==10 && bytes[1]==0x53 && bytes[2]==0x42 &&
         bytes[3]==0x49 && bytes[4]==0x46 && bytes[5]==1 && (bytes[6]==1||bytes[6]==2) &&
         bytes[7]>=2 && bytes[7]<=16 && bytes[8]>=1 && bytes[8]<=bytes[7] &&
         udpBytes==33u+72u*bytes[8])return remember(5);
      if(udpBytes>=57 && bytes[0]==11 && bytes[1]==0x53 && bytes[2]==0x42 &&
         bytes[3]==0x48 && bytes[4]==0x41 && bytes[5]==1 && bytes[8]==0 &&
         (bytes[7]==1||bytes[7]==2))
      {
        const uint32_t bodyBytes=(uint32_t(bytes[15])<<8)|bytes[16];
        if(bodyBytes<=1177 && udpBytes==57+bodyBytes &&
           ((bytes[6]==2&&bodyBytes>0)||((bytes[6]==1||bytes[6]==3)&&bodyBytes==0)))return remember(3);
      }
      if(udpBytes>=89 && bytes[0]==12 && bytes[1]==0x53 && bytes[2]==0x42 &&
         bytes[3]==0x4d && bytes[4]==0x46 && bytes[5]==1 && bytes[6]>=1 && bytes[6]<=3 &&
         bytes[7]<=1 && bytes[8]==0 && bytes[55]>=1 && bytes[55]<=16 &&
         bytes[56]>=1 && bytes[56]<=8 && bytes[57]<=bytes[56] && bytes[58]<=64 &&
         bytes[63]<=2 && bytes[64]==0 &&
         udpBytes==89u+2u*(bytes[55]+bytes[56]+bytes[58])+4u*bytes[57])return remember(5);
      if(udpBytes>=73 && bytes[0]==13 && bytes[1]==0x53 && bytes[2]==0x42 &&
         bytes[3]==0x4d && bytes[4]==0x53 && bytes[5]==1 && bytes[6]>=1 && bytes[6]<=2 &&
         bytes[7]>=1 && bytes[7]<=3 && bytes[8]>=1 && bytes[8]<=2 && bytes[65]<=1 &&
         bytes[66]==0 && bytes[67]==0 && bytes[68]==0 && bytes[69]==0 && bytes[70]==0 && bytes[71]==0 && bytes[72]==0 &&
         udpBytes==73u+72u*bytes[65] &&
         ((bytes[7]==3&&bytes[65]==0&&bytes[6]==1)||(bytes[7]!=3&&bytes[65]==1)))return remember(5);
      if(udpBytes==105 && bytes[0]==15 && bytes[1]==0x53 && bytes[2]==0x42 &&
         bytes[3]==0x52 && bytes[4]==0x43 && bytes[5]==1 && bytes[6]>=1 && bytes[6]<=5 &&
         bytes[7]==0 && (bytes[8]==0||(bytes[6]==4&&bytes[8]==1)) && bytes[15]==0 && bytes[16]==0)return remember(3);
      if(udpBytes>=225 && bytes[0]==17 && bytes[1]==0x53 && bytes[2]==0x42 && bytes[3]==0x43 && bytes[4]==0x44 &&
         bytes[5]==1 && bytes[6]>=1 && bytes[6]<=4 && bytes[7]>=2 && bytes[7]<=16 && bytes[8]>=1 && bytes[8]<=bytes[7] &&
         bytes[13]<bytes[8] && bytes[14]<=1 && bytes[15]==0x22 && bytes[16]==0xc4 && udpBytes==153u+72u*bytes[8])return remember(5);
      if(udpBytes==153 && bytes[0]==16 && bytes[1]==0x53 && bytes[2]==0x42 &&
         bytes[3]==0x54 && bytes[4]==0x57 && bytes[5]==1 && (bytes[6]==1||bytes[6]==2) &&
         bytes[7]==1 && bytes[8]==0 && bytes[37]==0x53 && bytes[38]==0x42 &&
         bytes[39]==0x54 && bytes[40]==0x4b && bytes[41]==1 && bytes[42]==1 && bytes[43]==0 && bytes[44]==0)return remember(3);
      if(udpBytes==65 && bytes[0]==14 && bytes[1]==0x53 && bytes[2]==0x42 &&
         bytes[3]==0x43 && bytes[4]==0x53 && bytes[5]==1 && (bytes[6]==6||bytes[6]==7) &&
         bytes[7]>=1 && bytes[7]<=2 && bytes[8]==0 && (bytes[15]!=255||bytes[16]!=255))return remember(5);
      if(udpBytes>=137 && bytes[0]==14 && bytes[1]==0x53 && bytes[2]==0x42 &&
         bytes[3]==0x43 && bytes[4]==0x53 && bytes[5]==1 && bytes[6]>=1 && bytes[6]<=5 &&
         bytes[7]>=1 && bytes[7]<=2 && (bytes[6]==1||bytes[7]==1) && bytes[8]==0 &&
         bytes[15]==0 && bytes[16]==0 && udpBytes==((bytes[6]==3||bytes[6]==4)?209u:137u) &&
         bytes[65]==0x53 && bytes[66]==0x42 && bytes[67]==0x4d && bytes[68]==0x43 &&
         bytes[69]==1 && bytes[70]==(bytes[6]==1?1:2) && bytes[71]==1 && bytes[72]==0)return remember(5);
      ++unknownControl;
    }
    if(algorithm=="qsr" && port==655)
    { if(udpBytes>=1 && (bytes[0]==1 || bytes[0]==3 || bytes[0]==4)) return remember(bytes[0]==1 ? 5 : 1); ++unknownControl; }
    if(algorithm=="dscr" && port==660)
    { if(udpBytes>=3 && bytes[0]>=1 && bytes[0]<=7 && bytes[1]==1 && bytes[2]==bytes[0]) return remember(1); ++unknownControl; }
    if(algorithm=="dscr" && port==29642)
    { if(udpBytes>=26 && bytes[0]==0x44 && bytes[1]==0x53 && bytes[2]==0x47 && bytes[3]==0x34 && bytes[16]>=1 && bytes[16]<=4) return remember(bytes[16]==4 ? 3 : 1); ++unknownControl; }
    return remember(0);
  }
  static void PhyTx(SecureRoutingMetrics* self, uint32_t node, Ptr<const Packet> packet, double)
  {
    ++self->total.packets; self->total.mpduBytes+=packet->GetSize();
    uint32_t payload=0; unsigned mask=self->Classify(packet,payload);
    auto add=[&](Cost& c){ ++c.packets; c.mpduBytes+=packet->GetSize(); c.udpPayloadBytes+=payload; };
    if(mask&1) add(self->control);
    if(mask&2) add(self->security);
    if(mask&4) add(self->sharedSecurity);
    if(self->m_pending.at(node)!=8) ++self->ambiguousPpdus;
    self->m_pending[node]=mask;
  }
  static void PhyState(SecureRoutingMetrics* self, uint32_t node, Time start, Time duration, WifiPhyState state)
  {
    if(state!=WifiPhyState::TX) return;
    double seconds=std::max(0.0,std::min(duration.GetSeconds(),self->stop-start.GetSeconds()));
    self->total.airtimeS+=seconds; const unsigned mask=self->m_pending.at(node);
    if(mask==8) { ++self->invalidTrace; return; }
    if(mask&1) self->control.airtimeS+=seconds;
    if(mask&2) self->security.airtimeS+=seconds;
    if(mask&4) self->sharedSecurity.airtimeS+=seconds;
    self->m_pending[node]=8;
  }
  static void Energy(SecureRoutingMetrics* self, uint32_t node, double, double value)
  { self->m_remaining.at(node)=value; }
  static void Expired(SecureRoutingMetrics* self, Ptr<const WifiMpdu>) { ++self->queueExpired; }
  static void Overflow(SecureRoutingMetrics* self, Ptr<const WifiMpdu>) { ++self->queueOverflow; }

  void Attach(const NodeContainer& container, const NetDeviceContainer& devices,
              const EnergySourceContainer& sources)
  {
    NS_ABORT_MSG_IF(container.GetN()!=nodes || devices.GetN()!=nodes || sources.GetN()!=nodes,"Metric attachment size mismatch");
    m_nodes=container; m_pending.assign(nodes,8); m_sources.clear(); m_sources.reserve(nodes);
    m_initial.resize(nodes); m_remaining.resize(nodes);
    for(uint32_t i=0;i<nodes;++i)
    {
      m_sources.push_back(sources.Get(i));
      auto wifi=DynamicCast<WifiNetDevice>(devices.Get(i));
      NS_ABORT_MSG_IF(!wifi,"Metrics require Wi-Fi"); m_wifi.push_back(wifi);
      auto ipv4=container.Get(i)->GetObject<Ipv4L3Protocol>();
      Connect(ipv4,"SendOutgoing",MakeBoundCallback(&Outgoing,this,i));
      Connect(ipv4,"UnicastForward",MakeBoundCallback(&Outgoing,this,i));
      Connect(ipv4,"LocalDeliver",MakeBoundCallback(&LocalDeliver,this,i));
      Connect(wifi->GetPhy(),"PhyTxBegin",MakeBoundCallback(&PhyTx,this,i));
      Connect(wifi->GetPhy()->GetState(),"State",MakeBoundCallback(&PhyState,this,i));
      Connect(sources.Get(i),"RemainingEnergy",MakeBoundCallback(&Energy,this,i));
      m_initial[i]=m_remaining[i]=sources.Get(i)->GetInitialEnergy();
      auto queue=wifi->GetMac()->GetTxop()->GetWifiMacQueue();
      Connect(queue,"Expired",MakeBoundCallback(&Expired,this));
      Connect(queue,"DropBeforeEnqueue",MakeBoundCallback(&Overflow,this));
    }
    SampleLinks();
  }
  void Finalize()
  {
    m_linkEvent.Cancel();
    rightCensoredLinks=m_links.size();
    consumedJ=0;
    for(uint32_t i=0;i<nodes;++i)
    {
      // Run is finished. This terminal update cannot affect any forwarding
      // decision; it closes the energy integration at precisely stop seconds.
      m_remaining[i]=m_sources.at(i)->GetRemainingEnergy();
      consumedJ+=m_initial[i]-m_remaining[i];
    }
    for(auto& flow:flows) flow.submitted=flow.client->GetTotalTx()/100;
  }
  void Disconnect()
  {
    m_linkEvent.Cancel();
    for(uint32_t i=0;i<m_wifi.size();++i)
    {
      auto ip=m_nodes.Get(i)->GetObject<Ipv4L3Protocol>(); auto wifi=m_wifi[i];
      ip->TraceDisconnectWithoutContext("SendOutgoing",MakeBoundCallback(&Outgoing,this,i));
      ip->TraceDisconnectWithoutContext("UnicastForward",MakeBoundCallback(&Outgoing,this,i));
      ip->TraceDisconnectWithoutContext("LocalDeliver",MakeBoundCallback(&LocalDeliver,this,i));
      wifi->GetPhy()->TraceDisconnectWithoutContext("PhyTxBegin",MakeBoundCallback(&PhyTx,this,i));
      wifi->GetPhy()->GetState()->TraceDisconnectWithoutContext("State",MakeBoundCallback(&PhyState,this,i));
      m_sources.at(i)->TraceDisconnectWithoutContext("RemainingEnergy",MakeBoundCallback(&Energy,this,i));
      auto queue=wifi->GetMac()->GetTxop()->GetWifiMacQueue();
      queue->TraceDisconnectWithoutContext("Expired",MakeBoundCallback(&Expired,this));
      queue->TraceDisconnectWithoutContext("DropBeforeEnqueue",MakeBoundCallback(&Overflow,this));
    }
  }
  uint64_t Count(const std::string& name) const
  {
    uint64_t n=0;
    for(const auto& f:flows)
    {
      if(name=="attempts") n+=f.attempts;
      else if(name=="submitted") n+=f.submitted;
      else if(name=="unique") n+=f.sequences.size();
      else if(name=="bytes") n+=f.uniqueBytes;
      else if(name=="hops") n+=f.hopsSum;
      else if(name=="hop_samples") n+=f.hopsSamples;
      else if(name=="post_attempts") n+=f.postAttempts;
      else if(name=="post_unique") n+=f.postUnique;
      else if(name=="duplicates") n+=f.duplicates;
      else if(name=="invalid") n+=f.invalid;
      else if(name=="received") n+=f.received;
      else if(name=="zero_flows") n+=f.sequences.empty();
    }
    return n;
  }
  std::vector<double> Delays() const
  { std::vector<double> values; for(const auto& f:flows) values.insert(values.end(),f.delaysMs.begin(),f.delaysMs.end()); return values; }
  bool Validate(bool fixed) const
  {
    if(invalidTrace || unknownControl || ambiguousPpdus || Count("invalid") || Count("hop_samples")!=Count("unique") || attackDrops>maliciousOpportunities || control.mpduBytes>total.mpduBytes || security.mpduBytes>control.mpduBytes || sharedSecurity.mpduBytes>control.mpduBytes) return false;
    for(const auto& f:flows) if(f.submitted>f.attempts || f.sequences.size()>f.submitted || (fixed && f.attempts!=200)) return false;
    for(const auto& p:droppedPackets) if(deliveredPackets.count(p)) return false;
    return true;
  }
  // Results are deliberately named by numerator, population, units and scope.
  std::vector<std::pair<std::string,std::string>> Results() const
  {
    std::vector<std::pair<std::string,std::string>> r;
    auto num=[&](const std::string& key,double value){r.emplace_back(key,Number(value));};
    auto count=[&](const std::string& key,uint64_t value){r.emplace_back(key,std::to_string(value));};
    const auto a=Count("attempts"), s=Count("submitted"), u=Count("unique");
    count("AppAttempts_packets",a); count("AppSubmitted_packets",s); count("AppSendRefused_packets",a-s);
    count("UdpReceived_packets",Count("received")); count("UniqueDelivered_packets",u);
    count("DuplicateReceived_packets",Count("duplicates")); count("InvalidReceived_packets",Count("invalid"));
    count("ZeroDelivery_flows",Count("zero_flows")); count("UniqueDeliveredUdp_bytes",Count("bytes"));
    num("PDR_pct",Ratio(u,a,100)); num("SubmittedPDR_pct",Ratio(u,s,100));
    auto delays=Delays(); num("AvgEndToEndDelay_ms",Ratio(std::accumulate(delays.begin(),delays.end(),0.0),u));
    num("UdpDelayP50_ms",Percentile(delays,.5)); num("UdpDelayP95_ms",Percentile(delays,.95));
    count("DeliveredHop_samples",Count("hop_samples")); count("DeliveredHop_sum",Count("hops"));
    num("AverageHops_hops",Ratio(Count("hops"),Count("hop_samples")));
    num("BusinessWindow_s",stop-trafficStart); num("Throughput_kbps",8.0*Count("bytes")/(1000*(stop-trafficStart)));
    double sum=0,sq=0; for(const auto& f:flows){double rate=Ratio(f.sequences.size(),f.attempts); if(!std::isfinite(rate)) rate=0; sum+=rate; sq+=rate*rate;}
    num("DeliveryJain_ratio",Ratio(sum*sum,flows.size()*sq));
    auto costs=[&](const char* prefix,const Cost& c)
    { std::string p(prefix); count(p+"TxMpdu_packets",c.packets); count(p+"TxMpdu_bytes",c.mpduBytes); count(p+"TxUdpPayload_bytes",c.udpPayloadBytes); num(p+"Bandwidth_kbps",8.0*c.mpduBytes/(1000*stop)); num(p+"TxAirtime_s",c.airtimeS); };
    num("ControlWindow_s",stop); costs("Control",control); costs("DedicatedSecurityControl",security); costs("SharedSecurityControl",sharedSecurity);
    num("ControlOverhead_packetsPerAttempt",Ratio(control.packets,a));
    num("ControlOverhead_packetsPerDelivered",Ratio(control.packets,u));
    count("AllPhyTx_frames",total.packets); count("AllPhyTx_bytes",total.mpduBytes); num("AllPhyTxAirtime_s",total.airtimeS);
    num("RadioEnergyTotal_J",consumedJ); num("RadioEnergyAvg_J",Ratio(consumedJ,nodes));
    count("MacExpired_events",queueExpired); count("MacDropBeforeEnqueue_events",queueOverflow);
    count("CompletedRangeLink_episodes",completedRangeLinks); count("LeftCensoredRangeLink_episodes",leftCensoredLinks); count("RightCensoredRangeLink_episodes",rightCensoredLinks);
    num("AvgCompletedRangeLinkLifetime_s",Ratio(completedRangeLinkSeconds,completedRangeLinks)); num("RangeLinkSamplePeriod_s",.5);
    count("PostAttackGenerated_attempts",Count("post_attempts")); count("PostAttackGenerated_uniqueDelivered",Count("post_unique")); num("PostAttackGeneratedPDR_pct",Ratio(Count("post_unique"),Count("post_attempts"),100));
    count("RelayOpportunity_events",relayOpportunities); count("MaliciousRelayOpportunity_events",maliciousOpportunities);
    count("MaliciousExposed_uniqueBusinessPackets",maliciousPackets.size()); count("AttackDrop_events",attackDrops); count("AttackDropped_uniqueBusinessPackets",droppedPackets.size());
    num("AttackDropSuccess_pct",Ratio(attackDrops,maliciousOpportunities,100)); count("ForwardingSubmission_events",forwardSubmissions);
    uint64_t exposedDelivered=0; for(auto p:maliciousPackets) exposedDelivered+=deliveredPackets.count(p);
    count("MaliciousExposedDelivered_uniqueBusinessPackets",exposedDelivered);
    uint64_t tp=0,fp=0,fn=0,tn=0,etp=0,efn=0,evaluated=0,exposed=0,badRefused=0,goodRefused=0,refusals=0,reopened=0,goodReopen=0,badReopen=0,resumed=0,preFlag=0,preRefusal=0,handoffPairs=0,badHandoffPairs=0,flaggedPairs=0,observed=0,unobservedBad=0,everFlags=0;
    std::vector<double> detectDelays,restrictionDelays,reopenDelays;
    // All pairs must be enumerated, even if no event/transition was emitted.
    for(uint32_t i=0;i<nodes;++i) for(uint32_t j=0;j<nodes;++j) if(i!=j)
    {
      const auto it=pairs.find({i,j}); const PairState empty{}; const auto& p=it==pairs.end()?empty:it->second;
      const bool bad=dropTruth.at(j); evaluated+=p.evaluated;
      handoffPairs+=p.handoffRequests>0; badHandoffPairs+=bad && p.handoffRequests>0;
      flaggedPairs+=p.flagged;
      everFlags+=p.everFlagged || p.flagged;
      // Complete registered/observed population, including peers that NEVER
      // changed state. Unseen roster peers are coverage gaps, not invented
      // detection obligations. Actual malicious forwarding exposure is an
      // independent opportunity population, including unobserved pairs.
      if(p.registered)
      {
        ++observed;
        if(p.flagged) (bad?tp:fp)++; else (bad?fn:tn)++;
      }
      else unobservedBad+=bad;
      if(p.exposed) { ++exposed; if(p.flagged) ++etp; else ++efn; }
      if(p.refused) (bad?badRefused:goodRefused)++;
      refusals+=p.refusals;
      if(bad && p.firstFlag>=0 && p.firstDrop>=0)
      { if(p.firstFlag>=p.firstDrop) detectDelays.push_back(p.firstFlag-p.firstDrop); else ++preFlag; }
      if(bad && p.firstRefusal>=0 && p.firstOpportunity>=0)
      { if(p.firstRefusal>=p.firstOpportunity) restrictionDelays.push_back(p.firstRefusal-p.firstOpportunity); else ++preRefusal; }
      if(p.reopened) { ++reopened; (bad?badReopen:goodReopen)++; resumed+=p.resumed;
        if(p.firstReopenDelayS>=0) reopenDelays.push_back(p.firstReopenDelayS); }
    }
    count("OrderedPair_population",uint64_t(nodes)*(nodes-1)); count("Evaluated_pairs",evaluated); count("ActualMaliciousExposed_pairs",exposed);
    count("RegisteredObserved_pairs",observed); count("RegisteredObservedMalicious_pairs",tp+fn); count("RegisteredObservedBenign_pairs",fp+tn); count("UnobservedMaliciousRoster_pairs",unobservedBad);
    count("HandoffObligation_pairs",handoffPairs); count("MaliciousHandoffObligation_pairs",badHandoffPairs);
    count("EndWindowFlagged_pairs",flaggedPairs);
    count("EverWindowFlagged_pairs",everFlags);
    auto optCount=[&](const std::string& k,uint64_t v,bool available){r.emplace_back(k,available?std::to_string(v):"N/A");};
    const bool riskClassification=sbcrRisk && relayTruthAvailable;
    const bool classifier=detectionAvailable||riskClassification;
    r.emplace_back("IdentityDetection_scope",detectionAvailable?"dscr_registered_observed_zero_R":"N/A");
    r.emplace_back("LocalRisk_scope",sbcrRisk?"sbcr_evaluated_statistical_F_block":"N/A");
    for(auto prefix:{std::string("IdentityDetection"),std::string("LocalRisk")})
    {
      bool available=prefix=="IdentityDetection"?detectionAvailable:riskClassification;
      optCount(prefix+"TP_pairs",tp,available); optCount(prefix+"FP_pairs",fp,available); optCount(prefix+"FN_pairs",fn,available); optCount(prefix+"TN_pairs",tn,available);
      num(prefix+"Rate_pct",available?Ratio(tp,tp+fn,100):NAN); num(prefix+"FalsePositiveRate_pct",available?Ratio(fp,fp+tn,100):NAN); num(prefix+"MissRate_pct",available?Ratio(fn,tp+fn,100):NAN);
      num(prefix+"ExposedRate_pct",available?Ratio(etp,etp+efn,100):NAN);
    }
    optCount("FirstFlagDelay_samples",detectDelays.size(),classifier); num("AvgFirstFlagAfterActualDrop_s",classifier?Ratio(std::accumulate(detectDelays.begin(),detectDelays.end(),0.0),detectDelays.size()):NAN);
    optCount("FlagBeforeActualDrop_pairs",preFlag,classifier);
    optCount("CorrectLocalRefusal_pairs",badRefused,restrictionAvailable && relayTruthAvailable); optCount("BenignLocalRefusal_pairs",goodRefused,restrictionAvailable && relayTruthAvailable); optCount("LocalPolicyRefusal_events",refusals,restrictionAvailable);
    optCount("LocalRefusalDelay_samples",restrictionDelays.size(),restrictionAvailable); num("AvgLocalRefusalAfterExposure_s",restrictionAvailable?Ratio(std::accumulate(restrictionDelays.begin(),restrictionDelays.end(),0.0),restrictionDelays.size()):NAN); optCount("RefusalBeforeExposure_pairs",preRefusal,restrictionAvailable);
    optCount("LocalReopened_pairs",reopened,sbcrRisk); optCount("BenignLocalReopened_pairs",goodReopen,sbcrRisk && relayTruthAvailable); optCount("MaliciousLocalReopened_pairs",badReopen,sbcrRisk && relayTruthAvailable); optCount("ReopenedForwardingResumed_pairs",resumed,sbcrRisk);
    num("LocalReopenRate_pct",sbcrRisk?Ratio(reopened,everFlags,100):NAN); optCount("LocalReopenDelay_samples",reopenDelays.size(),sbcrRisk);
    num("AvgLocalReopenAfterBlock_s",sbcrRisk?Ratio(std::accumulate(reopenDelays.begin(),reopenDelays.end(),0.0),reopenDelays.size()):NAN);
    r.emplace_back("GlobalIsolation","N/A"); r.emplace_back("PostAttackEndRecovery","N/A"); r.emplace_back("IncrementalSharedSecurityCost","N/A");
    count("UnknownControl_events",unknownControl); count("InvalidTrace_events",invalidTrace); count("AmbiguousPpdu_events",ambiguousPpdus);
    return r;
  }
private:
  template<class T,class C> static void Connect(Ptr<T> object,const char* name,const C& callback)
  { NS_ABORT_MSG_IF(!object || !object->TraceConnectWithoutContext(name,callback),"Missing benchmark trace "<<name); }
  void SampleLinks()
  {
    const double now=Simulator::Now().GetSeconds();
    for(uint32_t i=0;i<nodes;++i) for(uint32_t j=i+1;j<nodes;++j)
    {
      bool up=!m_wifi[i]->GetPhy()->IsStateOff() && !m_wifi[j]->GetPhy()->IsStateOff() &&
        CalculateDistance(m_nodes.Get(i)->GetObject<MobilityModel>()->GetPosition(),m_nodes.Get(j)->GetObject<MobilityModel>()->GetPosition())<=range;
      Pair key{i,j}; auto it=m_links.find(key);
      if(up && it==m_links.end()) { m_links[key]={now,now==0}; if(now==0) ++leftCensoredLinks; }
      else if(!up && it!=m_links.end()) { if(!it->second.second){++completedRangeLinks; completedRangeLinkSeconds+=now-it->second.first;} m_links.erase(it); }
    }
    if(now+.5<stop) m_linkEvent=Simulator::Schedule(Seconds(.5),&SecureRoutingMetrics::SampleLinks,this);
  }
  NodeContainer m_nodes;
  std::vector<Ptr<EnergySource>> m_sources;
  std::vector<Ptr<WifiNetDevice>> m_wifi;
  std::vector<double> m_initial,m_remaining;
  std::vector<unsigned> m_pending;
  std::map<std::pair<Attempt,uint32_t>,uint32_t> m_upstream;
  std::map<std::tuple<uint32_t,uint32_t,uint16_t>,unsigned> m_fragmentClass;
  std::map<Pair,std::pair<double,bool>> m_links;
  EventId m_linkEvent;
};
}} // ns3::secure_benchmark
#endif
