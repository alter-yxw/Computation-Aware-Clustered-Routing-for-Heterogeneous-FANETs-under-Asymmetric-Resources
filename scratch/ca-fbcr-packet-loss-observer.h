/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_PACKET_LOSS_OBSERVER_H
#define CA_FBCR_PACKET_LOSS_OBSERVER_H

#include "ns3/arp-cache.h"
#include "ns3/arp-l3-protocol.h"
#include "ns3/ipv4-interface.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/log.h"
#include "ns3/node.h"
#include "ns3/simulator.h"
#include "ns3/txop.h"
#include "ns3/uinteger.h"
#include "ns3/wifi-mac-queue.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-mpdu.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-tx-vector.h"
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <streambuf>
#include <tuple>
#include <vector>

namespace ns3::cafbcr {

// Entry-point-only, passive observations. No Packet construction/copy, packet
// tags, RNG, events, additional RouteOutput, mobility reads or protocol feedback.
// In particular, do NOT attach PHY receive traces that call WifiPsdu::GetPacket:
// some of those native trace paths allocate a fresh packet UID when enabled.
class PacketLossObservationWriter
{
public:
  static constexpr const char* SCHEMA = "cafbcr-packet-loss-observation-v1";
  PacketLossObservationWriter(const std::string& directory, Time end, bool caNextHop)
    : m_dir(directory), m_end(end.GetNanoSeconds()), m_ca(caNextHop),
      m_events(m_dir / "packet-loss-events.csv"), m_routes(m_dir / "packet-route.csv"), m_tap(*this)
  {
    NS_ABORT_MSG_IF(!m_events || !m_routes, "Cannot open packet loss observations");
    m_events << "event_index,time_ns,event,node,interface,device,packet_uid,packet_bytes,framing,"
      "source_ip,destination_ip,protocol,ip_id,ttl,fragment_offset,more_fragments,ip_bytes,"
      "source_port,destination_port,reason_code,reason,next_ip,route_index,route_evidence,"
      "arp_cache_state,arp_entry_expired,mac_destination,mac_source,mac_sequence,mac_fragment,"
      "mac_retry,mac_next_ip,mac_route_agreement,packet_prefix_hex\n";
    m_routes << "schema,time_ns,stage,branch,node,packet_uid,source_ip,destination_ip,protocol,ip_id,ttl,"
      "packet_bytes,task,role,cluster,head,incoming_head,next_ip,next_node,next_role,"
      "next_neighbor_age_ns,cache_state,cache_age_ns,cached_gateway,cache_written,"
      "own_x,own_y,own_z,destination_x,destination_y,destination_z,next_x,next_y,next_z\n";
    if (m_ca) {
#ifdef NS3_LOG_ENABLE
      auto components = LogComponent::GetComponentList();
      auto found = components->find("cafbcrRoutingProtocol");
      NS_ABORT_MSG_IF(found == components->end(), "CA forwarding audit component unavailable");
      m_log = found->second;
      m_hadLogic = m_log->IsEnabled(LOG_LOGIC);
      m_oldClog = std::clog.rdbuf(&m_tap);
      m_log->Enable(LOG_LOGIC);
#else
      NS_ABORT_MSG("CA forwarding audit requires the existing logging-enabled build");
#endif
    }
  }

  ~PacketLossObservationWriter() { RestoreTap(); Disconnect(); }

  void Attach(uint32_t index, Ptr<Node> node)
  {
    NS_ABORT_MSG_IF(m_finalized || m_nodes.count(index), "Invalid packet observer attachment");
    NodeState state;
    state.ip = node->GetObject<Ipv4L3Protocol>();
    state.arp = node->GetObject<ArpL3Protocol>();
    NS_ABORT_MSG_IF(!state.ip || !state.arp, "Packet observer requires IPv4/ARP");
    for (uint32_t i = 0; i < state.ip->GetNInterfaces(); ++i) {
      auto iface = state.ip->GetInterface(i);
      if (auto cache = iface->GetArpCache()) {
        state.caches[i] = cache;
        Connect(cache, "Drop", MakeBoundCallback(&CacheDrop, this, index, i));
      }
    }
    Connect(state.ip, "Tx", MakeBoundCallback(&IpTx, this, index));
    Connect(state.ip, "Rx", MakeBoundCallback(&IpRx, this, index));
    Connect(state.ip, "Drop", MakeBoundCallback(&IpDrop, this, index));
    Connect(state.arp, "Drop", MakeBoundCallback(&ArpDrop, this, index));
    for (uint32_t d = 0; d < node->GetNDevices(); ++d) {
      auto wifi = DynamicCast<WifiNetDevice>(node->GetDevice(d));
      if (!wifi) continue;
      int32_t iface = state.ip->GetInterfaceForDevice(wifi);
      auto mac = wifi->GetMac();
      auto queue = mac->GetTxop() ? mac->GetTxop()->GetWifiMacQueue() : nullptr;
      state.wifi[d] = {iface, mac, queue};
      Connect(mac, "MacTx", MakeBoundCallback(&MacTx, this, index, d));
      Connect(mac, "MacRx", MakeBoundCallback(&MacRx, this, index, d));
      Connect(mac, "MacTxDrop", MakeBoundCallback(&MacTxDrop, this, index, d));
      Connect(mac, "DroppedMpdu", MakeBoundCallback(&MpduDrop, this, index, d));
      Connect(mac, "AckedMpdu", MakeBoundCallback(&Acked, this, index, d));
      Connect(mac, "MpduResponseTimeout", MakeBoundCallback(&Timeout, this, index, d));
      if (queue) Connect(queue, "Enqueue", MakeBoundCallback(&Enqueue, this, index, d));
      if (iface >= 0) for (uint32_t a = 0; a < state.ip->GetNAddresses(iface); ++a)
        m_macIps[Text(mac->GetAddress())].insert(Text(state.ip->GetAddress(iface, a).GetLocal()));
    }
    m_nodes.emplace(index, std::move(state));
  }

  void Finalize()
  {
    NS_ABORT_MSG_IF(m_finalized, "Packet observations already finalized");
    RestoreTap(); Disconnect(); m_finalized = true;
    std::ofstream config(m_dir / "packet-loss-config.json");
    config << "{\"schema\":\"" << SCHEMA << "\",\"nodes\":[";
    bool first = true;
    for (const auto& [n, state] : m_nodes) {
      if (!first) config << ',';
      first = false;
      config << "{\"node\":" << n << ",\"arp_caches\":[";
      bool cfirst = true;
      for (const auto& [i, cache] : state.caches) {
        UintegerValue retries, pending;
        cache->GetAttribute("MaxRetries", retries); cache->GetAttribute("PendingQueueSize", pending);
        if (!cfirst) config << ',';
        cfirst = false;
        config << "{\"interface\":" << i << ",\"wait_reply_ns\":" << cache->GetWaitReplyTimeout().GetNanoSeconds()
          << ",\"dead_ns\":" << cache->GetDeadTimeout().GetNanoSeconds()
          << ",\"alive_ns\":" << cache->GetAliveTimeout().GetNanoSeconds()
          << ",\"max_retries\":" << retries.Get() << ",\"pending_queue_size\":" << pending.Get() << '}';
      }
      config << "],\"wifi\":["; cfirst = true;
      for (const auto& [d, wifi] : state.wifi) {
        if (!cfirst) config << ',';
        cfirst = false;
        config << "{\"device\":" << d << ",\"interface\":" << wifi.iface
          << ",\"mac\":" << Quote(Text(wifi.mac->GetAddress())) << ",\"queue_observed\":" << (wifi.queue ? "true" : "false");
        if (wifi.queue) config << ",\"queue_max_size\":" << Quote(Text(wifi.queue->GetMaxSize()))
                              << ",\"queue_max_delay_ns\":" << wifi.queue->GetMaxDelay().GetNanoSeconds();
        config << '}';
      }
      config << "]}";
    }
    config << "],\"mac_to_ipv4\":{"; first = true;
    for (const auto& [mac, ips] : m_macIps) {
      if (!first) config << ',';
      first = false; config << Quote(mac) << ":[";
      bool ifirst = true;
      for (const auto& ip : ips) { if (!ifirst) config << ','; ifirst = false; config << Quote(ip); }
      config << ']';
    }
    config << "}}\n";
    m_events.close(); m_routes.close(); config.close();
    bool complete = Simulator::Now().GetNanoSeconds() >= m_end;
    bool valid = complete && !m_errors && bool(m_events) && bool(m_routes) && bool(config);
    std::ofstream summary(m_dir / "packet-loss-summary.json");
    summary << "{\"schema\":\"" << SCHEMA << "\",\"formal_eligible\":false,\"measurement_valid\":"
      << (valid ? "true" : "false") << ",\"window_complete\":" << (complete ? "true" : "false")
      << ",\"window_end_ns\":" << m_end << ",\"errors\":" << m_errors
      << ",\"after_cutoff_events\":" << m_after << ",\"event_rows\":" << m_eventCount
      << ",\"route_rows\":" << m_routeCount << ",\"ca_next_hop_enabled\":" << (m_ca ? "true" : "false")
      << ",\"route_binding\":\"same_node_UID_time_destination_protocol; forward_TTL_minus_one; full_IP_hop_key\""
      << ",\"arp_l3_split\":\"exact_IPv4_Tx_route_witness_and_read_only_cache_state_or_unknown\""
      << ",\"arp_cache_drop\":\"native_pending_retry_exhaustion; retries_cleared_before_callback\""
      << ",\"ack_scope\":\"one_MAC_hop_only; response_timeout_is_not_terminal_loss\""
      << ",\"count_scope\":\"native_events_not_unique_lost_demands; callbacks_may_overlap_or_precede_delivery\""
      << ",\"phy_receive_failure_cause\":\"unavailable_not_hooked_to_preserve_packet_UIDs\""
      << ",\"event_counts\":{";
    bool f = true;
    for (const auto& [key, count] : m_counts) { if (!f) summary << ','; f = false; summary << Quote(key) << ':' << count; }
    summary << "},\"reason_counts\":{"; f = true;
    for (const auto& [key, count] : m_reasons) { if (!f) summary << ','; f = false; summary << Quote(key) << ':' << count; }
    summary << "}}\n"; summary.close();
    NS_ABORT_MSG_IF(!valid || !summary, "Invalid/incomplete packet loss observations");
  }

private:
  struct Info {
    bool ip = false; uint32_t source = 0, destination = 0, ipBytes = 0;
    uint16_t id = 0, fragment = 0; uint8_t protocol = 0, ttl = 0;
    bool more = false; std::string sport = "null", dport = "null", framing = "unknown", prefix;
  };
  struct Witness {
    uint64_t index = 0; int64_t time = 0; std::string stage, source, destination, protocol, id, ttl, next;
  };
  struct Binding { Witness route; int32_t iface = -1; bool ambiguous = false; };
  struct WifiState { int32_t iface; Ptr<WifiMac> mac; Ptr<WifiMacQueue> queue; };
  struct NodeState { Ptr<Ipv4L3Protocol> ip; Ptr<ArpL3Protocol> arp;
    std::map<uint32_t, Ptr<ArpCache>> caches; std::map<uint32_t, WifiState> wifi; };
  using Key = std::tuple<uint32_t, uint64_t, uint32_t, uint32_t, uint8_t, uint16_t, uint8_t, uint16_t>;
  class Tap : public std::streambuf {
  public: explicit Tap(PacketLossObservationWriter& owner) : m_owner(owner) {}
  protected:
    std::streamsize xsputn(const char* s, std::streamsize n) override {
      auto result = m_owner.m_oldClog->sputn(s,n);
      for (std::streamsize i=0;i<n;++i) Accept(s[i]);
      return result;
    }
    int_type overflow(int_type ch) override {
      if (traits_type::eq_int_type(ch,traits_type::eof())) return traits_type::not_eof(ch);
      char c=traits_type::to_char_type(ch); auto result=m_owner.m_oldClog->sputc(c); Accept(c); return result;
    }
    int sync() override { return m_owner.m_oldClog->pubsync(); }
  private:
    void Accept(char c) {
      if(c=='\n') {m_owner.RouteLine(m_line); m_line.clear();}
      else if(m_line.size()<65536) m_line+=c;
      else {m_line.clear(); ++m_owner.m_errors;}
    }
    PacketLossObservationWriter& m_owner; std::string m_line;
  };

  template <typename T> static std::string Text(const T& value) { std::ostringstream s; s << value; return s.str(); }
  static std::string Quote(const std::string& value) {
    std::string s="\"";
    for (unsigned char c:value) { if(c=='"'||c=='\\') {s+='\\';s+=c;}
      else if(c=='\n') s+="\\n"; else if(c=='\r') s+="\\r"; else if(c=='\t') s+="\\t"; else if(c>=32) s+=c; }
    return s+'"';
  }
  template <typename T, typename C> static void Connect(Ptr<T> object, const char* name, C callback) {
    NS_ABORT_MSG_IF(!object->TraceConnectWithoutContext(name, callback), "Packet observation trace unavailable: " << name);
  }
  void RestoreTap() {
    if(m_oldClog) {std::clog.rdbuf(m_oldClog); m_oldClog=nullptr; if(!m_hadLogic) m_log->Disable(LOG_LOGIC);}
  }
  void Disconnect() {
    if(m_disconnected) return;
    m_disconnected=true;
    for(const auto& [n,s]:m_nodes) {
      s.ip->TraceDisconnectWithoutContext("Tx",MakeBoundCallback(&IpTx,this,n));
      s.ip->TraceDisconnectWithoutContext("Rx",MakeBoundCallback(&IpRx,this,n));
      s.ip->TraceDisconnectWithoutContext("Drop",MakeBoundCallback(&IpDrop,this,n));
      s.arp->TraceDisconnectWithoutContext("Drop",MakeBoundCallback(&ArpDrop,this,n));
      for(const auto& [i,c]:s.caches) c->TraceDisconnectWithoutContext("Drop",MakeBoundCallback(&CacheDrop,this,n,i));
      for(const auto& [d,w]:s.wifi) {
        w.mac->TraceDisconnectWithoutContext("MacTx",MakeBoundCallback(&MacTx,this,n,d));
        w.mac->TraceDisconnectWithoutContext("MacRx",MakeBoundCallback(&MacRx,this,n,d));
        w.mac->TraceDisconnectWithoutContext("MacTxDrop",MakeBoundCallback(&MacTxDrop,this,n,d));
        w.mac->TraceDisconnectWithoutContext("DroppedMpdu",MakeBoundCallback(&MpduDrop,this,n,d));
        w.mac->TraceDisconnectWithoutContext("AckedMpdu",MakeBoundCallback(&Acked,this,n,d));
        w.mac->TraceDisconnectWithoutContext("MpduResponseTimeout",MakeBoundCallback(&Timeout,this,n,d));
        if(w.queue) w.queue->TraceDisconnectWithoutContext("Enqueue",MakeBoundCallback(&Enqueue,this,n,d));
      }
    }
  }
  static uint16_t U16(const uint8_t* b) {return (uint16_t(b[0])<<8)|b[1];}
  static uint32_t U32(const uint8_t* b) {return (uint32_t(U16(b))<<16)|U16(b+2);}
  static Info Decode(Ptr<const Packet> p, bool llc, const Ipv4Header* header=nullptr) {
    Info r; std::array<uint8_t,80> bytes{};
    uint32_t length=p->CopyData(bytes.data(),std::min<uint32_t>(bytes.size(),p->GetSize()));
    static constexpr char hex[]="0123456789abcdef";
    for(uint32_t k=0;k<std::min<uint32_t>(length,64);++k) {r.prefix+=hex[bytes[k]>>4];r.prefix+=hex[bytes[k]&15];}
    unsigned offset=0, payload=0;
    if(header) {
      r.ip=true;r.framing="ipv4_payload";r.source=header->GetSource().Get();r.destination=header->GetDestination().Get();
      r.protocol=header->GetProtocol();r.id=header->GetIdentification();r.ttl=header->GetTtl();
      r.fragment=header->GetFragmentOffset();r.more=!header->IsLastFragment();
      r.ipBytes=header->GetSerializedSize()+p->GetSize();
    } else {
      if(llc) {
        if(length<8 || bytes[0]!=0xaa || bytes[1]!=0xaa || bytes[2]!=3) {r.framing="unparsed_llc";return r;}
        uint16_t type=U16(bytes.data()+6);
        if(type!=0x0800) {r.framing=type==0x0806?"llc_arp":"llc_other";return r;}
        offset=8;
      }
      r.framing=llc?"llc_ipv4":"ipv4_or_other";
      if(length<offset+20 || bytes[offset]>>4!=4) return r;
      unsigned ihl=(bytes[offset]&15)*4;
      if(ihl<20 || p->GetSize()<offset+ihl || U16(bytes.data()+offset+2)<ihl) return r;
      r.ip=true;r.framing=llc?"llc_ipv4":"ipv4";const auto* b=bytes.data()+offset;
      r.source=U32(b+12);r.destination=U32(b+16);r.ipBytes=U16(b+2);r.id=U16(b+4);r.ttl=b[8];r.protocol=b[9];
      auto fragment=U16(b+6);r.fragment=(fragment&8191)*8;r.more=(fragment&8192)!=0;payload=offset+ihl;
    }
    if(r.fragment==0 && (r.protocol==17||r.protocol==6) && length>=payload+4) {
      r.sport=Text(U16(bytes.data()+payload));r.dport=Text(U16(bytes.data()+payload+2));
    }
    return r;
  }
  static Key HopKey(uint32_t n, Ptr<const Packet> p, const Info& i) {
    return {n,p->GetUid(),i.source,i.destination,i.protocol,i.id,i.ttl,i.fragment};
  }
  void RouteLine(const std::string& line) {
    if(line.rfind("CA_FORWARD_V1,",0)!=0) return;
    if(Simulator::Now().GetNanoSeconds()>m_end) {++m_after;return;}
    std::vector<std::string> v;std::istringstream in(line);std::string part;
    while(std::getline(in,part,','))v.push_back(part);
    if(v.size()!=34) {++m_errors;return;}
    try {
      Witness w{m_routeCount,std::stoll(v[1]),v[2],v[6],v[7],v[8],v[9],v[10],v[17]};
      uint32_t n=std::stoul(v[4]);uint64_t uid=std::stoull(v[5]);
      if(w.time!=Simulator::Now().GetNanoSeconds() || (w.stage!="output"&&w.stage!="forward")) {++m_errors;return;}
      m_latest[{n,uid}]=std::move(w);m_routes<<line<<'\n';++m_routeCount;
    } catch(const std::exception&) {++m_errors;}
  }
  void Bind(uint32_t n, uint32_t iface, Ptr<const Packet> p, const Info& i) {
    if(!m_ca||!i.ip)return;
    auto w=m_latest.find({n,p->GetUid()});if(w==m_latest.end())return;
    const auto& r=w->second;
    if(r.time!=Simulator::Now().GetNanoSeconds() || r.destination!=Text(Ipv4Address(i.destination)) ||
       r.protocol!=Text(unsigned(i.protocol)) || r.next=="null")return;
    if(r.stage=="forward" && (r.ttl=="null" || r.id!=Text(i.id) ||
       r.source!=Text(Ipv4Address(i.source)) || std::stoul(r.ttl)!=unsigned(i.ttl)+1))return;
    auto key=HopKey(n,p,i); auto found=m_bindings.find(key);
    if(found!=m_bindings.end() && (found->second.route.next!=r.next || found->second.iface!=int32_t(iface)))
      found->second.ambiguous=true;
    else if(found==m_bindings.end())m_bindings.emplace(key,Binding{r,int32_t(iface),false});
  }
  void Write(const char* event,uint32_t n,int32_t iface,int32_t device,Ptr<const Packet> p,const Info& i,
             int32_t code=-1,std::string reason="not_applicable",const WifiMacHeader* mac=nullptr) {
    int64_t now=Simulator::Now().GetNanoSeconds();if(now>m_end){++m_after;return;}
    std::string next="null", route="null", evidence=m_ca?"no_matching_IPv4_Tx":"CA_audit_disabled";
    std::string state="unknown",expired="null",macNext="null",agreement="unknown";
    auto binding=i.ip?m_bindings.find(HopKey(n,p,i)):m_bindings.end();
    if(binding!=m_bindings.end()) {
      const auto& b=binding->second;
      if(b.ambiguous)evidence="ambiguous_IPv4_Tx";
      else {next=b.route.next;route=Text(b.route.index);evidence="matched_IPv4_Tx";if(iface<0)iface=b.iface;}
    }
    if(next!="null" && iface>=0) {
      auto c=m_nodes.at(n).caches.find(iface);
      if(c!=m_nodes.at(n).caches.end()) {
        // ArpCache::Lookup is only map.find; never invoke ArpL3Protocol::Lookup.
        auto entry=c->second->Lookup(Ipv4Address(next.c_str()));
        if(!entry)state="missing";
        else {
          state=entry->IsWaitReply()?"wait_reply":entry->IsDead()?"dead":entry->IsPermanent()?"permanent":
                entry->IsAutoGenerated()?"auto_generated":entry->IsAlive()?"alive":"unknown";
          expired=entry->IsExpired()?"1":"0";
        }
      }
    }
    if(std::string(event)=="arp_l3_drop") {
      reason=!i.ip?"arp_non_ipv4_drop_unclassified":evidence!="matched_IPv4_Tx"?"arp_ipv4_drop_unknown_next_hop":
        state=="dead"&&expired=="0"?"arp_dead_entry":state=="wait_reply"&&expired=="0"?
        "arp_pending_queue_full":"arp_ipv4_drop_unknown_cache_state";
    }
    if(mac) {
      auto known=m_macIps.find(Text(mac->GetAddr1()));
      if(known!=m_macIps.end() && known->second.size()==1)macNext=*known->second.begin();
      if(macNext!="null" && next!="null")agreement=macNext==next?"match":"different";
    }
    ++m_counts[event];++m_reasons[reason];
    m_events<<m_eventCount++<<','<<now<<','<<event<<','<<n<<','<<(iface<0?"null":Text(iface))<<','
      <<(device<0?"null":Text(device))<<','<<p->GetUid()<<','<<p->GetSize()<<','<<i.framing<<',';
    if(i.ip)m_events<<Ipv4Address(i.source)<<','<<Ipv4Address(i.destination)<<','<<unsigned(i.protocol)<<','<<i.id
      <<','<<unsigned(i.ttl)<<','<<i.fragment<<','<<i.more<<','<<i.ipBytes;
    else m_events<<"null,null,null,null,null,null,null,null";
    m_events<<','<<i.sport<<','<<i.dport<<','<<(code<0?"null":Text(code))<<','<<reason<<','<<next<<','<<route<<','
      <<evidence<<','<<state<<','<<expired<<',';
    if(mac)m_events<<mac->GetAddr1()<<','<<mac->GetAddr2()<<','<<mac->GetSequenceNumber()<<','
      <<unsigned(mac->GetFragmentNumber())<<','<<mac->IsRetry();
    else m_events<<"null,null,null,null,null";
    m_events<<','<<macNext<<','<<agreement<<','<<i.prefix<<'\n';
  }
  void MacEvent(const char* event,uint32_t n,uint32_t d,Ptr<const Packet> p,int32_t code=-1,
                const std::string& reason="not_applicable",const WifiMacHeader* mac=nullptr) {
    Write(event,n,m_nodes.at(n).wifi.at(d).iface,d,p,Decode(p,true),code,reason,mac);
  }
  static void IpTx(PacketLossObservationWriter* o,uint32_t n,Ptr<const Packet> p,Ptr<Ipv4>,uint32_t i) {
    auto info=Decode(p,false);o->Bind(n,i,p,info);o->Write("ipv4_tx",n,i,-1,p,info);
  }
  static void IpRx(PacketLossObservationWriter* o,uint32_t n,Ptr<const Packet> p,Ptr<Ipv4>,uint32_t i) {
    o->Write("ipv4_rx",n,i,-1,p,Decode(p,false));
  }
  static void IpDrop(PacketLossObservationWriter* o,uint32_t n,const Ipv4Header& h,Ptr<const Packet> p,
                     Ipv4L3Protocol::DropReason reason,Ptr<Ipv4>,uint32_t i) {
    const char* names[]={"invalid","ipv4_ttl_expired","ipv4_no_route","ipv4_bad_checksum","ipv4_interface_down",
      "ipv4_route_error","ipv4_fragment_timeout","ipv4_duplicate"};
    unsigned r=unsigned(reason);o->Write("ipv4_drop",n,i,-1,p,Decode(p,false,&h),r,r<8?names[r]:"ipv4_unknown_reason");
  }
  static void ArpDrop(PacketLossObservationWriter* o,uint32_t n,Ptr<const Packet> p) {
    o->Write("arp_l3_drop",n,-1,-1,p,Decode(p,false));
  }
  static void CacheDrop(PacketLossObservationWriter* o,uint32_t n,uint32_t i,Ptr<const Packet> p) {
    o->Write("arp_cache_drop",n,i,-1,p,Decode(p,false),-1,"arp_pending_retry_exhausted");
  }
  static void MacTx(PacketLossObservationWriter* o,uint32_t n,uint32_t d,Ptr<const Packet> p) {o->MacEvent("mac_tx",n,d,p);}
  static void MacRx(PacketLossObservationWriter* o,uint32_t n,uint32_t d,Ptr<const Packet> p) {o->MacEvent("mac_rx",n,d,p);}
  static void MacTxDrop(PacketLossObservationWriter* o,uint32_t n,uint32_t d,Ptr<const Packet> p) {
    o->MacEvent("mac_tx_drop",n,d,p,-1,"mac_tx_rejected_unspecified");
  }
  static void Enqueue(PacketLossObservationWriter* o,uint32_t n,uint32_t d,Ptr<const WifiMpdu> p) {
    o->MacEvent("mac_enqueue",n,d,p->GetPacket(),-1,"not_applicable",&p->GetHeader());
  }
  static void Acked(PacketLossObservationWriter* o,uint32_t n,uint32_t d,Ptr<const WifiMpdu> p) {
    o->MacEvent("mac_acked",n,d,p->GetPacket(),-1,"hop_acked",&p->GetHeader());
  }
  static void MpduDrop(PacketLossObservationWriter* o,uint32_t n,uint32_t d,WifiMacDropReason r,Ptr<const WifiMpdu> p) {
    const char* names[]={"mac_failed_enqueue","mac_expired_lifetime","mac_reached_retry_limit","mac_qos_old_packet"};
    o->MacEvent("mac_mpdu_drop",n,d,p->GetPacket(),unsigned(r),unsigned(r)<4?names[unsigned(r)]:"mac_unknown_reason",&p->GetHeader());
  }
  static void Timeout(PacketLossObservationWriter* o,uint32_t n,uint32_t d,uint8_t r,Ptr<const WifiMpdu> p,const WifiTxVector&) {
    o->MacEvent("mac_response_timeout",n,d,p->GetPacket(),r,"response_timeout_not_terminal",&p->GetHeader());
  }

  std::filesystem::path m_dir; int64_t m_end; bool m_ca, m_finalized=false,m_disconnected=false,m_hadLogic=false;
  std::ofstream m_events,m_routes; Tap m_tap; std::streambuf* m_oldClog=nullptr; LogComponent* m_log=nullptr;
  uint64_t m_eventCount=0,m_routeCount=0,m_errors=0,m_after=0;
  std::map<uint32_t,NodeState> m_nodes;std::map<std::string,std::set<std::string>> m_macIps;
  std::map<std::pair<uint32_t,uint64_t>,Witness> m_latest;std::map<Key,Binding> m_bindings;
  std::map<std::string,uint64_t> m_counts,m_reasons;
};
} // namespace ns3::cafbcr
#endif
