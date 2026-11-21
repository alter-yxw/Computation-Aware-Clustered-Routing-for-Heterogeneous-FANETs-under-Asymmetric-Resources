/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_NETWORK_OBSERVER_H
#define CA_FBCR_NETWORK_OBSERVER_H

#include "ca-fbcr-phy-completion-observer.h"
#include "ns3/abort.h"
#include "ns3/inet-socket-address.h"
#include "ns3/ipv4-l3-protocol.h"
#include "ns3/loopback-net-device.h"
#include "ns3/seq-ts-header.h"
#include "ns3/simulator.h"
#include "ns3/udp-client.h"
#include "ns3/udp-server.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-psdu.h"
#include <algorithm>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <tuple>
#include <vector>

namespace ns3::cafbcr {

// CA entrypoint only. No Packet construction/Copy/GetPacket-on-PSDU, tags,
// RNG, events or protocol feedback. The native IPv4 Tx trace makes a copy;
// its UID is preserved. WifiPsdu::GetPacket would allocate a NEW UID even
// for a single MPDU, so only existing MPDU payloads and size getters are read.
class NetworkObservationWriter
{
public:
  static constexpr const char* SCHEMA = "cafbcr-network-observation-v1";
  struct HopCounts { uint64_t delivered = 0, observed = 0, unknown = 0, sum = 0, duplicates = 0; };
  struct Cost { uint64_t packets = 0, bytes = 0, retries = 0, nominalNs = 0; };

  NetworkObservationWriter(const std::string& directory, Time begin, Time end,
                           uint16_t controlPort, bool caMessages = false, bool observeCompletion = false)
    : m_dir(directory), m_begin(begin.GetNanoSeconds()), m_end(end.GetNanoSeconds()),
      m_controlPort(controlPort), m_ca(caMessages), m_ip(m_dir / "network-ip.csv"),
      m_phy(m_dir / "network-phy.csv"), m_hops(m_dir / "udp-hops.csv")
  {
    NS_ABORT_MSG_IF(!m_ip || !m_phy || !m_hops, "Cannot open network observations");
    m_ip << "event,time_ns,node,interface,packet_uid,source_ip,destination_ip,protocol,ip_id,ttl,"
            "fragment_offset,more_fragments,ip_bytes,category,source_port,destination_port,message_type,first_source_tx,packet_prefix_hex\n";
    m_phy << "event_index,time_ns,node,device,psdu_count,mpdu_count,psdu_bytes,category,retry,"
             "nominal_duration_ns,full_window_ns,udp_window_ns,packet_uid,source_ip,destination_ip,"
             "protocol,ip_id,source_port,destination_port,message_type,ip_bytes,mac_header_bytes,payload_bytes,"
             "mode,channel_width_mhz,preamble,band,payload_prefix_hex\n";
    m_hops << "flow_index,packet_uid,rx_ns,tx_ns,first_eligible,in_window,origin_ttl,receive_ttl,"
              "ip_id,logical_hops,status\n";
    if (observeCompletion) m_completion=std::make_unique<PhyCompletionObserver>(directory,begin,end);
  }

  void AttachIp(uint32_t index, Ptr<Node> node)
  {
    auto ip = node->GetObject<Ipv4L3Protocol>();
    NS_ABORT_MSG_IF(!ip || m_nodes.count(index), "Invalid network observer attachment");
    m_nodes[index] = ip;
    NS_ABORT_MSG_IF(!ip->TraceConnectWithoutContext("Tx", MakeBoundCallback(&IpTx, this, index)) ||
      !ip->TraceConnectWithoutContext("UnicastForward", MakeBoundCallback(&Forward, this, index)) ||
      !ip->TraceConnectWithoutContext("LocalDeliver", MakeBoundCallback(&Deliver, this, index)),
      "IPv4 observation traces unavailable");
  }

  void AttachWifi(uint32_t index, Ptr<Node> node)
  {
    for (uint32_t d = 0; d < node->GetNDevices(); ++d) {
      auto wifi = DynamicCast<WifiNetDevice>(node->GetDevice(d));
      if (!wifi) continue;
      NS_ABORT_MSG_IF(wifi->GetNPhys() != 1, "Network observation supports one PHY per Wi-Fi device");
      auto phy = wifi->GetPhy();
      NS_ABORT_MSG_IF(!phy->TraceConnectWithoutContext("PhyTxPsduBegin",
        MakeBoundCallback(&Phy, this, index, d, phy)), "PSDU observation trace unavailable");
      m_phys.emplace_back(index, d, phy);
      if (m_completion) m_completion->Attach(index,d,phy);
    }
  }

  void AttachUdp(uint32_t index, uint32_t source, uint32_t destination, Ipv4Address sourceIp,
                 Ipv4Address destinationIp, uint16_t port, Ptr<UdpClient> client, Ptr<UdpServer> server)
  {
    NS_ABORT_MSG_IF(m_flows.count(index) || !client || !server, "Invalid hop flow attachment");
    m_flows[index] = Flow{source, destination, sourceIp, destinationIp, port, client, server, {}};
    NS_ABORT_MSG_IF(!client->TraceConnectWithoutContext("TxWithAddresses", MakeBoundCallback(&AppTx, this, index)) ||
      !server->TraceConnectWithoutContext("RxWithAddresses", MakeBoundCallback(&AppRx, this, index)),
      "UDP hop traces unavailable");
  }

  HopCounts GetHopCounts() const { return m_hopCount; }
  PhyCompletionObserver::Counts GetPhyCompletionCounts() const {
    return m_completion?m_completion->GetCounts():PhyCompletionObserver::Counts{};
  }

  void Finalize()
  {
    NS_ABORT_MSG_IF(m_finalized, "Network observations already finalized");
    m_finalized = true;
    for (auto& [index, ip] : m_nodes) {
      ip->TraceDisconnectWithoutContext("Tx", MakeBoundCallback(&IpTx, this, index));
      ip->TraceDisconnectWithoutContext("UnicastForward", MakeBoundCallback(&Forward, this, index));
      ip->TraceDisconnectWithoutContext("LocalDeliver", MakeBoundCallback(&Deliver, this, index));
    }
    for (auto& [index, device, phy] : m_phys)
      phy->TraceDisconnectWithoutContext("PhyTxPsduBegin", MakeBoundCallback(&Phy, this, index, device, phy));
    if (m_completion) m_completion->Finalize();
    for (auto& [index, f] : m_flows) {
      f.client->TraceDisconnectWithoutContext("TxWithAddresses", MakeBoundCallback(&AppTx, this, index));
      f.server->TraceDisconnectWithoutContext("RxWithAddresses", MakeBoundCallback(&AppRx, this, index));
    }
    std::ofstream costs(m_dir / "network-costs.csv");
    NS_ABORT_MSG_IF(!costs, "Cannot open network cost totals");
    costs << "window,layer,category,message_type,packets,bytes,retry_candidates,nominal_airtime_ns,kbps\n";
    for (const auto& [key, value] : m_cost) {
      const auto& [window, layer, category, type] = key;
      int64_t duration = window == "full" ? m_end : m_end - m_begin;
      costs << window << ',' << layer << ',' << category << ',' << unsigned(type) << ',' << value.packets << ','
            << value.bytes << ',' << value.retries << ',' << value.nominalNs << ','
            << (duration > 0 ? Number(value.bytes * 8.0 * 1e6 / duration) : "null") << '\n';
    }
    std::ofstream summary(m_dir / "network-summary.json");
    NS_ABORT_MSG_IF(!summary, "Cannot open network summary");
    const bool complete = Simulator::Now().GetNanoSeconds() >= m_end;
    const char* status = !complete ? "incomplete_window" : m_errors ? "invalid" : m_end <= m_begin ? "empty_window" :
      !m_hopCount.delivered ? "no_deliveries" : m_hopCount.unknown ? "partial_hops" : "observed";
    summary << "{\"schema\":\"" << SCHEMA << "\",\"formal_eligible\":false,"
      << "\"measurement_valid\":" << (complete && !m_errors ? "true" : "false")
      << ",\"status\":\"" << status << "\",\"window_complete\":" << (complete ? "true" : "false")
      << ",\"window_begin_ns\":" << m_begin << ",\"window_end_ns\":" << m_end
      << ",\"errors\":" << m_errors << ",\"unique_delivered\":" << m_hopCount.delivered
      << ",\"hop_observed\":" << m_hopCount.observed << ",\"hop_unknown\":" << m_hopCount.unknown
      << ",\"hop_sum\":" << m_hopCount.sum << ",\"duplicates\":" << m_hopCount.duplicates
      << ",\"hop_mean\":" << (complete && !m_errors && !m_hopCount.unknown && m_hopCount.observed
                                   ? Number(double(m_hopCount.sum) / m_hopCount.observed) : "null")
      << ",\"observed_subset_hop_mean\":" << (m_hopCount.observed ? Number(double(m_hopCount.sum) / m_hopCount.observed) : "null")
      << ",\"hop_definition\":\"first_eligible_unique_UDP_receipt; source_physical_TTL_minus_delivery_TTL_plus_one; self_zero\""
      << ",\"fragment_hops\":\"unavailable; reassembly may combine different fragment paths\""
      << ",\"ip_cost\":\"non_loopback_IPv4_Tx_including_per_hop_fragments; excludes_ARP_MAC_PHY_retries\""
      << ",\"phy_cost\":\"PhyTxPsduBegin_candidates; PSDU_bytes_include_MAC_FCS; retries_separate; nominal_duration_not_completed_airtime\""
      << ",\"phy_completion_observed\":false,\"phy_window_rule\":\"bytes_by_start_in_half_open_window; nominal_duration_clipped_to_window\""
      << ",\"mac_ACK_RTS_CTS_attribution\":\"mac_control_separate_not_assigned_to_routing\""
      << ",\"control_udp_port\":" << m_controlPort << ",\"wifi_devices\":" << m_phys.size() << "}\n";
    costs.close(); summary.close(); m_ip.close(); m_phy.close(); m_hops.close();
    NS_ABORT_MSG_IF(!costs || !summary || !m_ip || !m_phy || !m_hops, "Cannot write network observations");
  }

private:
  struct Info {
    Ipv4Address source, destination;
    uint16_t id = 0, sourcePort = 0, destinationPort = 0, fragment = 0;
    uint32_t bytes = 0;
    uint8_t protocol = 0, ttl = 0, type = 0;
    bool more = false;
    std::string category = "unclassified_ipv4";
  };
  using Key = std::tuple<uint64_t, uint32_t, uint32_t, uint8_t, uint16_t>;
  struct Origin { uint8_t ttl; bool fragmented; };
  struct Attempt { int64_t time; uint16_t sourcePort; uint32_t bytes; bool eligible = false; };
  struct Flow {
    uint32_t source, destination;
    Ipv4Address sourceIp, destinationIp;
    uint16_t port;
    Ptr<UdpClient> client;
    Ptr<UdpServer> server;
    std::map<uint64_t, Attempt> attempts;
  };

  static std::string Number(double value) { std::ostringstream s; s << std::setprecision(17) << value; return s.str(); }
  static std::string IpText(Ipv4Address ip, bool known) {
    if (!known) return "null";
    std::ostringstream s; s << ip; return s.str();
  }
  static std::string Prefix(Ptr<const Packet> p, uint32_t limit=48) {
    std::array<uint8_t, 160> bytes{};
    uint32_t n = p->CopyData(bytes.data(), std::min<uint32_t>({limit,uint32_t(bytes.size()),p->GetSize()}));
    std::ostringstream s;
    for (uint32_t j = 0; j < n; ++j) s << std::hex << std::setfill('0') << std::setw(2) << unsigned(bytes[j]);
    return s.str();
  }
  static uint16_t U16(const uint8_t* p) { return (uint16_t(p[0]) << 8) | p[1]; }
  static uint32_t U32(const uint8_t* p) { return (uint32_t(U16(p)) << 16) | U16(p + 2); }
  static Key Identity(uint64_t uid, const Info& i) { return {uid, i.source.Get(), i.destination.Get(), i.protocol, i.id}; }
  bool Window(int64_t now) const { return now >= m_begin && now < m_end; }
  static bool Fragmented(const Info& i) { return i.more || i.fragment; }

  void Transport(Info& i, const uint8_t* data, uint32_t size, uint64_t uid)
  {
    if (i.fragment) {
      auto it = m_classification.find(Identity(uid, i));
      if (it != m_classification.end()) {
        i.category = it->second.category; i.type = it->second.type;
        i.sourcePort = it->second.sourcePort; i.destinationPort = it->second.destinationPort;
      } else i.category = "unclassified_fragment";
      return;
    }
    i.category = i.protocol == 1 ? "icmp" : "other_ip";
    if (i.protocol != 17) return;
    if (size < 8) { i.category = "unclassified_udp"; return; }
    i.sourcePort = U16(data); i.destinationPort = U16(data + 2);
    if (i.sourcePort == m_controlPort || i.destinationPort == m_controlPort) {
      i.category = "routing_control";
      if (m_ca && size > 8 && data[8] >= 1 && data[8] <= 10) i.type = data[8];
    } else if (i.destinationPort == 9999) i.category = "task_request";
    else if (i.sourcePort == 9999) i.category = "task_completion";
    else {
      i.category = "other_udp";
      for (const auto& [index, f] : m_flows) {
        if (i.source == f.sourceIp && i.destination == f.destinationIp && i.destinationPort == f.port) {
          i.category = "udp_business"; break;
        }
      }
    }
    m_classification.emplace(Identity(uid, i), i);
  }

  Info FromHeader(const Ipv4Header& h, Ptr<const Packet> packet)
  {
    Info i;
    i.source = h.GetSource(); i.destination = h.GetDestination(); i.id = h.GetIdentification();
    i.protocol = h.GetProtocol(); i.ttl = h.GetTtl(); i.fragment = h.GetFragmentOffset();
    i.more = !h.IsLastFragment(); i.bytes = h.GetSerializedSize() + packet->GetSize();
    std::array<uint8_t, 16> data{};
    uint32_t size = packet->CopyData(data.data(), std::min<uint32_t>(data.size(), packet->GetSize()));
    Transport(i, data.data(), size, packet->GetUid());
    return i;
  }

  Info FromBytes(Ptr<const Packet> packet, uint32_t offset = 0)
  {
    Info i;
    std::array<uint8_t, 96> data{};
    uint32_t n = packet->CopyData(data.data(), std::min<uint32_t>(data.size(), packet->GetSize()));
    if (n < offset + 20 || data[offset] >> 4 != 4) return i;
    const auto* p = data.data() + offset;
    uint32_t ihl = (p[0] & 15) * 4;
    if (ihl < 20 || n < offset + ihl || U16(p + 2) < ihl || U16(p + 2) > packet->GetSize() - offset) return i;
    i.source = Ipv4Address(U32(p + 12)); i.destination = Ipv4Address(U32(p + 16));
    i.id = U16(p + 4); i.protocol = p[9]; i.ttl = p[8]; i.bytes = U16(p + 2);
    i.fragment = (U16(p + 6) & 0x1fff) * 8; i.more = U16(p + 6) & 0x2000;
    Transport(i, p + ihl, n - offset - ihl, packet->GetUid());
    return i;
  }

  void RecordIp(const char* event, uint32_t node, uint32_t interface, Ptr<const Packet> packet,
                const Info& i, bool origin = false)
  {
    m_ip << event << ',' << Simulator::Now().GetNanoSeconds() << ',' << node << ',' << interface << ','
      << packet->GetUid() << ',' << i.source << ',' << i.destination << ',' << unsigned(i.protocol) << ','
      << i.id << ',' << unsigned(i.ttl) << ',' << i.fragment << ',' << i.more << ',' << i.bytes << ','
      << i.category << ',' << i.sourcePort << ',' << i.destinationPort << ',' << unsigned(i.type) << ',' << origin << ','
      << Prefix(packet,i.type==8?160:((i.type==9 || i.type==10)?64:48)) << '\n';
  }

  static void IpTx(NetworkObservationWriter* s, uint32_t node, Ptr<const Packet> packet, Ptr<Ipv4> ip, uint32_t interface)
  {
    if (DynamicCast<LoopbackNetDevice>(ip->GetNetDevice(interface))) return;
    auto i = s->FromBytes(packet);
    bool source = false;
    for (uint32_t f = 0; f < ip->GetNInterfaces(); ++f)
      for (uint32_t a = 0; a < ip->GetNAddresses(f); ++a) source |= ip->GetAddress(f, a).GetLocal() == i.source;
    auto key = Identity(packet->GetUid(), i);
    bool first = source && s->m_origins.emplace(key, Origin{i.ttl, Fragmented(i)}).second;
    if (source && Fragmented(i)) s->m_origins.at(key).fragmented = true;
    if (!i.bytes) ++s->m_errors;
    s->RecordIp("tx", node, interface, packet, i, first);
    s->AddCost("ipv4_tx", i.category, i.type, i.bytes, false, 0);
  }
  static void Forward(NetworkObservationWriter* s, uint32_t node, const Ipv4Header& h, Ptr<const Packet> p, uint32_t interface)
  { s->RecordIp("forward", node, interface, p, s->FromHeader(h, p)); }
  static void Deliver(NetworkObservationWriter* s, uint32_t node, const Ipv4Header& h, Ptr<const Packet> p, uint32_t interface)
  {
    auto i = s->FromHeader(h, p);
    s->m_deliveries[{node, p->GetUid()}] = i;
    s->RecordIp("deliver", node, interface, p, i);
  }

  static void AppTx(NetworkObservationWriter* s, uint32_t index, Ptr<const Packet> p, const Address& from, const Address&)
  {
    uint16_t port = InetSocketAddress::IsMatchingType(from) ? InetSocketAddress::ConvertFrom(from).GetPort() : 0;
    if (!s->m_flows.at(index).attempts.emplace(p->GetUid(), Attempt{Simulator::Now().GetNanoSeconds(), port, p->GetSize() + 12}).second) ++s->m_errors;
  }
  static void AppRx(NetworkObservationWriter* s, uint32_t index, Ptr<const Packet> p, const Address& from, const Address& local)
  {
    auto& f = s->m_flows.at(index);
    int64_t now = Simulator::Now().GetNanoSeconds();
    auto attempt = f.attempts.find(p->GetUid());
    auto delivery = s->m_deliveries.find({f.destination, p->GetUid()});
    bool valid = attempt != f.attempts.end() && delivery != s->m_deliveries.end() &&
      InetSocketAddress::IsMatchingType(from) && InetSocketAddress::IsMatchingType(local);
    if (valid) {
      auto peer = InetSocketAddress::ConvertFrom(from), target = InetSocketAddress::ConvertFrom(local);
      valid = peer.GetIpv4() == f.sourceIp && peer.GetPort() == attempt->second.sourcePort &&
        target.GetPort() == f.port && delivery->second.source == f.sourceIp && delivery->second.destination == f.destinationIp &&
        p->GetSize() == attempt->second.bytes && now >= attempt->second.time;
      if (valid) {
        SeqTsHeader seq;
        valid = p->GetSize() >= 12 && p->PeekHeader(seq) == 12 && seq.GetTs().GetNanoSeconds() == attempt->second.time;
      }
    }
    bool inWindow = attempt != f.attempts.end() && s->Window(now) && s->Window(attempt->second.time);
    bool first = valid && inWindow && !attempt->second.eligible;
    std::optional<uint32_t> hop, ttl;
    std::string status = !valid ? "unmatched_delivery" : !inWindow ? "outside_window" : !first ? "duplicate" : "missing_origin";
    if (!valid) ++s->m_errors;
    if (valid && inWindow && !first) ++s->m_hopCount.duplicates;
    if (first) {
      attempt->second.eligible = true;
      ++s->m_hopCount.delivered;
      const auto& i = delivery->second;
      auto origin = s->m_origins.find(Identity(p->GetUid(), i));
      if (f.source == f.destination) { hop = 0; status = "observed_self"; }
      else if (origin != s->m_origins.end()) {
        ttl = origin->second.ttl;
        if (origin->second.fragmented || Fragmented(i)) status = "fragment_path_unavailable";
        else if (i.ttl > *ttl) status = "ttl_increased";
        else { hop = *ttl - i.ttl + 1; status = "observed"; }
      }
      if (hop) { ++s->m_hopCount.observed; s->m_hopCount.sum += *hop; }
      else ++s->m_hopCount.unknown;
    }
    s->m_hops << index << ',' << p->GetUid() << ',' << now << ','
      << (attempt != f.attempts.end() ? std::to_string(attempt->second.time) : "null") << ',' << first << ',' << inWindow << ','
      << (ttl ? std::to_string(*ttl) : "null") << ','
      << (delivery != s->m_deliveries.end() ? std::to_string(delivery->second.ttl) : "null") << ','
      << (delivery != s->m_deliveries.end() ? std::to_string(delivery->second.id) : "null") << ','
      << (hop ? std::to_string(*hop) : "null") << ',' << status << '\n';
  }

  static int64_t Overlap(int64_t time, int64_t duration, int64_t begin, int64_t end)
  { return std::max<int64_t>(0, std::min(time + duration, end) - std::max(time, begin)); }
  void AddCost(const std::string& layer, const std::string& category, uint8_t type,
               uint64_t bytes, bool retry, int64_t nominalNs)
  {
    int64_t now = Simulator::Now().GetNanoSeconds();
    for (const std::string window : {"full", "udp"}) {
      int64_t begin = window == "full" ? 0 : m_begin;
      auto& c = m_cost[{window, layer, category, type}];
      if (now >= begin && now < m_end) { ++c.packets; c.bytes += bytes; c.retries += retry; }
      c.nominalNs += Overlap(now, nominalNs, begin, m_end);
    }
  }

  static void Phy(NetworkObservationWriter* s, uint32_t node, uint32_t device, Ptr<WifiPhy> phy,
                  WifiConstPsduMap psdus, WifiTxVector vector, double)
  {
    uint64_t bytes = 0, mpdus = 0;
    for (auto& [station, psdu] : psdus) { bytes += psdu->GetSize(); mpdus += psdu->GetNMpdus(); }
    // v1 attributes only a single, non-aggregated MPDU PPDU. Anything else is
    // retained as an explicit unclassified aggregate, never proportionally split.
    std::string category = "unclassified_aggregate";
    Info info;
    uint64_t uid = 0, payload = 0, mac = 0;
    std::string prefix;
    bool retry = false;
    if (psdus.size() == 1 && mpdus == 1 && !psdus.begin()->second->IsAggregate()) {
      auto mpdu = *psdus.begin()->second->begin();
      const auto& h = mpdu->GetHeader();
      auto p = mpdu->GetPacket(); // existing MPDU payload, no packet allocation
      uid = p->GetUid(); payload = p->GetSize(); mac = h.GetSerializedSize(); retry = h.IsRetry();
      prefix = Prefix(p);
      category = h.IsCtl() ? "mac_control" : h.IsMgt() ? "management" : "unclassified_data";
      if (h.IsData() && !h.IsMoreFragments() && !h.GetFragmentNumber() && !(h.IsQosData() && h.IsQosAmsdu())) {
        std::array<uint8_t, 8> llc{};
        if (p->GetSize() >= 8 && p->CopyData(llc.data(), 8) == 8 && llc[0] == 0xaa && llc[1] == 0xaa && llc[2] == 3) {
          uint16_t ether = U16(llc.data() + 6);
          if (ether == 0x0806) category = "arp";
          else if (ether == 0x0800) { info = s->FromBytes(p, 8); category = info.category; }
        }
      }
    }
    int64_t now = Simulator::Now().GetNanoSeconds();
    int64_t duration = WifiPhy::CalculateTxDuration(psdus, vector, phy->GetPhyBand()).GetNanoSeconds();
    if (s->m_completion)
      s->m_completion->Candidate(s->m_phyIndex,node,device,psdus,duration,category,info.type,bytes,retry);
    s->AddCost("phy_candidate", category, info.type, bytes, retry, duration);
    s->m_phy << s->m_phyIndex++ << ',' << now << ',' << node << ',' << device << ',' << psdus.size() << ','
      << mpdus << ',' << bytes << ',' << category << ',' << retry << ',' << duration << ','
      << Overlap(now, duration, 0, s->m_end) << ',' << Overlap(now, duration, s->m_begin, s->m_end) << ','
      << uid << ',' << IpText(info.source, info.bytes != 0) << ',' << IpText(info.destination, info.bytes != 0) << ',' << unsigned(info.protocol) << ',' << info.id << ','
      << info.sourcePort << ',' << info.destinationPort << ',' << unsigned(info.type) << ',' << info.bytes << ',' << mac << ',' << payload << ','
      << vector.GetMode(psdus.begin()->first).GetUniqueName() << ',' << vector.GetChannelWidth() << ','
      << unsigned(vector.GetPreambleType()) << ',' << unsigned(phy->GetPhyBand()) << ',' << prefix << '\n';
  }

  std::filesystem::path m_dir;
  std::unique_ptr<PhyCompletionObserver> m_completion;
  int64_t m_begin, m_end;
  uint16_t m_controlPort;
  bool m_ca, m_finalized = false;
  std::ofstream m_ip, m_phy, m_hops;
  uint64_t m_errors = 0, m_phyIndex = 0;
  HopCounts m_hopCount;
  std::map<uint32_t, Ptr<Ipv4L3Protocol>> m_nodes;
  std::vector<std::tuple<uint32_t, uint32_t, Ptr<WifiPhy>>> m_phys;
  std::map<uint32_t, Flow> m_flows;
  std::map<Key, Origin> m_origins;
  std::map<Key, Info> m_classification;
  std::map<std::pair<uint32_t, uint64_t>, Info> m_deliveries;
  std::map<std::tuple<std::string, std::string, std::string, uint8_t>, Cost> m_cost;
};
} // namespace ns3::cafbcr
#endif
