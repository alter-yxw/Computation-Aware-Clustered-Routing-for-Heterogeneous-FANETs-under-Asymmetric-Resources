/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef CA_FBCR_PHY_COMPLETION_OBSERVER_H
#define CA_FBCR_PHY_COMPLETION_OBSERVER_H

#include "ns3/abort.h"
#include "ns3/simulator.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-phy-listener.h"
#include "ns3/wifi-phy-state-helper.h"
#include "ns3/wifi-psdu.h"
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <tuple>

namespace ns3::cafbcr {

// Passive CA-entrypoint observation. No Packet creation, RNG, simulation
// events, radio changes or protocol feedback. End's native GetProtocolDataUnit
// makes a UID-preserving Packet::Copy; this observer never calls that getter.
class PhyCompletionObserver
{
public:
  static constexpr const char* SCHEMA="cafbcr-phy-completion-v1";
  struct Counts {
    uint64_t candidates=0,completed=0,interrupted=0,open=0,indeterminate=0,unsupported=0;
    uint64_t completedBytes=0,localNs=0,unknownIntervals=0,errors=0;
  };
  PhyCompletionObserver(const std::string& directory,Time begin,Time end)
    :m_dir(directory),m_begin(begin.GetNanoSeconds()),m_end(end.GetNanoSeconds()),
     m_events(m_dir/"phy-completion-events.csv"),m_records(m_dir/"phy-completion.csv")
  {
    NS_ABORT_MSG_IF(!m_events || !m_records,"Cannot open PHY completion observations");
    m_events<<"event_index,time_ns,node,device,attempt_id,event,packet_uid,packet_bytes,duration_ns,phy_state\n";
    m_records<<"attempt_id,candidate_event_index,node,device,start_ns,nominal_duration_ns,psdu_count,mpdu_count,"
      "psdu_bytes,category,message_type,retry,tx_start_notified,ends_expected,ends_observed,terminal_ns,status,"
      "local_tx_ns,full_window_local_tx_ns,udp_window_local_tx_ns,full_window_nominal_ns,udp_window_nominal_ns,"
      "completed_before_cutoff,packet_members\n";
  }
  ~PhyCompletionObserver() { Detach(); }
  void Attach(uint32_t node,uint32_t device,Ptr<WifiPhy> phy)
  {
    const DeviceKey key{node,device};
    NS_ABORT_MSG_IF(m_done || m_devices.count(key),"Invalid PHY completion attachment");
    auto record=std::make_unique<Device>(this,key,phy);
    phy->RegisterListener(record.get());
    NS_ABORT_MSG_IF(!phy->TraceConnectWithoutContext("PhyTxEnd",MakeBoundCallback(&End,this,node,device)),
      "PHY completion end trace unavailable");
    m_devices.emplace(key,std::move(record));
  }
  void Candidate(uint64_t candidateIndex,uint32_t node,uint32_t device,WifiConstPsduMap psdus,
                 int64_t nominalNs,const std::string& category,uint8_t type,uint64_t bytes,bool retry)
  {
    auto& d=*m_devices.at({node,device});const auto now=Now();
    if (d.active) Close(d,"indeterminate_replaced_without_end",now,std::nullopt);
    Active a;a.id=m_attemptIndex++;a.candidateIndex=candidateIndex;a.start=now;a.nominal=nominalNs;
    a.psdus=psdus.size();a.bytes=bytes;a.category=category;a.type=type;a.retry=retry;
    for (const auto& [station,psdu]:psdus) for (const auto& mpdu:*psdu) {
      // Existing payload identity plus full MPDU size distinguishes local
      // members. A retry is another attempt even when these fields repeat.
      ++a.members[{mpdu->GetPacket()->GetUid(),mpdu->GetSize()}];++a.mpdus;
    }
    a.remaining=a.members;d.active=std::move(a);d.recentSyncOff.reset();
    if (now>=0 && now<m_end) ++m_counts.candidates;
    Event(d,"candidate",0,0,nominalNs);
  }
  Counts GetCounts() const { return m_counts; }
  void Finalize()
  {
    NS_ABORT_MSG_IF(m_done,"PHY completion already finalized");
    const auto cutoff=std::min(Now(),m_end);
    for (auto& [key,device]:m_devices) {
      Event(*device,"finalize");
      if (!device->active) continue;
      auto& a=*device->active;
      if (a.start+a.nominal<cutoff)
        Close(*device,"indeterminate_missing_end",cutoff,std::nullopt);
      else Close(*device,"open_at_cutoff",cutoff,a.started?
        std::optional<int64_t>(std::max<int64_t>(0,cutoff-a.start)):std::nullopt);
    }
    Detach();m_done=true;
    std::ofstream costs(m_dir/"phy-completion-costs.csv");
    costs<<"window,category,message_type,status,attempts_by_start,completed_frames_by_start,"
      "completed_psdu_bytes_by_start,observed_local_tx_ns,unknown_intervals\n";
    for (const auto& [key,c]:m_cost) {
      const auto& [window,category,type,status]=key;
      costs<<window<<','<<category<<','<<unsigned(type)<<','<<status<<','<<c.attempts<<','<<c.frames<<','
        <<c.bytes<<','<<c.localNs<<','<<c.unknown<<'\n';
    }
    const bool complete=Now()>=m_end;
    std::ofstream summary(m_dir/"phy-completion-summary.json");
    summary<<"{\"schema\":\""<<SCHEMA<<"\",\"formal_eligible\":false,\"window_complete\":"
      <<(complete?"true":"false")<<",\"measurement_valid\":"<<(complete && !m_counts.errors?"true":"false")
      <<",\"capture_end_ns\":"<<Now()<<",\"window_begin_ns\":"<<m_begin<<",\"window_end_ns\":"<<m_end
      <<",\"candidate_attempts\":"<<m_counts.candidates<<",\"completed_attempts\":"<<m_counts.completed
      <<",\"interrupted_attempts\":"<<m_counts.interrupted<<",\"open_at_cutoff\":"<<m_counts.open
      <<",\"indeterminate_attempts\":"<<m_counts.indeterminate<<",\"unsupported_attempts\":"<<m_counts.unsupported
      <<",\"completed_psdu_bytes\":"<<m_counts.completedBytes<<",\"observed_local_tx_ns\":"<<m_counts.localNs
      <<",\"unknown_intervals\":"<<m_counts.unknownIntervals<<",\"errors\":"<<m_counts.errors
      <<",\"interval_coverage_complete\":"<<(!m_counts.unknownIntervals?"true":"false")
      <<",\"completion_basis\":\"all_expected_MPDU_PhyTxEnd_events_for_one_local_attempt\""
      <<",\"interruption_basis\":\"observed_local_TX_start_to_OFF_sleep_switch; not_channel_receipt\""
      <<",\"channel_submission_independently_observed\":false,\"receiver_success_proven\":false"
      <<",\"aggregate_policy\":\"single_PSDU_can_complete_after_all_MPDU_ends; category_unsplit; multi_PSDU_unsupported\""
      <<",\"partial_bytes\":\"not_inferred; completed_bytes_only\",\"window_rule\":\"half_open_start; intervals_clipped; completion_end_le_cutoff\"}\n";
    m_events.close();m_records.close();costs.close();summary.close();
    NS_ABORT_MSG_IF(!m_events || !m_records || !costs || !summary,"Cannot write PHY completion observations");
  }
private:
  using DeviceKey=std::pair<uint32_t,uint32_t>;
  using MemberKey=std::pair<uint64_t,uint32_t>;
  struct Active {
    uint64_t id=0,candidateIndex=0,bytes=0,psdus=0,mpdus=0,ends=0;
    int64_t start=0,nominal=0;
    std::string category;uint8_t type=0;bool retry=false,started=false,invalid=false;
    std::map<MemberKey,uint32_t> members,remaining;
  };
  struct Device:WifiPhyListener {
    PhyCompletionObserver* owner;DeviceKey key;Ptr<WifiPhy> phy;
    std::optional<Active> active;
    std::optional<std::pair<uint64_t,int64_t>> recentSyncOff;
    Device(PhyCompletionObserver* o,DeviceKey k,Ptr<WifiPhy> p):owner(o),key(k),phy(p) {}
    void NotifyTxStart(Time duration,double) override { owner->Start(*this,duration.GetNanoSeconds()); }
    void NotifyOff() override { owner->Interrupt(*this,"off"); }
    void NotifySleep() override { owner->Interrupt(*this,"sleep"); }
    void NotifySwitchingStart(Time) override { owner->Interrupt(*this,"switch"); }
    void NotifyRxStart(Time) override {}
    void NotifyRxEndOk() override {}
    void NotifyRxEndError() override {}
    void NotifyCcaBusyStart(Time,WifiChannelListType,const std::vector<Time>&) override {}
    void NotifyWakeup() override {}
    void NotifyOn() override {}
  };
  struct Cost { uint64_t attempts=0,frames=0,bytes=0,localNs=0,unknown=0; };
  static int64_t Now() { return Simulator::Now().GetNanoSeconds(); }
  static int64_t Overlap(int64_t start,int64_t duration,int64_t begin,int64_t end)
  { return std::max<int64_t>(0,std::min(start+duration,end)-std::max(start,begin)); }
  void Event(const Device& d,const char* name,uint64_t uid=0,uint32_t bytes=0,int64_t duration=0,
             std::optional<uint64_t> attempt=std::nullopt)
  {
    m_events<<m_eventIndex++<<','<<Now()<<','<<d.key.first<<','<<d.key.second<<',';
    if (attempt) m_events<<*attempt;else if (d.active) m_events<<d.active->id;else m_events<<"null";
    m_events<<','<<name<<','<<uid<<','<<bytes<<','<<duration<<','<<int(d.phy->GetState()->GetState())<<'\n';
  }
  void Start(Device& d,int64_t duration)
  {
    if (!d.active) {
      if (d.recentSyncOff && d.recentSyncOff->second==Now())
        Event(d,"tx_start_after_sync_off",0,0,duration,d.recentSyncOff->first);
      else { Event(d,"orphan_tx_start",0,0,duration);++m_counts.errors; }
      return;
    }
    Event(d,"tx_start",0,0,duration);auto& a=*d.active;
    if (a.started || duration!=a.nominal || Now()!=a.start) {a.invalid=true;++m_counts.errors;}
    a.started=true;
  }
  void Interrupt(Device& d,const char* reason)
  {
    Event(d,reason);
    if (!d.active) return;
    const auto now=Now(),start=d.active->start;
    const auto id=d.active->id;
    if (now==start) {
      Close(d,std::string("interrupted_")+reason+"_synchronous",now,int64_t(0));
      d.recentSyncOff=std::make_pair(id,now);
    } else if (d.active->started && now<=start+d.active->nominal)
      Close(d,now>m_end?std::string("open_")+reason+"_after_cutoff":
        std::string("interrupted_")+reason,now,now-start);
    else Close(d,std::string("indeterminate_")+reason+"_without_matched_tx",now,std::nullopt);
  }
  static void End(PhyCompletionObserver* self,uint32_t node,uint32_t device,Ptr<const Packet> packet)
  {
    auto& d=*self->m_devices.at({node,device});
    self->Event(d,"end",packet->GetUid(),packet->GetSize());
    if (!d.active) {++self->m_counts.errors;return;}
    auto& a=*d.active;auto member=a.remaining.find({packet->GetUid(),packet->GetSize()});
    if (member==a.remaining.end() || !member->second) {a.invalid=true;++self->m_counts.errors;return;}
    if (!--member->second) a.remaining.erase(member);
    ++a.ends;
    if (!a.remaining.empty()) return;
    const auto now=Now();
    if (!a.started || a.invalid || now!=a.start+a.nominal)
      self->Close(d,"indeterminate_end_mismatch",now,std::nullopt);
    else if (a.psdus!=1) self->Close(d,"unsupported_multi_psdu",now,std::nullopt);
    else self->Close(d,now>self->m_end?"completed_after_cutoff":"completed",now,now-a.start);
  }
  void Close(Device& d,const std::string& status,int64_t terminal,std::optional<int64_t> local)
  {
    Event(d,("close_"+status).c_str());
    auto a=std::move(*d.active);d.active.reset();
    const auto cutoff=std::min(Now(),m_end);
    const bool eligible=a.start>=0 && a.start<m_end;
    const bool completed=status=="completed";
    const auto full=local?std::optional<int64_t>(Overlap(a.start,*local,0,cutoff)):std::nullopt;
    const auto udp=local?std::optional<int64_t>(Overlap(a.start,*local,m_begin,cutoff)):std::nullopt;
    if (eligible) {
      if (completed) {++m_counts.completed;m_counts.completedBytes+=a.bytes;}
      else if (status.rfind("interrupted_",0)==0) ++m_counts.interrupted;
      else if (status.rfind("open_",0)==0 || status=="completed_after_cutoff") ++m_counts.open;
      else if (status.rfind("unsupported_",0)==0) ++m_counts.unsupported;
      else ++m_counts.indeterminate;
      if (full) m_counts.localNs+=*full;else ++m_counts.unknownIntervals;
    }
    for (const std::string window:{"full","udp"}) {
      const auto begin=window=="full"?0:m_begin;
      auto& c=m_cost[{window,a.category,a.type,status}];
      if (a.start>=begin && a.start<m_end) {
        ++c.attempts;if (completed) {++c.frames;c.bytes+=a.bytes;}
        if (!local) ++c.unknown;
      }
      if (local) c.localNs+=Overlap(a.start,*local,begin,cutoff);
    }
    m_records<<a.id<<','<<a.candidateIndex<<','<<d.key.first<<','<<d.key.second<<','<<a.start<<','<<a.nominal<<','
      <<a.psdus<<','<<a.mpdus<<','<<a.bytes<<','<<a.category<<','<<unsigned(a.type)<<','<<a.retry<<','
      <<a.started<<','<<a.mpdus<<','<<a.ends<<','<<terminal<<','<<status<<',';
    if (local) m_records<<*local;else m_records<<"null";
    m_records<<',';if (full) m_records<<*full;else m_records<<"null";
    m_records<<',';if (udp) m_records<<*udp;else m_records<<"null";
    m_records<<','<<Overlap(a.start,a.nominal,0,m_end)<<','<<Overlap(a.start,a.nominal,m_begin,m_end)<<','
      <<completed<<',';
    bool first=true;for (const auto& [key,count]:a.members) for (uint32_t n=0;n<count;++n) {
      if (!first) m_records<<';';
      first=false;m_records<<key.first<<':'<<key.second;
    }
    m_records<<'\n';
  }
  void Detach()
  {
    if (m_detached) return;
    for (auto& [key,d]:m_devices) {
      d->phy->UnregisterListener(d.get());
      d->phy->TraceDisconnectWithoutContext("PhyTxEnd",MakeBoundCallback(&End,this,key.first,key.second));
    }
    m_detached=true;
  }
  std::filesystem::path m_dir;int64_t m_begin,m_end;
  std::ofstream m_events,m_records;bool m_done=false,m_detached=false;
  uint64_t m_attemptIndex=0,m_eventIndex=0;Counts m_counts;
  std::map<DeviceKey,std::unique_ptr<Device>> m_devices;
  std::map<std::tuple<std::string,std::string,uint8_t,std::string>,Cost> m_cost;
};
} // namespace ns3::cafbcr
#endif
