/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
#ifndef ICRA_RQUEUE_H
#define ICRA_RQUEUE_H

#include <vector>
#include "ns3/ipv4-routing-protocol.h"
#include "ns3/simulator.h"

namespace ns3{
namespace icra{


class QueueEntry{

public:
  typedef Ipv4RoutingProtocol::UnicastForwardCallback UnicastForwardCallback;
  typedef Ipv4RoutingProtocol::ErrorCallback ErrorCallback;

  // QueueEntry() : m_packet(nullptr), m_header(), m_ucb(),m_ecb(), m_expire(Simulator::Now())
  // {
  // }

  QueueEntry(Ptr<const Packet> pa = 0, Ipv4Header const & h = Ipv4Header (),
            UnicastForwardCallback ucb = UnicastForwardCallback (),
            ErrorCallback ecb = ErrorCallback (), Time exp = Simulator::Now ())
  : m_packet (pa),
    m_header (h),
    m_ucb (ucb),
    m_ecb (ecb),
    m_expire (exp + Simulator::Now ())
  {
    // Ensure the callback is not null.
    NS_ASSERT_MSG(!m_ucb.IsNull(), "UnicastForwardCallback cannot be null");
    NS_ASSERT_MSG(!m_ecb.IsNull(), "ErrorCallback cannot be null");
  }

  bool operator== (QueueEntry const & o) const
  {
    return ((m_packet == o.m_packet) && (m_header.GetDestination () == o.m_header.GetDestination ()) && (m_expire == o.m_expire));
  }

  UnicastForwardCallback GetUnicastForwardCallback () const    {  return m_ucb;    }
  void SetUnicastForwardCallback (UnicastForwardCallback ucb)  {  m_ucb = ucb;     }

  ErrorCallback GetErrorCallback () const                      {  return m_ecb;    }
  void SetErrorCallback (ErrorCallback ecb)                    {  m_ecb = ecb;     }

  Ptr<const Packet> GetPacket () const                         {  return m_packet; }
  void SetPacket (Ptr<const Packet> p)                         {  m_packet = p;    }

  Ipv4Header GetIpv4Header () const                            {  return m_header; }
  void SetIpv4Header (Ipv4Header h)                            {  m_header = h;    }

  void SetExpireTime (Time exp)                                {  m_expire = exp + Simulator::Now (); }
  Time GetExpireTime () const                                  {  return m_expire - Simulator::Now ();}

private:

  Ptr<const Packet> m_packet;  /// Data packet
  Ipv4Header m_header;  /// IP header
  UnicastForwardCallback m_ucb;  /// Unicast forward callback
  ErrorCallback m_ecb;  /// Error callback
  Time m_expire;  /// Expire time for queue entry
};



class RequestQueue{

public:
  RequestQueue (uint32_t maxLen, Time routeToQueueTimeout)
  : m_maxLen (maxLen),
    m_queueTimeout (routeToQueueTimeout)
  {
  }

  bool Enqueue(QueueEntry& entry);
  bool Dequeue (Ipv4Address dst, QueueEntry & entry);
  void DropPacketWithDst (Ipv4Address dst);/// Remove all packets with destination IP address dst
  bool Find (Ipv4Address dst);/// Finds whether a packet with destination dst exists in the queue
  uint32_t GetSize ();/// Number of entries

  ///\name Fields
  uint32_t GetMaxQueueLen () const      {    return m_maxLen;       }
  void SetMaxQueueLen (uint32_t len)    {    m_maxLen = len;        }

  Time GetQueueTimeout () const         {    return m_queueTimeout; }
  void SetQueueTimeout (Time t)         {    m_queueTimeout = t;    }

private:
  std::vector<QueueEntry> m_queue;

  void Purge ();  /// Remove all expired entries
  void Drop (QueueEntry en, std::string reason);  /// Notify that packet is dropped from queue by timeout

  uint32_t m_maxLen;    /// The maximum number of packets that we allow a routing protocol to buffer.

  Time m_queueTimeout;  /// The maximum period of time that a routing protocol is allowed to buffer a packet for, seconds.

};


} // namespace icra
} // namespace ns3
#endif /*ICRA_RQUEUE_H*/