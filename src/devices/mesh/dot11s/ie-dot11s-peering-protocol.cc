/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008,2009 IITP RAS
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Authors: Kirill Andreev <andreev@iitp.ru>
 */

#include "ie-dot11s-peering-protocol.h"
namespace ns3
{
namespace dot11s
{
uint8_t
IePeeringProtocol::GetInformationSize () const
{
  return 1;
}
IePeeringProtocol::IePeeringProtocol () :
  m_protocol (0)
{
}
WifiElementId
IePeeringProtocol::ElementId () const
{
  return IE11S_MESH_PEERING_PROTOCOL_VERSION;
}
void
IePeeringProtocol::SerializeInformation (Buffer::Iterator i) const
{
  i.WriteU8 (m_protocol);
}
uint8_t
IePeeringProtocol::DeserializeInformation (Buffer::Iterator i, uint8_t length)
{
  Buffer::Iterator start = i;
  m_protocol = i.ReadU8 ();
  return i.GetDistanceFrom (start);
}
void
IePeeringProtocol::PrintInformation (std::ostream& os) const
{
  os << "peering protocol = " << m_protocol;
}
} // namespace dot11s
} //namespace ns3
