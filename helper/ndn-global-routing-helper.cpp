/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * Copyright (c) 2011-2015  Regents of the University of California.
 *
 * This file is part of ndnSIM. See AUTHORS for complete list of ndnSIM authors and
 * contributors.
 *
 * ndnSIM is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * ndnSIM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ndnSIM, e.g., in COPYING.md file.  If not, see <http://www.gnu.org/licenses/>.
 **/

#if __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wunneeded-internal-declaration"
#endif

#include "ndn-global-routing-helper.hpp"

#include "model/ndn-l3-protocol.hpp"
#include "helper/ndn-fib-helper.hpp"
#include "model/ndn-net-device-link-service.hpp"
#include "model/ndn-global-router.hpp"

#include "daemon/table/fib.hpp"
#include "daemon/fw/forwarder.hpp"
#include "daemon/table/fib-entry.hpp"
#include "daemon/table/fib-nexthop.hpp"

#include "ns3/object.h"
#include "ns3/node.h"
#include "ns3/node-container.h"
#include "ns3/net-device.h"
#include "ns3/channel.h"
#include "ns3/log.h"
#include "ns3/assert.h"
#include "ns3/names.h"
#include "ns3/node-list.h"
#include "ns3/channel-list.h"
#include "ns3/object-factory.h"

//ZhangYu 2016-12-2
#include "ns3/application.h"
#include "ns3/point-to-point-net-device.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/channel.h"

//#include "NFD/daemon/face/face.hpp"

#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#include <boost/concept/assert.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include <unordered_map>

#include "boost-graph-ndn-global-routing-helper.hpp"

#include <math.h>

//2016-11-22 为了StringValue
#include "ns3/string.h"


NS_LOG_COMPONENT_DEFINE("ndn.GlobalRoutingHelper");
using namespace boost;
using namespace std;

namespace ns3 {
namespace ndn {

void
GlobalRoutingHelper::Install(Ptr<Node> node)
{
  NS_LOG_LOGIC("Node: " << node->GetId());

  Ptr<L3Protocol> ndn = node->GetObject<L3Protocol>();
  NS_ASSERT_MSG(ndn != 0, "Cannot install GlobalRoutingHelper before Ndn is installed on a node");

  Ptr<GlobalRouter> gr = node->GetObject<GlobalRouter>();
  if (gr != 0) {
    NS_LOG_DEBUG("GlobalRouter is already installed: " << gr);
    return; // already installed
  }

  gr = CreateObject<GlobalRouter>();
  node->AggregateObject(gr);

  for (auto& face : ndn->getForwarder()->getFaceTable()) {
    auto linkService = dynamic_cast<NetDeviceLinkService*>(face->getLinkService());
    if (linkService == nullptr) {
      NS_LOG_DEBUG("Skipping non-netdevice face");
      continue;
    }

    Ptr<NetDevice> nd = linkService->GetNetDevice();
    if (nd == 0) {
      NS_LOG_DEBUG("Not a NetDevice associated with NetDeviceFace");
      continue;
    }

    Ptr<Channel> ch = nd->GetChannel();

    if (ch == 0) {
      NS_LOG_DEBUG("Channel is not associated with NetDevice");
      continue;
    }

    if (ch->GetNDevices() == 2) // e.g., point-to-point channel
    {
      for (uint32_t deviceId = 0; deviceId < ch->GetNDevices(); deviceId++) {
        Ptr<NetDevice> otherSide = ch->GetDevice(deviceId);
        if (nd == otherSide)
          continue;

        Ptr<Node> otherNode = otherSide->GetNode();
        NS_ASSERT(otherNode != 0);

        Ptr<GlobalRouter> otherGr = otherNode->GetObject<GlobalRouter>();
        if (otherGr == 0) {
          Install(otherNode);
        }
        otherGr = otherNode->GetObject<GlobalRouter>();
        NS_ASSERT(otherGr != 0);
        gr->AddIncidency(face, otherGr);
      }
    }
    else {
      Ptr<GlobalRouter> grChannel = ch->GetObject<GlobalRouter>();
      if (grChannel == 0) {
        Install(ch);
      }
      grChannel = ch->GetObject<GlobalRouter>();

      gr->AddIncidency(face, grChannel);
    }
  }
}

void
GlobalRoutingHelper::Install(Ptr<Channel> channel)
{
  NS_LOG_LOGIC("Channel: " << channel->GetId());

  Ptr<GlobalRouter> gr = channel->GetObject<GlobalRouter>();
  if (gr != 0)
    return;

  gr = CreateObject<GlobalRouter>();
  channel->AggregateObject(gr);

  for (uint32_t deviceId = 0; deviceId < channel->GetNDevices(); deviceId++) {
    Ptr<NetDevice> dev = channel->GetDevice(deviceId);

    Ptr<Node> node = dev->GetNode();
    NS_ASSERT(node != 0);

    Ptr<GlobalRouter> grOther = node->GetObject<GlobalRouter>();
    if (grOther == 0) {
      Install(node);
    }
    grOther = node->GetObject<GlobalRouter>();
    NS_ASSERT(grOther != 0);

    gr->AddIncidency(0, grOther);
  }
}

void
GlobalRoutingHelper::Install(const NodeContainer& nodes)
{
  for (NodeContainer::Iterator node = nodes.Begin(); node != nodes.End(); node++) {
    Install(*node);
  }
}

void
GlobalRoutingHelper::InstallAll()
{
  Install(NodeContainer::GetGlobal());
}

void
GlobalRoutingHelper::AddOrigin(const std::string& prefix, Ptr<Node> node)
{
  Ptr<GlobalRouter> gr = node->GetObject<GlobalRouter>();
  NS_ASSERT_MSG(gr != 0, "GlobalRouter is not installed on the node");

  auto name = make_shared<Name>(prefix);
  gr->AddLocalPrefix(name);
}

void
GlobalRoutingHelper::AddOrigins(const std::string& prefix, const NodeContainer& nodes)
{
  for (NodeContainer::Iterator node = nodes.Begin(); node != nodes.End(); node++) {
    AddOrigin(prefix, *node);
  }
}

void
GlobalRoutingHelper::AddOrigin(const std::string& prefix, const std::string& nodeName)
{
  Ptr<Node> node = Names::Find<Node>(nodeName);
  NS_ASSERT_MSG(node != 0, nodeName << "is not a Node");

  AddOrigin(prefix, node);
}

void
GlobalRoutingHelper::AddOriginsForAll()
{
  for (NodeList::Iterator node = NodeList::Begin(); node != NodeList::End(); node++) {
    Ptr<GlobalRouter> gr = (*node)->GetObject<GlobalRouter>();
    std::string name = Names::FindName(*node);

    if (gr != 0 && !name.empty()) {
      AddOrigin("/" + name, *node);
    }
  }
}

void
GlobalRoutingHelper::CalculateRoutes()
{
  /**
   * Implementation of route calculation is heavily based on Boost Graph Library
   * See http://www.boost.org/doc/libs/1_49_0/libs/graph/doc/table_of_contents.html for more details
   */

  BOOST_CONCEPT_ASSERT((boost::VertexListGraphConcept<boost::NdnGlobalRouterGraph>));
  BOOST_CONCEPT_ASSERT((boost::IncidenceGraphConcept<boost::NdnGlobalRouterGraph>));

  boost::NdnGlobalRouterGraph graph;
  // typedef graph_traits < NdnGlobalRouterGraph >::vertex_descriptor vertex_descriptor;

  // For now we doing Dijkstra for every node.  Can be replaced with Bellman-Ford or Floyd-Warshall.
  // Other algorithms should be faster, but they need additional EdgeListGraph concept provided by
  // the graph, which
  // is not obviously how implement in an efficient manner
  for (NodeList::Iterator node = NodeList::Begin(); node != NodeList::End(); node++) {
    Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter>();
    if (source == 0) {
      NS_LOG_DEBUG("Node " << (*node)->GetId() << " does not export GlobalRouter interface");
      continue;
    }

    boost::DistancesMap distances;

    dijkstra_shortest_paths(graph, source,
                            // predecessor_map (boost::ref(predecessors))
                            // .
                            distance_map(boost::ref(distances))
                              .distance_inf(boost::WeightInf)
                              .distance_zero(boost::WeightZero)
                              .distance_compare(boost::WeightCompare())
                              .distance_combine(boost::WeightCombine()));

    // NS_LOG_DEBUG (predecessors.size () << ", " << distances.size ());

    Ptr<L3Protocol> L3protocol = (*node)->GetObject<L3Protocol>();
    shared_ptr<nfd::Forwarder> forwarder = L3protocol->getForwarder();

    NS_LOG_DEBUG("Reachability from Node: " << source->GetObject<Node>()->GetId());
    for (const auto& dist : distances) {
      if (dist.first == source)
        continue;
      else {
        // cout << "  Node " << dist.first->GetObject<Node> ()->GetId ();
        if (std::get<0>(dist.second) == 0) {
          // cout << " is unreachable" << endl;
        }
        else {
          for (const auto& prefix : dist.first->GetLocalPrefixes()) {
            NS_LOG_DEBUG(" prefix " << prefix << " reachable via face " << *std::get<0>(dist.second)
                         << " with distance " << std::get<1>(dist.second) << " with delay "
                         << std::get<2>(dist.second));

            FibHelper::AddRoute(*node, *prefix, std::get<0>(dist.second),
                                std::get<1>(dist.second));
          }
        }
      }
    }
  }
}

void
GlobalRoutingHelper::CalculateAllPossibleRoutes()
{
  /**
   * Implementation of route calculation is heavily based on Boost Graph Library
   * See http://www.boost.org/doc/libs/1_49_0/libs/graph/doc/table_of_contents.html for more details
   */

  BOOST_CONCEPT_ASSERT((boost::VertexListGraphConcept<boost::NdnGlobalRouterGraph>));
  BOOST_CONCEPT_ASSERT((boost::IncidenceGraphConcept<boost::NdnGlobalRouterGraph>));

  boost::NdnGlobalRouterGraph graph;
  // typedef graph_traits < NdnGlobalRouterGraph >::vertex_descriptor vertex_descriptor;

  // For now we doing Dijkstra for every node.  Can be replaced with Bellman-Ford or Floyd-Warshall.
  // Other algorithms should be faster, but they need additional EdgeListGraph concept provided by
  // the graph, which
  // is not obviously how implement in an efficient manner
  for (NodeList::Iterator node = NodeList::Begin(); node != NodeList::End(); node++) {
    Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter>();
    if (source == 0) {
      NS_LOG_DEBUG("Node " << (*node)->GetId() << " does not export GlobalRouter interface");
      continue;
    }

    Ptr<L3Protocol> L3protocol = (*node)->GetObject<L3Protocol>();
    shared_ptr<nfd::Forwarder> forwarder = L3protocol->getForwarder();

    NS_LOG_DEBUG("Reachability from Node: " << source->GetObject<Node>()->GetId() << " ("
                                            << Names::FindName(source->GetObject<Node>()) << ")");

    Ptr<L3Protocol> l3 = source->GetObject<L3Protocol>();
    NS_ASSERT(l3 != 0);

    // remember interface statuses
    std::list<nfd::FaceId> faceIds;
    std::unordered_map<nfd::FaceId, uint16_t> originalMetrics;
    for (auto& i : l3->getForwarder()->getFaceTable()) {
      shared_ptr<Face> nfdFace = std::dynamic_pointer_cast<Face>(i);
      faceIds.push_back(nfdFace->getId());
      originalMetrics[nfdFace->getId()] = nfdFace->getMetric();
      nfdFace->setMetric(std::numeric_limits<uint16_t>::max() - 1);
      // value std::numeric_limits<uint16_t>::max () MUST NOT be used (reserved)
    }

    for (auto& faceId : faceIds) {
      shared_ptr<Face> face = l3->getForwarder()->getFaceTable().get(faceId);
      auto linkService = dynamic_cast<NetDeviceLinkService*>(face->getLinkService());
      if (linkService == nullptr) {
        NS_LOG_DEBUG("Skipping non-netdevice face");
        continue;
      }

      // enabling only faceId
      face->setMetric(originalMetrics[faceId]);

      boost::DistancesMap distances;

      NS_LOG_DEBUG("-----------");

      dijkstra_shortest_paths(graph, source,
                              // predecessor_map (boost::ref(predecessors))
                              // .
                              distance_map(boost::ref(distances))
                                .distance_inf(boost::WeightInf)
                                .distance_zero(boost::WeightZero)
                                .distance_compare(boost::WeightCompare())
                                .distance_combine(boost::WeightCombine()));

      // NS_LOG_DEBUG (predecessors.size () << ", " << distances.size ());

      for (const auto& dist : distances) {
        if (dist.first == source)
          continue;
        else {
          // cout << "  Node " << dist.first->GetObject<Node> ()->GetId ();
          if (std::get<0>(dist.second) == 0) {
            // cout << " is unreachable" << endl;
          }
          else {
            for (const auto& prefix : dist.first->GetLocalPrefixes()) {
              NS_LOG_DEBUG(" prefix " << *prefix << " reachable via face "
                           << *std::get<0>(dist.second)
                           << " with distance " << std::get<1>(dist.second)
                           << " with delay " << std::get<2>(dist.second));

              if (std::get<0>(dist.second)->getMetric() == std::numeric_limits<uint16_t>::max() - 1)
                continue;

              FibHelper::AddRoute(*node, *prefix, std::get<0>(dist.second),
                                  std::get<1>(dist.second));
            }
          }
        }
      }

      // disabling the face again
      face->setMetric(std::numeric_limits<uint16_t>::max() - 1);
    }

    // recover original interface statuses
    for (auto& i : originalMetrics) {
      l3->getForwarder()->getFaceTable().get(i.first)->setMetric(i.second);
    }
  }
}
/* ZhangYu 2016-12-5，modify for the new version of ndnSIM.  global no common link multi-path routing pair first,
 * not like the previous noCommLinkMultiPath routing in which all the multipaths for a pair of source and destination are get before calculate the next pair.
 * In pair first, we would calculate the shortest path for all of the pair first, then the second shortest path for all the pair, until all of the paths are found.
 */
void
GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutesPairFirst()
{
	CalculateNoCommLinkMultiPathRoutesPairFirst(false);
}
/* ZhangYu 2016-12-1 为了和Matlab中的算法保持一致，搜索出一条路径后0->1->4后，添加路由0->1->4，设置其为Max-1
 * 将当setReverseRoute是True时，还要设置4->1->0为Max-1，但是不添加路由4->1->0
 */
void
GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutesPairFirst(bool setReverseRoute)
{
	BOOST_CONCEPT_ASSERT(( VertexListGraphConcept< NdnGlobalRouterGraph > ));
	BOOST_CONCEPT_ASSERT((IncidenceGraphConcept<NdnGlobalRouterGraph>));
	NdnGlobalRouterGraph graph;
	typedef graph_traits<NdnGlobalRouterGraph>::vertex_descriptor vertex_descriptor;
	BackupRestoreOriginalMetrics("Backup");

	bool foundPath=true;
    //2015-2-25，为了能够只计算响应的 Producer 节点到源节点的路径，需要匹配 curPrefix。原来的 Multipath 计算了所有的 procucer 到当前的 consumer
    string curPrefix;
	//ZhangYu 2015-1-7 设置循环一轮节点，如果能找到一条路径，那么foundPath=true,否则维持false，导致不再进行下一轮循环。
	while(foundPath)
	{
		foundPath=false;
		for (NodeList::Iterator node = NodeList::Begin (); node != NodeList::End (); node++)
		{
			   Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter> ();
				if (source == 0)	{
					NS_LOG_DEBUG ("Node " << (*node)->GetId () << " does not export GlobalRouter interface");
					continue;
				}

				//开始计算最短路
				for(uint32_t appId=0; appId<(*node)->GetNApplications();appId++)
				{
					NS_LOG_DEBUG("ZhangYu 2016-11-30 appId:" <<(*node)->GetApplication(appId)->GetInstanceTypeId().GetName());
					std::string appTypeStr= (*node)->GetApplication(appId)->GetInstanceTypeId().GetName();
					if(std::string::npos!= appTypeStr.find("Consumer"))	//only calculate the Consumer/source node 2015-1-8
					{
						//NS_LOG_DEBUG("ZhangYu 2014-1-1 is consumer node Id: " << (*node)->GetId() <<" " << (appTypeStr.find("Consumer")) <<"'  "<< appTypeStr);
						NS_LOG_DEBUG ("===== Reachability from source Node: " << source->GetObject<Node> ()->GetId () << " (" << Names::FindName (source->GetObject<Node> ()) << ")");
 						//2015-2-25，添加的路由计算时只计算和 consumeer 拥有同样的 Prefix 的 Producer 的路由，这样保证添加的Fib和设置为无穷大的只是一对源目的节点对之间的多路径。
						//虽然代码支持，这里没有仔细考虑多个 Application 时是否能正确运行，目前的一个节点只装在一个 consumer 或者一个 procuer，后面的也一样
						StringValue tmp;
						(*node)->GetApplication(appId)->GetAttribute("Prefix",tmp);
						curPrefix=tmp.Get();
						//NS_LOG_DEBUG("ZhangYu 2015-2-25   consumer1->GetApplication(0)->GetAttribute(Prefix,: " << curPrefix << std::endl);

						DistancesMap    distances;
						PredecessorsMap predecessors;
						dijkstra_shortest_paths (graph, source,
												 predecessor_map (boost::ref(predecessors))
												 .
												 distance_map (boost::ref(distances))
												 .
												 distance_inf (WeightInf)
												 .
												 distance_zero (WeightZero)
												 .
												 distance_compare (boost::WeightCompare ())
												 .
												 distance_combine (boost::ZYWeightCombine ())
												 );
						for (DistancesMap::iterator dist = distances.begin (); dist != distances.end (); dist++)
						//for (const auto& dist : distances)
						{
							if (dist->first == source)
								continue;
							else
							{
								if (std::get<0>(dist->second) == 0)
								{
									NS_LOG_DEBUG("  Node " << dist->first->GetObject<Node> ()->GetId () << " is unreachable" << endl);
								}
								else
								{
									//NS_LOG_DEBUG("ZhangYu 2014-1-3, Node:" << dist->first->GetObject<Node>()->GetId()<< "   face:" << *dist->second.get<0>()<<"  with distance:" <<dist->second.get<1>());

									//下面的语句使得为每个producer的节点的每个应用添加路由fibs，为0就不循环，一个节点有多个Apps时循环（这里循环执行有点冗余，因为步骤一样，只是prefix不同，但是为了代码清爽，就这样了）
									//NS_LOG_DEBUG("ZhangYu 2014-2-7 dist->first->GetLocalPrefixes.size(): " <<dist->first->GetLocalPrefixes().size());

									for (const auto& prefix : dist->first->GetLocalPrefixes ())
									{
										//2015-2-26 只有和 consumer 具有相同 Prefix 的producer 才添加FIB，修改其路径上的边的代价为无穷
                                        if (curPrefix==prefix->toUri())
                                        {
                                            Ptr<GlobalRouter> curNode =dist->first ;
                                            Ptr<GlobalRouter> preNode;
                                            NS_LOG_DEBUG("ZhangYu 2014-1-7 producer Node: " << curNode->GetObject<Node>()->GetId() );

                                            while (curNode!=source)
                                            {
                                                preNode=predecessors[curNode];
                                                NS_LOG_DEBUG("ZhangYu 2016-12-4 curNode: " << curNode->GetObject<Node>()->GetId() << "；      preNode: " << preNode->GetObject<Node>()->GetId() );
                                                //ZhangYu 2016-12 ZYWeightCombine的返回值是，假设路径为0-1-4，对于节点4，返回的face是1-4的，返回的距离是
                                                NS_LOG_DEBUG("ZhangYu  2014-12-4 prefix: " << *prefix << "  Node: " << preNode->GetObject<Node>()->GetId()
                                                             << "  reachable via face: " << *std::get<0>(distances[curNode])
															 << " LocalUri： " << std::get<0>(distances[curNode])->getLocalUri()
															 << " RemoteUri" << std::get<0>(distances[curNode])->getRemoteUri()
                                                             << "  with distance: " << std::get<1>(distances[curNode])-std::get<1>(distances[preNode])
                                                             << "  with delay " << std::get<2>(distances[curNode]) << endl);

                                                if(std::get<1>(distances[curNode])-std::get<1>(distances[preNode])<std::numeric_limits<uint16_t>::max())
                                                {
													FibHelper::AddRoute(preNode->GetObject<Node>(), *prefix, std::get<0>(distances[curNode]),
																		std::get<1>(distances[curNode])-std::get<1>(distances[preNode]));
                                                    //distances是Map类型，dist->first得到的是key，dist->second得到的是Value，而直接使用distances[curNode]得到的也是Value
                                                    foundPath=true;
                                                    //前面执行完了回溯路径，添加fib，后面的是把这条路径上的Link设置为不可用
                                                    std::get<0>(distances[curNode])->setMetric(std::numeric_limits<uint16_t>::max() - 1);
                                                }
                                                else
                                                	NS_LOG_DEBUG("ZhangYu 2014-3-22 didnot add fib, because greater than uint16::max");
                                                if(setReverseRoute)	{
													//原始代码中，当 curNode 是4时，设置的是节点1到节点4的Face，反向链路应该是节点4到节点1的 Face。
													Ptr<L3Protocol> l3 = curNode->GetObject<L3Protocol>();
													for (auto& i : l3->getForwarder()->getFaceTable()) {
													  shared_ptr<Face> nfdFace = std::dynamic_pointer_cast<Face>(i);
													  if(nfdFace->getLocalUri()==std::get<0>(distances[curNode])->getRemoteUri())	{
														  nfdFace->setMetric(std::numeric_limits<uint16_t>::max() - 1);
														  //value std::numeric_limits<uint16_t>::max () MUST NOT be used (reserved)
														  NS_LOG_DEBUG("ZhangYu 2016-12-5 Reverse Edge set to Max"  << nfdFace->getLocalUri());
													  }
													}
                                                }
                                                curNode=preNode;
                                            }
                                        }
									}
								}
							}
						}
					}
				}
		}
	}
    BackupRestoreOriginalMetrics("Restore");
}

//tobedelete std::vector <std::vector<uint16_t> >  originalMetric;  	//注意这里的> >之间要有空格，否则error: ‘>>’ should be ‘> >’ within a nested template argument list
std::unordered_map<std::uint32_t,std::unordered_map<nfd::FaceId, uint16_t> > originalMetrics;
void
GlobalRoutingHelper::BackupRestoreOriginalMetrics(const std::string action)
{
	BOOST_CONCEPT_ASSERT((VertexListGraphConcept<NdnGlobalRouterGraph > ));
	BOOST_CONCEPT_ASSERT((IncidenceGraphConcept<NdnGlobalRouterGraph>));
	NdnGlobalRouterGraph graph;
	typedef graph_traits<NdnGlobalRouterGraph>::vertex_descriptor vertex_descriptor;

	for (NodeList::Iterator node = NodeList::Begin(); node != NodeList::End(); node++) {
		Ptr<GlobalRouter> source = (*node)->GetObject<GlobalRouter>();
	    if (source == 0) {
	      NS_LOG_DEBUG("Node " << (*node)->GetId() << " does not export GlobalRouter interface");
	      continue;
	    }
	    std::uint32_t nodeNo=(*node)->GetId();

	    Ptr<L3Protocol> l3 = source->GetObject<L3Protocol>();
	    NS_ASSERT(l3 != 0);

	    // remember interface statuses
	    std::list<nfd::FaceId> faceIds;

	    NS_LOG_DEBUG("ZhangYu 2016-12-4 Node: " << (*node)->GetId() << "  Node Name: " << Names::FindName(*node));
	    if (action=="Backup&Initial")	{
	 		for (auto& i : l3->getForwarder()->getFaceTable()) {
			shared_ptr<Face> nfdFace = std::dynamic_pointer_cast<Face>(i);
			NS_LOG_DEBUG("ZhangYu 2016-12-4 nfdFace Id:" << nfdFace->getId() << "    nfdFace:" << *nfdFace <<"   localUri: " << nfdFace->getLocalUri() << "   remoteUri: "  << nfdFace->getRemoteUri());
			faceIds.push_back(nfdFace->getId());
			originalMetrics[nodeNo][nfdFace->getId()] = nfdFace->getMetric();
			nfdFace->setMetric(std::numeric_limits<uint16_t>::max() - 1);
			// value std::numeric_limits<uint16_t>::max () MUST NOT be used (reserved)
			}
	    }
	    else if (action=="Backup")	{
	 		for (auto& i : l3->getForwarder()->getFaceTable()) {
			shared_ptr<Face> nfdFace = std::dynamic_pointer_cast<Face>(i);
			NS_LOG_DEBUG("ZhangYu 2016-12-4 nfdFace Id:" << nfdFace->getId() << "    nfdFace:" << *nfdFace <<"   localUri: " << nfdFace->getLocalUri() << "   remoteUri: "  << nfdFace->getRemoteUri());
			faceIds.push_back(nfdFace->getId());
			originalMetrics[nodeNo][nfdFace->getId()] = nfdFace->getMetric();
			}
	    }
	  	else if (action=="Restore")
	  	{
		    // recover original interface statuses
		    for (auto& i : originalMetrics[nodeNo]) {
		      l3->getForwarder()->getFaceTable().get(i.first)->setMetric(i.second);
		    }
	    }
		else
			NS_LOG_DEBUG("ZhangYu  input a wrong action string for function BackupRestoreOriginalMetrics");
	}
}

} // namespace ndn
} // namespace ns3
