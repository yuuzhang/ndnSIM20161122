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

// ndn-grid.cpp

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/point-to-point-layout-module.h"
#include "ns3/ndnSIM-module.h"

//2016-12-7
#include <boost/lexical_cast.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/lambda/bind.hpp>

namespace ns3 {

/**

 */

int
main(int argc, char* argv[])
{
	bool manualAssign=false;
	int InterestsPerSec=200;
	int simulationSpan=50;
	string routingName="MultiPath";

	CommandLine cmd;
	cmd.AddValue("InterestsPerSec","Interests emit by consumer per second",InterestsPerSec);
	cmd.AddValue("simulationSpan","Simulation span time by seconds",simulationSpan);
	cmd.AddValue ("routingName", "could be Flooding, BestRoute, MultiPath, MultiPathPairFirst", routingName);
	// Read optional command-line parameters (e.g., enable visualizer with ./waf --run=<> --visualize
	cmd.Parse(argc,argv);
	std::cout << "routingName: " << routingName << "   " << InterestsPerSec << " " << simulationSpan << std::endl;

	// LogComponentEnable("ndn.CbisGlobalRoutingHelper", LOG_LEVEL_INFO);
	// Setting default parameters for PointToPoint links and channels
	Config::SetDefault("ns3::PointToPointNetDevice::DataRate", StringValue("1Mbps"));
	Config::SetDefault("ns3::PointToPointChannel::Delay", StringValue("1ms"));
	Config::SetDefault("ns3::DropTailQueue::MaxPackets", StringValue("10"));

	AnnotatedTopologyReader topologyReader ("", 20);

	topologyReader.SetFileName ("src/ndnSIM/examples/topologies/topo-for-CompareMultiPath.txt");
	//topologyReader.SetFileName ("src/ndnSIM/examples/topologies/topo-6-node.txt");

	topologyReader.Read ();
	int nodesNumber=topologyReader.GetNodes().size();

	// Install CCNx stack on all nodes
	ndn::StackHelper ndnHelper;
	  // Install NDN stack on all nodes
	  ndnHelper.SetOldContentStore("ns3::ndn::cs::Lru", "MaxSize",
	                               "100"); // default ContentStore parameters
	// ndnHelper.SetForwardingStrategy ("ns3::ndn::fw::SmartFlooding");
	// ndnHelper.SetContentStore ("ns3::ndn::cs::Lru", "MaxSize", "10");
	ndnHelper.InstallAll();

	// Choosing forwarding strategy
	ndn::StrategyChoiceHelper::InstallAll("/prefix", "/localhost/nfd/strategy/ncc");
	//ndn::StrategyChoiceHelper::InstallAll("/prefix", "/localhost/nfd/strategy/best-route");

	// Installing global routing interface on all nodes
	// ndn::CbisGlobalRoutingHelper ndnGlobalRoutingHelper;
	ndn::GlobalRoutingHelper ndnGlobalRoutingHelper;
	ndnGlobalRoutingHelper.InstallAll();

	// Getting containers for the consumer/producer
	NodeContainer consumerNodes;
	consumerNodes.Add(Names::Find<Node>("Node0"));
	Ptr<Node> producer = Names::Find<Node>("Node4");

	// Getting containers for the consumer/producer
//	Ptr<Node> consumers[4] = {Names::Find<Node>("leaf-1"), Names::Find<Node>("leaf-2"),
//							Names::Find<Node>("leaf-3"), Names::Find<Node>("leaf-4")};
//	Ptr<Node> producer = Names::Find<Node>("root");
//
//	for (int i = 0; i < 4; i++) {
//	ndn::AppHelper consumerHelper("ns3::ndn::ConsumerCbr");
//	consumerHelper.SetAttribute("Frequency", StringValue("10")); // 100 interests a second
//
//	// Each consumer will express the same data /root/<seq-no>
//	consumerHelper.SetPrefix("/root");
//	ApplicationContainer app = consumerHelper.Install(consumers[i]);
//	app.Start(Seconds(0.01 * i));
//	}

	// Install CCNx applications
	std::string prefix = "/prefix";

	ndn::AppHelper consumerHelper("ns3::ndn::ConsumerZipfMandelbrot");
	// ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerCbr");
	consumerHelper.SetPrefix(prefix);
	consumerHelper.SetAttribute("Frequency", StringValue (boost::lexical_cast<std::string>(InterestsPerSec)));        // 100 interests a second
	consumerHelper.SetAttribute("NumberOfContents", StringValue("100")); // 10 different contents
	// consumerHelper.SetAttribute ("Randomize", StringValue ("uniform")); // 100 interests a second
	ApplicationContainer app = consumerHelper.Install(consumerNodes);
	app.Start(Seconds(0.00 ));

	ndn::AppHelper producerHelper("ns3::ndn::Producer");
	producerHelper.SetPrefix(prefix);
	producerHelper.SetAttribute("PayloadSize", StringValue("100"));
	producerHelper.Install(producer);
	ndnGlobalRoutingHelper.AddOrigins(prefix, producer);

    //Calculate and install FIBs
	if(routingName.compare("BestRoute")==0){
	  ndn::GlobalRoutingHelper::CalculateRoutes ();
	}
	else if(routingName.compare("MultiPath")==0){
		ndn::GlobalRoutingHelper::CalculateNoCommLinkMultiPathRoutesPairFirst();
	}
	else if(routingName.compare("Flooding")==0){
		ndn::GlobalRoutingHelper::CalculateAllPossibleRoutes();
	}

	Simulator::Stop (Seconds (simulationSpan));
	//ZhangYu Add the trace，不愿意文件名称还有大小写的区别，所以把 routingName 全部转为小写
	std::transform(routingName.begin(), routingName.end(), routingName.begin(), ::tolower);
	string filename=routingName+"-"+boost::lexical_cast<std::string>(InterestsPerSec)+".txt";
	ndn::CsTracer::InstallAll ("cs-trace-"+filename, Seconds (1));
	ndn::L3RateTracer::InstallAll ("rate-trace-"+filename, Seconds (1));
	ndn::AppDelayTracer::InstallAll ("app-delays-trace-"+filename);
	L2RateTracer::InstallAll ("drop-trace-"+filename, Seconds (1));

	Simulator::Run();
	Simulator::Destroy();

  return 0;
}

} // namespace ns3

int
main(int argc, char* argv[])
{
  return ns3::main(argc, argv);
}
