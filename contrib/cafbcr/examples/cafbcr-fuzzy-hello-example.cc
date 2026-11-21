/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright (c) 2026 The CA-FBCR Authors.
 */
/*
 * 示例: CAFBCR 路由协议 FuzzyHello 功能演示
 *
 * 本例演示如何在 CAFBCR 路由协议中启用和使用 FuzzyHello 动态 Hello 间隔优化功能。
 *
 * 编译: ./ns3 build
 * 运行:  ./ns3 run contrib/cafbcr/examples/cafbcr-fuzzy-hello-example
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/energy-module.h"
#include "ns3/applications-module.h"
#include "ns3/fanet-helper.h"
#include "ns3/cafbcr-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("CafbcrFuzzyHelloExample");

int main(int argc, char *argv[])
{
  // ==================== 参数配置 ====================
  uint32_t nNodes = 10;                    // 节点数
  double duration = 100.0;                 // 仿真时长（秒）
  bool useFuzzyHello = true;               // 是否启用 FuzzyHello
  double simSpeed = 30.0;                  // 节点速度（m/s）
  double nodeRange = 500.0;                // 通信范围（米）

  CommandLine cmd;
  cmd.AddValue("nNodes", "Number of nodes", nNodes);
  cmd.AddValue("duration", "Simulation duration in seconds", duration);
  cmd.AddValue("useFuzzy", "Use Fuzzy Hello (1) or Fixed Hello (0)", useFuzzyHello);
  cmd.AddValue("speed", "Node speed (m/s)", simSpeed);
  cmd.AddValue("range", "Communication range (m)", nodeRange);
  cmd.Parse(argc, argv);

  // ==================== 启用日志输出 ====================
  // LogComponentEnable("CafbcrRoutingProtocol", LOG_LEVEL_DEBUG);
  // LogComponentEnable("CafbcrFuzzyHelloManager", LOG_LEVEL_DEBUG);
  LogComponentEnable("CafbcrFuzzyHelloExample", LOG_LEVEL_INFO);

  // ==================== 创建节点 ====================
  NS_LOG_INFO("Creating " << nNodes << " nodes...");
  NodeContainer nodes;
  nodes.Create(nNodes);

  // ==================== 配置 WiFi ====================
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211a);
  WifiMacHelper wifiMac;
  wifiMac.SetType("ns3::AdhocWifiMac");

  YansWifiPhyHelper wifiPhy;
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
  wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss("ns3::RangeRatePropagationLossModel",
                                  "MaxRange", DoubleValue(nodeRange));
  wifiPhy.SetChannel(wifiChannel.Create());

  NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, nodes);

  // ==================== 配置网络协议栈 ====================
  InternetStackHelper internet;
  internet.Install(nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

  // ==================== 配置 CAFBCR 路由 ====================
  NS_LOG_INFO("Installing CAFBCR routing protocol...");
  NS_LOG_INFO("Using " << (useFuzzyHello ? "FUZZY" : "FIXED") << " Hello mode");

  CafbcrHelper cafbcrHelper;
  cafbcrHelper.Set("HelloMode", UintegerValue(useFuzzyHello ? 1 : 0));
  cafbcrHelper.Set("HelloInterval", TimeValue(Seconds(2.0)));
  cafbcrHelper.Install(nodes, devices);

  // ==================== 配置能量模型 ====================
  NS_LOG_INFO("Installing energy model...");
  BasicEnergySourceHelper energySourceHelper;
  energySourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(100.0));
  EnergySourceContainer energySources = energySourceHelper.Install(nodes);

  WifiRadioEnergyModelHelper radioEnergyHelper;
  radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.0174));
  radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.0119));
  DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(devices, energySources);

  // ==================== 配置移动模型 ====================
  NS_LOG_INFO("Setting up mobility model with speed " << simSpeed << " m/s...");
  MobilityHelper mobility;

  // 创建随机漫步移动模型
  ObjectFactory pos;
  pos.SetTypeId("ns3::UniformRandomVariable");
  pos.Set("Min", DoubleValue(0.0));
  pos.Set("Max", DoubleValue(2000.0));
  Ptr<ObjectFactory> positionAllocator = new ObjectFactory(pos);

  std::ostringstream oss;
  oss << "ns3::UniformRandomVariable|Min=0.0|Max=" << 2000.0;

  mobility.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
                                "X", StringValue("ns3::UniformRandomVariable|Min=0.0|Max=2000.0"),
                                "Y", StringValue("ns3::UniformRandomVariable|Min=0.0|Max=2000.0"),
                                "Z", StringValue("ns3::UniformRandomVariable|Min=100.0|Max=150.0"));

  // 随机游走移动模型
  mobility.SetMobilityModel("ns3::RandomWaypointMobilityModel",
                            "Speed", StringValue("ns3::UniformRandomVariable|Min=" +
                                                   std::to_string(simSpeed * 0.5) + "|Max=" +
                                                   std::to_string(simSpeed * 1.5)),
                            "Pause", StringValue("ns3::UniformRandomVariable|Min=2.0|Max=5.0"),
                            "PositionAllocator", PointerValue(
                              CreateObjectWithAttributes<RandomRectanglePositionAllocator>(
                                "X", StringValue("ns3::UniformRandomVariable|Min=0.0|Max=2000.0"),
                                "Y", StringValue("ns3::UniformRandomVariable|Min=0.0|Max=2000.0"),
                                "Z", StringValue("ns3::UniformRandomVariable|Min=100.0|Max=150.0"))));

  mobility.Install(nodes);

  // ==================== 配置应用 ====================
  // 可选：添加 Ping 或其他应用以生成流量
  // ...

  // ==================== 运行仿真 ====================
  NS_LOG_INFO("Starting simulation for " << duration << " seconds...");
  Simulator::Stop(Seconds(duration));
  Simulator::Run();
  Simulator::Destroy();

  NS_LOG_INFO("Simulation completed!");
  return 0;
}
