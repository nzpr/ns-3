#include <fstream>
#include <iostream>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/olsr-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/yans-wifi-helper.h"
#include <sys/stat.h>
#include <sys/types.h>
#include "stdlib.h"
#include "stdio.h"
#include "string.h"



NS_LOG_COMPONENT_DEFINE ("ROUTING_STUDY");

using namespace ns3;

class Simulation
{
public:
  Simulation()
  {
    nWifis                    = 100;
    nSources                    = 1;
    TotalTime                 = 150;
    dataPort                  = 9;
    control_port              = 776;
    dataRate                  = "5300bps";
    dataPacketSize            = 20;
    distance                  = 150;
    minNodeSpeed              = 0.00001;
    maxNodeSpeed              = 0.00001;
    txp                       = 7.5;
    macMode                   = "DsssRate11Mbps";
    collectMetadata           = false;
    showMemory                  = true;    
  }

  uint32_t                                                                                                          
  GetSimulationTime()                                                                                                
  {                 
     return TotalTime;
  }  

  ~Simulation()
  {
  };

  void
  GetArgs(int argc, char *argv[])
  {
    CommandLine cmd;   
    cmd.AddValue ("showMemory", "Show memory consumption"
        "[bool, по умолчанию 1]", showMemory);
    cmd.AddValue ("nWifis", "Number of nodes in the network"
        "[по умолчанию 100]", nWifis);
    cmd.AddValue ("nSources", "Number of traffic sources"
        "[по умолчанию 1]", nSources);    
    cmd.AddValue ("collectMetadata", "Install FlowMonitor to collect traffic statictics"
        "[default false]", collectMetadata);    
    cmd.Parse (argc, argv);   
  } 

  int
  Init(int argc, char *argv[])
  {
    int i = argc;    
    GetArgs(argc, argv);
    InitNetwork();
    InitRouting();
    InitInternetStack();
    initTraffic();
    InitMobility();
    return 0;
  }

  void
  InitNetwork()
  {
    if (showMemory == true) ShowMemory();
    adhocNodes.Create (nWifis);    
    WifiHelper wifi;    
    wifi.SetStandard (WIFI_PHY_STANDARD_80211g);        
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
        "DataMode",StringValue  (macMode),
        "ControlMode",StringValue (macMode));        
    Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
        StringValue (macMode));    
    YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();    
    YansWifiChannelHelper wifiChannel;    
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");    
    wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel");    
    wifiPhy.SetChannel (wifiChannel.Create ());    
    wifiPhy.Set ("TxPowerStart",DoubleValue (txp));
    wifiPhy.Set ("TxPowerEnd", DoubleValue (txp));    
    wifiPhy.Set ("TxGain",DoubleValue (0.0));
    wifiPhy.Set ("RxGain",DoubleValue (0.0));    
    NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();    
    wifiMac.SetType ("ns3::AdhocWifiMac");    
    adhocDevices = wifi.Install (wifiPhy, wifiMac, adhocNodes);    
  }

  void
  InitRouting()
  {      
    OlsrHelper olsr; 
    list.Add (olsr, 100);
    control_port = 698;
  }

  void
  InitInternetStack()
  {    
    InternetStackHelper internet;  
    internet.SetRoutingHelper (list);    
    internet.Install (adhocNodes);
    Ipv4AddressHelper addressAdhoc;   
    addressAdhoc.SetBase ("10.1.64.0", "255.255.192.0");  
    adhocInterfaces = addressAdhoc.Assign (adhocDevices);
  }

  void
  InitMobility()
  {    
    MobilityHelper mobilityAdhoc;
    ObjectFactory pos;
    pos.SetTypeId ("ns3::RandomRectanglePositionAllocator");
    std::stringstream s;
    s << "ns3::UniformRandomVariable[Min=0.0|Max=" << sqrt(nWifis)*distance << "]";
    pos.Set ("X", StringValue (s.str()));
    pos.Set ("Y", StringValue (s.str()));
    Ptr<PositionAllocator> PositionAlloc = pos.Create ()->GetObject<PositionAllocator> ();    
    std::stringstream speed;
    speed << "ns3::UniformRandomVariable[Min=" << minNodeSpeed << "|Max=" << maxNodeSpeed << "]";
    mobilityAdhoc.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
        "Speed", StringValue(speed.str()),
        "Pause", StringValue("ns3::ConstantRandomVariable[Constant=0]"),
        "PositionAllocator", PointerValue (PositionAlloc));
    mobilityAdhoc.SetPositionAllocator (PositionAlloc);
    mobilityAdhoc.Install (adhocNodes);
  }

  void
  initTraffic()
  {    
    OnOffHelper onoff1 ("ns3::UdpSocketFactory",Address ());    
    onoff1.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));    
    onoff1.SetAttribute ("OffTime",  StringValue ("ns3::ConstantRandomVariable[Constant=0]"));    
    onoff1.SetAttribute ("PacketSize", UintegerValue(dataPacketSize));    
    onoff1.SetAttribute ("DataRate",  DataRateValue (DataRate (dataRate)));    
    Ptr<Node> reciever = adhocNodes.Get (0);        
    Ipv4Address rcvAddr = reciever->GetObject<Ipv4>()->GetAddress(1,0).GetLocal();
    TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");        
    Ptr<Socket> sink = Socket::CreateSocket (reciever, tid);        
    InetSocketAddress local = InetSocketAddress (rcvAddr, dataPort);        
    sink->Bind (local);        
    onoff1.SetAttribute ("Remote", AddressValue(InetSocketAddress (rcvAddr, dataPort)));        
    ApplicationContainer temp = onoff1.Install (adhocNodes.Get (nWifis-1));        
    temp.Start (Seconds(0));        
    temp.Stop (Seconds (TotalTime));

    if(collectMetadata)
         fm = flowmonHelper.InstallAll();
   }  


  int parseLine(char* line){
        int i = strlen(line);
        while (*line < '0' || *line > '9') line++;
        line[i-3] = '\0';
        i = atoi(line);
        return i;
    }
    

  int getValue() { 
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128]; 
    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmRSS:", 6) == 0){
        result = parseLine(line);
        break;
            }
        }
    fclose(file);
    return result;
    }

  void
  ShowMemory()
  {
    double now = Simulator::Now ().GetSeconds (); 
    NS_LOG_UNCOND ("Memory consumption: " << getValue() << " kbytes");
    void (Simulation::*printMemory)() = &Simulation::ShowMemory;
    Simulator::Schedule (Seconds (1), printMemory, this);
  }  
 

private:                 
  uint32_t nWifis;        
  uint32_t distance;        
  double TotalTime;       
  uint32_t dataPort;         
  uint32_t dataPacketSize;
  double minNodeSpeed;    
  double maxNodeSpeed;    
  double txp;             
  bool collectMetadata;             
  std::string dataRate;   
  std::string macMode;        
  bool showMemory;            
  int nSources;  
  NodeContainer adhocNodes;       
  Ipv4ListRoutingHelper list;     
  NetDeviceContainer adhocDevices;
  Ptr<FlowMonitor> fm;            
  FlowMonitorHelper flowmonHelper;
  uint32_t control_port;          
  Ipv4InterfaceContainer adhocInterfaces;  
};

int
main (int argc, char *argv[])
{
  Simulation exp;
  if (exp.Init(argc, argv) == -1) 
    return 0;  
  Simulator::Stop (Seconds (exp.GetSimulationTime()));  
  Simulator::Run ();  
  Simulator::Destroy ();
  return 0;
}



