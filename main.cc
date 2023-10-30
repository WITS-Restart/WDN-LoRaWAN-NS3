#include "ns3/callback.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/lora-device-address.h"
#include "ns3/lora-frame-header.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-phy.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/pointer.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/lora-helper.h"
#include "ns3/node-container.h"
#include "ns3/mobility-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/double.h"
#include "ns3/random-variable-stream.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/command-line.h"
#include "ns3/network-server-helper.h"
#include "ns3/correlated-shadowing-propagation-loss-model.h"
#include "ns3/building-penetration-loss.h"
#include "ns3/building-allocator.h"
#include "ns3/buildings-helper.h"
#include "ns3/forwarder-helper.h"
#include <algorithm>
#include <ctime>
#include "ns3/animation-interface.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/file-helper.h"

#include <sys/stat.h>
#include <sys/types.h>

#include <unistd.h>
#include <iostream>
#include <filesystem>
#include <stdio.h>
#include <stdlib.h>

using namespace ns3;
using namespace lorawan;


NS_LOG_COMPONENT_DEFINE ("DerEnergy");

// Network settings
int nDevices = 0;
int nGateways = 2;
double radius = 1000;
double simulationTime = 1*60*60;
int appPeriodSeconds = 1*60;
int runId = 1;
// Channel model
bool realisticChannelModel = false;

std::vector<int> packetsSent (6, 0);
std::vector<int> packetsReceived (6, 0);

std::stringstream buffer;
std::stringstream adr_buffer;

std::stringstream pkt_log_buffer;  
std::vector<std::vector<std::string>> content_gw;
std::vector<std::vector<std::string>> content;
std::vector<std::vector<std::string>> adr_content;


bool custom_node_position=true;
bool enable_gw_position=true;
bool enable_pkt_log = true;
bool en_adr = true;

bool enable_energy_log = true;

std::vector<FileHelper> fileHelpers;
std::string output_dir = "output_files_test";
std::string adr_conf = "SF_allocation/ADR.csv";

bool extract_adr = false;


void
OnTransmissionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  LoraTag tag;
  packet->PeekPacketTag(tag);
  //std::cout << "TX packet->GetUid():" << packet->GetUid() << std::endl;
  
  if (enable_pkt_log){
    //DEBUG show packet details
    Ptr<Packet> packetCopy = packet->Copy();
    LorawanMacHeader mHdr;
    packetCopy->RemoveHeader (mHdr);
    LoraFrameHeader fHdr;
    packetCopy->RemoveHeader (fHdr);
    Time currentTime = Simulator::Now();
    //std::cout <<  currentTime.GetSeconds() <<","<< std::to_string(tag.GetSpreadingFactor()) <<",TX," << fHdr.GetFCnt() << "," << fHdr.GetAddress().GetNwkAddr()  << std::endl;
    
    /*
    Ptr<Node> object = endDevices.Get(fHdr.GetAddress().GetNwkAddr()-1864);
    Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
    Vector m_position = position->GetPosition();
    */
    
    
    pkt_log_buffer << currentTime.GetSeconds() <<","<< std::to_string(tag.GetSpreadingFactor()) <<",TX," << fHdr.GetFCnt() << "," << fHdr.GetAddress().GetNwkAddr() <<"\n";
    
    
    
    
  }
  packetsSent.at(tag.GetSpreadingFactor()-7)++;
}

void
OnPacketReceptionCallback (Ptr<Packet const> packet, uint32_t systemId)
{
  NS_LOG_FUNCTION (packet << systemId);
  LoraTag tag;
  packet->PeekPacketTag(tag);
  //std::cout << "RX packet->GetUid():" << packet->GetUid() << std::endl;
  if (enable_pkt_log){
    Ptr<Packet> packetCopy = packet->Copy();
    LorawanMacHeader mHdr;
    packetCopy->RemoveHeader (mHdr);
    LoraFrameHeader fHdr;
    Time currentTime = Simulator::Now();
    packetCopy->RemoveHeader (fHdr);
    pkt_log_buffer << currentTime.GetSeconds() <<","<< std::to_string(tag.GetSpreadingFactor()) <<",RX," << fHdr.GetFCnt() << "," << fHdr.GetAddress().GetNwkAddr()  << "\n";
    packetsReceived.at(tag.GetSpreadingFactor()-7)++;
  }
}

int
main (int argc, char *argv[])
{

  ns3::Packet::EnablePrinting();
  ns3::PacketMetadata::Enable ();
  std::string interferenceMatrix = "goursaud";
  std::string nodes = "betweeness";
  fileHelpers.clear();
  



  CommandLine cmd;
  cmd.AddValue ("nDevices", "Number of end devices to include in the simulation", nDevices);
  cmd.AddValue ("nGateways", "Number of end devices to include in the simulation", nGateways);
  cmd.AddValue ("simulationTime", "Simulation Time", simulationTime);
  cmd.AddValue ("appPeriodSeconds", "Duty Cycle", appPeriodSeconds);
  cmd.AddValue ("interferenceMatrix", "Interference matrix to use [aloha, goursaud]", interferenceMatrix);
  cmd.AddValue ("runId", "Experiment Run Id", runId);
  cmd.AddValue ("radius", "Radius of the deployment", radius);
  cmd.AddValue ("realisticChannelModel", "Enable Realistic Channel Model", realisticChannelModel);
  cmd.AddValue ("adr", "Enable ADR", en_adr);
  cmd.AddValue ("nodes", "nodes position", nodes);
  cmd.AddValue ("output_dir", "Experiment output directory", output_dir);
  cmd.AddValue ("extract_adr", "Extract ADR allocation (ONLY LogDistance Propagation)", extract_adr);
  cmd.AddValue ("adr_conf", "Use pre-allocated ADR", adr_conf);
  

  cmd.Parse (argc, argv);

  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (1);

  // Creating a directory
  if (mkdir(output_dir.c_str(), 0777) == -1)
      std::cerr << "Error :  " << output_dir << " " << strerror(errno) << std::endl;
  else std::cout << output_dir <<" Directory created" << std::endl;
  if (!extract_adr){
    if (mkdir((output_dir+"/battery-level").c_str(), 0777) == -1)
      std::cerr << "Error :  " << output_dir+"/battery-level" << " " << strerror(errno) << std::endl;
    else std::cout << output_dir+"/battery-level"  <<" Directory created" << std::endl;
  }


  // NOT WORKS!!
  //LogComponentEnable("LoraPacketTracker", LOG_LEVEL_ALL);
  //LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
  //LogComponentEnable("LoraChannel", LOG_LEVEL_INFO);
  // Set up logging
  //LogComponentEnable ("AlohaThroughput", LOG_LEVEL_ALL);

  // Make all devices use SF7 (i.e., DR5)
  // Config::SetDefault ("ns3::EndDeviceLorawanMac::DataRate", UintegerValue (5));

  //buffer << "nDevices,appPeriodSeconds,simulationTime,SF,totPacketsSent,receivedPackets,interferedPackets,noMoreGwPackets,underSensitivityPackets\n";
  buffer << "nDevices,appPeriodSeconds,simulationTime,SF,totPacketsSent,totReceivedPackets\n";

  pkt_log_buffer<< "timestamp,SF,FLAG,Fcnt,nwkAddr\n";

  if (interferenceMatrix == "aloha")
  {
    LoraInterferenceHelper::collisionMatrix = LoraInterferenceHelper::ALOHA;
  }
  else if (interferenceMatrix == "goursaud")
  {
    LoraInterferenceHelper::collisionMatrix = LoraInterferenceHelper::GOURSAUD;
  }

  /***********
   *  Setup  *
   ***********/
    std::vector<int> dr_list;
    if (en_adr){
        dr_list.push_back(-1); //ADR
    }else{
      dr_list.push_back(0); //Use only DR=0 (SF12)

//      //add all DR:
//      for (int ii=0; ii<6; ii++){
//        dr_list.push_back(ii);
//      }

    }

  //int maxDevices=nDevices;
  // int nDevices_list[] = {100};
  std::vector<int> nDevices_list;
  //int nDevices_list[] = {85,65,45,25,5};
  if (nDevices==0){ //NO nDevices specified in args
      nDevices_list.push_back(85);
      nDevices_list.push_back(65);
      nDevices_list.push_back(45);
      nDevices_list.push_back(25);
      nDevices_list.push_back(5);
  }else{
    nDevices_list.push_back(nDevices); //use a specified value of nDevice
  }


  if (custom_node_position){
    //nDevices_list = {80,60,40,20};
    // OPEN NODE POSITION FILE
    std::vector<std::string> row;
    std::string line, word;

    /*
    char cwd[1024];
    getcwd(cwd, sizeof(cwd));
    printf("Current working dir: %s\n", cwd);
      std::system("echo -n '1. Current Directory is '; pwd");
    //system("mkdir temp");
    std::system("pwd");
    */
    //std::cout << "Current working directory: " << tmp << std::endl;

    //std::string centrality_table_file_name = "scratch/der-energy/conf/centrality_table_"+nodes+"-NS3-scalar.csv";
    std::string centrality_table_file_name = "scratch/der-energy/conf/centrality_table_"+nodes+"-NS3-scalar.csv";
    std::cout << centrality_table_file_name << std::endl;
    std::fstream file (centrality_table_file_name, std::ios::in);

    if(file.is_open()) {
      while(getline(file, line)) {
        row.clear();
        std::stringstream str(line);
        while(getline(str, word, ';'))
                row.push_back(word);
        content.push_back(row);
      }
    } else
    {
        std::cout << "Could not open the file centrality_table_\n";
    }
    file.close();

   }

  if (enable_gw_position){
        // OPEN NODE POSITION FILE
      
      std::vector<std::string> row;
      std::string line, word;
      std::fstream file ("scratch/der-energy/conf/gw-position.txt", std::ios::in);
      if(file.is_open()) {
        while(getline(file, line)) {
          row.clear();
          std::stringstream str(line);
          while(getline(str, word, ';'))
                  row.push_back(word);
          content_gw.push_back(row);
        }
      } else std::cout << "Could not open the file_gw\n";
      file.close();
  }
  //for (int nDevices=1000;nDevices<=maxDevices;nDevices+=100){
  //
  for(int n_devices : nDevices_list){

    for (int DR : dr_list){
      
      std::string DR_str = (DR==-1)?"ADR":std::to_string(DR);

      std::cout << "n_devices=" << n_devices << " DR=" << DR_str << std::endl;
      
      //reset data structure (useful for loop)
      int siz = packetsSent.size();
      //no memory allocating
      packetsSent.resize(0);
      packetsSent.resize(siz, 0);

      siz = packetsReceived.size();
      //no memory allocating
      packetsReceived.resize(0);
      packetsReceived.resize(siz, 0);

      // Mobility
      MobilityHelper mobility;
      if (!custom_node_position){
        mobility.SetPositionAllocator ("ns3::UniformDiscPositionAllocator", "rho", DoubleValue (radius),"X", DoubleValue (0.0), "Y", DoubleValue (0.0));
      }
      mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
      
      /************************
       *  Create the channel  *
       ************************/

      // Create the lora channel object
      Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel> ();
      loss->SetPathLossExponent (4.3);
      loss->SetReference (1, 7.7);

      if (realisticChannelModel) {
        // Create the correlated shadowing component
        Ptr<CorrelatedShadowingPropagationLossModel> shadowing = CreateObject<CorrelatedShadowingPropagationLossModel> ();

        // Aggregate shadowing to the logdistance loss
        loss->SetNext (shadowing);

        // Add the effect to the channel propagation loss
        //Ptr<BuildingPenetrationLoss> buildingLoss = CreateObject<BuildingPenetrationLoss> ();

        //shadowing->SetNext (buildingLoss);
      }

      Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel> ();

      Ptr<LoraChannel> channel = CreateObject<LoraChannel> (loss, delay);

      /************************
       *  Create the helpers  *
       ************************/

      // Create the LoraPhyHelper
      LoraPhyHelper phyHelper = LoraPhyHelper ();
      phyHelper.SetChannel (channel);

      // Create the LorawanMacHelper
      LorawanMacHelper macHelper = LorawanMacHelper ();
      macHelper.SetRegion (LorawanMacHelper::ALOHA);

      // Create the LoraHelper
      LoraHelper helper = LoraHelper ();
      helper.EnablePacketTracking (); // Output filename

      //Create the NetworkServerHelper
      NetworkServerHelper nsHelper = NetworkServerHelper ();

      //Create the ForwarderHelper
      ForwarderHelper forHelper = ForwarderHelper ();

      /************************
       *  Create End Devices  *
       ************************/
      
      // Create a set of nodes
      NodeContainer endDevices;
      endDevices.Create (n_devices);

      // Assign a mobility model to each node
      mobility.Install (endDevices);

      // Make it so that nodes are at a certain height > 0
      for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
      {
        Ptr<MobilityModel> mobility = (*j)->GetObject<MobilityModel> ();
        Vector position = mobility->GetPosition ();
        position.z = 1.2;
        mobility->SetPosition (position);
      }


      // SET CUSTOM NODE POSITION
      if (custom_node_position){
        int index = 1;

        // iterate our nodes and print their position.
        for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End(); ++j) {
          Ptr<Node> object = *j;
          Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
          NS_ASSERT (position != 0);
          //Vector pos = position->GetPosition ();
          Vector m_position = position->GetPosition();
          //Vector m_velocity = position->GetVelocity();

//          std::cout << "x=" << m_position.x << ", y=" << m_position.y << ", z=" << m_position.z << std::endl;
          //m_position.x = (atof(content[index][6].c_str())-493747);
          //m_position.y = ((atof(content[index][7].c_str())-1376302));

//          m_position.x = (atof(content[index][2].c_str())-493747);
//          m_position.y = (atof(content[index][3].c_str())-1376302);
          m_position.x = (atof(content[index][2].c_str()));
          m_position.y = (atof(content[index][3].c_str()));

          //std::cout << "x=" << m_position.x << ", y=" << m_position.y << ", z=" << m_position.z << std::endl << std::endl;


          //mob->SetVelocity(m_velocity);
          position->SetPosition(m_position);

          index = index + 1;
          if (index == n_devices+1){
            break;
          }
        }
      }
      // Create the LoraNetDevices of the end devices
      uint8_t nwkId = 54;
      uint32_t nwkAddr = 1864;
      Ptr<LoraDeviceAddressGenerator> addrGen =
          CreateObject<LoraDeviceAddressGenerator> (nwkId, nwkAddr);

      // Create the LoraNetDevices of the end devices
      macHelper.SetAddressGenerator (addrGen);
      phyHelper.SetDeviceType (LoraPhyHelper::ED);
      macHelper.SetDeviceType (LorawanMacHelper::ED_A);

      NetDeviceContainer endDevicesNetDevices = helper.Install (phyHelper, macHelper, endDevices);

      //helper.Install (phyHelper, macHelper, endDevices);

      // Now end devices are connected to the channel

      // Connect trace sources
      for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j) {
          Ptr<Node> node = *j;
          Ptr<LoraNetDevice> loraNetDevice = node->GetDevice (0)->GetObject<LoraNetDevice> ();
          Ptr<LoraPhy> phy = loraNetDevice->GetPhy ();
        }

      /*********************
       *  Create Gateways  *
       *********************/

      // Create the gateway nodes (allocate them uniformely on the disc)
      NodeContainer gateways;
      gateways.Create (nGateways);

      //  Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator> ();
      //  // Make it so that nodes are at a certain height > 0
      //  allocator->Add (Vector (0.0, 0.0, 15.0));
      //  mobility.SetPositionAllocator (allocator);
      mobility.Install (gateways);



      int index = 1;
      
      if (enable_gw_position){
        // iterate our gateways and print their position.
        for (NodeContainer::Iterator j = gateways.Begin (); j != gateways.End(); ++j) {
          Ptr<Node> object = *j;
          Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
          NS_ASSERT (position != 0);
          Vector m_position = position->GetPosition();

          m_position.x = (atof(content_gw[index][1].c_str()));
          m_position.y = (atof(content_gw[index][2].c_str()));        
          m_position.z = (atof(content_gw[index][3].c_str()));          

          position->SetPosition(m_position);
          index = index + 1;

        }
      }
      // Create a netdevice for each gateway
      phyHelper.SetDeviceType (LoraPhyHelper::GW);
      macHelper.SetDeviceType (LorawanMacHelper::GW);
      helper.Install (phyHelper, macHelper, gateways);

      NS_LOG_DEBUG ("Completed configuration");

      /*********************************************
       *  Install applications on the end devices  *
       *********************************************/

      Time appStopTime = Seconds (simulationTime);
      int packetSize = 50;
      PeriodicSenderHelper appHelper = PeriodicSenderHelper ();
      appHelper.SetPeriod (Seconds (appPeriodSeconds));
      appHelper.SetPacketSize (packetSize);
      ApplicationContainer appContainer = appHelper.Install (endDevices);
      
      Ptr<UniformRandomVariable> x = CreateObject<UniformRandomVariable>();
      appContainer.Start (Seconds (0));
      appContainer.Stop (appStopTime);

      std::ofstream outputFile;
      // Delete contents of the file as it is opened
      outputFile.open ("durations.txt", std::ofstream::out | std::ofstream::trunc);
      for (uint8_t sf = 7; sf <= 12; sf++) {
        LoraTxParameters txParams;
        txParams.sf = sf;
        txParams.headerDisabled = 0;
        txParams.codingRate = 1;
        txParams.bandwidthHz = 125000;
        txParams.nPreamble = 8;
        txParams.crcEnabled = 1;
        txParams.lowDataRateOptimizationEnabled =
            LoraPhy::GetTSym (txParams) > MilliSeconds (16) ? true : false;
        Ptr<Packet> pkt = Create<Packet> (packetSize);

        LoraFrameHeader frameHdr = LoraFrameHeader ();
        frameHdr.SetAsUplink ();
        frameHdr.SetFPort (1);
        frameHdr.SetAddress (LoraDeviceAddress ());
        frameHdr.SetAdr (0);
        frameHdr.SetAdrAckReq (0);
        frameHdr.SetFCnt (0);
        pkt->AddHeader (frameHdr);

        LorawanMacHeader macHdr = LorawanMacHeader ();
        macHdr.SetMType (ns3::lorawan::LorawanMacHeader::UNCONFIRMED_DATA_UP);
        macHdr.SetMajor (1);
        pkt->AddHeader (macHdr);

        outputFile << LoraPhy::GetOnAirTime (pkt, txParams).GetMicroSeconds() << " ";
      }
      outputFile.close ();

      /**************************
       *  Create Network Server  *
       ***************************/

      // Create the NS node
      NodeContainer networkServer;
      networkServer.Create (1);

      // Create a NS for the network
      nsHelper.SetEndDevices (endDevices);
      nsHelper.SetGateways (gateways);
      nsHelper.Install (networkServer);


      Ptr<ListPositionAllocator> allocator2 = CreateObject<ListPositionAllocator> ();
      // Make it so that nodes are at a certain height > 0
      allocator2->Add (Vector (4000, -300, 0.0));
      mobility.SetPositionAllocator (allocator2);
      mobility.Install (networkServer);
      //Create a forwarder for each gateway
      forHelper.Install (gateways);

      // Install trace sources
      for (NodeContainer::Iterator node = gateways.Begin (); node != gateways.End(); node++) {
        (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
            "ReceivedPacket", MakeCallback (OnPacketReceptionCallback));
      }

      // Install trace sources
      for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End(); node++) {
        (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetPhy ()->TraceConnectWithoutContext (
            "StartSending", MakeCallback (OnTransmissionCallback));
      }

      // ============================
      // SAVE ADR ALLOCATION AND EXIT
      // ============================
      if (extract_adr){
        std::vector<int> sfQuantity (7);
        sfQuantity = macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
        std::cout << "SF allocation" << std::endl;
        for (uint32_t i = 0; i < 7; ++i)
        {
          std::cout<<"SF"<<std::to_string(i+7)<<": "<<std::to_string(sfQuantity[i]) <<" | ";
        }
        std::cout << std::endl;
        
        adr_buffer << "x,y,SF\n";
        //std::string output_adr = output_dir+"ADR-"+nodes+".csv";
        std::string output_adr = output_dir+"ADR-GW-"+std::to_string(nGateways)+".csv";
      
        for (NodeContainer::Iterator nn = endDevices.Begin (); nn != endDevices.End(); nn++) {
          Ptr<Node> object = *nn;
          Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
          Vector m_position = position->GetPosition();
          Ptr<ClassAEndDeviceLorawanMac> mac = object->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac()->GetObject<ClassAEndDeviceLorawanMac> ();
          adr_buffer << m_position.x << "," << m_position.y <<","<< 12-int(mac->GetDataRate ()) << "\n";  

          std::cout <<"["<< nn-endDevices.Begin() <<"]" <<m_position.x << "," << m_position.y <<","<< 12-int(mac->GetDataRate ()) << std::endl;
        }
      
        std::cout << "--------------------------------" << std::endl;  
        std::cout << "SAVE ADR ALLOCATION " << output_adr << std::endl;
        std::cout << "--------------------------------" << std::endl;  
        std::ofstream outputFileADR(output_adr);

        if (outputFileADR.is_open()) {
            outputFileADR << adr_buffer.rdbuf();
            outputFileADR.close();
            std::cout << "Content written to file successfully!" << std::endl;
        } else {
            std::cerr << "Unable to open file for writing." << std::endl;
        }
          return 0;
      }

      if (en_adr){
        if (!adr_conf.empty()){
          // ============================
          // GET ADR ALLOCATION FROM FILE
          // ============================

          std::cout << "PRENDI DAL FILE DI CONFIGURAZIONE!!!" << std::endl;
          std::vector<std::string> row;
          std::string line, word;
          std::string filename = "scratch/der-energy/"+adr_conf;
          std::fstream file (filename, std::ios::in);

          if(file.is_open()) {
            while(getline(file, line)) {
              row.clear();
              std::stringstream str(line);
              while(getline(str, word, ','))
                      row.push_back(word);
              adr_content.push_back(row);
            }
          } else std::cout << "Could not open the file adr_conf\n";
          file.close();
          std::cout << "ESCI" << std::endl;
          //force Spreading factor
          for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End(); node++) {
            Ptr<Node> object = *node;
            Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
            Ptr<ClassAEndDeviceLorawanMac> mac = (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac()->GetObject<ClassAEndDeviceLorawanMac> ();

            //Search position in adr_content
            for (int i_adr=0;i_adr<86;i_adr++){
//                          std::cout << i_adr <<"," << adr_content[i_adr][0]<<"," << adr_content[i_adr][1]<<"," << adr_content[i_adr][2]<<"," << std::endl;

              double x = atof(adr_content[i_adr][0].c_str());
              double y = atof(adr_content[i_adr][1].c_str());
              int curr_DR = int(12 - atof(adr_content[i_adr][2].c_str()));
              double EPSILON = 0.0001;
//              std::cout << i_adr <<"," << std::endl;
//              std::cout << i_adr <<"," <<x << ","  << y<< "," << curr_DR << std::endl;
              if ((abs(position->GetPosition().x - x ) < EPSILON) && (abs(position->GetPosition().y - y ) < EPSILON)){
                  mac->SetDataRate (curr_DR);  
//                  std::cout <<"["<< i_adr <<"]" <<"x:" << x<<" X:"<< position->GetPosition().x  <<" y: "<< y <<" Y:"<< position->GetPosition().y<< " DR:" << curr_DR << std::endl;
                  std::cout << i_adr <<"," <<position->GetPosition().x << ","  << position->GetPosition().y<< "," << curr_DR << std::endl;
                  break;
              }
            }
//            exit(1);

          }
        }else{
          // ============================
          //          ASSIGN SF 
          // ============================
          std::vector<int> sfQuantity (7);
          sfQuantity = macHelper.SetSpreadingFactorsUp (endDevices, gateways, channel);
          std::cout << "SF allocation" << std::endl;
          for (uint32_t i = 0; i < 7; ++i)
          {
            std::cout<<"SF"<<std::to_string(i+7)<<": "<<std::to_string(sfQuantity[i]) <<" | ";
          }
          std::cout << std::endl;
        }
      }else{
        //force Spreading factor
        for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End(); node++) {
          Ptr<Node> object = *node;
          Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
          Ptr<ClassAEndDeviceLorawanMac> mac = (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac()->GetObject<ClassAEndDeviceLorawanMac> ();          
          mac->SetDataRate (DR);
        }
      }




//      for (NodeContainer::Iterator node = endDevices.Begin (); node != endDevices.End(); node++) {
//
//          Ptr<Node> object = *node;
//          Ptr<MobilityModel> position = object->GetObject<MobilityModel> ();
//          //NS_ASSERT (position != 0);
//          Vector m_position = position->GetPosition();
//
//          Ptr<ClassAEndDeviceLorawanMac> mac = (*node)->GetDevice (0)->GetObject<LoraNetDevice> ()->GetMac()->GetObject<ClassAEndDeviceLorawanMac> ();
//          buffer << node - endDevices.Begin() << ":"  << m_position.x << ":" << m_position.y << ":" <<unsigned(mac->GetDataRate())<< ",";
//      }


/*
      //std::string anim_filename="output_files/lora-animation-nDevices"+std::to_string(n_devices)+"-SF"+std::to_string(12-DR) +"N"+std::to_string(n_devices)+".xml" ;
      std::string SF_str_xml  = (DR==-1)?"ADR":("SF"+std::to_string(12-DR));
      std::string anim_filename=output_dir+"/lora-animation-nDevices"+std::to_string(n_devices)+"-"+SF_str_xml+"-GW" + std::to_string(nGateways)+".xml" ;
      AnimationInterface anim (anim_filename); // Mandatory
      for (uint32_t i = 0; i < endDevices.GetN (); ++i) {
        std::string ED_label = "ED-" + std::to_string(i);
        anim.UpdateNodeDescription (endDevices.Get (i), ED_label); // Optional
        anim.UpdateNodeColor (endDevices.Get (i), 255, 0, 0); // Optional
        anim.UpdateNodeSize (endDevices.Get (i) -> GetId (), 100, 100);
      }
      for (uint32_t i = 0; i < gateways.GetN (); ++i) {
        std::string GW_label = "GW-" + std::to_string(i);
        anim.UpdateNodeDescription (gateways.Get (i), GW_label); // Optional
        anim.UpdateNodeColor (gateways.Get (i), 0, 255, 0); // Optional
        anim.UpdateNodeSize (gateways.Get (i) -> GetId (), 100, 100);
      }

      for (uint32_t i = 0; i < networkServer.GetN (); ++i) {
        std::string GW_label = "NS-" + std::to_string(i);
        anim.UpdateNodeDescription (networkServer.Get (i), GW_label); // Optional
        anim.UpdateNodeColor (networkServer.Get (i), 0, 0, 255); // Optional
        anim.UpdateNodeSize (networkServer.Get (i) -> GetId (), 100, 100);
      }


      anim.EnablePacketMetadata (); // Optional
      anim.EnableWifiMacCounters (Seconds (0), Seconds (10)); //Optional
      anim.EnableWifiPhyCounters (Seconds (0), Seconds (10)); //Optional
*/
      /************************
       * Install Energy Model *
       ************************/

      BasicEnergySourceHelper basicSourceHelper;
      LoraRadioEnergyModelHelper radioEnergyHelper;


      // configure energy source
      basicSourceHelper.Set ("BasicEnergySourceInitialEnergyJ", DoubleValue (10000)); // Energy in J
      basicSourceHelper.Set ("BasicEnergySupplyVoltageV", DoubleValue (3.3));

      radioEnergyHelper.Set ("StandbyCurrentA", DoubleValue (0.0014));
      radioEnergyHelper.Set ("TxCurrentA", DoubleValue (0.028));
      radioEnergyHelper.Set ("SleepCurrentA", DoubleValue (0.0000015));
      radioEnergyHelper.Set ("RxCurrentA", DoubleValue (0.0112));

      radioEnergyHelper.SetTxCurrentModel ("ns3::ConstantLoraTxCurrentModel",
                                          "TxCurrent", DoubleValue (0.028));

      // install source on EDs' nodes
      EnergySourceContainer sources = basicSourceHelper.Install (endDevices);
      Names::Clear();
      if (enable_energy_log){
        for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
        {
          std::string EnergySource_label = "/Names/EnergySource-" + std::to_string(j-endDevices.Begin ());
          //std::cout << EnergySource_label << std::endl;
          Names::Add (EnergySource_label, sources.Get (j-endDevices.Begin ()));
        }
      }


      // install device model
      DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install
          (endDevicesNetDevices, sources);

      /**************
       * Get output *
       **************/

      

      // run for loop from 0 to vecSize


      if (enable_energy_log){
        for (NodeContainer::Iterator j = endDevices.Begin (); j != endDevices.End (); ++j)
        {
          int curr_i = int(j-endDevices.Begin ());
          FileHelper fileHelper;
          std::string EnergySource_label = "/Names/EnergySource-" + std::to_string(curr_i);
          
          std::string SF_str_1  = (DR==-1)?"ADR":("SF"+std::to_string(12-DR));

          fileHelper.ConfigureFile (output_dir+"/battery-level/DEV-"+ std::to_string(curr_i) + "-nDevices" + std::to_string(n_devices)+"-GW" + std::to_string(nGateways) + "-" + SF_str_1, FileAggregator::SPACE_SEPARATED);


          fileHelper.WriteProbe ("ns3::DoubleProbe", EnergySource_label+"/RemainingEnergy", "Output");
          fileHelpers.push_back(fileHelper);

        }
      }
   

/*
      FileHelper fileHelper;
      fileHelper.ConfigureFile ("output_files/battery-level/pippo-0", FileAggregator::SPACE_SEPARATED);
      fileHelper.WriteProbe ("ns3::DoubleProbe", "/Names/EnergySource-0/RemainingEnergy", "Output");

      FileHelper fileHelper2;
      fileHelper2.ConfigureFile ("output_files/battery-level/pippo-1", FileAggregator::SPACE_SEPARATED);
      fileHelper2.WriteProbe ("ns3::DoubleProbe", "/Names/EnergySource-1/RemainingEnergy", "Output");
*/


      ////////////////
      // Simulation //
      ////////////////
      Simulator::Stop (appStopTime + Hours (1.1));
      NS_LOG_INFO ("Running simulation...");
      Simulator::Run ();

      Simulator::Destroy ();

      /////////////////////////////
      // Print results to stdout //
      /////////////////////////////
      NS_LOG_INFO ("Computing performance metrics...");
      /*
      for (int i = 0; i < 6; i++)
      {
        if (packetsSent.at(i) > 0) 
          //std::cout  << n_devices << "," << i+7 << "," << packetsSent.at(i) << "," << packetsReceived.at(i) << std::endl;
          buffer << n_devices << "," << appPeriodSeconds <<","<< simulationTime << ","  <<i+7 << "," << packetsSent.at(i) << "," << packetsReceived.at(i) << "\n";

      }
      */
      LoraPacketTracker &tracker = helper.GetPacketTracker ();
      //std::string tracker_output = (tracker.PrintPhyPacketsPerGw (Seconds (0), Hours (2), n_devices));

      NS_LOG_INFO ("Printing total sent MAC-layer packets and successful MAC-layer packets");
      std::string tracker_output = tracker.CountMacPacketsGlobally (Seconds (0), appStopTime + Hours (1));

      replace(tracker_output.begin(), tracker_output.end(), ' ', ',');
      // nDevices,appPeriodSeconds,simulationTime,SF,totPacketsSent,totReceivedPackets
      std::string SF_str  = (DR==-1)?"ADR":("SF"+std::to_string(12-DR));
      std::cout << n_devices << "," << appPeriodSeconds << "," << simulationTime << ","  << SF_str <<  "," << tracker_output << std::endl;
      buffer <<    n_devices << "," << appPeriodSeconds << "," << simulationTime << ","  << SF_str <<  "," << tracker_output << "\n";
      //get SF for all devices and print


      std::string output_file="";
      if (realisticChannelModel)
        output_file = output_dir+"/results-nDevices"+std::to_string(n_devices)+"-GW" + std::to_string(nGateways) + "-" + SF_str+ "-" + std::to_string(runId) +"-realch";
      else
        output_file = output_dir+"/results-nDevices"+std::to_string(n_devices)+"-GW" + std::to_string(nGateways)+ "-" +SF_str+ "-" + std::to_string(runId) + "";

      if (enable_pkt_log){
        std::cout << "--------------------------------" << std::endl;  
        std::cout << "SAVE PACKET DETAILS FILE TO " << output_file+"-PKT.csv" << std::endl;
        std::cout << "--------------------------------" << std::endl;  
        std::ofstream outputFilePKT(output_file+"-PKT.csv");

        if (outputFilePKT.is_open()) {
            outputFilePKT <<   pkt_log_buffer.rdbuf();
            outputFilePKT.close();
            std::cout << "Content written to file successfully!" << std::endl;
        } else {
            std::cerr << "Unable to open file for writing." << std::endl;
        }
      }

    } // of DR


  } // of n_devics
  
  std::cout << "FINISH!!!"<<std::endl;

  //SAVE BUFFER PKT TO FILE
  std::string output_file="";
  std::string SF_str_2  = (dr_list[0]==-1)?"ADR":("SF"+std::to_string(12-dr_list[0]));
  std::string nDevices_str  = (nDevices==0)?"":("nDevices"+std::to_string(nDevices));

  if (realisticChannelModel)
    //output_file = "output_files/results-nDevices"+std::to_string(nDevices)+"-GW" + std::to_string(nGateways) + "-" + std::to_string(runId) + "-realch";
    output_file = output_dir+"/results-"+nDevices_str+"-"+"GW" + std::to_string(nGateways) + "-" +SF_str_2+ "-"+std::to_string(runId) + "-realch";
  else
    //output_file = "output_files/results-nDevices"+std::to_string(nDevices)+"-GW" + std::to_string(nGateways) + "-" + std::to_string(runId) + "";
    output_file = output_dir+"/results-"+nDevices_str+"-"+"GW" + std::to_string(nGateways) + "-" +SF_str_2+ "-"+std::to_string(runId);

  std::cout << "--------------------------------" << std::endl;
  std::cout << "SAVE STATS FILE TO " << output_file+".csv" << std::endl;
  std::cout << "--------------------------------" << std::endl;

  std::ofstream outputFile(output_file+".csv");
  if (outputFile.is_open()) {
      outputFile << buffer.rdbuf();
      outputFile.close();
      std::cout << "Content written to file successfully." << std::endl;
  } else {
      std::cerr << "Unable to open file for writing." << std::endl;
  }

  return 0;
}
