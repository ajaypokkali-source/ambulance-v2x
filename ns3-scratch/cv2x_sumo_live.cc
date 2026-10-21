/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */

// Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
//
// SPDX-License-Identifier: GPL-2.0-only

/**
 * \ingroup examples
 * \file cttc-nr-v2x-demo-simple.cc
 * \brief A cozy, simple, NR V2X demo (in a tutorial style)
 *
 * This example describes how to setup an NR sidelink out-of-coverage simulation
 * using the 3GPP channel model from TR 37.885. This example simulates a
 * simple topology consist of 2 UEs, where UE-1 transmits, and UE-2 receives.
 *
 * Have a look at the possible parameters to know what you can configure
 * through the command line.
 *
 * With the default configuration, the example will create a flow that will
 * go through a subband or bandwidth part. For that,
 * specifically, one band with a single CC, and one bandwidth part is used.
 *
 * The example will print on-screen the average Packet Inter-Reception (PIR)
 * type 2 computed as defined in 37.885. Moreover, it saves MAC and PHY layer
 * traces in a sqlite3 database using ns-3 stats module. Moreover, since there
 * is only one transmitter in the scenario, sensing is by default not enabled.
 *
 * \code{.unparsed}
$ ./ns3 run "cttc-nr-v2x-demo-simple --help"
    \endcode
 *
 */

/*
 * Include part. Often, you will have to include the headers for an entire module;
 * do that by including the name of the module you need with the suffix "-module.h".
 */

#include "ns3/antenna-module.h"
#include "ns3/applications-module.h"
#include "ns3/config-store-module.h"
#include "ns3/config-store.h"
#include "ns3/core-module.h"
#include "ns3/header.h"
#include "ns3/internet-module.h"
#include "ns3/log.h"
#include "ns3/lte-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/nr-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/stats-module.h"

#include <iomanip>
#include <cerrno>
#include <cstring>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

/*
 * Use, always, the namespace ns3. All the NR classes are inside such namespace.
 */
using namespace ns3;

/*
 * With this line, we will be able to see the logs of the file by enabling the
 * component "CttcNrV2xDemoSimple", in this way:
 *
 * $ export NS_LOG="CttcNrV2xDemoSimple=level_info|prefix_func|prefix_time"
 */
NS_LOG_COMPONENT_DEFINE("CttcNrV2xDemoSimple");

/*
 * Global methods and variable to hook trace sources from different layers of
 * the protocol stack.
 */

/**
 * \brief Method to listen the trace SlPscchScheduling of NrUeMac, which gets
 *        triggered upon the transmission of SCI format 1-A from UE MAC.
 *
 * \param pscchStats Pointer to the UeMacPscchTxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param pscchStatsParams Parameters of the trace source.
 */
void
NotifySlPscchScheduling(UeMacPscchTxOutputStats* pscchStats,
                        const SlPscchUeMacStatParameters pscchStatsParams)
{
    pscchStats->Save(pscchStatsParams);
}

/**
 * \brief Method to listen the trace SlPsschScheduling of NrUeMac, which gets
 *        triggered upon the transmission of SCI format 2-A and data from UE MAC.
 *
 * \param psschStats Pointer to the UeMacPsschTxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param psschStatsParams Parameters of the trace source.
 */
void
NotifySlPsschScheduling(UeMacPsschTxOutputStats* psschStats,
                        const SlPsschUeMacStatParameters psschStatsParams)
{
    psschStats->Save(psschStatsParams);
}

/**
 * \brief Method to listen the trace RxPscchTraceUe of NrSpectrumPhy, which gets
 *        triggered upon the reception of SCI format 1-A.
 *
 * \param pscchStats Pointer to the UePhyPscchRxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param pscchStatsParams Parameters of the trace source.
 */
void
NotifySlPscchRx(UePhyPscchRxOutputStats* pscchStats,
                const SlRxCtrlPacketTraceParams pscchStatsParams)
{
    pscchStats->Save(pscchStatsParams);
}

/**
 * \brief Method to listen the trace RxPsschTraceUe of NrSpectrumPhy, which gets
 *        triggered upon the reception of SCI format 2-A and data.
 *
 * \param psschStats Pointer to the UePhyPsschRxOutputStats class,
 *        which is responsible to write the trace source parameters to a database.
 * \param psschStatsParams Parameters of the trace source.
 */
void
NotifySlPsschRx(UePhyPsschRxOutputStats* psschStats,
                const SlRxDataPacketTraceParams psschStatsParams)
{
    psschStats->Save(psschStatsParams);
}

/**
 * \brief Method to listen the application level traces of type TxWithAddresses
 *        and RxWithAddresses.
 * \param stats Pointer to the UeToUePktTxRxOutputStats class,
 *        which is responsible to write the trace source parameters to a database. *
 * \param node The pointer to the TX or RX node
 * \param localAddrs The local IPV4 address of the node
 * \param txRx The string indicating the type of node, i.e., TX or RX
 * \param p The packet
 * \param srcAddrs The source address from the trace
 * \param dstAddrs The destination address from the trace
 * \param seqTsSizeHeader The SeqTsSizeHeader
 */
void
UePacketTraceDb(UeToUePktTxRxOutputStats* stats,
                Ptr<Node> node,
                const Address& localAddrs,
                std::string txRx,
                Ptr<const Packet> p,
                const Address& srcAddrs,
                const Address& dstAddrs,
                const SeqTsSizeHeader& seqTsSizeHeader)
{
    uint32_t nodeId = node->GetId();
    uint64_t imsi = node->GetDevice(0)->GetObject<NrUeNetDevice>()->GetImsi();
    uint32_t seq = seqTsSizeHeader.GetSeq();
    uint32_t pktSize = p->GetSize() + seqTsSizeHeader.GetSerializedSize();

    stats->Save(txRx, localAddrs, nodeId, imsi, pktSize, srcAddrs, dstAddrs, seq);
}

/*
 * Global variables to count TX/RX packets and bytes.
 */

uint32_t txByteCounter = 0;
uint32_t txPktCounter = 0;

std::vector<uint32_t> rxByteCounterPerUe;
std::vector<uint32_t> rxPktCounterPerUe;

struct LiveBridgeState
{
    int fd{-1};
    sockaddr_in resultAddr{};
    std::vector<Ptr<MobilityModel>> mobility;
    std::vector<bool> active;
    uint64_t currentStep{0};
    bool stopRequested{false};
};

LiveBridgeState g_bridge;

// IPv4 address of every UE, indexed by UE index.
// Used to identify the sender of a received packet.
std::vector<Ipv4Address> g_ueIpv4Addresses;

/*
 * On-demand sockets for intersection UEs.
 *
 * UE8..UE17 = J1..J10
 */
std::vector<Ptr<Socket>> g_i2iSockets;

uint32_t g_i2iSequence = 0;
const uint32_t g_i2iPacketSize = 200;
constexpr uint16_t g_i2iPort = 8001;

// I2I uses a separate multicast destination for each target junction.
// UE8 (J1) maps to 225.0.1.1, ..., UE17 (J10) maps to 225.0.1.10.
// Each destination also gets a distinct sidelink destination L2 ID.
Ipv4Address
GetI2IGroupAddress(uint32_t targetUe)
{
    std::ostringstream address;
    address << "225.0.1." << (targetUe - 7);
    return Ipv4Address(address.str().c_str());
}

uint32_t
GetI2IDestinationL2Id(uint32_t targetUe)
{
    return 100 + targetUe; // UE8..UE17 => destination L2 IDs 108..117
}


class I2IAppHeader : public Header
{
  public:
    I2IAppHeader()
        : m_magic(0x49324931), m_ambulanceUe(0), m_targetUe(0), m_messageId(0), m_hopCount(0)
    {
    }

    I2IAppHeader(uint32_t ambulanceUe,
                 uint32_t targetUe,
                 uint32_t messageId,
                 uint32_t hopCount)
        : m_magic(0x49324931),
          m_ambulanceUe(ambulanceUe),
          m_targetUe(targetUe),
          m_messageId(messageId),
          m_hopCount(hopCount)
    {
    }

    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::I2IAppHeader")
                                .SetParent<Header>()
                                .AddConstructor<I2IAppHeader>();
        return tid;
    }

    TypeId GetInstanceTypeId() const override
    {
        return GetTypeId();
    }

    uint32_t GetSerializedSize() const override
    {
        return 20;
    }

    void Serialize(Buffer::Iterator start) const override
    {
        start.WriteHtonU32(m_magic);
        start.WriteHtonU32(m_ambulanceUe);
        start.WriteHtonU32(m_targetUe);
        start.WriteHtonU32(m_messageId);
        start.WriteHtonU32(m_hopCount);
    }

    uint32_t Deserialize(Buffer::Iterator start) override
    {
        m_magic = start.ReadNtohU32();
        m_ambulanceUe = start.ReadNtohU32();
        m_targetUe = start.ReadNtohU32();
        m_messageId = start.ReadNtohU32();
        m_hopCount = start.ReadNtohU32();
        return 20;
    }

    void Print(std::ostream& os) const override
    {
        os << "I2I ambulanceUE=" << m_ambulanceUe
           << " targetUE=" << m_targetUe
           << " messageId=" << m_messageId
           << " hop=" << m_hopCount;
    }

    bool IsValid() const
    {
        return m_magic == 0x49324931 &&
               m_ambulanceUe < 8 &&
               m_targetUe >= 8 && m_targetUe < 18 &&
               m_hopCount < 10;
    }

    uint32_t GetAmbulanceUe() const { return m_ambulanceUe; }
    uint32_t GetTargetUe() const { return m_targetUe; }
    uint32_t GetMessageId() const { return m_messageId; }
    uint32_t GetHopCount() const { return m_hopCount; }

  private:
    uint32_t m_magic;
    uint32_t m_ambulanceUe;
    uint32_t m_targetUe;
    uint32_t m_messageId;
    uint32_t m_hopCount;
};

void
SendBridgeMessage(const std::string& msg)
{
    if (g_bridge.fd < 0)
    {
        return;
    }

    sendto(g_bridge.fd,
           msg.data(),
           msg.size(),
           0,
           reinterpret_cast<const sockaddr*>(&g_bridge.resultAddr),
           sizeof(g_bridge.resultAddr));
}

int32_t
FindSenderIndex(const Address& from)
{
    if (!InetSocketAddress::IsMatchingType(from))
    {
        return -1;
    }

    Ipv4Address source =
        InetSocketAddress::ConvertFrom(from).GetIpv4();

    for (uint32_t i = 0; i < g_ueIpv4Addresses.size(); ++i)
    {
        if (g_ueIpv4Addresses[i] == source)
        {
            return static_cast<int32_t>(i);
        }
    }

    return -1;
}

void
SendBridgeResult(int32_t senderIndex, uint32_t receiverIndex)
{
    std::ostringstream oss;

    // Protocol:
    // RX <step> <senderUE> <receiverUE> <ns3-time>
    oss << "RX "
        << g_bridge.currentStep << " "
        << senderIndex << " "
        << receiverIndex << " "
        << std::fixed << std::setprecision(6)
        << Simulator::Now().GetSeconds();

    SendBridgeMessage(oss.str());
}

/**
 * Receive callback bound to a specific receiver UE.
 */
void
SendBridgeI2IRx(int32_t senderIndex,
                uint32_t receiverIndex,
                const I2IAppHeader& metadata)
{
    std::ostringstream oss;
    oss << "RX " << g_bridge.currentStep << " "
        << senderIndex << " " << receiverIndex << " "
        << std::fixed << std::setprecision(6)
        << Simulator::Now().GetSeconds()
        << " I2I "
        << metadata.GetAmbulanceUe() << " "
        << metadata.GetTargetUe() << " "
        << metadata.GetMessageId() << " "
        << metadata.GetHopCount();
    SendBridgeMessage(oss.str());
}

void
ReceivePacket(uint32_t receiverIndex,
              Ptr<const Packet> packet,
              const Address& from)
{
    // Preserve complete PHY/application reception counters for all UEs.
    if (receiverIndex < rxByteCounterPerUe.size())
    {
        rxByteCounterPerUe[receiverIndex] += packet->GetSize();
        rxPktCounterPerUe[receiverIndex]++;
    }

    // Only intersection UEs participate in controller decisions.
    if (receiverIndex < 8 || receiverIndex >= 18)
    {
        return;
    }

    const int32_t senderIndex = FindSenderIndex(from);

    // Ambulance-to-intersection packet: legacy emergency-alert payload.
    if (senderIndex >= 0 && senderIndex < 8)
    {
        SendBridgeResult(senderIndex, receiverIndex);
        return;
    }

    // Intersection transmissions are I2I. Parse their explicit metadata;
    // do not infer the ambulance from a mutable source-junction mapping.
    if (senderIndex < 8 || senderIndex >= 18 ||
        senderIndex == static_cast<int32_t>(receiverIndex))
    {
        return;
    }

    Ptr<Packet> copy = packet->Copy();
    SeqTsSizeHeader sequenceHeader;
    if (copy->RemoveHeader(sequenceHeader) == 0)
    {
        return;
    }

    I2IAppHeader metadata;
    if (copy->RemoveHeader(metadata) == 0 || !metadata.IsValid())
    {
        return;
    }

    // Diagnostic: show packets delivered to the intersection application
    // before the final target-payload check. No line here means the packet
    // did not reach this application's ReceivePacket callback.
    std::cout << "[I2I RX RAW] UE" << senderIndex
              << " -> UE" << receiverIndex
              << " targetUE=" << metadata.GetTargetUe()
              << " ambulanceUE=" << metadata.GetAmbulanceUe()
              << " messageId=" << metadata.GetMessageId()
              << " hop=" << metadata.GetHopCount()
              << " t=" << Simulator::Now().GetSeconds() << std::endl;

    // Each I2I destination has its own group address/L2 ID, and only that
    // target UE is configured with a matching RECEIVE sidelink bearer.
    // Keep this payload check as a second guard against misdelivery.
    if (metadata.GetTargetUe() != receiverIndex)
    {
        std::cout << "[I2I RX FILTER] actual UE" << receiverIndex
                  << " is not intended UE" << metadata.GetTargetUe()
                  << std::endl;
        return;
    }

    SendBridgeI2IRx(senderIndex, receiverIndex, metadata);
}


void
SendBridgeI2ITx(uint32_t senderIndex,
                uint32_t targetIndex,
                uint32_t ambulanceUe,
                uint32_t messageId,
                uint32_t hopCount)
{
    std::ostringstream oss;
    oss << "TXI2I " << g_bridge.currentStep << " "
        << senderIndex << " " << targetIndex << " "
        << ambulanceUe << " " << messageId << " " << hopCount << " "
        << std::fixed << std::setprecision(6)
        << Simulator::Now().GetSeconds();
    SendBridgeMessage(oss.str());
}

void
SendI2IPacket(uint32_t senderIndex,
              uint32_t targetIndex,
              uint32_t ambulanceUe,
              uint32_t messageId,
              uint32_t hopCount)
{
    if (senderIndex < 8 || senderIndex >= 18 ||
        targetIndex < 8 || targetIndex >= 18 ||
        senderIndex == targetIndex ||
        ambulanceUe >= 8 || hopCount >= 10)
    {
        std::cerr << "[I2I] Invalid metadata/source/target for packet" << std::endl;
        return;
    }

    if (senderIndex >= g_i2iSockets.size() || !g_i2iSockets[senderIndex])
    {
        std::cerr << "[I2I] No socket for UE" << senderIndex << std::endl;
        return;
    }

    Ptr<Packet> packet = Create<Packet>(g_i2iPacketSize);

    I2IAppHeader metadata(ambulanceUe, targetIndex, messageId, hopCount);
    packet->AddHeader(metadata);

    SeqTsSizeHeader sequenceHeader;
    sequenceHeader.SetSeq(g_i2iSequence++);
    sequenceHeader.SetSize(packet->GetSize());
    packet->AddHeader(sequenceHeader);

    // Route the datagram to this target junction's unique I2I group.
    // Its matching sidelink RECEIVE bearer is installed only on targetIndex.
    const int sent = g_i2iSockets[senderIndex]->SendTo(
        packet,
        0,
        InetSocketAddress(GetI2IGroupAddress(targetIndex), g_i2iPort));
    if (sent < 0)
    {
        std::cerr << "[I2I] Socket send failed UE" << senderIndex
                  << " -> UE" << targetIndex << std::endl;
        return;
    }

    SendBridgeI2ITx(senderIndex, targetIndex, ambulanceUe, messageId, hopCount);

    std::cout << "[I2I TX] UE" << senderIndex << " -> UE" << targetIndex
              << " at t=" << Simulator::Now().GetSeconds()
              << " s ambulanceUE=" << ambulanceUe
              << " messageId=" << messageId
              << " hop=" << hopCount << std::endl;
}

void
TransmitPacket(uint32_t senderIndex, Ptr<const Packet> packet)
{
    txByteCounter += packet->GetSize();
    txPktCounter++;

    std::ostringstream oss;

    // Protocol:
    // TX <step> <senderUE> <ns3-time>
    oss << "TX "
        << g_bridge.currentStep << " "
        << senderIndex << " "
        << std::fixed << std::setprecision(6)
        << Simulator::Now().GetSeconds();

    SendBridgeMessage(oss.str());
}

bool
InitLiveBridge(uint16_t inputPort, uint16_t resultPort, uint16_t ueNum)
{
    g_bridge.fd = socket(AF_INET, SOCK_DGRAM, 0);

    if (g_bridge.fd < 0)
    {
        std::cerr << "[BRIDGE] socket() failed: " << std::strerror(errno) << std::endl;
        return false;
    }

    int yes = 1;
    setsockopt(g_bridge.fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in listenAddr{};
    listenAddr.sin_family = AF_INET;
    listenAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    listenAddr.sin_port = htons(inputPort);

    if (bind(g_bridge.fd,
             reinterpret_cast<const sockaddr*>(&listenAddr),
             sizeof(listenAddr)) < 0)
    {
        std::cerr << "[BRIDGE] bind(" << inputPort << ") failed: "
                  << std::strerror(errno) << std::endl;
        close(g_bridge.fd);
        g_bridge.fd = -1;
        return false;
    }

    int flags = fcntl(g_bridge.fd, F_GETFL, 0);
    fcntl(g_bridge.fd, F_SETFL, flags | O_NONBLOCK);

    g_bridge.resultAddr.sin_family = AF_INET;
    g_bridge.resultAddr.sin_port = htons(resultPort);
    inet_pton(AF_INET, "127.0.0.1", &g_bridge.resultAddr.sin_addr);

    g_bridge.mobility.resize(ueNum);
    g_bridge.active.assign(ueNum, false);

    std::cout << "[BRIDGE] Ready: SUMO state UDP 127.0.0.1:" << inputPort
              << ", RX results UDP 127.0.0.1:" << resultPort << std::endl;

    // Tell Python that ns-3 is ready before it starts SUMO.
    SendBridgeMessage("READY");

    return true;
}

void
PollLiveBridge()
{
    if (g_bridge.fd >= 0)
    {
        char buffer[1024];

        while (true)
        {
            sockaddr_in src{};
            socklen_t srcLen = sizeof(src);

            ssize_t n = recvfrom(
                g_bridge.fd,
                buffer,
                sizeof(buffer) - 1,
                0,
                reinterpret_cast<sockaddr*>(&src),
                &srcLen);

            if (n <= 0)
            {
                break;
            }

            buffer[n] = '\0';

            std::istringstream iss{std::string(buffer)};
            std::string command;
            iss >> command;

            if (command == "STATE")
            {
                uint64_t step = 0;
                uint32_t slot = 0;
                int active = 0;
                double x = 0.0;
                double y = 0.0;

                iss >> step >> slot >> active >> x >> y;

                if (iss && slot < g_bridge.mobility.size())
                {
                    g_bridge.currentStep = step;
                    g_bridge.active[slot] = active != 0;

                    if (g_bridge.mobility[slot])
                    {
                        if (active != 0)
                        {
                            g_bridge.mobility[slot]->SetPosition(Vector(x, y, 1.5));
                        }
                        else
                        {
                            const double far = 100000.0 + 10000.0 * slot;
                            g_bridge.mobility[slot]->SetPosition(Vector(far, far, 1.5));
                        }
                    }
                }
            }
            else if (command == "I2I")
            {
                uint32_t senderIndex = 0;
                uint32_t targetIndex = 0;
                uint32_t ambulanceUe = 0;
                uint32_t messageId = 0;
                uint32_t hopCount = 0;

                iss >> senderIndex >> targetIndex >> ambulanceUe >> messageId >> hopCount;
                if (iss)
                {
                    SendI2IPacket(senderIndex,
                                  targetIndex,
                                  ambulanceUe,
                                  messageId,
                                  hopCount);
                }
                else
                {
                    std::cerr << "[I2I] Invalid bridge command: "
                              << buffer << std::endl;
                }
            }
            else if (command == "STOP")
            {
                g_bridge.stopRequested = true;
                Simulator::Stop();
                break;
            }
        }
    }

    if (!g_bridge.stopRequested)
    {
        Simulator::Schedule(MilliSeconds(10), &PollLiveBridge);
    }
}

void
CloseLiveBridge()
{
    if (g_bridge.fd >= 0)
    {
        close(g_bridge.fd);
        g_bridge.fd = -1;
    }
}

/*
 * Global variable used to compute PIR
 */
uint64_t pirCounter =
    0; //!< counter to count how many time we computed the PIR. It is used to compute average PIR
Time lastPktRxTime; //!< Global variable to store the RX time of a packet
Time pir;           //!< Global variable to store PIR value

/**
 * \brief This method listens to the packet sink application trace Rx.
 * \param packet The packet
 * \param from The address of the transmitter
 */
void
ComputePir(Ptr<const Packet> packet, const Address&)
{
    if (pirCounter == 0 && lastPktRxTime.GetSeconds() == 0.0)
    {
        // this the first packet, just store the time and get out
        lastPktRxTime = Simulator::Now();
        return;
    }
    pir = pir + (Simulator::Now() - lastPktRxTime);
    lastPktRxTime = Simulator::Now();
    pirCounter++;
}

int
main(int argc, char* argv[])
{
    GlobalValue::Bind("SimulatorImplementationType",
                      StringValue("ns3::RealtimeSimulatorImpl"));

    /*
     * Variables that represent the parameters we will accept as input by the
     * command line. Each of them is initialized with a default value.
     */
    // Scenario parameters (that we will use inside this script):
    uint16_t interUeDistance = 20; // meters
    bool logging = false;

    // Traffic parameters (that we will use inside this script:)
    bool useIPv6 = false; // default IPV4
    uint32_t udpPacketSizeBe = 200;
    double dataRateBe = 16; // 16 kilobits per second
    bool harqEnabled = true;
    Time delayBudget = Seconds(0); // Use T2 as selection window edge

    // Simulation parameters.
    Time simTime = Seconds(600);
    uint16_t bridgeInputPort = 5000;
    uint16_t bridgeResultPort = 5001;
    // Sidelink bearers activation time
    Time slBearersActivationTime = Seconds(2.0);

    // NR parameters. We will take the input from the command line, and then we
    // will pass them inside the NR module.
    uint16_t numerologyBwpSl = 2;
    double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
    uint16_t bandwidthBandSl = 400;         // Multiple of 100 KHz; 400 = 40 MHz
    double txPower = 23;                    // dBm

    // Where we will store the output files.
    std::string simTag = "default";
    std::string outputDir = "./";

    /*
     * From here, we instruct the ns3::CommandLine class of all the input parameters
     * that we may accept as input, as well as their description, and the storage
     * variable.
     */
    CommandLine cmd(__FILE__);

    cmd.AddValue("interUeDistance", "The distance among the UEs in the topology", interUeDistance);
    cmd.AddValue("logging", "Enable logging", logging);
    cmd.AddValue("useIPv6", "Use IPv6 instead of IPv4", useIPv6);
    cmd.AddValue("packetSizeBe",
                 "packet size in bytes to be used by best effort traffic",
                 udpPacketSizeBe);
    cmd.AddValue("dataRateBe",
                 "The data rate in kilobits per second for best effort traffic",
                 dataRateBe);
    cmd.AddValue("simTime", "Simulation time in seconds", simTime);
    cmd.AddValue("bridgeInputPort",
                 "UDP port receiving SUMO state updates",
                 bridgeInputPort);
    cmd.AddValue("bridgeResultPort",
                 "UDP port sending NR-V2X reception events",
                 bridgeResultPort);
    cmd.AddValue("slBearerActivationTime",
                 "Sidelik bearer activation time in seconds",
                 slBearersActivationTime);
    cmd.AddValue("numerologyBwpSl",
                 "The numerology to be used in sidelink bandwidth part",
                 numerologyBwpSl);
    cmd.AddValue("centralFrequencyBandSl",
                 "The central frequency to be used for sidelink band/channel",
                 centralFrequencyBandSl);
    cmd.AddValue("bandwidthBandSl",
                 "The system bandwidth to be used for sidelink",
                 bandwidthBandSl);
    cmd.AddValue("txPower", "total tx power in dBm", txPower);
    cmd.AddValue("simTag",
                 "tag to be appended to output filenames to distinguish simulation campaigns",
                 simTag);
    cmd.AddValue("outputDir", "directory where to store simulation results", outputDir);

    // Parse the command line
    cmd.Parse(argc, argv);

    // Final simulation time is the addition of:
    // simTime + slBearersActivationTime + 10 ms additional delay for UEs to activate the bearers
    Time finalSlBearersActivationTime = slBearersActivationTime + Seconds(0.01);
    Time finalSimTime = simTime + finalSlBearersActivationTime;
    std::cout << "Final Simulation duration " << finalSimTime.GetSeconds() << std::endl;

    /*
     * Check if the frequency is in the allowed range.
     * If you need to add other checks, here is the best position to put them.
     */
    NS_ABORT_IF(centralFrequencyBandSl > 6e9);

    /*
     * If the logging variable is set to true, enable the log of some components
     * through the code. The same effect can be obtained through the use
     * of the NS_LOG environment variable:
     *
     * export NS_LOG="UdpClient=level_info|prefix_time|prefix_func|prefix_node:UdpServer=..."
     *
     * Usually, the environment variable way is preferred, as it is more customizable,
     * and more expressive.
     */
    if (logging)
    {
        LogLevel logLevel =
            (LogLevel)(LOG_PREFIX_FUNC | LOG_PREFIX_TIME | LOG_PREFIX_NODE | LOG_LEVEL_ALL);
        LogComponentEnable("UdpClient", logLevel);
        LogComponentEnable("UdpServer", logLevel);
        LogComponentEnable("LtePdcp", logLevel);
        LogComponentEnable("NrSlHelper", logLevel);
        LogComponentEnable("NrSlUeRrc", logLevel);
        LogComponentEnable("NrUePhy", logLevel);
        LogComponentEnable("NrSpectrumPhy", logLevel);
    }

    /*
     * Default values for the simulation. We are progressively removing all
     * the instances of SetDefault, but we need it for legacy code (LTE)
     */
    Config::SetDefault("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue(999999999));

    /*
     * Create a NodeContainer for the UEs, name it as per their traffic type.
     */
    NodeContainer ueVoiceContainer;
    uint16_t ueNum = 600;
    ueVoiceContainer.Create(ueNum);

    /*
     * Assign mobility to the UEs.
     *  1. Set mobility model type.
     *  2. Assign position to the UEss
     *  3. Install mobility model
     */

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    Ptr<ListPositionAllocator> positionAllocUe = CreateObject<ListPositionAllocator>();

    for (uint16_t i = 0; i < ueNum; ++i)
    {
        const double far = 100000.0 + 10000.0 * i;
        positionAllocUe->Add(Vector(far, far, 1.5));
    }

    mobility.SetPositionAllocator(positionAllocUe);
    mobility.Install(ueVoiceContainer);

    rxByteCounterPerUe.assign(ueNum, 0);
    rxPktCounterPerUe.assign(ueNum, 0);

    g_bridge.mobility.resize(ueNum);
    for (uint16_t i = 0; i < ueNum; ++i)
    {
        g_bridge.mobility[i] =
            ueVoiceContainer.Get(i)->GetObject<MobilityModel>();
    }

    if (!InitLiveBridge(bridgeInputPort, bridgeResultPort, ueNum))
    {
        return 1;
    }

    /* The default topology is the following:
     *
     *         UE1..........(20 m)..........UE2
     *   (0.0, 0.0, 1.5)               (20, 0.0, 1.5)
     */

    /*
     * Setup the NR module. We create the various helpers needed for the
     * NR simulation:
     * - EpcHelper, which will setup the core network
     * - NrHelper, which takes care of creating and connecting the various
     * part of the NR stack
     */
    Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper>();
    Ptr<NrHelper> nrHelper = CreateObject<NrHelper>();

    // Put the pointers inside nrHelper
    nrHelper->SetEpcHelper(epcHelper);

    /*
     * Spectrum division. We create one operational band, containing
     * one component carrier, and a single bandwidth part
     * centered at the frequency specified by the input parameters.
     * We will use the StreetCanyon channel modeling.
     */
    BandwidthPartInfoPtrVector allBwps;
    CcBwpCreator ccBwpCreator;
    const uint8_t numCcPerBand = 1;

    /* Create the configuration for the CcBwpHelper. SimpleOperationBandConf
     * creates a single BWP per CC
     */
    CcBwpCreator::SimpleOperationBandConf bandConfSl(centralFrequencyBandSl,
                                                     bandwidthBandSl,
                                                     numCcPerBand,
                                                     BandwidthPartInfo::V2V_Highway);

    // By using the configuration created, it is time to make the operation bands
    OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc(bandConfSl);

    /*
     * The configured spectrum division is:
     * ------------Band1--------------
     * ------------CC1----------------
     * ------------BwpSl--------------
     */

    /*
     * Attributes of ThreeGppChannelModel still cannot be set in our way.
     * TODO: Coordinate with Tommaso
     */
    Config::SetDefault("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue(MilliSeconds(100)));
    nrHelper->SetChannelConditionModelAttribute("UpdatePeriod", TimeValue(MilliSeconds(0)));
    nrHelper->SetPathlossAttribute("ShadowingEnabled", BooleanValue(false));

    /*
     * Initialize channel and pathloss, plus other things inside bandSl. If needed,
     * the band configuration can be done manually, but we leave it for more
     * sophisticated examples. For the moment, this method will take care
     * of all the spectrum initialization needs.
     */
    nrHelper->InitializeOperationBand(&bandSl);
    allBwps = CcBwpCreator::GetAllBwps({bandSl});

    /*
     * allBwps contains all the spectrum configuration needed for the nrHelper.
     *
     * Now, we can setup the attributes. We can have three kind of attributes:
     * (i) parameters that are valid for all the bandwidth parts and applies to
     * all nodes, (ii) parameters that are valid for all the bandwidth parts
     * and applies to some node only, and (iii) parameters that are different for
     * every bandwidth parts. The approach is:
     *
     * - for (i): Configure the attribute through the helper, and then install;
     * - for (ii): Configure the attribute through the helper, and then install
     * for the first set of nodes. Then, change the attribute through the helper,
     * and install again;
     * - for (iii): Install, and then configure the attributes by retrieving
     * the pointer needed, and calling "SetAttribute" on top of such pointer.
     *
     */

    Packet::EnableChecking();
    Packet::EnablePrinting();

    /*
     *  Case (i): Attributes valid for all the nodes
     */
    // Core latency
    epcHelper->SetAttribute("S1uLinkDelay", TimeValue(MilliSeconds(0)));

    /*
     * Antennas for all the UEs
     * We are not using beamforming in SL, rather we are using
     * quasi-omnidirectional transmission and reception, which is the default
     * configuration of the beams.
     */
    nrHelper->SetUeAntennaAttribute("NumRows", UintegerValue(1));
    nrHelper->SetUeAntennaAttribute("NumColumns", UintegerValue(2));
    nrHelper->SetUeAntennaAttribute("AntennaElement",
                                    PointerValue(CreateObject<IsotropicAntennaModel>()));

    nrHelper->SetUePhyAttribute("TxPower", DoubleValue(txPower));

    // NR Sidelink attribute of UE MAC, which are would be common for all the UEs
    nrHelper->SetUeMacTypeId(NrSlUeMac::GetTypeId());
    nrHelper->SetUeMacAttribute("EnableSensing", BooleanValue(false));
    nrHelper->SetUeMacAttribute("T1", UintegerValue(2));
    nrHelper->SetUeMacAttribute("T2", UintegerValue(33));
    nrHelper->SetUeMacAttribute("ActivePoolId", UintegerValue(0));

    uint8_t bwpIdForGbrMcptt = 0;

    nrHelper->SetBwpManagerTypeId(TypeId::LookupByName("ns3::NrSlBwpManagerUe"));
    // following parameter has no impact at the moment because:
    // 1. No support for PQI based mapping between the application and the LCs
    // 2. No scheduler to consider PQI
    // However, till such time all the NR SL examples should use GBR_MC_PUSH_TO_TALK
    // because we hard coded the PQI 65 in UE RRC.
    nrHelper->SetUeBwpManagerAlgorithmAttribute("GBR_MC_PUSH_TO_TALK",
                                                UintegerValue(bwpIdForGbrMcptt));

    std::set<uint8_t> bwpIdContainer;
    bwpIdContainer.insert(bwpIdForGbrMcptt);

    /*
     * We miss many other parameters. By default, not configuring them is equivalent
     * to use the default values. Please, have a look at the documentation to see
     * what are the default values for all the attributes you are not seeing here.
     */

    /*
     * Case (ii): Attributes valid for a subset of the nodes
     */

    // NOT PRESENT IN THIS SIMPLE EXAMPLE

    /*
     * We have configured the attributes we needed. Now, install and get the pointers
     * to the NetDevices, which contains all the NR stack:
     */
    NetDeviceContainer ueVoiceNetDev = nrHelper->InstallUeDevice(ueVoiceContainer, allBwps);

    /*
     * Case (iii): Go node for node and change the attributes we have to setup
     * per-node.
     */

    // When all the configuration is done, explicitly call UpdateConfig ()
    for (auto it = ueVoiceNetDev.Begin(); it != ueVoiceNetDev.End(); ++it)
    {
        DynamicCast<NrUeNetDevice>(*it)->UpdateConfig();
    }

    /*
     * Configure Sidelink. We create the following helpers needed for the
     * NR Sidelink, i.e., V2X simulation:
     * - NrSlHelper, which will configure the UEs protocol stack to be ready to
     *   perform Sidelink related procedures.
     * - EpcHelper, which takes care of triggering the call to EpcUeNas class
     *   to establish the NR Sidelink bearer (s). We note that, at this stage
     *   just communicate the pointer of already instantiated EpcHelper object,
     *   which is the same pointer communicated to the NrHelper above.
     */
    Ptr<NrSlHelper> nrSlHelper = CreateObject<NrSlHelper>();
    // Put the pointers inside NrSlHelper
    nrSlHelper->SetEpcHelper(epcHelper);

    /*
     * Set the SL error model and AMC
     * Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1,
     *                   ns3::NrEesmIrT2, ns3::NrLteMiErrorModel
     * AMC type: NrAmc::ShannonModel or NrAmc::ErrorModel
     */
    std::string errorModel = "ns3::NrEesmIrT1";
    nrSlHelper->SetSlErrorModel(errorModel);
    nrSlHelper->SetUeSlAmcAttribute("AmcModel", EnumValue(NrAmc::ErrorModel));

    /*
     * Set the SL scheduler attributes
     * In this example we use NrSlUeMacSchedulerFixedMcs scheduler, which uses
     * a fixed MCS value
     */
    nrSlHelper->SetNrSlSchedulerTypeId(NrSlUeMacSchedulerFixedMcs::GetTypeId());
    nrSlHelper->SetUeSlSchedulerAttribute("Mcs", UintegerValue(14));

    /*
     * Very important method to configure UE protocol stack, i.e., it would
     * configure all the SAPs among the layers, setup callbacks, configure
     * error model, configure AMC, and configure ChunkProcessor in Interference
     * API.
     */
    nrSlHelper->PrepareUeForSidelink(ueVoiceNetDev, bwpIdContainer);

    /*
     * Start preparing for all the sub Structs/RRC Information Element (IEs)
     * of LteRrcSap::SidelinkPreconfigNr. This is the main structure, which would
     * hold all the pre-configuration related to Sidelink.
     */

    // SlResourcePoolNr IE
    LteRrcSap::SlResourcePoolNr slResourcePoolNr;
    // get it from pool factory
    Ptr<NrSlCommResourcePoolFactory> ptrFactory = Create<NrSlCommResourcePoolFactory>();
    /*
     * Above pool factory is created to help the users of the simulator to create
     * a pool with valid default configuration. Please have a look at the
     * constructor of NrSlCommResourcePoolFactory class.
     *
     * In the following, we show how one could change those default pool parameter
     * values as per the need.
     */
    std::vector<std::bitset<1>> slBitmap = {1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 1, 1};
    ptrFactory->SetSlTimeResources(slBitmap);
    ptrFactory->SetSlSensingWindow(100); // T0 in ms
    ptrFactory->SetSlSelectionWindow(5);
    ptrFactory->SetSlFreqResourcePscch(10); // PSCCH RBs
    ptrFactory->SetSlSubchannelSize(50);
    ptrFactory->SetSlMaxNumPerReserve(3);
    std::list<uint16_t> resourceReservePeriodList = {0, 100}; // in ms
    ptrFactory->SetSlResourceReservePeriodList(resourceReservePeriodList);
    // Once parameters are configured, we can create the pool
    LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool();
    slResourcePoolNr = pool;

    // Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
    LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
    slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
    // Pool id, ranges from 0 to 15
    uint16_t poolId = 0;
    LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
    slResourcePoolIdNr.id = poolId;
    slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
    slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

    // Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
    LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
    // Array for pools, we insert the pool in the array as per its poolId
    slBwpPoolConfigCommonNr.slTxPoolSelectedNormal[slResourcePoolIdNr.id] = slresoPoolConfigNr;

    // Configure the BWP IE
    LteRrcSap::Bwp bwp;
    bwp.numerology = numerologyBwpSl;
    bwp.symbolsPerSlots = 14;
    bwp.rbPerRbg = 1;
    bwp.bandwidth = bandwidthBandSl;

    // Configure the SlBwpGeneric IE
    LteRrcSap::SlBwpGeneric slBwpGeneric;
    slBwpGeneric.bwp = bwp;
    slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum(14);
    slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum(0);

    // Configure the SlBwpConfigCommonNr IE
    LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
    slBwpConfigCommonNr.haveSlBwpGeneric = true;
    slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
    slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
    slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

    // Configure the SlFreqConfigCommonNr IE, which hold the array to store
    // the configuration of all Sidelink BWP (s).
    LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
    // Array for BWPs. Here we will iterate over the BWPs, which
    // we want to use for SL.
    for (const auto& it : bwpIdContainer)
    {
        // it is the BWP id
        slFreConfigCommonNr.slBwpList[it] = slBwpConfigCommonNr;
    }

    // Configure the TddUlDlConfigCommon IE
    LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
    tddUlDlConfigCommon.tddPattern = "DL|DL|DL|F|UL|UL|UL|UL|UL|UL|";

    // Configure the SlPreconfigGeneralNr IE
    LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
    slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

    // Configure the SlUeSelectedConfig IE
    LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
    slUeSelectedPreConfig.slProbResourceKeep = 0;
    // Configure the SlPsschTxParameters IE
    LteRrcSap::SlPsschTxParameters psschParams;
    psschParams.slMaxTxTransNumPssch = 5;
    // Configure the SlPsschTxConfigList IE
    LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
    pscchTxConfigList.slPsschTxParameters[0] = psschParams;
    slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

    /*
     * Finally, configure the SidelinkPreconfigNr This is the main structure
     * that needs to be communicated to NrSlUeRrc class
     */
    LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
    slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
    slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
    slPreConfigNr.slPreconfigFreqInfoList[0] = slFreConfigCommonNr;

    // Communicate the above pre-configuration to the NrSlHelper
    nrSlHelper->InstallNrSlPreConfiguration(ueVoiceNetDev, slPreConfigNr);

    /****************************** End SL Configuration ***********************/

    /*
     * Fix the random streams
     */
    int64_t stream = 1;
    stream += nrHelper->AssignStreams(ueVoiceNetDev, stream);
    stream += nrSlHelper->AssignStreams(ueVoiceNetDev, stream);

    /*
     * Configure the IP stack, and activate NR Sidelink bearer (s) as per the
     * configured time.
     *
     * This example supports IPV4 and IPV6
     */

    InternetStackHelper internet;
    internet.Install(ueVoiceContainer);
    stream += internet.AssignStreams(ueVoiceContainer, stream);
    uint32_t dstL2Id = 255;
    Ipv4Address groupAddress4("225.0.0.0"); // use multicast address as destination
    Ipv6Address groupAddress6("ff0e::1");   // use multicast address as destination
    Address remoteAddress;
    Address localAddress;
    uint16_t port = 8000;
    Ptr<LteSlTft> tft;
    SidelinkInfo slInfo;
    slInfo.m_castType = SidelinkInfo::CastType::Groupcast;
    slInfo.m_dstL2Id = dstL2Id;
    slInfo.m_rri = MilliSeconds(100);
    slInfo.m_pdb = delayBudget;
    slInfo.m_harqEnabled = harqEnabled;

    if (!useIPv6)
    {
        Ipv4InterfaceContainer ueIpIface;
        ueIpIface = epcHelper->AssignUeIpv4Address(ueVoiceNetDev);

        // Keep the IPv4 address of every UE so an RX callback
        // can identify which UE transmitted the packet.
        g_ueIpv4Addresses.resize(ueVoiceContainer.GetN());

        for (uint32_t u = 0; u < ueVoiceContainer.GetN(); ++u)
        {
            g_ueIpv4Addresses[u] = ueIpIface.GetAddress(u);
        }

        // set the default gateway for the UE
        Ipv4StaticRoutingHelper ipv4RoutingHelper;
        for (uint32_t u = 0; u < ueVoiceContainer.GetN(); ++u)
        {
            Ptr<Node> ueNode = ueVoiceContainer.Get(u);
            // Set the default gateway for the UE
            Ptr<Ipv4StaticRouting> ueStaticRouting =
                ipv4RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv4>());
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress(), 1);
        }
        remoteAddress = InetSocketAddress(groupAddress4, port);
        localAddress = InetSocketAddress(Ipv4Address::GetAny(), port);
        tft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress4, slInfo);
        // This shared group is retained for ambulance V2I beacons.
        nrSlHelper->ActivateNrSlBearer(finalSlBearersActivationTime, ueVoiceNetDev, tft);

        /*
         * I2I target-specific sidelink groups.
         *
         * For every target J1..J10, all junction UEs can transmit to that
         * target-specific group, but only the target UE is activated with
         * a matching RECEIVE bearer. This prevents adjacent and distant
         * junctions from receiving every I2I packet at the sidelink bearer.
         *
         * The NR-V2X v1.1 scenario uses Groupcast bearers; this is a groupcast
         * destination with a single configured receiver, not NR Unicast.
         */
        NetDeviceContainer intersectionNetDevices;
        for (uint16_t ue = 8; ue < 18; ++ue)
        {
            intersectionNetDevices.Add(ueVoiceNetDev.Get(ue));
        }

        for (uint32_t targetUe = 8; targetUe < 18; ++targetUe)
        {
            const Ipv4Address targetGroupAddress = GetI2IGroupAddress(targetUe);
            SidelinkInfo targetSlInfo;
            targetSlInfo.m_castType = SidelinkInfo::CastType::Groupcast;
            targetSlInfo.m_dstL2Id = GetI2IDestinationL2Id(targetUe);
            targetSlInfo.m_rri = MilliSeconds(100);
            targetSlInfo.m_pdb = delayBudget;
            targetSlInfo.m_harqEnabled = harqEnabled;

            Ptr<LteSlTft> txTft = Create<LteSlTft>(
                LteSlTft::Direction::TRANSMIT,
                targetGroupAddress,
                targetSlInfo);
            nrSlHelper->ActivateNrSlBearer(
                finalSlBearersActivationTime,
                intersectionNetDevices,
                txTft);

            NetDeviceContainer targetNetDevice;
            targetNetDevice.Add(ueVoiceNetDev.Get(targetUe));
            Ptr<LteSlTft> rxTft = Create<LteSlTft>(
                LteSlTft::Direction::RECEIVE,
                targetGroupAddress,
                targetSlInfo);
            nrSlHelper->ActivateNrSlBearer(
                finalSlBearersActivationTime,
                targetNetDevice,
                rxTft);

            std::cout << "[I2I] UE" << targetUe
                      << " is the only receiver for group " << targetGroupAddress
                      << " (L2 ID " << targetSlInfo.m_dstL2Id << ")"
                      << std::endl;
        }

        /*
         * On-demand I2I sockets for J1..J10.
         *
         * UE8  = J1
         * UE9  = J2
         * ...
         * UE17 = J10
         */
        g_i2iSockets.resize(
            ueVoiceContainer.GetN()
        );

        for (uint16_t i = 8; i < 18; ++i)
        {
            Ptr<Socket> i2iSocket =
                Socket::CreateSocket(
                    ueVoiceContainer.Get(i),
                    UdpSocketFactory::GetTypeId()
                );

            if (i2iSocket->Bind() == -1)
            {
                std::cerr
                    << "[I2I] Bind failed for UE"
                    << i
                    << std::endl;

                return 1;
            }

            // Do not connect to the shared V2I multicast address. SendTo()
            // chooses the target-specific I2I group for each packet.

            g_i2iSockets[i] =
                i2iSocket;

            std::cout
                << "[I2I] Socket ready on UE"
                << i
                << std::endl;
        }

    }
    else
    {
        Ipv6InterfaceContainer ueIpIface;
        ueIpIface = epcHelper->AssignUeIpv6Address(ueVoiceNetDev);

        // set the default gateway for the UE
        Ipv6StaticRoutingHelper ipv6RoutingHelper;
        for (uint32_t u = 0; u < ueVoiceContainer.GetN(); ++u)
        {
            Ptr<Node> ueNode = ueVoiceContainer.Get(u);
            // Set the default gateway for the UE
            Ptr<Ipv6StaticRouting> ueStaticRouting =
                ipv6RoutingHelper.GetStaticRouting(ueNode->GetObject<Ipv6>());
            ueStaticRouting->SetDefaultRoute(epcHelper->GetUeDefaultGatewayAddress6(), 1);
        }
        remoteAddress = Inet6SocketAddress(groupAddress6, port);
        localAddress = Inet6SocketAddress(Ipv6Address::GetAny(), port);
        tft = Create<LteSlTft>(LteSlTft::Direction::BIDIRECTIONAL, groupAddress6, slInfo);
        // Set Sidelink bearers
        nrSlHelper->ActivateNrSlBearer(finalSlBearersActivationTime, ueVoiceNetDev, tft);
    }

    /*
     * Configure the applications:
     * Client app: OnOff application configure to generate CBR traffic
     * Server app: PacketSink application.
     */

    // Set Application in the UEs
    OnOffHelper sidelinkClient("ns3::UdpSocketFactory", remoteAddress);
    sidelinkClient.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));
    std::string dataRateBeString = std::to_string(dataRateBe) + "kb/s";
    std::cout << "Data rate " << DataRate(dataRateBeString) << std::endl;
    sidelinkClient.SetConstantRate(DataRate(dataRateBeString), udpPacketSizeBe);

    ApplicationContainer clientApps;

    // UE0..UE7 are the eight ambulances.
    // Every ambulance broadcasts the emergency application traffic.
    for (uint16_t i = 0; i < 8; ++i)
    {
        clientApps.Add(
            sidelinkClient.Install(ueVoiceContainer.Get(i))
        );

        std::cout
            << "[NR-V2X] TX application installed on ambulance UE"
            << i
            << std::endl;
    }

    // onoff application will send the first packet at :
    // finalSlBearersActivationTime + ((Pkt size in bits) / (Data rate in bits per sec))
    clientApps.Start(finalSlBearersActivationTime);
    clientApps.Stop(finalSimTime);

    // Output app start, stop and duration
    double realAppStart =
        finalSlBearersActivationTime.GetSeconds() +
        ((double)udpPacketSizeBe * 8.0 / (DataRate(dataRateBeString).GetBitRate()));
    double appStopTime = (finalSimTime).GetSeconds();

    std::cout << "App start time " << realAppStart << " sec" << std::endl;
    std::cout << "App stop time " << appStopTime << " sec" << std::endl;

    ApplicationContainer serverApps;
    PacketSinkHelper sidelinkSink("ns3::UdpSocketFactory", localAddress);
    sidelinkSink.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));

    // UE0..UE7 are ambulances (transmitters).
    // UE8..UE599 are intersections/cars and receive emergency traffic.
    for (uint16_t i = 8; i < ueNum; ++i)
    {
        serverApps.Add(
            sidelinkSink.Install(ueVoiceContainer.Get(i))
        );
    }

    serverApps.Start(Seconds(0.0));

    // I2I uses a separate application port so V2I broadcast reception is
    // unchanged. Each junction listens on this port; the target-specific
    // sidelink RECEIVE bearer above determines which junction actually gets
    // a given I2I group packet.
    ApplicationContainer i2iServerApps;
    if (!useIPv6)
    {
        PacketSinkHelper i2iSink(
            "ns3::UdpSocketFactory",
            InetSocketAddress(Ipv4Address::GetAny(), g_i2iPort));
        i2iSink.SetAttribute("EnableSeqTsSizeHeader", BooleanValue(true));

        for (uint16_t targetUe = 8; targetUe < 18; ++targetUe)
        {
            i2iServerApps.Add(i2iSink.Install(ueVoiceContainer.Get(targetUe)));
        }
        i2iServerApps.Start(Seconds(0.0));

        for (uint32_t targetUe = 8; targetUe < 18; ++targetUe)
        {
            Ptr<PacketSink> sink = DynamicCast<PacketSink>(i2iServerApps.Get(targetUe - 8));
            if (!sink || !sink->TraceConnectWithoutContext(
                             "Rx",
                             MakeBoundCallback(&ReceivePacket, targetUe)))
            {
                std::cerr << "[I2I] Failed to connect RX trace for UE"
                          << targetUe << std::endl;
                return 1;
            }
        }
    }

    /*
     * Hook the traces, to be used to compute average PIR and to data to be
     * stored in a database
     */

    // Trace receptions; use the following to be robust to node ID changes
    std::ostringstream path;
    // Trace receptions for every non-ambulance UE.
    // Their PacketSink is ApplicationList/0 because the sinks are
    // installed on these nodes without an OnOff application.
    for (uint16_t i = 8; i < ueNum; ++i)
    {
        path << "/NodeList/" << ueVoiceContainer.Get(i)->GetId()
             << "/ApplicationList/0/$ns3::PacketSink/Rx";

        Config::ConnectWithoutContext(
            path.str(),
            MakeBoundCallback(
                &ReceivePacket,
                static_cast<uint32_t>(i)
            )
        );

        path.str("");
    }

    // Trace transmissions from all eight ambulance applications.
    // Bind the actual UE index so the Python side knows who transmitted.
    for (uint16_t i = 0; i < 8; ++i)
    {
        path << "/NodeList/" << ueVoiceContainer.Get(i)->GetId()
             << "/ApplicationList/0/$ns3::OnOffApplication/Tx";

        Config::ConnectWithoutContext(
            path.str(),
            MakeBoundCallback(
                &TransmitPacket,
                static_cast<uint32_t>(i)
            )
        );

        path.str("");
    }

    // Datebase setup
    std::string exampleName = simTag + "-" + "nr-v2x-simple-demo";
    SQLiteOutput db(outputDir + exampleName + ".db");

    UeMacPscchTxOutputStats pscchStats;
    pscchStats.SetDb(&db, "pscchTxUeMac");
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                  "ComponentCarrierMapUe/*/NrUeMac/SlPscchScheduling",
                                  MakeBoundCallback(&NotifySlPscchScheduling, &pscchStats));

    UeMacPsschTxOutputStats psschStats;
    psschStats.SetDb(&db, "psschTxUeMac");
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/"
                                  "ComponentCarrierMapUe/*/NrUeMac/SlPsschScheduling",
                                  MakeBoundCallback(&NotifySlPsschScheduling, &psschStats));

    UePhyPscchRxOutputStats pscchPhyStats;
    pscchPhyStats.SetDb(&db, "pscchRxUePhy");
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
        "SpectrumPhy/RxPscchTraceUe",
        MakeBoundCallback(&NotifySlPscchRx, &pscchPhyStats));

    UePhyPsschRxOutputStats psschPhyStats;
    psschPhyStats.SetDb(&db, "psschRxUePhy");
    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::NrUeNetDevice/ComponentCarrierMapUe/*/NrUePhy/"
        "SpectrumPhy/RxPsschTraceUe",
        MakeBoundCallback(&NotifySlPsschRx, &psschPhyStats));

    UeToUePktTxRxOutputStats pktStats;
    pktStats.SetDb(&db, "pktTxRx");

    if (!useIPv6)
    {
        // Set Tx traces
        for (uint32_t ac = 0; ac < clientApps.GetN(); ac++)
        {
            Ipv4Address localAddrs = clientApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv4L3Protocol>()
                                         ->GetAddress(1, 0)
                                         .GetLocal();
            std::cout << "Tx address: " << localAddrs << std::endl;
            clientApps.Get(ac)->TraceConnect("TxWithSeqTsSize",
                                             "tx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               ueVoiceContainer.Get(0),
                                                               localAddrs));
        }

        // Set Rx traces
        for (uint32_t ac = 0; ac < serverApps.GetN(); ac++)
        {
            Ipv4Address localAddrs = serverApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv4L3Protocol>()
                                         ->GetAddress(1, 0)
                                         .GetLocal();
            std::cout << "Rx address: " << localAddrs << std::endl;
            serverApps.Get(ac)->TraceConnect("RxWithSeqTsSize",
                                             "rx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               ueVoiceContainer.Get(1),
                                                               localAddrs));
        }
    }
    else
    {
        // Set Tx traces
        for (uint32_t ac = 0; ac < clientApps.GetN(); ac++)
        {
            clientApps.Get(ac)->GetNode()->GetObject<Ipv6L3Protocol>()->AddMulticastAddress(
                groupAddress6);
            Ipv6Address localAddrs = clientApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv6L3Protocol>()
                                         ->GetAddress(1, 1)
                                         .GetAddress();
            std::cout << "Tx address: " << localAddrs << std::endl;
            clientApps.Get(ac)->TraceConnect("TxWithSeqTsSize",
                                             "tx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               ueVoiceContainer.Get(0),
                                                               localAddrs));
        }

        // Set Rx traces
        for (uint32_t ac = 0; ac < serverApps.GetN(); ac++)
        {
            serverApps.Get(ac)->GetNode()->GetObject<Ipv6L3Protocol>()->AddMulticastAddress(
                groupAddress6);
            Ipv6Address localAddrs = serverApps.Get(ac)
                                         ->GetNode()
                                         ->GetObject<Ipv6L3Protocol>()
                                         ->GetAddress(1, 1)
                                         .GetAddress();
            std::cout << "Rx address: " << localAddrs << std::endl;
            serverApps.Get(ac)->TraceConnect("RxWithSeqTsSize",
                                             "rx",
                                             MakeBoundCallback(&UePacketTraceDb,
                                                               &pktStats,
                                                               ueVoiceContainer.Get(1),
                                                               localAddrs));
        }
    }

    Simulator::ScheduleNow(&PollLiveBridge);

    Simulator::Stop(finalSimTime);
    Simulator::Run();

    std::cout << "Total Tx bits = " << txByteCounter * 8 << std::endl;
    std::cout << "Total Tx packets = " << txPktCounter << std::endl;

    uint64_t totalRxPackets = 0;
    uint64_t totalRxBytes = 0;

    std::cout << "\n===== LIVE NR-V2X RESULTS =====" << std::endl;
    std::cout << "UE0..UE7   = ambulances" << std::endl;
    std::cout << "UE8..UE17  = intersections J1..J10" << std::endl;
    std::cout << "UE18..UE551 = SUMO vehicles" << std::endl;

    for (uint16_t i = 8; i < ueNum; ++i)
    {
        totalRxPackets += rxPktCounterPerUe[i];
        totalRxBytes += rxByteCounterPerUe[i];

        std::cout << "UE" << i
                  << " RX packets = " << rxPktCounterPerUe[i]
                  << std::endl;
    }

    std::cout << "Total Rx packets = " << totalRxPackets << std::endl;
    std::cout << "Total Rx bytes   = " << totalRxBytes << std::endl;

    /*
     * VERY IMPORTANT: Do not forget to empty the database cache, which would
     * dump the data store towards the end of the simulation in to a database.
     */
    pktStats.EmptyCache();
    pscchStats.EmptyCache();
    psschStats.EmptyCache();
    pscchPhyStats.EmptyCache();
    psschPhyStats.EmptyCache();

    CloseLiveBridge();
    Simulator::Destroy();
    return 0;
}
