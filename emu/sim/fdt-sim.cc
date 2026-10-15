// fdt-sim: discrete-event simulation (not real time) of the shared IEEE 802.11 channel of Fleet-DT v2 with
// the hosts inside ns-3 (scale study, emu/experiments/addendum/ADDENDUM.md, section A2).
//
// The channel, the Wi-Fi nodes, the AP monitor, the ground truth, the external BSS and the jammer are the
// code of fdt-emu.cc (emu/ns3), unchanged. In place of the VMs, each node runs IPv4 and TCP of ns-3 and:
//   vessel i  the broker of the vessel with the regulator core of the Mosquitto plugin (fdt_reg.c, the same
//             C code, linked), listeners 1883 (control bridge) and 1884 (data bridge), and the sources of
//             the run (the release instants and sizes that emu/sim/simrun.py computes with fdt_source.py)
//   GS        the bridges b<i>-ctl and b<i>-data of the GS broker (MQTT 3.1.1 over TCP; QoS 2 delivered at
//             PUBREL, as Mosquitto) and the link to the AS: at each phase 2 the GS events (demands, vessel
//             logs, accepted data messages) and the monitor batches go to the driver on stdout as one JSON
//             line; the driver runs ASCore.phase2(t) (fleetdt/as2, unchanged) and replies on stdin with the
//             tables and the compute time; the tables leave on the control bridges after that time
//
// Outputs (directory --outdir): gt.bin and summary.json in the formats of fdt-emu (emu/README.md).
#include "minijson.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/propagation-module.h"
#include "ns3/spectrum-module.h"
#include "ns3/waveform-generator-helper.h"
#include "ns3/waveform-generator.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-phy-listener.h"

#include <algorithm>
#include <array>
#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <time.h>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <sstream>

extern "C"
{
#include "fdt_dtp2.h"
#include "fdt_reg.h"
}

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("FdtSim");

namespace
{

// ---------------------------------------------------------------------------------------------
// clocks and global state

std::atomic<bool> g_stop{false};

int64_t
WallNs()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int64_t g_originNs = 0; // host CLOCK_REALTIME at simulator time 0
// actions that need the shift of the scenario time (run start - origin), run just before Simulator::Run
std::vector<std::function<void(double)>> g_deferred;

int64_t
NowNs()
{
    return Simulator::Now().GetNanoSeconds();
}

int64_t
HostUs(int64_t simNs)
{
    return (g_originNs + simNs) / 1000;
}

uint32_t g_M = 0;          // vessels
int g_nAp2 = -1;           // node index of the external AP (or -1)
int g_nSta2 = -1;          // node index of the external STA
int g_nJam = -1;           // node index of the jammer
std::vector<std::string> g_names;               // node names: gs, v1..vM, ap2, sta2, jam
std::map<Mac48Address, int> g_wifiMacToNode;    // Wi-Fi MAC -> node index
std::vector<Mac48Address> g_wifiMac;            // node index -> Wi-Fi MAC
std::vector<Ptr<WifiNetDevice>> g_wifiDev;      // node index -> device
std::vector<Ptr<WifiPhy>> g_phy;                // node index -> PHY

// ---------------------------------------------------------------------------------------------
// ground truth (gt.bin)

#pragma pack(push, 1)

struct GtRec
{
    int64_t t_ns;      // start (TX: PPDU start; RX: PPDU start at the receiver; STATE: state start)
    uint32_t dur_ns;   // duration
    uint32_t uid;      // packet UID of the MSDU (matches TX and RX of the same frame)
    uint32_t rate_kbps;
    uint16_t node;     // node that logs the event
    uint16_t size;     // MPDU bytes (MAC header + body + FCS); STATE: 0
    uint16_t ip_len;   // IP packet bytes (data frames with an IPv4 payload), else 0
    int16_t src;       // node index of addr2 (transmitter); -1 unknown
    int16_t dst;       // node index of addr1 (receiver); -2 group address, -1 unknown
    uint16_t sport;    // TCP/UDP ports of the IPv4 payload, else 0
    uint16_t dport;
    uint8_t ev;        // 0 TX, 1 RX_OK, 2 RX_ERR, 3 RX_DROP, 4 STATE, 5 JAM
    uint8_t kind;      // 0 data, 1 beacon, 2 management, 3 ACK, 4 other control, 5 jammer, 6 QoS/null
    uint8_t ip_proto;  // IPv4 protocol, else 0
    uint8_t flags;     // bit0 retry, bit1 QoS data, bits 4..7 TID
    uint8_t reason;    // RX_DROP: WifiPhyRxfailureReason; STATE: WifiPhyState; TX: power dBm + 128
    uint8_t pad;
};

#pragma pack(pop)

static_assert(sizeof(GtRec) == 40, "GtRec layout");

FILE* g_gt = nullptr;
uint64_t g_gtCount = 0;
std::string g_gtStates = "ap"; // ap | all | none

// The simulation thread never blocks on the file: records go to a buffer that a writer thread drains
// (a write() that waits for the file system would show up as lag of the real-time scheduler).
std::vector<GtRec> g_gtBuf;
std::vector<std::vector<GtRec>> g_gtQueue;
std::mutex g_gtMutex;
std::condition_variable g_gtCv;
bool g_gtDone = false;
std::thread g_gtThread;

void
GtWriterLoop()
{
    std::unique_lock<std::mutex> lk(g_gtMutex);
    while (true)
    {
        g_gtCv.wait(lk, [] { return g_gtDone || !g_gtQueue.empty(); });
        std::vector<std::vector<GtRec>> q;
        q.swap(g_gtQueue);
        bool done = g_gtDone;
        lk.unlock();
        for (auto& b : q)
        {
            fwrite(b.data(), sizeof(GtRec), b.size(), g_gt);
        }
        fflush(g_gt);
        lk.lock();
        if (done && g_gtQueue.empty())
        {
            return;
        }
    }
}

void
GtFlush()
{
    if (g_gtBuf.empty())
    {
        return;
    }
    std::vector<GtRec> b;
    b.reserve(16384);
    b.swap(g_gtBuf);
    {
        std::lock_guard<std::mutex> lk(g_gtMutex);
        g_gtQueue.push_back(std::move(b));
    }
    g_gtCv.notify_one();
}

void
GtWrite(const GtRec& r)
{
    if (g_gt)
    {
        g_gtBuf.push_back(r);
        g_gtCount++;
        if (g_gtBuf.size() >= 16384)
        {
            GtFlush();
        }
    }
}

void
GtClose()
{
    if (!g_gt)
    {
        return;
    }
    GtFlush();
    {
        std::lock_guard<std::mutex> lk(g_gtMutex);
        g_gtDone = true;
    }
    g_gtCv.notify_one();
    g_gtThread.join();
    fclose(g_gt);
    g_gt = nullptr;
}

// ---------------------------------------------------------------------------------------------
// frame description from a MAC header and its body

struct FrameInfo
{
    WifiMacHeader hdr;
    uint32_t size = 0; // MPDU bytes
    uint32_t uid = 0;
    uint16_t ipLen = 0;
    uint8_t ipProto = 0;
    uint16_t sport = 0, dport = 0;
    uint8_t kind = 0;
    int src = -1, dst = -1;
};

int
NodeOf(const Mac48Address& a)
{
    if (a.IsGroup())
    {
        return -2;
    }
    auto it = g_wifiMacToNode.find(a);
    return it == g_wifiMacToNode.end() ? -1 : it->second;
}

uint8_t
KindOf(const WifiMacHeader& h)
{
    if (h.IsBeacon())
    {
        return 1;
    }
    if (h.IsMgt())
    {
        return 2;
    }
    if (h.IsAck())
    {
        return 3;
    }
    if (h.IsCtl())
    {
        return 4;
    }
    if (h.IsData() && !h.HasData())
    {
        return 6;
    }
    return 0;
}

// body = MSDU (LLC/SNAP + payload), possibly followed by trailer bytes that are ignored
void
ParseBody(FrameInfo& fi, Ptr<const Packet> body, uint32_t bodyLen)
{
    if (fi.kind != 0 || bodyLen < 8 + 20)
    {
        return;
    }
    uint8_t b[8 + 24];
    body->CopyData(b, sizeof b);
    uint16_t etype = (b[6] << 8) | b[7];
    if (etype != 0x0800)
    {
        return;
    }
    const uint8_t* ip = b + 8;
    fi.ipLen = (ip[2] << 8) | ip[3];
    fi.ipProto = ip[9];
    uint32_t ihl = (ip[0] & 0x0f) * 4;
    if ((fi.ipProto == 6 || fi.ipProto == 17) && ihl == 20 && bodyLen >= 8 + 24)
    {
        fi.sport = (ip[20] << 8) | ip[21];
        fi.dport = (ip[22] << 8) | ip[23];
    }
}

FrameInfo
InfoFromPsdu(Ptr<const WifiPsdu> psdu)
{
    FrameInfo fi;
    Ptr<const WifiMpdu> mpdu = *psdu->begin();
    fi.hdr = mpdu->GetHeader();
    fi.size = psdu->GetSize();
    fi.uid = uint32_t(mpdu->GetPacket()->GetUid());
    fi.kind = KindOf(fi.hdr);
    fi.src = fi.hdr.IsCtl() && !fi.hdr.IsRts() ? -1 : NodeOf(fi.hdr.GetAddr2());
    fi.dst = NodeOf(fi.hdr.GetAddr1());
    ParseBody(fi, mpdu->GetPacket(), mpdu->GetPacket()->GetSize());
    return fi;
}

// pdu = MAC header + body + FCS (as given by the RxOk / RxError traces)
FrameInfo
InfoFromPdu(Ptr<const Packet> pdu)
{
    FrameInfo fi;
    Ptr<Packet> p = pdu->Copy();
    fi.size = p->GetSize();
    fi.uid = uint32_t(pdu->GetUid());
    p->RemoveHeader(fi.hdr);
    fi.kind = KindOf(fi.hdr);
    fi.src = fi.hdr.IsCtl() && !fi.hdr.IsRts() ? -1 : NodeOf(fi.hdr.GetAddr2());
    fi.dst = NodeOf(fi.hdr.GetAddr1());
    uint32_t body = p->GetSize() >= 4 ? p->GetSize() - 4 : 0;
    ParseBody(fi, p, body);
    return fi;
}

GtRec
GtFrom(uint8_t ev, int node, int64_t t, int64_t dur, const FrameInfo& fi, uint32_t rateKbps)
{
    GtRec r{};
    r.t_ns = t;
    r.dur_ns = uint32_t(std::max<int64_t>(0, dur));
    r.uid = fi.uid;
    r.rate_kbps = rateKbps;
    r.node = uint16_t(node);
    r.size = uint16_t(std::min<uint32_t>(fi.size, 65535));
    r.ip_len = fi.ipLen;
    r.src = int16_t(fi.src);
    r.dst = int16_t(fi.dst);
    r.sport = fi.sport;
    r.dport = fi.dport;
    r.ev = ev;
    r.kind = fi.kind;
    r.ip_proto = fi.ipProto;
    r.flags = (fi.hdr.IsRetry() ? 1 : 0) | (fi.hdr.IsQosData() ? 2 : 0) |
              (fi.hdr.IsQosData() ? (fi.hdr.GetQosTid() << 4) : 0);
    return r;
}

uint32_t
RateKbps(const WifiTxVector& v)
{
    return uint32_t(v.GetMode().GetDataRate(v) / 1000);
}

// ---------------------------------------------------------------------------------------------
// AP monitor (PR6, SPEC_v2 4.3)

struct FrRec
{
    int64_t start, end, frameEnd; // ns
    int sta, dir, ipLen, ok, kind;
    int ac; // access category of a QoS data frame (ns-3 AcIndex: 0 BE, 1 BK, 2 VI, 3 VO), -1 otherwise
    uint32_t rate;
    Mac48Address tx; // transmitter of the frame (the receiver of its ACK)
    bool needAck, done;
};

std::vector<std::string> g_monOut; // monitor batches since the last phase 2

class Monitor;
Monitor* g_mon = nullptr;

class Monitor
{
  public:
    int m_sock = -1;
    sockaddr_in m_dst{};
    int64_t m_periodNs = 5000000;
    int64_t m_from = 0; // start of the open batch (ns)
    std::deque<FrRec> m_fr;
    // busy timeline: closed intervals and the current one
    std::vector<std::pair<int64_t, int64_t>> m_busy;
    int64_t m_curStart = -1, m_busyUntil = -1;
    uint64_t m_batches = 0, m_sendErr = 0, m_records = 0;
    Mac48Address m_apMac;
    Time m_sifs = MicroSeconds(10);

    void Busy(int64_t s, int64_t e)
    {
        if (e <= s)
        {
            return;
        }
        if (m_curStart < 0 || s > m_busyUntil)
        {
            if (m_curStart >= 0)
            {
                m_busy.emplace_back(m_curStart, m_busyUntil);
            }
            m_curStart = s;
            m_busyUntil = e;
        }
        else
        {
            m_busyUntil = std::max(m_busyUntil, e);
        }
    }

    int StaOf(const Mac48Address& a)
    {
        if (a.IsGroup())
        {
            return 0;
        }
        int n = NodeOf(a);
        return (n >= 1 && n <= int(g_M)) ? n : -1;
    }

    static int MonKind(uint8_t k)
    {
        return k == 0 ? 0 : (k == 1 ? 1 : 2);
    }

    // access category of the frame from the TID (= UP) of a QoS data frame (IEEE 802.11-2020 Table 10-1)
    static int AcOf(const WifiMacHeader& h)
    {
        if (!h.IsQosData())
        {
            return -1;
        }
        return int(QosUtilsMapTidToAc(h.GetQosTid()));
    }

    void Add(const FrameInfo& fi, int64_t start, int64_t frameEnd, uint32_t rate, bool apTx)
    {
        FrRec r{};
        r.start = start;
        r.frameEnd = frameEnd;
        r.end = frameEnd;
        r.rate = rate;
        r.kind = MonKind(fi.kind);
        r.ac = AcOf(fi.hdr);
        r.ipLen = fi.ipLen;
        r.tx = fi.hdr.GetAddr2();
        if (apTx)
        {
            r.sta = StaOf(fi.hdr.GetAddr1());
            r.dir = 1;
        }
        else
        {
            r.sta = StaOf(fi.hdr.GetAddr2());
            r.dir = (fi.hdr.GetAddr1() == m_apMac) ? 0 : (fi.hdr.IsFromDs() ? 1 : 0);
            if (r.sta == 0)
            {
                r.sta = -1;
            }
        }
        r.needAck = !fi.hdr.GetAddr1().IsGroup() && (fi.hdr.IsData() || fi.hdr.IsMgt()) &&
                    !(fi.hdr.IsQosData() && fi.hdr.IsQosNoAck());
        r.ok = 1;
        r.done = !r.needAck;
        m_fr.push_back(r);
        if (r.needAck)
        {
            Simulator::Schedule(NanoSeconds(frameEnd - NowNs()) + MicroSeconds(150),
                                &Monitor::Timeout,
                                this,
                                frameEnd,
                                r.tx);
        }
    }

    // a failed reception (FCS error) at the AP: one record with ok = 0
    void AddFailed(const FrameInfo& fi, int64_t start, int64_t end, uint32_t rate)
    {
        FrRec r{};
        r.start = start;
        r.frameEnd = r.end = end;
        r.rate = rate;
        r.kind = MonKind(fi.kind);
        r.ac = AcOf(fi.hdr);
        r.ipLen = fi.ipLen;
        r.sta = StaOf(fi.hdr.GetAddr2());
        if (r.sta == 0)
        {
            r.sta = -1;
        }
        r.dir = (fi.hdr.GetAddr1() == m_apMac) ? 0 : (fi.hdr.IsFromDs() ? 1 : 0);
        r.ok = 0;
        r.done = true;
        m_fr.push_back(r);
    }

    // an ACK to `to` that starts at ackStart and ends at ackEnd
    void Ack(const Mac48Address& to, int64_t ackStart, int64_t ackEnd, uint32_t rate, bool apTx)
    {
        for (auto it = m_fr.rbegin(); it != m_fr.rend(); ++it)
        {
            if (it->needAck && !it->done && it->tx == to)
            {
                int64_t gap = ackStart - it->frameEnd;
                if (gap >= m_sifs.GetNanoSeconds() - 3000 && gap <= m_sifs.GetNanoSeconds() + 30000)
                {
                    it->done = true;
                    it->ok = 1;
                    it->end = ackEnd;
                    return;
                }
            }
        }
        // ACK of an exchange whose frame the AP did not decode (e.g. external BSS)
        FrRec r{};
        r.start = ackStart;
        r.frameEnd = r.end = ackEnd;
        r.rate = rate;
        r.kind = 2;
        r.ac = -1;
        r.sta = apTx ? StaOf(to) : -1;
        r.dir = apTx ? 1 : 0;
        r.ok = 1;
        r.done = true;
        m_fr.push_back(r);
    }

    void Timeout(int64_t frameEnd, Mac48Address tx)
    {
        for (auto& r : m_fr)
        {
            if (!r.done && r.frameEnd == frameEnd && r.tx == tx)
            {
                r.done = true;
                r.ok = 0;
                r.end = r.frameEnd;
            }
        }
    }

    // periodic batch (every m_periodNs)
    void Emit()
    {
        Batch();
        Simulator::Schedule(NanoSeconds(m_periodNs), &Monitor::Emit, this);
    }

    // extra batches at the end of Q(t) and at the end of W(t) of every step (paper Section V-B): host
    // times stepEpoch + k Ts + offset, so that the AS has the records of Q(t) when phase 2 starts
    int64_t m_stepNs = 0;
    std::vector<int64_t> m_flushOffNs;
    int64_t m_stepEpochSimNs = 0; // simulator time of a step start

    void ScheduleFlushes()
    {
        if (m_stepNs <= 0 || m_flushOffNs.empty())
        {
            return;
        }
        int64_t now = NowNs();
        for (int64_t off : m_flushOffNs)
        {
            int64_t k = (now - m_stepEpochSimNs - off) / m_stepNs;
            int64_t t = m_stepEpochSimNs + off + k * m_stepNs;
            while (t <= now)
            {
                t += m_stepNs;
            }
            Simulator::Schedule(NanoSeconds(t - now), &Monitor::Flush, this, off);
        }
    }

    void Flush(int64_t off)
    {
        if (NowNs() > m_from)
        {
            Batch();
        }
        Simulator::Schedule(NanoSeconds(m_stepNs), &Monitor::Flush, this, off);
    }

    void Batch()
    {
        int64_t to = NowNs();
        std::string js;
        js.reserve(4096);
        char buf[160];
        snprintf(buf,
                 sizeof buf,
                 "{\"from_us\":%lld,\"to_us\":%lld,\"busy\":[",
                 (long long)HostUs(m_from),
                 (long long)HostUs(to));
        js += buf;
        bool first = true;
        auto emitIv = [&](int64_t s, int64_t e) {
            s = std::max(s, m_from);
            e = std::min(e, to);
            if (e <= s)
            {
                return;
            }
            snprintf(buf,
                     sizeof buf,
                     "%s[%lld,%lld]",
                     first ? "" : ",",
                     (long long)HostUs(s),
                     (long long)HostUs(e));
            js += buf;
            first = false;
        };
        for (auto& iv : m_busy)
        {
            emitIv(iv.first, iv.second);
        }
        m_busy.erase(std::remove_if(m_busy.begin(),
                                    m_busy.end(),
                                    [to](const std::pair<int64_t, int64_t>& iv) { return iv.second <= to; }),
                     m_busy.end());
        if (m_curStart >= 0)
        {
            emitIv(m_curStart, m_busyUntil);
        }
        js += "],\"fr\":[";
        first = true;
        std::vector<FrRec> out;
        for (auto it = m_fr.begin(); it != m_fr.end();)
        {
            if (it->done && it->end <= to)
            {
                out.push_back(*it);
                it = m_fr.erase(it);
            }
            else
            {
                ++it;
            }
        }
        std::sort(out.begin(), out.end(), [](const FrRec& a, const FrRec& b) { return a.start < b.start; });
        for (auto& r : out)
        {
            snprintf(buf,
                     sizeof buf,
                     "%s[%lld,%lld,%d,%d,%d,%u,%d,%d,%d]",
                     first ? "" : ",",
                     (long long)HostUs(r.start),
                     (long long)HostUs(r.end),
                     r.sta,
                     r.dir,
                     r.ipLen,
                     r.rate,
                     r.ok,
                     r.kind,
                     r.ac);
            js += buf;
            first = false;
        }
        js += "]}";
        m_records += out.size();
        g_monOut.push_back(js); // to the driver at the next phase 2
        m_batches++;
        m_from = to;
    }
};

class MonListener : public WifiPhyListener
{
  public:
    void NotifyRxStart(Time d) override
    {
        g_mon->Busy(NowNs(), NowNs() + d.GetNanoSeconds());
    }

    void NotifyRxEndOk() override
    {
    }

    void NotifyRxEndError(const WifiTxVector&) override
    {
    }

    void NotifyTxStart(Time d, dBm_u) override
    {
        g_mon->Busy(NowNs(), NowNs() + d.GetNanoSeconds());
    }

    void NotifyCcaBusyStart(Time d, WifiChannelListType t, const std::vector<Time>&) override
    {
        if (t == WIFI_CHANLIST_PRIMARY)
        {
            g_mon->Busy(NowNs(), NowNs() + d.GetNanoSeconds());
        }
    }

    void NotifySwitchingStart(Time) override
    {
    }

    void NotifySleep() override
    {
    }

    void NotifyOff() override
    {
    }

    void NotifyWakeup() override
    {
    }

    void NotifyOn() override
    {
    }
};

// ---------------------------------------------------------------------------------------------
// PHY traces of every Wi-Fi node (ground truth) and of the AP (monitor)

struct RxCtx
{
    WifiTxVector txv;
    int64_t payloadStart = 0;
    bool valid = false;
};

std::vector<RxCtx> g_rx;
std::vector<uint64_t> g_txCount, g_rxOk, g_rxErr, g_rxDrop;

void
OnTxPsdu(int node, WifiConstPsduMap psdus, WifiTxVector txv, double txPowerW)
{
    Ptr<const WifiPsdu> psdu = psdus.begin()->second;
    FrameInfo fi = InfoFromPsdu(psdu);
    int64_t dur = WifiPhy::CalculateTxDuration(psdu->GetSize(), txv, g_phy[node]->GetPhyBand())
                      .GetNanoSeconds();
    uint32_t rate = RateKbps(txv);
    int64_t now = NowNs();
    GtRec r = GtFrom(0, node, now, dur, fi, rate);
    r.src = int16_t(node);
    r.reason = uint8_t(std::clamp(int(std::lround(10 * std::log10(txPowerW * 1000.0))) + 128, 0, 255));
    GtWrite(r);
    g_txCount[node]++;
    if (node == 0 && g_mon)
    {
        if (fi.hdr.IsAck())
        {
            g_mon->Ack(fi.hdr.GetAddr1(), now, now + dur, rate, true);
        }
        else
        {
            g_mon->Add(fi, now, now + dur, rate, true);
        }
    }
}

void
OnRxPayloadBegin(int node, WifiTxVector txv, Time psduDuration)
{
    g_rx[node].txv = txv;
    g_rx[node].payloadStart = NowNs();
    g_rx[node].valid = true;
}

void
OnRxEnd(int node, bool ok, Ptr<const Packet> pdu)
{
    FrameInfo fi = InfoFromPdu(pdu);
    int64_t now = NowNs();
    uint32_t rate = 0;
    int64_t dur = 0;
    if (g_rx[node].valid)
    {
        const WifiTxVector& txv = g_rx[node].txv;
        rate = RateKbps(txv);
        dur = WifiPhy::CalculateTxDuration(fi.size, txv, g_phy[node]->GetPhyBand()).GetNanoSeconds();
    }
    g_rx[node].valid = false;
    GtWrite(GtFrom(ok ? 1 : 2, node, now - dur, dur, fi, rate));
    (ok ? g_rxOk : g_rxErr)[node]++;
    if (node == 0 && g_mon)
    {
        if (!ok)
        {
            if (!fi.hdr.IsCtl())
            {
                g_mon->AddFailed(fi, now - dur, now, rate);
            }
        }
        else if (fi.hdr.IsAck())
        {
            g_mon->Ack(fi.hdr.GetAddr1(), now - dur, now, rate, false);
        }
        else
        {
            g_mon->Add(fi, now - dur, now, rate, false);
        }
    }
}

void
OnRxOk(int node, Ptr<const Packet> p, double, WifiMode, WifiPreamble)
{
    OnRxEnd(node, true, p);
}

void
OnRxError(int node, Ptr<const Packet> p, double)
{
    OnRxEnd(node, false, p);
}

void
OnPpduDrop(int node, Ptr<const WifiPpdu> ppdu, WifiPhyRxfailureReason reason)
{
    g_rxDrop[node]++;
    if (!g_gt)
    {
        return;
    }
    FrameInfo fi;
    Ptr<const WifiPsdu> psdu = ppdu->GetPsdu();
    if (psdu && psdu->GetNMpdus() > 0)
    {
        fi = InfoFromPsdu(psdu);
    }
    const WifiTxVector& txv = ppdu->GetTxVector();
    GtRec r = GtFrom(3, node, NowNs(), ppdu->GetTxDuration().GetNanoSeconds(), fi, RateKbps(txv));
    r.reason = uint8_t(reason);
    GtWrite(r);
}

void
OnState(int node, Time start, Time duration, WifiPhyState state)
{
    GtRec r{};
    r.t_ns = start.GetNanoSeconds();
    r.dur_ns = uint32_t(duration.GetNanoSeconds());
    r.node = uint16_t(node);
    r.src = r.dst = -1;
    r.ev = 4;
    r.reason = uint8_t(state);
    GtWrite(r);
}

// ---------------------------------------------------------------------------------------------
// lag of the real-time scheduler

FILE* g_lagF = nullptr;
std::vector<float> g_lagUs;
int64_t g_lagEveryNs = 10000000;
bool g_realtime = true;

void
LagTick()
{
    int64_t lag = WallNs() - (g_originNs + NowNs());
    float us = float(lag / 1000.0);
    g_lagUs.push_back(us); // written to lag.txt at the end (no file I/O in the simulation thread)
    if (g_stop.load())
    {
        Simulator::Stop();
        return;
    }
    Simulator::Schedule(NanoSeconds(g_lagEveryNs), &LagTick);
}

struct LagStats
{
    double max = 0, p99 = 0, p999 = 0, p50 = 0, mean = 0;
    size_t n = 0;
};

LagStats
ComputeLag(std::vector<float> v, size_t skip)
{
    LagStats s;
    if (v.size() <= skip)
    {
        return s;
    }
    v.erase(v.begin(), v.begin() + skip);
    s.n = v.size();
    double sum = 0;
    for (float x : v)
    {
        sum += x;
    }
    s.mean = sum / v.size();
    std::sort(v.begin(), v.end());
    auto q = [&](double f) { return double(v[std::min(v.size() - 1, size_t(f * v.size()))]); };
    s.p50 = q(0.5);
    s.p99 = q(0.99);
    s.p999 = q(0.999);
    s.max = v.back();
    return s;
}

void
Status()
{
    std::cerr << "STATUS t=" << Simulator::Now().GetSeconds() << "s gt=" << g_gtCount
              << " mon_batches=" << (g_mon ? g_mon->m_batches : 0) << std::endl;
    Simulator::Schedule(Seconds(10), &Status);
}

// ---------------------------------------------------------------------------------------------
// association

uint32_t g_assoc = 0;

void
OnAssoc(int node, Mac48Address bssid)
{
    g_assoc++;
    std::cerr << "ASSOC " << g_names[node] << " t=" << Simulator::Now().GetSeconds() << std::endl;
    if (g_assoc == g_M)
    {
        std::cerr << "READY t=" << Simulator::Now().GetSeconds() << std::endl;
    }
}

void
OnDeAssoc(int node, Mac48Address bssid)
{
    if (g_assoc > 0)
    {
        g_assoc--;
    }
    std::cerr << "DEASSOC " << g_names[node] << " t=" << Simulator::Now().GetSeconds() << std::endl;
}

void
OnSignal(int)
{
    g_stop.store(true);
}

void
ReportPhy(std::ostream& os)
{
    for (size_t n = 0; n < g_phy.size(); n++)
    {
        if (!g_phy[n])
        {
            continue;
        }
        Ptr<WifiMac> mac = g_wifiDev[n]->GetMac();
        Ptr<QosTxop> be = mac->GetQosTxop(AC_BE);
        Ptr<QosTxop> vo = mac->GetQosTxop(AC_VO);
        os << (n ? "," : "") << "\"" << g_names[n] << "\":{\"mac\":\"" << g_wifiMac[n]
           << "\",\"slot_us\":" << g_phy[n]->GetSlot().GetMicroSeconds()
           << ",\"sifs_us\":" << g_phy[n]->GetSifs().GetMicroSeconds()
           << ",\"be_aifsn\":" << int(be->Txop::GetAifsn()) << ",\"be_cwmin\":" << be->Txop::GetMinCw()
           << ",\"be_cwmax\":" << be->Txop::GetMaxCw() << ",\"vo_aifsn\":" << int(vo->Txop::GetAifsn())
           << ",\"vo_cwmin\":" << vo->Txop::GetMinCw() << ",\"vo_cwmax\":" << vo->Txop::GetMaxCw() << "}";
    }
}

void
PrintPhyReport()
{
    std::cerr << "PHYCONF {";
    ReportPhy(std::cout);
    std::cerr << "}" << std::endl;
}

double g_centerMhz = 2412; // operating channel of the BSS (set from the AP PHY)

Ptr<SpectrumValue>
JamPsd(double powerDbm)
{
    return WifiSpectrumValueHelper::CreateOfdmTxPowerSpectralDensity(MHz_u{g_centerMhz},
                                                                     MHz_u{20},
                                                                     Watt_u{std::pow(10.0, (powerDbm - 30) / 10)},
                                                                     MHz_u{2});
}

void
JamOn(Ptr<WaveformGenerator> wg, double periodUs, double duty, double powerDbm)
{
    wg->Stop();
    wg->SetTxPowerSpectralDensity(JamPsd(powerDbm));
    wg->SetPeriod(MicroSeconds(periodUs));
    wg->SetDutyCycle(duty);
    wg->Start();
}

void
JamOff(Ptr<WaveformGenerator> wg)
{
    wg->Stop();
}

void
OnJamTxStart(int node, double periodUs, Ptr<WaveformGenerator> wg, Ptr<const Packet>)
{
    GtRec r{};
    r.t_ns = NowNs();
    r.dur_ns = uint32_t(wg->GetPeriod().GetNanoSeconds() * wg->GetDutyCycle());
    r.node = uint16_t(node);
    r.src = int16_t(node);
    r.dst = -2;
    r.ev = 5;
    r.kind = 5;
    GtWrite(r);
}

// ---------------------------------------------------------------------------------------------
// Hosts of the simulation (fdt-sim): the vessel brokers with the regulator core (fdt_reg.c, the code of the
// Mosquitto plugin), the MQTT bridges of the GS over ns-3 TCP, the sources, and the link to the AS driver

// MQTT 3.1.1 packets of the bridges: PUBLISH (QoS 0, 1, 2), PUBACK, PUBREC, PUBREL, PUBCOMP
struct MqttMsg
{
    uint8_t type = 0; // upper nibble of byte 0
    uint8_t flags = 0;
    uint16_t pid = 0;
    std::string topic;
    std::string payload;
};

void
PutVarint(std::string& o, uint32_t v)
{
    do
    {
        uint8_t b = v % 128;
        v /= 128;
        if (v)
        {
            b |= 0x80;
        }
        o.push_back(char(b));
    } while (v);
}

std::string
EncPublish(const std::string& topic, int qos, uint16_t pid, const uint8_t* p, size_t n)
{
    std::string o;
    o.reserve(8 + topic.size() + n);
    o.push_back(char(0x30 | (qos << 1)));
    PutVarint(o, uint32_t(2 + topic.size() + (qos ? 2 : 0) + n));
    o.push_back(char(topic.size() >> 8));
    o.push_back(char(topic.size() & 0xff));
    o += topic;
    if (qos)
    {
        o.push_back(char(pid >> 8));
        o.push_back(char(pid & 0xff));
    }
    o.append(reinterpret_cast<const char*>(p), n);
    return o;
}

std::string
EncAck(uint8_t b0, uint16_t pid)
{
    std::string o;
    o.push_back(char(b0));
    o.push_back(char(2));
    o.push_back(char(pid >> 8));
    o.push_back(char(pid & 0xff));
    return o;
}

class MqttParser
{
  public:
    void Feed(const uint8_t* p, size_t n)
    {
        m_buf.append(reinterpret_cast<const char*>(p), n);
    }

    bool Next(MqttMsg& m)
    {
        size_t avail = m_buf.size() - m_off;
        if (avail < 2)
        {
            Compact();
            return false;
        }
        const uint8_t* b = reinterpret_cast<const uint8_t*>(m_buf.data()) + m_off;
        uint32_t rem = 0, mul = 1;
        size_t k = 1;
        for (;; k++)
        {
            if (k >= avail)
            {
                Compact();
                return false;
            }
            rem += (b[k] & 0x7f) * mul;
            mul *= 128;
            if (!(b[k] & 0x80))
            {
                break;
            }
            NS_ABORT_MSG_IF(k >= 4, "MQTT: bad remaining length");
        }
        size_t hdr = k + 1;
        if (avail < hdr + rem)
        {
            Compact();
            return false;
        }
        m = MqttMsg();
        m.type = b[0] >> 4;
        m.flags = b[0] & 0x0f;
        const uint8_t* v = b + hdr;
        if (m.type == 3)
        {
            uint16_t tl = uint16_t((v[0] << 8) | v[1]);
            m.topic.assign(reinterpret_cast<const char*>(v + 2), tl);
            size_t o = 2 + tl;
            int qos = (m.flags >> 1) & 3;
            if (qos)
            {
                m.pid = uint16_t((v[o] << 8) | v[o + 1]);
                o += 2;
            }
            m.payload.assign(reinterpret_cast<const char*>(v + o), rem - o);
        }
        else if (rem >= 2)
        {
            m.pid = uint16_t((v[0] << 8) | v[1]);
        }
        m_off += hdr + rem;
        return true;
    }

  private:
    void Compact()
    {
        if (m_off > 0)
        {
            m_buf.erase(0, m_off);
            m_off = 0;
        }
    }

    std::string m_buf;
    size_t m_off = 0;
};

// one TCP connection of a bridge: the broker writes each MQTT packet with one send(); the bytes that do
// not fit in the send buffer wait in the out queue of the broker (Mosquitto out_packet)
class Conn
{
  public:
    Ptr<Socket> sock;
    std::deque<std::string> txq;
    size_t txqOff = 0;
    uint64_t txqBytes = 0;
    MqttParser rx;
    std::function<void(const MqttMsg&)> onMsg;
    uint32_t nextTx = 0; // NextTxSequence of the socket (trace)
    bool up = false;

    void Attach(Ptr<Socket> s)
    {
        sock = s;
        up = true;
        s->SetRecvCallback(MakeCallback(&Conn::OnRecv, this));
        s->SetSendCallback(MakeCallback(&Conn::OnSpace, this));
        s->TraceConnectWithoutContext("NextTxSequence", MakeCallback(&Conn::OnNextTx, this));
    }

    void OnNextTx(SequenceNumber32, SequenceNumber32 v)
    {
        nextTx = v.GetValue();
    }

    void Write(std::string bytes)
    {
        txqBytes += bytes.size();
        txq.push_back(std::move(bytes));
        Flush();
    }

    void Flush()
    {
        while (up && !txq.empty())
        {
            uint32_t avail = sock->GetTxAvailable();
            if (avail == 0)
            {
                return;
            }
            std::string& f = txq.front();
            size_t n = std::min<size_t>(avail, f.size() - txqOff);
            Ptr<Packet> p = Create<Packet>(reinterpret_cast<const uint8_t*>(f.data()) + txqOff, uint32_t(n));
            if (sock->Send(p) <= 0)
            {
                return;
            }
            txqOff += n;
            txqBytes -= n;
            if (txqOff == f.size())
            {
                txq.pop_front();
                txqOff = 0;
            }
        }
    }

    void OnSpace(Ptr<Socket>, uint32_t)
    {
        Flush();
    }

    void OnRecv(Ptr<Socket> s)
    {
        Ptr<Packet> p;
        std::vector<uint8_t> b;
        while ((p = s->Recv()))
        {
            b.resize(p->GetSize());
            p->CopyData(b.data(), b.size());
            rx.Feed(b.data(), b.size());
            MqttMsg m;
            while (rx.Next(m))
            {
                onMsg(m);
            }
        }
    }

    // SIOCOUTQNSD, SIOCOUTQ and tcpi_bytes_acked of the socket (ns-3 ISN 0: the data starts at 1)
    void State(int64_t& nsd, int64_t& outq, uint64_t& acked)
    {
        Ptr<TcpSocketBase> t = DynamicCast<TcpSocketBase>(sock);
        Ptr<TcpTxBuffer> tb = t->GetTxBuffer();
        uint32_t head = tb->HeadSequence().GetValue(), tail = tb->TailSequence().GetValue();
        outq = int64_t(uint32_t(tail - head));
        uint32_t nx = nextTx;
        nsd = (int32_t(tail - nx) > 0) ? int64_t(uint32_t(tail - nx)) : 0;
        acked = head >= 1 ? uint64_t(head - 1) : 0;
    }
};

// ---------------------------------------------------------------------------------------------
// link to the AS driver (emu/sim/simrun.py): one JSON line per phase 2 on stdout, one reply on stdin

struct GsEvent
{
    int64_t atUs;
    std::string js;
};

std::vector<GsEvent> g_gsEv;          // events of the GS since the last phase 2 (demand, vlog, data)
int64_t g_d0Us = 100, g_d1Us = 100, g_d2Us = 300; // GS broker -> AS: QoS 0, QoS 1, QoS 2 (PUBREL + 2 RTT)
int64_t g_asDelayUs = 300;            // monitor batch -> AS (UDP, monitor bridge, MQTT)
int64_t g_pubUs = 100;                // AS -> GS broker -> bridge
FILE* g_proto = nullptr;              // protocol stream (stdout)
FILE* g_protoIn = nullptr;            // replies (stdin)

std::string
HexOf(const std::string& s, size_t n)
{
    static const char* H = "0123456789abcdef";
    std::string o;
    n = std::min(n, s.size());
    o.reserve(2 * n);
    for (size_t k = 0; k < n; k++)
    {
        uint8_t c = uint8_t(s[k]);
        o.push_back(H[c >> 4]);
        o.push_back(H[c & 15]);
    }
    return o;
}

std::string
JsonStr(const std::string& s)
{
    std::string o = "\"";
    for (char c : s)
    {
        if (c == '"' || c == '\\')
        {
            o.push_back('\\');
            o.push_back(c);
        }
        else if (uint8_t(c) < 0x20)
        {
            char b[8];
            snprintf(b, sizeof b, "\\u%04x", uint8_t(c));
            o += b;
        }
        else
        {
            o.push_back(c);
        }
    }
    o.push_back('"');
    return o;
}

void
GsEventAdd(int64_t atUs, std::string js)
{
    g_gsEv.push_back({atUs, std::move(js)});
}

// ---------------------------------------------------------------------------------------------
// vessel: broker with the regulator core, data listener 1884, control listener 1883, sources

struct SrcMsg
{
    int64_t atUs;
    int l;
    uint32_t j;
    bool output;
    std::vector<uint32_t> sizes; // message lengths (an output message holds its 8-byte header)
};

class Vessel;
std::vector<Vessel*> g_vessels; // index i (1..M)
std::vector<int64_t> g_tabMargin; // v(t) - arrival of the table of step t at its vessel (us)
struct fdt_dtp2 g_dtp2;
int g_regMode = 0;

class Vessel
{
  public:
    int i = 0;
    Ptr<Node> node;
    struct fdt_reg reg;
    Conn data, ctl;
    Ptr<Socket> lData, lCtl;
    std::vector<SrcMsg> src;
    size_t srcPos = 0;
    uint16_t pidData = 0, pidCtl = 0;
    int64_t deadlineUs = INT64_MAX; // instant that the regulator asked for
    EventId tickEv;
    std::string pfxTx;
    uint64_t pubErr = 0, rawCount = 0, levels = 0, goals = 0;

    static int IoPublish(void* ctx, const char* topic, int qos, const void* payload, size_t len)
    {
        Vessel* v = static_cast<Vessel*>(ctx);
        std::string t(topic), p(static_cast<const char*>(payload), len);
        // the broker queues the plugin message and writes it in the next iteration of its loop
        Simulator::ScheduleNow(&Vessel::Deliver, v, t, qos, p);
        return 0;
    }

    static void IoSock(void* ctx, struct fdt_sock* s)
    {
        Vessel* v = static_cast<Vessel*>(ctx);
        memset(s, 0, sizeof *s);
        if (!v->data.up)
        {
            return;
        }
        int64_t nsd, outq;
        uint64_t acked;
        v->data.State(nsd, outq, acked);
        s->n = 1;
        s->s[0].nsd = nsd;
        s->s[0].outq = outq;
        s->s[0].acked = acked;
        s->s[0].has_acked = 1;
    }

    void Deliver(std::string topic, int qos, std::string payload)
    {
        if (topic.compare(0, pfxTx.size(), pfxTx) == 0)
        {
            if (!data.up)
            {
                pubErr++;
                return;
            }
            int64_t nsd, outq;
            uint64_t acked;
            data.State(nsd, outq, acked);
            uint64_t written = acked + uint64_t(outq > 0 ? outq : 0);
            uint16_t pid = 0;
            if (qos)
            {
                pid = NextPid(pidData);
            }
            std::string pkt =
                EncPublish(topic, qos, pid, reinterpret_cast<const uint8_t*>(payload.data()), payload.size());
            uint32_t want = fdt2_publish_bytes(topic.size(), qos, payload.size());
            NS_ABORT_MSG_IF(pkt.size() != want, "PUBLISH size " << pkt.size() << " != " << want);
            fdt_reg_on_out(&reg, 0, qos, uint32_t(pkt.size()), uint32_t(payload.size()), written, 1);
            data.Write(std::move(pkt));
        }
        else
        {
            // control bridge: the GS subscribed demand at QoS 1 and the vessel log at QoS 0
            if (!ctl.up)
            {
                pubErr++;
                return;
            }
            int q = std::min(qos, 1);
            uint16_t pid = q ? NextPid(pidCtl) : 0;
            ctl.Write(EncPublish(topic, q, pid, reinterpret_cast<const uint8_t*>(payload.data()), payload.size()));
        }
    }

    static uint16_t NextPid(uint16_t& p)
    {
        p = uint16_t(p + 1);
        if (p == 0)
        {
            p = 1;
        }
        return p;
    }

    void OnData(const MqttMsg& m)
    {
        if (m.type == 5) // PUBREC -> PUBREL
        {
            data.Write(EncAck(0x62, m.pid));
        }
        Activity();
    }

    void OnCtl(const MqttMsg& m)
    {
        if (m.type == 3)
        {
            int qos = (m.flags >> 1) & 3;
            if (qos == 1)
            {
                ctl.Write(EncAck(0x40, m.pid));
            }
            if (m.topic == "boat/b" + std::to_string(i) + "/levels")
            {
                levels++;
                try
                {
                    mj::Value tb = mj::Parser(m.payload).Parse();
                    int64_t t = int64_t(tb["t"].Num(0));
                    g_tabMargin.push_back(fdt2_v_us(&g_dtp2, t) - HostUs(NowNs()));
                }
                catch (const std::exception&)
                {
                }
                fdt_reg_on_levels(&reg, m.payload.data(), m.payload.size(), HostUs(NowNs()));
            }
            else
            {
                goals++;
            }
        }
        Activity();
    }

    // a raw message of a source reaches the broker (MQTT over loopback)
    void Raw(int l, std::string payload)
    {
        rawCount++;
        fdt_reg_on_raw(&reg, l, payload.data(), payload.size(), HostUs(NowNs()));
        Activity();
    }

    void SrcNext()
    {
        if (srcPos >= src.size())
        {
            return;
        }
        int64_t now = HostUs(NowNs());
        while (srcPos < src.size() && src[srcPos].atUs <= now)
        {
            const SrcMsg& s = src[srcPos++];
            for (size_t k = 0; k < s.sizes.size(); k++)
            {
                uint32_t len = s.sizes[k];
                std::string b(len, '\0');
                uint8_t h[20];
                size_t hl = 0;
                if (s.output)
                {
                    fdt_put_u32(h, s.j);
                    fdt_put_u16(h + 4, uint16_t(k));
                    fdt_put_u16(h + 6, uint16_t(s.sizes.size()));
                    hl = 8;
                    if (len >= 16)
                    {
                        PutU64(h + 8, uint64_t(s.atUs));
                        hl = 16;
                    }
                }
                else
                {
                    fdt_put_u32(h, s.j);
                    PutU64(h + 4, uint64_t(s.atUs));
                    hl = 12;
                }
                memcpy(&b[0], h, std::min<size_t>(hl, len));
                Simulator::Schedule(MicroSeconds(50), &Vessel::Raw, this, s.l, b);
            }
        }
        if (srcPos < src.size())
        {
            Simulator::Schedule(NanoSeconds((src[srcPos].atUs - now) * 1000), &Vessel::SrcNext, this);
        }
    }

    static void PutU64(uint8_t* p, uint64_t v)
    {
        for (int k = 7; k >= 0; k--)
        {
            p[7 - k] = uint8_t(v >> (8 * k));
        }
    }

    // MOSQ_EVT_TICK with patch 0001: the tick runs when its deadline has passed, at the first loop
    // iteration after it; the loop sleeps at least 1 ms and whole milliseconds
    void Tick()
    {
        int64_t now = HostUs(NowNs());
        int64_t next = fdt_reg_tick(&reg, now);
        deadlineUs = next;
        int64_t dly = next - HostUs(NowNs());
        int64_t ms = dly <= 1000 ? 1 : (dly + 999) / 1000;
        ms = std::min<int64_t>(ms, 1000);
        Simulator::Cancel(tickEv);
        tickEv = Simulator::Schedule(MilliSeconds(ms), &Vessel::Tick, this);
    }

    // an event of the broker loop (network, plugin message): the tick runs if its deadline has passed
    void Activity()
    {
        if (HostUs(NowNs()) >= deadlineUs)
        {
            Simulator::Cancel(tickEv);
            tickEv = Simulator::ScheduleNow(&Vessel::Tick, this);
        }
    }

    void AcceptData(Ptr<Socket> s, const Address& a)
    {
        Accept(&data, s, a);
    }

    void AcceptCtl(Ptr<Socket> s, const Address& a)
    {
        Accept(&ctl, s, a);
    }

    void Accept(Conn* c, Ptr<Socket> s, const Address&)
    {
        if (c == &ctl)
        {
            s->SetPriority(6);
            c->onMsg = [this](const MqttMsg& m) { OnCtl(m); };
        }
        else
        {
            c->onMsg = [this](const MqttMsg& m) { OnData(m); };
            fdt_reg_sock_reset(&reg, 0);
        }
        c->Attach(s);
    }
};

// ---------------------------------------------------------------------------------------------
// GS: the bridges b<i>-ctl and b<i>-data of the GS broker; the AS through the driver

struct GsLink
{
    int i = 0;
    Conn data, ctl;
    uint16_t pidCtl = 0;
    std::map<uint16_t, std::string> q2; // QoS 2 messages that wait for PUBREL: topic + payload
    std::map<uint16_t, std::string> q2t;
};

std::vector<GsLink*> g_gs; // index i
uint64_t g_gsDataMsgs = 0, g_gsDemands = 0, g_gsVlogs = 0;

void
GsAccept(int i, const std::string& topic, const std::string& payload, int64_t atUs)
{
    // acceptor input: the topic and the first 40 bytes of the payload (hex) with its length
    g_gsDataMsgs++;
    std::string js = "[\"a\"," + std::to_string(atUs) + "," + JsonStr(topic) + "," + std::to_string(payload.size()) +
                     ",\"" + HexOf(payload, 40) + "\"]";
    GsEventAdd(atUs, std::move(js));
}

void
GsOnData(GsLink* g, const MqttMsg& m)
{
    int64_t now = HostUs(NowNs());
    if (m.type == 3)
    {
        int qos = (m.flags >> 1) & 3;
        if (qos == 2)
        {
            g->q2t[m.pid] = m.topic;
            g->q2[m.pid] = m.payload;
            g->data.Write(EncAck(0x50, m.pid)); // PUBREC
        }
        else
        {
            if (qos == 1)
            {
                g->data.Write(EncAck(0x40, m.pid));
            }
            GsAccept(g->i, m.topic, m.payload, now + g_d0Us);
        }
    }
    else if (m.type == 6) // PUBREL: Mosquitto releases the message to the subscribers, PUBCOMP
    {
        g->data.Write(EncAck(0x70, m.pid));
        auto it = g->q2.find(m.pid);
        if (it != g->q2.end())
        {
            GsAccept(g->i, g->q2t[m.pid], it->second, now + g_d2Us);
            g->q2.erase(it);
            g->q2t.erase(m.pid);
        }
    }
}

void
GsOnCtl(GsLink* g, const MqttMsg& m)
{
    int64_t now = HostUs(NowNs());
    if (m.type != 3)
    {
        return;
    }
    int qos = (m.flags >> 1) & 3;
    if (qos == 1)
    {
        g->ctl.Write(EncAck(0x40, m.pid));
    }
    std::string pre = "boat/b" + std::to_string(g->i) + "/";
    if (m.topic == pre + "demand")
    {
        g_gsDemands++;
        GsEventAdd(now + g_d1Us, "[\"d\"," + std::to_string(now + g_d1Us) + "," + std::to_string(g->i) + "," +
                                     JsonStr(m.payload) + "]");
    }
    else if (m.topic == pre + "regulator")
    {
        g_gsVlogs++;
        GsEventAdd(now + g_d0Us, "[\"v\"," + std::to_string(now + g_d0Us) + "," + std::to_string(g->i) + "," +
                                     JsonStr(m.payload) + "]");
    }
}

void
GsPublishCtl(int i, std::string topic, std::string payload)
{
    GsLink* g = g_gs[i];
    if (!g->ctl.up)
    {
        return;
    }
    uint16_t pid = Vessel::NextPid(g->pidCtl);
    g->ctl.Write(EncPublish(topic, 1, pid, reinterpret_cast<const uint8_t*>(payload.data()), payload.size()));
}

Ptr<Node> g_gsNode;

void
GsConnected(int i, int which, Ptr<Socket> s)
{
    GsLink* g = g_gs[i];
    if (which == 1)
    {
        g->ctl.onMsg = [g](const MqttMsg& m) { GsOnCtl(g, m); };
        g->ctl.Attach(s);
    }
    else
    {
        g->data.onMsg = [g](const MqttMsg& m) { GsOnData(g, m); };
        g->data.Attach(s);
    }
}

void
GsConnectFailed(int i, int which, Ptr<Socket>)
{
    NS_FATAL_ERROR("bridge " << which << " of vessel " << i << " did not connect");
}

void
GsConnect(int i)
{
    Ipv4Address va((10u << 24) | (10u << 16) | uint32_t(10 + i));
    for (int which : {1, 2})
    {
        Ptr<Socket> c = Socket::CreateSocket(g_gsNode, TcpSocketFactory::GetTypeId());
        if (which == 1)
        {
            c->SetPriority(6);
        }
        c->SetConnectCallback(MakeBoundCallback(&GsConnected, i, which), MakeBoundCallback(&GsConnectFailed, i, which));
        c->Connect(InetSocketAddress(va, which == 1 ? 1883 : 1884));
    }
}

// phase 2 of step t: the events of the GS and the monitor batches go to the driver, which runs
// ASCore.phase2(t) and returns the tables and the compute time
uint64_t g_p2 = 0;

void
Phase2(int64_t t, int64_t last)
{
    int64_t now = HostUs(NowNs());
    std::stable_sort(g_gsEv.begin(), g_gsEv.end(), [](const GsEvent& a, const GsEvent& b) { return a.atUs < b.atUs; });
    std::string out;
    out.reserve(1 << 16);
    out += "{\"k\":\"p2\",\"t\":" + std::to_string(t) + ",\"now_us\":" + std::to_string(now) + ",\"mon\":[";
    for (size_t k = 0; k < g_monOut.size(); k++)
    {
        out += (k ? "," : "") + g_monOut[k];
    }
    g_monOut.clear();
    out += "],\"ev\":[";
    size_t kept = 0, n = 0;
    std::vector<GsEvent> rest;
    for (auto& e : g_gsEv)
    {
        if (e.atUs <= now)
        {
            out += (n++ ? "," : "") + e.js;
        }
        else
        {
            rest.push_back(e);
            kept++;
        }
    }
    g_gsEv.swap(rest);
    out += "]}\n";
    fwrite(out.data(), 1, out.size(), g_proto);
    fflush(g_proto);
    // reply: {"c_us": compute, "tabs": [[i, "<table json>"], ...]}
    static std::string line;
    line.clear();
    char buf[1 << 16];
    while (fgets(buf, sizeof buf, g_protoIn))
    {
        line += buf;
        if (!line.empty() && line.back() == '\n')
        {
            break;
        }
    }
    NS_ABORT_MSG_IF(line.empty(), "driver closed the protocol stream");
    mj::Value r = mj::Parser(line).Parse();
    int64_t cus = int64_t(r["c_us"].Num(0));
    const mj::Value& tabs = r["tabs"];
    for (size_t k = 0; k < tabs.size(); k++)
    {
        int i = int(tabs[k][0].Num());
        std::string tb = tabs[k][1].Str();
        Simulator::Schedule(MicroSeconds(cus + g_pubUs), &GsPublishCtl, i, "boat/b" + std::to_string(i) + "/levels", tb);
    }
    g_p2++;
    if (t < last)
    {
        Simulator::Schedule(NanoSeconds(g_dtp2.Ts_us * 1000), &Phase2, t + 1, last);
    }
}

} // namespace

int
main(int argc, char* argv[])
{
    std::string scenarioPath, outdir = ".", dtpPath, schedPath, regMode = "fleetdt", flush;
    int64_t S0 = 0, steps = 0, t0Us = 0, endUs = 0, p2OffUs = 0;
    double leadS = 6.0;
    bool gt = true;
    uint32_t delAckCount = 2;
    double delAckMs = 40;

    CommandLine cmd(__FILE__);
    cmd.AddValue("delAckCount", "TCP: segments per delayed ACK (ns-3 DelAckCount)", delAckCount);
    cmd.AddValue("delAckMs", "TCP: delayed ACK timeout (ms)", delAckMs);
    cmd.AddValue("scenario", "scenario JSON (emu/scenarios)", scenarioPath);
    cmd.AddValue("outdir", "output directory (gt.bin, summary.json)", outdir);
    cmd.AddValue("dtp", "DTP v2 of the run", dtpPath);
    cmd.AddValue("schedule", "sources and goals (emu/sim/simrun.py)", schedPath);
    cmd.AddValue("regMode", "mode of the vessel regulator", regMode);
    cmd.AddValue("S0", "first step of the AS", S0);
    cmd.AddValue("steps", "phases 2 of the AS: S0 .. S0 + steps", steps);
    cmd.AddValue("t0Us", "host time (us) of start(S0) = scenario time 0", t0Us);
    cmd.AddValue("leadS", "simulated seconds before start(S0)", leadS);
    cmd.AddValue("endUs", "host time (us) of the end of the run", endUs);
    cmd.AddValue("p2OffUs", "phase 2 of the AS: start(t) + this offset (us)", p2OffUs);
    cmd.AddValue("pubUs", "AS publish -> bridge (us)", g_pubUs);
    cmd.AddValue("d0Us", "GS broker -> AS, QoS 0 (us)", g_d0Us);
    cmd.AddValue("d1Us", "GS broker -> AS, QoS 1 (us)", g_d1Us);
    cmd.AddValue("d2Us", "GS broker -> AS after PUBREL, QoS 2 (us)", g_d2Us);
    cmd.AddValue("gt", "write the ground truth gt.bin", gt);
    cmd.AddValue("gtStates", "PHY state records in gt.bin: ap, all or none", g_gtStates);
    cmd.AddValue("flush", "TS_US:OFF_US[,OFF_US...]: extra monitor batches at t0 + k TS + OFF", flush);
    cmd.Parse(argc, argv);
    NS_ABORT_MSG_IF(dtpPath.empty() || schedPath.empty() || !t0Us || !endUs || !steps, "missing arguments");

    {
        char err[256];
        NS_ABORT_MSG_IF(fdt2_dtp_load(dtpPath.c_str(), &g_dtp2, err, sizeof err), "DTP: " << err);
        g_regMode = fdt_mode_parse(regMode.c_str());
        NS_ABORT_MSG_IF(g_regMode < 0, "unknown regulator mode " << regMode);
        NS_ABORT_MSG_IF(g_regMode == FDT_MODE_B2, "mode b2 (three data bridges) is not simulated");
    }
    // the protocol stream to the driver is stdout; everything else goes to stderr
    g_proto = fdopen(dup(1), "w");
    g_protoIn = stdin;
    if (dup2(2, 1) < 0)
    {
        NS_FATAL_ERROR("dup2");
    }

    mj::Value sc = mj::ParseFile(scenarioPath);
    g_M = uint32_t(sc["M"].Num(2));
    NS_ABORT_MSG_IF(int(g_M) != g_dtp2.M, "scenario M != DTP M");
    g_realtime = false;
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(false));
    RngSeedManager::SetSeed(uint32_t(sc["seed"].Num(1)));
    RngSeedManager::SetRun(uint32_t(sc["run"].Num(1)));
    const mj::Value& q = sc["mac_queue"];
    Config::SetDefault("ns3::WifiMacQueue::MaxSize",
                       QueueSizeValue(QueueSize(std::to_string(int(q["packets"].Num(100))) + "p")));
    Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(MilliSeconds(q["max_delay_ms"].Num(500))));
    // TCP of the hosts (Linux defaults where ns-3 differs): MSS of a 1500-byte MTU with timestamps, CUBIC,
    // SACK, no Nagle, minimum RTO 200 ms, delayed ACK of 40 ms or every second segment, large buffers
    Config::SetDefault("ns3::TcpL4Protocol::SocketType", TypeIdValue(TcpCubic::GetTypeId()));
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(1448));
    Config::SetDefault("ns3::TcpSocket::SndBufSize", UintegerValue(4 << 20));
    Config::SetDefault("ns3::TcpSocket::RcvBufSize", UintegerValue(4 << 20));
    Config::SetDefault("ns3::TcpSocket::DelAckTimeout", TimeValue(MicroSeconds(int64_t(delAckMs * 1e3))));
    Config::SetDefault("ns3::TcpSocket::DelAckCount", UintegerValue(delAckCount));
    Config::SetDefault("ns3::TcpSocket::TcpNoDelay", BooleanValue(true));
    Config::SetDefault("ns3::TcpSocket::InitialCwnd", UintegerValue(10));
    Config::SetDefault("ns3::TcpSocketBase::MinRto", TimeValue(MilliSeconds(200)));
    Config::SetDefault("ns3::TcpSocketBase::Sack", BooleanValue(true));
    Config::SetDefault("ns3::TcpSocketBase::Timestamp", BooleanValue(true));
    // ---- nodes: 0 GS (AP), 1..M vessels, then ap2, sta2, jam
    const mj::Value& ext = sc["external_bss"];
    const mj::Value& jam = sc["jammer"];
    bool hasExt = ext.Has("schedule") && ext["schedule"].size() > 0;
    bool hasJam = jam.Has("schedule") && jam["schedule"].size() > 0;
    NodeContainer all;
    all.Create(g_M + 1 + (hasExt ? 2 : 0) + (hasJam ? 1 : 0));
    g_names.push_back("gs");
    for (uint32_t i = 1; i <= g_M; i++)
    {
        g_names.push_back("v" + std::to_string(i));
    }
    if (hasExt)
    {
        g_nAp2 = int(g_names.size());
        g_names.push_back("ap2");
        g_nSta2 = int(g_names.size());
        g_names.push_back("sta2");
    }
    if (hasJam)
    {
        g_nJam = int(g_names.size());
        g_names.push_back("jam");
    }
    auto idx = [&](const std::string& name) {
        for (size_t k = 0; k < g_names.size(); k++)
        {
            if (g_names[k] == name)
            {
                return int(k);
            }
        }
        NS_FATAL_ERROR("unknown node name " << name);
        return -1;
    };

    // ---- mobility and channel
    // PHY model: "spectrum" (SpectrumWifiPhy + MultiModelSpectrumChannel, needed by the jammer) or "yans"
    // (YansWifiPhy + YansWifiChannel: same 802.11 MAC/PHY abstraction, cheaper per receiver)
    std::string phyModel = sc["phy"].Str("spectrum");
    NS_ABORT_MSG_IF(phyModel != "spectrum" && phyModel != "yans", "phy must be spectrum or yans");
    NS_ABORT_MSG_IF(phyModel == "yans" && hasJam, "the jammer needs phy = spectrum");
    Ptr<PropagationLossModel> lossModel;
    const mj::Value& ch = sc["channel"];
    std::string model = ch["model"].Str("matrix");
    const mj::Value& pos = ch["positions"];
    for (size_t n = 0; n < g_names.size(); n++)
    {
        const mj::Value& wp = pos[g_names[n]];
        if (wp.size() > 1 && wp[0].type == mj::Value::ARR)
        {
            // the node waits at the first waypoint until its time (shifted by the run start), then moves
            Ptr<WaypointMobilityModel> m = CreateObject<WaypointMobilityModel>();
            m->AddWaypoint(Waypoint(Seconds(0), Vector(wp[0][1].Num(), wp[0][2].Num(), wp[0][3].Num())));
            std::vector<std::array<double, 4>> pts;
            for (size_t k = 0; k < wp.size(); k++)
            {
                pts.push_back({wp[k][0].Num(), wp[k][1].Num(), wp[k][2].Num(), wp[k][3].Num()});
            }
            g_deferred.push_back([m, pts](double shift) {
                double last = 0;
                for (auto& p : pts)
                {
                    double at = shift + p[0];
                    if (at <= last)
                    {
                        continue;
                    }
                    m->AddWaypoint(Waypoint(Seconds(at), Vector(p[1], p[2], p[3])));
                    last = at;
                }
            });
            all.Get(n)->AggregateObject(m);
        }
        else
        {
            Ptr<ConstantPositionMobilityModel> m = CreateObject<ConstantPositionMobilityModel>();
            const mj::Value& p = (wp.size() && wp[0].type == mj::Value::ARR) ? wp[0] : wp;
            if (p.size() >= 4)
            {
                m->SetPosition(Vector(p[1].Num(), p[2].Num(), p[3].Num()));
            }
            else if (p.size() == 3)
            {
                m->SetPosition(Vector(p[0].Num(), p[1].Num(), p[2].Num()));
            }
            else
            {
                m->SetPosition(Vector(10.0 * n, 0, 2));
            }
            all.Get(n)->AggregateObject(m);
        }
    }
    if (model == "matrix")
    {
        Ptr<MatrixPropagationLossModel> loss = CreateObject<MatrixPropagationLossModel>();
        loss->SetDefaultLoss(ch["default_loss_db"].Num(70));
        const mj::Value& L = ch["loss"];
        for (size_t k = 0; k < L.size(); k++)
        {
            int a = idx(L[k][0].Str()), b = idx(L[k][1].Str());
            loss->SetLoss(all.Get(a)->GetObject<MobilityModel>(),
                          all.Get(b)->GetObject<MobilityModel>(),
                          L[k][2].Num(),
                          true);
        }
        lossModel = loss;
    }
    else if (model == "tworay")
    {
        Ptr<TwoRayGroundPropagationLossModel> tr = CreateObject<TwoRayGroundPropagationLossModel>();
        tr->SetFrequency(ch["frequency_hz"].Num(2.412e9));
        tr->SetHeightAboveZ(0);
        tr->SetMinDistance(ch["min_distance_m"].Num(1));
        tr->SetSystemLoss(ch["system_loss"].Num(1));
        const mj::Value& nk = ch["nakagami"];
        if (nk.type == mj::Value::OBJ)
        {
            Ptr<NakagamiPropagationLossModel> nak = CreateObject<NakagamiPropagationLossModel>();
            nak->SetAttribute("m0", DoubleValue(nk["m0"].Num(1.5)));
            nak->SetAttribute("m1", DoubleValue(nk["m1"].Num(0.75)));
            nak->SetAttribute("m2", DoubleValue(nk["m2"].Num(0.75)));
            nak->SetAttribute("Distance1", DoubleValue(nk["d1"].Num(80)));
            nak->SetAttribute("Distance2", DoubleValue(nk["d2"].Num(200)));
            nak->AssignStreams(int64_t(sc["seed"].Num(1)) * 100);
            tr->SetNext(nak);
        }
        lossModel = tr;
    }
    else
    {
        NS_FATAL_ERROR("channel.model must be matrix or tworay");
    }

    // ---- Wi-Fi
    const mj::Value& rate = sc["rate"];
    WifiHelper wifi;
    // "11g" (ERP-OFDM, default) or "11a": OFDM without the DSSS rates of 11g, whose exchange airtime equals
    // that of ERP-OFDM (SIFS 16 us = SIFS 10 us + signal extension 6 us); rate-adaptive managers (Minstrel)
    // need it, because they also try the DSSS rates, which the real radios do not use (PR7) and which
    // SpectrumWifiPhy cannot send in a 20 MHz channel
    std::string standard = sc["standard"].Str("11g");
    NS_ABORT_MSG_IF(standard != "11g" && standard != "11a", "standard must be 11g or 11a");
    WifiStandard wstd = standard == "11a" ? WIFI_STANDARD_80211a : WIFI_STANDARD_80211g;
    std::string mp = standard == "11a" ? "OfdmRate" : "ErpOfdmRate"; // mode name prefix
    wifi.SetStandard(wstd);
    std::string nonUnicast = rate["non_unicast"].Str(mp + "6Mbps");
    if (rate["manager"].Str("constant") == "minstrel")
    {
        wifi.SetRemoteStationManager("ns3::MinstrelWifiManager", "NonUnicastMode", StringValue(nonUnicast));
    }
    else
    {
        wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                     "DataMode",
                                     StringValue(rate["data"].Str(mp + "24Mbps")),
                                     "ControlMode",
                                     StringValue(rate["control"].Str(mp + "24Mbps")),
                                     "NonUnicastMode",
                                     StringValue(nonUnicast));
    }
    Ptr<MultiModelSpectrumChannel> channel;
    SpectrumWifiPhyHelper sphy;
    YansWifiPhyHelper yphy;
    if (phyModel == "spectrum")
    {
        channel = CreateObject<MultiModelSpectrumChannel>();
        channel->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
        channel->AddPropagationLossModel(lossModel);
        sphy.SetChannel(channel);
    }
    else
    {
        Ptr<YansWifiChannel> ych = CreateObject<YansWifiChannel>();
        ych->SetPropagationDelayModel(CreateObject<ConstantSpeedPropagationDelayModel>());
        ych->SetPropagationLossModel(lossModel);
        yphy.SetChannel(ych);
    }
    WifiPhyHelper& phy = (phyModel == "spectrum") ? static_cast<WifiPhyHelper&>(sphy)
                                                  : static_cast<WifiPhyHelper&>(yphy);
    double txp = sc["tx_power_dbm"].Num(20);
    phy.Set("TxPowerStart", DoubleValue(txp));
    phy.Set("TxPowerEnd", DoubleValue(txp));
    if (sc.Has("cca_ed_dbm"))
    {
        phy.Set("CcaEdThreshold", DoubleValue(sc["cca_ed_dbm"].Num()));
    }
    const mj::Value& e = sc["edca"];
    auto ival = [&](const char* k, int def) { return int(e[k].Num(def)); };
    WifiMacHelper mac;
    Ssid ssid("fleet");
    NodeContainer vessels;
    for (uint32_t i = 1; i <= g_M; i++)
    {
        vessels.Add(all.Get(i));
    }
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "QosSupported", BooleanValue(true));
    NetDeviceContainer staDevs = wifi.Install(phy, mac, vessels);
    mac.SetType("ns3::ApWifiMac",
                "Ssid",
                SsidValue(ssid),
                "QosSupported",
                BooleanValue(true),
                "BeaconInterval",
                TimeValue(MicroSeconds(1024 * int(sc["beacon_tu"].Num(100)))),
                "AifsnsForSta",
                StringValue("BE " + std::to_string(ival("aifsn_sta", 3))),
                "CwMinsForSta",
                StringValue("BE " + std::to_string(ival("cwmin_sta", 15))),
                "CwMaxsForSta",
                StringValue("BE " + std::to_string(ival("cwmax_sta", 1023))));
    NetDeviceContainer apDevs = wifi.Install(phy, mac, all.Get(0));
    {
        Ptr<QosTxop> be = DynamicCast<WifiNetDevice>(apDevs.Get(0))->GetMac()->GetQosTxop(AC_BE);
        be->SetAifsn(uint8_t(ival("aifsn_ap", 1)));
        be->SetMinCw(uint32_t(ival("cwmin_ap", 7)));
        be->SetMaxCw(uint32_t(ival("cwmax_ap", 1023)));
    }
    NetDeviceContainer extDevs;
    if (hasExt)
    {
        Ssid ssid2("external");
        WifiHelper w2;
        w2.SetStandard(wstd);
        w2.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                   "DataMode",
                                   StringValue(ext["data"].Str(mp + "24Mbps")),
                                   "ControlMode",
                                   StringValue(mp + "24Mbps"),
                                   "NonUnicastMode",
                                   StringValue(mp + "6Mbps"));
        mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid2), "QosSupported", BooleanValue(true));
        extDevs.Add(w2.Install(phy, mac, all.Get(g_nAp2)));
        mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid2), "QosSupported", BooleanValue(true));
        extDevs.Add(w2.Install(phy, mac, all.Get(g_nSta2)));
    }
    int64_t stream = 1000;
    stream += wifi.AssignStreams(apDevs, stream);
    stream += wifi.AssignStreams(staDevs, stream);
    if (hasExt)
    {
        stream += wifi.AssignStreams(extDevs, stream);
    }

    // BSS basic rate set: the ERP-OFDM rates of the DTP first, so that management frames and beacons use
    // the lowest of them (ns-3 sends management frames at basic mode 0, by default DSSS 1 Mb/s)
    {
        std::vector<std::string> basic;
        const mj::Value& bm = sc["basic_modes"];
        for (size_t k = 0; k < bm.size(); k++)
        {
            basic.push_back(bm[k].Str());
        }
        if (bm.type != mj::Value::ARR)
        {
            basic = {mp + "6Mbps", mp + "12Mbps", mp + "24Mbps"};
        }
        NetDeviceContainer allDevs(apDevs, staDevs);
        allDevs.Add(extDevs);
        for (uint32_t k = 0; k < allDevs.GetN(); k++)
        {
            auto rsm = DynamicCast<WifiNetDevice>(allDevs.Get(k))->GetRemoteStationManager();
            for (auto& m : basic)
            {
                rsm->AddBasicMode(WifiMode(m));
            }
        }
    }

    size_t nWifi = g_M + 1 + (hasExt ? 2 : 0);
    g_centerMhz = DynamicCast<WifiNetDevice>(apDevs.Get(0))->GetPhy()->GetFrequency();
    g_wifiDev.resize(nWifi);
    g_phy.resize(nWifi);
    g_wifiMac.resize(nWifi);
    g_rx.resize(nWifi);
    g_txCount.assign(nWifi, 0);
    g_rxOk.assign(nWifi, 0);
    g_rxErr.assign(nWifi, 0);
    g_rxDrop.assign(nWifi, 0);
    auto reg = [&](int n, Ptr<NetDevice> d) {
        g_wifiDev[n] = DynamicCast<WifiNetDevice>(d);
        g_phy[n] = g_wifiDev[n]->GetPhy();
        g_wifiMac[n] = Mac48Address::ConvertFrom(d->GetAddress());
        g_wifiMacToNode[g_wifiMac[n]] = n;
    };
    reg(0, apDevs.Get(0));
    for (uint32_t i = 1; i <= g_M; i++)
    {
        reg(int(i), staDevs.Get(i - 1));
    }
    if (hasExt)
    {
        reg(g_nAp2, extDevs.Get(0));
        reg(g_nSta2, extDevs.Get(1));
    }

    // ---- outputs
    std::string mkdir = "mkdir -p '" + outdir + "'";
    if (system(mkdir.c_str()) != 0)
    {
        NS_FATAL_ERROR("cannot create " << outdir);
    }
    if (gt)
    {
        g_gt = fopen((outdir + "/gt.bin").c_str(), "wb");
        setvbuf(g_gt, nullptr, _IOFBF, 1 << 20);
    }
    // ---- traces
    for (size_t n = 0; n < nWifi; n++)
    {
        int node = int(n);
        Ptr<WifiPhy> p = g_phy[n];
        p->TraceConnectWithoutContext("PhyTxPsduBegin", MakeBoundCallback(&OnTxPsdu, node));
        p->TraceConnectWithoutContext("PhyRxPayloadBegin", MakeBoundCallback(&OnRxPayloadBegin, node));
        p->TraceConnectWithoutContext("PhyRxPpduDrop", MakeBoundCallback(&OnPpduDrop, node));
        p->GetState()->TraceConnectWithoutContext("RxOk", MakeBoundCallback(&OnRxOk, node));
        p->GetState()->TraceConnectWithoutContext("RxError", MakeBoundCallback(&OnRxError, node));
        if (g_gt && (g_gtStates == "all" || (g_gtStates == "ap" && n == 0)))
        {
            p->GetState()->TraceConnectWithoutContext("State", MakeBoundCallback(&OnState, node));
        }
        Ptr<StaWifiMac> sm = DynamicCast<StaWifiMac>(g_wifiDev[n]->GetMac());
        if (sm && n >= 1 && n <= g_M)
        {
            sm->TraceConnectWithoutContext("Assoc", MakeBoundCallback(&OnAssoc, node));
            sm->TraceConnectWithoutContext("DeAssoc", MakeBoundCallback(&OnDeAssoc, node));
        }
    }

    // ---- monitor
    Monitor mon;
    g_mon = &mon;
    mon.m_apMac = g_wifiMac[0];
    mon.m_sifs = g_phy[0]->GetSifs();
    mon.m_periodNs = int64_t(sc["monitor_period_ms"].Num(5) * 1e6);
    auto monListener = std::make_shared<MonListener>();
    g_phy[0]->RegisterListener(monListener);

    // ---- hosts: IPv4 on the Wi-Fi devices of the GS and the vessels (no queue disc: the IP packets reach
    // the Wi-Fi MAC queue directly, as the packets of the VMs through the relay of fdt-emu), static ARP
    {
        InternetStackHelper inet;
        NodeContainer fleet;
        for (uint32_t h = 0; h <= g_M; h++)
        {
            fleet.Add(all.Get(h));
        }
        inet.Install(fleet);
        NetDeviceContainer fdevs;
        for (uint32_t h = 0; h <= g_M; h++)
        {
            Ptr<Node> n = all.Get(h);
            Ptr<WifiNetDevice> d = g_wifiDev[h];
            d->SetMtu(1500);
            Ptr<Ipv4> ip = n->GetObject<Ipv4>();
            int32_t k = ip->AddInterface(d);
            uint32_t a = (10u << 24) | (10u << 16) | (h == 0 ? 1u : 10u + h);
            ip->AddAddress(k, Ipv4InterfaceAddress(Ipv4Address(a), Ipv4Mask("255.255.255.0")));
            ip->SetMetric(k, 1);
            ip->SetUp(k);
            fdevs.Add(d);
        }
        NeighborCacheHelper nch;
        nch.PopulateNeighborCache(fdevs);
    }
    // ---- schedule of the sources and of the goals
    g_vessels.assign(g_M + 1, nullptr);
    g_gs.assign(g_M + 1, nullptr);
    std::vector<std::vector<std::pair<int64_t, uint32_t>>> goalsOf(g_M + 1);
    for (uint32_t i = 1; i <= g_M; i++)
    {
        g_vessels[i] = new Vessel();
        g_vessels[i]->i = int(i);
        g_vessels[i]->node = all.Get(i);
        g_vessels[i]->pfxTx = "boat/b" + std::to_string(i) + "/tx/";
        g_gs[i] = new GsLink();
        g_gs[i]->i = int(i);
    }
    {
        // lines: "S at_us vessel l j kind size1 size2 ..." (kind 0 stream, 1 output); "G at_us vessel bytes"
        std::ifstream f(schedPath);
        NS_ABORT_MSG_IF(!f, "cannot open " << schedPath);
        std::string line;
        uint64_t ns = 0, ng = 0;
        while (std::getline(f, line))
        {
            if (line.empty())
            {
                continue;
            }
            std::istringstream is(line);
            std::string tag;
            is >> tag;
            if (tag == "S")
            {
                SrcMsg m;
                int v, kind;
                is >> m.atUs >> v >> m.l >> m.j >> kind;
                m.output = kind == 1;
                uint32_t s;
                while (is >> s)
                {
                    m.sizes.push_back(s);
                }
                NS_ABORT_MSG_IF(v < 1 || v > int(g_M) || m.sizes.empty(), "schedule: bad line " << line);
                g_vessels[v]->src.push_back(std::move(m));
                ns++;
            }
            else if (tag == "G")
            {
                int64_t at;
                int v;
                uint32_t b;
                is >> at >> v >> b;
                goalsOf[v].emplace_back(at, b);
                ng++;
            }
        }
        std::cerr << "SCHEDULE sources=" << ns << " goals=" << ng << std::endl;
    }
    g_gsNode = all.Get(0);
    g_originNs = t0Us * 1000 - int64_t(leadS * 1e9);
    double shift = leadS;
    int64_t startNs = 0;
    for (uint32_t i = 1; i <= g_M; i++)
    {
        Vessel* v = g_vessels[i];
        std::stable_sort(v->src.begin(), v->src.end(), [](const SrcMsg& a, const SrcMsg& b) { return a.atUs < b.atUs; });
        struct fdt_io io = {v, &Vessel::IoPublish, &Vessel::IoSock};
        NS_ABORT_MSG_IF(fdt_reg_init(&v->reg, &g_dtp2, int(i), g_regMode, 4096, HostUs(startNs), &io), "fdt_reg_init");
        fdt_reg_set_p1_offset_guard(&v->reg, -1, -1);
        Ptr<Socket> ld = Socket::CreateSocket(v->node, TcpSocketFactory::GetTypeId());
        ld->Bind(InetSocketAddress(Ipv4Address::GetAny(), 1884));
        ld->Listen();
        ld->SetAcceptCallback(MakeNullCallback<bool, Ptr<Socket>, const Address&>(),
                              MakeCallback(&Vessel::AcceptData, v));
        Ptr<Socket> lc = Socket::CreateSocket(v->node, TcpSocketFactory::GetTypeId());
        lc->Bind(InetSocketAddress(Ipv4Address::GetAny(), 1883));
        lc->Listen();
        lc->SetAcceptCallback(MakeNullCallback<bool, Ptr<Socket>, const Address&>(),
                              MakeCallback(&Vessel::AcceptCtl, v));
        v->lData = ld;
        v->lCtl = lc;
        // the bridges of the GS connect 1 s into the run, the regulator ticks from 1.5 s, the sources follow
        // their schedule
        Simulator::Schedule(Seconds(1.0 + 0.002 * i), &GsConnect, int(i));
        Simulator::Schedule(Seconds(1.5 + 0.0001 * i), &Vessel::Tick, v);
        if (!v->src.empty())
        {
            int64_t at = std::max<int64_t>(0, (v->src[0].atUs * 1000 - g_originNs));
            Simulator::Schedule(NanoSeconds(at), &Vessel::SrcNext, v);
        }
        for (auto& gl : goalsOf[i])
        {
            int64_t at = gl.first * 1000 - g_originNs;
            if (at < 0)
            {
                continue;
            }
            std::string payload(gl.second, 'g');
            Simulator::Schedule(NanoSeconds(at), &GsPublishCtl, int(i), "boat/b" + std::to_string(i) + "/goals", payload);
        }
    }
    // phase 2 of the AS from step S0
    {
        int64_t at = (t0Us + p2OffUs) * 1000 - g_originNs;
        Simulator::Schedule(NanoSeconds(at), &Phase2, S0, S0 + steps);
    }

    // ---- external BSS (CSMA interferer): OnOff UDP between ap2 and sta2
    if (hasExt)
    {
        InternetStackHelper inet;
        NodeContainer ext2(all.Get(g_nAp2), all.Get(g_nSta2));
        inet.Install(ext2);
        Ipv4AddressHelper ip;
        ip.SetBase("10.20.0.0", "255.255.255.0");
        Ipv4InterfaceContainer ifs = ip.Assign(extDevs);
        bool up = ext["dir"].Str("up") == "up";
        int rx = up ? 0 : 1, tx = up ? 1 : 0;
        PacketSinkHelper sink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 9));
        sink.Install(ext2.Get(rx)).Start(Seconds(0));
        const mj::Value& S = ext["schedule"];
        for (size_t k = 0; k < S.size(); k++)
        {
            OnOffHelper oo("ns3::UdpSocketFactory", InetSocketAddress(ifs.GetAddress(rx), 9));
            oo.SetAttribute("DataRate", DataRateValue(DataRate(uint64_t(S[k][2].Num()))));
            oo.SetAttribute("PacketSize", UintegerValue(uint32_t(S[k][3].Num(1400))));
            oo.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1e9]"));
            oo.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
            ApplicationContainer a = oo.Install(ext2.Get(tx));
            double t0 = S[k][0].Num(), t1 = S[k][1].Num();
            a.Start(Seconds(1e6)); // replaced by the deferred start
            a.Stop(Seconds(1e6));
            g_deferred.push_back([a, t0, t1](double shift) {
                ApplicationContainer b = a;
                b.Start(Seconds(std::max(0.0, shift + t0)));
                b.Stop(Seconds(std::max(0.0, shift + t1)));
            });
        }
    }

    // ---- non-CSMA jammer
    if (hasJam)
    {
        WaveformGeneratorHelper wgh;
        wgh.SetChannel(channel);
        wgh.SetTxPowerSpectralDensity(JamPsd(jam["schedule"][0][4].Num(10)));
        NetDeviceContainer jd = wgh.Install(all.Get(g_nJam));
        Ptr<WaveformGenerator> wg =
            DynamicCast<WaveformGenerator>(DynamicCast<NonCommunicatingNetDevice>(jd.Get(0))->GetPhy());
        wg->TraceConnectWithoutContext("TxStart", MakeBoundCallback(&OnJamTxStart, g_nJam, 0.0, wg));
        const mj::Value& S = jam["schedule"];
        for (size_t k = 0; k < S.size(); k++)
        {
            double t0 = S[k][0].Num(), t1 = S[k][1].Num(), per = S[k][2].Num(), duty = S[k][3].Num(),
                   pw = S[k][4].Num(10);
            g_deferred.push_back([wg, t0, t1, per, duty, pw](double shift) {
                if (shift + t1 <= 0)
                {
                    return;
                }
                Simulator::Schedule(Seconds(std::max(0.0, shift + t0)), &JamOn, wg, per, duty, pw);
                Simulator::Schedule(Seconds(shift + t1), &JamOff, wg);
            });
        }
    }

    // ---- run
    Simulator::Schedule(NanoSeconds(mon.m_periodNs), &Monitor::Emit, &mon);
    Simulator::Schedule(Seconds(10), &Status);
    int64_t stopNs = endUs * 1000 - g_originNs;
    Simulator::Stop(NanoSeconds(stopNs));
    for (auto& f : g_deferred)
    {
        f(shift);
    }
    if (!flush.empty())
    {
        auto colon = flush.find(':');
        NS_ABORT_MSG_IF(colon == std::string::npos, "--flush=TS_US:OFF_US[,OFF_US...]");
        mon.m_stepNs = int64_t(std::stod(flush.substr(0, colon)) * 1e3);
        std::string rest = flush.substr(colon + 1);
        size_t p0 = 0;
        while (p0 < rest.size())
        {
            size_t p1 = rest.find(',', p0);
            if (p1 == std::string::npos)
            {
                p1 = rest.size();
            }
            mon.m_flushOffNs.push_back(int64_t(std::stod(rest.substr(p0, p1 - p0)) * 1e3));
            p0 = p1 + 1;
        }
        mon.m_stepEpochSimNs = int64_t(shift * 1e9);
        mon.ScheduleFlushes();
    }
    std::cerr << "START nodes=" << g_names.size() << " M=" << g_M << " S0=" << S0 << " steps=" << steps
              << " outdir=" << outdir << " stop_s=" << stopNs / 1e9 << std::endl;
    if (g_gt)
    {
        std::ostringstream hj;
        hj << "{\"origin_ns\":" << g_originNs << ",\"shift_s\":" << shift << ",\"record_size\":" << sizeof(GtRec)
           << ",\"M\":" << g_M << ",\"simulated\":true"
           << ",\"scenario\":\"" << scenarioPath << "\",\"nodes\":[";
        for (size_t n = 0; n < g_names.size(); n++)
        {
            hj << (n ? "," : "") << "{\"name\":\"" << g_names[n] << "\",\"mac\":\""
               << (n < g_wifiMac.size() ? g_wifiMac[n] : Mac48Address()) << "\"}";
        }
        hj << "]}";
        std::string h = hj.str();
        uint32_t len = uint32_t(h.size());
        fwrite("FDTGT1\0\0", 1, 8, g_gt);
        fwrite(&len, 4, 1, g_gt);
        fwrite(h.data(), 1, len, g_gt);
        fflush(g_gt);
        g_gtBuf.reserve(16384);
        g_gtThread = std::thread(GtWriterLoop);
    }
    auto wall0 = std::chrono::steady_clock::now();
    Simulator::Run();
    double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - wall0).count();
    int64_t simEnd = NowNs();

    // ---- summary
    GtClose();
    std::ofstream sm(outdir + "/summary.json");
    sm << "{\"origin_ns\":" << g_originNs << ",\"shift_s\":" << shift << ",\"sim_end_s\":" << simEnd / 1e9
       << ",\"wall_s\":" << wall << ",\"simulated\":true,\"M\":" << g_M << ",\"nodes\":[";
    for (size_t n = 0; n < g_names.size(); n++)
    {
        sm << (n ? "," : "") << "\"" << g_names[n] << "\"";
    }
    sm << "],\"gt_records\":" << g_gtCount << ",\"monitor\":{\"batches\":" << mon.m_batches
       << ",\"records\":" << mon.m_records << ",\"send_errors\":0},\"phase2\":" << g_p2
       << ",\"gs\":{\"data_msgs\":" << g_gsDataMsgs << ",\"demands\":" << g_gsDemands << ",\"vlogs\":" << g_gsVlogs
       << "},\"hosts\":[],\"vessels\":[";
    for (uint32_t i = 1; i <= g_M; i++)
    {
        Vessel* v = g_vessels[i];
        const struct fdt_reg_stats& s = v->reg.st_;
        sm << (i > 1 ? "," : "") << "{\"i\":" << i << ",\"raw\":" << s.raw << ",\"released\":" << s.released
           << ",\"discarded\":" << s.discarded << ",\"withheld\":" << s.withheld << ",\"inc\":" << s.inc
           << ",\"over\":" << s.over << ",\"tab_ok\":" << s.tab_ok << ",\"tab_default\":" << s.tab_default
           << ",\"tab_late\":" << s.tab_late << ",\"outs\":" << s.outs << ",\"resync\":" << s.resync
           << ",\"pub_err\":" << v->pubErr << ",\"levels\":" << v->levels << ",\"goals\":" << v->goals << "}";
    }
    sm << "]";
    if (!g_tabMargin.empty())
    {
        std::vector<int64_t> v = g_tabMargin;
        std::sort(v.begin(), v.end());
        auto qq = [&](double f) { return v[std::min(v.size() - 1, size_t(f * (v.size() - 1)))]; };
        size_t neg = 0;
        for (auto x : v)
        {
            neg += x < 0;
        }
        sm << ",\"table_margin_us\":{\"n\":" << v.size() << ",\"late\":" << neg << ",\"min\":" << v.front()
           << ",\"p001\":" << qq(0.001) << ",\"p01\":" << qq(0.01) << ",\"p50\":" << qq(0.5) << "}";
    }
    sm << ",\"phy\":{";
    for (size_t n = 0; n < nWifi; n++)
    {
        sm << (n ? "," : "") << "\"" << g_names[n] << "\":{\"tx\":" << g_txCount[n] << ",\"rx_ok\":" << g_rxOk[n]
           << ",\"rx_err\":" << g_rxErr[n] << ",\"rx_drop\":" << g_rxDrop[n] << "}";
    }
    sm << "},\"conf\":{";
    ReportPhy(sm);
    sm << "}}\n";
    sm.close();
    fprintf(g_proto, "{\"k\":\"end\",\"sim_s\":%.3f,\"wall_s\":%.1f}\n", simEnd / 1e9, wall);
    fflush(g_proto);
    std::cerr << "END sim=" << simEnd / 1e9 << "s wall=" << wall << "s gt=" << g_gtCount << std::endl;
    Simulator::Destroy();
    return 0;
}
