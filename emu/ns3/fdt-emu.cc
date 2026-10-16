// fdt-emu: real-time ns-3 emulator of the shared IEEE 802.11 channel of Fleet-DT v2
// (SPEC_v2.md Section 7; paper Section VI).
//
// One AP node (the GS) and M STA nodes (the vessels), 802.11g ERP-OFDM with QoS (EDCA) on
// SpectrumWifiPhy + MultiModelSpectrumChannel. Each node relays the IPv4 unicast packets of one
// QEMU/KVM VM: the VM's data NIC is a `-netdev dgram` UDP pair, the emulator holds the other end in an
// FdNetDevice (one datagram = one Ethernet frame), and an L2 relay maps the destination VM MAC to the
// Wi-Fi node of the destination, sends the IPv4 packet with WifiNetDevice::Send (UP from the DSCP and
// the port hints via SocketPriorityTag) and, on reception, rebuilds the Ethernet frame with the VM MACs.
//
// Outputs (directory --outdir):
//   gt.bin       ground truth: every PHY TX, RX ok, RX error and PPDU drop of every node, the PHY
//                states of the AP (or of all nodes), the jammer bursts (format: emu/README.md)
//   lag.txt      every 10 ms: simulator time (us) and lag = wall clock - (origin + simulator time) (us)
//   summary.json counters, lag statistics, configuration report
//   monitor      UDP JSON batches (SPEC_v2 4.3) every 5 ms to --monitor (monitor_bridge.py)
//   counters.jsonl  with --cnt: the nl80211 counters of the AP (survey busy/active time in whole ms,
//                station bytes, packets, tx_failed) read at the step boundaries, one JSON line per read
//                in the sample format of fdt_chmon --record; also sent as UDP datagrams to --cntDst
//                (fleetdt/monitor_adapter.py computes y(t) from them)
//
// Usage (from the ns-3 tree):  ./ns3 run "fdt-emu --scenario=S.json --outdir=D --basePort=41000"

#include "minijson.h"

#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/fd-net-device-module.h"
#include "ns3/fd-reader.h"
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
#include <map>
#include <random>
#include <netinet/in.h>
#include <sys/socket.h>
#include <time.h>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <unistd.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("FdtEmu");

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
        if (m_sock >= 0)
        {
            if (sendto(m_sock, js.data(), js.size(), MSG_DONTWAIT, (sockaddr*)&m_dst, sizeof m_dst) < 0)
            {
                m_sendErr++;
            }
        }
        m_batches++;
        m_from = to;
    }
};

// ---------------------------------------------------------------------------------------------
// nl80211 counter export of the AP (paper R6, Section V-B)
//
// Only what the driver of a deployed AP (mac80211 + ath9k) gives through nl80211, read at the step
// boundaries as radio/channel_monitor/fdt_chmon.c reads it:
//   survey of the channel in use: NL80211_SURVEY_INFO_TIME and NL80211_SURVEY_INFO_TIME_BUSY, whole ms
//   station dump of every associated vessel: RX/TX bytes (64 bit), RX/TX packets, TX failed and
//   TX retries (32 bit)
// One read = one JSON line in the sample format of `fdt_chmon --record` (sums over the stations, the
// way fdt_chmon sums a station dump) plus the counters of each station in "sta". The busy intervals
// and the per-frame success flags of the monitor stream (ground truth) do not enter the record.
//
// Semantics of the emulated counters:
//   busy        union of the AP PHY TX, RX and primary-channel CCA-busy periods (rx_clear)
//   time        radio-on time (simulator time since the start of the emulator)
//   quantization "harvest" (ath9k, default): at each harvest (each read, and every cntHarvestMs > 0,
//               the ANI poll) the driver adds floor(time since the last harvest / 1 ms) to each
//               counter and drops the remainder; "cumulative": floor(total time / 1 ms)
//   rx_*        unicast data, QoS-null and management frames that the AP receives without error from
//               the station, without duplicates (Retry bit set and the sequence control of the last
//               frame of the same TID, as mac80211 drops them before it counts), bytes = MPDU without
//               FCS (skb->len)
//   tx_*        unicast data, QoS-null and management frames that the AP sends to the station, once per
//               MPDU at its first transmission (Retry bit clear), bytes = MPDU without FCS
//   tx_failed   MPDUs to the station dropped at the retry limit, or at the lifetime after at least one
//               failed transmission (Retry bit set)
//   tx_retries  transmissions with the Retry bit set
//   a station that deassociates leaves the dump and comes back with zeroed counters
// Read instants: runStartUs + k TS + OFF + U[0, JITTER] (host clock), one uniform draw per read from
// a generator of its own (seeded by cntSeed), so the random streams of the channel model do not change.

int64_t
FloorDiv(int64_t a, int64_t b)
{
    int64_t q = a / b;
    return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}

struct StaCounters
{
    bool assoc = false;
    uint64_t rxBytes = 0, txBytes = 0;
    uint32_t rxPackets = 0, txPackets = 0, txFailed = 0, txRetries = 0, rxDup = 0;
    std::map<uint8_t, uint16_t> lastSeqCtl; // TID (16: non-QoS) -> sequence control of the last frame
};

class Nl80211Counters;
Nl80211Counters* g_cnt = nullptr;

class Nl80211Counters
{
  public:
    int m_sock = -1;
    sockaddr_in m_dst{};
    FILE* m_file = nullptr;
    int64_t m_stepNs = 125000000, m_offNs = 500000, m_jitterNs = 0;
    int64_t m_epochSimNs = 0; // simulator time of a step start (runStartUs)
    bool m_cumulative = false;
    int64_t m_harvestNs = 0;
    std::mt19937_64 m_rng{1};
    Mac48Address m_apMac;
    std::vector<StaCounters> m_sta; // node index 1..M
    uint64_t m_reads = 0, m_sendErr = 0;

    // busy timeline: total of the closed intervals and the open one
    int64_t m_closedNs = 0, m_curS = -1, m_curE = -1;
    // survey counters (ms) and the state of the last harvest
    uint64_t m_timeMs = 0, m_busyMs = 0;
    int64_t m_lastHarvestNs = 0, m_lastBusyNs = 0;
    int64_t m_k = 0; // index of the next read

    void Busy(int64_t s, int64_t e)
    {
        if (e <= s)
        {
            return;
        }
        if (m_curS < 0 || s > m_curE)
        {
            if (m_curS >= 0)
            {
                m_closedNs += m_curE - m_curS;
            }
            m_curS = s;
            m_curE = e;
        }
        else
        {
            m_curE = std::max(m_curE, e);
        }
    }

    int64_t BusyNs(int64_t at) const
    {
        int64_t b = m_closedNs;
        if (m_curS >= 0 && at > m_curS)
        {
            b += std::min(at, m_curE) - m_curS;
        }
        return b;
    }

    void Harvest(int64_t at)
    {
        int64_t busy = BusyNs(at);
        m_timeMs += uint64_t((at - m_lastHarvestNs) / 1000000);
        m_busyMs += uint64_t((busy - m_lastBusyNs) / 1000000);
        m_lastHarvestNs = at;
        m_lastBusyNs = busy;
    }

    void PeriodicHarvest()
    {
        Harvest(NowNs());
        Simulator::Schedule(NanoSeconds(m_harvestNs), &Nl80211Counters::PeriodicHarvest, this);
    }

    static bool Counted(const FrameInfo& fi)
    {
        return fi.kind == 0 || fi.kind == 2 || fi.kind == 6; // data, management, QoS/null
    }

    StaCounters* Sta(const Mac48Address& a)
    {
        int n = NodeOf(a);
        if (n < 1 || n > int(g_M) || !m_sta[n].assoc)
        {
            return nullptr;
        }
        return &m_sta[n];
    }

    // a PSDU that the AP starts to send (size with FCS)
    void ApTx(const FrameInfo& fi)
    {
        if (fi.hdr.GetAddr1().IsGroup() || !Counted(fi))
        {
            return;
        }
        StaCounters* s = Sta(fi.hdr.GetAddr1());
        if (!s)
        {
            return;
        }
        if (fi.hdr.IsRetry())
        {
            s->txRetries++;
        }
        else
        {
            s->txPackets++;
            s->txBytes += fi.size >= 4 ? fi.size - 4 : 0;
        }
    }

    // a frame that the AP receives without error (size with FCS)
    void ApRx(const FrameInfo& fi)
    {
        if (fi.hdr.GetAddr1() != m_apMac || !Counted(fi))
        {
            return;
        }
        StaCounters* s = Sta(fi.hdr.GetAddr2());
        if (!s)
        {
            return;
        }
        uint8_t tid = fi.hdr.IsQosData() ? fi.hdr.GetQosTid() : 16;
        uint16_t sc = fi.hdr.GetSequenceControl();
        auto it = s->lastSeqCtl.find(tid);
        if (fi.hdr.IsRetry() && it != s->lastSeqCtl.end() && it->second == sc)
        {
            s->rxDup++;
            return;
        }
        s->lastSeqCtl[tid] = sc;
        s->rxPackets++;
        s->rxBytes += fi.size >= 4 ? fi.size - 4 : 0;
    }

    void ApDropped(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu)
    {
        const WifiMacHeader& h = mpdu->GetHeader();
        if (h.GetAddr1().IsGroup() || !(h.IsData() || h.IsMgt()))
        {
            return;
        }
        bool failed = reason == WIFI_MAC_DROP_REACHED_RETRY_LIMIT ||
                      (reason == WIFI_MAC_DROP_EXPIRED_LIFETIME && h.IsRetry());
        StaCounters* s = Sta(h.GetAddr1());
        if (failed && s)
        {
            s->txFailed++;
        }
    }

    void SetAssoc(int node, bool on)
    {
        if (node < 1 || node > int(g_M))
        {
            return;
        }
        if (!on || !m_sta[node].assoc)
        {
            m_sta[node] = StaCounters(); // a new station entry starts from zero
        }
        m_sta[node].assoc = on;
    }

    int64_t ReadAt(int64_t k)
    {
        int64_t j = 0;
        if (m_jitterNs > 0)
        {
            j = std::uniform_int_distribution<int64_t>(0, m_jitterNs)(m_rng);
        }
        return m_epochSimNs + k * m_stepNs + m_offNs + j;
    }

    void Start()
    {
        NS_ABORT_MSG_IF(m_stepNs <= 0 || m_offNs < 0 || m_offNs + m_jitterNs >= m_stepNs,
                        "--cnt: need TS > 0 and 0 <= OFF, OFF + JITTER < TS");
        int64_t now = NowNs();
        m_k = FloorDiv(now - m_epochSimNs - m_offNs, m_stepNs) + 1;
        int64_t at = ReadAt(m_k);
        Simulator::Schedule(NanoSeconds(at - now), &Nl80211Counters::Read, this);
        if (!m_cumulative && m_harvestNs > 0)
        {
            Simulator::Schedule(NanoSeconds(m_harvestNs), &Nl80211Counters::PeriodicHarvest, this);
        }
    }

    void Read()
    {
        int64_t now = NowNs();
        uint64_t timeMs, busyMs;
        if (m_cumulative)
        {
            timeMs = uint64_t(now / 1000000);
            busyMs = uint64_t(BusyNs(now) / 1000000);
        }
        else
        {
            Harvest(now);
            timeMs = m_timeMs;
            busyMs = m_busyMs;
        }
        uint64_t rxb = 0, txb = 0;
        uint32_t rxp = 0, txp = 0, txf = 0; // 32-bit sums, as fdt_chmon sums a station dump
        int nsta = 0;
        std::string sta;
        char buf[320];
        for (uint32_t n = 1; n <= g_M; n++)
        {
            const StaCounters& s = m_sta[n];
            if (!s.assoc)
            {
                continue;
            }
            rxb += s.rxBytes;
            txb += s.txBytes;
            rxp += s.rxPackets;
            txp += s.txPackets;
            txf += s.txFailed;
            snprintf(buf,
                     sizeof buf,
                     "%s{\"i\":%u,\"rx_bytes\":%llu,\"tx_bytes\":%llu,\"rx_packets\":%u,\"tx_packets\":%u,"
                     "\"tx_failed\":%u,\"tx_retries\":%u}",
                     nsta ? "," : "",
                     n,
                     (unsigned long long)s.rxBytes,
                     (unsigned long long)s.txBytes,
                     s.rxPackets,
                     s.txPackets,
                     s.txFailed,
                     s.txRetries);
            sta += buf;
            nsta++;
        }
        int64_t us = HostUs(now);
        std::string js;
        js.reserve(256 + sta.size());
        snprintf(buf,
                 sizeof buf,
                 "{\"at\":%lld.%06lld,\"survey\":1,\"time_ms\":%llu,\"busy_ms\":%llu,\"rx_bytes\":%llu,"
                 "\"tx_bytes\":%llu,\"rx_packets\":%u,\"tx_packets\":%u,\"tx_failed\":%u,\"stations\":%d,"
                 "\"sta\":[",
                 (long long)(us / 1000000),
                 (long long)(us % 1000000),
                 (unsigned long long)timeMs,
                 (unsigned long long)busyMs,
                 (unsigned long long)rxb,
                 (unsigned long long)txb,
                 rxp,
                 txp,
                 txf,
                 nsta);
        js += buf;
        js += sta;
        js += "]}\n";
        if (m_sock >= 0 &&
            sendto(m_sock, js.data(), js.size(), MSG_DONTWAIT, (sockaddr*)&m_dst, sizeof m_dst) < 0)
        {
            m_sendErr++;
        }
        if (m_file)
        {
            fputs(js.c_str(), m_file);
        }
        m_reads++;
        m_k++;
        int64_t at = ReadAt(m_k);
        Simulator::Schedule(NanoSeconds(at - now), &Nl80211Counters::Read, this);
    }
};

void
OnApDroppedMpdu(WifiMacDropReason reason, Ptr<const WifiMpdu> mpdu)
{
    if (g_cnt)
    {
        g_cnt->ApDropped(reason, mpdu);
    }
}

class MonListener : public WifiPhyListener
{
  public:
    void NotifyRxStart(Time d) override
    {
        g_mon->Busy(NowNs(), NowNs() + d.GetNanoSeconds());
        if (g_cnt)
        {
            g_cnt->Busy(NowNs(), NowNs() + d.GetNanoSeconds());
        }
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
        if (g_cnt)
        {
            g_cnt->Busy(NowNs(), NowNs() + d.GetNanoSeconds());
        }
    }

    void NotifyCcaBusyStart(Time d, WifiChannelListType t, const std::vector<Time>&) override
    {
        if (t == WIFI_CHANLIST_PRIMARY)
        {
            g_mon->Busy(NowNs(), NowNs() + d.GetNanoSeconds());
            if (g_cnt)
            {
                g_cnt->Busy(NowNs(), NowNs() + d.GetNanoSeconds());
            }
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
    if (node == 0 && g_cnt)
    {
        g_cnt->ApTx(fi);
    }
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
    if (node == 0 && ok && g_cnt)
    {
        g_cnt->ApRx(fi);
    }
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
// L2 relay between the VMs and the Wi-Fi nodes

class UdpFdReader : public FdReader
{
  protected:
    FdReader::Data DoRead() override
    {
        auto buf = (uint8_t*)malloc(2048);
        ssize_t len = recv(m_fd, buf, 2048, 0);
        if (len <= 0)
        {
            free(buf);
            // a transient socket error (e.g. ICMP port unreachable) must not stop the reader
            return FdReader::Data(nullptr, -1);
        }
        return FdReader::Data(buf, len);
    }
};

// FdNetDevice on an unconnected UDP socket: sendto() the QEMU end of the dgram pair
class UdpFdNetDevice : public FdNetDevice
{
  public:
    sockaddr_in m_peer{};

  protected:
    ssize_t Write(uint8_t* buffer, size_t length) override
    {
        return sendto(GetFileDescriptor(), buffer, length, MSG_DONTWAIT, (sockaddr*)&m_peer, sizeof m_peer);
    }

    Ptr<FdReader> DoCreateFdReader() override
    {
        return Create<UdpFdReader>();
    }
};

struct Host
{
    Ptr<Node> node;
    Ptr<UdpFdNetDevice> fd;
    Mac48Address vmMac;
    uint32_t ip = 0; // host order
    uint64_t fromVm = 0, toVm = 0, dropFromVm = 0, dropToVm = 0;
};

std::vector<Host> g_hosts;                 // 0 = GS, i = vessel i
std::map<Mac48Address, int> g_vmMacToHost; // VM MAC -> host index
std::map<int, int> g_dscpUp;                // DSCP -> UP
std::map<int, int> g_tcpPortUp;             // TCP port -> UP (either direction)
uint64_t g_nonIp = 0;

int
HostOfIp(uint32_t ip)
{
    for (size_t h = 0; h < g_hosts.size(); h++)
    {
        if (g_hosts[h].ip == ip)
        {
            return int(h);
        }
    }
    return -1;
}

void
FromVm(int h,
       Ptr<NetDevice>,
       Ptr<const Packet> p,
       uint16_t proto,
       const Address&,
       const Address& to,
       NetDevice::PacketType)
{
    Host& H = g_hosts[h];
    if (proto != 0x0800 || p->GetSize() < 20)
    {
        g_nonIp++;
        return;
    }
    Ptr<Packet> pkt = p->Copy();
    uint8_t ip[24];
    pkt->CopyData(ip, std::min<uint32_t>(pkt->GetSize(), sizeof ip));
    uint32_t tot = (ip[2] << 8) | ip[3];
    if (tot >= 20 && tot < pkt->GetSize())
    {
        pkt->RemoveAtEnd(pkt->GetSize() - tot); // Ethernet padding
    }
    Mac48Address dmac = Mac48Address::ConvertFrom(to);
    int d = -1;
    auto it = g_vmMacToHost.find(dmac);
    if (it != g_vmMacToHost.end())
    {
        d = it->second;
    }
    else if (!dmac.IsGroup())
    {
        uint32_t dip = (uint32_t(ip[16]) << 24) | (ip[17] << 16) | (ip[18] << 8) | ip[19];
        d = HostOfIp(dip);
    }
    if (d < 0 || d == h)
    {
        H.dropFromVm++;
        return;
    }
    int up = 0;
    auto du = g_dscpUp.find(ip[1] >> 2);
    if (du != g_dscpUp.end())
    {
        up = du->second;
    }
    uint32_t ihl = (ip[0] & 0x0f) * 4;
    if (ip[9] == 6 && ihl == 20 && pkt->GetSize() >= 24)
    {
        int sp = (ip[20] << 8) | ip[21], dp = (ip[22] << 8) | ip[23];
        for (int port : {sp, dp})
        {
            auto pu = g_tcpPortUp.find(port);
            if (pu != g_tcpPortUp.end())
            {
                up = std::max(up, pu->second);
            }
        }
    }
    SocketPriorityTag tag;
    tag.SetPriority(uint8_t(up));
    pkt->ReplacePacketTag(tag);
    Mac48Address wdst = (h != 0 && d == 0) ? g_wifiMac[0] : g_wifiMac[d];
    if (g_wifiDev[h]->Send(pkt, wdst, 0x0800))
    {
        H.fromVm++;
    }
    else
    {
        H.dropFromVm++;
    }
    return;
}

void
FromWifi(int h,
         Ptr<NetDevice>,
         Ptr<const Packet> p,
         uint16_t proto,
         const Address& from,
         const Address&,
         NetDevice::PacketType)
{
    Host& H = g_hosts[h];
    if (proto != 0x0800 || !H.fd)
    {
        return;
    }
    int s = NodeOf(Mac48Address::ConvertFrom(from));
    if (s < 0 || s > int(g_M))
    {
        H.dropToVm++;
        return;
    }
    Ptr<Packet> pkt = p->Copy();
    pkt->RemoveAllPacketTags();
    if (H.fd->SendFrom(pkt, g_hosts[s].vmMac, H.vmMac, 0x0800))
    {
        H.toVm++;
    }
    else
    {
        H.dropToVm++;
    }
}

// built-in sources for tests without VMs: each vessel sends `size`-byte IPv4/UDP packets at `rate`
void
SelfTestSend(int h, double interval, uint32_t size)
{
    uint8_t ip[28] = {0x45, 0, 0, 0, 0, 0, 0x40, 0, 64, 17, 0, 0};
    ip[2] = size >> 8;
    ip[3] = size & 0xff;
    uint32_t sip = g_hosts[h].ip, dip = g_hosts[0].ip;
    for (int k = 0; k < 4; k++)
    {
        ip[12 + k] = (sip >> (24 - 8 * k)) & 0xff;
        ip[16 + k] = (dip >> (24 - 8 * k)) & 0xff;
    }
    ip[20] = 0x9c;
    ip[21] = 0x40; // sport 40000
    ip[22] = 0x9c;
    ip[23] = 0x41;
    Ptr<Packet> full = Create<Packet>(ip, 28);
    full->AddAtEnd(Create<Packet>(size - 28));
    g_wifiDev[h]->Send(full, g_wifiMac[0], 0x0800);
    Simulator::Schedule(Seconds(interval), &SelfTestSend, h, interval, size);
}

// downlink self test: UDP packets of `size` IP bytes from the GS (AP) to vessel h
void
SelfTestSendDown(int h, double interval, uint32_t size)
{
    uint8_t ip[28] = {0x45, 0, 0, 0, 0, 0, 0x40, 0, 64, 17, 0, 0};
    ip[2] = size >> 8;
    ip[3] = size & 0xff;
    uint32_t sip = g_hosts[0].ip, dip = g_hosts[h].ip;
    for (int k = 0; k < 4; k++)
    {
        ip[12 + k] = (sip >> (24 - 8 * k)) & 0xff;
        ip[16 + k] = (dip >> (24 - 8 * k)) & 0xff;
    }
    ip[20] = 0x9c;
    ip[21] = 0x42; // sport 40002
    ip[22] = 0x9c;
    ip[23] = 0x43;
    Ptr<Packet> full = Create<Packet>(ip, 28);
    full->AddAtEnd(Create<Packet>(size - 28));
    g_wifiDev[0]->Send(full, g_wifiMac[h], 0x0800);
    Simulator::Schedule(Seconds(interval), &SelfTestSendDown, h, interval, size);
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

uint64_t g_lastRelay = 0;

void
Status()
{
    LagStats ls = ComputeLag(std::vector<float>(g_lagUs.end() - std::min<size_t>(g_lagUs.size(), 1000),
                                                g_lagUs.end()),
                             0);
    uint64_t relay = 0;
    for (auto& h : g_hosts)
    {
        relay += h.fromVm + h.toVm;
    }
    if (!g_realtime)
    {
        ls = LagStats();
    }
    std::cout << "STATUS t=" << Simulator::Now().GetSeconds() << "s lag10s_p99_us=" << ls.p99
              << " lag10s_max_us=" << ls.max << " relayed=" << relay << " (+" << (relay - g_lastRelay)
              << ") gt=" << g_gtCount << " mon_batches=" << (g_mon ? g_mon->m_batches : 0) << std::endl;
    g_lastRelay = relay;
    Simulator::Schedule(Seconds(10), &Status);
}

// ---------------------------------------------------------------------------------------------
// association

uint32_t g_assoc = 0;

void
OnAssoc(int node, Mac48Address bssid)
{
    if (g_cnt)
    {
        g_cnt->SetAssoc(node, true);
    }
    g_assoc++;
    std::cout << "ASSOC " << g_names[node] << " t=" << Simulator::Now().GetSeconds() << std::endl;
    if (g_assoc == g_M)
    {
        std::cout << "READY t=" << Simulator::Now().GetSeconds() << std::endl;
    }
}

void
OnDeAssoc(int node, Mac48Address bssid)
{
    if (g_cnt)
    {
        g_cnt->SetAssoc(node, false);
    }
    if (g_assoc > 0)
    {
        g_assoc--;
    }
    std::cout << "DEASSOC " << g_names[node] << " t=" << Simulator::Now().GetSeconds() << std::endl;
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
    std::cout << "PHYCONF {";
    ReportPhy(std::cout);
    std::cout << "}" << std::endl;
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

} // namespace

int
main(int argc, char* argv[])
{
    std::string scenarioPath, outdir = ".", monitor = "127.0.0.1:9550", rtMode = "HardLimit";
    uint32_t basePort = 41000;
    double duration = 0, hardLimitMs = 50;
    bool vms = true, gt = true;
    std::string selftest, selftestDown;
    std::string runStartUs; // host time (us) of scenario time 0 of the schedules and the waypoints
    std::string flush;      // "TS_US:OFF1_US,OFF2_US": extra monitor batches at runStart + k Ts + off
    std::string cntSpec;    // "TS_US:OFF_US[:JITTER_US]": nl80211 counter reads at runStart + k Ts + off + U[0, jitter]
    std::string cntDst = "none";
    std::string cntQuant = "harvest";
    double cntHarvestMs = 0;
    bool cntFile = true;
    int64_t cntSeed = -1;

    CommandLine cmd(__FILE__);
    cmd.AddValue("scenario", "scenario JSON (emu/scenarios)", scenarioPath);
    cmd.AddValue("outdir", "output directory", outdir);
    cmd.AddValue("basePort", "UDP port of host h: QEMU base+2h, emulator base+2h+1", basePort);
    cmd.AddValue("monitor", "host:port of monitor_bridge.py (none: no monitor stream)", monitor);
    cmd.AddValue("duration", "seconds of emulated time (0: until SIGTERM/SIGINT)", duration);
    cmd.AddValue("rt", "HardLimit, BestEffort or none (not real time; with --vms=0)", rtMode);
    cmd.AddValue("hardLimitMs", "lag at which HardLimit aborts (ms)", hardLimitMs);
    cmd.AddValue("vms", "attach the VM sockets", vms);
    cmd.AddValue("gt", "write the ground truth gt.bin", gt);
    cmd.AddValue("gtStates", "PHY state records in gt.bin: ap, all or none", g_gtStates);
    cmd.AddValue("selftest", "RATE_BPS,SIZE: built-in uplink sources of every vessel (no VMs)", selftest);
    cmd.AddValue("selftestDown", "RATE_BPS,SIZE: built-in downlink sources of the GS to every vessel (no VMs)",
                 selftestDown);
    cmd.AddValue("runStartUs",
                 "host time (us) at which the scenario time 0 of the interference schedules and the waypoints "
                 "falls (default: the start of the emulator)",
                 runStartUs);
    cmd.AddValue("flush",
                 "TS_US:OFF_US[,OFF_US...]: extra monitor batches at the host times runStartUs + k TS + OFF "
                 "(the ends of Q(t) and W(t))",
                 flush);
    cmd.AddValue("cnt",
                 "TS_US:OFF_US[:JITTER_US]: export the nl80211 counters of the AP, read at the host times "
                 "runStartUs + k TS + OFF + U[0, JITTER] (empty: no export)",
                 cntSpec);
    cmd.AddValue("cntDst", "host:port of the counter datagrams (monitor_adapter.py; none: file only)", cntDst);
    cmd.AddValue("cntFile", "write the counter reads to OUTDIR/counters.jsonl", cntFile);
    cmd.AddValue("cntQuant",
                 "survey quantization: harvest (ath9k: floor per harvest, remainder dropped) or cumulative",
                 cntQuant);
    cmd.AddValue("cntHarvestMs", "extra survey harvests every MS (ath9k ANI poll; 0: only at the reads)", cntHarvestMs);
    cmd.AddValue("cntSeed", "seed of the read jitter (default: the scenario seed)", cntSeed);
    cmd.Parse(argc, argv);

    mj::Value sc = mj::ParseFile(scenarioPath);
    g_M = uint32_t(sc["M"].Num(2));
    if (sc.Has("duration_s") && duration == 0)
    {
        duration = sc["duration_s"].Num();
    }

    if (rtMode == "none")
    {
        g_realtime = false;
        // offline (as fast as possible) runs of the same model, only with --vms=0
        NS_ABORT_MSG_IF(vms, "--rt=none needs --vms=0");
    }
    else
    {
        GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
        Config::SetDefault("ns3::RealtimeSimulatorImpl::SynchronizationMode",
                           StringValue(rtMode == "HardLimit" ? "HardLimit" : "BestEffort"));
        Config::SetDefault("ns3::RealtimeSimulatorImpl::HardLimit", TimeValue(MilliSeconds(hardLimitMs)));
    }
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(false));
    RngSeedManager::SetSeed(uint32_t(sc["seed"].Num(1)));
    RngSeedManager::SetRun(uint32_t(sc["run"].Num(1)));
    const mj::Value& q = sc["mac_queue"];
    Config::SetDefault("ns3::WifiMacQueue::MaxSize",
                       QueueSizeValue(QueueSize(std::to_string(int(q["packets"].Num(100))) + "p")));
    Config::SetDefault("ns3::WifiMacQueue::MaxDelay", TimeValue(MilliSeconds(q["max_delay_ms"].Num(500))));

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
    g_lagF = fopen((outdir + "/lag.txt").c_str(), "w");
    setvbuf(g_lagF, nullptr, _IOFBF, 1 << 16);

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
    if (!monitor.empty() && monitor != "none")
    {
        auto colon = monitor.rfind(':');
        mon.m_sock = socket(AF_INET, SOCK_DGRAM, 0);
        mon.m_dst.sin_family = AF_INET;
        mon.m_dst.sin_port = htons(uint16_t(std::stoi(monitor.substr(colon + 1))));
        inet_pton(AF_INET, monitor.substr(0, colon).c_str(), &mon.m_dst.sin_addr);
        int sz = 4 << 20;
        setsockopt(mon.m_sock, SOL_SOCKET, SO_SNDBUF, &sz, sizeof sz);
    }
    // ---- nl80211 counter export
    Nl80211Counters cnt;
    if (!cntSpec.empty())
    {
        std::vector<int64_t> v;
        size_t p0 = 0;
        while (p0 <= cntSpec.size())
        {
            size_t p1 = cntSpec.find(':', p0);
            if (p1 == std::string::npos)
            {
                p1 = cntSpec.size();
            }
            v.push_back(int64_t(std::stod(cntSpec.substr(p0, p1 - p0)) * 1e3));
            p0 = p1 + 1;
        }
        NS_ABORT_MSG_IF(v.size() < 2 || v.size() > 3, "--cnt=TS_US:OFF_US[:JITTER_US]");
        NS_ABORT_MSG_IF(cntQuant != "harvest" && cntQuant != "cumulative", "--cntQuant: harvest or cumulative");
        cnt.m_stepNs = v[0];
        cnt.m_offNs = v[1];
        cnt.m_jitterNs = v.size() > 2 ? v[2] : 0;
        cnt.m_cumulative = cntQuant == "cumulative";
        cnt.m_harvestNs = int64_t(cntHarvestMs * 1e6);
        cnt.m_rng.seed(uint64_t(cntSeed >= 0 ? cntSeed : int64_t(sc["seed"].Num(1)) * 7919 + 17));
        cnt.m_apMac = g_wifiMac[0];
        cnt.m_sta.resize(g_M + 1);
        if (!cntDst.empty() && cntDst != "none")
        {
            auto colon = cntDst.rfind(':');
            cnt.m_sock = socket(AF_INET, SOCK_DGRAM, 0);
            cnt.m_dst.sin_family = AF_INET;
            cnt.m_dst.sin_port = htons(uint16_t(std::stoi(cntDst.substr(colon + 1))));
            inet_pton(AF_INET, cntDst.substr(0, colon).c_str(), &cnt.m_dst.sin_addr);
        }
        if (cntFile)
        {
            cnt.m_file = fopen((outdir + "/counters.jsonl").c_str(), "w");
            NS_ABORT_MSG_IF(!cnt.m_file, "cannot write " << outdir << "/counters.jsonl");
            setvbuf(cnt.m_file, nullptr, _IOFBF, 1 << 16);
        }
        g_cnt = &cnt;
        g_wifiDev[0]->GetMac()->TraceConnectWithoutContext("DroppedMpdu", MakeCallback(&OnApDroppedMpdu));
    }
    // the state helper keeps a weak reference: the listener lives as long as main()
    auto monListener = std::make_shared<MonListener>();
    g_phy[0]->RegisterListener(monListener);

    // ---- hosts (VM relay)
    const mj::Value& prio = sc["prio"];
    if (prio.Has("dscp"))
    {
        for (auto& kv : prio["dscp"].o)
        {
            g_dscpUp[std::stoi(kv.first)] = int(kv.second.Num());
        }
    }
    else
    {
        g_dscpUp = {{46, 6}, {34, 5}};
    }
    if (prio.Has("tcp_ports"))
    {
        for (auto& kv : prio["tcp_ports"].o)
        {
            g_tcpPortUp[std::stoi(kv.first)] = int(kv.second.Num());
        }
    }
    else
    {
        g_tcpPortUp = {{1883, 6}};
    }
    g_hosts.resize(g_M + 1);
    for (uint32_t h = 0; h <= g_M; h++)
    {
        Host& H = g_hosts[h];
        H.node = all.Get(h);
        char macs[32];
        snprintf(macs, sizeof macs, "52:54:00:fd:00:%02x", h);
        H.vmMac = Mac48Address(macs);
        H.ip = (10u << 24) | (10u << 16) | (h == 0 ? 1u : 10u + h);
        g_vmMacToHost[H.vmMac] = int(h);
        H.node->RegisterProtocolHandler(MakeBoundCallback(&FromWifi, int(h)), 0, g_wifiDev[h], false);
        if (!vms)
        {
            continue;
        }
        int s = socket(AF_INET, SOCK_DGRAM, 0);
        int sz = 4 << 20;
        setsockopt(s, SOL_SOCKET, SO_RCVBUF, &sz, sizeof sz);
        setsockopt(s, SOL_SOCKET, SO_SNDBUF, &sz, sizeof sz);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        a.sin_port = htons(uint16_t(basePort + 2 * h + 1));
        if (bind(s, (sockaddr*)&a, sizeof a) < 0)
        {
            NS_FATAL_ERROR("bind 127.0.0.1:" << basePort + 2 * h + 1 << ": " << strerror(errno));
        }
        H.fd = CreateObject<UdpFdNetDevice>();
        H.fd->m_peer = a;
        H.fd->m_peer.sin_port = htons(uint16_t(basePort + 2 * h));
        char fmac[32];
        snprintf(fmac, sizeof fmac, "02:fd:ee:00:00:%02x", h);
        H.fd->SetAddress(Mac48Address(fmac));
        H.fd->SetFileDescriptor(s);
        H.fd->SetEncapsulationMode(FdNetDevice::DIX);
        H.node->AddDevice(H.fd);
        H.node->RegisterProtocolHandler(MakeBoundCallback(&FromVm, int(h)), 0, H.fd, true);
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

    // ---- self test sources
    if (!selftest.empty())
    {
        auto comma = selftest.find(',');
        double rbps = std::stod(selftest.substr(0, comma));
        uint32_t size = uint32_t(std::stoul(selftest.substr(comma + 1)));
        for (uint32_t i = 1; i <= g_M; i++)
        {
            Simulator::Schedule(Seconds(1.0 + 0.001 * i), &SelfTestSend, int(i), size * 8.0 / rbps, size);
        }
    }
    if (!selftestDown.empty())
    {
        auto comma = selftestDown.find(',');
        double rbps = std::stod(selftestDown.substr(0, comma));
        uint32_t size = uint32_t(std::stoul(selftestDown.substr(comma + 1)));
        for (uint32_t i = 1; i <= g_M; i++)
        {
            Simulator::Schedule(Seconds(1.0005 + 0.001 * i), &SelfTestSendDown, int(i), size * 8.0 / rbps, size);
        }
    }

    // ---- run
    signal(SIGTERM, OnSignal);
    signal(SIGINT, OnSignal);
    Simulator::Schedule(Seconds(0), &LagTick);
    Simulator::Schedule(NanoSeconds(mon.m_periodNs), &Monitor::Emit, &mon);
    Simulator::Schedule(Seconds(10), &Status);
    Simulator::Schedule(Seconds(2), &PrintPhyReport);
    if (duration > 0)
    {
        Simulator::Stop(Seconds(duration));
    }
    std::cout << "START nodes=" << g_names.size() << " M=" << g_M << " vms=" << vms
              << " basePort=" << basePort << " outdir=" << outdir << std::endl;
    g_originNs = WallNs();
    double shift = runStartUs.empty() ? 0.0 : (std::stod(runStartUs) * 1e3 - double(g_originNs)) / 1e9;
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
    if (g_cnt)
    {
        cnt.m_epochSimNs = int64_t(shift * 1e9);
        cnt.Start();
    }
    std::cout << "SHIFT " << shift << " s (scenario time 0 at host us " << (runStartUs.empty() ? "origin" : runStartUs)
              << ")" << std::endl;
    if (g_gt)
    {
        // header: magic, u32 length, JSON (origin, node names and MACs, record layout)
        std::ostringstream hj;
        hj << "{\"origin_ns\":" << g_originNs << ",\"shift_s\":" << shift << ",\"record_size\":" << sizeof(GtRec)
           << ",\"M\":" << g_M
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
    Simulator::Run();
    int64_t simEnd = NowNs();

    // ---- summary
    GtClose();
    if (cnt.m_file)
    {
        fclose(cnt.m_file);
        cnt.m_file = nullptr;
    }
    g_cnt = nullptr;
    for (size_t k = 0; k < g_lagUs.size(); k++)
    {
        fprintf(g_lagF, "%lld %.1f\n", (long long)(k * g_lagEveryNs / 1000), g_lagUs[k]);
    }
    fclose(g_lagF);
    // lag statistics after a warm-up of 3 s (association, reconnection of the TCP connections of the VMs)
    // and over the whole run
    LagStats ls = g_realtime ? ComputeLag(g_lagUs, std::min<size_t>(g_lagUs.size() / 2, 300)) : LagStats();
    LagStats lsAll = g_realtime ? ComputeLag(g_lagUs, 0) : LagStats();
    std::ofstream sm(outdir + "/summary.json");
    sm << "{\"origin_ns\":" << g_originNs << ",\"shift_s\":" << shift << ",\"sim_end_s\":" << simEnd / 1e9
       << ",\"M\":" << g_M
       << ",\"nodes\":[";
    for (size_t n = 0; n < g_names.size(); n++)
    {
        sm << (n ? "," : "") << "\"" << g_names[n] << "\"";
    }
    sm << "],\"lag_us\":{\"n\":" << ls.n << ",\"mean\":" << ls.mean << ",\"p50\":" << ls.p50
       << ",\"p99\":" << ls.p99 << ",\"p999\":" << ls.p999 << ",\"max\":" << ls.max
       << "},\"lag_us_all\":{\"p99\":" << lsAll.p99 << ",\"p999\":" << lsAll.p999 << ",\"max\":" << lsAll.max
       << "},\"gt_records\":"
       << g_gtCount << ",\"monitor\":{\"batches\":" << mon.m_batches << ",\"records\":" << mon.m_records
       << ",\"send_errors\":" << mon.m_sendErr << "},\"nl80211\":{\"reads\":" << cnt.m_reads
       << ",\"send_errors\":" << cnt.m_sendErr << ",\"quant\":\"" << (cntSpec.empty() ? "off" : cntQuant)
       << "\"},\"non_ip_from_vm\":" << g_nonIp << ",\"hosts\":[";
    for (size_t h = 0; h < g_hosts.size(); h++)
    {
        auto& H = g_hosts[h];
        sm << (h ? "," : "") << "{\"from_vm\":" << H.fromVm << ",\"to_vm\":" << H.toVm
           << ",\"drop_from_vm\":" << H.dropFromVm << ",\"drop_to_vm\":" << H.dropToVm << "}";
    }
    sm << "],\"phy\":{";
    for (size_t n = 0; n < nWifi; n++)
    {
        sm << (n ? "," : "") << "\"" << g_names[n] << "\":{\"tx\":" << g_txCount[n] << ",\"rx_ok\":" << g_rxOk[n]
           << ",\"rx_err\":" << g_rxErr[n] << ",\"rx_drop\":" << g_rxDrop[n] << "}";
    }
    sm << "},\"conf\":{";
    ReportPhy(sm);
    sm << "}}\n";
    sm.close();
    std::cout << "END sim=" << simEnd / 1e9 << "s lag_us(after 3 s) p50=" << ls.p50 << " p99=" << ls.p99
              << " p999=" << ls.p999 << " max=" << ls.max << " | whole run p99=" << lsAll.p99
              << " max=" << lsAll.max << " gt=" << g_gtCount << std::endl;
    Simulator::Destroy();
    return 0;
}
