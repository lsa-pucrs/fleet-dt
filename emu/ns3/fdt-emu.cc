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
//   steplag.csv  with --phaseLag: per step, the largest lag of each phase, the lag at the start of
//                phase 3 and at the end of the step, and the catch-up flag (emu/README.md)
//   stepbytes.csv with --stepBytes: per step and vessel, the unique IP packets (and their bytes) that the
//                AP receives without error, in phase 3 and in the whole step (offline subset replays)
//   ap.pcap      with --pcap=1: the IPv4 packets that the AP node hands to the GS (uplink of the vessels,
//                once per MSDU) and that the GS hands to the AP node (downlink), LINKTYPE_RAW, time stamps
//                in nanoseconds of the host clock (origin + simulator time); the E0 conformance check
//                (emu/experiments/conformance.py) reads the MQTT QoS of every forwarded message from it
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
#include "ns3/traffic-control-module.h"
#include "ns3/waveform-generator-helper.h"
#include "ns3/waveform-generator.h"
#include "ns3/wifi-module.h"
#include "ns3/wifi-phy-listener.h"

#include <algorithm>
#include <array>
#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <complex>
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
int64_t g_shiftNs = 0;  // simulator time of the scenario time 0 (start of the first step of a run)
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
// lag of each phase of each step (--phaseLag; paper Section VI, validity of the runs)
//
// The lag of the real-time scheduler, wall clock - (origin + simulator time), is sampled every
// --phaseLagEveryUs of simulator time, at every PHY TX start and RX end of every node (where the
// lag builds up), and at the exact simulator times of the step start, of the start of phase 3 and of
// the end of the step (scenario time k TS, k TS + T1 + T2, (k + 1) TS). Per step k >= 0 it keeps the
// largest lag of phase 1, 2 and 3, the lag at the start of phase 3 and at the end of the step; the
// wall-clock duration of the simulated phase 3 is T3 + (lag_end - lag_p3), so a step whose
// |lag_end - lag_p3| exceeds --catchupUs (default 1 ms) is flagged as catch-up. With --abortLagUs > 0,
// a lag above it at scenario time >= 0 stops the emulator (exit status 3, "ABORT" line and
// summary.json "phase_lag.aborted"); ns-3's own HardLimit (--hardLimitMs) stays as a backstop, because
// it applies from the start of the emulator, when the reconnection of the VMs gives lags of several ms.

struct StepLag
{
    int64_t k = -1;
    float max[3] = {0, 0, 0};
    float p3 = -1, end = -1;
    uint32_t n = 0;
};

struct PhaseLag
{
    bool on = false;
    int64_t ts = 125000000, t1 = 20000000, t2 = 10000000, everyNs = 250000;
    int64_t abortNs = 0, catchupNs = 1000000;
    StepLag cur;
    std::vector<StepLag> done;
    bool aborted = false;
    int64_t abortStep = -1;
    float abortLagUs = 0;
    float runMax = 0;

    void Close()
    {
        if (cur.k >= 0)
        {
            done.push_back(cur);
        }
        cur = StepLag();
    }

    // one lag sample at the current simulator time; boundary 1: start of phase 3, 2: end of step k - 1
    void Sample(int boundary = 0)
    {
        int64_t s = NowNs() - g_shiftNs;
        if (!on || s < 0)
        {
            return;
        }
        float lag = float((WallNs() - (g_originNs + NowNs())) / 1000.0);
        int64_t k = s / ts;
        int64_t off = s - k * ts;
        if (boundary == 2 && cur.k == k - 1)
        {
            cur.end = lag;
            cur.max[2] = std::max(cur.max[2], lag);
        }
        if (k != cur.k)
        {
            Close();
            cur.k = k;
        }
        int ph = off < t1 ? 0 : (off < t1 + t2 ? 1 : 2);
        cur.max[ph] = std::max(cur.max[ph], lag);
        cur.n++;
        if (boundary == 1)
        {
            cur.p3 = lag;
        }
        runMax = std::max(runMax, lag);
        if (abortNs > 0 && lag * 1000.0 > abortNs && !aborted)
        {
            aborted = true;
            abortStep = k;
            abortLagUs = lag;
            std::cout << "ABORT lag " << lag << " us > " << abortNs / 1000 << " us at step " << k
                      << " phase " << ph + 1 << std::endl;
            Simulator::Stop();
        }
    }

    void Periodic()
    {
        Sample(0);
        Simulator::Schedule(NanoSeconds(everyNs), &PhaseLag::Periodic, this);
    }

    // boundary events of step k (scenario time k TS): end of step k - 1 / start of k, start of phase 3
    void Boundary(int64_t k)
    {
        Sample(2);
        Simulator::Schedule(NanoSeconds(t1 + t2), &PhaseLag::Phase3, this);
        Simulator::Schedule(NanoSeconds(ts), &PhaseLag::Boundary, this, k + 1);
    }

    void Phase3()
    {
        Sample(1);
    }

    void Start()
    {
        int64_t now = NowNs();
        int64_t k = now <= g_shiftNs ? 0 : (now - g_shiftNs) / ts + 1;
        Simulator::Schedule(NanoSeconds(g_shiftNs + k * ts - now), &PhaseLag::Boundary, this, k);
        Simulator::Schedule(NanoSeconds(everyNs), &PhaseLag::Periodic, this);
    }

    void Write(const std::string& path)
    {
        Close();
        FILE* f = fopen(path.c_str(), "w");
        if (!f)
        {
            return;
        }
        fprintf(f, "k,max1_us,max2_us,max3_us,lag_p3_us,lag_end_us,catchup,n\n");
        for (auto& r : done)
        {
            bool cu = r.p3 >= 0 && r.end >= 0 && std::fabs(r.end - r.p3) * 1000.0 > catchupNs;
            fprintf(f,
                    "%lld,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%u\n",
                    (long long)r.k,
                    r.max[0],
                    r.max[1],
                    r.max[2],
                    r.p3,
                    r.end,
                    cu ? 1 : 0,
                    r.n);
        }
        fclose(f);
    }
};

PhaseLag g_plag;

// ---------------------------------------------------------------------------------------------
// bytes per step and vessel at the AP (--stepBytes; offline subset replays of emu/replay_subsets.py)
//
// Every IPv4 data frame from vessel i to the AP that the AP receives without error counts once (the
// first reception of its MSDU; a retransmission after a lost ACK carries the same packet UID) in the
// step k = floor((t_end - shift) / TS) of the end t_end of its reception, in "phase 3" when
// t_end - shift - k TS >= P3OFF (= T1 + T2).

struct StepBytes
{
    bool on = false;
    int64_t ts = 125000000, p3off = 30000000;
    // (k, vessel) -> frames3, bytes3, frames, bytes
    std::map<std::pair<int64_t, int>, std::array<uint64_t, 4>> acc;
    std::vector<std::deque<uint32_t>> lastUids; // per vessel: UIDs of the latest receptions

    void Rx(const FrameInfo& fi, int64_t endNs)
    {
        if (!on || fi.kind != 0 || fi.ipLen == 0 || fi.src < 1 || fi.src > int(g_M) || fi.dst != 0)
        {
            return;
        }
        auto& q = lastUids[fi.src];
        if (std::find(q.begin(), q.end(), fi.uid) != q.end())
        {
            return;
        }
        q.push_back(fi.uid);
        if (q.size() > 64)
        {
            q.pop_front();
        }
        int64_t s = endNs - g_shiftNs;
        if (s < 0)
        {
            return;
        }
        int64_t k = s / ts;
        auto& a = acc[{k, fi.src}];
        if (s - k * ts >= p3off)
        {
            a[0]++;
            a[1] += fi.ipLen;
        }
        a[2]++;
        a[3] += fi.ipLen;
    }

    void Write(const std::string& path)
    {
        FILE* f = fopen(path.c_str(), "w");
        if (!f)
        {
            return;
        }
        fprintf(f, "k,i,frames3,bytes3,frames,bytes\n");
        for (auto& kv : acc)
        {
            fprintf(f,
                    "%lld,%d,%llu,%llu,%llu,%llu\n",
                    (long long)kv.first.first,
                    kv.first.second,
                    (unsigned long long)kv.second[0],
                    (unsigned long long)kv.second[1],
                    (unsigned long long)kv.second[2],
                    (unsigned long long)kv.second[3]);
        }
        fclose(f);
    }
};

StepBytes g_sb;

// ---------------------------------------------------------------------------------------------
// pcap of the AP node (--pcap): IPv4 packets between the AP node and the GS VM

struct ApPcap
{
    FILE* f = nullptr;
    uint32_t snap = 65535;
    uint64_t packets = 0, bytes = 0, errors = 0;

    bool Open(const std::string& path, uint32_t snaplen)
    {
        f = fopen(path.c_str(), "wb");
        if (!f)
        {
            return false;
        }
        // a 4 MiB stdio buffer: the writes run in the simulation thread (E0 runs only)
        setvbuf(f, nullptr, _IOFBF, 4 << 20);
        snap = snaplen;
        // pcap global header with nanosecond time stamps (magic a1b23c4d), version 2.4, LINKTYPE_RAW (101)
        uint32_t magic = 0xa1b23c4d, zone = 0, sigfigs = 0, link = 101;
        uint16_t vmaj = 2, vmin = 4;
        bool ok = fwrite(&magic, 4, 1, f) == 1 && fwrite(&vmaj, 2, 1, f) == 1 && fwrite(&vmin, 2, 1, f) == 1 &&
                  fwrite(&zone, 4, 1, f) == 1 && fwrite(&sigfigs, 4, 1, f) == 1 && fwrite(&snap, 4, 1, f) == 1 &&
                  fwrite(&link, 4, 1, f) == 1;
        if (!ok)
        {
            errors++;
        }
        return true;
    }

    void Write(Ptr<const Packet> p)
    {
        if (!f)
        {
            return;
        }
        uint32_t len = p->GetSize();
        uint32_t incl = std::min(len, snap);
        std::vector<uint8_t> buf(incl);
        p->CopyData(buf.data(), incl);
        int64_t ns = g_originNs + Simulator::Now().GetNanoSeconds();
        uint32_t rec[4] = {uint32_t(ns / 1000000000), uint32_t(ns % 1000000000), incl, len};
        if (fwrite(rec, sizeof rec, 1, f) != 1 || (incl && fwrite(buf.data(), incl, 1, f) != 1))
        {
            errors++;
        }
        packets++;
        bytes += len;
    }

    void Close()
    {
        if (f)
        {
            fclose(f);
            f = nullptr;
        }
    }
};

ApPcap g_pcap;

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
    if (g_plag.on)
    {
        g_plag.Sample();
    }
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
    if (g_plag.on)
    {
        g_plag.Sample();
    }
    if (node == 0 && ok && g_cnt)
    {
        g_cnt->ApRx(fi);
    }
    if (node == 0 && ok && g_sb.on)
    {
        g_sb.Rx(fi, now);
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

// Queue disc in front of the MAC queue of a Wi-Fi node (scenario "aqm"): the relay hands its packets to
// the traffic-control layer of the node, as the Linux stack hands them to the qdisc of the radio. The
// item carries the UP of the relay in the DS field that the select-queue callback of the Wi-Fi device
// reads (WifiHelper::SelectQueueByDSField: UP = DS field >> 5), so the AC of a packet does not change.
class RelayQueueItem : public QueueDiscItem
{
  public:
    RelayQueueItem(Ptr<Packet> p, const Address& addr, uint16_t protocol, uint8_t up)
        : QueueDiscItem(p, addr, protocol),
          m_up(up)
    {
    }

    void AddHeader() override
    {
    }

    bool Mark() override
    {
        return false;
    }

    bool GetUint8Value(Uint8Values field, uint8_t& value) const override
    {
        if (field == IP_DSFIELD)
        {
            value = uint8_t(m_up << 5);
            return true;
        }
        return false;
    }

  private:
    uint8_t m_up;
};

std::vector<Ptr<TrafficControlLayer>> g_tc; // node index -> traffic-control layer (null: no queue disc)

// send an IPv4 packet of the relay (or of a self-test source) on the Wi-Fi device of node h
bool
RelaySend(int h, Ptr<Packet> pkt, Mac48Address dst, uint8_t up)
{
    if (h < int(g_tc.size()) && g_tc[h])
    {
        g_tc[h]->Send(g_wifiDev[h], Create<RelayQueueItem>(pkt, dst, 0x0800, up));
        return true;
    }
    return g_wifiDev[h]->Send(pkt, dst, 0x0800);
}

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
    if (h == 0)
    {
        g_pcap.Write(pkt);
    }
    if (RelaySend(h, pkt, wdst, uint8_t(up)))
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
    if (h == 0 && proto == 0x0800)
    {
        g_pcap.Write(p);
    }
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
    RelaySend(h, full, g_wifiMac[0], 0);
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
    RelaySend(0, full, g_wifiMac[h], 0);
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
           << "\",\"center_mhz\":" << static_cast<double>(g_phy[n]->GetFrequency())
           << ",\"slot_us\":" << g_phy[n]->GetSlot().GetMicroSeconds()
           << ",\"sifs_us\":" << g_phy[n]->GetSifs().GetMicroSeconds()
           << ",\"be_aifsn\":" << int(be->Txop::GetAifsn()) << ",\"be_cwmin\":" << be->Txop::GetMinCw()
           << ",\"be_cwmax\":" << be->Txop::GetMaxCw() << ",\"vo_aifsn\":" << int(vo->Txop::GetAifsn())
           << ",\"vo_cwmin\":" << vo->Txop::GetMinCw() << ",\"vo_cwmax\":" << vo->Txop::GetMaxCw();
        auto rsm = g_wifiDev[n]->GetRemoteStationManager();
        os << ",\"basic0\":\"" << (rsm->GetNBasicModes() ? rsm->GetBasicMode(0).GetUniqueName() : std::string("none"))
           << "\",\"rts_threshold\":" << WIFI_MAX_RTS_THRESHOLD
           << ",\"be_queue_p\":" << be->GetWifiMacQueue()->GetMaxSize().GetValue()
           << ",\"be_queue_delay_ms\":" << be->GetWifiMacQueue()->GetMaxDelay().GetMilliSeconds()
           << ",\"qdisc\":" << (n < g_tc.size() && g_tc[n] ? "\"mq+codel\"" : "null") << "}";
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

// ---------------------------------------------------------------------------------------------
// channel models of the paper scenarios

double
ScenarioNowS()
{
    return double(NowNs() - g_shiftNs) / 1e9;
}

using MobKey = std::pair<const MobilityModel*, const MobilityModel*>;

// Matrix loss with three per-pair extensions (channel.model "matrix"):
//   loss    [[a, b, dB], ...]                      constant loss of the pair (symmetric)
//   walk    {"dt_s": D, "pairs": [[a, b, [dB...]]]} loss of the pair at scenario time t: linear
//           interpolation of the series at t / D (S6; the first value before 0, the last after the end)
//   fading  [[a, b, m], ...]                       Nakagami-m power fading per frame and receiver on top
//           of the loss of the pair (unit mean power gain Gamma(m, 1/m); S9, links at the detection limit)
class FdtMatrixLoss : public PropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::FdtMatrixLoss")
                                .SetParent<PropagationLossModel>()
                                .SetGroupName("Propagation")
                                .AddConstructor<FdtMatrixLoss>();
        return tid;
    }

    // the generator exists only with fading pairs, so that the other scenarios keep their random streams
    void EnableFading(int64_t stream)
    {
        m_gamma = CreateObject<GammaRandomVariable>();
        m_gamma->SetStream(stream);
    }

    double m_default = 70;
    std::map<MobKey, double> m_loss;
    double m_walkDt = 1.0;
    std::map<MobKey, std::vector<double>> m_walk;
    std::map<MobKey, double> m_fadeM;
    // timed entries [a, b, loss, t0, t1] of channel.loss: the loss of the pair during [t0, t1) of scenario
    // time (E0 outages), ahead of the walk and of the fixed loss of the pair
    std::map<MobKey, std::vector<std::array<double, 3>>> m_window;

    double LossOf(const MobilityModel* a, const MobilityModel* b) const
    {
        auto o = m_window.find({a, b});
        if (o != m_window.end())
        {
            double t = ScenarioNowS();
            for (const auto& e : o->second)
            {
                if (t >= e[0] && t < e[1])
                {
                    return e[2];
                }
            }
        }
        auto w = m_walk.find({a, b});
        if (w != m_walk.end() && !w->second.empty())
        {
            const auto& v = w->second;
            double x = ScenarioNowS() / m_walkDt;
            if (x <= 0)
            {
                return v.front();
            }
            size_t i = size_t(x);
            if (i + 1 >= v.size())
            {
                return v.back();
            }
            double f = x - double(i);
            return v[i] + f * (v[i + 1] - v[i]);
        }
        auto it = m_loss.find({a, b});
        return it == m_loss.end() ? m_default : it->second;
    }

  private:
    double DoCalcRxPower(double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override
    {
        double rx = txPowerDbm - LossOf(PeekPointer(a), PeekPointer(b));
        auto f = m_fadeM.find({PeekPointer(a), PeekPointer(b)});
        if (m_gamma && f != m_fadeM.end())
        {
            double m = f->second;
            double g = m_gamma->GetValue(m, 1.0 / m);
            rx += 10.0 * std::log10(std::max(g, 1e-30));
        }
        return rx;
    }

    int64_t DoAssignStreams(int64_t stream) override
    {
        if (!m_gamma)
        {
            return 0;
        }
        m_gamma->SetStream(stream);
        return 1;
    }

    Ptr<GammaRandomVariable> m_gamma;
};

// Two-ray model over the sea surface with moving antennas (channel.model "sea"; scenario S7).
//
// Geometry (flat surface at z = 0, heights above the mean sea level): direct ray of length
// r1 = sqrt(d^2 + (ha - hb)^2), ray reflected at the specular point of length r2 = sqrt(d^2 + (ha + hb)^2),
// grazing angle psi = atan((ha + hb) / d). Received power
//   Pr = Pt (lambda / 4 pi)^2 | Fa1 Fb1 e^{-j k r1} / r1 + Gamma(psi) rho(psi) Fa2 Fb2 e^{-j k r2} / r2 |^2
// with the Fresnel reflection coefficient Gamma of sea water (complex permittivity
// eps_r - j 60 lambda sigma; vertical or horizontal polarization), the roughness factor of Miller, Brown
// and Vegh, rho = exp(-2 g^2) I0(2 g^2), g = 2 pi sigma_h sin(psi) / lambda, and the field patterns F of a
// half-wave dipole, F(theta) = cos(pi/2 cos theta) / sin theta (theta: angle between the ray and the
// dipole axis; F = 1 broadside, so the link budget of a horizontal ray is that of the 0 dBi antennas
// of the other scenarios). Motion of node n (channel.motion.<name>): antenna height
// z + A sin(2 pi t / T + phi) (heave), roll angle R sin(2 pi t / T + phi_r) about the heading of the node
// (the direction of its velocity; the last heading when it stops), which tilts the vertical dipole in
// the athwartship plane. t = scenario time. The AP (no motion entry) has a fixed vertical dipole.
// Sources of the model and of its parameters: W. Wang et al., "Wireless channel models for maritime
// communications", IEEE Access 6 (2018) (two-ray over the sea); K. Yang et al., "Long-distance
// propagation measurements of mobile radio channel over sea at 2 GHz", IEEE VTC-Fall 2011 (two-ray
// interference pattern measured over sea); ITU-R P.527 (permittivity and conductivity of sea water);
// D. E. Miller, G. S. Brown, L. Vegh, Radio Science 19(3) (1984) (roughness factor).
class SeaTwoRayLoss : public PropagationLossModel
{
  public:
    static TypeId GetTypeId()
    {
        static TypeId tid = TypeId("ns3::FdtSeaTwoRayLoss")
                                .SetParent<PropagationLossModel>()
                                .SetGroupName("Propagation")
                                .AddConstructor<SeaTwoRayLoss>();
        return tid;
    }

    struct Motion
    {
        double heave = 0, period = 1, phase = 0, roll = 0, rollPhase = 0; // m, s, rad, rad, rad
    };

    double m_lambda = 299792458.0 / 2.412e9;
    std::complex<double> m_eps{70.0, 0.0};
    bool m_vertical = true;
    double m_roughSigma = 0.0;
    double m_minDistance = 1.0;
    bool m_dipole = true;
    std::map<const MobilityModel*, Motion> m_motion;
    mutable std::map<const MobilityModel*, double> m_heading;

    void SetMaterial(double freqHz, double epsR, double sigma)
    {
        m_lambda = 299792458.0 / freqHz;
        m_eps = std::complex<double>(epsR, -60.0 * m_lambda * sigma);
    }

    double Height(const MobilityModel* m, double z, double t) const
    {
        auto it = m_motion.find(m);
        if (it == m_motion.end())
        {
            return z;
        }
        const Motion& mo = it->second;
        return z + mo.heave * std::sin(2 * M_PI * t / mo.period + mo.phase);
    }

    double Roll(const MobilityModel* m, double t) const
    {
        auto it = m_motion.find(m);
        if (it == m_motion.end())
        {
            return 0.0;
        }
        const Motion& mo = it->second;
        return mo.roll * std::sin(2 * M_PI * t / mo.period + mo.rollPhase);
    }

    double Heading(Ptr<MobilityModel> m) const
    {
        Vector v = m->GetVelocity();
        if (v.x * v.x + v.y * v.y > 1e-4)
        {
            double h = std::atan2(v.y, v.x);
            m_heading[PeekPointer(m)] = h;
            return h;
        }
        auto it = m_heading.find(PeekPointer(m));
        return it == m_heading.end() ? 0.0 : it->second;
    }

    // field pattern of the dipole of node m for a ray of elevation e and world azimuth az
    double Pattern(Ptr<MobilityModel> m, double e, double az, double t) const
    {
        if (!m_dipole)
        {
            return 1.0;
        }
        double r = Roll(PeekPointer(m), t);
        double rel = az - Heading(m);
        // dipole axis after a roll r about the heading: (0, -sin r, cos r) in the frame (forward, port, up)
        double c = -std::cos(e) * std::sin(rel) * std::sin(r) + std::sin(e) * std::cos(r);
        c = std::clamp(c, -1.0, 1.0);
        double s = std::sqrt(1.0 - c * c);
        if (s < 1e-9)
        {
            return 0.0;
        }
        return std::fabs(std::cos(M_PI / 2 * c)) / s;
    }

    std::complex<double> Reflection(double psi) const
    {
        double sp = std::sin(psi), cp = std::cos(psi);
        std::complex<double> root = std::sqrt(m_eps - cp * cp);
        if (m_vertical)
        {
            return (m_eps * sp - root) / (m_eps * sp + root);
        }
        return (sp - root) / (sp + root);
    }

    double Roughness(double psi) const
    {
        if (m_roughSigma <= 0)
        {
            return 1.0;
        }
        double g = 2 * M_PI * m_roughSigma * std::sin(psi) / m_lambda;
        double x = 2 * g * g;
        return std::exp(-x) * std::cyl_bessel_i(0.0, x);
    }

    // power gain of the link (linear), for the tests of the model
    double Gain(Ptr<MobilityModel> a, Ptr<MobilityModel> b) const
    {
        double t = ScenarioNowS();
        Vector pa = a->GetPosition(), pb = b->GetPosition();
        double ha = std::max(0.05, Height(PeekPointer(a), pa.z, t));
        double hb = std::max(0.05, Height(PeekPointer(b), pb.z, t));
        double dx = pb.x - pa.x, dy = pb.y - pa.y;
        double d = std::max(m_minDistance, std::sqrt(dx * dx + dy * dy));
        double azAB = std::atan2(dy, dx), azBA = std::atan2(-dy, -dx);
        double r1 = std::sqrt(d * d + (ha - hb) * (ha - hb));
        double r2 = std::sqrt(d * d + (ha + hb) * (ha + hb));
        double psi = std::atan2(ha + hb, d);
        double eAB = std::atan2(hb - ha, d);
        double k = 2 * M_PI / m_lambda;
        double fa1 = Pattern(a, eAB, azAB, t), fb1 = Pattern(b, -eAB, azBA, t);
        double fa2 = Pattern(a, -psi, azAB, t), fb2 = Pattern(b, -psi, azBA, t);
        std::complex<double> e1 = std::polar(fa1 * fb1 / r1, -k * r1);
        std::complex<double> e2 = Reflection(psi) * Roughness(psi) * std::polar(fa2 * fb2 / r2, -k * r2);
        double amp = std::abs(e1 + e2) * m_lambda / (4 * M_PI);
        return amp * amp;
    }

  private:
    double DoCalcRxPower(double txPowerDbm, Ptr<MobilityModel> a, Ptr<MobilityModel> b) const override
    {
        double g = Gain(a, b);
        return g > 0 ? txPowerDbm + 10.0 * std::log10(g) : -1000.0;
    }

    int64_t DoAssignStreams(int64_t) override
    {
        return 0;
    }
};

struct LossProbe
{
    Ptr<PropagationLossModel> model;
    double txDbm = 20;
    int64_t dtNs = 100000000;
    std::vector<std::pair<int, int>> pairs;
    std::vector<Ptr<MobilityModel>> mob;
    FILE* f = nullptr;

    void Tick()
    {
        double t = ScenarioNowS();
        for (auto& p : pairs)
        {
            fprintf(f,
                    "%.4f,%s,%s,%.3f\n",
                    t,
                    g_names[p.first].c_str(),
                    g_names[p.second].c_str(),
                    model->CalcRxPower(txDbm, mob[p.first], mob[p.second]));
        }
        Simulator::Schedule(NanoSeconds(dtNs), &LossProbe::Tick, this);
    }
};

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
    double shiftS = -1;           // offline: scenario time 0 at this simulator time (s), instead of runStartUs
    std::string selftestSet;      // vessels with a self-test source (default: all)
    std::string stepBytesSpec;    // "TS_US:P3OFF_US": stepbytes.csv
    std::string phaseLagSpec;     // "TS_US:T1_US:T2_US": steplag.csv
    double phaseLagEveryUs = 250, abortLagUs = 0, catchupUs = 1000;
    std::string probeLoss;        // "A-B[,C-D...]:DT_MS": lossprobe.csv (diagnostics only)
    bool pcap = false;            // ap.pcap
    uint32_t pcapSnap = 65535;

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
    cmd.AddValue("shiftS",
                 "simulator time (s) of the scenario time 0 (offline replays; overrides runStartUs; < 0: off)",
                 shiftS);
    cmd.AddValue("selftestSet", "comma-separated vessel indices with a self-test source (default: every vessel)",
                 selftestSet);
    cmd.AddValue("stepBytes",
                 "TS_US:P3OFF_US: write OUTDIR/stepbytes.csv (per step and vessel, unique IP packets received "
                 "by the AP, in phase 3 = [k TS + P3OFF, (k + 1) TS) of scenario time and in the step)",
                 stepBytesSpec);
    cmd.AddValue("phaseLag",
                 "TS_US:T1_US:T2_US: write OUTDIR/steplag.csv (largest lag of each phase of each step, real time only)",
                 phaseLagSpec);
    cmd.AddValue("phaseLagEveryUs", "period of the extra lag samples of --phaseLag (us of simulator time)",
                 phaseLagEveryUs);
    cmd.AddValue("abortLagUs",
                 "stop the emulator (exit status 3) when the lag exceeds this value at scenario time >= 0 "
                 "(needs --phaseLag; 0: record only)",
                 abortLagUs);
    cmd.AddValue("catchupUs", "catch-up flag of steplag.csv: |lag_end - lag_p3| above this value (us)", catchupUs);
    cmd.AddValue("probeLoss",
                 "A-B[,C-D...]:DT_MS: write OUTDIR/lossprobe.csv, the received power of each pair every DT_MS "
                 "(diagnostics: with fading models the probe draws from the fading streams and changes the run)",
                 probeLoss);
    cmd.AddValue("pcap",
                 "write OUTDIR/ap.pcap: the IPv4 packets between the AP node and the GS (LINKTYPE_RAW, host-clock "
                 "time stamps in ns)",
                 pcap);
    cmd.AddValue("pcapSnap", "bytes kept of each packet in ap.pcap", pcapSnap);
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
    auto mobOf = [&](const std::string& name) {
        return PeekPointer(all.Get(idx(name))->GetObject<MobilityModel>());
    };
    if (model == "matrix")
    {
        Ptr<FdtMatrixLoss> loss = CreateObject<FdtMatrixLoss>();
        loss->m_default = ch["default_loss_db"].Num(70);
        const mj::Value& L = ch["loss"];
        for (size_t k = 0; k < L.size(); k++)
        {
            auto a = mobOf(L[k][0].Str()), b = mobOf(L[k][1].Str());
            if (L[k].size() >= 5)
            {
                // timed entry [a, b, loss, t0, t1]
                double t0 = L[k][3].Num(), t1 = L[k][4].Num();
                NS_ABORT_MSG_IF(!(t1 > t0), "channel.loss: a timed entry needs t1 > t0");
                std::array<double, 3> e = {t0, t1, L[k][2].Num()};
                loss->m_window[{a, b}].push_back(e);
                if (a != b)
                {
                    loss->m_window[{b, a}].push_back(e);
                }
                continue;
            }
            loss->m_loss[{a, b}] = loss->m_loss[{b, a}] = L[k][2].Num();
        }
        const mj::Value& W = ch["walk"];
        if (W.type == mj::Value::OBJ)
        {
            loss->m_walkDt = W["dt_s"].Num(1.0);
            NS_ABORT_MSG_IF(loss->m_walkDt <= 0, "channel.walk.dt_s must be > 0");
            const mj::Value& P = W["pairs"];
            for (size_t k = 0; k < P.size(); k++)
            {
                std::vector<double> v;
                for (size_t j = 0; j < P[k][2].size(); j++)
                {
                    v.push_back(P[k][2][j].Num());
                }
                auto a = mobOf(P[k][0].Str()), b = mobOf(P[k][1].Str());
                loss->m_walk[{a, b}] = loss->m_walk[{b, a}] = v;
            }
        }
        const mj::Value& F = ch["fading"];
        for (size_t k = 0; k < F.size(); k++)
        {
            auto a = mobOf(F[k][0].Str()), b = mobOf(F[k][1].Str());
            NS_ABORT_MSG_IF(F[k][2].Num() <= 0, "channel.fading: m must be > 0");
            loss->m_fadeM[{a, b}] = loss->m_fadeM[{b, a}] = F[k][2].Num();
        }
        if (!loss->m_fadeM.empty())
        {
            loss->EnableFading(int64_t(sc["seed"].Num(1)) * 100 + 7);
        }
        lossModel = loss;
    }
    else if (model == "sea")
    {
        Ptr<SeaTwoRayLoss> sea = CreateObject<SeaTwoRayLoss>();
        sea->SetMaterial(ch["frequency_hz"].Num(2.412e9), ch["eps_r"].Num(70), ch["sigma_s_m"].Num(5));
        std::string pol = ch["polarization"].Str("V");
        NS_ABORT_MSG_IF(pol != "V" && pol != "H", "channel.polarization must be V or H");
        sea->m_vertical = pol == "V";
        sea->m_roughSigma = ch["rough_sigma_m"].Num(0);
        sea->m_minDistance = ch["min_distance_m"].Num(1);
        sea->m_dipole = ch["antenna"].Str("dipole") == "dipole";
        const mj::Value& mo = ch["motion"];
        if (mo.type == mj::Value::OBJ)
        {
            for (auto& kv : mo.o)
            {
                SeaTwoRayLoss::Motion m;
                m.heave = kv.second["heave_m"].Num(0);
                m.period = kv.second["period_s"].Num(1);
                m.phase = kv.second["phase_rad"].Num(0);
                m.roll = kv.second["roll_deg"].Num(0) * M_PI / 180.0;
                m.rollPhase = kv.second["roll_phase_rad"].Num(0);
                NS_ABORT_MSG_IF(m.period <= 0, "channel.motion." << kv.first << ".period_s must be > 0");
                sea->m_motion[mobOf(kv.first)] = m;
            }
        }
        lossModel = sea;
        const mj::Value& nk = ch["nakagami"];
        if (nk.type == mj::Value::OBJ)
        {
            Ptr<NakagamiPropagationLossModel> nak = CreateObject<NakagamiPropagationLossModel>();
            nak->SetAttribute("m0", DoubleValue(nk["m0"].Num(5)));
            nak->SetAttribute("m1", DoubleValue(nk["m1"].Num(5)));
            nak->SetAttribute("m2", DoubleValue(nk["m2"].Num(5)));
            nak->SetAttribute("Distance1", DoubleValue(nk["d1"].Num(80)));
            nak->SetAttribute("Distance2", DoubleValue(nk["d2"].Num(200)));
            nak->AssignStreams(int64_t(sc["seed"].Num(1)) * 100);
            sea->SetNext(nak);
        }
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
        NS_FATAL_ERROR("channel.model must be matrix, tworay or sea");
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
    // operating channel: "channel_number" (20 MHz; 2.4 GHz band for 11g, 5 GHz for 11a), else the ns-3
    // default of the standard (11g: channel 1, 11a: channel 36)
    if (sc.Has("channel_number"))
    {
        int chn = int(sc["channel_number"].Num());
        std::string band = standard == "11a" ? "BAND_5GHZ" : "BAND_2_4GHZ";
        NS_ABORT_MSG_IF(standard == "11g" && (chn < 1 || chn > 13), "channel_number: 1..13 for 11g");
        phy.Set("ChannelSettings", StringValue("{" + std::to_string(chn) + ", 20, " + band + ", 0}"));
    }
    double txp = sc["tx_power_dbm"].Num(20);
    phy.Set("TxPowerStart", DoubleValue(txp));
    phy.Set("TxPowerEnd", DoubleValue(txp));
    if (sc.Has("cca_ed_dbm"))
    {
        phy.Set("CcaEdThreshold", DoubleValue(sc["cca_ed_dbm"].Num()));
    }
    if (sc.Has("rx_noise_figure_db"))
    {
        phy.Set("RxNoiseFigure", DoubleValue(sc["rx_noise_figure_db"].Num()));
    }
    // preamble detection (ns-3 ThresholdPreambleDetectionModel: RSSI >= MinimumRssi and SNR >= Threshold)
    if (sc.Has("preamble_min_rssi_dbm") || sc.Has("preamble_snr_db"))
    {
        phy.SetPreambleDetectionModel("ns3::ThresholdPreambleDetectionModel",
                                      "MinimumRssi",
                                      DoubleValue(sc["preamble_min_rssi_dbm"].Num(-82)),
                                      "Threshold",
                                      DoubleValue(sc["preamble_snr_db"].Num(4)));
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
        mac.SetType("ns3::ApWifiMac",
                    "Ssid",
                    SsidValue(ssid2),
                    "QosSupported",
                    BooleanValue(true),
                    "BeaconInterval",
                    TimeValue(MicroSeconds(1024 * int(ext["beacon_tu"].Num(100)))));
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
        // the external BSS has its own basic rate set ("external_bss.basic_modes", default that of the
        // fleet BSS): ["DsssRate1Mbps"] makes its beacons 1 Mb/s DSSS frames (S2, S9; needs phy yans,
        // because SpectrumWifiPhy cannot send DSSS in a 20 MHz channel)
        std::vector<std::string> basicExt = basic;
        const mj::Value& be = ext["basic_modes"];
        if (be.type == mj::Value::ARR)
        {
            basicExt.clear();
            for (size_t k = 0; k < be.size(); k++)
            {
                basicExt.push_back(be[k].Str());
            }
        }
        for (auto& m : basicExt)
        {
            NS_ABORT_MSG_IF(hasExt && phyModel == "spectrum" && m.rfind("Dsss", 0) == 0,
                            "external_bss.basic_modes " << m << " (DSSS) needs phy = yans");
        }
        NetDeviceContainer fleetDevs(apDevs, staDevs);
        for (uint32_t k = 0; k < fleetDevs.GetN() + extDevs.GetN(); k++)
        {
            bool isExt = k >= fleetDevs.GetN();
            Ptr<NetDevice> d = isExt ? extDevs.Get(k - fleetDevs.GetN()) : fleetDevs.Get(k);
            auto rsm = DynamicCast<WifiNetDevice>(d)->GetRemoteStationManager();
            for (auto& m : (isExt ? basicExt : basic))
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

    // R7 guards: RTS/CTS off (threshold above any PSDU) and no A-MSDU / A-MPDU on every MAC (non-HT
    // 802.11g aggregates nothing; the attributes make it explicit)
    for (size_t n = 0; n < nWifi; n++)
    {
        g_wifiDev[n]->GetRemoteStationManager()->SetAttribute("RtsCtsThreshold",
                                                              UintegerValue(WIFI_MAX_RTS_THRESHOLD));
        Ptr<WifiMac> m = g_wifiDev[n]->GetMac();
        for (const char* ac : {"BE", "BK", "VI", "VO"})
        {
            m->SetAttribute(std::string(ac) + "_MaxAmsduSize", UintegerValue(0));
            m->SetAttribute(std::string(ac) + "_MaxAmpduSize", UintegerValue(0));
        }
    }

    // MAC queues per node: "mac_queue" is the default of every node; "fleet_queue" overrides it for the
    // AP and the vessels, "station_queue" for the vessels (e.g. the shallow device queue under CoDel)
    auto setQueue = [&](size_t n, const mj::Value& qv) {
        Ptr<WifiMac> m = g_wifiDev[n]->GetMac();
        for (AcIndex ac : {AC_BE, AC_BK, AC_VI, AC_VO})
        {
            Ptr<WifiMacQueue> wq = m->GetQosTxop(ac)->GetWifiMacQueue();
            if (qv.Has("packets"))
            {
                wq->SetMaxSize(QueueSize(std::to_string(int(qv["packets"].Num())) + "p"));
            }
            if (qv.Has("max_delay_ms"))
            {
                wq->SetAttribute("MaxDelay", TimeValue(MilliSeconds(qv["max_delay_ms"].Num())));
            }
        }
    };
    if (sc["fleet_queue"].type == mj::Value::OBJ)
    {
        for (size_t n = 0; n <= g_M; n++)
        {
            setQueue(n, sc["fleet_queue"]);
        }
    }
    if (sc["station_queue"].type == mj::Value::OBJ)
    {
        for (size_t n = 1; n <= g_M; n++)
        {
            setQueue(n, sc["station_queue"]);
        }
    }
    // queue disc above the MAC queues ("aqm"): mq root with one CoDel per access category (the layout of
    // a Linux qdisc on a 4-queue Wi-Fi device), on the vessels ("nodes": "stations") or on the AP too
    // ("fleet"); the relay hands the packets of these nodes to their traffic-control layer
    g_tc.assign(nWifi, nullptr);
    std::vector<Ptr<QueueDisc>> qdRoot(nWifi);
    const mj::Value& aqm = sc["aqm"];
    std::string aqmType = aqm["type"].Str("none");
    NS_ABORT_MSG_IF(aqmType != "none" && aqmType != "codel", "aqm.type must be none or codel");
    if (aqmType == "codel")
    {
        std::string where = aqm["nodes"].Str("stations");
        NS_ABORT_MSG_IF(where != "stations" && where != "fleet", "aqm.nodes must be stations or fleet");
        for (size_t n = (where == "fleet" ? 0 : 1); n <= g_M; n++)
        {
            Ptr<Node> node = g_wifiDev[n]->GetNode();
            Ptr<TrafficControlLayer> tc = CreateObject<TrafficControlLayer>();
            node->AggregateObject(tc);
            TrafficControlHelper tch;
            uint16_t handle = tch.SetRootQueueDisc("ns3::MqQueueDisc");
            TrafficControlHelper::ClassIdList cls = tch.AddQueueDiscClasses(handle, 4, "ns3::QueueDiscClass");
            tch.AddChildQueueDiscs(handle,
                                   cls,
                                   "ns3::CoDelQueueDisc",
                                   "MaxSize",
                                   QueueSizeValue(QueueSize(std::to_string(int(aqm["limit_packets"].Num(1000))) + "p")),
                                   "Target",
                                   TimeValue(MicroSeconds(int64_t(aqm["target_ms"].Num(5) * 1000))),
                                   "Interval",
                                   TimeValue(MicroSeconds(int64_t(aqm["interval_ms"].Num(100) * 1000))));
            QueueDiscContainer qdc = tch.Install(g_wifiDev[n]);
            g_tc[n] = tc;
            qdRoot[n] = qdc.Get(0);
        }
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
    std::vector<bool> stOn(g_M + 1, selftestSet.empty());
    if (!selftestSet.empty())
    {
        size_t p0 = 0;
        while (p0 < selftestSet.size())
        {
            size_t p1 = selftestSet.find(',', p0);
            if (p1 == std::string::npos)
            {
                p1 = selftestSet.size();
            }
            int i = std::stoi(selftestSet.substr(p0, p1 - p0));
            NS_ABORT_MSG_IF(i < 1 || i > int(g_M), "--selftestSet: vessel index " << i << " outside 1.." << g_M);
            stOn[i] = true;
            p0 = p1 + 1;
        }
    }
    if (!selftest.empty())
    {
        auto comma = selftest.find(',');
        double rbps = std::stod(selftest.substr(0, comma));
        uint32_t size = uint32_t(std::stoul(selftest.substr(comma + 1)));
        for (uint32_t i = 1; i <= g_M; i++)
        {
            if (stOn[i])
            {
                Simulator::Schedule(Seconds(1.0 + 0.001 * i), &SelfTestSend, int(i), size * 8.0 / rbps, size);
            }
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
    if (shiftS >= 0)
    {
        shift = shiftS;
    }
    g_shiftNs = int64_t(std::llround(shift * 1e9));
    auto parseUs = [](const std::string& spec, const char* opt, size_t nmin, size_t nmax) {
        std::vector<int64_t> v;
        size_t p0 = 0;
        while (p0 <= spec.size())
        {
            size_t p1 = spec.find(':', p0);
            if (p1 == std::string::npos)
            {
                p1 = spec.size();
            }
            v.push_back(int64_t(std::llround(std::stod(spec.substr(p0, p1 - p0)) * 1e3)));
            p0 = p1 + 1;
        }
        NS_ABORT_MSG_IF(v.size() < nmin || v.size() > nmax, "bad value of --" << opt << ": " << spec);
        return v;
    };
    if (!stepBytesSpec.empty())
    {
        auto v = parseUs(stepBytesSpec, "stepBytes", 2, 2);
        NS_ABORT_MSG_IF(v[0] <= 0 || v[1] < 0 || v[1] >= v[0], "--stepBytes=TS_US:P3OFF_US with 0 <= P3OFF < TS");
        g_sb.on = true;
        g_sb.ts = v[0];
        g_sb.p3off = v[1];
        g_sb.lastUids.assign(g_M + 1, {});
    }
    if (pcap)
    {
        NS_ABORT_MSG_IF(!g_pcap.Open(outdir + "/ap.pcap", pcapSnap), "cannot write " << outdir << "/ap.pcap");
    }
    if (!phaseLagSpec.empty() && g_realtime)
    {
        auto v = parseUs(phaseLagSpec, "phaseLag", 3, 3);
        NS_ABORT_MSG_IF(v[0] <= 0 || v[1] < 0 || v[2] < 0 || v[1] + v[2] >= v[0],
                        "--phaseLag=TS_US:T1_US:T2_US with T1 + T2 < TS");
        g_plag.on = true;
        g_plag.ts = v[0];
        g_plag.t1 = v[1];
        g_plag.t2 = v[2];
        g_plag.everyNs = std::max<int64_t>(10000, int64_t(phaseLagEveryUs * 1e3));
        g_plag.abortNs = int64_t(abortLagUs * 1e3);
        g_plag.catchupNs = int64_t(catchupUs * 1e3);
        g_plag.Start();
    }
    NS_ABORT_MSG_IF(abortLagUs > 0 && !g_plag.on, "--abortLagUs needs --phaseLag and real time");
    LossProbe probe;
    if (!probeLoss.empty())
    {
        auto colon = probeLoss.rfind(':');
        NS_ABORT_MSG_IF(colon == std::string::npos, "--probeLoss=A-B[,C-D...]:DT_MS");
        probe.model = lossModel;
        probe.txDbm = txp;
        probe.dtNs = int64_t(std::stod(probeLoss.substr(colon + 1)) * 1e6);
        NS_ABORT_MSG_IF(probe.dtNs <= 0, "--probeLoss: DT_MS > 0");
        for (size_t n = 0; n < g_names.size(); n++)
        {
            probe.mob.push_back(all.Get(n)->GetObject<MobilityModel>());
        }
        std::string list = probeLoss.substr(0, colon);
        size_t p0 = 0;
        while (p0 < list.size())
        {
            size_t p1 = list.find(',', p0);
            if (p1 == std::string::npos)
            {
                p1 = list.size();
            }
            std::string pr = list.substr(p0, p1 - p0);
            auto dash = pr.find('-');
            NS_ABORT_MSG_IF(dash == std::string::npos, "--probeLoss: pair A-B");
            probe.pairs.emplace_back(idx(pr.substr(0, dash)), idx(pr.substr(dash + 1)));
            p0 = p1 + 1;
        }
        probe.f = fopen((outdir + "/lossprobe.csv").c_str(), "w");
        NS_ABORT_MSG_IF(!probe.f, "cannot write lossprobe.csv");
        fprintf(probe.f, "t_s,a,b,rx_dbm\n");
        Simulator::Schedule(NanoSeconds(std::max<int64_t>(0, g_shiftNs)), &LossProbe::Tick, &probe);
    }
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
    if (g_plag.on)
    {
        g_plag.Write(outdir + "/steplag.csv");
    }
    if (g_sb.on)
    {
        g_sb.Write(outdir + "/stepbytes.csv");
    }
    g_pcap.Close();
    if (probe.f)
    {
        fclose(probe.f);
        probe.f = nullptr;
    }
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
       << "\"},\"phase_lag\":{\"on\":" << (g_plag.on ? "true" : "false") << ",\"steps\":" << g_plag.done.size()
       << ",\"max_us\":" << g_plag.runMax << ",\"abort_us\":" << g_plag.abortNs / 1000
       << ",\"aborted\":" << (g_plag.aborted ? "true" : "false") << ",\"abort_step\":" << g_plag.abortStep
       << ",\"abort_lag_us\":" << g_plag.abortLagUs << "},\"step_bytes\":" << (g_sb.on ? "true" : "false")
       << ",\"pcap\":{\"on\":" << (pcap ? "true" : "false") << ",\"packets\":" << g_pcap.packets
       << ",\"bytes\":" << g_pcap.bytes << ",\"errors\":" << g_pcap.errors << "}"
       << ",\"qdisc\":{";
    {
        bool first = true;
        for (size_t n = 0; n < qdRoot.size(); n++)
        {
            if (!qdRoot[n])
            {
                continue;
            }
            uint64_t drop = 0, sent = 0;
            for (size_t c = 0; c < qdRoot[n]->GetNQueueDiscClasses(); c++)
            {
                const QueueDisc::Stats& st = qdRoot[n]->GetQueueDiscClass(c)->GetQueueDisc()->GetStats();
                drop += st.nTotalDroppedPackets;
                sent += st.nTotalSentPackets;
            }
            sm << (first ? "" : ",") << "\"" << g_names[n] << "\":{\"sent\":" << sent << ",\"dropped\":" << drop << "}";
            first = false;
        }
    }
    sm << "},\"non_ip_from_vm\":" << g_nonIp << ",\"hosts\":[";
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
    return g_plag.aborted ? 3 : 0;
}
