# Full-software test on a simulated Wi-Fi network

The whole stack runs over an emulated 802.11 channel: `mac80211_hwsim` gives 4 radios to the
kernel, `wmediumd` serializes their frames on one medium with the airtime of 802.11 DCF (DIFS,
backoff, frame at the rate of the attempt, SIFS, ACK), and each node runs in its own network
namespace with the real programs: hostapd and wpa_supplicant, the brokers with the bridges and the
vessel regulator, the DTI programs, the synthetic load, the AS, the VE collector, and the channel
monitor reading the survey and the station counters of the GS radio through nl80211.

| Namespace | Radio | Role | Address |
|---|---|---|---|
| `gs` | wlan0 | GS radio (access point) and GS PC | 10.0.0.10 |
| `b1`, `b2` | wlan1, wlan2 | vessel radio and RP4 | 10.0.0.99, 10.0.0.98 |
| `inj` | wlan3 | injector (monitor interface), software stand-in of TX99 | none |

Channel 6, no HT, no WMM, one legacy rate of 24 Mbit/s.

## Patches

| File | Program | Change |
|---|---|---|
| `patch_hwsim.py` | `mac80211_hwsim` (Linux 6.12) | the survey of the channel in use reports cumulative time and the airtime of every frame that the radio hears or sends; upstream reports a fixed 12.5 % busy time |
| `patch_wmediumd.py` | `wmediumd` (commit 717e5d7) | each delivered frame carries the rate of the attempt that succeeded; upstream reports 2 Mbit/s for every frame |

## Use

RHEL kernels ship no `mac80211_hwsim`, so the test runs in a Debian 13 VM (KVM):

```bash
tests/hwsim/vm.sh up                       # on the GS PC; ssh on 127.0.0.1:2222
tests/hwsim/vm.sh sync
tests/hwsim/vm.sh ssh 'mkdir -p ~/dist && cat > ~/dist/mosquitto-2.1.2.tar.gz' < mosquitto-2.1.2.tar.gz
tests/hwsim/vm.sh ssh 'cat > ~/dist/cJSON-1.7.19.tar.gz' < cJSON-1.7.19.tar.gz
tests/hwsim/vm.sh ssh 'cd ~/fleet-dt && tests/hwsim/provision.sh ~/dist'
tests/hwsim/vm.sh ssh 'cd ~/fleet-dt && tests/hwsim/run.sh ~/runs/hwsim-1'                  # 90 s, burst 30..60 s at duty 0.7
tests/hwsim/vm.sh ssh 'cd ~/fleet-dt && tests/hwsim/run.sh ~/runs/x --seconds 120 --plan "20 20 0.5; 60 30 0.8"'
```

`run.sh` writes the DTA, the decisions of the AS, the counter samples of the channel monitor, the
metrics of Section VI and `summary.json` (clean steps and burst steps apart), and checks Algorithm 1
and r ≤ d on every G(t), the membership in M(t), the drop of y(t), the admission of the video topic
in the clean steps and its rejection in the bursts, δ ≥ 0.95 and μ ≤ 0.05.

## Limits of the model

| Aspect | Simulation | Bullet M2 with ath9k |
|---|---|---|
| Contention | wmediumd delivers frames in arrival order across stations, with mean backoff | DCF with random backoff and collisions |
| Busy time | airtime of the frames that the radio decodes and sends | energy detection of the receiver (CCA), also for undecodable signals |
| Interference | broadcast frames of a fourth radio at a chosen duty | TX99: continuous transmission, 100 % duty |
| Propagation | fixed SNR of 40 dB per link, no loss | distance, multipath, loss |
