# Runbook

Installation and operation of Fleet-DT on the bench and in the field. Addresses follow
`radio/openwrt/` and `system/`; change them in one place per host (`/etc/fleetdt/fleetdt.env`, the
UCI files, the broker configurations).

| Host | Address | Software |
|---|---|---|
| GS PC (RHEL 10) | 192.168.1.10 | patched Mosquitto in `/opt/mosquitto-2.1`, GS broker, AS, VE, chrony server |
| GS radio (Bullet M2) | 192.168.1.20 | OpenWrt, `fdt_chmon` |
| vessel i radio (Bullet M2) | 192.168.1.20 + i | OpenWrt, station with WDS |
| vessel 1 RP4 (Raspbian 10) | 192.168.1.99 | patched Mosquitto, vessel regulator, `fdt_dti`, publishers, chrony client |
| vessel 2 RP4 (Raspbian 10) | 192.168.1.98 | as vessel 1 |
| injector (Bullet M2) | none | OpenWrt with `CONFIG_ATH9K_TX99=y`, `tx99.sh` |

## 1. Installation

On the GS PC and on each RP4:

```bash
git clone <repository> /opt/fleet-dt/src && cd /opt/fleet-dt/src
mosquitto/build.sh ~/dist                 # mosquitto-2.1.2.tar.gz and cJSON-1.7.19.tar.gz in ~/dist
make && sudo make install
sudo install -d /etc/fleetdt /var/lib/fleetdt
sudo install -m 0644 dtp.json /etc/fleetdt/dtp.json
sudo install -m 0644 system/fleetdt.env /etc/fleetdt/fleetdt.env       # set FDT_VESSEL on each RP4
sudo install -m 0644 system/sysctl/99-fleetdt.conf /etc/sysctl.d/ && sudo sysctl --system
sudo install -m 0755 system/net/fleetdt-quickack.sh /usr/local/sbin/
```

GS PC:

```bash
pip install -r requirements.txt
python3 -m fleetdt.gsconf gs --vessel 1=192.168.1.99 --vessel 2=192.168.1.98 | sudo tee /etc/fleetdt/gs.conf
sudo install -m 0644 system/chrony/gs.conf /etc/chrony.d/fleetdt.conf && sudo systemctl restart chronyd
sudo install -m 0644 system/systemd/gs/*.service /etc/systemd/system/
sudo systemctl enable --now fleetdt-gs-broker fleetdt-as fleetdt-ve
```

Each RP4 (vessel i):

```bash
sudo apt install python3-opencv python3-numpy && pip3 install "paho-mqtt>=2" typing_extensions pymavlink
python3 -m fleetdt.gsconf vessel --vessel 1 | sudo tee /etc/fleetdt/mosquitto.conf
sudo install -m 0644 system/chrony/vessel.conf /etc/chrony/conf.d/fleetdt.conf && sudo systemctl restart chrony
sudo install -m 0644 system/systemd/vessel/*.service /etc/systemd/system/
sudo systemctl enable --now fleetdt-broker fleetdt-dti fleetdt-mav fleetdt-zed
```

GS radio: copy `radio/openwrt/gs/` over the root of the radio (set the key), install the package
`fdt-chmon` (see `radio/openwrt/README.md`), and `/etc/init.d/fdt-chmon enable && /etc/init.d/fdt-chmon start`.
Vessel radios: `radio/openwrt/vessel/`, with the address 192.168.1.20 + i.

## 2. Checks before an operation

| Check | Command | Expected |
|---|---|---|
| clock of each RP4 | `chronyc tracking` | offset under 0.5 ms |
| bridges | `mosquitto_sub -t 'fleet/bridge/#' -v` on the GS | `1` for every `b<i>-ctl` and `b<i>-data` |
| demand tables | `mosquitto_sub -t 'boat/+/demand' -C 16 -F '%t'` | 8 tables per second and vessel |
| channel monitor | `mosquitto_sub -t fleet/throughput` | one JSON per step |
| AS | `journalctl -u fleetdt-as` | no "published … ms after start(t)+T1" warnings |
| vessel regulator | `mosquitto_sub -t 'boat/+/regulator'` | `have_d` and `have_p` equal 1, `late_us` under 2000 |
| VE | http://192.168.1.10:8080 | G(t) with every vessel of the fleet |

## 3. Operation

The DTA grows in `/var/lib/fleetdt/dta.jsonl` (one G(t) per line); the decisions of the AS in
`/var/lib/fleetdt/as.jsonl`. Move both files to a run directory at the end of each run. The
operation control software publishes `fleet/order` (σ and N) and `boat/b<i>/goals`.

Interference runs: on the injector,
`tx99.sh plan -s plan.txt -a <Unix time of the start>` with one burst per line of `plan.txt`.

## 4. After a run

```bash
python3 -m fleetdt.analysis runs/*/dta.jsonl --dtp /etc/fleetdt/dtp.json --json > metrics.json
python3 -m fleetdt.outer runs/c3/dta.jsonl --window 4800 --every 4800      # condition C3
python3 -m fleetdt.ve replay runs/c1/dta.jsonl
```

## 5. Faults

| Symptom | Cause | Action |
|---|---|---|
| a vessel leaves M(t) in every step | demand table after start(t) + T1: clock offset or control bridge down | `chronyc tracking` on the RP4; `fleet/bridge/b<i>-ctl/state` |
| `have_p` 0 in the step log | priority table after phase 3 started | check the AS warnings and the control bridge |
| `late` messages in G(t) | phase 3 overran the step: Ĉ(t) above C(t) | lower U or run the outer loop (`--outer-dta`) |
| `fdt_chmon` publishes nothing | no survey of the channel in use | `fdt_chmon --iface wlan0 --dump` must print `"survey":1` |
| priority tables arrive about 40 ms late | a client without TCP_NODELAY | every Fleet-DT client sets it; check third-party clients on the GS broker |
