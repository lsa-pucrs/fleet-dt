# OpenWrt configuration of the radios

Every radio is a Ubiquiti Bullet M2 (XM board: Atheros AR7241 rev 1 SoC, AR9280 radio on PCIe 168c:002a,
subsystem 0777:e202, ath9k driver) with OpenWrt 22.03.7 (r20341-591b7e93d3, target ath79/tiny, image
`ubnt_bullet-m-ar7240`, the OpenWrt image of the XM Bullet M), as the capture of the vessel radio of
2026-09-17 and the airOS boot log of the GS radio (sysid 0xe202, SoC id 0x0101) record. The files mirror the
root of each radio. Addresses are those of the bench of `RUNBOOK.md`; the key `CHANGE_ME` is a
placeholder (the key of the field network stays out of the repository).

| Directory | Radio | Role |
|---|---|---|
| `gs/` | GS radio | access point with WDS, bridged to the GS PC; runs the channel monitor (`fdt_chmon`) |
| `vessel/` | radio of each vessel | station with WDS, bridged to the RP4 of the vessel |
| `injector/` | extra Bullet M2 | OpenWrt built with `CONFIG_ATH9K_TX99=y`; runs `radio/injector/tx99.sh` |

All radios use channel 6 (2437 MHz), 20 MHz, no HT, and the fixed legacy rate of 24 Mbit/s, the
R_PHY of `dtp.json`. `hotplug.d/iface/99-fdt-rate` fixes the rate when the interface comes up.

Build of the channel monitor for the radio: `radio/channel_monitor/openwrt/build-sdk.sh` runs the
OpenWrt SDK container of the radios (ath79/tiny, 22.03.7) and writes `build/openwrt/fdt-chmon_2.0-1_mips_24kc.ipk`. By hand: copy `radio/channel_monitor/openwrt/` to
`package/fdt-chmon/` of the OpenWrt SDK of the target (ath79), set `FDT_SRC` in its Makefile to the
root of this repository, and run `make package/fdt-chmon/compile`. The package depends on
`libmosquitto-nossl` and `cJSON`.
