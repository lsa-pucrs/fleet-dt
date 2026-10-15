# Host configuration

| File | Host | Install as | Purpose |
|---|---|---|---|
| `chrony/gs.conf` | GS PC | `/etc/chrony.d/fleetdt.conf` (RHEL) | the GS serves time to the vessels and to the radios (R4) |
| `chrony/vessel.conf` | vessel RP4 | `/etc/chrony/conf.d/fleetdt.conf` (Raspbian) | the vessel follows the GS every 1 to 4 s (R4) |
| `sysctl/99-fleetdt.conf` | GS and vessels | `/etc/sysctl.d/99-fleetdt.conf` | TCP keeps its window across phases 1 and 2; no autocorking (R8) |
| `net/fleetdt-quickack.sh` | GS and vessels | `/usr/local/sbin/`, run after the link comes up | immediate TCP ACKs on the route of the fleet network (R8) |
| `systemd/vessel/*.service` | vessel RP4 | `/etc/systemd/system/` | broker with the vessel regulator, DTI program, publishers |
| `systemd/gs/*.service` | GS PC | `/etc/systemd/system/` | GS broker, AS, VE |
| `fleetdt.env` | every host | `/etc/fleetdt/fleetdt.env` | vessel index and addresses read by the units |

The units expect the C programs in `/opt/fleet-dt` (`make install`), the patched Mosquitto in
`/opt/mosquitto-2.1` (`mosquitto/build.sh`), the repository in `/opt/fleet-dt/src`, and the DTP file
in `/etc/fleetdt/dtp.json`.
