#!/bin/bash
# Inside a VM of the emulator (emu/run.py ssh v1 'bash ~/fleet-dt/emu/validation/tick_lateness.sh'): lateness of
# 1 ms plugin ticks of the guest Mosquitto (patches of mosquitto/patches), idle and with 2000 messages/s of
# unrelated traffic (which wakes the loop before the tick deadlines: patch 0001), broker under SCHED_FIFO 50.
set -e
D=$(mktemp -d /tmp/tickprobe.XXXX)
cc -O2 -fPIC -shared -I/opt/mosquitto-2.1/include -o $D/tickprobe.so ~/fleet-dt/emu/validation/tickprobe.c
echo "== guest Mosquitto: $(/opt/mosquitto-2.1/sbin/mosquitto -h 2>&1 | head -1)"
echo "== patches of the guest build:"; cat /opt/mosquitto-2.1/patches.sha256
for mode in idle traffic; do
	printf 'listener 18830 127.0.0.1\nallow_anonymous true\nplugin %s/tickprobe.so\nplugin_opt_out %s/%s.json\nplugin_opt_period_us 1000\n' \
		$D $D $mode > $D/m.conf
	/opt/mosquitto-2.1/sbin/mosquitto -c $D/m.conf > $D/m.log 2>&1 & p=$!
	sudo chrt -f -p 50 $p > /dev/null
	sleep 0.5
	if [ $mode = traffic ]; then
		python3 - <<'PY' &
import time, paho.mqtt.client as m
c = m.Client(m.CallbackAPIVersion.VERSION2); c.connect("127.0.0.1", 18830); c.loop_start()
t0 = time.time(); k = 0
while time.time() - t0 < 20:
    c.publish("x/y", b"0" * 100, qos=0); k += 1
    nxt = t0 + k / 2000.0
    d = nxt - time.time()
    if d > 0: time.sleep(d)
c.loop_stop()
PY
		q=$!
		sleep 20; wait $q || true
	else
		sleep 20
	fi
	kill -TERM $p; wait $p || true
	echo "== $mode: $(cat $D/$mode.json)"
done
rm -rf $D
