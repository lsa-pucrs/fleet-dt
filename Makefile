# Fleet-DT: C programs (vessel regulator plugin v2, v1 DTI program, channel monitor) and tests.
#
#   make                 build everything into build/ (fdt_chmon_host: the channel monitor of the
#                        emulation host without MQTT, backend c of fleetdt/monitor_adapter.py)
#   make test-c          C unit tests of the v2 regulator and of mode paper (test-c-san: with ASan and UBSan)
#   make test-paper      integration test of the paper stack (tests/test_paper_stack.py, needs MOSQ)
#   make test-vessel     integration test of the vessel side v2 (tests/vessel_v2, needs MOSQ)
#   make test            test-c, test-vessel and the Python tests
#   make install         install the C programs into PREFIX (vessel or GS)
#
# MOSQ is the prefix of the patched Mosquitto 2.1.2 and of cJSON 1.7.19 (mosquitto/build.sh); the
# default takes /opt/mosquitto-2.1, else /opt/mosquitto-2.1.2-fleetdt.
# On the GS radio (OpenWrt) the channel monitor builds with the OpenWrt SDK: radio/channel_monitor/openwrt/.

MOSQ    ?= $(firstword $(wildcard /opt/mosquitto-2.1 /opt/mosquitto-2.1.2-fleetdt) /opt/mosquitto-2.1)
PREFIX  ?= /opt/fleet-dt
CC      ?= cc
PYTHON  ?= python3
CFLAGS  ?= -O2 -g -Wall -Wextra -Wno-unused-parameter -std=gnu11
CPPFLAGS += -Icommon -Ivessel/regulator -I$(MOSQ)/include
LDFLAGS += -L$(MOSQ)/lib -Wl,-rpath,$(MOSQ)/lib
B = build
SANFLAGS = -fsanitize=address -fsanitize=undefined -fno-omit-frame-pointer

COMMON = common/fdt_common.c
COMMON_H = common/fdt_common.h
DTP2 = common/fdt_dtp2.c common/fdt_common.c
DTP2_H = common/fdt_dtp2.h common/fdt_common.h
REG = vessel/regulator/fdt_reg.c
REG_H = vessel/regulator/fdt_reg.h
# mode paper of the regulator (fdt_reg_paper.c) and its DTP reader and 0-based clock (fdt_paper.c)
PREG = vessel/regulator/fdt_reg_paper.c common/fdt_paper.c
PREG_H = vessel/regulator/fdt_reg_paper.h common/fdt_paper.h

all: $(B)/fdt_regulator.so $(B)/fdt_dti $(B)/fdt_chmon $(B)/fdt_chmon_host $(B)/test_reg $(B)/test_reg_paper

$(B):
	mkdir -p $(B)

# v2: vessel regulator and DTI program in one plugin
$(B)/fdt_regulator.so: vessel/regulator/fdt_regulator_plugin.c vessel/regulator/fdt_sock.c vessel/regulator/fdt_sock.h $(REG) $(REG_H) $(PREG) $(PREG_H) $(DTP2) $(DTP2_H) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -fPIC -shared -o $@ vessel/regulator/fdt_regulator_plugin.c vessel/regulator/fdt_sock.c $(REG) $(PREG) $(DTP2) $(LDFLAGS) -lcjson -lm

# v1 programs (DTP v1, dtp.json); fdt_dti computes D_i(t) with the reader and the bound of mode paper
$(B)/fdt_dti: vessel/dti/fdt_dti.c common/fdt_paper.c common/fdt_paper.h $(COMMON) $(COMMON_H) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -pthread -o $@ vessel/dti/fdt_dti.c common/fdt_paper.c $(COMMON) $(LDFLAGS) -lmosquitto -lcjson -lm

$(B)/fdt_chmon: radio/channel_monitor/fdt_chmon.c $(COMMON) $(COMMON_H) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ radio/channel_monitor/fdt_chmon.c $(COMMON) $(LDFLAGS) -lmosquitto -lcjson -lm

# the recipe of radio/channel_monitor/build_host.sh (FDT_NO_MQTT, -ffp-contract=off) with the cJSON of MOSQ:
# it needs a C compiler and cJSON only, which every program here needs, so it is part of all
$(B)/fdt_chmon_host: radio/channel_monitor/fdt_chmon.c radio/channel_monitor/build_host.sh $(COMMON) $(COMMON_H) | $(B)
	CC="$(CC)" CJSON_CFLAGS="-I$(MOSQ)/include" CJSON_LIBS="-L$(MOSQ)/lib -Wl,-rpath,$(MOSQ)/lib -lcjson" \
		sh radio/channel_monitor/build_host.sh $@ > /dev/null

$(B)/test_reg: tests/c/test_reg.c $(REG) $(REG_H) $(DTP2) $(DTP2_H) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tests/c/test_reg.c $(REG) $(DTP2) $(LDFLAGS) -lcjson -lm

$(B)/test_reg_san: tests/c/test_reg.c $(REG) $(REG_H) $(DTP2) $(DTP2_H) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANFLAGS) -o $@ tests/c/test_reg.c $(REG) $(DTP2) $(LDFLAGS) -lcjson -lm

$(B)/test_reg_paper: tests/c/test_reg_paper.c $(PREG) $(PREG_H) $(REG_H) $(COMMON) $(COMMON_H) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ tests/c/test_reg_paper.c $(PREG) $(COMMON) $(LDFLAGS) -lcjson -lm

$(B)/test_reg_paper_san: tests/c/test_reg_paper.c $(PREG) $(PREG_H) $(REG_H) $(COMMON) $(COMMON_H) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SANFLAGS) -o $@ tests/c/test_reg_paper.c $(PREG) $(COMMON) $(LDFLAGS) -lcjson -lm

# user-space CPU cap of the vessel QEMUs (emu/vm/cpu_profile.py, mechanism "user"; no MOSQ needed)
$(B)/fdt_cpucap: emu/vm/cpucap.c | $(B)
	$(CC) -O2 -g -Wall -Wextra -o $@ emu/vm/cpucap.c

$(B)/cpucap_probe: emu/vm/cpucap_probe.c | $(B)
	$(CC) -O2 -g -Wall -o $@ emu/vm/cpucap_probe.c

test-c: $(B)/test_reg $(B)/test_reg_paper
	$(B)/test_reg
	$(B)/test_reg_paper

test-c-san: $(B)/test_reg_san $(B)/test_reg_paper_san
	$(B)/test_reg_san
	$(B)/test_reg_paper_san

test-paper: $(B)/fdt_regulator.so $(B)/fdt_dti
	FDT_BUILD=$(abspath $(B)) MOSQ=$(MOSQ) $(PYTHON) -m pytest -q tests/test_paper_stack.py

test-vessel: $(B)/fdt_regulator.so
	FDT_BUILD=$(abspath $(B)) MOSQ=$(MOSQ) $(PYTHON) -m pytest -q tests/vessel_v2

# tests/test_integration.py is the end-to-end test of the paper stack with dtp.json (fdt_dti, the
# regulator in mode paper, fdt_chmon, AS, VE); it runs when MOSQ holds the patched Mosquitto.
test: all test-c
	FDT_BUILD=$(abspath $(B)) MOSQ=$(MOSQ) $(PYTHON) -m pytest -q tests

install: all
	install -d $(DESTDIR)$(PREFIX)/lib $(DESTDIR)$(PREFIX)/bin
	install -m 0644 $(B)/fdt_regulator.so $(DESTDIR)$(PREFIX)/lib/
	install -m 0755 $(B)/fdt_dti $(B)/fdt_chmon $(DESTDIR)$(PREFIX)/bin/
	install -m 0755 vessel/publishers/fdt_source.py $(DESTDIR)$(PREFIX)/bin/

clean:
	rm -rf $(B)

.PHONY: all test test-c test-c-san test-vessel test-paper install clean
