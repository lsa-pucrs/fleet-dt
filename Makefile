# Fleet-DT: C programs (vessel regulator plugin, DTI program, channel monitor) and tests.
#
#   make                 build everything into build/
#   make test            C unit test and the Python tests (the integration test needs MOSQ)
#   make install         install the C programs into PREFIX (vessel or GS)
#
# MOSQ is the prefix of the patched Mosquitto 2.1.2 and of cJSON 1.7.19 (mosquitto/build.sh).
# On the GS radio (OpenWrt) the channel monitor builds with the OpenWrt SDK: radio/channel_monitor/openwrt/.

MOSQ    ?= /opt/mosquitto-2.1
PREFIX  ?= /opt/fleet-dt
CC      ?= cc
PYTHON  ?= python3
CFLAGS  ?= -O2 -g -Wall -Wextra -Wno-unused-parameter -std=gnu11
CPPFLAGS += -Icommon -Ivessel/regulator -I$(MOSQ)/include
LDFLAGS += -L$(MOSQ)/lib -Wl,-rpath,$(MOSQ)/lib
B = build
# sanitizers for the unit test where libasan exists: make SAN="-fsanitize=address,undefined"
SAN     ?=

COMMON = common/fdt_common.c
COMMON_H = common/fdt_common.h

all: $(B)/fdt_regulator.so $(B)/fdt_dti $(B)/fdt_chmon $(B)/test_reg

$(B):
	mkdir -p $(B)

$(B)/fdt_regulator.so: vessel/regulator/fdt_regulator_plugin.c vessel/regulator/fdt_reg.c vessel/regulator/fdt_reg.h $(COMMON) $(COMMON_H) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -fPIC -shared -o $@ vessel/regulator/fdt_regulator_plugin.c vessel/regulator/fdt_reg.c $(COMMON) $(LDFLAGS) -lcjson -lm

$(B)/fdt_dti: vessel/dti/fdt_dti.c $(COMMON) $(COMMON_H) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -pthread -o $@ vessel/dti/fdt_dti.c $(COMMON) $(LDFLAGS) -lmosquitto -lcjson -lm

$(B)/fdt_chmon: radio/channel_monitor/fdt_chmon.c $(COMMON) $(COMMON_H) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ radio/channel_monitor/fdt_chmon.c $(COMMON) $(LDFLAGS) -lmosquitto -lcjson -lm

$(B)/test_reg: tests/c/test_reg.c vessel/regulator/fdt_reg.c vessel/regulator/fdt_reg.h $(COMMON) $(COMMON_H) | $(B)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(SAN) -o $@ tests/c/test_reg.c vessel/regulator/fdt_reg.c $(COMMON) $(LDFLAGS) -lcjson -lm

test: all
	$(B)/test_reg
	FDT_BUILD=$(abspath $(B)) MOSQ=$(MOSQ) $(PYTHON) -m pytest -q tests

install: all
	install -d $(DESTDIR)$(PREFIX)/lib $(DESTDIR)$(PREFIX)/bin
	install -m 0644 $(B)/fdt_regulator.so $(DESTDIR)$(PREFIX)/lib/
	install -m 0755 $(B)/fdt_dti $(B)/fdt_chmon $(DESTDIR)$(PREFIX)/bin/

clean:
	rm -rf $(B)

.PHONY: all test install clean
