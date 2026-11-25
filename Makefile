# fleet-dt, build of the model, the tests, the examples and the benchmarks.
#
#   make lib       the static library, no external dependency
#   make test      the unit suite; every published figure is checked here
#   make examples  the runnable daemons
#   make bench     the measurement campaign; writes results/
#   make all       lib + test + examples + bench
#
# Anything needing a third-party SDK lives behind its own target and skips
# with a notice when the SDK is absent, so this file never fails for a reason
# the reader cannot act on.

CC      = gcc
# -MMD -MP emit a .d beside every .o listing the headers it included, and the
# generated rules are pulled in below. Without them, editing a header rebuilds
# nothing: a constant changed in tools/injector/injector.h once left every
# object stale and the simulation kept running the old value, which looks
# exactly like a change that did not work.
CFLAGS  = -std=c18 -Wall -Wextra -Werror -pedantic-errors -O2 -MMD -MP \
          -Iinclude -Itools -Iadapters
LDLIBS  = -lm

LIB     = libfleetdt.a
LIBSRC  = $(wildcard src/*.c)
LIBOBJ  = $(LIBSRC:.c=.o)

# The injector is a tool, not part of the library: it belongs to the Section
# V-B campaign rather than to the model. It is compiled once and linked into
# whatever needs it.
TOOLSRC = tools/injector/injector.c
TOOLOBJ = $(TOOLSRC:.c=.o)

# Adapters with no external dependency: the Ardupilot ingest mapping and the
# HSDT camera boundary. Those that do need an SDK -- MQTT, WeBots -- live
# behind their own targets further down.
ADAPTSRC = adapters/mavlink/fdt_mavlink.c adapters/rtsp/fdt_rtsp_fake.c \
           adapters/mqtt/fdt_mqtt.c adapters/mqtt/fdt_mqtt_wire.c
ADAPTOBJ = $(ADAPTSRC:.c=.o)

TESTSRC = $(wildcard tests/test_*.c)
TESTBIN = $(TESTSRC:tests/test_%.c=fleet_dt_test_%)

EXAMPLESRC = $(wildcard examples/*.c)
EXAMPLEBIN = $(EXAMPLESRC:.c=)

BENCHSRC = $(wildcard tools/bench/bench_*.c)
BENCHBIN = $(BENCHSRC:.c=)

RESULTS = results

# Port the MQTT integration check runs its own broker on, chosen so it cannot
# collide with a broker already serving a fleet on 1883.
MQTT_IT_PORT = 18830

.PHONY: all lib test examples bench report syntax mqtt mqtt-test webots clean

# Adapters needing a third-party SDK. Each skips with a notice rather than
# failing the build, so a reader without the SDK still gets a green tree.

all: lib test examples bench syntax report

lib: $(LIB)

$(LIB): $(LIBOBJ)
	ar rcs $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

fleet_dt_test_%: tests/test_%.c $(LIB) $(TOOLOBJ) $(ADAPTOBJ)
	$(CC) $(CFLAGS) $< $(TOOLOBJ) $(ADAPTOBJ) $(LIB) $(LDLIBS) -o $@

test: $(TESTBIN)
	@for t in $(TESTBIN); do echo "== $$t"; ./$$t || exit 1; done

examples: $(EXAMPLEBIN)

examples/%: examples/%.c $(LIB)
	$(CC) $(CFLAGS) $< $(LIB) $(LDLIBS) -o $@

tools/bench/bench_%: tools/bench/bench_%.c $(LIB) $(TOOLOBJ)
	$(CC) $(CFLAGS) $< $(TOOLOBJ) $(LIB) $(LDLIBS) -o $@

bench: $(BENCHBIN)
	@mkdir -p $(RESULTS)
	@for b in $(BENCHBIN); do echo "== $$b"; ./$$b || exit 1; echo; done
	@echo "artefacts in $(RESULTS)/"

# Compile coverage for the adapters whose SDK is not installed here. Parsing
# and type-checking them against stub headers is the difference between
# "written" and "known to compile"; it does not replace a real build, which is
# the only thing that can catch a wrong assumption about the API.
syntax:
	@$(CC) $(CFLAGS) -Itools/stubs -fsyntax-only \
	  adapters/webots/controllers/fdt_controller/fdt_controller.c && \
	  echo "syntax ok: adapters/webots"
	@$(CC) $(CFLAGS) -fsyntax-only tests/it_mqtt.c && \
	  echo "syntax ok: tests/it_mqtt.c"

# The MQTT client is the repository's own (adapters/mqtt/fdt_mqtt_wire.c and
# fdt_mqtt.c, over POSIX sockets) and builds with every other adapter. This
# target stays for the scripts that call it.
mqtt: $(LIB) $(ADAPTOBJ)
	@echo "the MQTT client is built with the adapters; no library needed"

# Integration check for the MQTT client: needs a mosquitto broker on PATH,
# the broker the paper names. Kept out of `make test` so the unit suite
# needs no running service. It starts
# two brokers, ground and boat, joined by the bridge rules of
# config/mosquitto/boat.conf, and runs the regulated path across them.
MQTT_IT_BOAT_PORT = 18831

mqtt-test: $(LIB) $(ADAPTOBJ)
	@if ! command -v mosquitto >/dev/null 2>&1; then \
	  echo "no mosquitto broker on PATH; skipping the integration check."; \
	else \
	  $(CC) $(CFLAGS) tests/it_mqtt.c \
	    $(ADAPTOBJ) $(LIB) $(LDLIBS) -o fleet_dt_it_mqtt && \
	  { echo "listener $(MQTT_IT_PORT) 127.0.0.1"; \
	    echo "allow_anonymous true"; \
	    echo "pid_file /tmp/fleet-dt-it.pid"; } > /tmp/fleet-dt-it.conf && \
	  { echo "listener $(MQTT_IT_BOAT_PORT) 127.0.0.1"; \
	    echo "allow_anonymous true"; \
	    echo "pid_file /tmp/fleet-dt-it-boat.pid"; \
	    echo "connection ground"; \
	    echo "address 127.0.0.1:$(MQTT_IT_PORT)"; \
	    grep -E '^topic ' config/mosquitto/boat.conf; \
	    echo "bridge_protocol_version mqttv50"; } > /tmp/fleet-dt-it-boat.conf && \
	  mosquitto -c /tmp/fleet-dt-it.conf -d >/dev/null 2>&1 && \
	  mosquitto -c /tmp/fleet-dt-it-boat.conf -d >/dev/null 2>&1 && \
	  for i in 1 2 3 4 5 6 7 8 9 10; do \
	    ss -ltn 2>/dev/null | grep -q ':$(MQTT_IT_BOAT_PORT)' && break; \
	    sleep 0.2; \
	  done && sleep 1 && \
	  ./fleet_dt_it_mqtt $(MQTT_IT_PORT) $(MQTT_IT_BOAT_PORT); rc=$$?; \
	  kill `cat /tmp/fleet-dt-it-boat.pid` 2>/dev/null; \
	  kill `cat /tmp/fleet-dt-it.pid` 2>/dev/null; \
	  exit $$rc; \
	fi

webots: $(LIB) $(TOOLOBJ)
	@if [ -z "$$WEBOTS_HOME" ]; then \
	  echo "WEBOTS_HOME is not set; skipping the WeBots controller."; \
	  echo "  see adapters/webots/README.md"; \
	else \
	  $(CC) $(CFLAGS) -I$$WEBOTS_HOME/include/controller/c \
	    adapters/webots/controllers/fdt_controller/fdt_controller.c \
	    $(TOOLOBJ) $(LIB) -L$$WEBOTS_HOME/lib/controller \
	    -lController $(LDLIBS) \
	    -o adapters/webots/controllers/fdt_controller/fdt_controller && \
	  echo "built adapters/webots/controllers/fdt_controller/fdt_controller" && \
	  echo "open the world with:  webots adapters/webots/worlds/jundia_fleet.wbt"; \
	fi

tools/report/report: tools/report/report.c $(LIB)
	$(CC) $(CFLAGS) $< $(LIB) $(LDLIBS) -o $@

report: tools/report/report
	@./tools/report/report

# Header dependencies for everything compiled so far.
-include $(LIBOBJ:.o=.d) $(TOOLOBJ:.o=.d) $(ADAPTOBJ:.o=.d)

clean:
	rm -f $(LIBOBJ:.o=.d) $(TOOLOBJ:.o=.d) $(ADAPTOBJ:.o=.d)
	rm -f $(LIBOBJ) $(TOOLOBJ) $(ADAPTOBJ) $(LIB) $(TESTBIN) $(EXAMPLEBIN) \
	      $(BENCHBIN) tools/report/report fleet_dt_it_mqtt \
	      adapters/webots/controllers/fdt_controller/fdt_controller
	rm -rf $(RESULTS)/selftest
# results/ itself is NOT removed. It is committed, so that the charts in
# docs/RESULTS.md resolve on a fresh clone, and a clean that deleted it would
# stage the deletion of tracked artefacts -- which is exactly what happened
# once. Only the plotter's self-test output, which is ignored, is swept.
