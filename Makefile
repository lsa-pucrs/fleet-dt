# fleet-dt — build of the model, the tests, the examples and the benchmarks.
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
CFLAGS  = -std=c18 -Wall -Wextra -Werror -pedantic-errors -O2 \
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
ADAPTSRC = adapters/mavlink/fdt_mavlink.c adapters/rtsp/fdt_rtsp_fake.c
ADAPTOBJ = $(ADAPTSRC:.c=.o)

TESTSRC = $(wildcard tests/test_*.c)
TESTBIN = $(TESTSRC:tests/test_%.c=fleet_dt_test_%)

EXAMPLESRC = $(wildcard examples/*.c)
EXAMPLEBIN = $(EXAMPLESRC:.c=)

BENCHSRC = $(wildcard tools/bench/bench_*.c)
BENCHBIN = $(BENCHSRC:.c=)

RESULTS = results

.PHONY: all lib test examples bench clean

all: lib test examples bench

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

clean:
	rm -f $(LIBOBJ) $(TOOLOBJ) $(ADAPTOBJ) $(LIB) $(TESTBIN) $(EXAMPLEBIN) \
	      $(BENCHBIN) libfleetdt_mqtt.a adapters/mqtt/fdt_mqtt.o
	rm -rf $(RESULTS)
