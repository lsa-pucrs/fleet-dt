# fleet-dt — build of the model, the tests, the examples and the benchmarks.
#
# The library itself has no external dependency: `make lib` and `make test`
# work on a bare toolchain. Anything that needs a third-party SDK lives behind
# its own target and skips with a notice when the SDK is absent.

CC      = gcc
CFLAGS  = -std=c18 -Wall -Wextra -Werror -pedantic-errors -O2 -Iinclude
LDLIBS  = -lm

LIB     = libfleetdt.a
LIBSRC  = $(wildcard src/*.c)
LIBOBJ  = $(LIBSRC:.c=.o)

TESTSRC = $(wildcard tests/test_*.c)
TESTBIN = $(TESTSRC:tests/test_%.c=fleet_dt_test_%)

RESULTS = results

.PHONY: all lib test clean

all: lib

lib: $(LIB)

$(LIB): $(LIBOBJ)
	ar rcs $@ $^

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

fleet_dt_test_%: tests/test_%.c $(LIB)
	$(CC) $(CFLAGS) $< $(LIB) $(LDLIBS) -o $@

test: $(TESTBIN)
	@for t in $(TESTBIN); do echo "== $$t"; ./$$t || exit 1; done

clean:
	rm -f $(LIBOBJ) $(LIB) $(TESTBIN)
	rm -rf $(RESULTS)
