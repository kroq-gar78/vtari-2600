SOURCES=$(filter-out graphics_test.c,$(wildcard *.c))
HEADERS=$(wildcard *.h)
OBJECTS=$(patsubst %.c,%.o,$(SOURCES))
TARGET=cpu

OKFILES = ${wildcard *.ok}
OUTFILES = ${patsubst %.ok,%.out,${OKFILES}}
RESFILES = ${patsubst %.ok,%.result,${OKFILES}}
DIFFFILES = ${patsubst %.ok,%.diff,${OKFILES}}

SFILES = ${wildcard *.S}
TESTFILES = ${patsubst %.S,%,${SFILES}}

TESTDIR=testdir/

CFLAGS = -std=c99 -g -O0 -Wall -Werror -Wno-unused-variable -DRENDER_FPS -lm $(shell sdl2-config --cflags) $(shell sdl2-config --libs) -lSDL2_ttf
test: CFLAGS += -DMOS_6502

.PHONY: all test force

all : $(TARGET)

test: $(TARGET) Makefile

%.o : %.c $(HEADERS) Makefile compiler_flags
	$(CC) $(CFLAGS) -MD -c $< -o $@

$(TARGET): $(OBJECTS) Makefile compiler_flags
	$(CC) $(OBJECTS) $(CFLAGS) -o $(TARGET)

${OUTFILES} : %.out : %.ok % $(TARGET)
	./$(TARGET) $* > $*.out

${DIFFFILES} : %.diff : %.out %.ok
	diff -abwBu $*.ok $*.out > $*.diff; test $$? -le 1

${RESFILES} : %.result : %.out %.ok %.diff
	@echo -n "$*  ... "
	@( [ -s $*.diff ] && echo "fail") || echo "pass"

clean: Makefile
	rm -f $(TARGET) *.o *.d *.out *.diff *.result $(TESTFILES)

# solution to changing compiler flags
# from: https://stackoverflow.com/a/3237349
compiler_flags: force
	@echo "Checking flags ..."
	@echo '$(CFLAGS)' | cmp -s - $@ || echo '$(CFLAGS)' > $@

-include *.d
