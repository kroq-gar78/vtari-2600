SOURCES=$(filter-out graphics_test.c cmdline.c,$(wildcard *.c)) cmdline.c
HEADERS=$(filter-out cmdline.h,$(wildcard *.h)) cmdline.h
OBJECTS=$(patsubst %.c,%.o,$(SOURCES))
TARGET=cpu

OKFILES = ${wildcard *.ok}
OUTFILES = ${patsubst %.ok,%.out,${OKFILES}}
RESFILES = ${patsubst %.ok,%.result,${OKFILES}}
DIFFFILES = ${patsubst %.ok,%.diff,${OKFILES}}

SFILES = ${wildcard *.S}
TESTFILES = ${patsubst %.S,%,${SFILES}}

TESTDIR=testdir/

CFLAGS := -MD -std=c99 -g -O0 -Wall -Werror -Wno-unused-variable -Wno-unused-but-set-variable -DATARI_2600 -DRENDER_FPS $(shell sdl2-config --cflags)
test: CFLAGS += -DMOS_6502 -DMOS_6502_TEST
test: CFLAGS := $(filter-out -DATARI_2600, $(CFLAGS))

LDLIBS := -lm $(shell sdl2-config --libs) -lSDL2_ttf

.PHONY: all test force

all : $(TARGET)

test: $(TARGET) Makefile

%.o : %.c $(HEADERS) Makefile compiler_flags
	$(CC) $(CFLAGS) -MD -c $< -o $@

$(TARGET): $(OBJECTS)

${OUTFILES} : %.out : %.ok % $(TARGET)
	./$(TARGET) $* > $*.out

${DIFFFILES} : %.diff : %.out %.ok
	diff -abwBu $*.ok $*.out > $*.diff; test $$? -le 1

${RESFILES} : %.result : %.out %.ok %.diff
	@echo -n "$*  ... "
	@( [ -s $*.diff ] && echo "fail") || echo "pass"

clean: Makefile
	rm -f $(TARGET) *.o *.d *.out *.diff *.result $(TESTFILES) cmdline.c cmdline.h

cmdline.h: gengetopt.in
	gengetopt -i gengetopt.in --include-getopt

cmdline.c: cmdline.h

# solution to changing compiler flags
# from: https://stackoverflow.com/a/3237349
compiler_flags: force
	@echo "Checking flags ..."
	@echo '$(CFLAGS)' | cmp -s - $@ || echo '$(CFLAGS)' > $@

-include *.d
