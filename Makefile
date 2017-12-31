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

TESTDIR=tests
TEST_RAMFILE=test_output_ram

CFLAGS := -MD -std=c99 -g3 -O0 -Wall -Werror -Wno-unused-variable -Wno-unused-but-set-variable -DATARI_2600 -DRENDER_FPS $(shell sdl2-config --cflags)
testbuild: CFLAGS += -DMOS_6502 -DMOS_6502_TEST
testbuild: CFLAGS := $(filter-out -DATARI_2600, $(CFLAGS))

LDLIBS := -lm $(shell sdl2-config --libs) -lSDL2_ttf

.PHONY: all force test

all : $(TARGET)

testbuild: $(TARGET) Makefile

$(TESTDIR)/AllSuiteA.result: testbuild
	@echo "Testing $(TESTDIR)/AllSuiteA.bin"
	./$(TARGET) -b 0x45c0 -s 0x4000 --dump-ram=$(TEST_RAMFILE) --no-graphics $(TESTDIR)/AllSuiteA.bin
	@if [ `grep '0x0210' < "$(TEST_RAMFILE)"` = "0x0210,0xff" ] ; then \
		echo "pass" > "$(TESTDIR)/AllSuiteA.result";\
		echo "Test $(TESTDIR)/AllSuiteA.bin passed";\
	else\
		echo "fail" > "$(TESTDIR)/AllSuiteA.result";\
		echo "Test $(TESTDIR)/AllSuiteA.bin FAILED";\
	fi

test: $(TESTDIR)/AllSuiteA.result

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
	rm -f $(TARGET) *.o *.d *.out *.diff *.result $(TESTFILES) cmdline.c cmdline.h $(TEST_RAMFILE)

cmdline.h: gengetopt.in
	gengetopt -i gengetopt.in --include-getopt

cmdline.c: cmdline.h

# solution to changing compiler flags
# from: https://stackoverflow.com/a/3237349
compiler_flags: force
	@echo "Checking flags ..."
	@echo '$(CFLAGS)' | cmp -s - $@ || echo '$(CFLAGS)' > $@

-include *.d
