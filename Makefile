SOURCES=$(wildcard *.c)
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

CFLAGS = -std=c99 -g -O0 -Wall -Werror -Wno-unused-variable -DATARI_2600 -lm $(shell sdl2-config --cflags) $(shell sdl2-config --libs)

all : $(TARGET)

test: ${RESFILES}

%.o : %.c $(HEADERS) Makefile
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET): $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(CFLAGS) -o $(TARGET)

${OUTFILES} : %.out : %.ok % $(TARGET)
	./$(TARGET) $* > $*.out

${DIFFFILES} : %.diff : %.out %.ok
	diff -abwBu $*.ok $*.out > $*.diff; test $$? -le 1

${RESFILES} : %.result : %.out %.ok %.diff
	@echo -n "$*  ... "
	@( [ -s $*.diff ] && echo "fail") || echo "pass"

testemail:
	$(MAKE) -C $(TESTDIR) summary

clean: Makefile
	rm -f $(TARGET) *.o *.d *.out *.diff *.result $(TESTFILES)
