CC := gcc
LD := gcc
AR := ar
#CFLAGS_RELEASE := -Wall -std=c99 -Isrc -Ofast -ffast-math -mtune=native -march=native -pthread
CFLAGS_RELEASE := -Wall -std=c99 -Isrc -g -pthread
#LDFLAGS_RELEASE := -Ofast -ffast-math -mtune=native -march=native -flto -fuse-ld=gold -lm -lSDL2 -lSDL2_gfx -pthread
LDFLAGS_RELEASE := -g -flto -fuse-ld=gold -lm -lSDL2 -lSDL2_gfx -pthread
CFLAGS_DEBUG := -Wall -std=c99 -g3 --coverage -pthread
LDFLAGS_DEBUG := --coverage -lm -pthread
TEST_CFLAGS := -Wall -std=c99 -g3 -Isrc -pthread
TEST_LDFLAGS := --coverage -lm -pthread
BUILD_ROOT := $(shell pwd)
SOURCES := $(shell find src -maxdepth 1 -name '*.c')
TEST_SOURCES := $(shell find src/test -maxdepth 1 -name '*.c')
BINARY_SOURCES := $(shell find src/bin -maxdepth 1 -name '*.c')
OBJS_DEBUG := $(patsubst src/%.c,obj/debug/%.o,$(SOURCES))
OBJS_RELEASE := $(patsubst src/%.c,obj/release/%.o,$(SOURCES))
TEST_BINARIES := $(patsubst src/test/%.c,bin/test/%,$(TEST_SOURCES))
BINARIES := $(patsubst src/bin/%.c,bin/%,$(BINARY_SOURCES))

all: coverage binaries | Makefile

.SECONDARY:
obj/bin obj/test obj/debug obj/release bin bin/test notes:
	mkdir -p $@

bin/%: obj/bin/%.o $(OBJS_RELEASE) | bin
	$(CC) $(LDFLAGS_RELEASE) -o $@ $^

bin/test/%: obj/test/%.o $(OBJS_DEBUG) | bin/test 
	$(CC) $(TEST_LDFLAGS) -o $@ $^

obj/release/%.o: src/%.c | obj/release
	$(CC) $(CFLAGS_RELEASE) -c -o $@ $^

obj/debug/%.o: src/%.c | obj/debug notes
	$(CC) $(CFLAGS_DEBUG) -c $^ -o notes/$*.o
	mv notes/$*.o $@

obj/test/%.o: src/test/%.c | obj/test
	$(CC) $(TEST_CFLAGS) -c $^ -o $@

obj/bin/%.o: src/bin/%.c | obj/bin
	$(CC) $(CFLAGS_RELEASE) -c -o $@ $^

.PHONY:
binaries: $(BINARIES)

.PHONY:
test: $(TEST_BINARIES)
	for test in $^ ; do ./$$test; done

.PHONY:
coverage: test
	lcov -t "result" -o cov.info -c -d notes
	genhtml -o cov cov.info

clean:
	rm -rf bin obj notes cov

