CC := gcc
LD := gcc
AR := ar
CFLAGS_RELEASE := -Wall -std=c99 -Isrc -Ofast -ffast-math -mtune=native -march=native
LDFLAGS_RELEASE := -Ofast -ffast-math -mtune=native -march=native -flto -fuse-ld=gold -lm
CFLAGS_DEBUG := -Wall -std=c99 -g3 --coverage
LDFLAGS_DEBUG := --coverage -lm
TEST_CFLAGS := -Wall -std=c99 -g3 -Isrc
TEST_LDFLAGS := --coverage -lm
BUILD_ROOT := $(shell pwd)
SOURCES := $(shell find src -maxdepth 1 -name '*.c')
TEST_SOURCES := $(shell find src/test -maxdepth 1 -name '*.c')
BINARY_SOURCES := $(shell find src/bin -maxdepth 1 -name '*.c')
OBJS_DEBUG := $(patsubst src/%.c,obj/debug/%.o,$(SOURCES))
OBJS_RELEASE := $(patsubst src/%.c,obj/release/%.o,$(SOURCES))
TEST_BINARIES := $(patsubst src/test/%.c,bin/test/%,$(TEST_SOURCES))
BINARIES := $(patsubst src/bin/%.c,bin/%,$(BINARY_SOURCES))

all: coverage $(BINARIES) | Makefile

.SECONDARY:
obj/bin obj/test obj/debug obj/release bin bin/test notes:
	mkdir -p $@

bin/test/%: obj/test/%.o $(OBJS_DEBUG) | bin/test 
	$(CC) $(TEST_LDFLAGS) -o $@ $^

bin/%: objs/bin/%.o $(OBJ_RELEASE) | bin
	$(CC) $(LDFLAGS_RELEASE) -o $@ $^

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
test: $(TEST_BINARIES)
	for test in $^ ; do ./$$test; done

.PHONY:
coverage: test
	lcov -t "result" -o cov.info -c -d notes
	genhtml -o cov cov.info

clean:
	rm -rf bin obj notes cov

