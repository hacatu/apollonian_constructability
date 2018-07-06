CCXX := clang++
LDXX := clang++
ARXX := ar
CXXFLAGS_RELEASE := -Wall -std=c++17 -Isrc -Ofast -ffast-math -mtune=native -march=native
#CXXFLAGS_RELEASE := -Wall -std=c++17 -Isrc -g -pthread
LDXXFLAGS_RELEASE := -Ofast -ffast-math -mtune=native -march=native -flto -fuse-ld=gold -lm -lSDL2 -lSDL2_gfx
#LDXXFLAGS_RELEASE := -g -flto -fuse-ld=gold -lm -lSDL2 -lSDL2_gfx
#CXXFLAGS_DEBUG := -Wall -std=c++17 -Ofast -ffast-math -mtune=native -march=native
CXXFLAGS_DEBUG := -Wall -std=c++17 -g3 --coverage
#LDXXFLAGS_DEBUG := -Ofast -ffast-math -mtune=native -march=native -flto -fuse-ld=gold -lm
LDXXFLAGS_DEBUG := --coverage -lm
TEST_CXXFLAGS := -Wall -std=c++17 -g3 -Isrc
TEST_LDXXFLAGS := --coverage -lm
BUILD_ROOT := $(shell pwd)
SOURCES_XX := $(shell find src -maxdepth 1 -name '*.cxx')
TEST_SOURCES_XX := $(shell find src/test -maxdepth 1 -name '*.cxx')
BINARY_SOURCES_XX := $(shell find src/binary -maxdepth 1 -name '*.cxx')
OBJS_DEBUG := $(patsubst src/%.cxx,obj/debug/%.o,$(SOURCES_XX))
OBJS_RELEASE := $(patsubst src/%.cxx,obj/release/%.o,$(SOURCES_XX))
TEST_BINARIES := $(patsubst src/test/%.cxx,bin/test/%,$(TEST_SOURCES_XX))
BINARIES := $(patsubst src/binary/%.cxx,bin/%,$(BINARY_SOURCES_XX))

all: coverage binaries | Makefile

.SECONDARY:
obj/bin obj/test obj/debug obj/release bin bin/test notes:
	mkdir -p $@

bin/%: obj/bin/%.o $(OBJS_RELEASE) | bin
	$(LDXX) $(LDXXFLAGS_RELEASE) -o $@ $^

bin/test/%: obj/test/%.o $(OBJS_DEBUG) | bin/test 
	$(LDXX) $(TEST_LDXXFLAGS) -o $@ $^

obj/release/%.o: src/%.cxx | obj/release
	$(CCXX) $(CXXFLAGS_RELEASE) -c -o $@ $^

obj/debug/%.o: src/%.cxx | obj/debug notes
	$(CCXX) $(CXXFLAGS_DEBUG) -c $^ -o notes/$*.o
	mv notes/$*.o $@

obj/test/%.o: src/test/%.cxx | obj/test
	$(CCXX) $(TEST_CXXFLAGS) -c $^ -o $@

obj/bin/%.o: src/binary/%.cxx | obj/bin
	$(CCXX) $(CXXFLAGS_RELEASE) -c -o $@ $^

.PHONY:
binaries: $(BINARIES)

.PHONY:
test: $(TEST_BINARIES)
	for test in $^ ; do ./$$test; done

.PHONY:
coverage: test
	-lcov -t "result" -o cov.info -c -d notes
	-genhtml -o cov cov.info

clean:
	rm -rf bin obj notes cov

