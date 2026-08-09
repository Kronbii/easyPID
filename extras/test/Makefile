# Host-side tests for easyPID.
#
# These compile the library against a mock Arduino.h with a virtual clock, so
# the control behaviour can be exercised deterministically without hardware.
# Nothing here is compiled by the Arduino IDE: everything under extras/ is
# ignored by the Arduino library specification.
#
#   make           build and run both suites
#   make test      same
#   make clean     remove build artefacts

CXX      ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Wpedantic -O1
SRCDIR    = ../../src
INCLUDES  = -I mock -I $(SRCDIR)
LIBSRC    = $(SRCDIR)/easyPID.cpp $(SRCDIR)/PIDTuner.cpp

BINARIES  = test_easypid test_examples

.PHONY: all test clean
all: test

test: $(BINARIES)
	@echo "--- library behaviour ---"
	@./test_easypid
	@echo
	@echo "--- example constants ---"
	@./test_examples

test_easypid: test_easypid.cpp $(LIBSRC) mock/Arduino.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) test_easypid.cpp $(LIBSRC) -o $@

test_examples: test_examples.cpp $(LIBSRC) mock/Arduino.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) test_examples.cpp $(LIBSRC) -o $@

clean:
	rm -f $(BINARIES)
