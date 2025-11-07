# ============================================================================
# Makefile for RobotLib Simulation Examples
# ============================================================================
# Usage:
#   make              - Build all examples
#   make basic        - Build basic simulation
#   make pid          - Build PID navigation
#   make line         - Build line follower
#   make clean        - Remove all built files
#   make run-basic    - Build and run basic simulation
# ============================================================================

CXX = g++
CXXFLAGS = -std=c++11 -Wall -Wextra -I../include
LDFLAGS = -lSDL2

# Detect OS and adjust SDL2 flags
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
    # macOS
    LDFLAGS += -L/usr/local/lib
    CXXFLAGS += -I/usr/local/include
endif

EXAMPLES = basic_simulation pid_navigation line_follower

.PHONY: all clean help run-basic run-pid run-line

all: $(EXAMPLES)

basic_simulation: 01_basic_simulation.cpp
	@echo "Building basic simulation..."
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)
	@echo "✓ Built: $@"

pid_navigation: 02_pid_navigation.cpp
	@echo "Building PID navigation..."
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)
	@echo "✓ Built: $@"

line_follower: 03_line_follower.cpp
	@echo "Building line follower..."
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)
	@echo "✓ Built: $@"

# Shortcuts
basic: basic_simulation
pid: pid_navigation
line: line_follower

# Run targets
run-basic: basic_simulation
	@echo "Running basic simulation..."
	./basic_simulation

run-pid: pid_navigation
	@echo "Running PID navigation..."
	./pid_navigation

run-line: line_follower
	@echo "Running line follower..."
	./line_follower

clean:
	@echo "Cleaning build files..."
	rm -f $(EXAMPLES)
	@echo "✓ Clean complete"

help:
	@echo "RobotLib Simulation Examples"
	@echo ""
	@echo "Targets:"
	@echo "  make              - Build all examples"
	@echo "  make basic        - Build basic simulation"
	@echo "  make pid          - Build PID navigation"
	@echo "  make line         - Build line follower"
	@echo "  make clean        - Remove all built files"
	@echo "  make run-basic    - Build and run basic simulation"
	@echo "  make run-pid      - Build and run PID navigation"
	@echo "  make run-line     - Build and run line follower"
	@echo ""
	@echo "Requirements:"
	@echo "  - C++11 compiler"
	@echo "  - SDL2 library"
	@echo ""
	@echo "Install SDL2:"
	@echo "  Ubuntu/Debian: sudo apt-get install libsdl2-dev"
	@echo "  macOS:         brew install sdl2"
	@echo "  Windows:       Download from libsdl.org"
