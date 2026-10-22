# Makefile for NeTrainSim
# Usage:
#   make linux      - Build for Linux
#   make mac        - Build for macOS
#   make clean      - Clean all builds
#   make clean-linux - Clean Linux build only
#   make clean-mac   - Clean macOS build only
#   make run-linux   - Build and run Linux with customizable parameters
#   make run-mac     - Build and run macOS with customizable parameters
#
# Parameter examples:
#   make run-linux NODES=custom/nodes.dat LINKS=custom/links.dat
#   make run-linux EXTRA_ARGS="--verbose --seed 42"
#   make run-mac OUTPUT=results EXTRA_ARGS="--debug"

.PHONY: linux mac clean clean-linux clean-mac run-linux run-mac help

# Detect OS for default target
UNAME_S := $(shell uname -s)
DEFAULT_TARGET := $(shell echo $(UNAME_S) | tr '[:upper:]' '[:lower:]')

# Default parameter values (can be overridden on command line)
NODES    ?= src/data/sampleProject/nodesFile.dat
LINKS    ?= src/data/sampleProject/linksFile.dat
TRAIN    ?= src/data/sampleProject/dieselTrain.dat
OUTPUT   ?= res
EXTRA_ARGS ?=

# Default target
help:
	@echo "NeTrainSim Makefile"
	@echo ""
	@echo "Build targets:"
	@echo "  make linux          - Build for Linux"
	@echo "  make mac            - Build for macOS"
	@echo ""
	@echo "Run targets:"
	@echo "  make run-linux      - Build and run Linux simulation"
	@echo "  make run-mac        - Build and run macOS simulation"
	@echo ""
	@echo "Run parameters (can be overridden):"
	@echo "  NODES=path          - Nodes file (default: $(NODES))"
	@echo "  LINKS=path          - Links file (default: $(LINKS))"
	@echo "  TRAIN=path          - Train file (default: $(TRAIN))"
	@echo "  OUTPUT=path         - Output directory (default: $(OUTPUT))"
	@echo "  EXTRA_ARGS=\"args\"   - Additional arguments to pass"
	@echo ""
	@echo "Examples:"
	@echo "  make run-linux NODES=test.nodes OUTPUT=test_results"
	@echo "  make run-linux EXTRA_ARGS=\"--verbose --seed 123\""
	@echo "  make run-mac TRAIN=custom.train EXTRA_ARGS=\"--debug\""
	@echo ""
	@echo "Clean targets:"
	@echo "  make clean          - Clean all builds"
	@echo "  make clean-linux    - Clean Linux build only"
	@echo "  make clean-mac      - Clean macOS build only"
	@echo ""

# Linux build
linux:
	@echo "Building for Linux..."
	cmake -B build-linux -DCMAKE_BUILD_TYPE=Release \
		-DBUILD_GUI=OFF -DBUILD_SERVER=OFF
	cmake --build build-linux --target NeTrainSimConsole -j$(shell nproc)
	@echo "Linux build complete!"

# macOS build
mac:
	@echo "Building for macOS..."
	cmake -B build-mac -DCMAKE_BUILD_TYPE=Release \
		-DBUILD_GUI=OFF -DBUILD_SERVER=OFF \
		-DCMAKE_PREFIX_PATH=$(shell brew --prefix qt6 2>/dev/null || echo "")
	cmake --build build-mac --target NeTrainSimConsole -j$(shell sysctl -n hw.logicalcpu)
	@echo "macOS build complete!"

# Run Linux build with customizable parameters
run-linux: linux
	@mkdir -p $(OUTPUT)
	@echo "========================================="
	@echo "Running NeTrainSim on Linux..."
	@echo "  Nodes:  $(NODES)"
	@echo "  Links:  $(LINKS)"
	@echo "  Train:  $(TRAIN)"
	@echo "  Output: $(OUTPUT)"
	@if [ -n "$(EXTRA_ARGS)" ]; then echo "  Extra:  $(EXTRA_ARGS)"; fi
	@echo "========================================="
	./build-linux/src/NeTrainSimConsole/NeTrainSim \
		-n $(NODES) \
		-l $(LINKS) \
		-t $(TRAIN) \
		-o $(OUTPUT) \
		$(EXTRA_ARGS)
	@echo "========================================="
	@echo "Simulation complete! Results in $(OUTPUT)/"
	@echo "========================================="

# Run macOS build with customizable parameters
run-mac: mac
	@mkdir -p $(OUTPUT)
	@echo "========================================="
	@echo "Running NeTrainSim on macOS..."
	@echo "  Nodes:  $(NODES)"
	@echo "  Links:  $(LINKS)"
	@echo "  Train:  $(TRAIN)"
	@echo "  Output: $(OUTPUT)"
	@if [ -n "$(EXTRA_ARGS)" ]; then echo "  Extra:  $(EXTRA_ARGS)"; fi
	@echo "========================================="
	./build-mac/src/NeTrainSimConsole/NeTrainSim \
		-n $(NODES) \
		-l $(LINKS) \
		-t $(TRAIN) \
		-o $(OUTPUT) \
		$(EXTRA_ARGS)
	@echo "========================================="
	@echo "Simulation complete! Results in $(OUTPUT)/"
	@echo "========================================="

# Clean specific builds
clean-linux:
	@echo "Cleaning Linux build..."
	rm -rf build-linux
	@echo "Linux build cleaned."

clean-mac:
	@echo "Cleaning macOS build..."
	rm -rf build-mac
	@echo "macOS build cleaned."

# Clean all builds
clean: clean-linux clean-mac
	rm -rf $(OUTPUT)
	@echo "All builds and results cleaned."

# Smart default based on detected OS
ifeq ($(UNAME_S),Linux)
default: run-linux
else ifeq ($(UNAME_S),Darwin)
default: run-mac
else
default: help
endif
