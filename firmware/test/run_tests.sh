#!/bin/sh
# Builds and runs the PC unit tests (needs g++). Run from anywhere.
set -e
cd "$(dirname "$0")/.."
g++ -std=c++17 -Wall -Wextra -O1 \
  -Ilibraries/PlacaProtocol/src -Ireceiver \
  test/test_all.cpp libraries/PlacaProtocol/src/PlacaProtocol.cpp receiver/supervisor.cpp \
  -o test/test_all.exe
./test/test_all.exe
