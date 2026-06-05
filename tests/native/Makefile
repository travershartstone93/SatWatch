CXX = g++
CXXFLAGS = -std=c++17 -O2 -Wall -I.

all: test_pipeline

test_pipeline: test_pipeline.cpp stb_decode.cpp mock_esp32.h ../../esp32-LiveSat-s3/stb_image.h
	$(CXX) $(CXXFLAGS) -o $@ test_pipeline.cpp stb_decode.cpp

clean:
	rm -f test_pipeline test_*.ppm /tmp/native_test_dl.bin

test: test_pipeline
	./test_pipeline

.PHONY: all clean test
