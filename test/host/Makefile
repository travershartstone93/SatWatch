CXX = g++
CXXFLAGS = -std=c++17 -Wall -I../../esp32-LiveSat-s3
SRCS = test_main.cpp ../../esp32-LiveSat-s3/sat_logic.cpp
TARGET = run_tests

all: $(TARGET)
	./$(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f $(TARGET)
