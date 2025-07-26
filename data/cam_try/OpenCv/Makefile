# Makefile for Camera Acquisition Project
# Usage: 
#   make           # build the project
#   make clean     # clean up build artifacts

CXX ?= g++
SRC = CameraAcquisition.cpp
OUT = CameraAcquisition
INCLUDE = -I/usr/include/opencv4 -I../../include
LIBS = -L/usr/lib/x86_64-linux-gnu -lopencv_core -lopencv_imgproc -lopencv_highgui -lMVSDK

all: $(OUT)

$(OUT): $(SRC)
	$(CXX) $(INCLUDE) -o $(OUT) $(SRC) $(LIBS)

clean:
	rm -f *.o 
	rm -f $(OUT)
