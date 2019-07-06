SRCFILE:=zed_capture.cpp
TARGET:=zed_capture

ARM_COMPILE:=1

ifdef ARM_COMPILE
CXX:=arm-linux-gnueabihf-g++
INC_PATH:=$(HOME)/local_install/opencv/build/install/include/opencv4
LIB_PATH:=$(HOME)/local_install/opencv/build/install/lib
CXXFLAGS:=-std=c++11
else
CXX:=g++-5
INC_PATH:=$(HOME)/local_install/opencv/debug_x86/include/opencv4
LIB_PATH:=$(HOME)/local_install/opencv/debug_x86/lib
CXXFLAGS:=-std=c++11
endif

# ( cd $(HOME)/local_install/opencv/debug_install_x86/lib && ls *.so | sed -e 's/libopencv_/-lopencv_/' -e 's/\.so$//' | tr '\n' ' ' )
OPENCV_LIBS=-lopencv_calib3d -lopencv_core -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_videoio -lopencv_video

all: $(TARGET)

$(TARGET): $(SRCFILE)
	$(CXX) -g $(CXXFLAGS) $< -I $(INC_PATH) -L $(LIB_PATH) $(OPENCV_LIBS) -o $@ -Wl,-rpath=$(LIB_PATH)

clean:
	rm -f $(TARGET)
