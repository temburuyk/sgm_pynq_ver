#petalinux has to be sourced first
#do not uncomment
# source /opt/Xilinx/petalinux-v2017.1/settings.sh

# APP_NAME=elp_with_cpp_class
APP_NAME=zed_capture_5sec


.PHONY: $(APP_NAME).elf

$(APP_NAME).elf: $(APP_NAME).cpp
	#compile
	arm-linux-gnueabihf-g++ -Wall -O3 -g3 -c -fmessage-length=0 -fopenmp -o $(APP_NAME).o -I/home/mandardatar/local_install/opencv/build/install/include/opencv4 $(APP_NAME).cpp
	#link
	arm-linux-gnueabihf-g++ -O3 -Wl,-rpath=/mnt/lib -fopenmp -o $(APP_NAME).elf $(APP_NAME).o -L/home/mandardatar/local_install/opencv/build/install/lib -lopencv_calib3d -lopencv_core  -lopencv_features2d -lopencv_flann -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_videoio -lopencv_video
	#scp -r -o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null $(APP_NAME).elf root@10.107.88.118:/mnt/ 

clean:
	rm $(APP_NAME).elf $(APP_NAME).o
