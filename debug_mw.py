# import struct
# import sys


# for i in range(IMG_HEIGHT):
#     for j in range(IMG_WIDTH):
#             test1[i*IMG_WIDTH+j] = Image_buf[640*480*10+i*IMG_WIDTH+j]

# cv2.imwrite('/home/xilinx/sgm_pynq_ver/rec_iml_buffer.png',test1.reshape(IMG_HEIGHT,IMG_WIDTH))


# empty = cv2.imread('/home/xilinx/sgm_pynq_ver/test_images/empty.png', 0)

# for i in range(IMG_HEIGHT):
#     for j in range(IMG_WIDTH):
#             empty[i][j] = Image_buf[640*480*2+i*IMG_WIDTH+j]

# cv2.imwrite('/home/xilinx/sgm_pynq_ver/disp_im_buffer.png',empty)


# f=open(sys.argv[2],"wb")

# for i in range(IMG_HEIGHT) :
# 	for j in range(IMG_WIDTH):
# 		byte=struct.pack('B',image[i,j])
# 		f.write(byte)

# f.close()

import cv2
import ctypes
from pynq import Overlay
from pynq import Xlnk
import numpy as np

xlnk = Xlnk()

Image_buf =  xlnk.cma_alloc(0x000100, data_type = "unsigned char")
Image_buf_phy_addr = xlnk.cma_get_phy_addr(Image_buf)

Image_buf_vir_addr = id(Image_buf)

#ctypes.cast(Image_buf_vir_addr, ctypes.py_object).value

#POINTER(c_ubyte)

imagel = cv2.imread('/home/xilinx/sgm_pynq_ver/test_images/teddy_left.ppm', 0);

test1 = np.zeros([16,16],np.ubyte)

count = 0
while(count < 10000):
	print(count)
	count = count + 1
	for i in range(16):
		for j in range(16):
			Image_buf[i*16 + j] = (i*j + (count*3))%200
			test1[i][j] = Image_buf[i*16 + j]
	cv2.imwrite('local.png',test1)
	cv2.waitKey(100)

#cv2.imshow('image',imagel)
cv2.waitKey(0)
cv2.destroyAllWindows()



import cffi

ffi = cffi.FFI()
test_buf =  xlnk.cma_alloc(0x0A, data_type = "unsigned char")
buffer = ffi.buffer(test_buf,10)  #converts c data object to python memory readable object
array = np.frombuffer(buffer, dtype=np.ubyte).reshape((2,5)) # converts the buffer to np object which access the same memory

Disp_time1 = time.time()
Image_buf[0:image_size] = test1[0:image_size]                       #left raw image
Image_buf[image_size:image_size*2] = test2[0:image_size]            #right raw image
Data_time = time.time() - Disp_time1 
print("Data transfer time ",Data_time)

Disp_time1 = time.time()
np.copyto(rec_iml_buffer, disp_im_buffer)
Data_time = time.time() - Disp_time1 
print("Data transfer time ",Data_time)

