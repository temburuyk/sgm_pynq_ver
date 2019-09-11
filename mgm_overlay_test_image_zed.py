from pynq import Overlay
from pynq import Xlnk
import numpy as np
import cv2
import struct
import sys
import time

IMG_WIDTH         =640
IMG_HEIGHT        =480
FILTER_SIZE     =9
FILTER_OFFS     =(int)(FILTER_SIZE/2)
SECTIONS        =1
SECTION_HEIGHT  =(int)((IMG_HEIGHT-2*FILTER_OFFS)/SECTIONS)
DISP_IMG_HEIGHT =SECTIONS*SECTION_HEIGHT
BYTES_PER_PIXEL =1
TOTAL_BYTES     =DISP_IMG_HEIGHT*IMG_WIDTH*BYTES_PER_PIXEL
ADDRESS_OFFSET  =int(TOTAL_BYTES/SECTIONS)
image_size = int(IMG_WIDTH*IMG_HEIGHT)
ZED_IMAGE_WIDTH   =1344
ZED_IMAGE_WIDTH_2 = int(ZED_IMAGE_WIDTH/2)
ZED_IMAGE_HEIGHT  =376

#overlay = Overlay('/home/xilinx/sgm_pynq_ver/census_mgm_multi_bit/design_1.bit')
overlay = Overlay('/home/xilinx/sgm_pynq_ver/census_mgm_bit/design_1.bit')
overlay

image = cv2.imread('/home/xilinx/sgm_pynq_ver/test_images/right_image32.jpg', 0);
imagel = np.zeros((ZED_IMAGE_HEIGHT,ZED_IMAGE_WIDTH),dtype=np.ubyte)
imager = np.zeros((ZED_IMAGE_HEIGHT,ZED_IMAGE_WIDTH),dtype=np.ubyte)
rectified_left = np.zeros((ZED_IMAGE_HEIGHT,ZED_IMAGE_WIDTH),dtype=np.ubyte)
rectified_right = np.zeros((ZED_IMAGE_HEIGHT,ZED_IMAGE_WIDTH),dtype=np.ubyte)
buffer_left = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
buffer_right = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
imagel[0:ZED_IMAGE_HEIGHT,0:ZED_IMAGE_WIDTH_2] = image[0:ZED_IMAGE_HEIGHT,0:ZED_IMAGE_WIDTH_2]
imager[0:ZED_IMAGE_HEIGHT,0:ZED_IMAGE_WIDTH_2] = image[0:ZED_IMAGE_HEIGHT,ZED_IMAGE_WIDTH_2:ZED_IMAGE_WIDTH]


# imagel[0:ZED_IMAGE_HEIGHT,0:640] = image[0:ZED_IMAGE_HEIGHT,0:640]
# imager[0:ZED_IMAGE_HEIGHT,0:640] = image[0:ZED_IMAGE_HEIGHT,672:1312]

fs_left = cv2.FileStorage("/home/xilinx/sgm_pynq_ver/zed_camera/zed_left_calibration.yml", cv2.FILE_STORAGE_READ)
fs_right = cv2.FileStorage("/home/xilinx/sgm_pynq_ver/zed_camera/zed_right_calibration.yml", cv2.FILE_STORAGE_READ)
left_rmap0 = fs_left.getNode("rmap0").mat()
left_rmap1 = fs_left.getNode("rmap1").mat()
right_rmap0 = fs_right.getNode("rmap0").mat()
right_rmap1 = fs_right.getNode("rmap1").mat()



print(overlay.ip_dict.keys())

SGM_GreyCost_0 = overlay.SGM_GreyCost_0
SGM_GreyCost_1 = overlay.SGM_GreyCost_1
SGM_GreyCost_2 = overlay.SGM_GreyCost_2
SGM_GreyCost_3 = overlay.SGM_GreyCost_3
SGM_GreyCost_4 = overlay.SGM_GreyCost_4
##sgm_optim_1 = overlay.#sgm_optim_1
remap_hls_0 = overlay.remap_hls_0
remap_hls_1 = overlay.remap_hls_1

#sgm offsets
inL_offs = SGM_GreyCost_0.register_map.inL.address
inR_offs = SGM_GreyCost_0.register_map.inR.address
outD_offs = SGM_GreyCost_0.register_map.outD.address
CTRL_reg_offset = SGM_GreyCost_0.register_map.CTRL.address

#remap offsets
map_data_V = remap_hls_0.register_map.map_data_V.address
pin_V = remap_hls_0.register_map.pin_V.address
pout_V = remap_hls_0.register_map.pout_V.address

xlnk = Xlnk()

Image_buf =  xlnk.cma_alloc(0x0A00000, data_type = "unsigned char")

Image_buf_phy_addr = xlnk.cma_get_phy_addr(Image_buf)


rectified_left = cv2.remap (imagel, left_rmap0, left_rmap1, cv2.INTER_LINEAR)
rectified_right = cv2.remap (imager, right_rmap0, right_rmap1, cv2.INTER_LINEAR)

buffer_left[0:ZED_IMAGE_HEIGHT,0:640] = rectified_left[0:ZED_IMAGE_HEIGHT,0:IMG_WIDTH]
buffer_right[0:ZED_IMAGE_HEIGHT,0:640] = rectified_right[0:ZED_IMAGE_HEIGHT,0:IMG_WIDTH]

test1 = buffer_left.flatten()
test2 = buffer_right.flatten()

Image_buf[0:image_size] = test1[0:image_size]                       #left raw image
Image_buf[image_size:image_size*2] = test2[0:image_size]            #right raw image

remap_hls_0.write(map_data_V,Image_buf_phy_addr+2*image_size)
remap_hls_1.write(map_data_V,Image_buf_phy_addr+6*image_size)
remap_hls_0.write(pin_V,Image_buf_phy_addr+0*image_size)
remap_hls_0.write(pout_V,Image_buf_phy_addr+10*image_size)
remap_hls_1.write(pin_V,Image_buf_phy_addr+1*image_size)
remap_hls_1.write(pout_V,Image_buf_phy_addr+11*image_size)

SGM_GreyCost_0.write(inL_offs,Image_buf_phy_addr+0*image_size)
SGM_GreyCost_0.write(inR_offs,Image_buf_phy_addr+1*image_size)
SGM_GreyCost_0.write(outD_offs,Image_buf_phy_addr+12*image_size)
SGM_GreyCost_1.write(inL_offs,Image_buf_phy_addr+10*image_size + ADDRESS_OFFSET)
SGM_GreyCost_1.write(inR_offs,Image_buf_phy_addr+11*image_size + ADDRESS_OFFSET)
SGM_GreyCost_1.write(outD_offs,Image_buf_phy_addr+12*image_size + ADDRESS_OFFSET)
SGM_GreyCost_2.write(inL_offs,Image_buf_phy_addr+10*image_size + 2*ADDRESS_OFFSET)
SGM_GreyCost_2.write(inR_offs,Image_buf_phy_addr+11*image_size + 2*ADDRESS_OFFSET)
SGM_GreyCost_2.write(outD_offs,Image_buf_phy_addr+12*image_size + 2*ADDRESS_OFFSET)
SGM_GreyCost_3.write(inL_offs,Image_buf_phy_addr+10*image_size + 3*ADDRESS_OFFSET)
SGM_GreyCost_3.write(inR_offs,Image_buf_phy_addr+11*image_size + 3*ADDRESS_OFFSET)
SGM_GreyCost_3.write(outD_offs,Image_buf_phy_addr+12*image_size + 3*ADDRESS_OFFSET)
SGM_GreyCost_4.write(inL_offs,Image_buf_phy_addr+10*image_size + 4*ADDRESS_OFFSET)
SGM_GreyCost_4.write(inR_offs,Image_buf_phy_addr+11*image_size + 4*ADDRESS_OFFSET)
SGM_GreyCost_4.write(outD_offs,Image_buf_phy_addr+12*image_size + 4*ADDRESS_OFFSET)


SGM_GreyCost_4.write(CTRL_reg_offset,0b00000001)
SGM_GreyCost_3.write(CTRL_reg_offset,0b00000001)
SGM_GreyCost_2.write(CTRL_reg_offset,0b00000001)
SGM_GreyCost_1.write(CTRL_reg_offset,0b00000001)
SGM_GreyCost_0.write(CTRL_reg_offset,0b00000001)

count = 0
sad0_done = False
sgm1_done = False
start_time = time.time()
while(not(sad0_done)):
	sad0_done = sad0_done or SGM_GreyCost_0.register_map.CTRL.AP_DONE
	#sgm1_done = sgm1_done or sgm_optim_1.register_map.CTRL.AP_DONE
	#print(count)
	#count = count + 1
	#pass

print("--- %s seconds ---" % (time.time() - start_time))


raw_iml_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
raw_imr_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
rec_iml_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
rec_imr_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
disp_im_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)


# while(count < 5000):
# 	print(count)
# 	count = count +1
# 	for i in range(IMG_HEIGHT):
# 		for j in range(IMG_WIDTH):
# 			disp_im_buffer[i][j] = Image_buf[12*image_size+i*IMG_WIDTH+j]
# 	cv2.imwrite('/home/xilinx/sgm_pynq_ver/output_images/disp_im_buffer.png',disp_im_buffer)
# 	Image_buf[0:image_size] = test1[0:image_size]                       #left raw image
# 	Image_buf[image_size:image_size*2] = test2[0:image_size]            #right raw image

for i in range(IMG_HEIGHT):
	for j in range(IMG_WIDTH):
			disp_im_buffer[i][j] = Image_buf[12*image_size+i*IMG_WIDTH+j]
			raw_imr_buffer[i][j] = Image_buf[1*image_size+i*IMG_WIDTH+j]
			raw_iml_buffer[i][j] = Image_buf[0*image_size+i*IMG_WIDTH+j]
			rec_imr_buffer[i][j] = Image_buf[11*image_size+i*IMG_WIDTH+j]
			rec_iml_buffer[i][j] = Image_buf[10*image_size+i*IMG_WIDTH+j]

cv2.imwrite('/home/xilinx/sgm_pynq_ver/output_images/raw_iml_buffer.png',raw_iml_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/output_images/raw_imr_buffer.png',raw_imr_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/output_images/rec_iml_buffer.png',rec_iml_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/output_images/rec_imr_buffer.png',rec_imr_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/output_images/disp_im_buffer.png',disp_im_buffer)



xlnk.cma_free(Image_buf)

