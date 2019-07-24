from pynq import Overlay
from pynq import Xlnk
import numpy as np
import cv2
import time

IMG_WIDTH         =640
IMG_HEIGHT        =480
FILTER_SIZE     =9
FILTER_OFFS     =(int)(FILTER_SIZE/2)
SECTIONS        =2
SECTION_HEIGHT  =(int)((IMG_HEIGHT-2*FILTER_OFFS)/SECTIONS)
DISP_IMG_HEIGHT =SECTIONS*SECTION_HEIGHT
BYTES_PER_PIXEL =1
TOTAL_BYTES     =DISP_IMG_HEIGHT*IMG_WIDTH*BYTES_PER_PIXEL
ADDRESS_OFFSET  =int(TOTAL_BYTES/SECTIONS)
image_size = int(IMG_WIDTH*IMG_HEIGHT)

left_rmap = np.fromfile("/home/xilinx/sgm_pynq_ver/elp_capture/elp640_left_rmap.bin",dtype=np.ubyte,count=-1,sep='')
right_rmap = np.fromfile("/home/xilinx/sgm_pynq_ver/elp_capture/elp640_right_rmap.bin",dtype=np.ubyte,count=-1,sep='')

overlay = Overlay('/home/xilinx/sgm_pynq_ver/backup_bitstream2/design_1.bit')
overlay

capl = cv2.VideoCapture(0)
capr = cv2.VideoCapture(1)

print(overlay.ip_dict.keys())

sgm_optim_0 = overlay.sgm_optim_0
sgm_optim_1 = overlay.sgm_optim_1
remap_hls_0 = overlay.remap_hls_0
remap_hls_1 = overlay.remap_hls_1

#sgm offsets
inL_offs = sgm_optim_0.register_map.inL.address
inR_offs = sgm_optim_0.register_map.inR.address
outD_offs = sgm_optim_0.register_map.outD.address
CTRL_reg_offset = sgm_optim_0.register_map.CTRL.address

#remap offsets
map_data_V = remap_hls_0.register_map.map_data_V.address
pin_V = remap_hls_0.register_map.pin_V.address
pout_V = remap_hls_0.register_map.pout_V.address

xlnk = Xlnk()

Image_buf =  xlnk.cma_alloc(0x1000000, data_type = "unsigned char")

Image_buf_phy_addr = xlnk.cma_get_phy_addr(Image_buf)

ret, framel = capl.read()
ret, framer = capr.read()
test1 = cv2.cvtColor(framel, cv2.COLOR_BGR2GRAY).flatten()
test2 = cv2.cvtColor(framer, cv2.COLOR_BGR2GRAY).flatten()
Image_buf[0:image_size] = test1[0:image_size]                       #left raw image
Image_buf[image_size:image_size*2] = test2[0:image_size]            #right raw image
Image_buf[image_size*2:image_size*6]= left_rmap[0:image_size*4]     #left cam rec map
Image_buf[image_size*6:image_size*10]= right_rmap[0:image_size*4]     #right cam rec map

remap_hls_0.write(map_data_V,Image_buf_phy_addr+2*image_size)
remap_hls_1.write(map_data_V,Image_buf_phy_addr+6*image_size)
remap_hls_0.write(pin_V,Image_buf_phy_addr+0*image_size)
remap_hls_0.write(pout_V,Image_buf_phy_addr+10*image_size)
remap_hls_1.write(pin_V,Image_buf_phy_addr+1*image_size)
remap_hls_1.write(pout_V,Image_buf_phy_addr+11*image_size)

sgm_optim_0.write(inL_offs,Image_buf_phy_addr+10*image_size)
sgm_optim_0.write(inR_offs,Image_buf_phy_addr+11*image_size)
sgm_optim_0.write(outD_offs,Image_buf_phy_addr+12*image_size)
sgm_optim_1.write(inL_offs,Image_buf_phy_addr+10*image_size+ADDRESS_OFFSET)
sgm_optim_1.write(inR_offs,Image_buf_phy_addr+11*image_size+ADDRESS_OFFSET)
sgm_optim_1.write(outD_offs,Image_buf_phy_addr+12*image_size+ADDRESS_OFFSET)

sgm_optim_0.write(CTRL_reg_offset,0b00000001)
sgm_optim_1.write(CTRL_reg_offset,0b00000001)
remap_hls_0.write(CTRL_reg_offset,0b00000001)
remap_hls_1.write(CTRL_reg_offset,0b00000001)

count = 0
sgm0_done = False
sgm1_done = False
start_time = time.time()
while(not(sgm0_done) or not(sgm1_done)):
	sgm0_done = sgm0_done or sgm_optim_0.register_map.CTRL.AP_DONE
	sgm1_done = sgm1_done or sgm_optim_1.register_map.CTRL.AP_DONE
	#print(count)
	#count = count + 1
	pass

print("--- %s seconds ---" % (time.time() - start_time))

raw_iml_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
raw_imr_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
rec_iml_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
rec_imr_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
disp_im_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)

for i in range(IMG_HEIGHT):
    for j in range(IMG_WIDTH):
            disp_im_buffer[i][j] = Image_buf[12*image_size+i*IMG_WIDTH+j]
            raw_imr_buffer[i][j] = Image_buf[1*image_size+i*IMG_WIDTH+j]
            raw_iml_buffer[i][j] = Image_buf[0*image_size+i*IMG_WIDTH+j]
            rec_imr_buffer[i][j] = Image_buf[11*image_size+i*IMG_WIDTH+j]
            rec_iml_buffer[i][j] = Image_buf[10*image_size+i*IMG_WIDTH+j]

cv2.imwrite('/home/xilinx/sgm_pynq_ver/raw_iml_buffer.png',raw_iml_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/raw_imr_buffer.png',raw_imr_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/rec_iml_buffer.png',rec_iml_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/rec_imr_buffer.png',rec_imr_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/disp_im_buffer.png',disp_im_buffer)



Xlnk.cma_free(Image_buf)