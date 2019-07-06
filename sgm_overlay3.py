from pynq import Overlay
from pynq import Xlnk
import numpy as np
import cv2

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

overlay = Overlay('/home/xilinx/sgm_pynq_ver/design_1.bit')
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
pin_V = remap_hls_0.register_map.pin_V.address
pout_V = remap_hls_0.register_map.pout_V.address

xlnk = Xlnk()

Image_buf =  xlnk.cma_alloc(0x1000000, data_type = "unsigned char")

Image_buf_phy_addr = xlnk.cma_get_phy_addr(Image_buf)

test1 = cv2.cvtColor(framel, cv2.COLOR_BGR2GRAY).flatten()
test2 = cv2.cvtColor(framer, cv2.COLOR_BGR2GRAY).flatten()
Image_buf[0:image_size] = test1[0:image_size]
Image_buf[image_size:image_size*2] = test2[0:image_size]

remap_hls_0.write(pin_V,Image_buf_phy_addr+3*image_size)
remap_hls_0.write(pout_V,Image_buf_phy_addr+4*image_size)
remap_hls_1.write(pin_V,Image_buf_phy_addr+5*image_size)
remap_hls_1.write(pout_V,Image_buf_phy_addr+6*image_size)

sgm_optim_0.write(inL_offs,Image_buf_phy_addr)
sgm_optim_0.write(inR_offs,Image_buf_phy_addr+image_size)
sgm_optim_0.write(outD_offs,Image_buf_phy_addr+2*image_size)
sgm_optim_1.write(inL_offs,Image_buf_phy_addr+ADDRESS_OFFSET)
sgm_optim_1.write(inR_offs,Image_buf_phy_addr+image_size+ADDRESS_OFFSET)
sgm_optim_1.write(outD_offs,Image_buf_phy_addr+2*image_size+ADDRESS_OFFSET)

sgm_optim_0.write(CTRL_reg_offset,0b00000001)
sgm_optim_1.write(CTRL_reg_offset,0b00000001)
remap_hls_0.write(CTRL_reg_offset,0b00000001)
remap_hls_1.write(CTRL_reg_offset,0b00000001)

count = 0
while(not(sgm_optim_0.register_map.CTRL.AP_DONE) or not(sgm_optim_1.register_map.CTRL.AP_DONE)):
        count = count+1

raw_iml_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
raw_imr_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
rec_iml_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
rec_imr_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)
disp_im_buffer = np.zeros((IMG_HEIGHT,IMG_WIDTH),dtype=np.ubyte)

for i in range(IMG_HEIGHT):
    for j in range(IMG_WIDTH):
            disp_im_buffer[i][j] = Image_buf[640*480*2+i*IMG_WIDTH+j]
            raw_iml_buffer[i][j] = Image_buf[640*480*1+i*IMG_WIDTH+j]
            raw_iml_buffer[i][j] = Image_buf[640*480*0+i*IMG_WIDTH+j]

cv2.imwrite('/home/xilinx/sgm_pynq_ver/raw_iml_buffer.png',raw_iml_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/raw_imr_buffer.png',raw_imr_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/rec_iml_buffer.png',rec_iml_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/rec_imr_buffer.png',rec_imr_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/disp_im_buffer.png',disp_im_buffer)


Xlnk.cma_free(Image_buf)