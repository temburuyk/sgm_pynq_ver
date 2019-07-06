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

imagel = cv2.imread('/home/xilinx/sgm_pynq_ver/test_images/raw_iml_buffer.png', 0);
imager = cv2.imread('/home/xilinx/sgm_pynq_ver/test_images/raw_imr_buffer.png', 0);


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

# raw_iml_buffer = xlnk.cma_array(shape=(image_size,), dtype=np.ubyte)
# raw_imr_buffer = xlnk.cma_array(shape=(image_size,), dtype=np.ubyte)
# rec_iml_buffer = xlnk.cma_array(shape=(image_size,), dtype=np.ubyte)
# rec_imr_buffer = xlnk.cma_array(shape=(image_size,), dtype=np.ubyte)
# disp_im_buffer = xlnk.cma_array(shape=(image_size,), dtype=np.ubyte)

# raw_iml_buffer_phy_addr = raw_iml_buffer.physical_address
# raw_imr_buffer_phy_addr = raw_iml_buffer.physical_address
# rec_iml_buffer_phy_addr = raw_iml_buffer.physical_address
# rec_imr_buffer_phy_addr = raw_iml_buffer.physical_address
# disp_im_buffer_phy_addr = disp_im_buffer.physical_address

#raw_iml_buffer.data = cv2.cvtColor(framel, cv2.COLOR_BGR2GRAY).flatten().data
#raw_imr_buffer.data = cv2.cvtColor(framer, cv2.COLOR_BGR2GRAY).flatten().data
test1 = imagel.flatten()
test2 = imager.flatten()
for i in range(IMG_HEIGHT):
	for j in range(IMG_WIDTH):
		Image_buf[i*IMG_WIDTH+j] = test1[i*IMG_WIDTH+j]
		Image_buf[image_size + i*IMG_WIDTH+j] = test2[i*IMG_WIDTH+j]


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

raw_iml_buffer = np.zeros(IMG_HEIGHT,IMG_WIDTH)
raw_imr_buffer = np.zeros(IMG_HEIGHT,IMG_WIDTH)
rec_iml_buffer = np.zeros(IMG_HEIGHT,IMG_WIDTH)
rec_imr_buffer = np.zeros(IMG_HEIGHT,IMG_WIDTH)
disp_im_buffer = np.zeros(IMG_HEIGHT,IMG_WIDTH)

for i in range(IMG_HEIGHT):
    for j in range(IMG_WIDTH):
            disp_im_buffer[i][j] = Image_buf[640*480*2+i*IMG_WIDTH+j]
            raw_iml_buffer[i][ij = Image_buf[640*480*1+i*IMG_WIDTH+j]
            raw_iml_buffer[i][ij = Image_buf[640*480*0+i*IMG_WIDTH+j]

cv2.imwrite('/home/xilinx/sgm_pynq_ver/raw_iml_buffer.png',raw_iml_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/raw_imr_buffer.png',raw_imr_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/rec_iml_buffer.png',rec_iml_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/rec_imr_buffer.png',rec_imr_buffer)
cv2.imwrite('/home/xilinx/sgm_pynq_ver/disp_im_buffer.png',disp_im_buffer)



# raw_iml_buffer.close()
# raw_imr_buffer.close()
# rec_iml_buffer.close()
# rec_imr_buffer.close()
# disp_im_buffer.close()