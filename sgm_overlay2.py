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
ADDRESS_OFFSET  =TOTAL_BYTES/SECTIONS
image_size = IMG_WIDTH*IMG_HEIGHT

overlay = Overlay('/home/xilinx/sgm_pynq_ver/design_1.bit')
overlay

capl = cv2.VideoCapture(0)
capr = cv2.VideoCapture(1)

print(overlay.ip_dict.keys())

sgm_optim_0 = overlay.sgm_optim_0
sgm_optim_1 = overlay.sgm_optim_1
remap_hls_0 = overlay.remap_hls_0
remap_hls_1 = overlay.remap_hls_1

inL_offs = sgm_optim_0.register_map.inL.address
inR_offs = sgm_optim_0.register_map.inR.address
outD_offs = sgm_optim_0.register_map.outD.address
CTRL_reg_offset = sgm_optim_0.register_map.CTRL.address

xlnk = Xlnk()

raw_iml_buffer = xlnk.cma_array(shape=(image_size,), dtype=np.uint8)
raw_imr_buffer = xlnk.cma_array(shape=(image_size,), dtype=np.uint8)
rec_iml_buffer = xlnk.cma_array(shape=(image_size,), dtype=np.uint8)
rec_imr_buffer = xlnk.cma_array(shape=(image_size,), dtype=np.uint8)
disp_im_buffer = xlnk.cma_array(shape=(image_size,), dtype=np.uint8)

raw_iml_buffer_phy_addr = raw_iml_buffer.physical_address
raw_imr_buffer_phy_addr = raw_iml_buffer.physical_address
rec_iml_buffer_phy_addr = raw_iml_buffer.physical_address
rec_imr_buffer_phy_addr = raw_iml_buffer.physical_address
disp_im_buffer_phy_addr = disp_im_buffer.physical_address

ret, framel = capl.read()
ret, framer = capr.read()

raw_iml_buffer.data = cv2.cvtColor(framel, cv2.COLOR_BGR2GRAY).flatten().data
raw_imr_buffer.data = cv2.cvtColor(framer, cv2.COLOR_BGR2GRAY).flatten().data

sgm_optim_0.write(inL_offs,raw_iml_buffer_phy_addr)
sgm_optim_0.write(inR_offs,raw_imr_buffer_phy_addr)
sgm_optim_0.write(outD_offs,disp_im_buffer_phy_addr)
sgm_optim_1.write(inL_offs,raw_iml_buffer_phy_addr+ADDRESS_OFFSET)
sgm_optim_1.write(inR_offs,raw_imr_buffer_phy_addr+ADDRESS_OFFSET)
sgm_optim_1.write(outD_offs,disp_im_buffer_phy_addr+ADDRESS_OFFSET)

sgm_optim_0.write(CTRL_reg_offset,1)
sgm_optim_1.write(CTRL_reg_offset,1)

count = 0
while(not(sgm_optim_0.register_map.CTRL.AP_DONE) or not(sgm_optim_1.register_map.CTRL.AP_DONE)):
        count = count+1

cv2.imwrite('/home/xilinx/sgm_pynq_ver/raw_iml_buffer.png',raw_iml_buffer.reshape(IMG_HEIGHT,IMG_WIDTH))
cv2.imwrite('/home/xilinx/sgm_pynq_ver/raw_imr_buffer.png',raw_imr_buffer.reshape(IMG_HEIGHT,IMG_WIDTH))
cv2.imwrite('/home/xilinx/sgm_pynq_ver/rec_iml_buffer.png',rec_iml_buffer.reshape(IMG_HEIGHT,IMG_WIDTH))
cv2.imwrite('/home/xilinx/sgm_pynq_ver/rec_imr_buffer.png',rec_imr_buffer.reshape(IMG_HEIGHT,IMG_WIDTH))
cv2.imwrite('/home/xilinx/sgm_pynq_ver/disp_im_buffer.png',disp_im_buffer.reshape(IMG_HEIGHT,IMG_WIDTH))

capl.release()
capr.release()

raw_iml_buffer.close()
raw_imr_buffer.close()
rec_iml_buffer.close()
rec_imr_buffer.close()
disp_im_buffer.close()