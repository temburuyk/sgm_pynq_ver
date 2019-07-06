from pynq import Overlay
from pynq import Xlnk
import numpy as np


overlay = Overlay('/home/xilinx/sgm_pynq_ver/design_1.bit')

overlay

print(overlay.ip_dict.keys())

sgm_optim_0 = overlay.sgm_optim_0
sgm_optim_1 = overlay.sgm_optim_1
remap_hls_0 = overlay.remap_hls_0
remap_hls_1 = overlay.remap_hls_1

sgm_optim_0.register_map


xlnk = Xlnk()

Image_buf =  xlnk.cma_alloc(0x1000000, data_type = "unsigned char")

Image_buf_phy_addr = xlnk.cma_get_phy_addr(Image_buf)

print(hex(Image_buf_phy_addr))

#Run the code of elp with the above physical address


