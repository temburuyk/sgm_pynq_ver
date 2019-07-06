
for i in range(IMG_HEIGHT):
    for j in range(IMG_WIDTH):
            test1[i*IMG_WIDTH+j] = Image_buf[640*480*10+i*IMG_WIDTH+j]

cv2.imwrite('/home/xilinx/sgm_pynq_ver/rec_iml_buffer.png',test1.reshape(IMG_HEIGHT,IMG_WIDTH))


empty = cv2.imread('/home/xilinx/sgm_pynq_ver/test_images/empty.png', 0)

for i in range(IMG_HEIGHT):
    for j in range(IMG_WIDTH):
            empty[i][j] = Image_buf[640*480*2+i*IMG_WIDTH+j]

cv2.imwrite('/home/xilinx/sgm_pynq_ver/disp_im_buffer.png',empty)