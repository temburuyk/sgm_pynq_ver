import cv2



cv2.namedWindow('image', cv2.WINDOW_NORMAL)
count = 0
while(count < 5000):
	print(count)
	count = count + 1
	image = cv2.imread('/home/xilinx/sgm_pynq_ver/output_images/disp_im_buffer.png', 0);
	cv2.imshow('image',image)
	cv2.waitKey(1)

cv2.waitKey(0)
cv2.destroyAllWindows()	