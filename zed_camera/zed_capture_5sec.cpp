// Example file downloaded from https://gist.github.com/mike168m/6dd4eb42b2ec906e064d
// Converted to c++

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/v4l2-common.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <string.h>
#include <fstream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#define ZED_IMAGE_WIDTH 1344
#define ZED_IMAGE_HEIGHT 376

using namespace std;
using namespace cv;

class ZedVideoCapture {
    public:
                ZedVideoCapture     (const char *deviceName);
               ~ZedVideoCapture     ();
        void    captureFrame        (Mat &leftMat, Mat &rightMat);
    private:
        int         m_fd;
        char       *m_buffer;
        v4l2_buffer m_bufferinfo;
        int         m_type;
        bool        m_isOpened;
        string      m_deviceName;
};

ZedVideoCapture::ZedVideoCapture (const char *deviceName) {
    m_isOpened = false;
    m_deviceName = deviceName;
    // 1.  Open the device
    m_fd = open (deviceName, O_RDWR);
    if (m_fd < 0) {
        perror ("Failed to open device, OPEN");
        return;
    }

    // 2. Ask the device if it can capture frames
    v4l2_capability capability;
    if (ioctl (m_fd, VIDIOC_QUERYCAP, &capability) < 0){
        // something went wrong... exit
        perror("Failed to get device capabilities, VIDIOC_QUERYCAP");
        return;
    }

    // 3. Set Image format
    v4l2_format imageFormat;
    imageFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    imageFormat.fmt.pix.width = ZED_IMAGE_WIDTH;
    imageFormat.fmt.pix.height = ZED_IMAGE_HEIGHT;
    imageFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    imageFormat.fmt.pix.field = V4L2_FIELD_NONE;
    // tell the device you are using this format
    if(ioctl(m_fd, VIDIOC_S_FMT, &imageFormat) < 0){
        perror("Device could not set format, VIDIOC_S_FMT");
        return;
    }


    // 4. Request Buffers from the device
    v4l2_requestbuffers requestBuffer = {0};
    requestBuffer.count = 1; // one request buffer
    requestBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; // request a buffer wich we an use for capturing frames
    requestBuffer.memory = V4L2_MEMORY_MMAP;

    if(ioctl(m_fd, VIDIOC_REQBUFS, &requestBuffer) < 0){
        perror("Could not request buffer from device, VIDIOC_REQBUFS");
        return;
    }


    // 5. Quety the buffer to get raw data ie. ask for the you requested buffer
    // and allocate memory for it
    v4l2_buffer queryBuffer = {0};
    queryBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    queryBuffer.memory = V4L2_MEMORY_MMAP;
    queryBuffer.index = 0;
    if(ioctl(m_fd, VIDIOC_QUERYBUF, &queryBuffer) < 0){
        perror("Device did not return the buffer information, VIDIOC_QUERYBUF");
        return;
    }
    // use a pointer to point to the newly created buffer
    // mmap() will map the memory address of the device to
    // an address in memory
    m_buffer = (char*)mmap(NULL, queryBuffer.length, PROT_READ | PROT_WRITE, MAP_SHARED,
                        m_fd, queryBuffer.m.offset);
    memset (m_buffer, 0, queryBuffer.length);


    // 6. Get a frame
    // Create a new buffer type so the device knows whichbuffer we are talking about
    memset(&m_bufferinfo, 0, sizeof(m_bufferinfo));
    m_bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    m_bufferinfo.memory = V4L2_MEMORY_MMAP;
    m_bufferinfo.index = 0;

    // Activate streaming
    m_type = m_bufferinfo.type;
    if(ioctl (m_fd, VIDIOC_STREAMON, &m_type) < 0){
        perror("Could not start streaming, VIDIOC_STREAMON");
        return;
    }
    m_isOpened = true;
}

ZedVideoCapture::~ZedVideoCapture () {
    // end streaming
    if (ioctl (m_fd, VIDIOC_STREAMOFF, &m_type) < 0){
        perror("Could not end streaming, VIDIOC_STREAMOFF");
        return;
    }

    close (m_fd);
}

void ZedVideoCapture::captureFrame (Mat &leftMat, Mat &rightMat) {
    if (false == m_isOpened) {
        cerr << "ZedVideoCapture::captureFrame : device not opened" << endl;
        return;
    }

    // Queue the buffer
    if(ioctl(m_fd, VIDIOC_QBUF, &m_bufferinfo) < 0){
        perror("Could not queue buffer, VIDIOC_QBUF");
        return;
    }

    // Dequeue the buffer
    if(ioctl(m_fd, VIDIOC_DQBUF, &m_bufferinfo) < 0){
        perror("Could not dequeue the buffer, VIDIOC_DQBUF");
        return;
    }
    // Frames get written after dequeuing the buffer

    // cout << "Buffer has: " << (double)m_bufferinfo.bytesused / 1024
    //         << " KBytes of data" << endl;

    assert (m_bufferinfo.bytesused >= (ZED_IMAGE_WIDTH * ZED_IMAGE_HEIGHT * 2));

    leftMat = Mat::zeros (ZED_IMAGE_HEIGHT, ZED_IMAGE_WIDTH / 2, CV_8U);
    rightMat = Mat::zeros (ZED_IMAGE_HEIGHT, ZED_IMAGE_WIDTH / 2, CV_8U);

    for (int i = 0; i < ZED_IMAGE_HEIGHT; i++) {
        for (int j = 0; j < ZED_IMAGE_WIDTH/2; j++) {
            leftMat.at<uchar>(i, j) = m_buffer[(i*ZED_IMAGE_WIDTH+j)*2];
        }
        for (int j = 0; j < ZED_IMAGE_WIDTH/2; j++) {
            rightMat.at<uchar>(i, j) = m_buffer[(i*ZED_IMAGE_WIDTH+j)*2+(ZED_IMAGE_WIDTH/2)*2];
        }
    }
}

#if 0
unsigned char leftBuffer[640*480];
unsigned char rightBuffer[640*480];
#else
unsigned char *leftBuffer  = NULL;
unsigned char *rightBuffer = NULL;
unsigned int res_mem_low_addr = 0x1a000000;
unsigned int res_mem_length   = 0x1000000; //32MB
uint32_t* reg_ptr = NULL;
#endif

void reserved_memory_setup () {

    int fd = open ("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        printf ("ERROR : failed to open /dev/mem\n");
        return;
    }

    res_mem_low_addr = 0x1a000000;
    res_mem_length   = 0x1000000; //32MB
    unsigned char *ptr = (unsigned char *) mmap (NULL, res_mem_length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, res_mem_low_addr);
    reg_ptr = (uint32_t*)mmap (NULL, res_mem_length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, res_mem_low_addr);
    printf ("INFO : /dev/mem mapped at [0x%08x]\n", (int) ptr);

    leftBuffer = ptr;
    rightBuffer = ptr + 0x00100000;

    // void *memset(void *s, int c, size_t n);
    memset (leftBuffer, 0, 640 * 480);
    memset (rightBuffer, 0, 640 * 480);
}

void setup_vga () {
    int fd = open ("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        printf ("ERROR : failed to open /dev/mem\n");
        return;
    }
    //address of vga peripheral from vivado project
    unsigned int vga_low_addr = 0x43c20000; 
    unsigned int vga_length   = 16;
    unsigned char *ptr = (unsigned char *) mmap (NULL, vga_length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, vga_low_addr);
    int *int_ptr = (int *) ptr;
    //base address of the image to be displayed (depth image)
    int_ptr[1] = 0x1a200000;
    //starting the peripheral by moving 1 into the register
    int_ptr[0] = 1;
}

void setup_sgm_top () {
    int fd = open ("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        printf ("ERROR : failed to open /dev/mem\n");
        return;
    }
    //address of sgm_top peripheral from vivado project
    unsigned int sgm_low_addr = 0x43c00000;
    unsigned int sgm_length   = 16;
    unsigned char *ptr = (unsigned char *) mmap (NULL, sgm_length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, sgm_low_addr);
    int *int_ptr = (int *) ptr;
    //base address of left image
    int_ptr[4] = 0x1a000000;
    //base address of right image
    int_ptr[6] = 0x1a100000;
    //base address of disparity image
    int_ptr[8] = 0x1a200000;
    //starting the peripheral on autorestart
    int_ptr[0] = 0x81;
}

//address offset calculation for the second peripheral
#define IMG_WIDTH         640
#define IMG_HEIGHT        480

#define FILTER_SIZE     9
#define FILTER_OFFS     (int)(FILTER_SIZE/2) //4
#define SECTIONS        5
#define SECTION_HEIGHT  (int)((IMG_HEIGHT-2*FILTER_OFFS)/SECTIONS)
#define DISP_IMG_HEIGHT SECTIONS*SECTION_HEIGHT
#define BYTES_PER_PIXEL 1
#define TOTAL_BYTES     DISP_IMG_HEIGHT*IMG_WIDTH*BYTES_PER_PIXEL
#define ADDRESS_OFFSET  TOTAL_BYTES/SECTIONS //bytes

void setup_sgm_bottom () {
    int fd = open ("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        printf ("ERROR : failed to open /dev/mem\n");
        return;
    }
    //address of sgm_top peripheral from vivado project
    unsigned int sgm_low_addr = 0x43c10000;
    unsigned int sgm_length   = 16;
    unsigned char *ptr = (unsigned char *) mmap (NULL, sgm_length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, sgm_low_addr);
    int *int_ptr = (int *) ptr;
    int_ptr[4] = 0x1a000000 + ADDRESS_OFFSET;
    int_ptr[6] = 0x1a100000 + ADDRESS_OFFSET;
    int_ptr[8] = 0x1a200000 + ADDRESS_OFFSET;
    int_ptr[0] = 0x81;
}

//if there was a third sgm peripheral we would have used: int_ptr[4] = 0x1a000000 + 2*ADDRESS_OFFSET;
void peripheral_setup(){
    //map the memory range for controlling peripherals
    //starts from 0x43C00000 and goes to 0x43C<n>FFFF where n is the number of AXI Master slave peripherals
    //64kB for each peripheral
    //we will map 1 MB memory just to be safe
/*    unsigned int register_mem_low_addr = 0x43C00000;
    unsigned int register_mem_length   = 0x80000; //1MB
    int fd_reg = open ("/dev/mem", O_RDWR);
    uint32_t* reg_ptr = (uint32_t*)mmap (NULL, register_mem_length, PROT_READ|PROT_WRITE, MAP_SHARED, fd_reg, register_mem_low_addr);
    */
    //blocks: 0:sgm, 1:sgm, 2:vga, 3:remap_l, 4:remap_r
    //sgm registers: 0 control -> bits 31-8:reserved, 7:autorestart, 6-4:reserved, 3:ready, 2:idle, 1:done, 0:start(LSB) 
    //               4 inL
    //               5 reserved
    //               6 inR
    //               7 reserved
    //               8 out
    //vga registers: 0 control -> bit 0:start(LSB)
    //               1 image_addr
    //remap regstrs: 0 control
    //               4 map_data
    //               5 reserved
    //               6 pin
    //               7 reserved
    //               8 pout
    
    //fill the axi_slave registers with physical addresses of data
    //remap: map
    uint32_t map_left_addr  = res_mem_low_addr;
    uint32_t map_right_addr = res_mem_low_addr + 4*IMG_WIDTH*IMG_HEIGHT; 
    *(reg_ptr + 3*0x04000 + 4) =  map_left_addr; //0x1a000000 
    *(reg_ptr + 4*0x04000 + 4) =  map_right_addr;
    //remap: pin
    uint32_t raw_left_img  = map_right_addr + 4*IMG_WIDTH*IMG_HEIGHT;
    uint32_t raw_right_img = raw_left_img   + IMG_WIDTH*IMG_HEIGHT; 
    *(reg_ptr + 3*0x04000 + 6) = raw_left_img; 
    *(reg_ptr + 4*0x04000 + 6) = raw_right_img;
    //remap: pout
    uint32_t rect_left_img  = raw_right_img + IMG_WIDTH*IMG_HEIGHT;
    uint32_t rect_right_img = rect_left_img + IMG_WIDTH*IMG_HEIGHT; 
    *(reg_ptr + 3*0x04000 + 8) = rect_left_img; 
    *(reg_ptr + 4*0x04000 + 8) = rect_right_img; 
    //sgm: inL
    *(reg_ptr + 0*0x04000 + 4) = rect_left_img; 
    *(reg_ptr + 1*0x04000 + 4) = rect_left_img + ADDRESS_OFFSET;
    *(reg_ptr + 5*0x04000 + 4) = rect_left_img + 2*ADDRESS_OFFSET;
    *(reg_ptr + 6*0x04000 + 4) = rect_left_img + 3*ADDRESS_OFFSET; 
    *(reg_ptr + 7*0x04000 + 4) = rect_left_img + 4*ADDRESS_OFFSET;   
    //sgm: inR
    *(reg_ptr + 0*0x04000 + 6) = rect_right_img;
    *(reg_ptr + 1*0x04000 + 6) = rect_right_img +  ADDRESS_OFFSET;
    *(reg_ptr + 5*0x04000 + 6) = rect_right_img +  2*ADDRESS_OFFSET;
    *(reg_ptr + 6*0x04000 + 6) = rect_right_img +  3*ADDRESS_OFFSET;
    *(reg_ptr + 7*0x04000 + 6) = rect_right_img +  4*ADDRESS_OFFSET;
    //sgm: outD
    uint32_t disp_img = rect_right_img + IMG_WIDTH*IMG_HEIGHT;
    *(reg_ptr + 0*0x04000 + 8) = disp_img;
    *(reg_ptr + 1*0x04000 + 8) = disp_img +  ADDRESS_OFFSET; 
    *(reg_ptr + 5*0x04000 + 8) = disp_img +  2*ADDRESS_OFFSET;
    *(reg_ptr + 6*0x04000 + 8) = disp_img +  3*ADDRESS_OFFSET;
    *(reg_ptr + 7*0x04000 + 8) = disp_img +  4*ADDRESS_OFFSET;
    //vga
    *(reg_ptr + 2*0x04000 + 1) = disp_img; //configured to show disp image

    //start peripherals on autorestart
    //sgm
    *(reg_ptr + 0*0x04000 + 0) = *(reg_ptr + 0*0x04000 + 0) | 0b10000001; 
    *(reg_ptr + 1*0x04000 + 0) = *(reg_ptr + 1*0x04000 + 0) | 0b10000001; 
    *(reg_ptr + 5*0x04000 + 0) = *(reg_ptr + 0*0x04000 + 0) | 0b10000001; 
    *(reg_ptr + 6*0x04000 + 0) = *(reg_ptr + 1*0x04000 + 0) | 0b10000001; 
    *(reg_ptr + 7*0x04000 + 0) = *(reg_ptr + 1*0x04000 + 0) | 0b10000001; 
    //remap
    *(reg_ptr + 3*0x04000 + 0) = *(reg_ptr + 3*0x04000 + 0) | 0b10000001; 
    *(reg_ptr + 4*0x04000 + 0) = *(reg_ptr + 4*0x04000 + 0) | 0b10000001; 
    //vga
    *(reg_ptr + 2*0x04000 + 0) = *(reg_ptr + 2*0x04000 + 0) | 0b10000001;
}

//function used for writing file to memory in linux_xsct
//used for writing maps to memory
//address can be in hex 0x...
//length = no. of words (word = 4 bytes)
void dump_file_to_location (unsigned int addr, unsigned int length, string src_filename)
{
    int fd = open ("/dev/mem", O_RDWR|O_SYNC);
    if (fd < 0)
    {
        fprintf (stderr, "ERROR : failed to open /dev/mem\n");
        return;
    }
    // mmap the device into memory
    // Obtained from hdf file or Vivado block design address editor tab
    addr = addr & 0xFFFFFFFC;
    unsigned int page_size   =  sysconf(_SC_PAGESIZE);
    unsigned int page_addr   = (addr & (~(page_size-1)));
    unsigned int page_offset =  addr - page_addr;
    unsigned int rounded_length = page_size * ((length*4 + page_offset + (page_size - 1)) / page_size);

    unsigned char *ptr = (unsigned char *) mmap (NULL, rounded_length, PROT_READ|PROT_WRITE, MAP_SHARED, fd, page_addr);
    fprintf (stderr, "INFO : /dev/mem mapped at [0x%08x]\n", (unsigned int) ptr);
    fprintf (stderr, "INFO : given address's byte offset from beginning of mapped memory [0x%08x]\n", page_offset);

    int *start_ptr = (int *) &(ptr[page_offset]);

    FILE *pFin = fopen (src_filename.c_str (), "r");
    if (NULL == pFin)
    {
        perror ("failed to open file for writing");
    }
    else
    {
        unsigned int numberOfWordsRead = fread (start_ptr, 4, length, pFin);
        if (length != numberOfWordsRead)
        {
            fprintf (stderr, "failed to read items from file.\n");
        }
        fclose (pFin);
    }

    munmap (ptr, rounded_length);
    close (fd);
}

#define imshow(a,b)
#define waitKey()

#if 1
int main (int argc, char* argv[]) {

    //Read calibration maps from yml files
    Mat left_rmap0, left_rmap1;
    FileStorage fs_left ("zed_left_calibration.yml", FileStorage::READ);
    fs_left["rmap0"] >> left_rmap0;
    fs_left["rmap1"] >> left_rmap1;

    Mat right_rmap0, right_rmap1;
    FileStorage fs_right ("zed_right_calibration.yml", FileStorage::READ);
    fs_right["rmap0"] >> right_rmap0;
    fs_right["rmap1"] >> right_rmap1;

    reserved_memory_setup ();
    //write left map at the beginning of the reserved memory
    dump_file_to_location(res_mem_low_addr, IMG_WIDTH*IMG_HEIGHT, "elp640_left_rmap.bin");
    //write right map below left map
    dump_file_to_location(res_mem_low_addr+4*IMG_WIDTH*IMG_HEIGHT, IMG_WIDTH*IMG_HEIGHT, "elp640_right_rmap.bin");

	//setup_vga ();

    ZedVideoCapture v ("/dev/video0");

    int t = -1;
    // for (int t = 0; t < 100; t++) {
    while (true) {
        t = t + 1;
        Mat view_left;
        Mat view_right;
        v.captureFrame (view_left, view_right);

        // char title0[1024];
        // char title1[1024];
        // sprintf (title0, "left image (%dx%d)", view_left.cols, view_left.rows);
        // sprintf (title1, "right image (%dx%d)", view_right.cols, view_right.rows);

        // imshow (title0, view_left);
        // imshow (title1, view_right);

/*        Mat rectified_left;
        remap (view_left, rectified_left, left_rmap0, left_rmap1, INTER_LINEAR);
        Mat rectified_right;
        remap (view_right, rectified_right, right_rmap0, right_rmap1, INTER_LINEAR);
*/
        unsigned char *left_data = view_left.data;
        for (int i = 0; i < view_left.rows; i++) {
            for (int j = 0; j < view_left.cols; j++) {
                leftBuffer[i*640+j] = left_data[i*view_left.step + j];
            }
        }
        unsigned char *right_data = view_right.data;
        for (int i = 0; i < view_right.rows; i++) {
            for (int j = 0; j < view_right.cols; j++) {
                rightBuffer[i*640+j] = right_data[i*view_right.step + j];
            }
        }
        // Mat left640_480;
        // left640_480 = Mat::zeros (480, 640, CV_8U);
        // for (int i = 0; i < left640_480.rows; i++) {
        //     for (int j = 0; j < left640_480.cols; j++) {
        //         left640_480.at<uchar> (i, j) = leftBuffer[i*640+j];
        //     }
        // }
        // Mat right640_480;
        // right640_480 = Mat::zeros (480, 640, CV_8U);
        // for (int i = 0; i < right640_480.rows; i++) {
        //     for (int j = 0; j < right640_480.cols; j++) {
        //         right640_480.at<uchar> (i, j) = rightBuffer[i*640+j];
        //     }
        // }

        // imshow ("rectified_left", left640_480);
        // imshow ("rectified_right", right640_480);
        // imwrite ("rectified_left.png", left640_480);
        // imwrite ("rectified_right.png", right640_480);
        // // imshow ("rectified_left", rectified_left);
        // // imshow ("rectified_right", rectified_right);

        // waitKey ();
        // cout << t;
        // sleep (1);
        if (t == 0) {
            //setup_sgm_top ();
            //setup_sgm_bottom ();
            peripheral_setup();
        }
        // sleep (1);
        // cout << "Done" << endl;
        if (9 == (t % 10)) {
            cout << ".";
        }
    }

    return 0;
}
#endif
