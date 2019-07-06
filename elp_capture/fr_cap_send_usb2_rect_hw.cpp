//to be run on Zedboard ARM
//captures frames from camera0 camera1
//stores them into reserved memory
//conditionally sends them to a server

//storing uncompressed images directly from camera is not possible in USB2
//work around: 1. write compressed images to files
//             2. read the images again using opencv & store into res space in uncompressed format

//remap + stereo matching done in hw

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

//opencv
#include <opencv2/core/core.hpp>
// #include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp> //req for cv2.remap()
// #include <opencv2/calib3d.hpp>
// #include <opencv2/imgcodecs.hpp>
// #include <opencv2/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>

//measuring time
#include <ctime>

//openmp
#include <omp.h>

//socket communincation client
#include <sys/socket.h> 
#include <netinet/in.h> //C
#include <arpa/inet.h> //C++ 
#include <string.h>
#include <unistd.h> //C++ read 

using namespace std;
using namespace cv;

#define NUM_IMAGES        10             //no of images to be captured
#define THROW_AWAY_FRAMES 0             //'THROW_AWAY_FRAMES' no of frames are thrown away before the first frame is captured
                                        // should be useful in sunlight to allow cam to adjust exposure 
#define SERVER_IP         "10.107.88.24" 
#define PORT_LEFT         8080           //socket comm port
#define PORT_RIGHT        8081           //socket comm port
#define SEND_FLAG         0              //set this to send data to server, 0 to only capture
#define IMG_WIDTH         640
#define IMG_HEIGHT        480
//frames are intermediately stored into these files
#define FRAME_0           "/tmp/frame_0.jpg" 
#define FRAME_1           "/tmp/frame_1.jpg"
//map files required for rectification
#define LEFT_RMAP         "left_rmap.yml"
#define RIGHT_RMAP        "right_rmap.yml"

//Defined for SGM with python VGA block
//---------------------------begin---------------------------------------
#define FILTER_SIZE     9
#define FILTER_OFFS     (int)(FILTER_SIZE/2) //4
#define SECTIONS        2
#define SECTION_HEIGHT  (int)((IMG_HEIGHT-2*FILTER_OFFS)/SECTIONS)
#define DISP_IMG_HEIGHT SECTIONS*SECTION_HEIGHT
#define BYTES_PER_PIXEL 1
#define TOTAL_BYTES     DISP_IMG_HEIGHT*IMG_WIDTH*BYTES_PER_PIXEL
#define ADDRESS_OFFSET  TOTAL_BYTES/SECTIONS //bytes

//-------------------------end----------------------------------------

class ElpVideoCapture 
{
    public:
                ElpVideoCapture     (const char *deviceName);
               ~ElpVideoCapture     ();
        void    captureFrame        (const char* outputFileName, char* raw_img_ptr, int devId, int send_flag, int sock, const char* map_file);
    private:
        int         m_fd;
        char       *m_buffer;
        v4l2_buffer m_bufferinfo;
        int         m_type;
        bool        m_isOpened;
        string      m_deviceName;
};

//constructor
ElpVideoCapture::ElpVideoCapture (const char *deviceName) 
{
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
    //capturing 640*480, not worrying about fish eye, we could capture max res (920*760) and then crop
    imageFormat.fmt.pix.width = IMG_WIDTH;
    imageFormat.fmt.pix.height = IMG_HEIGHT;
    imageFormat.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
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

//Destructor
ElpVideoCapture::~ElpVideoCapture () 
{
    // end streaming
    if (ioctl (m_fd, VIDIOC_STREAMOFF, &m_type) < 0){
        perror("Could not end streaming, VIDIOC_STREAMOFF");
        return;
    }

    close (m_fd);
}

int socket_setup(string server_ip_addr, uint16_t port)
{
    int sock = 0; 
    struct sockaddr_in serv_addr; 
    
    //create a socket, sock is the returned fd
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
        printf("\n Socket creation error \n"); 
        return -1; 
    } 

    //write 0s in the serv_addr location 
    memset(&serv_addr, '0', sizeof(serv_addr)); 
    //fill the serv_addr fields appropriately
    serv_addr.sin_family = AF_INET; 
    serv_addr.sin_port = htons(port); 
    // Convert IPv4 and IPv6 addresses from text to binary form 
    if(inet_pton(AF_INET, server_ip_addr.c_str(), &serv_addr.sin_addr)<=0) 
    { 
        printf("\nInvalid address/ Address not supported \n"); 
        return -1; 
    } 


    //try connecting to server
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
    { 
        printf("\nConnection Failed \n Check if server is running\n"); 
        return -1; 
    } 

    return sock;

}

//rectification----------begin---------------------------------------------------------------------------
unsigned char interpolate (unsigned char p00, unsigned char p01, unsigned char p10, unsigned char p11, unsigned short fractional) {
    unsigned short fractional_x = fractional & 0x1F;            // fractional (5-bit) column index
    unsigned short fractional_y = (fractional >> 5) & 0x1F;     // fractional (5-bit) row index
    // interpolate along x-axis in top border row
    float p0x = (float (p00) * (32.0 - float (fractional_x)) + float (p01) * float (fractional_x)) / 32.0;
    // interpolate along x-axis in bottom border row
    float p1x = (float (p10) * (32.0 - float (fractional_x)) + float (p11) * float (fractional_x)) / 32.0;
    // interpolate along y-axis
    unsigned char result = (unsigned char) ((float (p0x) * (32.0 - float (fractional_y)) + float (p1x) * float (fractional_y)) / 32.0);
    // return p00;
    return result;
}

#define USE_OPENCV 1

void remap_image (Mat &rmap0, Mat &rmap1, Mat &inImg, Mat &outImg) {
#ifdef USE_OPENCV
    remap (inImg, outImg, rmap0, rmap1, INTER_LINEAR);
#else
    outImg = inImg.clone ();
    for (int i = 0; i < rmap0.rows; i++) {          // row index
        for (int j = 0; j < rmap0.cols; j++) {      // column index
            Vec2s xy = rmap0.at<Vec2s> (i, j);
            short y0 = xy[0];                       // row part
            short x0 = xy[1];                       // column part
            unsigned short fractional = rmap1.at<ushort> (i, j);
            unsigned char p00, p01, p10, p11;
            p00 = inImg.at<uchar> (x0,     y0);
            p01 = inImg.at<uchar> (x0,     y0 + 1);
            p10 = inImg.at<uchar> (x0 + 1, y0);
            p11 = inImg.at<uchar> (x0 + 1, y0 + 1);
            unsigned char result = interpolate (p00, p01, p10, p11, fractional);
            outImg.at<uchar> (i, j) = result;
        }
    }
#endif
}
//rectification--------end-------------------------------------------------------------------------------------

//class function to capture frame, store it to reserved space, rectify image and send it to server
void ElpVideoCapture::captureFrame (const char* outputFileName, char* raw_img_ptr, int devId, int send_flag, int sock, const char* map_file) 
{
    if (false == m_isOpened) {
        cerr << "ElpVideoCapture::captureFrame : device not opened" << endl;
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
            // << " KBytes of data" << endl;

    // Write the data out to file
    ofstream outFile;
    outFile.open(outputFileName, ios::binary );
    outFile.write (m_buffer, m_bufferinfo.bytesused);
    // Close the file
    outFile.close();

    // Read the stored file as grayscale
    Mat raw_image = imread(outputFileName, 0);   

    //does not copy the image, hence cannot be used with pointers
    // Mat cropped = image(Rect((image.cols- IMG_WIDTH)/2 ,(image.rows- IMG_HEIGHT)/2,IMG_WIDTH,IMG_HEIGHT));
    
    #if 0
    //rectification
    Mat rmap0, rmap1;
    //read maps from the map_file (left_rmap.yml, right_rmap.yml)
    FileStorage fs (map_file, FileStorage::READ);
    if (false == fs.isOpened ()) {
        printf("could not open map file\n");
    }
    fs["rmap0"] >> rmap0;
    fs["rmap1"] >> rmap1;
    //overwrite the raw image with the rectified one
    Mat image = raw_image;
    remap_image (rmap0, rmap1, raw_image, image);
    #endif

    //debug: write the rectified image to file
    //vector<int> compression_params;
    //compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    //compression_params.push_back(9);
    //char rect_file[50];
    //sprintf(rect_file,"/tmp/rect_frame_%d",devId);
    //imwrite(rect_file, image); 

    //assign the base pointer
    unsigned char* image_ptr = (unsigned char*)(raw_image.data);

    //store image into reserved space
    //        dest_ptr                          src_ptr         size_bytes
    memcpy(raw_img_ptr+devId*IMG_WIDTH*IMG_HEIGHT, image_ptr, IMG_WIDTH*IMG_HEIGHT);
    // cout << image.rows << endl;
    // cout << image.cols << endl;

    #if 0
    //incase we want to crop
    int v_offs = (image.rows - IMG_HEIGHT)/2;
    int h_offs = (image.cols - IMG_WIDTH)/2;
    for (int y=0; y < IMG_HEIGHT; y++)   
    {
        //memcpy has to be done row by row, may have problems, not tested
        memcpy(raw_img_ptr + devId*IMG_WIDTH*IMG_HEIGHT, image_ptr + ( v_offs + y )*image.cols + h_offs, IMG_WIDTH );
        for (int x=0; x < IMG_WIDTH; x++)
        {
            printf("%d\n",*( image_ptr + ( v_offs + y )*image.cols + h_offs + x));
        }
    }
    #endif

    //Debug
    #if 0
    for (int y=0; y < image.rows; y++)   
    {
        for (int x=0; x < image.cols; x++)
        {
            printf("%d\n",*(image_ptr + image.channels()*(image.cols*y + x)));
        }
    }
    #endif

    //send left or right frame to server
    if(send_flag)
    {
        send(sock , raw_img_ptr+devId*IMG_WIDTH*IMG_HEIGHT , IMG_HEIGHT*IMG_WIDTH , 0 ); 

    }

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

int main (int argc, char* argv[]) 
{

    unsigned int res_mem_low_addr = 0x18400000;
    unsigned int res_mem_length   = 0x1000000; //32MB

    //write left map at the beginning of the reserved memory
    dump_file_to_location(res_mem_low_addr, IMG_WIDTH*IMG_HEIGHT, "elp640_left_rmap.bin");
    //write right map below left map
    dump_file_to_location(res_mem_low_addr+4*IMG_WIDTH*IMG_HEIGHT, IMG_WIDTH*IMG_HEIGHT, "elp640_right_rmap.bin");

    //map the reserved memory below right map and return ptr to it
    //raw images captured by camera will be stored at this ptr
    //8*IMG_WIDTH*IMG_HEIGHT is the space req for 2 maps
    int fd_mem = open ("/dev/mem", O_RDWR);
    char* raw_img_ptr = (char*)mmap (NULL, res_mem_length-8*IMG_WIDTH*IMG_HEIGHT, PROT_READ|PROT_WRITE, MAP_SHARED, fd_mem,
                                     res_mem_low_addr+8*IMG_WIDTH*IMG_HEIGHT);
    
    //map the memory range for controlling peripherals
    //starts from 0x43C00000 and goes to 0x43C<n>FFFF where n is the number of AXI Master slave peripherals
    //64kB for each peripheral
    //we will map 1 MB memory just to be safe
    unsigned int register_mem_low_addr = 0x43C00000;
    unsigned int register_mem_length   = 0x80000; //1MB
    int fd_reg = open ("/dev/mem", O_RDWR);
    uint32_t* reg_ptr = (uint32_t*)mmap (NULL, register_mem_length, PROT_READ|PROT_WRITE, MAP_SHARED, fd_reg, register_mem_low_addr);
    
    //from vivado project: sgm_and_remap
    //sgm: 2 sgm blocks at 43c0 and 43c1 , vga block at 43c2, 2 blocks of remap (l and r) at 43c3 and 43c4
    //registers of different blocks can be accessed as follows:
    // *(reg_ptr + block_no*0x4000 + reg_no)
    uint16_t sgm0_block_number=0,sgm1_block_number=1,remapl_block_number=2,remapr_block_number=3;
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
    *(reg_ptr + remapl_block_number*0x04000 + 4) =  map_left_addr; //0x1a000000 
    *(reg_ptr + remapr_block_number*0x04000 + 4) =  map_right_addr;
    //remap: pin
    uint32_t raw_left_img  = map_right_addr + 4*IMG_WIDTH*IMG_HEIGHT;
    uint32_t raw_right_img = raw_left_img   + IMG_WIDTH*IMG_HEIGHT; 
    *(reg_ptr + remapl_block_number*0x04000 + 6) = raw_left_img; 
    *(reg_ptr + remapr_block_number*0x04000 + 6) = raw_right_img;
    //remap: pout
    uint32_t rect_left_img  = raw_right_img + IMG_WIDTH*IMG_HEIGHT;
    uint32_t rect_right_img = rect_left_img + IMG_WIDTH*IMG_HEIGHT; 
    *(reg_ptr + remapl_block_number*0x04000 + 8) = rect_left_img; 
    *(reg_ptr + remapr_block_number*0x04000 + 8) = rect_right_img; 
    //sgm: inL
    *(reg_ptr + sgm0_block_number*0x04000 + 4) = rect_left_img; 
    *(reg_ptr + sgm1_block_number*0x04000 + 4) = rect_left_img + ADDRESS_OFFSET; 
    //sgm: inR
    *(reg_ptr + sgm0_block_number*0x04000 + 6) = rect_right_img;
    *(reg_ptr + sgm1_block_number*0x04000 + 6) = rect_right_img +  ADDRESS_OFFSET; 
    //sgm: outD
    uint32_t disp_img = rect_right_img + IMG_WIDTH*IMG_HEIGHT;
    *(reg_ptr + sgm0_block_number*0x04000 + 8) = disp_img;
    *(reg_ptr + sgm1_block_number*0x04000 + 8) = disp_img +  ADDRESS_OFFSET; 
    //vga
    //*(reg_ptr + 2*0x04000 + 1) = disp_img; //configured to show disp image

    //start peripherals on autorestart
    //sgm
    *(reg_ptr + sgm0_block_number*0x04000 + 0) = *(reg_ptr + sgm0_block_number*0x04000 + 0) | 0b10000001; 
    *(reg_ptr + sgm1_block_number*0x04000 + 0) = *(reg_ptr + sgm1_block_number*0x04000 + 0) | 0b10000001; 
    //remap
    *(reg_ptr + remapl_block_number*0x04000 + 0) = *(reg_ptr + remapl_block_number*0x04000 + 0) | 0b10000001; 
    *(reg_ptr + remapr_block_number*0x04000 + 0) = *(reg_ptr + remapr_block_number  *0x04000 + 0) | 0b10000001; 
    //vga
    //*(reg_ptr + 2*0x04000 + 0) = *(reg_ptr + 2*0x04000 + 0) | 0b00000001;   
    //setup the socket
    int sock_left = 0;
    int sock_right = 0;
    if(SEND_FLAG)
    {
        sock_left  = socket_setup(SERVER_IP, PORT_LEFT);
        sock_right = socket_setup(SERVER_IP, PORT_RIGHT);
    }

    //declare two objects of the class ElpVideoCapture
    //it is observed that video1 is the left camera and video0 is right which is opposite of our assumption
    //hence reverse it her itself
    ElpVideoCapture v0 ("/dev/video1");
    ElpVideoCapture v1 ("/dev/video0");
    //from here on '1' stands for right, '0' stands for left

    //for measuring time
    struct timespec requestStart, requestEnd;
    clock_gettime(CLOCK_REALTIME, &requestStart);

    //syntax: captureFrame(const char* outputFileName, char* raw_img_ptr, int devId, int send_flag, int sock);
    //capture frames
    for (int t = 0; t < NUM_IMAGES; t++) 
    {
        //'THROW_AWAY_FRAMES' no of frames are thrown away and then first frame is written, giving time for the camera to adjust exposure
        //send flag is always 0
        if (0 == t) 
        {
            for (int r = 0; r < THROW_AWAY_FRAMES; r++) 
            {
                #pragma omp parallel for
                for(int devId=0; devId<2; devId++)
                {
                    devId ? v1.captureFrame (FRAME_1,raw_img_ptr,devId,0,sock_right,RIGHT_RMAP) : v0.captureFrame (FRAME_0,raw_img_ptr,devId,0,sock_left,LEFT_RMAP);  
                }            
            }
        }
        
        #pragma omp parallel for
        for(int devId=0; devId<2; devId++)
        {
            devId ? v1.captureFrame (FRAME_1,raw_img_ptr,devId,SEND_FLAG,sock_right,RIGHT_RMAP) : v0.captureFrame (FRAME_0,raw_img_ptr,devId,SEND_FLAG,sock_left,LEFT_RMAP);  
        }


    }

    clock_gettime(CLOCK_REALTIME, &requestEnd);
    
    // Calculate time it took
    long int total_usec_elapsed = (requestEnd.tv_sec*1e9+requestEnd.tv_nsec - requestStart.tv_sec*1e9-requestStart.tv_nsec)/1e3;
    long int avg_usec_elapsed = total_usec_elapsed/NUM_IMAGES;
    printf ("micro seconds elapsed for capturing %d frames: %ld\n", NUM_IMAGES, total_usec_elapsed);
    printf ("average micro seconds elapsed for capturing 1 frame pair: %ld\n", avg_usec_elapsed);

/*    for(int k=0; k<IMG_WIDTH*IMG_HEIGHT;k++)
    {
        disp_img
        printf("%d \n",);
    }*/
    
    return 0;
}






