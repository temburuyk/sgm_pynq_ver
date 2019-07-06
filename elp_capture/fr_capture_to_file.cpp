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
using namespace std;

#define NUM_IMAGES        100000             //no of images to be captured
#define THROW_AWAY_FRAMES 0             //'THROW_AWAY_FRAMES' no of frames are thrown away before the first frame is captured
#define IMG_WIDTH         640
#define IMG_HEIGHT        480
//frames are intermediately stored into these files
#define FRAME_0           "/tmp/frame_0.jpg" 
#define FRAME_1           "/tmp/frame_1.jpg"

class ElpVideoCapture 
{
    public:
                ElpVideoCapture     (const char *deviceName);
               ~ElpVideoCapture     ();
                void captureFrame   (const char* outputFileName);
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


void ElpVideoCapture::captureFrame (const char* outputFileName)
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
}

int main (int argc, char* argv[]) 
{
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
                    devId ? v1.captureFrame (FRAME_1) : v0.captureFrame (FRAME_0);  
                }            
            }
        }
        
        #pragma omp parallel for
        for(int devId=0; devId<2; devId++)
        {
            devId ? v1.captureFrame (FRAME_1) : v0.captureFrame (FRAME_0);  
        }


    }

    clock_gettime(CLOCK_REALTIME, &requestEnd);
    
    // Calculate time it took
    long int total_usec_elapsed = (requestEnd.tv_sec*1e9+requestEnd.tv_nsec - requestStart.tv_sec*1e9-requestStart.tv_nsec)/1e3;
    long int avg_usec_elapsed = total_usec_elapsed/NUM_IMAGES;
    printf ("micro seconds elapsed for capturing %d frames: %ld\n", NUM_IMAGES, total_usec_elapsed);
    printf ("average micro seconds elapsed for capturing 1 frame pair: %ld\n", avg_usec_elapsed);

    
    
    return 0;
}
