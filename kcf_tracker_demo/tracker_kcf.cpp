// c++
#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <thread>
#include <chrono>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>  
#include <geometry_msgs/Pose.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml.hpp>
#include "std_msgs/Int32MultiArray.h"
#include "kcftracker.hpp"
#include "ImageQueue.h"

using namespace std;
using namespace cv;

#define IMAGEQ_NAME "buffer"
#define MARKER_SIZE 0.18
#define Interval 2
#define F1 300
#define F2 300
#define C1 320
#define C2 240

static const std::string RGB_WINDOW = "RGB Image window";
int id = 0;
//! Camera related parameters.
int frameWidth_;
int frameHeight_;

std_msgs::Header imageHeader_;
cv::Mat camImageCopy_;
boost::shared_mutex mutexImageCallback_;
bool imageStatus_ = false;
boost::shared_mutex mutexImageStatus_;

// 图像接收回调函数，接收web_cam的话题，并将图像保存在camImageCopy_中
bool FromMemery = false;
void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_DEBUG("[EllipseDetector] USB image received.");

    cv_bridge::CvImagePtr cam_image;

    try {
        cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        imageHeader_ = msg->header;
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (cam_image) {
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            camImageCopy_ = cam_image->image.clone();
        }
        {
            boost::unique_lock<boost::shared_mutex> lockImageStatus(mutexImageStatus_);
            imageStatus_ = true;
        }
        frameWidth_ = cam_image->image.size().width;
        frameHeight_ = cam_image->image.size().height;
    }
    return;
}

// 用此函数查看是否收到图像话题
bool getImageStatus(void)
{
    boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
    return imageStatus_;
}

//! ROS subscriber and publisher.
image_transport::Subscriber imageSubscriber_;
ros::Publisher pose_pub;


cv::Rect selectRect;
cv::Point origin;
cv::Rect result;


bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;


// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

image_transport::Publisher image_pub;

void arrayCallback(const std_msgs::Int32MultiArray::ConstPtr& array)
{

	int i = 0;
    int Arr[6];
	// print all the remaining numbers
	for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		Arr[i] = *it;
		i++;    
	}
    id = Arr[1];
    selectRect.x = Arr[2];
    selectRect.y = Arr[3];
    selectRect.width = Arr[4];
    selectRect.height = Arr[5];
    printf("bbox of id is=%d\n",id);
    bRenewROI = true;
    if(bRenewROI) printf("set the init\n");
    FromMemery = true;
	return;
}
int main(int argc, char **argv)
{

    ros::init(argc, argv, "tracker_ros");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); 
    ros::Rate loop_rate(30);
    
    // 接收图像的话题
    imageSubscriber_ = it.subscribe("/camera/rgb/image_raw", 1, cameraCallback);

    // 椭圆检测结果，xyz
    pose_pub = nh.advertise<geometry_msgs::Pose>("/vision/ellipse", 1);
    
    sensor_msgs::ImagePtr msg_ellipse;

    const auto wait_duration = std::chrono::milliseconds(2000);

    //cv::namedWindow(RGB_WINDOW);
    //cv::setMouseCallback(RGB_WINDOW, onMouse, 0);

    auto imageQ = imagequeue::ImageQueue();

    image_pub = it.advertise("/camera/rgb/trackmsg", 1);
    sensor_msgs::ImagePtr krackmsg;
    ros::Subscriber sub2 = nh.subscribe("bbox", 40, arrayCallback);
    Mat frame;
    /*
    FromMemery = true;
    bRenewROI  = true;
    selectRect.x = 200;
    selectRect.y = 200;
    selectRect.width = 200;
    selectRect.height = 200;
    */
    while (ros::ok())
    {
        while (!getImageStatus()) 
        {
            printf("Waiting for image.\n");
            std::this_thread::sleep_for(wait_duration);
            ros::spinOnce();
        }
        
        
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            frame = camImageCopy_.clone();
        }
        
        static bool need_tracking_det = false;
        
        
        if(FromMemery){
            //打开缓冲队列                    
            if (!imageQ.opened()) {
                try {
                    printf("tracker trying to link the queue\n");
                    if(imageQ.open(IMAGEQ_NAME)) printf("succeed to open the quene!");
                    else printf("not open the quene");
                    } catch (std::system_error &ex) {
                        printf("error when imageQ.open");
                }
            }     
            if(imageQ.find(id, frame)) printf("get the frame\n");
            else printf("NO the frame\n");
            while(imageQ.find(id, frame)){
                printf("id=%d\n",id);
                if (bRenewROI){
                    tracker.init(selectRect, frame);
                    cv::rectangle(frame, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);
                    bRenewROI = false;
                    bBeginKCF = true;
                    id = id+1;
                    printf("tracker have inited!\n");
                }
                else if (bBeginKCF){
                    result = tracker.update(frame);
                    printf("tracker have updated frome memery!\n");
                    id = id+Interval;
                    cv::rectangle(frame, result, cv::Scalar(255, 0, 0), 2, 8, 0);
                    krackmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
                    // 将图像通过话题发布出去
                    image_pub.publish(krackmsg); 
                    ros::Duration(0.002).sleep();
                   
                }
            }
            FromMemery = false;
        }
        
        else if (bBeginKCF){
            result = tracker.update(frame);
            printf("tracker have updated frome web_cam!\n");
            cv::rectangle(frame, result, cv::Scalar(255, 0, 0), 2, 8, 0);
            
        }
        else if(!bBeginKCF){
            printf("Waiting for bbox.\n");
        }
        
        krackmsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        // 将图像通过话题发布出去
        image_pub.publish(krackmsg);
        //imshow(RGB_WINDOW, frame);
        //waitKey(1);
        

        ros::spinOnce();
        loop_rate.sleep();
    }
}




