#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>  
#include <geometry_msgs/Pose.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>
#include <thread>
#include <chrono>
#include <opencv2/aruco.hpp>
#include "ImageQueue.h"

#define IMAGEQ_NAME "buffer"
using namespace std;
using namespace cv;

image_transport::Subscriber imageSubscriber2;

int frameWidth_;
int frameHeight_;

std_msgs::Header imageHeader_;
cv::Mat camImageCopy_;
boost::shared_mutex mutexImageCallback_;
bool imageStatus_ = false;
boost::shared_mutex mutexImageStatus_;


bool getImageStatus(void)
{
    boost::shared_lock<boost::shared_mutex> lock(mutexImageStatus_);
    return imageStatus_;
}


void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_DEBUG("image received.");

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



int main(int argc, char **argv)
{
    int id = 0;
    ros::init(argc, argv, "add_memery");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); 
    ros::Rate loop_rate(30);
    // 接收图像的话题
    imageSubscriber2 = it.subscribe("/camera/rgb/image_raw", 1, cameraCallback);


    Mat frame;

    auto imageQ = imagequeue::ImageQueue();
    
    while (ros::ok()){

        while (!getImageStatus()) {
            printf("Waiting for image.\n");
            ros::spinOnce();
        }
       
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            frame = camImageCopy_.clone();
        }

        if (!imageQ.opened()) {
            try {
                printf("trying to open the queue\n");
                imageQ.open(IMAGEQ_NAME, 1000, frame);
                    } catch (std::system_error &ex) {
                        printf("error: imageQ.open");
                }     
        }
        
        if(imageQ.push(frame)) printf("Successfully pushed\n");
        
        cv::imshow("add_memery",frame);
        if(waitKey(30)==27)    
            ros::shutdown();
        ros::spinOnce();
        loop_rate.sleep();
         
        
	}
		
}

