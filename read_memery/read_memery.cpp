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


int main(int argc, char **argv)
{
    int id = 0;
    ros::init(argc, argv, "read_memery");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);
    

    Mat frame;

    auto imageQ = imagequeue::ImageQueue();
    
    while (ros::ok()){
       
        if (!imageQ.opened()) {
            try {
                printf("trying to link the queue\n");
                imageQ.open(IMAGEQ_NAME);
                    } catch (std::system_error &ex) {
                        printf("error: imageQ.open");
                }     
        }
        
        if (imageQ.find(id, frame)){
            cv::imshow("aa",frame);
            if(waitKey(10)==27)    
            ros::shutdown();
        }
        else{
            printf("Image with ID %u NOT FOUND.\n", id);
        }
        id = id+1;
        ros::spinOnce();
        loop_rate.sleep();
	}
		
}

