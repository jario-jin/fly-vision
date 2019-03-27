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
#include "std_msgs/Int32MultiArray.h"

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
    ROS_DEBUG("IDimage received.");

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

//读取照片中的ID，如果返回值是-1，则读取失败
int read_id(Mat frame){
    int id;
    Mat idmark;
    vector<Mat> channel(3);
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    cv::Rect idroi = cv::Rect(0, 0, 24, 24);
    frame(idroi).copyTo(idmark);
    split(idmark, channel);

    vector< int > id_1;
    vector< int > id_k;
    vector< int > id_m;
    vector< vector<Point2f> > markerCorners;
    cv::aruco::detectMarkers(channel[0], dictionary, markerCorners, id_1);
    if(id_1.size()>0) {
        cv::aruco::detectMarkers(channel[1], dictionary, markerCorners, id_k);
        if(id_k.size()>0) {
            cv::aruco::detectMarkers(channel[2], dictionary, markerCorners, id_m);
            if(id_m.size()>0) {
                id = id_m[0]*1000*1000+id_k[0]*1000+id_1[0];
            }
            else id = -1;
        }
        else id = -1;
    }
    else id = -1;

    return id;	
}

bool select_flag = false;
bool selected = false;
cv::Rect selectRect;
cv::Point origin;
 Mat frame;
void onMouse(int event, int x, int y, int, void*)
{
    if (select_flag)
    {
        selectRect.x = MIN(origin.x, x);        
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);   
        selectRect.height = abs(y - origin.y);
        selectRect &= cv::Rect(0, 0, frameWidth_, frameHeight_);
        cv::rectangle(frame, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        select_flag = true; 
        origin = cv::Point(x, y);       
        selectRect = cv::Rect(x, y, 0, 0);  
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        if (selectRect.width*selectRect.height < 64)
        {
            ;
        }
        else
        {
            select_flag = false;
            selected = true;
        } 
    }
}

ros::Publisher pub_bbox;
std_msgs::Int32MultiArray bbox;
int id = 0;
void timerCallback(const ros::TimerEvent& event)
{
    pub_bbox.publish(bbox);
    printf("id=%d\n",id);
}

int main(int argc, char **argv)
{
  
    ros::init(argc, argv, "GroundStation");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); 
    ros::Rate loop_rate(30);

    cv::namedWindow("GroundStation");
    cv::setMouseCallback("GroundStation", onMouse, 0);    
    
    pub_bbox = nh.advertise<std_msgs::Int32MultiArray>("bbox", 40);
   
    // 接收图像的话题
    imageSubscriber2 = it.subscribe("/camera/rgb/trackmsg", 1, cameraCallback);
    //const auto wait_duration = std::chrono::milliseconds(2000)
    ros::Timer timer;
    while (ros::ok()){
        
        while (!getImageStatus()) {
            printf("Waiting for image.\n");
            //std::this_thread::sleep_for(wait_duration);
        ros::spinOnce();
            }
        
       
        {
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            frame = camImageCopy_.clone();
        }
        if(select_flag){
            cv::rectangle(frame, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);
        }
        if(selected){
            id = read_id(frame);
            bbox.data={0,id,selectRect.x,selectRect.y,selectRect.width,selectRect.height};
            
            //ros::Duration(1).sleep();
            timer = nh.createTimer(ros::Duration(1), timerCallback, true);
            selected=false;
        }
       
        imshow("GroundStation", frame);
        waitKey(1);
        
        
        ros::spinOnce();
        loop_rate.sleep();
	}
		
}

