#include <ros/ros.h>  
#include <image_transport/image_transport.h>  
#include <cv_bridge/cv_bridge.h>  
#include <sensor_msgs/image_encodings.h>  
#include <geometry_msgs/Pose.h>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
using namespace std;
using namespace cv;


image_transport::Publisher image_pub;

Mat add_id(int id, Mat frame){
    cv::Mat markerImage; 
    vector<Mat> channel(3);
    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    channel[0]  = cv::Mat::zeros(24, 24, CV_8UC1);
    channel[1]  = cv::Mat::zeros(24, 24, CV_8UC1);
    channel[2]  = cv::Mat::zeros(24, 24, CV_8UC1);
    
    channel[0].setTo(cv::Scalar(255));
    channel[1].setTo(cv::Scalar(255));
    channel[2].setTo(cv::Scalar(255));
    cv::Rect myroi = cv::Rect(4, 4, 16, 16);
    cv::Rect idroi = cv::Rect(0, 0, 24, 24);
    
    int id_m = id/1000/1000;
    int id_k = id/1000-id_m*1000;
    int id_1 = id%1000%1000;
    
    dictionary.drawMarker(id_1, 16, markerImage, 1);
    markerImage.copyTo(channel[0](myroi));
    dictionary.drawMarker(id_k, 16, markerImage, 1);
    markerImage.copyTo(channel[1](myroi));
    dictionary.drawMarker(id_m, 16, markerImage, 1);
    markerImage.copyTo(channel[2](myroi));
	
    Mat idmark;
    merge(channel, idmark);
    idmark.copyTo(frame(idroi));
    return frame;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "web_cam");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); 
    ros::Rate loop_rate(30);
    // 在这里修改发布话题名称
    image_pub = it.advertise("/camera/rgb/image_raw", 1);

    // 用系统默认驱动读取摄像头0，使用其他摄像头ID，请在这里修改
    cv::VideoCapture cap(0);
    cv::Mat frame;

    sensor_msgs::ImagePtr msg;
    int id = 0;
    while (ros::ok())
	{
        cap >> frame;
        if (!frame.empty())
        {
            //添加ID            
            frame = add_id(id,frame);
            id = id+1;            
            // 设置图像帧格式->bgr8
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            // 将图像通过话题发布出去
            image_pub.publish(msg); 
        }
        ros::spinOnce();
        // 按照设定的帧率延时，ros::Rate loop_rate(30)
        loop_rate.sleep();
    }

    cap.release();
}




