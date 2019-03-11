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

//! ROS subscriber and publisher.



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


int id=0;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "read_id");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh); 
    ros::Rate loop_rate(30);


    
    // 接收图像的话题
    imageSubscriber2 = it.subscribe("/camera/rgb/image_id", 1, cameraCallback);
	//const auto wait_duration = std::chrono::milliseconds(2000);

	cv::Mat markerImage; 
	vector<Mat> channel(3);
    	
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    
	Mat idmark;
	cv::Rect idroi=cv::Rect(0, 0,24,24);


	while (ros::ok())
    	{
		while (!getImageStatus()) 
        	{
            printf("Waiting for image.\n");
            //std::this_thread::sleep_for(wait_duration);
            ros::spinOnce();
        	}

        	Mat frame;
        	{
            boost::unique_lock<boost::shared_mutex> lockImageCallback(mutexImageCallback_);
            frame = camImageCopy_.clone();
        	}

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
					id=id_m[0]*1000*1000+id_k[0]*1000+id_1[0];
				printf("sucessed!,ID=%d\n",id);
				}
				else printf("failed!\n");
			}
			else printf("failed!\n");
		}
		else printf("failed!\n");
		 
		imshow("id_1", channel[0]);
		imshow("id_k", channel[1]);
		imshow("id_m", channel[2]);
       	waitKey(5);
        
        ros::spinOnce();
        loop_rate.sleep();

	}
	
	
}




