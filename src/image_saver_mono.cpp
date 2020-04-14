#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
using namespace std;

std::string BASE = "/home/manohar/try/left_images/";
std::string PREFIX = "image";
std::string POSTFIX = ".png";

ros::Duration save_every_sec = ros::Duration( 1, 0 );

// Global variables
int i=0;
ros::Time last_save(0,0);

void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    std::cout << "Received Image t=" << msg->header.stamp << endl;;
    try
    {
        cv::Mat image = cv::imdecode(cv::Mat(msg->data),1);//convert compressed image data to cv::Mat

        ros::Duration diff = msg->header.stamp - last_save;
        // cout << "diff=" << diff << endl;
        if( diff > save_every_sec )
        {
            last_save = msg->header.stamp;

            #if 1
            char padding[50];
            sprintf( padding, "%06d", i );
            std::string fname =  BASE+"/"+PREFIX+padding+POSTFIX;
            i++;
            std::cout << "Write file" << fname << "\n";
            cv::imwrite(fname, image );
            // #else
            cv::imshow("view", image);
            cv::waitKey(10);
            #endif
        } else
        {
            // std::cout << "NOP\n";
        }
    }
    catch (cv_bridge::Exception& e)
    {
    ROS_ERROR("Could not convert to image!");
    }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_saver_mono");
  ros::NodeHandle nh("~");
  // cv::namedWindow("view");
  // cv::startWindowThread();

  std::string COMPRESSED_IMAGE_TOPIC =  "/camera/fisheye1/image_raw/compressed";
  std::cout << "COMPRESSED_IMAGE_TOPIC = " << COMPRESSED_IMAGE_TOPIC << std::endl;
  ros::Subscriber sub = nh.subscribe(COMPRESSED_IMAGE_TOPIC, 1, imageCallback);
  ros::spin();
}
