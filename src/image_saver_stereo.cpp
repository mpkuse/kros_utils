#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


std::string COMPRESSED_LEFT_IMAGE_TOPIC =  "/camera/fisheye1/image_raw/compressed";
std::string COMPRESSED_RIGHT_IMAGE_TOPIC =  "/camera/fisheye2/image_raw/compressed";

std::string LEFT_IMAGE_TOPIC =  "/camera/fisheye1/image_raw";
std::string RIGHT_IMAGE_TOPIC =  "/camera/fisheye2/image_raw";


std::string BASE = "/home/manohar/try/sync_stereo_pairs/";
std::string LEFT_PREFIX = "leftimage-";
std::string RIGHT_PREFIX = "rightimage-";
std::string POSTFIX = ".png";

ros::Duration save_every_sec = ros::Duration( 0.5 );


// GLOBAL variables
ros::Time last_save(0,0);


/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    this->signalMessage(msg);
  }
};

// Callback for synchronized messages
void callback(const sensor_msgs::Image::ConstPtr &l_img,
              const sensor_msgs::Image::ConstPtr &r_img
              // const sensor_msgs::CameraInfo::ConstPtr &l_info,
              // const sensor_msgs::CameraInfo::ConstPtr &r_info
          )
{
    std::cout << "callback\t";
    std::cout << "l_img.t=" << l_img->header.stamp << "\t";
    std::cout << "r_img.t=" << r_img->header.stamp << "\t";

    ros::Duration diff = l_img->header.stamp - last_save;
    if( diff < save_every_sec )
    {
        std::cout << "....SKIP\n";
        return;
    }
    std::cout << std::endl;


    last_save = l_img->header.stamp;


  // Stereo dataset is class variable to store data
  // TODO PROCESSS IMAGE
  cv::Mat l_image = cv_bridge::toCvShare(l_img, "bgr8")->image;
  cv::Mat r_image = cv_bridge::toCvShare(r_img, "bgr8")->image;

  char padding[50];
  #if 0 // 0 for i, 1 for timestamp
  sprintf( padding, "%06d", i );
  #else
  sprintf( padding, "%ld", l_img->header.stamp.toNSec() );
  #endif

  std::string fname_leftimage =  BASE+"/"+LEFT_PREFIX+padding+POSTFIX;
  std::string fname_rightimage =  BASE+"/"+RIGHT_PREFIX+padding+POSTFIX;
  std::cout << "Write file" << fname_leftimage << ", " << fname_rightimage << "\n";
  cv::imwrite(fname_leftimage, l_image );
  cv::imwrite(fname_rightimage, r_image );

  cv::imshow( "left view", l_image );
  cv::imshow( "right_view", r_image );
  cv::waitKey(10);

}


// Callback for synchronized messages
void callback_compressed(const sensor_msgs::CompressedImage::ConstPtr &l_img,
              const sensor_msgs::CompressedImage::ConstPtr &r_img
              // const sensor_msgs::CameraInfo::ConstPtr &l_info,
              // const sensor_msgs::CameraInfo::ConstPtr &r_info
          )
{
    std::cout << "callback_compressed\t";
    std::cout << "l_img.t=" << l_img->header.stamp << "\t";
    std::cout << "r_img.t=" << r_img->header.stamp << "\t";

    ros::Duration diff = l_img->header.stamp - last_save;
    if( diff < save_every_sec )
    {
        std::cout << "....SKIP\n";
        return;
    }
    std::cout << std::endl;


    last_save = l_img->header.stamp;

    // TODO SAVE/PROCESS IMAGE PAIR
    cv::Mat l_image = cv::imdecode(cv::Mat(l_img->data),1);//convert compressed image data to cv::Mat
    cv::Mat r_image = cv::imdecode(cv::Mat(r_img->data),1);//convert compressed image data to cv::Mat

    char padding[50];
    #if 0 // 0 for i, 1 for timestamp
    sprintf( padding, "%06d", i );
    #else
    sprintf( padding, "%ld", l_img->header.stamp.toNSec() );
    #endif

    std::string fname_leftimage =  BASE+"/"+LEFT_PREFIX+padding+POSTFIX;
    std::string fname_rightimage =  BASE+"/"+RIGHT_PREFIX+padding+POSTFIX;
    std::cout << "Write file" << fname_leftimage << ", " << fname_rightimage << "\n";
    cv::imwrite(fname_leftimage, l_image );
    cv::imwrite(fname_rightimage, r_image );

    cv::imshow( "left view", l_image );
    cv::imshow( "right_view", r_image );
    cv::waitKey(10);

}

// Load bag
void loadBag(const std::string &filename)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);


    std::string l_cam_image = LEFT_IMAGE_TOPIC; //"/camera/fisheye1/image_raw";
    std::string r_cam_image = RIGHT_IMAGE_TOPIC; //"/camera/fisheye2/image_raw";

  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(l_cam_image);
  topics.push_back(r_cam_image);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // Set up fake subscribers to capture images
  BagSubscriber<sensor_msgs::Image> l_img_sub, r_img_sub;
  // BagSubscriber<sensor_msgs::CameraInfo> l_info_sub, r_info_sub;

  // Use time synchronizer to make sure we get properly synchronized images
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(l_img_sub, r_img_sub, 25);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  // Load all messages into our stereo dataset
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if (m.getTopic() == l_cam_image || ("/" + m.getTopic() == l_cam_image))
    {
      sensor_msgs::Image::ConstPtr l_img = m.instantiate<sensor_msgs::Image>();
      if (l_img != NULL)
        l_img_sub.newMessage(l_img);
    }

    if (m.getTopic() == r_cam_image || ("/" + m.getTopic() == r_cam_image))
    {
      sensor_msgs::Image::ConstPtr r_img = m.instantiate<sensor_msgs::Image>();
      if (r_img != NULL)
        r_img_sub.newMessage(r_img);
    }

  }
  bag.close();
}


// Load bag compressed image
void loadBagCompressed(const std::string &filename)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);


  std::string l_cam_image = COMPRESSED_LEFT_IMAGE_TOPIC; //"/camera/fisheye1/image_raw/compressed";
  std::string r_cam_image = COMPRESSED_RIGHT_IMAGE_TOPIC; //"/camera/fisheye2/image_raw/compressed";

  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(l_cam_image);
  topics.push_back(r_cam_image);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // Set up fake subscribers to capture images
  BagSubscriber<sensor_msgs::CompressedImage> l_img_sub, r_img_sub;
  // BagSubscriber<sensor_msgs::CameraInfo> l_info_sub, r_info_sub;

  // Use time synchronizer to make sure we get properly synchronized images
  message_filters::TimeSynchronizer<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> sync(l_img_sub, r_img_sub, 25);
  sync.registerCallback(boost::bind(&callback_compressed, _1, _2));

  // Load all messages into our stereo dataset
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if (m.getTopic() == l_cam_image || ("/" + m.getTopic() == l_cam_image))
    {
      sensor_msgs::CompressedImage::ConstPtr l_img = m.instantiate<sensor_msgs::CompressedImage>();
      if (l_img != NULL)
        l_img_sub.newMessage(l_img);
    }

    if (m.getTopic() == r_cam_image || ("/" + m.getTopic() == r_cam_image))
    {
      sensor_msgs::CompressedImage::ConstPtr r_img = m.instantiate<sensor_msgs::CompressedImage>();
      if (r_img != NULL)
        r_img_sub.newMessage(r_img);
    }


  }
  bag.close();
}

int main( int argc, char ** argv )
{
    std::cout << "+++++ image_saver_stereo +++++\n";
    ros::init(argc, argv, "image_saver_stereo");
    ros::NodeHandle nh("~");

    if( argc != 2 )
    {
        std::cout << "INVALID USAGE....\nsample usage:\n\t" << argv[0] << " " << "filename.bag" << std::endl;
        exit(1);
    }
    std::cout << "Load Bag : " << argv[1] << " argc="<< argc << std::endl;


    // loadBag( argv[1] );
    loadBagCompressed( argv[1] );
}
