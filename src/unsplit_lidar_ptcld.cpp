/// @file This tool will help to unsplit the lidar cloud.
/// Some lidar drivers publish every cloud as a scan. These
/// are often at very high frequency like 1500 hz for example.
/// This tool cummulates N pointclouds and publishes at 1500/N hz.
/// Usually you want to set N so that 1500/N becomes 10.
#include <iostream>
#include <string>
using namespace std;

#include <ros/ros.h>
#include <std_msgs/String.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>

class UnSplitLidarCloud
{
    public:
    UnSplitLidarCloud()
    {
        //---- Read params
        ros::NodeHandle private_nh("~");
        private_nh.param("LIDARCLOUD_TOPIC_NAME", LIDARCLOUD_TOPIC_NAME, std::string("xxhorizontal_laser_3d"));
        private_nh.param("OUTPUTCLOUD_TOPIC_NAME", OUTPUTCLOUD_TOPIC_NAME, std::string("xxoutput"));

        private_nh.param("cummulate_N_clouds", cummulate_N_clouds, -1 );
        assert( cummulate_N_clouds > 1 );


        // LIDARCLOUD_TOPIC_NAME = "horizontal_laser_3d";
        // OUTPUTCLOUD_TOPIC_NAME = "output";

        //---- Subscribes and Publishers
        cout  << "Subscribe LIDARCLOUD_TOPIC_NAME=" << LIDARCLOUD_TOPIC_NAME  << endl;
        sub_lidar = nh_.subscribe(LIDARCLOUD_TOPIC_NAME, 1000, &UnSplitLidarCloud::loam_pointcloud_callback, this);

        cout << "Advertise OUTPUTCLOUD_TOPIC_NAME=" << OUTPUTCLOUD_TOPIC_NAME << endl;
        pub_output_cloud = nh_.advertise<sensor_msgs::PointCloud2>(OUTPUTCLOUD_TOPIC_NAME, 100);



    }

    //---- callbacks
    void loam_pointcloud_callback( const sensor_msgs::PointCloud2::ConstPtr& cloud_msg )
    {
        idx++;
        if( idx > 0 )
        {
            double delta_t = cloud_msg->header.stamp.toSec() - prev_stamp.toSec();
            input_data_rate = 1.0 / delta_t;
        }

        // Container for original & filtered data
        pcl::PointCloud<pcl::PointXYZ> cloud;

        // Convert to PCL data type
        pcl::fromROSMsg(*cloud_msg, cloud );
        mergedPointCloud += cloud;


        #if 1
        cout << ".";
        #else
        cout << "===\n";
        cout << "cloud.size= " << cloud.size() << endl;
        cout << "mergedPointCloud.size (before)= " << mergedPointCloud.size() << endl;
        cout << "mergedPointCloud.size (after)= " << mergedPointCloud.size() << endl;
        #endif

        if( idx > 0 && idx%cummulate_N_clouds == 0 )
        {
            idx_publish++;
            if( idx_publish > 0 ) {
                double delta_t = cloud_msg->header.stamp.toSec() - prev_publish_stamp.toSec();
                publish_rate = 1.0 / delta_t;
            }

            // publish mergedPointCloud

            sensor_msgs::PointCloud2 msg_output;
            pcl::toROSMsg(mergedPointCloud, msg_output );
            msg_output.header = cloud_msg->header;

            // cout << "mergedPointCloud.size = " << mergedPointCloud.size() << endl;
            // cout << "msg_output size = " << msg_output.width * msg_output.height << endl;
            pub_output_cloud.publish( msg_output );




            cout << "publish merged # " << idx_publish << "\n";
            cout << "input cloud is @ " << input_data_rate << "hz " << endl;
            cout << "publish_rate is @ " << publish_rate << "hz " << endl;

            prev_publish_stamp = cloud_msg->header.stamp;
            mergedPointCloud.clear();
        }

        prev_stamp = cloud_msg->header.stamp;




    }

    private:
    // ros
    ros::NodeHandle nh_;
    ros::Subscriber sub_lidar;
    ros::Publisher pub_output_cloud;

    // global variables
    string LIDARCLOUD_TOPIC_NAME, OUTPUTCLOUD_TOPIC_NAME;

    int idx = -1;
    ros::Time prev_stamp;
    double input_data_rate = -1;


    int idx_publish = -1;
    ros::Time prev_publish_stamp;
    double publish_rate = -1;

    int cummulate_N_clouds = 150;
    pcl::PointCloud<pcl::PointXYZ>  mergedPointCloud;

};

int main(int argc, char ** argv )
{
    ros::init(argc, argv, "unsplit_lidar_ptcld");
    std::cout<<"+++ unsplit_lidar_ptcld +++ \n";

    UnSplitLidarCloud fusion_;

    ros::spin();
    return 0;
}
