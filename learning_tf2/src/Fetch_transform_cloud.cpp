#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/passthrough.h>
#include <std_msgs/Int32.h>

class PointCloud2Transform
{
  public: 
    PointCloud2Transform(ros::NodeHandle n) : node(n), cloud_(new pcl::PointCloud<pcl::PointXYZRGB>()), has_cloud_(true)
    {
      // Target frame to transform cloud
      world_frame_ = "base_link";
      node.getParam("frame_id", world_frame_);

      // Range filter for cloud
      range_filter_.setFilterFieldName("z");
      range_filter_.setFilterLimits(0.0, 1.8);

      // Build a publisher to publish transformed cloud
      g_pub = node.advertise<sensor_msgs::PointCloud2>("base_cloud", 1);

      // Subscribe cloud from camera topic
      ros::Subscriber sub2 = node.subscribe("/request_cloud", 1, &PointCloud2Transform::callback2, this);
      sub_ = node.subscribe("/head_camera/depth_registered/points", 1, &PointCloud2Transform::callback, this);

      ros::Duration(1.0).sleep();
      ros::spin();
    }

  private:
    void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
      if (!has_cloud_)
      {
        // Wait the tf data
        ros::Duration(0.1).sleep();

        // Convert ROS sensor_msgs to pcl data type
        pcl::fromROSMsg(*cloud_msg, *cloud_);

        has_cloud_ = true;
      }
    }

    void callback2(const std_msgs::Int32ConstPtr& msg)
    {
      has_cloud_ = false;

      while (!has_cloud_)
      {
        ros::Duration(0.1).sleep();
        ros::spinOnce();   
      }

      // Filter out noisy long-range points 
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
      range_filter_.setInputCloud(cloud_);
      range_filter_.filter(*cloud_filtered);
      
      // Transform cloud to target frame
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl_ros::transformPointCloud(world_frame_, *cloud_filtered, *transformed_cloud, listener);
      
      // Convert pcl data to ROS type and publish
      sensor_msgs::PointCloud2 output;  
      pcl::toROSMsg(*transformed_cloud, output);
      g_pub.publish (output);
      ROS_INFO_STREAM("Published cloud with width, height, frame: " << transformed_cloud->width << ", " << transformed_cloud->height << ", " << output.header.frame_id);
    }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  bool has_cloud_;
  ros::Subscriber sub_;
  ros::Publisher g_pub; 
  ros::NodeHandle node;
  std::string world_frame_;
  tf::TransformListener listener;
  pcl::PassThrough<pcl::PointXYZRGB> range_filter_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cloud_transform");
  ros::NodeHandle n;
  PointCloud2Transform basecloud(n);
  return 0;
}
