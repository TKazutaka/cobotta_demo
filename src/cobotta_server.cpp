#include <string>
#include <vector>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

template < typename T > std::string to_string( const T& n )
{
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cobotta_server");

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  double timeout = 10.0;

  std::string reference_link_name = "world";
  std::string object_basename = "object";
  unsigned int object_number = 1;
  std::string object_name = object_basename + "_" + to_string(object_number);

  std::vector<geometry_msgs::TransformStamped> object_transforms;
  geometry_msgs::TransformStamped object_transform;

  ros::Time lookup_time = ros::Time::now();

  while(true)
  {
    try
    {
      std::cout << object_name << std::endl;
      object_transform = tf_buffer.lookupTransform(reference_link_name, object_name, lookup_time, ros::Duration(timeout));
    }
    catch(tf2::TransformException &ex)
    {
      ROS_INFO("Found %d objects", object_number - 1);
      break;
    }
    object_transforms.push_back(object_transform);
    object_number++;
    object_name = object_basename + "_" + to_string(object_number);
  }

  std::cout << sizeof(object_transforms)/sizeof(object_transform) << std::endl;

  return 0;
}
