#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <string>
#include <vector>
#include <unordered_map>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

template < typename T > std::string to_string( const T& n )
{
    std::ostringstream stm ;
    stm << n ;
    return stm.str() ;
}

void transform_to_array(geometry_msgs::TransformStamped transform, double* data_array)
{
  data_array[0] = transform.transform.translation.x;
  data_array[1] = transform.transform.translation.y;
  data_array[2] = transform.transform.translation.z;
  data_array[3] = transform.transform.rotation.x;
  data_array[4] = transform.transform.rotation.y;
  data_array[5] = transform.transform.rotation.z;
  data_array[6] = transform.transform.rotation.w;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "cobotta_server");
  ros::NodeHandle nh("~");

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  double timeout = 10.0;

  std::string reference_link_name = "world";
  std::string object_basename = "object";
  unsigned int object_number = 1;
  std::string object_name = object_basename + "_" + to_string(object_number);

  std::vector<geometry_msgs::TransformStamped> object_transforms;
  std::unordered_map<std::string, std::vector<geometry_msgs::TransformStamped>> client_trasforms;
  client_trasforms["cobotta_center"] = std::vector<geometry_msgs::TransformStamped>();
  client_trasforms["cobotta_right"] = std::vector<geometry_msgs::TransformStamped>();
  client_trasforms["cobotta_left"] = std::vector<geometry_msgs::TransformStamped>();
  geometry_msgs::TransformStamped object_transform;

  const int buf_size = 8000;
  double object_transform_data[buf_size];
  memset(object_transform_data, 0, sizeof(object_transform_data));

  int sockfd, client1_sockfd, client2_sockfd, client3_sockfd;
  std::unordered_map<std::string, std::string> clients_name_info = {
    {"192.168.1.1", "cobotta_center"},
    {"192.168.1.2", "cobotta_right"},
    {"192.168.1.3", "cobotta_left"},
  };
  std::unordered_map<std::string, int> clients_sockfd_info;
  struct sockaddr_in addr, from_addr;
  struct in_addr inaddr;
  socklen_t socklen = sizeof(struct sockaddr_in);
  const int port_num = nh.param<int>("port_num", 50000);

  addr.sin_family = AF_INET;
  addr.sin_port = htons(50000);
  addr.sin_addr.s_addr = INADDR_ANY;

  if(bind(sockfd, (struct sockaddr*)&addr, sizeof(addr)) < 0)
  {
    perror("bind");
  }

  if(listen(sockfd, SOMAXCONN) < 0)
  {
    perror("listen");
  }

  if((client1_sockfd = accept(sockfd, (struct sockaddr*)&from_addr, &socklen)) < 0)
  {
    perror("accept1");
  }
  inaddr.s_addr = from_addr.sin_addr.s_addr;
  clients_sockfd_info[inet_ntoa(inaddr)] = client1_sockfd;
  ROS_INFO("%s connected.", inet_ntoa(inaddr));

  if((client2_sockfd = accept(sockfd, (struct sockaddr*)&from_addr, &socklen)) < 0)
  {
    perror("accept2");
  }

  if((client3_sockfd = accept(sockfd, (struct sockaddr*)&from_addr, &socklen)) < 0)
  {
    perror("accept3");
  }

  ros::Time lookup_time = ros::Time(0);

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

  std::cout << object_transforms.size() << std::endl;

  // Add algorithm for distribute tasks to each client here
  int robot_num = 0;
  std::string robot_names[] = {
    "cobotta_center", "cobotta_right", "cobotta_left"
  };
  for(auto trans : object_transforms)
  {
    client_trasforms[robot_names[robot_num]].push_back(trans);
    robot_num++;
    if(robot_num < 2)
    {
      robot_num = 0;
    }
  }
  // end dummy argorithm

  robot_num = 0;
  for(auto client_trasform : client_trasforms)
  {
    for(auto trans : client_trasform.second)
    {
      // TODO: convert TF from world frame to based on each robot base frame
      transform_to_array(trans, object_transform_data);
      write(clients_sockfd_info[client_trasform.first], object_transform_data, buf_size);
    }
  }

  close(client1_sockfd);
  close(client2_sockfd);
  close(client3_sockfd);
  close(sockfd);

  return 0;
}
