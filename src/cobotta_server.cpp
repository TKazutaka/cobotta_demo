#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <string>
#include <vector>
#include <map> 
#include <unordered_map>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <iostream>
#include <cmath>

// #define PRINT(...) \
// std::cout << "[" << __LINE__ << "] " << #__VA_ARGS__ << std::endl; \
// __VA_ARGS__;

#define PRINT(...) __VA_ARGS__

  struct data
  {
    std::string name;
    double distance;

  };

    auto operator<<(std::ostream& os, const data& datum)
      -> std::ostream&
    {
      return os << "{" << datum.name << ", " << datum.distance << "}";
    }


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
  //data_array[3] = transform.transform.rotation.x;
  //data_array[4] = transform.transform.rotation.y;
  //data_array[5] = transform.transform.rotation.z;
  //data_array[6] = transform.transform.rotation.w;
}

double cobotta_center_array[] = {-0.35, 0.0, 0.0};
double cobotta_right_array[] = {0.10, -0.35, 0.0};
double cobotta_left_array[] = {0.10, 0.35, 0.0};

int distance(double x,double y){
  double distance = 0.0;
  distance = x;
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
  std::string object_name = object_basename + "_" + to_string(object_number) + "_static";

  std::unordered_map<std::string, std::vector<geometry_msgs::TransformStamped>> object_transforms;
  std::unordered_map<std::string, std::vector<geometry_msgs::TransformStamped>> client_trasforms;
  std::map<std::string, double > cobotta_center_to_object_distance;
  std::map<std::string, double > cobotta_right_to_object_distance;
  std::map<std::string, double > cobotta_left_to_object_distance;
  client_trasforms["cobotta_center"] = std::vector<geometry_msgs::TransformStamped>();
  client_trasforms["cobotta_right"] = std::vector<geometry_msgs::TransformStamped>();
  client_trasforms["cobotta_left"] = std::vector<geometry_msgs::TransformStamped>();
  geometry_msgs::TransformStamped object_transform;

  const int buf_size = 24;
  double object_transform_data[3];
  memset(object_transform_data, 0, sizeof(object_transform_data));

  int sockfd, client1_sockfd, client2_sockfd, client3_sockfd;
  std::unordered_map<std::string, std::string> clients_name_info = {
    {"192.168.0.103", "cobotta_center"},
    {"192.168.0.102", "cobotta_right"},
    {"192.168.0.101", "cobotta_left"},
  };
  std::unordered_map<std::string, int> clients_sockfd_info;
  struct sockaddr_in addr, from_addr;
  struct in_addr inaddr;
  socklen_t socklen = sizeof(struct sockaddr_in);
  const int port_num = nh.param<int>("port_num", 50000);

  if ((sockfd= socket(AF_INET, SOCK_STREAM, 0)) <0)
  {
   perror("socket");
  }


  addr.sin_family = AF_INET;
  addr.sin_port = htons(50000);
  addr.sin_addr.s_addr = INADDR_ANY;

  int yes = 1;

  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes)) < 0)
  {
    perror("ERROR on setsockopt");
    exit(1);
  }


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
  clients_sockfd_info[clients_name_info[inet_ntoa(inaddr)]] = client1_sockfd;
  ROS_INFO("%s connected.", inet_ntoa(inaddr));

  if((client2_sockfd = accept(sockfd, (struct sockaddr*)&from_addr, &socklen)) < 0)
  {
    perror("accept2");
  }
  inaddr.s_addr = from_addr.sin_addr.s_addr;
  clients_sockfd_info[clients_name_info[inet_ntoa(inaddr)]] = client2_sockfd;
  ROS_INFO("%s connected.", inet_ntoa(inaddr));

  if((client3_sockfd = accept(sockfd, (struct sockaddr*)&from_addr, &socklen)) < 0)
  {
    perror("accept3");
  }
  inaddr.s_addr = from_addr.sin_addr.s_addr;
  clients_sockfd_info[clients_name_info[inet_ntoa(inaddr)]] = client3_sockfd;
  ROS_INFO("%s connected.", inet_ntoa(inaddr));

  ros::Time lookup_time = ros::Time(0);

  while(true)
  {
    try
    {
      std::cout << object_name << std::endl;
      object_transform = tf_buffer.lookupTransform(reference_link_name, object_name, lookup_time, ros::Duration(timeout));
      object_transforms[object_name].push_back(object_transform);
    }
    catch(tf2::TransformException &ex)
    {
      ROS_INFO("Found %d objects", object_number - 1);
      break;
    }
    object_number++;
    object_name = object_basename + "_" + to_string(object_number) + "_static";
  }

  // Add algorithm for distribute tasks to each client here
  int robot_num = 0;
  std::string robot_names[] = {
    "cobotta_center", "cobotta_right", "cobotta_left"
  };
  std::string static_object_name;
    std::cout <<"hypot_befor" << std::endl;
  for(auto trans : object_transforms)
  {
    for(auto cobotta_trans : trans.second)
    {
      cobotta_center_to_object_distance[trans.first] 
        = std::hypot(
            cobotta_center_array[0] - cobotta_trans.transform.translation.x,
            cobotta_center_array[1] - cobotta_trans.transform.translation.y);

      cobotta_right_to_object_distance[trans.first]
        = std::hypot(
            cobotta_right_array[0] - cobotta_trans.transform.translation.x,
            cobotta_right_array[1] -cobotta_trans.transform.translation.y);

      cobotta_left_to_object_distance[trans.first] = std::hypot(cobotta_left_array[0] - cobotta_trans.transform.translation.x, cobotta_left_array[1] -cobotta_trans.transform.translation.y);
    }
  }
  std::cout <<"hypot_finish" << std::endl;
  //vectorへ変換
  std::vector<data> center_sort {};
  std::vector<data> right_sort  {};
  std::vector<data> left_sort   {};
  

  for (const auto& each : cobotta_center_to_object_distance)
  {
    center_sort.push_back({each.first, each.second});
    std::cerr << "center_sort: " << center_sort.back() << std::endl;;
  }

  for (const auto& each : cobotta_right_to_object_distance)
  {
    right_sort.push_back({each.first, each.second});
    std::cerr << "right_sort: " << right_sort.back() << std::endl;;
  }

  for (auto itr = cobotta_left_to_object_distance.begin(); itr != cobotta_left_to_object_distance.end(); ++itr)
  {
    left_sort.push_back({itr->first, itr->second});
    std::cerr << "left_sort: " << left_sort.back() << std::endl;;
  }

  // std::vector<data> center_value {cobotta_center_to_object_distance.size()};
  // std::vector<data> right_value  {cobotta_right_to_object_distance.size()};
  // std::vector<data> left_value   {cobotta_left_to_object_distance.size()};

  std::vector<data> center_value;
  std::vector<data> right_value;
  std::vector<data> left_value;
  center_value.clear();
  right_value.clear();
  left_value.clear();

  // assert(center_value.size() == cobotta_center_to_object_distance.size());
  // assert(right_value.size()  == cobotta_right_to_object_distance.size());
  // assert(left_value.size()   == cobotta_left_to_object_distance.size());


  //昇順ソート
  sort(center_sort.begin(), center_sort.end(), [](const data &x, const data &y) { return x.distance < y.distance; }); 

  sort(right_sort.begin(), right_sort.end(), [](const data &x, const data &y) { return x.distance < y.distance; });

  sort(left_sort.begin(), left_sort.end(), [](const data &x, const data &y) { return x.distance < y.distance; });  
  
  int center_number = 0;
  int   left_number = 0;
  int  right_number = 0;
  int  count_number = 0;

  int count = object_number - 1;

  std::cout << "object_number"<<object_number << std::endl;
  std::cout << "count"<<count << std::endl;

  std::cout <<"sort_finish" << std::endl;

  std::vector<std::string> finish_object;
  finish_object.clear();

  int center_count = 0;
  int left_count = 0;
  int right_count = 0;
  bool center_flag = true;
  bool left_flag = true;
  bool right_flag = true;
  bool all_flag = true;

  while (all_flag)
  {
    std::cerr << "; count\t\t; " << count << std::endl;
    std::cerr << "; center_number\t; " << center_number << std::endl;
    std::cerr << ";   left_number\t; " <<   left_number << std::endl;
    std::cerr << ";  right_number\t; " <<  right_number << std::endl;

    #define DISPLAY_EACH_CONTAINER_ELEMENT(CONTAINER)                         \
    std::cerr << "; " #CONTAINER "\t; size " << CONTAINER.size() << std::endl; \
    for (const auto& each : CONTAINER)                                        \
    {                                                                         \
      std::cerr << ";\t\t; " << each << std::endl;                            \
    }

    DISPLAY_EACH_CONTAINER_ELEMENT(center_sort);
    DISPLAY_EACH_CONTAINER_ELEMENT(  left_sort);
    DISPLAY_EACH_CONTAINER_ELEMENT( right_sort);

    DISPLAY_EACH_CONTAINER_ELEMENT(center_value);
    DISPLAY_EACH_CONTAINER_ELEMENT(  left_value);
    DISPLAY_EACH_CONTAINER_ELEMENT( right_value);

    std::cerr << ";" << std::endl;

    while (center_flag)
    {
      if (center_count >= count)
      {
        center_flag = false;
        break;
      }

      if (finish_object.size() == 0)
      {
        center_value.push_back(center_sort[center_count]);
        finish_object.push_back(center_sort[center_count].name);
        center_count += 1;
        break;
      }
      else{
        auto itr = std::find(finish_object.begin(), finish_object.end(), center_sort[center_count].name);
        if (itr != finish_object.end())
        {
          center_count += 1;
          continue;
        }
        else
        {
          center_value.push_back(center_sort[center_count]);
          finish_object.push_back(center_sort[center_count].name);
          center_count += 1;
          break;
        }
      }
    }

    while (left_flag)
    {
      if (left_count >= count)
      {
        left_flag = false;
        break;
      }

      if (finish_object.size() == 0)
      {
        left_value.push_back(left_sort[left_count]);
        finish_object.push_back(left_sort[left_count].name);
        left_count += 1;
        break;
      }
      else
      {
        auto itr = std::find(finish_object.begin(), finish_object.end(), left_sort[left_count].name);
        if (itr != finish_object.end())
        {
          left_count += 1;
          continue;
        }
        else
        {
          left_value.push_back(left_sort[left_count]);
          finish_object.push_back(left_sort[left_count].name);
          left_count += 1;
          break;
        }
      }
    }

    while (right_flag)
    {
      if (right_count >= count)
      {
        right_flag = false;
        break;
      }

      if (finish_object.size() == 0)
      {
        right_value.push_back(right_sort[right_count]);
        finish_object.push_back(right_sort[right_count].name);
        right_count += 1;
        break;
      }
      else
      {
        auto itr = std::find(finish_object.begin(), finish_object.end(), right_sort[right_count].name);
        if (itr != finish_object.end())
        {
          right_count += 1;
          continue;
        }
        else
        {
          right_value.push_back(right_sort[right_count]);
          finish_object.push_back(right_sort[right_count].name);
          right_count += 1;
          break;
        }
      }
    }

    if (center_flag == false && right_flag == false && left_flag == false)
    {
      all_flag = false;
      break;
    }
    else
    {
      continue;
    }

    // if (center_sort[center_number].name == left_sort[left_number].name)
    // {
    //   if (center_sort[center_number].distance >= left_sort[left_number].distance)
    //   {
    //     center_number++;
    //   }
    //   else
    //   {
    //     left_number++;
    //   }

    //   continue;
    // }

    // if(center_sort[center_number].name == right_sort[right_number].name)
    // {
    //   if(center_sort[center_number].distance >= right_sort[right_number].distance)
    //   {
    //     center_number++;
    //   }
    //   else
    //   {
    //     right_number++;
    //   }

    //   continue;
    // }

    // if (right_sort[right_number].name == left_sort[left_number].name)
    // {
    //   if(right_sort[right_number].distance >= left_sort[left_number].distance)
    //   {
    //     right_number++;
    //   }
    //   else
    //   {
    //     left_number++;
    //   }

    //   continue;
    // }

    // if (   center_sort[center_number].name != right_sort[right_number].name
    //     && center_sort[center_number].name != left_sort[left_number].name 
    //     && right_sort[right_number].name != left_sort[left_number].name)
    // {
    //   //assert(center_number < center_value.size());
    //   //assert(left_number   < left_value.size());
    //   //assert(right_number  < right_value.size());

    //   PRINT(center_value[count_number] = center_sort[center_number]);
    //   PRINT(left_value[count_number] = left_sort[left_number]);
    //   PRINT(right_value[count_number] = right_sort[right_number]);
    //   PRINT(center_number++);
    //   PRINT(right_number++);
    //   PRINT(left_number++);
    //   PRINT(count_number++);
      

    //   // std::cout << "=====================" << std::endl;
    //   // std::cout << left_sort[left_number].name << std::endl;
    //   // std::cout << "=====================" << std::endl;
    // }

    // std::cerr << __LINE__ << std::endl;
  }

  std::cout <<"Distribution_finish" << std::endl;

  for(auto left : left_value)
  {
    try
    {
      static_object_name = left.name;
      std::cout << static_object_name << std::endl;
      object_transform = tf_buffer.lookupTransform(robot_names[2], left.name, lookup_time, ros::Duration(timeout));
    }
    catch(tf2::TransformException &ex)
    {
      ROS_ERROR("Cannot find %s objects", static_object_name.c_str());
    }
    client_trasforms[robot_names[2]].push_back(object_transform);
  }
for(auto right : right_value)
  {
    try
    {
      static_object_name = right.name;
      std::cout << static_object_name << std::endl;
      object_transform = tf_buffer.lookupTransform(robot_names[1], right.name, lookup_time, ros::Duration(timeout));
    }
    catch(tf2::TransformException &ex)
    {
      ROS_ERROR("Cannot find %s objects", static_object_name.c_str());
    }
    client_trasforms[robot_names[1]].push_back(object_transform);
  }

for(auto center : center_value)
  {
    try
    {
      static_object_name = center.name;
      std::cout << static_object_name << std::endl;
      object_transform = tf_buffer.lookupTransform(robot_names[0], center.name, lookup_time, ros::Duration(timeout));
    }
    catch(tf2::TransformException &ex)
    {
      ROS_ERROR("Cannot find %s objects", static_object_name.c_str());
    }
    client_trasforms[robot_names[0]].push_back(object_transform);
  }
/*for(auto trans : object_transforms)
  {
    try
    {
      static_object_name = trans.first;
      std::cout << static_object_name << std::endl;
      object_transform = tf_buffer.lookupTransform(robot_names[robot_num], trans.first, lookup_time, ros::Duration(timeout));
    }
    catch(tf2::TransformException &ex)
    {
      ROS_ERROR("Cannot find %s objects", static_object_name.c_str());
    }
    client_trasforms[robot_names[robot_num]].push_back(object_transform);
    robot_num++;
    if(robot_num > 2)
    {
      robot_num = 0;
    }
  }*/
  // end dummy argorithm
  std::cout <<client_trasforms["cobotta_center"].size()<< std::endl;
  std::cout <<client_trasforms["cobotta_left"].size()<< std::endl;
  std::cout <<client_trasforms["cobotta_right"].size()<< std::endl;
  robot_num = 0;
  for(auto client_trasform : client_trasforms)
  {
    for(auto trans : client_trasform.second)
    {
      //transform_to_array(trans, object_transform_data);
      std::cout << "=====================" << std::endl;
      std::cout << client_trasform.first << std::endl;
      std::cout << clients_sockfd_info[client_trasform.first] << std::endl;
      std::cout << trans.transform.translation.x << std::endl;
      std::cout << trans.transform.translation.y << std::endl;
      std::cout << trans.transform.translation.z << std::endl;
      std::cout << "=====================" << std::endl;
      object_transform_data[0] = trans.transform.translation.x;
      object_transform_data[1] = trans.transform.translation.y;
      object_transform_data[2] = trans.transform.translation.z;
      write(clients_sockfd_info[client_trasform.first], object_transform_data, buf_size);
      sleep(1);
    }
  }

  close(client1_sockfd);
  close(client2_sockfd);
  close(client3_sockfd);
  close(sockfd);

  return 0;
}
