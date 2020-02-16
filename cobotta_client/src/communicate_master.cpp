#include <cobotta_client/communicate_master.h>

#include <iostream>

#include <geometry_msgs/PoseStamped.h>

#include <denso_state_srvs/Moving.h>
#include <denso_gripper_srvs/ActivateGripper.h>
#include <denso_gripper_srvs/Move.h>

static const int MIN_BUFFER_SIZE = 100;
static const int MAX_BUFFER_SIZE = 100000;
static const double WORK_OFFSET = 0.05;

using communicate_master::CommunicateMaster;

// Constructor
CommunicateMaster::CommunicateMaster(ros::NodeHandle& nh) : nh_(nh), sockfd_(-1)
{
  ros::param::param<int>("~port_number", port_number_, 1234); // port number for socket
  ros::param::param<int>("~object_pose_buffer_size", object_pose_buffer_size_, 10000); // recieve data buffer from master
  ros::param::param<int>("~message_buffer_size", message_buffer_size_, 1000); // send data buffer to master
  ros::param::param<std::string>("~master_ip_address", master_ip_address_, "None"); // master ip address
  ros::param::param<double>("~grasp_offset", grasp_offset_, 0.1); // offset for grasping
  moving_service_ = nh.serviceClient<denso_state_srvs::Moving>("/state_behavior/moving");
  gripper_activate_service_ = nh.serviceClient<denso_gripper_srvs::ActivateGripper>("/parallel_gripper/activate");
  gripper_open_service_ = nh.serviceClient<denso_gripper_srvs::Move>("/parallel_gripper/open");
  gripper_close_service_ = nh.serviceClient<denso_gripper_srvs::Move>("/parallel_gripper/close");

  std::cout << "[SUCCESS] Initialize CommunicateMaster class !! " << std::endl;
}

// Destractor
CommunicateMaster::~CommunicateMaster()
{
  close(sockfd_);
  std::cout << "[FINISH] Finish socket connection !!" << std::endl;
}

// Function to connect with master
bool CommunicateMaster::connectMaster()
{
  // Except error of recieving buffer size from master
  if (object_pose_buffer_size_ < MIN_BUFFER_SIZE)
  {
    std::cerr << "[FAILURE] Buffer size is too few to recieve the data from master !! " << std::endl;
    return false;
  }
  else if (object_pose_buffer_size_ > MAX_BUFFER_SIZE)
  {
    std::cerr << "[FAILURE] Buffer size is too much to recieve the data from master !! " << std::endl;
    return false;
  }
  else
  {
    std::cout << "[SUCCESS] Set recieving buffer size to " << object_pose_buffer_size_ << " " << std::endl;
  }

  // Except error of sending buffer size to master
  // if (message_buffer_size_ < MIN_BUFFER_SIZE)
  // {
  //   std::cerr << "[FAILURE] Buffer size is too few to send the data to master !! " << std::endl;
  //   return false;
  // }
  // else if (message_buffer_size_ > MAX_BUFFER_SIZE)
  // {
  //   std::cerr << "[FAILURE] Buffer size is too much to send the data to master !! " << std::endl;
  //   return false;
  // }
  // else
  // {
  //   std::cout << "[SUCCESS] Set sending buffer size to " << message_buffer_size_ << " " << std::endl;
  // }

  // Create socket
  if ((sockfd_ = socket(AF_INET, SOCK_STREAM, 0)) < 0)
  {
    std::cerr << "[FAILURE] Failed to ready client socket !!" << std::endl;
    return false;
  }
  else
  {
    std::cout << "[SUCCESS] Success to ready client socket !!" << std::endl;
  }

  // Convert std::string to char contena for matching type
  char convert_ip[master_ip_address_.size()];
  for (int i=0; i < master_ip_address_.size(); i++)
  {
    convert_ip[i] = master_ip_address_[i];
  }

  addr_.sin_family = AF_INET;
  addr_.sin_port = htons(port_number_);
  addr_.sin_addr.s_addr = inet_addr(convert_ip);

  // Connect with master
  connect(sockfd_, (struct sockaddr* )&addr_, sizeof(struct sockaddr_in));
  std::cout << "[SUCCESS] Connect master !!" << std::endl;

  return true;
}

void* CommunicateMaster::waitReceiveThread(void* p_param)
{
  int recv_size;

  while(1)
  {
    double object_pose[object_pose_buffer_size_] = {};
    recv_size = recv(sockfd_, object_pose, object_pose_buffer_size_, 0);

    if (recv_size == 0)
    {
      break;
    }

    std::vector<double> object_pose_vec;
    object_pose_vec.clear();
    for (int i; i < object_pose_buffer_size_; i++)
    {
      object_pose_vec.push_back(object_pose[i]);
    }

    work_order_.push(object_pose_vec);
  }

  return NULL;
}

// Function for moveing robot
bool CommunicateMaster::moveRobot()
{
  pthread_t tid;
  pthread_create(&tid, NULL, (THREADFUNCPTR) &CommunicateMaster::waitReceiveThread, NULL);
  pthread_join(tid, NULL);

  // Ready data contena for receiving from master
  // double object_pose[object_pose_buffer_size_] = {};

  std::cout << "\n======================================="
            << "\n    Receive object pose from master    "
            << "\n======================================="
            << std::endl;

  // Recieve object pose from master
  // recv(sockfd_, object_pose, object_pose_buffer_size_, 0);

  // std::cout << "\n[SUCCESS] Complete to receive object pose !! " << std::endl;

  if (work_order_.empty())
  {
    std::cout << "\n[INFO] Not work order from master !!" << std::endl;
    return true;
  }

  std::vector<double> object_pose;
  object_pose.clear();
  object_pose = work_order_.front();
  work_order_.pop();

  std::cout << "\n[SUCCESS] Complete to receive object pose !! " << std::endl;

  std::cout << "\n====================================="
            << "\n      Activate parallel gripper      "
            << "\n====================================="
            << std::endl;

  // Activate parallel gripper
  denso_gripper_srvs::ActivateGripper activate_srv;
  if (!gripper_activate_service_.call(activate_srv))
  {
    ROS_ERROR("Failed to activate parallel gripper !!");
    return false;
  }

  std::cout << "\n[SUCCESS] Complete to activate parallel gripper !! " << std::endl;

  std::cout << "\n========================================="
            << "\n   Robot move to grasp object position   "
            << "\n========================================="
            << std::endl;

  // Define robot position for grasping object
  geometry_msgs::PoseStamped grasp_position;
  grasp_position.pose.position.x = object_pose[0];
  grasp_position.pose.position.y = object_pose[1];
  grasp_position.pose.position.z = object_pose[2]; // initialize
  grasp_position.pose.orientation.x = 1;
  grasp_position.pose.orientation.y = 0;
  grasp_position.pose.orientation.z = 0;
  grasp_position.pose.orientation.w = 0;

  // Moving robot to the position for grasping object
  denso_state_srvs::Moving move_srv;
  grasp_position.pose.position.z = object_pose[2] + grasp_offset_ + WORK_OFFSET;
  move_srv.request.target_pose = grasp_position;
  if (!moving_service_.call(move_srv))
  {
    ROS_ERROR("Failed to move robot !!");
    return false;
  }

  // Moving robot to the position for grasping object
  grasp_position.pose.position.z = object_pose[2] + grasp_offset_;
  move_srv.request.target_pose = grasp_position;
  if (!moving_service_.call(move_srv))
  {
    ROS_ERROR("Failed to move robot !!");
    return false;
  }

  std::cout << "\n[SUCCESS] Complete to move to grasp object position !! " << std::endl;

  std::cout << "\n=============================================="
            << "\n  Close parallel gripper for grasping object  "
            << "\n=============================================="
            << std::endl;

  // Close parallel gripper for grasping object
  denso_gripper_srvs::Move gripper_move_srv;
  if (!gripper_close_service_.call(gripper_move_srv))
  {
    ROS_ERROR("Failed to close gripper !!");
    return false;
  }

  std::cout << "\n[SUCCESS] Complete to grasp object !! " << std::endl;

  std::cout << "\n===================================="
            << "\n   Robot move to transport object   "
            << "\n===================================="
            << std::endl;

  // Define robot position for transporting object
  geometry_msgs::PoseStamped transport_position;
  transport_position.pose.position.x = 0; // TODO transport position x
  transport_position.pose.position.y = 0; // TODO transport position y
  transport_position.pose.position.z = 0; // TODO transport position z
  transport_position.pose.orientation.x = 1;
  transport_position.pose.orientation.y = 0;
  transport_position.pose.orientation.z = 0;
  transport_position.pose.orientation.w = 0;

  // Moving robot to the position for transporting object
  transport_position.pose.position.z = WORK_OFFSET; // TODO add transport position z
  move_srv.request.target_pose = transport_position;
  if (!moving_service_.call(move_srv))
  {
    ROS_ERROR("Failed to move robot !!");
    return false;
  }

  transport_position.pose.position.z = 0; // TODO add transport position z
  move_srv.request.target_pose = transport_position;
  if (!moving_service_.call(move_srv))
  {
    ROS_ERROR("Failed to move robot !!");
    return false;
  }

  std::cout << "\n[SUCCESS] Complete to transport object !! " << std::endl;

  std::cout << "\n=============================================="
            << "\n   Open parallel gripper for release object   "
            << "\n=============================================="
            << std::endl;

  // Open parallel gripper for releasing object
  if (!gripper_open_service_.call(gripper_move_srv))
  {
    ROS_ERROR("Failed to open gripper !!");
    return false;
  }

  std::cout << "\n[SUCCESS] Complete to release object !! " << std::endl;

  // Define robot default position
  geometry_msgs::PoseStamped default_position;
  default_position.pose.position.x = -0.034074;
  default_position.pose.position.y = -0.070623;
  default_position.pose.position.z = 0.34166;
  default_position.pose.orientation.x = 0.46538;
  default_position.pose.orientation.y = 0.0057988;
  default_position.pose.orientation.z = 0.81983;
  default_position.pose.orientation.w =0.33357;

  std::cout << "\n===================================="
            << "\n   Robot move to initial position   "
            << "\n===================================="
            << std::endl;

  // Moving robot to the default position
  move_srv.request.target_pose = default_position;
  if (!moving_service_.call(move_srv))
  {
    ROS_ERROR("Failed to move robot !!");
    return false;
  }

  std::cout << "\n[SUCCESS] Complete to move to initial position !! " << std::endl;

  std::cout << "\n===================================="
            << "\n   Send finish message to master    "
            << "\n===================================="
            << std::endl;

  // Send success message to master
  // if (send(sockfd_, "success", message_buffer_size_, 0) < 0)
  // {
  //   std::cerr << "\n[FAILURE] Failed to send finish message to master !! " << std::endl;
  //   return false;
  // }

  std::cout << "\n*************************************"
            << "\n**********  Success work  ***********"
            << "\n*************************************"
            << std::endl;

  return true;
}