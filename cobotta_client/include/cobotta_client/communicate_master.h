#ifndef COMMUNICATE_MASTER_H
#define COMMUNICATE_MASTER_H

#include <ros/ros.h>

#include <string>
#include <vector>
#include <queue>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

typedef void * (*THREADFUNCPTR)(void *);

namespace communicate_master
{
class CommunicateMaster
{
public:
  CommunicateMaster(ros::NodeHandle& nh);
  ~CommunicateMaster();
  bool connectMaster();
  bool moveRobot();
private:
  void* waitReceiveThread(void* p_param);
private:
  ros::NodeHandle nh_;
  ros::ServiceClient moving_service_;
  ros::ServiceClient gripper_activate_service_;
  ros::ServiceClient gripper_open_service_;
  ros::ServiceClient gripper_close_service_;
  int port_number_;
  int sockfd_;
  int object_pose_buffer_size_;
  int message_buffer_size_;
  std::string master_ip_address_;
  std::queue<std::vector<double> > work_order_;
  struct sockaddr_in addr_;
  double grasp_offset_;
};
} // namespace communicate_master

#endif // COMMUNICATE_MASTER_H