#include <cobotta_client/communicate_master.h>

using communicate_master::CommunicateMaster;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "communicate_master_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  CommunicateMaster cm(nh);

  if (!cm.connectMaster())
  {
    std::cerr << "\n[FAILURE] Failed to connect master \n" << std::endl;
    return -1;
  }

  ros::Rate rate(10);
  while (ros::ok())
  {
    if (!cm.moveRobot())
    {
      std::cerr << "\n[ERROR] Robot has error while working !! \n" << std::endl;
      return -1;
    }
    rate.sleep();
  }

  std::cout << "[FINISH] Finish process communicate_master_node !! " << std::endl;
  return 0;
}