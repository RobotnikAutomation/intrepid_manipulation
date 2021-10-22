#include <intrepid_deep_grasp_demo/deep_grasp_demo.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "deep_grasp_demo");
  ros::NodeHandle n;

  DeepGraspDemo dgd(n);
  dgd.asyncStart();

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}
