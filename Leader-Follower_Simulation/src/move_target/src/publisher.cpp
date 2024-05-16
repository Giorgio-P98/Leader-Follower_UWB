
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <vector>
#include <random>

using namespace std;
using namespace std::chrono_literals;

void setPoint(trajectory_msgs::msg::JointTrajectory *trajectoire, double val_x, double val_y, double rot_z)
{
  trajectoire->points[0].positions[0] = val_x;
  trajectoire->points[0].positions[1] = val_y;
  trajectoire->points[0].positions[2] = rot_z;
}

vector<vector<double>> gen_traj(double time_length, double period, double vel)
{
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<double> dist(0,1 / 3.6);
  int n = int(time_length / period);
  int nse = int(n / 22);
  vector<vector<double>> v(n, vector<double>{0, 0, 0});

  for (int i = 1; i < nse; i++)
  {
    v[i][0] = v[i - 1][0] + vel * period; //+ dist(gen) * period;
    v[i][1] = v[i - 1][1]; //+ dist(gen) * period;
    v[i][2] = 0;
  }
  for (int i = nse; i < 3 * int(nse/2); i++)
  {
    v[i][0] = v[i - 1][0]; //+ dist(gen) * period;
    v[i][1] = v[i - 1][1] + vel * period; //+ dist(gen) * period;
    v[i][2] = M_PI_2;
  }
  for (int i = 3 * int(nse/2); i < 5 * int(nse/2); i++)
  {
    v[i][0] = v[i - 1][0] - vel * period; //+ dist(gen) * period;
    v[i][1] = v[i - 1][1]; //+ dist(gen) * period;
    v[i][2] = M_PI;
  }
  for (int i = 5 * int(nse/2); i < 6 * int(nse/2); i++)
  {
    v[i][0] = v[i - 1][0]; //+ dist(gen) * period;
    v[i][1] = v[i - 1][1] - vel * period; //+ dist(gen) * period;
    v[i][2] = M_PI + M_PI_2;
  }


  // the prism is moving back and forth for testing the drone response to this moves
  // for (int i = 1; i < int(n / 4); i++)
  // {
  //   v[i][0] = v[i - 1][0] + vel * period + dist(gen) * period;
  //   v[i][1] = v[i - 1][1] + dist(gen) * period;
  //   v[i][2] = 0;
  // }
  // for (int i = int(n / 4); i < int(n / 2); i++)
  // {
  //   v[i][0] = v[i - 1][0] - vel * period + dist(gen) * period;
  //   v[i][1] = v[i - 1][1] + dist(gen) * period;
  //   v[i][2] = M_PI;
  // }
  // for (int i = int(n / 2); i < int(3 * (n / 4)); i++)
  // {
  //   v[i][0] = v[i - 1][0] + dist(gen) * period;
  //   v[i][1] = v[i - 1][1] + vel * period + dist(gen) * period;
  //   v[i][2] = M_PI_2;
  // }
  // for (int i = int(3 * (n / 4)); i < n; i++)
  // {
  //   v[i][0] = v[i - 1][0] + dist(gen) * period;
  //   v[i][1] = v[i - 1][1] - vel * period + dist(gen) * period;
  //   v[i][2] = M_PI + M_PI_2;
  // }


  return v;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  double period = 0.01;
  vector<vector<double>> trj = gen_traj(200, period, 1);

  auto node = rclcpp::Node::make_shared("traj_publisher_node");
  auto publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "set_joint_trajectory", 10);

  trajectory_msgs::msg::JointTrajectory message;
  message.header.frame_id = "world";
  message.joint_names.resize(3);
  message.joint_names[0] = "movingCubex";
  message.joint_names[1] = "movingCubey";
  message.joint_names[2] = "rotationz";

  message.points.resize(1);
  message.points[0].positions.resize(3);
  message.points[0].positions[0] = 0;
  message.points[0].positions[1] = 0;
  message.points[0].positions[2] = 0;

  rclcpp::Rate loop_rate(10ms);
  int i = 0;

  while (rclcpp::ok() && i < 2000)
  {
    publisher->publish(message);
    rclcpp::spin_some(node);
    loop_rate.sleep();
    i++;
  }
  i = 0;

  while (rclcpp::ok())
  {
    publisher->publish(message);
    setPoint(&message, trj[i][0], trj[i][1], trj[i][2]);

    rclcpp::spin_some(node);
    loop_rate.sleep();
    i++;
  }

  rclcpp::shutdown();
  return 0;
}
