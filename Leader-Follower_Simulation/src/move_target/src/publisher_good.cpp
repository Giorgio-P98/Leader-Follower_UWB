
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include <vector>
#include <random>
#include <math.h>

using namespace std;
using namespace std::chrono_literals;

int TRAJECTORY_WANTED = 0;                // if 0 -> sin-like traj, if 1 -> square traj
double VELOCITY = 5.0;                    // velocity in km/h

void setPoint(trajectory_msgs::msg::JointTrajectory *trajectoire, double val_x, double val_y)
{
  trajectoire->points[0].positions[0] = val_x;
  trajectoire->points[0].positions[1] = val_y;
}

vector<vector<double>> circle(double vel, double dt, double radius, double x0, double y0, double portion, int verse)
{
  vel = vel / 3.6;
  double omega = vel / radius;
  double period = 2 * M_PI / omega;
  double vx,vy;
  vector<vector<double>> vect;
  vect.push_back(vector<double>{radius, 0});
  if (verse == 0)
  {
    vx = 0;
    vy = vel;
  }
  else
  {
    vx = 0;
    vy = -vel;
  }

  int i = 1;
  while (dt * i <= period * portion)
  {
    vect.push_back(vector<double>{vect[i - 1][0] + vx * dt, vect[i - 1][1] + vy * dt});
    double angle = atan2(vect[i][1], vect[i][0]);
    if (verse == 0)
    {
      vx = vel * (-sin(angle));
      vy = vel * (cos(angle));
    }
    else
    {
      vx = vel * (sin(angle));
      vy = -vel * (cos(angle));
    }
    i = i + 1;
  }
  for (int i = 0; i < int(vect.size()); i++)
  {
    vect[i][0] =  vect[i][0] -radius + x0 ;
    vect[i][1] =  vect[i][1] + y0; 
  }
  
  return vect;
}

vector<vector<double>> line(double vel, double dt, double length, double x0, double y0, double direction)
{
  vel = vel / 3.6;
  double vx = vel * cos(direction);
  double vy = vel * sin(direction);
  vector<vector<double>> vect;
  vect.push_back(vector<double>{x0, y0});
  int i = 1;
  do
  {
    vect.push_back(vector<double>{vect[i - 1][0] + vx * dt, vect[i - 1][1] + vy * dt});
    i = i + 1;
  } while (sqrt(pow(vect[i - 1][0] - vect[0][0], 2) + pow(vect[i - 1][1] - vect[0][1], 2)) < length);
  return vect;
}

vector<vector<double>> sine_like(double vel, double dt, double amplitude, double x0, double y0)
{
  vector<vector<double>> vect;
  vect = circle(vel,dt,amplitude,x0,y0,0.5,0);
  vector<vector<double>> vect2;
  vect2 = circle(vel,dt,amplitude,vect.back()[0],vect.back()[1],0.5,1);
  vect.insert(vect.end(), vect2.begin(), vect2.end());
  vect2 = circle(vel,dt,amplitude,vect.back()[0],vect.back()[1],0.5,0);
  vect.insert(vect.end(), vect2.begin(), vect2.end());
  vect2 = circle(vel,dt,amplitude,vect.back()[0],vect.back()[1],0.5,1);
  vect.insert(vect.end(), vect2.begin(), vect2.end());

  return vect;
}

vector<vector<double>> gen_traj(double time_length, double period, double vel)
{
  vel = vel / 3.6;
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<double> dist(0, 0);
  int n = int(time_length / period);
  vector<vector<double>> v(n, vector<double>{0, 0});
  for (int i = 1; i < int(n / 5); i++)
  {
    v[i][0] = v[i - 1][0] + vel * period + dist(gen) * period;
    v[i][1] = v[i - 1][1] + dist(gen) * period;
  }
  for (int i = int(n / 5); i < int(n / 4); i++)
  {
    v[i][0] = v[i - 1][0];
    v[i][1] = v[i - 1][1];
  }
  for (int i = int(n / 4); i < int(n / 2); i++)
  {
    v[i][0] = v[i - 1][0] - vel * period + dist(gen) * period;
    v[i][1] = v[i - 1][1] + dist(gen) * period;
  }
  for (int i = int(n / 2); i < int(3 * (n / 4)); i++)
  {
    v[i][0] = v[i - 1][0] + dist(gen) * period;
    v[i][1] = v[i - 1][1] + vel * period + dist(gen) * period;
  }
  for (int i = int(3 * (n / 4)); i < n; i++)
  {
    v[i][0] = v[i - 1][0] + dist(gen) * period;
    v[i][1] = v[i - 1][1] - vel * period + dist(gen) * period;
  }

  return v;
}

vector<vector<double>> gen_traj_square(double time_length, double period, double vel)
{
  vel = vel / 3.6;
  std::random_device rd{};
  std::mt19937 gen{rd()};
  std::normal_distribution<double> dist(0, 0.2);
  std::normal_distribution<double> distv(0, 0.2);
  int n = int(time_length / period);
  vector<vector<double>> v(n, vector<double>{0, 0});
  for (int i = 1; i < int(n / 4); i++)
  {
    v[i][0] = v[i - 1][0] + (distv(gen) + vel) * period + dist(gen) * period;
    v[i][1] = v[i - 1][1] + dist(gen) * period;
  }
  for (int i = int(n / 4); i < int(n / 2); i++)
  {
    v[i][0] = v[i - 1][0] + dist(gen) * period;
    v[i][1] = v[i - 1][1] + (distv(gen) + vel) * period + dist(gen) * period;
  }
  for (int i = int(n / 2); i < int(3 * (n / 4)); i++)
  {
    v[i][0] = v[i - 1][0] - (distv(gen) + vel) * period + dist(gen) * period;
    v[i][1] = v[i - 1][1] + dist(gen) * period;
  }
  for (int i = int(3 * (n / 4)); i < n; i++)
  {
    v[i][0] = v[i - 1][0] + dist(gen) * period;
    v[i][1] = v[i - 1][1] - (distv(gen) + vel) * period + dist(gen) * period;
  }

  return v;
}

int main(int argc, char *argv[])
{

  rclcpp::init(argc, argv);

  double period = 0.02;

  // // Velocity in km/h
  // // vector<vector<double>> trj = gen_traj(60, period, 5.5);
  // vector<vector<double>> trj;
  // trj.push_back(vector<double> {0,0});
  // vector<vector<double>> trj1 = line(1.0, period, 30.0, 0.0, 0.0, 0.0);
  // trj.insert(trj.end(), trj1.begin(), trj1.end());
  // // vector<vector<double>> trj2 = circle(5.0, period, 10.0,trj.back()[0], trj.back()[1], 0.33,0);
  // // trj.insert(trj.end(), trj2.begin(), trj2.end());
  // // vector<vector<double>> trj3 = line(5.0, period, 20.0, trj.back()[0], trj.back()[1], M_PI);
  // // trj.insert(trj.end(), trj3.begin(), trj3.end());
  // // vector<vector<double>> trj4 =circle(5.0, period, 7.0,trj.back()[0], trj.back()[1], 0.6,1);
  // // trj.insert(trj.end(), trj4.begin(), trj4.end());
  // vector<vector<double>> trjs = sine_like(1.0, period, 5, trj.back()[0], trj.back()[1]);
  // trj.insert(trj.end(), trjs.begin(), trjs.end());

  vector<vector<double>> trj;
  if (TRAJECTORY_WANTED == 0)
  {
    trj.push_back(vector<double> {0,0});
    vector<vector<double>> trj1 = line(VELOCITY, period, 30.0, 0.0, 0.0, 0.0);
    trj.insert(trj.end(), trj1.begin(), trj1.end());
    vector<vector<double>> trjs = sine_like(VELOCITY, period, 5, trj.back()[0], trj.back()[1]);
    trj.insert(trj.end(), trjs.begin(), trjs.end());
  }
  else if (TRAJECTORY_WANTED == 1)
  {
    vector<vector<double>> trj =  gen_traj_square(100, period, VELOCITY);
  }
  else 
  {
    printf("Error in the trajectory selection, perhaps you have wrongly define TRAJECTORY_WANTED");
    return 1;
  }

  auto node = rclcpp::Node::make_shared("traj_publisher_node");
  auto publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "set_joint_trajectory", 10);

  trajectory_msgs::msg::JointTrajectory message;
  message.header.frame_id = "world";
  message.joint_names.resize(2);
  message.joint_names[0] = "movingCubex";
  message.joint_names[1] = "movingCubey";

  message.points.resize(1);
  message.points[0].positions.resize(2);
  message.points[0].positions[0] = 0;
  message.points[0].positions[1] = 0;

  rclcpp::Rate loop_rate(20ms);
  int i = 0;
  while (rclcpp::ok() && i < 1400)
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
    setPoint(&message, trj[i][0], trj[i][1]);

    rclcpp::spin_some(node);
    loop_rate.sleep();
    i++;
  }

  rclcpp::shutdown();
  return 0;
}
