#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

using namespace ur_rtde;
using namespace std::chrono;

void test_speedJ(RTDEControlInterface *rtde_control,
                 RTDEReceiveInterface *rtde_receive, std::vector<double> joint_q,
                 double dt, double acceleration);

int main(int argc, char *argv[]) {
  // Parameters
  double acceleration = 1;
  double dt = 0.004;
  RTDEControlInterface rtde_control("127.0.0.1");
  RTDEReceiveInterface rtde_receive("127.0.0.1");
  std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) 
      test_speedJ(&rtde_control, &rtde_receive, joint_q, dt*(i+1), acceleration*(i+1));

  rtde_control.stopScript();
  return 0;
}

void test_speedJ(RTDEControlInterface *rtde_control,
                 RTDEReceiveInterface *rtde_receive, std::vector<double> joint_q,
                 double dt, double acceleration) {
  std::vector<double> joint_speed = {0.0, 0.0, 0.001, 0.0, 0.0, 0.0};
  double k = 0.001;
  std::vector<double> timestamp;
  std::vector<double> target_velocity;
  std::vector<double> command_velocity;
  std::vector<double> actual_velocity;

  rtde_control->moveJ(joint_q);

  auto t_init = high_resolution_clock::now();

  for (unsigned int i = 0; i < 25; i++) {
    auto t_start = high_resolution_clock::now();
    rtde_control->speedJ(joint_speed, acceleration, dt);

    timestamp.push_back(
        std::chrono::duration<double>(t_start - t_init).count());
    command_velocity.push_back(joint_speed[2]);
    target_velocity.push_back(rtde_receive->getTargetQd()[2]);
    actual_velocity.push_back(rtde_receive->getActualQd()[2]);

    joint_speed[2] += (i + 1) * k;

    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt) {
      std::this_thread::sleep_for(
          std::chrono::duration<double>(dt - t_duration.count()));
    }
  }

  for (uint i = 0; i < 25; i++) {
    auto t_start = high_resolution_clock::now();
    rtde_control->speedJ(joint_speed, acceleration, dt);

    timestamp.push_back(
        std::chrono::duration<double>(t_start - t_init).count());
    command_velocity.push_back(joint_speed[2]);
    target_velocity.push_back(rtde_receive->getTargetQd()[2]);
    actual_velocity.push_back(rtde_receive->getActualQd()[2]);

    joint_speed[2] -= (i + 1) * k;

    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt) {
      std::this_thread::sleep_for(
          std::chrono::duration<double>(dt - t_duration.count()));
    }
  }

  char filename[40];
  std::sprintf(filename, "out_dt=%.4lf_a=%.2lf.csv", dt, acceleration);
  auto fd = std::fopen(filename, "w");
  std::fprintf(
      fd, "i, timestamp, command_velocity, target_velocity, actual_velocity\n");
  for (int i = 0; i < 50; i++) {
    std::fprintf(fd, "%2d, %lf, %lf, %lf, %lf\n ", i, timestamp[i],
                 command_velocity[i], target_velocity[i], actual_velocity[i]);
  }
  std::fclose(fd);
  rtde_control->speedStop();
}
