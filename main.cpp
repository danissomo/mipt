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
                 RTDEReceiveInterface *rtde_receive,
                 std::vector<double> joint_q, double dt, double acceleration,
                 double k, int j_move_count);
std::vector<double> calc_speed(RTDEReceiveInterface *rtde_receive, double dt,
                               std::vector<double> qdd);

int main(int argc, char *argv[]) {
  // Parameters
  double acceleration = 1;
  double dt = 0.004;
  RTDEControlInterface rtde_control("127.0.0.1");
  RTDEReceiveInterface rtde_receive("127.0.0.1");
  std::vector<double> joint_q = {-1.54, -1.83, -2.28, -0.59, 1.60, 0.023};

  // for (int i = 0; i < 3; i++)
  //   for (int j = 1; j < 3; j++)
  //     test_speedJ(&rtde_control, &rtde_receive, joint_q, dt * (i + 1),
  //                 acceleration * (j + 1), 0.001, 1);
  // for (int i = 0; i < 3; i++)
  //   for (int j = 1; j < 3; j++)
  //     test_speedJ(&rtde_control, &rtde_receive, joint_q, dt * (i + 1),
  //                 acceleration * (j + 1), 0.001, 2);

  // for (int i = 0; i < 3; i++)
  //   for (int j = 1; j < 3; j++)
  //     test_speedJ(&rtde_control, &rtde_receive, joint_q, dt * (i + 1),
  //                 acceleration * (j + 1), 0.002, 1);
   test_speedJ(&rtde_control, &rtde_receive, joint_q, dt * 2,
                   acceleration, 0.1, 1);

  rtde_control.stopScript();
  return 0;
}

void test_speedJ(RTDEControlInterface *rtde_control,
                 RTDEReceiveInterface *rtde_receive,
                 std::vector<double> joint_q, double dt, double acceleration,
                 double k, int j_move_count) {
  std::vector<double> joint_speed = {0.0, 0.0, 0.001, 0.0, 0.0, 0.0};
  auto buf = j_move_count;
  for (int i = 1; i < buf; i++)
    if (i - 1 == 2)
      buf++;
    else
      joint_speed[i - 1] = 0.001;

  std::vector<double> timestamp;
  std::vector<double> target_velocity;
  std::vector<double> command_velocity;
  std::vector<double> actual_velocity;
  std::vector<double> actual_q;
  std::vector<double> theory_q;

  rtde_control->moveJ(joint_q);

  auto t_init = high_resolution_clock::now();

  for (unsigned int i = 0; i < 25; i++) {
    auto t_start = high_resolution_clock::now();
    joint_speed =  calc_speed(rtde_receive, dt, {0, 0 , (i + 1) * k, 0, 0, 0}) ;
    rtde_control->speedJ(joint_speed, (i + 1) * k);

    timestamp.push_back(
        std::chrono::duration<double>(t_start - t_init).count());
    command_velocity.push_back(joint_speed[2]);
    target_velocity.push_back(rtde_receive->getTargetQd()[2]);
    actual_velocity.push_back(rtde_receive->getActualQd()[2]);
    actual_q.push_back(rtde_receive->getTargetQ()[2]);
    theory_q.push_back(i == 0 ? joint_q[2]
                              : theory_q.back() + joint_speed[2] * dt);

    

    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt) {
      std::this_thread::sleep_for(
          std::chrono::duration<double>(dt - t_duration.count()));
    }
  }

  for (uint i = 0; i < 25; i++) {
    auto t_start = high_resolution_clock::now();
    joint_speed= calc_speed(rtde_receive, dt, {0.0, 0.0 , -double((i + 1) * k), 0.0, 0.0, 0.0});
    rtde_control->speedJ(joint_speed, (i + 1) * k );

    timestamp.push_back(
        std::chrono::duration<double>(t_start - t_init).count());
    command_velocity.push_back(joint_speed[2]);
    target_velocity.push_back(rtde_receive->getTargetQd()[2]);
    actual_velocity.push_back(rtde_receive->getActualQd()[2]);
    actual_q.push_back(rtde_receive->getTargetQ()[2]);
    theory_q.push_back(theory_q.back() + joint_speed[2] * dt);

    

    auto t_stop = high_resolution_clock::now();
    auto t_duration = std::chrono::duration<double>(t_stop - t_start);

    if (t_duration.count() < dt) {
      std::this_thread::sleep_for(
          std::chrono::duration<double>(dt - t_duration.count()));
    }
  }

  char filename[256];
  std::sprintf(filename, "dt=%.4lf_a=%.2lf_k=%.4lf_jmv=%d.csv", dt,
               acceleration, k, j_move_count);
  auto fd = std::fopen(filename, "w");
  std::fprintf(fd,
               "i, timestamp, command_velocity, target_velocity, "
               "actual_velocity, theory_q, actual_q, actual_qdd\n");

  for (int i = 0; i < actual_velocity.size(); i++) {
    auto qdd  =0.0 ;
    if(i != 0)
    qdd = (actual_velocity[i] -  actual_velocity[i - 1] ) / (timestamp[i] -  timestamp[i-1]);
    std::fprintf(fd, "%2d, %lf, %lf, %lf, %lf, %lf, %lf, %lf\n ", i,
                 timestamp[i], command_velocity[i], target_velocity[i],
                 actual_velocity[i], theory_q[i], actual_q[i], qdd);
  }
  std::fclose(fd);
  rtde_control->speedStop();
}
std::vector<double> calc_speed(RTDEReceiveInterface *rtde_receive, double dt,
                               std::vector<double> qdd) {
  auto res = rtde_receive->getActualQd();
  for (size_t i = 0; i < res.size(); i++) res[i] += qdd[i] * dt;
  return res;
}
