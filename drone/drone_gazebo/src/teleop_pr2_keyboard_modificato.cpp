/*
 * teleop_pr2_keyboard
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Kevin Watts
// Taken from: https://github.com/AlessioTonioni/Autonomous-Flight-ROS

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_F 0x66

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45
#define KEYCODE_Z_CAP 0x5A
#define KEYCODE_X_CAP 0x58

class TeleopPR2Keyboard
{
private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run;
  geometry_msgs::Twist cmd;
  geometry_msgs::TwistStamped stamped_cmd;

  ros::NodeHandle n_;
  ros::Publisher vel_pub_;
  ros::Publisher stamped_vel_pub_;

public:
  void init()
  {
    cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

    vel_pub_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    stamped_vel_pub_ = n_.advertise<geometry_msgs::TwistStamped>("/cmd_vel/stamped", 1);

    ros::NodeHandle n_private("~");
    n_private.param("walk_vel", walk_vel, 1.0);
    n_private.param("run_vel", run_vel, 2.0);
    n_private.param("yaw_rate", yaw_rate, 1.0);
    n_private.param("yaw_run_rate", yaw_rate_run, 2.0);
  }

  ~TeleopPR2Keyboard()
  {
  }
  void keyboardLoop();
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_base_keyboard");

  TeleopPR2Keyboard tpk;
  tpk.init();

  signal(SIGINT, quit);

  tpk.keyboardLoop();

  return (0);
}

void TeleopPR2Keyboard::keyboardLoop()
{
  char c;
  bool dirty = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'wasd' to translate");
  puts("Use 'zx' for altitude control");
  puts("Use 'qe' to yaw");
  puts("Press 'Shift' to run");
  puts("Press 'f' to stop");

  for (;;)
  {
    // get the next event from the keyboard
    if (read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    cmd.linear.x = cmd.linear.y = cmd.angular.z = cmd.linear.z = 0;

    switch (c)
    {
      // Walking
      case KEYCODE_W:
        cmd.linear.x = walk_vel;
        dirty = true;
        break;
      case KEYCODE_S:
        cmd.linear.x = -walk_vel;
        dirty = true;
        break;
      case KEYCODE_A:
        cmd.linear.y = walk_vel;
        dirty = true;
        break;
      case KEYCODE_D:
        cmd.linear.y = -walk_vel;
        dirty = true;
        break;
      case KEYCODE_Q:
        cmd.angular.z = yaw_rate;
        dirty = true;
        break;
      case KEYCODE_E:
        cmd.angular.z = -yaw_rate;
        dirty = true;
        break;
      case KEYCODE_Z:
        cmd.linear.z = walk_vel;
        dirty = true;
        break;
      case KEYCODE_X:
        cmd.linear.z = -walk_vel;
        dirty = true;
        break;
      case KEYCODE_F:
        dirty = true;
        break;

      // Running
      case KEYCODE_W_CAP:
        cmd.linear.x = run_vel;
        dirty = true;
        break;
      case KEYCODE_S_CAP:
        cmd.linear.x = -run_vel;
        dirty = true;
        break;
      case KEYCODE_A_CAP:
        cmd.linear.y = run_vel;
        dirty = true;
        break;
      case KEYCODE_D_CAP:
        cmd.linear.y = -run_vel;
        dirty = true;
        break;
      case KEYCODE_Q_CAP:
        cmd.angular.z = yaw_rate_run;
        dirty = true;
        break;
      case KEYCODE_E_CAP:
        cmd.angular.z = -yaw_rate_run;
        dirty = true;
        break;
      case KEYCODE_Z_CAP:
        cmd.linear.z = run_vel;
        dirty = true;
        break;
      case KEYCODE_X_CAP:
        cmd.linear.z = -run_vel;
        dirty = true;
        break;
    }

    if (dirty == true)
    {
      stamped_cmd.twist = cmd;
      stamped_cmd.header.stamp = ros::Time::now();

      stamped_vel_pub_.publish(stamped_cmd);
      vel_pub_.publish(cmd);
    }
  }
}
