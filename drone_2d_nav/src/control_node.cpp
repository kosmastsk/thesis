/*
* Copyright (c) 2019 Kosmas Tsiakas
*
* GNU GENERAL PUBLIC LICENSE
*    Version 3, 29 June 2007
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "drone_2d_nav/control.h"

/* Main function */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "control");

  if (argc != 2)
  {
    ROS_INFO("\n[USAGE] rosrun drone_gazebo control_node <desired_height_value>\n");
    return 1;
  }
  else
  {
    control::Controller controller(argv);
    ros::spin();
  }

  return 0;
}
