#include "drone_coverage/online_coverage.h"

namespace drone_coverage
{
OnlineCoverage::OnlineCoverage()
{
  ROS_INFO("Coverage object created\n");
  _octomap_loaded = 0;

  _map_sub = _nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &OnlineCoverage::octomapCallback, this);
  _pose_sub = _nh.subscribe("/amcl_pose", 1000, &OnlineCoverage::poseCallback, this);

  _covered_pub = _nh.advertise<octomap_msgs::Octomap>("/octomap_covered", 1000);

  _nh.param<double>("/world/min_obstacle_height", _min_obstacle_height, 0.3);

  // Get configurations
  _nh.param<double>("/rfid/range", _rfid_range, 1);
  _nh.param<double>("/rfid/hfov", _rfid_hfov, 60);
  _nh.param<double>("/rfid/vfov", _rfid_vfov, 30);
  _nh.param<double>("/rfid/direction/x", _rfid_direction_x, 1);
  _nh.param<double>("/rfid/direction/y", _rfid_direction_y, 0);
  _nh.param<double>("/rfid/direction/z", _rfid_direction_z, 0);

  // Adjust values
  _rfid_hfov = (_rfid_hfov / 180.0) * M_PI;
  _rfid_vfov = (_rfid_vfov / 180.0) * M_PI;
}

OnlineCoverage::~OnlineCoverage()
{
  if (_octomap != NULL)
    delete _octomap;
  if (_covered != NULL)
    delete _covered;
}

void OnlineCoverage::octomapCallback(const octomap_msgs::OctomapConstPtr& msg)
{
  // Load octomap msg
  octomap::AbstractOcTree* abstract = octomap_msgs::msgToMap(*msg);
  if (abstract)
  {
    octomap::ColorOcTree* coloroctree = dynamic_cast<octomap::ColorOcTree*>(abstract);
    _octomap = reinterpret_cast<octomap::OcTree*>(coloroctree);

    if (_octomap == NULL)
    {
      ROS_WARN("Octomap message does not contain an OcTree\n");
      return;
    }
    else
    {
      ROS_INFO("Octomap successfully loaded\n");
      _octomap->expand();  // bbx work currently only with expanded tree
    }
  }
  else
  {
    ROS_WARN("Could not deserialize message to OcTree");
    return;
  }

  _octomap_resolution = _octomap->getResolution();

  // Now that we have the resolution we can initialize the new octomap
  _covered = new octomap::ColorOcTree(_octomap_resolution);

  _octomap_loaded = 1;
}

void OnlineCoverage::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (!_octomap_loaded)
    return;

  // Find the covered surface of the waypoints left after post process
  std::string sensor_shape;
  _nh.param<std::string>("/rfid/shape", sensor_shape, "orthogonal");
  if (sensor_shape == "orthogonal")
  {
    calculateOrthogonalCoverage(msg->pose);
  }
  else
  {
    calculateCircularCoverage(msg->pose);
  }

  publishCoveredSurface();
}

void OnlineCoverage::calculateOrthogonalCoverage(const geometry_msgs::Pose pose)
{
  octomap::point3d wall_point;

  octomath::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
  octomath::Quaternion orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

  double yaw = orientation.toEuler().yaw();
  // Horizontal FOV degrees
  for (double horizontal = yaw - _rfid_hfov / 2; horizontal <= yaw + _rfid_hfov / 2; horizontal += DEGREE)
  {
    // Vertical FOV degrees
    for (double vertical = -_rfid_vfov / 2; vertical <= _rfid_vfov / 2; vertical += DEGREE)
    {
      // direction at which we are facing the point
      octomap::point3d direction(_rfid_direction_x, _rfid_direction_y, _rfid_direction_z);

      if (_octomap->castRay(position, direction.rotate_IP(0, vertical, horizontal), wall_point, true, _rfid_range))
      {
        // Ground elimination
        if (wall_point.z() < _min_obstacle_height)
          continue;

        if (_covered->insertRay(position, wall_point, _rfid_range))
        {
          octomap::ColorOcTreeNode* node = _covered->search(wall_point);
          if (node != NULL)
            node->setColor(0.5, 0.5, 0.5);
        }
      }
    }  // vertical loop
  }    // horizontal loop
}

void OnlineCoverage::calculateCircularCoverage(const geometry_msgs::Pose pose)
{
  octomap::point3d wall_point;

  octomath::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
  octomath::Quaternion orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

  double yaw = orientation.toEuler().yaw();
  // Horizontal FOV degrees
  for (double horizontal = yaw - _rfid_hfov / 2; horizontal <= yaw + _rfid_hfov / 2; horizontal += DEGREE)
  {
    // Vertical FOV degrees
    for (double vertical = -_rfid_vfov / 2; vertical <= _rfid_vfov / 2; vertical += DEGREE)
    {
      // direction at which we are facing the point
      octomap::point3d direction(_rfid_direction_x, _rfid_direction_y, _rfid_direction_z);

      if (_octomap->castRay(position, direction.rotate_IP(0, vertical, horizontal), wall_point, true, _rfid_range))
      {
        // Ground elimination
        if (wall_point.z() < _min_obstacle_height)
          continue;

        // Make the coverage circular, cut the points that are larger than the range==radius
        if (position.distanceXY(wall_point) > _rfid_range)
          continue;

        if (_covered->insertRay(position, wall_point, _rfid_range))
        {
          octomap::ColorOcTreeNode* node = _covered->search(wall_point);
          if (node != NULL)
            node->setColor(1, 0, 0);
        }
      }
    }  // vertical loop
  }    // horizontal loop
}

void OnlineCoverage::publishCoveredSurface()
{
  _covered->toMaxLikelihood();
  _covered->prune();
  octomap_msgs::Octomap msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.binary = false;
  msg.id = _covered->getTreeType();
  msg.resolution = _covered->getResolution();
  if (octomap_msgs::fullMapToMsg(*_covered, msg))
    _covered_pub.publish(msg);
}

}  // namespace drone_coverage
