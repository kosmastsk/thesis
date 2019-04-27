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

  // Get configurations
  _nh.param<double>("/rfid/range", _rfid_range, 1);
  _nh.param<double>("/rfid/hfov", _rfid_hfov, 60);
  _nh.param<double>("/rfid/vfov", _rfid_vfov, 30);

  // Adjust values
  _rfid_hfov = (_rfid_hfov / 180.0) * M_PI;
  _rfid_vfov = (_rfid_vfov / 180.0) * M_PI;

  // Reset some variables that will be filled later
  _covered = new octomap::ColorOcTree(_octomap_resolution);
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

  _octomap_loaded = 1;
}

void OnlineCoverage::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  if (!_octomap_loaded)
    return;

  calculateCoverage(msg->pose);

  publishCoveredSurface();
}

void OnlineCoverage::calculateCoverage(const geometry_msgs::Pose pose)
{
  octomap::point3d wall_point;

  octomath::Vector3 position(pose.position.x, pose.position.y, pose.position.z);
  octomath::Quaternion orientation(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);

  double yaw = orientation.toEuler().yaw();
  //
  Horizontal FOV degrees for (double horizontal = yaw - _rfid_hfov / 2; horizontal <= yaw + _rfid_hfov / 2;
                              horizontal += DEGREE)
  {
    // Vertical FOV degrees
    for (double vertical = -_rfid_vfov / 2; vertical <= _rfid_vfov / 2; vertical += DEGREE)
    {
      // direction at which we are facing the point
      octomap::point3d direction(1, 0, 0);

      if (_octomap->castRay(position, direction.rotate_IP(0, vertical, horizontal), wall_point, true, _rfid_range))
      {
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
  msg.header.frame_id = "/map";
  msg.binary = false;
  msg.id = _covered->getTreeType();
  msg.resolution = _octomap_resolution;
  if (octomap_msgs::fullMapToMsg(*_covered, msg))
    _covered_pub.publish(msg);
}

}  // namespace drone_coverage
