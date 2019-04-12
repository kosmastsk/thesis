#include "drone_coverage/coverage.h"

namespace drone_coverage
{
Coverage::Coverage()
{
  ros::WallTime startTime = ros::WallTime::now();

  ROS_INFO("Wall Finder object created\n");
  _octomap_loaded = 0;
  _map_sub = _nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &Coverage::octomapCallback, this);

  _covered_pub = _nh.advertise<octomap_msgs::Octomap>("/covered_surface", 1);
  _vis_pub = _nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);

  // Get initial positions from the Parameter Server
  _nh.param<double>("/x_pos", _init_pose[0], 0);
  _nh.param<double>("/y_pos", _init_pose[1], 0);
  _nh.param<double>("/z_pos", _init_pose[2], 0);

  // Get configurations
  _nh.param<double>("/rfid/range", _rfid_range, 1);
  _nh.param<double>("/rfid/hfov", _rfid_hfov, 60);
  _nh.param<double>("/rfid/vfov", _rfid_vfov, 30);
  _nh.param<double>("/uav/footprint_radius", _uav_radius, 0.4);
  _nh.param<double>("/uav/safety_offset", _uav_safety_offset, 0.3);

  // Adjust values
  _uav_safety_offset += _uav_radius;
  _rfid_hfov = (_rfid_hfov / 180.0) * M_PI;
  _rfid_vfov = (_rfid_vfov / 180.0) * M_PI;

  _nh.param<double>("/world/min_obstacle_height", _min_obstacle_height, 0.3);

  while (!_octomap_loaded)
  {
    ROS_INFO_ONCE("Waiting to load octomap, cannot procceed.......\n");
    ros::spinOnce();
    ros::Rate(10).sleep();
  }

  // Working offline
  // We can start in the beginning of the bounds plus the safety distance of the drone
  _init_pose[0] = _min_bounds[0] + _uav_safety_offset;
  _init_pose[1] = _min_bounds[1] + _uav_safety_offset;
  _init_pose[2] = _min_bounds[2] + _min_obstacle_height + _rfid_range * tan(_rfid_vfov / 2);

  // Reset some variables that will be filled later
  _walls = new octomap::OcTree(_octomap_resolution);

  // The initial position of the sensor/drone is not ON the bounds, but we start with an offset that allows the sensor
  // to ray around
  _sensor_position.x() = _init_pose[0];
  _sensor_position.y() = _init_pose[1];
  _sensor_position.z() = _init_pose[2];

  // Locate the walls in the octomap
  Coverage::calculateWaypointsAndCoverage();

  // Publish the points as an Octomap
  Coverage::publishCoveredSurface();

  // Publish sensor positions / waypoints
  Coverage::publishWaypoints();

  double dt = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO_STREAM("Coverage Finder took " << dt << " seconds.");
}

Coverage::~Coverage()
{
  if (_octomap != NULL)
    delete _octomap;
  if (_walls != NULL)
    delete _walls;
}

void Coverage::octomapCallback(const octomap_msgs::OctomapConstPtr& msg)
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
    }
  }
  else
  {
    ROS_WARN("Could not deserialize message to OcTree");
    return;
  }

  _octomap->getMetricMin(_min_bounds[0], _min_bounds[1], _min_bounds[2]);
  _octomap->getMetricMax(_max_bounds[0], _max_bounds[1], _max_bounds[2]);
  _octomap_resolution = _octomap->getResolution();

  // We don't want points that are under the ground --> bound < 0 -->convert them to 0
  _min_bounds[2] = (_min_bounds[2] < 0) ? 0 : _min_bounds[2];

  _octomap_loaded = 1;

  ROS_INFO("Octomap bounds are (x,y,z) : \n [min]  %f, %f, %f\n [max]  %f, %f, %f", _min_bounds[0], _min_bounds[1],
           _min_bounds[2], _max_bounds[0], _max_bounds[1], _max_bounds[2]);
}

void Coverage::calculateWaypointsAndCoverage()
{
  octomap::point3d wall_point;
  bool ray_success, point_inserted;
  double best_yaw = 0;

  // For every best valid point inside the world, raycast
  // The step between each position is the half coverage of the sensor
  // Except from z, that is calculated from the vfov
  // NOTE we are starting from the initial allowed position and reaching the max allowed bounds in the end.
  // If someone wants to start scanning from a different position, more checks need to be done
  while (_sensor_position.z() <= _max_bounds[2] - _uav_safety_offset)
  {
    while (_sensor_position.x() <= _max_bounds[0] - _uav_safety_offset)
    {
      while (_sensor_position.y() <= _max_bounds[1] - _uav_safety_offset)
      {
        bool yaw_found = findBestYaw(_sensor_position, best_yaw);

        // If that position offers no coverage, continue to the next one
        // Check if this position is safe in the octomap
        // https://github.com/OctoMap/octomap/issues/42
        if (!yaw_found || !safeCheck(_sensor_position))
        {
          // Next point in y
          _sensor_position.y() += 0.5 * _rfid_range;
          continue;
        }

        // Save the sensor pose that gives that best view
        // X, Y, Z, Roll, Pitch, Yaw
        octomath::Pose6D pose(_sensor_position.x(), _sensor_position.y(), _sensor_position.z(), 0, 0, best_yaw);
        _points.push_back(pose);

        // For the best view, specific yaw, calculate the covered surface by the sensor and add it to the octomap
        // Horizontal FOV degrees
        for (double horizontal = best_yaw - _rfid_hfov / 2; horizontal <= best_yaw + _rfid_hfov / 2;
             horizontal += DEGREE)
        {
          // Vertical FOV degrees
          for (double vertical = -_rfid_vfov / 2; vertical <= _rfid_vfov / 2; vertical += DEGREE)
          {
            // direction at which we are facing the point
            octomap::point3d direction(1, 0, 0);

            // Get every point on the direction vector that belongs to the FOV
            ray_success = _octomap->castRay(_sensor_position, direction.rotate_IP(0, vertical, horizontal), wall_point,
                                            true, _rfid_range);

            // Ground elimination
            if (wall_point.z() < _min_obstacle_height)
              continue;

            if (ray_success)
            {
              point_inserted = _walls->insertRay(_sensor_position, wall_point, _rfid_range);
            }
          }  // vertical loop

        }  // horizontal loop

        // Next point in y
        _sensor_position.y() += 0.5 * _rfid_range;
      }

      // Next point in x
      _sensor_position.x() += 0.5 * _rfid_range;

      // Reinitialize z position
      _sensor_position.y() = _init_pose[1];
    }

    // Next point in z
    // Using triangle geometry, we increase the height of the drone, so that the vfov allows us to cover every point on
    // the wall at least twice
    _sensor_position.z() += _rfid_range * tan(_rfid_vfov / 2);
    ROS_INFO("Setting z to %f\n", _sensor_position.z());

    // Reinitialize x and y position
    _sensor_position.x() = _init_pose[0];
    _sensor_position.y() = _init_pose[1];
  }
}

void Coverage::publishCoveredSurface()
{
  ROS_INFO("Publishing covered surface. Use RViz to visualize it..\n");
  _walls->toMaxLikelihood();
  _walls->prune();
  octomap_msgs::Octomap msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/map";
  msg.binary = true;
  msg.id = _walls->getTreeType();
  ROS_DEBUG("Tree class type: %s", msg.id.c_str());
  msg.resolution = _octomap_resolution;
  if (octomap_msgs::binaryMapToMsg(*_walls, msg))
    _covered_pub.publish(msg);
}

void Coverage::publishWaypoints()
{
  ROS_INFO("Publishing waypoints..\n");
  // Publish path as markers

  for (std::size_t idx = 0; idx < _points.size(); idx++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "coverage_path_planning";
    marker.id = idx;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = _points.at(idx).x();
    marker.pose.position.y = _points.at(idx).y();
    marker.pose.position.z = _points.at(idx).z();
    marker.pose.orientation.x = _points.at(idx).rot().x();
    marker.pose.orientation.y = _points.at(idx).rot().y();
    marker.pose.orientation.z = _points.at(idx).rot().z();
    marker.pose.orientation.w = _points.at(idx).rot().u();
    marker.scale.x = 0.4;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    _vis_pub.publish(marker);
    ros::Duration(0.001).sleep();
  }

  ROS_INFO("Finished!\n");
}

bool Coverage::safeCheck(octomap::point3d sensor_position)
{
  // TODO
  bool safe = 1;

  return safe;
}

bool Coverage::findBestYaw(octomap::point3d sensor_position, double& best_yaw)
{
  octomap::point3d wall_point;

  double best_coverage = 0;

  for (double yaw = -M_PI; yaw <= M_PI; yaw += M_PI / 4)
  {
    octomap::point3d direction(1, 0, 0);

    // Find the normal vector on the wall
    // Get every point on the direction vector
    bool ray_success =
        _octomap->castRay(sensor_position, direction.rotate_IP(0, 0, yaw), wall_point, true, _rfid_range);

    if (ray_success)
    {
      double coverage = findCoverage(wall_point, direction);
      ROS_DEBUG("coverage VS best_coverage : %f -- %f\n", coverage, best_coverage);
      if (coverage > best_coverage)
      {
        best_coverage = coverage;
        best_yaw = yaw;
      }
    }
  }

  // If none of the points provides better coverage than 0, then skip
  if (best_coverage == 0)
    return 0;
  else
    return 1;
}

double Coverage::findCoverage(const octomap::point3d& wall_point, const octomap::point3d& direction)
{
  double coverage;
  std::vector<octomap::point3d> normals;
  octomap::point3d normal = direction;

  if (_octomap->getNormals(wall_point, normals, false))
  {
    ROS_DEBUG("MC algorithm gives %d normals in voxel at (%f, %f, %f)\n", normals.size(), wall_point.x(),
              wall_point.y(), wall_point.z());
    // There is a chance of having zero normal vectors. This usually happens in a surface.
    // To deal with it, we suppose that the nearby nodes are on the same surface, and we try to find the normal vector
    // of a neighbor node
    // Starting with a small bounding box and expanding, as we are not able to find any normal vectors
    while (normals.size() == 0)
    {
      octomap::point3d offset_to_check(1, 1, 1);
      for (octomap::OcTree::leaf_bbx_iterator
               it = _octomap->begin_leafs_bbx(wall_point - offset_to_check, wall_point + offset_to_check),
               end = _octomap->end_leafs_bbx();
           it != end; ++it)
      {
        octomap::point3d new_wall_point = it.getCoordinate();
        _octomap->getNormals(new_wall_point, normals, false);
        if (normals.size() == 0)
          offset_to_check *= 2;
        else
          break;
      }
    }

    // Find the mean
    for (unsigned i = 0; i < normals.size(); ++i)
    {
      normal += normals[i];
    }
    // Normalize it
    normal /= normals.size();
    octomap::point3d unit_vector = normal.normalized();
    coverage = unit_vector.dot(direction.normalized());
  }
  else
  {
    coverage = 0;
    ROS_INFO("query point unknown (no normals)\n");
  }

  // Return the absolute value of the coverage metric
  return fabs(coverage);
}

}  // namespace drone_coverage
