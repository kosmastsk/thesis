#include "drone_coverage/coverage.h"

namespace drone_coverage
{
Coverage::Coverage()
{
  ros::WallTime startTime = ros::WallTime::now();

  ROS_INFO("Coverage object created\n");
  _octomap_loaded = 0;
  _ogm_loaded = 0;

  _ogm = new nav_msgs::OccupancyGrid();

  _map_sub = _nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &Coverage::octomapCallback, this);
  _ogm_sub = _nh.subscribe<nav_msgs::OccupancyGrid>("/projected_map", 1, &Coverage::ogmCallback, this);

  _covered_pub = _nh.advertise<octomap_msgs::Octomap>("/covered_surface", 1);
  _vis_pub = _nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);
  _waypoints_pub = _nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/waypoints_smooth", 1000);

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
    ROS_INFO_ONCE("Waiting to load octomap, cannot proceed.......\n");
    ros::spinOnce();
    ros::Rate(10).sleep();
  }

  while (!_ogm_loaded)
  {
    ROS_INFO_ONCE("Waiting to load 2D map, cannot proceed.......\n");
    ros::spinOnce();
    ros::Rate(10).sleep();
  }

  // Working offline
  // We can start in the beginning of the bounds plus the safety distance of the drone
  _init_pose[0] = _min_bounds[0] + _uav_safety_offset;
  _init_pose[1] = _min_bounds[1] + _uav_safety_offset;
  _init_pose[2] = _min_bounds[2] + _min_obstacle_height + _rfid_range * tan(_rfid_vfov / 2);

  // Reset some variables that will be filled later
  _covered = new octomap::OcTree(_octomap_resolution);

  // The initial position of the sensor/drone is not ON the bounds, but we start with an offset that allows the sensor
  // to ray around
  _sensor_position.x() = _init_pose[0];
  _sensor_position.y() = _init_pose[1];
  _sensor_position.z() = _init_pose[2];

  // Find the best points for the drone to be, that ensure max coverage
  calculateWaypoints();

  // Find the covered surface of the waypoints left after post process
  calculateCoverage();

  // Find the covered surface of the waypoints left after post process
  ROS_INFO("%f%% of the surface will be covered\n", evaluateCoverage(_octomap, _covered));

  // Publish the points as an Octomap
  publishCoveredSurface();

  ROS_INFO("Waiting to publish the ordered waypoints....\n");

  // Create a graph with all points
  _graph = generateGraph(_nh, _points);

  // Apply a hill-climbing algorithm to find the best combination of the waypoints
  _points = hillClimbingBase(_nh, _graph, _points);

  // Publish sensor positions / waypoints
  publishWaypoints(_points);
  visualizeWaypoints(_points);

  double dt = (ros::WallTime::now() - startTime).toSec();
  ROS_INFO_STREAM("Coverage took " << dt << " seconds.");
}

Coverage::~Coverage()
{
  if (_octomap != NULL)
    delete _octomap;
  if (_covered != NULL)
    delete _covered;
  if (_ogm != NULL)
    delete _ogm;
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
      _octomap->expand();  // bbx work currently only with expanded tree
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
  // _min_bounds[2] = (_min_bounds[2] < 0) ? 0 : _min_bounds[2];

  _octomap_loaded = 1;

  ROS_INFO("Octomap bounds are (x,y,z) : \n [min]  %f, %f, %f\n [max]  %f, %f, %f", _min_bounds[0], _min_bounds[1],
           _min_bounds[2], _max_bounds[0], _max_bounds[1], _max_bounds[2]);
}

void Coverage::ogmCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
  _ogm->data = msg->data;
  _ogm->info = msg->info;
  _ogm->header = msg->header;
  _ogm_loaded = true;
  ROS_INFO("OGM loaded...\n");
}

void Coverage::calculateWaypoints()
{
  ROS_INFO("Calculating waypoints...\n");
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
        /*
        * Necessary checks for obstacles and points that should not be waypoints
        * Each one, checks the same with the different way, to exclude as many points as possible that should be
        * excluded
        */

        // Check if is occupied node in octomap
        octomap::OcTreeNode* node = _octomap->search(_sensor_position);
        if (node != NULL && _octomap->isNodeOccupied(node))
        {
          // Next point in y
          _sensor_position.y() = proceedOneStep(_sensor_position.y());
          continue;
        }

        // Check for obstacles around
        bool obstacle_found = 0;
        for (double safe_check = -M_PI; safe_check <= M_PI; safe_check += M_PI / 8)
        {
          octomap::point3d direction(1, 1, 0);  // combination of x and y
          if (_octomap->castRay(_sensor_position, direction.rotate_IP(0, 0, safe_check), wall_point, true, 2))
          {
            if (_sensor_position.distance(wall_point) < _uav_safety_offset)
            {
              obstacle_found = 1;
              break;
            }
          }
        }

        if (obstacle_found)
        {
          // Next point in y
          _sensor_position.y() = proceedOneStep(_sensor_position.y());
          continue;
        }

        // Check if this position is safe in the octomap using the OGM(projected octomap)
        // https://github.com/OctoMap/octomap/issues/42
        if (!safeCheckFrom2D(_sensor_position))
        {
          // Next point in y
          _sensor_position.y() = proceedOneStep(_sensor_position.y());
          continue;
        }

        /*
        * ******** END OF CHECKS ********
        */

        // If that position offers no coverage, continue to the next one
        // Otherwise, give the yaw with the best view
        if (!findBestYaw(_sensor_position, best_yaw))
        {
          // Next point in y
          _sensor_position.y() = proceedOneStep(_sensor_position.y());
          continue;
        }

        // Save the sensor pose that gives that best view
        // X, Y, Z, Roll, Pitch, Yaw
        octomath::Pose6D pose(_sensor_position.x(), _sensor_position.y(), _sensor_position.z(), 0, 0, best_yaw);
        _points.push_back(pose);

        // Next point in y
        _sensor_position.y() = proceedOneStep(_sensor_position.y());
      }

      // Next point in x
      _sensor_position.x() = proceedOneStep(_sensor_position.x());

      // Reinitialize z position
      _sensor_position.y() = _init_pose[1];
    }

    // Next point in z
    // Using triangle geometry, we increase the height of the drone, so that the vfov allows us to cover every point on
    // the wall at least twice
    _sensor_position.z() += _rfid_range * tan(_rfid_vfov / 2);
    // ROS_INFO("Setting z to %f\n", _sensor_position.z());

    // Reinitialize x and y position
    _sensor_position.x() = _init_pose[0];
    _sensor_position.y() = _init_pose[1];
  }

  // Post-process the waypoints to remove noise and outliers
  ROS_INFO("Waypoints post-processing...\n");

  // Start a recursive method to find neighbors of all points
  std::vector<octomath::Pose6D> final_points;

  // Initialize _discovered_nodes vector
  _discovered_nodes.resize(_points.size());
  std::fill(_discovered_nodes.begin(), _discovered_nodes.end(), false);

  int undiscovered_nodes = _points.size();

  for (int root = 0; root < _points.size(); root++)
  {
    _discovered_nodes.at(root) = 1;
    findNeighbors(root);

    // How many nodes have been undiscovered
    undiscovered_nodes = std::count(_discovered_nodes.begin(), _discovered_nodes.end(), 0);

    // We are satisfied with the number of nodes discoveres, so continue with the rest of the coverage utilities

    // If more than 90% of the nodes are undiscovered, restart the process
    if (undiscovered_nodes > 0.90 * _points.size())
      std::fill(_discovered_nodes.begin(), _discovered_nodes.end(), false);  // Revert the discovered nodes
    else
      break;
  }

  ROS_INFO("%d nodes have been undiscovered\n", undiscovered_nodes);

  // Copy the points that are visible to the final vector
  for (int i = 0; i < _points.size(); i++)
  {
    if (_discovered_nodes.at(i) == 1)
    {
      final_points.push_back(_points.at(i));
    }
  }

  // Make sure the size of the final points is positive, Otherwise something is wrong
  ROS_ASSERT(final_points.size() > 0);

  ROS_INFO("Number of points before : %zu\n", _points.size());
  ROS_INFO("Number of points after : %zu\n", final_points.size());

  // Save final points to the Class variable
  _points.clear();
  _points = final_points;
}

void Coverage::calculateCoverage()
{
  ROS_INFO("Calculating coverage...\n");
  octomap::point3d wall_point;
  // For each one of the points, calculate coverage
  for (int i = 0; i < _points.size(); i++)
  {
    double yaw = _points.at(i).yaw();
    // For the best view, specific yaw, calculate the covered surface by the sensor and add it to the octomap
    // Horizontal FOV degrees
    for (double horizontal = yaw - _rfid_hfov / 2; horizontal <= yaw + _rfid_hfov / 2; horizontal += DEGREE)
    {
      // Vertical FOV degrees
      for (double vertical = -_rfid_vfov / 2; vertical <= _rfid_vfov / 2; vertical += DEGREE)
      {
        // direction at which we are facing the point
        octomap::point3d direction(1, 0, 0);

        // Get every point on the direction vector that belongs to the FOV
        bool ray_success = _octomap->castRay(_points.at(i).trans(), direction.rotate_IP(0, vertical, horizontal),
                                             wall_point, true, _rfid_range);

        // Ground elimination
        if (wall_point.z() < _min_obstacle_height)
          continue;

        if (ray_success)
        {
          _covered->insertRay(_points.at(i).trans(), wall_point, _rfid_range);
        }
      }  // vertical loop

    }  // horizontal loop
  }
}

float Coverage::evaluateCoverage(octomap::OcTree* octomap, octomap::OcTree* covered)
{
  covered->toMaxLikelihood();
  covered->prune();

  // Use floats to make the division work later on
  float octomap_leafs = float(octomap->getNumLeafNodes());
  float covered_leafs = float(covered->getNumLeafNodes());
  ROS_INFO("[Number of leafs] octomap %f : covered %f\n", octomap_leafs, covered_leafs);

  float percentage = 100 * (covered_leafs / octomap_leafs);

  return percentage;
}

void Coverage::findNeighbors(int root)
{
  ROS_DEBUG("calling findNeighbors with root %d\n", root);
  // For every node
  for (int i = 0; i < _points.size(); i++)
  {
    if (i == root)
      continue;

    // If node is undiscovered
    if (_discovered_nodes.at(i) == 0)
    {
      // If distance is fine
      // Check distance and visibility from root
      double distance = _points.at(root).distance(_points.at(i));
      if (distance < 0.75 * _rfid_range)
      {
        // Check visibility
        if (getVisibility(_points.at(root).trans(), _points.at(i).trans()))
        {
          _discovered_nodes.at(i) = 1;
          // Call the funcion recursively
          findNeighbors(i);
        }
      }
    }
  }
}

bool Coverage::safeCheckFrom2D(octomap::point3d sensor_position)
{
  // Transform each point in the map's coordinates, using the resolution and the _ogm_origin
  int cell_x = (sensor_position.x() - _ogm->info.origin.position.x) / _ogm->info.resolution;
  int cell_y = (sensor_position.y() - _ogm->info.origin.position.y) / _ogm->info.resolution;
  // ROS_INFO("position x, position y %f %f\n", sensor_position.x(), sensor_position.y());
  // ROS_INFO("cell x, cell y : %d %d\n", cell_x, cell_y);

  // Check using the 2D map, that the position of the drone is safe
  if (_ogm->data[cell_x + _ogm->info.width * cell_y] == 100)
    return 0;  // not safe

  return 1;
}

bool Coverage::findBestYaw(octomap::point3d sensor_position, double& best_yaw)
{
  octomap::point3d wall_point;

  double best_coverage = 0;

  for (double yaw = -M_PI; yaw <= M_PI; yaw += _rfid_hfov / 2)
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
    ROS_DEBUG("MC algorithm gives %zu normals in voxel at (%f, %f, %f)\n", normals.size(), wall_point.x(),
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

bool Coverage::getVisibility(const octomap::point3d view_point, const octomap::point3d point_to_test)
{
  // Get all nodes in a line
  octomap::KeyRay key_ray;

  _octomap->computeRayKeys(view_point, point_to_test, key_ray);

  const octomap::OcTreeKey& point_to_test_key = _octomap->coordToKey(point_to_test);

  // Now check if there are any unknown or occupied nodes in the ray,
  // except for the point_to_test key.
  for (octomap::OcTreeKey key : key_ray)
  {
    if (key != point_to_test_key)
    {
      octomap::OcTreeNode* node = _octomap->search(key);

      if (node != NULL && _octomap->isNodeOccupied(node))
        return false;
    }
  }
  return true;
}

double Coverage::proceedOneStep(double coord)
{
  return coord + 0.5 * _rfid_range;
}

void Coverage::publishCoveredSurface()
{
  ROS_INFO("Publishing covered surface. Use RViz to visualize it..\n");
  _covered->toMaxLikelihood();
  _covered->prune();
  octomap_msgs::Octomap msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/map";
  msg.binary = true;
  msg.id = _covered->getTreeType();
  ROS_DEBUG("Tree class type: %s", msg.id.c_str());
  msg.resolution = _octomap_resolution;
  if (octomap_msgs::binaryMapToMsg(*_covered, msg))
    _covered_pub.publish(msg);
}

void Coverage::visualizeWaypoints(std::vector<octomath::Pose6D> points)
{
  ROS_INFO("Visualizing waypoints..\n");
  // Publish path as markers

  for (std::size_t idx = 0; idx < points.size(); idx++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "coverage_path_planning";
    marker.id = idx;
    // marker.type = visualization_msgs::Marker::ARROW;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = std::to_string(idx);
    // marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    // marker.mesh_resource = "package://drone_description/meshes/quadrotor/quadrotor_base.dae";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = points.at(idx).x();
    marker.pose.position.y = points.at(idx).y();
    marker.pose.position.z = points.at(idx).z();
    marker.pose.orientation.x = points.at(idx).rot().x();
    marker.pose.orientation.y = points.at(idx).rot().y();
    marker.pose.orientation.z = points.at(idx).rot().z();
    marker.pose.orientation.w = points.at(idx).rot().u();
    // marker.scale.x = 1;  // 0.3;
    // marker.scale.y = 1;  // 0.1;
    marker.scale.z = 0.4;  // 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    _vis_pub.publish(marker);
    ros::Duration(0.001).sleep();
  }

  ROS_INFO("Finished!\n");
}

void Coverage::publishWaypoints(std::vector<octomath::Pose6D> points)
{
  ROS_INFO("Publishing waypoints..\n");
  trajectory_msgs::MultiDOFJointTrajectory msg;
  // Header
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  // joint names
  // Blank

  // points
  msg.points.resize(points.size());
  for (int i = 0; i < points.size(); i++)
  {
    msg.points[i].transforms.resize(1);
    // Translation
    msg.points[i].transforms[0].translation.x = points.at(i).x();
    msg.points[i].transforms[0].translation.y = points.at(i).y();
    msg.points[i].transforms[0].translation.z = points.at(i).z();

    // Orientation
    msg.points[i].transforms[0].rotation.x = points.at(i).rot().x();
    msg.points[i].transforms[0].rotation.y = points.at(i).rot().y();
    msg.points[i].transforms[0].rotation.z = points.at(i).rot().z();
    msg.points[i].transforms[0].rotation.w = points.at(i).rot().u();
  }

  _waypoints_pub.publish(msg);
}

}  // namespace drone_coverage
