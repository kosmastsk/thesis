#include "drone_coverage/coverage.h"

namespace drone_coverage
{
Coverage::Coverage()
{
  ros::WallTime startTime = ros::WallTime::now();

  ROS_DEBUG("Coverage object created\n");
  _octomap_loaded = 0;
  _ogm_loaded = 0;

  _ogm = new nav_msgs::OccupancyGrid();

  _map_sub = _nh.subscribe<octomap_msgs::Octomap>("/octomap_binary", 1, &Coverage::octomapCallback, this);
  _ogm_sub = _nh.subscribe<nav_msgs::OccupancyGrid>("/projected_map", 1, &Coverage::ogmCallback, this);

  _covered_pub = _nh.advertise<octomap_msgs::Octomap>("/covered_surface", 1);
  _vis_pub = _nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1000);
  _waypoints_pub_slice = _nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/coverage/waypoints/slice", 1000);
  _waypoints_pub_lift = _nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>("/coverage/waypoints/lift", 1000);

  // Get initial positions from the Parameter Server
  _nh.param<double>("/x_pos", _init_pose[0], 0);
  _nh.param<double>("/y_pos", _init_pose[1], 0);
  _nh.param<double>("/z_pos", _init_pose[2], 0);

  // Get configurations
  _nh.param<double>("/sensor/rfid/range", _rfid_range, 1);
  _nh.param<double>("/sensor/rfid/hfov", _rfid_hfov, 60);
  _nh.param<double>("/sensor/rfid/vfov", _rfid_vfov, 30);
  _nh.param<double>("/sensor/rfid/direction/x", _rfid_direction_x, 1);
  _nh.param<double>("/sensor/rfid/direction/y", _rfid_direction_y, 0);
  _nh.param<double>("/sensor/rfid/direction/z", _rfid_direction_z, 0);
  _nh.param<double>("/uav/footprint_radius", _uav_radius, 0.4);
  _nh.param<double>("/uav/safety_offset", _uav_safety_offset, 0.3);

  //  _nh.param<double>("/coverage/subsampling_step", _subsampling_step, 1);

  // Adjust values
  _uav_safety_offset += _uav_radius;
  // Convert to rads
  _rfid_hfov = (_rfid_hfov / 180.0) * M_PI;
  _rfid_vfov = (_rfid_vfov / 180.0) * M_PI;

  _nh.param<double>("/world/min_obstacle_height", _min_obstacle_height, 0.3);
  _nh.param<double>("/coverage/step", _sampling_step, 0.5);

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

  postprocessWaypoints();

  // Find the covered surface of the waypoints left after post process
  std::string sensor_shape;
  _nh.param<std::string>("/sensor/rfid/shape", sensor_shape, "orthogonal");
  if (sensor_shape == "orthogonal")
  {
    calculateOrthogonalCoverage();
  }
  else
  {
    calculateCircularCoverage();
  }

  // Find the covered surface of the waypoints left after post process
  ROS_INFO("%f%% of the surface will be covered\n", evaluateCoverage(_octomap, _covered));

  // Publish the points as an Octomap
  publishCoveredSurface();

  ROS_INFO("Waiting to publish the ordered waypoints....\n");

  // Create a graph with all points
  _graph = generateGraph(_nh, _xy_points);

  // Apply a hill-climbing algorithm to find the best combination of the waypoints
  _xy_points = calculateOptimalPath(_nh, _graph, _xy_points, _octomap);

  // Go back from Point_xy to Pose6D elements
  std::string method;
  _nh.param<std::string>("/coverage/method", method, "lift");

  _slice_points = revertTo6D(_xy_points, _xyzrpy_points, "slice");
  _lift_points = revertTo6D(_xy_points, _xyzrpy_points, "lift");

  _slice_points = postprocessPath(_nh, _slice_points);
  _lift_points = postprocessPath(_nh, _lift_points);

  // Publish sensor positions / waypoints
  publishWaypoints(_slice_points, "slice");
  publishWaypoints(_lift_points, "lift");
  visualizeWaypoints(_lift_points);

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

  _octomap_loaded = 1;

  ROS_INFO("Octomap bounds are (x,y,z) : \n [min]  %f, %f, %f\n [max]  %f, %f, %f\n", _min_bounds[0], _min_bounds[1],
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
    while (_sensor_position.y() <= _max_bounds[1] - _uav_safety_offset)
    {
      while (_sensor_position.x() <= _max_bounds[0] - _uav_safety_offset)
      {
        // /*
        // * Necessary checks for obstacles and points that should not be waypoints
        // * Each one, checks the same with the different way, to exclude as many points as possible that should be
        // *excluded* /

        // Check if is occupied node in octomap
        octomap::OcTreeNode* node = _octomap->search(_sensor_position);
        if (node != NULL && _octomap->isNodeOccupied(node))
        {
          // Next point in x
          _sensor_position.x() = proceedOneStep(_sensor_position.x());
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
          // Next point in x
          _sensor_position.x() = proceedOneStep(_sensor_position.x());
          continue;
        }

        // Check if this position is safe in the octomap using the OGM(projected octomap)
        // https://github.com/OctoMap/octomap/issues/42
        if (!safeCheckFrom2D(_sensor_position))
        {
          // Next point in x
          _sensor_position.x() = proceedOneStep(_sensor_position.x());
          continue;
        }

        /*
        * ******** END OF CHECKS ********
        */

        // If that position offers no coverage, continue to the next one
        // Otherwise, give the yaw with the best view
        if (!findBestYaw(_sensor_position, best_yaw))
        {
          // Next point in x
          _sensor_position.x() = proceedOneStep(_sensor_position.x());
          continue;
        }

        // Save the sensor pose that gives that best view
        // X, Y, Z, Roll, Pitch, Yaw
        octomath::Pose6D pose(_sensor_position.x(), _sensor_position.y(), _sensor_position.z(), 0, 0, best_yaw);
        _points.push_back(pose);

        // Next point in x
        _sensor_position.x() = proceedOneStep(_sensor_position.x());
      }
      // Next point in y
      _sensor_position.y() = proceedOneStep(_sensor_position.y());

      // Reinitialize x position
      _sensor_position.x() = _init_pose[0];
    }

    // Next point in z Using triangle geometry, we increase the height of the drone, so that the vfov allows us to cover
    // every point on the wall at least twice
    _sensor_position.z() += _rfid_range * tan(_rfid_vfov / 2);
    ROS_INFO("Setting z to %f\n", _sensor_position.z());

    // Reinitialize x and y position
    _sensor_position.x() = _init_pose[0];
    _sensor_position.y() = _init_pose[1];
  }
}

void Coverage::postprocessWaypoints()
{
  // Post-process the waypoints to remove noise and outliers
  ROS_INFO("Waypoints post-processing...\n");

  // Remove the waypoints that are inside obstacles and cannot be remove with octomap functions
  removeNonVisibleWaypoints();

  // Create the vector of x,y points that must be provided in the graph to calculate the optimal path
  reduceDimensionality();
}

void Coverage::removeNonVisibleWaypoints()
{
  ROS_INFO("Removing non-visible waypoints....\n");
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

void Coverage::reduceDimensionality()
{
  ROS_INFO("Reducing dimensionality...\n");
  for (int i = 0; i < _points.size(); i++)
  {
    Point_xy xy;
    xy = std::make_pair(_points.at(i).x(), _points.at(i).y());
    std::vector<Point_xy>::iterator it = std::find(_xy_points.begin(), _xy_points.end(), xy);

    if (it == _xy_points.end())
    {  // does not exist
      _xy_points.push_back(xy);
      _xyzrpy_points.resize(_xy_points.size());
    }
    it = std::find(_xy_points.begin(), _xy_points.end(), xy);
    int index = std::distance(_xy_points.begin(), it);
    _xyzrpy_points.at(index).push_back(_points.at(i));
  }

  ROS_INFO("%zu points will be used to generate the path...\n", _xy_points.size());
}

void Coverage::calculateOrthogonalCoverage()
{
  ROS_INFO("Calculating orthogonal coverage...\n");
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
        octomap::point3d direction(_rfid_direction_x, _rfid_direction_y, _rfid_direction_z);

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

void Coverage::calculateCircularCoverage()
{
  ROS_INFO("Calculating circular coverage...\n");
  octomap::point3d wall_point;
  // For each one of the points, calculate coverage
  for (int i = 0; i < _points.size(); i++)
  {
    double yaw = _points.at(i).yaw();
    // For the best view, specific yaw, calculate the covered surface by the sensor and add it to the octomap
    // Horizontal FOV degrees
    // Adapt the for limits
    for (double horizontal = yaw - _rfid_hfov / 2; horizontal <= yaw + _rfid_hfov / 2; horizontal += DEGREE)
    {
      // Vertical FOV degrees
      for (double vertical = -_rfid_vfov / 2; vertical <= _rfid_vfov / 2; vertical += DEGREE)
      {
        // direction at which we are facing the point
        octomap::point3d direction(_rfid_direction_x, _rfid_direction_y, _rfid_direction_z);

        // Get every point on the direction vector that belongs to the FOV
        bool ray_success = _octomap->castRay(_points.at(i).trans(), direction.rotate_IP(0, vertical, horizontal),
                                             wall_point, true, _rfid_range);

        // Make the coverage circular, cut the points that are larger than the range==radius
        if (_points.at(i).trans().distance(wall_point) > _rfid_range)
          continue;

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
  octomap->toMaxLikelihood();
  octomap->prune();
  covered->toMaxLikelihood();
  covered->prune();

  float octomap_volume = calculateOccupiedVolume(octomap);
  ROS_INFO("octomap volume %f [m^3]\n", octomap_volume);
  float covered_volume = calculateOccupiedVolume(covered);
  ROS_INFO("covered volume %f [m^3]\n", covered_volume);

  float percentage = 100 * (covered_volume / octomap_volume);

  return percentage;
}

float Coverage::calculateOccupiedVolume(octomap::OcTree* octomap)
{
  float vol_occ = 0;
  double bbxMinX, bbxMinY, bbxMinZ, bbxMaxX, bbxMaxY, bbxMaxZ;
  octomap->getMetricMax(bbxMaxX, bbxMaxY, bbxMaxZ);
  octomap->getMetricMin(bbxMinX, bbxMinY, bbxMinZ);

  octomap::point3d min(bbxMinX, bbxMinY, bbxMinZ);
  octomap::point3d max(bbxMaxX, bbxMaxY, bbxMaxZ);

  if (octomap)
  {  // can be NULL
    for (octomap::OcTree::leaf_bbx_iterator it = octomap->begin_leafs_bbx(min, max), end = octomap->end_leafs_bbx();
         it != end; ++it)
    {
      double side_length = it.getSize();
      if (octomap->isNodeOccupied(*it))
        // occupied leaf node
        vol_occ += side_length * side_length * side_length;
    }
  }
  return vol_occ;
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
        if (checkIfVisible(_points.at(root).trans(), _points.at(i).trans(), _octomap))
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
    // direction at which we are facing the point
    octomap::point3d direction(_rfid_direction_x, _rfid_direction_y, _rfid_direction_z);

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

std::vector<octomath::Pose6D> Coverage::revertTo6D(std::vector<Point_xy> xy_points,
                                                   std::vector<std::vector<octomath::Pose6D>> _xyzrpy_points,
                                                   std::string method)
{
  ROS_INFO("Converting back to 6D....\n");

  std::vector<octomath::Pose6D> output;

  if (method == "lift")
  {
    bool is_low = 1;
    for (int i = 0; i < xy_points.size(); i++)
    {
      // Find the index where the i-th (x,y) point is in the _xyzrpy_points
      Point_xy xy = _xy_points.at(i);
      int index;
      for (int k = 0; k < _xy_points.size(); k++)
      {
        // Getting the (x,y) from the 0 place is enough, the rest are the same
        Point_xy tmp = std::make_pair(_xyzrpy_points.at(k).at(0).x(), _xyzrpy_points.at(k).at(0).y());
        if (xy == tmp)
        {
          index = k;
          break;
        }
      }

      int j;
      if (is_low)
      {
        for (j = 0; j < _xyzrpy_points.at(index).size(); j++)
        {
          output.push_back(_xyzrpy_points.at(index).at(j));
        }
        is_low = 0;
      }
      else
      {
        for (j = _xyzrpy_points.at(index).size() - 1; j >= 0; j--)
        {
          output.push_back(_xyzrpy_points.at(index).at(j));
        }
        is_low = 1;
      }
    }
  }
  else if (method == "slice")
  {
    while (output.size() != _points.size())
    {
      for (int i = 0; i < xy_points.size(); i++)
      {
        // Find the index where the i-th (x,y) point is in the _xyzrpy_points
        Point_xy xy = _xy_points.at(i);
        int index;
        for (int k = 0; k < _xy_points.size(); k++)
        {
          // Getting the (x,y) from the 0 place is enough, the rest are the same
          if (_xyzrpy_points.at(k).size() != 0)
          {
            Point_xy tmp = std::make_pair(_xyzrpy_points.at(k).front().x(), _xyzrpy_points.at(k).front().y());
            if (xy == tmp)
            {
              index = k;
              break;
            }
          }
        }
        int j = _xyzrpy_points.at(index).size();
        if (j > 0)
        {
          output.push_back(_xyzrpy_points.at(index).at(0));
          _xyzrpy_points.at(index).erase(_xyzrpy_points.at(index).begin());
        }
      }
    }
  }
  return output;
}

std::vector<octomath::Pose6D> Coverage::postprocessPath(ros::NodeHandle nh, std::vector<octomath::Pose6D> points)
{
  ROS_INFO("Post-processing path...\n");
  std::vector<octomath::Pose6D> output;

  double step_xy = _sampling_step;
  double step_z = ceil(_rfid_range * tan(_rfid_vfov / 2));

  bool eliminate = false;
  for (int i = 0; i < points.size(); i++)
  {
    double x = points.at(i).x();
    double y = points.at(i).y();
    double z = points.at(i).z();
    if (eliminate)
    {
      if (i == 0)
        ;  // Just add the first point
      else if (i == points.size() - 1)
        ;  // Just add the last point
      else
      {
        double x_previous = points.at(i - 1).x();
        double y_previous = points.at(i - 1).y();
        double z_previous = points.at(i - 1).z();
        double x_next = points.at(i + 1).x();
        double y_next = points.at(i + 1).y();
        double z_next = points.at(i + 1).z();

        if ((fabs(x - x_previous) <= step_xy && fabs(x - x_next) <= step_xy && y == y_previous && y == y_next &&
             z == z_previous && z == z_next) ||
            (fabs(y - y_previous) <= step_xy && fabs(y - y_next) <= step_xy && x == x_previous && x == x_next &&
             z == z_previous && z == z_next) ||
            (fabs(z - z_previous) <= step_z && fabs(z - z_next) <= step_z && x == x_previous && x == x_next &&
             y == y_previous && y == y_next))
        {
          eliminate = false;
          continue;
        }
      }
    }
    else
      eliminate = true;
    output.push_back(points.at(i));
  }
  ROS_INFO("new size %zu\n", output.size());
  return output;
}

double Coverage::proceedOneStep(double coord)
{
  return coord + _sampling_step;
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
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.text = std::to_string(idx);
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = points.at(idx).x();
    marker.pose.position.y = points.at(idx).y();
    marker.pose.position.z = points.at(idx).z();
    marker.pose.orientation.x = points.at(idx).rot().x();
    marker.pose.orientation.y = points.at(idx).rot().y();
    marker.pose.orientation.z = points.at(idx).rot().z();
    marker.pose.orientation.w = points.at(idx).rot().u();
    marker.scale.z = 0.4;
    marker.color.a = 1.0;
    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 1;
    _vis_pub.publish(marker);
    ros::Duration(0.001).sleep();
  }

  ROS_INFO("Finished!\n");
}

void Coverage::publishWaypoints(std::vector<octomath::Pose6D> points, std::string method)
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
  if (method == "lift")
    _waypoints_pub_lift.publish(msg);
  else
    _waypoints_pub_slice.publish(msg);
}

}  // namespace drone_coverage
