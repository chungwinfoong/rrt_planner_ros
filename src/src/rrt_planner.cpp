#include "rrt_planner/rrt_planner.h"
#include <ros/console.h>

namespace rrt_planner
{

  RRTPlanner::RRTPlanner(ros::NodeHandle *node)
      : nh_(node),
        private_nh_("~"),
        map_received_(false),
        init_pose_received_(false),
        goal_received_(false)
  {
    // Get map and path topics from parameter server
    std::string map_topic, path_topic;
    private_nh_.param<std::string>("map_topic", map_topic, "/map");
    private_nh_.param<std::string>("path_topic", path_topic, "/path");

    int goal_buffer = 50;
    int step_size = 50;
    int max_iterations = 1000;
    if (node->getParam("goal_buffer", goal_buffer))
    {
      ROS_INFO("Successfully retrieved goal_buffer: %d", goal_buffer);
    }
    else
    {
      ROS_ERROR("Failed to retrieve my_param. Using default value: %d", goal_buffer);
    }
    if (node->getParam("step_size", step_size))
    {
      ROS_INFO("Successfully retrieved step_size: %d", step_size);
    }
    else
    {
      ROS_ERROR("Failed to retrieve my_param. Using default value: %d", step_size);
    }
    if (node->getParam("max_iterations", max_iterations))
    {
      ROS_INFO("Successfully retrieved max_iterations: %d", max_iterations);
    }
    else
    {
      ROS_ERROR("Failed to retrieve my_param. Using default value: %d", max_iterations);
    }

    // Subscribe to map topic
    map_sub_ = nh_->subscribe<const nav_msgs::OccupancyGrid::Ptr &>(
        map_topic, 1, &RRTPlanner::mapCallback, this);

    // Subscribe to initial pose topic that is published by RViz
    init_pose_sub_ = nh_->subscribe<const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &>(
        "/initialpose", 1, &RRTPlanner::initPoseCallback, this);

    // Subscribe to goal topic that is published by RViz
    goal_sub_ = nh_->subscribe<const geometry_msgs::PoseStamped::ConstPtr &>(
        "/move_base_simple/goal", 1, &RRTPlanner::goalCallback, this);

    // Advertise topic where calculated path is going to be published
    path_pub_ = nh_->advertise<nav_msgs::Path>(path_topic, 1, true);

    // This loops until the node is running, will exit when the node is killed
    while (ros::ok())
    {
      // if map, initial pose, and goal have been received
      // build the map image, draw initial pose and goal, and plan
      if (map_received_ && init_pose_received_ && goal_received_)
      {
        buildMapImage();
        drawGoalInitPose();
        plan(goal_buffer, step_size, max_iterations);
      }
      else
      {
        if (map_received_)
        {
          displayMapImage();
        }
        ros::Duration(0.1).sleep();
        ros::spinOnce();
      }
    }
  }

  void RRTPlanner::mapCallback(const nav_msgs::OccupancyGrid::Ptr &msg)
  {
    map_grid_ = msg;

    // Build and display the map image
    buildMapImage();
    displayMapImage();

    // Reset these values for a new planning iteration
    map_received_ = true;
    init_pose_received_ = false;
    goal_received_ = false;

    ROS_INFO("Map obtained successfully. Please provide initial pose and goal through RViz.");
  }

  void RRTPlanner::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
  {
    if (init_pose_received_)
    {
      buildMapImage();
    }

    // Convert mas to Point2D
    poseToPoint(init_pose_, msg->pose.pose);

    // Reject the initial pose if the given point is occupied in the map
    if (!isPointUnoccupied(init_pose_))
    {
      init_pose_received_ = false;
      ROS_WARN(
          "The initial pose specified is on or too close to an obstacle please specify another point");
    }
    else
    {
      init_pose_received_ = true;
      drawGoalInitPose();
      ROS_INFO("Initial pose obtained successfully. x: %d y: %d", init_pose_.x(), init_pose_.y());
    }

    displayMapImage();
  }

  void RRTPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
  {
    if (goal_received_)
    {
      buildMapImage();
    }

    // Convert msg to Point2D
    poseToPoint(goal_, msg->pose);

    // Reject the goal pose if the given point is occupied in the map
    if (!isPointUnoccupied(goal_))
    {
      goal_received_ = false;
      ROS_WARN("The goal specified is on or too close to an obstacle please specify another point");
    }
    else
    {
      goal_received_ = true;
      drawGoalInitPose();
      ROS_INFO("Goal obtained successfully. x: %d y: %d", goal_.x(), goal_.y());
    }

    displayMapImage();
  }

  void RRTPlanner::drawGoalInitPose()
  {
    if (goal_received_)
    {
      drawCircle(goal_, 8, cv::Scalar(12, 255, 43));
    }
    if (init_pose_received_)
    {
      drawCircle(init_pose_, 8, cv::Scalar(255, 200, 0));
    }
  }

  void RRTPlanner::plan(int goal_buffer, int step_size, int max_iterations)
  {
    // Reset these values so planning only happens once for a
    // given pair of initial pose and goal points
    goal_received_ = false;
    init_pose_received_ = false;

    // TODO: Fill out this function with the RRT algorithm logic to plan a collision-free
    //       path through the map starting from the initial pose and ending at the goal pose

    std::vector<Point2D> tree;
    tree.push_back(init_pose_);

    // Initialise random point generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> rand_x(0, map_grid_->info.width - 1);
    std::uniform_int_distribution<int> rand_y(0, map_grid_->info.height - 1);

    for (int i = 0; i < max_iterations; i++)
    {
      Point2D random_point(rand_x(gen), rand_y(gen));

      Point2D nearest_node = findNearestNode(tree, random_point);
      if ((nearest_node.x() == -1) || (nearest_node.x() == -1))
      {
        continue;
      }

      Point2D new_node = extendTree(tree, nearest_node, random_point, step_size);

      if (isPointUnoccupied(new_node))
      {
        tree.push_back(new_node);

        drawLine(nearest_node, new_node, cv::Scalar(0, 255, 0), 2);
        drawCircle(new_node, 4, cv::Scalar(0, 0, 255));

        publishPath();

        float distance_to_goal = calculatePointDistance(new_node, goal_);

        if (distance_to_goal < goal_buffer)
        {
          // Backtrack to construct the path
          Point2D current_node = new_node;

          while ((current_node.x() != init_pose_.x()) || (current_node.y() != init_pose_.y()))
          {
            planned_path.push_back(current_node);
            Point2D next_node = findNearestBacktrackNode(tree, current_node);
            drawLine(current_node, next_node, cv::Scalar(255, 0, 0), 2);
            current_node = next_node;
            publishPath();
          }
          planned_path.push_back(init_pose_);

          // Reverse the path to start from the initial pose
          std::reverse(planned_path.begin(), planned_path.end());

          planned_path.push_back(goal_);
          planned_path.push_back(new_node); // Add the final goal node

          return;
        }
      }
    }
  }

  float RRTPlanner::calculatePointDistance(Point2D point1, Point2D point2)
  {
    return std::sqrt((float)std::pow(point1.x() - point2.x(), (float)2) + std::pow((float)point1.y() - point2.y(), (float)2));
  }

  Point2D RRTPlanner::findNearestNode(std::vector<Point2D> &tree, Point2D point)
  {
    Point2D nearest_node(-1, -1);
    float min_distance = -1;

    for (Point2D &tree_node : tree)
    {
      float distance = calculatePointDistance(tree_node, point);
      if ((distance < min_distance) || (min_distance == -1))
      {
        float dx = ((float)point.x() - tree_node.x()) / distance;
        float dy = ((float)point.y() - tree_node.y()) / distance;

        bool obstacle_found = false;

        for (float step = 0; step < distance; step += (float)0.5)
        {
          int sub_x = static_cast<int>(tree_node.x() + step * dx);
          int sub_y = static_cast<int>(tree_node.y() + step * dy);

          if (!isPointUnoccupied(Point2D(sub_x, sub_y)))
          {
            obstacle_found = true;
            break;
          }
        }

        if (!obstacle_found)
        {
          min_distance = distance;
          nearest_node = tree_node;
        }
      }
    }

    return nearest_node;
  }

  Point2D RRTPlanner::findNearestBacktrackNode(std::vector<Point2D> &tree, Point2D point)
  {
    Point2D nearest_node(-1, -1);
    float min_distance = -1;

    for (Point2D &tree_node : tree)
    {
      float distance = calculatePointDistance(tree_node, point);
      if ((distance < min_distance) || (min_distance == -1))
      {
        float dx = ((float)point.x() - tree_node.x()) / distance;
        float dy = ((float)point.y() - tree_node.y()) / distance;

        bool obstacle_found = false;

        for (float step = 0; step < distance; step += (float)0.5)
        {
          int sub_x = static_cast<int>(tree_node.x() + step * dx);
          int sub_y = static_cast<int>(tree_node.y() + step * dy);

          if (!isPointBacktrackNode(Point2D(sub_x, sub_y)))
          {
            obstacle_found = true;
            break;
          }

          if (!obstacle_found)
          {
            min_distance = distance;
            nearest_node = tree_node;
          }
        }
      }
    }

    return nearest_node;
  }

  Point2D RRTPlanner::extendTree(std::vector<Point2D> &tree, Point2D &nearest_node, Point2D &random_point, int step_size)
  {
    float distance_to_random = calculatePointDistance(nearest_node, random_point);

    if (distance_to_random <= step_size)
    {
      return random_point;
    }

    float dx = ((float)random_point.x() - nearest_node.x()) / distance_to_random;
    float dy = ((float)random_point.y() - nearest_node.y()) / distance_to_random;

    int new_x = nearest_node.x() + static_cast<int>(step_size * dx);
    int new_y = nearest_node.y() + static_cast<int>(step_size * dy);

    return Point2D(new_x, new_y);
  }

  void RRTPlanner::publishPath()
  {
    // Create new Path msg
    nav_msgs::Path path;
    path.header.frame_id = map_grid_->header.frame_id;
    path.header.stamp = ros::Time::now();

    // TODO: Fill nav_msgs::Path msg with the path calculated by RRT
    std::vector<geometry_msgs::PoseStamped> poses;
    for (int i = 0; i < planned_path.size(); i++)
    {
      poses.push_back(pointToPose(planned_path[i]));
    }
    path.poses = poses;

    // Publish the calculated path
    path_pub_.publish(path);

    displayMapImage();
  }

  bool RRTPlanner::isPointUnoccupied(const Point2D &p)
  {
    // Check if a given point is occupied/free in the map
    cv::Vec3b point = map_->at<cv::Vec3b>(map_grid_->info.height - p.x() - 1, p.y());

    if (point[0] == 0 && point[1] == 0 && point[2] == 0)
    {
      ROS_INFO("(%d, %d) is occupied with r: %d g: %d b: %d", p.x(), p.y(), point[0], point[1], point[2]);
      return false;
    }
    ROS_INFO("(%d, %d) is NOT occupied with r: %d g: %d b: %d", p.x(), p.y(), point[0], point[1], point[2]);
    return true;
  }

  bool RRTPlanner::isPointBacktrackNode(const Point2D &p)
  {
    // Check if a given point is occupied/free in the map
    cv::Vec3b point = map_->at<cv::Vec3b>(map_grid_->info.height - p.x() - 1, p.y());

    if (point[0] == 255 && point[1] == 0 && point[2] == 0)
    {
      ROS_INFO("(%d, %d) has been backtracked with r: %d g: %d b: %d", p.x(), p.y(), point[0], point[1], point[2]);
      return false;
    }
    ROS_INFO("(%d, %d) has NOT been backtracked with r: %d g: %d b: %d", p.x(), p.y(), point[0], point[1], point[2]);
    return true;
  }

  void RRTPlanner::buildMapImage()
  {
    // Create a new opencv matrix with the same height and width as the received map
    map_ = std::unique_ptr<cv::Mat>(new cv::Mat(map_grid_->info.height,
                                                map_grid_->info.width,
                                                CV_8UC3,
                                                cv::Scalar::all(255)));

    // Fill the opencv matrix pixels with the map points
    for (int i = 0; i < map_grid_->info.height; i++)
    {
      for (int j = 0; j < map_grid_->info.width; j++)
      {
        if (map_grid_->data[toIndex(i, j)])
        {
          map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(0, 0, 0);
        }
        else
        {
          map_->at<cv::Vec3b>(map_grid_->info.height - i - 1, j) = cv::Vec3b(255, 255, 255);
        }
      }
    }
  }

  void RRTPlanner::displayMapImage(int delay)
  {
    cv::imshow("Output", *map_);
    cv::waitKey(delay);
  }

  void RRTPlanner::drawCircle(Point2D &p, int radius, const cv::Scalar &color)
  {
    cv::circle(
        *map_,
        cv::Point(p.y(), map_grid_->info.height - p.x() - 1),
        radius,
        color,
        -1);
  }

  void RRTPlanner::drawLine(Point2D &p1, Point2D &p2, const cv::Scalar &color, int thickness)
  {
    cv::line(
        *map_,
        cv::Point(p2.y(), map_grid_->info.height - p2.x() - 1),
        cv::Point(p1.y(), map_grid_->info.height - p1.x() - 1),
        color,
        thickness);
  }

  inline geometry_msgs::PoseStamped RRTPlanner::pointToPose(const Point2D &p)
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = p.y() * map_grid_->info.resolution;
    pose.pose.position.y = p.x() * map_grid_->info.resolution;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = map_grid_->header.frame_id;
    return pose;
  }

  inline void RRTPlanner::poseToPoint(Point2D &p, const geometry_msgs::Pose &pose)
  {
    p.x(pose.position.y / map_grid_->info.resolution);
    p.y(pose.position.x / map_grid_->info.resolution);
  }

  inline int RRTPlanner::toIndex(int x, int y)
  {
    return x * map_grid_->info.width + y;
  }

} // namespace rrt_planner
