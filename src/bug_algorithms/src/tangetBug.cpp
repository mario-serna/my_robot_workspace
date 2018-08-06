#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose2D.h"
#include "vector"
#include "algorithm"
#include "std_srvs/SetBool.h"
#include "utils.h"
#include "bug_algorithms/bugSwitch.h"
#include "bug_algorithms/nodeState.h"
#include "bug_algorithms/algorithmState.h"
#include <ros/callback_queue.h>
#include "visualization_msgs/Marker.h"

static int rate_hz = 20;
static bool isSimulation = true;
static bool isPoseReady = false;
static bool areSubsDown = true;
static float linear_vel_, angular_vel_;
static bug_algorithms::nodeState nodeStateGlobal;
static string node_name = "tangetBug";
static int algorithm_id = 6;

enum NodeStates {Waiting, Initializing, Executing, Pause, Stopping};
static int node_state_ = Waiting;

enum States {GoToPoint, GoToPointFollowing, FollowBoundary, Success, Fail};
static int state_ = GoToPoint;
static int count_state_time = 0; // Seconds the robot is in a state
static int count_loop = 0;
static int count_tolerance = 0; // Seconds the robot must wait
static int laser_align_index;
static int laser_samples = 727;

static float laser_angle = PI/1.5; // +-120 degrees
static float max_laser_range = 4.0;
static float yaw_ = 0;
static float yaw_error_allowed = 5 * (PI/180); // 5 degrees
static geometry_msgs::Point position_ = geometry_msgs::Point();
static geometry_msgs::Point laser_position_ = geometry_msgs::Point();
static geometry_msgs::Point initial_position_ = geometry_msgs::Point();
static geometry_msgs::Point desired_position_ = geometry_msgs::Point();
static geometry_msgs::Point hit_point = geometry_msgs::Point();
static geometry_msgs::Point leave_point = geometry_msgs::Point();
// M is the point on the sensed obstacle which has the shorted distance to the goal
static geometry_msgs::Point m_point = geometry_msgs::Point();

static bool isClosestPointInit = false;
static bool isHitPointInit = false;
static bool leaveCondition = false;
static int count_same_point = 0;
static float yaw_precision_ = PI/90; // 2 degrees
static float desired_yaw = 0.0;
static float err_yaw = 0.0;
static float dist_precision_ = 0.3;
static float dist_detection = 0.4;
static float initial_to_goal_distance = 0.0;
static float current_to_goal_distance = 0.0;
static float free_distance = 0.0;
static float best_distance = 0.0;
// Followed distance is the shortest distance between the sensed boundary and the goal
static float followed_distance = 0.0;
// Reach_distance is the shortest distance between blocking obstacle and goal
// (or the current distance to goal if no blocking obstacle visible)
static float reach_distance = 0.0;
static float leave_distance = 0.0;
static bool lockPoint = false;
static bool isInitAlign = true;
geometry_msgs::Point target = geometry_msgs::Point();

static bool reverseCriterion;

static bool isPreviousReady = false;
static geometry_msgs::Point previous_position_ = geometry_msgs::Point();
static float path_length = 0.0;

// Marker variables
static bool pubMarker = false;
static bool isPathMarkerReady = false;
static visualization_msgs::Marker start_marker = visualization_msgs::Marker();
static visualization_msgs::Marker goal_marker = visualization_msgs::Marker();
static visualization_msgs::Marker path_marker = visualization_msgs::Marker();
static visualization_msgs::Marker path_marker_go_to = visualization_msgs::Marker();
static visualization_msgs::Marker path_marker_follow = visualization_msgs::Marker();

static map<string, float> regions_ = {
  {"right",0},
  {"front_right",0},
  {"front",0},
  {"front_left",0},
  {"left",0}
};

static map<int, string> node_state_desc = {
  {Waiting, "Waiting"},
  {Initializing, "Initializing"},
  {Executing, "Executing"},
  {Pause, "Pause"},
  {Stopping, "Stopping"}
};

static map<int, string> state_desc_ = {
  {GoToPoint, "Go to the point"},
  {GoToPointFollowing, "Go to the discontinuity point"},
  {FollowBoundary, "Follow the obstacle boundary"},
  {Success, "Goal reached"},
  {Fail, "Fail - Goal not reachable"}
};

geometry_msgs::Twist twist_msg;
ros::Publisher node_state_pub;
ros::Publisher target_point_pub;
ros::Publisher algorithm_state_pub;
ros::Publisher marker_pub;
ros::Subscriber node_state_sub;
ros::Subscriber odom_sub;
ros::Subscriber laser_sub;
ros::Subscriber laser_align_sub;
ros::Subscriber laser_detect_discont_sub;
ros::Subscriber lost_obstacle_sub;
ros::ServiceClient srv_client_go_to_point;
ros::ServiceClient srv_client_follow_boundary;

ros::CallbackQueue custom_queue;

void subscriberNotify(string m){
  cout << "Subscriber active: " << m << endl;
}

bool bugSwitch(bug_algorithms::bugSwitchRequest& request, bug_algorithms::bugSwitchResponse& response){
  string message;

  response.success = true;
  message.append(node_state_desc[node_state_]);
  message.append("->");
  message.append(node_state_desc[request.state]);

  if(node_state_ == Waiting && request.state == Initializing){
    node_state_ = request.state;
  } else if(node_state_ == Initializing && request.state == Stopping){
    node_state_ = request.state;
  } else if(node_state_ == Executing && (request.state == Pause || request.state == Stopping)){
    node_state_ = request.state;
  } else if(node_state_ == Pause && (request.state == Executing || request.state == Stopping)){
    node_state_ = request.state;
  } else{
    response.success = false;
    message = "";
    message.append("Current state: ");
    message.append(node_state_desc[node_state_]);
    message.append(" | State not allowed: ");
    message.append(node_state_desc[request.state]);
  }

  response.message = message;

  return true;
}

void publishNodeState(){

  bug_algorithms::nodeState n = bug_algorithms::nodeState();
  n.algorithm = algorithm_id;
  n.node_state = node_state_;
  n.node_state_desc = node_state_desc[node_state_];
  n.bug_state = state_;
  n.bug_state_desc = state_desc_[state_];

  node_state_pub.publish(n);
}

void nodeStateCallback(const bug_algorithms::nodeState::ConstPtr& msg){
  nodeStateGlobal.algorithm = msg->algorithm;
  nodeStateGlobal.bug_state = msg->bug_state;
  nodeStateGlobal.bug_state_desc = msg->bug_state_desc;
  nodeStateGlobal.node_state = msg->node_state;
  nodeStateGlobal.node_state_desc = msg->node_state_desc;
}

void publishPathMarkers(){
  if(isPathMarkerReady){
    marker_pub.publish(start_marker);
    marker_pub.publish(goal_marker);
    marker_pub.publish(path_marker);
    marker_pub.publish(path_marker_go_to);
    marker_pub.publish(path_marker_follow);
  }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
  //subscriberNotify(node_name);
  position_ = msg->pose.pose.position;

  tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  yaw_ = yaw;

  // Laser pose stimation
  laser_position_.x  = position_.x + round(0.12*cos(yaw), 2);
  laser_position_.y  = position_.y + round(0.12*sin(yaw), 2);

  isPoseReady = true;

  if(isPreviousReady){
    float a_x,a_y,b_x,b_y;
    a_x = round(previous_position_.x, 1);
    a_y = round(previous_position_.y, 1);
    b_x = round(position_.x, 1);
    b_y = round(position_.y, 1);
    path_length = path_length + getDistance(a_x,a_y,b_x,b_y);
  }
  else{
    isPreviousReady = true;
    path_length = 0;
  }
  previous_position_ = position_;

  bug_algorithms::algorithmState a_state;
  a_state.algorithm = algorithm_id;
  a_state.name = node_name;
  a_state.pose_x = position_.x;
  a_state.pose_y = position_.y;
  a_state.yaw = yaw_;
  a_state.initial_to_goal_distance = initial_to_goal_distance;
  a_state.current_to_goal_distance = current_to_goal_distance;
  a_state.best_distance = best_distance;
  a_state.path_length = path_length;
  algorithm_state_pub.publish(a_state);

  if(pubMarker && isPathMarkerReady){
    pubMarker = false;
    // Publishing path marker
    path_marker.points.push_back(position_);
    marker_pub.publish(path_marker);
    if(state_ == GoToPoint || state_ == GoToPointFollowing){
      path_marker_go_to.points.push_back(position_);
      marker_pub.publish(path_marker_go_to);
    } else{
      path_marker_follow.points.push_back(position_);
      marker_pub.publish(path_marker_follow);
    }
  }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  // Check the utils.h file for the functions processRay and myfn
  regions_["right"] = *min_element(begin(msg->ranges), begin(msg->ranges)+200, myfn);
  regions_["right"] = processRay(regions_["right"], max_laser_range);
  regions_["front_right"] = *min_element(begin(msg->ranges)+200, begin(msg->ranges)+310, myfn);
  regions_["front_right"] = processRay(regions_["front_right"], max_laser_range);
  regions_["front"] = *min_element(begin(msg->ranges)+310, begin(msg->ranges)+420, myfn);
  regions_["front"] = processRay(regions_["front"], max_laser_range);
  regions_["front_left"] = *min_element(begin(msg->ranges)+420, begin(msg->ranges)+530, myfn);
  regions_["front_left"] = processRay(regions_["front_left"], max_laser_range);
  regions_["left"] = *min_element(begin(msg->ranges)+530, end(msg->ranges), myfn);
  regions_["left"] = processRay(regions_["left"], max_laser_range);

  laser_samples = msg->ranges.size();

  //ROS_INFO("IMP\nRight: %f \nFront_right: %f \nFront: %f \nFront_left: %f \nLeft: %f",
  //         regions_["right"], regions_["front_right"], regions_["front"], regions_["front_left"], regions_["left"]);
}

void laserAlignCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

  float d_yaw = atan2(desired_position_.y - laser_position_.y, desired_position_.x - laser_position_.x);
  float e_yaw = normalizeAngle(d_yaw - yaw_);
  float free_temp;

  if(abs(e_yaw) < PI/1.5){
    laser_align_index = mapRange(e_yaw, -laser_angle, laser_angle, 0, msg->ranges.size()-1);
    free_temp = processRaySimple(msg->ranges[laser_align_index], max_laser_range);
    // Avoiding laser error
    if(free_temp > 0.15)
      free_distance = free_temp;
    /*if(isnan(msg->ranges[laser_align_index])){
      free_distance = 4;
    } else{
      free_distance = msg->ranges[laser_align_index] > 4 ? 4 : msg->ranges[laser_align_index];
    }*/
    //    cout << "Err Yaw: " << e_yaw << " | Des Yaw: " << d_yaw << " | Cur Yaw: " << yaw_ << endl;
    //    cout << "N: " << laser_align_index << endl;
    //    cout << "Laser: " << free_distance << endl;
  } else{
    free_distance = 0;
    laser_align_index = -1;
  }
}

void laserDetectDiscontinuityCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

  int n;
  int size = msg->ranges.size();
  float best_point_distance = 9999.0;
  float point_distance, current_to_point_distance;
  float a,b;
  geometry_msgs::Point p = geometry_msgs::Point();
  vector<int> disPointIndex = vector<int>();
  vector<float> disPointVal = vector<float>();
  bool temp_lock = false;

  desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
  err_yaw = normalizeAngle(desired_yaw - yaw_);

  int ray_inf_count = 0;
  // Detecting discontinuity points
  for(int i = 2; i < size-2; i++){
    a = isnan(msg->ranges[i]) ? 4.5 : (msg->ranges[i] > 4 ? 4.5 : msg->ranges[i]);
    b = isnan(msg->ranges[i+1]) ? 4.5 : (msg->ranges[i+1] > 4 ? 4.5 : msg->ranges[i+1]);

    if(a > 0.15 && a > 0.15){
      if(abs(a - b) > 0.4){
        if(a < 4 && b < 4){
          disPointIndex.push_back(i);
          disPointVal.push_back(a);
          disPointIndex.push_back(i+1);
          disPointVal.push_back(b);
          i++;
        } else if(a < b){
          disPointIndex.push_back(i);
          disPointVal.push_back(a);
        } else{
          int temp_size = disPointIndex.size();
          if(temp_size > 0){
            if(abs(i - disPointIndex[temp_size-1]) > 10){
              disPointIndex.push_back(i+1);
              disPointVal.push_back(b);
              i++;
            } else{
              disPointIndex.pop_back();
              disPointVal.pop_back();
            }
          } else{
            disPointIndex.push_back(i+1);
            disPointVal.push_back(b);
            i++;
          }
        }
      }
    }
  }

  if(disPointIndex.size() < 2){
    // Check discontinuity for right side
    a = isnan(msg->ranges[0]) ? 4 : (msg->ranges[0] > 4 ? 4 : msg->ranges[0]);
    b = isnan(msg->ranges[1]) ? 4 : (msg->ranges[1] > 4 ? 4 : msg->ranges[1]);

    if(a > 0.15 && a < 4 && b > 0.15 && b < 4){
      if(abs(a - b) < 0.2){
        disPointIndex.push_back(0);
        disPointVal.push_back(a);
      }
    }

    // Check discontinuity for left side
    a = isnan(msg->ranges[size-1]) ? 4 : (msg->ranges[size-1] > 4 ? 4 : msg->ranges[size-1]);
    b = isnan(msg->ranges[size-2]) ? 4 : (msg->ranges[size-2] > 4 ? 4 : msg->ranges[size-2]);

    if(a > 0.15 && a < 4 && b > 0.15 && b < 4){
      if(abs(a - b) < 0.2){
        disPointIndex.push_back(size-1);
        disPointVal.push_back(a);
      }
    }

    if(disPointIndex.size() > 0){
      temp_lock = true;
    }
  }

  if((current_to_goal_distance - free_distance) <= 0){
    target = desired_position_;
    lockPoint = false;
  } else if(isInitAlign || (!lockPoint && free_distance > 3)){
    target = desired_position_;
    // Make sure the robot is align
    if(abs(err_yaw) < PI/20)
      isInitAlign = false;

  } else{

    if(!lockPoint){

      // Compute and select best discontinuity point
      if(disPointIndex.size() > 0){

        for(int i = 0; i < disPointIndex.size(); i++){
          if(disPointVal[i] > 0.5){
            float val = mapRangeFloat(disPointIndex[i], 0, size-1, -laser_angle, laser_angle);
            // Discontinuity point pose stimation
            p.x  = laser_position_.x + (disPointVal[i]*cos(yaw_ + val));
            p.y  = laser_position_.y + (disPointVal[i]*sin(yaw_ + val));

            current_to_point_distance = getDistance(position_, p);
            // Total distance = current position to point + point position to goal
            point_distance = current_to_point_distance + getDistance(p, desired_position_);

            if(point_distance < best_point_distance){
              best_point_distance = point_distance;
              n = disPointIndex[i];
              target = p;
              target.z = best_point_distance;
            }

            //cout << "Ind: " << disPointIndex[i] << " | Val: " << val;
            //cout << " | X: " << p.x << " | Y: " << p.y << " | Dist: " << point_distance << endl;
          }
        }

        if(temp_lock && best_point_distance < 999 && best_point_distance > current_to_goal_distance && disPointIndex.size() < 3 && (abs(err_yaw) < PI/6)){
          lockPoint = true;
        } else if(best_point_distance > 999 || best_point_distance < current_to_goal_distance){
          best_point_distance = 0;
          target = desired_position_;
          target.z = best_point_distance;
        }

        //cout << "Align point: " << laser_align_index << endl;
        //cout << "Best point: " << n << endl;
        //cout << "Best distance: " << best_point_distance << endl;
      } else {

        best_point_distance = 0;
        target = desired_position_;
        target.z = best_point_distance;
        //cout << "Best point: " << laser_align_index << endl;
        //cout << "Best distance: " << best_point_distance << endl;
      }
    } else{
      cout << "Point Locked!!" << endl;
      cout << "X: " << target.x << " | Y: " << target.y << endl;
    }
  }

  target_point_pub.publish(target);
  cout << "Discontinuity points: " << disPointIndex.size() << endl;
}

void waitPose(){
  /*This prevents the robot's pose from being 0*/

  bool wait = true;
  ROS_INFO("Waiting for pose...");

  while(!isPoseReady && node_state_ == Initializing){
    ros::spinOnce();
  }
  ROS_INFO("Robot: %f, %f", position_.x, position_.y);

}

void changeState(int state){
  bug_algorithms::bugSwitch srv;

  count_state_time = 0;
  state_ = state;
  string s = state_desc_[state_];
  cout << node_name << " - State changed to: " << s << endl;
  //ROS_INFO("%s - State changed to: %s", node_name.c_str(), s.c_str());

  if(state_ == GoToPoint){
    //ROS_INFO("Calling go to point!");
    srv.request.state = Initializing;
    srv_client_go_to_point.call(srv);
    srv.request.state = Stopping;
    srv_client_follow_boundary.call(srv);
  } else if(state_ == GoToPointFollowing){
    //ROS_INFO("Calling follow boundary!");
    srv.request.state = Stopping;
    srv_client_go_to_point.call(srv);
    srv.request.state = Initializing;
    srv_client_follow_boundary.call(srv);
  } else if(state_ == FollowBoundary){
    //ROS_INFO("Calling follow boundary!");
    srv.request.state = Stopping;
    srv_client_go_to_point.call(srv);
    srv.request.state = Initializing;
    srv_client_follow_boundary.call(srv);
  } else if(state_ == Success || state_ == Fail){
    // Publishing the current node state before changing to waiting state
    publishNodeState();
    // Stopping all services
    srv.request.state = Stopping;
    srv_client_go_to_point.call(srv);
    srv.request.state = Stopping;
    srv_client_follow_boundary.call(srv);
    node_state_ = Waiting;
  }
  publishNodeState();
}

void lostObstacleCallback(const std_msgs::Bool::ConstPtr& msg){
  //subscriberNotify(node_name);
  if(state_ == FollowBoundary && msg->data == true){
    ROS_INFO("Obstacle lost!");
    changeState(GoToPoint);
  }
}

bool isOnPointRange(geometry_msgs::Point robot, geometry_msgs::Point point,float tolerance){
  if((abs(robot.x - point.x) < tolerance) &&
     (abs(robot.y - point.y) < tolerance)){
    //ROS_INFO("Robot is in the hit point range!");
    return true;
  }
  //ROS_INFO("Robot is out of range from the hit point!");
  return false;
}

void initMarkers(){
  string frame_id = isSimulation ? "odom" : "RosAria/pose" ;

  // Publishing initial and goal markers
  geometry_msgs::Vector3 scale = geometry_msgs::Vector3();
  std_msgs::ColorRGBA color = std_msgs::ColorRGBA();
  scale.x = 0.3;
  scale.y = 0.3;
  scale.z = 0.1;
  color.a = 1.0;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  // Deleting existing markers
  //visualization_msgs::Marker m = createMarker(frame_id, node_name, 0, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::DELETEALL, initial_position_, scale, color);
  //marker_pub.publish(m);
  start_marker = createMarker(frame_id, node_name, 1, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, initial_position_, scale, color);
  marker_pub.publish(start_marker);
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  goal_marker = createMarker(frame_id, node_name, 2, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD, desired_position_, scale, color);
  marker_pub.publish(goal_marker);

  // For line strip only scale.x is used and it controls the width of the line segments
  scale.x = 0.1;
  color.a = 1.0;
  color.r = 0.0;
  color.g = 1.0;
  color.b = 0.0;
  path_marker = createMarker(frame_id, node_name, 3, visualization_msgs::Marker::LINE_STRIP, visualization_msgs::Marker::ADD, geometry_msgs::Point(), scale, color);
  // Point of reference
  geometry_msgs::Point p = geometry_msgs::Point();
  p.x = 0;
  p.y = 0;
  p.z = 0.1;
  scale.x = 0.1;
  scale.y = 0.1;
  scale.z = 0.1;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  path_marker_go_to = createMarker(frame_id, node_name, 4, visualization_msgs::Marker::CUBE_LIST, visualization_msgs::Marker::ADD, p, scale, color);
  color.r = 1.0;
  color.g = 0.0;
  color.b = 0.0;
  path_marker_follow = createMarker(frame_id, node_name, 5, visualization_msgs::Marker::CUBE_LIST, visualization_msgs::Marker::ADD, p, scale, color);

  isPathMarkerReady = true;
}

void initBug(ros::NodeHandle& nh){
  ROS_INFO("Initializing %s Node", node_name.c_str());

  string vel_topic, odom_topic, laser_topic;

  nh.getParam("/simulation", isSimulation);
  nh.getParam("/desired_x", desired_position_.x);
  nh.getParam("/desired_y", desired_position_.y);
  nh.getParam("/velocity", linear_vel_);
  nh.getParam("/reverse", reverseCriterion);

  angular_vel_ = linear_vel_+0.1;

  // Restating the static variables
  hit_point = geometry_msgs::Point();
  leave_point = geometry_msgs::Point();
  m_point = geometry_msgs::Point();
  target = geometry_msgs::Point();
  target.x = desired_position_.x;
  target.y = desired_position_.y;

  isPoseReady = false;
  isHitPointInit = false;
  leaveCondition = false;
  lockPoint = false;

  // Path length variables
  isPreviousReady = false;
  isInitAlign = true;
  path_length = 0;

  // Marker
  isPathMarkerReady = false;

  if(isSimulation){
    cout << node_name << " | Using GazeboSim topics\n";
    vel_topic = "cmd_vel";
    odom_topic = "odom";
    laser_topic = "p3dx/laser/scan";
  } else{
    cout << node_name << " | Using RosAria topics\n";
    vel_topic = "RosAria/cmd_vel";
    odom_topic = "RosAria/pose";
    laser_topic = "scan";
  }

  // create options for subscriber and pass pointer to our custom queue
  ros::SubscribeOptions ops =
      ros::SubscribeOptions::create<sensor_msgs::LaserScan>(
        laser_topic, // topic name
        1, // queue length
        laserDetectDiscontinuityCallback, // callback
        ros::VoidPtr(), // tracked object, we don't need one thus NULL
        &custom_queue // pointer to callback queue object
        );
  laser_detect_discont_sub = nh.subscribe(ops);

  laser_sub = nh.subscribe(laser_topic, rate_hz, laserCallback);
  laser_align_sub = nh.subscribe(laser_topic, rate_hz, laserAlignCallback);
  lost_obstacle_sub = nh.subscribe("lost_obstacle", rate_hz, lostObstacleCallback);
  odom_sub = nh.subscribe(odom_topic, rate_hz, odomCallback);
  node_state_sub = nh.subscribe("bugServer/bugNodeState", rate_hz, nodeStateCallback);

  ROS_INFO("Waiting for goToPointAdvance service");
  ros::service::waitForService("goToPointAdvanceSwitch");
  ROS_INFO("Waiting for followBoundaryAdvanceSwitch service");
  ros::service::waitForService("followBoundaryAdvanceSwitch");

  srv_client_go_to_point = nh.serviceClient<bug_algorithms::bugSwitch>("goToPointAdvanceSwitch", true);
  srv_client_follow_boundary = nh.serviceClient<bug_algorithms::bugSwitch>("followBoundaryAdvanceSwitch", true);

  waitPose();

  initial_position_ = position_;

  initial_to_goal_distance = getDistance(initial_position_, desired_position_);
  current_to_goal_distance = initial_to_goal_distance;
  best_distance = current_to_goal_distance;
  reach_distance = current_to_goal_distance;
  followed_distance = current_to_goal_distance;

  initMarkers();

  // Subscribers are ready
  areSubsDown = false;

  // Check if the current state is Initializing for the triggered Stopping case
  if(node_state_ == Initializing){
    // Changing node state to Executing
    node_state_ = Executing;

    // Initialize going to the point
    changeState(GoToPoint);
  }
}

void pauseBug(bool isPause){
  bug_algorithms::bugSwitch srv;
  publishNodeState();
  // Stopping all services
  if(isPause){
    srv.request.state = Pause;
    srv_client_go_to_point.call(srv);
    srv_client_follow_boundary.call(srv);
  } else{
    if(state_ == GoToPoint){
      srv.request.state = Executing;
      srv_client_go_to_point.call(srv);
    } else{
      srv.request.state = Executing;
      srv_client_follow_boundary.call(srv);
    }
  }
}

int isFreePath(geometry_msgs::Point desired, float tol){
  float desired_yaw = atan2(desired.y - position_.y, desired.x - position_.x);
  float err_yaw = normalizeAngle(desired_yaw - yaw_);

  if(abs(err_yaw) < (PI/2)){
    // Check if there is a free path
    if(abs(err_yaw) < (PI/6) && regions_["front_left"] > dist_detection+tol
       && regions_["front"] > dist_detection+(tol-0.05) && regions_["front_right"] > dist_detection+tol){
      //ROS_INFO("Less than 30 deg");
      return 1;

    } else if(err_yaw > 0 && (abs(err_yaw) > (PI/6)) && (abs(err_yaw) < (PI/2))
              && regions_["left"] > dist_detection+0.2 && regions_["front_left"] > dist_detection+0.1){
      //ROS_INFO("Between 30 and 90 - to the left");
      return 1;

    } else if(err_yaw < 0 && (abs(err_yaw) > (PI/6)) && (abs(err_yaw) < (PI/2))
              && regions_["right"] > dist_detection+0.2 && regions_["front_right"] > dist_detection+0.1){
      //ROS_INFO("Between 30 and 90 - to the right");
      return 1;
    }
  } else{
    // Ignore
    return 2;
  }

  // There is not a free path
  return 0;
}

void bugConditions(){

  bool isCurrentClose = false;
  bool isReachDistanceClose = false;
  geometry_msgs::Point p = geometry_msgs::Point();

  // Updating tanget bug distance
  if(free_distance > 0){
    float val = mapRangeFloat(laser_align_index, 0, laser_samples-1, -laser_angle, laser_angle);
    // Pose stimation
    p.x  = laser_position_.x + (free_distance*cos(yaw_ + val));
    p.y  = laser_position_.y + (free_distance*sin(yaw_ + val));
    reach_distance = getDistance(p, desired_position_);

    if(reach_distance+0.1 < followed_distance){
      cout << "Reach: " << reach_distance << " | Followed: " << followed_distance <<  endl;
      m_point = p;
      followed_distance = reach_distance;
      isReachDistanceClose = true;
    }
  }

  current_to_goal_distance = getDistance(position_, desired_position_);
  desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
  err_yaw = normalizeAngle(desired_yaw - yaw_);

  if(current_to_goal_distance+0.1 < best_distance){
    best_distance = current_to_goal_distance;
    isCurrentClose = true;
  }

  if(state_ == GoToPoint || state_ == GoToPointFollowing){
    if(state_ == GoToPoint && isFreePath(target, 0) == 0){
      changeState(GoToPointFollowing);
      return;
    }

    if(state_ == GoToPointFollowing){
      if(lockPoint && !isOnPointRange(position_, target, 0.5)){
        changeState(GoToPoint);
        return;
      }
      if(!leaveCondition){
        if(free_distance > 1){
          //cout << "Leave condition 2: " << current_to_goal_distance - free_distance << endl;
          //cout << "Free: " << free_distance << endl;
          leaveCondition = true;
          count_tolerance = 0;
        } else if((current_to_goal_distance - free_distance) <= 0){
          //cout << "Leave condition 1: " << current_to_goal_distance - free_distance << endl;
          //cout << "Free: " << free_distance << endl;
          leaveCondition = true;
          count_tolerance = 0;
        }

      } else if(count_tolerance > 1){
        leaveCondition = false;
        if(isFreePath(desired_position_, 0.1) == 1){
          changeState(GoToPoint);
          return;
        }
      }
    }

    if(regions_["left"] < dist_detection+0.1 || regions_["right"] < dist_detection+0.1){
      if(lockPoint && isOnPointRange(position_, target, 0.5)){
        cout << "Condition 1" << endl;
        lockPoint = false;
        leaveCondition = false;
        changeState(FollowBoundary);
        return;
      }

      if(current_to_goal_distance-0.2 > best_distance){
        cout << "Condition 2" << endl;
        lockPoint = false;
        leaveCondition = false;
        changeState(FollowBoundary);
        return;
      }

      /*if(reach_distance-0.2 > followed_distance){
        lockPoint = false;
        leaveCondition = false;
        cout << "Condition 3" << endl;
        cout << "Reach: " << reach_distance-0.2 << " | Followed: " << followed_distance <<  endl;
        changeState(FollowBoundary);
        return;
      }*/
    }

    if(getDistance(position_, desired_position_) < 0.3){
      changeState(Success);
      return;
    }
  } else if(state_ == FollowBoundary){
    lockPoint = false;

    // Terminate boundary following behavior when reach_distance < followed_distance
    /*if(reach_distance < followed_distance){
         changeState(GoToPoint);
       }*/

    // If the free distance from the current position to goal is gratter than rangeMax (4m) then set freeDistance = rangeMax

    /*float hit_to_goal_distance = getDistance(hit_point, desired_position_);

       if(count_state_time > 10){

         if(getDistance(position_, hit_point) < 0.3){
           count_same_point++;
           if(reverseCriterion && count_same_point == 2){
             cout << "Count for hit point is: " << count_same_point << endl;
             changeState(Fail);
             return;
           } else if(!reverseCriterion){
             cout << "Count for hit point is: " << count_same_point << endl;
             changeState(Fail);
             return;
           }
           cout << "Count for hit point is: " << count_same_point << endl;
           count_state_time = 0;

         }
       }*/

    if(!leaveCondition){

      if(isCurrentClose){
        if(free_distance > 1){
          cout << "Leave condition 2: " << current_to_goal_distance - free_distance << endl;
          cout << "Free: " << free_distance << endl;
          leaveCondition = true;
          count_tolerance = 0;
        } else if((current_to_goal_distance - free_distance) <= 0){
          cout << "Leave condition 1: " << current_to_goal_distance - free_distance << endl;
          cout << "Free: " << free_distance << endl;
          leaveCondition = true;
          count_tolerance = 0;
        }

      }

    } else{
      leaveCondition = false;
      if(isFreePath(desired_position_, 0.1) == 0){
        changeState(GoToPointFollowing);
      } else{
        changeState(GoToPoint);
      }
    }
  }
}

void shutDownSubscribers(){
  cout << node_name << ": shutting down subscribers" << endl;
  laser_sub.shutdown();
  laser_align_sub.shutdown();
  laser_detect_discont_sub.shutdown();
  odom_sub.shutdown();
  lost_obstacle_sub.shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  // While the node is in Pause, it only will call the pauseBug function once.
  bool pauseBand = true;
  int count = 0;

  nodeStateGlobal.algorithm = algorithm_id;

  node_state_pub = nh.advertise<bug_algorithms::nodeState>("bugServer/bugNodeStateInternal", rate_hz);
  target_point_pub = nh.advertise<geometry_msgs::Point>("bugServer/targetPoint", rate_hz);
  algorithm_state_pub = nh.advertise<bug_algorithms::algorithmState>("bugServer/algorithmState", rate_hz);
  marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", rate_hz);

  ros::ServiceServer service = nh.advertiseService("bugServer/tangetBugSwitch", bugSwitch);

  ros::Rate rate(rate_hz);
  while(!ros::isShuttingDown()){
    if(node_state_ == Waiting){
      if(!areSubsDown){
        shutDownSubscribers();
        areSubsDown = true;
      }

    } else if(node_state_ == Initializing){
      initBug(nh);

    } else if(node_state_ == Executing){
      if(!pauseBand){
        pauseBug(false);
      }
      pauseBand = true;
      bugConditions();

    } else if(node_state_ == Pause){
      if(pauseBand){
        ROS_INFO("%s paused", node_name.c_str());
        pauseBug(true);
        pauseBand = false;
      }

    } else if(node_state_ == Stopping){
      changeState(Fail);
    }
    //publishNodeState();

    ros::spinOnce();
    rate.sleep();

    count_loop++;

    if(count_loop == rate_hz){
      count_state_time++;
      count_tolerance++;
      count_loop = 0;
      pubMarker = true;
      publishPathMarkers();
      ++count;
      if(count > 0){
        custom_queue.callAvailable();
        count = 0;
      }
    }
  }

  return 0;
}
