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

static int rate_hz = 20;
static bool isSimulation = true;
static bool isPoseReady = false;
static bool areSubsDown = true;
static float linear_vel_, angular_vel_;
static bug_algorithms::nodeState nodeStateGlobal;
static string node_name = "bug1";
static int algorithm_id = 1;

enum NodeStates {Waiting, Initializing, Executing, Pause, Stopping};
static int node_state_ = Waiting;

enum States {GoToPoint, Circumnavigate, FollowBoundary, Success, Fail};
static int state_ = GoToPoint;
static int count_state_time = 0; // Seconds the robot is in a state
static int count_loop = 0;

static float yaw_ = 0;
static float yaw_error_allowed = 5 * (PI/180); // 5 degrees
static geometry_msgs::Point position_ = geometry_msgs::Point();
static geometry_msgs::Point initial_position_ = geometry_msgs::Point();
static geometry_msgs::Point desired_position_ = geometry_msgs::Point();
static geometry_msgs::Point circ_starting_point = geometry_msgs::Point();
static geometry_msgs::Point circ_closest_point = geometry_msgs::Point();
static bool isClosestPointInit = false;
static int count_closestPoint = 0;
static float yaw_precision_ = PI/90; // 2 degrees
static float dist_precision_ = 0.3;
static float dist_detection = 0.4;
static float initial_to_goal_distance = 0.0;
static float current_to_goal_distance = 0.0;

static bool isPreviousReady = false;
static geometry_msgs::Point previous_position_ = geometry_msgs::Point();
static float path_length = 0.0;

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
  {Circumnavigate, "Circumnavigate the obstacle: looking for the closest point"},
  {FollowBoundary, "Follow the obstacle boundary: going to the closest point"},
  {Success, "Goal reached"},
  {Fail, "Fail - Goal not reachable"}
};

geometry_msgs::Twist twist_msg;
ros::Publisher node_state_pub;
ros::Publisher algorithm_state_pub;
ros::Subscriber node_state_sub;
ros::Subscriber odom_sub;
ros::Subscriber laser_sub;
ros::Subscriber lost_obstacle_sub;
ros::ServiceClient srv_client_go_to_point;
ros::ServiceClient srv_client_follow_boundary;

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

void nodeStateCallback(const bug_algorithms::nodeState::ConstPtr& msg)
{
  nodeStateGlobal.algorithm = msg->algorithm;
  nodeStateGlobal.bug_state = msg->bug_state;
  nodeStateGlobal.bug_state_desc = msg->bug_state_desc;
  nodeStateGlobal.node_state = msg->node_state;
  nodeStateGlobal.node_state_desc = msg->node_state_desc;
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
  a_state.path_length = path_length;
  algorithm_state_pub.publish(a_state);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  // Check the utils.h file for the functions processRay and myfn
  regions_["right"] = *min_element(begin(msg->ranges), begin(msg->ranges)+200, myfn);
  regions_["right"] = processRay(regions_["right"]);
  regions_["front_right"] = *min_element(begin(msg->ranges)+200, begin(msg->ranges)+310, myfn);
  regions_["front_right"] = processRay(regions_["front_right"]);
  regions_["front"] = *min_element(begin(msg->ranges)+310, begin(msg->ranges)+420, myfn);
  regions_["front"] = processRay(regions_["front"]);
  regions_["front_left"] = *min_element(begin(msg->ranges)+420, begin(msg->ranges)+530, myfn);
  regions_["front_left"] = processRay(regions_["front_left"]);
  regions_["left"] = *min_element(begin(msg->ranges)+530, end(msg->ranges), myfn);
  regions_["left"] = processRay(regions_["left"]);

  //ROS_INFO("IMP\nRight: %f \nFront_right: %f \nFront: %f \nFront_left: %f \nLeft: %f",
  //         regions_["right"], regions_["front_right"], regions_["front"], regions_["front_left"], regions_["left"]);
}

void waitPose(){
  /*This prevents the robot's pose from being 0*/

  bool wait = true;
  ROS_INFO("Waiting for pose...");

  while(!isPoseReady){
    ros::spinOnce();
  }
  ROS_INFO("Robot: %f, %f", position_.x, position_.y);

}

void changeState(int state){
  bug_algorithms::bugSwitch srv;

  count_state_time = 0;
  state_ = state;
  string s = state_desc_[state_];
  ROS_INFO("%s - State changed to: %s", node_name.c_str(), s.c_str());

  if(state_ == GoToPoint){
    //ROS_INFO("Calling go to point!");
    srv.request.state = Initializing;
    srv_client_go_to_point.call(srv);
    srv.request.state = Stopping;
    srv_client_follow_boundary.call(srv);
  } else if(state_ == Circumnavigate){
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
  if((state_ == Circumnavigate || state_ == FollowBoundary) && msg->data == true){
    ROS_INFO("Obstacle lost!");
    changeState(GoToPoint);
  }
}

void initBug(ros::NodeHandle& nh){
  ROS_INFO("Initializing %s Node", node_name.c_str());

  string vel_topic, odom_topic, laser_topic;
  nh.getParam("simulation", isSimulation);
  nh.getParam("desired_x", desired_position_.x);
  nh.getParam("desired_y", desired_position_.y);
  nh.getParam("velocity", linear_vel_);

  angular_vel_ = linear_vel_+0.1;

  // Restating the static variables
  circ_starting_point = geometry_msgs::Point();
  circ_closest_point = geometry_msgs::Point();
  isClosestPointInit = false;
  isPoseReady = false;

  // Path length variables
  isPreviousReady = false;
  path_length = 0;

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

  laser_sub = nh.subscribe(laser_topic, rate_hz, laserCallback);
  lost_obstacle_sub = nh.subscribe("lost_obstacle", rate_hz, lostObstacleCallback);
  odom_sub = nh.subscribe(odom_topic, rate_hz, odomCallback);
  node_state_sub = nh.subscribe("bugServer/bugNodeState", rate_hz, nodeStateCallback);

  ROS_INFO("Waiting for goToPoint service");
  ros::service::waitForService("goToPointSwitch");
  ROS_INFO("Waiting for followBoundary service");
  ros::service::waitForService("followBoundarySwitch");

  srv_client_go_to_point = nh.serviceClient<bug_algorithms::bugSwitch>("goToPointSwitch", true);
  srv_client_follow_boundary = nh.serviceClient<bug_algorithms::bugSwitch>("followBoundarySwitch", true);

  waitPose();
  initial_position_ = position_;

  initial_to_goal_distance = getDistance(initial_position_, desired_position_);
  current_to_goal_distance = initial_to_goal_distance;

  // Subscribers are ready
  areSubsDown = false;

  // Changing node state to Executing
  node_state_ = Executing;

  // Initialize going to the point
  changeState(GoToPoint);
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

void bugConditions(){
  current_to_goal_distance = getDistance(position_, desired_position_);

  if(state_ == GoToPoint){
    float desired_yaw, err_yaw;
    desired_yaw = atan2(desired_position_.y - position_.y, desired_position_.x - position_.x);
    err_yaw = normalizeAngle(desired_yaw - yaw_);

    // Checking if there is an obstacle in the way
    if((abs(err_yaw) < 0.1) &&
       ((regions_["front_right"] > 0.15 && regions_["front_right"] < dist_detection-0.1) ||
        (regions_["front"] > 0.15 && regions_["front"] < dist_detection) ||
        (regions_["front_left"] > 0.15 && regions_["front_left"] < dist_detection-0.1))){
      // If the value of this variable is false then
      // it's necessary to reset the values for the starting and closest point of circumnavigation
      if(isClosestPointInit){
        // If the closest point is rediscovered twice then the goal is not reachable
        if(count_closestPoint > 1){
          changeState(Fail);

        } else{
          // Checking if the robot is at the closest point
          if(getDistance(position_, circ_closest_point) < 0.3){
            count_closestPoint++;
            cout << "Count for closest point is: " << count_closestPoint << endl;

            // First chance to try to reach a closer point
            // This avoid the case when the robot leaves the closest point for an obstacle
            // but it detects a new one in front of it without moving forward
            if(count_closestPoint < 2){
              circ_starting_point = position_;
              circ_closest_point = position_;
              changeState(Circumnavigate);
            }
          } else{
            // Reset the variables
            circ_starting_point = position_;
            circ_closest_point = position_;
            count_closestPoint = 0;
            changeState(Circumnavigate);
          }
        }

      } else{
        // Reset the variables
        circ_starting_point = position_;
        circ_closest_point = position_;
        isClosestPointInit = true;
        count_closestPoint = 0;
        changeState(Circumnavigate);
      }

      // Checking if the robot reaches the goal with a presition of 0.3 meters
    } else if(getDistance(position_, desired_position_) < 0.3){
      changeState(Success);
    }
  } else if(state_ == Circumnavigate){
    // Updating the closest point
    if(getDistance(position_,desired_position_) < getDistance(circ_closest_point, desired_position_))
      circ_closest_point = position_;

    // Waiting for a time, after that check the condition
    if(count_state_time > 20 && getDistance(position_, circ_starting_point) < 0.3)
      changeState(FollowBoundary);

  } else if(state_ == FollowBoundary){
    if(getDistance(position_, circ_closest_point) < 0.3)
      changeState(GoToPoint);
  }
}

void shutDownSubscribers(){
  cout << node_name << ": shutting down subscribers" << endl;
  laser_sub.shutdown();
  odom_sub.shutdown();
  lost_obstacle_sub.shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh;
  // While the node is in Pause, it only will call the pauseBug function once.
  bool pauseBand = true;

  nodeStateGlobal.algorithm = 1;

  node_state_pub = nh.advertise<bug_algorithms::nodeState>("bugServer/bugNodeStateInternal", rate_hz);
  algorithm_state_pub = nh.advertise<bug_algorithms::algorithmState>("bugServer/algorithmState", rate_hz);

  ros::ServiceServer service = nh.advertiseService("bugServer/bug1Switch", bugSwitch);

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
      count_loop = 0;
    }
  }

  return 0;
}
