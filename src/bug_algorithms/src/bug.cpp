#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "tf/tf.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose2D.h"
#include "vector"
#include "algorithm"

using namespace std;

enum BugAlgorithms {Bug1, Bug2, DistBug, TangetBug, IntensitiveBug, PointBug };

ros::Publisher pose_pub;
ros::Publisher vel_pub;
ros::Publisher laser_pub;
ros::Publisher laser_pose_pub;
ros::Subscriber odom_sub;
ros::Subscriber laser_sub;
ros::Subscriber laser_pose_sub;
ros::Subscriber sonar_sub;
ros::Subscriber pose_sub;

// Simulator
static bool isSimulation;
static bool isUsingLaser = true;
static bool isLaserReady = false;
static bool isPoseReady = false;
static bool isLaserPoseReady = false;
static int rate;
static int bugAlgorithm;

// Robot
geometry_msgs::Pose2D robot_pose2d;
geometry_msgs::Pose2D laser_pose2d;
geometry_msgs::Point initial, goal;
geometry_msgs::Point pNearestGoal;
static double linearSpeed;
static double angularSpeed;
static vector<float> laser;
static int laserIndex[] = {0, 60, 90, 121, 227, 303, 363, 393, 500, 606, 606, 636, 666, 726};
static int laserDegrees[] = {-30,-20,0,10,45,70,90,110,135,170,180,200,210};
static int sameHitPoint = 0;
static int sameNearestPoint = 0;
static int sameNearestPointAux = 0;
static bool samePointCondition = false;
static bool sameNearestPointLock = false;
// 1 = upward | -1 = downward
static int robot_direction = 1;

// Extra behaviors
static bool useReverseCriterion = true;
static bool useChooseDirection = true;
static bool reversingCondition = false;

// Point where obstacle is detected
geometry_msgs::Point pHitObstacle;
static bool isHitPointInitialized = false;

// Distances
static float pointTolerance = 0.2;
static float initial2GoalDistance = 0.0;
static float currentDistance = 0.0;
static float hitPointDistance = 0.0;
static float bestDistance = 99999.0;
static float freeDistance = 0.0;
static bool isNewNearestPoint = false;

// Surrounding Flags
static bool leavingCondition = false;
static bool isRounding = false;
static bool isLeft;

// Conditions for surrounding obstacle
static bool flag = false;
static bool flag2 = false;
static bool flag3 = false;

/* Condition triggered if the robot detects an obstacle
while it is running the surrounding behavior*/
static bool flag4 = true;

// Minimun tolerance distance for obstacle detection
static const float frontalMindist = 0.38;
static const float sideMinDist = frontalMindist + 0.15;
static const float sideMinDistTolerance = 0.05;

const double PI = 3.1415926;

double degrees2radians(double angle_in_degrees){
  return angle_in_degrees * PI / 180.0;
}

double radians2degrees(double angle_in_radians){
  return angle_in_radians * 180.0 / PI;
}

double normCyclic(double val, double min, double max){
  if(val >= min){
    return min + fmod((val - min), (max - min));
  } else{
    return max - fmod((min - val), (max - min));
  }
}

double normDeg360(double angle){
  return normCyclic(angle, 0 ,360);
}

double round(double var, int presition)
{
  // 37.66666 * 100 =3766.66
  // 3766.66 + .5 =37.6716    for rounding off value
  // then type cast to int so value is 3766
  // then divided by 100 so the value converted into 37.66
  double value = (int)(var * pow(10, presition) + .5);
  return (double)value / pow(10, presition);
}

geometry_msgs::Point pose2DToPoint(geometry_msgs::Pose2D pose){
  geometry_msgs::Point p;
  p.x = pose.x;
  p.y = pose.y;
  return p;
}

double getDistance(double x1, double y1, double x2, double y2){
  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double getPendant(geometry_msgs::Point a, geometry_msgs::Point b){
  double m;

  a.x = round(a.x, 2);
  a.y = round(a.y, 2);
  b.x = round(b.x, 2);
  b.y = round(b.y, 2);

  if((b.x - a.x) != 0){
    if((b.y - a.y) != 0)
      m = (b.y - a.y) / (b.x - a.x);
    //m = round((b.y - a.y) / (b.x - a.x), 1);
    else
      b.x - a.x < 0 ? m = -99999 : m = 0 ;
  } else{
    m = 99999;
  }
  //  cout << "b.x: "<<b.x << " | a.x: "<<a.x << endl;
  //  cout << "b.y: "<<b.y << " | a.y: "<<a.y << endl;
  //  cout<<"M: "<< m <<endl;
  //  cout<<"Msign: "<< copysign(1, m) <<endl;

  return m;
}

double getAlpha(geometry_msgs::Point a, geometry_msgs::Point b){
  double alpha;
  double m;

  robot_direction = 1;

  //TODO: poner condicion de tolerancia al ingresar una posicion muy cercana al robot para evitar que gire.
  // La pendiente debe ser diferente de 0

  m = getPendant(a, b);

  //ROS_INFO("M: %f", m);

  if(m != 99999){
    if((b.y - a.y) < 0){
      robot_direction = -1;
    }

    if(robot_direction == 1){
      alpha = radians2degrees(atan(m));

      if(copysign(1, m) == -1){
        m == -99999 ? m = 0 : m;
        alpha = 180 + radians2degrees(atan(m));
      }

    } else{
      if(copysign(1, m) == -1){
        m == -99999 ? m = 0 : m;
        alpha = radians2degrees(atan(m));
      } else{
        alpha = robot_direction * (180 - radians2degrees(atan(m)));
      }
    }
  } else{
    alpha = 90;
    if((b.y - a.y) < 0){
      alpha = -90;
      robot_direction = -1;
    }
  }

  return alpha;

}

bool isOnPendant(geometry_msgs::Point initial, geometry_msgs::Point goal, geometry_msgs::Point point){
  //Way 1: Less expensive but more unstable
  double m1, m2 = 0;

  m1 = round(getPendant(initial, goal), 2);
  m2 = round(getPendant(point, goal), 2);

  m1 < -50 || m1 > 50 ? m1 = 99999: m1;
  m2 < -50 || m2 > 50 ? m2 = 99999: m2;

  cout << "M1: " << m1 << " | M2: " << m2 << endl;

  return abs(m1 - m2) <= 0.05;
}

bool isOnPendant(){
  // Way 2: More stable but more expensive
  double initial2CurrentDistance = getDistance(initial.x, initial.y, robot_pose2d.x, robot_pose2d.y);

  //ROS_INFO("SumIPG: %f | DistIG: %f", initial2CurrentDistance+currentDistance, initial2GoalDistance);

  return (initial2CurrentDistance+currentDistance)-initial2GoalDistance <= 0.05;
}

bool isRobotAlign(int tolerance){

  ros::spinOnce();

  // Robot angle in degrees
  double robot_deg_angle = radians2degrees(robot_pose2d.theta);

  double alpha, angle = 0;

  alpha = round(getAlpha(pose2DToPoint(robot_pose2d), goal),1);

  if(robot_deg_angle < 0){
    robot_deg_angle = -1 * normDeg360(360 - robot_deg_angle);
  }

  if(robot_direction == 1){
    angle = alpha - robot_deg_angle;
  } else{
    angle = alpha + (robot_direction * robot_deg_angle);
  }

  if(angle >= 0){
    angle <= 180 ? angle : angle = -360 + angle;
  } else{
    angle >= -180 ? angle : angle = 360 + angle;
  }

  //ROS_INFO("Alpha: %f | R_Deg: %f | Diff: %f", alpha, robot_deg_angle, abs(angle));
  
  return abs(angle) <= tolerance;
}

int isLaserRangeAlign(int minRange, int maxRange, float tolerance){

  ros::spinOnce();

  // Robot angle in degrees
  double robot_deg_angle = radians2degrees(robot_pose2d.theta);

  double alpha = 0;

  alpha = round(getAlpha(pose2DToPoint(laser_pose2d), goal),1);
  robot_deg_angle = round(robot_deg_angle, 1);

  for(int i = minRange; i <= maxRange; i++){
    if(abs(alpha - (robot_deg_angle+(laserDegrees[i]-90))) <= tolerance){
      //ROS_INFO("Alpha: %f | R_Deg: %f | Index: %i | Cond: %f", alpha, robot_deg_angle, i, robot_deg_angle+(laserDegrees[i]-90));
      return i;
    }
  }

  return -1;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  geometry_msgs::Pose2D pose2d, laser2d;
  pose2d.x = round(msg->pose.pose.position.x, 2);
  pose2d.y = round(msg->pose.pose.position.y, 2);

  tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  pose2d.theta = round(yaw, 4);

  //ROS_INFO("Odom: [%f, %f, %f]", pose2d.x, pose2d.y, pose2d.theta);

  // Laser pose stimation
  laser2d.x  = pose2d.x + round(0.12*cos(yaw), 2);
  laser2d.y  = pose2d.y + round(0.12*sin(yaw), 2);
  laser2d.theta = pose2d.theta;

  pose_pub.publish(pose2d);
  laser_pose_pub.publish(laser2d);

}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
  //laser = msg->ranges;
  laser.clear();
  isLaserReady = true;
  // 240° range 0 - 726
  // 240 / 727
  // 1 item = 0.33°
  //msg->range index = laser range degress / 0.33
  //msg->ranges array index:  0  | 60 | 90 |121 |227 |303 |363 |393 |500 |606 |636 |666 |726
  //Laser range degress:      0° | 20°| 30°| 40°| 75°|100°|120°|130°|165°|200°|210°|220°|240°
  //Real Degress:            330°|350°| 0° | 10°| 45°| 70°| 90°|100°|135°|170°|180°|190°|210°
  //laser array index:        0  | 1  | 2  | 3  | 4  | 5  | 6  | 7  | 8  | 9  | 10 | 11 | 12

  //Laser as sonar
  // 180° range: 90 - 636
  // 90 | 168 | 246 | 324 | 402 | 480 | 558 | 636 |
  // 0  |  1  |  2  |  3  |  4  |  5  |  6  |  7  |

  if(isUsingLaser){
    for(int i = 0; i < 13; i++){
      if(isnan(msg->ranges[laserIndex[i]]) || isinf(msg->ranges[laserIndex[i]])){
        laser.push_back(9999);
      } else{
        if(msg->ranges[laserIndex[i]] <= 0.1){
          laser.push_back(0);
        } else{
          laser.push_back(round(msg->ranges[laserIndex[i]], 2));
        }
      }
    }
  } else{
    laser = msg->ranges;
  }
  //ROS_INFO("Laser: %i", msg->ranges.size());
  //ROS_INFO("Laser Side: %f| %f| %f", laser[0], laser[2], laser[3]);
  //ROS_INFO("Laser Front: %f| %f| %f", laser[4], laser[5], laser[6]);
  //laser_pub.publish(laser);
}

void laserRegionCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

}

void sonarCallback(const sensor_msgs::PointCloud::ConstPtr& msg){
  //ROS_INFO("Sonar size: %i", msg->points.size());
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
  robot_pose2d.x = msg->x;
  robot_pose2d.y = msg->y;
  robot_pose2d.theta = msg->theta;
  isPoseReady = true;
  //ROS_INFO("Pose: [%f, %f, %f]", robot_pose2d.x, robot_pose2d.y, robot_pose2d.theta);
}

void laserPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
  laser_pose2d.x = msg->x;
  laser_pose2d.y = msg->y;
  laser_pose2d.theta = msg->theta;
  isLaserPoseReady = true;
  //ROS_INFO("Pose: [%f, %f, %f]", laser_pose2d.x, laser_pose2d.y, laser_pose2d.theta);
}

ros::Time waitClock(){
  // this wait is needed to ensure this ros node has gotten
  // simulation published /clock message, containing
  // simulation time.
  ros::Time last_ros_time_;
  bool wait = true;
  ROS_INFO("Waiting for clock...");
  if(isSimulation){
    while (wait)
    {
      last_ros_time_ = ros::Time::now();
      if (last_ros_time_.toSec() > 0){
        wait = false;
        ROS_INFO("Time: %f", last_ros_time_.toSec());
      }
    }
  }

  ROS_INFO("Clock ok");

  return last_ros_time_;
}

void waitLaser(){
  /*This prevents the robot's pose from being 0*/

  bool wait = true;
  ROS_INFO("Waiting for laser...");

  while(!isLaserReady){
    ros::spinOnce();
  }
  ROS_INFO("Laser ok");

}

void waitPose(){
  /*This prevents the robot's pose from being 0*/

  bool wait = true;
  ROS_INFO("Waiting for pose...");

  while(!isPoseReady){
    ros::spinOnce();
  }
  ROS_INFO("Robot: %f, %f, %f", robot_pose2d.x, robot_pose2d.y, robot_pose2d.theta);

}

void publishVelocity(double linearX, double angularZ){
  geometry_msgs::Twist vel_msg;

  vel_msg.linear.x = linearX;
  vel_msg.linear.y = 0;
  vel_msg.linear.z = 0;

  vel_msg.angular.x = 0;
  vel_msg.angular.y = 0;
  vel_msg.angular.z = angularZ;

  vel_pub.publish(vel_msg);
  ros::spinOnce();
}

bool move(double speed, double distance, bool isForward){
  geometry_msgs::Twist vel_msg;

  bool goalReached = false;

  // distance = speed * time

  // Linear velocity
  isForward ? vel_msg.linear.x = abs(speed) : vel_msg.linear.x = -abs(speed);

  vel_msg.linear.y = vel_msg.linear.z = 0;

  // Angular velocity
  vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0;

  ros::Time time = waitClock();
  double t0 = ros::Time::now().toSec();
  double t1;
  double current_distance = 0;
  ros::Rate loop_rate(rate);

  waitPose();
  waitLaser();
  geometry_msgs::Pose2D initial_pose = robot_pose2d;

  ROS_INFO("Init: %f", robot_pose2d.x);
  ROS_INFO("Moving speed: %f | init time: %f", vel_msg.linear.x, t0);
  do{
    vel_pub.publish(vel_msg);
    t1 = ros::Time::now().toSec();
    current_distance = speed * (t1 - t0);
    //current_distance = getDistance(initial_pose.x, initial_pose.y, robot_pose2d.x, robot_pose2d.y);
    ROS_INFO("Dist: DD(%f) | CR(%f) | RP(%f) | IP(%f)", distance, current_distance, robot_pose2d.x, initial_pose.x);
    ros::spinOnce();
    loop_rate.sleep();
  }while(current_distance < distance);

  if(current_distance >= distance){
    goalReached = true;
    ROS_INFO("Distance reached: %f", (t1-t0));
  } else{
    ROS_INFO("There is an obstacle!: %f", (t1-t0));
  }

  vel_msg.linear.x = 0;
  vel_pub.publish(vel_msg);
  ROS_INFO("Stop time: %f", (t1-t0));
  ROS_INFO("Position: IP(%f , %f) | FP(%f , %f)", initial_pose.x, initial_pose.y, robot_pose2d.x, robot_pose2d.y);
  ROS_INFO("Distance: %f", getDistance(initial_pose.x, initial_pose.y, robot_pose2d.x, robot_pose2d.y));

  return goalReached;
}

void rotate(double angular_speed, double relative_angle, bool clockwise){
  geometry_msgs::Twist vel_msg;

  // Linear velocity
  vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z = 0;

  // Angular velocity
  vel_msg.angular.x = vel_msg.angular.y = 0;

  // Convention: Clockwise = negative angle
  // if clockwise is true then turn right (negative) else turn left (positive)
  clockwise ? vel_msg.angular.z = -abs(angular_speed) : vel_msg.angular.z = abs(angular_speed);

  ros::Time time = waitClock();
  double t0 = ros::Time::now().toSec();
  double t1;
  double current_angle = 0;
  ros::Rate loop_rate(100);

  waitPose();
  double init_angle = robot_pose2d.theta;

  do{
    vel_pub.publish(vel_msg);
    t1 = ros::Time::now().toSec();
    current_angle = angular_speed * (t1 - t0);
    //ROS_INFO("Rotating: CA(%f) | RA(%f) | DA(%f)", current_angle, robot_pose2d.theta, relative_angle);
    ros::spinOnce();
    loop_rate.sleep();
  }while(current_angle < relative_angle);

  vel_msg.angular.z = 0;
  vel_pub.publish(vel_msg);
  ROS_INFO("Stop: %f | %f | %f", robot_pose2d.theta, current_angle, relative_angle);
  ROS_INFO("Initial angle: %f | Final angle: %f | Angle: %f", init_angle, robot_pose2d.theta, abs(robot_pose2d.theta - init_angle));
  ROS_INFO("Time: %f", t1 - t0);
}

void rotateWithAlpha(double angularSpeed, double alpha, bool clockwise){
  clockwise ? angularSpeed = -abs(angularSpeed) : angularSpeed = abs(angularSpeed);

  ros::Rate loop_rate(100);
  do{
    publishVelocity(0, angularSpeed);
    //ROS_INFO("Rotating: %f | %f", robot_pose2d.theta, alpha);
    ros::spinOnce();
    loop_rate.sleep();
  } while(abs(robot_pose2d.theta - alpha) > 0.03);

  publishVelocity(0, 0);
  ROS_INFO("Stop: %f | %f | %f", robot_pose2d.theta, alpha, abs(robot_pose2d.theta-alpha));
  //ROS_INFO("Initial angle: %f | Final angle: %f | Angle: %f", init_angle, robot_pose2d.theta, abs(robot_pose2d.theta - init_angle));
  //ROS_INFO("Time: %f", t1 - t0);
}

void alignTo(geometry_msgs::Point goal, double distanceTolerance){
  publishVelocity(0, 0);
  waitPose();

  double alpha = 0;
  double ang = 0;

  // Robot angle in degrees
  double robot_deg_angle = radians2degrees(robot_pose2d.theta);
  robot_deg_angle = round(robot_deg_angle, 1);

  //TODO: poner condicion de tolerancia al ingresar una posicion muy cercana al robot para evitar que gire.

  alpha = getAlpha(pose2DToPoint(robot_pose2d), goal);

  //ROS_INFO("Degress Angle: %f | Dir: %d", robot_deg_angle, robot_direction);

  if(robot_deg_angle < 0){
    robot_deg_angle = -1 * normDeg360(360 - robot_deg_angle);
  }

  if(robot_direction == 1){
    ang = alpha - robot_deg_angle;
  } else{
    ang = alpha + (robot_direction * robot_deg_angle);
  }

  //ROS_INFO("1)Alpha: %f | Angle: %f | D_Angle: %f", alpha, ang, robot_deg_angle);

  if(ang >= 0){
    ang <= 180 ? ang : ang = -360 + ang;
  } else{
    ang >= -180 ? ang : ang = 360 + ang;
  }

  //ROS_INFO("2)Alpha: %f | Angle: %f | D_Angle: %f", alpha, ang, robot_deg_angle);
  //ROS_INFO("Robot x: %f | y: %f | theta: %f | Dir: %d", robot_pose2d.x, robot_pose2d.y, robot_pose2d.theta, robot_direction);
  bool clockwise =((copysign(1, ang) == -1) ? true : false);

  ROS_INFO("Aligning to goal...");

  rotateWithAlpha(angularSpeed, round(degrees2radians(alpha), 4), clockwise);

  ROS_INFO("Robot is aligned!");
  //rotate(0.2, degrees2radians(abs(ang)), clockwise);

}

void chooseInitialBoundaryDirection(){
  // Choosing the initial boundary following direction based on orientation at the hit point
  if(laser[5] < laser[7] && laser[4] < laser[8]){
    isLeft = true;
    //ROS_INFO("Laser[5]: %f | Laser[7]: %f", laser[5], laser[7]);
    //ROS_INFO("Laser[4]: %f | Laser[8]: %f", laser[4], laser[8]);
  } else{
    isLeft = false;
    //ROS_INFO("Laser[5]: %f | Laser[7]: %f", laser[5], laser[7]);
    //ROS_INFO("Laser[4]: %f | Laser[8]: %f", laser[4], laser[8]);
  }

  if(laser[2] <= frontalMindist){
    isLeft = true;
    //ROS_INFO("Laser[2]: %f | Laser[10]: %f", laser[2], laser[10]);
  } else if(laser[10] <= frontalMindist){
    isLeft = false;
    //ROS_INFO("Laser[2]: %f | Laser[10]: %f", laser[2], laser[10]);
  }

  ROS_INFO("Chosing direction: %i", isLeft);
}

void checkLoopArroundObstacleByHitPoint(bool useChooseDirection){
  if((abs(robot_pose2d.x - pHitObstacle.x) < pointTolerance) &&
     (abs(robot_pose2d.y - pHitObstacle.y) < pointTolerance)){
    sameHitPoint++;

    // If sameNearestPoint is gratter than 1 then the goal is not reachable
    if(sameHitPoint > 1){
      isRounding = false;
      samePointCondition = true;
    }else{
      isRounding = true;
    }

    // If the robot arrives to the same point then it changes its rotating direction
    isLeft = useChooseDirection ? !isLeft : false;
    ROS_INFO("Same hit point: %i", sameNearestPoint);
  } else{
    if(currentDistance + pointTolerance < hitPointDistance){
      sameHitPoint = 0;
      isLeft = useChooseDirection ? isLeft : true;
      ROS_INFO("Check loop New hit point: %f", hitPointDistance);
    }
  }
}

void checkBestDistance(){
  currentDistance = getDistance(robot_pose2d.x, robot_pose2d.y, goal.x, goal.y);

  if(currentDistance + pointTolerance < bestDistance){
    bestDistance = currentDistance;
    pNearestGoal = pose2DToPoint(robot_pose2d);
    isNewNearestPoint = true;
    ROS_INFO("Nearest point: %f | %f", pNearestGoal.x, pNearestGoal.y);
  }
}

bool isObstacle(){

  if(laser[6] <= frontalMindist && laser[6] != 0){
    return true;
    ROS_INFO("Front: Obstacle detected!! D: %f", laser[6]);
  } else if((laser[4] <= frontalMindist && laser[4] != 0)  || (laser[5] <= frontalMindist && laser[5] != 0)){
    return true;
    ROS_INFO("Right Front: Obstacle detected!! D: %f | %f", laser[4], laser[5]);
  } else if((laser[7] <= frontalMindist && laser[7] != 0) || (laser[8] <= frontalMindist && laser[8] != 0)){
    return true;
    ROS_INFO("Left Front: Obstacle detected!! D: %f | %f", laser[7], laser[8]);
  } else if((laser[2] <= frontalMindist && laser[2] != 0)){
    return true;
    ROS_INFO("Side left: Obstacle detected!! D: %f", laser[2]);
  } else if((laser[10] <= frontalMindist && laser[10] != 0)){
    return true;
    ROS_INFO("Side right: Obstacle detected!! D: %f",laser[10]);
  }

  return false;

}

bool isOutOfHitPointRange(){
  if((abs(robot_pose2d.x - pHitObstacle.x) < pointTolerance) &&
     (abs(robot_pose2d.y - pHitObstacle.y) < pointTolerance)){
    //ROS_INFO("Robot is in the hit point range!");
    return false;
  }
  //ROS_INFO("Robot is out of range from the hit point!");
  return true;
}

void detectObstacle(bool useChooseDirection, bool checkLoop){

  if(isObstacle()){
    isRounding = true;

    if(useChooseDirection)
      chooseInitialBoundaryDirection();
    else
      isLeft = true;

    if(!isHitPointInitialized){
      pHitObstacle = pose2DToPoint(robot_pose2d);
      pNearestGoal = pose2DToPoint(robot_pose2d);
      isHitPointInitialized = true;

      currentDistance = getDistance(robot_pose2d.x, robot_pose2d.y, goal.x, goal.y);
      hitPointDistance = getDistance(pHitObstacle.x, pHitObstacle.y, goal.x, goal.y);

      ROS_INFO("Initializing hit point: %f", hitPointDistance);
    } else{

      /*if(checkLoop)
        checkLoopArroundObstacleByHitPoint(useChooseDirection);*/

      currentDistance = getDistance(robot_pose2d.x, robot_pose2d.y, goal.x, goal.y);

      if(currentDistance + pointTolerance < hitPointDistance){
        pHitObstacle = pose2DToPoint(robot_pose2d);
        pNearestGoal = pose2DToPoint(robot_pose2d);
        hitPointDistance = getDistance(pHitObstacle.x, pHitObstacle.y, goal.x, goal.y);
        ROS_INFO("New hit point: %f", hitPointDistance);
      }

    }

    publishVelocity(0, 0);
  }
}

void checkReversingCriterion(){
  // Local reversing criterion
  if(useReverseCriterion){
    if(!reversingCondition){
      if(!isRobotAlign(165)){
        isLeft = !isLeft;
        reversingCondition = true;
        ROS_INFO("Reversing criterion... | isLeft: %i", isLeft);
      }
    }
  }
}

bool loseObstacle(bool isSpinLeft){

  if(isSpinLeft){
    if(laser[1] > 1 && laser[2] > 1 && laser[3] > 1){
      ROS_INFO("Obstacle lost!!!");
      return true;
    }
  } else{
    if(laser[9] > 1 && laser[10] > 1 && laser[11] > 1){
      ROS_INFO("Obstacle lost!!!");
      return true;
    }
  }

  return false;
}

bool isLoopArroundObstacle(){

  if(isNewNearestPoint){
    sameNearestPointAux = 0;
    isNewNearestPoint = false;
  }
  else{
    if((abs(robot_pose2d.x - pNearestGoal.x) < pointTolerance) &&
       (abs(robot_pose2d.y - pNearestGoal.y) < pointTolerance)){

      if(!sameNearestPointLock){
        sameNearestPointLock = true;
        sameNearestPointAux++;
        ROS_INFO("Same nearest point: %i", sameNearestPointAux);

        //Goal not reachable
        if(sameNearestPointAux > 1){
          samePointCondition = true;
          return true;
        }
      }

    }else{
      sameNearestPointLock = false;
    }
  }

  return false;
}

bool bug2LeaveConditions(){
  bool ignore = true;
  // Way 1
  //if(isOnPendant(initial, goal, pose2DToPoint(robot_pose2d))){
  // Way 2
  if(isOnPendant()){

    // Checking if there is a free path to the goal
    if(isLaserRangeAlign(2,2,30) != -1){
      if(laser[2] > sideMinDist){
        ignore = false;
      }
    } else if(isLaserRangeAlign(10,10,30) != -1){
      if(laser[10] > sideMinDist){
        ignore = false;
      }
    }

    if(!ignore && currentDistance+pointTolerance < hitPointDistance){
      leavingCondition = true;
      ROS_INFO("CurrentDist: %f | HitPointDist: %f", currentDistance+pointTolerance, hitPointDistance);
      ROS_INFO("Leave point!!!");
    } else{
      leavingCondition = false;
      //ROS_INFO("Leave point ignored!");
    }
  } else{
    leavingCondition = false;
  }
}

bool distBugLeaveConditions(){
  float step = 2;
  int laserIndexAlign = -1;

  //Return the laser ray index that is align to goal
  // Laser index [3-9] | Tolerance: 1 degree
  laserIndexAlign = isLaserRangeAlign(3, 9, 1);

  if(laserIndexAlign != -1){
    // If the free distance from the current position to goal is gratter than rangeMax (4.5m) then set freeDistance = rangeMax
    freeDistance = laser[laserIndexAlign] < 4 ? laser[laserIndexAlign] : 4;

    if((currentDistance - freeDistance) <= 0){
      leavingCondition = true;
      ROS_INFO("Leaving condition 1...");
      ROS_INFO("Laser[%i]: %f | Cond1: %f | Cond2: %f ", laserIndexAlign, freeDistance, currentDistance - freeDistance, bestDistance - step);
      return true;
    }

    if(currentDistance - freeDistance <= bestDistance - step){
      leavingCondition = true;
      ROS_INFO("Leaving condition 2...");
      ROS_INFO("Laser[%i]: %f | Cond1: %f | Cond2: %f ", laserIndexAlign, freeDistance, currentDistance - freeDistance, bestDistance - step);
      return true;
    }

  }

  return false;
}

void bugAlgorithmsLeaveConditions(){
  switch (bugAlgorithm) {
  case Bug2:
    bug2LeaveConditions();
    break;
  case DistBug:
    distBugLeaveConditions();
    break;
  default:
    break;
  }
}

bool surroundObstacle(){

  int losingObstacleIteration = 0;
  int losingObstacleTolerance = 1200;

  ros::Rate loop_rate(100);

  if(isLeft){
    while(!flag && ((laser[1] != laser[3] && laser[1] >= sideMinDist+sideMinDistTolerance) || laser[0] >= sideMinDist+0.25)){
      // Turn left
      if(laser[2] <= sideMinDist && (laser[1] > 5 || laser[3] > 5)){
        flag = true;
      }

      if(losingObstacleIteration > losingObstacleTolerance){
        ROS_INFO("Obstacle lost!!!");
        return true;
      }
      losingObstacleIteration++;

      if(!leavingCondition){
        checkReversingCriterion();
        bugAlgorithmsLeaveConditions();
      }

      //ROS_INFO("Left Cond 1 - L0: %f ", laser[0]);
      //ROS_INFO("Left Cond 1| i: %i", losingObstacleIteration);
      publishVelocity(0, angularSpeed);
      ros::spinOnce();
      loop_rate.sleep();
    }

    flag = true;
    flag3 = true;
    losingObstacleIteration = 0;

    while(flag2 && laser[1] > sideMinDist && laser[3] > sideMinDist){
      // Turn right
      //ROS_INFO("Cond 2 - L0: %f | L1: %f | L2: %f | L3: %f", laser[0], laser[1], laser[2], laser[3]);
      //ROS_INFO("Left Cond 2 | i: %i", losingObstacleIteration);
      if(losingObstacleIteration > losingObstacleTolerance){
        ROS_INFO("Obstacle lost!!!");
        return true;
      }
      losingObstacleIteration++;

      if(!leavingCondition){
        checkReversingCriterion();
        bugAlgorithmsLeaveConditions();
      }

      publishVelocity(0, -angularSpeed);
      flag3 = false;
      ros::spinOnce();
      loop_rate.sleep();
    }

    losingObstacleIteration = 0;

    while(flag3 && (laser[1] <= sideMinDist-sideMinDistTolerance || laser[3] <= sideMinDist-sideMinDistTolerance)){
      // Turn left
      //ROS_INFO("Cond 3 - L0: %f | L1: %f | L2: %f | L3: %f", laser[0], laser[1], laser[2], laser[3]);
      //ROS_INFO("Left Cond 3| i: %i", losingObstacleIteration);
      if(losingObstacleIteration > losingObstacleTolerance){
        ROS_INFO("Obstacle lost!!!");
        return true;
      }
      losingObstacleIteration++;

      if(!leavingCondition){
        checkReversingCriterion();
        bugAlgorithmsLeaveConditions();
      }

      publishVelocity(0, angularSpeed);
      ros::spinOnce();
      loop_rate.sleep();
    }
  } else{
    while(!flag && ((laser[11] != laser[9] && laser[11] >= sideMinDist+sideMinDistTolerance) || laser[12] >= sideMinDist+0.25)){
      // Turn right
      if(laser[10] <= sideMinDist && (laser[11] > 5 || laser[9] > 5)){
        flag = true;
      }

      if(losingObstacleIteration > losingObstacleTolerance){
        ROS_INFO("Obstacle lost!!!");
        return true;
      }
      losingObstacleIteration++;

      if(!leavingCondition){
        checkReversingCriterion();
        bugAlgorithmsLeaveConditions();
      }

      //ROS_INFO("Right Cond 1 - L12: %f", laser[12]);
      //ROS_INFO("Right Cond 1| i: %i", losingObstacleIteration);
      publishVelocity(0, -angularSpeed);
      ros::spinOnce();
      loop_rate.sleep();
    }

    flag = true;
    flag3 = true;
    losingObstacleIteration = 0;

    while(flag2 && laser[11] > sideMinDist && laser[9] > sideMinDist){
      // Turn left
      //ROS_INFO("Cond 2 - L0: %f | L1: %f | L2: %f | L3: %f", laser[0], laser[1], laser[2], laser[3]);
      //ROS_INFO("Right Cond 2| i: %i", losingObstacleIteration);
      if(losingObstacleIteration > losingObstacleTolerance){
        ROS_INFO("Obstacle lost!!!");
        return true;
      }
      losingObstacleIteration++;

      if(!leavingCondition){
        checkReversingCriterion();
        bugAlgorithmsLeaveConditions();
      }

      publishVelocity(0, angularSpeed);
      flag3 = false;
      ros::spinOnce();
      loop_rate.sleep();
    }

    losingObstacleIteration = 0;

    while(flag3 && (laser[11] <= sideMinDist-sideMinDistTolerance || laser[9] <= sideMinDist-sideMinDistTolerance)){
      // Turn right
      //ROS_INFO("Cond 3 - L0: %f | L1: %f | L2: %f | L3: %f", laser[0], laser[1], laser[2], laser[3]);
      //ROS_INFO("Right Cond 3| i: %i", losingObstacleIteration);
      if(losingObstacleIteration > losingObstacleTolerance){
        ROS_INFO("Obstacle lost!!!");
        return true;
      }
      losingObstacleIteration++;

      if(!leavingCondition){
        checkReversingCriterion();
        bugAlgorithmsLeaveConditions();
      }

      publishVelocity(0, -angularSpeed);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  // Checking if there is not any obtacle in front the robot
  if(flag4 && laser[5] > frontalMindist && laser[6] > frontalMindist && laser[7] > frontalMindist
     && (laser[4] > frontalMindist+0.05 && laser[8] > frontalMindist+0.05)){

    if(!leavingCondition){
      checkReversingCriterion();
      bugAlgorithmsLeaveConditions();
    }

    publishVelocity(linearSpeed, 0);
    ros::Duration(0.15).sleep(); // sleep for half a second
    //ROS_INFO("Move");
    flag2 = true;
  } else{
    isLeft ? publishVelocity(0, angularSpeed) : publishVelocity(0 , -angularSpeed);

    // Disabling condition 2
    flag2 = false;

    //ROS_INFO("Rotate");

  }

  return false;
}

bool surroundObstacleWithBug1(){

  isRounding = false;
  bool isReturning = false;
  int returningCount = 0;

  isLeft = true;

  useReverseCriterion = false;

  // Boolean Flags Initialization
  flag = false;
  flag2 = false;
  flag3 = false;

  // Iterations
  int iteration = 0;

  ros::Rate loop_rate(100);

  detectObstacle(false, true);

  while(isRounding){

    checkBestDistance();

    // Surronding the obstacle until the obstacle is lost
    if(surroundObstacle()){
      isRounding = false;
      return true;
    }

    if(returningCount == 0 && isOutOfHitPointRange()){
      returningCount = 1;
    } else if(returningCount == 1 && isOutOfHitPointRange() == false){
      isReturning = true;
      ROS_INFO("Returning..");
    }

    if(isReturning){
      if(isLoopArroundObstacle()){
        return false;
      } else if((abs(robot_pose2d.x - pNearestGoal.x) < pointTolerance) &&
                (abs(robot_pose2d.y - pNearestGoal.y) < pointTolerance)){
        ROS_INFO("Rounding ended");

        return true;
      }
    }

    ros::spinOnce();
    loop_rate.sleep();

  }

  return false;
}

bool surroundObstacleWithBug2(){

  isRounding = false;
  isLeft = true;
  reversingCondition = false;

  bool checkSameHitPoint = false;

  // Flag
  flag = false;
  flag2 = false;
  flag3 = false;

  sameHitPoint = 0;

  ros::Rate loop_rate(100);

  // UseChooseDirection | CheckLoopByHitPoint
  detectObstacle(useChooseDirection, true);

  while(isRounding){
    checkBestDistance();

    // Surronding the obstacle until the obstacle is lost
    if(surroundObstacle()){
      isRounding = false;
      return true;
    }

    if(!checkSameHitPoint && isOutOfHitPointRange()){
      checkSameHitPoint = true;
    } else if(checkSameHitPoint && isOutOfHitPointRange() == false){
      if(useReverseCriterion && reversingCondition){
        ++sameHitPoint;
        ROS_INFO("Same hit point: %i", sameHitPoint);
        checkSameHitPoint = false;
        if(sameHitPoint == 2){
          samePointCondition = true;
          ROS_INFO("Same hit point condition");
          return false;
        }
      } else if(!useReverseCriterion){
        samePointCondition = true;
        ROS_INFO("Same hit point..");
        return false;
      }
    }

    // Waiting until the robot is out of the m-line when it hits the obstacle.
    if(checkSameHitPoint && leavingCondition){
      isRounding = false;
      ROS_INFO("Rounding ended");
      return true;
    } else{
      leavingCondition = false;
    }

    ros::spinOnce();
    loop_rate.sleep();

  }

  return false;
}

bool surroundObstacleWithDistBug(){
  isRounding = false;
  isLeft = false;

  // Flag
  flag = false;
  flag2 = false;
  flag3 = false;

  //Leave Conditions
  leavingCondition = false;
  reversingCondition = false;
  isNewNearestPoint = false;
  sameNearestPointLock = false;

  // Iterations
  int iteration = 0;

  ros::Rate loop_rate(100);

  // UseChooseDirection | CheckLoopByHitPoint
  detectObstacle(useChooseDirection, true);

  while(isRounding){

    checkBestDistance();

    // Surronding the obstacle until the obstacle is lost
    if(surroundObstacle()){
      isRounding = false;
      return true;
    }

    // If the reversing criterion is used, it is necessary to wait until the condition holds.
    if(useReverseCriterion && reversingCondition){
      if(isLoopArroundObstacle()){
        return false;
      }
    } else if(!useReverseCriterion){
      if(isLoopArroundObstacle()){
        return false;
      }
    }

    // Making time for avoiding the same obstacle
    if(leavingCondition){
      //ROS_INFO("Num: %i",iteration);
      iteration++;
    }
    // distBug leaving conditions have been moved to surroundObstacle function,
    // because it is necessary to check the leaving conditions at the same time
    // that the robot is executing the boundary following behavior
    /*else{
      distBugLeaveConditions();
    }*/

    if(iteration > 10){
      isRounding = false;
      ROS_INFO("Rounding ended");

      return true;
    }

    ros::spinOnce();
    loop_rate.sleep();

  }

  return false;
}

void moveToGoal(geometry_msgs::Point goal, double speed, double distanceTolerance){

  publishVelocity(0, 0);
  initial = pose2DToPoint(robot_pose2d);
  initial2GoalDistance  = getDistance(initial.x, initial.y, goal.x, goal.y);
  currentDistance = getDistance(robot_pose2d.x, robot_pose2d.y, goal.x, goal.y);

  double robotDegAngle = radians2degrees(robot_pose2d.theta);
  robotDegAngle = round(robotDegAngle, 1);

  double distanceAux = 99999;
  bool rotation = true;
  bool isReached = false;

  ros::Rate loop_rate(100);

  while(!isReached && !samePointCondition){

    currentDistance = getDistance(robot_pose2d.x, robot_pose2d.y, goal.x, goal.y);

    //ROS_INFO("Distance: %f | Robot: %f | %f", distance , robot_pose2d.x, robot_pose2d.y);

    if(currentDistance < distanceAux){
      distanceAux = currentDistance;
    } else if(currentDistance < distanceTolerance){
      isReached = true;
    } /*else if(distance >= distanceAux+distanceTolerance){
      isReached = true;
    }*/

    if(rotation){
      ros::Duration(0.5).sleep(); // sleep for half a second
      alignTo(goal, 0);
    }

    publishVelocity(speed, 0);

    switch (bugAlgorithm) {
    case Bug2:
      rotation = surroundObstacleWithBug2();
      break;
    case DistBug:
      rotation = surroundObstacleWithDistBug();
      break;
    default:
      rotation = surroundObstacleWithBug1();
      break;
    }

    //ROS_INFO("Rotate? %i", rotation);

    ros::spinOnce();
    loop_rate.sleep();
  }

  publishVelocity(0, 0);
  if(samePointCondition){
    ROS_INFO("Goal not reached! %f | same: %i", currentDistance, samePointCondition);
  } else{
    ROS_INFO("Goal reached! %f| same: %i", currentDistance, samePointCondition);
  }
}

int main(int argc, char **argv)
{
  string cmd_vel_topic, odom_topic, laser_topic, sonar_topic;
  ros::init(argc, argv, "bugAlgorithms");
  ros::NodeHandle nh;

  // Number of messages per second
  rate = 100;
  double initTime, endTime;
  ros::Rate rosRate(rate);

  cout << "Use GazeboSim(1) or Pioneer3dx(0)?" << endl;
  cin >> isSimulation;
  //  cout << "Use Laser? (0/1)" << endl;
  //  cin >> isUsingLaser;

  if(isSimulation){
    cout << "Using GazeboSim topics\n";
    cmd_vel_topic = "cmd_vel";
    odom_topic = "odom";
    laser_topic = "p3dx/laser/scan";
  } else{
    cout << "Using RosAria topics\n";
    cmd_vel_topic = "RosAria/cmd_vel";
    odom_topic = "RosAria/pose";
    sonar_topic = "RosAria/sonar";
    laser_topic = "scan";
  }

  pose_pub = nh.advertise<geometry_msgs::Pose2D>("pose2d", rate);
  laser_pose_pub = nh.advertise<geometry_msgs::Pose2D>("laser_pose_2d", rate);
  vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic, rate);

  odom_sub = nh.subscribe(odom_topic, rate, odomCallback);
  //laser_sub = nh.subscribe(laser_topic, rate, laserCallback);
  laser_sub = nh.subscribe(laser_topic, rate, laserCallback);
  //sonar_sub = nh.subscribe(sonar_topic, rate, sonarCallback);
  pose_sub = nh.subscribe("pose2d", rate, poseCallback);
  laser_pose_sub = nh.subscribe("laser_pose_2d", rate, laserPoseCallback);

  publishVelocity(0, 0);

  linearSpeed = 0.2;
  angularSpeed = 0.3;

  cout << "------------------------------\n";
  cout << "Bug Algorithms\n";
  cout << "------------------------------\n";
  cout << "(default) Bug1\n";
  cout << "(1) Bug2\n";
  cout << "(2) DistBug\n";
  cout << "(3) TangetBug\n";
  cout << "(4) IntensitiveBug\n";
  cout << "(5) PointBug\n";
  cout << "Option: ";
  cin >> bugAlgorithm;

  waitPose();
  waitLaser();

  cout << "Goal? \n";
  cout << "X: ";
  cin >> goal.x;
  cout << "Y: ";
  cin >> goal.y;

  initTime = waitClock().toSec();

  moveToGoal(goal, linearSpeed, 0.35);
  //alignTo(goal, 0.3);

  //  while (ros::ok() && !isAlign(goal)) {
  //    publishVelocity(0, 0.2);
  //  }

  publishVelocity(0, 0);

  endTime = ros::Time::now().toSec();

  ROS_INFO("Finished time: %f", endTime-initTime);

  /*while(ros::ok()){
    //isObstacle();
    ros::spinOnce();
    rosRate.sleep();
  }*/

  return 0;
}
