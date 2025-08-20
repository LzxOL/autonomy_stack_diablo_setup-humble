#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

#include "motion_msgs/msg/motion_ctrl.hpp"
#include "motion_msgs/msg/robot_status.hpp"
#include "motion_msgs/msg/leg_motors.hpp"
#include "ception_msgs/msg/imu_euler.hpp"

#include "tita_locomotion_interfaces/msg/locomotion_cmd.hpp" //2025.7.21新增头文件，用于修改类型冲突问题
#include "sensor_msgs/msg/imu.hpp"
using namespace std;

const double PI = 3.1415926;

double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.15;  // 增大方向差阈值到约23度，允许在较大方向偏差下也能开始移动 默认为0.4
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool manualMode = false;
bool autonomyMode = true;
double autonomySpeed = 0.5;  // 设置自主模式下的目标速度
double joyToSpeedDelay = 2.0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
float joyManualFwd = 0;
float joyManualYaw = 0;
int safetyStop = 0;
int slowDown = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = false;
int pathPointID = 0;
bool pathInit = false;
bool navFwd = true;
double switchTime = 0;
int vehicleReady = -1;
float bodyHeight = 1.0;
float bodyPitch = 0;

float bodyLow = 0.28;
float bodyHigh = 0.52;
float bodySpeedGain = 0.5;
float bodyMaxSpeed = 0.2;

float pitchRateGain = 2.0;
float pitchMaxRate = 0.5;

nav_msgs::msg::Path path;
rclcpp::Node::SharedPtr nh;

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x + cos(yaw) * sensorOffsetX - sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y + sin(yaw) * sensorOffsetX + cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * PI / 180.0 || fabs(pitch) > inclThre * PI / 180.0) && useInclToStop) {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * PI / 180.0) && useInclRateToSlow) {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  int pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (int i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joyTime = nh->now().seconds(); 
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);
  joySpeed = joySpeedRaw;
  if (joySpeed > 1.0) joySpeed = 1.0;
  if (joy->axes[4] == 0) joySpeed = 0;
  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  joyManualFwd = joy->axes[4];
  joyManualYaw = joy->axes[3];

  if (joy->axes[2] > -0.1) {
    autonomyMode = false;
  } else {
    autonomyMode = true;
  }

  if (joy->axes[5] > -0.1) {
    manualMode = false;
  } else {
    manualMode = true;
  }
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
{
  double speedTime = nh->now().seconds();
  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
{
  safetyStop = stop->data;
}

void slowDownHandler(const std_msgs::msg::Int8::ConstSharedPtr slow)
{
  slowDown = slow->data;
}

void robotStatusHandler(const tita_locomotion_interfaces::msg::LocomotionCmd::ConstSharedPtr status) 
{
  // if (status->robot_mode_msg == 3) vehicleReady = 1;
  // else vehicleReady = 0;
  if (status->fsm_mode == "transform_up") {
    vehicleReady = 1;  // 只有这两种模式下，底盘"就绪"
  } else {
      vehicleReady = 0;
  }
  
  // // 添加调试打印
  // RCLCPP_INFO(nh->get_logger(), "robotStatusHandler: fsm_mode = '%s', vehicleReady = %d", 
  //              status->fsm_mode.c_str(), vehicleReady);
  
  //printf ("Ctrl mode %d, Robot mode %d\n", status->ctrl_mode_msg, status->robot_mode_msg);
}

void motorStatusHandler(const motion_msgs::msg::LegMotors::ConstSharedPtr status)
{
  bodyHeight = (status->left_leg_length + status->right_leg_length - bodyLow) / (bodyHigh - bodyLow);
}

void imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  // 提取四元数
    tf2::Quaternion quat(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );
 
    // 转换为欧拉角（roll, pitch, yaw）
    tf2::Matrix3x3 mat(quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);  // 获取欧拉角（单位：弧度）
 
    // 如果需要角度制，可以转换（可选）
    // roll = roll * 180.0 / M_PI;
    // pitch = pitch * 180.0 / M_PI;
    // yaw = yaw * 180.0 / M_PI;
 
    // 赋值给全局变量或类成员变量
    bodyPitch = pitch; 
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pathFollower");

  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<int>("pubSkipNum", pubSkipNum);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("lookAheadDis", lookAheadDis);
  nh->declare_parameter<double>("yawRateGain", yawRateGain);
  nh->declare_parameter<double>("stopYawRateGain", stopYawRateGain);
  nh->declare_parameter<double>("maxYawRate", maxYawRate);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("maxAccel", maxAccel);
  nh->declare_parameter<double>("switchTimeThre", switchTimeThre);
  nh->declare_parameter<double>("dirDiffThre", dirDiffThre);
  nh->declare_parameter<double>("stopDisThre", stopDisThre);
  nh->declare_parameter<double>("slowDwnDisThre", slowDwnDisThre);
  nh->declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow);
  nh->declare_parameter<double>("inclRateThre", inclRateThre);
  nh->declare_parameter<double>("slowRate1", slowRate1);
  nh->declare_parameter<double>("slowRate2", slowRate2);
  nh->declare_parameter<double>("slowTime1", slowTime1);
  nh->declare_parameter<double>("slowTime2", slowTime2);
  nh->declare_parameter<bool>("useInclToStop", useInclToStop);
  nh->declare_parameter<double>("inclThre", inclThre);
  nh->declare_parameter<double>("stopTime", stopTime);
  nh->declare_parameter<bool>("noRotAtStop", noRotAtStop);
  nh->declare_parameter<bool>("noRotAtGoal", noRotAtGoal);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);

  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("pubSkipNum", pubSkipNum);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("yawRateGain", yawRateGain);
  nh->get_parameter("stopYawRateGain", stopYawRateGain);
  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("maxAccel", maxAccel);
  nh->get_parameter("switchTimeThre", switchTimeThre);
  nh->get_parameter("dirDiffThre", dirDiffThre);
  nh->get_parameter("stopDisThre", stopDisThre);
  nh->get_parameter("slowDwnDisThre", slowDwnDisThre);
  nh->get_parameter("useInclRateToSlow", useInclRateToSlow);
  nh->get_parameter("inclRateThre", inclRateThre);
  nh->get_parameter("slowRate1", slowRate1);
  nh->get_parameter("slowRate2", slowRate2);
  nh->get_parameter("slowTime1", slowTime1);
  nh->get_parameter("slowTime2", slowTime2);
  nh->get_parameter("useInclToStop", useInclToStop);
  nh->get_parameter("inclThre", inclThre);
  nh->get_parameter("stopTime", stopTime);
  nh->get_parameter("noRotAtStop", noRotAtStop);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);

  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/tita4264886/joy", 5, joystickHandler);

  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/tita4264886/robot_vel_32", 5, speedHandler);

  auto subStop = nh->create_subscription<std_msgs::msg::Int8>("/stop", 5, stopHandler);

  auto subSlowDown = nh->create_subscription<std_msgs::msg::Int8>("/slow_down", 5, slowDownHandler);

  // auto subRobotStatus = nh->create_subscription<motion_msgs::msg::RobotStatus>("/tita4264886/perception/detector/angle_data", 5, robotStatusHandler); //
  auto subRobotStatus = nh->create_subscription<tita_locomotion_interfaces::msg::LocomotionCmd>("/tita4264886/command/user/command", 5, robotStatusHandler); //

  auto subMotorStatus = nh->create_subscription<motion_msgs::msg::LegMotors>("/tita4264886/locomotion/motors_status", 5, motorStatusHandler);//

  //auto subIMU = nh->create_subscription<ception_msgs::msg::IMUEuler>("/tita4264886/imu_sensor_broadcaster/imu", 5, imuHandler);//
  auto subIMU = nh->create_subscription<sensor_msgs::msg::Imu>("/tita4264886/imu_sensor_broadcaster/imu", 5, imuHandler);

  // auto pubSpeed = nh->create_publisher<geometry_msgs::msg::TwistStampe>("/tita4264886/command/user/command", 5);
  // geometry_msgs::msg::TwistStamped cmd_vel;
  auto pubSpeed = nh->create_publisher<tita_locomotion_interfaces::msg::LocomotionCmd>("/tita4264886/command/user/command", 5);
  tita_locomotion_interfaces::msg::LocomotionCmd cmd_vel;
  cmd_vel.header.frame_id = "tita4264886/base_link";

  auto pubMotionCtrl = nh->create_publisher<motion_msgs::msg::MotionCtrl>("/tita4264886/command/active/command", 5);
  motion_msgs::msg::MotionCtrl ctrl_msg;

  auto pubClearing = nh->create_publisher<std_msgs::msg::Float32>("/map_clearing", 5);
  std_msgs::msg::Float32 clear_msg;
  clear_msg.data = 8.0;

  auto pubClearingExt = nh->create_publisher<std_msgs::msg::Float32>("/cloud_clearing", 5);
  std_msgs::msg::Float32 clear_msg_ext;
  clear_msg_ext.data = 30.0;

  ctrl_msg.value.forward = 0;
  ctrl_msg.value.left = 0;
  ctrl_msg.value.up = 0;
  ctrl_msg.value.roll = 0;
  ctrl_msg.value.pitch = 0;
  ctrl_msg.value.leg_split = 0;
  ctrl_msg.mode.pitch_ctrl_mode = true;
  ctrl_msg.mode.roll_ctrl_mode = false;
  ctrl_msg.mode.height_ctrl_mode = true;
  ctrl_msg.mode.stand_mode = true;
  ctrl_msg.mode.jump_mode = false;
  ctrl_msg.mode.split_mode = false;

  if (autonomyMode) { 
    joySpeed = autonomySpeed / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
  RCLCPP_INFO(nh->get_logger(), "autonomyMode = %d, joySpeed = %.3f, autonomySpeed = %.3f, maxSpeed = %.3f",
                   autonomyMode, joySpeed, autonomySpeed, maxSpeed);
  int ctrlInitFrameCount = 150;
  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (pathInit) {
      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec) 
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      int pathSize = path.poses.size();
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      // 添加调试输出：路径信息
      RCLCPP_INFO(nh->get_logger(), "路径信息: 点数=%d, 终点距离=%.3f", pathSize, endDis);

      float disX, disY, dis;
      while (pathPointID < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;
      if (dirDiff > PI) dirDiff -= 2 * PI;
      else if (dirDiff < -PI) dirDiff += 2 * PI;

      // // 添加调试输出：方向差和距离
      // RCLCPP_INFO(nh->get_logger(), "路径点信息: ID=%d, 距离=%.3f, 方向差=%.3f弧度(%.1f度), 阈值=%.3f弧度", 
      //            pathPointID, dis, dirDiff, dirDiff*180.0/PI, dirDiffThre);

      if (twoWayDrive) {
        double time = nh->now().seconds();
        if (fabs(dirDiff) > PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }
      joySpeed = 1;
      float joySpeed2 = maxSpeed * joySpeed; 
      // RCLCPP_INFO(nh->get_logger(), "joySpeed2 = %0.3f, maxSpeed = %.3f, joySpeed = %.3f", 
      //            joySpeed2, maxSpeed, joySpeed);
      if (!navFwd) {
        dirDiff += PI;
        if (dirDiff > PI) dirDiff -= 2 * PI;
        joySpeed2 *= -1;
      }

      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
      else vehicleYawRate = -yawRateGain * dirDiff;

      // 若角速度被限幅，打印一次提示
      float rawYawRate = vehicleYawRate;
      if (vehicleYawRate > maxYawRate * PI / 180.0) vehicleYawRate = maxYawRate * PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * PI / 180.0) vehicleYawRate = -maxYawRate * PI / 180.0;
      if (fabs(rawYawRate - vehicleYawRate) > 1e-6) {
        RCLCPP_DEBUG(nh->get_logger(), "角速度被限幅: raw=%.3f, clamped=%.3f(rad/s), maxYawRate=%.1f(deg/s)",
                     rawYawRate, vehicleYawRate, maxYawRate);
      }

      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }

      // 添加调试输出：路径点数判断
      if (pathSize <= 1) {
        // RCLCPP_INFO(nh->get_logger(), "速度设为0: 路径点数 (%d) <= 1", pathSize);
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) { 
        joySpeed2 *= endDis / slowDwnDisThre;
        // RCLCPP_INFO(nh->get_logger(), "减速: 终点距离(%.3f) / 减速阈值(%.3f) < joySpeed(%.3f), 结果joySpeed2=%.3f", 
        //            endDis, slowDwnDisThre, joySpeed, joySpeed2);
      }

      float joySpeed3 = joySpeed2;
      if (odomTime < slowInitTime + slowTime1 && slowInitTime > 0 || slowDown == 1) joySpeed3 *= slowRate1;
      else if (odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0 || slowDown == 2) joySpeed3 *= slowRate2;

      // RCLCPP_INFO(nh->get_logger(), "joySpeed3 = %0.3f, joySpeed2 = %.3f", 
      //            joySpeed3, joySpeed2);
      // 添加调试输出：方向差和停止距离判断
      RCLCPP_INFO(nh->get_logger(), "速度计算条件: |dirDiff|=%.3f < 阈值(%.3f)=%s, dis=%.3f > 停止阈值(%.3f)=%s", 
                 fabs(dirDiff), dirDiffThre, fabs(dirDiff) < dirDiffThre ? "true" : "false",
                 dis, stopDisThre, dis > stopDisThre ? "true" : "false");

      if (fabs(dirDiff) < dirDiffThre && dis > stopDisThre) {
        if (vehicleSpeed < joySpeed3) {
          vehicleSpeed += maxAccel / 100.0;  // 增加速度增量为原来的2倍，确保可以突破阈值
        }
        else if (vehicleSpeed > joySpeed3) {
          vehicleSpeed -= maxAccel / 100.0;
        }
      } else {
        if (vehicleSpeed > 0) {
          vehicleSpeed -= maxAccel / 100.0;
        }
        else if (vehicleSpeed < 0) {
          vehicleSpeed += maxAccel / 100.0;  // 增加速度增量，确保可以快速归零
        }
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1) {
        vehicleSpeed = 0;
      }
      if (safetyStop >= 2) vehicleYawRate = 0;

      // 诊断：原地打转或原地不动的原因输出
      bool cond_dir_ok = fabs(dirDiff) < dirDiffThre;
      bool cond_dis_ok = dis > stopDisThre;
      bool cond_path_ok = pathSize > 1;
      bool cond_freeze = (odomTime < stopInitTime + stopTime && stopInitTime > 0);
      bool cond_safety_stop = (safetyStop >= 1);
      bool in_place_turn = (fabs(vehicleSpeed) < 1e-4 && fabs(vehicleYawRate) > 1e-3);
      bool fully_stop = (fabs(vehicleSpeed) < 1e-4 && fabs(vehicleYawRate) < 1e-3);
      if (in_place_turn) {
        RCLCPP_WARN(nh->get_logger(),
          "原地转圈诊断: dirOK=%s (|dirDiff|=%.3f<thre=%.3f), disOK=%s (dis=%.3f>stopThre=%.3f), pathOK=%s (pathSize=%d), freeze=%s, safetyStop=%s",
          cond_dir_ok ? "true" : "false",
          fabs(dirDiff), dirDiffThre,
          cond_dis_ok ? "true" : "false",
          dis, stopDisThre,
          cond_path_ok ? "true" : "false", pathSize,
          cond_freeze ? "true" : "false",
          cond_safety_stop ? "true" : "false");
      } else if (fully_stop) {
        RCLCPP_WARN(nh->get_logger(),
          "原地不动诊断: 可能原因=[%s%s%s%s%s]",
          (!cond_path_ok ? "路径点过少; " : ""),
          (!cond_dis_ok ? "距离目标过近(停止阈值); " : ""),
          (!cond_dir_ok ? "方向偏差过大; " : ""),
          (cond_freeze ? "安全停止(倾斜/设定); " : ""),
          (cond_safety_stop ? "外部安全停止; " : ""));
      }

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        if (fabs(vehicleSpeed) <= maxAccel / 200.0) {  // 将阈值减半，确保速度可以超过零点判断阈值
          cmd_vel.twist.linear.x = 0;

        }
        else cmd_vel.twist.linear.x = vehicleSpeed;
        cmd_vel.twist.angular.z = vehicleYawRate;
        
        RCLCPP_INFO(nh->get_logger(), "发布速度命令: linear.x=%.5f, angular.z=%.5f", 
                   cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
        
        if (manualMode) {
          cmd_vel.twist.linear.x = maxSpeed * joyManualFwd;
          cmd_vel.twist.angular.z = maxYawRate * PI / 180.0 * joyManualYaw;
        }
        cmd_vel.fsm_mode = "transform_up";
        cmd_vel.pose.position.z = 0.1;
        pubSpeed->publish(cmd_vel);

        if (vehicleReady != 0) {
          ctrl_msg.value.forward = cmd_vel.twist.linear.x;
          ctrl_msg.value.left = cmd_vel.twist.angular.z;

          float desiredHeight = 1.0;
          if (slowDown == 1) desiredHeight = 0;
          else if (slowDown == 2) desiredHeight = 0.5;

          ctrl_msg.value.up = bodySpeedGain * (desiredHeight - bodyHeight);
          if (ctrl_msg.value.up < -bodyMaxSpeed) ctrl_msg.value.up = -bodyMaxSpeed;
          else if (ctrl_msg.value.up > bodyMaxSpeed) ctrl_msg.value.up = bodyMaxSpeed;

          ctrl_msg.value.pitch = -pitchRateGain * bodyPitch;
          if (ctrl_msg.value.pitch < -pitchMaxRate) ctrl_msg.value.pitch = -pitchMaxRate;
          else if (ctrl_msg.value.pitch > pitchMaxRate) ctrl_msg.value.pitch = pitchMaxRate;
        } else {
          ctrl_msg.value.forward = 0;
          ctrl_msg.value.left = 0;
          ctrl_msg.value.up = 0;
          ctrl_msg.value.pitch = 0;

          pubClearing->publish(clear_msg);
          pubClearingExt->publish(clear_msg_ext);
        }

        ctrl_msg.mode_mark = false;
        if (ctrlInitFrameCount > 0) {
          ctrl_msg.mode_mark = true;
          ctrlInitFrameCount--;
        }

        pubMotionCtrl->publish(ctrl_msg);

        pubSkipCount = pubSkipNum;
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
