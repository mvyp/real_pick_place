#include "torque_fun.h"
#include <closely_pick_place.h>
#include <ros/console.h>

#include <memory.h>
#include <tf_conversions/tf_eigen.h>

const double FINGER_MAX = 6400;
const double K_ = 0.08;
const double S_ = 1;

using namespace kinova;

tf::Quaternion EulerZYZ_to_Quaternion(double tz1, double ty, double tz2) {
  tf::Quaternion q;
  tf::Matrix3x3 rot;
  tf::Matrix3x3 rot_temp;
  rot.setIdentity();

  rot_temp.setEulerYPR(tz1, 0.0, 0.0);
  rot *= rot_temp;
  rot_temp.setEulerYPR(0.0, ty, 0.0);
  rot *= rot_temp;
  rot_temp.setEulerYPR(tz2, 0.0, 0.0);
  rot *= rot_temp;
  rot.getRotation(q);
  return q;
}
tf::Quaternion rot_to_Quaternion(geometry_msgs::Vector3 a,
                                 geometry_msgs::Vector3 b,
                                 geometry_msgs::Vector3 c) {
  tf::Quaternion q;
  tf::Matrix3x3 rot(a.x, b.x, c.x, a.y, b.y, c.y, a.z, b.z, c.z);

  rot.getRotation(q);
  return q;
}

PickPlace::PickPlace(ros::NodeHandle &nh) : nh_(nh) {
  //    if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
  //    ros::console::levels::Debug))
  //    {
  //        ros::console::notifyLoggerLevelsChanged();
  //    }

  ros::NodeHandle pn("~");

  nh_.param<std::string>("/robot_type", robot_type_, "j2s7s300");
  nh_.param<bool>("/robot_connected", robot_connected_, true);

  if (robot_connected_) {
    sub_joint_ = nh_.subscribe<sensor_msgs::JointState>(
        "/j2s7s300_driver/out/joint_state", 1, &PickPlace::get_current_state,
        this);

    sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>(
        "/" + robot_type_ + "_driver/out/tool_pose", 1,
        &PickPlace::get_current_pose, this);
  }
  sub_position_ = nh_.subscribe("/cloud/goal_pose", 1, &PickPlace::get_grasp_scope, this);
  // Before we can load the planner, we need two objects, a RobotModel and a
  // PlanningScene.
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model_ = robot_model_loader.getModel();

  // construct a `PlanningScene` that maintains the state of the world
  // (including the robot).
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  planning_scene_monitor_.reset(
      new planning_scene_monitor::PlanningSceneMonitor("robot_description"));

  //  every time need retrive current robot state, do the following.
  robot_state::RobotState &robot_state =
      planning_scene_->getCurrentStateNonConst();
  const robot_state::JointModelGroup *joint_model_group =
      robot_state.getJointModelGroup("arm");

  group_ = new moveit::planning_interface::MoveGroupInterface("arm");
  group_->setPlannerId("RRTstarkConfigDefault");
  gripper_group_ =
      new moveit::planning_interface::MoveGroupInterface("gripper");

  group_->setEndEffectorLink(robot_type_ + "_end_effector");

  finger_client_ =
      new actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction>(
          "/" + robot_type_ + "_driver/fingers_action/finger_positions", false);
  while (robot_connected_ &&
         !finger_client_->waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the finger action server to come up");
  }

  pub_co_ =
      nh_.advertise<moveit_msgs::CollisionObject>("/collision_object", 10);
  pub_aco_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>(
      "/attached_collision_object", 10);
  pub_planning_scene_diff_ =
      nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

  int arm_joint_num = robot_type_[3] - '0';
  joint_names_.resize(arm_joint_num);
  joint_values_.resize(joint_names_.size());
  for (uint i = 0; i < joint_names_.size(); i++) {
    joint_names_[i] =
        robot_type_ + "_joint_" + boost::lexical_cast<std::string>(i + 1);
  }

  // define start pose before grasp
  start_pose_.header.frame_id = "root";
  start_pose_.header.stamp = ros::Time::now();
  start_pose_.pose.position.x = 0.5;
  start_pose_.pose.position.y = -0.5;
  start_pose_.pose.position.z = 0.5;
  tf::Quaternion q;
  q = EulerZYZ_to_Quaternion(-M_PI / 4, M_PI / 2, M_PI);
  start_pose_.pose.orientation.x = q.x();
  start_pose_.pose.orientation.y = q.y();
  start_pose_.pose.orientation.z = q.z();
  start_pose_.pose.orientation.w = q.w();

  result_ = false;
  Flag = 0;
  
}

PickPlace::~PickPlace() {
  // shut down pub and subs
  sub_joint_.shutdown();
  sub_pose_.shutdown();
  pub_co_.shutdown();
  pub_aco_.shutdown();
  pub_planning_scene_diff_.shutdown();

  // release memory
  delete group_;
  delete gripper_group_;
  delete finger_client_;
}

void PickPlace::get_current_state(const sensor_msgs::JointStateConstPtr &msg) {
  boost::mutex::scoped_lock lock(mutex_state_);
  current_state_ = *msg;
}

void PickPlace::get_current_pose(
    const geometry_msgs::PoseStampedConstPtr &msg) {
  boost::mutex::scoped_lock lock(mutex_pose_);
  current_pose_ = *msg;
}

/**
 * @brief PickPlace::gripper_action
 * @param gripper_rad close for 6400 and open for 0.0
 * @return true is gripper motion reaches the goal
 */
bool PickPlace::gripper_action(double finger_turn) {
  if (robot_connected_ == false) {
    if (finger_turn > 0.5 * FINGER_MAX) {
      gripper_group_->setNamedTarget("Close");
    } else {
      gripper_group_->setNamedTarget("Open");
    }
    gripper_group_->move();
    return true;
  }

  if (finger_turn < 0) {
    finger_turn = 0.0;
  } else {
    finger_turn = std::min(finger_turn, FINGER_MAX);
  }

  kinova_msgs::SetFingersPositionGoal goal;
  goal.fingers.finger1 = finger_turn;
  goal.fingers.finger2 = goal.fingers.finger1;
  goal.fingers.finger3 = goal.fingers.finger1;
  finger_client_->sendGoal(goal);

  if (finger_client_->waitForResult(ros::Duration(5.0))) {
    finger_client_->getResult();
    return true;
  } else {
    finger_client_->cancelAllGoals();
    ROS_WARN_STREAM("The gripper action timed-out");
    return false;
  }
}

void PickPlace::clear_workscene() {
  // remove table
  co_.id = "table";
  co_.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co_.publish(co_);

  // // remove target
  // co_.id = "target_cylinder";
  // co_.operation = moveit_msgs::CollisionObject::REMOVE;
  // pub_co_.publish(co_);

  // // remove attached target
  // aco_.object.operation = moveit_msgs::CollisionObject::REMOVE;
  // pub_aco_.publish(aco_);

  planning_scene_msg_.world.collision_objects.clear();
  planning_scene_msg_.is_diff = true;
  pub_planning_scene_diff_.publish(planning_scene_msg_);

  clear_obstacle();
}

void PickPlace::build_workscene() {
  co_.header.frame_id = "root";
  co_.header.stamp = ros::Time::now();

  // remove table
  co_.id = "table";
  co_.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co_.publish(co_);

  // add table
  co_.primitives.resize(1);
  co_.primitive_poses.resize(1);
  co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<
                                      shape_msgs::SolidPrimitive::BOX>::value);
  co_.operation = moveit_msgs::CollisionObject::ADD;

  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 3;
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 3;
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.001;
  co_.primitive_poses[0].position.x = 0.0;
  co_.primitive_poses[0].position.y = 0.0;
  co_.primitive_poses[0].position.z = -0.41;
  pub_co_.publish(co_);
  planning_scene_msg_.world.collision_objects.push_back(co_);
  planning_scene_msg_.is_diff = true;
  pub_planning_scene_diff_.publish(planning_scene_msg_);
  ros::WallDuration(0.1).sleep();
}

void PickPlace::clear_obstacle() {
  // co_.id = "pole";
  // co_.operation = moveit_msgs::CollisionObject::REMOVE;
  // pub_co_.publish(co_);
  // planning_scene_msg_.world.collision_objects.push_back(co_);

  co_.id = "bot_obstacle";
  co_.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co_.publish(co_);
  planning_scene_msg_.world.collision_objects.push_back(co_);

  co_.id = "top_obstacle";
  co_.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co_.publish(co_);
  planning_scene_msg_.world.collision_objects.push_back(co_);

  co_.id = "table1";
  co_.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co_.publish(co_);
  planning_scene_msg_.world.collision_objects.push_back(co_);

  planning_scene_msg_.is_diff = true;
  pub_planning_scene_diff_.publish(planning_scene_msg_);
  ros::WallDuration(0.1).sleep();
  //      ROS_WARN_STREAM(__PRETTY_FUNCTION__ << ": LINE " << __LINE__ << ":
  //      remove pole "); std::cin >> pause_;
}

// void PickPlace::add_obstacle() {
//   clear_obstacle();

//   co_.id = "pole";
//   co_.primitives.resize(1);
//   co_.primitive_poses.resize(1);
//   co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//   co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<
//                                       shape_msgs::SolidPrimitive::BOX>::value);
//   co_.operation = moveit_msgs::CollisionObject::ADD;

//   co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.05;
//   co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05;
//   co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.5;

//   co_.primitive_poses[0].position.x = 0;
//   co_.primitive_poses[0].position.y = 0.2;
//   co_.primitive_poses[0].position.z = 0.75;

//   pub_co_.publish(co_);
//   planning_scene_msg_.world.collision_objects.push_back(co_);

//   planning_scene_msg_.is_diff = true;
//   pub_planning_scene_diff_.publish(planning_scene_msg_);
//   ros::WallDuration(0.1).sleep();
// }

void PickPlace::add_complex_obstacle() {
  clear_obstacle();

  // add obstacle between robot and object
  co_.id = "top_obstacle";
  co_.primitives.resize(1);
  co_.primitive_poses.resize(1);
  co_.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  co_.primitives[0].dimensions.resize(geometric_shapes::SolidPrimitiveDimCount<
                                      shape_msgs::SolidPrimitive::BOX>::value);
  co_.operation = moveit_msgs::CollisionObject::ADD;

  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.12;
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.09;
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.5;

  co_.primitive_poses[0].position.x = 0.0;
  co_.primitive_poses[0].position.y = 0.24;
  co_.primitive_poses[0].position.z = -0.03;

  pub_co_.publish(co_);
  planning_scene_msg_.world.collision_objects.push_back(co_);

  co_.id = "bot_obstacle";
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.45;
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.45;
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.41;
  co_.primitive_poses[0].position.x = 0;
  co_.primitive_poses[0].position.y = 0.0;
  co_.primitive_poses[0].position.z = -0.41 / 2 - 0.03;

  pub_co_.publish(co_);
  planning_scene_msg_.world.collision_objects.push_back(co_);

  co_.id = "table1";
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1;
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.6;
  co_.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.001;
  co_.primitive_poses[0].position.x = 0;
  co_.primitive_poses[0].position.y = -0.3-0.6/2;
  co_.primitive_poses[0].position.z = 0.36;

  pub_co_.publish(co_);
  planning_scene_msg_.world.collision_objects.push_back(co_);

  planning_scene_msg_.is_diff = true;
  pub_planning_scene_diff_.publish(planning_scene_msg_);
  ros::WallDuration(0.1).sleep();
}

void PickPlace::define_cartesian_pose(gpd_ros::GraspConfig pose0) {
  tf::Quaternion q;
  // define grasp pose
  pregrasp_pose_.header.frame_id = "root";
  pregrasp_pose_.header.stamp = ros::Time::now();
  pregrasp_pose_.pose.position = pose0.position;

  q = rot_to_Quaternion(pose0.approach, pose0.binormal, pose0.axis);
  pregrasp_pose_.pose.orientation.x = q.x();
  pregrasp_pose_.pose.orientation.y = q.y();
  pregrasp_pose_.pose.orientation.z = q.z();
  pregrasp_pose_.pose.orientation.w = q.w();

  grasp_pose_ = pregrasp_pose_;
  Eigen::Matrix4d Trans;
  Trans << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, K_, 0, 0, 0, 1;
  Eigen::Matrix4d Rot;
  Rot << pose0.approach.x, pose0.binormal.x, pose0.axis.x, pose0.position.x,
      pose0.approach.y, pose0.binormal.y, pose0.axis.y, pose0.position.y,
      pose0.approach.z, pose0.binormal.z, pose0.axis.z, pose0.position.z, 0, 0,
      0, 1;
  Eigen::Matrix4d Tr = Rot * Trans;
  grasp_pose_.pose.position.x = static_cast<double>(Tr(0, 3));
  grasp_pose_.pose.position.y = static_cast<double>(Tr(1, 3));
  grasp_pose_.pose.position.z = static_cast<double>(Tr(2, 3));
  // postgrasp_pose_ = grasp_pose_;
  // postgrasp_pose_.pose.position.z = grasp_pose_.pose.position.z + 0.05;
}

// int PickPlace::define_grasp_width(gpd_ros::GraspConfig msg) {
//   int score = msg.score.data;

//   return score;
// }

void PickPlace::setup_constrain(geometry_msgs::Pose target_pose,
                                bool orientation, bool position) {
  if ((!orientation) && (!position)) {
    ROS_WARN("Neither orientation nor position constrain applied.");
    return;
  }

  moveit_msgs::Constraints grasp_constrains;

  // setup constrains
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = robot_type_ + "_end_effector";
  ocm.header.frame_id = "root";
  ocm.orientation = target_pose.orientation;
  ocm.absolute_x_axis_tolerance = 2 * M_PI;
  ocm.absolute_y_axis_tolerance = 2 * M_PI;
  ocm.absolute_z_axis_tolerance = M_PI / 10;
  ocm.weight = 0.5;
  if (orientation) {
    grasp_constrains.orientation_constraints.push_back(ocm);
  }

  /* Define position constrain box based on current pose and target pose. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);

  // group_->getCurrentPose() does not work.
  //    current_pose_ = group_->getCurrentPose();
  geometry_msgs::Pose current_pose;
  { // scope for mutex update
    boost::mutex::scoped_lock lock_pose(mutex_pose_);
    current_pose = current_pose_.pose;

  }

  double constrain_box_scale = 2.0;
  primitive.dimensions[0] =
      constrain_box_scale *
      std::abs(target_pose.position.x - current_pose.position.x);
  primitive.dimensions[1] =
      constrain_box_scale *
      std::abs(target_pose.position.y - current_pose.position.y);
  primitive.dimensions[2] =
      constrain_box_scale *
      std::abs(target_pose.position.z - current_pose.position.z);

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  // place between start point and goal point.
  box_pose.position.x =
      (target_pose.position.x + current_pose.position.x) / 2.0;
  box_pose.position.y =
      (target_pose.position.y + current_pose.position.y) / 2.0;
  box_pose.position.z =
      (target_pose.position.z + current_pose.position.z) / 2.0;

  moveit_msgs::PositionConstraint pcm;
  pcm.link_name = robot_type_ + "_end_effector";
  pcm.header.frame_id = "root";
  pcm.constraint_region.primitives.push_back(primitive);
  pcm.constraint_region.primitive_poses.push_back(box_pose);
  pcm.weight = 0.5;
  if (position) {
    grasp_constrains.position_constraints.push_back(pcm);
  }

  group_->setPathConstraints(grasp_constrains);
}

void PickPlace::check_constrain() {
  moveit_msgs::Constraints grasp_constrains = group_->getPathConstraints();
  bool has_constrain = false;
  ROS_INFO("check constrain result: ");
  if (!grasp_constrains.orientation_constraints.empty()) {
    has_constrain = true;
    ROS_INFO("Has orientation constrain. ");
  }
  if (!grasp_constrains.position_constraints.empty()) {
    has_constrain = true;
    ROS_INFO("Has position constrain. ");
  }
  if (has_constrain == false) {
    ROS_INFO("No constrain. ");
  }
}

bool PickPlace::evaluate_plan(
    moveit::planning_interface::MoveGroupInterface &group, 
    const geometry_msgs::PoseStamped &target_pose){
  group_->setPoseTarget(target_pose);
  ROS_INFO("No. %d ", Flag);
  ROS_INFO("Setting plan time to 15 sec");
  group.setPlanningTime(15);
  result_ = (group.plan(first_plan) == moveit_msgs::MoveItErrorCodes::SUCCESS);
  if (result_ == false) {
    Flag++;
    return false;
  } else {
    ROS_INFO("No. %d SUCCESS", Flag);
    return true;
  }
}
bool PickPlace::evaluate_cartesian_plan(geometry_msgs::PoseStamped pre_grasp,
                                        geometry_msgs::PoseStamped fin_grasp) {
  // 获取当前位姿数据最为机械臂运动的起始位姿

  geometry_msgs::Pose pre_grasp_pose = pre_grasp.pose;
  geometry_msgs::Pose fin_grasp_pose = fin_grasp.pose;

  std::vector<geometry_msgs::Pose> waypoints;

  //将初始位姿加入路点列表
  waypoints.push_back(pre_grasp_pose);
  waypoints.push_back(fin_grasp_pose);

  // 笛卡尔空间下的路径规划
  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 2;
  const double eef_step = 0.001;
  double fraction = 0.0;
  int maxtries = 100; //最大尝试规划次数
  int attempts = 0;   //已经尝试规划次数

  while (fraction < 1.0 && attempts < maxtries) {
    fraction = group_->computeCartesianPath(waypoints, eef_step, jump_threshold,
                                            trajectory, true);
    attempts++;

    if (attempts % 10 == 0)
      ROS_INFO("Still trying after %d attempts...", attempts);
  }

  if (fraction > 0.8) {
    ROS_INFO("Path computed successfully. Moving the arm.");

    // 生成机械臂的运动规划数据
    second_plan.trajectory_ = trajectory;
    return true;
  } else {
    ROS_INFO("Path planning failed with only %0.6f success after %d attempts.",
             fraction, maxtries);

    Flag++;
    return false;
  }
}
gpd_ros::GraspConfig PickPlace::trans_grasp_pose(gpd_ros::GraspConfig pose1) {
  gpd_ros::GraspConfig pose2 = pose1;
  Eigen::Matrix4d Trans;
  Trans <<   0, 0, 1, 0, 
             1, 0, 0, 0, 
             0, 1, 0, 0,
             0, 0, 0, 1;
  Eigen::Matrix4d Rot1;
  Rot1 << pose2.approach.x, pose2.binormal.x, pose2.axis.x, pose2.position.x,
      pose2.approach.y, pose2.binormal.y, pose2.axis.y, pose2.position.y,
      pose2.approach.z, pose2.binormal.z, pose2.axis.z, pose2.position.z, 0, 0,
      0, 1;
  Eigen::Matrix4d Tr1 = Rot1 * Trans;
  pose2.approach.x = static_cast<double>(Tr1(0, 0));
  pose2.approach.y = static_cast<double>(Tr1(1, 0));
  pose2.approach.z = static_cast<double>(Tr1(2, 0));
  pose2.binormal.x = static_cast<double>(Tr1(0, 1));
  pose2.binormal.y = static_cast<double>(Tr1(1, 1));
  pose2.binormal.z = static_cast<double>(Tr1(2, 1));
  pose2.axis.x = static_cast<double>(Tr1(0, 2));
  pose2.axis.y = static_cast<double>(Tr1(1, 2));
  pose2.axis.z = static_cast<double>(Tr1(2, 2));
  transform1.setOrigin( tf::Vector3(pose1.position.x, pose1.position.y,pose1.position.z) );
  tf::Quaternion q = rot_to_Quaternion(pose1.approach,pose1.binormal,pose1.axis);
  transform1.setRotation(q);

  transform2.setOrigin( tf::Vector3(pose2.position.x, pose2.position.y,pose2.position.z) );
  q = rot_to_Quaternion(pose2.approach,pose2.binormal,pose2.axis);
  transform2.setRotation(q);
  int k=0;
  while(k<5)
  {
      br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", "grasp_pose1"));
      br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "world", "grasp_pose2"));
      k++;
      sleep(1);
      std::cout << "show pose" <<std::endl;
  }
  return pose2;
}

void PickPlace::get_grasp_scope(const gpd_ros::GraspConfig &mygoal) {
  boost::mutex::scoped_lock lock(mutex_grasp_pose_);
  grasp_scope.push_back(mygoal.position.x);
  grasp_scope.push_back(mygoal.position.y);
  grasp_scope.push_back(mygoal.position.z);
}

void PickPlace::my_pick(const gpd_ros::GraspConfigList &mygoalList) {

  // 读取力矩
  // std::shared_ptr<std::vector<float>> torque_vector_ptr(
  //     new std::vector<float>(7));
  // auto torque_list = torque_value(torque_vector_ptr);
  // ROS_INFO_STREAM((*torque_list)[6]);
  // 加入工作环境
  build_workscene();
  ros::WallDuration(0.5).sleep();
  add_complex_obstacle();
  ROS_INFO_STREAM("Add obstacle");
  // group_->setNamedTarget("Wait");
  // group_->move();
  // ros::WallDuration(1).sleep();
  group_->setNamedTarget("Home");
  group_->move();
  ROS_INFO_STREAM("STAR ...");
  ROS_INFO_STREAM("*************************");
  ROS_INFO_STREAM("*************************");
  ROS_INFO_STREAM("*************************");
  //寻找合适的
  bool f_first = 0;
  bool f_second = 1;

  for (int i = 0; i < mygoalList.grasps.size(); i++) {

    {
      ROS_INFO_STREAM("try");
      if (sqrt(pow(grasp_scope[0]-mygoalList.grasps[i].position.x, 2) + pow(grasp_scope[1]-mygoalList.grasps[i].position.y, 2) +
               pow(grasp_scope[2]-mygoalList.grasps[i].position.z, 2)) < S_) {
        mygoal = trans_grasp_pose(mygoalList.grasps[i]);
        define_cartesian_pose(mygoal);
        if ((f_first = evaluate_plan(*group_, pregrasp_pose_)) 
            // &&
            // (f_second = evaluate_cartesian_plan(pregrasp_pose_, grasp_pose_))
            ) {
          break;
        }
      }
    }
  }
  if (f_first && f_second) {
    ROS_INFO_STREAM("Go to pre-grasp position ...");

    group_->execute(first_plan);

    gripper_action(0.0);
    ros::WallDuration(0.5).sleep();
    ROS_INFO_STREAM("Go to grasp position ...");
    geometry_msgs::PoseStamped copy_pose;
    { // scope for mutex update
      boost::mutex::scoped_lock lock_state(mutex_pose_);
      copy_pose = current_pose_;
    }
    // setup_constrain(grasp_pose_.pose, true, true);
    // group_->setStartStateToCurrentState();
    // evaluate_cartesian_plan(copy_pose, grasp_pose_);
    evaluate_plan(*group_, grasp_pose_);
    group_->execute(first_plan);
    } 
  else {
    ROS_INFO_STREAM("Fail to pre-grasp ...");
    ROS_INFO_STREAM("try to grasp ...");
  }
  ROS_INFO_STREAM("Grasping ...");
  gripper_action(FINGER_MAX);
  ros::WallDuration(2).sleep();
  ROS_INFO_STREAM("Planning to return to Home  ...");

  group_->setNamedTarget("Home");
  group_->move();

  // group_->setPoseTarget(start_pose_);
  // evaluate_plan(*group_);
  ROS_INFO_STREAM("Releasing gripper ...");

  gripper_action(0.0); // full open

  clear_workscene();
  Flag = 0;
  return;
}


void PickPlace::getInvK(geometry_msgs::Pose &eef_pose,
                        std::vector<double> &joint_value) {
  // TODO: transform cartesian command to joint space, and alway motion plan
  // in joint space.
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pick_place_demo");
  ros::NodeHandle node;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  kinova::PickPlace pick_place(node);

  ros::Subscriber sub =
      node.subscribe("/detect_grasps/clustered_grasps", 10,
                     &kinova::PickPlace::my_pick, &pick_place);
  ros::MultiThreadedSpinner();
  ros::waitForShutdown();
  return 0;
}