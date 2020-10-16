// Used to generate an articular trajectory (i.e. position of each joint for every time step)
// from a mathematically defined Cartesian space trajectory (x, y, z).
// Terminal 1) launch gazebo: roslaunch baxter_gazebo baxter_world.launch
// Terminal 2) enable the robot: rosrun baxtetools enable_robot.py -e
//             launch the trajcetory action server: rosrun baxteinterface joint_trajectory_action_server.py
// Terminal 3) launch moveit: roslaunch baxter_moveit_config baxter_grippers.launch
// Terminal 4) launch "moveit.launch": roslaunch edlut_ros moveit.launch
//             (ROS .launch file launching this node and remaping robot joint states:
              // <launch>
              //   <node pkg="edlut_ros" name="moveit" type="moveit" output="screen">
              //      <remap from="joint_states" to="robot/joint_states" />
              //   </node>
              //
              // </launch>


#define _USE_MATH_DEFINES

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <baxter_core_msgs/JointCommand.h>

class MakeCircle{
private:
  int samples;
  std::string mode;

  // Coordinates of the first point of the trajectory
  double start_x;
  double start_y;
  double start_z;

  double radius;

  int num_circles;

public:
  // Cartesian Coordinates of the trajectory
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> z;

  // Joint angular coordinates of the trajectory
  std::vector<double> joint_0, joint_1, joint_2, joint_3, joint_4, joint_5, joint_6;

  MakeCircle(int samples, double radius, double start_x, double start_y, double start_z, std::string mode, int num_circles):
            samples(samples), radius(radius), start_x(start_x), start_y(start_y), start_z(start_z), mode(mode), num_circles(num_circles){
    this->x = std::vector<double> (samples*this->num_circles);
    this->y = std::vector<double> (samples*this->num_circles);
    this->z = std::vector<double> (samples*this->num_circles);
  }


  void CirclePoints(){
    if (this->mode == "horizontal"){
      this->HorizontalCircle();
    }
    else if (this->mode == "vertical"){
      this->VerticalCircle();
    }
    else if (this->mode == "lissajou"){
      this->LissajouCurve();
    }
    else if (this->mode == "eight"){
      this->EightTrajectory();
    }
    else if (this->mode == "inclined"){
      this->InclinedCircle();
    }
  }

  void VerticalCircle(){
    double dt = 2*M_PI / this->samples;
    for (int i=0; i<this->samples * this->num_circles; i++){
      this->x[i] = this->start_x;
      this->y[i] = this->radius * sin(M_PI + (i*dt));
      this->z[i] = this->radius * cos(M_PI + (i*dt));
    }
  }

  void HorizontalCircle(){
    double dt = 2*M_PI / this->samples;
    for (int i=0; i<this->samples * this->num_circles; i++){
      this->x[i] = this->radius * cos(M_PI + (i*dt));
      this->y[i] = this->radius * sin(M_PI + (i*dt));
      this->z[i] = this->start_z;
    }
  }

  void EightTrajectory(){
    double dt = 2*M_PI / this->samples;
    for (int i=0; i<this->samples * this->num_circles; i++){
      this->x[i] = this->radius * sin(2* (M_PI + (i*dt)))/2.0;
      this->y[i] = this->radius * cos(M_PI + (i*dt));
      this->z[i] = this->start_z;
    }
  }

  void LissajouCurve(){
    double dt = 2*M_PI / this->samples;
    for (int i=0; i<this->samples * this->num_circles; i++){
      this->x[i] = this->radius * cos(M_PI + (i*dt));
      this->y[i] = this->radius * sin(M_PI + (i*dt) - M_PI/4.0);
      this->z[i] = this->start_z;
    }
  }

  void InclinedCircle(){
    double dt = 2*M_PI / this->samples;
    for (int i=0; i<this->samples * this->num_circles; i++){
      this->x[i] = this->radius * cos(M_PI + (i*dt)) * cos(M_PI/6.0);
      this->y[i] = this->radius * sin(M_PI + (i*dt));
      this->z[i] = this->radius * cos(M_PI + (i*dt)) * sin(M_PI/6.0);
      // this->z[i] = std::sqrt(1/(4*M_PI) - std::pow(this->x[i],2) - std::pow(this->y[i],2));
    }
  }
};

int main(int argc, char** argv)
{
  //--------------- Setup ---------------
  //
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  static const std::string PLANNING_GROUP = "left_arm";
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  //
  //--------------- Setup ---------------


  //--------------- Visualization ---------------
  //
  // The package MoveItVisualTools provides many capabilties for visualizing objects, robots,
  // and trajectories in RViz as well as debugging tools such as step-by-step introspection of a script
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("world");
  visual_tools.deleteAllMarkers();
  // Remote control is an introspection tool that allows users to step through a high level script
  // via buttons and keyboard shortcuts in RViz
  visual_tools.loadRemoteControl();
  // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
  // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  visual_tools.trigger();
  //
  //--------------- Visualization ---------------



  //--------------- Circle parameters ---------------
  //
  int samples = 1000;
  double radius = 0.12;
  std::string mode = "inclined";
  double trajectory_period = 2.0;
  double dt = trajectory_period / samples;
  int num_circles = 2;
  //
  //--------------- Circle parameters ---------------


  //--------------- Joint constraints ---------------
  if (mode == "horizontaAAAl"){
    moveit_msgs::JointConstraint constraint_e0;
    constraint_e0.joint_name = "left_e0";
    constraint_e0.position = 0.0;
    constraint_e0.tolerance_above = 0.1;
    constraint_e0.tolerance_below = 0.1;
    constraint_e0.weight = 1.0;

    moveit_msgs::JointConstraint constraint_w0;
    constraint_w0.joint_name = "left_w0";
    constraint_w0.position = 0.0;
    constraint_w0.tolerance_above = 0.4;
    constraint_w0.tolerance_below = 0.4;
    constraint_w0.weight = 0.8;

    moveit_msgs::JointConstraint constraint_w2;
    constraint_w2.joint_name = "left_w2";
    constraint_w2.position = 0.0;
    constraint_w2.tolerance_above = 0.5;
    constraint_w2.tolerance_below = 0.5;
    constraint_w2.weight = 0.5;

    moveit_msgs::Constraints test_constraints;
    test_constraints.joint_constraints.push_back(constraint_e0);
    test_constraints.joint_constraints.push_back(constraint_w0);
    test_constraints.joint_constraints.push_back(constraint_w2);
    move_group.setPathConstraints(test_constraints);
  }
  //--------------- Joint constraints ---------------

  //--------------- Move to Start Position ---------------
  //
  geometry_msgs::PoseStamped current_pose;
  current_pose = move_group.getCurrentPose("left_gripper");

  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  geometry_msgs::Pose init_pose_1;

  if (mode == "horizontal" or mode =="eight" or mode == "lissajou" or mode == "inclined"){
    // start point tested for radius from 0.04 to 0.12
    // joint_group_positions[0] = -0.144;
    // joint_group_positions[1] = -1.066;
    // joint_group_positions[2] = 0.026;
    // joint_group_positions[3] = 1.879;
    // joint_group_positions[4] = -0.020;
    // joint_group_positions[5] = 0.738;
    // joint_group_positions[6] = 0.021;

    joint_group_positions[0] = -0.75;
    joint_group_positions[1] = -0.35;
    joint_group_positions[2] = -0.0;
    joint_group_positions[3] = 1.35;
    joint_group_positions[4] = 0.0;
    joint_group_positions[5] = 0.57;
    joint_group_positions[6] = 0.0;

    // init_pose_1.position.x = 0.9266079492236827;
    // init_pose_1.position.y = 0.2895493844652417;
    // init_pose_1.position.z = 0.07755809783756715;
    // init_pose_1.orientation.x = -0.012690455069238814;
    // init_pose_1.orientation.y = 0.7175208039102806;
    // init_pose_1.orientation.z = 0.0;
    // init_pose_1.orientation.w = 0.6963119357655239;

  }
  else if (mode == "vertical"){
    //molinillo
    joint_group_positions[0] = -0.75;
    joint_group_positions[1] = -0.35;
    joint_group_positions[2] = -0.0;
    joint_group_positions[3] = 1.35;
    joint_group_positions[4] = 0.0;
    joint_group_positions[5] = -1.0;
    joint_group_positions[6] = 0.0;

    // init_pose_1.position.x = 0.9266079492236827;
    // init_pose_1.position.y = 0.2895493844652417;
    // init_pose_1.position.z = 0.07755809783756715;
    // init_pose_1.orientation.x = -0.012690455069238814;
    // init_pose_1.orientation.y = 0.7175208039102806;
    // init_pose_1.orientation.z = 0.0;
    // init_pose_1.orientation.w = 0.6963119357655239;

    //circle
    // joint_group_positions[0] = -0.75;
    // joint_group_positions[1] = 0.0;
    // joint_group_positions[2] = -0.0;
    // joint_group_positions[3] = 1.0;
    // joint_group_positions[4] = 1.57;
    // joint_group_positions[5] = -1.57;
    // joint_group_positions[6] = -0.57;

    // joint_group_positions[0] = 0.2;
    // joint_group_positions[1] = 0.0;
    // joint_group_positions[2] = -1.57;
    // joint_group_positions[3] = 1.7;
    // joint_group_positions[4] = 0.0;
    // joint_group_positions[5] = 0.8;
    // joint_group_positions[6] = 0.0;

    // joint_group_positions[0] = 0.2;
    // joint_group_positions[1] = 0.0;
    // joint_group_positions[2] = -1.57;
    // joint_group_positions[3] = 1.4;
    // joint_group_positions[4] = 0.0;
    // joint_group_positions[5] = 1.15;
    // joint_group_positions[6] = 0.0;
  }
  // move_group.setPoseTarget(init_pose_1);

  move_group.setJointValueTarget(joint_group_positions);

  // geometry_msgs::Pose target_pose1;
  // target_pose1 = current_pose.pose;
  // target_pose1.position.x = 0.52;
  // target_pose1.position.y = 0.58;
  // target_pose1.position.z = 0.0;
  // move_group.setPoseTarget(target_pose1);

  move_group.move();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  current_pose = move_group.getCurrentPose("left_gripper");


  double start_x, start_y, start_z;

  if (mode == "horizontal" or mode == "lissajou" or mode == "inclined"){
    start_x = current_pose.pose.position.x - radius;
    start_y = current_pose.pose.position.y;
    start_z = current_pose.pose.position.z;
  }
  else if (mode == "vertical"){
    start_x = current_pose.pose.position.x;
    start_y = current_pose.pose.position.y;
    start_z = current_pose.pose.position.z - radius;
  }
  else if (mode == "eight"){
    start_x = current_pose.pose.position.x;
    start_y = current_pose.pose.position.y + radius;
    start_z = current_pose.pose.position.z;
  }


  geometry_msgs::Pose init_circle_pose;
  init_circle_pose = current_pose.pose;
  init_circle_pose.position.x = start_x;
  init_circle_pose.position.y = start_y;
  init_circle_pose.position.z = start_z;

  move_group.setPoseTarget(init_circle_pose);
  move_group.move();

  ROS_INFO_NAMED("tutorial", "Moving to starting pose %s", success ? "" : "FAILED");
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  // TO open the RvizVisualToolsGui: In Moveit GUI, go to Panels > Add new panel > RvizVisualToolsGui

  //
  //--------------- Move to Start Position ---------------

  //--------------- Create Circle ---------------
  //
  MakeCircle make_circle = MakeCircle(samples, radius, start_x, start_y, start_z, mode, num_circles);
  make_circle.CirclePoints();
  //
  //--------------- Create Circle ---------------

  //move_group.clearPathConstraints();
  //--------------- Trajectory planning ---------------
  //
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose target_pose = current_pose.pose;
  geometry_msgs::Pose init_pose = current_pose.pose;

  std::ofstream file_xyz;
  file_xyz.open("/home/baxter/catkin_ws/moveit_trajectory_inclined_xyz.txt");

  if (mode=="horizontal" or mode == "eight" or mode == "lissajou"){
    for (int i=0; i<samples * num_circles; i++){
      target_pose.position.x = init_pose.position.x + make_circle.x[i];
      target_pose.position.y = init_pose.position.y + make_circle.y[i];
      waypoints.push_back(target_pose);
      // Write to file ideal circle coordinates
      if (i<=samples){
        file_xyz << target_pose.position.x << " " << target_pose.position.y << " " << target_pose.position.z << std::endl;
      }
    }
  }
  else if (mode == "vertical"){
    for (int i=0; i<samples * num_circles; i++){
      target_pose.position.z = init_pose.position.z + make_circle.z[i];
      target_pose.position.y = init_pose.position.y + make_circle.y[i];
      waypoints.push_back(target_pose);
      // Write to file ideal circle coordinates
      if (i<=samples){
        file_xyz << target_pose.position.x << " " << target_pose.position.y << " " << target_pose.position.z << std::endl;
      }
    }
  }
  else if (mode == "inclined"){
    for (int i=0; i<samples * num_circles; i++){
      target_pose.position.x = init_pose.position.x + make_circle.x[i];
      target_pose.position.y = init_pose.position.y + make_circle.y[i];
      target_pose.position.z = init_pose.position.z + make_circle.z[i];
      waypoints.push_back(target_pose);
      // Write to file ideal circle coordinates
      if (i<=samples){
        file_xyz << target_pose.position.x << " " << target_pose.position.y << " " << target_pose.position.z << std::endl;
      }
    }
  }

  file_xyz.close();

  move_group.setMaxVelocityScalingFactor(1.0);
  move_group.setGoalTolerance(0.001);

  moveit_msgs::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 1.0;
  double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, move_group.getPathConstraints());
  ROS_INFO("MOVE GROUP WAYPOINTS");
  for (int i=0; i<trajectory.joint_trajectory.points.size(); i++){
    trajectory.joint_trajectory.points[i].time_from_start = ros::Duration (i*dt);
    // std::cout << i <<" : " << trajectory.joint_trajectory.points[i].time_from_start << std::endl;
  }
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  my_plan2.trajectory_ = trajectory;
  move_group.execute(my_plan2);
  ROS_INFO("Cartesian path (%.2f%% achieved)", fraction * 100.0);

  //
  //--------------- Trajectory planning ---------------

  //--------------- Write Trajectory to file  ---------------
  //
  std::ofstream file;
  file.open("/home/baxter/catkin_ws/moveit_trajectory_inclined.txt");
  for (int i=0; i<trajectory.joint_trajectory.points.size(); i++){
    file << trajectory.joint_trajectory.points[i].positions[0] << " " << trajectory.joint_trajectory.points[i].positions[1] << " " << trajectory.joint_trajectory.points[i].positions[2] << " "
    << trajectory.joint_trajectory.points[i].positions[3] << " " << trajectory.joint_trajectory.points[i].positions[4] << " " << trajectory.joint_trajectory.points[i].positions[5] << " "
    << trajectory.joint_trajectory.points[i].positions[6] << std::endl ;
  }

  // for (int i=0; i<trajectory.joint_trajectory.points.size(); i++){
  //   file << trajectory.joint_trajectory.points[i].positions[0] << " " << trajectory.joint_trajectory.points[i].positions[1] << " " << trajectory.joint_trajectory.points[i].positions[2] << " "
  //   << trajectory.joint_trajectory.points[i].positions[3] << " " << trajectory.joint_trajectory.points[i].positions[4] << " " << trajectory.joint_trajectory.points[i].positions[5] << " "
  //   << trajectory.joint_trajectory.points[i].positions[6] << std::endl ;
  // }
  file.close();
  //
  //--------------- Write Trajectory to file  ---------------



  ros::shutdown();
  return 0;
}
