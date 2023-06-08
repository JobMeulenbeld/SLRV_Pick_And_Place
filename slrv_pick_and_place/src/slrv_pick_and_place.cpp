/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors original code: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */
/* Author adjusted code: Job Meulenbeld*/

#include "slrv_pick_and_place/slrv_pick_and_place.h"

slrvPickAndPlace::slrvPickAndPlace() : 
  node_handle_(""),
  priv_node_handle_("~"),
  mode_state_(0),
  demo_count_(0),
  machine_state_(0),
  machine_begin(false)
{
  present_joint_angle_.resize(NUM_OF_JOINT_AND_TOOL, 0.0);
  present_kinematic_position_.resize(3, 0.0);

  joint_name_.push_back("joint1");
  joint_name_.push_back("joint2");
  joint_name_.push_back("joint3");
  joint_name_.push_back("joint4");

  initServiceClient();
  initPubSub();
}

slrvPickAndPlace::~slrvPickAndPlace()
{
  if (ros::isStarted()) 
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void slrvPickAndPlace::initServiceClient()
{
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
  goal_task_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path");
}

void slrvPickAndPlace::initPubSub()
{
  slrv_status_publisher_ = node_handle_.advertise<slrv_pick_and_place::Status>("/SLRV_Status", 100);

  open_manipulator_states_sub_ = node_handle_.subscribe("states", 10, &slrvPickAndPlace::manipulatorStatesCallback, this);
  open_manipulator_joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &slrvPickAndPlace::jointStatesCallback, this);
  open_manipulator_kinematics_pose_sub_ = node_handle_.subscribe("gripper/kinematics_pose", 10, &slrvPickAndPlace::kinematicsPoseCallback, this);
  ar_pose_marker_sub_ = node_handle_.subscribe("/ar_pose_marker", 10, &slrvPickAndPlace::arPoseMarkerCallback, this);
  slrv_id_listener_ = node_handle_.subscribe("/Pick_And_Place", 10, &slrvPickAndPlace::slrvPickAndPlaceCallback, this);
}

bool slrvPickAndPlace::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if (goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool slrvPickAndPlace::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back("gripper");
  srv.request.joint_position.position = joint_angle;

  if (goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool slrvPickAndPlace::setTaskSpacePath(std::vector<double> kinematics_pose,std::vector<double> kienmatics_orientation, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

  srv.request.kinematics_pose.pose.orientation.w = kienmatics_orientation.at(0);
  srv.request.kinematics_pose.pose.orientation.x = kienmatics_orientation.at(1);
  srv.request.kinematics_pose.pose.orientation.y = kienmatics_orientation.at(2);
  srv.request.kinematics_pose.pose.orientation.z = kienmatics_orientation.at(3);

  srv.request.path_time = path_time;

  if (goal_task_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void slrvPickAndPlace::manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
{
  if (msg->open_manipulator_moving_state == msg->IS_MOVING)
    open_manipulator_is_moving_ = true;
  else
    open_manipulator_is_moving_ = false;
}

void slrvPickAndPlace::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT_AND_TOOL);
  for (int i = 0; i < msg->name.size(); i ++)
  {
    if (!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
    else if (!msg->name.at(i).compare("gripper"))  temp_angle.at(4) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;
}

void slrvPickAndPlace::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);

  present_kinematic_position_ = temp_position;
}

void slrvPickAndPlace::arPoseMarkerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
  std::vector<ArMarker> temp_buffer;
  for (int i = 0; i < msg->markers.size(); i ++)
  {
    ArMarker temp;
    temp.id = msg->markers.at(i).id;
    temp.position[0] = msg->markers.at(i).pose.pose.position.x;
    temp.position[1] = msg->markers.at(i).pose.pose.position.y;
    temp.position[2] = msg->markers.at(i).pose.pose.position.z;

    temp_buffer.push_back(temp);
  }

  ar_marker_pose = temp_buffer;
}

void slrvPickAndPlace::slrvPickAndPlaceCallback(const slrv_pick_and_place::Swap &msg){
  bool found_id1 = false;
  bool found_id2 = false;
  std::ostringstream oss;

  if(machine_begin){
    slrv_pick_and_place::Status status;
    status.statusID = STATUS::Busy;
    status.context = "Machine is not accepting new commands. Wait until the machine finishes before issuing a new command";
    slrv_status_publisher_.publish(status);
    return;
  }

  if(msg.pos1 > 9 || msg.pos1 < 0){
    //Return publish with invalid input
    slrv_pick_and_place::Status status;
    status.statusID = STATUS::InvalidInput;
    oss << "ID " << msg.pos1 << " is not a valid ID, please enter an ID with a number from 0 to 9!";
    status.context = oss.str();
    slrv_status_publisher_.publish(msg);
    return;
  }
  if(msg.pos2 > 9 || msg.pos2 < 0){
    //Return publish with invalid input
    slrv_pick_and_place::Status status;
    status.statusID = STATUS::InvalidInput;
    oss << "ID " << msg.pos2 << " is not a valid ID, please enter an ID with a number from 0 to 9!";
    status.context = oss.str();
    slrv_status_publisher_.publish(status);
    return;
  }
  if(msg.pos1 == msg.pos2){
    //Return publish with invalid input
    slrv_pick_and_place::Status status;
    status.statusID = STATUS::InvalidInput;
    oss << "ID " << msg.pos1 << " and " << msg.pos2 << " are equal, can't swap!";
    status.context = oss.str();
    slrv_status_publisher_.publish(status);
    return;
  } 

  for(int i = 0; i < ar_marker_pose.size(); i++){
    if(ar_marker_pose[i].id == msg.pos1){
      found_id1 = true;
    }
    else if(ar_marker_pose[i].id == msg.pos2){
      found_id2 = true;
    }
  }

  if(found_id1 && found_id2){
    //Return publish with valid input!
    slrv_pick_and_place::Status status;
    status.statusID = STATUS::AcceptedCommand;
    status.context = "Command was received successfully!";
    slrv_status_publisher_.publish(status);

    machine_begin = msg.begin;
    vec_swap.push_back(msg);
  }
  else{
    //Return publish with invalid input
    slrv_pick_and_place::Status status;
    status.statusID = STATUS::InvalidInput;
    if(!found_id1 && !found_id2){
      oss << "Could not find the markers with ID " << msg.pos1 << " and ID " << msg.pos2;
      status.context = oss.str();
    }
    else if(found_id1 && !found_id2){
      oss << "Found ID " << msg.pos1 << " but could not find the marker with ID " << msg.pos2;
      status.context = oss.str();
    }
    else if(!found_id1 && found_id2){
      oss << "Found ID " << msg.pos2 << " but could not find the marker with ID " << msg.pos1;
      status.context = oss.str();
    }
    slrv_status_publisher_.publish(status);
    return;
  }
}

void slrvPickAndPlace::publishCallback(const ros::TimerEvent&)
{
  printText();
  if (kbhit()) setModeState(std::getchar());

  if (mode_state_ == HOME_POSE)
  {
    std::vector<double> joint_angle;

    slrv_pick_and_place::Status status;
    status.statusID = STATUS::Busy;
    status.context = "Moving to home position";
    slrv_status_publisher_.publish(status);

    joint_angle.push_back( 0.00);
    joint_angle.push_back(-1.05);
    joint_angle.push_back( 0.35);
    joint_angle.push_back( 0.70);
    setJointSpacePath(joint_name_, joint_angle, 2.0);

    std::vector<double> gripper_value;
    gripper_value.push_back(0.0);
    setToolControl(gripper_value);
    mode_state_ = 0;
  }
  else if (mode_state_ == DEMO_START)
  {
    if (!open_manipulator_is_moving_){
      demoSequence();
    }
  }
  else if (mode_state_ == DEMO_STOP)
  {

  }
}

void slrvPickAndPlace::setModeState(char ch)
{
  if (ch == '1')
    mode_state_ = HOME_POSE;
  else if (ch == '2')
  {
    printf("Starting demo!");
    mode_state_ = DEMO_START;
    demo_count_ = 0;
    machine_state_ = 0;
  }
  else if (ch == '3')
    mode_state_ = DEMO_STOP;
}

ArMarker slrvPickAndPlace::find(std::vector<ArMarker> vec, int id){
  ArMarker marker;
  for(int i = 0; i < vec.size(); i++){
    if(vec[i].id == id){
      marker = vec[i];
      return marker;
    }
  }
  marker.id = -1;
  return marker;
}

void slrvPickAndPlace::demoSequence()
{
  std::vector<double> joint_angle;
  std::vector<double> kinematics_position;
  std::vector<double> kinematics_orientation;
  std::vector<double> gripper_value;

  switch (demo_count_)
  {
  case 0: // Go into home position
    joint_angle.push_back( 0.00);
    joint_angle.push_back(-1.05);
    joint_angle.push_back( 0.35);
    joint_angle.push_back( 0.70);
    setJointSpacePath(joint_name_, joint_angle, 1.5);
    machine_state_++;
    demo_count_ ++;
    break;
  case 1: // Go to the initial start position
    joint_angle.push_back( 0.01);
    joint_angle.push_back(-0.80);
    joint_angle.push_back( 0.00);
    joint_angle.push_back( 1.90);
    setJointSpacePath(joint_name_, joint_angle, 1.0);
    demo_count_ ++;
    break;
  case 2: // Open the gripper and wait for an action
    setJointSpacePath(joint_name_, present_joint_angle_, 3.0);
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    machine_state_++;
    demo_count_ ++;
    break;
  case 3:{ // Go over the vector with commands. Get a command and store the location of ID 1 in POS1
           // Get the X and Y coordinate from POS1 and go to that location

    if(vec_swap.size() > 0 && machine_begin){
      to_swap = vec_swap[0];
      vec_swap.pop_back();
      pos1 = find(ar_marker_pose, to_swap.pos1);
      kinematics_position.push_back(pos1.position[0]);
      kinematics_position.push_back(pos1.position[1]);
      kinematics_position.push_back(0.05);
      kinematics_orientation.push_back(0.74);
      kinematics_orientation.push_back(0.00);
      kinematics_orientation.push_back(0.66);
      kinematics_orientation.push_back(0.00);
      setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
      machine_state_++;
      demo_count_ ++;

      std::ostringstream oss;
      slrv_pick_and_place::Status status;
      status.statusID = STATUS::Busy;
      oss << "Moving box with ID " << pos1.id << " to the platform";
      status.context = oss.str();
      slrv_status_publisher_.publish(status);
      return;
    }
    else if(vec_swap.size() == 0 && machine_begin){
      machine_begin = false;
      slrv_pick_and_place::Status status;
      status.statusID = STATUS::WaitingOnCommand;
      status.context = "The machine is ready to receive new commands";
      slrv_status_publisher_.publish(status);
    }

    slrv_pick_and_place::Status status;
    status.statusID = STATUS::WaitingOnCommand;
    status.context = "Waiting on commands";
    slrv_status_publisher_.publish(status);

    demo_count_ = 1;  // If there is no command
    machine_state_ = 2;
    break;
  }     
  case 4: // Grip the box
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.push_back(-0.001);
    setToolControl(gripper_value);
    demo_count_ ++;
    break;
  case 5: // Go back to the initial position
    joint_angle.push_back( 0.01);
    joint_angle.push_back(-0.80);
    joint_angle.push_back( 0.00);
    joint_angle.push_back( 1.90);
    setJointSpacePath(joint_name_, joint_angle, 1.0);
    demo_count_ ++;
    break;
  case 6: // Go to the placement position (on top of the platform)
    joint_angle.push_back( 1.57);
    joint_angle.push_back(-0.21);
    joint_angle.push_back(-0.15);
    joint_angle.push_back( 1.89);
    setJointSpacePath(joint_name_, joint_angle, 1.0);
    demo_count_ ++;
    break;
  case 7: // place the box on the platform
    kinematics_position.push_back(present_kinematic_position_.at(0));
    kinematics_position.push_back(present_kinematic_position_.at(1));
    kinematics_position.push_back(present_kinematic_position_.at(2)-0.020);

    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);

    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);

    demo_count_ ++;
    break;
  case 8: // Open the gripper
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_ ++;
    break;
  case 9: // move up after placing the box
    kinematics_position.push_back(present_kinematic_position_.at(0));
    kinematics_position.push_back(present_kinematic_position_.at(1));
    kinematics_position.push_back(0.135);
    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);
    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
    demo_count_ ++;
    break;
  case 10: // move back to initial position
    joint_angle.push_back( 0.01);
    joint_angle.push_back(-0.80);
    joint_angle.push_back( 0.00);
    joint_angle.push_back( 1.90);
    setJointSpacePath(joint_name_, joint_angle, 1.0);
    demo_count_ ++;
    break;
  case 11: // Open the gripper
    //setJointSpacePath(joint_name_, present_joint_angle_, 3.0);
    //gripper_value.push_back(0.012);
    //setToolControl(gripper_value);
    machine_state_++;
    demo_count_ ++;
    break;
  case 12: // Go to the location of the second box which is stored in POS2
    pos2 = find(ar_marker_pose, to_swap.pos2);
    kinematics_position.push_back(pos2.position[0]);
    kinematics_position.push_back(pos2.position[1]);
    kinematics_position.push_back(0.05);
    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);
    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
    demo_count_ ++;
    break;
  case 13: // Grip the box
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.push_back(-0.001);
    setToolControl(gripper_value);
    demo_count_ ++;
    break;
  case 14: // Move back to initial position
    joint_angle.push_back( 0.01);
    joint_angle.push_back(-0.80);
    joint_angle.push_back( 0.00);
    joint_angle.push_back( 1.90);
    setJointSpacePath(joint_name_, joint_angle, 1.0);
    demo_count_ ++;
    break;
  case 15: // Place the box from POS2 on the location of POS1
    kinematics_position.push_back(pos1.position[0]);
    kinematics_position.push_back(pos1.position[1]);
    kinematics_position.push_back(0.05);
    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);
    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
    demo_count_ ++;
    break;
  case 16: // Open the gripper
    setJointSpacePath(joint_name_, present_joint_angle_, 3.0);
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_ ++;
    break;
  case 17: // Move up after placing the box
    kinematics_position.push_back(present_kinematic_position_.at(0));
    kinematics_position.push_back(present_kinematic_position_.at(1));
    kinematics_position.push_back(0.085);
    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);
    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
    machine_state_++;
    demo_count_ ++;
    break;
  case 18: // Move back to initial position
    joint_angle.push_back( 0.01);
    joint_angle.push_back(-0.80);
    joint_angle.push_back( 0.00);
    joint_angle.push_back( 1.90);
    setJointSpacePath(joint_name_, joint_angle, 1.0);
    demo_count_ ++;
    break;
  case 19: // Go to the placement position (on top of the platform)
    joint_angle.push_back( 1.57);
    joint_angle.push_back(-0.21);
    joint_angle.push_back(-0.15);
    joint_angle.push_back( 1.89);
    setJointSpacePath(joint_name_, joint_angle, 1.0);
    demo_count_ ++;
    break;
  case 20: // Get the box with ID1 from the platform
    kinematics_position.push_back(present_kinematic_position_.at(0));
    kinematics_position.push_back(present_kinematic_position_.at(1));
    kinematics_position.push_back(present_kinematic_position_.at(2)-0.020);

    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);

    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);

    demo_count_ ++;
    break;
  case 21: // Grip the box
    setJointSpacePath(joint_name_, present_joint_angle_, 1.0);
    gripper_value.push_back(-0.001);
    setToolControl(gripper_value);
    demo_count_ ++;
    break;
  case 22: // Move up after grabbing the box
    kinematics_position.push_back(present_kinematic_position_.at(0));
    kinematics_position.push_back(present_kinematic_position_.at(1));
    kinematics_position.push_back(0.105);
    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);
    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
    demo_count_ ++;
    break;
  case 23: // Move back to initial position
    joint_angle.push_back( 0.01);
    joint_angle.push_back(-0.80);
    joint_angle.push_back( 0.00);
    joint_angle.push_back( 1.90);
    setJointSpacePath(joint_name_, joint_angle, 1.0);
    demo_count_ ++;
    break;
  case 24: // Place the box with ID 1 on the position of POS2
    kinematics_position.push_back(pos2.position[0]);
    kinematics_position.push_back(pos2.position[1]);
    kinematics_position.push_back(0.05);
    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);
    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
    demo_count_ ++;
    break;
  case 25: // Open the gripper to place the box
    setJointSpacePath(joint_name_, present_joint_angle_, 3.0);
    gripper_value.push_back(0.010);
    setToolControl(gripper_value);
    demo_count_ ++;
    break;
  case 26: // Move up after placing the box
    kinematics_position.push_back(present_kinematic_position_.at(0));
    kinematics_position.push_back(present_kinematic_position_.at(1));
    kinematics_position.push_back(0.85);
    kinematics_orientation.push_back(0.74);
    kinematics_orientation.push_back(0.00);
    kinematics_orientation.push_back(0.66);
    kinematics_orientation.push_back(0.00);
    setTaskSpacePath(kinematics_position, kinematics_orientation, 2.0);
    machine_state_++;
    demo_count_ ++;
    break;
  case 27: // Go back to home position
    joint_angle.push_back( 0.00);
    joint_angle.push_back(-1.05);
    joint_angle.push_back( 0.35);
    joint_angle.push_back( 0.70);
    setJointSpacePath(joint_name_, joint_angle, 1.5);
    machine_state_ = 0;
    demo_count_ = 1;
    break;
  }
}


void slrvPickAndPlace::printText()
{
  system("clear");

  printf("\n");
  printf("--------------------------------------\n");
  printf("Pick and Place demonstration for SLRV!\n");
  printf("--------------------------------------\n");

  printf("1 : Home pose\n");
  printf("2 : start\n");
  printf("3 : Stop\n");

  printf("--------------------------------------\n");

  if (mode_state_ == DEMO_START)
  {
    std::ostringstream oss;

    switch(machine_state_)
    {
    case 1: {//Setting up machine
      slrv_pick_and_place::Status status;
      status.statusID = STATUS::Busy;
      status.context = "Setting up machine";
      slrv_status_publisher_.publish(status);
      break;
    }
    case 2: {//Getting Command
      break;
    }
    case 3: {//Move ID 1
      
      slrv_pick_and_place::Status status;
      status.statusID = STATUS::Busy;
      oss << "Moving box with ID " << pos1.id << " to the platform";
      status.context = oss.str();
      slrv_status_publisher_.publish(status);
      break;
    }
    case 12: {//Move ID 2
      slrv_pick_and_place::Status status;
      status.statusID = STATUS::Busy;
      oss << "Moving box with ID " << pos2.id << "to old location of the box with ID " << pos1.id;
      status.context = oss.str();
      slrv_status_publisher_.publish(status);
      break;
    }
    case 24: {//Move ID 1
      slrv_pick_and_place::Status status;
      status.statusID = STATUS::Busy;
      oss << "Moving box with ID " << pos1.id << "to old location of the box with ID " << pos2.id;
      status.context = oss.str();
      slrv_status_publisher_.publish(status);
      break;
    }
      break;
    }
  }
  else if (mode_state_ == DEMO_STOP)
  {
    printf("The end of demo\n");
  }

  printf("-----------------------------\n");
  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         present_joint_angle_.at(0),
         present_joint_angle_.at(1),
         present_joint_angle_.at(2),
         present_joint_angle_.at(3));
  printf("Present Tool Position: %.3lf\n", present_joint_angle_.at(4));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         present_kinematic_position_.at(0),
         present_kinematic_position_.at(1),
         present_kinematic_position_.at(2));
  printf("-----------------------------\n");

  if (vec_swap.size()) printf("\nCommands to execute:\n");
  for (int i = 0; i < vec_swap.size(); i ++)
  {
    printf("Swap id: %d with id: %d\n", vec_swap[i].pos1, vec_swap[i].pos2);
  }

  // if (ar_marker_pose.size()) printf("AR marker detected.\n");
  for (int i = 0; i < ar_marker_pose.size(); i ++)
  {
    std::ostringstream oss;
    slrv_pick_and_place::Status status;
    status.statusID = STATUS::GeneralInfo;
    oss << "Found Marker with ID " << ar_marker_pose[i].id;
    status.context = oss.str();
    slrv_status_publisher_.publish(status);
  }

  
}

bool slrvPickAndPlace::kbhit()
{
  termios term;
  tcgetattr(0, &term);

  termios term2 = term;
  term2.c_lflag &= ~ICANON;
  tcsetattr(0, TCSANOW, &term2);

  int byteswaiting;
  ioctl(0, FIONREAD, &byteswaiting);
  tcsetattr(0, TCSANOW, &term);
  return byteswaiting > 0;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "slrv_pick_and_place");
  ros::NodeHandle node_handle("");

  slrvPickAndPlace slrv_pick_and_place;

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(1)/*1000ms*/, &slrvPickAndPlace::publishCallback, &slrv_pick_and_place);

  while (ros::ok())
  {
    ros::spinOnce();
  }
  return 0;
}
