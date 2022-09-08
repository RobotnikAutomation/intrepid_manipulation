#include <intrepid_deep_grasp_demo/deep_grasp_demo.h>

DeepGraspDemo::DeepGraspDemo(ros::NodeHandle h) : RComponent(h)
{
  init(h);
}
DeepGraspDemo::~DeepGraspDemo()
{
}

void DeepGraspDemo::rosReadParams()
{
  bool required = true;
  bool not_required = false;


  arm_group_name_ = "arm";
  readParam(pnh_, "arm_group_name", arm_group_name_, arm_group_name_, required);

  world_frame_ = "world";
  readParam(pnh_, "world_frame", world_frame_, world_frame_, required);

  look_dist_ = 0.30;
  readParam(pnh_, "look_dist", look_dist_, look_dist_, required);

  double timeout = 20;
  readParam(pnh_, "move_group_timeout", timeout, timeout, required);
  move_group_timeout_ = ros::WallDuration(timeout);

  host = "localhost";
  readParam(pnh_, "host", host, host, required);  

  port = 33829;
  readParam(pnh_, "port", port, port, required);

  desired_constraint_name_ = "";
  readParam(pnh_, "moveit_constraint", desired_constraint_name_, desired_constraint_name_, required); 

  camera_in_pcl_topic_ = "";
  readParam(pnh_, "input_pcl_topic", camera_in_pcl_topic_, camera_in_pcl_topic_, required); 

  merged_out_pcl_topic_ = "filtered_pcl/points";
  readParam(pnh_, "output_pcl_topic", merged_out_pcl_topic_, merged_out_pcl_topic_, required); 

  point_cloud_frame_ = "";
  readParam(pnh_, "camera_optical_frame", point_cloud_frame_, point_cloud_frame_, required); 
  
  end_effector_frame_ = "";
  readParam(pnh_, "hand_frame", end_effector_frame_, end_effector_frame_, required); 

  scan_move_linear_ = 0.10;
  readParam(pnh_, "move_linear", scan_move_linear_, scan_move_linear_, required); 

  scan_move_angular_ = 0.30;
  readParam(pnh_, "move_angular", scan_move_angular_, scan_move_angular_, required); 
}

int DeepGraspDemo::rosSetup()
{
  if (ros_initialized)
  {
    RCOMPONENT_INFO("Already initialized");

    return rcomponent::INITIALIZED;
  }

  // Configure moveit move_group interface

  move_group_tf2_buffer_.reset(new tf2_ros::Buffer);

  try
  {
    move_group_.reset(
        new moveit::planning_interface::MoveGroupInterface(arm_group_name_, move_group_tf2_buffer_, move_group_timeout_));
  }
  catch (const std::runtime_error& e)
  {
    RCOMPONENT_ERROR("Cannot create move group with group name: %s. Is MoveIt running? Group name is correct?",
                     arm_group_name_.c_str());
    RCOMPONENT_ERROR_STREAM("Exception: " << e.what());
    return rcomponent::ERROR;
  }

  // Configure moveit move_group_gripper interface

  move_group_gripper_tf2_buffer_.reset(new tf2_ros::Buffer);

  try
  {
    move_group_gripper_.reset(
        new moveit::planning_interface::MoveGroupInterface("gripper", move_group_gripper_tf2_buffer_, move_group_timeout_));
  }
  catch (const std::runtime_error& e)
  {
    RCOMPONENT_ERROR("Cannot create move group with group name: %s. Is MoveIt running? Group name is correct?",
                     arm_group_name_.c_str());
    RCOMPONENT_ERROR_STREAM("Exception: " << e.what());
    return rcomponent::ERROR;
  }

  // Configure moveit planning scene interface

  bool wait = true;
  std::string name_space = "";
  planning_scene_interface_.reset(
        new moveit::planning_interface::PlanningSceneInterface(name_space, wait));

  // Configure moveit planning scene monitor
  string robot_description = "robot_description";
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description));      

  // update the planning scene monitor with the current state
  bool success = planning_scene_monitor_->requestPlanningSceneState("get_planning_scene");
  RCOMPONENT_INFO_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));

  // keep up to date with new changes
  planning_scene_monitor_->startSceneMonitor("move_group/monitored_planning_scene");

  // planning_scene_(new planning_scene::PlanningScene(move_group_->getRobotModel()));

  // Set PickupObject action server 
  bool autostart = false;

  pick_object_as_.reset(
      new actionlib::SimpleActionServer<intrepid_manipulation_msgs::PickupObjectAction>(pnh_, "pickup_object", autostart));
  pick_object_as_->registerGoalCallback(boost::bind(&DeepGraspDemo::goalCB, this, std::string("pickup_object")));
  pick_object_as_->registerPreemptCallback(boost::bind(&DeepGraspDemo::preemptCB, this));

  pickup_from_as_.reset(
      new actionlib::SimpleActionServer<intrepid_manipulation_msgs::PickupFromAction>(pnh_, "pickup_from", autostart));
  pickup_from_as_->registerGoalCallback(boost::bind(&DeepGraspDemo::goalCB, this, std::string("pickup_from")));
  pickup_from_as_->registerPreemptCallback(boost::bind(&DeepGraspDemo::preemptCB, this));

  place_on_as_.reset(
      new actionlib::SimpleActionServer<intrepid_manipulation_msgs::PlaceOnAction>(pnh_, "place_on", autostart));
  place_on_as_->registerGoalCallback(boost::bind(&DeepGraspDemo::goalCB, this, std::string("place_on")));
  place_on_as_->registerPreemptCallback(boost::bind(&DeepGraspDemo::preemptCB, this));

  // Set PickupObject action client
  pick_object_ac_.reset(new DGDAC(nh_, "deep_grasp_demo/pickup_object", false));
  
  // Perform connection to moveit warehouse 
  conn_ = moveit_warehouse::loadDatabase();
  conn_->setParams(host, port);

  ROS_INFO("Connecting to warehouse on %s:%d", host.c_str(), port);

  while (!conn_->connect())
  {
    ROS_ERROR("Failed to connect to DB on %s:%d ", host, port);
    ros::Duration(2).sleep();
    conn_->setParams(host, port);
  }
  
  move_group_->setConstraintsDatabase(host,port);
  std::vector< std::string > stored_constraints = move_group_->getKnownConstraints();
  if (stored_constraints.empty()){
    ROS_WARN("There are no constraints stored in database");}
  else
  {
    ROS_INFO("Constraints currently stored in database:");
    for (const std::string& name : stored_constraints)
      ROS_INFO(" * %s", name.c_str());
  }

  merged_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(merged_out_pcl_topic_, 10);

  // Subscribe to point cloud topic
  point_cloud_sub = nh_.subscribe(camera_in_pcl_topic_,1, &DeepGraspDemo::point_cloud_callback, this);


  // Subscribe to intrepid RVIZ GUI topic
  intrepid_gui_sub = nh_.subscribe("/intrepid_rviz_dashboard",10, &DeepGraspDemo::intrepid_gui_callback, this);

  // Set up pickup object goal topic publisher
  pick_goal_pub = nh_.advertise<intrepid_manipulation_msgs::PickupObjectActionGoal>("deep_grasp_demo/pickup_object/goal", 10);
  pick_cancel_pub = nh_.advertise<actionlib_msgs::GoalID>("deep_grasp_demo/pickup_object/cancel", 10);
  // Set up rviz overlay text publisher
  overlay_text_pub = nh_.advertise<jsk_rviz_plugins::OverlayText>("/intrepid_rviz_text", 10);


  // TF Listener and Broadcaster
  tf_listener = new  tf2_ros::TransformListener(tfBuffer);


  // Construct MTC deep pick and place task pipeline
  deep_pick_task.reset(new intrepid_deep_grasp_demo::DeepPickTask("deep_pick_task", nh_));
  deep_pick_task->loadParameters();

  dgd_pcl_pub_timer = pnh_.createTimer(ros::Duration(0.1), std::bind(&DeepGraspDemo::pubPointCloudCallback, this));


  return RComponent::rosSetup();
}

int DeepGraspDemo::rosShutdown()
{
  return RComponent::rosShutdown();
}

int DeepGraspDemo::setup()
{
  // Checks if has been initialized
  int setup_result;

  setup_result = rcomponent::RComponent::setup();
  if (setup_result != rcomponent::OK)
  {
    return setup_result;
  }


  move_group_gripper_->setMaxVelocityScalingFactor(1.0);

  // Start pick object action server
  pick_object_as_->start();
  pickup_from_as_->start();
  place_on_as_->start();
  RCOMPONENT_INFO_STREAM("Started server: pickup object");

  // Set pick object action client
  pick_object_ac_->waitForServer();
  if(pick_object_ac_->isServerConnected()){RCOMPONENT_INFO_STREAM("Action client connected to pickup object server");}

  // Initialize execute flag
  allow_execute_ = false;
  allow_start_ = false;
  allow_display_ = false;
  stop_execution_ = false;

  publish_cloud_ = false;

  action_ready_flag_ = true;

  // Initialize pose message checking variables
  null_pose = geometry_msgs::Pose();

  // Initialize point clouds

/*   input_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  filtered_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>); */
  merged_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  filtered_merged_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  cloud_in_msg = sensor_msgs::PointCloud2();

  // Setup RVIZ overlay text description
  rviz_text_msg = jsk_rviz_plugins::OverlayText();
  rviz_text_msg.width = 1000;
  rviz_text_msg.height = 50;
  rviz_text_msg.left = 30;
  rviz_text_msg.top = 30;
  rviz_text_msg.text_size = 12;
  rviz_text_msg.line_width = 2;
  rviz_text_msg.font = "DejaVu Sans Mono";
  rviz_text_msg.fg_color = set_text_color(feedback_color);
  rviz_text_msg.bg_color = set_text_color(background_color);

  rviz_text_msg.text = "";
  rviz_text_msg.fg_color = set_text_color(feedback_color);
  overlay_text_pub.publish(rviz_text_msg);

  // Create overlay text publisher timer
  publish_computation_time = false;
  publish_computation_time_timer = pnh_.createTimer(ros::Duration(0.1), std::bind(&DeepGraspDemo::publishComputationTimeCallback, this));

  // Create planning scene
  if(create_planning_scene()){
    return rcomponent::OK;
  }else{
    return rcomponent::ERROR;
  }
}

void DeepGraspDemo::standbyState()
{
  // Move to home position without selected constraints
  move_group_->detachObject();  
  move_group_->clearPathConstraints();
  move_group_->setMaxAccelerationScalingFactor(1.0);
  move_group_->setMaxVelocityScalingFactor(1.0);
  move_group_->setNamedTarget("Home");
  bool success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success_move){

    ROS_INFO("Moved to home position, ready to take commands");
 
    switchToState(robotnik_msgs::State::READY_STATE);
    rviz_text_msg.text = "READY TO RECEIVE COMMANDS";
    rviz_text_msg.fg_color = set_text_color(feedback_color);
    overlay_text_pub.publish(rviz_text_msg);

  }else{
    ROS_WARN("Could not move to home position");
  }

}

void DeepGraspDemo::readyState()
{ 
  action_ready_flag_ = true;

  if (pick_object_as_->isActive() == false && pickup_from_as_->isActive() == false && place_on_as_->isActive() == false)
  {
    ROS_INFO_THROTTLE(3, "I do not have a goal");
    return;
  }

  action_ready_flag_ = false;

  ROS_INFO_THROTTLE(3, "I have a new goal!");

  // Select constraint
  move_group_->setPathConstraints(desired_constraint_name_);
  global_constraint = move_group_->getPathConstraints();

  if(desired_constraint_name_ != global_constraint.name){
    if(pick_object_as_->isActive() == true){ 
      pick_object_result_.success = false;
      pick_object_result_.message = "Desired moveit constraint not available in database";
      pick_object_as_->setAborted(pick_object_result_);
      return;
    }
  }

  ROS_INFO("Planning with Constraint: %s", global_constraint.name.c_str());


  if(pick_object_as_->isActive() == true){ 

    RCOMPONENT_INFO_STREAM("Pickup object goal has started");
    
    // Check whether pre pick goal pose is zero
    bool is_pose_zero_ = pick_object_goal_->pick_position.pose == null_pose;
    
    // If goal pose message is valid move to desired position
    if (!is_pose_zero_){
      if(move_to_look_pose(pick_object_goal_->pick_position)){
        RCOMPONENT_INFO_STREAM("Moved to pre-pick position");
      }else{
        RCOMPONENT_INFO_STREAM("Execution failed, could not move to pre-pick position");
        pick_object_result_.success = false;
        pick_object_result_.message = "Execution failed, could not move to pre-pick position";
        pick_object_as_->setAborted(pick_object_result_);
        return;      
      }
    }

    move_group_->clearPathConstraints();

    rviz_text_msg.text = "SCANNING AREA: GENERATING POINT CLOUD";
    rviz_text_msg.fg_color = set_text_color(feedback_color);
    overlay_text_pub.publish(rviz_text_msg);

    if(!scanObject()){
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return;
    }

    if (!pick_object_as_->isActive()) return;

   /*     pick_object_result_.success = true;
    pick_object_result_.message = "Execution complete";
    pick_object_as_->setSucceeded(pick_object_result_); */

    rviz_text_msg.text = "STARTING DEEP-LEARNING GRASP POSE DETECTION ALGORITHM";
    rviz_text_msg.fg_color = set_text_color(feedback_color);
    overlay_text_pub.publish(rviz_text_msg);
    ros::Duration(2.0).sleep();
    timer_start_ = ros::WallTime::now();
    publish_computation_time = true;
    // Initialize pick and place task
    deep_pick_task->init();

    if (!pick_object_as_->isActive()) return;

    if (deep_pick_task->plan())
    { //action_ready_flag_ = false;
      publish_computation_time = false;
      publish_cloud_ = false;
      RCOMPONENT_INFO_STREAM("Planning succeded");
      rviz_text_msg.text = "OPTIMAL MOTION PLAN FOUND";
      rviz_text_msg.fg_color = set_text_color(success_color);
      overlay_text_pub.publish(rviz_text_msg);
      ros::Duration(2.0).sleep();
      rviz_text_msg.text = "";
      rviz_text_msg.fg_color = set_text_color(feedback_color);
      overlay_text_pub.publish(rviz_text_msg);

      while(pick_object_as_->isActive()){

        if(allow_execute_){ 
          allow_execute_ = false;
          rviz_text_msg.text = "EXECUTING MOTION PLAN";
          rviz_text_msg.fg_color = set_text_color(feedback_color);
          overlay_text_pub.publish(rviz_text_msg);

          if(deep_pick_task->execute()){
            RCOMPONENT_INFO_STREAM("Execution complete");
            pick_object_result_.success = true;
            pick_object_result_.message = "Execution complete";
            pick_object_as_->setSucceeded(pick_object_result_);
            rviz_text_msg.text = "EXECUTION SUCCESSFUL";
            rviz_text_msg.fg_color = set_text_color(success_color);
            overlay_text_pub.publish(rviz_text_msg);
            return;
          }else{
            RCOMPONENT_INFO_STREAM("Execution failed");
            pick_object_result_.success = false;
            pick_object_result_.message = "Execution failed";
            pick_object_as_->setAborted(pick_object_result_);
            rviz_text_msg.text = "EXECUTION FAILED";
            rviz_text_msg.fg_color = set_text_color(warning_color);
            overlay_text_pub.publish(rviz_text_msg);
            return;
          }
        }

      }


      rviz_text_msg.text = "PICKUP OBJECT ACTION WAS PREEMPTED";
      rviz_text_msg.fg_color = set_text_color(warning_color);
      overlay_text_pub.publish(rviz_text_msg);
      return;

    }
    else
    {
      publish_computation_time = false;
      RCOMPONENT_INFO_STREAM("Planning failed");
      pick_object_result_.success = false;
      pick_object_result_.message = "Planning failed";
      pick_object_as_->setAborted(pick_object_result_);
      rviz_text_msg.text = "PLANNING FAILED";
      rviz_text_msg.fg_color = set_text_color(warning_color);
      overlay_text_pub.publish(rviz_text_msg);
      return;
    }
        
  }   

  if(pickup_from_as_->isActive() == true){     // Get desired goal and set as target
    geometry_msgs::PoseStamped pose = pickup_from_goal_->pick_pose;
    // Call move_to_pose function
    pickup_from_pose(pose);
  }   
  
  if(place_on_as_->isActive() == true){     // Get desired goal and set as target
    geometry_msgs::PoseStamped pose = place_on_goal_->place_pose;
    // Call move_to_pose function
    place_on_pose(pose);
  }   

}

void DeepGraspDemo::goalCB(const std::string& action)
{
  RCOMPONENT_INFO_STREAM("I have received an action to: " << action);
  std::string action_ = action;
  if (action_ == "pickup_object"){
    ROS_INFO_STREAM("IS ACTION SERVER ACTIVE " << pick_object_as_->isActive());
    ROS_INFO_STREAM("IS GOAL AVAILABLE " << pick_object_as_->isNewGoalAvailable());
    pick_object_as_->isPreemptRequested();
    pick_object_goal_ = pick_object_as_->acceptNewGoal();
  }
  if (action_ == "pickup_from"){
    pickup_from_goal_ = pickup_from_as_->acceptNewGoal();
    pickup_from_as_->isPreemptRequested();
  }
  if (action_ == "place_on"){
    place_on_goal_ = place_on_as_->acceptNewGoal();
    place_on_as_->isPreemptRequested();
  }

}

void DeepGraspDemo::preemptCB()
{
  RCOMPONENT_INFO_STREAM("ACTION: Preempted");
  //result_.success = false;
  //result_.message = "Goal has been cancelled, stopping execution.";
  // set the action state to preempted
  move_group_->stop();

  publish_computation_time = false;
  publish_cloud_ = false;
  //deep_pick_task->preemptTask();

  rviz_text_msg.text = "GOAL IS CANCELLED";
  rviz_text_msg.fg_color = set_text_color(feedback_color);
  overlay_text_pub.publish(rviz_text_msg);

  pick_object_result_.success = false;
  pick_object_result_.message = "Goal was cancelled";
  pick_object_as_->setPreempted(pick_object_result_);
  pickup_from_as_->setPreempted();
  place_on_as_->setPreempted();
}


void DeepGraspDemo::pickup_from_pose(geometry_msgs::PoseStamped pose){
  move_group_gripper_->setStartStateToCurrentState ();
  move_group_gripper_->setNamedTarget("Open");
  move_group_gripper_->move();
  // Set pre-position goal
  move_group_->setPoseTarget(pose);
  // Plan to pre-position goal
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // If plan is successful execute trajectory
  if(success_plan){
    pickup_from_feedback_.state.clear();
    pickup_from_feedback_.state = "Plan to desired pick position computed";
    pickup_from_as_->publishFeedback(pickup_from_feedback_);

    //Check if goal is active and move to pre-position goal
    if (!pickup_from_as_->isActive()) return;
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //Check if execute is successful
    if(success_execute){
      move_group_gripper_->setStartStateToCurrentState ();
      move_group_gripper_->setNamedTarget("Close");
      move_group_gripper_->move();
      move_group_->setNamedTarget("Look");
      bool success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      if(success_move){
       move_group_->setNamedTarget("Place_final");
       success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      }
      pickup_from_feedback_.state.clear();
      pickup_from_feedback_.state = "Moved end-effector to desired pick pose";
      pickup_from_as_->publishFeedback(pickup_from_feedback_);

      pickup_from_result_.success = true;
      pickup_from_result_.message = "Move end-effector to desired pick pose action: SUCCESSFUL";
      pickup_from_as_->setSucceeded(pickup_from_result_);



      return;
    }else{
      pickup_from_result_.success = false;
      pickup_from_result_.message = "Could not move end-effector to desired pick pose";
      pickup_from_as_->setAborted(pickup_from_result_);
      return;
    }
          
  }else{
    pickup_from_result_.success = false;
    pickup_from_result_.message = "Could not plan to desired pick pose";
    pickup_from_as_->setAborted(pickup_from_result_);
    return;
  }
}

void DeepGraspDemo::place_on_pose(geometry_msgs::PoseStamped pose){

      move_group_->setNamedTarget("Look");
      bool success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  // Set pre-position goal
  move_group_->setPoseTarget(pose);
  // Plan to pre-position goal
  success_plan = (move_group_->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // If plan is successful execute trajectory
  if(success_plan){
    place_on_feedback_.state.clear();
    place_on_feedback_.state = "Plan to desired place position computed";
    place_on_as_->publishFeedback(place_on_feedback_);

    //Check if goal is active and move to pre-position goal
    if (!place_on_as_->isActive()) return;
    success_execute = (move_group_->execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //Check if execute is successful
    if(success_execute){
      move_group_gripper_->setStartStateToCurrentState ();
      move_group_gripper_->setNamedTarget("Open");
      move_group_gripper_->move(); 

      move_group_->setNamedTarget("Home");
      success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	 
      place_on_feedback_.state.clear();
      place_on_feedback_.state = "Moved end-effector to desired place pose";
      place_on_as_->publishFeedback(place_on_feedback_);

      place_on_result_.success = true;
      place_on_result_.message = "Move end-effector to desired place pose action: SUCCESSFUL";
      place_on_as_->setSucceeded(place_on_result_);
      return;
    }else{
      place_on_result_.success = false;
      place_on_result_.message = "Could not move end-effector to desired place pose";
      place_on_as_->setAborted(place_on_result_);
      return;
    }
          
  }else{
    place_on_result_.success = false;
    place_on_result_.message = "Could not plan to desired place pose";
    place_on_as_->setAborted(place_on_result_);
    return;
  }
}

bool DeepGraspDemo::create_planning_scene()
{
  std::vector<Object_Builder> parsed_objects; //
  std::vector<moveit_msgs::CollisionObject> moveit_objects; // Moveit objects defined in objects config yaml


  // Read and store parameter: objects from parameter server
  std::vector<std::string> object_names;
  bool required = true;
  readParam(pnh_, "objects", object_names, object_names, required); 

  // Create class Object_Builder objects
  for (auto const & object_name: object_names)
  { 
    parsed_objects.push_back(Object_Builder(ros::NodeHandle(pnh_ , object_name), object_name));
  }

  //Object_builder objects into Moveit's collision objects
  for (auto & parsed_object: parsed_objects)
  { 
    moveit_msgs::CollisionObject collision_object;
    collision_object = parsed_object.getObject();

    try{
     tfBuffer.lookupTransform(world_frame_,collision_object.header.frame_id,ros::Time(0),ros::Duration(1.0));
    }
    catch(tf2::TransformException ex){
      ROS_ERROR("Error when adding %s object to desired frame. Collision object and world frame not in same tf tree. Lookup Transform error: %s",collision_object.id.c_str(), ex.what());
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return false;
    }


    collision_object.operation = collision_object.ADD;
    moveit_objects.push_back(collision_object);
  }

  // Add collision objects into the world
  planning_scene_interface_->applyCollisionObjects(moveit_objects);

  return true;

}


std_msgs::ColorRGBA DeepGraspDemo::set_text_color(std::vector<double>& rgba_color){
  rgba_color.resize(4);
  std_msgs::ColorRGBA rgba_color_msg;
  rgba_color_msg.r = rgba_color[0];
  rgba_color_msg.g = rgba_color[1];
  rgba_color_msg.b = rgba_color[2];
  rgba_color_msg.a = rgba_color[3];
  return rgba_color_msg;
}

void DeepGraspDemo::intrepid_gui_callback(const sensor_msgs::Joy& msg){

  allow_start_ = msg.buttons[1];
  allow_display_ = msg.buttons[2];
  allow_execute_ = msg.buttons[3];
  stop_execution_ = msg.buttons[4];

  if(allow_display_){
    allow_display_ = false;
    rviz_text_msg.text = "DISPLAYING PLANNED PATH";
    rviz_text_msg.fg_color = set_text_color(feedback_color);
    overlay_text_pub.publish(rviz_text_msg);
    deep_pick_task->displayPlan();
  }
  if(allow_start_){
    allow_start_ = false;
    if(pick_object_as_->isActive()){
      //pick_object_as_->setPreempted(pick_object_result_);
      //pick_object_ac_->cancelGoal(); 
      //pick_object_ac_->waitForResult();
      actionlib_msgs::GoalID cancel_msg_ = actionlib_msgs::GoalID();
      pick_cancel_pub.publish(cancel_msg_);
      
    }

    while(!action_ready_flag_){
      
    }

    intrepid_manipulation_msgs::PickupObjectGoal msg;
    msg.pick_position.header.stamp = ros::Time::now();

    if(pick_object_ac_->isServerConnected() ){
      pick_object_ac_->sendGoal(msg);
    }
    
  }
  if(stop_execution_){
    stop_execution_ = false;
    move_group_->stop();
  }
}

bool DeepGraspDemo::scanObject(){

  merged_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  filtered_merged_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  geometry_msgs::PoseStamped look_pose = move_group_->getCurrentPose();
  geometry_msgs::TransformStamped look_pose_tf_msg;
  geometry_msgs::TransformStamped link_5_tf_msg;

  try{
    look_pose_tf_msg = tfBuffer.lookupTransform(world_frame_,point_cloud_frame_,ros::Time::now(),ros::Duration(2.0));
  }
  catch(tf2::TransformException ex){
    ROS_ERROR("Could not transform between camera cloud frame: %s and base frame: %s.", point_cloud_frame_.c_str(), world_frame_.c_str());
    return false;
  }

  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = end_effector_frame_;
  ocm.header.frame_id = world_frame_;
  ocm.orientation = look_pose.pose.orientation;
  ocm.absolute_x_axis_tolerance = 0.4;
  ocm.absolute_y_axis_tolerance = 0.4;
  ocm.absolute_z_axis_tolerance = 0.4;
  ocm.weight = 100.0;
  
  moveit_msgs::Constraints orientation_constraint = moveit_msgs::Constraints();
  orientation_constraint.orientation_constraints.push_back(ocm);

  moveit_msgs::Constraints merged_constraint = kinematic_constraints::mergeConstraints(global_constraint,orientation_constraint);
  
  move_group_->setPathConstraints(merged_constraint);

  move_rel_cartesian(0, scan_move_linear_, 0, 0.0, 0, 0, merged_constraint);
  move_rel_cartesian(0, 0.0, 0, scan_move_angular_, 0, 0, merged_constraint);
  addPointCloud();
  
  move_to_cartesian(look_pose.pose, merged_constraint);
  

  move_rel_cartesian(0, -scan_move_linear_, 0, 0.0, 0, 0, merged_constraint);
  move_rel_cartesian(0, 0.0, 0, -scan_move_angular_, 0, 0, merged_constraint);
  addPointCloud();
  
  move_to_cartesian(look_pose.pose, merged_constraint);
  

  move_rel_cartesian(scan_move_linear_, 0, 0, 0, 0.0, 0, merged_constraint);
  move_rel_cartesian(0.0, 0, 0, 0, -scan_move_angular_, 0, merged_constraint);
  addPointCloud();
  
  move_to_cartesian(look_pose.pose, merged_constraint);
  

  move_rel_cartesian(-scan_move_linear_, 0, 0, 0, 0.0, 0, merged_constraint);
  move_rel_cartesian(0.0, 0, 0, 0, +scan_move_angular_, 0, merged_constraint);
  addPointCloud();
  
  move_to_cartesian(look_pose.pose, merged_constraint);
  

  addPointCloud();

  ros::Duration(2.0).sleep();

  move_group_->clearPathConstraints();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr passthrough_cloud;
  passthrough_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(merged_cloud);

  if(world_frame_ == "robot_base_footprint"){
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.45, 2);
  }else{
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-2, 2);
  }

  pass.filter(*passthrough_cloud);

  try{
    look_pose_tf_msg = tfBuffer.lookupTransform(point_cloud_frame_,world_frame_,ros::Time::now(),ros::Duration(10.0));
  }
  catch(tf2::TransformException ex){
    ROS_ERROR("Could not transform between camera cloud frame: %s and base frame: %s.",point_cloud_frame_.c_str(), world_frame_.c_str());
    return false;
  }

  Eigen::Affine3d world_to_camera_ = tf2::transformToEigen(look_pose_tf_msg);
  pcl::transformPointCloud(*passthrough_cloud, *passthrough_cloud, world_to_camera_.matrix()); 

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (passthrough_cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*filtered_merged_cloud);
 
  publish_cloud_ = true;

  return true;
}

void DeepGraspDemo::move_rel_cartesian(const double& x, const double& y, const double& z, const double& roll, const double& pitch, const double& yaw, const moveit_msgs::Constraints& constraint){
  move_group_->setPoseReferenceFrame(end_effector_frame_);
  std::vector<geometry_msgs::Pose> waypoints = std::vector<geometry_msgs::Pose>();
  //waypoints.clear();
  geometry_msgs::Pose waypoint_cartesian_pose;
  waypoint_cartesian_pose = geometry_msgs::Pose();
  waypoint_cartesian_pose.position.x =  x;
  waypoint_cartesian_pose.position.y =  y;
  waypoint_cartesian_pose.position.z =  z;
  waypoint_cartesian_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  waypoints.push_back(waypoint_cartesian_pose);  
  moveit_msgs::RobotTrajectory trajectory; // Cartesian trajectory
  double success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, constraint, true);
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan; // Cartesian Plan
  cartesian_plan.trajectory_ = trajectory;
  move_group_->execute(cartesian_plan);
  move_group_->setPoseReferenceFrame(world_frame_);
}

void DeepGraspDemo::move_to_cartesian(const geometry_msgs::Pose& cartesian_pose, const moveit_msgs::Constraints& constraint){
  move_group_->setPoseReferenceFrame(world_frame_);
  std::vector<geometry_msgs::Pose> waypoints = std::vector<geometry_msgs::Pose>();
  //waypoints.clear();
  geometry_msgs::Pose waypoint_cartesian_pose;
  waypoint_cartesian_pose = geometry_msgs::Pose();
  waypoint_cartesian_pose = cartesian_pose;
  waypoints.push_back(waypoint_cartesian_pose); 
  moveit_msgs::RobotTrajectory trajectory; // Cartesian trajectory
  moveit::planning_interface::MoveGroupInterface::Plan cartesian_plan; // Cartesian Plan
  double success_cartesian_plan = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, constraint, true);
  cartesian_plan.trajectory_ = trajectory;
  move_group_->execute(cartesian_plan);
}

void DeepGraspDemo::point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg){
  cloud_in_msg = *msg;
}

void DeepGraspDemo::pubPointCloudCallback(){
  if(publish_cloud_){
    // Publish the cloud for visualization and debugging purposes
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*filtered_merged_cloud, cloud_msg);
    cloud_msg.header.frame_id = point_cloud_frame_;
    merged_cloud_pub_.publish(cloud_msg);
  }
}

void DeepGraspDemo::publishComputationTimeCallback(){
  if (publish_computation_time){
    double elapsed_time_ = (ros::WallTime::now() - timer_start_).toNSec() * 1e-9;
    rviz_text_msg.text = (boost::format("DEEP LEARNING ALGORITHM COMPUTATION TIME: %1$10.2f seconds\n") % elapsed_time_).str();
    rviz_text_msg.fg_color = set_text_color(feedback_color);
    overlay_text_pub.publish(rviz_text_msg);
  }
}


void DeepGraspDemo::addPointCloud(){
  
  /* sensor_msgs::PointCloud2 current_point_cloud = cloud_in_msg; */

  /* point_cloud_frame_ = msg.header.frame_id; */

  ROS_INFO_STREAM("Point cloud frame: " << point_cloud_frame_);

  sensor_msgs::PointCloud2 current_point_cloud = cloud_in_msg;

  geometry_msgs::TransformStamped current_pose_tf_msg;

  try{
    current_pose_tf_msg = tfBuffer.lookupTransform(world_frame_, point_cloud_frame_, current_point_cloud.header.stamp, ros::Duration(2));
  }
  catch(tf2::TransformException ex){
    ROS_ERROR("Could not transform between camera cloud frame: %s and base frame: %s. Lookup Transform error: %s",point_cloud_frame_.c_str(), world_frame_.c_str());
    switchToState(robotnik_msgs::State::FAILURE_STATE);
    return;
  }

  Eigen::Affine3d world_to_camera_ = tf2::transformToEigen(current_pose_tf_msg);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(current_point_cloud, *cloud.get());
  pcl::transformPointCloud(*cloud, *cloud, world_to_camera_.matrix()); 
  
/*   tf2::Stamped< tf2::Transform > current_pose_tf, base_pose_tf;

  tf2::fromMsg(current_pose_tf_msg, current_pose_tf); 
  tf2::fromMsg(base_pose_tf_msg, base_pose_tf); 

  tf2::Transform rel_tf= base_pose_tf.inverseTimes(current_pose_tf);

  Eigen::Affine3d current_to_base = tf2::transformToEigen(tf2::toMsg(rel_tf));

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(current_point_cloud, *cloud.get());
  pcl::transformPointCloud(*cloud, *cloud, current_to_base.matrix()); */
 
  *merged_cloud += *cloud;

}

bool DeepGraspDemo::move_to_look_pose(geometry_msgs::PoseStamped pose){
  
  pose.pose.position.z += look_dist_;
  move_group_->setPositionTarget(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
  bool success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  
  if(success_move){
    std::vector<double> joints;
    joints = move_group_->getCurrentJointValues();

    joints.at(5) = 3.1416;

    move_group_->setJointValueTarget(joints);
    return (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }else return false;
}
