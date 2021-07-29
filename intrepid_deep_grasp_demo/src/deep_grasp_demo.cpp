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

  required = true;
  arm_group_name_ = "arm";
  readParam(pnh_, "arm_group_name", arm_group_name_, arm_group_name_, required);

  required = true;
  world_frame_ = "world";
  readParam(pnh_, "world_frame", world_frame_, world_frame_, required);

  required = false;
  double timeout = 20;
  readParam(pnh_, "move_group_timeout", timeout, timeout, required);
  move_group_timeout_ = ros::WallDuration(timeout);

/*   required = true;
  host = "localhost";
  readParam(pnh_, "host", host, host, required);  

  required = true;
  port = 33829;
  readParam(pnh_, "port", port, port, required); */
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

  // Configure moveit planning scene interface

  bool wait = true;
  std::string name_space = "";
  planning_scene_interface_.reset(
        new moveit::planning_interface::PlanningSceneInterface(name_space, wait));

  // Configure moveit planning scene monitor
  string robot_description = "robot_description";
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description));      

  // update the planning scene monitor with the current state
  bool success = planning_scene_monitor_->requestPlanningSceneState("/get_planning_scene");
  RCOMPONENT_INFO_STREAM("Request planning scene " << (success ? "succeeded." : "failed."));

  // keep up to date with new changes
  planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");

  // planning_scene_(new planning_scene::PlanningScene(move_group_->getRobotModel()));

  // Set PickupObject action server 
  bool autostart = false;

  pick_object_as_.reset(
      new actionlib::SimpleActionServer<intrepid_manipulation_msgs::PickupObjectAction>(pnh_, "pickup_object", autostart));
  pick_object_as_->registerGoalCallback(boost::bind(&DeepGraspDemo::goalCB, this, std::string("pickup_object")));
  pick_object_as_->registerPreemptCallback(boost::bind(&DeepGraspDemo::preemptCB, this));

/*   conn_ = moveit_warehouse::loadDatabase();
  conn_->setParams(host, port);

  ROS_INFO("Connecting to warehouse on %s:%d", host.c_str(), port);

  while (!conn_->connect())
  {
    ROS_ERROR("Failed to connect to DB on %s:%d ", host.c_str(), port);
    ros::Duration(2).sleep();
    conn_->setParams(host, port);
  }
  

  move_group_->setConstraintsDatabase(host,port);
  std::vector< std::string > stored_constraints = move_group_->getKnownConstraints();
  if (stored_constraints.empty())
    ROS_WARN("There are no constraints stored in database");
  else
  {
    ROS_INFO("Constraints currently stored in database:");
    for (const std::string& name : stored_constraints)
      ROS_INFO(" * %s", name.c_str());
  } */

  // Subscribe to RVIZ GUI topic
  rviz_sub = nh_.subscribe("/rviz_visual_tools_gui",10, &DeepGraspDemo::rviz_callback, this);



/*   // Gazebo link_attacher service
  gripper_client = nh_.serviceClient<ur_msgs::SetIO>("arm/ur_hardware_interface/set_io");
  gazebo_link_attacher_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/attach");
  gazebo_link_detacher_client = nh_.serviceClient<gazebo_ros_link_attacher::Attach>("/link_attacher_node/detach");
 */

  //UNCOMMENT FOR VISUALIZATION
  // visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("robot_base_footprint","/move_group/display_grasp_markers"));
  // visual_tools_->deleteAllMarkers();
  // visual_tools_->trigger();

  // TF Listener and Broadcaster
  /* tf_latch_timer = pnh_.createTimer(ros::Duration(0.1), std::bind(&ComponentSorting::tfLatchCallback, this)); */
  tf_listener = new  tf2_ros::TransformListener(tfBuffer);

  // Construct MTC deep pick and place task pipeline
  //intrepid_deep_grasp_demo::DeepPickPlaceTask deep_pick_place_task("deep_pick_place_task", nh_);
  deep_pick_place_task.reset(new intrepid_deep_grasp_demo::DeepPickPlaceTask("deep_pick_place_task", nh_));
  deep_pick_place_task->loadParameters();

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

  // Start pick object action server
  pick_object_as_->start();
  RCOMPONENT_INFO_STREAM("Started server: pickup object");

  // Initialize execute flag
  allow_execute = false;

  // Create planning scene
  if(create_planning_scene()){
    return rcomponent::OK;
  }else{
    return rcomponent::ERROR;
  }
}

void DeepGraspDemo::standbyState()
{

//UNCOMMENT FOR VISUALIZATION
/*   robot_state_ = move_group_->getCurrentState();
  joint_model_group= robot_state_->getJointModelGroup("arm"); */

  // Move to home position without selected constraints
  move_group_->detachObject();  
  move_group_->clearPathConstraints();
  move_group_->setMaxAccelerationScalingFactor(1.0);
  move_group_->setMaxVelocityScalingFactor(1.0);
  move_group_->setNamedTarget("Home");
  success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success_move){

    ROS_INFO("Moved to home position, ready to take commands");

    // Select constraint
    /* move_group_->setPathConstraints(moveit_constraint); */

/*     current_constraint = move_group_->getPathConstraints();

    if(moveit_constraint != current_constraint.name){
      ROS_ERROR("Desired moveit_constraint is not available in database, please modify or run generate_path_constraints.cpp");
      switchToState(robotnik_msgs::State::FAILURE_STATE);
      return;
    }
 */
    switchToState(robotnik_msgs::State::READY_STATE);

  }else{
    ROS_WARN("Could not move to home position");
  }

}

void DeepGraspDemo::readyState()
{ 

  if (pick_object_as_->isActive() == false)
  {
    ROS_INFO_THROTTLE(3, "I do not have a goal");
    return;
  }

  ROS_INFO_THROTTLE(3, "I have a new goal!");

/*   // Select constraint
  move_group_->setPathConstraints(moveit_constraint);
  current_constraint = move_group_->getPathConstraints();

  if(moveit_constraint != current_constraint.name){
    if(pickup_from_as_->isActive() == true){ 
      pick_result_.success = false;
      pick_result_.message = "Moveit constraint not available in database";
      pickup_from_as_->setAborted(pick_result_);
    }

    if(place_on_as_->isActive() == true){
      place_result_.success = false;
      place_result_.message = "Moveit constraint not available in database";
      place_on_as_->setAborted(place_result_);
    }
  
  }

  ROS_INFO("Planning with Constraint: %s", current_constraint.name.c_str());
 */

  if(pick_object_as_->isActive() == true){ 

      
   /*     // Get desired goal and set as target
    std::string pick_position = pickup_from_goal_->from;
    //Check if position exists
    if(find(positions_to_use.begin(), positions_to_use.end(),pick_position) != end(positions_to_use)){
      pick_chain_movement(pick_position);
    }else {
      ROS_WARN("Position %s does not exist", pick_position.c_str());
            
      pick_result_.success = false;
      pick_result_.message = "Position does not exist";
      pickup_from_as_->setAborted(pick_result_);
      return;
    } */
    deep_pick_place_task->init();

    move_group_->setNamedTarget("Look");
    success_move = (move_group_->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (!pick_object_as_->isActive()) return;

    if (deep_pick_place_task->plan())
    {
      RCOMPONENT_INFO_STREAM("Planning succeded");
/*       pick_object_result_.success = true;
      pick_object_result_.message = "Planning complete";
      pick_object_as_->setSucceeded(pick_object_result_);
      return; */
      if (pnh_.param("execute", false))
      {
        while(pick_object_as_->isActive()){
          if(allow_execute == true){ 
            allow_execute == false;
            if(deep_pick_place_task->execute()){
              RCOMPONENT_INFO_STREAM("Execution complete");
              pick_object_result_.success = true;
              pick_object_result_.message = "Execution complete";
              pick_object_as_->setSucceeded(pick_object_result_);
              return;
            }else{
              RCOMPONENT_INFO_STREAM("Execution failed");
              pick_object_result_.success = false;
              pick_object_result_.message = "Execution failed";
              pick_object_as_->setAborted(pick_object_result_);
              return;
            }
          }
          ros::Duration(1.0).sleep();
        }
      }
      else
      {
        RCOMPONENT_INFO_STREAM("Execution disabled");
        pick_object_result_.success = false;
        pick_object_result_.message = "Execution disabled";
        pick_object_as_->setAborted(pick_object_result_);
        return;
      }
    }
    else
    {
      RCOMPONENT_INFO_STREAM("Planning failed");
      pick_object_result_.success = false;
      pick_object_result_.message = "Planning failed";
      pick_object_as_->setAborted(pick_object_result_);
      return;
    }
        
  }   
}

void DeepGraspDemo::goalCB(const std::string& action)
{
  RCOMPONENT_INFO_STREAM("I have received an action to: " << action);
  action_ = action;
  if (action_ == "pickup_object"){
    pick_object_goal_ = pick_object_as_->acceptNewGoal();}
}

void DeepGraspDemo::preemptCB()
{
  RCOMPONENT_INFO_STREAM("ACTION: Preempted");
  //result_.success = false;
  //result_.message = "Goal has been cancelled, stopping execution.";
  // set the action state to preempted
  /* deep_pick_place_task.preemptTask(); */
  pick_object_as_->setPreempted();
}

bool DeepGraspDemo::create_planning_scene()
{

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
      transform_stamped = tfBuffer.lookupTransform(world_frame_,collision_object.header.frame_id,ros::Time(0),ros::Duration(1.0));
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

void DeepGraspDemo::rviz_callback(const sensor_msgs::Joy& msg){
  allow_execute = msg.buttons[2];
}