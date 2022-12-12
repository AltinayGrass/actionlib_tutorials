#include <ros/ros.h>
#include <memory>
// MoveitCpp
#include <moveit/moveit_cpp/moveit_cpp.h>
#include <moveit/moveit_cpp/planning_component.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/SrGoalAction.h>

class SrGoalAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<actionlib_tutorials::SrGoalAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  ros::Subscriber sub; //override speed listener
  // create messages that are used to published feedback/result
  actionlib_tutorials::SrGoalFeedback feedback_;
  actionlib_tutorials::SrGoalResult result_;
  moveit_cpp::MoveItCppPtr moveit_cpp_ptr;
  moveit_cpp::PlanningComponentPtr planning_components;
  moveit::core::RobotStatePtr robot_start_state;
  moveit::core::RobotModelConstPtr robot_model_ptr;
  const moveit::core::JointModelGroup *joint_model_group_ptr;

public:

  SrGoalAction(std::string name) :
    nh_("/srgoal_server"),
    as_(nh_, name, boost::bind(&SrGoalAction::executeCB, this, _1), false),
    action_name_(name)
  {
    ros::Duration(1.0).sleep();
    ROS_INFO_STREAM_NAMED(LOGNAME, "Starting SrGoal Action Server...");
    moveit_cpp_ptr = std::make_shared<moveit_cpp::MoveItCpp>(nh_);
    moveit_cpp_ptr->getPlanningSceneMonitor()->providePlanningSceneService();
    planning_components = std::make_shared<moveit_cpp::PlanningComponent>(PLANNING_GROUP, moveit_cpp_ptr);
    robot_model_ptr = moveit_cpp_ptr->getRobotModel();
    robot_start_state = planning_components->getStartState();
    joint_model_group_ptr = robot_model_ptr->getJointModelGroup(PLANNING_GROUP);
    sub = nh_.subscribe("overide",1000,&SrGoalAction::overrideCallback, this);
    // We can also generate motion plans around objects in the collision scene.
    //
    // First we create the collision object
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "base_link";
    collision_object.id = "box";

    shape_msgs::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = { 0.1, 0.4, 0.1 };

    geometry_msgs::Pose box_pose;
    box_pose.position.x = 1.4;
    box_pose.position.y = 0.5;
    box_pose.position.z = 1.2;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add object to planning scene
    {  // Lock PlanningScene
      planning_scene_monitor::LockedPlanningSceneRW scene(moveit_cpp_ptr->getPlanningSceneMonitor());
      scene->processCollisionObjectMsg(collision_object);
    }  // Unlock PlanningScene
  
    planning_components->setStartStateToCurrentState();
    as_.start();
  }

  ~SrGoalAction(void)
  {
  }

  void executeCB(const actionlib_tutorials::SrGoalGoalConstPtr &goal)
  {

    bool success = true;
    planning_components->setStartStateToCurrentState();
    
    // at the beginning noerror 
    feedback_.noerror=true;

 // publish info to the console for the user
    ROS_INFO("%s: Executing, creating srgoal sequence of goal {x:%.2f y:%.2f z:%.2f q1:%.2f q2:%.2f q3:%.3f q4:%.3f  override:%d} with default velocity", action_name_.c_str(), 
              goal->pos.position.x,
              goal->pos.position.y,
              goal->pos.position.z, 
              goal->pos.orientation.x,
              goal->pos.orientation.y,
              goal->pos.orientation.z,
              goal->pos.orientation.w,
              m_override);

    // The first way to set the goal of the plan is by using geometry_msgs::PoseStamped ROS message type as follow
    geometry_msgs::PoseStamped target_pose1;
    target_pose1.header.frame_id = "base_link";
    target_pose1.pose.orientation.x = goal->pos.orientation.x;
    target_pose1.pose.orientation.y = goal->pos.orientation.y;
    target_pose1.pose.orientation.z = goal->pos.orientation.z;
    target_pose1.pose.orientation.w = goal->pos.orientation.w;
    
    target_pose1.pose.position.x = goal->pos.position.x;
    target_pose1.pose.position.y = goal->pos.position.y;
    target_pose1.pose.position.z = goal->pos.position.z;
    planning_components->setGoal(target_pose1, "flange");
    
    moveit_cpp::PlanningComponent::PlanRequestParameters params;// = {"","ompl",1,0.1,goal->vel/100.0,goal->acc/100.0};
    params.max_acceleration_scaling_factor=(goal->acc/100.0)*((double) m_override/100.0);
    params.max_velocity_scaling_factor=(goal->vel/100.0)*((double) m_override/100.0);
    params.planning_attempts=1;
    params.planning_pipeline="ompl";
    params.planning_time=0.1;
    params.planner_id="";
    
    // Now, we call the PlanningComponents to compute the plan and visualize it.
    // Note that we are just planning
    auto plan_solution = planning_components->plan(params);

    // Check if PlanningComponents succeeded in finding the plan
    if (plan_solution)
    {
      /* Uncomment if you want to execute the plan */
      planning_components->execute(); // Execute the plan */
    }

    // check that preempt has not been requested by the client
    if (as_.isPreemptRequested() || !ros::ok() || !plan_solution)
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      as_.setPreempted();
      success = false;
    }
    feedback_.noerror=success;
    // publish the feedback
    as_.publishFeedback(feedback_);
    // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes

    if(success)
    {
      result_.done = feedback_.noerror;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }
private:
  
  void overrideCallback(const std_msgs::UInt8::ConstPtr& ovr)
  
  {
    m_override=ovr->data;
  }

  uint m_override = 100;
  std::string PLANNING_GROUP = "manipulator";
  std::string LOGNAME = "srgoal_server_node";
};


int main(int argc, char** argv)
{
  int i;
  ros::init(argc, argv, "srgoal_srv");
  SrGoalAction srgoal("srgoal_srv");
  ros::spin();
  return 0;
}