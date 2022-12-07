#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib_tutorials/SrGoalAction.h>
#include <chrono>

#include "behaviortree_cpp/bt_factory.h"

// Custom type
struct Pose
{
    double x, y, z, q1, q2, q3, q4;
};

namespace BT
{
// This template specialization is needed only if you want
// to AUTOMATICALLY convert a NodeParameter into a Pose2D
// In other words, implement it if you want to be able to do:
//
//   TreeNode::getInput<Pose2D>(key, ...)
//
template <> inline

Pose convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 7)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose output;
        output.x     = convertFromString<double>(parts[0]);
        output.y     = convertFromString<double>(parts[1]);
        output.z     = convertFromString<double>(parts[2]);
        output.q1    = convertFromString<double>(parts[3]);
        output.q2    = convertFromString<double>(parts[4]);
        output.q3    = convertFromString<double>(parts[5]);
        output.q4    = convertFromString<double>(parts[6]);
        return output;
    }
}
} // end namespace BT

namespace chr = std::chrono;

// This is an asynchronous operation
class MoveBaseAction : public BT::StatefulActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    MoveBaseAction(const std::string& name, const BT::NodeConfig& config)
      : StatefulActionNode(name, config), ac("/srgoal_server/srgoal_srv", true)
    {
      ROS_INFO("Waiting for action server to start.");
      // wait for the action server to start
      ac.waitForServer(); //will wait for infinite time
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return{ BT::InputPort<Pose>("goal") };
    }

    // this function is invoked once at the beginning.
    BT::NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

  private:
    Pose _goal;
    chr::system_clock::time_point _completion_time;
    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<actionlib_tutorials::SrGoalAction> ac;
};


BT::NodeStatus MoveBaseAction::onStart()
{
  if ( !getInput<Pose>("goal", _goal))
  {
    throw BT::RuntimeError("missing required input [goal]");
  }
  printf("[ MoveBase: SEND REQUEST ]. goal: x=%.1f y=%.1f z=%.1f q1=%.1f q2=%.1f q3=%.1f q4=%.1f\n",
         _goal.x, _goal.y, _goal.z,_goal.q1, _goal.q2, _goal.q3, _goal.q4);
 
  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::SrGoalGoal goal;
  goal.pos.position.x = _goal.x;
  goal.pos.position.y = _goal.y;
  goal.pos.position.z = _goal.z;
  goal.pos.orientation.x = _goal.q1;
  goal.pos.orientation.y = _goal.q2;
  goal.pos.orientation.z = _goal.q3;
  goal.pos.orientation.w = _goal.q4;
  ac.sendGoal(goal);

  // We use this counter to simulate an action that takes a certain
  // amount of time to be completed (220 ms)
  
  _completion_time = chr::system_clock::now() + chr::milliseconds(450);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBaseAction::onRunning()
{
  // Pretend that we are checking if the reply has been received
  // you don't want to block inside this function too much time.
  std::this_thread::sleep_for(chr::milliseconds(10));

  actionlib::SimpleClientGoalState state = ac.getState();

  // Pretend that, after a certain amount of time,
  // we have completed the operation
  if( state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std::cout << "[ MoveBase: FINISHED ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

  if( state == actionlib::SimpleClientGoalState::PREEMPTED)
  {
    std::cout << "[ MoveBase: PREEMTED ]" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

void MoveBaseAction::onHalted()
{
  printf("[ MoveBase: ABORTED ]");
}

class SaySomething : public BT::SyncActionNode
{
public:
  // If your Node has ports, you must use this constructor signature 
  SaySomething(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  { }

  // It is mandatory to define this STATIC method.
  static BT::PortsList providedPorts()
  {
    // This action has a single input port called "message"
    return { BT::InputPort<std::string>("message") };
  }

  // Override the virtual function tick()
  BT::NodeStatus tick() override
  {
    //Optional<std::string> msg = getInput<std::string>("message");
    auto msg = getInput<std::string>("message");
    // Check if optional is valid. If not, throw its error

    if (!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ", 
                              msg.error() );
    }
    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
};


 // Simple function that return a NodeStatus
BT::NodeStatus CheckBattery()
{
  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}

/*
int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<actionlib_tutorials::FibonacciAction> ac("fibonacci", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  actionlib_tutorials::FibonacciGoal goal;
  goal.order = 20;
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}
*/

using namespace BT;

 int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_srgoal");
  BehaviorTreeFactory factory;
  factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
  factory.registerNodeType<MoveBaseAction>("MoveBase");
  factory.registerNodeType<SaySomething>("SaySomething");

  
  factory.registerBehaviorTreeFromFile("/home/agan/ws_moveit/src/actionlib_tutorials/launch/xml_tree.xml");
  auto tree = factory.createTree("MainTree");
 
    NodeStatus status = NodeStatus::IDLE;
#if 0
    // Tick the root until we receive either SUCCESS or RUNNING
    // same as: tree.tickRoot(Tree::WHILE_RUNNING)
    std::cout << "--- ticking\n";
    status = tree.tickWhileRunning();
    std::cout << "--- status: " << toStr(status) << "\n\n";
#else
    // If we need to run code between one tick() and the next,
    // we can implement our own while loop
    while (status != NodeStatus::SUCCESS)
    {
      std::cout << "--- ticking\n";
      status = tree.tickOnce();
      std::cout << "--- status: " << toStr(status) << "\n\n";
      
      // if failure, add some wait time
      if (status == NodeStatus::FAILURE)
      {
        std::cout << "--- aborted\n";
        break;
      }
      // if still running, add some wait time
      if (status == NodeStatus::RUNNING)
      {
        tree.sleep(std::chrono::milliseconds(100));
      }
    }
#endif

  return 0;
}
