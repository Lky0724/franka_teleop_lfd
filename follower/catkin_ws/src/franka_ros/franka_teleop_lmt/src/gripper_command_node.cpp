#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <std_msgs/Float64MultiArray.h> // CHANGED: New message type

using MoveClient = actionlib::SimpleActionClient<franka_gripper::MoveAction>;
using GraspClient = actionlib::SimpleActionClient<franka_gripper::GraspAction>;
using HomingClient = actionlib::SimpleActionClient<franka_gripper::HomingAction>;

class GripperCommander {
public:
    GripperCommander(ros::NodeHandle& nh) 
        : move_client_("/franka_gripper/move", true),
          grasp_client_("/franka_gripper/grasp", true),
          homing_client_("/franka_gripper/homing", true)
    {
        ROS_INFO("Waiting for gripper action servers...");
        move_client_.waitForServer();
        grasp_client_.waitForServer();
        homing_client_.waitForServer();
        ROS_INFO("Gripper action servers connected!");

        // Subscribe to Float64MultiArray
        sub_ = nh.subscribe("/cartesian_impedance_example_controller/LFcommand", 1, 
                            &GripperCommander::commandCallback, this);
    }

    void commandCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
        // Safety check: Ensure the array has enough elements
        if (msg->data.size() <= 6) {
            ROS_ERROR_THROTTLE(2.0, "Received array is too short! Expected at least 7 elements.");
            return;
        }

        // Extract the 7th element (index 6) and cast to integer
        int current_trigger_state = static_cast<int>(msg->data[6]);

        // CRITICAL: Edge Detection
        // We only send a command if the state has CHANGED from the last time.
        // This prevents flooding the action server with 1000 goals per second.
        if (current_trigger_state != last_trigger_state_) {
            
            ROS_INFO("Gripper state change detected: %d -> %d", last_trigger_state_, current_trigger_state);

            if (current_trigger_state == 1) {
                // Trigger == 1 -> GRASP OBJECT
                triggerGrasp();
            } 
            else if (current_trigger_state == 0) {
                // Trigger == 0 -> OPEN/RELEASE
                triggerMoveOpen();
            }
            
            // Update the state tracker
            last_trigger_state_ = current_trigger_state;
        }
    }

private:
    MoveClient move_client_;
    GraspClient grasp_client_;
    HomingClient homing_client_;
    ros::Subscriber sub_;
    
    // State tracker (init to -1 so the first message always triggers an action)
    int last_trigger_state_ = -1; 

    // --- PRESET PARAMETERS ---
    const double GRASP_FORCE = 40.0;    // Newtons
    const double GRASP_SPEED = 0.2;     // m/s
    const double OPEN_WIDTH  = 0.06;    // m (Max width for Panda)
    const double OBJECT_WIDTH = 0.04;   // m (Expected object size, rarely 0)

    void triggerMoveOpen() {
        franka_gripper::MoveGoal goal;
        goal.width = OPEN_WIDTH; 
        goal.speed = GRASP_SPEED;
        move_client_.sendGoal(goal);
        ROS_INFO("Sent OPEN command.");
    }

    void triggerGrasp() {
        franka_gripper::GraspGoal goal;
        goal.width = OBJECT_WIDTH;  // Expected object size (if unknown, set small like 0.01)
        goal.speed = GRASP_SPEED;
        goal.force = GRASP_FORCE;
        
        // Tolerances required for successful grasp feedback
        goal.epsilon.inner = 0.04; 
        goal.epsilon.outer = 0.04;

        grasp_client_.sendGoal(goal);
        ROS_INFO("Sent GRASP command.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "gripper_command_node");
    ros::NodeHandle nh;
    GripperCommander commander(nh);
    ros::spin();
    return 0;
}