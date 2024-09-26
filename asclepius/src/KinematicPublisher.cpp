#include <stdio.h>
#include <string>
#include <memory>
#include <vector>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl_parser/kdl_parser.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class KinematicPublisher : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    KinematicPublisher() : Node("kinematic_publisher")
    {
        // Declare parameters
        this->declare_parameter<std::string>("base_link", "base_link");
        this->declare_parameter<std::string>("tip_link", "link6");
        this->declare_parameter<std::string>("controller_name", "joint_trajectory_controller");

        // Get parameters
        base_link_ = this->get_parameter("base_link").as_string();
        tip_link_ = this->get_parameter("tip_link").as_string();
        controller_name_ = this->get_parameter("controller_name").as_string();

        // Set up subscriber for robot_description
        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "robot_description", qos,
            std::bind(&KinematicPublisher::robotDescriptionCallback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Waiting for URDF on /robot_description...");

        // Initialize variables
        urdf_received_ = false;

        // Create action client
        std::string action_name = "/" + controller_name_ + "/follow_joint_trajectory";
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, action_name);

        // Wait for the action server to be available
        if (!action_client_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Action server available.");
        }
    }

private:
    void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (urdf_received_)
            return; // Process URDF only once

        // Construct KDL tree from URDF
        const std::string urdf = msg->data;
        if (!kdl_parser::treeFromString(urdf, tree_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            return;
        }

        // Print basic information about the tree
        RCLCPP_INFO(this->get_logger(), "Number of joints: %d", tree_.getNrOfJoints());
        RCLCPP_INFO(this->get_logger(), "Number of segments: %d", tree_.getNrOfSegments());
        RCLCPP_INFO(this->get_logger(), "Root segment: %s", tree_.getRootSegment()->first.c_str());

        // Get KDL chain from base_link to tip_link
        if (!tree_.getChain(base_link_, tip_link_, chain_))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from %s to %s", base_link_.c_str(), tip_link_.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Chain number of joints: %d", chain_.getNrOfJoints());

        // Create IK solver
        solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);

        // Get joint names from the chain
        joint_names_.clear();
        for (const auto &segment : chain_.segments)
        {
            if (segment.getJoint().getType() != KDL::Joint::None)
            {
                joint_names_.push_back(segment.getJoint().getName());
            }
        }

        urdf_received_ = true;

        // Run usage example
        usageExample();
    }

    void getJointAngles()
    {
        KDL::JntArray q_init(chain_.getNrOfJoints());
        for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i)
        {
            q_init(i) = 0.0; // Initial guess for joint positions
        }

        // Desired end-effector position (modify as needed)
        KDL::Frame p_in(KDL::Vector(0.5, 0.0, 0.5));

        KDL::JntArray q_out(chain_.getNrOfJoints());

        // Run IK solver
        int ret = solver_->CartToJnt(q_init, p_in, q_out);
        if (ret >= 0)
        {
            // Write out joint positions
            RCLCPP_INFO(this->get_logger(), "IK solver succeeded. Joint positions:");
            for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i)
            {
                RCLCPP_INFO(this->get_logger(), "  %s: %f", joint_names_[i].c_str(), q_out(i));
            }

            // Send joint trajectory as an action goal
            sendJointTrajectoryAction(q_out);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "IK solver failed");
        }
    }

    void sendJointTrajectoryAction(const KDL::JntArray &joint_positions)
    {
        // Create a FollowJointTrajectory goal message
        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory.joint_names = joint_names_;

        // Create a trajectory point
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(joint_positions.rows());
        for (unsigned int i = 0; i < joint_positions.rows(); ++i)
        {
            point.positions[i] = joint_positions(i);
        }
        // Set the desired time to reach the point
        point.time_from_start = rclcpp::Duration::from_seconds(5.0); // 5 seconds to reach the target

        // Add the point to the trajectory
        goal_msg.trajectory.points.push_back(point);

        // Send the goal
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&KinematicPublisher::goalResponseCallback, this, _1);
        send_goal_options.feedback_callback = std::bind(&KinematicPublisher::feedbackCallback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&KinematicPublisher::resultCallback, this, _1);

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // Updated function signature
    void goalResponseCallback(GoalHandleFollowJointTrajectory::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedbackCallback(GoalHandleFollowJointTrajectory::SharedPtr,
                          const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback)
    {
        // RCLCPP_INFO(this->get_logger(), feedback);
        // You can process feedback here if needed
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }

    void resultCallback(const GoalHandleFollowJointTrajectory::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
        }
        rclcpp::shutdown();
    }

    void usageExample()
    {
        getJointAngles();
    }

    // Class members
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;

    KDL::Tree tree_;
    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> solver_;

    std::vector<std::string> joint_names_;
    std::string base_link_;
    std::string tip_link_;
    std::string controller_name_;

    bool urdf_received_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinematicPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
