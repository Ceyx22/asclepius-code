#include <stdio.h>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "kdl/tree.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "kdl_parser/kdl_parser.hpp"

using std::placeholders::_1;

class KinematicPublisher : public rclcpp::Node
{
public:
    KinematicPublisher() : Node("kinematic_publisher")
    {

        // Set up subscriber for joint states
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "robot_description", rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL), 
            std::bind(&KinematicPublisher::robotDescriptionCallback, this, _1));
    }

private:

    void robotDescriptionCallback(const std_msgs::msg::String& msg)
    {
        // Construct KDL tree from URDF
        const std::string urdf = msg.data;
        kdl_parser::treeFromString(urdf, tree_);
        // Print basic information about the tree
        std::cout << "nb joints:        " << tree_.getNrOfJoints()         << std::endl;
        std::cout << "nb segments:      " << tree_.getNrOfSegments()       << std::endl;
        std::cout << "root segment:     " << tree_.getRootSegment()->first << std::endl;
    }

    //! Cartesian x, z => hip and knee joint angles
    void getJointAngles(
        const double x, const double z, double & hip_angle, double & knee_angle)
    {
        // Prepare IK solver input variables
        KDL::JntArray q_init(chain_.getNrOfJoints());
        q_init(0) = -0.1;
        q_init(1) = 0.2;
        const KDL::Frame p_in(KDL::Vector(x, 0, z));
        KDL::JntArray q_out(chain_.getNrOfJoints());
        // Run IK solver
        solver_->CartToJnt(q_init, p_in, q_out);
        // Write out
        hip_angle = q_out(0);
        knee_angle = q_out(1);
    }

  
    // Calculate and print the joint angles
    // for moving the foot to {x: 0.3, z: -0.6}
    // void usageExample()
    // {
    //     double hip_angle, knee_angle;
    //     getJointAngles(0.3, -0.6, hip_angle, knee_angle);
    //     printf("knee and hip joint angles: %.1f, %.1f °\n",
    //         hip_angle*180/M_PI, knee_angle*180/M_PI);
    //     fflush(stdout);
    // }

    // Class members
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    KDL::Tree tree_;
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> solver_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KinematicPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
