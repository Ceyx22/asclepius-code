#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <asclepius/TransformationHelpers.hpp>

// Define the joint types
enum class JointType {
    FIXED,
    REVOLUTE,
    LINEAR
};

// // Placeholder helper functions (you need to define these properly based on your system's requirements)
// Eigen::Matrix4d T_from_URDF_origin(const urdf::Pose& origin);
// Eigen::Vector3d e_from_URDF_axis(const urdf::Vector3& axis);
// Eigen::Matrix4d T_from_Rp(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);
// Eigen::Matrix3d Rote(const Eigen::Vector3d& axis, double angle);
// Eigen::Matrix3d Reye();
// Eigen::Vector3d pzero();
// Eigen::Vector3d p_from_T(const Eigen::Matrix4d& T);
// Eigen::Matrix3d R_from_T(const Eigen::Matrix4d& T);
// Eigen::Vector3d cross(const Eigen::Vector3d& a, const Eigen::Vector3d& b);

// Define a single step in the kinematic chain
class KinematicStep {
public:
    KinematicStep(const Eigen::Matrix4d& Tshift, const Eigen::Vector3d& elocal, JointType type, const std::string& name)
        : Tshift(Tshift), elocal(elocal), type(type), name(name), dof(-1) {
        clear();
    }

    void clear() {
        T.setZero();
        p.setZero();
        R.setZero();
        e.setZero();
    }

    Eigen::Matrix4d Tshift;
    Eigen::Vector3d elocal;
    JointType type;
    std::string name;
    int dof;

    Eigen::Matrix4d T;
    Eigen::Vector3d p;
    Eigen::Matrix3d R;
    Eigen::Vector3d e;
    
    static KinematicStep FromRevoluteJoint(urdf::JointConstSharedPtr joint) {
        return KinematicStep(T_from_URDF_origin(joint->parent_to_joint_origin_transform),
                             e_from_URDF_axis(joint->axis),
                             JointType::REVOLUTE, joint->name);
    }

    static KinematicStep FromLinearJoint(urdf::JointConstSharedPtr joint) {
        return KinematicStep(T_from_URDF_origin(joint->parent_to_joint_origin_transform),
                             e_from_URDF_axis(joint->axis),
                             JointType::LINEAR, joint->name);
    }

    static KinematicStep FromFixedJoint(urdf::JointConstSharedPtr joint) {
        return KinematicStep(T_from_URDF_origin(joint->parent_to_joint_origin_transform),
                             Eigen::Vector3d::Zero(),
                             JointType::FIXED, joint->name);
    }
};

// Define the full kinematic chain
class KinematicChain {
public:
    KinematicChain(std::shared_ptr<rclcpp::Node> node, const std::string& baseframe, const std::string& tipframe, const std::vector<std::string>& expectedjointnames)
        : node(node), dofs(0) {
        load(baseframe, tipframe, expectedjointnames);
    }

    void load(const std::string& baseframe, const std::string& tipframe, const std::vector<std::string>& expectedjointnames) {
        RCLCPP_INFO(node->get_logger(), "Waiting for the URDF to be published...");

        std::string urdf;
        auto callback = [&urdf](const std_msgs::msg::String::SharedPtr msg) {
            urdf = msg->data;
        };

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        auto sub = node->create_subscription<std_msgs::msg::String>("/robot_description", qos, callback);

        rclcpp::Rate rate(10);  // 10 Hz
        while (urdf.empty() && rclcpp::ok()) {
            rclcpp::spin_some(node);
            rate.sleep();
        }

        if (urdf.empty()) {
            throw std::runtime_error("Failed to retrieve URDF");
        }

        urdf::Model robot;
        if (!robot.initString(urdf)) {
            throw std::runtime_error("Failed to parse URDF");
        }

        RCLCPP_INFO(node->get_logger(), "Processing URDF for robot '%s'", robot.getName().c_str());

        std::vector<std::string> jointnames;
        std::string frame = tipframe;
        while (frame != baseframe) {
            auto joint = robot.getJoint(frame);
            if (!joint) {
                throw std::runtime_error("Unable to find joint connecting to " + frame);
            }

            frame = joint->parent_link_name;

            if (joint->type == urdf::Joint::REVOLUTE || joint->type == urdf::Joint::CONTINUOUS) {
                steps.emplace_back(KinematicStep::FromRevoluteJoint(joint));
            } else if (joint->type == urdf::Joint::PRISMATIC) {
                steps.emplace_back(KinematicStep::FromLinearJoint(joint));
            } else if (joint->type == urdf::Joint::FIXED) {
                steps.emplace_back(KinematicStep::FromFixedJoint(joint));
            } else {
                throw std::runtime_error("Unknown joint type");
            }

            // Collect joint name for comparison
            if (joint->type != urdf::Joint::FIXED) {
                jointnames.push_back(joint->name);
            }
        }

        int dof = 0;
        for (auto& step : steps) {
            if (step.type != JointType::FIXED) {
                step.dof = dof++;
            }
        }
        dofs = dof;

        // Validate joint names with the expected joint names
        if (jointnames != expectedjointnames) {
            throw std::runtime_error("Joint names do not match expected names");
        }

        RCLCPP_INFO(node->get_logger(), "URDF has %zu steps, %d active DOFs", steps.size(), dofs);
    }

    void fkin(const Eigen::VectorXd& q, Eigen::Vector3d& ptip, Eigen::Matrix3d& Rtip) {
        if (q.size() != dofs) {
            throw std::runtime_error("Mismatch between joint angles and DOFs");
        }
        for (auto& step : steps){
            step.clear();
        }

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

        for (auto& step : steps) {
            T = T * step.Tshift;

            if (step.type == JointType::REVOLUTE) {
                T = T * T_from_Rp(Rote(step.elocal, q(step.dof)), pzero());
            } else if (step.type == JointType::LINEAR) {
                T = T * T_from_Rp(Reye(), step.elocal * q(step.dof));
            }

            step.T = T;
            step.p = p_from_T(T);
            step.R = R_from_T(T);
            step.e = R_from_T(T) * step.elocal;
        }

        ptip = p_from_T(T);
        Rtip = R_from_T(T);

        Eigen::MatrixXd Jv(3, dofs), Jw(3, dofs);
        Jv.setZero();
        Jw.setZero();

        for (const auto& step : steps) {
            if (step.type == JointType::REVOLUTE) {
                Jv.col(step.dof) = cross(step.e, ptip - step.p);
                Jw.col(step.dof) = step.e;
            } else if (step.type == JointType::LINEAR) {
                Jv.col(step.dof) = step.e;
                Jw.col(step.dof).setZero();
            }
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node;
    std::vector<KinematicStep> steps;
    int dofs;
};

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("kintest");

    std::vector<std::string> jointnames = {"theta1", "theta2", "theta3"};
    std::string baseframe = "world";
    std::string tipframe = "tip";

    KinematicChain chain(node, baseframe, tipframe, jointnames);

    auto test = [&chain](const Eigen::VectorXd& q) {
        Eigen::Vector3d ptip;
        Eigen::Matrix3d Rtip;
        chain.fkin(q, ptip, Rtip);

        std::cout << "q:\n" << q << std::endl;
        std::cout << "ptip(q):\n" << ptip << std::endl;
        std::cout << "Rtip(q):\n" << Rtip << std::endl;
        std::cout << "----------------------------------------" << std::endl;
    };

    Eigen::VectorXd q1(3), q2(3), q3(3);
    q1 << M_PI/9, M_PI/4.5, -M_PI/6;
    q2 << M_PI/6, M_PI/6, M_PI/3;
    q3 << -M_PI/4, 5*M_PI/12, 2*M_PI/3;

    test(q1);
    test(q2);
    test(q3);

    rclcpp::shutdown();
    return 0;
}
