#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trac_ik/trac_ik.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

class IKSolverNode : public rclcpp::Node
{
public:
  IKSolverNode() : Node("ik_solver_node")
  {
    RCLCPP_INFO(this->get_logger(), "C++ TRAC-IK Solver Node started");
    
    this->declare_parameter("chain_start", "base_link");
    this->declare_parameter("chain_end", "tool0");
    this->declare_parameter("timeout", 0.05);
    
    std::string chain_start = this->get_parameter("chain_start")->as_string();
    std::string chain_end = this->get_parameter("chain_end")->as_string();
    double timeout = this->get_parameter("timeout")->as_double();
    
    // Initialize TRAC-IK
    tracik = std::make_unique<TRAC_IK::TRAC_IK>(
      chain_start, chain_end, "robot_description", timeout, 1e-5
    );
    
    KDL::Chain chain;
    KDL::JntArray ll, ul;
    
    if (!tracik->getKDLChain(chain) || !tracik->getKDLLimits(ll, ul)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to init KDL chain from URDF");
    } else {
      RCLCPP_INFO(this->get_logger(), "TRAC-IK initialized successfully with %d joints", chain.getNrOfJoints());
    }
    
    sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&IKSolverNode::jointStateCallback, this, std::placeholders::_1)
    );
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = msg;
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_;
  std::unique_ptr<TRAC_IK::TRAC_IK> tracik;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IKSolverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
