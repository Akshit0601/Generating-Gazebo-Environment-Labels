#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gazebo/physics/Joint.hh>
#include <cstdlib>
//#include <unistd.h>
#include <gazebo_msgs/msg/model_state.hpp>
#include <ignition/math/Vector3.hh>
namespace gazebo
{
    class CatSteering : public ModelPlugin
    {
        
        public:
            CatSteering();
            void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

        private:
            void CatVehicleSimROSThread();
            void modelRead(const gazebo_msgs::ModelStates::ConstPtr& msg);

            physics::PhysicsEnginePtr physicsEngine;
            std::string robotNamespace;
            std::string tfScope;
            std::string speedTopic;
	        std::string tireTopic;
            std::string odomTopic;
            //ROS
            // ros::Subscriber sub_;
            rclcpp::Subscription sub_;

            ros::Publisher odom_pub;
            boost::thread ros_spinner_thread_;
            ros::NodeHandle* rosnode_;
            
	    ignition::math::Vector3<double> linear_vel;
	    ignition::math::Vector3<double> angular_vel;
            physics::JointPtr steering_joints[2];
            physics::JointController *j_cont;
            event::ConnectionPtr updateConnection;


            physics::ModelPtr model;
            physics::WorldPtr world;


            double updateRate;
            ros::Time prevUpdateTime;
    };
}