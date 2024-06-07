#include <thread>
#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/TransformStamped.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <std_srvs/Empty.h>
#include <xbot_msgs/JointCommand.h>

using namespace XBot::Cartesian;
bool start_searching_bool = false, tagDetected = false;
const double dt = 0.01;
double time_ = 0.0 ;
Eigen::Vector6d E;
bool start_searching(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    start_searching_bool = !start_searching_bool;
    return true;
};

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    if (msg->detections.size() > 0) {
        tagDetected = true;
    }else{
        tagDetected = false;
    }
}



int main(int argc, char **argv)
{
    const std::string robotName = "centauro";
    // Initialize ros node
    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle("");

    std_srvs::Empty srv;
    // Create a Buffer and a TransformListener
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    auto cfg = XBot::ConfigOptionsFromParamServer();
    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(cfg);
    auto robot = XBot::RobotInterface::getRobot(cfg);
    // initialize to a homing configuration
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    XBot::Cartesian::Utils::RobotStatePublisher rspub (model);
    robot->setControlMode(
        {
            {"j_wheel_1", XBot::ControlMode::Velocity()},
            {"j_wheel_2", XBot::ControlMode::Velocity()},
            {"j_wheel_3", XBot::ControlMode::Velocity()},
            {"j_wheel_4", XBot::ControlMode::Velocity()}
        }
    );

    auto ctx = std::make_shared<XBot::Cartesian::Context>(
                std::make_shared<XBot::Cartesian::Parameters>(dt),
                model
            );

    // load the ik problem given a yaml file
    std::string problem_description_string;
    nodeHandle.getParam("problem_description_wheel", problem_description_string);
    auto ik_pb_yaml = YAML::Load(problem_description_string);
    XBot::Cartesian::ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // we are finally ready to make the CartesIO solver "OpenSot"
    auto solver = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_pb, ctx
                                                       );
    ros::Subscriber sub = nodeHandle.subscribe("/tag_detections", 1000, tagDetectionsCallback);
    ros::ServiceServer service = nodeHandle.advertiseService("start_searching", start_searching);
    ros::Rate r(10);



    double roll_e, pitch_e, yaw_e;
    double K_x = 0.1, K_y = 0.2, K_yaw = 0.1;
    
    //frame name
    std::string parent_frame = "base_link";
    std::string child_frame = "tag_0";

    Eigen::VectorXd q, qdot, qddot;
    // tag to base translation
    geometry_msgs::TransformStamped tag_base_T; 
    auto car_task = solver->getTask("base_link");
    auto car_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(car_task);

    while (ros::ok())
    {
       
        if (tagDetected)
        {
            // std::cout << "tagDetected = " << tagDetected << std::endl;
            tag_base_T = tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
            /**
             * Error Calculate
            */
            double x_e = tag_base_T.transform.translation.x;
            double y_e = tag_base_T.transform.translation.y ;
            tf2::Quaternion q_;
            q_.setW(tag_base_T.transform.rotation.w);
            q_.setX(tag_base_T.transform.rotation.x);
            q_.setY(tag_base_T.transform.rotation.y);
            q_.setZ(tag_base_T.transform.rotation.z);
            tf2::Matrix3x3 m(q_);
            m.getRPY(roll_e, pitch_e, yaw_e);
            yaw_e = yaw_e + 1.6;
            /**
             * Velocity Controller
            */
            E[0] = K_x * x_e;
            E[1] = K_y * y_e;
            E[2] = 0;
            E[3] = 0;
            E[4] = 0;
            E[5] = K_yaw * 0;
            /**
             * 2.1 large error need to move it to tag
            */
            if ((abs(x_e) > 1 || abs(y_e) > 0.4 ))
            {                
                // std::cout << "x_e = " << x_e << std::endl;
                // std::cout << "y_e = " << y_e << std::endl;
                car_cartesian->setVelocityReference(E);
            } else {
                E.setZero();
                car_cartesian->setVelocityReference(E);
            }

            solver->update(time_, dt);
            model->getJointPosition(q);
            model->getJointVelocity(qdot);
            model->getJointAcceleration(qddot);
            q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
            qdot += dt * qddot;
            model->setJointPosition(q);
            model->setJointVelocity(qdot);
            model->update();
            robot->setPositionReference(q.tail(robot->getJointNum()));
            robot->setVelocityReference(qdot.tail(robot->getJointNum()));
            robot->move();
            time_ += dt;
            rspub.publishTransforms(ros::Time::now(), "");
            /**
             * Move Robot
            */

         }
        

            ros::spinOnce();
            r.sleep();
        }        
}





