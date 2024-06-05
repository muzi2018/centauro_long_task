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
bool start_turn_bool = false;
const double dt = 0.01;
int turn_num = 100;
bool start_turn(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    start_turn_bool = !start_turn_bool;
    return true;
};


void TurnAround(XBot::Cartesian::CartesianTask* car_cartesian){
    Eigen::Vector6d E;
    double yaw_e = -1 * 3.14 * 1/5;
    E[0] = 0;
    E[1] = 0;
    E[2] = 0;
    E[3] = 0;
    E[4] = 0;
    E[5] = 0.2 * yaw_e;
    car_cartesian->setVelocityReference(E); 
    // num -- ;
}



int main(int argc, char **argv)
{
    const std::string robotName = "centauro";
    // Initialize ros node
    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle("");

    std_srvs::Empty srv;

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
    nodeHandle.getParam("problem_description", problem_description_string);
    auto ik_pb_yaml = YAML::Load(problem_description_string);
    XBot::Cartesian::ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // we are finally ready to make the CartesIO solver "OpenSot"
    auto solver = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_pb, ctx
                                                       );
    ros::ServiceServer service = nodeHandle.advertiseService("start_turn", start_turn);
    ros::Rate r(10);
    double time = 0 ;
    Eigen::VectorXd q, qdot, qddot;

    auto car_task = solver->getTask("base_link");
    auto car_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(car_task);
    while (ros::ok())
    {
        while (!start_turn_bool)
        {
            ros::spinOnce();
            r.sleep();
        }
        TurnAround(car_cartesian.get());
        solver->update(time, dt);
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
        time += dt;
        rspub.publishTransforms(ros::Time::now(), "");
        /**
         * Move Robot
        */
        ros::spinOnce();
        r.sleep();
        }        
}





