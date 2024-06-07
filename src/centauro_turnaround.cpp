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
# define Offset_yaw 3.14/2
using namespace XBot::Cartesian;
bool start_searching_bool = false;
bool tagDetected = false;
int direction = 1; // -1: right, 1: left
double offset_yaw = Offset_yaw;
double roll_e, pitch_e, yaw_e;

const double dt = 0.01;
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

void TurnAround(XBot::Cartesian::CartesianTask* car_cartesian){
    Eigen::Vector6d E;
    if (!tagDetected)
    {
        double yaw_e_ = direction * 1 * 3.14 * 1/5;
        E[0] = 0;
        E[1] = 0;
        E[2] = 0;
        E[3] = 0;
        E[4] = 0;
        E[5] = 0.2 * yaw_e_;
        car_cartesian->setVelocityReference(E); 
        offset_yaw = Offset_yaw;
    }else{

        E[0] = 0;
        E[1] = 0;
        E[2] = 0;
        E[3] = 0;
        E[4] = 0;
        E[5] = 0.2 * (offset_yaw);
        offset_yaw -= 0.02 ;
        if (abs(offset_yaw) < 1.3)
        {
            offset_yaw = 0;
            E.setZero();
        }
        car_cartesian->setVelocityReference(E);
        // std::cout << "offset_yaw " << offset_yaw << std::endl;
        // std::cout << "1 * offset_yaw = " << 1 * offset_yaw << std::endl;
        // std::cout << "E[5]" << E[5] << std::endl;
    }
    

}

int main(int argc, char **argv)
{
    if (argc > 1)
    {
        direction = atoi(argv[1]);
    }
    
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

    // time
    
    double time = 0 ;
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
        // while (!start_searching_bool)
        // {
        //     // std::cout << "start_searching_bool: " << start_searching_bool << std::endl;
        //     ros::spinOnce();
        //     r.sleep();
        // }
        if (tagDetected)
        {
            tag_base_T = tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0));

            tf2::Quaternion q_;
            q_.setW(tag_base_T.transform.rotation.w);
            q_.setX(tag_base_T.transform.rotation.x);
            q_.setY(tag_base_T.transform.rotation.y);
            q_.setZ(tag_base_T.transform.rotation.z);
            tf2::Matrix3x3 m(q_);
            m.getRPY(roll_e, pitch_e, yaw_e);
            yaw_e = yaw_e + 1.6;
            // std::cout << "yaw: " << yaw_e << std::endl;
        }
        
        TurnAround(car_cartesian.get());
        std::cout << "yaw: " << yaw_e << std::endl;

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





