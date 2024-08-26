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
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h> 

using namespace XBot::Cartesian;
bool start_searching_bool = false, tagDetected = false;
const double dt = 0.01;
double time_ = 0.0 ;
Eigen::Vector6d E;
int direction = 1; // 1 : forward; -1 : backward

bool start_searching(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    start_searching_bool = !start_searching_bool;
    return true;
};

void tagDetectionsCallback(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{
    if (msg->detections.size() > 0) {
        tagDetected = true;
    } else {
        tagDetected = false;
    }
}

bool backward_bool = false;
void backwardCallback(const std_msgs::Bool::ConstPtr& msg)
{
    backward_bool = msg->data;

    if (backward_bool) {
        std::cout << "backward started" << std::endl;
    } else {
        std::cout << "backward stopped" << std::endl;
    }
}


void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    // Handle the received path message here
    ROS_INFO("Received a path with %lu poses", msg->poses.size());
    // You can access the poses like this:
    for (const auto& pose : msg->poses) {
        ROS_INFO("Pose: [%.2f, %.2f, %.2f]", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }
}


int main(int argc, char **argv)
{
    if (argc > 1)
    {
        direction = atoi(argv[1]);
    }
    const std::string robotName = "centauro_wheel";
    // Initialize ros node
    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle("");

    std_srvs::Empty srv;
    // Create a Buffer and a TransformListener
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);

    auto cfg = XBot::ConfigOptionsFromParamServer();
    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(cfg);
    auto robot = XBot::RobotInterface::getRobot(cfg);
    // initialize to a homing configuration
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    // qhome[44] = -0.5;
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

    // ros::Subscriber sub_backward = nodeHandle.subscribe("/open_flag", 1, backwardCallback);
    ros::Subscriber sub_path = nodeHandle.subscribe("/path", 10 , pathCallback);

    // ros::ServiceServer service = nodeHandle.advertiseService("start_searching", start_searching);
    ros::Rate r(10);

    double roll_e, pitch_e, yaw_e;
    double K_x = 0.1, K_y = 0.2, K_yaw = 0.1;
    Eigen::VectorXd q, qdot, qddot;


    auto car_task = solver->getTask("base_link");
    auto car_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(car_task);

    while (ros::ok())
    {
        // std::cout << "backward_bool = " << backward_bool << std::endl;
        if (1)
        {
            Eigen::Vector3d base_pos;
            const std::string base_frame = "base_link";
            model->getPointPosition(base_frame, Eigen::Vector3d::Zero(),base_pos); 
            // std::cout << "base_pos[0] = " << base_pos[0] << std::endl;

            // if (abs(base_pos[0]) >= 0.029)
            // {
            //     /* code */
            // }
            

            double x_e = 1;
            // E[0] = 0.8 * K_x * direction * x_e;
            E[0] = 0;
            E[1] = 0;
            E[2] = 0;
            E[3] = 0;
            E[4] = 0;
            E[5] = 0;
            // std::cout << "Running" << E[0] << std::endl;

            car_cartesian->setVelocityReference(E);
            solver->update(time_, dt);
            model->getJointPosition(q);
            model->getJointVelocity(qdot);
            model->getJointAcceleration(qddot);
            
            q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
            qdot += dt * qddot;
            model->setJointPosition(q);
            model->setJointVelocity(qdot);
            model->update();
            // qdot[42] = 0;
            robot->setPositionReference(q.tail(robot->getJointNum()));
            robot->setVelocityReference(qdot.tail(robot->getJointNum()));
            robot->move();
            time_ += dt;
            rspub.publishTransforms(ros::Time::now(), "");
            /**
             * Move Robot
            */
            ros::spinOnce();
            r.sleep();
        }


        }        
}





