#include <thread>
#include <ros/init.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <XBotInterface/RobotInterface.h>
#include <XBotInterface/ModelInterface.h>
#include <cartesian_interface/CartesianInterfaceImpl.h>
#include <cartesian_interface/utils/RobotStatePublisher.h>
#include <RobotInterfaceROS/ConfigFromParam.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/TransformStamped.h"
#include <Eigen/Dense>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
// #include <python.h>

using namespace XBot::Cartesian;
bool start_walking_bool = false;
double init_heigh;
int direction = -1; // 1: lower; -1: upper


bool start_walking(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    start_walking_bool = !start_walking_bool;
    return true;
};

int main(int argc, char **argv)
{
    // Py_Initialize();


    std::string filePath = "/home/wang/forest_ws/src/centauro_long_task/data/file.txt"; 
    std::ofstream outputFile(filePath);


    const std::string robotName = "centauro";
    // Initialize ros node
    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle("");

    auto cfg = XBot::ConfigOptionsFromParamServer();
    auto model = XBot::ModelInterface::getModel(cfg);
    auto robot = XBot::RobotInterface::getRobot(cfg);
    
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    model->setJointPosition(qhome);
    model->update();
    XBot::Cartesian::Utils::RobotStatePublisher rspub (model);


    const double dt = 0.01;
    double time = 0;
    auto ctx = std::make_shared<XBot::Cartesian::Context>(
                std::make_shared<XBot::Cartesian::Parameters>(dt),
                model
            );

    // load the ik problem given a yaml file
    std::string problem_description_string;
    nodeHandle.getParam("problem_description_manipulation", problem_description_string);

    auto ik_pb_yaml = YAML::Load(problem_description_string);
    XBot::Cartesian::ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // we are finally ready to make the CartesIO solver "OpenSot"
    auto solver = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_pb, ctx
                                                       );
    Eigen::VectorXd q, qdot, qddot;
    auto left_arm_task = solver->getTask("arm1_8");
    auto larm_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(left_arm_task);

    auto right_arm_task = solver->getTask("arm2_8");
    auto rarm_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(right_arm_task);
    int current_state4 = 0;

    ros::Rate r(100);
    ros::ServiceServer service = nodeHandle.advertiseService("start_walking", start_walking);
    ros::Publisher publisher = nodeHandle.advertise<std_msgs::Bool>("adjust_com", 100);

    Eigen::Affine3d L_Arm_ref;
    Eigen::Affine3d R_Arm_ref;
    int current_state = 0; // hand-crafted finite state machine!

    /**
     * Desire L_Arm R_Arm's translation vector
     */
        Eigen::Vector3d L_Arm_translation(0.8753, 0.2821, 0.06667);
        Eigen::Vector3d R_Arm_translation(0.8595, -0.1979, 0.1256);

    /**
     * Current L_Arm R_Arm's translation vector
     */

        // L_Arm_position
            // L_Arm_ref.translation() = 
            //  0.50576
            // 0.211833
            // 0.244283

        // Current R_Arm_ref: 
        //  0.524846
        // -0.224064
        //  0.274411



    if (argc > 1)
    {
        direction = atoi(argv[1]);
    }
    while (ros::ok())
    {
        if(current_state == 0) // here we command a reaching motion
        {
            std::cout << "Commanding hand" << std::endl;
            double target_time = 3.0;

            // larm_cartesian->getPoseReference(L_Arm_ref);
            // L_Arm_ref.translation() [0] = L_Arm_translation [0];
            // L_Arm_ref.translation() [1] = L_Arm_translation [1];
            // L_Arm_ref.translation() [2] = L_Arm_translation [2];
            // larm_cartesian->setPoseTarget(L_Arm_ref, target_time);


            rarm_cartesian->getPoseReference(R_Arm_ref);
            R_Arm_ref.translation() [0] = R_Arm_translation[0];
            // R_Arm_ref.translation() [1] = R_Arm_translation[1];
            // R_Arm_ref.translation() [2] = R_Arm_translation[2];
            rarm_cartesian->setPoseTarget(R_Arm_ref, target_time);

            current_state++;
        }
        if(current_state == 1) // here we check that the reaching started
        {
            if(larm_cartesian->getTaskState() == State::Reaching)
            {
                std::cout << "Motion started!" << std::endl;
                current_state++;
            }
        }
        if(current_state == 2) // here we wait for it to be completed
        {
            if(larm_cartesian->getTaskState() == State::Online && rarm_cartesian->getTaskState() == State::Online)
            {
                Eigen::Affine3d T;
                larm_cartesian->getCurrentPose(T);
                std::cout << "larm_cartesian larm current_state = " << std::endl << T.translation() << std::endl;
                std::cout << "Motion completed, final error is " <<
                            (T.inverse()*L_Arm_ref).translation().norm() << std::endl;

                rarm_cartesian->getCurrentPose(T);
                std::cout << "rarm current_state = " << std::endl << T.translation() << std::endl;
                std::cout << "Motion completed, final error is " <<
                            (T.inverse()*R_Arm_ref).translation().norm() << std::endl;

                current_state++;
            }
        }

        
        if(current_state == 3) // here we wait the robot to come to a stop
        {
            std::cout << "qdot norm is " << qdot.norm() << std::endl;
            if(qdot.norm() < 1e-3)
            {
                std::cout << "Robot came to a stop, press ENTER to exit.. \n";
                std::cin.ignore();
                current_state++;
                outputFile.close();
            }

        }
        
        if(current_state == 4) break;
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
        robot->move();
        time += dt;
        rspub.publishTransforms(ros::Time::now(), "");

        // get arm1 state 
        for (size_t i = 31; i <= 36; i++)
        {
            outputFile << q[i] << " " ;
        }
        outputFile << std::endl;

        r.sleep();
    }


}



