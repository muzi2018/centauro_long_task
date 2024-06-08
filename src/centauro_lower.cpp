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

using namespace XBot::Cartesian;
bool start_walking_bool = false;



bool start_walking(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    start_walking_bool = !start_walking_bool;
    return true;
};

int main(int argc, char **argv)
{
    const std::string robotName = "centauro";
    // Initialize ros node
    ros::init(argc, argv, robotName);
    ros::NodeHandle nodeHandle("");

    // Create a Buffer and a TransformListener
    // tf2_ros::Buffer tfBuffer;
    // tf2_ros::TransformListener tfListener(tfBuffer);


    // Get node parameters
    // std::string URDF_PATH, SRDF_PATH;
    // nodeHandle.getParam("/urdf_path", URDF_PATH);
    // nodeHandle.getParam("/srdf_path", SRDF_PATH);
    auto cfg = XBot::ConfigOptionsFromParamServer();
    // an option structure which is needed to make a model
    // XBot::ConfigOptions xbot_cfg;
    // // set the urdf and srdf path..
    // xbot_cfg.set_urdf_path(URDF_PATH);
    // xbot_cfg.set_srdf_path(SRDF_PATH);
    // // the following call is needed to generate some default joint IDs
    // xbot_cfg.generate_jidmap();
    // // some additional parameters..
    // xbot_cfg.set_parameter("is_model_floating_base", true);
    // xbot_cfg.set_parameter<std::string>("model_type", "RBDL");

    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(cfg);
    auto robot = XBot::RobotInterface::getRobot(cfg);
    
    // the "arms" group inside the SRDF
    // initialize to a homing configuration
    Eigen::VectorXd qhome;
    model->getRobotState("home", qhome);
    // qhome.setZero(); 
    model->setJointPosition(qhome);
    model->update();
    XBot::Cartesian::Utils::RobotStatePublisher rspub (model);


    // before constructing the problem description, let us build a
    // context object which stores some information, such as
    // the control period
    const double dt = 0.01;
    double time = 0, plan_time = 0;
    auto ctx = std::make_shared<XBot::Cartesian::Context>(
                std::make_shared<XBot::Cartesian::Parameters>(dt),
                model
            );

    // load the ik problem given a yaml file
    std::string problem_description_string;
    nodeHandle.getParam("problem_description_leg", problem_description_string);

    auto ik_pb_yaml = YAML::Load(problem_description_string);
    XBot::Cartesian::ProblemDescription ik_pb(ik_pb_yaml, ctx);

    // we are finally ready to make the CartesIO solver "OpenSot"
    auto solver = XBot::Cartesian::CartesianInterfaceImpl::MakeInstance("OpenSot",
                                                       ik_pb, ctx
                                                       );

    



    Eigen::VectorXd q, qdot, qddot;
    // auto right_arm_task = solver->getTask("arm2_8");
    // auto rarm_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(right_arm_task);



    /**leg task*/
    auto leg1_task = solver->getTask("wheel_1");
    auto leg1_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(leg1_task);

    /**leg task*/
    auto leg2_task = solver->getTask("wheel_2");
    auto leg2_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(leg2_task);

    /**leg task*/
    auto leg3_task = solver->getTask("wheel_3");
    auto leg3_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(leg3_task);

    /**leg task*/
    auto leg4_task = solver->getTask("wheel_4");
    auto leg4_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(leg4_task);

    /**com task*/
    auto com_task = solver->getTask("com");
    auto com_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(com_task);

    
    // // // get pose reference from task
    // // // ...
    Eigen::Affine3d RightArm_T_ref;
    Eigen::Affine3d Leg1_T_ref;
    Eigen::Affine3d Leg2_T_ref;
    Eigen::Affine3d Leg3_T_ref;
    Eigen::Affine3d Leg4_T_ref;
    Eigen::Affine3d Com_T_ref;
    
    
    double x, z;
    int leg_state = 1,num_leg = 2, segment = 0;
    double phase_time = 3, target_time = num_leg*phase_time;

    // the foot position in world coordinate
    Eigen::Vector3d leg1_pos;
    const std::string leg1_frame = "wheel_1";
    model->getPointPosition(leg1_frame, Eigen::Vector3d::Zero(),leg1_pos); 

    Eigen::Vector3d leg2_pos;
    const std::string leg2_frame = "wheel_2";
    model->getPointPosition(leg2_frame, Eigen::Vector3d::Zero(),leg2_pos); 

    Eigen::Vector3d leg3_pos;
    const std::string leg3_frame = "wheel_3";
    model->getPointPosition(leg3_frame, Eigen::Vector3d::Zero(),leg3_pos); 

    Eigen::Vector3d leg4_pos;
    const std::string leg4_frame = "wheel_4";
    model->getPointPosition(leg4_frame, Eigen::Vector3d::Zero(),leg4_pos); 

    Eigen::Vector3d leg_mid;
    leg_mid = leg1_pos + leg2_pos + leg3_pos + leg4_pos;

    Eigen::Vector3d com_pos;
    model->getCOM(com_pos);
    

    ROS_INFO_STREAM("wheel_1");
    ROS_INFO_STREAM(leg1_pos);
    ROS_INFO_STREAM("wheel_2");
    ROS_INFO_STREAM(leg2_pos);
    ROS_INFO_STREAM("wheel_3");
    ROS_INFO_STREAM(leg3_pos);
    ROS_INFO_STREAM("wheel_4");
    ROS_INFO_STREAM(leg4_pos);
    ROS_INFO_STREAM("com_pos");
    ROS_INFO_STREAM(com_pos);


    // Trajectory::WayPointVector wp;
    // Eigen::Affine3d w_T_f1 ;
    // w_T_f1.setIdentity();

    // w_T_f1.pretranslate(Eigen::Vector3d(0.2,0,0));
    // wp.emplace_back(w_T_f1,10);
    // leg1_cartesian->setWayPoints(wp);
    
    /** task*/
    int current_state1 = 0;
    int current_state2 = 0;
    int current_state3 = 0;
    int current_state4 = 0;


    bool state1_support = false;
    bool state2_support = false;
    bool state3_support = false;
    bool state4_support = false;

    int i=1; // mpc step index 
    double long_x = 0.1; // step long distance
    double leg_height = 0.1; // step height
    double seg_num = 300; // mpc segment number
    double seg_time = phase_time / seg_num; // mpc segment duration
    double seg_dis = long_x / seg_num; // each mpc step long

    double com_shift_x, com_shift_y, com_shift_z = -0.1;

    ros::Rate r(100);
    ros::ServiceServer service = nodeHandle.advertiseService("start_walking", start_walking);
    ros::Publisher publisher = nodeHandle.advertise<std_msgs::Bool>("adjust_com", 100);

    while (ros::ok())
    {
        model->getCOM(com_pos);
        model->getPointPosition(leg1_frame, Eigen::Vector3d::Zero(),leg1_pos);
        model->getPointPosition(leg2_frame, Eigen::Vector3d::Zero(),leg2_pos);
        model->getPointPosition(leg3_frame, Eigen::Vector3d::Zero(),leg3_pos);
        model->getPointPosition(leg4_frame, Eigen::Vector3d::Zero(),leg4_pos); 
        leg_mid = ( leg1_pos + leg2_pos + leg3_pos + leg4_pos)/4;
        Eigen::Vector3d base_pose;
        const std::string base_link_frame = "base_link";
        model->getPointPosition(base_link_frame, Eigen::Vector3d::Zero(),base_pose);
        double robot_heigh = base_pose[2] - leg_mid[2];
        std::cout << "robot_heigh = " << robot_heigh <<std::endl;

        ROS_INFO("current_state4 = %d", current_state4);
        // std::cout << "robot_z = " << robot_heigh << std::endl;
        if (robot_heigh < 0.55)
        {
            std_msgs::Bool msg;
            msg.data = true;
            publisher.publish(msg);
            current_state4 = 1000;
            
            ros::shutdown();
            return 0;
        }
        
        if (current_state4 == 0 )
        {
            // com trajectory
            com_cartesian->getPoseReference(Com_T_ref);
            Com_T_ref.pretranslate(Eigen::Vector3d(0, 0, com_shift_z));
            com_cartesian->setPoseTarget(Com_T_ref, 10);
            current_state4++;
        }
        if (current_state4 == 1)
        {
            if (com_cartesian->getTaskState() == State::Reaching)
            {
                current_state4++;
            }
            
        }
        if (current_state4 == 2)
        {
            if (com_cartesian->getTaskState() == State::Online)
            {
                current_state4 = 0;
            }
            
        }
        // std::cout << "Motion started!" << std::endl;
        solver->update(time, dt);
        model->getJointPosition(q);
        model->getJointVelocity(qdot);
        // std::cout << "qdot.size() = " << qdot.size() << std::endl;
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
        r.sleep();
    }
}



