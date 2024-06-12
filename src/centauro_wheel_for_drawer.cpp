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
    }else{
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
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    auto cfg = XBot::ConfigOptionsFromParamServer();
    // and we can make the model class
    auto model = XBot::ModelInterface::getModel(cfg);
    auto robot = XBot::RobotInterface::getRobot(cfg);
    // initialize to a homing configuration
    Eigen::VectorXd qhome;

    model->getRobotState("home", qhome);
    // std::cout << "qhome.size" << qhome.size() << std::endl;


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
    // ros::Subscriber sub = nodeHandle.subscribe("/tag_detections", 1000, tagDetectionsCallback);

    ros::Subscriber sub_backward = nodeHandle.subscribe("/open_flag", 1, backwardCallback);


    ros::ServiceServer service = nodeHandle.advertiseService("start_searching", start_searching);
    ros::Rate r(10);

    double roll_e, pitch_e, yaw_e;
    double K_x = 5, K_y = 0.2, K_yaw = 0.1;
    Eigen::VectorXd q, qdot, qddot;


    auto car_task = solver->getTask("base_link");
    auto car_cartesian = std::dynamic_pointer_cast<XBot::Cartesian::CartesianTask>(car_task);
    
    int count_forward = 0;
    int count_backward = 0;

    while (ros::ok())
    {
        // std::cout << "backward_bool = " << backward_bool << std::endl;
        if (1)
        {

            Eigen::Vector3d base_pos;
            const std::string base_frame = "base_link";
            model->getPointPosition(base_frame, Eigen::Vector3d::Zero(),base_pos); 
            // std::cout << "base_pos = " << base_pos << std::endl;

            double x_e = 0.5;
            if (direction == -1)
            {
                E[0] = 0;
                count_backward = count_backward + 1;

                // if (count_backward >= 11)
                // {
                //     count_backward = 11;
                //     E[0] = 0;
                // }

                std::cout << "count_backward = " << count_backward << std::endl;
            }else if (direction == 1)
            {
                /* code */
                E[0] = direction * x_e;
                count_forward = count_forward + 1;
                std::cout << "count_forward = " << count_forward << std::endl;
            }
            E[1] = 0;
            E[2] = 0;
            E[3] = 0;
            E[4] = 0;
            E[5] = 0;

            
            std::cout << "E: " << E << std::endl;
            // std::cout << "Running" << E[0] << std::endl;

            car_cartesian->setVelocityReference(E);
            solver->update(time_, dt);
            model->getJointPosition(q);
            model->getJointVelocity(qdot);
            model->getJointAcceleration(qddot);
            
            q += dt * qdot + 0.5 * std::pow(dt, 2) * qddot;
            qdot += dt * qddot;
            
            // std::cout << "qdot.size = " << qdot.size() << std::endl;
            // std::cout << "qddot.size = " << qddot.size() << std::endl;


            // q[ 0 ] =  -0.0018855216185921957;
            // q[ 1 ] =  -0.2011719774306072;
            // q[ 2 ] =  -0.0019966524197081855;
            // q[ 3 ] =  0.0033993627979932565;
            // q[ 4 ] =  -0.0034495029327205194;
            // q[ 5 ] =  0.004439609316802201;
            q[ 6 ] =  -0.6947201934626644;
            q[ 7 ] =  -1.6098530170101875;
            q[ 8 ] =  -1.9261441932454884;
            q[ 9 ] =  -0.3497121698568159;
            // q[ 10 ] =  5.506349104977607;
            // q[ 11 ] =  2.3202287376709005;
            q[ 12 ] =  0.6947346202330281;
            q[ 13 ] =  1.621651725779464;
            q[ 14 ] =  1.9269477417102965;
            q[ 15 ] =  0.34701567457213983;
            // q[ 16 ] =  3.5364793112672936;
            // q[ 17 ] =  -3.528104212517133;
            q[ 18 ] =  0.7983587818016566;
            q[ 19 ] =  1.5416675689150114;
            q[ 20 ] =  1.9684509620219706;
            q[ 21 ] =  0.40422116604563796;
            // q[ 22 ] =  3.8193584127612166;
            // q[ 23 ] =  -0.1550779792485414;
            q[ 24 ] =  -0.7983096186598166;
            q[ 25 ] =  -1.5458968273519493;
            q[ 26 ] =  -1.974789282615135;
            q[ 27 ] =  -0.3988661353672795;
            // q[ 28 ] =  4.985883322547674;
            // q[ 29 ] =  0.11492405835617893;
            q[ 30 ] =  0.4039944568376299;
            q[ 31 ] =  0.5006742869402812;
            q[ 32 ] =  0.29945826485022137;
            q[ 33 ] =  0.2994408973284771;
            q[ 34 ] =  -2.200131128120107;
            q[ 35 ] =  -0.0001740658925413898;
            q[ 36 ] =  -0.8001159502342114;
            q[ 37 ] =  0.7479437827362457;
            q[ 38 ] =  -0.9264480259414567;
            q[ 39 ] =  0.10258795046200221;
            q[ 40 ] =  -2.3163770137564526;
            q[ 41 ] =  0.9950850583600879;
            q[ 42 ] =  1.4861404917045418;
            // q[ 43 ] =  0.9;
            q[ 44 ] =  -0.0036306885623029516;
            q[ 45 ] =  0.0002926313199609307;


            qdot[6] = 0;qdot[7] = 0; qdot[8] = 0; 
            qdot[9] = 0;qdot[10] = 0; 
            qdot[12] = 0;qdot[13] = 0; qdot[14] = 0; 
            qdot[15] = 0;qdot[16] = 0; 
            qdot[18] = 0;qdot[19] = 0; qdot[20] = 0; 
            qdot[21] = 0;qdot[22] = 0; 
            qdot[24] = 0;qdot[25] = 0; qdot[26] = 0; 
            qdot[27] = 0;qdot[28] = 0; 

            qdot[31] = 0;qdot[32] = 0; qdot[33] = 0; 
            qdot[34] = 0;qdot[35] = 0; qdot[36] = 0; 
            qdot[37] = 0;qdot[38] = 0; qdot[39] = 0; 
            qdot[40] = 0;qdot[41] = 0; qdot[42] = 0; qdot[43] = 0; qdot[44] = 0;qdot[45] = 0;


            // qddot
            qddot[6] = 0;qddot[7] = 0; qddot[8] = 0; 
            qddot[9] = 0;qddot[10] = 0; 
            qddot[12] = 0;qddot[13] = 0; qddot[14] = 0; 
            qddot[15] = 0;qddot[16] = 0; 
            qddot[18] = 0;qddot[19] = 0; qddot[20] = 0; 
            qddot[21] = 0;qddot[22] = 0; 
            qddot[24] = 0;qddot[25] = 0; qddot[26] = 0; 
            qddot[27] = 0;qddot[28] = 0; 

            qddot[31] = 0;qddot[32] = 0; qddot[33] = 0; 
            qddot[34] = 0;qddot[35] = 0; qddot[36] = 0; 
            qddot[37] = 0;qddot[38] = 0; qddot[39] = 0; 
            qddot[40] = 0;qddot[41] = 0; qddot[42] = 0; qddot[43] = 0; qddot[44] = 0;qddot[45] = 0;

            
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





