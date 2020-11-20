#include "denmpc/mpc_node.h"

namespace denmpc
{
    MPCcontroller::MPCcontroller()
    {
        ros::param::get("/current_pose", _current_pose_topic);
        ros::param::get("/desired_pose", _desired_pose_topic);
        ros::param::get("/cmd_vel/denmpc", _cmd_vel_topic);
        ros::param::get("/mpc_plan", _mpc_plan_topic);
        ros::param::get("/max_speed", _max_speed);
        ros::param::get("/max_steering_angle", _max_steering_angle);
        ros::param::get("/max_acceleration", _max_acceleration);
        ros::param::get("/control_period", _control_period);
        ros::param::get("/distance_offset", _distance_offset);
        ros::param::get("/theta_offset", _theta_offset);
        ros::param::get("/horizon_diskretization", _horizon_diskr);
        ros::param::get("/horizon_length", _horizon_length);

        _received_pose = 0;
        _received_goal = 0;
        _initialized = 0;

        _state = GOAL_NOT_RECEIVED;

        // First advertise, them subscribe
        _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>(_cmd_vel_topic.c_str(), 10);
        _plan_pub = _nh.advertise<nav_msgs::Path>(_mpc_plan_topic.c_str(), 10);

        _goal_sub = _nh.subscribe<geometry_msgs::PoseStamped>(_desired_pose_topic.c_str(), 1, &MPCcontroller::goalCallback, this);
        _pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(_current_pose_topic.c_str(), 1, &MPCcontroller::poseCallback, this);

        // everything runs with the timer callback function
        _timer = _nh.createTimer(ros::Duration(_control_period), &MPCcontroller::timerCallback, this);

        initialize();
        std::cout << "MPC node initialized successfully\n";
    }

    MPCcontroller::~MPCcontroller()
    {
        delete _rbcar;
        delete _controller;
    }

    void MPCcontroller::initialize()
    {
        // Initialize Rbcar instance
        _rbcar = new Rbcar(0);
        // rbcar state={x,y,yaw} input={uforward,urotate}
        double rbcar_init_p[] = {1.0, 1.0, 0.0, //Q // TODO search more about these params, how to they affect
                                 1.0, 0.5};     // R
        //Define penaltys of state Q and inputs R
        _rbcar->setInitialParameter(rbcar_init_p);

        // Initialize controller instance
        _controller = new Cmscgmres(_rbcar, 0);
        _controller->setHorizonDiskretization(_horizon_diskr);
        _controller->setHorizonLength(_horizon_length);
        _controller->setTolerance(1e-8);
        _controller->setUpdateIntervall(_control_period);
        _controller->setMaximumNumberofIterations(10);

        // _controller->activateInfo_ControllerTrace();
        _controller->activateInfo_Controller();
        // _controller->startLogging2File();
        // _controller->activateInfo_ControllerStates();

        _controller->startAgents();

        _initialized = 1;
    }

    void MPCcontroller::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        // std::cout << "Goal callback\n";
        _desired_pose = *msg;
        _received_goal = 1;
        _state = GOAL_IN_PROGRESS;
    }

    void MPCcontroller::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        // std::cout << "Pose callback\n";
        _current_pose = *msg;
        _received_pose = 1;

        if (goalReached())
        {
            _state = GOAL_REACHED;
        }
    }

    void MPCcontroller::timerCallback(const ros::TimerEvent &event)
    {
        // std::cout << "Timer callback\n";
        if (!_initialized)
            return;

        if (!_received_pose)
            return;

        if (!_received_goal)
            return;

        _rbcar->stateCallback(_current_pose);
        _rbcar->desiredStateCallback(_desired_pose);

        if (goalReached())
        {
            std::cout << "Waiting for new goal to arrive .......\n";
            std::vector<double> action = {0, 0};
            publishVelocityCommand(action);
            _state = GOAL_NOT_RECEIVED;
            return;
        }

        ROS_INFO("Current position is : (%f, %f, %f)\n", _current_pose.pose.position.x, _current_pose.pose.position.y, _current_pose.pose.position.z);
        ROS_INFO("Desired position is : (%f, %f, %f)\n", _desired_pose.pose.position.x, _desired_pose.pose.position.y, _desired_pose.pose.position.z);

        _controller->getMeasurements();
        _controller->computeAction(ros::Time::now().toSec());
        double *action = _controller->returnAction();

        publishVelocityCommand(action);

        // Publish trajectory
        publishTrajectory(_controller->getX());
    }

    void MPCcontroller::publishVelocityCommand(double *cmd)
    {
        double *cmd_to_be_applied = cmd;
        // Clamp velocity
        if (cmd[0] > _max_speed)
            cmd_to_be_applied[0] = _max_speed;
        else if (cmd[0] < -1.0 * _max_speed)
            cmd_to_be_applied[0] = -1.0 * _max_speed;

        // check acceleration limits
        if (cmd_to_be_applied[0] > _previous_speed + _max_acceleration * _control_period)
        {
            cmd_to_be_applied[0] = _previous_speed + _max_acceleration * _control_period;
        }
        else if (cmd_to_be_applied[0] < _previous_speed - _max_acceleration * _control_period)
        {
            cmd_to_be_applied[0] = _previous_speed - _max_acceleration * _control_period;
        }

        if (cmd_to_be_applied[1] > _max_steering_angle)
            cmd_to_be_applied[1] = _max_steering_angle;
        else if (cmd_to_be_applied[1] < -1.0 * _max_steering_angle)
            cmd_to_be_applied[1] = -1.0 * _max_steering_angle;

        geometry_msgs::Twist msg;
        msg.linear.x = cmd_to_be_applied[0];
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = cmd_to_be_applied[1];

        std::cout << "Action to be applied " << cmd_to_be_applied[0] << " , " << cmd_to_be_applied[1] << std::endl;
        _cmd_vel_pub.publish(msg);

        _previous_speed = cmd_to_be_applied[0];
    }

    void MPCcontroller::publishVelocityCommand(std::vector<double> cmd)
    {
        std::vector<double> cmd_to_be_applied = cmd;
        // Clamp velocity
        if (cmd[0] > _max_speed)
            cmd_to_be_applied[0] = _max_speed;
        else if (cmd[0] < -1.0 * _max_speed)
            cmd_to_be_applied[0] = -1.0 * _max_speed;

        // check acceleration limits
        if (cmd_to_be_applied[0] > _previous_speed + _max_acceleration * _control_period)
        {
            cmd_to_be_applied[0] = _previous_speed + _max_acceleration * _control_period;
        }
        else if (cmd_to_be_applied[0] < _previous_speed - _max_acceleration * _control_period)
        {
            cmd_to_be_applied[0] = _previous_speed - _max_acceleration * _control_period;
        }

        if (cmd_to_be_applied[1] > _max_steering_angle)
            cmd_to_be_applied[1] = _max_steering_angle;
        else if (cmd_to_be_applied[1] < -1.0 * _max_steering_angle)
            cmd_to_be_applied[1] = -1.0 * _max_steering_angle;

        geometry_msgs::Twist msg;
        msg.linear.x = cmd_to_be_applied[0];
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = cmd_to_be_applied[1];

        std::cout << "Action to be applied " << cmd_to_be_applied[0] << " , " << cmd_to_be_applied[1] << std::endl;
        _cmd_vel_pub.publish(msg);

        _previous_speed = cmd_to_be_applied[0];
    }

    void MPCcontroller::publishTrajectory(double **x)
    {
        nav_msgs::Path mpc_plan;
        mpc_plan.header.stamp = ros::Time::now();
        mpc_plan.header.frame_id = "map";
        for (int i = 0; i < 2 * _horizon_diskr; i += 2)
        {
            geometry_msgs::PoseStamped tmp;
            tmp.pose.position.x = x[i][0];
            tmp.pose.position.y = x[i][1];
            tmp.pose.position.z = x[i][2];

            // Convert orientation from RPY to quaternion
            tf::Quaternion q;
            q.setRPY(x[i + 1][0] * M_PI / 180, x[i + 1][1] * M_PI / 180, x[i + 1][2] * M_PI / 180); // rad
            quaternionTFToMsg(q.normalize(), tmp.pose.orientation);
            mpc_plan.poses.push_back(tmp);
        }

        _plan_pub.publish(mpc_plan);
    }

    bool MPCcontroller::goalReached()
    {
        if (positionDiff(_current_pose, _desired_pose).x < _distance_offset && positionDiff(_current_pose, _desired_pose).y < _distance_offset && orientationDiff(_current_pose, _desired_pose) < _theta_offset)
            return 1;

        return 0;
    }

    geometry_msgs::Point MPCcontroller::positionDiff(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b)
    {
        geometry_msgs::Point out;
        out.x = abs(a.pose.position.x - b.pose.position.x);
        out.y = abs(a.pose.position.y - b.pose.position.y);
        out.z = abs(a.pose.position.z - b.pose.position.z);
        return out;
    }

    double MPCcontroller::orientationDiff(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b)
    {
        double yaw_a = tf::getYaw(a.pose.orientation);
        double yaw_b = tf::getYaw(b.pose.orientation);
        return (abs(yaw_a - yaw_b));
    }

} // namespace denmpc

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_node");

    ros::NodeHandle nh;

    denmpc::MPCcontroller mpc;

    ros::spin();

    return 0;
}
