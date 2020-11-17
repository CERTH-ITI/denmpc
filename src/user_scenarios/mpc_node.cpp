#include "rbcar_pose_tracking.h"

namespace denmpc
{
    MPCcontroller::MPCcontroller()
    {
        ros::param::get("/current_pose", _current_pose_topic);
        ros::param::get("/desired_pose", _desired_pose_topic);
        ros::param::get("/cmd_vel/denmpc", _cmd_vel_topic);
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

        // First advertise, them subscribe
        _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>(_cmd_vel_topic.c_str(), 10);

        _goal_sub = _nh.subscribe<geometry_msgs::PoseStamped>(_desired_pose_topic.c_str(), 10, &MPCcontroller::goalCallback, this);

        _pose_sub = _nh.subscribe<geometry_msgs::PoseStamped>(_current_pose_topic.c_str(), 10, &MPCcontroller::poseCallback, this);

        // everything runs with the timer callback function
        _timer = _nh.createTimer(ros::Duration(_control_period), &MPCcontroller::timerCallback, this);

        initialize();
        std::cout << "Initialized\n";
    }

    MPCcontroller::~MPCcontroller()
    {
    }

    void MPCcontroller::initialize()
    {
        // if (!_received_pose)
        // {
        //     ROS_WARN("Did not receive pose, cannot continue\n");
        //     return;
        // }

        // Initialize Rbcar instance
        _rbcar = new Rbcar(0);
        double rbcar0_init_p[] = {1.0, 1.0, 1.0, 1.0, 0.5};
        // Initial desired pose
        // double rbcar0_init_x[] = {0.0, 0.0, 0.0};
        // double rbcar0_init_xdes[] = {0.0, 0.0, 0.0};
        // _rbcar->setInitialState(rbcar0_init_x);
        // _rbcar->setInitialDesiredState(rbcar0_init_xdes);
        _rbcar->setInitialParameter(rbcar0_init_p);
        _rbcar->setMaxConstraints(_max_speed, _max_steering_angle, _max_acceleration);
        // _rbcar->setControlPeriod(_control_period);
        // _rbcar->setOffset(_distance_offset, _theta_offset);

        // _agent_list.push_back(_rbcar);

        // Initialize controller instance
        _controller = new Cmscgmres(_rbcar, 0);
        _controller->setHorizonDiskretization(_horizon_diskr);
        _controller->setHorizonLength(_horizon_length);
        _controller->setTolerance(1e-8);
        _controller->setUpdateIntervall(_control_period);
        _controller->setMaximumNumberofIterations(10);

        // _controller->activateInfo_ControllerTrace();
        // _controller->activateInfo_Controller();
        // _controller->startLogging2File();
        // _controller->activateInfo_ControllerStates();

        // _controller_list.push_back(_controller);

        _controller->startAgents();

        _initialized = 1;
    }

    void MPCcontroller::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        // std::cout << "Goal callback\n";
        _desired_pose = *msg;
        _received_goal = 1;
    }

    void MPCcontroller::poseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        // std::cout << "Pose callback\n";
        _current_pose = *msg;
        _received_pose = 1;
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

        double *action;
        _controller->getMeasurements();
        _controller->computeAction(ros::Time::now().toSec());
        action = _controller->returnAction();

        publishVelocityCommand(action);

        // Publish trajectory
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

} // namespace denmpc

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_node");

    ros::NodeHandle nh;

    denmpc::MPCcontroller mpc;

    ros::spin();

    return 0;
}

/*

    bool goalReached()
    {
        double *current_state, *desired_state;
        current_state = defvector(dim_x_);
        desired_state = defvector(dim_x_);

        this->getState(current_state);
        this->getDesiredState(desired_state);

        if ((abs(current_state[0] - desired_state[0]) < distance_offset) && (abs(current_state[1] - desired_state[1]) < distance_offset) && (abs(current_state[2] - desired_state[2]) < theta_offset))
            return 1;

        return 0;
    };
*/