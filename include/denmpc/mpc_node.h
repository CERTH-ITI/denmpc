#ifndef RBCAR_POSE_TRACKING_H_
#define RBCAR_POSE_TRACKING_H_

#include <iostream>
#include <string>

#include <ros/ros.h>

#include <denmpc/Agent.h>
#include <denmpc/Constraint.h>
#include <denmpc/Cmscgmres.h>
#include <denmpc/rbcar.h>

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

// status
#define GOAL_NOT_RECEIVED 0
#define GOAL_REACHED 1
#define GOAL_IN_PROGRESS 2

namespace denmpc
{
    class MPCcontroller
    {
    private:
        ros::NodeHandle _nh;

        Rbcar *_rbcar;
        Cmscgmres *_controller; // kernel solver: "Condensed Multiple Shooting Generalized Minimal Residuum Method"

        std::string _current_pose_topic, _desired_pose_topic, _cmd_vel_topic, _mpc_plan_topic;

        double _max_speed, _max_steering_angle, _max_acceleration;
        double _control_period;
        double _distance_offset, _theta_offset;
        int _state;

        double _horizon_diskr, _horizon_length;

        ros::Publisher _cmd_vel_pub;
        ros::Publisher _plan_pub;
        ros::Timer _timer;
        ros::Subscriber _goal_sub;
        ros::Subscriber _pose_sub;
        bool _initialized;
        bool _received_pose;
        bool _received_goal;
        geometry_msgs::PoseStamped _current_pose, _desired_pose;
        double _previous_speed;

        // Methods
        void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void poseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void timerCallback(const ros::TimerEvent &event);

        void initialize();
        void publishVelocityCommand(double *cmd);
        void publishVelocityCommand(std::vector<double> cmd);
        void publishTrajectory(double **x);
        bool goalReached();
        geometry_msgs::Point positionDiff(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b); // diff in x,y,z
        double orientationDiff(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b); // diff in yaw

    public:
        MPCcontroller();
        ~MPCcontroller();
    };

} // namespace denmpc

#endif