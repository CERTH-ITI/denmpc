#ifndef RBCAR_POSE_TRACKING_H_
#define RBCAR_POSE_TRACKING_H_

#include <iostream>
#include <ros/ros.h>
#include <Agent.h>
#include <Constraint.h>

#include <rbcar.h>

#include <Cmscgmres.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <string>

namespace denmpc
{
    class MPCcontroller
    {

    private:
        ros::NodeHandle _nh;

        // std::vector<Agent *> _agent_list;
        Rbcar *_rbcar;
        // std::vector<Controller *> _controller_list;
        Cmscgmres *_controller; // kernel solver: "Condensed Multiple Shooting Generalized Minimal Residuum Method"

        std::string _current_pose_topic, _desired_pose_topic, _cmd_vel_topic;

        double _max_speed, _max_steering_angle, _max_acceleration;
        double _control_period;
        double _distance_offset, _theta_offset;

        double _horizon_diskr, _horizon_length;

        ros::Publisher _cmd_vel_pub;
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

    public:
        MPCcontroller();
        ~MPCcontroller();
    };

} // namespace denmpc

#endif