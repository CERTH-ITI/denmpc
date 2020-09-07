/**
 * @file    Rbcar.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Exported with Mathematica Code Generator by Jan Dentler
 */

#ifndef RBCAR_H_
#define RBCAR_H_

#include <Agent.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

/*******************************************************
 * Agent                                               * 
 *******************************************************/

class Rbcar : public Agent
{
    geometry_msgs::PoseWithCovarianceStamped subscriber0_old_msg_;
    geometry_msgs::PoseStamped subscriber1_old_msg_;

    double max_speed, max_steering_angle, max_acceleration;

    double current_speed, previous_speed;

    double control_period;

    double distance_offset, theta_offset;

    void start() { printf("Agent%i: Rbcar started", this->id_); };
    void pause() { printf("Agent%i: Rbcar paused", this->id_); };
    void stop() { printf("Agent%i: Rbcar stopped", this->id_); };

public:
    Rbcar(int id = 0);
    Rbcar(
        std::string pose,
        std::string desiredpose,
        std::string cmd_vel,
        double max_speed,
        double max_steering_angle,
        double max_acceleration,
        double current_speed,
        double previous_speed,
        double control_period,
        double distance_offset,
        double theta_offset,
        double *init_x,
        double *init_xdes,
        double *init_u,
        double *init_udes,
        double *init_p,
        double *init_d,
        int id);
    void setStateSubscriberRosTopicName(std::string rostopicname)
    {
        ros_state_subscribers_[0]->shutdown();
        *ros_state_subscribers_[0] = ros_node_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(rostopicname, 1, &Rbcar::subStateCallback, this);
    };
    void subStateCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
    {
        std::vector<double> tmp(dim_x_, 0);

        tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        double tmp1, tmp2, yaw;
        tf::Matrix3x3(quat).getEulerYPR(yaw, tmp1, tmp2);

        tmp[0] = msg->pose.pose.position.x;
        tmp[1] = msg->pose.pose.position.y;
        tmp[2] = yaw;
        this->setState(tmp);

        subscriber0_old_msg_ = *msg;
    };
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
    void publishZeroCmd()
    {
        geometry_msgs::Twist msg;
        msg.linear.x = 0;
        msg.linear.y = 0;
        msg.linear.z = 0;
        msg.angular.x = 0;
        msg.angular.y = 0;
        msg.angular.z = 0;
        ros_publishers_[0]->publish(msg);
    };
    void setDesiredStateSubscriberRosTopicName(std::string rostopicname)
    {
        ros_desired_state_subscribers_[0]->shutdown();
        *ros_desired_state_subscribers_[0] = ros_node_.subscribe<geometry_msgs::PoseStamped>(rostopicname, 1, &Rbcar::subDesiredStateCallback, this);
    };
    void subDesiredStateCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        ROS_DEBUG("Desired position is : (%f, %f)\n", msg->pose.position.x, msg->pose.position.y);
        std::vector<double> tmp(dim_x_, 0);

        tf::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        double tmp1, tmp2, yaw;
        tf::Matrix3x3(quat).getEulerYPR(yaw, tmp1, tmp2);

        tmp[0] = msg->pose.position.x;
        tmp[1] = msg->pose.position.y;
        tmp[2] = yaw;
        this->setDesiredState(tmp);

        subscriber1_old_msg_ = *msg;
    }

    void setPublisherRosTopicName(std::string rostopicname)
    {
        ros_publishers_[0]->shutdown();
        *ros_publishers_[0] = ros_node_.advertise<geometry_msgs::Twist>(rostopicname, 1);
    };
    void setMaxConstraints(double _max_speed, double _max_steering_angle, double _max_acceleration)
    {
        max_speed = _max_speed;
        max_steering_angle = _max_steering_angle;
        max_acceleration = _max_acceleration;
    };
    void setControlPeriod(double _control_period)
    {
        control_period = _control_period;
    };
    void setOffset(double _distance_offset, double _theta_offset)
    {
        distance_offset = _distance_offset;
        theta_offset = _theta_offset;
    };
    void rosPublishActuation()
    {
        bool enable_actuation;
        ros::param::get("enable_actuation", enable_actuation);
        if (goalReached() || !enable_actuation)
            publishZeroCmd();
        else
        {
            // clamp velocity
            if (u_[0] > max_speed)
                u_[0] = max_speed;
            else if (u_[0] < -1.0 * max_speed)
                u_[0] = -1.0 * max_speed;

            // check acceleration limits
            if (u_[0] > previous_speed + max_acceleration * control_period)
            {
                u_[0] = previous_speed + max_acceleration * control_period;
            }
            else if (u_[0] < previous_speed - max_acceleration * control_period)
            {
                u_[0] = previous_speed - max_acceleration * control_period;
            }

            // clamp steering angle
            if (u_[1] > max_steering_angle)
                u_[1] = max_steering_angle;
            else if (u_[1] < -1.0 * max_steering_angle)
                u_[1] = -1.0 * max_steering_angle;

            geometry_msgs::Twist msg;
            msg.linear.x = u_[0];
            msg.linear.y = 0;
            msg.linear.z = 0;
            msg.angular.x = 0;
            msg.angular.y = 0;
            msg.angular.z = u_[1];
            ros_publishers_[0]->publish(msg);

            previous_speed = u_[0];
        }
    }
    void f(double *out, double t, double *x, double *u, double *d, double *p);
    void dfdxlambda(double *out, double t, double *x, double *u, double *d, double *p, double *lambda);
    void dfdulambda(double *out, double t, double *x, double *u, double *d, double *p, double *lambda);
    void l(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes);
    void dldx(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes);
    void dldu(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes);
    void v(double *out, double t, double *x, double *p, double *xdes);
    void dvdx(double *out, double t, double *x, double *p, double *xdes);
    void ci(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes);
    void dcidxmui(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *mui);
    void dcidumui(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *mui);
    void cia(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *slack);
    void dciadxmui(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *mui, double *slack);
    void dciadumui(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *mu, double *slack);
    void dciadamui(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *mui, double *slack);
};
#endif
