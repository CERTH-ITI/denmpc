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

#include "denmpc/Agent.h"
#include <geometry_msgs/PoseStamped.h>

// x : states
// u : controls
// y : outputs
// d : disturbances
// p : parameters

class Rbcar : public Agent
{
    void start()
    {
        printf("Agent%i: Rbcar started", this->id_);
    };
    void pause()
    {
        printf("Agent%i: Rbcar paused", this->id_);
    };
    void stop()
    {
        printf("Agent%i: Rbcar stopped", this->id_);
    };

public:
    Rbcar(int id = 0);
    Rbcar(
        double *init_x,
        double *init_xdes,
        double *init_u,
        double *init_udes,
        double *init_p,
        double *init_d,
        int id);

    void stateCallback(geometry_msgs::PoseStamped msg)
    {
        std::vector<double> tmp(dim_x_, 0);

        tf::Quaternion quat(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
        double tmp1, tmp2, yaw;
        tf::Matrix3x3(quat).getEulerYPR(yaw, tmp1, tmp2);

        tmp[0] = msg.pose.position.x;
        tmp[1] = msg.pose.position.y;
        tmp[2] = yaw;
        this->setState(tmp);
    };

    void desiredStateCallback(geometry_msgs::PoseStamped msg)
    {
        std::vector<double> tmp(dim_x_, 0);

        tf::Quaternion quat(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
        double tmp1, tmp2, yaw;
        tf::Matrix3x3(quat).getEulerYPR(yaw, tmp1, tmp2);

        tmp[0] = msg.pose.position.x;
        tmp[1] = msg.pose.position.y;
        tmp[2] = yaw;
        this->setDesiredState(tmp);
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
