/**
 * @file    Rbcar.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Exported with Mathematica Code Generator by Jan Dentler
 */

#include <rbcar.h>

/*******************************************************
 * Constructor                                         * 
 *******************************************************/
Rbcar::Rbcar(int id) : Agent(id)
{
    //Set initial values
    dim_x_ = 3;
    dim_xdes_ = 3;
    dim_u_ = 2;
    dim_udes_ = 2;
    dim_y_ = 0;
    dim_ydes_ = 0;
    dim_p_ = 5;
    dim_d_ = 0;
    dim_l_ = 1;
    dim_v_ = 1;
    dim_eq_ = 0;
    dim_ineq_ = 2;

    max_speed = 1.0;
    max_steering_angle = 0.8;
    max_acceleration = 1.0;
    previous_speed = 0.0;
    control_period = 0.01;
    distance_offset = 0.5;
    theta_offset = 0.5;

    //Allocate vectors
    x_ = defvector(dim_x_);
    xdes_ = defvector(dim_xdes_);
    u_ = defvector(dim_u_);
    udes_ = defvector(dim_udes_);
    y_ = defvector(dim_y_);
    ydes_ = defvector(dim_ydes_);
    d_ = defvector(dim_d_);
    p_ = defvector(dim_p_);
    x_init_ = defvector(dim_x_);
    xdes_init_ = defvector(dim_xdes_);
    u_init_ = defvector(dim_u_);
    udes_init_ = defvector(dim_udes_);
    y_init_ = defvector(dim_y_);
    ydes_init_ = defvector(dim_ydes_);
    d_init_ = defvector(dim_d_);
    p_init_ = defvector(dim_p_);

    //Creating control publisher
    ros::Publisher *pub0 = new ros::Publisher();
    //Starting Advertising
    *pub0 = ros_node_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    //Adding publisher to array
    ros_publishers_.push_back(pub0);

    //Creating subscriber
    ros::Subscriber *sub0 = new ros::Subscriber();

    //Starting subscription
    *sub0 = ros_node_.subscribe("/amcl_pose", 1, &Rbcar::subStateCallback, this);

    //Adding subscriber to array
    ros_state_subscribers_.push_back(sub0);

    //Creating subscriber
    ros::Subscriber *sub1 = new ros::Subscriber();

    //Starting subscription
    *sub1 = ros_node_.subscribe("/next_goal", 1, &Rbcar::subDesiredStateCallback, this);

    //Adding subscriber to array
    ros_desired_state_subscribers_.push_back(sub1);
}
Rbcar::Rbcar(
    std::string state_subscriber_topic,
    std::string desired_state_subscriber_topic,
    std::string control_publish_topic,
    double max_speed = 1.0,
    double max_steering_angle = 0.8,
    double max_acceleration = 1.0,
    double current_speed = 0.0,
    double previous_speed = 0.0,
    double control_period = 0.01,
    double distance_offset = 0.5,
    double theta_offset = 0.5,
    double *init_x = NULL,
    double *init_xdes = NULL,
    double *init_u = NULL,
    double *init_udes = NULL,
    double *init_p = NULL,
    double *init_d = NULL,
    int id = 0) : Rbcar(id)
{
    if (init_x != NULL)
    {
        this->setInitialState(init_x);
    };
    if (init_xdes != NULL)
    {
        this->setInitialDesiredState(init_xdes);
    };
    if (init_u != NULL)
    {
        this->setInitialControl(init_u);
    };
    if (init_udes != NULL)
    {
        this->setInitialDesiredControl(init_udes);
    };
    if (init_p != NULL)
    {
        this->setInitialParameter(init_p);
    };
    this->reset2initialstate();
    this->setStateSubscriberRosTopicName(state_subscriber_topic);
    this->setDesiredStateSubscriberRosTopicName(desired_state_subscriber_topic);
    this->setPublisherRosTopicName(control_publish_topic);
};

/******************************************************** 
 * System dynamics                                      * 
 ********************************************************/
void Rbcar::f(double *out, double t, double *x, double *u, double *d, double *p)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::f(...)" << endl;
#endif
    out[0] = u[0] * cos(x[2]);
    out[1] = u[0] * sin(x[2]);
    out[2] = u[1];
}
void Rbcar::dfdxlambda(double *out, double t, double *x, double *u, double *d, double *p, double *lambda)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::dfdx(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = lambda[1] * u[0] * cos(x[2]) - lambda[0] * u[0] * sin(x[2]);
    ;
}
void Rbcar::dfdulambda(double *out, double t, double *x, double *u, double *d, double *p, double *lambda)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::dfdu(...)" << endl;
#endif
    out[0] = lambda[0] * cos(x[2]) + lambda[1] * sin(x[2]);
    out[1] = lambda[2];
}

/******************************************************** 
 * Stage costs                                          * 
 ********************************************************/
void Rbcar::l(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::l(...)" << endl;
#endif
    out[0] = p[3] * pow(-u[0] + udes[0], 2.) + p[4] * pow(-u[1] + udes[1], 2.) + p[0] * pow(-x[0] + xdes[0], 2.) + p[1] * pow(-x[1] + xdes[1], 2.) + p[2] * pow(-x[2] + xdes[2], 2.);
}
void Rbcar::dldx(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::dldx(...)" << endl;
#endif
    out[0] = -2. * p[0] * (-x[0] + xdes[0]);
    out[1] = -2. * p[1] * (-x[1] + xdes[1]);
    out[2] = -2. * p[2] * (-x[2] + xdes[2]);
}
void Rbcar::dldu(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::dldu(...)" << endl;
#endif
    out[0] = -2. * p[3] * (-u[0] + udes[0]);
    out[1] = -2. * p[4] * (-u[1] + udes[1]);
}

/******************************************************** 
 * Final costs                                          * 
 ********************************************************/
void Rbcar::v(double *out, double t, double *x, double *p, double *xdes)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::v(...)" << endl;
#endif
    out[0] = p[0] * pow(-x[0] + xdes[0], 2.) + p[1] * pow(-x[1] + xdes[1], 2.) + p[2] * pow(-x[2] + xdes[2], 2.);
}
void Rbcar::dvdx(double *out, double t, double *x, double *p, double *xdes)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::dvdx(...)" << endl;
#endif
    out[0] = -2. * p[0] * (-x[0] + xdes[0]);
    out[1] = -2. * p[1] * (-x[1] + xdes[1]);
    out[2] = -2. * p[2] * (-x[2] + xdes[2]);
}

/******************************************************** 
 * Inequality constraints                               * 
 ********************************************************/
void Rbcar::ci(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::ci(...)" << endl;
#endif
    out[0] = -0.5 + 0.5 * pow(u[0], 2.);
    out[1] = -0.5 + 0.5 * pow(u[1], 2.);
}
void Rbcar::dcidxmui(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *mui)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::dcidxmui(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
}
void Rbcar::dcidumui(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *mui)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::dcidumui(...)" << endl;
#endif
    out[0] = mui[0] * u[0];
    out[1] = mui[1] * u[1];
}
void Rbcar::cia(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *slack)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::cia(...)" << endl;
#endif
    out[0] = -0.5 + pow(slack[0], 2.) + 0.5 * pow(u[0], 2.);
    out[1] = -0.5 + pow(slack[1], 2.) + 0.5 * pow(u[1], 2.);
}
void Rbcar::dciadxmui(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *mui, double *slack)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::dciadxmui(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
    out[2] = 0;
}
void Rbcar::dciadumui(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *mui, double *slack)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::dciadumui(...)" << endl;
#endif
    out[0] = mui[0] * u[0];
    out[1] = mui[1] * u[1];
}
void Rbcar::dciadamui(double *out, double t, double *x, double *u, double *p, double *pc, double *xdes, double *udes, double *mui, double *slack)
{
#ifdef DEBUG_FUNCTION_TRACE
    cout << "exec Rbcar::dciadamui(...)" << endl;
#endif
    out[0] = 0;
    out[1] = 0;
}
