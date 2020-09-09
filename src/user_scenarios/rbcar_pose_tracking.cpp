/**
 * @file    scenario.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   User Scenario
 *
 * Scenario is defining containing int main() and defining the control scenario
 */

#include <Scheduler.h>
#include <Agent.h>
#include <Constraint.h>
#include <Coupling.h>

#include <rbcar.h>

#include <Cmscgmres.h>
#include <Event.h>
#include <std_msgs/Bool.h>

int main(int argc, char **argv)
{

	//Initialize ros node
	ros::init(argc, argv, "controller");

	/****** Initialize Agent Instances ******/
	std::vector<Agent *> agentlist;

	//Initialize: Ardrone instance: ardrone1
	Rbcar *rbcar0 = new Rbcar(agentlist.size());

	// rbcar state={x, y, yaw} input={uforward, urotate}
	double rbcar0_init_p[] = {1.0, 1.0, 1.0, 1.0, 0.5};
	// Initial desired pose
	double rbcar0_init_x[] = {0.0, 0.0, 0.0};
	double rbcar0_init_xdes[] = {0.0, 0.0, 0.0};
	rbcar0->setInitialState(rbcar0_init_x);
	rbcar0->setInitialDesiredState(rbcar0_init_xdes);
	rbcar0->setInitialParameter(rbcar0_init_p);

	std::string rbcar_pose_topic, rbcar_desiredpose_topic, rbcar_cmdvel_topic;

	ros::param::get("/rbcar_pose_topic", rbcar_pose_topic);
	ros::param::get("/rbcar_desiredpose_topic", rbcar_desiredpose_topic);
	ros::param::get("/rbcar_cmdvel_topic", rbcar_cmdvel_topic);

	rbcar0->setStateSubscriberRosTopicName(rbcar_pose_topic.c_str());
	rbcar0->setDesiredStateSubscriberRosTopicName(rbcar_desiredpose_topic.c_str());
	rbcar0->setPublisherRosTopicName(rbcar_cmdvel_topic.c_str());

	double max_speed, max_steering_angle, max_acceleration;
	ros::param::get("/max_speed", max_speed);
	ros::param::get("/max_steering_angle", max_steering_angle);
	ros::param::get("/max_acceleration", max_acceleration);
	rbcar0->setMaxConstraints(max_speed, max_steering_angle, max_acceleration);

	double control_period;
	ros::param::get("/control_period", control_period);
	rbcar0->setControlPeriod(control_period);

	double distance_offset, theta_offset;
	ros::param::get("/distance_offset", distance_offset);
	ros::param::get("/theta_offset", theta_offset);
	rbcar0->setOffset(distance_offset, theta_offset);

	agentlist.push_back(rbcar0); /*add to agentlist*/

	/****** Initialize Coupling Instances ******/
	std::vector<Controller *> controllerlist;

	//Initialize: Controller
	double horizon_diskr, horizon_length;
	ros::param::get("/horizon_diskretization", horizon_diskr);
	ros::param::get("/horizon_length", horizon_length);

	Cmscgmres *controller1 = new Cmscgmres(agentlist, controllerlist.size());
	controller1->setHorizonDiskretization(horizon_diskr);
	controller1->setHorizonLength(horizon_length);
	controller1->setTolerance(1e-8);
	controller1->setUpdateIntervall(control_period);
	controller1->setMaximumNumberofIterations(10);
	// controller1->activateInfo_ControllerTrace();
	// controller1->activateInfo_Controller();
	// controller1->startLogging2File();
	// controller1->activateInfo_ControllerStates();
	controllerlist.push_back(controller1);

	// controller1->getX(); // contains the continuous states (X,Y,Z) from position 0 to 19 with step 2

	/****** Initialize Events ******/
	std::vector<Event *> eventlist;

	/****** Initialize Scheduler ******/
	Scheduler scheduler(argc, argv, controllerlist, eventlist);
	scheduler.run_control(control_period);
	// scheduler.run_vrep(0.01);
	//	scheduler.run_simulation(0.1,10);
};
