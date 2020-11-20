/**
 * @file    Controller.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Controller Container
 *
 *	Controller represents a Container Class for MPC-Controllers.
 *	It generates the functions required for optimization, such as
 *	the Hamiltonian and its derivatives.
 *	The generation is based on a concatenation of all controlled agents, constraints.
 *	The concatenation is in respect to the chosen constraint handling:
 *	1. Auxiliary Variable Method as implemented by Ohtsuka
 *	where the inequality is transformed to an equality constraint with the help of slack variables
 *	Ohtsuka, T., “A Continuation/GMRES Method for Fast Computation of Nonlinear Receding Horizon Control,” Automatica, Vol. 40, No. 4, Apr. 2004, pp. 563-574.
 *	2. Simple Primal Barrier Method
 *	where the inequality is transformed into stage cost with the help of a logarithmic barrier function.
 */

#include "denmpc/Controller.h"

void Controller::startAgents()
{
	agent_->start();
}

void Controller::setActiveSetWorkingSet(double *mui)
{
	//Set working set
	for (int i = 0; i < dim_ineqcon_conc_; i++)
	{
		workingset_[i] = mui >= 0 ? true : false;
	}
}

void Controller::getMeasurements()
{
	agent_->getDesiredState(xdes_conc_ + agent_->index_xdes_);
	agent_->getDesiredControl(udes_conc_ + agent_->index_udes_);
	agent_->getState(x_conc_ + agent_->index_x_);
	//		agent_->getDisturbance	 (d_conc_+agent_->index_d_);
	agent_->getParameter(p_conc_ + agent_->index_p_);
	//Loop over constraints
	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		tmp_constraint_ptr_->getParameter(p_conc_ + tmp_constraint_ptr_->index_pc_);
	}
}

double *Controller::returnAction()
{
	agent_->setControl(u_conc_ + agent_->index_u_);
	return agent_->getControl_Ptr();
};

Controller::~Controller()
{
	//Typicall u,mu,mui,slack are concatenated in the optvar_conc_
	MathLib::free(optvar_conc_);
	MathLib::free(x_conc_);
	u_conc_ = NULL;
	MathLib::free(xdes_conc_);
	MathLib::free(udes_conc_);
	MathLib::free(d_conc_);
	MathLib::free(p_conc_);
	MathLib::free(lambda_conc_);
	mu_conc_ = NULL;
	mui_conc_ = NULL;
	slack_conc_ = NULL;
	MathLib::free(slack_penalty_conc_);
}

Controller::Controller(Agent *_agent, int _id = 0)
{
	id_ = _id;
	agent_ = _agent;
	primalbarrierfactor_ = 0.0001; //0.001;
	exteriorpenaltyfactor_ = 100;
	mu_init_ = 0.0001;
	mui_init_ = 0.0001;
	slack_init_ = 0.01;
	slack_penalty_init_ = 0.01;

	// log_stringstream_ << "Controller Log File:" << std::endl;

	flag_show_controllerinfo_ = false; // true
	flag_show_controllerstates_ = false;
	flag_save_controllerlog_ = false;
	flag_show_controllertrace_ = false;

	if (flag_show_controllertrace_)
	{
		flag_show_controllerinfo_ = true;
		flag_show_controllerstates_ = true;
	}

	time_t currenttimestamp;
	tm *now;
	currenttimestamp = time(0);
	now = localtime(&currenttimestamp);
	std::stringstream filename;
	filename << now->tm_year + 1900 << boost::format("%|02|") % (now->tm_mon + 1)
			 << boost::format("%|02|") % (now->tm_mday) << "_"
			 << boost::format("%|02|") % (now->tm_hour)
			 << boost::format("%|02|") % (now->tm_min)
			 << boost::format("%|02|") % (now->tm_sec) << "_"
			 << "controller" << id_ << ".log";
	log_filename_ = filename.str();

	constrainthandlingmethod_ = METHOD_PRIMALBARRIER;
	// constrainthandlingmethod_=METHOD_EXTERIORPENALTY;
	// constrainthandlingmethod_ = METHOD_AUXILIARYVARIABLE;
	// constrainthandlingmethod_ = METHOD_ACTIVESET;

	//Set Agent states to initial values
	agent_->reset2initialstate();

	//Initialize Concatenation Indices
	initConcatenation();

	//	//Save Agent Arrays in concatenated arrays
	//	initArrayValues();
}

void Controller::initConcatenation()
{
	dim_x_conc_ = 0;
	dim_u_conc_ = 0;
	dim_xdes_conc_ = 0;
	dim_udes_conc_ = 0;
	dim_d_conc_ = 0;
	dim_p_conc_ = 0;
	dim_lambda_conc_ = 0;
	dim_eqcon_conc_ = 0;
	dim_ineqcon_conc_ = 0;

	//Get index
	agent_->initIndex(dim_x_conc_, dim_u_conc_, dim_xdes_conc_, dim_udes_conc_, dim_d_conc_, dim_p_conc_, dim_lambda_conc_);
	dim_x_conc_ += agent_->getState_Dim();
	dim_u_conc_ += agent_->getControl_Dim();
	dim_xdes_conc_ += agent_->getDesiredState_Dim();
	dim_udes_conc_ += agent_->getDesiredControl_Dim();
	dim_d_conc_ += agent_->getDisturbance_Dim();
	dim_p_conc_ += agent_->getParameter_Dim();
	dim_lambda_conc_ += agent_->getState_Dim();

	//Loop through constraints
	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		//Get index
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		tmp_constraint_ptr_->initIndex(
			agent_->index_x_,
			agent_->index_u_,
			agent_->index_xdes_,
			agent_->index_udes_,
			agent_->index_p_,
			dim_p_conc_,
			dim_eqcon_conc_,
			dim_ineqcon_conc_);
		//Concatenate equality lagrangemultipliers
		dim_eqcon_conc_ += agent_->constraint_[it_constraint]->getEqualityConstraint_Dim();
		//Concatenate inequality lagrangemultipliers
		dim_ineqcon_conc_ += agent_->constraint_[it_constraint]->getInequalityConstraint_Dim();
		//Concatenate parameters
		dim_p_conc_ += tmp_constraint_ptr_->getParameter_Dim();
	}
	switch (constrainthandlingmethod_)
	{
	case METHOD_PRIMALBARRIER:
		//The optimized variables are inputs and equality constraint multipliers
		dim_optvar_conc_ = dim_u_conc_ + dim_eqcon_conc_;
		optvar_conc_ = MathLib::defvector(dim_optvar_conc_);
		x_conc_ = MathLib::defvector(dim_x_conc_);
		u_conc_ = optvar_conc_;
		d_conc_ = MathLib::defvector(dim_d_conc_);
		p_conc_ = MathLib::defvector(dim_p_conc_);
		xdes_conc_ = MathLib::defvector(dim_xdes_conc_);
		udes_conc_ = MathLib::defvector(dim_udes_conc_);
		lambda_conc_ = MathLib::defvector(dim_lambda_conc_);
		mu_conc_ = optvar_conc_ + dim_u_conc_;
		//INITIALIZE

		//Get values from agent and store them in concatenated arrays
		agent_->getState(x_conc_ + agent_->index_x_);
		agent_->getControl(u_conc_ + agent_->index_u_);
		agent_->getDesiredState(xdes_conc_ + agent_->index_xdes_);
		agent_->getDesiredControl(udes_conc_ + agent_->index_udes_);
		agent_->getParameter(p_conc_ + agent_->index_p_);
		//Loop over constraints
		for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
		{
			tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
			tmp_constraint_ptr_->getParameter(p_conc_ + tmp_constraint_ptr_->index_pc_);
		}

		//Initialize mu
		for (int it_eq = 0; it_eq < dim_eqcon_conc_; it_eq++)
		{
			mu_conc_[it_eq] = mu_init_;
		}

		break;

	case METHOD_AUXILIARYVARIABLE:
		//The optimized variables are inputs and equality constraint multipliers, inequality constraint multipliers and slack variables
		dim_optvar_conc_ = dim_u_conc_ + dim_eqcon_conc_ + 2 * dim_ineqcon_conc_;
		optvar_conc_ = MathLib::defvector(dim_optvar_conc_);
		x_conc_ = MathLib::defvector(dim_x_conc_);
		u_conc_ = optvar_conc_;
		xdes_conc_ = MathLib::defvector(dim_xdes_conc_);
		udes_conc_ = MathLib::defvector(dim_udes_conc_);
		d_conc_ = MathLib::defvector(dim_d_conc_);
		p_conc_ = MathLib::defvector(dim_p_conc_);
		lambda_conc_ = MathLib::defvector(dim_lambda_conc_);
		mu_conc_ = optvar_conc_ + dim_u_conc_;
		mui_conc_ = optvar_conc_ + dim_u_conc_ + dim_eqcon_conc_;
		slack_conc_ = optvar_conc_ + dim_u_conc_ + dim_eqcon_conc_ + dim_ineqcon_conc_;
		slack_penalty_conc_ = MathLib::defvector(dim_ineqcon_conc_);
		//INITIALIZE
		//Get values from agent and store them in concatenated arrays
		agent_->getState(x_conc_ + agent_->index_x_);
		agent_->getControl(u_conc_ + agent_->index_u_);
		agent_->getDesiredState(xdes_conc_ + agent_->index_xdes_);
		agent_->getDesiredControl(udes_conc_ + agent_->index_udes_);
		agent_->getParameter(p_conc_ + agent_->index_p_);
		//Loop over constraints
		for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
		{
			tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
			tmp_constraint_ptr_->getParameter(p_conc_ + tmp_constraint_ptr_->index_pc_);
		}

		//Initialize mu
		for (int it_eq = 0; it_eq < dim_eqcon_conc_; it_eq++)
		{
			mu_conc_[it_eq] = mu_init_;
		}
		//Initialize mui
		for (int it_ineq = 0; it_ineq < dim_ineqcon_conc_; it_ineq++)
		{
			mui_conc_[it_ineq] = mui_init_;
		}
		//Initialize slack
		for (int it_ineq = 0; it_ineq < dim_ineqcon_conc_; it_ineq++)
		{
			slack_conc_[it_ineq] = slack_init_;
			slack_penalty_conc_[it_ineq] = slack_penalty_init_;
		}
		break;

	case METHOD_ACTIVESET:
		//The optimized variables are inputs and equality constraint multipliers, inequality constraint multipliers
		dim_optvar_conc_ = dim_u_conc_ + dim_eqcon_conc_ + dim_ineqcon_conc_;
		optvar_conc_ = MathLib::defvector(dim_optvar_conc_);
		x_conc_ = MathLib::defvector(dim_x_conc_);
		u_conc_ = optvar_conc_;
		xdes_conc_ = MathLib::defvector(dim_xdes_conc_);
		udes_conc_ = MathLib::defvector(dim_udes_conc_);
		p_conc_ = MathLib::defvector(dim_p_conc_);
		d_conc_ = MathLib::defvector(dim_d_conc_);
		lambda_conc_ = MathLib::defvector(dim_lambda_conc_);
		mu_conc_ = optvar_conc_ + dim_u_conc_;
		mui_conc_ = optvar_conc_ + dim_u_conc_ + dim_eqcon_conc_;
		workingset_ = MathLib::defboolvector(dim_ineqcon_conc_);

		//INITIALIZE
		//Get values from agent and store them in concatenated arrays
		agent_->getState(x_conc_ + agent_->index_x_);
		agent_->getControl(u_conc_ + agent_->index_u_);
		agent_->getDesiredState(xdes_conc_ + agent_->index_xdes_);
		agent_->getDesiredControl(udes_conc_ + agent_->index_udes_);
		agent_->getParameter(p_conc_ + agent_->index_p_);
		//Loop over constraints
		for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
		{
			tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
			tmp_constraint_ptr_->getParameter(p_conc_ + tmp_constraint_ptr_->index_pc_);
		}

		//Initialize mu
		for (int it_eq = 0; it_eq < dim_eqcon_conc_; it_eq++)
		{
			mu_conc_[it_eq] = mu_init_;
		}
		//Initialize mui
		for (int it_ineq = 0; it_ineq < dim_ineqcon_conc_; it_ineq++)
		{
			mui_conc_[it_ineq] = mui_init_;
		}

		break;

	case METHOD_EXTERIORPENALTY:
		//The optimized variables are inputs and equality constraint multipliers
		dim_optvar_conc_ = dim_u_conc_ + dim_eqcon_conc_;
		optvar_conc_ = MathLib::defvector(dim_optvar_conc_);
		x_conc_ = MathLib::defvector(dim_x_conc_);
		u_conc_ = optvar_conc_;
		d_conc_ = MathLib::defvector(dim_d_conc_);
		p_conc_ = MathLib::defvector(dim_p_conc_);
		xdes_conc_ = MathLib::defvector(dim_xdes_conc_);
		udes_conc_ = MathLib::defvector(dim_udes_conc_);
		lambda_conc_ = MathLib::defvector(dim_lambda_conc_);
		mu_conc_ = optvar_conc_ + dim_u_conc_;
		//INITIALIZE
		//Get values from agent and store them in concatenated arrays
		agent_->getState(x_conc_ + agent_->index_x_);
		agent_->getControl(u_conc_ + agent_->index_u_);
		agent_->getDesiredState(xdes_conc_ + agent_->index_xdes_);
		agent_->getDesiredControl(udes_conc_ + agent_->index_udes_);
		agent_->getParameter(p_conc_ + agent_->index_p_);
		//Loop over constraints
		for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
		{
			tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
			tmp_constraint_ptr_->getParameter(p_conc_ + tmp_constraint_ptr_->index_pc_);
		}

		//Initialize mu
		for (int it_eq = 0; it_eq < dim_eqcon_conc_; it_eq++)
		{
			mu_conc_[it_eq] = mu_init_;
		}

		break;

	default:
		std::cout << "!!! ERROR: COMPOSER::init() NO VALID INEQUALITY CONSTRAINT METHOD CHOSEN" << std::endl;
	}
}

/***** system function f *****/
void Controller::f(double *out, double t, double *x, double *u, double *d, double *p)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::f()" << std::endl;
#endif
	agent_->f(out + agent_->index_x_, t, x + agent_->index_x_, u + agent_->index_u_, d + agent_->index_d_, p + agent_->index_p_);
}
void Controller::dfdxlambda(double *out, double t, double *x, double *u, double *d, double *p, double *lambda)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dfdx_lambda()" << std::endl;
#endif
	double tmp_x[dim_x_conc_];

	//Check if state derivative is non-zero
	if (agent_->dim_x_ > 0)
	{
		//Get function values
		agent_->dfdxlambda(
			tmp_x + agent_->index_x_,
			t,
			x + agent_->index_x_,
			u + agent_->index_u_,
			d + agent_->index_d_,
			p + agent_->index_p_,
			lambda + agent_->index_lambda_);
		//Save tmp to output and reset tmp
		for (int it_state = agent_->index_x_; it_state < agent_->index_x_ + agent_->dim_x_; it_state++)
		{
			out[it_state] += tmp_x[it_state];
		}
	}
}
void Controller::dfdulambda(double *out, double t, double *x, double *u, double *d, double *p, double *lambda)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dfdu_lambda()" << std::endl;
#endif
	double tmp_u[dim_u_conc_];
	//Check if state derivative is non-zero
	if (agent_->dim_u_ > 0)
	{
		//Get function values
		agent_->dfdulambda(
			tmp_u + agent_->index_u_,
			t,
			x + agent_->index_x_,
			u + agent_->index_u_,
			d + agent_->index_d_,
			p + agent_->index_p_,
			lambda + agent_->index_lambda_);
		//Save tmp to output
		for (int it_control = agent_->index_u_; it_control < agent_->index_u_ + agent_->dim_u_; it_control++)
		{
			out[it_control] += tmp_u[it_control];
		}
	}
}
/***** state cost l *****/
void Controller::l(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::l()" << std::endl;
#endif
	double tmp = 0;
	*out = 0;
	agent_->l(&tmp, t, x + agent_->index_x_, u + agent_->index_u_, p + agent_->index_p_, xdes + agent_->index_xdes_, udes + agent_->index_udes_);
	*out += tmp;
	tmp = 0;
	//}
	//Loop over constraints in agents
	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_l_ > 0)
		{
			tmp_constraint_ptr_->l(&tmp, t, x + tmp_constraint_ptr_->index_x_, u + tmp_constraint_ptr_->index_u_, p + tmp_constraint_ptr_->index_p_, p + tmp_constraint_ptr_->index_pc_, xdes + tmp_constraint_ptr_->index_xdes_, udes + tmp_constraint_ptr_->index_udes_);
			*out += tmp;
			tmp = 0;
		}
	}
}
void Controller::dldx(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dldx()" << std::endl;
#endif
	double tmp_x[dim_x_conc_];
	memset(out, 0, dim_x_conc_ * sizeof(double));
	memset(tmp_x, 0, dim_x_conc_ * sizeof(double));
	//	for(int it_state=0; it_state<dim_x_conc_; it_state++){tmp_x[it_state]=0;out[it_state]=0;};

	if (agent_->dim_x_ > 0)
	{
		//Get function values
		agent_->dldx(
			tmp_x + agent_->index_x_,
			t,
			x + agent_->index_x_,
			u + agent_->index_u_,
			p + agent_->index_p_,
			xdes + agent_->index_xdes_,
			udes + agent_->index_udes_);
		//Save tmp to output and reset tmp
		for (int it_state = agent_->index_x_; it_state < agent_->index_x_ + agent_->dim_x_; it_state++)
		{
			out[it_state] += tmp_x[it_state];
		}
	}
	//Loop over constraints in agents
	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_l_ > 0 && tmp_constraint_ptr_->dim_x_ > 0)
		{
			//Get function values
			tmp_constraint_ptr_->dldx(
				tmp_x + tmp_constraint_ptr_->index_x_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_);
			//Save tmp to output and reset tmp
			for (int it_state = tmp_constraint_ptr_->index_x_; it_state < tmp_constraint_ptr_->index_x_ + agent_->dim_x_; it_state++)
			{
				out[it_state] += tmp_x[it_state];
			}
		}
	}
}
void Controller::dldu(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dldu()" << std::endl;
#endif
	double tmp_u[dim_u_conc_];
	memset(out, 0, dim_u_conc_ * sizeof(double));
	memset(tmp_u, 0, dim_u_conc_ * sizeof(double));
	//	for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;out[it_control]=0;};

	if (agent_->dim_u_ > 0)
	{
		//Get function values
		agent_->dldu(
			tmp_u + agent_->index_u_,
			t,
			x + agent_->index_x_,
			u + agent_->index_u_,
			p + agent_->index_p_,
			xdes + agent_->index_xdes_,
			udes + agent_->index_udes_);
		//Save tmp to output and reset tmp
		for (int it_control = agent_->index_u_; it_control < agent_->index_u_ + agent_->dim_u_; it_control++)
		{
			out[it_control] += tmp_u[it_control];
		}
	}
	//Loop over constraints in agents
	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_l_ > 0 && tmp_constraint_ptr_->dim_u_ > 0)
		{
			//Get function values
			agent_->constraint_[it_constraint]->dldu(
				tmp_u + tmp_constraint_ptr_->index_u_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_);
			//Add tmp to out and reset tmp
			for (int it_control = tmp_constraint_ptr_->index_u_; it_control < tmp_constraint_ptr_->index_u_ + agent_->dim_u_; it_control++)
			{
				out[it_control] += tmp_u[it_control];
			}
		}
	}
}
/***** final cost v *****/
void Controller::v(double *out, double t, double *x, double *p, double *xdes)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::v()" << std::endl;
#endif
	double tmp = 0;
	*out = 0;
	agent_->v(&tmp, t, x + agent_->index_x_, p + agent_->index_p_, xdes + agent_->index_xdes_);
	*out += tmp;
	tmp = 0;
	//Loop over constraints in agents
	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_v_ > 0)
		{
			tmp_constraint_ptr_->v(&tmp, t, x + tmp_constraint_ptr_->index_x_, p + tmp_constraint_ptr_->index_p_, p + tmp_constraint_ptr_->index_pc_, xdes + tmp_constraint_ptr_->index_xdes_);
			*out += tmp;
			tmp = 0;
		}
	}
}
void Controller::dvdx(double *out, double t, double *x, double *p, double *xdes)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dvdx()" << std::endl;
#endif
	double tmp_x[dim_x_conc_];
	memset(out, 0, dim_x_conc_ * sizeof(double));
	memset(tmp_x, 0, dim_x_conc_ * sizeof(double));
	//	for(int it_state=0; it_state<dim_x_conc_; it_state++){tmp_x[it_state]=0;out[it_state]=0;};

	if (agent_->dim_x_ > 0)
	{
		//Get function values
		agent_->dvdx(
			tmp_x + agent_->index_x_,
			t,
			x + agent_->index_x_,
			p + agent_->index_p_,
			xdes + agent_->index_xdes_);
		//Save tmp to output
		for (int it_state = agent_->index_x_; it_state < agent_->index_x_ + agent_->dim_x_; it_state++)
		{
			out[it_state] += tmp_x[it_state];
		}
	}
	//Loop over constraints in agents
	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->getFinalCost_Dim() > 0 && tmp_constraint_ptr_->dim_x_ > 0)
		{
			//Get function values
			tmp_constraint_ptr_->dvdx(
				tmp_x + tmp_constraint_ptr_->index_x_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_);
			//Add tmp to out and reset tmp
			for (int it_state = tmp_constraint_ptr_->index_x_; it_state < tmp_constraint_ptr_->index_x_ + agent_->dim_x_; it_state++)
			{
				out[it_state] += tmp_x[it_state];
			}
		}
	}
}
/***** equality constraint *****/
void Controller::c(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::c()" << std::endl;
#endif

	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_eq_ > 0)
		{
			//Call function
			tmp_constraint_ptr_->c(
				out + tmp_constraint_ptr_->index_mu_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_);
		}
	}
}
void Controller::dcdxmu(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *mu)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dcdx_mu()" << std::endl;
#endif
	double tmp_x[dim_x_conc_];
	memset(out, 0, dim_x_conc_ * sizeof(double));
	memset(tmp_x, 0, dim_x_conc_ * sizeof(double));
	//	for(int it_state=0; it_state<dim_x_conc_; it_state++){tmp_x[it_state]=0;out[it_state]=0;};

	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_eq_ > 0 && tmp_constraint_ptr_->dim_x_ > 0)
		{
			//Call function
			tmp_constraint_ptr_->dcdxmu(
				tmp_x + tmp_constraint_ptr_->index_x_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_,
				mu + tmp_constraint_ptr_->index_mu_);
			//Add tmp to out and reset tmp
			for (int it_state = tmp_constraint_ptr_->index_x_; it_state < tmp_constraint_ptr_->index_x_ + agent_->dim_x_; it_state++)
			{
				out[it_state] += tmp_x[it_state];
			}
		}
	}
}
void Controller::dcdumu(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *mu)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dcdu_mu()" << std::endl;
#endif
	double tmp_u[dim_u_conc_];
	memset(out, 0, dim_u_conc_ * sizeof(double));
	memset(tmp_u, 0, dim_u_conc_ * sizeof(double));
	//	for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;out[it_control]=0;};

	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_eq_ > 0 && tmp_constraint_ptr_->dim_u_ > 0)
		{
			//Call function
			tmp_constraint_ptr_->dcdumu(
				tmp_u + tmp_constraint_ptr_->index_u_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_,
				mu + tmp_constraint_ptr_->index_mu_);
			//Initialize to tmp vector to zero
			for (int it_control = tmp_constraint_ptr_->index_u_; it_control < tmp_constraint_ptr_->index_u_ + agent_->dim_u_; it_control++)
			{
				out[it_control] += dim_u_conc_[tmp_u];
			}
		}
	}
}
/***** inequality constraint *****/
void Controller::ci(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::ci()" << std::endl;
#endif

	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_ineq_ > 0)
		{
			//Call function
			tmp_constraint_ptr_->ci(
				out + tmp_constraint_ptr_->index_mui_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_);
		}
	}
}
void Controller::dcidxmui(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dcidx_mui()" << std::endl;
#endif
	double tmp_x[dim_x_conc_];
	memset(out, 0, dim_x_conc_ * sizeof(double));
	memset(tmp_x, 0, dim_x_conc_ * sizeof(double));
	//	for(int it_state=0; it_state<dim_x_conc_; it_state++){tmp_x[it_state]=0;out[it_state]=0;};

	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_ineq_ > 0 && tmp_constraint_ptr_->dim_x_ > 0)
		{
			//Call function
			tmp_constraint_ptr_->dcidxmui(
				tmp_x + tmp_constraint_ptr_->index_x_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_,
				mui + tmp_constraint_ptr_->index_mui_);
			//Initialize to tmp vector to zero
			for (int it_state = tmp_constraint_ptr_->index_x_; it_state < tmp_constraint_ptr_->index_x_ + agent_->dim_x_; it_state++)
			{
				out[it_state] += tmp_x[it_state];
			}
		}
	}
}
void Controller::dcidumui(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dcidu_mui()" << std::endl;
#endif
	double tmp_u[dim_u_conc_];
	memset(out, 0, dim_u_conc_ * sizeof(double));
	memset(tmp_u, 0, dim_u_conc_ * sizeof(double));
	//	for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;out[it_control]=0;};

	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_ineq_ > 0 && tmp_constraint_ptr_->dim_u_ > 0)
		{
			//Call function
			tmp_constraint_ptr_->dcidumui(
				tmp_u + tmp_constraint_ptr_->index_u_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_,
				mui + tmp_constraint_ptr_->index_mui_);
			//Initialize to tmp vector to zero
			for (int it_control = tmp_constraint_ptr_->index_u_; it_control < tmp_constraint_ptr_->index_u_ + agent_->dim_u_; it_control++)
			{
				out[it_control] += tmp_u[it_control];
			}
		}
	}
}

/***** inequality constraint *****/
void Controller::cia(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *slack)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::ci()" << std::endl;
#endif

	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_ineq_ > 0)
		{
			//Call function
			tmp_constraint_ptr_->cia(
				out + tmp_constraint_ptr_->index_mui_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_,
				slack + tmp_constraint_ptr_->index_mui_);
		}
	}
}
void Controller::dciadxmui(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui, double *slack)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dcidx_mui()" << std::endl;
#endif
	double tmp_x[dim_x_conc_];
	memset(out, 0, dim_x_conc_ * sizeof(double));
	memset(tmp_x, 0, dim_x_conc_ * sizeof(double));
	//	for(int it_state=0; it_state<dim_x_conc_; it_state++){tmp_x[it_state]=0;out[it_state]=0;};

	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_ineq_ > 0 && tmp_constraint_ptr_->dim_x_ > 0)
		{
			//Call function
			tmp_constraint_ptr_->dciadxmui(
				tmp_x + tmp_constraint_ptr_->index_x_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_,
				mui + tmp_constraint_ptr_->index_mui_,
				slack + tmp_constraint_ptr_->index_mui_);
			//Initialize to tmp vector to zero
			for (int it_state = tmp_constraint_ptr_->index_x_; it_state < tmp_constraint_ptr_->index_x_ + agent_->dim_x_; it_state++)
			{
				out[it_state] += tmp_x[it_state];
			}
		}
	}
}
void Controller::dciadumui(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui, double *slack)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dcidu_mui()" << std::endl;
#endif
	double tmp_u[dim_u_conc_];
	memset(out, 0, dim_u_conc_ * sizeof(double));
	memset(tmp_u, 0, dim_u_conc_ * sizeof(double));
	//	for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;out[it_control]=0;};

	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_ineq_ > 0 && tmp_constraint_ptr_->dim_u_ > 0)
		{
			//Call function
			tmp_constraint_ptr_->dciadumui(
				tmp_u + tmp_constraint_ptr_->index_u_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_,
				mui + tmp_constraint_ptr_->index_mui_,
				slack + tmp_constraint_ptr_->index_mui_);
			//Initialize to tmp vector to zero
			for (int it_control = tmp_constraint_ptr_->index_u_; it_control < tmp_constraint_ptr_->index_u_ + agent_->dim_u_; it_control++)
			{
				out[it_control] += tmp_u[it_control];
			}
		}
	}
}

void Controller::dciadamui(double *out, double t, double *x, double *u, double *p, double *xdes, double *udes, double *mui, double *slack)
{
#ifdef DEBUG_FUNCTION_TRACE
	std::cout << "exec Controller::dcidu_mui()" << std::endl;
#endif
	double tmp_u[dim_u_conc_];
	memset(out, 0, dim_u_conc_ * sizeof(double));
	memset(tmp_u, 0, dim_u_conc_ * sizeof(double));
	//	for(int it_control=0; it_control<dim_u_conc_; it_control++){tmp_u[it_control]=0;out[it_control]=0;};

	for (int it_constraint = 0; it_constraint < agent_->getConstraint_Dim(); it_constraint++)
	{
		tmp_constraint_ptr_ = agent_->constraint_[it_constraint];
		if (tmp_constraint_ptr_->dim_ineq_ > 0 && tmp_constraint_ptr_->dim_u_ > 0)
		{
			//Call function
			tmp_constraint_ptr_->dciadamui(
				out + tmp_constraint_ptr_->index_mui_,
				t,
				x + tmp_constraint_ptr_->index_x_,
				u + tmp_constraint_ptr_->index_u_,
				p + tmp_constraint_ptr_->index_p_,
				p + tmp_constraint_ptr_->index_pc_,
				xdes + tmp_constraint_ptr_->index_xdes_,
				udes + tmp_constraint_ptr_->index_udes_,
				mui + tmp_constraint_ptr_->index_mui_,
				slack + tmp_constraint_ptr_->index_mui_);
			//Initialize to tmp vector to zero
		}
	}
}
