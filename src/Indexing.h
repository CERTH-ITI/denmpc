/**
 * @file    Indexing.cpp
 * @Author  Jan Dentler (jan.dentler@uni.lu)
 *          University of Luxembourg
 * @date    27.February, 2017
 * @time    23:23h
 * @license GPLv3
 * @brief   Index Container
 *
 * Indexing represents the container class for all indices
 * within the concatenated optimization variables vectors.
 */

#ifndef _INDEXING_H
#define _INDEXING_H

struct AgentIndex
{
	//Pointers
	int index_x_;	   //Index of state
	int index_u_;	   //Index of control
	int index_xdes_;   //Index of desired state
	int index_udes_;   //Index of desired control
	int index_d_;	   //Index of disturbance
	int index_p_;	   //Index of parameters
	int index_lambda_; //index of state lagrange multiplier

	void initIndex(int _index_x, int _index_u, int _index_xdes, int _index_udes, int _index_d, int _index_p, int _index_lambda)
	{
		index_x_ = _index_x;
		index_u_ = _index_u;
		index_xdes_ = _index_xdes;
		index_udes_ = _index_udes;
		index_d_ = _index_d;
		index_p_ = _index_p;
		index_lambda_ = _index_lambda;
	}
};

struct ConstraintIndex
{
	//Pointers
	int index_x_;	 //Index of state
	int index_u_;	 //Index of control
	int index_xdes_; //Index of desired state
	int index_udes_; //Index of desired control
	int index_p_;	 //Index of parameters
	int index_pc_;	 //Index of parameters
	int index_mu_;	 //index of equality lagrange multiplier
	int index_mui_;	 //index of inequality lagrange multiplier

	void initIndex(int _index_x, int _index_u, int _index_xdes, int _index_udes, int _index_p, int _index_pc, int _index_mu, int _index_mui)
	{
		index_x_ = _index_x;
		index_u_ = _index_u;
		index_xdes_ = _index_xdes;
		index_udes_ = _index_udes;
		index_p_ = _index_p;
		index_pc_ = _index_pc;
		index_mu_ = _index_mu;
		index_mui_ = _index_mui;
	}
};

#endif
