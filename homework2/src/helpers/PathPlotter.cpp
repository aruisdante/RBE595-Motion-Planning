/**
 * @file PathPlotter.cpp
 *
 * @date   Feb 28, 2013
 * @author parallels
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include <iostream>
#include <fstream>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
//*****************LOCAL DEPENDANCIES**************************//
#include "PathPlotter.h"
#include "BenchmarkConfigurationPaths.h"
//**********************NAMESPACES*****************************//

typedef const SimpleMotionPlanningTree::SimplePathNode* NodePtr;
/**
 * @author Adam Panzica
 * @brief  Writes out a state to a line in the output file
 * @param num_state The number of states to write out
 * @param node      The node to write the state out to
 * @param file      ofstream to write the data to
 * @return true if sucessfully written, else false
 */
bool writeOutState(int num_state, const SimpleMotionPlanningTree::SimplePathNode* node, std::ofstream& file)
{
	if(file.is_open())
	{
		const ompl::base::State *state =  node->node_state_;
		const ompl::base::RealVectorStateSpace::StateType* realVectorState =state->as<ompl::base::RealVectorStateSpace::StateType>();
		std::stringstream state_info;
		state_info<<num_state;
		double x     = (*realVectorState)[0];
		double y     = (*realVectorState)[1];
		double theta = (*realVectorState)[2];

		//std::cout<<x<<","<<y<<","<<theta<<std::endl;

		file<<num_state<<","<<x<<","<<y<<","<<theta<<std::endl;
		return true;
	}
	else
	{
		return false;
	}
}

bool writeOutHeader(std::string& state_list, std::ofstream& file)
{
	if(file.is_open())
	{
		std::stringstream header;
		header<<"Number_Of_States,"<<state_list<<std::endl;
		file<<header.str();
		return true;
	}
	else
	{
		return false;
	}
}


bool path_utilities::writeOutPath(const SimpleMotionPlanningTree::SimplePathNode* start_node, int num_states, std::string& state_list, std::string& file_name)
{
	std::stringstream file_path;
	file_path<<RESULT_PATH<<"/"<<file_name<<".txt";
	//std::cout<<"...attempting to open "<<file_path.str()<<"...";
	std::ofstream output_file(file_path.str().c_str(), std::ios::out|std::ios::trunc);
	;

	bool success = true;
	//std::cout<<"Plot Debug...";
	if (output_file.is_open())
	{
		//std::cout<<"...file was open...";
		success &= writeOutHeader(state_list, output_file);
		if(success)
		{
			//std::cout<<"...wrote headers...";
			NodePtr next_node = start_node;
			while(next_node != NULL)
			{
				success  &= writeOutState(num_states, next_node, output_file);
				next_node = next_node->parent_node_;
			}
		}
		output_file.close();
		return success;
	}
	else
	{
		return false;
	}
}

