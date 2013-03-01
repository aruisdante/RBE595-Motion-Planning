/**
 * @file PathPlotter.h
 *
 * @date   Feb 28, 2013
 * @author parallels
 * @brief  Function definition for path plotting
 */

//License File


#ifndef PATHPLOTTER_H_
#define PATHPLOTTER_H_


//****************SYSTEM DEPENDANCIES**************************//
#include<string>
//*****************LOCAL DEPENDANCIES**************************//
#include "MotionPlanningTree.h"
//**********************NAMESPACES*****************************//

namespace path_utilities
{
/**
 * @author Adam Panzica
 * @brief  Writes out the states in a path to a file
 * @param [in] start_node  Node to start constructing the path from
 * @param [in] num_states  The number of states to print out
 * @param [in] state_list  The names of the states, in the order they are in the state vector and comma seperated
 * @param [in] file_name   Name of the file to write out to
 * @return true if sucessfully written to, else false
 */
bool writeOutPath(const SimpleMotionPlanningTree::SimplePathNode* start_node, int num_states, std::string& state_list, std::string& file_name);

/**
 * @author Adam Panzica
 * @brief  Writes out the states in a tree to a file
 * @param [in] tree        The tree to write out
 * @param [in] num_states  The number of states to print out
 * @param [in] state_list  The names of the states, in the order they are in the state vector and comma seperated
 * @param [in] file_name   Name of the file to write out to
 * @return true if sucessfully written to, else false
 */
bool writeOutTree(const SimpleMotionPlanningTree& tree, int num_states, std::string& state_list, std::string& file_name);
};

#endif /* PATHPLOTTER_H_ */
