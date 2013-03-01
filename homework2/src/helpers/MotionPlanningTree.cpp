/**
 * @file MotionPlanningTree.cpp
 *
 * @date   Feb 28, 2013
 * @author parallels
 * @brief  implementation for Motion Planning Tree
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include<iostream>
//*****************LOCAL DEPENDANCIES**************************//
#include "MotionPlanningTree.h"
//**********************NAMESPACES*****************************//

SimpleMotionPlanningTree::SimpleMotionPlanningTree(const ompl::base::SpaceInformationPtr& si)
{
	this->si_ = si;
}

SimpleMotionPlanningTree::~SimpleMotionPlanningTree()
{
	this->flushTree();
}

SimpleMotionPlanningTree::size_type SimpleMotionPlanningTree::size() const
{
	return this->nodes_.size();
}

void SimpleMotionPlanningTree::addNode(SimpleMotionPlanningTree::SimplePathNode* node, size_type& index)
{
	this->nodes_.push_back(node);
	index = this->nodes_.size();
}

SimpleMotionPlanningTree::SimplePathNode* SimpleMotionPlanningTree::addNode(size_type& index)
{
	SimpleMotionPlanningTree::SimplePathNode* new_node = new SimplePathNode(this->si_);
	this->nodes_.push_back(new_node);
	index = this->nodes_.size()-1;
	return new_node;
}

SimpleMotionPlanningTree::SimplePathNode* SimpleMotionPlanningTree::getNode(size_type index) const
{
	if(index < this->nodes_.size())
	{
		return this->nodes_.at(index);
	}
	else
	{
		return NULL;
	}
}

void SimpleMotionPlanningTree::flushTree()
{
	for (size_type index = 0; index < this->nodes_.size(); ++index)
	{
		SimpleMotionPlanningTree::SimplePathNode* node = this->nodes_.at(index);
		if(node!=NULL)
		{
			if(node->node_state_) this->si_->freeState(node->node_state_);
			delete node;
		}

	}
	this->nodes_.clear();
}
