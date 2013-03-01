/**
 * @file RandomTreePlanner.cpp
 *
 * @date   Feb 26, 2013
 * @author parallels
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <limits>
#include <iostream>
//*****************LOCAL DEPENDANCIES**************************//
#include"RandomTreePlanner.h"
#include "PathPlotter.h"
//**********************NAMESPACES*****************************//


using namespace ompl;

RandomTreePlanner::RandomTreePlanner(const base::SpaceInformationPtr& si): base::Planner(si, "RTP"), node_tree_(this->si_)
{
	//Set a default goal sampling bias of 0.5
	this->goal_bias_                  = RTP_DEFAULT_GOAL_BIAS;
	this->maxDistance_                 = 0.0;
	this->lastGoalMotion_             = NULL;
	// the specifications of this planner (ompl::base::PlannerSpecs)
	this->specs_.directed             = true;
	this->specs_.approximateSolutions = true;

	//Registers the goal bias as a parameter of the planner
	Planner::declareParam<double>("goal_bias", this, &RandomTreePlanner::setBias, &RandomTreePlanner::getBias, "0.:.05:1.");
	Planner::declareParam<double>("range", this, &RandomTreePlanner::setRange, &RandomTreePlanner::getRange, "0.:1.:10000.");
}

RandomTreePlanner::~RandomTreePlanner(void)
{
	this->freeMemory();
}
base::PlannerStatus RandomTreePlanner::solve(const base::PlannerTerminationCondition &ptc)
{
	SimpleMotionPlanningTree::size_type node_index;
	// make sure the planner is configured correctly; ompl::base::Planner::checkValidity
	// ensures that there is at least one input state and a ompl::base::Goal object specified
	checkValidity();
	// get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
	base::Goal                 *goal   = this->pdef_->getGoal().get();
	base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);
	// get input states with PlannerInputStates helper, pis_
	NodePtr start_node = NULL;
	while (const base::State *st = pis_.nextStart())
	{
		start_node = this->node_tree_.addNode(node_index);
		this->si_->copyState(start_node->node_state_, st);
		start_node->parent_node_= NULL;
	}

	//If we didn't get any valid start states, we're done
	if(this->node_tree_.size()==0)
	{
		OMPL_ERROR("Did not get a valid starting condition!");
		return base::PlannerStatus::INVALID_START;
	}

	if (!sampler_) sampler_ = si_->allocStateSampler();

	OMPL_INFORM("Starting with %d initial states!", this->node_tree_.size());

	double  approxdif    = std::numeric_limits<double>::infinity();
	NodePtr       solution  = NULL;
	NodePtr       approx_sol = NULL;
	NodePtr       n_rand = new SimpleMotionPlanningTree::SimplePathNode(this->si_);
	base::State*  q_rand = n_rand->node_state_;
	base::State*  xstate = si_->allocState();

	while (ptc() == false)
	{
		/* sample random state (with goal biasing) */
		if (goal_s && this->ran_gen_.uniform01() < this->goal_bias_ && goal_s->canSample())
			goal_s->sampleGoal(q_rand);
		else
			sampler_->sampleUniform(q_rand);

		NodePtr tree_node = NULL;
		//get a random node from the tree
		while(tree_node == NULL)
		{
			SimpleMotionPlanningTree::size_type index = std::floor(((this->node_tree_.size()-1)*this->ran_gen_.uniform01()));
			//std::cout<<"Tree Size: "<<this->node_tree_.size()<<", Tree Index:"<<index<<std::endl;
			tree_node = this->node_tree_.getNode(index);
		}
		//Extending state
		base::State *dstate = q_rand;
		double d = si_->distance(tree_node->node_state_, q_rand);
		if (d > this->maxDistance_)
		{
			si_->getStateSpace()->interpolate(tree_node->node_state_, q_rand, this->maxDistance_ / d, xstate);
			dstate = xstate;
		}

		//Check to see if the connection is valid
		if (si_->checkMotion(tree_node->node_state_, dstate))
		{
			NodePtr new_node = this->node_tree_.addNode(node_index);
			si_->copyState(new_node->node_state_, dstate);
			new_node->parent_node_ = tree_node;

			double dist = 0.0;
			bool sat = goal->isSatisfied(new_node->node_state_, &dist);
			if (sat)
			{
				approxdif = dist;
				solution  = new_node;
				break;
			}
			if (dist < approxdif)
			{
				approxdif  = dist;
				approx_sol = new_node;
			}

		}

	}
	bool solved = false;
	bool approximate = false;
	if (solution == NULL)
	{
		solution    = approx_sol;
		approximate = true;
	}

	//Write out the solution path;
	std::string filename("Random_Tree_Planner_Results");
	std::string state_list("x,y,theta");
	std::cout<<"Plotting Path..."<<(path_utilities::writeOutPath(solution, 3, state_list, filename)?"Succeded":"Failed")<<std::endl;

	if (solution != NULL)
	{
		lastGoalMotion_ = solution;

		/* construct the solution path */
		std::vector<NodePtr> mpath;
		while (solution != NULL)
		{
			mpath.push_back(solution);
			solution = solution->parent_node_;
		}

		std::cout<<"Solution Path Size: "<<mpath.size()<<", Tree Size: "<<this->node_tree_.size()<<std::endl;

		/* set the solution path */
		geometric::PathGeometric *path = new geometric::PathGeometric(si_);
		for (int i = mpath.size() - 1 ; i >= 0 ; --i)
			path->append(mpath[i]->node_state_);
		pdef_->addSolutionPath(base::PathPtr(path), approximate, approxdif);
		solved = true;
	}

	si_->freeState(xstate);
	if (n_rand->node_state_)
		si_->freeState(n_rand->node_state_);
	if (n_rand) delete n_rand;

	OMPL_INFORM("Created %u states", this->node_tree_.size());

	return base::PlannerStatus(solved, approximate);

}

void RandomTreePlanner::clear(void)
{
	Planner::clear();
	// clear the data structures here
	this->sampler_.reset();
	this->freeMemory();
	this->lastGoalMotion_ = NULL;

}

void RandomTreePlanner::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
}

void RandomTreePlanner::getPlannerData(base::PlannerData &data) const
 {
	    Planner::getPlannerData(data);

	    if (lastGoalMotion_!=NULL)
	        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->node_state_));

	    for (unsigned int i = 0 ; i < this->node_tree_.size() ; ++i)
	    {
	        if (node_tree_.getNode(i)->parent_node_ == NULL)
	            data.addStartVertex(base::PlannerDataVertex(node_tree_.getNode(i)->node_state_));
	        else
	            data.addEdge(base::PlannerDataVertex(node_tree_.getNode(i)->parent_node_->node_state_),
	                         base::PlannerDataVertex(node_tree_.getNode(i)->node_state_));
	    }
 }

void RandomTreePlanner::setBias(double bias)
{
	this->goal_bias_ = bias;
}

double RandomTreePlanner::getBias() const
{
	return this->goal_bias_;
}

void RandomTreePlanner::freeMemory()
{
	this->node_tree_.flushTree();
}

double RandomTreePlanner::getRange() const
{
	return this->maxDistance_;
}

void RandomTreePlanner::setRange(double distance)
{
	this->maxDistance_ = distance;
}

