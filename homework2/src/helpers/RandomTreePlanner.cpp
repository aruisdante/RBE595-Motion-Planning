/**
 * @file RandomTreePlanner.cpp
 *
 * @date   Feb 26, 2013
 * @author parallels
 * @brief \todo
 */

//License File

//****************SYSTEM DEPENDANCIES**************************//
//*****************LOCAL DEPENDANCIES**************************//
#include"RandomTreePlanner.h"
//**********************NAMESPACES*****************************//


using namespace ompl;


RandomTreePlanner::~RandomTreePlanner(void)
 {
     // free any allocated memory
 }
 base::PlannerStatus RandomTreePlanner::solve(const base::PlannerTerminationCondition &ptc)
 {
     // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
     // ensures that there is at least one input state and a ompl::base::Goal object specified
     checkValidity();
     // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
     base::Goal                 *goal = this->pdef_->getGoal().get();
     // get input states with PlannerInputStates helper, pis_
     while (const base::State *st = pis_.nextStart())
     {
         RTPNode* start_node = new RandomTreePlanner::RTPNode(this->si_);
         this->node_tree_.push_back(start_node);
     }

     //If we didn't get any valid start states, we're done
     if(this->node_tree_.size()==0)
     {
    	 OMPL_ERROR("Did not get a valid starting condition!");
    	 return base::PlannerStatus::INVALID_START;
     }

     if (!sampler_) sampler_ = si_->allocStateSampler();

     OMPL_INFORM("Starting with %d initial states!", this->node_tree_.size());

     RTPNode*     n_rand = new RTPNode(this->si_);
     base::State* q_rand = n_rand->node_state_;

     while (ptc() == false)
     {
         //Check to see if we should bias to the goal

     }
     // When a solution path is computed, save it here
     //pdef_->addSolutionPath(...);
     // Return a value from the PlannerStatus enumeration.
     // See ompl::base::PlannerStatus for the possible return values
     return base::PlannerStatus::EXACT_SOLUTION;
 }
 void RandomTreePlanner::clear(void)
 {
     Planner::clear();
     // clear the data structures here
     this->sampler_.reset();
     this->freeMemory();
     this->setBias(RTP_DEFAULT_GOAL_BIAS);
 }
 // optional, if additional setup/configuration is needed, the setup() method can be implemented
 void RandomTreePlanner::setup(void)
 {
     Planner::setup();
     // perhaps attempt some auto-configuration

     //sc.configure...
 }
 void RandomTreePlanner::getPlannerData(base::PlannerData &data) const
 {
     // fill data with the states and edges that were created
     // in the exploration data structure
     // perhaps also fill control::PlannerData
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
	 for(RandomTreePlanner::node_tree_t::iterator node_itr = this->node_tree_.begin(); node_itr<this->node_tree_.end(); node_itr++)
	{
		 if((*node_itr)->node_state_!=NULL) this->si_->freeState((*node_itr)->node_state_);
		 delete *node_itr;
	}

 }

