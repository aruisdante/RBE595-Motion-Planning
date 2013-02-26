/**
 * @file RandomTreePlanner.h
 *
 * @date   Feb 26, 2013
 * @author Adam Panzica
 * @brief  Class definition for the Random Tree Planner
 */

//License File



#ifndef RANDOMTREEPLANNER_H_
#define RANDOMTREEPLANNER_H_

//****************SYSTEM DEPENDANCIES**************************//
#include <vector>
#include <ompl/base/Planner.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
//*****************LOCAL DEPENDANCIES**************************//

//**********************NAMESPACES*****************************//
#define RTP_DEFAULT_GOAL_BIAS 0.5

namespace ompl
{
	/**
	 * @author Adam Panzica
	 * @brief  Class which defines a Randoom Tree based planner for use with ompl
	 *
	 * The Random Tree Planner is a sample based planner which takes a random sample, q_rand, and a random node on the tree, and attempts to connect them.
	 * With some probablitiy \f$\sigma\f$ the planner choses the goal configuration as q_rand.
	 */
    class RandomTreePlanner : public base::Planner
    {
    public:
    	/**
    	 * @author Adam Panzica
    	 * @brief Constructs a new RandomTreePlanner
    	 * @param si
    	 */
    	RandomTreePlanner(const base::SpaceInformationPtr& si): base::Planner(si, "Random Tree Planner")
    	 {
    		//Set a default goal sampling bias of 0.5
    		this->goal_bias_                  = RTP_DEFAULT_GOAL_BIAS;
    	     // the specifications of this planner (ompl::base::PlannerSpecs)
    		this->specs_.directed             = false;
    		this->specs_.recognizedGoal       = base::GOAL_SAMPLEABLE_REGION;
    		this->specs_.optimizingPaths      = false;
    		this->specs_.multithreaded        = false;
    	    this->specs_.approximateSolutions = false;
    		this->specs_.provingSolutionNonExistence = false;

    		//Registers the goal bias as a parameter of the planner
    		Planner::declareParam<double>("goal_bias", this, &RandomTreePlanner::setBias, &RandomTreePlanner::getBias, "0.:.05:1.");
    	 }
        virtual ~RandomTreePlanner(void);
        /**
         * @author Adam Panzica
         * @brief Attempts to solve the planning problem
         * @param ptc
         * @return The status of the path at termination
         */
        virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

        /**
         * @author Adam Panzica
         * @brief  Performs any cleanup activities for the planner
         */
        virtual void clear(void);
        /**
         * @author Adam Panzica
         * @brief  Performs any setup activities for the planner
         */
        virtual void setup(void);
        /**
         * @author Adam Panzica
         * @brief  Gets the last set of planner data
         * @param data Data object to fill
         */
        virtual void getPlannerData(base::PlannerData &data) const;

        /**
         * @author Adam Panzica
         * @brief  Sets the bias of selecting the goal point as q_rand
         * @param bias The probabilty of selecting goal, 0-1
         */
        virtual void setBias(double bias);

        /**
         * @author Adam Panzica
         * @brief  Gets the bias of selecting the goal point
         * @return Probabilty, 0-1
         */
        virtual double getBias() const;

    protected:
        /**
         * The node type that will be stored in the planer's tree. Holds a state and a pointer to a parent
         */
        class RTPNode
        {
        public:
        	/**
        	 * Empty constructor
        	 */
        	RTPNode():node_state_(NULL), parent_node_(NULL){};
        	/**
        	 * @author Adam Panzica
        	 * @brief  Constructs a new RTPNode wth a given space information for pre-allocating a state
        	 * @param si
        	 */
        	RTPNode(const base::SpaceInformationPtr& si): node_state_(si->allocState()), parent_node_(NULL){};
        	virtual ~RTPNode(){};
        	base::State*   node_state_;
        	RTPNode*       parent_node_;
        };
        typedef std::vector<RTPNode*> node_tree_t;

        /**
         * Cleans up the memory use for the tree
         */
        void freeMemory();

        /**
         * The probability of selecting the goal as q_rand
         */
        double goal_bias_;

        /**
         * The state sampler
         */
        base::StateSamplerPtr sampler_;

        /**
         * The random number generator
         */
        RNG ran_gen_;

        node_tree_t node_tree_;
    };
}


#endif /* RANDOMTREEPLANNER_H_ */
