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
#include "ompl/geometric/planners/PlannerIncludes.h"
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
//*****************LOCAL DEPENDANCIES**************************//
#include "MotionPlanningTree.h"
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
    	typedef SimpleMotionPlanningTree::SimplePathNode  Node;
    	typedef SimpleMotionPlanningTree::SimplePathNode* NodePtr;
    	/**
    	 * @author Adam Panzica
    	 * @brief Constructs a new RandomTreePlanner
    	 * @param si
    	 */
    	RandomTreePlanner(const base::SpaceInformationPtr& si);
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

        virtual void setRange(double distance);

        /**
         * @author Adam Panzica
         * @brief  Gets the bias of selecting the goal point
         * @return Probabilty, 0-1
         */
        virtual double getBias() const;

        virtual double getRange() const;


    protected:

        double maxDistance_;

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

        SimpleMotionPlanningTree node_tree_;

        NodePtr  lastGoalMotion_;
    };
}


#endif /* RANDOMTREEPLANNER_H_ */
