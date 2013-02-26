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
#include <ompl/base/Planner.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
//*****************LOCAL DEPENDANCIES**************************//

//**********************NAMESPACES*****************************//


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
    	     // the specifications of this planner (ompl::base::PlannerSpecs)
    	    // specs_.approximateSolutions = ...;
    	    // specs_.recognizedGoal = ...;
    	    // ...
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
    };
}


#endif /* RANDOMTREEPLANNER_H_ */
