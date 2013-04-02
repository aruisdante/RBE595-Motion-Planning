/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#ifndef RG_RRT_H_
#define RG_RRT_H_

#include <ompl/control/planners/PlannerIncludes.h>
#include <ompl/datastructures/NearestNeighbors.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace ompl;
using namespace ompl::control;
namespace homework3
{

        /**
           @anchor cRRT
           @par Short description
           RRT is a tree-based motion planner that uses the following
           idea: RRT samples a random state @b qr in the state space,
           then finds the state @b qc among the previously seen states
           that is closest to @b qr and expands from @b qc towards @b
           qr, until a state @b qm is reached. @b qm is then added to
           the exploration tree.
           This implementation is intended for systems with differential constraints.
           @par External documentation
           S.M. LaValle and J.J. Kuffner, Randomized kinodynamic planning, <em>Intl. J. of Robotics Research</em>, vol. 20, pp. 378–400, May 2001. DOI: <a href="http://dx.doi.org/10.1177/02783640122067453">10.1177/02783640122067453</a><br>
           <a href="http://ijr.sagepub.com/content/20/5/378.full.pdf">[PDF]</a>
           <a href="http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html">[more]</a>
        */

        /** \brief Rapidly-exploring Random Tree */
        class RG_RRT : public base::Planner
        {
        public:

            /** \brief Constructor */
            RG_RRT(const SpaceInformationPtr &si);

            virtual ~RG_RRT(void);

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            virtual void clear(void);

            /** In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias(void) const
            {
                return goalBias_;
            }

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself */
            bool getIntermediateStates(void) const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN<Motion*>());
            }

            virtual void setup(void);

        protected:


            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:

                Motion(void) : state(NULL), control(NULL), steps(0), parent(NULL)
                {
                }

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si) : si(si), state(si->allocState()), control(si->allocControl()), steps(0), parent(NULL)
                {
                }

                ~Motion(void)
                {
//                    for(int i=0; i<reachable_states.size(); i++)
//                    {
//                        if(si!=NULL)
//                        {
//                            if(reachable_states.at(i)!=NULL)
//                            {
//                                si->freeState(reachable_states.at(i));
//                            }
//                        }
//                    }
//                    reachable_states.clear();
                }

                void generateReachability()
                {
                    //std::cout<<"Generating Reachability for state...";
                    const base::StateValidityCheckerPtr checker = si->getStateValidityChecker();
                    control::RealVectorControlSpace* ctrls = si->getControlSpace()->as<control::RealVectorControlSpace>();
                    const base::RealVectorBounds bounds = ctrls->getBounds();
                    double min_ctrl = bounds.low.at(0);
                    double max_ctrl = bounds.high.at(0);
                    double slope    = (max_ctrl-min_ctrl)/10.0;
                    //std::cout<<"Found Control Bounds of "<<min_ctrl<<","<<max_ctrl<<std::endl;
                    //Build the reachability set
                    for(int i=0; i<10; i++)
                    {
                        //std::cout<<"Building a new reachability state "<<std::endl;
                        reachable_states.push_back(si->allocState());
                        control::Control* c = si->allocControl();
                        c->as<control::RealVectorControlSpace::ControlType>()->values[0] = slope*(double)i+min_ctrl;
                        //std::cout<<"Calculated Control value of "<<c->as<control::RealVectorControlSpace::ControlType>()->values[0]<<std::endl;
                        si->getStatePropagator()->propagate(this->state,control,si->getPropagationStepSize()*10,reachable_states.at(i));
                        //std::cout<<"Propogated Forwards in time... "<<std::endl;
                        si->freeControl(c);
                    }

                    //Make sure all of the values in the reachability set were valid states
                    //std::cout<<"Trimming to only valid states, from size "<<reachable_states.size()<<"... "<<std::endl;
                    for(int i=reachable_states.size()-1; i>=0; i--)
                    {
                        if(!checker->isValid(reachable_states.at(i)))
                        {
                            si->freeState(reachable_states.at(i));
                            reachable_states.pop_back();
                        }
                    }
                    //std::cout<<"Reachable States Trimmed to "<<reachable_states.size()<<std::endl;

                }

                const SpaceInformation* si;

                /** \brief The state contained by the motion */
                base::State       *state;

                std::vector<base::State*> reachable_states;

                /** \brief The control contained by the motion */
                Control           *control;

                /** \brief The number of steps the control is applied for */
                unsigned int       steps;

                /** \brief The parent motion in the exploration tree */
                Motion            *parent;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory(void);

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion* a, const Motion* b) const
            {
                double normal_distance = si_->distance(a->state, b->state);
                double reachable_distance = std::numeric_limits<double>::infinity();
                for(int i=0; i<a->reachable_states.size(); i++)
                {
                    double test_dist = si_->distance(a->reachable_states.at(i), b->state);
                    if(test_dist<reachable_distance && test_dist<normal_distance)
                    {
                        reachable_distance = test_dist;
                    }
                }

                return reachable_distance;
            }

            /** \brief State sampler */
            base::StateSamplerPtr                          sampler_;

            /** \brief Control sampler */
            DirectedControlSamplerPtr                      controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation                        *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            boost::shared_ptr< NearestNeighbors<Motion*> > nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
            double                                         goalBias_;

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool                                           addIntermediateStates_;

            /** \brief The random number generator */
            RNG                                            rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion                                         *lastGoalMotion_;
        };
}

#endif
