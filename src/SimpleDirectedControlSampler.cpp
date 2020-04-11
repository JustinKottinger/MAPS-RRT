/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Willow Garage, Inc.
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
/* Extended by: Justin Kottinger */

// my includes
#include "../includes/SimpleDirectedControlSamplerMAPS.h"
// #include "ompl/control/SimpleDirectedControlSampler.h"
// ompl includes
#include "ompl/control/SpaceInformation.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "../includes/MyPlanner.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;


ompl::control::SimpleDirectedControlSamplerMAPS::SimpleDirectedControlSamplerMAPS(const SpaceInformation *si, base::Goal *Goal, unsigned int k)
  : si_(si), cs_(si->allocControlSampler()), numControlSamples_(k), goal(Goal), siC_(si_->getControlSpace()) //DirectedControlSampler(si), 
{
}

ompl::control::SimpleDirectedControlSamplerMAPS::~SimpleDirectedControlSamplerMAPS() = default;

unsigned int ompl::control::SimpleDirectedControlSamplerMAPS::sampleToMAPS(Control *control, const base::State *source,
                                                                   base::State *dest)
{
    return getBestControlMAPS(control, source, dest, nullptr);
}

unsigned int ompl::control::SimpleDirectedControlSamplerMAPS::sampleToMAPS(Control *control, const Control *previous,
                                                                   const base::State *source, base::State *dest)
{
    return getBestControlMAPS(control, source, dest, previous);
}

bool ompl::control::SimpleDirectedControlSamplerMAPS::FoundGoal(
    const ob::RealVectorStateSpace::StateType * S, 
    const ob::RealVectorStateSpace::StateType * G)
{

    // tolorance is currently hard coded, will update with generalized formula
    const double tol = 0.5;

    // if vehicle x-values are within tolorance
    if (abs(S->values[0] - G->values[0]) <= tol)
    {
        // if also the y-values are within tolorance
        if (abs(S->values[1] - G->values[1]) <= tol)
        {
            return true;
        }
    }
    return false;
}


unsigned int ompl::control::SimpleDirectedControlSamplerMAPS::getBestControlMAPS(Control *control, const base::State *source,
                                                                         base::State *dest, const Control *previous)
{
    // get information regarding source node
    const ob::CompoundStateSpace::StateType *src = source->as<ob::CompoundStateSpace::StateType>();

    const auto *SRCxyState1 = src->as<ob::RealVectorStateSpace::StateType>(0);

    const auto *SRCxyState2 = src->as<ob::RealVectorStateSpace::StateType>(2);

    // get information regarding goal node
    auto *goal_state = dynamic_cast<base::GoalState *>(goal);

    const ob::State *gs = goal_state->getState();

    const ob::CompoundStateSpace::StateType *GState = gs->as<ob::CompoundStateSpace::StateType>();

    const auto *GoalxyState1 = GState->as<ob::RealVectorStateSpace::StateType>(0);

    const auto *GoalxyState2 = GState->as<ob::RealVectorStateSpace::StateType>(2);

    // check to see if either vehicle found goal
    bool found1 = FoundGoal(SRCxyState1, GoalxyState1);
    bool found2 = FoundGoal(SRCxyState2, GoalxyState2);

    // if vehicle one found goal, then keep it in place 
    if (found1)
    {
        // propogate as normal
        // Sample the first control
        if (previous != nullptr)
            cs_->sampleNext(control, previous, source);
        else
            cs_->sample(control, source);

        const unsigned int minDuration = si_->getMinControlDuration();
        const unsigned int maxDuration = si_->getMaxControlDuration();

        unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);

        // Propagate the first control, and find how far it is from the target state
        base::State *bestState = si_->allocState();
        steps = si_->propagateWhileValid(source, control, steps, bestState);

        if (numControlSamples_ > 1)
        {
            Control *tempControl = si_->allocControl();
            base::State *tempState = si_->allocState();
            double bestDistance = si_->distance(bestState, dest);
    
            // Sample k-1 more controls, and save the control that gets closest to target
            for (unsigned int i = 1; i < numControlSamples_; ++i)
            {
                unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
                if (previous != nullptr)
                    cs_->sampleNext(tempControl, previous, source);
                else
                    cs_->sample(tempControl, source);

                sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
                double tempDistance = si_->distance(tempState, dest);
                if (tempDistance < bestDistance)
                {
                    si_->copyState(bestState, tempState);
                    si_->copyControl(control, tempControl);
                    bestDistance = tempDistance;
                    steps = sampleSteps;
                }
            }

            si_->freeState(tempState);
            si_->freeControl(tempControl);
        }

        si_->copyState(dest, bestState);
        si_->freeState(bestState);

        // override the destination and controls for vehicle 1
        double *cntrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        // overriding controls
        cntrl[0] = 0.0;
        cntrl[1] = 0.0;

        ompl::base::CompoundStateSpace::StateType *destination = 
        	dest->as<ompl::base::CompoundStateSpace::StateType>();

        ob::RealVectorStateSpace::StateType *xyDest1 = 
        	destination->as<ob::RealVectorStateSpace::StateType>(0);

        // overriding state
        xyDest1->values[0] = SRCxyState1->values[0];
        xyDest1->values[1] = SRCxyState1->values[1];

        ob::SO2StateSpace::StateType *rot1 = destination->as<ob::SO2StateSpace::StateType>(1);

        rot1->value = src->as<ob::SO2StateSpace::StateType>(1)->value;

        return steps;

    }
    // if only vehilce two found goal, then keep it in place
    else if (found2)
    {
        // propogate as normal
        // Sample the first control
        if (previous != nullptr)
            cs_->sampleNext(control, previous, source);
        else
            cs_->sample(control, source);

        const unsigned int minDuration = si_->getMinControlDuration();
        const unsigned int maxDuration = si_->getMaxControlDuration();

        unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);

        // Propagate the first control, and find how far it is from the target state
        base::State *bestState = si_->allocState();
        steps = si_->propagateWhileValid(source, control, steps, bestState);

        if (numControlSamples_ > 1)
        {
            Control *tempControl = si_->allocControl();
            base::State *tempState = si_->allocState();
            double bestDistance = si_->distance(bestState, dest);
    
            // Sample k-1 more controls, and save the control that gets closest to target
            for (unsigned int i = 1; i < numControlSamples_; ++i)
            {
                unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
                if (previous != nullptr)
                    cs_->sampleNext(tempControl, previous, source);
                else
                    cs_->sample(tempControl, source);

                sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
                double tempDistance = si_->distance(tempState, dest);
                if (tempDistance < bestDistance)
                {
                    si_->copyState(bestState, tempState);
                    si_->copyControl(control, tempControl);
                    bestDistance = tempDistance;
                    steps = sampleSteps;
                }
            }

            si_->freeState(tempState);
            si_->freeControl(tempControl);
        }

        si_->copyState(dest, bestState);
        si_->freeState(bestState);

        // override the destination and controls for vehicle 2
        double *cntrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        // overriding the controls
        cntrl[2] = 0.0;
        cntrl[3] = 0.0;

        ompl::base::CompoundStateSpace::StateType *destination = 
        	dest->as<ompl::base::CompoundStateSpace::StateType>();

        ob::RealVectorStateSpace::StateType *xyDest2 = 
        	destination->as<ob::RealVectorStateSpace::StateType>(2);

        // overriding the state
        xyDest2->values[0] = SRCxyState2->values[0];
        xyDest2->values[1] = SRCxyState2->values[1];

        ob::SO2StateSpace::StateType *rot2 = destination->as<ob::SO2StateSpace::StateType>(3);

        rot2->value = src->as<ob::SO2StateSpace::StateType>(3)->value;

        return steps;

    }
    else
    {
    	// propogate as normal
        // Sample the first control
        if (previous != nullptr)
            cs_->sampleNext(control, previous, source);
        else
            cs_->sample(control, source);

        const unsigned int minDuration = si_->getMinControlDuration();
        const unsigned int maxDuration = si_->getMaxControlDuration();

        unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);

        // Propagate the first control, and find how far it is from the target state
        base::State *bestState = si_->allocState();
        steps = si_->propagateWhileValid(source, control, steps, bestState);

        if (numControlSamples_ > 1)
        {
            Control *tempControl = si_->allocControl();
            base::State *tempState = si_->allocState();
            double bestDistance = si_->distance(bestState, dest);
    
            // Sample k-1 more controls, and save the control that gets closest to target
            for (unsigned int i = 1; i < numControlSamples_; ++i)
            {
                unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
                if (previous != nullptr)
                    cs_->sampleNext(tempControl, previous, source);
                else
                    cs_->sample(tempControl, source);

                sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
                double tempDistance = si_->distance(tempState, dest);
                if (tempDistance < bestDistance)
                {
                    si_->copyState(bestState, tempState);
                    si_->copyControl(control, tempControl);
                    bestDistance = tempDistance;
                    steps = sampleSteps;
                }
            }

            si_->freeState(tempState);
            si_->freeControl(tempControl);
        }
        
        si_->copyState(dest, bestState);
        si_->freeState(bestState);

        return steps;
    }
}
