/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Authors: Zakary Littlefield */

// my includes
#include "../includes/MAPS_SST.h"
#include "../includes/ReadWorld.h"
#include "../includes/KinematicCar.h"
#include "../includes/MAPSRRTPathControl.h"
#include <boost/numeric/odeint/integrate/integrate_const.hpp>
#include "../includes/SimpleDirectedControlSamplerMAPS.h"

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <chrono>
#include <bits/stdc++.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <algorithm>
#include <typeinfo>
#include <string>


using namespace boost::numeric::odeint;
namespace bg = boost::geometry;


ompl::control::MAPSSST::MAPSSST(const SpaceInformationPtr &si, 
    int NumVehicles, int NumControls, int DimofEachVehicle,
    int MaxSegments, std::vector<double> goal, double radius, 
    bool benchmark,  std::string model, unsigned int k, 
    std::string solutionName) : base::Planner(si, "SST")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
    prevSolution_.clear();
    prevSolutionControls_.clear();
    prevSolutionSteps_.clear();

    MaxSegments_ = MaxSegments;
    benchmark_ = benchmark;
    model_ = model;
    NumVs = NumVehicles;
    NumCs = NumControls;
    dim = DimofEachVehicle;
    g = goal;
    time_ = 0.0;
    radius_ = radius;
    SolName_ = solutionName;

    Planner::declareParam<double>("goal_bias", this, &MAPSSST::setGoalBias, &MAPSSST::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("selection_radius", this, &MAPSSST::setSelectionRadius, &MAPSSST::getSelectionRadius, "0.:.1:"
                                                                                                                "100");
    Planner::declareParam<double>("pruning_radius", this, &MAPSSST::setPruningRadius, &MAPSSST::getPruningRadius, "0.:.1:100");
}

ompl::control::MAPSSST::~MAPSSST()
{
    freeMemory();
}

void ompl::control::MAPSSST::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
    if (!witnesses_)
        witnesses_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    witnesses_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                    {
                                        return distanceFunction(a, b);
                                    });

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
            if (dynamic_cast<base::MaximizeMinClearanceObjective *>(opt_.get()) ||
                dynamic_cast<base::MinimaxObjective *>(opt_.get()))
                OMPL_WARN("%s: Asymptotic near-optimality has only been proven with Lipschitz continuous cost "
                          "functions w.r.t. state and control. This optimization objective will result in undefined "
                          "behavior",
                          getName().c_str());
        }
        else
        {
            OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            pdef_->setOptimizationObjective(opt_);
        }
    }

    prevSolutionCost_ = opt_->infiniteCost();
}

void ompl::control::MAPSSST::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    if (witnesses_)
        witnesses_->clear();
    if (opt_)
        prevSolutionCost_ = opt_->infiniteCost();
}

void ompl::control::MAPSSST::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state_)
                si_->freeState(motion->state_);
            if (motion->control_)
                siC_->freeControl(motion->control_);
            delete motion;
        }
    }
    if (witnesses_)
    {
        std::vector<Motion *> witnesses;
        witnesses_->list(witnesses);
        for (auto &witness : witnesses)
        {
            delete witness;
        }
    }
    for (auto &i : prevSolution_)
    {
        if (i)
            si_->freeState(i);
    }
    prevSolution_.clear();
    for (auto &prevSolutionControl : prevSolutionControls_)
    {
        if (prevSolutionControl)
            siC_->freeControl(prevSolutionControl);
    }
    prevSolutionControls_.clear();
    prevSolutionSteps_.clear();
}

std::vector<double> ompl::control::MAPSSST::getDistance(const base::State *st)
{
    std::vector<double> distance;   
    if (model_ == "2KinematicCars")
        distance = TwoKinDistance(st);
    else if (model_ == "2Linear")
        distance = TwoLinearDistance(st);
    else if (model_ == "3Linear")
        distance = ThreeLinearDistance(st);
    else if (model_ == "3Unicycle")
        distance = ThreeUnicycleDistance(st);
    else if (model_ == "2Unicycle")
        distance = TwoUnicycleDistance(st);
    else if (model_ == "3KinematicCars")
        distance = ThreeKinDistance(st);
    else
    {
        std::cout << "Current Distance Model Not Implemented." << std::endl;
        exit(1);
    }
    return distance;
}

std::vector<double> ompl::control::MAPSSST::ThreeKinDistance(const ob::State *st)
{
    
    std::vector<double> distance;
    auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
    auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
    auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
    auto xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(4);

    double deltax_v1 = pow((g[0] - xyState1_->values[0]), 2);
    double deltay_v1 = pow((g[1] - xyState1_->values[1]), 2);
    double deltax_v2 = pow((g[4] - xyState2_->values[0]), 2);
    double deltay_v2 = pow((g[5] - xyState2_->values[1]), 2);
    double deltax_v3 = pow((g[8] - xyState3_->values[0]), 2);
    double deltay_v3 = pow((g[9] - xyState3_->values[1]), 2);

    double d1 = sqrt(deltax_v1 + deltay_v1);
    double d2 = sqrt(deltax_v2 + deltay_v2);
    double d3 = sqrt(deltax_v3 + deltay_v3);

    distance.push_back(d1);
    distance.push_back(d2);
    distance.push_back(d3);

    return distance;
}

std::vector<double> ompl::control::MAPSSST::TwoUnicycleDistance(const ob::State *st)
{
    
    std::vector<double> distance;
    auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
    auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
    auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);

    double deltax_v1 = pow((g[0] - xyState1_->values[0]), 2);
    double deltay_v1 = pow((g[1] - xyState1_->values[1]), 2);
    double deltax_v2 = pow((g[4] - xyState2_->values[0]), 2);
    double deltay_v2 = pow((g[5] - xyState2_->values[1]), 2);

    double d1 = sqrt(deltax_v1 + deltay_v1);
    double d2 = sqrt(deltax_v2 + deltay_v2);

    distance.push_back(d1);
    distance.push_back(d2);

    return distance;
}


std::vector<double> ompl::control::MAPSSST::ThreeUnicycleDistance(const ob::State *st)
{
    
    std::vector<double> distance;
    auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
    auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
    auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
    auto xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(4);

    double deltax_v1 = pow((g[0] - xyState1_->values[0]), 2);
    double deltay_v1 = pow((g[1] - xyState1_->values[1]), 2);
    double deltax_v2 = pow((g[4] - xyState2_->values[0]), 2);
    double deltay_v2 = pow((g[5] - xyState2_->values[1]), 2);
    double deltax_v3 = pow((g[8] - xyState3_->values[0]), 2);
    double deltay_v3 = pow((g[9] - xyState3_->values[1]), 2);

    double d1 = sqrt(deltax_v1 + deltay_v1);
    double d2 = sqrt(deltax_v2 + deltay_v2);
    double d3 = sqrt(deltax_v3 + deltay_v3);

    distance.push_back(d1);
    distance.push_back(d2);
    distance.push_back(d3);

    return distance;
}

std::vector<double> ompl::control::MAPSSST::ThreeLinearDistance(const ob::State *st)
{
    std::vector<double> distance;
    auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
    auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
    auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);
    auto xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);

    double deltax_v1 = pow((g[0] - xyState1_->values[0]), 2);
    double deltay_v1 = pow((g[1] - xyState1_->values[1]), 2);
    double deltax_v2 = pow((g[2] - xyState2_->values[0]), 2);
    double deltay_v2 = pow((g[3] - xyState2_->values[1]), 2);
    double deltax_v3 = pow((g[4] - xyState3_->values[0]), 2);
    double deltay_v3 = pow((g[5] - xyState3_->values[1]), 2);

    double d1 = sqrt(deltax_v1 + deltay_v1);
    double d2 = sqrt(deltax_v2 + deltay_v2);
    double d3 = sqrt(deltax_v3 + deltay_v3);

    distance.push_back(d1);
    distance.push_back(d2);
    distance.push_back(d3);

    return distance;
}

std::vector<double> ompl::control::MAPSSST::TwoLinearDistance(const ob::State *st)
{
    std::vector<double> distance;
    auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
    auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
    auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);

    double deltax_v1 = pow((g[0] - xyState1_->values[0]), 2);
    double deltay_v1 = pow((g[1] - xyState1_->values[1]), 2);
    double deltax_v2 = pow((g[2] - xyState2_->values[0]), 2);
    double deltay_v2 = pow((g[3] - xyState2_->values[1]), 2);

    double d1 = sqrt(deltax_v1 + deltay_v1);
    double d2 = sqrt(deltax_v2 + deltay_v2);

    distance.push_back(d1);
    distance.push_back(d2);

    return distance;
}

std::vector<double> ompl::control::MAPSSST::TwoKinDistance(const ob::State *st)
{
    std::vector<double> distances;

    // extract the vector spaces of the vehicles
    auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
    auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
    auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);

    // calculate the squared distance of each element for each vehicle
    double deltax_v1 = pow((g[0] - xyState1_->values[0]), 2);
    double deltay_v1 = pow((g[1] - xyState1_->values[1]), 2);
    double deltax_v2 = pow((g[3] - xyState2_->values[0]), 2);
    double deltay_v2 = pow((g[4] - xyState2_->values[1]), 2);

    // calculate true distance
    double d1 = sqrt(deltax_v1 + deltay_v1);
    double d2 = sqrt(deltax_v2 + deltay_v2);

    distances.push_back(d1);
    distances.push_back(d2);
    return distances;
}


int ompl::control::MAPSSST::MultiAgentControlSampler(Motion *motion,Control *RandCtrl, Control *previous, 
    const base::State *source, base::State *dest)
{

    // reset the path distance
    std::vector<double> MotionPathDistance;
    motion->AllVehicleDistance = MotionPathDistance;

    auto cs_ = siC_->allocControlSampler();

    std::vector<int> NoPropNeeded;
    std::numeric_limits<double>::infinity();

    // define an instance of the goal region
    // ob::GoalPtr goal (new MyArbitraryGoal(ss->getSpaceInformation(), gol, Radius));

    std::vector<double> d = getDistance(source);

    for (int i = 0; i < NumVs; i++)
    {
        if (d[i] <= radius_)
        {
            NoPropNeeded.push_back(i);
        }
    }

    // propogate as normal
    // Sample the first control
    if (previous != nullptr)
        cs_->sampleNext(RandCtrl, previous, source);
    else
        cs_->sample(RandCtrl, source);

    const unsigned int minDuration = siC_->getMinControlDuration();
    const unsigned int maxDuration = siC_->getMaxControlDuration();

    unsigned int steps = cs_->sampleStepCount(minDuration, maxDuration);

    // Propagate the first control, and find how far it is from the target state
    base::State *bestState = siC_->allocState();

    steps = propagateWhileValid(motion, source, RandCtrl, steps, bestState, NoPropNeeded);


    if (numControlSamples_ > 1)
    {
        Control *tempControl = siC_->allocControl();
        base::State *tempState = siC_->allocState();
        double bestDistance = siC_->distance(bestState, dest);
    
        // Sample k-1 more controls, and save the control that gets closest to target
        for (unsigned int i = 1; i < numControlSamples_; ++i)
        {
            unsigned int sampleSteps = cs_->sampleStepCount(minDuration, maxDuration);
            if (previous != nullptr)
                cs_->sampleNext(tempControl, previous, source);
            else
                cs_->sample(tempControl, source);

            sampleSteps = propagateWhileValid(motion, source, tempControl, sampleSteps, 
                tempState, NoPropNeeded);
            double tempDistance = siC_->distance(tempState, dest);
            if (tempDistance < bestDistance)
            {
                siC_->copyState(bestState, tempState);
                siC_->copyControl(RandCtrl, tempControl);
                bestDistance = tempDistance;
                steps = sampleSteps;
            }
        }

        siC_->freeState(tempState);
        siC_->freeControl(tempControl);
    }
        
    siC_->copyState(dest, bestState);
    siC_->freeState(bestState);

    return steps;
}

unsigned int ompl::control::MAPSSST::propagateWhileValid(Motion *motion,const base::State *state, 
    Control *control, int steps, base::State *result, const std::vector<int> DoNotProgogate)
{
    // std::cout << motion << std::endl;
    // std::vector<std::vector<points>> path  [[v1point1, v2point1], [v1point2, v2point 2], ...]
    std::vector<std::vector<Point>> path;

    std::vector<double> AllVehiclePathLength;

    if (steps == 0)
    {
        if (result != state)
            siC_->copyState(result, state);
        return 0;
    }

    double stepSize_ = siC_->getPropagationStepSize();

    double signedStepSize = steps > 0 ? stepSize_ : -stepSize_;
    steps = abs(steps);

    auto stateProp = siC_->getStatePropagator();

    path.push_back(MakeLinearPath(state));
    
    stateProp->propagate(state, control, signedStepSize, result);

    overrideStates(DoNotProgogate, state, result, control);

    // // if we found a valid state after one step, we can go on
    if (siC_->isValid(result))
    {
        // since it is valid, we can add the node, hence, we need to store the linear path
        path.push_back(MakeLinearPath(result));
        Get2DimDistance(motion, state, result);

        base::State *temp1 = result;
        base::State *temp2 = siC_->allocState();
        base::State *toDelete = temp2;
        unsigned int r = steps;

        // for the remaining number of steps
        for (int i = 1; i < steps; ++i)
        {
            stateProp->propagate(temp1, control, signedStepSize, temp2);
            overrideStates(DoNotProgogate, temp1, temp2, control);
            if (siC_->isValid(temp2))
            {
                // since it is valid, we can add the node, hence, we need to store the linear path
                path.push_back(MakeLinearPath(temp2));
                Get2DimDistance(motion, temp1, temp2);
                std::swap(temp1, temp2);
            }
            else
            {
                // the last valid state is temp1;
                r = i;
                break;
            }
        }

        // if we finished the for-loop without finding an invalid state, the last valid state is temp1
        // make sure result contains that information
        if (result != temp1)
            si_->copyState(result, temp1);

        // free the temporary memory
        siC_->freeState(toDelete);

        motion->LinearPath = path;
        // motion->AllVehicleDistance = AllVehiclePathLength;

        int sz = motion->AllVehicleDistance.size();
        // std::cout << sz << std::endl;

        return r;
    }
    // if the first propagation step produced an invalid step, return 0 steps
    // the last valid state is the starting one (assumed to be valid)
    motion->LinearPath = path;
    // motion->AllVehicleDistance = AllVehiclePathLength;

    if (result != state)
        si_->copyState(result, state);
    return 0;
}

void ompl::control::MAPSSST::overrideStates(const std::vector<int> DoNotProp, const base::State *source, 
    base::State *result, Control *control)
{
    if (model_ == "2KinematicCars" || model_ == "3KinematicCars")
        OverrideKinCars(DoNotProp, source, result, control);
    else if ((model_ == "2Linear") || (model_ == "3Linear"))
        OverrideLinCars(DoNotProp, source, result, control);
    else if (model_ == "3Unicycle" || model_ == "2Unicycle")
        OverrideUniCars(DoNotProp, source, result, control);
    else
    {
        std::cout << "Current Override States Model Not Implemented." << std::endl;
        exit(1);
    }
}

void ompl::control::MAPSSST::OverrideUniCars(const std::vector<int> DoNotProp, const base::State *source, 
    base::State *result, Control *control)
{
    // this function takes in a state and a list of vehicle indexes that do not need to be propogated
    // it changes the state s.t. state propogation and controls are null for any vehicles in the goal
    if (DoNotProp.size() > 0)
    {
        // this array has a list of vehicles that do not need to be progpogated
        // we need to iterate through all of them

        // get the entire compound state for result
        ompl::base::CompoundStateSpace::StateType *destination = 
            result->as<ompl::base::CompoundStateSpace::StateType>();

        // get the entire compound state space for source
        const ob::CompoundStateSpace::StateType *src = 
            source->as<ob::CompoundStateSpace::StateType>();

        double *cntrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        for (int i = 0; i < DoNotProp.size(); i++)
        {
            // overriding controls 
            // indexing method: ControlDimension(vehicle) + ControlIndex
            cntrl[NumCs*DoNotProp[i] + 0] = 0.0;
            cntrl[NumCs*DoNotProp[i] + 1] = 0.0;

            // next, make the source state the destination state for the indiv. vehicles
            // indexing method: 2(vehicle) or 2(vehicle) + 1

            // get the specific xy state of result 
            ob::RealVectorStateSpace::StateType *xyDest = 
                destination->as<ob::RealVectorStateSpace::StateType>(2*DoNotProp[i]);

            // get the specific xy state of the source
            const auto *SRCxyState = src->as<ob::RealVectorStateSpace::StateType>(2*DoNotProp[i]);

            // get the specific orientation of result
            ob::SO2StateSpace::StateType *rotd = destination->as<ob::SO2StateSpace::StateType>(2*DoNotProp[i] + 1);

            // get the specific orientation of the source
            const auto *rots = src->as<ob::SO2StateSpace::StateType>(2*DoNotProp[i] + 1);

            // overriding the position state
            xyDest->values[0] = SRCxyState->values[0];
            xyDest->values[1] = SRCxyState->values[1];

            // overriding the orientation
            rotd->value = rots->value;

        }
    }
}


void ompl::control::MAPSSST::OverrideLinCars(const std::vector<int> DoNotProp, const base::State *source, 
    base::State *result, Control *control)
{
    // this function takes in a state and a list of vehicle indexes that do not need to be propogated
    // it changes the state s.t. state propogation and controls are null for any vehicles in the goal
    if (DoNotProp.size() > 0)
    {
        // this array has a list of vehicles that do not need to be progpogated
        // we need to iterate through all of them

        // get the entire compound state for result
        ompl::base::CompoundStateSpace::StateType *destination = 
            result->as<ompl::base::CompoundStateSpace::StateType>();

        // get the entire compound state space for source
        const ob::CompoundStateSpace::StateType *src = 
            source->as<ob::CompoundStateSpace::StateType>();

        double *cntrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        for (int i = 0; i < DoNotProp.size(); i++)
        {
            // overriding controls 
            // indexing method: ControlDimension(vehicle) + ControlIndex
            cntrl[NumCs*DoNotProp[i] + 0] = 0.0;
            cntrl[NumCs*DoNotProp[i] + 1] = 0.0;

            // next, make the source state the destination state for the indiv. vehicles
            // indexing method: 2(vehicle) or 2(vehicle) + 1

            // get the specific xy state of result 
            ob::RealVectorStateSpace::StateType *xyDest = 
                destination->as<ob::RealVectorStateSpace::StateType>(DoNotProp[i]);

            // get the specific xy state of the source
            const auto *SRCxyState = src->as<ob::RealVectorStateSpace::StateType>(DoNotProp[i]);

            // overriding the position state
            xyDest->values[0] = SRCxyState->values[0];
            xyDest->values[1] = SRCxyState->values[1];

        }
    }
}

void ompl::control::MAPSSST::OverrideKinCars(const std::vector<int> DoNotProp, const base::State *source, 
    base::State *result, Control *control)
{
    // this function takes in a state and a list of vehicle indexes that do not need to be propogated
    // it changes the state s.t. state propogation and controls are null for any vehicles in the goal
    if (DoNotProp.size() > 0)
    {
        // this array has a list of vehicles that do not need to be progpogated
        // we need to iterate through all of them

        // get the entire compound state for result
        ompl::base::CompoundStateSpace::StateType *destination = 
            result->as<ompl::base::CompoundStateSpace::StateType>();

        // get the entire compound state space for source
        const ob::CompoundStateSpace::StateType *src = 
            source->as<ob::CompoundStateSpace::StateType>();

        double *cntrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        for (int i = 0; i < DoNotProp.size(); i++)
        {
            // overriding controls 
            // indexing method: ControlDimension(vehicle) + ControlIndex
            cntrl[NumCs*DoNotProp[i] + 0] = 0.0;
            cntrl[NumCs*DoNotProp[i] + 1] = 0.0;

            // next, make the source state the destination state for the indiv. vehicles
            // indexing method: 2(vehicle) or 2(vehicle) + 1

            // get the specific xy state of result 
            ob::RealVectorStateSpace::StateType *xyDest = 
                destination->as<ob::RealVectorStateSpace::StateType>(2*DoNotProp[i]);

            // get the specific xy state of the source
            const auto *SRCxyState = src->as<ob::RealVectorStateSpace::StateType>(2*DoNotProp[i]);

            // get the specific orientation of result
            ob::SO2StateSpace::StateType *rotd = destination->as<ob::SO2StateSpace::StateType>(2*DoNotProp[i] + 1);

            // get the specific orientation of the source
            const auto *rots = src->as<ob::SO2StateSpace::StateType>(2*DoNotProp[i] + 1);

            // overriding the position state
            xyDest->values[0] = SRCxyState->values[0];
            xyDest->values[1] = SRCxyState->values[1];

            // overriding the orientation
            rotd->value = rots->value;

        }
    }
}

std::vector<Point> ompl::control::MAPSSST::MakeLinearPath(const base::State *st) const
{
    std::vector<Point> VehiclePoints;
    if (model_ == "2KinematicCars" || model_ == "3KinematicCars")
        VehiclePoints = MakeKinPath(st);
    else if ((model_ == "2Linear") || (model_ == "3Linear"))
        VehiclePoints = MakeLinPath(st);
    else if (model_ == "3Unicycle" || model_ == "2Unicycle")
        VehiclePoints = MakeUniPath(st);
    else
    {
        std::cout << "Current Linear Path Model Not Implemented." << std::endl;
        exit(1);
    }
    return VehiclePoints;
}

// std::vector<points>  [v1child, v2child]
std::vector<Point> ompl::control::MAPSSST::MakeUniPath(const base::State *result) const
{
    //  this function will take in a source state and a result state
    // it will return a line in the 2D plane from these states (projected onto 2D)
    std::vector<Point> VehiclePoints;

    // get the compound state of the result
    const ompl::base::CompoundStateSpace::StateType *destination = 
        result->as<ompl::base::CompoundStateSpace::StateType>();

    for (int i = 0; i < NumVs; i++)
    {
        // get the xy point of the result per vehicle
        const ob::RealVectorStateSpace::StateType *DESTxyState = 
            destination->as<ob::RealVectorStateSpace::StateType>(2 * i);

        auto r = DESTxyState->values;

        // get the point for the vehicle state
        point child(r[0], r[1]);

        // generate a segement of those points per vehicle
        VehiclePoints.push_back(child);

    }

    // this returns a list of lines from the source point to the result point for each vehicle
    return VehiclePoints;
}

// std::vector<points>  [v1child, v2child]
std::vector<Point> ompl::control::MAPSSST::MakeLinPath(const base::State *result) const
{
    //  this function will take in a source state and a result state
    // it will return a line in the 2D plane from these states (projected onto 2D)
    std::vector<Point> VehiclePoints;

    // get the compound state of the result
    const ompl::base::CompoundStateSpace::StateType *destination = 
        result->as<ompl::base::CompoundStateSpace::StateType>();

    for (int i = 0; i < NumVs; i++)
    {
        // get the xy point of the result per vehicle
        const ob::RealVectorStateSpace::StateType *DESTxyState = 
            destination->as<ob::RealVectorStateSpace::StateType>(i);

        auto r = DESTxyState->values;

        // get the point for the vehicle state
        point child(r[0], r[1]);

        // generate a segement of those points per vehicle
        VehiclePoints.push_back(child);

    }

    // this returns a list of lines from the source point to the result point for each vehicle
    return VehiclePoints;
}




// std::vector<points>  [v1child, v2child]
std::vector<Point> ompl::control::MAPSSST::MakeKinPath(const base::State *result) const
{
    //  this function will take in a source state and a result state
    // it will return a line in the 2D plane from these states (projected onto 2D)
    std::vector<Point> VehiclePoints;

    // get the compound state of the result
    const ompl::base::CompoundStateSpace::StateType *destination = 
        result->as<ompl::base::CompoundStateSpace::StateType>();

    for (int i = 0; i < NumVs; i++)
    {
        // get the xy point of the result per vehicle
        const ob::RealVectorStateSpace::StateType *DESTxyState = 
            destination->as<ob::RealVectorStateSpace::StateType>(2*i);

        auto r = DESTxyState->values;

        // get the point for the vehicle state
        point child(r[0], r[1]);

        // generate a segement of those points per vehicle
        VehiclePoints.push_back(child);

    }

    // this returns a list of lines from the source point to the result point for each vehicle
    return VehiclePoints;
}

// different from distance function above in that this function saves distances to the node
void ompl::control::MAPSSST::Get2DimDistance(Motion *motion, const base::State *source, 
    const base::State *result)
{
    if ((model_ == "2KinematicCars") || (model_ == "3Unicycle") || model_ == "3KinematicCars" || (model_ == "2Unicycle"))
        Get2DimDist2KinCars(motion, source, result);
    else if ((model_ == "2Linear") || (model_ == "3Linear"))
        Get2DimDist2LinCars(motion, source, result);
    else
    {
        std::cout << "Current Distance Model Not Implemented." << std::endl;
        exit(1);
    }
}

void ompl::control::MAPSSST::Get2DimDist2LinCars(Motion *motion, const base::State *source, 
    const base::State *result)
{
    // allocate the returnable -- [distance between ]
    std::vector<double> AllVehicleDist;

    // get the entire compound state for result
    const ompl::base::CompoundStateSpace::StateType *destination = 
        result->as<ompl::base::CompoundStateSpace::StateType>();

    // get the entire compound state space for source
    const ob::CompoundStateSpace::StateType *src = 
        source->as<ob::CompoundStateSpace::StateType>();

    for (int i = 0; i < NumVs; i++)
    {
        // get the xy point of the source
        const ob::RealVectorStateSpace::StateType *SRCxyState = 
            src->as<ob::RealVectorStateSpace::StateType>(i);

        // get the xy point of the result per vehicle
        const ob::RealVectorStateSpace::StateType *DESTxyState = 
            destination->as<ob::RealVectorStateSpace::StateType>(i);

        auto s = SRCxyState->values;

        auto r = DESTxyState->values;

        // get the point for the vehicle state
        Point parent(s[0], s[1]);
        Point child(r[0], r[1]);

        double dist = bg::distance(parent, child);
        AllVehicleDist.push_back(dist);
    }

    // at end of loop, AllVehicleDistance contains the linear distance between the source and result
    // next, we add it to the existing distances that are there
    if ((motion->AllVehicleDistance).size() == 0)
    {
        // std::cout << "here" << std::endl;
        motion->AllVehicleDistance = AllVehicleDist;
    }
    else
    {

        for (int v = 0; v < NumVs; v++)
        {
            (motion->AllVehicleDistance)[v] = (motion->AllVehicleDistance)[v] + AllVehicleDist[v];
        }
    }
}




// different from distance function above in that this function saves distances to the node
void ompl::control::MAPSSST::Get2DimDist2KinCars(Motion *motion, const base::State *source, 
    const base::State *result)
{
    // allocate the returnable -- [distance between ]
    std::vector<double> AllVehicleDist;

    // get the entire compound state for result
    const ompl::base::CompoundStateSpace::StateType *destination = 
        result->as<ompl::base::CompoundStateSpace::StateType>();

    // get the entire compound state space for source
    const ob::CompoundStateSpace::StateType *src = 
        source->as<ob::CompoundStateSpace::StateType>();

    for (int i = 0; i < NumVs; i++)
    {
        // get the xy point of the source
        const ob::RealVectorStateSpace::StateType *SRCxyState = 
            src->as<ob::RealVectorStateSpace::StateType>(2*i);

        // get the xy point of the result per vehicle
        const ob::RealVectorStateSpace::StateType *DESTxyState = 
            destination->as<ob::RealVectorStateSpace::StateType>(2*i);

        auto s = SRCxyState->values;

        auto r = DESTxyState->values;

        // get the point for the vehicle state
        Point parent(s[0], s[1]);
        Point child(r[0], r[1]);

        double dist = bg::distance(parent, child);
        AllVehicleDist.push_back(dist);
    }

    // at end of loop, AllVehicleDistance contains the linear distance between the source and result
    // next, we add it to the existing distances that are there
    if ((motion->AllVehicleDistance).size() == 0)
    {
        // std::cout << "here" << std::endl;
        motion->AllVehicleDistance = AllVehicleDist;
    }
    else
    {

        for (int v = 0; v < NumVs; v++)
        {
            (motion->AllVehicleDistance)[v] = (motion->AllVehicleDistance)[v] + AllVehicleDist[v];
        }
    }
    // std::cout << "Dist: " << (motion->AllVehicleDistance)[0] << " " << 
    //     (motion->AllVehicleDistance)[1] << std::endl;
}

void ompl::control::MAPSSST::FindTotalIntersections(Motion *NewMotion)
{
    // bool intersect
    std::clock_t start, end; 
    start = std::clock(); 
    int NumIntersect = Project2D(NewMotion);

    // if (NewMotion->LocationsOfIntersect.size() != 0)
    //     std::cout << NewMotion->LocationsOfIntersect.size();

    int ParentIntersections = NewMotion->parent_->NumIntersections;

    if (NumIntersect > 0)  // only checks parent -> child 
    {
        NewMotion->NumIntersections = ParentIntersections + NumIntersect;
        NewMotion->cost = NewMotion->parent_->cost + 1;
    }
    else
    {
        NewMotion->cost = NewMotion->parent_->cost;
        NewMotion->NumIntersections = ParentIntersections;
    }

    end = std::clock();
    double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    time_ = time_ + time_taken;
}

int ompl::control::MAPSSST::Project2D(Motion *NewMotion)
{
    int intersects;
    if (NumVs == 2)
        intersects = Project2D_2Vehicles(NewMotion);
    else if (NumVs == 3)
        intersects = Project2D_3Vehicles(NewMotion);
    else
    {
        std::cout << "Current Number of Vehicles Not Supported." << std::endl;
        exit(1);
    }
    return intersects;
}

int ompl::control::MAPSSST::Project2D_3Vehicles(Motion *NewMotion)
{
    int NumIntersect = 0;

    std::vector<Motion *> LocOfInt;

    // get linear path of the current motion for each vehicle
    std::vector<std::vector<Point>> NewPath = NewMotion->LinearPath;

    // get the total distance traveled in the new motion [distv1, distv2, ...]
    std::vector<double> NewMotionDist = NewMotion->AllVehicleDistance;

    // all vehicle new motion source [v1src, v2src, ...]
    std::vector<Point> AllVehicleSourcePosNew = NewPath.back();

    // create segments for all vehicles[[list of segments for v1path], [list of segments for v2path]]
    std::vector<std::vector<Segment>> AllVehicleNewMotionPath;

    for (int v = 0; v < NumVs; v++)
    {
        std::vector<Segment> VehiclePath;
        for (int j = 0; j < NewPath.size() - 1; j++)
        {
            Point parent = (NewPath[j][v]);

            Point child = (NewPath[j + 1][v]);

            Segment VSeg(parent, child);

            VehiclePath.push_back(VSeg);
        }
        AllVehicleNewMotionPath.push_back(VehiclePath);
    }


    Motion *CurrMotion = NewMotion;
    if (CurrMotion->parent_)
    {
        int CurrCost = CurrMotion->parent_->cost;
        
        while (CurrMotion->parent_ && CurrMotion->parent_->cost == CurrCost)
            {
                // current motion path
                std::vector<std::vector<Point>> CurrPath = CurrMotion->LinearPath;
    
                // current motion source point [v1src, v2src, ...]
                std::vector<Point> AllVehicleSourcePosCurr = CurrPath.back();
    
                // current motion distance traveled [distv1, distv2, ...]
                std::vector<double> CurrMotionDist = CurrMotion->AllVehicleDistance;
    
                // create segments for CurrMotion
                std::vector<std::vector<Segment>> AllVehicleCurrMotionPath;
                for (int v = 0; v < NumVs; v++)
                {
                    std::vector<Segment> VehiclePath;
                    for (int j = 0; j < CurrPath.size() - 1; j++)
                    {
                        Point parent = (CurrPath[j][v]);
    
                        Point child = (CurrPath[j + 1][v]);
    
                        Segment VSeg(parent, child);
    
                        VehiclePath.push_back(VSeg);
                    }
                    AllVehicleCurrMotionPath.push_back(VehiclePath);
                }
    
                for (int v = 0; v < NumVs; v++)
                {
                    // check if currmotion is close to new motion (opposing vehicles only)
                    // if on v1, we check against v2 and v3
                    if (v == 0)
                    {
                        // indexing is 0, 1, 2
                        // first, get distance between 0, 1 and check them
                        double SrcDistance = bg::distance(AllVehicleSourcePosNew[0], 
                            AllVehicleSourcePosCurr[1]);
    
                        // next, get total distance traveled between 0, 1
                        double TotalDistanceTraveled = NewMotionDist[0] + CurrMotionDist[1];
    
                        // check to see if it is possible to intersect
                        if (SrcDistance < TotalDistanceTraveled)
                        {
                            // std::cout << "here" << std::endl;
                            // in this case, we need to check for intersection
    
                              // now, we have some segments that need to be checked for intersection
                              // this is done via a double for loop for 0, 1
                              for (int v1 = 0; v1 < AllVehicleNewMotionPath[0].size(); v1++)
                              {
                                  for (int v2 = 0; v2 < AllVehicleCurrMotionPath[1].size(); v2++)
                                  {
        
                                    bool intersection = boost::geometry::intersects(AllVehicleNewMotionPath[0][v1], 
                                        AllVehicleCurrMotionPath[1][v2]);
    
                                    if (intersection)
                                    {
                                        NumIntersect += 1;
                                        LocOfInt.push_back(CurrMotion);
                                        // std::cout << "returning true" << std::endl;
                                        // return true;
                                    }
                                }
                            }
                        }
                        // next, get distance between 0, 2 and check them
                        SrcDistance = bg::distance(AllVehicleSourcePosNew[0], 
                            AllVehicleSourcePosCurr[2]);
    
                        // next, get total distance traveled between 0, 1
                        TotalDistanceTraveled = NewMotionDist[0] + CurrMotionDist[2];
    
                        // check to see if it is possible to intersect
                        if (SrcDistance < TotalDistanceTraveled)
                        {
                            // std::cout << "here" << std::endl;
                            // in this case, we need to check for intersection
    
                              // now, we have some segments that need to be checked for intersection
                              // this is done via a double for loop for 0, 1
                              for (int v1 = 0; v1 < AllVehicleNewMotionPath[0].size(); v1++)
                              {
                                  for (int v2 = 0; v2 < AllVehicleCurrMotionPath[2].size(); v2++)
                                  {
        
                                    bool intersection = boost::geometry::intersects(AllVehicleNewMotionPath[0][v1], 
                                        AllVehicleCurrMotionPath[2][v2]);
    
                                if (intersection)
                                {
                                    NumIntersect += 1;
                                    LocOfInt.push_back(CurrMotion);
                                    // std::cout << "returning true" << std::endl;
                                    // return true;
                                }
                            }
                        }
                    }
                }
                // when checking vehicle two, we check currmotion vehicle 1
                    if (v == 1)
                    {
                        // this needs to check NewMotion V2 vs CurrMotion V1
    
                        // first, get distance between 1, 0
                        double SrcDistance = bg::distance(AllVehicleSourcePosNew[1], 
                            AllVehicleSourcePosCurr[0]);
    
                        // next, get total distance traveled between the two vehicles
                        double TotalDistanceTraveled = NewMotionDist[1] + CurrMotionDist[0];
    
                        // check to see if it is possible to intersect
                        if (SrcDistance < TotalDistanceTraveled)
                        {
                            // std::cout << "here" << std::endl;
                            // in this case, we need to check for intersection
                            // now, we have some segments that need to be checked for intersection
                            // this is done via a double for loop
                            for (int v2 = 0; v2 < AllVehicleNewMotionPath[1].size(); v2++)
                            {
                                for (int v1 = 0; v1 < AllVehicleCurrMotionPath[0].size(); v1++)
                                {
                                    bool intersection = boost::geometry::intersects(AllVehicleNewMotionPath[1][v2], 
                                        AllVehicleCurrMotionPath[0][v1]);
    
                                if (intersection)
                                {
                                    NumIntersect += 1;
                                    LocOfInt.push_back(CurrMotion);
                                    // std::cout << "returning true" << std::endl;
                                    // return true;
                                }
                            }
                        }
                        }
                        // next, get distance between 1, 2
                        SrcDistance = bg::distance(AllVehicleSourcePosNew[1], 
                            AllVehicleSourcePosCurr[2]);
    
                        // next, get total distance traveled between the two vehicles
                        TotalDistanceTraveled = NewMotionDist[1] + CurrMotionDist[2];
    
                        // check to see if it is possible to intersect
                        if (SrcDistance < TotalDistanceTraveled)
                        {
                            // std::cout << "here" << std::endl;
                            // in this case, we need to check for intersection
                            // now, we have some segments that need to be checked for intersection
                            // this is done via a double for loop
                            for (int v2 = 0; v2 < AllVehicleNewMotionPath[1].size(); v2++)
                            {
                                for (int v1 = 0; v1 < AllVehicleCurrMotionPath[2].size(); v1++)
                                {
                                    bool intersection = boost::geometry::intersects(AllVehicleNewMotionPath[1][v2], 
                                        AllVehicleCurrMotionPath[2][v1]);
    
                                    if (intersection)
                                    {
                                        NumIntersect += 1;
                                        LocOfInt.push_back(CurrMotion);
                                        // std::cout << "returning true" << std::endl;
                                        // return true;
                                    }
                                }
                            }
                        }
                    }
                    //  NEED IF V==2 HERE
                    if (v == 2)
                    {
                        // this needs to check NewMotion V2 vs CurrMotion V1
    
                        // first, get distance between 2, 0
                        double SrcDistance = bg::distance(AllVehicleSourcePosNew[2], 
                            AllVehicleSourcePosCurr[0]);
    
                        // next, get total distance traveled between the two vehicles
                        double TotalDistanceTraveled = NewMotionDist[2] + CurrMotionDist[0];
    
                        // check to see if it is possible to intersect
                        if (SrcDistance < TotalDistanceTraveled)
                        {
                            // std::cout << "here" << std::endl;
                            // in this case, we need to check for intersection
                            // now, we have some segments that need to be checked for intersection
                            // this is done via a double for loop
                            for (int v2 = 0; v2 < AllVehicleNewMotionPath[2].size(); v2++)
                            {
                                for (int v1 = 0; v1 < AllVehicleCurrMotionPath[0].size(); v1++)
                                {
                                    bool intersection = boost::geometry::intersects(AllVehicleNewMotionPath[2][v2], 
                                        AllVehicleCurrMotionPath[0][v1]);
    
                                    if (intersection)
                                    {
                                        NumIntersect += 1;
                                        LocOfInt.push_back(CurrMotion);
                                        // std::cout << "returning true" << std::endl;
                                        // return true;
                                    }
                                }
                            }
                        }
                        // next, get distance between 2, 1
                        SrcDistance = bg::distance(AllVehicleSourcePosNew[2], 
                            AllVehicleSourcePosCurr[1]);
    
                        // next, get total distance traveled between the two vehicles
                        TotalDistanceTraveled = NewMotionDist[2] + CurrMotionDist[1];
    
                        // check to see if it is possible to intersect
                        if (SrcDistance < TotalDistanceTraveled)
                        {
                            // std::cout << "here" << std::endl;
                            // in this case, we need to check for intersection
                            // now, we have some segments that need to be checked for intersection
                            // this is done via a double for loop
                            for (int v2 = 0; v2 < AllVehicleNewMotionPath[2].size(); v2++)
                            {
                                for (int v1 = 0; v1 < AllVehicleCurrMotionPath[1].size(); v1++)
                                {
                                    bool intersection = boost::geometry::intersects(AllVehicleNewMotionPath[2][v2], 
                                        AllVehicleCurrMotionPath[1][v1]);
    
                                if (intersection)
                                {
                                    NumIntersect += 1;
                                    LocOfInt.push_back(CurrMotion);
                                    // std::cout << "returning true" << std::endl;
                                    // return true;
                                }
                            }
                        }
                    }
                }
            }
            CurrMotion = CurrMotion->parent_;
        }
    } 
    NewMotion->LocationsOfIntersect = LocOfInt;
    return NumIntersect;
}


int ompl::control::MAPSSST::Project2D_2Vehicles(Motion *NewMotion)
{
    // std::cout << "here";
    int NumIntersect = 0;

    std::vector<Motion *> LocOfInt;

    // get linear path of the current motion for each vehicle
    std::vector<std::vector<Point>> NewPath = NewMotion->LinearPath;

    // get the total distance traveled in the new motion [distv1, distv2, ...]
    std::vector<double> NewMotionDist = NewMotion->AllVehicleDistance;

    // all vehicle new motion source [v1src, v2src, ...]
    std::vector<Point> AllVehicleSourcePosNew = NewPath.back();

    // create segments for all vehicles[[list of segments for v1path], [list of segments for v2path]]
    std::vector<std::vector<Segment>> AllVehicleNewMotionPath;

    for (int v = 0; v < NumVs; v++)
    {
        std::vector<Segment> VehiclePath;
        for (int j = 0; j < NewPath.size() - 1; j++)
        {
            Point parent = (NewPath[j][v]);

            Point child = (NewPath[j + 1][v]);

            Segment VSeg(parent, child);

            VehiclePath.push_back(VSeg);
        }
        AllVehicleNewMotionPath.push_back(VehiclePath);
    }

    Motion *CurrMotion = NewMotion;
    if (CurrMotion->parent_)
    {
        int CurrCost = CurrMotion->parent_->cost;

        while (CurrMotion->parent_ && CurrMotion->parent_->cost == CurrCost)
        {
            // current motion path
            std::vector<std::vector<Point>> CurrPath = CurrMotion->LinearPath;
    
            // current motion source point [v1src, v2src, ...]
            std::vector<Point> AllVehicleSourcePosCurr = CurrPath.back();
    
            // current motion distance traveled [distv1, distv2, ...]
            std::vector<double> CurrMotionDist = CurrMotion->AllVehicleDistance;
    
            // create segments for CurrMotion
            std::vector<std::vector<Segment>> AllVehicleCurrMotionPath;
            for (int v = 0; v < NumVs; v++)
            {
                std::vector<Segment> VehiclePath;
                for (int j = 0; j < CurrPath.size() - 1; j++)
                {
                    Point parent = (CurrPath[j][v]);
    
                    Point child = (CurrPath[j + 1][v]);
    
                    Segment VSeg(parent, child);
    
                    VehiclePath.push_back(VSeg);
                }
                AllVehicleCurrMotionPath.push_back(VehiclePath);
            }
            for (int v = 0; v < NumVs; v++)
            {
                // check if currmotion is close to new motion (opposing vehicles only)
                if (v == 0)
                {
                    // this needs to check NewMotion V1 vs CurrMotion V2
    
                    // first, get distance between sources
                    double SrcDistance = bg::distance(AllVehicleSourcePosNew[0], 
                        AllVehicleSourcePosCurr[1]);
    
                    // next, get total distance traveled between the two vehicles
                    double TotalDistanceTraveled = NewMotionDist[0] + CurrMotionDist[1];
    
                    // check to see if it is possible to intersect
                    if (SrcDistance < TotalDistanceTraveled)
                    {
                        // std::cout << "here" << std::endl;
                        // in this case, we need to check for intersection
    
                        // now, we have some segments that need to be checked for intersection
                        // this is done via a double for loop
                        for (int v1 = 0; v1 < AllVehicleNewMotionPath[0].size(); v1++)
                        {
                            for (int v2 = 0; v2 < AllVehicleCurrMotionPath[1].size(); v2++)
                            {
                                bool intersection = boost::geometry::intersects(AllVehicleNewMotionPath[0][v1], 
                                    AllVehicleCurrMotionPath[1][v2]);
    
                                if (intersection)
                                {
                                    NumIntersect += 1;
                                    LocOfInt.push_back(CurrMotion);
                                    // std::cout << "returning true" << std::endl;
                                    // return true;
                                }
                            }
                        }
                    }
                }
                // when checking vehicle two, we check currmotion vehicle 1
                if (v == 1)
                {
                    // this needs to check NewMotion V2 vs CurrMotion V1
    
                    // first, get distance between sources
                    double SrcDistance = bg::distance(AllVehicleSourcePosNew[1], 
                        AllVehicleSourcePosCurr[0]);
    
                    // next, get total distance traveled between the two vehicles
                    double TotalDistanceTraveled = NewMotionDist[1] + CurrMotionDist[0];
    
                    // check to see if it is possible to intersect
                    if (SrcDistance < TotalDistanceTraveled)
                    {
                        // std::cout << "here" << std::endl;
                        // in this case, we need to check for intersection
                        // now, we have some segments that need to be checked for intersection
                        // this is done via a double for loop
                        for (int v2 = 0; v2 < AllVehicleNewMotionPath[1].size(); v2++)
                        {
                            for (int v1 = 0; v1 < AllVehicleCurrMotionPath[0].size(); v1++)
                            {
                                bool intersection = boost::geometry::intersects(AllVehicleNewMotionPath[1][v2], 
                                    AllVehicleCurrMotionPath[0][v1]);
    
                                if (intersection)
                                {
                                    NumIntersect += 1;
                                    LocOfInt.push_back(CurrMotion);
                                    // std::cout << "returning true" << std::endl;
                                    // return true;
                                }
                            }
                        }
                    }
                }
            }
            CurrMotion = CurrMotion->parent_;
        }
    }
    NewMotion->LocationsOfIntersect = LocOfInt;
    return NumIntersect;
}

ompl::control::MAPSSST::Motion *ompl::control::MAPSSST::selectNode(ompl::control::MAPSSST::Motion *sample)
{
    std::vector<Motion *> ret;
    Motion *selected = nullptr;
    base::Cost bestCost = opt_->infiniteCost();
    nn_->nearestR(sample, selectionRadius_, ret);
    for (auto &i : ret)
    {
        if (!i->inactive_ && opt_->isCostBetterThan(i->accCost_, bestCost))
        {
            bestCost = i->accCost_;
            selected = i;
        }
    }
    if (selected == nullptr)
    {
        int k = 1;
        while (selected == nullptr)
        {
            nn_->nearestK(sample, k, ret);
            for (unsigned int i = 0; i < ret.size() && selected == nullptr; i++)
                if (!ret[i]->inactive_)
                    selected = ret[i];
            k += 5;
        }
    }
    return selected;
}

ompl::control::MAPSSST::Witness *ompl::control::MAPSSST::findClosestWitness(ompl::control::MAPSSST::Motion *node)
{
    if (witnesses_->size() > 0)
    {
        auto *closest = static_cast<Witness *>(witnesses_->nearest(node));
        if (distanceFunction(closest, node) > pruningRadius_)
        {
            closest = new Witness(siC_);
            closest->linkRep(node);
            si_->copyState(closest->state_, node->state_);
            witnesses_->add(closest);
        }
        return closest;
    }
    else
    {
        auto *closest = new Witness(siC_);
        closest->linkRep(node);
        si_->copyState(closest->state_, node->state_);
        witnesses_->add(closest);
        return closest;
    }
}

ompl::base::PlannerStatus ompl::control::MAPSSST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state_, st);
        siC_->nullControl(motion->control_);
        nn_->add(motion);
        motion->accCost_ = opt_->identityCost();
        findClosestWitness(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure\n", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state_;
    Control *rctrl = rmotion->control_;
    base::State *xstate = si_->allocState();

    unsigned iterations = 0;

    bool solved = false;
    bool approximate = false;

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = selectNode(rmotion);

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        controlSampler_->sample(rctrl);
        unsigned int cd = rng_.uniformInt(siC_->getMinControlDuration(), siC_->getMaxControlDuration());
        unsigned int propCd = siC_->propagateWhileValid(nmotion->state_, rctrl, cd, rstate);
        unsigned int propCd_test = MultiAgentControlSampler(rmotion, rctrl, nmotion->control_, nmotion->state_, rmotion->state_);

        if (propCd_test == cd)
        {
            base::Cost incCost = opt_->motionCost(nmotion->state_, rstate);
            base::Cost cost = opt_->combineCosts(nmotion->accCost_, incCost);
            Witness *closestWitness = findClosestWitness(rmotion);

            if (closestWitness->rep_ == rmotion || opt_->isCostBetterThan(cost, closestWitness->rep_->accCost_))
            {
                Motion *oldRep = closestWitness->rep_;
                /* create a motion */
                auto *motion = new Motion(siC_);
                motion->accCost_ = cost;
                si_->copyState(motion->state_, rmotion->state_);
                siC_->copyControl(motion->control_, rctrl);
                motion->steps_ = cd;
                motion->parent_ = nmotion;
                nmotion->numChildren_++;
                motion->LinearPath = rmotion->LinearPath;
                motion->AllVehicleDistance = rmotion->AllVehicleDistance;
                closestWitness->linkRep(motion);

                FindTotalIntersections(motion);

                if (motion->cost <= MaxSegments_)
                {
                    nn_->add(motion);
                    double dist = 0.0;
                    bool solv = goal->isSatisfied(motion->state_, &dist);
                
                    if (solv && opt_->isCostBetterThan(motion->accCost_, prevSolutionCost_))
                    {
                        approxdif = dist;
                        solution = motion;

                        for (auto &i : prevSolution_)
                            if (i)
                                si_->freeState(i);
                        prevSolution_.clear();
                        for (auto &prevSolutionControl : prevSolutionControls_)
                            if (prevSolutionControl)
                                siC_->freeControl(prevSolutionControl);
                        prevSolutionControls_.clear();
                        prevSolutionSteps_.clear();

                        Motion *solTrav = solution;
                        while (solTrav->parent_ != nullptr)
                        {
                            prevSolution_.push_back(si_->cloneState(solTrav->state_));
                            prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_));
                            prevSolutionSteps_.push_back(solTrav->steps_);
                            solTrav = solTrav->parent_;
                        }
                        prevSolution_.push_back(si_->cloneState(solTrav->state_));
                        prevSolutionCost_ = solution->accCost_;

                        // write the path to file
                        lastGoalMotion_ = solution;
                        Motion *lm = solution;

                        std::vector<Motion *> mpath;
                        while (lastGoalMotion_ != nullptr)
                        {
                            // std::cout << solution->GetCost() << std::endl;
                            mpath.push_back(lastGoalMotion_);
                            lastGoalMotion_ = lastGoalMotion_->parent_;
                        }
                        /* set the solution path */
                        // auto path(std::make_shared<PathControl>(si_));
                        auto path(std::make_shared<MAPSRRTPathControl>(si_));

                        FinalCost_ = mpath[0]->GetCost();

                        SolName_ = "txt/SST/path" + std::to_string(solution->accCost_.value()) + ".txt";

                        for (int i = mpath.size() - 1; i >= 0; --i)
                        {
                            if (mpath[i]->parent_)
                            {
                                //  additional method added by Justin Kottinger
                                path->append(mpath[i]->state_, mpath[i]->control_, mpath[i]->steps_ * siC_->getPropagationStepSize(), mpath[i]->GetCost());
                                // path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
                            }
                            else
                            {
                                // the root node does not have a parent
                                path->append(mpath[i]->state_, mpath[i]->GetCost());
                                
                            }
                            // previous
                            // path->append(prevSolution_[i], prevSolutionControls_[i - 1],
                            //              prevSolutionSteps_[i - 1] * siC_->getPropagationStepSize());
                        // not sure why this is here yet
                        }
                        path->append(prevSolution_[0]);
                        solved = true;
                        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    
                        if (benchmark_ == false)
                        {
                            pdef_->addSolutionPath(path, approximate, approxdif, getName());
                            std::ofstream PathFile;
                            PathFile.open(SolName_);
                            if (PathFile.fail())
                            {
                              std::cerr << "ERROR: Could not open path.txt" << std::endl;
                              exit(1);
                            }
                            else
                            {
                              OMPL_INFORM("Writing solution to path.txt");
                              // PathFile << data << std::endl;
                              path->printAsMatrix(PathFile);
                              PathFile.close();
                              OMPL_INFORM("Computation completed successfully");
                              // path.print(std::cout);  // this prints out the solution
                            }
                        }

                        // found a solution with better cost than prior
                        OMPL_INFORM("Found solution with cost %.2f", solution->accCost_.value());
                        OMPL_INFORM("Number of segments %i", solution->cost);
                        sufficientlyShort = opt_->isSatisfied(solution->accCost_);
                        if (sufficientlyShort)
                            break;
                    }
                    if (solution == nullptr && dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;

                        for (auto &i : prevSolution_)
                            if (i)
                                si_->freeState(i);
                        prevSolution_.clear();
                        for (auto &prevSolutionControl : prevSolutionControls_)
                            if (prevSolutionControl)
                                siC_->freeControl(prevSolutionControl);
                        prevSolutionControls_.clear();
                        prevSolutionSteps_.clear();

                        Motion *solTrav = approxsol;
                        while (solTrav->parent_ != nullptr)
                        {
                            prevSolution_.push_back(si_->cloneState(solTrav->state_));
                            prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_));
                            prevSolutionSteps_.push_back(solTrav->steps_);
                            solTrav = solTrav->parent_;
                        }
                        prevSolution_.push_back(si_->cloneState(solTrav->state_));
                    }

                    if (oldRep != rmotion)
                    {
                        while (oldRep->inactive_ && oldRep->numChildren_ == 0)
                        {
                            oldRep->inactive_ = true;
                            nn_->remove(oldRep);

                            if (oldRep->state_)
                                si_->freeState(oldRep->state_);
                            if (oldRep->control_)
                                siC_->freeControl(oldRep->control_);

                            oldRep->state_ = nullptr;
                            oldRep->control_ = nullptr;
                            oldRep->parent_->numChildren_--;
                            Motion *oldRepParent = oldRep->parent_;
                            delete oldRep;
                            oldRep = oldRepParent;
                        }
                    }
                }
            }
        }
        iterations++;
    }

    
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;
        Motion *lm = solution;

        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            // std::cout << solution->GetCost() << std::endl;
            mpath.push_back(solution);
            solution = solution->parent_;
        }
        /* set the solution path */
        // auto path(std::make_shared<PathControl>(si_));
        auto path(std::make_shared<MAPSRRTPathControl>(si_));

        FinalCost_ = mpath[0]->GetCost();

        SolName_ = "txt/SST/path" + std::to_string(lastGoalMotion_->accCost_.value()) + ".txt";

        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent_)
            {
                //  additional method added by Justin Kottinger
                path->append(mpath[i]->state_, mpath[i]->control_, mpath[i]->steps_ * siC_->getPropagationStepSize(), mpath[i]->GetCost());
                // path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            }
            else
            {
                // the root node does not have a parent
                path->append(mpath[i]->state_, mpath[i]->GetCost());
                
            }
            // previous
            // path->append(prevSolution_[i], prevSolutionControls_[i - 1],
            //              prevSolutionSteps_[i - 1] * siC_->getPropagationStepSize());
        // not sure why this is here yet
        path->append(prevSolution_[0]);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    
        if (benchmark_ == false)
            pdef_->addSolutionPath(path, approximate, approxdif, getName());
        std::ofstream PathFile;
        PathFile.open(SolName_);
        if (PathFile.fail())
        {
          std::cerr << "ERROR: Could not open path.txt" << std::endl;
          exit(1);
        }
        else
        {
          OMPL_INFORM("Writing solution to path.txt");
          // PathFile << data << std::endl;
          path->printAsMatrix(PathFile);
          PathFile.close();
          OMPL_INFORM("Computation completed successfully");
          // path.print(std::cout);  // this prints out the solution
        }
    }

    si_->freeState(xstate);
    if (rmotion->state_)
        si_->freeState(rmotion->state_);
    if (rmotion->control_)
        siC_->freeControl(rmotion->control_);
    delete rmotion;

    OMPL_INFORM("%s: Solution can explained in %i segment(s)", getName().c_str(), FinalCost_);
    OMPL_INFORM("%s: Created %u states in %u iterations", getName().c_str(), nn_->size(), iterations);

    return base::PlannerStatus(solved, approximate);
}

void ompl::control::MAPSSST::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    std::vector<Motion *> allMotions;
    if (nn_)
        nn_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->numChildren_ == 0)
        {
            allMotions.push_back(motion);
        }
    }
    for (unsigned i = 0; i < allMotions.size(); i++)
    {
        if (allMotions[i]->parent_ != nullptr)
        {
            allMotions.push_back(allMotions[i]->parent_);
        }
    }

    double delta = siC_->getPropagationStepSize();

    if (prevSolution_.size() != 0)
        data.addGoalVertex(base::PlannerDataVertex(prevSolution_[0]));

    for (auto m : allMotions)
    {
        if (m->parent_)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent_->state_), base::PlannerDataVertex(m->state_),
                             control::PlannerDataEdgeControl(m->control_, m->steps_ * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent_->state_), base::PlannerDataVertex(m->state_));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state_));
    }
}
