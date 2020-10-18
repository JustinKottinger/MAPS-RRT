/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Ryan Luna */

#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>
// my includes
#include "../includes/MAPS_EST.h"
#include "../includes/ReadWorld.h"
#include "../includes/KinematicCar.h"
#include "../includes/MAPSRRTPathControl.h"
#include <boost/numeric/odeint/integrate/integrate_const.hpp>
#include "../includes/SimpleDirectedControlSamplerMAPS.h"
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


ompl::control::MAPSEST::MAPSEST(const SpaceInformationPtr &si, 
	int NumVehicles, int NumControls, int DimofEachVehicle,
    int MaxSegments, std::vector<double> goal, double radius, 
    bool benchmark,  std::string model, unsigned int k, 
    std::string solutionName) : base::Planner(si, "MAPSEST")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

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

    Planner::declareParam<double>("range", this, &MAPSEST::setRange, &MAPSEST::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &MAPSEST::setGoalBias, &MAPSEST::getGoalBias, "0.:.05:1.");

    addPlannerProgressProperty("best cost REAL", [this] { return FinalCostProperty(); });
    addPlannerProgressProperty("segmenting time REAL", [this] { return FinalTimeProperty(); });
}

ompl::control::MAPSEST::~MAPSEST()
{
    freeMemory();
}

void ompl::control::MAPSEST::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);

    tree_.grid.setDimension(projectionEvaluator_->getDimension());
}

void ompl::control::MAPSEST::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    tree_.grid.clear();
    tree_.size = 0;
    pdf_.clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::MAPSEST::freeMemory()
{
    for (const auto &it : tree_.grid)
    {
        for (const auto &motion : it.second->data.motions_)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

std::vector<double> ompl::control::MAPSEST::getDistance(const base::State *st) const
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
    else if (model_ == "2Unicycle2ndOrder")
        distance = Two2ndOrderUnicycleDistance(st);
    else if (model_ == "3KinematicCars")
        distance = ThreeKinDistance(st);
    else if (model_ == "2Linear2ndOrder")
        distance = Two2ndOrderLinearDistance(st);
    else if (model_ == "2KinematicCars2ndOrder")
        distance = Two2ndOrderCarDistance(st);
    else
    {
        std::cout << "Current Distance Model Not Implemented." << std::endl;
        exit(1);
    }
    return distance;
}

std::vector<double> ompl::control::MAPSEST::Two2ndOrderCarDistance(const ob::State *st) const
{
  
  std::vector<double> distance;
  auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
  auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
  auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);

  double deltax_v1 = pow((g[0] - xyState1_->values[0]), 2);
  double deltay_v1 = pow((g[1] - xyState1_->values[1]), 2);
  double deltax_v2 = pow((g[5] - xyState2_->values[0]), 2);
  double deltay_v2 = pow((g[6] - xyState2_->values[1]), 2);

  double d1 = sqrt(deltax_v1 + deltay_v1);
  double d2 = sqrt(deltax_v2 + deltay_v2);

  distance.push_back(d1);
  distance.push_back(d2);

  return distance;
}

std::vector<double> ompl::control::MAPSEST::Two2ndOrderLinearDistance(const ob::State *st) const
{
  std::vector<double> distance;
  auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
  auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
  auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);

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

std::vector<double> ompl::control::MAPSEST::Two2ndOrderUnicycleDistance(const ob::State *st) const
{
  
  std::vector<double> distance;
  auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
  auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
  auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(3);

  double deltax_v1 = pow((g[0] - xyState1_->values[0]), 2);
  double deltay_v1 = pow((g[1] - xyState1_->values[1]), 2);
  double deltax_v2 = pow((g[5] - xyState2_->values[0]), 2);
  double deltay_v2 = pow((g[6] - xyState2_->values[1]), 2);

  double d1 = sqrt(deltax_v1 + deltay_v1);
  double d2 = sqrt(deltax_v2 + deltay_v2);

  distance.push_back(d1);
  distance.push_back(d2);

  return distance;
}

std::vector<double> ompl::control::MAPSEST::ThreeKinDistance(const ob::State *st) const
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

std::vector<double> ompl::control::MAPSEST::TwoUnicycleDistance(const ob::State *st) const
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


std::vector<double> ompl::control::MAPSEST::ThreeUnicycleDistance(const ob::State *st) const
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

std::vector<double> ompl::control::MAPSEST::ThreeLinearDistance(const ob::State *st) const
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

std::vector<double> ompl::control::MAPSEST::TwoLinearDistance(const ob::State *st) const
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

std::vector<double> ompl::control::MAPSEST::TwoKinDistance(const ob::State *st) const
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


int ompl::control::MAPSEST::MultiAgentControlSampler(Motion *motion,Control *RandCtrl, Control *previous, 
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

unsigned int ompl::control::MAPSEST::propagateWhileValid(Motion *motion,const base::State *state, 
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

void ompl::control::MAPSEST::overrideStates(const std::vector<int> DoNotProp, const base::State *source, 
    base::State *result, Control *control)
{
    if (model_ == "2KinematicCars" || model_ == "3KinematicCars")
        OverrideKinCars(DoNotProp, source, result, control);
    else if ((model_ == "2Linear") || (model_ == "3Linear"))
        OverrideLinCars(DoNotProp, source, result, control);
    else if (model_ == "3Unicycle" || model_ == "2Unicycle")
        OverrideUniCars(DoNotProp, source, result, control);
    else if (model_ == "2Unicycle2ndOrder")
        Override2ndOrderUniCars(DoNotProp, source, result, control);
    else if (model_ == "2Linear2ndOrder")
        Override2ndOrderLinCars(DoNotProp, source, result, control);
    else if (model_ == "2KinematicCars2ndOrder")
        Override2ndOrderKinCars(DoNotProp, source, result, control);
    else
    {
        std::cout << "Current Override States Model Not Implemented." << std::endl;
        exit(1);
    }
}

void ompl::control::MAPSEST::Override2ndOrderKinCars(const std::vector<int> DoNotProp, const base::State *source, 
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
            xyDest->values[2] = SRCxyState->values[2];
            xyDest->values[3] = SRCxyState->values[3];

            // overriding the orientation
            rotd->value = rots->value;

        }
    }
}

void ompl::control::MAPSEST::Override2ndOrderUniCars(const std::vector<int> DoNotProp, const base::State *source, 
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
                destination->as<ob::RealVectorStateSpace::StateType>(3*DoNotProp[i]);

            // get the specific xy state of the source
            const auto *SRCxyState = src->as<ob::RealVectorStateSpace::StateType>(3*DoNotProp[i]);

            // get the specific orientation of result
            ob::SO2StateSpace::StateType *rotd = destination->as<ob::SO2StateSpace::StateType>(3*DoNotProp[i] + 1);

            // get the specific orientation of the source
            const auto *rots = src->as<ob::SO2StateSpace::StateType>(3*DoNotProp[i] + 1);

            // get the specific omega of result
            ob::RealVectorStateSpace::StateType *omegad = destination->as<ob::RealVectorStateSpace::StateType>(3*DoNotProp[i] + 2);

            // get the specific omega of the source
            const auto *omegas = src->as<ob::RealVectorStateSpace::StateType>(3*DoNotProp[i] + 2);

            // overriding the position state
            xyDest->values[0] = SRCxyState->values[0];
            xyDest->values[1] = SRCxyState->values[1];

            // overriding the orientation
            rotd->value = rots->value;

            // overriding the omega
            omegad->values[0] = omegas->values[0];

        }
    }
}

void ompl::control::MAPSEST::OverrideUniCars(const std::vector<int> DoNotProp, const base::State *source, 
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


void ompl::control::MAPSEST::OverrideLinCars(const std::vector<int> DoNotProp, const base::State *source, 
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

void ompl::control::MAPSEST::Override2ndOrderLinCars(const std::vector<int> DoNotProp, const base::State *source, 
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

            // overriding the state
            xyDest->values[0] = SRCxyState->values[0];
            xyDest->values[1] = SRCxyState->values[1];
            xyDest->values[2] = SRCxyState->values[2];
            xyDest->values[3] = SRCxyState->values[3];

        }
    }
}

void ompl::control::MAPSEST::OverrideKinCars(const std::vector<int> DoNotProp, const base::State *source, 
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

std::vector<Point> ompl::control::MAPSEST::MakeLinearPath(const base::State *st) const
{
    std::vector<Point> VehiclePoints;
    if (model_ == "2KinematicCars" || model_ == "3KinematicCars" || model_ == "2KinematicCars2ndOrder")
        VehiclePoints = MakeKinPath(st);
    else if ((model_ == "2Linear") || (model_ == "3Linear") || (model_ == "2Linear2ndOrder"))
        VehiclePoints = MakeLinPath(st);
    else if (model_ == "3Unicycle" || model_ == "2Unicycle")
        VehiclePoints = MakeUniPath(st);
    else if (model_ == "2Unicycle2ndOrder")
        VehiclePoints = Make2ndOrderUniPath(st);
    else
    {
        std::cout << "Current Linear Path Model Not Implemented." << std::endl;
        exit(1);
    }
    return VehiclePoints;
}

std::vector<Point> ompl::control::MAPSEST::Make2ndOrderUniPath(const base::State *result) const
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
            destination->as<ob::RealVectorStateSpace::StateType>(3 * i);

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
std::vector<Point> ompl::control::MAPSEST::MakeUniPath(const base::State *result) const
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
std::vector<Point> ompl::control::MAPSEST::MakeLinPath(const base::State *result) const
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
std::vector<Point> ompl::control::MAPSEST::MakeKinPath(const base::State *result) const
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
void ompl::control::MAPSEST::Get2DimDistance(Motion *motion, const base::State *source, 
    const base::State *result)
{
    if ((model_ == "2KinematicCars") || (model_ == "3Unicycle") 
        || model_ == "3KinematicCars" || (model_ == "2Unicycle") || model_ == "2KinematicCars2ndOrder")
        Get2DimDist2KinCars(motion, source, result);
    else if ((model_ == "2Linear") || (model_ == "3Linear") || (model_ == "2Linear2ndOrder"))
        Get2DimDist2LinCars(motion, source, result);
    else if (model_ == "2Unicycle2ndOrder")
        Get2DimDist2ndOrderUni(motion, source, result);
    else
    {
        std::cout << "Current Distance Model Not Implemented." << std::endl;
        exit(1);
    }
}

// different from distance function above in that this function saves distances to the node
void ompl::control::MAPSEST::Get2DimDist2ndOrderUni(Motion *motion, const base::State *source, 
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
            src->as<ob::RealVectorStateSpace::StateType>(3*i);

        // get the xy point of the result per vehicle
        const ob::RealVectorStateSpace::StateType *DESTxyState = 
            destination->as<ob::RealVectorStateSpace::StateType>(3*i);

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

void ompl::control::MAPSEST::Get2DimDist2LinCars(Motion *motion, const base::State *source, 
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
void ompl::control::MAPSEST::Get2DimDist2KinCars(Motion *motion, const base::State *source, 
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

void ompl::control::MAPSEST::FindTotalIntersections(Motion *NewMotion)
{
    // bool intersect
    std::clock_t start, end; 
    start = std::clock();
    int NumIntersect = Project2D(NewMotion);

    // if (NewMotion->LocationsOfIntersect.size() != 0)
    //     std::cout << NewMotion->LocationsOfIntersect.size();

    int ParentIntersections = NewMotion->parent->NumIntersections;

    if (NumIntersect > 0)  // only checks parent -> child 
    {
        NewMotion->NumIntersections = ParentIntersections + NumIntersect;
        NewMotion->cost = NewMotion->parent->cost + 1;
    }
    else
    {
        NewMotion->cost = NewMotion->parent->cost;
        NewMotion->NumIntersections = ParentIntersections;
    }

    end = std::clock();
    double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    time_ = time_ + time_taken;
}

int ompl::control::MAPSEST::Project2D(Motion *NewMotion)
{
    int intersects;
    if (NumVs == 2)
    {
        intersects = Project2D_2Vehicles(NewMotion);
    }
    else if (NumVs == 3)
        intersects = Project2D_3Vehicles(NewMotion);
    else
    {
        std::cout << "Current Number of Vehicles Not Supported." << std::endl;
        exit(1);
    }
    return intersects;
}

int ompl::control::MAPSEST::Project2D_3Vehicles(Motion *NewMotion)
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
    if (CurrMotion->parent)
    {
        int CurrCost = CurrMotion->parent->cost;
        
        while (CurrMotion->parent && CurrMotion->parent->cost == CurrCost)
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
            CurrMotion = CurrMotion->parent;
        }
    } 
    NewMotion->LocationsOfIntersect = LocOfInt;
    return NumIntersect;
}


int ompl::control::MAPSEST::Project2D_2Vehicles(Motion *NewMotion)
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

    if (CurrMotion->parent)
    {
        int CurrCost = CurrMotion->parent->cost;

        while (CurrMotion->parent && CurrMotion->parent->cost == CurrCost)
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
            CurrMotion = CurrMotion->parent;
        }
    }
    NewMotion->LocationsOfIntersect = LocOfInt;
    return NumIntersect;
}


ompl::base::PlannerStatus ompl::control::MAPSEST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    // Initializing tree with start state(s)
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        addMotion(motion);
    }

    if (tree_.grid.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    // Ensure that we have a state sampler AND a control sampler
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), tree_.size);

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(siC_);
    bool solved = false;
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    base::State *xstate = siC_->allocState();

    while (!ptc)
    {
        // Select a state to expand the tree from
        Motion *existing = selectMotion();
        assert(existing);

        // sample a random state (with goal biasing) near the state selected for expansion
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rmotion->state);
        else
        {
            if (!sampler_->sampleNear(rmotion->state, existing->state, maxDistance_))
                continue;
        }

        // Extend a motion toward the state we just sampled
        // unsigned int duration =
            // controlSampler_->sampleTo(rmotion->control, existing->control, existing->state, rmotion->state);
        
        unsigned int duration = MultiAgentControlSampler(rmotion, rmotion->control, existing->control, existing->state, rmotion->state);

        auto *motion = new Motion(siC_);
        // copy state and controls that were sampled into the motion
        // si_->copyState(motion->state, rmotion->state);
        // siC_->copyControl(motion->control, rctrl);
        // motion->steps = cd;
        // motion->parent = nmotion;
        // motion->LinearPath = rmotion->LinearPath;
        // motion->AllVehicleDistance = rmotion->AllVehicleDistance;
        // If the system was propagated for a meaningful amount of time, save into the tree
        if (duration >= siC_->getMinControlDuration())
        {
            // create a motion to the resulting state
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, rmotion->state);
            siC_->copyControl(motion->control, rmotion->control);
            motion->steps = duration;
            motion->parent = existing;
            motion->LinearPath = rmotion->LinearPath;
        	motion->AllVehicleDistance = rmotion->AllVehicleDistance;

            FindTotalIntersections(motion);

            if (motion->cost <= MaxSegments_)
            {
            	// save the state
            	addMotion(motion);

            	// Check if this state is the goal state, or improves the bMAPSEST solution so far
            	double dist = 0.0;
            	solved = goal->isSatisfied(motion->state, &dist);
            	if (solved)
            	{
            	    approxdif = dist;
            	    solution = motion;
            	    break;
            	}
            	if (dist < approxdif)
            	{
            	    approxdif = dist;
            	    approxsol = motion;
            	}
            }
        }
    }

    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    // Constructing the solution path
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        auto path(std::make_shared<MAPSRRTPathControl>(si_));
        FinalCost_ = mpath[0]->GetCost();
        for (int i = mpath.size() - 1; i >= 0; --i)
        {
            if (mpath[i]->parent)
            {
                //  additional method added by Justin Kottinger
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize(), mpath[i]->GetCost());
                // path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            }
            else
            {
                // the root node does not have a parent
                path->append(mpath[i]->state, mpath[i]->GetCost());
                
            }
        }
        solved = true;
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
    }

    // Cleaning up memory
    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states in %u cells", getName().c_str(), tree_.size, tree_.grid.size());

    return base::PlannerStatus(solved, approximate);
}

ompl::control::MAPSEST::Motion *ompl::control::MAPSEST::selectMotion()
{
    GridCell *cell = pdf_.sample(rng_.uniform01());
    return cell && !cell->data.empty() ? cell->data[rng_.uniformInt(0, cell->data.size() - 1)] : nullptr;
}

void ompl::control::MAPSEST::addMotion(Motion *motion)
{
    Grid<MotionInfo>::Coord coord(projectionEvaluator_->getDimension());
    projectionEvaluator_->computeCoordinates(motion->state, coord);
    GridCell *cell = tree_.grid.getCell(coord);
    if (cell)
    {
        cell->data.push_back(motion);
        pdf_.update(cell->data.elem_, 1.0 / cell->data.size());
    }
    else
    {
        cell = tree_.grid.createCell(coord);
        cell->data.push_back(motion);
        tree_.grid.add(cell);
        cell->data.elem_ = pdf_.add(cell, 1.0);
    }
    tree_.size++;
}

void ompl::control::MAPSEST::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<MotionInfo> motionInfo;
    tree_.grid.getContent(motionInfo);

    double stepSize = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &mi : motionInfo)
        for (auto &motion : mi.motions_)
        {
            if (motion->parent)
            {
                if (data.hasControls())
                    data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state),
                                 PlannerDataEdgeControl(motion->control, motion->steps * stepSize));
                else
                    data.addEdge(base::PlannerDataVertex(motion->parent->state),
                                 base::PlannerDataVertex(motion->state));
            }
            else
                data.addStartVertex(base::PlannerDataVertex(motion->state));
        }
}
