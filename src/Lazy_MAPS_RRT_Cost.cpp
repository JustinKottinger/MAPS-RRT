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

/* Author: Justin Kottinger */

// #include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/PathControl.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/spaces/SE2StateSpace.h"
// std includes
#include <limits>
#include <chrono>
#include <bits/stdc++.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/numeric/odeint.hpp>
#include <vector>
#include <algorithm>
#include <typeinfo>
// my includes
#include "../includes/Lazy_MAPS_RRT_Cost.h"
#include "../includes/ReadWorld.h"
#include "../includes/KinematicCar.h"
#include "../includes/MAPSRRTPathControl.h"
#include <boost/numeric/odeint/integrate/integrate_const.hpp>
#include "../includes/SimpleDirectedControlSamplerMAPS.h"


using namespace boost::numeric::odeint;
namespace bg = boost::geometry;


struct compare
{
    int key;
    compare(int const &i): key(i) { }

    bool operator()(int const &i)
    {
        return (i == key);
    }
};


// constructor
ompl::control::LazyMAPSRRTcost::LazyMAPSRRTcost(const ompl::control::SpaceInformationPtr &si, 
    int NumVehicles, int NumControls, int DimofEachVehicle,
    int MaxSegments, std::vector<double> goal, double radius, 
    bool benchmark,  std::string model, unsigned int k, std::string solutionName) : base::Planner(si, "Lazy MAPS-RRT Cost"), numControlSamples_(k)
{
    specs_.approximateSolutions = true;
    // get the address of the SpaceInformation
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

    Planner::declareParam<double>("goal_bias", this, &LazyMAPSRRTcost::setGoalBias, &LazyMAPSRRTcost::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &LazyMAPSRRTcost::setIntermediateStates, &LazyMAPSRRTcost::getIntermediateStates);

    addPlannerProgressProperty("best cost REAL", [this] { return FinalCostProperty(); });
    addPlannerProgressProperty("segmenting time REAL", [this] { return FinalTimeProperty(); });

}

ompl::control::LazyMAPSRRTcost::~LazyMAPSRRTcost()
{
    freeMemory();
}


void ompl::control::LazyMAPSRRTcost::setup()
{
    base::Planner::setup();
    // A nearest-neighbors datastructure containing the tree of motions
    // if not - null (meaning there is a tree present already) we reset it
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    // next, we set the distance function of the tree
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
    // the planner is now set-up
}

void ompl::control::LazyMAPSRRTcost::clear()
{
    // reset the planner and samplers
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    // free the memory
    freeMemory();
    // if the datastructure for motions exists, clear it to nothing
    if (nn_)
        nn_->clear();
    // remove the information stored at the lastGoalMotion_ location
    lastGoalMotion_ = nullptr;
}

void ompl::control::LazyMAPSRRTcost::freeMemory()
{
    // if the datastructer of motions exists
    if (nn_)
    {
        // create a list of motions
        std::vector<Motion *> motions;
        nn_->list(motions);
        // for all the motions stored in datastructure
        for (auto &motion : motions)
        {
            // if the motion has a state, remove it
            if (motion->state)
                si_->freeState(motion->state);
            // if the motion has a control, remove that also
            if (motion->control)
                siC_->freeControl(motion->control);
            // delete the entire motion now that its contents are also cleared
            // note that I think if one were to simply delete motion, the state and control
            //      may linger in memory somewhere
            delete motion;
        }
    }
}

std::vector<double> ompl::control::LazyMAPSRRTcost::getDistance(const base::State *st)
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

std::vector<double> ompl::control::LazyMAPSRRTcost::ThreeKinDistance(const ob::State *st)
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

std::vector<double> ompl::control::LazyMAPSRRTcost::TwoUnicycleDistance(const ob::State *st)
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

std::vector<double> ompl::control::LazyMAPSRRTcost::ThreeUnicycleDistance(const ob::State *st)
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

std::vector<double> ompl::control::LazyMAPSRRTcost::ThreeLinearDistance(const ob::State *st)
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

std::vector<double> ompl::control::LazyMAPSRRTcost::TwoLinearDistance(const ob::State *st)
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

std::vector<double> ompl::control::LazyMAPSRRTcost::TwoKinDistance(const ob::State *st)
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

int ompl::control::LazyMAPSRRTcost::MultiAgentControlSampler(Motion *motion,Control *RandCtrl, Control *previous, 
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

void ompl::control::LazyMAPSRRTcost::overrideStates(const std::vector<int> DoNotProp, const base::State *source, 
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

void ompl::control::LazyMAPSRRTcost::OverrideUniCars(const std::vector<int> DoNotProp, const base::State *source, 
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


void ompl::control::LazyMAPSRRTcost::OverrideLinCars(const std::vector<int> DoNotProp, const base::State *source, 
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

void ompl::control::LazyMAPSRRTcost::OverrideKinCars(const std::vector<int> DoNotProp, const base::State *source, 
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


std::vector<Point> ompl::control::LazyMAPSRRTcost::MakeLinearPath(const base::State *st) const
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
std::vector<Point> ompl::control::LazyMAPSRRTcost::MakeUniPath(const base::State *result) const
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
std::vector<Point> ompl::control::LazyMAPSRRTcost::MakeLinPath(const base::State *result) const
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
std::vector<Point> ompl::control::LazyMAPSRRTcost::MakeKinPath(const base::State *result) const
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
void ompl::control::LazyMAPSRRTcost::Get2DimDistance(Motion *motion, const base::State *source, 
    const base::State *result)
{
    if ((model_ == "2KinematicCars") || (model_ == "3Unicycle") || model_ == "3KinematicCars" || model_ == "2Unicycle")
        Get2DimDist2KinCars(motion, source, result);
    else if ((model_ == "2Linear") || (model_ == "3Linear"))
        Get2DimDist2LinCars(motion, source, result);
    else
    {
        std::cout << "Current Distance Model Not Implemented." << std::endl;
        exit(1);
    }
}

void ompl::control::LazyMAPSRRTcost::Get2DimDist2LinCars(Motion *motion, const base::State *source, 
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
void ompl::control::LazyMAPSRRTcost::Get2DimDist2KinCars(Motion *motion, const base::State *source, 
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


unsigned int ompl::control::LazyMAPSRRTcost::propagateWhileValid(Motion *motion,const base::State *state, 
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

void ompl::control::LazyMAPSRRTcost::FindTotalIntersections(Motion *NewMotion)
{
    // this function no longer works!
    std::clock_t start, end; 
    start = std::clock(); 
    // bool intersect
    std::vector<bool> NumIntersect = Project2D(NewMotion, 1);

    // if (NewMotion->LocationsOfIntersect.size() != 0)
    //     std::cout << NewMotion->LocationsOfIntersect.size();

    int ParentIntersections = NewMotion->parent->NumIntersections;

    if (NumIntersect[1] == true)
        NewMotion->NumIntersections = ParentIntersections + NumIntersect[1];
    else
        NewMotion->NumIntersections = ParentIntersections;

    end = std::clock();
    double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    time_ = time_ + time_taken;
}

std::vector<bool> ompl::control::LazyMAPSRRTcost::Project2D(Motion *NewMotion, const int depth)
{
    std::vector<bool> intersect;
    if (NumVs == 2)
        intersect = Project2D_2Vehicles(NewMotion, depth);
    else if (NumVs == 3)
        intersect = Project2D_3Vehicles(NewMotion);
    else
    {
        std::cout << "Current Number of Vehicles Not Supported." << std::endl;
        exit(1);
    }
    return intersect;
}

// note that this is not yet implemented
std::vector<bool> ompl::control::LazyMAPSRRTcost::Project2D_3Vehicles(Motion *NewMotion)
{
    int NumIntersect = 0;

    std::vector<bool> done;

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

    while (CurrMotion->parent != nullptr)
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
    NewMotion->LocationsOfIntersect = LocOfInt;
    return done;
}


std::vector<bool> ompl::control::LazyMAPSRRTcost::Project2D_2Vehicles(Motion *NewMotion, const int depth)
{
    int NumIntersect = 0;

    std::vector<bool> done;

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

    // while (CurrMotion->parent != nullptr)
    for (int i = 0; i < depth; i++)
    {
        if (CurrMotion->parent == nullptr)
        {
            // we have reached the beginning of the motion
            // done iterating through
            // this will be used to exit the while loop in FindTotalIntersections()
            done.push_back(true);
            break;
        }
        if (done.size() == 0)
            done.push_back(false);
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
                                // done.push_back(true);
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
                                // done.push_back(true);
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
    if (NumIntersect > 0)
        done.push_back(true);
    else
        done.push_back(false);
    NewMotion->LocationsOfIntersect = LocOfInt;
    return done;
}

unsigned int ompl::control::LazyMAPSRRTcost::FindTotalPathCost(Motion *LastMotion)
{
    // std::cout << "beginning cost calc" << std::endl;
    // std::cout << "in here" << std::endl;
    Motion *CurrMotion = LastMotion;
    // if (CurrMotion->LocationsOfIntersect.size() != 0)
    // {
    //     std::cout << CurrMotion->LocationsOfIntersect.size() << std::endl;
    // }
    
    int NumSegs = 1;
    int depth = 1;
    bool done = false;

    while (!done)
    {
        // std::cout << CurrMotion->NumIntersections << std::endl;
        // if ((CurrMotion->NumIntersections) == (BeginSegment->NumIntersections))
        // {
        //     std::cout << "there is a change in intersection" << std::endl;
        // }
        // else
        // {

        // need to check for a segmentation

        // THIS LINE WORKS!!!
        // std::vector<bool> info = CheckSegmentation(CurrMotion, depth, done);

        // EXPERIMENTAL!!
        std::vector<bool> info = CheckSegmentationTest(CurrMotion, depth);

        // std::cout << info[0] << info[1] << std::endl;


        if (info[1]) // if found an intersection
        {
            if (depth == 1) 
            {
                CurrMotion->SetCost(NumSegs);
                CurrMotion = CurrMotion->parent;
            }
            // we found an intersection in the segment
            // thus, we need to set the cost and then reset
            else
            {
                for (int i = 0; i < depth - 1; i++)
                {
                    // change the cost of all motions prior to the intersection
                    // this is depth-1 because there was no intersection prior to this iteration
                    // after chenging the cost, we move back to that motion
                    if (CurrMotion->parent != nullptr)
                    {
                        CurrMotion->SetCost(NumSegs);
                        CurrMotion = CurrMotion->parent;
                    }
                    else
                    {
                        CurrMotion->SetCost(NumSegs);
                        break;
                    }  
                }
            }
            NumSegs += 1;
            depth = 1;
            std::cout << NumSegs << std::endl;
        }
        if (info[0])  // if done
        {
            done = info[0];
            while (CurrMotion->parent != nullptr)
            {
                CurrMotion->SetCost(NumSegs);
                CurrMotion = CurrMotion->parent;
            }
        }
        else
        {
            depth += 1;
        }
        // }
    // CurrMotion = CurrMotion->parent;
    }
    // CurrMotion->parent->SetCost(NumSegs);
    CurrMotion->SetCost(NumSegs);
    // std::cout << "out" << std::endl;
    return NumSegs;
}

std::vector<bool> ompl::control::LazyMAPSRRTcost::CheckSegmentationTest(Motion *motion, int depth)
{
	std::vector<bool> info;

	std::vector<Motion *> CurrSegment;

	const Motion *CurrMotion = motion;

	Motion *CreateSegment = motion;

	for (int i = 0; i < depth; i++)
	{
		if (CreateSegment->parent != nullptr)
		{
			CurrSegment.push_back(CreateSegment);
			CreateSegment = CreateSegment->parent;
		}
		else
		{
			CurrSegment.push_back(CreateSegment);
			info.push_back(true);
			break;
		}

	}
	if (info.size() == 0)
		info.push_back(false);

	for (int j = 0; j < CurrSegment.size(); j++)
	{
        // std::cout << CurrSegment[j] << std::endl;
        // std::cout << CurrMotion->LocationsOfIntersect.size() << std::endl;

        for (int check = 0; check < CurrSegment.size(); check++)
        {

		  for (int m = 0; m < CurrSegment[check]->LocationsOfIntersect.size(); m++)
		  {
                // std::cout << CurrSegment[j] << std::endl;
                // std::cout << CurrMotion->LocationsOfIntersect[m] << std::endl;
		  	if  (CurrSegment[j] == CurrSegment[check]->LocationsOfIntersect[m])
		  	{
                    std::cout << "here" << std::endl;
		  		// there is an intersection within segment...
		  		info.push_back(true);
		  		return info;
		  	}
		  }
        }
	}
	info.push_back(false);
	return info;
}

std::vector<bool> ompl::control::LazyMAPSRRTcost::CheckSegmentation(Motion *motion, int depth, bool done)
{
    // std::cout << "in segment" << std::endl;
    std::vector<bool> info;
    // copy the motion into a local vatiable
    const Motion *CurrMotion = motion;

    // // initialize a vector of lines for each vehilces motion
    std::vector<Segment> V1motions;
    std::vector<Segment> V2motions;

    // create segments for all vehicles[[list of segments for v1path], [list of segments for v2path]]
    std::vector<std::vector<Segment>> AllVehicleNewMotionPath;

    AllVehicleNewMotionPath.push_back(V1motions);
    AllVehicleNewMotionPath.push_back(V2motions);

    //  for the specified number of times, we are going to create a line from parent to state
    // for each vehicle
    for (int i = 0; i < depth; i++)
    {
        // std::cout << "getting path" << std::endl;
        std::vector<std::vector<Point>> CurrPath = CurrMotion->LinearPath;
        // std::cout << CurrPath.size() << std::endl;
        // std::cout << CurrPath[0].size() << std::endl;
        // std::cout << CurrPath[1].size() << std::endl;
        for (int v = 0; v < NumVs; v++)
        {
            for (int j = 0; j < CurrPath.size() - 1; j++)
            {
                // std::cout << "getting parent" << std::endl;
                Point parent = (CurrPath[j][v]);
                // std::cout << "getting child" << std::endl;
                Point child = (CurrPath[j + 1][v]);

                Segment VSeg(parent, child);

                AllVehicleNewMotionPath[v].push_back(VSeg);
            }
            
        }
        if (CurrMotion->parent->LinearPath.size() == 0)
        {
            i = 10000;
            done = true;
            info.push_back(done);
            break;
        }
        else
        {
            CurrMotion = CurrMotion->parent;
        }

    }
    if (info.size() == 0)
        info.push_back(false);

    // std::cout << "out of while loop" << std::endl;
    // std::cout << AllVehicleNewMotionPath.size() << " " << AllVehicleNewMotionPath[0].size() << 
    //     " " << AllVehicleNewMotionPath[1].size() << std::endl;
    // now that we have a vector of line segments, we need to check if the segments intersect
    for (int v1 = 0; v1 < AllVehicleNewMotionPath[0].size(); v1++)
    {
        for (int v2 = 0; v2 < AllVehicleNewMotionPath[1].size(); v2++)
        {
            bool intersect = boost::geometry::intersects(AllVehicleNewMotionPath[0][v1], AllVehicleNewMotionPath[1][v2]);
            if (intersect)
            {
                // intersection was found, no need to continue with for loop
                info.push_back(true);
                // std::cout << "out segments" << std::endl;
                return info;
            }
        }
    }
    // std::cout << "end of double for loop" << std::endl;
    // no intersection was found
    info.push_back(false);
    // std::cout << "out segment" << std::endl;
    return info; 
}



// main algorithm
ompl::base::PlannerStatus ompl::control::LazyMAPSRRTcost::solve(const base::PlannerTerminationCondition &ptc)
{
    std::clock_t start, end;
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = siC_->allocStateSampler();
    // if (!controlSampler_)
    //     controlSampler_ = siC_->allocDirectedControlSampler();

    // MAPS uses its own control sampler 

    SimpleDirectedControlSamplerMAPS CntrlSampler(siC_, goal, g);


    OMPL_INFORM("%s: Starting planning with %u states already in datastructure.", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    base::State *xstate = siC_->allocState();
    // std::cout << "here";
    // exit(1);

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            // std::cout << "accessing goal_s" << std::endl;
            goal_s->sampleGoal(rstate);
        }
        else
        {
            sampler_->sampleUniform(rstate);
        }

        // std::cout << "success" << std::endl;
        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        // extended by Justin for MAPS RRT
        // std::cout << "accessing sample goal biasing" << std::endl;
        // unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
        // unsigned int cd = CntrlSampler.sampleToMAPS(rctrl, nmotion->control, nmotion->state, rmotion->state);
        unsigned int cd = MultiAgentControlSampler(rmotion, rctrl, nmotion->control, nmotion->state, rmotion->state);
        
        if (cd >= siC_->getMinControlDuration())
        {

            /* create a motion */
            auto *motion = new Motion(siC_);
            // copy state and controls that were sampled into the motion
            si_->copyState(motion->state, rmotion->state);
            siC_->copyControl(motion->control, rctrl);
            motion->steps = cd;
            motion->parent = nmotion;
            motion->LinearPath = rmotion->LinearPath;
            motion->AllVehicleDistance = rmotion->AllVehicleDistance;
            // std::cout << motion->parent << std::endl;
            
            nn_->add(motion);
            double dist = 0.0;
            bool solv = goal->isSatisfied(motion->state, &dist);
            
            if (solv)
            {
                std::cout << "found solution" << std::endl;
                approxdif = dist;
                solution = motion;
                // FindTotalIntersections(solution);
                // unsigned int cost = FindTotalPathCost(solution);
                int cost = PostProcess(solution);

                if (cost <= MaxSegments_)
                {                     
                    // start = std::clock();
                    break;
                }
                else
                {
                    // remove all irrelevant motions
                    
                    const int origCost = solution->cost;
                    int currCost = solution->cost;
                    Motion* CurrMotion = solution;
                    int n = 0;
                    while (currCost == origCost)
                    {
                        // update cost
                        currCost = CurrMotion->parent->cost;
                        Motion* temp = CurrMotion->parent;
                        // remove node
                        nn_->remove(CurrMotion);
                        CurrMotion = temp;
                        n += 1;
                    }
                    std::cout << "not valid sol -- pruned " << n << "nodes" << std::endl;
                }
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = motion;
            }
        }
    }

    // end = std::clock();
    // double time_taken = double(end - start); // double(CLOCKS_PER_SEC);
    // std::cout << time_taken << std::endl;

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    // Motion *last = solution;
    


    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;
        // Motion *lm = solution;
        // std::cout << lm->LocationsOfIntersect.size() << std::endl;
        // FindTotalIntersections(lm);
        // int MotionCost = FindTotalSegments(lm);
        // std::cout << MotionCost << std::endl;
        // FindTotalIntersections(lastGoalMotion_);
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            // std::cout << solution->GetCost() << std::endl;
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        // note that we use a different PathControl class
        // see MAPSRRTPathControl for more informaiton
        auto path(std::make_shared<MAPSRRTPathControl>(si_));
        // auto path(std::make_shared<PathControl>(si_));
        // auto path(std::make_shared<PathControl>(si_));
        FinalCost_ = mpath[0]->GetCost();
        for (int i = mpath.size() - 1; i >= 0; --i)
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

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);
    OMPL_INFORM("%s: Time Spent Segmentating %f", getName().c_str(), time_);
    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
    // To see the error from exact sol to sol found
    OMPL_INFORM("%s: Solution Tollorance was %f", getName().c_str(), approxdif);
    OMPL_INFORM("%s: Solution can explained in %i segment(s)", getName().c_str(), FinalCost_);
    return base::PlannerStatus(solved, approximate);
}

int ompl::control::LazyMAPSRRTcost::PostProcess(Motion *lastmotion)
{
    // eventually do some stuff here...
    int depth = 1;
    int NumSegs = 1;
    // bool done = false; 
    Motion *CurrMotion = lastmotion;

    while (CurrMotion->parent != nullptr)
    {

        std::vector<bool> intersect = PostProcess2DProject(CurrMotion, depth);

        if (intersect[0])
        {
            // if intersection was found
            if (intersect[1])
            {
                for (int i = 0; i < depth - 1; i++)
                {
                    // change the cost of all motions prior to the intersection
                    // this is depth-1 because there was no intersection prior to this iteration
                    // after chenging the cost, we move back to that motion
                    if (CurrMotion != nullptr)
                    {
                        CurrMotion->cost = NumSegs;
                        CurrMotion = CurrMotion->parent;
                    }
                    else
                    {
                        CurrMotion->cost = NumSegs;
                        break;
                    }  
                }
                // there was in intersection, thus, need to add cost to future path segments
                NumSegs += 1;
            }
            // if no intersection was found, then we just propogate back
            else
            {
                // no intersection, just set the rest of the motion costs to a current one
                while (CurrMotion->parent != nullptr)
                {
                    CurrMotion->cost = NumSegs;
                    CurrMotion = CurrMotion->parent;
                }
            }
            // we are done so we change this to exit loop
            // done = true;
        }
        // not done and intersection
        else if (intersect[1])
        {
            if (depth == 1) // the motion itself causes an intersection!
            {
                // NumSegs += 1;
                CurrMotion->cost = NumSegs;
                CurrMotion = CurrMotion->parent;
            }
            // change the cost of all motions prior to the intersection
            // this is depth-1 because there was no intersection prior to this iteration
            // after chenging the cost, we move back to that motion
            else
            {
            for (int i = 0; i < depth - 1; i++)
                {
                    // change the cost of all motions prior to the intersection
                    // this is depth-1 because there was no intersection prior to this iteration
                    // after chenging the cost, we move back to that motion
                    if (CurrMotion != nullptr)
                    {
                        CurrMotion->cost = NumSegs;
                        CurrMotion = CurrMotion->parent;
                    }
                    else
                    {
                        CurrMotion->cost = NumSegs;
                        break;
                    }  
                }
            
            }
            NumSegs += 1;
            // there was in intersection, thus, need to add cost to future path segments
            
            // change depth to 1 since we reset the projection at the current motion
            depth = 1;
            // Motion = Motion->parent;

        }
        // else
        // {
            // not done and no intersection
            // set the cost to whatever it may be at this moment
        else if (!intersect[1])
        {
            // Motion->SetCost(NumSegs);
            depth += 1;
        }
        
                
    }
    CurrMotion->cost = NumSegs;
    return NumSegs;
}


std::vector<bool> ompl::control::LazyMAPSRRTcost::PostProcess2DProject(const Motion *motion, int depth)
{
    // std::cout << "in 2DProject" << std::endl;

    // initalize the return vector
    // will contain information regarding if we have reached the beginning of the motion and if there
    // was an intersection
    std::vector<bool> done;
    // this function is going to take in a motion 
    // and linearly interpolate between the current state and parent state
    // for a specified number of times
    // Then, it takes the lines and checks for intersections
    // if there is an intersection, then we return false

    // copy the motion into a local vatiable
    const Motion *CurrMotion = motion;

    // // initialize a vector of lines for each vehilces motion
    std::vector<Segment> V1motions;
    std::vector<Segment> V2motions;

    //  for the specified number of times, we are going to create a line from parent to state
    // for each vehicle
    for (int i = 0; i < depth; i++)
    {
        if (CurrMotion->parent == nullptr)
        {
            // we have reached the beginning of the motion
            // done iterating through
            // this will be used to exit the while loop in FindTotalIntersections()
            done.push_back(true);
            break;
        }
        else
        {
            // get the parent state for both vehicles
            const ob::CompoundStateSpace::StateType *Par = CurrMotion->parent->state->
                as<ob::CompoundStateSpace::StateType>();

            const auto *ParxyState1 = Par->as<ob::RealVectorStateSpace::StateType>(0);
            const auto *Parrot1 = Par->as<ob::SO2StateSpace::StateType>(1);
            const auto *ParxyState2 = Par->as<ob::RealVectorStateSpace::StateType>(2);
            const auto *Parrot2 = Par->as<ob::SO2StateSpace::StateType>(3);

            // construct a class with the control inputs to be used for integration
            // this is how I figured out how to use boost::numeric::odeint with control inputs
            // need to go back eventually to combine with other class I have
            // seems counter-productive to have two very similar classes
            TwoKinCarsODE Model(CurrMotion->control);

            // initialize the vector of states to be filled during integration
            std::vector<state_type> x_vec;
            std::vector<double> times;

            // ompl types do not communicate with boost 
            // Thus, need to translate the parent node
            state_type p = {ParxyState1->values[0], ParxyState1->values[1], Parrot1->value,
                         ParxyState2->values[0], ParxyState2->values[1], Parrot2->value}; // initial conditions
            
            // integrate from the parent state with control specified in the construction of model

            integrate( Model , p , 0.0 , (CurrMotion->steps) * (siC_->getPropagationStepSize()), 0.01, push_back_state_and_time(x_vec, times));

            // now that we integrated, we need to interpolate the pieces together
            for (int j=0; j < x_vec.size() - 1; j++)
            {
                // get the parent
                point A1(x_vec[j][0], x_vec[j][1]);
                point A2(x_vec[j][3], x_vec[j][4]);
                // get the child
                point B1(x_vec[j + 1][0], x_vec[j + 1][1]);
                point B2(x_vec[j + 1][3], x_vec[j + 1][4]);

                // linearly interpolate between the nodes
                Segment V1path(A1, B1);
                Segment V2path(A2, B2);

                // add the lines to the vector to check for intersections 
                V1motions.push_back(V1path);
                V2motions.push_back(V2path);
            }
        }
        // move on to the next line segment
        CurrMotion = CurrMotion->parent;
    }

    // if we found the start node, we tell other function that we are at the beginning
    // otherwise, we need to tell the other function we are not yet done
    if (done.size() == 0)
        done.push_back(false);

    // now that we have a vector of line segments, we need to check if the segments intersect
    for (int i = 0; i < V1motions.size(); i++)
    {
        for(int j = 0; j < V2motions.size(); j++)
        {
            bool intersect = boost::geometry::intersects(V1motions[i], V2motions[j]);
            if (intersect)
            {
                // intersection was found, no need to continue with for loop
                done.push_back(true);
                return done;
            }
        }
    }
    // no intersection was found
    done.push_back(false);
    return done; 
}

void ompl::control::LazyMAPSRRTcost::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}