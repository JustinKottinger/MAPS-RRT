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

#include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "../includes/MyRRT.h"
#include <limits>
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "../includes/SimpleDirectedControlSamplerMAPS.h"
#include "../includes/RRTplusPathControl.h"
#include "../includes/KinematicCar.h"
#include <iostream>
#include <fstream>
#include <valarray>
#include <chrono>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/numeric/odeint.hpp>
#include "../includes/MAPSRRTPathControl.h"

using namespace boost::numeric::odeint;
namespace bg = boost::geometry;

ompl::control::MyRRT::MyRRT(const SpaceInformationPtr &si) : base::Planner(si, "RRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &MyRRT::setGoalBias, &MyRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &MyRRT::setIntermediateStates, &MyRRT::getIntermediateStates);

    addPlannerProgressProperty("best cost REAL", [this] { return FinalCostProperty(); });
    addPlannerProgressProperty("segmenting time REAL", [this] { return FinalSegTimeProperty(); });
}

ompl::control::MyRRT::~MyRRT()
{
    freeMemory();
}

void ompl::control::MyRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
}

void ompl::control::MyRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::MyRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::control::MyRRT::solve(const base::PlannerTerminationCondition &ptc)
{
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
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();


    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);


        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);
        if (addIntermediateStates_)
        {
            // this code is contributed by Jennifer Barry
            std::vector<base::State *> pstates;
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

            if (cd >= siC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for (; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    auto *motion = new Motion();
                    motion->state = pstates[p];
                    // we need multiple copies of rctrl
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;
                    nn_->add(motion);
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

                // free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
                for (auto &pstate : pstates)
                    si_->freeState(pstate);
        }
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
                /* create a motion */
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;

                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state, &dist);
                if (solv)
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

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    auto start = std::chrono::steady_clock::now();
    // PostProcess(solution);
    auto end = std::chrono::steady_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    SegmentTime_ = (dur * 0.000001);  // convert microseconds to seconds

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
            {
                path->append(mpath[i]->state);
                // FinalCost_ = mpath[i]->cost;
            }
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    // std::cout << SegmentTime_ << std::endl;
    // std::cout << FinalCost_ << std::endl;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return base::PlannerStatus(solved, approximate);
}

void ompl::control::MyRRT::PostProcess(Motion *lastmotion)
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
}


std::vector<bool> ompl::control::MyRRT::PostProcess2DProject(const Motion *motion, int depth)
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

void ompl::control::MyRRT::getPlannerData(base::PlannerData &data) const
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
