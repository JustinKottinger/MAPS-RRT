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
// #include "ompl/control/PathControl.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/spaces/SE2StateSpace.h"
// std includes
#include <limits>
#include <boost/numeric/odeint.hpp>
// my includes
#include "../includes/MyPlanner.h"
#include "../includes/ReadWorld.h"
#include "../includes/KinematicCar.h"
#include "../includes/MAPSRRTPathControl.h"
#include <boost/numeric/odeint/integrate/integrate_const.hpp>


using namespace boost::numeric::odeint;

typedef boost::array< double , 6 > state_type;


// constructor
ompl::control::MAPSRRT::MAPSRRT(const SpaceInformationPtr &si, int NumVehicles, int DimofEachVehicle,
    int MaxSegments) : base::Planner(si, "MAPS-RRT")
{
    specs_.approximateSolutions = true;
    // get the address of the SpaceInformation
    siC_ = si.get();

    MaxSegments_ = MaxSegments;

    NumVs = NumVehicles;
    dim = DimofEachVehicle;

    Planner::declareParam<double>("goal_bias", this, &MAPSRRT::setGoalBias, &MAPSRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &MAPSRRT::setIntermediateStates, &MAPSRRT::getIntermediateStates);
}

ompl::control::MAPSRRT::~MAPSRRT()
{
    freeMemory();
}


void ompl::control::MAPSRRT::setup()
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

void ompl::control::MAPSRRT::clear()
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

void ompl::control::MAPSRRT::freeMemory()
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

// MAPS uses a 2D projection of the state to determine when a vehicle crosses another
// currently done my linear interpolation
// this may need to be a state
// inputs are two states, then, we can work backward for both vehicles
std::vector<bool> ompl::control::MAPSRRT::Project2D(const Motion *motion, int depth)
{

    // if this vector is size >1, we have reached the end
    std::vector<bool> done;
    // this function is going to take in a motion 
    // and linearly interpolate between the current state and parent state
    // for a specified number of times
    // Then, it takes the lines and checks for intersections
    // if there is an intersection, then we return false

    // copy the motion into a local vatiable
    const Motion *CurrMotion = motion;

    // const ob::CompoundStateSpace::StateType *Curr = CurrMotion->state->
    //         as<ob::CompoundStateSpace::StateType>();

    // const auto *CurrxyState1 = Curr->as<ob::RealVectorStateSpace::StateType>(0);
    // const auto *CurrxyState2 = Curr->as<ob::RealVectorStateSpace::StateType>(2);

    // const ob::CompoundStateSpace::StateType *Par = CurrMotion->parent->state->
    //             as<ob::CompoundStateSpace::StateType>();

    // const auto *ParxyState1 = Par->as<ob::RealVectorStateSpace::StateType>(0);
    // const auto *Parrot1 = Par->as<ob::SO2StateSpace::StateType>(1);
    // const auto *ParxyState2 = Par->as<ob::RealVectorStateSpace::StateType>(2);
    // const auto *Parrot2 = Par->as<ob::SO2StateSpace::StateType>(3);

    // // initialize a vector of lines for each vehilces motion
    std::vector<Segment> V1motions;
    std::vector<Segment> V2motions;

    //  for the specified number of times, we are going to create a line from parent to state
    // for each vehicle
    for (int i = 0; i < depth; i++)
    {
        // std::cout << "current depth: " << i << std::endl;
        // get the current state for both vehicles
        // const ob::CompoundStateSpace::StateType *Curr = CurrMotion->state->
            // as<ob::CompoundStateSpace::StateType>();

        // const auto *CurrxyState1 = Curr->as<ob::RealVectorStateSpace::StateType>(0);
        // const auto *CurrxyState2 = Curr->as<ob::RealVectorStateSpace::StateType>(2);
        if (CurrMotion->parent == nullptr)
        {
            // std::cout << "should break out now" << std::endl;
            done.push_back(true);
            break;
        }
        else
        {
            // std::cout << "continuing on" << std::endl;
            // get the parent state for both vehicles
            const ob::CompoundStateSpace::StateType *Par = CurrMotion->parent->state->
                as<ob::CompoundStateSpace::StateType>();

            const auto *ParxyState1 = Par->as<ob::RealVectorStateSpace::StateType>(0);
            const auto *Parrot1 = Par->as<ob::SO2StateSpace::StateType>(1);
            const auto *ParxyState2 = Par->as<ob::RealVectorStateSpace::StateType>(2);
            const auto *Parrot2 = Par->as<ob::SO2StateSpace::StateType>(3);

            TwoKinCarsODE Model(CurrMotion->control);

            std::vector<state_type> x_vec;
            std::vector<double> times;


            state_type p = {ParxyState1->values[0], ParxyState1->values[1], Parrot1->value,
                         ParxyState2->values[0], ParxyState2->values[1], Parrot2->value}; // initial conditions
            
            // point V1parent(ParxyState1->values[0], ParxyState1->values[1]);
            // point V2parent(ParxyState2->values[0], ParxyState2->values[1]);

            integrate( Model , p , 0.0 , (CurrMotion->steps) * (siC_->getPropagationStepSize()), 0.01, push_back_state_and_time(x_vec, times));

            // std::cout << "Vehicle 1 Error" << std::endl;
            // std::cout << parent[0] - CurrxyState1->values[0] << " " << parent[1] - CurrxyState1->values[1] << std::endl;
            // for (int j=0; j < x_vec.size(); j++)
            //     std::cout << times[j] << " " << x_vec[j][0] << " " << x_vec[j][1] << std::endl;
            // std::cout << parent[0] << " " << parent[1] << std::endl;
            // std::cout << x_vec.back()[0] << ' ' << x_vec.back()[1] << std::endl;

            for (int j=0; j < x_vec.size() - 1; j++)
            {
                // get the parent
                point A1(x_vec[j][0], x_vec[j][1]);
                point A2(x_vec[j][3], x_vec[j][4]);
                // get the child
                point B1(x_vec[j + 1][0], x_vec[j + 1][1]);
                point B2(x_vec[j + 1][3], x_vec[j + 1][4]);

                // std::cout << "A1" << std::endl;
                // std::cout << bg::get<0>(A1) << std::endl;
                // std::cout << bg::get<1>(A1) << std::endl;
                // std::cout << "A2" << std::endl;
                // std::cout << bg::get<0>(A2) << std::endl;
                // std::cout << bg::get<1>(A2) << std::endl;
                // std::cout << "B1" << std::endl;
                // std::cout << bg::get<0>(B1) << std::endl;
                // std::cout << bg::get<1>(B1) << std::endl;
                // std::cout << "B2" << std::endl;
                // std::cout << bg::get<0>(B2) << std::endl;
                // std::cout << bg::get<1>(B2) << std::endl;

                Segment V1path(A1, B1);
                Segment V2path(A2, B2);

                V1motions.push_back(V1path);
                V2motions.push_back(V2path);
                // std::cout << "V1" << std::endl;
                // std::cout << bg::get<0, 0>(V1path) << std::endl;  // should be A1[0]
                // std::cout << bg::get<0, 1>(V1path) << std::endl;  // should be A1[1]
                // std::cout << bg::get<1, 0>(V1path) << std::endl;  // should be B1[0]
                // std::cout << bg::get<1, 1>(V1path) << std::endl;  // should be B1[1]
                // std::cout << "V2" << std::endl;
                // std::cout << bg::get<0, 0>(V2path) << std::endl;  // should be A2[0]
                // std::cout << bg::get<0, 1>(V2path) << std::endl;  // should be A2[1]
                // std::cout << bg::get<1, 0>(V2path) << std::endl;  // should be B2[0]
                // std::cout << bg::get<1, 1>(V2path) << std::endl;  // should be B2[1]

            }


            // test integration here

            // TwoKinCarsODE Model(CurrMotion->control);

            // std::vector<state_type> x_vec;
            // std::vector<double> times;



            // state_type parent = {ParxyState1->values[0], ParxyState1->values[1], Parrot1->value,
            //                      ParxyState2->values[0], ParxyState2->values[1], Parrot2->value}; // initial conditions
            // // integrate( Model , parent , 0.0 , (CurrMotion->steps) * (siC_->getPropagationStepSize()), 0.01, push_back_state_and_time( x_vec , times ) );

            // // std::cout << "Vehicle 1 Difference" << std::endl;
            // // std::cout << x_vec[-1][0] - CurrxyState1->values[0] << ' ' << x_vec[-1][1] - CurrxyState1->values[1] << std::endl;
            // // std::cout << "Vehicle 2 Difference" << std::endl;
            // // std::cout << x_vec[-1][3] - ParxyState2->values[0] << ' ' << x_vec[0][4] - ParxyState2->values[1] << std::endl;





            // // turn the states into points
            // // first, the current state points
            // point V1current(CurrxyState1->values[0], CurrxyState1->values[1]);
            // point V2current(CurrxyState2->values[0], CurrxyState2->values[1]);
            // // next, the parent state points
            

            // // four points make two lines
            // Segment V1path(V1current, V1parent);
            // Segment V2path(V2current, V2parent);

            // // append these segments to thier respective vectors
            // V1motions.push_back(V1path);
            // V2motions.push_back(V2path);
            // std::cout << bg::get<0, 0>(V1motions[i]) << std::endl;
            // std::cout << bg::get<0, 1>(V1motions[i]) << std::endl;
            // std::cout << bg::get<1, 0>(V1motions[i]) << std::endl;
            // std::cout << bg::get<1, 1>(V1motions[i]) << std::endl; 
        }
        // std::cout << "updating" << std::endl;
        // move on to the next line segment
        CurrMotion = CurrMotion->parent;
    }
    if (done.size() == 0)
        done.push_back(false);

    // std::cout << "Size of interpolation: " << V1motions.size() << std::endl;
    // std::cout << "Size of interpolation: " << V2motions.size() << std::endl;
    // if (done.size() == 1)
    //     return done;
    // else
    // {
    // now that we have a vector of line segments, we need to check if the segments intersect
    for (int i = 0; i < V1motions.size(); i++)
    {
        for(int j = 0; j < V2motions.size(); j++)
        {
            bool intersect = boost::geometry::intersects(V1motions[i], V2motions[j]);
            if (intersect)
            {
                // std::cout << "V1" << std::endl;
                // std::cout << bg::get<0, 0>(V1motions[i]) << std::endl;  // should be A1[0]
                // std::cout << bg::get<0, 1>(V1motions[i]) << std::endl;  // should be A1[1]
                // std::cout << bg::get<1, 0>(V1motions[i]) << std::endl;  // should be B1[0]
                // std::cout << bg::get<1, 1>(V1motions[i]) << std::endl;  // should be B1[1]
                // std::cout << "V2" << std::endl;
                // std::cout << bg::get<0, 0>(V2motions[j]) << std::endl;  // should be A2[0]
                // std::cout << bg::get<0, 1>(V2motions[j]) << std::endl;  // should be A2[1]
                // std::cout << bg::get<1, 0>(V2motions[j]) << std::endl;  // should be B2[0]
                // std::cout << bg::get<1, 1>(V2motions[j]) << std::endl;  // should be B2[1]
            
                // std::cout << "return true" << std::endl;
                // done.push_back(bg::get<0, 0>(V1motions[i]));
                // done.push_back(bg::get<0, 1>(V1motions[i]));
                // done.push_back(bg::get<1, 0>(V1motions[i]));
                // done.push_back(bg::get<1, 1>(V1motions[i]));
                // done.push_back(true);
                done.push_back(true);
                return done;
            }
        }
    }
    // std::cout << "return false" << std::endl;
    done.push_back(false);
    return done;
    // }
    
        
    // // even though a motion object contains enough information 
    // // to get all this informaiton with one argument, that would only allow 
    // // us to chack the current state and its parent
    // // this planner requires us to check ALL segmentationg at ANY given time
    // // thus, we use two motion objects s.t the states we check, do not need to be parents

    // // get V1 current state
    // const ob::CompoundStateSpace::StateType *CurrV1 = V1motion->state->as<ob::CompoundStateSpace::StateType>();

    // const auto *CurrxyState1 = CurrV1->as<ob::RealVectorStateSpace::StateType>(0);

    // // get V1 parent vehicles state
    // const ob::CompoundStateSpace::StateType *ParV1 = V1motion->parent->state->as<ob::CompoundStateSpace::StateType>();
    
    // const auto *ParxyState1 = ParV1->as<ob::RealVectorStateSpace::StateType>(0);

    // // get V2 current state
    // const ob::CompoundStateSpace::StateType *CurrV2 = V2motion->state->as<ob::CompoundStateSpace::StateType>();
    // const auto *CurrxyState2 = CurrV2->as<ob::RealVectorStateSpace::StateType>(2);

    // // get V2 parent state
    // const ob::CompoundStateSpace::StateType *ParV2 = V2motion->parent->state->as<ob::CompoundStateSpace::StateType>();
    // const auto *ParxyState2 = ParV2->as<ob::RealVectorStateSpace::StateType>(2);

    //  // turn these states into points so that we can use boost

    // point V1current(CurrxyState1->values[0], CurrxyState1->values[1]);
    // point V2current(CurrxyState2->values[0], CurrxyState2->values[1]);

    // point V1parent(ParxyState1->values[0], ParxyState1->values[1]);
    // point V2parent(ParxyState2->values[0], ParxyState2->values[1]);

    // // std::cout << bg::dsv(V1current) << bg::dsv(V1parent) << std::endl;
    // // std::cout << bg::dsv(V2current) << bg::dsv(V2parent) << std::endl;

    // // linearly interpolate from parent to current

    // Segment Vehicle1Path(V1parent, V1current);
    // Segment Vehicle2Path(V2parent, V2current);

    // // see if they intersect

    // bool intersect = boost::geometry::intersects(Vehicle1Path, Vehicle2Path);

    // if(intersect)
    // {
    //     return true;
    // }
    // else
    // {
    //     return false; 
    // }
    
}

void ompl::control::MAPSRRT::FindTotalIntersections(Motion *LastMotion)
{
    // segmentations begin at 1
    int depth = 1;
    int NumSegs = 1;
    bool done = false; 

    // we start from the end of the tree and work our way back
    Motion *Motion = LastMotion;

    // std::cout << "new motion now" << std::endl;

    while (!done)
    {
        
        // std::cout << "depth: " << depth << std::endl;
        // std::cout << LastMotion->parent << std::endl;
        std::vector<bool> intersect = Project2D(Motion, depth);
        // std::cout << "value of intersect function: " << intersect << std::endl;
        // this is true if I found the end of the motion
        if (intersect[0])
        {
            // if i found an intersection
            if (intersect[1])
            {
                for (int i = 0; i < depth - 1; i++)
                {
                    // std::cout << LastMotion << std::endl;
                    if (Motion != nullptr)
                    {
                        Motion->SetCost(NumSegs);
                        Motion = Motion->parent;
                    }
                    else
                    {
                        Motion->SetCost(NumSegs);
                        break;
                    }  
                }
                NumSegs += 1;
            }
            else
            {
                // no intersection, just set the rest of the motion costs to a current one
                while (Motion != nullptr)
                {
                    Motion->SetCost(NumSegs);
                    Motion = Motion->parent;
                }
            }
            done = true;
            // std::cout << "should be exiting now..." << std::endl;
        }
        // not done and intersection
        else if (intersect[1])
        {            
            // std::cout << "found segment" << std::endl;
            // not done but there was an intersection
            // we can go to a depth of its current value minus 1
            
            for (int i = 0; i < depth - 1; i++)
            {

                // std::cout << LastMotion << std::endl;
                if (Motion != nullptr)
                {
                    Motion->SetCost(NumSegs);
                    Motion = Motion->parent;
                }
                else
                {
                    Motion->SetCost(NumSegs);
                    break;
                }  
            }
            NumSegs += 1;
            
            // LastMotion = ResetProjection(LastMotion, depth);
            // const ob::CompoundStateSpace::StateType *Curr = LastMotion->state->
            //     as<ob::CompoundStateSpace::StateType>();
            // const auto *CurrxyState1 = Curr->as<ob::RealVectorStateSpace::StateType>(0);
            // std::cout << "Just reset and added segment. New Starting Point for V1 is..." << std::endl;
            // std::cout << CurrxyState1->values[0] << ' ' << CurrxyState1->values[1] << std::endl;
            depth = 1;

        }
        else
        {
            // not done and no intersection
            Motion->SetCost(NumSegs);
            depth += 1;
        }
        
    }
    // std::cout << "exiting motion" << std::endl;
    // std::cout << "Number of Segments: " << NumSegs << std::endl;
    // return NumSegs;
}

// const ompl::control::MAPSRRT::Motion * ompl::control::MAPSRRT::ResetProjection(const Motion *motion, int depth)
// {
//     const Motion *NewMotion = motion;
//     std::cout << "in this function now" << std::endl;
//     std::cout << NewMotion << std::endl;
//     for (int i = 0; i <= depth; i++)
//     {
//         std::cout << NewMotion << std::endl;
//         if (NewMotion->parent != nullptr)
//         {
//             NewMotion = NewMotion->parent;
//         }
//         else
//         {
//             break;
//         }
        
//     }
    // return NewMotion;
// }

// void CopyCosts(Motion *a, Motion *b)
// {

// }



// main algorithm
ompl::base::PlannerStatus ompl::control::MAPSRRT::solve(const base::PlannerTerminationCondition &ptc)
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

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure.", getName().c_str(), nn_->size());

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
                // this if statement changes the solution ptr if it finds a better solution path
                if (dist < approxdif)
                {
                    // motion->SetCost(cost);
                    approxdif = dist;
                    approxsol = motion;
                    int q = 1;
                    FindTotalIntersections(approxsol);
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

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        // find solution cost
        // solution = FindTotalIntersections(solution);

        // std::cout << "Exited Function" << std::endl;
        /* construct the solution path */
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
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
            {
                //  additional method added by Justin Kottinger
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize(), mpath[i]->GetCost());
            }
            else
            {
                path->append(mpath[i]->state, mpath[i]->GetCost());
            }
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        std::ofstream PathFile;
        PathFile.open("txt/path.txt");
        if (PathFile.fail())
        {
          std::cerr << "ERROR: Could not open path.txt" << std::endl;
          exit(1);
        }
        else
        {
          std::cout << "Writing solution to path.txt" << std::endl;
          // PathFile << data << std::endl;
          path->printAsMatrix(PathFile);
          PathFile.close();
          std::cout << "Computation completed successfully" << std::endl;
          // path.print(std::cout);  // this prints out the solution
        }
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());
    // To see the error from exact sol to sol found
    OMPL_INFORM("%s: Solution Error in compound state space was %f", getName().c_str(), approxdif);
    return base::PlannerStatus(solved, approximate);
}

void ompl::control::MAPSRRT::getPlannerData(base::PlannerData &data) const
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