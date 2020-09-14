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

// #ifndef OMPL_CONTROL_PLANNERS_RRT_RRT_
// #define OMPL_CONTROL_PLANNERS_RRT_RRT_

// #include "ompl/control/planners/rrt/RRT.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <boost/geometry.hpp>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace bg = boost::geometry;

typedef boost::array< double , 6 > state_type;
typedef bg::model::point<double, 2, bg::cs::cartesian> Point;
typedef boost::geometry::model::segment<Point> Segment;
typedef bg::model::polygon<Point> polygon;


namespace ompl
{
    namespace control
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
           S.M. LaValle and J.J. Kuffner, Randomized kinodynamic planning, <em>Intl. J. of Robotics Research</em>, vol.
           20, pp. 378–400, May 2001. DOI: [10.1177/02783640122067453](http://dx.doi.org/10.1177/02783640122067453)<br>
           [[PDF]](http://ijr.sagepub.com/content/20/5/378.full.pdf)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/rrtpubs.html)
        */

        /** \brief Rapidly-exploring Random Tree */
        class AO_X : public base::Planner
        {
        public:
            /** \brief Constructor */
            AO_X(const ompl::control::SpaceInformationPtr &si, int NumVehicles, int NumControls,
                int DimofEachVehicle, int MaxSegments, std::vector<double> goal, double radius, 
                bool benchmark, std::string model, unsigned int k = 1);

            ~AO_X() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            void clear() override;

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
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                // In MAPS, each motion has a cost
                // the cost is the number of segments that are required 
                // to explain the path with this motion

                void SetCost(int a)
                {
                    cost = a;
                }

                int GetCost() const
                {
                    return cost;
                }

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The control contained by the motion */
                Control *control{nullptr};

                /** \brief The number of steps the control is applied for */
                unsigned int steps{0};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                int cost{1};

                int NumIntersections{0};

                std::vector<std::vector<Point>> LinearPath;

                std::vector<double> AllVehicleDistance;

                std::vector<Motion *> LocationsOfIntersect;

            private:

                
            };
            int MultiAgentControlSampler(Motion *motion, Control *RandCtrl, Control *previous, 
                const base::State *source, base::State *dest);

            std::vector<double> getDistance(const base::State *st);

            std::vector<double> ThreeUnicycleDistance(const ob::State *st);

            std::vector<double> ThreeLinearDistance(const base::State *st);

            std::vector<double> TwoLinearDistance(const base::State *st);

            std::vector<double> TwoKinDistance(const base::State *st);

            unsigned int propagateWhileValid(Motion *motion, const base::State *state, Control *control,
                int steps, base::State *result, std::vector<int> DoNotProgogate);

            void overrideStates(const std::vector<int> NoPropogation, const base::State *s, 
                base::State *r, Control *control);
            
            void OverrideKinCars(const std::vector<int> NoPropogation, const base::State *s, 
                base::State *r, Control *control);

            void OverrideLinCars(const std::vector<int> NoPropogation, const base::State *s, 
                base::State *r, Control *control);

            void OverrideUniCars(const std::vector<int> DoNotProp, const base::State *source, 
                base::State *result, Control *control);

            std::vector<Point> MakeLinearPath(const base::State *result) const;

            std::vector<Point> MakeUniPath(const base::State *result) const;

            std::vector<Point> MakeKinPath(const base::State *result) const;
            
            std::vector<Point> MakeLinPath(const base::State *result) const;


            void FindTotalIntersections(Motion *motion);
            // bool
            int Project2D(Motion *motion);

            int Project2D_3Vehicles(Motion *motion);

            int Project2D_2Vehicles(Motion *motion);

            void Get2DimDistance(Motion *motion, const base::State *source, 
                const base::State *result);

            void Get2DimDist2KinCars(Motion *motion, const base::State *source, 
                const base::State *result);

            void Get2DimDist2LinCars(Motion *motion, const base::State *source, 
                const base::State *result);

            unsigned int FindTotalPathCost(Motion *motion);

            std::vector<bool> CheckSegmentation(Motion *motion, int d, bool end);

            std::vector<bool> CheckSegmentationTest(Motion *motion, int depth);

            // std::vector<Motion> GenerateMotionList(Motion *motion);

            // std::vector<std::vector<double>> GeneratePathLengths(std::vector<Motion> CurrPath);

            // int FindTotalSegments(const Motion *m);

            // int FindTotalPathCost(const Motion *m);

            // const Motion * ResetProjection(const Motion *motion, int d);


            // std::vector<bool>  Project2D(const Motion *a, int d);

            // void FindTotalIntersections(Motion *motion);

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            // const SpaceInformationPtr si_;

            // const SpaceInformationPtr Csi_;
            std::string model_;

            // used for projections in the obs checking
            int NumVs;

            // used for indexing controls
            int NumCs;

            // used for projections in the obs checking
            int dim;

            // The number of disjoint segments allowed in the solution
            int MaxSegments_;

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            // const SpaceInformationPtr Csi_{&siC_};

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_{false};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            // the final cost
            int FinalCost_{0};

            // the goal vector
            std::vector<double> g;

            // the goal radius
            double radius_;

            // number of samples
            unsigned int numControlSamples_;

            double time_;

            bool benchmark_;

            std::string FinalCostProperty() const
            {
                return std::to_string(FinalCost_);
            }
        };
    }
}

// #endif