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

#ifndef OMPL_CONTROL_PLANNERS_MAPSEST_MAPSEST_
#define OMPL_CONTROL_PLANNERS_MAPSEST_MAPSEST_

#include "ompl/datastructures/Grid.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/datastructures/PDF.h"
#include <unordered_map>
#include <vector>
// my includes
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>


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
           @anchor cMAPSEST
           @par Short description
           MAPSEST is a tree-based motion planner that attempts to detect
           the less explored area of the space through the use of a
           grid imposed on a projection of the state space. Using this
           information, MAPSEST continues tree expansion primarily from
           less explored areas.  It is important to set the projection
           the algorithm uses (setProjectionEvaluator() function). If
           no projection is set, the planner will attempt to use the
           default projection associated to the state space. An
           exception is thrown if no default projection is available
           either.
           @par External documentation
           D. Hsu, J.-C. Latombe, and R. Motwani, Path planning in expansive configuration spaces,
           <em>Intl. J. Computational Geometry and Applications</em>,
           vol. 9, no. 4-5, pp. 495–512, 1999. DOI:
           [10.1142/S0218195999000285](http://dx.doi.org/10.1142/S0218195999000285)<br>
           [[PDF]](http://bigbird.comp.nus.edu.sg/pmwiki/farm/motion/uploads/Site/ijcga96.pdf)
        */

        /** \brief Expansive Space Trees */
        class MAPSEST : public base::Planner
        {
        public:
            /** \brief Constructor */
            MAPSEST(const SpaceInformationPtr &si, int NumVehicles, int NumControls, 
                int DimofEachVehicle, int MaxSegments, std::vector<double> goal, 
                double radius, bool benchmark, std::string model, unsigned int k = 1, 
                std::string solutionName = "txt/path.txt");

            ~MAPSEST() override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set the projection evaluator. This class is
                able to compute the projection of a given state.  */
            void setProjectionEvaluator(const base::ProjectionEvaluatorPtr &projectionEvaluator)
            {
                projectionEvaluator_ = projectionEvaluator;
            }

            /** \brief Set the projection evaluator (select one from
                the ones registered with the state space). */
            void setProjectionEvaluator(const std::string &name)
            {
                projectionEvaluator_ = si_->getStateSpace()->getProjection(name);
            }

            /** \brief Get the projection evaluator */
            const base::ProjectionEvaluatorPtr &getProjectionEvaluator() const
            {
                return projectionEvaluator_;
            }

            void setup() override;

            void getPlannerData(base::PlannerData &data) const override;

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
            };

            struct MotionInfo;

            /** \brief A grid cell */
            typedef Grid<MotionInfo>::Cell GridCell;

            /** \brief A PDF of grid cells */
            typedef PDF<GridCell *> CellPDF;

            /** \brief A struct containing an array of motions and a corresponding PDF element */
            struct MotionInfo
            {
                Motion *operator[](unsigned int i)
                {
                    return motions_[i];
                }
                const Motion *operator[](unsigned int i) const
                {
                    return motions_[i];
                }
                void push_back(Motion *m)
                {
                    motions_.push_back(m);
                }
                unsigned int size() const
                {
                    return motions_.size();
                }
                bool empty() const
                {
                    return motions_.empty();
                }
                std::vector<Motion *> motions_;
                CellPDF::Element *elem_;
            };

            /** \brief The data contained by a tree of exploration */
            struct TreeData
            {
                TreeData() = default;

                /** \brief A grid where each cell contains an array of motions */
                Grid<MotionInfo> grid{0};

                /** \brief The total number of motions in the grid */
                unsigned int size{0};
            };

            std::vector<double> getDistance(const base::State *st) const;

            std::vector<double> Two2ndOrderCarDistance(const ob::State *st) const;

            std::vector<double> Two2ndOrderLinearDistance(const ob::State *st) const;

            std::vector<double> Two2ndOrderUnicycleDistance(const ob::State *st) const;

            std::vector<double> TwoUnicycleDistance(const ob::State *st) const;

            std::vector<double> ThreeUnicycleDistance(const ob::State *st) const;

            std::vector<double> ThreeLinearDistance(const base::State *st) const;

            std::vector<double> TwoLinearDistance(const base::State *st) const;

            std::vector<double> TwoKinDistance(const base::State *st) const;

            std::vector<double> ThreeKinDistance(const base::State *st) const;

            int MultiAgentControlSampler(Motion *motion, Control *RandCtrl, Control *previous, 
                const base::State *source, base::State *dest);

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

            void Override2ndOrderUniCars(const std::vector<int> DoNotProp, const base::State *source, 
                base::State *result, Control *control);

            void Override2ndOrderLinCars(const std::vector<int> DoNotProp, const base::State *source, 
                base::State *result, Control *control);

            void Override2ndOrderKinCars(const std::vector<int> DoNotProp, const base::State *source, 
                base::State *result, Control *control);

            std::vector<Point> MakeLinearPath(const base::State *result) const;

            std::vector<Point> MakeUniPath(const base::State *result) const;

            std::vector<Point> MakeKinPath(const base::State *result) const;
            
            std::vector<Point> MakeLinPath(const base::State *result) const;

            std::vector<Point> Make2ndOrderUniPath(const base::State *result) const;

            void Get2DimDistance(Motion *motion, const base::State *source, 
                const base::State *result);

            void Get2DimDist2KinCars(Motion *motion, const base::State *source, 
                const base::State *result);

            void Get2DimDist2LinCars(Motion *motion, const base::State *source, 
                const base::State *result);

            void Get2DimDist2ndOrderUni(Motion *motion, const base::State *source, 
                const base::State *result);

            void FindTotalIntersections(Motion *motion);
            
            int Project2D(Motion *motion);

            int Project2D_3Vehicles(Motion *motion);

            int Project2D_2Vehicles(Motion *motion);

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Add a motion to the exploration tree */
            void addMotion(Motion *motion);

            /** \brief Select a motion to continue the expansion of the tree from */
            Motion *selectMotion();

            /** \brief Valid state sampler */
            base::ValidStateSamplerPtr sampler_;

            /** \brief Directed control sampler */
            DirectedControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief The exploration tree constructed by this algorithm */
            TreeData tree_;

            /** \brief This algorithm uses a discretization (a grid) to guide the exploration. The exploration is
             * imposed on a projection of the state space. */
            base::ProjectionEvaluatorPtr projectionEvaluator_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The PDF used for selecting a cell from which to sample a motion */
            CellPDF pdf_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

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

            // the final cost
            int FinalCost_{0};

            // the goal vector
            std::vector<double> g;

            // the goal radius
            double radius_;

            // text file name
            std::string SolName_;

            // number of samples
            unsigned int numControlSamples_;

            double time_{0};

            bool benchmark_;

            std::string FinalTimeProperty() const
            {
                return std::to_string(time_);
            }

            std::string FinalCostProperty() const
            {
                return std::to_string(FinalCost_);
            }
        };
    }
}

#endif
