// ompl includes
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateProjections.h>
#include <ompl/base/Goal.h>
#include <ompl/base/ProjectionEvaluator.h>
#include "ompl/tools/benchmark/Benchmark.h"
// std includes
#include <iostream>
#include <valarray>
#include <limits>
#include <math.h>
#include <typeinfo>
#include <string>
#include <Eigen/Core>
// my includes
#include "../includes/ReadWorld.h"
#include "../includes/CreateSimpleSetup.h"
#include "../includes/KinematicCar.h"
#include "../includes/LinearCar.h"
#include "../includes/Unicycle.h"
#include "../includes/MyPlanner.h"
#include "../includes/MyPlannerMotion.h"
#include "../includes/MyPlannerCost.h"
#include "../includes/Lazy_MAPS_RRT_Cost.h"
#include "../includes/MAPS_SST.h"
#include "../includes/MAPS_EST.h"
#include "../includes/MAPSRRTPathControl.h"
#include "../includes/RRTplus.h"
#include "../includes/MyRRT.h"
#include <bits/stdc++.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace bg = boost::geometry;

// Local paths to examples
// txt/world_mlExample.txt
// txt/Shaull/3agents/2ndOrderLinear/example3_3_2ndOrderLinear.txt

class KinematicCarStateValidityChecker : public ob::StateValidityChecker
{
public:
    KinematicCarStateValidityChecker(const ob::SpaceInformationPtr &si, 
      std::vector<double> obs) : ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {
        if (!space_->satisfiesBounds(state))
        {
          return false;
        }

        const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

        const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

        double x = pos->values[0];
        double y = pos->values[1];
            
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          if (x>(obstacle[i] - buffer_) && x<(obstacle[i] + obstacle[i+3] + buffer_) && 
            y>(obstacle[i+1] - buffer_) && y<(obstacle[i+1]+obstacle[i+4] + buffer_))
          {
            return false;
          }
                
        }    
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.2;
};


class mlExampleStateValidityChecker : public ob::StateValidityChecker
{
public:
    mlExampleStateValidityChecker(const ob::SpaceInformationPtr &si, 
      std::vector<double> obs) : ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {
        if (!space_->satisfiesBounds(state))
        {
          return false;
        }

        auto cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();

        auto xyzState = cs_->as<ob::RealVectorStateSpace::StateType>(0);
        
        const double x = xyzState->values[0];
        const double y = xyzState->values[1];
        const double z = xyzState->values[2];

        // double x = pos->values[0];
        // double y = pos->values[1];
            
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          if (x>(obstacle[i] - buffer_) && x<(obstacle[i] + 1 + buffer_) && 
            y>(obstacle[i+1] - buffer_) && y<(obstacle[i+1] + 1 + buffer_) && 
            z>(obstacle[i+2] - buffer_) && z<(obstacle[i+2] + 1 + buffer_))
          {
            return false;
          }       
        }    
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.2;
};


// used for 3 agent planning for kinematic car
class ThreeKinematicCarStateValidityChecker : public ob::StateValidityChecker
{
public:
    ThreeKinematicCarStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {

        if (!space_->satisfiesBounds(state))
        {
          return false;
        }


        ThreeKinematicCarsModel vehicles = ThreeKinematicCarsModel(state);
        std::vector<polygon> vs = vehicles.GetPolygons();

        auto cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();

        // if that tests passes, check colliding with objects
        // first, test for vehicles colliding with each other
        for (int v1 = 0; v1 < vs.size(); v1++)
        {
          for (int v2 = 0; v2 < vs.size(); v2++)
          {
            // make sure we are not checking the same vehicle -- this would return false
            if (v1 != v2)
            {
              bool test = boost::geometry::disjoint(vs[v1], vs[v2]);
              if (!test)
                return false;
            }
          }
        }

        for (int v = 0; v < vs.size(); v++)
        {
          for (int i = 0; i < obstacle.size(); i=i+6)
          {
            auto xyState = cs_->as<ob::RealVectorStateSpace::StateType>(2 * v);
            double x = xyState->values[0];
            double y = xyState->values[1];

            // std::cout << obstacle[i] << obstacle[i + 1] << obstacle[i+3] << obstacle[i+4] << std::endl;

            if ((x>(obstacle[i] - buffer_)) && (x<(obstacle[i] + obstacle[i+3] + buffer_)))
            {
              if ((y>(obstacle[i+1] - buffer_)) && (y<(obstacle[i+1]+obstacle[i+4] + buffer_)))
              {
                return false;
              }
            }           
          }
        }
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.2;
};

// used for 2 agent planning for kinematic car
class TwoKinematicCarStateValidityChecker : public ob::StateValidityChecker
{
public:
    TwoKinematicCarStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {

        if (!space_->satisfiesBounds(state))
        {
          return false;
        }


        TwoKinematicCarsModel vehicles = TwoKinematicCarsModel(state);
        std::vector<polygon> vs = vehicles.GetPolygons();

        auto cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();

        // first, test for vehicles colliding with each other
        bool test = boost::geometry::disjoint(vs[0], vs[1]);
        if (!test)
        {
          return false;
        }
        
        for (int v = 0; v < vs.size(); v++)
        {
          for (int i = 0; i < obstacle.size(); i=i+6)
          {
            auto xyState = cs_->as<ob::RealVectorStateSpace::StateType>(2 * v);
            double x = xyState->values[0];
            double y = xyState->values[1];

            // std::cout << obstacle[i] << obstacle[i + 1] << obstacle[i+3] << obstacle[i+4] << std::endl;

            if ((x>(obstacle[i] - buffer_)) && (x<(obstacle[i] + obstacle[i+3] + buffer_)))
            {
              if ((y>(obstacle[i+1] - buffer_)) && (y<(obstacle[i+1]+obstacle[i+4] + buffer_)))
              {
                // if (i == 0)
                //   std::cout << "properly found obs." << std::endl;
                return false;
              }
            }           
          }
        }
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.2;
};

// used for 2 agent planning for linear dyn
class TwoLinearCarStateValidityChecker : public ob::StateValidityChecker
{
public:
    TwoLinearCarStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {
        


        if (!space_->satisfiesBounds(state))
        {
          return false;
        }


        TwoLinearCars vehicles = TwoLinearCars(state);
        std::vector<polygon> vs = vehicles.GetPolygons();

        // first, test for vehicles colliding with each other
        bool test = boost::geometry::disjoint(vs[0], vs[1]);
        if (!test)
        {
          	return false;
        }

        // if that tests passes, check colliding with objects


        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          // // create obs object
          point BotR( obstacle[i], obstacle[i+1]);
          point TopR(obstacle[i], obstacle[i+1] + obstacle[i + 4]);
          point BotL(obstacle[i] + obstacle[i + 3], obstacle[i+1]);
          point TopL(obstacle[i] + obstacle[i + 3], obstacle[i+1] + obstacle[i + 4]);

          // create instance of polygon
          polygon obst;
          // // add the outer points to the shape
          obst.outer().push_back(BotR);
          obst.outer().push_back(TopR);
          obst.outer().push_back(TopL);
          obst.outer().push_back(BotL);
          obst.outer().push_back(BotR);

          // for all vehicles, see if they collide with obs
          for (int v = 0; v < vs.size(); v++)
          {
            bool disjoint = boost::geometry::disjoint(vs[v], obst);

            if (!disjoint)
            {
              return false;
            }
          }
        }  
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.2;
};


// used for 2 agent planning for dynamical car
class Two2ndOrderCarStateValidityChecker : public ob::StateValidityChecker
{
public:
    Two2ndOrderCarStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {

        if (!space_->satisfiesBounds(state))
        {
          return false;
        }


        TwoKinematicCarsModel vehicles = TwoKinematicCarsModel(state);
        std::vector<polygon> vs = vehicles.GetPolygons();

        auto cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();

        // // first, test for vehicles colliding with each other
        // bool test = boost::geometry::disjoint(vs[0], vs[1]);
        // if (!test)
        // {
        //   return false;
        // }
        
        // for (int v = 0; v < vs.size(); v++)
        // {
        //   for (int i = 0; i < obstacle.size(); i=i+6)
        //   {
        //     auto xyState = cs_->as<ob::RealVectorStateSpace::StateType>(2 * v);
        //     double x = xyState->values[0];
        //     double y = xyState->values[1];

        //     // std::cout << obstacle[i] << obstacle[i + 1] << obstacle[i+3] << obstacle[i+4] << std::endl;

        //     if ((x>(obstacle[i] - buffer_)) && (x<(obstacle[i] + obstacle[i+3] + buffer_)))
        //     {
        //       if ((y>(obstacle[i+1] - buffer_)) && (y<(obstacle[i+1]+obstacle[i+4] + buffer_)))
        //       {
        //         // if (i == 0)
        //         //   std::cout << "properly found obs." << std::endl;
        //         return false;
        //       }
        //     }           
        //   }
        // }
        // if that test passes, check colliding with objects
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          // // create obs object
          point BotR( obstacle[i], obstacle[i+1]);
          point TopR(obstacle[i], obstacle[i+1] + obstacle[i + 4]);
          point BotL(obstacle[i] + obstacle[i + 3], obstacle[i+1]);
          point TopL(obstacle[i] + obstacle[i + 3], obstacle[i+1] + obstacle[i + 4]);

          // create instance of polygon
          polygon obst;
          // // add the outer points to the shape
          obst.outer().push_back(BotR);
          obst.outer().push_back(TopR);
          obst.outer().push_back(TopL);
          obst.outer().push_back(BotL);
          obst.outer().push_back(BotR);

          // for all vehicles, see if they collide with obs
          for (int v = 0; v < vs.size(); v++)
          {
            bool disjoint = boost::geometry::disjoint(vs[v], obst);

            if (!disjoint)
            {
              return false;
            }
          }
        }
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.2;
};

// used for 2 agent planning for linear dyn
class Two2ndOrderLinearCarStateValidityChecker : public ob::StateValidityChecker
{
public:
    Two2ndOrderLinearCarStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {
        if (!space_->satisfiesBounds(state))
        {
          return false;
        }


        TwoLinearCars vehicles = TwoLinearCars(state);
        std::vector<polygon> vs = vehicles.GetPolygons();

        // first, test for vehicles colliding with each other
        bool test = boost::geometry::disjoint(vs[0], vs[1]);
        if (!test)
        {
            return false;
        }

        // if that test passes, check colliding with objects
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          // // create obs object
          point BotR( obstacle[i], obstacle[i+1]);
          point TopR(obstacle[i], obstacle[i+1] + obstacle[i + 4]);
          point BotL(obstacle[i] + obstacle[i + 3], obstacle[i+1]);
          point TopL(obstacle[i] + obstacle[i + 3], obstacle[i+1] + obstacle[i + 4]);

          // create instance of polygon
          polygon obst;
          // // add the outer points to the shape
          obst.outer().push_back(BotR);
          obst.outer().push_back(TopR);
          obst.outer().push_back(TopL);
          obst.outer().push_back(BotL);
          obst.outer().push_back(BotR);

          // for all vehicles, see if they collide with obs
          for (int v = 0; v < vs.size(); v++)
          {
            bool disjoint = boost::geometry::disjoint(vs[v], obst);

            if (!disjoint)
            {
              return false;
            }
          }
        }  
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.2;
};

// used for 3 agent planning for linear dyn
class Three2ndOrderLinearCarStateValidityChecker : public ob::StateValidityChecker
{
public:
    Three2ndOrderLinearCarStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {
        if (!space_->satisfiesBounds(state))
        {
          return false;
        }


        ThreeLinearCars vehicles = ThreeLinearCars(state);
        std::vector<polygon> vs = vehicles.GetPolygons();

        // first, test for vehicles colliding with each other
        for (int v1 = 0; v1 < vs.size(); v1++)
        {
          for (int v2 = 0; v2 < vs.size(); v2++)
          {
            // make sure we are not checking the same vehicle -- this would return false
            if (v1 != v2)
            {
              bool test = boost::geometry::disjoint(vs[v1], vs[v2]);
              if (!test)
              {
                return false;
              }
            }
          }
        }

        // if that test passes, check colliding with objects
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          // // create obs object
          point BotR( obstacle[i], obstacle[i+1]);
          point TopR(obstacle[i], obstacle[i+1] + obstacle[i + 4]);
          point BotL(obstacle[i] + obstacle[i + 3], obstacle[i+1]);
          point TopL(obstacle[i] + obstacle[i + 3], obstacle[i+1] + obstacle[i + 4]);

          // create instance of polygon
          polygon obst;
          // // add the outer points to the shape
          obst.outer().push_back(BotR);
          obst.outer().push_back(TopR);
          obst.outer().push_back(TopL);
          obst.outer().push_back(BotL);
          obst.outer().push_back(BotR);

          // for all vehicles, see if they collide with obs
          for (int v = 0; v < vs.size(); v++)
          {
            bool disjoint = boost::geometry::disjoint(vs[v], obst);

            if (!disjoint)
            {
              return false;
            }
          }
        }  
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.2;
};


class ThreeLinearCarStateValidityChecker : public ob::StateValidityChecker
{
public:
    ThreeLinearCarStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {
        
        if (!space_->satisfiesBounds(state))
        {
          return false;
        }


        ThreeLinearCars vehicles = ThreeLinearCars(state);
        std::vector<polygon> vs = vehicles.GetPolygons();

        // first, test for vehicles colliding with each other
        for (int v1 = 0; v1 < vs.size(); v1++)
        {
          for (int v2 = 0; v2 < vs.size(); v2++)
          {
            // make sure we are not checking the same vehicle -- this would return false
            if (v1 != v2)
            {
              bool test = boost::geometry::disjoint(vs[v1], vs[v2]);
              if (!test)
              {
                return false;
              }
            }
          }
        }
        // if that tests passes, check colliding with objects
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          // // create obs object
          point BotR( obstacle[i], obstacle[i+1]);
          point TopR(obstacle[i], obstacle[i+1] + obstacle[i + 4]);
          point BotL(obstacle[i] + obstacle[i + 3], obstacle[i+1]);
          point TopL(obstacle[i] + obstacle[i + 3], obstacle[i+1] + obstacle[i + 4]);

          // create instance of polygon
          polygon obst;
          // // add the outer points to the shape
          obst.outer().push_back(BotR);
          obst.outer().push_back(TopR);
          obst.outer().push_back(TopL);
          obst.outer().push_back(BotL);
          obst.outer().push_back(BotR);

          // for all vehicles, see if they collide with obs
          for (int v = 0; v < vs.size(); v++)
          {
            bool disjoint = boost::geometry::disjoint(vs[v], obst);

            if (!disjoint)
            {
              return false;
            }
          }
        }  
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.02;
};


class ThreeUnicycleStateValidityChecker : public ob::StateValidityChecker
{
public:
    ThreeUnicycleStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {
        if (!space_->satisfiesBounds(state))
        {
          return false;
        }

        auto cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();
        ThreeUnicycleModels vehicles = ThreeUnicycleModels(state);
        std::vector<polygon> vs = vehicles.GetPolygons();

        // first, test for vehicles colliding with each other
        for (int v1 = 0; v1 < vs.size(); v1++)
        {
          for (int v2 = 0; v2 < vs.size(); v2++)
          {
            // make sure we are not checking the same vehicle -- this would return false
            if (v1 != v2)
            {
              bool test = boost::geometry::disjoint(vs[v1], vs[v2]);
              if (!test)
                return false;
            }
          }
        }

        // if that tests passes, check colliding with objects
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          // // create obs object
          point BotR( obstacle[i], obstacle[i+1]);
          point TopR(obstacle[i], obstacle[i+1] + obstacle[i + 4]);
          point BotL(obstacle[i] + obstacle[i + 3], obstacle[i+1]);
          point TopL(obstacle[i] + obstacle[i + 3], obstacle[i+1] + obstacle[i + 4]);

          // create instance of polygon
          polygon obst;
          // // add the outer points to the shape
          obst.outer().push_back(BotR);
          obst.outer().push_back(TopR);
          obst.outer().push_back(TopL);
          obst.outer().push_back(BotL);
          obst.outer().push_back(BotR);

          // for all vehicles, see if they collide with obs
          for (int v = 0; v < vs.size(); v++)
          {
            bool disjoint = boost::geometry::disjoint(vs[v], obst);

            if (!disjoint)
            {
              return false;
            }
          }
        } 

        // for (int v = 0; v < vs.size(); v++)
        // {
        //   for (int i = 0; i < obstacle.size(); i=i+6)
        //   {
        //     auto xyState = cs_->as<ob::RealVectorStateSpace::StateType>(2 * v);
        //     double x = xyState->values[0];
        //     double y = xyState->values[1];

        //     // std::cout << obstacle[i] << obstacle[i + 1] << obstacle[i+3] << obstacle[i+4] << std::endl;

        //     if ((x>(obstacle[i] - buffer_)) && (x<(obstacle[i] + obstacle[i+3] + buffer_)))
        //     {
        //       if ((y>(obstacle[i+1] - buffer_)) && (y<(obstacle[i+1]+obstacle[i+4] + buffer_)))
        //       {
        //         return false;
        //       }
        //     }           
        //   }
        // } 
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.1;
};


class TwoUnicycleStateValidityChecker : public ob::StateValidityChecker
{
public:
    TwoUnicycleStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        // const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {
        if (!space_->satisfiesBounds(state))
        {
          return false;
        }

        auto cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();
        TwoUnicycleModels vehicles = TwoUnicycleModels(state);
        std::vector<polygon> vs = vehicles.GetPolygons();

        // first, test for vehicles colliding with each other
        bool test = boost::geometry::disjoint(vs[0], vs[1]);
        if (!test)
        {
            return false;
        }

        // if that tests passes, check colliding with objects
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          // // create obs object
          point BotR( obstacle[i], obstacle[i+1]);
          point TopR(obstacle[i], obstacle[i+1] + obstacle[i + 4]);
          point BotL(obstacle[i] + obstacle[i + 3], obstacle[i+1]);
          point TopL(obstacle[i] + obstacle[i + 3], obstacle[i+1] + obstacle[i + 4]);

          // create instance of polygon
          polygon obst;
          // // add the outer points to the shape
          obst.outer().push_back(BotR);
          obst.outer().push_back(TopR);
          obst.outer().push_back(TopL);
          obst.outer().push_back(BotL);
          obst.outer().push_back(BotR);

          // for all vehicles, see if they collide with obs
          for (int v = 0; v < vs.size(); v++)
          {
            bool disjoint = boost::geometry::disjoint(vs[v], obst);

            if (!disjoint)
            {
              return false;
            }
          }
        } 

        // for (int v = 0; v < vs.size(); v++)
        // {
        //   for (int i = 0; i < obstacle.size(); i=i+6)
        //   {
        //     auto xyState = cs_->as<ob::RealVectorStateSpace::StateType>(2 * v);
        //     double x = xyState->values[0];
        //     double y = xyState->values[1];

        //     // std::cout << obstacle[i] << obstacle[i + 1] << obstacle[i+3] << obstacle[i+4] << std::endl;

        //     if ((x>(obstacle[i] - buffer_)) && (x<(obstacle[i] + obstacle[i+3] + buffer_)))
        //     {
        //       if ((y>(obstacle[i+1] - buffer_)) && (y<(obstacle[i+1]+obstacle[i+4] + buffer_)))
        //       {
        //         return false;
        //       }
        //     }           
        //   }
        // } 
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.1;
};


class Two2ndOrderUnicycleStateValidityChecker : public ob::StateValidityChecker
{
public:
    Two2ndOrderUnicycleStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
       ob::StateValidityChecker(si)
    {
        space_ = si->getStateSpace();
        obstacle = obs;
        const double toll_ = 0.2;
    }

    virtual bool isValid(const ob::State *state) const
    {
        if (!space_->satisfiesBounds(state))
        {
          return false;
        }

        auto cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();
        Two2ndOrderUnicycleModels vehicles = Two2ndOrderUnicycleModels(state);
        std::vector<polygon> vs = vehicles.GetPolygons();

        // first, test for vehicles colliding with each other
        bool test = boost::geometry::disjoint(vs[0], vs[1]);
        if (!test)
        {
            return false;
        }

        // if that tests passes, check colliding with objects
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          // // create obs object
          point BotR( obstacle[i], obstacle[i+1]);
          point TopR(obstacle[i], obstacle[i+1] + obstacle[i + 4]);
          point BotL(obstacle[i] + obstacle[i + 3], obstacle[i+1]);
          point TopL(obstacle[i] + obstacle[i + 3], obstacle[i+1] + obstacle[i + 4]);

          // create instance of polygon
          polygon obst;
          // // add the outer points to the shape
          obst.outer().push_back(BotR);
          obst.outer().push_back(TopR);
          obst.outer().push_back(TopL);
          obst.outer().push_back(BotL);
          obst.outer().push_back(BotR);

          // for all vehicles, see if they collide with obs
          for (int v = 0; v < vs.size(); v++)
          {
            bool disjoint = boost::geometry::disjoint(vs[v], obst);

            if (!disjoint)
            {
              return false;
            }
          }
        } 

        // first, test for vehicles colliding with each other
        // for (int v1 = 0; v1 < vs.size(); v1++)
        // {
        //   for (int v2 = 0; v2 < vs.size(); v2++)
        //   {
        //     // make sure we are not checking the same vehicle -- this would return false
        //     if (v1 != v2)
        //     {
        //       bool test = boost::geometry::disjoint(vs[v1], vs[v2]);
        //       if (!test)
        //         return false;
        //     }
        //   }
        // }

        // for (int v = 0; v < vs.size(); v++)
        // {
        //   for (int i = 0; i < obstacle.size(); i=i+6)
        //   {
        //     auto xyState = cs_->as<ob::RealVectorStateSpace::StateType>(2 * v);
        //     double x = xyState->values[0];
        //     double y = xyState->values[1];

        //     // std::cout << obstacle[i] << obstacle[i + 1] << obstacle[i+3] << obstacle[i+4] << std::endl;

        //     if ((x>(obstacle[i] - buffer_)) && (x<(obstacle[i] + obstacle[i+3] + buffer_)))
        //     {
        //       if ((y>(obstacle[i+1] - buffer_)) && (y<(obstacle[i+1]+obstacle[i+4] + buffer_)))
        //       {
        //         return false;
        //       }
        //     }           
        //   }
        // } 
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.1;
};


class MyArbitraryGoal : public ompl::base::Goal
{
  public:
    MyArbitraryGoal(const ob::SpaceInformationPtr &si, std::vector<double> _goal, const double _toll, 
      std::string _model) : ompl::base::Goal(si)
    {
      goal = _goal;  // not vehicle state specific
      toll = _toll;
      model = _model;
    }
    std::vector<double> goal;
    double toll;
    std::string model;

    virtual bool isSatisfied(const ob::State *st) const
    {
      std::vector<double> d = getDistance(st);

      // double distance = d[0] + d[1];
      // if (d[2] <= toll)
      //   return true;
      // else
      //   return false;
      // std::cout << toll << std::endl;
      for (int v=0; v < d.size(); v++)
      {
        if (d[v] > toll)
          return false;
      }
      return true;
    }

    virtual bool isSatisfied(const ob::State *st, double *distance) const
    {
        bool result = isSatisfied(st);
        std::vector<double> d = getDistance(st);
        double dist = calcMagnitude(d);
        if (distance != NULL)
        {
            if (result == false)
              {
                
                *distance = dist;
              }
            else  // we found goal 
            {
              *distance = dist;
            }
        }
        return result;
    }

    virtual double calcMagnitude(std::vector<double> distances) const 
    {
      double dist = 0.0;

      for (int v=0; v < distances.size(); v++)
      {
        dist += pow(distances[v], 2);
      }
      return sqrt(dist);
    }

    virtual std::vector<double> getDistance(const ob::State *st) const
    {
      std::vector<double> distance;
      if (model == "KinematicCar")
        distance = KinCarDistance(st);
      else if (model == "mlExample")
      	distance = mlExampleDistance(st);
      else if (model == "2KinematicCars")
        distance = TwoKinDistance(st);
      else if (model == "2Linear")
        distance = TwoLinearDistance(st);
      else if (model == "3Linear")
        distance = ThreeLinearDistance(st);
      else if (model == "3Unicycle")
        distance = ThreeUnicycleDistance(st);
      else if (model == "2Unicycle")
        distance = TwoUnicycleDistance(st);
      else if (model == "2Unicycle2ndOrder")
        distance = Two2ndOrderUnicycleDistance(st);
      else if (model == "3KinematicCars")
        distance = ThreeKinDistance(st);
      else if (model == "2Linear2ndOrder")
        distance = Two2ndOrderLinearDistance(st);
      else if (model == "2KinematicCars2ndOrder")
        distance = Two2ndOrderCarDistance(st);
      else if (model == "3Linear2ndOrder")
        distance = ThreeLinear2ndOrderDistance(st);
      else
      {
        std::cout << "Current Goal Model Not Implemented" << std::endl;
        exit(1);
      }
      return distance;
    }

    virtual std::vector<double> mlExampleDistance(const ob::State *st) const
    {
      
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);

      double deltax = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay = pow((goal[1] - xyState1_->values[1]), 2);
      double deltaz = pow((goal[2] - xyState1_->values[2]), 2);

      double d1 = sqrt(deltax + deltay + deltaz);

      distance.push_back(d1);

      return distance;
    }

    virtual std::vector<double> KinCarDistance(const ob::State *st) const
    {
      
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);

      double deltax_v1 = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay_v1 = pow((goal[1] - xyState1_->values[1]), 2);

      double d1 = sqrt(deltax_v1 + deltay_v1);

      distance.push_back(d1);

      return distance;
    }

    virtual std::vector<double> TwoUnicycleDistance(const ob::State *st) const
    {
      
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
      auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);

      double deltax_v1 = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay_v1 = pow((goal[1] - xyState1_->values[1]), 2);
      double deltax_v2 = pow((goal[4] - xyState2_->values[0]), 2);
      double deltay_v2 = pow((goal[5] - xyState2_->values[1]), 2);

      double d1 = sqrt(deltax_v1 + deltay_v1);
      double d2 = sqrt(deltax_v2 + deltay_v2);

      distance.push_back(d1);
      distance.push_back(d2);

      return distance;
    }

    virtual std::vector<double> Two2ndOrderCarDistance(const ob::State *st) const
    {
      
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
      auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);

      double deltax_v1 = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay_v1 = pow((goal[1] - xyState1_->values[1]), 2);
      double deltax_v2 = pow((goal[5] - xyState2_->values[0]), 2);
      double deltay_v2 = pow((goal[6] - xyState2_->values[1]), 2);

      double d1 = sqrt(deltax_v1 + deltay_v1);
      double d2 = sqrt(deltax_v2 + deltay_v2);

      distance.push_back(d1);
      distance.push_back(d2);

      return distance;
    }

    virtual std::vector<double> Two2ndOrderUnicycleDistance(const ob::State *st) const
    {
      
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
      auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(3);

      double deltax_v1 = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay_v1 = pow((goal[1] - xyState1_->values[1]), 2);
      double deltax_v2 = pow((goal[5] - xyState2_->values[0]), 2);
      double deltay_v2 = pow((goal[6] - xyState2_->values[1]), 2);

      double d1 = sqrt(deltax_v1 + deltay_v1);
      double d2 = sqrt(deltax_v2 + deltay_v2);

      distance.push_back(d1);
      distance.push_back(d2);

      return distance;
    }

    virtual std::vector<double> ThreeUnicycleDistance(const ob::State *st) const
    {
      
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
      auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
      auto xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(4);

      double deltax_v1 = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay_v1 = pow((goal[1] - xyState1_->values[1]), 2);
      double deltax_v2 = pow((goal[4] - xyState2_->values[0]), 2);
      double deltay_v2 = pow((goal[5] - xyState2_->values[1]), 2);
      double deltax_v3 = pow((goal[8] - xyState3_->values[0]), 2);
      double deltay_v3 = pow((goal[9] - xyState3_->values[1]), 2);

      double d1 = sqrt(deltax_v1 + deltay_v1);
      double d2 = sqrt(deltax_v2 + deltay_v2);
      double d3 = sqrt(deltax_v3 + deltay_v3);

      distance.push_back(d1);
      distance.push_back(d2);
      distance.push_back(d3);

      return distance;
    }

    virtual std::vector<double> ThreeLinearDistance(const ob::State *st) const
    {
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
      auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);
      auto xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);

      double deltax_v1 = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay_v1 = pow((goal[1] - xyState1_->values[1]), 2);
      double deltax_v2 = pow((goal[2] - xyState2_->values[0]), 2);
      double deltay_v2 = pow((goal[3] - xyState2_->values[1]), 2);
      double deltax_v3 = pow((goal[4] - xyState3_->values[0]), 2);
      double deltay_v3 = pow((goal[5] - xyState3_->values[1]), 2);

      double d1 = sqrt(deltax_v1 + deltay_v1);
      double d2 = sqrt(deltax_v2 + deltay_v2);
      double d3 = sqrt(deltax_v3 + deltay_v3);

      distance.push_back(d1);
      distance.push_back(d2);
      distance.push_back(d3);

      return distance;
    }

    virtual std::vector<double> ThreeLinear2ndOrderDistance(const ob::State *st) const
    {
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
      auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);
      auto xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);

      double deltax_v1 = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay_v1 = pow((goal[1] - xyState1_->values[1]), 2);
      double deltax_v2 = pow((goal[4] - xyState2_->values[0]), 2);
      double deltay_v2 = pow((goal[5] - xyState2_->values[1]), 2);
      double deltax_v3 = pow((goal[8] - xyState3_->values[0]), 2);
      double deltay_v3 = pow((goal[9] - xyState3_->values[1]), 2);

      double d1 = sqrt(deltax_v1 + deltay_v1);
      double d2 = sqrt(deltax_v2 + deltay_v2);
      double d3 = sqrt(deltax_v3 + deltay_v3);

      distance.push_back(d1);
      distance.push_back(d2);
      distance.push_back(d3);

      return distance;
    }

    virtual std::vector<double> TwoLinearDistance(const ob::State *st) const
    {
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
      auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);

      double deltax_v1 = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay_v1 = pow((goal[1] - xyState1_->values[1]), 2);
      double deltax_v2 = pow((goal[2] - xyState2_->values[0]), 2);
      double deltay_v2 = pow((goal[3] - xyState2_->values[1]), 2);

      double d1 = sqrt(deltax_v1 + deltay_v1);
      double d2 = sqrt(deltax_v2 + deltay_v2);

      distance.push_back(d1);
      distance.push_back(d2);

      return distance;
    }

    virtual std::vector<double> Two2ndOrderLinearDistance(const ob::State *st) const
    {
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
      auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);

      double deltax_v1 = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay_v1 = pow((goal[1] - xyState1_->values[1]), 2);
      double deltax_v2 = pow((goal[4] - xyState2_->values[0]), 2);
      double deltay_v2 = pow((goal[5] - xyState2_->values[1]), 2);

      double d1 = sqrt(deltax_v1 + deltay_v1);
      double d2 = sqrt(deltax_v2 + deltay_v2);

      distance.push_back(d1);
      distance.push_back(d2);

      return distance;
    }

    virtual std::vector<double> TwoKinDistance(const ob::State *st) const
    {
      // std::cout << "i am inside goal function here" << std::endl;
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
      auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);

      double deltax_v1 = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay_v1 = pow((goal[1] - xyState1_->values[1]), 2);
      double deltax_v2 = pow((goal[3] - xyState2_->values[0]), 2);
      double deltay_v2 = pow((goal[4] - xyState2_->values[1]), 2);

      double d1 = sqrt(deltax_v1 + deltay_v1);
      double d2 = sqrt(deltax_v2 + deltay_v2);

      // double d = sqrt(deltax_v1 + deltay_v1 + deltax_v2 + deltay_v2);

      distance.push_back(d1);
      distance.push_back(d2);
      // distances.push_back(d);
      return distance;
    }

    virtual std::vector<double> ThreeKinDistance(const ob::State *st) const
    {
      
      std::vector<double> distance;
      auto cs_ = st->as<ompl::base::CompoundStateSpace::StateType>();
      auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
      auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
      auto xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(4);

      double deltax_v1 = pow((goal[0] - xyState1_->values[0]), 2);
      double deltay_v1 = pow((goal[1] - xyState1_->values[1]), 2);
      double deltax_v2 = pow((goal[3] - xyState2_->values[0]), 2);
      double deltay_v2 = pow((goal[4] - xyState2_->values[1]), 2);
      double deltax_v3 = pow((goal[6] - xyState3_->values[0]), 2);
      double deltay_v3 = pow((goal[7] - xyState3_->values[1]), 2);

      double d1 = sqrt(deltax_v1 + deltay_v1);
      double d2 = sqrt(deltax_v2 + deltay_v2);
      double d3 = sqrt(deltax_v3 + deltay_v3);

      distance.push_back(d1);
      distance.push_back(d2);
      distance.push_back(d3);

      return distance;
    }
};

// // the projection for EST / KPIECE planners
// class MyProjection : public ob::ProjectionEvaluator
// {
// public:
 
//   MyProjection(const ob::StateSpacePtr &space, const int NumVs,
//     const std::string model) : ob::ProjectionEvaluator(space)
//   {
//     numVs_ = NumVs;
//     model_ = model;
//   }
//   int numVs_;
//   std::string model_;
//   const double x_ = 2.0;
//   const double y_ = 2.0;
 
//   virtual unsigned int getDimension(void) const
//   {
//     if (numVs_ == 2)
//       return 4;
//     else if (numVs_ == 3)
//       return 6;
//     else
//     {
//       std::cout << "Projection Not implemented for specified number of vehicles." << std::endl;
//       exit(1);
//     }
    
//   }
 
//   virtual void defaultCellSizes(void)
//   {
//     if (numVs_ == 2)
//     {
//       cellSizes_.resize(4);
//       cellSizes_[0] = x_;
//       cellSizes_[1] = y_;
//       cellSizes_[2] = x_;
//       cellSizes_[3] = y_;
//     }
//     else if (numVs_ == 3)
//     {
//       cellSizes_.resize(6);
//       cellSizes_[0] = x_;
//       cellSizes_[1] = y_;
//       cellSizes_[2] = x_;
//       cellSizes_[3] = y_;
//       cellSizes_[4] = x_;
//       cellSizes_[5] = y_;
//     } 
//   }

//   virtual void project(const ob::State *state, ob::EuclideanProjection &projection) const  // 
//   {
//       auto cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();
      
//       if (model_ == "2Unicycle2ndOrder" )
//       {
//         if (numVs_ == 2)
//         {
//           auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
//           auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(3);
  
//             // projection will be 2 vehicles x, y pos
//             projection(0) = xyState1_->values[0];
//             projection(1) = xyState1_->values[1];
//             projection(2) = xyState2_->values[0];
//             projection(3) = xyState2_->values[1];
//         }
//         else
//         {
//           std::cout << "Projection Not Implemented for Current Number of Unicycle vehicles" << std::endl;
//           exit(1);
//         }
//       }
//       else if (model_ == "2KinematicCars2ndOrder" )
//       {
//         if (numVs_ == 2)
//         {
//           auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
//           auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
  
//             // projection will be 2 vehicles x, y pos
//             projection(0) = xyState1_->values[0];
//             projection(1) = xyState1_->values[1];
//             projection(2) = xyState2_->values[0];
//             projection(3) = xyState2_->values[1];
//         }
//         else
//         {
//           std::cout << "Projection Not Implemented for Current Number of Unicycle vehicles" << std::endl;
//           exit(1);
//         }
//       }
//       else if (model_ == "2Linear2ndOrder")
//       {
//         if (numVs_ == 2)
//         {
//           auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
//           auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);
  
//             // projection will be 2 vehicles x, y pos
//             projection(0) = xyState1_->values[0];
//             projection(1) = xyState1_->values[1];
//             projection(2) = xyState2_->values[0];
//             projection(3) = xyState2_->values[1];
//         }
//         else
//         {
//           std::cout << "Projection Not Implemented for Current Number of Linear vehicles" << std::endl;
//           exit(1);
//         }
//       }
//       else if (model_ == "2Linear" || model_ == "3Linear" || model_ == "3Linear2ndOrder")
//       {
//         std::cout << "using default projection... user beware" << std::endl;
//       	if (numVs_ == 2)
//         {
//           auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
//           auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);
  
//             // projection will be 2 vehicles x, y pos
//             projection(0) = xyState1_->values[0];
//             projection(1) = xyState1_->values[1];
//             projection(2) = xyState2_->values[0];
//             projection(3) = xyState2_->values[1];
//         }
//         else if (numVs_ == 3)
//         {
//           auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
//           auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(1);
//           auto xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
  
//           // projection will be average of 2 vehicle positions
//           projection(0) = xyState1_->values[0];
//           projection(1) = xyState1_->values[1];
//           projection(2) = xyState2_->values[0];
//           projection(3) = xyState2_->values[1];
//           projection(4) = xyState3_->values[0];
//           projection(5) = xyState3_->values[1];
//         }

//       }
//       else
//       {
//         if (model_ != "3Linear" && model_ != "2Linear" )
//         {
//           if (numVs_ == 2)
//           {
//             auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
//             auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
  
//             // projection will be 2 vehicles x, y pos
//             projection(0) = xyState1_->values[0];
//             projection(1) = xyState1_->values[1];
//             projection(2) = xyState2_->values[0];
//             projection(3) = xyState2_->values[1];
//           }
//           else if (numVs_ == 3)
//           {
//             auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
//             auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
//             auto xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(4);
  
//             // projection will be average of 2 vehicle positions
//             projection(0) = xyState1_->values[0];
//             projection(1) = xyState1_->values[1];
//             projection(2) = xyState2_->values[0];
//             projection(3) = xyState2_->values[1];
//             projection(4) = xyState3_->values[0];
//             projection(5) = xyState3_->values[1];
//           }
//           else
//           {
//             std::cout << "Projection Not Implemented for Current Number of vehicles" << std::endl;
//             exit(1);
//           }
//       }
//         std::cout << "Projection Not Implemented for Current Model" << std::endl;
//         exit(1);
//       }
//   }
// };


// // this state validity checker always returns TRUE
// // it is used so that the user can get everything else working prior to worrying about 
// // collision checking
// bool isStateValid_true(const ob::State *state)
// {
//   // auto cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();
//   // auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
//   // auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
//   // auto xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(4);

//   // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
//   return true;
// }

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}


bool plan(std::string planner, std::vector<double> bndry, std::vector<double> gol, 
    std::vector<double> obs, std::vector<double> start, std::string model, 
    const double planningTime, const double Radius, std::vector<std::string> data, 
    const int NumberofVehicles, const int NumberofControls, const int DimensionOfVehicle, 
    int MaxSegments, bool benchmark, std::string solutionName = "txt/path.txt")
{
    bool solved_ = false;
    oc::SimpleSetupPtr ss;
    CreateSimpleSetup(ss, bndry, gol, start, model, Radius);

    const oc::SpaceInformationPtr si = ss->getSpaceInformation();

    if (model == "KinematicCar")
    {
      ss->setStateValidityChecker(std::make_shared<KinematicCarStateValidityChecker>(
        si, obs));
    }
    else if (model == "mlExample")
    {
    	ss->setStateValidityChecker(std::make_shared<mlExampleStateValidityChecker>(
        si, obs));
    }
    else if (model == "2KinematicCars")
    {
      ss->setStateValidityChecker(std::make_shared<TwoKinematicCarStateValidityChecker>(
        si, obs));
    }
    else if (model == "3KinematicCars")
    {
      ss->setStateValidityChecker(std::make_shared<ThreeKinematicCarStateValidityChecker>(
        si, obs));
    }
    else if (model == "2Linear")
    {
      ss->setStateValidityChecker(std::make_shared<TwoLinearCarStateValidityChecker>(
        si, obs));
    }
    else if (model == "3Linear")
    {
      ss->setStateValidityChecker(std::make_shared<ThreeLinearCarStateValidityChecker>(
        si, obs));
    }
    else if (model == "3Unicycle")
    {
      ss->setStateValidityChecker(std::make_shared<ThreeUnicycleStateValidityChecker>(
        si, obs));
      // set state validity checking for this space
      // ss->setStateValidityChecker([](const ob::State *state) { return isStateValid_3Unicycle(state); });
    }
    else if (model == "2Unicycle")
    {
      ss->setStateValidityChecker(std::make_shared<TwoUnicycleStateValidityChecker>(
        si, obs));
      // set state validity checking for this space
      // ss->setStateValidityChecker([](const ob::State *state) { return isStateValid_3Unicycle(state); });
    }
    else if (model == "2Unicycle2ndOrder")
    {
      ss->setStateValidityChecker(std::make_shared<Two2ndOrderUnicycleStateValidityChecker>(
        si, obs));
      // set state validity checking for this space
      // ss->setStateValidityChecker([](const ob::State *state) { return isStateValid_3Unicycle(state); });
    }
    else if (model == "2Linear2ndOrder")
    {
      ss->setStateValidityChecker(std::make_shared<Two2ndOrderLinearCarStateValidityChecker>(
        si, obs));
    }
    else if (model == "2KinematicCars2ndOrder")
    {
      ss->setStateValidityChecker(std::make_shared<Two2ndOrderCarStateValidityChecker>(
        si, obs));
    }
    else if (model == "3Linear2ndOrder")
    {
      ss->setStateValidityChecker(std::make_shared<Three2ndOrderLinearCarStateValidityChecker>(
        si, obs));
    }
    else
    {
      std::cout << "Current State Validity Checker Not Implemented For Model." << std::endl;
      exit(1);
    }

    ob::GoalPtr goal (new MyArbitraryGoal(ss->getSpaceInformation(), gol, Radius, model));

    ss->setGoal(goal);
    
    // ob::PlannerPtr planner(new oc::MAPSRRT(ss->getSpaceInformation(),
    //     NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius));

    if (planner == "RRT")
    {
      ob::PlannerPtr planner(new oc::MyRRT(ss->getSpaceInformation()));

      ss->setPlanner(planner);

      ss->setup();

      ob::PlannerStatus solved = ss->solve(planningTime);
    }

    else if (planner == "MAPSRRT")
    {
      ob::PlannerPtr planner(new oc::MAPSRRT(ss->getSpaceInformation(),
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark));

      ss->setPlanner(planner);

      ss->setup();

      ob::PlannerStatus solved = ss->solve(planningTime);
    }
    // else if (planner == "MAPSEST")
    // {
    //   const ob::StateSpacePtr space = ss->getStateSpace();

    //   // space->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new MyProjection(space)));

    //   // space->registerDefaultProjection(new ob::MyProjection(space));

    //   // ob::ProjectionEvaluatorPtr Projector(new MyProjection(space));

    //   // space->registerDefaultProjection(
    //   //   ob::ProjectionEvaluatorPtr(new MyProjection(space, NumberofVehicles, model)));

    //   // space->registerProjection("myProjection", ob::ProjectionEvaluatorPtr(new MyProjection(space)));

    //   ob::PlannerPtr planner(new oc::MAPSEST(ss->getSpaceInformation(),
    //     NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model));

    //   // planner->setProjectionEvaluator("myProjection");

    //   ss->setPlanner(planner);

    //   ss->setup();

    //   ob::PlannerStatus solved = ss->solve(planningTime);
    // }
    else if (planner == "MAPSRRTmotion")
    {
      ob::PlannerPtr planner(new oc::MAPSRRTmotion(ss->getSpaceInformation(),
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model));

      ss->setPlanner(planner);

      ss->setup();

      ob::PlannerStatus solved = ss->solve(planningTime);
    }
    else if (planner == "MAPSRRTcost")
    {
      std::cout << "all initialized, planning now" << std::endl;
      ob::PlannerPtr planner(new oc::MAPSRRTcost(ss->getSpaceInformation(),
       NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model, 1, solutionName));

      ss->setPlanner(planner);

      ss->setup();

      ob::PlannerStatus solved = ss->solve(planningTime);
    }
    else if (planner == "LazyMAPSRRTcost")
    {
      std::cout << "all initialized, planning now" << std::endl;
      ob::PlannerPtr planner(new oc::LazyMAPSRRTcost(ss->getSpaceInformation(),
       NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model, 1, solutionName));

      ss->setPlanner(planner);

      ss->setup();

      ob::PlannerStatus solved = ss->solve(planningTime);
    }
    else if (planner == "MAPSSST")
    {
    	std::cout << "all initialized, planning now" << std::endl;
      	ob::PlannerPtr planner(new oc::MAPSSST(ss->getSpaceInformation(),
       	NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model, solutionName));

      	ss->setPlanner(planner);

      	ss->setup();

      	ob::PlannerStatus solved = ss->solve(planningTime);
    }

    else if (planner == "RRTplus")
    {
      ob::PlannerPtr planner(new oc::RRTplus(ss->getSpaceInformation(), gol, NumberofVehicles, 
        NumberofControls, Radius));

      ss->setPlanner(planner);

      ss->setup();

      ob::PlannerStatus solved = ss->solve(planningTime);

    }
    else if (planner == "SST")
    {
      ob::PlannerPtr planner(new oc::SST(ss->getSpaceInformation()));
      // ss->setOptimizationObjective(getPathLengthObjective(si));

      ss->setPlanner(planner);

      ss->setup();

      ob::PlannerStatus solved = ss->solve(planningTime);
      
      if (solved)
      {
        std::cout << "Found solution:" << std::endl;

        solved_ = true;

        oc::PathControl path = ss->getSolutionPath();
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
          path.printAsMatrix(PathFile);
          PathFile.close();
          std::cout << "Computation completed successfully" << std::endl;
          // path.print(std::cout);  // this prints out the solution
        }
      }

    }
    else
      std::cout << "Incorrect planner selected... exiting." << std::endl;

    return solved_;

}

void benchmark(std::vector<double> bndry, std::vector<double> gol, 
    std::vector<double> obs, std::vector<double> start, std::string model, 
    const double planningTime, const double Radius, std::vector<std::string> data, 
    const int NumberofVehicles, const int NumberofControls, const int DimensionOfVehicle, 
    int MaxSegments, bool benchmark, int numRuns, const char* logName)
{
  //  set up the space using simple setup
  oc::SimpleSetupPtr ss;
  CreateSimpleSetup(ss, bndry, gol, start, model, Radius);

  const oc::SpaceInformationPtr si = ss->getSpaceInformation();

  if (model == "KinematicCar")
  {
    ss->setStateValidityChecker(std::make_shared<KinematicCarStateValidityChecker>(
      si, obs));
  }
  else if (model == "2KinematicCars")
  {
    ss->setStateValidityChecker(std::make_shared<TwoKinematicCarStateValidityChecker>(
      si, obs));
  }
  else if (model == "3Linear")
  {
    ss->setStateValidityChecker(std::make_shared<ThreeLinearCarStateValidityChecker>(
      si, obs));
  }

  ob::GoalPtr goal (new MyArbitraryGoal(ss->getSpaceInformation(), gol, Radius, model));
    
  ss->setGoal(goal);

  const ob::StateSpacePtr space = ss->getStateSpace();

  // initialize the benchmark object
  ompl::tools::Benchmark b(*(ss.get()), "Lazy vs Complete");

  // initialize planners

  // MAPSRRT
  // ob::PlannerPtr MapsRrt(new oc::MAPSRRT(ss->getSpaceInformation(),
  //       NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark));

  // MAPSRRT Motion
  // ob::PlannerPtr MapsRrtMotion(new oc::MAPSRRTmotion(ss->getSpaceInformation(),
  //       NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model));
  
  // MAPS-RRT Cost
  ob::PlannerPtr MapsRrtCost(new oc::MAPSRRTcost(ss->getSpaceInformation(),
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model));
  
  // Lazy MAPS-RRT Cost
  // ob::PlannerPtr LazyMAPS(new oc::LazyMAPSRRTcost(ss->getSpaceInformation(),
  //      NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model));


  // MAPSEST
  // space->registerDefaultProjection(
  //       ob::ProjectionEvaluatorPtr(new MyProjection(space, NumberofVehicles, model)));

  // ob::PlannerPtr MapsEst(new oc::MAPSEST(ss->getSpaceInformation(),
  //   NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model));


  // RRT
  // ob::PlannerPtr Rrt(new oc::MyRRT(ss->getSpaceInformation()));

  // RRT +
  // ob::PlannerPtr RrtPlus(new oc::RRTplus(ss->getSpaceInformation(), gol, NumberofVehicles,
    // NumberofControls, Radius));

  // add the planners to the benchmark
  // b.addPlanner(LazyMAPS);
  b.addPlanner(MapsRrtCost);
  // b.addPlanner(Rrt);
  // b.addPlanner(RrtPlus);

  // define parameters of the benchmark
  ompl::tools::Benchmark::Request req;
  req.maxTime = planningTime;
  req.maxMem = 100000.0;
  req.runCount = numRuns;
  req.displayProgress = true;

  // After the Benchmark class is defined, the events can be optionally registered:
  b.benchmark(req);

  // This will generate a file of the form results.log
  char buffer [1024];
  int n = sprintf (buffer, "./benchmarking_results/%s.log", logName);
  b.saveResultsToFile(buffer);


  return; 


}

// txt/Shaull/2agents/2ndOrderLin/example1_2Linear2ndOrder.txt

// AO_X(Planner, bndry, gol, obs, strt, DynModel, Tollorance, data, 
//           NumVs, NumCs, dim, numRuns, bench);

void AO_X(std::string planner, std::vector<double> bndry, std::vector<double> gol, 
    std::vector<double> obs, std::vector<double> start, std::string model, 
    const double Radius, std::vector<std::string> data, const int NumberofVehicles, 
    const int NumberofControls, const int DimensionOfVehicle, 
    int numRuns, bool benchmark)
{
  // set initial cost to infinity
  // double MaxSegments = std::numeric_limits<double>::infinity();
  int MaxSegments = 1000;

  // set solve time
  const double SolveTime = 60;

  // run planner to get initial plan
  std::string solutionName = "txt/AO_X/path" + std::to_string(MaxSegments) + ".txt";
  bool solved = plan(planner, bndry, gol, obs, start, model, SolveTime, Radius, data, 
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, 
        benchmark, solutionName);

  
  if (solved)
  {

    // get the final cost and call it c
    for (int i = 0; i < numRuns; i++)
    {
    
      int MaxSegments = getSolutionCost() - 1;
      std::cout << MaxSegments << std::endl;
      if (MaxSegments == 0 || !solved)
        break;
      else
      {
        solutionName = "txt/AO_X/path" + std::to_string(MaxSegments) + ".txt";
        solved = plan(planner, bndry, gol, obs, start, model, SolveTime, Radius, data, 
          NumberofVehicles, NumberofControls, DimensionOfVehicle, 
          MaxSegments, benchmark, solutionName);
      }
    }
  }
}


void MAPS_SST_star(std::vector<double> bndry, std::vector<double> gol, 
    std::vector<double> obs, std::vector<double> start, std::string model, 
    const double Radius, std::vector<std::string> data, const int NumberofVehicles, 
    const int NumberofControls, const int DimensionOfVehicle, 
    int numRuns, const int MaxSegs, bool benchmark, std::string solutionName = "txt/path.txt")
{
  // set up the problem
  oc::SimpleSetupPtr ss;
  CreateSimpleSetup(ss, bndry, gol, start, model, Radius);

  const oc::SpaceInformationPtr si = ss->getSpaceInformation();

  if (model == "KinematicCar")
  {
    ss->setStateValidityChecker(std::make_shared<KinematicCarStateValidityChecker>(
      si, obs));
  }
  else if (model == "2KinematicCars")
  {
    ss->setStateValidityChecker(std::make_shared<TwoKinematicCarStateValidityChecker>(
      si, obs));
  }
  else if (model == "3KinematicCars")
  {
    ss->setStateValidityChecker(std::make_shared<ThreeKinematicCarStateValidityChecker>(
      si, obs));
  }
  else if (model == "2Linear")
  {
    ss->setStateValidityChecker(std::make_shared<TwoLinearCarStateValidityChecker>(
      si, obs));
  }
  else if (model == "3Linear")
  {
    ss->setStateValidityChecker(std::make_shared<ThreeLinearCarStateValidityChecker>(
      si, obs));
  }
  else if (model == "3Unicycle")
  {
    ss->setStateValidityChecker(std::make_shared<ThreeUnicycleStateValidityChecker>(
      si, obs));
    // set state validity checking for this space
    // ss->setStateValidityChecker([](const ob::State *state) { return isStateValid_3Unicycle(state); });
  }
  else if (model == "2Unicycle")
  {
    ss->setStateValidityChecker(std::make_shared<TwoUnicycleStateValidityChecker>(
      si, obs));
    // set state validity checking for this space
    // ss->setStateValidityChecker([](const ob::State *state) { return isStateValid_3Unicycle(state); });
  }
  else
  {
    std::cout << "Current State Validity Checker Not Implemented For Model." << std::endl;
    exit(1);
  }

  ob::GoalPtr goal (new MyArbitraryGoal(ss->getSpaceInformation(), gol, Radius, model));

  ss->setGoal(goal);

  // set the initial path length bound to inf
  double maxLength = std::numeric_limits<double>::infinity();

  // set tuning params, decay factor, and planning time
  const double xi = 0.5;
  const double planningTime = 100.0;
  double selRad = 0.2;
  double pruneRad = 0.1;
  int j = 0;
  // set d and l which are dimensionality of state/input spaces respectively
  double d = ss->getSpaceInformation()->getStateSpace()->getDimension();
  double l = ss->getSpaceInformation()->getControlSpace()->getDimension();

  for (int i = 0; i < numRuns; i++)
  {
    auto planner(std::make_shared<oc::MAPSSST>(ss->getSpaceInformation(), 
    NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegs, gol, 
    Radius, benchmark, model, solutionName, maxLength));

    planner->setSelectionRadius(selRad);
    planner->setPruningRadius(pruneRad);

    ss->setPlanner(planner);
    ss->setup();

    ob::PlannerStatus solved = ss->solve(planningTime);

    if (solved)
    {
      selRad = xi * selRad;
      pruneRad = xi * pruneRad;
      j += 1;
      maxLength = planner->getFinalPathLength();
      // planningTime = InitplanningTime * (1 + std::log10(j)) * pow(xi, (-(d + l + 1)*j));
      // std::cout <<"New Planning Time: " << planningTime << std::endl;
    }
    else
      break;
  }
}


int main(int /*argc*/, char ** /*argv*/)
{

    // ***********************************
    // Mutable section for running locally
    // ***********************************

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    // Define the multi-agent problem from world file
    std::string world;
    std::string Task;
    std::string Planner;
    std::string DynModel;
    int dim;
    int NumVs;
    int NumCs;
    std::vector<std::string> data;
    std::vector<double> bndry;
    std::vector<double> gol;
    std::vector<double> obs;
    std::vector<double> strt;

    // pos obs for ex 3 w/ 2 agents
    //obstacle 3.75 2.75 -0.5233 0.25 2 1.5233
    // obstacle 2.75 2.75 -0.5233 0.25 3 1.5233

    std::cout << "Enter the path to the word file: " << std::endl;
    std::cin >> world;

    readFile(world, bndry, gol, obs, strt, DynModel, dim, NumVs, NumCs, data);
    // MyPrint(obs);
    double SolveTime;
    double Tollorance;

    // need to add this to read file eventually
    int MaxSegments;

    int NumberofBenchmarkRuns;


    std::cout << "Would you like to plan with one planner or benchmark all planners?" << std::endl;
    std::cout << "Choices: plan, or benchmark" << std::endl;
    std::cin >> Task;

    if (Task == "plan")
    {
      bool bench = false;

      std::cout << "Enter the planner you wish to use." << std::endl;
      std::cout << "Choices: AO-X, MAPSRRT, MAPSEST, MAPSRRTmotion, MAPSRRTcost, LazyMAPSRRTcost, MAPSSST, MAPSSST*, SST, RRT, or RRTplus" << std::endl;
      std::cout << "Enter: ";
      std::cin >> Planner;

      if (Planner == "AO-X")
      {
        std::cout << "Enter the planner you wish to use." << std::endl;
        std::cout << "Choices: MAPSRRT, MAPSRRTmotion, MAPSRRTcost, RRT, or RRTplus" << std::endl;
        std::cout << "Enter: ";
        std::cin >> Planner;

        int numRuns;
        std::cout << "Enter how many runs you would like to perform." << std::endl;
        std::cout << "Note that the planner approaches true optimality as the number approaches infinity." << std::endl;
        std::cout << "Enter: ";
        std::cin >> numRuns;

        std::cout << "Enter the radius of each vehicles goal region: ";
        std::cin >> Tollorance;

        AO_X(Planner, bndry, gol, obs, strt, DynModel, Tollorance, data, 
          NumVs, NumCs, dim, numRuns, bench);
      }
      else if (Planner == "MAPSSST*")
      {
        std::cout << "Enter the maximum number of segments (should be integer): ";
        std::cin >> MaxSegments;

        int numRuns;
        std::cout << "Enter how many runs you would like to perform." << std::endl;
        std::cout << "Note that the planner approaches true optimality as the number approaches infinity." << std::endl;
        std::cout << "Enter: ";
        std::cin >> numRuns;

        std::cout << "Enter the radius of each vehicles goal region: ";
        std::cin >> Tollorance;

        MAPS_SST_star(bndry, gol, obs, strt, DynModel, Tollorance, data, 
          NumVs, NumCs, dim, numRuns, MaxSegments, bench);

      }

      // enter non-MAPS planner's here
      else if (Planner == "RRT" || Planner == "RRTplus" || Planner == "SST")
      {
        MaxSegments = 1;
        std::cout << "Enter the maximum solve time (seconds): ";
        std::cin >> SolveTime;

        std::cout << "Enter the radius of each vehicles goal region: ";
        std::cin >> Tollorance;

        plan(Planner, bndry, gol, obs, strt, DynModel, SolveTime, Tollorance, data, 
        NumVs, NumCs, dim, MaxSegments, bench);
      }
      else
      {
        std::cout << "Enter the maximum number of segments (should be integer): ";
        std::cin >> MaxSegments;
        std::cout << "Enter the maximum solve time (seconds): ";
        std::cin >> SolveTime;

        std::cout << "Enter the radius of each vehicles goal region: ";
        std::cin >> Tollorance;

        plan(Planner, bndry, gol, obs, strt, DynModel, SolveTime, Tollorance, data, 
        NumVs, NumCs, dim, MaxSegments, bench);
      }
    }

    else if (Task == "benchmark")
    {
      bool bench = true;
      
      std::cout << "Enter number of planning runs (must be integer): ";
      std::cin >> NumberofBenchmarkRuns;

      std::cout << "Enter the maximum solve time (seconds): ";
      std::cin >> SolveTime;

      std::cout << "Enter the maximum number of segments for MAPS planners (should be integer): ";
        std::cin >> MaxSegments;

      std::cout << "Enter the radius of each vehicles goal region: ";
      std::cin >> Tollorance;

      const char* name = "results_congested_1c_1";

      benchmark(bndry, gol, obs, strt, DynModel, SolveTime, Tollorance, data, NumVs, NumCs, dim, 
        MaxSegments, bench, NumberofBenchmarkRuns, name);

    }

    else
      std::cout << "No meaningful task selected... exiting.";

    return 0;
}











