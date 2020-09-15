// ompl includes
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/base/Goal.h>
// #include <ompl/base/goals/GoalRegion.h>
// #include <ompl/control/planners/rrt/RRT.h>
// std includes
#include <iostream>
#include <valarray>
#include <limits>
#include <math.h>
#include <typeinfo>
// my includes
#include "../includes/ReadWorld.h"
#include "../includes/CreateSimpleSetup.h"
#include "../includes/KinematicCar.h"
#include "../includes/LinearCar.h"
#include "../includes/Unicycle.h"
#include "../includes/MyPlanner.h"
#include "../includes/MyPlannerMotion.h"
#include "../includes/MyPlannerCost.h"
#include "../includes/MAPSRRTPathControl.h"
#include "ompl/tools/benchmark/Benchmark.h"
#include "../includes/RRTplus.h"
#include "../includes/MyRRT.h"
// #include "../includes/RRTplusPathControl.h"


class KinematicCarStateValidityChecker : public ob::StateValidityChecker
{
public:
    KinematicCarStateValidityChecker(const ob::SpaceInformationPtr &si, std::vector<double> obs) :
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

        // for (int i = 0; i < obstacle.size(); i=i+6)
        // {
        //   // // create obs object
        //   double x = obstacle[i];
        //   double y = obstacle[i+1];
        //   double xdist = obstacle[3];
        //   double ydist = obstacle[4];

        //   point BotL(x, y);
        //   point TopL(x, y + ydist);
        //   point BotR(x + xdist, y);
        //   point TopR(x + xdist, y + ydist);

        //   // create instance of polygon
        //   polygon obst;
        //   // // add the outer points to the shape
        //   obst.outer().push_back(BotL);
        //   obst.outer().push_back(TopL);
        //   obst.outer().push_back(TopR);
        //   obst.outer().push_back(BotR);
        //   obst.outer().push_back(BotL);

        //   // for all vehicles, see if they collide with obs
        //   for (int v = 0; v < vs.size(); v++)
        //   {
        //     bool disjoint = boost::geometry::disjoint(vs[v], obst);

        //     if (!disjoint)
        //     {
        //       return false;
        //     }
        //   }
        // }  
        return true;
    }

    ob::StateSpacePtr space_;
    std::vector<double> obstacle;
    const double buffer_ = 0.2;
};

// used for 2 agent planning for kinematic car
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
        // if that tests passes, check colliding with objects
        // for (int i = 0; i < obstacle.size(); i=i+6)
        // {
        //   // // create obs object
        //   double x = obstacle[i];
        //   double y = obstacle[i + 1];
        //   double xdist = obstacle[i + 3];
        //   double ydist = obstacle[i + 4];

        //   point BotL(x, y);
        //   point TopL(x, y + ydist);
        //   point BotR(x + xdist, y);
        //   point TopR(x + xdist, y + ydist);

        //   // create instance of polygon
        //   polygon obst;
        //   // // add the outer points to the shape
        //   obst.outer().push_back(BotL);
        //   obst.outer().push_back(TopL);
        //   obst.outer().push_back(TopR);
        //   obst.outer().push_back(BotR);
        //   obst.outer().push_back(BotL);

        //   // for all vehicles, see if they collide with obs
        //   for (int v = 0; v < vs.size(); v++)
        //   {
        //     bool disjoint = boost::geometry::disjoint(vs[v], obst);

        //     if (!disjoint)
        //     {
        //       return false;
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
      if (model == "2KinematicCars")
        distance = TwoKinDistance(st);
      else if (model == "2Linear")
        distance = TwoLinearDistance(st);
      else if (model == "3Linear")
        distance = ThreeLinearDistance(st);
      else if (model == "3Unicycle")
        distance = ThreeUnicycleDistance(st);
      else
      {
        std::cout << "Current Model Not Implemented" << std::endl;
        exit(1);
      }
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
};


// this state validity checker always returns TRUE
// it is used so that the user can get everything else working prior to worrying about 
// collision checking
bool isStateValid_3Unicycle(const ob::State *state)
{
  auto cs_ = state->as<ompl::base::CompoundStateSpace::StateType>();
  auto xyState1_ = cs_->as<ob::RealVectorStateSpace::StateType>(0);
  auto xyState2_ = cs_->as<ob::RealVectorStateSpace::StateType>(2);
  auto xyState3_ = cs_->as<ob::RealVectorStateSpace::StateType>(4);

  // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
  return true;
}


void plan(std::string planner, std::vector<double> bndry, std::vector<double> gol, 
    std::vector<double> obs, std::vector<double> start, std::string model, 
    const double planningTime, const double Radius, std::vector<std::string> data, 
    const int NumberofVehicles, const int NumberofControls, const int DimensionOfVehicle, 
    int MaxSegments, bool benchmark)
{
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
    else
    {
      std::cout << "Current Model Not Implemented." << std::endl;
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

      if (solved)
      {
        std::cout << "Found solution:" << std::endl;

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

    else if (planner == "MAPSRRT")
    {
      ob::PlannerPtr planner(new oc::MAPSRRT(ss->getSpaceInformation(),
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark));

      ss->setPlanner(planner);

      ss->setup();

      ob::PlannerStatus solved = ss->solve(planningTime);
    }

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
      ob::PlannerPtr planner(new oc::MAPSRRTcost(ss->getSpaceInformation(),
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model));

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

    else
      std::cout << "Incorrect planner selected... exiting." << std::endl;
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

  // initialize the benchmark object
  ompl::tools::Benchmark b(*(ss.get()), "my experiment");

  // initialize planners

  // MAPSRRT
  // ob::PlannerPtr MapsRrt(new oc::MAPSRRT(ss->getSpaceInformation(),
        // NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark));

  // MAPSRRT Motion
  ob::PlannerPtr MapsRrtMotion(new oc::MAPSRRTmotion(ss->getSpaceInformation(),
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model));
  
  // MAPSRRT Cost
  ob::PlannerPtr MapsRrtCost(new oc::MAPSRRTcost(ss->getSpaceInformation(),
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark, model));
  
  // RRT
  // ob::PlannerPtr Rrt(new oc::MyRRT(ss->getSpaceInformation()));

  // RRT +
  // ob::PlannerPtr RrtPlus(new oc::RRTplus(ss->getSpaceInformation(), gol, NumberofVehicles,
    // NumberofControls, Radius));

  // add the planners to the benchmark
  // b.addPlanner(MapsRrt);
  b.addPlanner(MapsRrtMotion);
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
  plan(planner, bndry, gol, obs, start, model, SolveTime, Radius, data, 
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, benchmark);

  // get the final cost and call it c
  for (int i = 0; i < numRuns; i++)
  {
    int MaxSegments = getSolutionCost() - 1;

    plan(planner, bndry, gol, obs, start, model, SolveTime, Radius, data, 
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, benchmark);
    
    // run planner for cost of c-1

    // update get new cost and call it c. 
  }
}


int main(int /*argc*/, char ** /*argv*/)
{

    // ***********************************
    // Mutable section for running locally
    // ***********************************

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    // Define the multi-agent problem from world file
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

    readFile("txt/RSS_World.txt", bndry, gol, obs, strt, DynModel, dim, NumVs, NumCs, data);
    
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
      std::cout << "Choices: AO-X, MAPSRRT, MAPSRRTmotion, MAPSRRTcost, RRT, or RRTplus" << std::endl;
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

      else if (Planner == "RRT" || Planner == "RRTplus")
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

      const char* name = "results";

      benchmark(bndry, gol, obs, strt, DynModel, SolveTime, Tollorance, data, NumVs, NumCs, dim, 
        MaxSegments, bench, NumberofBenchmarkRuns, name);

    }

    else
      std::cout << "No meaningful task selected... exiting.";

    return 0;

    // *******************************************
    // Unmutable section for benchmarking remotely
    // *******************************************

    // std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
    // // Define the multi-agent problem from world file
    // std::string DynModel;
    // int dim;
    // int NumVs;
    // int NumCs;
    // std::vector<std::string> data;
    // std::vector<double> bndry;
    // std::vector<double> gol;
    // std::vector<double> obs;
    // std::vector<double> strt;

    // readFile("txt/RSS_World_Hard_3Linear.txt", bndry, gol, obs, strt, DynModel, dim, NumVs, NumCs, data);
    
    // double SolveTime = 100;  // 10 minute planning time
    // double Tollorance = 1.0;  // radius of 1 unit

    // int MaxSegments = 7;

    // int NumberofBenchmarkRuns = 500;

    // bool bench = true;

    // const char* name = "results_unlimited_500";

    // benchmark(bndry, gol, obs, strt, DynModel, SolveTime, Tollorance, data, NumVs, NumCs, dim, 
    //   MaxSegments, bench, NumberofBenchmarkRuns, name);

    return 0;
}











