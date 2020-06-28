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
#include "../includes/MyPlanner.h"
#include "../includes/MyPlannerMotion.h"
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

// used for 2 agent planning
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
        const ob::CompoundStateSpace::StateType *cs = state->as<ob::CompoundStateSpace::StateType>();

        // recall these are indexed exactly the same order as they are added in Compound state space
        // see CreateSimpleSetup to understand these indexes

        const auto *xyState1 = cs->as<ob::RealVectorStateSpace::StateType>(0);

        const auto *xyState2 = cs->as<ob::RealVectorStateSpace::StateType>(2);

        // const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

        // const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

        // const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

        double x1 = xyState1->values[0];
        double y1 = xyState1->values[1];

        double x2 = xyState2->values[0];
        double y2 = xyState2->values[1];
            
        for (int i = 0; i < obstacle.size(); i=i+6)
        {
          if (x1>(obstacle[i] - buffer_) && x1<(obstacle[i] + obstacle[i+3] + buffer_) && 
            y1>(obstacle[i+1] - buffer_) && y1<(obstacle[i+1]+obstacle[i+4] + buffer_))
          {
            return false;
          }
          else if((x2>(obstacle[i] - buffer_) && x2<(obstacle[i] + obstacle[i+3] + buffer_) && 
                 y2>(obstacle[i+1] - buffer_) && y2<(obstacle[i+1]+obstacle[i+4] + buffer_)))
          {
            return false;
          }
          else if (abs(x1 - x2) <= 2*buffer_ && abs(y1 - y2) <= 2*buffer_)
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


class MyArbitraryGoal : public ompl::base::Goal
{
  public:
    MyArbitraryGoal(const ob::SpaceInformationPtr &si, std::vector<double> _goal, const double _toll) : ompl::base::Goal(si)
    {
      goal = _goal;  // [x1, y1, th1, x2, y2, th2]
      toll = _toll;
    }
    std::vector<double> goal;
    double toll;

    virtual bool isSatisfied(const ob::State *st) const
    {
      std::vector<double> d = TwoVehicleDistance(st);

      // double distance = d[0] + d[1];
      // if (d[2] <= toll)
      //   return true;
      // else
      //   return false;
      // std::cout << toll << std::endl;

      if (d[0] <= toll)
      {
        if (d[1] <= toll)
        {
          // MyPrint(d);
          return true;
        }
        else
          return false;
      }
      else
        return false;
    }

    virtual bool isSatisfied(const ob::State *st, double *distance) const
    {
        bool result = isSatisfied(st);
        std::vector<double> d = TwoVehicleDistance(st);
        double dist = sqrt(pow(d[0], 2) + pow(d[1], 2));
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

    virtual std::vector<double> TwoVehicleDistance(const ob::State *st) const
    {
      std::vector<double> distances;
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

      distances.push_back(d1);
      distances.push_back(d2);
      // distances.push_back(d);
      return distances;
    }
};

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

    // ob::GoalPtr goal (new MyGoalRegion(ss->getSpaceInformation(), gol, Tollorance));

    ob::GoalPtr goal (new MyArbitraryGoal(ss->getSpaceInformation(), gol, Radius));
    
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
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark));

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

  ob::GoalPtr goal (new MyArbitraryGoal(ss->getSpaceInformation(), gol, Radius));
    
  ss->setGoal(goal);

  // initialize the benchmark object
  ompl::tools::Benchmark b(*(ss.get()), "my experiment");

  // initialize the three planners
  ob::PlannerPtr MapsRrt(new oc::MAPSRRT(ss->getSpaceInformation(),
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark));

  ob::PlannerPtr MapsRrtMotion(new oc::MAPSRRTmotion(ss->getSpaceInformation(),
        NumberofVehicles, NumberofControls, DimensionOfVehicle, MaxSegments, gol, Radius, benchmark));
  
  ob::PlannerPtr Rrt(new oc::MyRRT(ss->getSpaceInformation()));

  ob::PlannerPtr RrtPlus(new oc::RRTplus(ss->getSpaceInformation(), gol, NumberofVehicles,
    NumberofControls, Radius));

  // addPlannerProgressProperty("best cost REAL", std::bind(&MapsRrtMotion::FinalCostProperty, this));


  // add the planners to the benchmark
  // b.addPlanner(MapsRrt);
  b.addPlanner(MapsRrtMotion);
  // b.addPlanner(Rrt);
  // b.addPlanner(RrtPlus);

  // define parameters of the benchmark
  ompl::tools::Benchmark::Request req;
  req.maxTime = planningTime;
  req.maxMem = 2000.0;
  req.runCount = numRuns;
  req.displayProgress = true;

  // After the Benchmark class is defined, the events can be optionally registered:
  b.benchmark(req);

  // This will generate a file of the form results.log
  char buffer [1024];
  int n = sprintf (buffer, "./benchmarking_results/%s.log", logName);
  b.saveResultsToFile(buffer);

  // b.saveResultsToFile("./benchmarking_results/test.log");

  return; 


}


int main(int /*argc*/, char ** /*argv*/)
{
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
      std::cout << "Choices: MAPSRRT, MAPSRRTmotion, RRT, or RRTplus" << std::endl;
      std::cout << "Enter: ";
      std::cin >> Planner;

      std::cout << "Enter the maximum solve time (seconds): ";
      std::cin >> SolveTime;

      std::cout << "Enter the radius of each vehicles goal region: ";
      std::cin >> Tollorance;

      if (Planner == "RRT" || Planner == "RRTplus")
        MaxSegments = 1;
      else
      {
        std::cout << "Enter the maximum number of segments (should be integer): ";
        std::cin >> MaxSegments;
      }

    
      plan(Planner, bndry, gol, obs, strt, DynModel, SolveTime, Tollorance, data, 
        NumVs, NumCs, dim, MaxSegments, bench);
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
}











