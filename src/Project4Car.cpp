///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Theodoros Tavoulareas, Efe Eboreime
//////////////////////////////////////

#include <iostream>
#include <functional>
#include <fstream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/base/State.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include "ompl/tools/benchmark/Benchmark.h"

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // The dimension of your projection for the car
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // Projection for the car
        const auto *cstate = state->as<ompl::base::CompoundState>();
        auto r2state = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
        projection(0) = r2state->values[0];
        projection(1) = r2state->values[1];
    }
};

void carODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
            ompl::control::ODESolver::StateType &qdot)
{
    // ODE for the car's dynamics
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    // q = (x,y,theta,v)
    // u = (ω, vdot)
    qdot.resize(q.size(),0);
    qdot[0] = q[3]*cos(q[2]);
    qdot[1] = q[3]*sin(q[2]);
    qdot[2] = u[0];
    qdot[3] = u[1];
}

void makeStreet(std::vector<Rectangle> &obstacles)
{
    // Vector of rectangles of the street environment.
    Rectangle rect;

    //bottom
    rect.x = -10;
    rect.y = -10;
    rect.height = 4;
    rect.width = 20;
    obstacles.push_back(rect);

    // middle left
    rect.x = -10;
    rect.y = -4;
    rect.height = 8;
    rect.width = 10;
    obstacles.push_back(rect);

    // middle right
    rect.x = 2;
    rect.y = -4;
    rect.height = 8;
    rect.width = 8;
    obstacles.push_back(rect);

    //top
    rect.x = -10;
    rect.y = 6;
    rect.height = 2;
    rect.width = 20;
    obstacles.push_back(rect);
}

bool isStateValid(const ompl::control::SpaceInformation *si, const ompl::base::State *state,
                     const std::vector <Rectangle> &obstacles) 
{

    const auto *cstate = state->as<ompl::base::CompoundState>();
    // position of the car
    auto pos = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
    // assume that the car is a point system for collision checking
    bool isCollisionValid = isValidStatePoint(pos, obstacles);
    // the state is valid and within the bounds
    return si->satisfiesBounds(state) && isCollisionValid;
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles)
{
    // Create and setup the car's state space, control space, validity checker, everything we need for planning

    // state for theta (SO(2))
    auto so2state = std::make_shared<ompl::base::SO2StateSpace>();
    // state for position (R^2)
    auto posState = std::make_shared<ompl::base::RealVectorStateSpace>(2);

    // set bounds
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    posState->setBounds(bounds);

    // state for velocity (R)
    auto vstate = std::make_shared<ompl::base::RealVectorStateSpace>(1);
    // set bounds for velocity
    ompl::base::RealVectorBounds vbounds(1);
    vbounds.setLow(-3);
    vbounds.setHigh(3);
    vstate->setBounds(vbounds);

    // the total state space for the car is: R^2 x SO(2) x R
    ompl::base::StateSpacePtr space;
    space = posState + so2state + vstate;

    // create a control space (acceleration)
    auto cspace(std::make_shared<ompl::control::RealVectorControlSpace>(space, 2));
    // set bounds for control
    ompl::base::RealVectorBounds cbounds(2);
    cbounds.setLow(-1);
    cbounds.setHigh(1);
    cspace->setBounds(cbounds);

    // define a simple setup class
    ompl::control::SimpleSetupPtr ss(std::make_shared<ompl::control::SimpleSetup>(cspace));

    // set state validity checking for this space
    ompl::control::SpaceInformation *si = ss->getSpaceInformation().get();
    auto isValid = std::bind(isStateValid, si, std::placeholders::_1, obstacles);
    ss->setStateValidityChecker(isValid);

    // Use the ODESolver to propagate the system.
    auto odeSolver(std::make_shared<ompl::control::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));
    si->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));

    // start state for planning
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> start(space);
    start[0] = -8;
    start[1] = -5;
    start[2] = 0;
    start[3] = 0;

    // goal state for planning
    ompl::base::ScopedState<ompl::base::CompoundStateSpace> goal(space);
    goal[0] = 5;
    goal[1] = 5;
    goal[2] = 1;
    goal[3] = 0;

    ss->setStartAndGoalStates(start, goal, 1);

    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // Do some motion planning for the car
    // choice is what planner to use.
    switch (choice) {
        case 1:
        {
            auto planner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation()));
            ss->setPlanner(planner);
        }
        break;
        case 2:
        {
            auto si = ss->getSpaceInformation();
            auto space = si->getControlSpace()->getStateSpace();
            auto planner(std::make_shared<ompl::control::KPIECE1>(si));
            
            space->registerProjection("myProjection", ompl::base::ProjectionEvaluatorPtr(
                    new CarProjection(space->as<ompl::base::StateSpace>())));
            planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("myProjection");
            ss->setPlanner(planner);
        }
        break;
        case 3:
        {
            auto planner(std::make_shared<ompl::control::RGRRT>(ss->getSpaceInformation()));
            ss->setPlanner(planner);
        }
        break;
    }

    ss->setup();

    // attempt to solve the problem within 10 seconds of planning time
    ompl::base::PlannerStatus solved = ss->solve(10.0);

    if (solved)
    {
        // solution path
        ompl::control::PathControl path = ss->getSolutionPath();
        path.interpolate();
        std::cout << "Found solution:" << std::endl;
        // "geometrize" path
        ss->getSolutionPath().asGeometric();
        path.printAsMatrix(std::cout);

        // save path to path_car.txt
        std::ofstream myfile;
        myfile.open("path_car.txt");
        path.asGeometric().printAsMatrix(myfile);
        myfile.close();
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void benchmarkCar(ompl::control::SimpleSetupPtr &ss)
{
    double runtime_limit = 60.0;
    double memory_limit = 10000.0;  // set high because memory usage is not always estimated correctly
    double run_count = 50;

    // create the benchmark object and add all the planners we'd like to run
    ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
    ompl::tools::Benchmark b(*ss, "Car");

    // RRT
    b.addPlanner(std::make_shared<ompl::control::RRT>(ss->getSpaceInformation() ));

    auto si = ss->getSpaceInformation();
    auto space = si->getControlSpace()->getStateSpace();
    // KPIECE
    auto planner(std::make_shared<ompl::control::KPIECE1>(si));
    space->registerProjection("myProjection", ompl::base::ProjectionEvaluatorPtr(new CarProjection(space->as<ompl::base::StateSpace>())));
    planner->as<ompl::control::KPIECE1>()->setProjectionEvaluator("myProjection");
    ss->setPlanner(planner);

    b.addPlanner(planner);

    // RG-RRT
    auto planner1(std::make_shared<ompl::control::RGRRT>(si));
    ss->setPlanner(planner1);
    b.addPlanner(planner1);

    b.benchmark(request);
    b.saveResultsToFile();
}

int main(int /* argc*/, char ** /*argv*/)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
