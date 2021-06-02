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

#include "ompl/geometric/planners/rrt/RRTHybrid.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/base/goals/GoalStates.h"
#include <sys/stat.h>
#include <fstream>

ompl::geometric::RRTHybrid::RRTHybrid(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "RRTHybridintermediate" : "RRTHybrid")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RRTHybrid::setRange, &RRTHybrid::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTHybrid::setGoalBias, &RRTHybrid::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RRTHybrid::setIntermediateStates, &RRTHybrid::getIntermediateStates,
                                "0,1");

    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::RRTHybrid::~RRTHybrid()
{
    freeMemory();
}

void ompl::geometric::RRTHybrid::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::RRTHybrid::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::RRTHybrid::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::RRTHybrid::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    char buffer[80];
    std::time_t rawtime = std::time(nullptr);
    std::strftime(buffer, 80, "%F_%T", std::localtime(&rawtime));
    std::string dir = "/tmp/ompl_logs" + std::string(buffer);
    mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    std::string filepath_start_stop = dir + "/plan_bounds" + std::to_string(start_time.time_since_epoch().count()) + ".csv";
    std::string filepath = dir + "/plan" + std::to_string(start_time.time_since_epoch().count()) + ".csv";

    auto goal_vals = goal->as<ompl::base::GoalStates>()->getState(0)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    goal->print();
    {
      std::ofstream file;
      file.open(filepath_start_stop);
      for (int i = 0; i < si_->getStateDimension(); i++)
          file << "start_j" << i << ",";
      for (int i = 0; i < si_->getStateDimension(); i++)
          file << "stop_j" << i << ",";
      file << "\n";

      sampler_->sampleUniform(rstate);
      /* find closest state in the tree */
      Motion *nmotion = nn_->nearest(rmotion);

      auto start_vals = nmotion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;

      for (int i = 0; i < si_->getStateDimension(); i++)
          file << start_vals[i] << ",";
      for (int i = 0; i < si_->getStateDimension(); i++)
//          file << start_vals[i] << ",";
          file << goal_vals[i] << ",";
      file << "\n";
      file.close();
    }
    {
        std::ofstream file;
        file.open(filepath);

        for (int i = 0; i < si_->getStateDimension(); i++)
            file << "nearest_joint_" << i << ",";
        for (int i = 0; i < si_->getStateDimension(); i++)
            file << "random_joint_" << i << ",";
        file << "random_valid,";
        for (int i = 0; i < si_->getStateDimension(); i++)
            file << "smart_joint_" << i << ",";
        file << "smart_valid,goal_reached,\n";
        file.close();
    }

    while (!ptc)
    {
        /* sample random state (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);
        base::State *dstate = rstate;

        /* find state to add */
        double d = si_->distance(nmotion->state, rstate);
        if (d > maxDistance_)
        {
            si_->getStateSpace()->interpolate(nmotion->state, rstate, maxDistance_ / d, xstate);
            dstate = xstate;
        }

        {
            auto nearest_vals = nmotion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            auto random_vals = dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            std::fstream file;
            file.open(filepath, std::fstream::app);
            for (int i = 0; i < si_->getStateDimension(); i++)
                file << nearest_vals[i] << ",";
            for (int i = 0; i < si_->getStateDimension(); i++)
                file << random_vals[i] << ",";
            file.close();
        }

        if (si_->checkMotion(nmotion->state, dstate))
        {
            {
                std::fstream file;
                file.open(filepath, std::fstream::app);
                file << true << ",";
                file.close();
            }

            if (addIntermediateStates_)
            {
                std::vector<base::State *> states;
                const unsigned int count = si_->getStateSpace()->validSegmentCount(nmotion->state, dstate);

                if (si_->getMotionStates(nmotion->state, dstate, states, count, true, true))
                    si_->freeState(states[0]);

                for (std::size_t i = 1; i < states.size(); ++i)
                {
                    auto *motion = new Motion;
                    motion->state = states[i];
                    motion->parent = nmotion;
                    nn_->add(motion);

                    nmotion = motion;
                }
            }
            else
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, dstate);
                motion->parent = nmotion;
                nn_->add(motion);

                nmotion = motion;
            }


            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                {
                    auto random_vals = dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                    std::fstream file;
                    file.open(filepath, std::fstream::app);
                    for (int i = 0; i < si_->getStateDimension(); i++)
                        file << random_vals[i] << ",";
                    file << true << "," << true << ",\n";
                    file.close();
                }

                approxdif = dist;
                solution = nmotion;
                break;
            }

            // UNIQUE STUFF START
            auto vals = dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            std::cout << "DIMENTIONS: " << si_->getStateDimension() << std::endl;
            std::cout << "[";
            for (int i = 0; i < si_->getStateDimension() - 1; i++)
            {
                std::cout << vals[i] << ", ";
            }
            std::cout << vals[si_->getStateDimension() - 1] << "]" << std::endl;

            if (gen_state_)
            {
                auto *smart_motion = new Motion(si_);
                base::State *smart_state = smart_motion->state;
                gen_state_(dstate, smart_state);

                base::State *xtra_state = si_->allocState();

                double d_smart_state = si_->distance(dstate, smart_state);
                if (d_smart_state > maxDistance_)
                {
                    si_->getStateSpace()->interpolate(dstate, smart_state, maxDistance_ / d_smart_state, xtra_state);
                    smart_state = xtra_state;
                }

                {
                    auto smart_vals = smart_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                    std::fstream file;
                    file.open(filepath, std::fstream::app);
                    for (int i = 0; i < si_->getStateDimension(); i++)
                        file << smart_vals[i] << ",";
                    file.close();
                }

                if (si_->checkMotion(dstate, smart_state))
                {
                    {
                        std::fstream file;
                        file.open(filepath, std::fstream::app);
                        file << true << "," << false << ",\n";
                        file.close();
                    }
                    std::cout << "*******Smart state is valid" << std::endl;
                    auto *motion = new Motion(si_);
                    si_->copyState(motion->state, smart_state);
                    motion->parent = smart_motion;
                    nn_->add(motion);

                    nmotion = motion;
                }
                else
                {
                    {
                        std::fstream file;
                        file.open(filepath, std::fstream::app);
                        file << false << "," << false << ",\n";
                        file.close();
                    }
                    std::cout << "Smart state is INVALID*******" << std::endl;
                }

                auto vals2 = smart_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                std::cout << "OUT: [";
                for (int i = 0; i < si_->getStateDimension() - 1; i++)
                {
                    std::cout << vals2[i] << ", ";
                }
                std::cout << vals2[si_->getStateDimension() - 1] << "]" << std::endl;

//                double dist = 0.0;
//                sat = goal->isSatisfied(smart_motion->state, &dist);
            }
            // UNIQUE STUFF END

            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
            }
        }
        else
        {
            {
                auto random_vals = dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                std::fstream file;
                file.open(filepath, std::fstream::app);
                file << false << ",";
                for (int i = 0; i < si_->getStateDimension(); i++)
                    file << random_vals[i] << ",";
                file << false << "," << false << ",\n";
                file.close();
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

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::geometric::RRTHybrid::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
