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

#include "ompl/geometric/planners/rrt/NewPlanner.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/String.h"
#include <math.h>
#include <thread>
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/util/GeometricEquations.h"


ompl::geometric::NewPlanner::NewPlanner(const base::SpaceInformationPtr &si, bool addIntermediateStates)
    : base::Planner(si, addIntermediateStates ? "NewPlannerIntermediate" : "NewPlanner")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &NewPlanner::setRange, &NewPlanner::getRange, "0.:1.:10000.");
    Planner::declareParam<bool>("intermediate_states", this, &NewPlanner::setIntermediateStates,
                                &NewPlanner::getIntermediateStates, "0,1");

    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    addIntermediateStates_ = addIntermediateStates;
    rejected_motions_ = new std::vector<Motion*>();



//    additional_motions_ = new std::vector<Motion>();
}

ompl::geometric::NewPlanner::~NewPlanner()
{
    freeMemory();
}

void ompl::geometric::NewPlanner::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    if(!rejected_motions_)
      rejected_motions_ = new std::vector<Motion*>();

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tRejected_)
        tRejected_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!rejected_motions_)
      rejected_motions_ = new std::vector<Motion*>();

//    if(!compareFn)
//      compareFn = new CostIndexCompare(costs, *opt_);

    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tRejected_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

}

void ompl::geometric::NewPlanner::freeMemory()
{
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }

    if (rejected_motions_)
    {
        for (auto &motion: *rejected_motions_)
            delete motion;
        rejected_motions_->clear();
    }


}

void ompl::geometric::NewPlanner::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    if (rejected_motions_)
      rejected_motions_->clear();

}

ompl::geometric::NewPlanner::GrowState ompl::geometric::NewPlanner::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                             Motion *rmotion)
{
    std::vector<Motion*> nearestMotions; // = new std::vector<Motion*>();
    std::vector<base::Cost> incCosts;
    std::vector<std::size_t> sortedCostIndices;
    std::size_t removeIndex{0};
    base::Cost sampleCost;
    Motion *nmotion = nullptr;

   // std::unique_lock<std::mutex> lck(mtx_tree_);
    tree->nearestR(rmotion, tgi.quasiR, nearestMotions);
    //lck.unlock();

    base::State *dstate = rmotion->state;
    bool validState = si_->isValid(dstate);
    bool reach = true;
    if (nearestMotions.empty())
    {

      if (validState) {
          if (tgi.rejected) {
            addToRejectedMotion(dstate);
          } else { // State is rejected if it is not connected either to start and goal tree
            tgi.rejected = true;
          }
        }
        return REJECTED;
    }
    else
    {
      costs.clear();
      costs.resize(nearestMotions.size());
//      costs.shrink_to_fit();
      incCosts.resize(nearestMotions.size());
//      incCosts.shrink_to_fit();
      sortedCostIndices.resize(nearestMotions.size());
//      sortedCostIndices.shrink_to_fit();
//      CostIndexCompare compare(costs, *opt_);
      bool validMotion = false;
      for (std::size_t i = 0; i < nearestMotions.size(); ++i)
      {
        incCosts[i] =pdef_->getOptimizationObjective()->motionCost(nearestMotions[i]->state, dstate);
        costs[i] = pdef_->getOptimizationObjective()->combineCosts(nearestMotions[i]->cost, incCosts[i]);
      }
      for (std::size_t i = 0; i < nearestMotions.size(); ++i)
        sortedCostIndices[i] = i;


      std::sort(sortedCostIndices.begin(), sortedCostIndices.end(), *(compareFn));




      for (std::vector<std::size_t>::const_iterator i = sortedCostIndices.begin();
           i != sortedCostIndices.end(); i++)
      {
        validMotion = tgi.start ? si_->checkMotion(nearestMotions[*i]->state, dstate) :
                        validState && si_->checkMotion(dstate, nearestMotions[*i]->state);

        if (validMotion)
        {
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, dstate);
          motion->parent = nearestMotions[*i];
          motion->root = nearestMotions[*i]->root;
          motion->cost = costs[*i];
          tree->add(motion);
          tgi.xmotion = motion;
          sampleCost = costs[*i];
          removeIndex++;
          nmotion = motion;
          break;
        }

      }

      sortedCostIndices.erase(sortedCostIndices.begin(),sortedCostIndices.begin()+removeIndex);
      incCosts.erase(incCosts.begin(),incCosts.begin()+removeIndex);
   //   nearestMotions.erase(nearestMotions.begin(),nearestMotions.begin()+removeIndex);
      costs.erase(costs.begin(),costs.begin()+removeIndex);
      if(sortedCostIndices.size() > 1  && nearestMotions.size() > 1 && costs.size() > 1 && incCosts.size() > 1)
      {
        for (std::size_t i = 0; i < std::min(static_cast<int>(sortedCostIndices.size()), 5); ++i)
        {
//          incCosts[i] =  pdef_->getOptimizationObjective()->motionCost(nearestMotions[i]->state, dstate);
          costs[i] =  pdef_->getOptimizationObjective()->combineCosts(sampleCost, incCosts[i]);
          OMPL_INFORM("Mniejszy koszt?");
          if(opt_->isCostBetterThan((costs[i]), nearestMotions[i]->cost))
          {
            OMPL_INFORM("TAK LEPSZY KOSZT !!Mniejszyszt?");
            if(si_->checkMotion(nearestMotions[i]->state, dstate))
            {
              OMPL_INFORM("dodano mniejszy koszt?");
              nearestMotions[i]->parent = nmotion;
              nearestMotions[i]->cost = costs[i];
            }
          }
        }
      }

      /* assume we can reach the state we go towards */
      if (!validMotion)
      {
        if (validState) {
          if (tgi.rejected) {
            addToRejectedMotion(dstate);
          } else { // State is rejected if it is not connected either to start and goal tree
            tgi.rejected = true;
          }
        }
        return TRAPPED;
      }
      /* find state to add */
//      double d = si_->distance(nmotion->state, rmotion->state);
//      if (d > maxDistance_) {
//        si_->getStateSpace()->interpolate(nmotion->state, rmotion->state, maxDistance_ / d, tgi.xstate);
//
//        /* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
//         * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it
//         * thinks it is making progress, when none is actually occurring. */
//        if (si_->equalStates(nmotion->state, tgi.xstate))
//          return TRAPPED;
//
//        dstate = tgi.xstate;
//        reach = false;
//      }
//
//      if (addIntermediateStates_) {
//        const base::State *astate = tgi.start ? nmotion->state : dstate;
//        const base::State *bstate = tgi.start ? dstate : nmotion->state;
//
//        std::vector<base::State *> states;
//        const unsigned int count = si_->getStateSpace()->validSegmentCount(astate, bstate);
//
//        if (si_->getMotionStates(astate, bstate, states, count, true, true))
//          si_->freeState(states[0]);
//
//        for (std::size_t i = 1; i < states.size(); ++i) {
//          auto *motion = new Motion;
//          motion->state = states[i];
//          motion->parent = nmotion;
//          motion->root = nmotion->root;
//          tree->add(motion);
//          nmotion = motion;
//        }
//
//        tgi.xmotion = nmotion;
//      }
//      else {
//        auto *motion = new Motion(si_);
//        si_->copyState(motion->state, dstate);
//        motion->parent = nmotion;
//        motion->root = nmotion->root;
//        motion->cost = nmotion->cost;
//        tree->add(motion);
//
//        tgi.xmotion = motion;
//      }
    }
    nearestMotions.clear();
//    nearestMotions.shrink_to_fit();
    return reach ? REACHED : ADVANCED;
}

ompl::base::PlannerStatus ompl::geometric::NewPlanner::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

/**** TODO Move it to Setup */
    if (pdef_->hasOptimizationObjective())
        opt_ = pdef_->getOptimizationObjective();
    else
    {
      OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                  "planning time.",
                  getName().c_str());
        opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
        pdef_->setOptimizationObjective(opt_);
    }
    if(!compareFn)
      compareFn.reset(new CostIndexCompare(costs, *opt_));// std::unique_ new CostIndexCompare(costs, *opt_);
/****/
    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        motion->cost = opt_->identityCost();
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));


    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool startTree = true;
    bool solved = false;
    double dim = static_cast<double>(si_->getStateDimension());
    double ballVolumeUnit = calculateUnitBallVolume(dim);
    assert(ballVolumeUnit > 0 && dim > 0);
//    double gammaval = (0.75*maxDistance_)* 2.0 * powf((1.0 + (1.0/dim)),(1.0/dim)) * powf((1.0/ballVolumeUnit),(1.0/dim));
    double gammaval = (0.75*maxDistance_)* 2.0 * powf((1.0 + (1.0/dim)),(1.0/dim)) * powf((1.0/ballVolumeUnit),(1.0/dim));

    std::vector<Motion*> reuse_motions_ ;
//

    int iterration = 0;
//    std::promise<void> exitSignal;
//    std::future<void> futureObj = exitSignal.get_future();
//    std::thread th(&NewPlanner::checkForNewMotion, this, std::move(futureObj), &reuse_motions_);
    bool test = false;
  OMPL_INFORM("SampleCount?? %i", tgi.sampleCount);
  tgi.sampleCount = 2; // For the benchmark for some reason

    while (!ptc)
    {

        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                motion->cost = opt_->identityCost();
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }
        /* sample random state */
        tgi.rejected = false;
        if (iterration < 10)
          sampler_->sampleUniform(rstate);
        else
        {
          if (!rejected_motions_->empty())
          {
            if (rejectedReuse(&reuse_motions_)) {
            //    std::unique_lock<std::mutex> lck_r(mtx_rejected_);
                OMPL_WARN("REuse!!!!!!!!!!1");
                si_->copyState(rstate, reuse_motions_[0]->state);
                delete *reuse_motions_.begin();
                reuse_motions_.erase(reuse_motions_.begin());
              //  lck_r.unlock();
              }else
                sampler_->sampleUniform(rstate);

          }else
            sampler_->sampleUniform(rstate);

          iterration = 0;
        }
      iterration++;
//

      tgi.quasiR = gammaval * powf((std::log(tgi.sampleCount) / tgi.sampleCount), (1.0 / dim));
//      tgi.quasiR = gammaval * powf(tgi.sampleCount,(-1.0/dim))*powf(log(tgi.sampleCount),(1.0/dim));
        if(!test) {
          OMPL_WARN("R2 : %f ", tgi.quasiR);
          test = true;
        }
          GrowState gs = growTree(tree, tgi, rmotion);

          if (gs == TRAPPED || gs == REJECTED)
              continue;
//        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need to copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);
            if (gs == REACHED)
              tgi.sampleCount++;

            GrowState gsc = ADVANCED;
            tgi.start = startTree;
            gsc = growTree(otherTree, tgi, rmotion);


          if(gsc == REJECTED || gs == REJECTED || gs == TRAPPED)
          {
            continue;
          }
           tgi.sampleCount++;


            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
            Motion *goalMotion = startTree ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                // on the solution path

              if (startMotion->parent != nullptr)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion *> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                auto path(std::make_shared<PathGeometric>(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (auto &i : mpath2)
                    path->append(i->state);

                pdef_->addSolutionPath(path, false, 0.0, getName());
                solved = true;
                break;
            }
            else
            {
                // We didn't reach the goal, but if we were extending the start
                // tree, then we can mark/improve the approximate path so far.
                if (!startTree)
                {
                    // We were working from the startTree.
                    double dist = 0.0;
                    goal->isSatisfied(tgi.xmotion->state, &dist);
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = tgi.xmotion;
                    }
                }
            }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;
    //exitSignal.set_value();
    //th.join();
    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());
//  OMPL_WARN("Size of rejected samples: ( %i )", rejected_motions_->size());

    if (approxsol && !solved)
    {
        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (approxsol != nullptr)
        {
            mpath.push_back(approxsol);
            approxsol = approxsol->parent;
        }

        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path, true, approxdif, getName());
      return base::PlannerStatus::APPROXIMATE_SOLUTION;
    }

    for (auto &motion : reuse_motions_)
    {
       delete motion; ;
    }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::NewPlanner::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    // Add some info.
    data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}

double ompl::geometric::NewPlanner::calculateUnitBallVolume(int dim)
{
    if (dim == 0)
        return 1;
    else if (dim == 1)
        return 2;
    return ((2 * M_PI)/dim)*calculateUnitBallVolume(dim-2);
}

void ompl::geometric::NewPlanner::checkForNewMotion(std::future<void> futureObj,std::vector<Motion*> * reuseMotions) {

//  std::vector<Motion*> sRejected;
//  std::vector<Motion*> gRejected;
  std::vector<Motion *> local_rejected_;
  bool connectionPoint = false;

  std::size_t rMotionSize = 0;
  while (futureObj.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout) {


    if (!rejected_motions_->empty()) {
      rMotionSize = rejected_motions_->size();
      for (std::size_t index = 0; index < rMotionSize; index++) {
////        tGoal_->nearestR(*iter,tgi.quasiR, gRejected); // tgi.QuasiR data race - any way there will be one more time checked in main thread
////        tGoal_->nearestR(*iter,tgi.quasiR, sRejected);
        std::unique_lock<std::mutex> lck(mtx_tree_);
        Motion *gRejected = tGoal_->nearest((*rejected_motions_)[index]);
        Motion *sRejected = tStart_->nearest((*rejected_motions_)[index]);
        lck.unlock();
        connectionPoint = false;
        if (gRejected != nullptr) {
          if (si_->distance(gRejected->state, (*rejected_motions_)[index]->state) < tgi.quasiR) {
            if (si_->checkMotion(gRejected->state, (*rejected_motions_)[0]->state)) {
              std::unique_lock<std::mutex> lck_r(mtx_rejected_);
              reuseMotions->push_back(std::move((*rejected_motions_)[index]));
              rejected_motions_->erase(rejected_motions_->begin() + index);
              rMotionSize = rMotionSize - 1;
              connectionPoint = true;
              lck_r.unlock();
            }
          }
        }

        if (sRejected != nullptr) {
          if (si_->distance(sRejected->state, (*rejected_motions_)[index]->state) < tgi.quasiR) {
            if (si_->checkMotion(sRejected->state, (*rejected_motions_)[0]->state)) {
              if (!connectionPoint) {
                std::unique_lock<std::mutex> lck_r(mtx_rejected_);
                reuseMotions->push_back(std::move((*rejected_motions_)[index]));
                rejected_motions_->erase(rejected_motions_->begin() + index);
                rMotionSize = rMotionSize - 1;
                lck_r.unlock();
              } else
                reuseMotions->insert(reuseMotions->begin(), std::move((*rejected_motions_)[index]));
            }
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
//      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
  }
  reuseMotions->clear();
//  reuseMotions->shrink_to_fit();
}

void ompl::geometric::NewPlanner::addToRejectedMotion(base::State *state) {

//  if (rejected_motions_->size() > 1)
//  {
//    auto iter = std::find_if(rejected_motions_->begin(), rejected_motions_->end(),
//                             [&](Motion * rMotion){return si_->equalStates(rMotion->state,state);});
//
//    if (iter == rejected_motions_->end())
//    {
//      OMPL_WARN("Rejected added");
//      auto *rejectedMotion = new Motion(si_);
//      si_->copyState(rejectedMotion->state, state);
//      rejectedMotion->cost = opt_->identityCost();
//      rejected_motions_->push_back(rejectedMotion);
//    }
//  }else
//  {
//    mtx.try_lock();
//    std::unique_lock<std::mutex> lck_r(mtx_rejected_);

    auto *rejectedMotion = new Motion(si_);
    si_->copyState(rejectedMotion->state, state);
    rejectedMotion->cost = opt_->identityCost();
    rejected_motions_->push_back(rejectedMotion);
//    lck_r.unlock();


//  mtx.unlock();
//  }
}

bool ompl::geometric::NewPlanner::rejectedReuse(std::vector<Motion*> *reuseMotions) {

//  std::vector<Motion*> sRejected;
//  std::vector<Motion*> gRejected;
  bool connectionPoint = false;
  std::size_t rMotionSize = 0;
  bool status = false;
  OMPL_WARN("REUSE ATTEMPT");

    if (!rejected_motions_->empty()) {
      rMotionSize = rejected_motions_->size();
      for (std::size_t index = 0; index < rMotionSize; index++) {
////        tGoal_->nearestR(*iter,tgi.quasiR, gRejected); // tgi.QuasiR data race - any way there will be one more time checked in main thread
////        tGoal_->nearestR(*iter,tgi.quasiR, sRejected);
//        std::unique_lock<std::mutex> lck(mtx_tree_);
        Motion *gRejected = tGoal_->nearest((*rejected_motions_)[index]);
        Motion *sRejected = tStart_->nearest((*rejected_motions_)[index]);
//        lck.unlock();
        connectionPoint = false;
        if (gRejected != nullptr) {
          if (si_->distance(gRejected->state, (*rejected_motions_)[index]->state) < tgi.quasiR) {
            if (si_->checkMotion(gRejected->state, (*rejected_motions_)[0]->state)) {
         //     std::unique_lock<std::mutex> lck_r(mtx_rejected_);
              reuseMotions->push_back(std::move((*rejected_motions_)[index]));
              rejected_motions_->erase(rejected_motions_->begin() + index);
              rMotionSize = rMotionSize - 1;
              connectionPoint = true;
          //    lck_r.unlock();
            }
          }
        }

        if (sRejected != nullptr) {
          if (si_->distance(sRejected->state, (*rejected_motions_)[index]->state) < tgi.quasiR) {
            if (si_->checkMotion(sRejected->state, (*rejected_motions_)[0]->state)) {
              if (!connectionPoint) {
//                std::unique_lock<std::mutex> lck_r(mtx_rejected_);
                reuseMotions->push_back(std::move((*rejected_motions_)[index]));
                rejected_motions_->erase(rejected_motions_->begin() + index);
                rMotionSize = rMotionSize - 1;
//                lck_r.unlock();
              }// else
//                reuseMotions->insert(reuseMotions->begin(), std::move((*rejected_motions_)[index]));
            }
          }
        }
        if(sRejected != nullptr || gRejected != nullptr)
            status = true;
//        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
//      std::this_thread::sleep_for(std::chrono::milliseconds(200));

    }
  }
//  reuseMotions->shrink_to_fit();
