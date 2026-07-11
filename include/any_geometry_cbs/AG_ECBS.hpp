#pragma once

#include <map>

#include "a_star_rules2_epslion.hpp"

namespace libMultiRobotPlanning {


template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class AG_ECBS {
 public:
  AG_ECBS(Environment& environment, float w) : m_env(environment), m_w(w) {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost> >& solution) {
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.LB = 0;
    start.id = 0;

    Timer t1;
    double time_limit = 100;

    for (size_t i = 0; i < initialStates.size(); ++i) {
      if (i < solution.size() && solution[i].states.size() > 1) {
        start.solution[i] = solution[i];
      } else {
        LowLevelEnvironment llenv(m_env, i, start.constraints[i], start.solution);
        LowLevelSearch_t lowLevel(llenv, m_w);
        
        bool success = lowLevel.search(initialStates[i], start.solution[i], i);
        if (!success) {
          return false;
        }
      }
      start.cost += start.solution[i].cost;
      start.LB += start.solution[i].fmin;
    }
    start.focalHeuristic = m_env.focalHeuristic(start.solution); 
    
    openSet_t open;
    focalSet_t focal;

    auto handle = open.push(start);
    (*handle).handle = handle;
    focal.push(handle);
    Cost bestCost = (*handle).cost;

    solution.clear();
    int id = 1;

    t1.stop();
    double t1_time = t1.elapsedSeconds();
    Timer t2;

    while (!open.empty()) {

      t2.stop();
      if (t2.elapsedSeconds() + t1_time > time_limit) {
        return false;
      }
      t1_time = t2.elapsedSeconds() + t1_time;
      t2.reset();

      Cost oldBestCost = bestCost;
      bestCost = open.top().cost;
      if (bestCost > oldBestCost) {
        auto iter = open.ordered_begin();
        auto iterEnd = open.ordered_end();
        for (; iter != iterEnd; ++iter) {
          Cost val = iter->cost;
          if (val > oldBestCost * m_w && val <= bestCost * m_w) {
            const HighLevelNode& n = *iter;
            focal.push(n.handle);
          }
          if (val > bestCost * m_w) {
            break;
          }
        }
      }

      auto h = focal.top();
      HighLevelNode P = *h;

      m_env.onExpandHighLevelNode(P.cost);

      focal.pop();
      open.erase(h);

      Conflict conflict;
      // 使用角点子块数量启发式选择冲突
      if (!m_env.getHeuristic1Conflict(P.solution, conflict)) {
        solution = P.solution;
        return true;
      }

      std::map<size_t, Constraints> constraints;
      m_env.createSYMMultiConstraintsFromConflict(conflict, constraints); 

      for (const auto& c : constraints) {  

        size_t i = c.first;
        if (conflict.agent1 == i) {
            if (m_env.isAgentAtGoal(conflict.x1, conflict.y1, i)) {
              continue;
            }
          }
        else if (conflict.agent2 == i) {
            if (m_env.isAgentAtGoal(conflict.x2, conflict.y2, i)) {
              continue;
            }
        }
        HighLevelNode newNode = P; 
        newNode.id = id; 
        
        Constraints c_temp = c.second;
        newNode.constraints[i].DuplicateMemberCheck(c_temp);

        assert(!newNode.constraints[i].overlap(c_temp));

        newNode.constraints[i].add(c_temp);  

        newNode.cost -= newNode.solution[i].cost;
        newNode.LB -= newNode.solution[i].fmin;

        LowLevelEnvironment llenv(m_env, i, newNode.constraints[i],
                                  newNode.solution); 
        LowLevelSearch_t lowLevel(llenv, m_w);                            
        bool success = lowLevel.search(initialStates[i], newNode.solution[i], i);
        newNode.cost += newNode.solution[i].cost;
        newNode.LB += newNode.solution[i].fmin;
        newNode.focalHeuristic = m_env.focalHeuristic(newNode.solution);


        if (success) {
          auto handle = open.push(newNode);
          (*handle).handle = handle;
          if (newNode.cost <= bestCost * m_w) {
            focal.push(handle);
          }
        }

        ++id;
      }
    }

    return false;
  }
    
 private:

    struct HighLevelNode;

    typedef typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                           boost::heap::mutable_<true> >
      openSet_t;
    typedef typename openSet_t::handle_type handle_t;
    
    struct HighLevelNode {
    std::vector<PlanResult<State, Action, Cost> > solution;
    std::vector<Constraints> constraints;

    Cost cost;
    Cost LB;  // sum of fmin of solution
    Cost focalHeuristic;
    int id;
    handle_t handle;

    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }

    bool hasSameConstraints(size_t agentIdx, const Constraints& otherConstraints) const {
    return constraints[agentIdx] == otherConstraints;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << " LB: " << c.LB
         << " focal: " << c.focalHeuristic << std::endl;
      for (size_t i = 0; i < c.solution.size(); ++i) {
        os << "Agent: " << i << std::endl;
        os << " States:" << std::endl;
        for (size_t t = 0; t < c.solution[i].states.size(); ++t) {
          os << "  " << c.solution[i].states[t].first << std::endl;
        }
        os << " Constraints:" << std::endl;
        os << c.constraints[i];
        os << " cost: " << c.solution[i].cost << std::endl;
      }
      return os;
    }
  };

  struct compareFocalHeuristic {
    bool operator()(const handle_t& h1, const handle_t& h2) const {
      if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
        return (*h1).focalHeuristic > (*h2).focalHeuristic;
      }
      return (*h1).cost > (*h2).cost;
    }
  };

  typedef typename boost::heap::d_ary_heap<
      handle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
      boost::heap::compare<compareFocalHeuristic> >
      focalSet_t;

  struct LowLevelEnvironment {
    LowLevelEnvironment(
        Environment& env, size_t agentIdx, const Constraints& constraints,
        const std::vector<PlanResult<State, Action, Cost> >& solution)
        : m_env(env),
          m_solution(solution) {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
    }

    Cost focalStateHeuristic(const State& s, Cost gScore) {
      return m_env.focalStateHeuristic(s, gScore, m_solution);
    }

    Cost focalTransitionHeuristic(const State& s1, const State& s2,
                                  Cost gScoreS1, Cost gScoreS2) {
      return m_env.focalTransitionHeuristic(s1, s2, gScoreS1, gScoreS2,
                                            m_solution);
    }

    Cost focalRotateHeuristic(const State& s1, const State& s2,
                                  Cost gScoreS1, Cost gScoreS2) {
      return m_env.focalRotateHeuristic(s1, s2, gScoreS1, gScoreS2,
                                            m_solution);
    }

    Cost focalConflictAgentsHeuristic(const State& s, const State& s1, const State& s2,
                                  Cost gScoreS1, Cost gScoreS2) {
      return m_env.focalConflictAgentsHeuristic(s, s1, s2, gScoreS1, gScoreS2,
                                            m_solution);
    }


    bool isSolution(const State& s) { return m_env.isSolution(s); }

    void getNeighbors(const State& s,
                      std::vector<Neighbor<State, Action, Cost> >& neighbors, size_t agentIdx) {
      m_env.getNeighbors(s, neighbors, agentIdx);
    }

    void onExpandNode(const State& s, Cost fScore, Cost gScore) {
      m_env.onExpandLowLevelNode(s, fScore, gScore);
    }

    void onDiscover(const State& /*s*/, Cost /*fScore*/, Cost /*gScore*/) {
    }

   private:
    Environment& m_env;
    const std::vector<PlanResult<State, Action, Cost> >& m_solution;
  };

 private:
  Environment& m_env;
  float m_w;
  typedef AStarRulesEpsilon<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
