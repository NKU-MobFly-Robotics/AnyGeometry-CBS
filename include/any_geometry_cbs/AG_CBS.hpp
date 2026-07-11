#pragma once

#include <map>
#include <thread>
#include <atomic>
#include "a_star_rules2.hpp"

namespace libMultiRobotPlanning {


template <typename State, typename Action, typename Cost, typename Conflict,
          typename Constraints, typename Environment>
class AG_CBS {
 public:
  AG_CBS(Environment& environment) : m_env(environment) {}

  bool search(const std::vector<State>& initialStates,
              std::vector<PlanResult<State, Action, Cost> >& solution) {
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;

    Timer t1;
    double time_limit = 100;


    for (size_t i = 0; i < initialStates.size(); ++i) {
      LowLevelEnvironment llenv(m_env, i, start.constraints[i]);
      LowLevelSearch_t lowLevel(llenv);
      bool success = lowLevel.search(initialStates[i], start.solution[i], i);
      if (!success) {
        return false;
      }
      start.cost += start.solution[i].cost;
    }

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >
        open;

    auto handle = open.push(start);
    (*handle).handle = handle;

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

      HighLevelNode P = open.top();
      m_env.onExpandHighLevelNode(P.cost);

      open.pop();

      Conflict conflict;
      if (!m_env.getFirstConflict(P.solution, conflict)) {
        solution = P.solution;
        return true;
      }

      std::map<size_t, Constraints> constraints;
      // 使用多约束 (symmetric multi-constraints)
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

        LowLevelEnvironment llenv(m_env, i, newNode.constraints[i]);  
        LowLevelSearch_t lowLevel(llenv);                             
        bool success = lowLevel.search(initialStates[i], newNode.solution[i], i);
   
        newNode.cost += newNode.solution[i].cost;

        if (success) {
          auto handle = open.push(newNode);
          (*handle).handle = handle;
        }

        ++id;
      }
    }

    return false;
  }
    struct HighLevelNode {
    std::vector<PlanResult<State, Action, Cost> > solution;
    std::vector<Constraints> constraints;

    Cost cost;

    int id;

    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;

    bool operator<(const HighLevelNode& n) const {
      return cost > n.cost;
    }

    bool hasSameConstraints(size_t agentIdx, const Constraints& otherConstraints) const {
    return constraints[agentIdx] == otherConstraints;
    }

    friend std::ostream& operator<<(std::ostream& os, const HighLevelNode& c) {
      os << "id: " << c.id << " cost: " << c.cost << std::endl;
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

 private:


  struct LowLevelEnvironment {
    LowLevelEnvironment(Environment& env, size_t agentIdx,
                        const Constraints& constraints)
        : m_env(env)
    {
      m_env.setLowLevelContext(agentIdx, &constraints);
    }

    Cost admissibleHeuristic(const State& s) {
      return m_env.admissibleHeuristic(s);
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
  };

 private:
  Environment& m_env;
  typedef AStar<State, Action, Cost, LowLevelEnvironment> LowLevelSearch_t;
};

}  // namespace libMultiRobotPlanning
