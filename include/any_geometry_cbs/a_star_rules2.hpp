#pragma once

#ifdef USE_FIBONACCI_HEAP
#include <boost/heap/fibonacci_heap.hpp>
#endif

#include <boost/heap/d_ary_heap.hpp>
#include <unordered_map>
#include <unordered_set>

#include "neighbor.hpp"
#include "planresult.hpp"

namespace libMultiRobotPlanning {


template <typename State, typename Action, typename Cost, typename Environment,
          typename StateHasher = std::hash<State> >
class AStar {
 public:
  AStar(Environment& environment) : m_env(environment) {}

  bool search(const State& startState,
              PlanResult<State, Action, Cost>& solution,  int id, Cost initialCost = 0) {
    solution.states.clear();
    solution.states.push_back(std::make_pair<>(startState, 0));
    solution.actions.clear();
    solution.cost = 0;
    solution.rotation.clear();
    solution.rotation.push_back(std::make_pair<>(startState, 1));

    openSet_t openSet;
    std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToHeap;
    std::unordered_set<State, StateHasher> closedSet;
    std::unordered_map<State, std::tuple<State, Action, Cost, Cost, Cost>,
                       StateHasher>
        cameFrom; 

    auto handle = openSet.push(
        Node(startState, m_env.admissibleHeuristic(startState), initialCost));
    stateToHeap.insert(std::make_pair<>(startState, handle));
    (*handle).handle = handle;

    std::vector<Neighbor<State, Action, Cost> > neighbors;
    neighbors.reserve(10);

    while (!openSet.empty()) {
      Node current = openSet.top();
      m_env.onExpandNode(current.state, current.fScore, current.gScore);
      
      if (m_env.isSolution(current.state)) {
        solution.states.clear();
        solution.actions.clear();
        solution.rotation.clear();
        auto iter = cameFrom.find(current.state);
        while (iter != cameFrom.end()) {
          solution.states.push_back(
              std::make_pair<>(iter->first, std::get<3>(iter->second)));
          solution.actions.push_back(std::make_pair<>(
              std::get<1>(iter->second), std::get<2>(iter->second)));
          solution.rotation.push_back(std::make_pair<>(
              iter->first, std::get<4>(iter->second))); 
          iter = cameFrom.find(std::get<0>(iter->second));
        }
        solution.states.push_back(std::make_pair<>(startState, initialCost));
        solution.rotation.push_back(std::make_pair<>(startState, 0));
        std::reverse(solution.states.begin(), solution.states.end());
        std::reverse(solution.actions.begin(), solution.actions.end());
        std::reverse(solution.rotation.begin(), solution.rotation.end()); 
        solution.cost = current.gScore;
        solution.fmin = current.fScore;

        return true;
      }

      openSet.pop();
      stateToHeap.erase(current.state);
      closedSet.insert(current.state);

      neighbors.clear();
      m_env.getNeighbors(current.state, neighbors, id);
      for (const Neighbor<State, Action, Cost>& neighbor : neighbors) {
        if (closedSet.find(neighbor.state) == closedSet.end()) {
          Cost tentative_gScore = current.gScore + neighbor.cost;
          auto iter = stateToHeap.find(neighbor.state);
          if (iter == stateToHeap.end()) {  // Discover a new node
            Cost fScore =
                tentative_gScore + m_env.admissibleHeuristic(neighbor.state) + (neighbor.state.o == current.state.o ? 0 : 0.6);  // Lazy Rotate 如果旋转，代价会暂时增加0.6
            auto handle =
                openSet.push(Node(neighbor.state, fScore, tentative_gScore));
            (*handle).handle = handle;
            stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
            m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
          } else {
            auto handle = iter->second;
            if (tentative_gScore >= (*handle).gScore) {
              continue;
            }

            Cost delta = (*handle).gScore - tentative_gScore;
            (*handle).gScore = tentative_gScore;
            (*handle).fScore -= delta;
            openSet.increase(handle);
            m_env.onDiscover(neighbor.state, (*handle).fScore,
                             (*handle).gScore);
          }

          cameFrom.erase(neighbor.state);
          cameFrom.insert(std::make_pair<>(
              neighbor.state,
              std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                tentative_gScore, 1)));  
        }
      }
    }

    return false;
  }

 private:
  struct Node {
    Node(const State& state, Cost fScore, Cost gScore)
        : state(state), fScore(fScore), gScore(gScore) {}

    bool operator<(const Node& other) const {
      // Our heap is a maximum heap, so we invert the comperator function here
      if (fScore != other.fScore) {
        return fScore > other.fScore;
      } else {
        return gScore < other.gScore;
      }
    }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state << " fScore: " << node.fScore
         << " gScore: " << node.gScore;
      return os;
    }

    State state;

    Cost fScore;
    Cost gScore;

#ifdef USE_FIBONACCI_HEAP
    typename boost::heap::fibonacci_heap<Node>::handle_type handle;
#else
    typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                     boost::heap::mutable_<true> >::handle_type
        handle;
#endif
  };

#ifdef USE_FIBONACCI_HEAP
  typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;
#else
  typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                           boost::heap::mutable_<true> >
      openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;
#endif

 private:
  Environment& m_env;
};

}  // namespace libMultiRobotPlanning
