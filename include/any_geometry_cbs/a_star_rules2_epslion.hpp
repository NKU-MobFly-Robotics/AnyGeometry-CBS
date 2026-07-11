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
class AStarRulesEpsilon {
 public:
  AStarRulesEpsilon(Environment& environment, float w) : m_env(environment), m_w(w) {}
  
  bool search(const State& startState,
              PlanResult<State, Action, Cost>& solution, int id) {

    solution.states.clear();
    solution.states.push_back(std::make_pair<>(startState, 0));
    solution.actions.clear();
    solution.cost = 0;
    solution.rotation.clear();
    solution.rotation.push_back(std::make_pair<>(startState, 1));

    openSet_t openSet;
    focalSet_t focalSet; // subset of open nodes that are within suboptimality bound
    std::unordered_map<State, fibHeapHandle_t, StateHasher> stateToHeap;
    std::unordered_set<State, StateHasher> closedSet;
    std::unordered_map<State, std::tuple<State, Action, Cost, Cost, Cost>,
                       StateHasher>
        cameFrom; 
    auto handle = openSet.push(
        Node(startState, m_env.admissibleHeuristic(startState), 0, 0));
    stateToHeap.insert(std::make_pair<>(startState, handle));
    (*handle).handle = handle;

    focalSet.push(handle);

    std::vector<Neighbor<State, Action, Cost> > neighbors;
    neighbors.reserve(10);

    Cost bestFScore = (*handle).fScore;
    

    std::unordered_set<Node, NodeHash, NodeEqual> hasPoped ;
    while (!openSet.empty()) {
      Cost oldBestFScore = bestFScore;
      bestFScore = openSet.top().fScore;
      if (bestFScore > oldBestFScore) {
        auto iter = openSet.ordered_begin();
        auto iterEnd = openSet.ordered_end();
        for (; iter != iterEnd; ++iter) {
          Cost val = iter->fScore;
          if (val > oldBestFScore * m_w && val <= bestFScore * m_w) {
            const Node& n = *iter;
            focalSet.push(n.handle);
          }
          if (val > bestFScore * m_w) {
            break;
          }
        }
      }

      auto currentHandle = focalSet.top();
      Node current = *currentHandle;
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
              iter->first, std::get<4>(iter->second))); //////
          iter = cameFrom.find(std::get<0>(iter->second));
        }
        solution.states.push_back(std::make_pair<>(startState, 0));
        solution.rotation.push_back(std::make_pair<>(startState, 0));
        std::reverse(solution.states.begin(), solution.states.end());
        std::reverse(solution.actions.begin(), solution.actions.end());
        std::reverse(solution.rotation.begin(), solution.rotation.end()); 
        solution.cost = current.gScore;
        solution.fmin = openSet.top().fScore;
        return true;
        
      }
      focalSet.pop();
      if (hasPoped.find(*currentHandle) == hasPoped.end()) {
          bool contains = false;
          for (auto& node : openSet) {
              if (node.handle == currentHandle) {
                  contains = true;
                  break;
              }
          }
          if (contains) {
              openSet.erase(currentHandle);
              hasPoped.insert(*currentHandle);
          }
      }

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
                tentative_gScore + m_env.admissibleHeuristic(neighbor.state) + (neighbor.state.o == current.state.o ? 0 : 0.6);  // Lazy Rotate 
            Cost focalHeuristic = 
                current.focalHeuristic +
                m_env.focalStateHeuristic(neighbor.state, tentative_gScore) +
                m_env.focalTransitionHeuristic(current.state, neighbor.state,
                                               current.gScore,
                                               tentative_gScore) + 
                m_env.focalRotateHeuristic(current.state, neighbor.state,
                                               current.gScore,
                                               tentative_gScore);
                
            auto handle = openSet.push(Node(neighbor.state, fScore, tentative_gScore, focalHeuristic));
            (*handle).handle = handle;
            if (fScore <= bestFScore * m_w) {
              focalSet.push(handle);
            }
            stateToHeap.insert(std::make_pair<>(neighbor.state, handle));
            m_env.onDiscover(neighbor.state, fScore, tentative_gScore);
          } else {
            auto handle = iter->second;
            if (tentative_gScore >= (*handle).gScore) {
              continue;
            }
            Cost last_gScore = (*handle).gScore;
            Cost last_fScore = (*handle).fScore;
            // update f and gScore
            Cost delta = last_gScore - tentative_gScore;
            (*handle).gScore = tentative_gScore;
            (*handle).fScore -= delta;
            openSet.increase(handle);
            m_env.onDiscover(neighbor.state, (*handle).fScore,
                             (*handle).gScore);
            if ((*handle).fScore <= bestFScore * m_w &&
                last_fScore > bestFScore * m_w) {
              focalSet.push(handle);
            }
          }

          cameFrom.erase(neighbor.state);
          cameFrom.insert(std::make_pair<>(
              neighbor.state,
              std::make_tuple<>(current.state, neighbor.action, neighbor.cost,
                                tentative_gScore,neighbor.state.o)));  //////
        }
      }
    }

    return false;
  }



 private:

  struct Node;

  #ifdef USE_FIBONACCI_HEAP
    typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
    typedef typename openSet_t::handle_type fibHeapHandle_t;
  // typedef typename boost::heap::fibonacci_heap<fibHeapHandle_t,
  // boost::heap::compare<compareFocalHeuristic> > focalSet_t;
  #else
    typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
                                            boost::heap::mutable_<true> >
        openSet_t;
    typedef typename openSet_t::handle_type fibHeapHandle_t;
  // typedef typename boost::heap::d_ary_heap<fibHeapHandle_t,
  // boost::heap::arity<2>, boost::heap::mutable_<true>,
  // boost::heap::compare<compareFocalHeuristic> > focalSet_t;
  #endif


  struct Node {
    Node(const State& state, Cost fScore, Cost gScore, Cost focalHeuristic)
        : state(state), fScore(fScore), gScore(gScore), focalHeuristic(focalHeuristic) {}

    bool operator<(const Node& other) const {
      if (fScore != other.fScore) {
        return fScore > other.fScore;
      } else {
        return gScore < other.gScore;
      }
    }

    bool operator==(const Node& other) const {
      return state == other.state && fScore == other.fScore &&
             gScore == other.gScore && focalHeuristic == other.focalHeuristic;
    }
    bool operator!=(const Node& other) const { 
      return state != other.state || fScore != other.fScore ||
             gScore!= other.gScore || focalHeuristic!= other.focalHeuristic; }

    friend std::ostream& operator<<(std::ostream& os, const Node& node) {
      os << "state: " << node.state << " fScore: " << node.fScore
         << " gScore: " << node.gScore << " focal: " << node.focalHeuristic;
      return os;
    }
    State state;

    Cost fScore;
    Cost gScore;
    Cost focalHeuristic;

    fibHeapHandle_t handle;

  };

    struct NodeHash {
      std::size_t operator()(const Node& node) const {
        std::size_t stateHash = std::hash<State>()(node.state);
        std::size_t fScoreHash = std::hash<Cost>()(node.fScore);
        std::size_t gScoreHash = std::hash<Cost>()(node.gScore);
        std::size_t focalHeuristicHash = std::hash<Cost>()(node.focalHeuristic);
        std::size_t combinedHash = stateHash;
        combinedHash = combinedHash * 31 + fScoreHash;
        combinedHash = combinedHash * 31 + gScoreHash;
        combinedHash = combinedHash * 31 + focalHeuristicHash;

        return combinedHash;
      }
    };

    struct NodeEqual {
      bool operator()(const Node& lhs, const Node& rhs) const {
        return lhs == rhs;
      }
    };


  struct compareFocalHeuristic {
    bool operator()(const fibHeapHandle_t& h1,
                    const fibHeapHandle_t& h2) const {

      if ((*h1).focalHeuristic != (*h2).focalHeuristic) {
        return (*h1).focalHeuristic > (*h2).focalHeuristic;
        // } else if ((*h1).fScore != (*h2).fScore) {
        //   return (*h1).fScore > (*h2).fScore;
      } else if ((*h1).fScore != (*h2).fScore) {
        return (*h1).fScore > (*h2).fScore;
      } else {
        return (*h1).gScore < (*h2).gScore;
      }
    }
  };

#ifdef USE_FIBONACCI_HEAP
  typedef typename boost::heap::fibonacci_heap<Node> openSet_t;
  typedef typename openSet_t::handle_type fibHeapHandle_t;
#else
//   typedef typename boost::heap::d_ary_heap<Node, boost::heap::arity<2>,
//                                            boost::heap::mutable_<true> >
//       openSet_t;
//   typedef typename openSet_t::handle_type fibHeapHandle_t;
  typedef typename boost::heap::d_ary_heap<
    fibHeapHandle_t, boost::heap::arity<2>, boost::heap::mutable_<true>,
    boost::heap::compare<compareFocalHeuristic> >
    focalSet_t;
#endif

 private:
  Environment& m_env;
  float m_w;

};  
}  // namespace libMultiRobotPlanning