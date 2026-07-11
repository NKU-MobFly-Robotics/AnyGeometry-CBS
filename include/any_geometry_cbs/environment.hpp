#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>
#include <string>
#include <map>
#include <fstream>
#include <chrono>
#include <utility>
#include <set>

#include <boost/functional/hash.hpp>
#include <Eigen/Eigen>

#include "AG_CBS.hpp"

using namespace libMultiRobotPlanning;

struct State {
  State(int time, int x, int y, int o) : time(time), x(x), y(y), o(o) {}

  bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y && o == s.o;
  }

  bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << ": (" << s.x << "," << s.y << "," << s.o << ")";
  }

  int time;
  int x;
  int y;
  int o;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.o);
    return seed;
  }
};
}  // namespace std

enum class Action {
  Up_1,
  Down_1,
  Left_1,
  Right_1,
  Wait_1,
  Up_0,
  Down_0,
  Left_0,
  Right_0,
  Wait_0,
  Up_2,
  Down_2,
  Left_2,
  Right_2,
  Wait_2,
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up_1:
      os << "Up_1";
      break;
    case Action::Down_1:
      os << "Down_1";
      break;
    case Action::Left_1:
      os << "Left_1";
      break;
    case Action::Right_1:
      os << "Right_1";
      break;
    case Action::Wait_1:
      os << "Wait_1";
      break;
    case Action::Up_0:
      os << "Up_0";
      break;
    case Action::Down_0:
      os << "Down_0";
      break;
    case Action::Left_0:
      os << "Left_0";
      break;
    case Action::Right_0:
      os << "Right_0";
      break;
    case Action::Wait_0:
      os << "Wait_0";
      break;
    case Action::Up_2:
      os << "Up_2";
      break;
    case Action::Down_2:
      os << "Down_2";
      break;
    case Action::Left_2:
      os << "Left_2";
      break;
    case Action::Right_2:
      os << "Right_2";
      break;
    case Action::Wait_2:
      os << "Wait_2";
      break;

  }
  return os;
}

///

struct Conflict { 
  enum Type {
    Vertex,
    Edge,
    ShapeV,
    ShapeE,
    RotateC, 
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;
  int time1;
  int time2;

  int x1;
  int y1;
  int o1; 
  int i1; 
  int x2;
  int y2;
  int o2;
  int i2;
  int data1[6];
  int rc_type;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << "," << c.o1  << "," << c.i1 << ")";
      case Edge:
        return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << ","  << c.o1 << "," << c.x2
                  << "," << c.y2 << "," << c.o2<< ")";
      case ShapeV:
        return os << c.time << ": ShapeV-xyoi(" << c.x1 << "," << c.y1 << "," << c.o1 << "," << c.i1 << "," << c.x2 
                  << "," << c.y2 << "," << c.o2 << "," << c.i2<< ")";
      case ShapeE:
        return os << c.time << ": ShapeE-xyoi(" << c.x1 << "," << c.y1 << "," << c.o1 << "," << c.i1 << "," << c.x2
                  << "," << c.y2 << "," << c.o2 << "," << c.i2 << ")";
      case RotateC:
        return os << c.time1 << " " << c.time2 << ": RotateC-xyoi(" << c.x1 << "," << c.y1 << "," << c.o1 << "," << c.i1 << "," << c.x2
                  << "," << c.y2 << "," << c.o2 << "," << c.i2 << ")";
    }
    return os;
  }
};

struct VertexConstraint {
  VertexConstraint(int time, int x, int y, int o, int i) : time(time), x(x), y(y), o(o), i(i) {}
  int time;
  int x;
  int y;
  int o;
  int i;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, x, y, o, i) < std::tie(other.time, other.x, other.y, other.o, other.i);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, x, y, o, i) == std::tie(other.time, other.x, other.y, other.o, other.i);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    return os << "VC(" << c.time << "," << c.x << "," << c.y << "," << c.o << "," << c.i<< ")";
  }
};

namespace std {
template <>
struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.o);
    boost::hash_combine(seed, s.i);
    return seed;
  }
};
}  // namespace std

struct EdgeConstraint {
  EdgeConstraint(int time, int x1, int y1, int o1, int x2, int y2, int o2)
      : time(time), x1(x1), y1(y1), o1(o1),  x2(x2), y2(y2) , o2(o2){}
  int time;
  int x1;
  int y1;
  int o1;
  int x2;
  int y2;
  int o2;

  bool operator<(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1,o1, x2, y2, o2) <
           std::tie(other.time, other.x1, other.y1, other.o1, other.x2, other.y2, other.o2);
  }

  bool operator==(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1,o1, x2, y2, o2) ==
           std::tie(other.time, other.x1, other.y1, other.o1, other.x2, other.y2, other.o2);
  }

  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
    return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.o1 << "," << c.x2
              << "," << c.y2 << "," << c.o2<< ")";
  }
};

namespace std {
template <>
struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.o1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    boost::hash_combine(seed, s.o2);
 
    return seed;
  }
};
}  // namespace std

struct ShapeVConstraint {
  ShapeVConstraint(int time, int x1, int y1, int o1, int i1, int x2, int y2, int o2, int i2)
      : time(time), x1(x1), y1(y1), o1(o1), i1(i1), x2(x2), y2(y2), o2(o2), i2(i2){}
  int time;
  int x1;
  int y1;
  int o1;
  int i1;
  int x2;
  int y2;
  int o2;
  int i2;
  bool operator<(const ShapeVConstraint& other) const {
    return std::tie(time, x1, y1, o1, i1, x2, y2, o2, i2) <
           std::tie(other.time, other.x1, other.y1, other.o1,other.i1, other.x2, other.y2, other.o2, other.i2);
  }

  bool operator==(const ShapeVConstraint& other) const {
    return std::tie(time, x1, y1, o1, i1, x2, y2, o2, i2) ==
           std::tie(other.time, other.x1, other.y1, other.o1,other.i1, other.x2, other.y2, other.o2, other.i2);
  }

  friend std::ostream& operator<<(std::ostream& os, const ShapeVConstraint& c) {
    return os << "ShapeV(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.o1 << "," << c.i1 << ","<< c.x2
              << "," << c.y2 << "," << c.o2 << "," << c.i2 <<")";
  }
};
namespace std {
template <>
struct hash<ShapeVConstraint> {
  size_t operator()(const ShapeVConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.o1);
    boost::hash_combine(seed, s.i1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    boost::hash_combine(seed, s.o2);
    boost::hash_combine(seed, s.i2);
    return seed;
  }
};
}  // namespace std

struct ShapeEConstraint {
    ShapeEConstraint(int time, int x1a, int y1a, int x2a, int y2a, int o1, int i1, int x1b, int y1b, int x2b, int y2b, int o2, int i2)
    : time(time), x1a(x1a), y1a(y1a), x2a(x2a), y2a(y2a), o1(o1), i1(i1), x1b(x1b), y1b(y1b),
      x2b(x2b), y2b(y2b), o2(o2), i2(i2) {}
    int time, x1a, y1a, x2a, y2a, o1, i1, x1b, y1b, x2b, y2b, o2, i2;

    bool operator<(const ShapeEConstraint& other) const
    {
        return std::tie(time, x1a, y1a, x2a, y2a, o1, i1, x1b, y1b, x2b, y2b, o2, i2) <
               std::tie(other.time, other.x1a, other.y1a, other.x2a, other.y2a, other.o1, other.i1, other.x1b, other.y1b, other.x2b, other.y2b, other.o2, other.i2);  
    }
    bool operator==(const ShapeEConstraint& other) const
    {
        return std::tie(time, x1a, y1a, x2a, y2a, o1, i1, x1b, y1b, x2b, y2b, o2, i2) ==
               std::tie(other.time, other.x1a, other.y1a, other.x2a, other.y2a, other.o1, other.i1, other.x1b, other.y1b, other.x2b, other.y2b, other.o2, other.i2);
    }
    friend std::ostream& operator<<(std::ostream& os, const ShapeEConstraint& s)
    {
        return os << "ShapeE(" << s.time << ", " << s.x1a << ", " << s.y1a << ", " << s.x2a << ", " << s.y2a << ", " << s.o1 << ", " << s.i1 << ", " << s.x1b << ", " << s.y1b << ", " << s.x2b << ", " << s.y2b << ", " << s.o2 << ", " << s.i2 << ")";
    }
};
namespace std {
template<>
struct hash<ShapeEConstraint> {
    size_t operator()(const ShapeEConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1a);
    boost::hash_combine(seed, s.y1a);
    boost::hash_combine(seed, s.x2a);
    boost::hash_combine(seed, s.y2a);
    boost::hash_combine(seed, s.o1);
    boost::hash_combine(seed, s.i1);
    boost::hash_combine(seed, s.x1b);
    boost::hash_combine(seed, s.y1b);
    boost::hash_combine(seed, s.x2b);
    boost::hash_combine(seed, s.y2b);
    boost::hash_combine(seed, s.o2);
    boost::hash_combine(seed, s.i2);
    return seed;
    }
};
} //namespace std

struct RotateConstraint {
  RotateConstraint(int time1, int x1, int y1, int o1, int i1, int time2, int x2, int y2, int o2, int i2)
      : time1(time1), x1(x1), y1(y1), o1(o1), i1(i1), time2(time2), x2(x2), y2(y2), o2(o2), i2(i2){}
  int time1;
  int x1;
  int y1;
  int o1;
  int i1;
  int time2;
  int x2;
  int y2;
  int o2;
  int i2;
  bool operator<(const RotateConstraint& other) const {
    return std::tie(time1, x1, y1, o1, i1, time2, x2, y2, o2, i2) <
           std::tie(other.time1, other.x1, other.y1, other.o1, other.i1, other.time2, other.x2, other.y2, other.o2, other.i2);
  }
  bool operator==(const RotateConstraint& other) const {
    return std::tie(time1, x1, y1, o1, i1, time2, x2, y2, o2, i2) ==
           std::tie(other.time1, other.x1, other.y1, other.o1, other.i1, other.time2, other.x2, other.y2, other.o2, other.i2);
  }
  friend std::ostream
  &operator<<(std::ostream &os, const RotateConstraint &s) {
    return os << "RotateC(" << s.time1 << "," << s.x1 << "," << s.y1 << "," << s.o1 << "," << s.i1
     << "," << s.time2 << "," << s.x2 << "," << s.y2 << "," << s.o2 << "," << s.i2 << ")";
  }
};
namespace std {
  template<> struct hash<RotateConstraint> {
    size_t operator()(const RotateConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time1);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.o1);
    boost::hash_combine(seed, s.i1);
    boost::hash_combine(seed, s.time2);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    boost::hash_combine(seed, s.o2);
    boost::hash_combine(seed, s.i2);
    return seed;
  }
  };
} //namespace std

struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;
  std::unordered_set<ShapeVConstraint> shapeVConstraints;
  std::unordered_set<ShapeEConstraint> shapeEConstraints;
  std::unordered_set<RotateConstraint> rotateConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
    shapeVConstraints.insert(other.shapeVConstraints.begin(),
                            other.shapeVConstraints.end());
    shapeEConstraints.insert(other.shapeEConstraints.begin(),
                            other.shapeEConstraints.end());
    rotateConstraints.insert(other.rotateConstraints.begin(),
                            other.rotateConstraints.end());     
  }

  void DuplicateMemberCheck(Constraints& other) {
    for (auto& vc : vertexConstraints) {
      if (other.vertexConstraints.count(vc) > 0) {
        other.vertexConstraints.erase(vc);
      }
    }
    for (auto& ec : edgeConstraints) {
      if (other.edgeConstraints.count(ec) > 0) {
        other.edgeConstraints.erase(ec);
      }
    }
  }


  bool overlap(const Constraints& other) const {
    for (const auto& vc : vertexConstraints) {
      if (other.vertexConstraints.count(vc) > 0) {
        return true;
      }
    }
    for (const auto& ec : edgeConstraints) {
      if (other.edgeConstraints.count(ec) > 0) {
        return true;
      }
    }
    for (const auto& svc : shapeVConstraints) {
      if (other.shapeVConstraints.count(svc) > 0) {
        return true;
      }
    }
    for (const auto& sec : shapeEConstraints) {
      if (other.shapeEConstraints.count(sec) > 0) {
        return true;
      }
    }
    for (const auto& rc : rotateConstraints) {
      if (other.rotateConstraints.count(rc) > 0) {
        return true;
      }
    }
    return false;
  }

  //重载== 
  bool operator==(const Constraints& other) const {
    return vertexConstraints == other.vertexConstraints &&
           edgeConstraints == other.edgeConstraints &&
           shapeVConstraints == other.shapeVConstraints &&
           shapeEConstraints == other.shapeEConstraints &&
           rotateConstraints == other.rotateConstraints;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    for (const auto& svc : c.shapeVConstraints) {
      os << svc << std::endl;
    }
    for (const auto& sec : c.shapeEConstraints) {
      os << sec << std::endl;
    }
    for (const auto& rc : c.rotateConstraints) {
      os << rc << std::endl;
    }
    return os;
  }
};

struct Location {
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
  Location operator+(const Location& other) const {
    return Location(x + other.x, y + other.y);
  }

};

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

///
class  Environment {
 public:
  Environment(size_t dimx, size_t dimy, std::unordered_set<Location> obstacles,
              std::vector<Location> goals, bool disappearAtGoal = true, std::vector<std::vector<Location>> shapes = std::vector<std::vector<Location>>(),
              std::vector<std::vector<std::vector<Location>>> shapes_sheet = std::vector<std::vector<std::vector<Location>>>(), std::vector<int> h1_list = std::vector<int>())
      : m_dimx(dimx),
        m_dimy(dimy),
        m_obstacles(std::move(obstacles)),
        m_goals(std::move(goals)),
        m_agentIdx(0),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
        m_disappearAtGoal(disappearAtGoal),
        m_shapes(std::move(shapes)) ,
        m_shapes_sheet(std::move(shapes_sheet)) ,
        m_h1_list(std::move(h1_list)) //NOLINT
  {}

  Environment(const Environment&) = delete;
  Environment& operator=(const Environment&) = delete;

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints) {
    assert(constraints);  // NOLINT
    m_agentIdx = agentIdx;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    for (const auto& vc : constraints->vertexConstraints) {
      if (vc.x == m_goals[m_agentIdx].x && vc.y == m_goals[m_agentIdx].y) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
  }

  float admissibleHeuristic(const State& s) { 
    return  std::abs(s.x - m_goals[m_agentIdx].x) + std::abs(s.y - m_goals[m_agentIdx].y);
  }

  // low-level 

int focalConflictAgentsHeuristic( 
      const State& s, const State& s1a, const State& s1b,int /*gScoreS1a*/, int /*gScoreS1b*/,
      const std::vector<PlanResult<State, Action, float> >& solution) {
    int numConflicts = 0;
    int SH2 = 0;
    std::set<int> ConflictAgents;

    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State state2 = getState(i, solution, s.time);
        std::vector<Location> shape1 = m_shapes_sheet[m_agentIdx][s.o] ;
        std::vector<Location> shape2 = m_shapes_sheet[i][state2.o] ;
        if (shape_vertex_collide(s, state2, shape1, shape2)) 
        {
          ++numConflicts;
          SH2 = SH2 +  m_h1_list[i] + m_h1_list[m_agentIdx] + m_shapes_sheet[i][1].size() + m_shapes_sheet[m_agentIdx][1].size();
          ConflictAgents.insert(i);
          ConflictAgents.insert(m_agentIdx);
        }
        State s2a = getState(i, solution, s1a.time);
        State s2b = getState(i, solution, s1b.time);
        if (shape_edge_collide(s1a, s1b, s2a, s2b, m_agentIdx, i))
        {
          ++numConflicts;
          SH2 = SH2 +  m_h1_list[i] + m_h1_list[m_agentIdx] + m_shapes_sheet[i][1].size() + m_shapes_sheet[m_agentIdx][1].size();
          ConflictAgents.insert(i);
          ConflictAgents.insert(m_agentIdx);
        }
        if (s1a.o != s1b.o || s2a.o != s2b.o) {
              std::vector<State> RC_results;
              int RC_type;
              if (RC_AABB(s1a, s1b, s2a, s2b, m_agentIdx, i, RC_results, RC_type)) {
                numConflicts++;
                SH2 = SH2 +  m_h1_list[i] + m_h1_list[m_agentIdx] + m_shapes_sheet[i][1].size() + m_shapes_sheet[m_agentIdx][1].size();
                ConflictAgents.insert(i);
                ConflictAgents.insert(m_agentIdx);
              }
        }
      }
    }
    // return numConflicts;
    // return SH2;
    return ConflictAgents.size();
  }


  
  int focalStateHeuristic( 
      const State& s, int /*gScore*/,
      const std::vector<PlanResult<State, Action, float> >& solution) {
    int numConflicts = 0;
    int SH2 = 0;

    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        // State state2 = getState(i, solution, s.time);
        // if (s.equalExceptTime(state2)) {
        //   ++numConflicts;
        // }
        State state2 = getState(i, solution, s.time);
        std::vector<Location> shape1 = m_shapes_sheet[m_agentIdx][s.o] ;
        std::vector<Location> shape2 = m_shapes_sheet[i][state2.o] ;
        if (shape_vertex_collide(s, state2, shape1, shape2)) 
        {
          ++numConflicts;
          SH2 = SH2 +  m_h1_list[i] + m_h1_list[m_agentIdx] + m_shapes_sheet[i][1].size() + m_shapes_sheet[m_agentIdx][1].size();

        }
      }
    }
    // return numConflicts;
    return SH2;
  }

  // low-level
  int focalTransitionHeuristic(
      const State& s1a, const State& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
      const std::vector<PlanResult<State, Action, float> >& solution) {
    int numConflicts = 0;
    int SH2 = 0;

    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && !solution[i].states.empty()) {
        State s2a = getState(i, solution, s1a.time);
        State s2b = getState(i, solution, s1b.time);
        // if (s1a.equalExceptTime(s2b) && s1b.equalExceptTime(s2a)) {
        //   ++numConflicts;
        // }
        if (shape_edge_collide(s1a, s1b, s2a, s2b, m_agentIdx, i))
        {
          ++numConflicts;
          SH2 = SH2 +  m_h1_list[i] + m_h1_list[m_agentIdx] + m_shapes_sheet[i][1].size() + m_shapes_sheet[m_agentIdx][1].size();

        }
      }
    }
    // return numConflicts;
    return SH2;
  }

  int focalRotateHeuristic(
      const State& s1a, const State& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
      const std::vector<PlanResult<State, Action, float> >& solution){
        int numConflicts = 0;
        int SH2 = 0;

        
        for (size_t i = 0; i < solution.size(); ++i) {
           if (i != m_agentIdx && !solution[i].states.empty()) {
            State s2a = getState(i, solution, s1a.time);
            State s2b = getState(i, solution, s1b.time);
            if (s1a.o != s1b.o || s2a.o != s2b.o) {
              std::vector<State> RC_results;
              int RC_type;
              if (RC_AABB(s1a, s1b, s2a, s2b, m_agentIdx, i, RC_results, RC_type)) {
                numConflicts++;
                SH2 = SH2 +  m_h1_list[i] + m_h1_list[m_agentIdx] + m_shapes_sheet[i][1].size() + m_shapes_sheet[m_agentIdx][1].size();
              }
            }
           }
        }
      // return numConflicts;
      return SH2;
    }

  
  int focalHeuristic(
      const std::vector<PlanResult<State, Action, float> >& solution) {
    int numConflicts = 0;
    int max_t = 0;
    int SH2 = 0;
    std::set<int> ConflictAgents;

    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          // if (state1.equalExceptTime(state2)) {
          //   ++numConflicts;
          // }
          std::vector<Location> shape1 = m_shapes_sheet[i][state1.o];
          std::vector<Location> shape2 = m_shapes_sheet[j][state2.o];
          if(shape_vertex_collide(state1, state2, shape1, shape2))
          {
            ++numConflicts;
            SH2 = SH2 + (m_h1_list[i] + m_h1_list[j] + m_shapes_sheet[i][1].size() + m_shapes_sheet[j][1].size()) ;
            ConflictAgents.insert(i);
            ConflictAgents.insert(j);

          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          // if (state1a.equalExceptTime(state2b) &&
          //     state1b.equalExceptTime(state2a)) {
          //   ++numConflicts;
          // }
          if (shape_edge_collide(state1a, state1b, state2a, state2b, i, j))
          {
            ++numConflicts;
            SH2 = SH2 + (m_h1_list[i] + m_h1_list[j] + m_shapes_sheet[i][1].size() + m_shapes_sheet[j][1].size()) ;
            ConflictAgents.insert(i);
            ConflictAgents.insert(j);

          }
        }
      }
      // rotate collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.o != state1b.o || state2a.o != state2b.o) {
            std::vector<State> RC_something;
            int RC_type;
            if (RC_AABB(state1a, state1b, state2a, state2b, i, j, RC_something, RC_type))
            {
              ++numConflicts;
              SH2 = SH2 + (m_h1_list[i] + m_h1_list[j] + m_shapes_sheet[i][1].size() + m_shapes_sheet[j][1].size()) ;
              ConflictAgents.insert(i);
              ConflictAgents.insert(j);

            }
          }
        }
      }
    }
    // return numConflicts;
    return SH2;
    // return ConflictAgents.size();


  }

  bool isSolution(const State& s) {
    return s.x == m_goals[m_agentIdx].x && s.y == m_goals[m_agentIdx].y &&
           s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, float> >& neighbors, size_t agentIdx) {
    neighbors.clear();
    {
      State n(s.time + 1, s.x, s.y, s.o);
      if (stateValid(n, agentIdx) && transitionValid(s, n) ) {
        neighbors.emplace_back(
            Neighbor<State, Action, float>(n, Action::Wait_0, 1));
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y, s.o);
      if (stateValid(n, agentIdx) && transitionValid(s, n) ) {
        neighbors.emplace_back(
            Neighbor<State, Action, float>(n, Action::Left_0, 1));
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y, s.o);
      if (stateValid(n, agentIdx) && transitionValid(s, n) ) {
        neighbors.emplace_back(
            Neighbor<State, Action, float>(n, Action::Right_0, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1, s.o);
      if (stateValid(n, agentIdx)&& transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<State, Action, float>(n, Action::Up_0, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1, s.o);
      if (stateValid(n, agentIdx)&& transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, float>(n, Action::Down_0, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y, o_turn_left(s.o));
      if (stateValid(n, agentIdx)&& transitionValid(s, n)&& AABB_check(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, float>(n, Action::Wait_1, 1));
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y, o_turn_left(s.o));
      if (stateValid(n, agentIdx) && transitionValid(s, n)&& AABB_check(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, float>(n, Action::Left_1, 1));
      }
    }
    {
        State n(s.time + 1, s.x + 1, s.y, o_turn_left(s.o));
        if (stateValid(n, agentIdx) && transitionValid(s, n)&& AABB_check(s, n)) {
            neighbors.emplace_back(
                Neighbor<State, Action, float>(n, Action::Right_1, 1));
        }
    }
    {
        State n(s.time + 1, s.x, s.y + 1, o_turn_left(s.o));
        if (stateValid(n, agentIdx) && transitionValid(s, n)&& AABB_check(s, n)) {
            neighbors.emplace_back(
                Neighbor<State, Action, float>(n, Action::Up_1, 1));
        }
    }
    {
        State n(s.time + 1, s.x, s.y - 1, o_turn_left(s.o));
        if (stateValid(n, agentIdx)&& transitionValid(s, n)&& AABB_check(s, n)) {
            neighbors.emplace_back(
                Neighbor<State, Action, float>(n, Action::Down_1, 1));
        }
    }
    {
        State n(s.time + 1, s.x, s.y, o_turn_right(s.o));
        if (stateValid(n, agentIdx) && transitionValid(s, n)&& AABB_check(s, n)) {
            neighbors.emplace_back(
                Neighbor<State, Action, float>(n, Action::Wait_2, 1));
        }
    }
    {
        State n(s.time + 1, s.x - 1, s.y, o_turn_right(s.o));
        if (stateValid(n, agentIdx)&& transitionValid(s, n)&& AABB_check(s, n)) {
            neighbors.emplace_back(
                Neighbor<State, Action, float>(n, Action::Left_2, 1));
        }
    }
    {
        State n(s.time + 1, s.x + 1, s.y, o_turn_right(s.o));
        if (stateValid(n, agentIdx)&& transitionValid(s, n)&& AABB_check(s, n)) {
            neighbors.emplace_back(
                Neighbor<State, Action, float>(n, Action::Right_2, 1));
        }
    }
    {
        State n(s.time + 1, s.x, s.y + 1, o_turn_right(s.o));
        if (stateValid(n, agentIdx)&& transitionValid(s, n)&& AABB_check(s, n)) {
            neighbors.emplace_back(
                Neighbor<State, Action, float>(n, Action::Up_2, 1));
        }
    }
    {
        State n(s.time + 1, s.x, s.y - 1, o_turn_right(s.o));
        if (stateValid(n, agentIdx)&& transitionValid(s, n)&& AABB_check(s, n)) {
            neighbors.emplace_back(
                Neighbor<State, Action, float>(n, Action::Down_2, 1));
        }
    }
  }

  int o_turn_left(int o)
  {
    if (o ==1) {return 4;}
    else return o-1;
  }

  int o_turn_right(int o)
  {
    if (o ==4) {return 1;}
    else return o+1;
  }

  bool RC_AABB(const State& s1a, const State& s2a, const State& s1b, const State& s2b, size_t i, size_t j, std::vector<State> & RC_states, int & RC_type)
  {
    std::vector<Location> shape1a = m_shapes_sheet[i][s1a.o] ;
    std::vector<Location> shape2a = m_shapes_sheet[i][s2a.o] ;
    std::vector<Location> shape1b = m_shapes_sheet[j][s1b.o] ;
    std::vector<Location> shape2b = m_shapes_sheet[j][s2b.o] ;
    if (s1a.o != s2a.o && s1b.o == s2b.o)
    {
      std::vector<Location>:: iterator it1a = shape1a.begin();
      std::vector<Location>:: iterator it2a = shape2a.begin();
      while (it1a != shape1a.end() && it2a != shape2a.end())
      {
        int AB_xmin = std::min( s1a.x+ it1a->x, s2a.x+ it2a->x);
        int AB_xmax = std::max( s1a.x+ it1a->x, s2a.x+ it2a->x);
        int AB_ymin = std::min( s1a.y+ it1a->y, s2a.y+ it2a->y);
        int AB_ymax = std::max( s1a.y+ it1a->y, s2a.y+ it2a->y);

        std::vector<Location>:: iterator it1b = shape1b.begin();
        while (it1b != shape1b.end())
        {
          int XX = s1b.x +it1b->x ;
          int YY = s1b.y + it1b->y ;
          if( XX >= AB_xmin && XX <= AB_xmax && YY >= AB_ymin && YY <= AB_ymax)
          {
            RC_states.push_back(s2a);
            RC_states.push_back(s1b);
            RC_type = 1;
            return true;
          }

          it1b++;
        }
        it1a++;
        it2a++;
      }
    }
    if (s1b.o != s2b.o && s1a.o == s2a.o)
    {
      std::vector<Location>:: iterator it1b = shape1b.begin();
      std::vector<Location>:: iterator it2b = shape2b.begin();
      while (it1b != shape1b.end() && it2b != shape2b.end())
      {
        int AB_xmin = std::min( s1b.x+ it1b->x, s2b.x+ it2b->x);
        int AB_xmax = std::max( s1b.x+ it1b->x, s2b.x+ it2b->x);
        int AB_ymin = std::min( s1b.y+ it1b->y, s2b.y+ it2b->y);
        int AB_ymax = std::max( s1b.y+ it1b->y, s2b.y+ it2b->y);

        std::vector<Location>:: iterator it1a = shape1a.begin();
        while (it1a != shape1a.end())
        {
          int XX = s1a.x + it1a->x;
          int YY = s1a.y + it1a->y;
          if(XX >= AB_xmin && XX <= AB_xmax && YY >= AB_ymin && YY <= AB_ymax)
          {
            RC_states.push_back(s1a);
            RC_states.push_back(s2b);
            RC_type = 2;
            return true;
          }
          it1a++;
        }
        it1b++;
        it2b++;
      }
    }

    if (s1a.o != s2a.o && s1b.o != s2b.o) {
      //检测两个包围盒有无冲突区域
      bool isCollide = false;
      isCollide = TwoRotateAABBCollide(s1a, s2a, s1b, s2b, shape1a, shape2a, shape1b, shape2b);
      if (isCollide) {
        RC_states.push_back(s2a);
        RC_states.push_back(s2b);
        RC_type = 3;
        return true;
      }
      
    }

    return false;
  }

  bool TwoRotateAABBCollide(
    const State& s1a,const State& s2a,const State& s1b,const State& s2b, std::vector<Location>& shape1a, std::vector<Location>& shape2a, std::vector<Location>& shape1b, std::vector<Location>& shape2b) {
      int A_xmin, A_xmax, A_ymin, A_ymax;
      int B_xmin, B_xmax, B_ymin, B_ymax;
      std::vector<Location>:: iterator it1a = shape1a.begin();
      std::vector<Location>:: iterator it2a = shape2a.begin();
      while (it1a != shape1a.end() && it2a != shape2a.end())
      {
        A_xmin = std::min( s1a.x+ it1a->x, s2a.x+ it2a->x);
        A_xmax = std::max( s1a.x+ it1a->x, s2a.x+ it2a->x);
        A_ymin = std::min( s1a.y+ it1a->y, s2a.y+ it2a->y);
        A_ymax = std::max( s1a.y+ it1a->y, s2a.y+ it2a->y);

        std::vector<Location>:: iterator it1b = shape1b.begin();
        std::vector<Location>:: iterator it2b = shape2b.begin();
        while (it1b != shape1b.end() && it2b != shape2b.end())
        {
          B_xmin = std::min( s1b.x+ it1b->x, s2b.x+ it2b->x);
          B_xmax = std::max( s1b.x+ it1b->x, s2b.x+ it2b->x);
          B_ymin = std::min( s1b.y+ it1b->y, s2b.y+ it2b->y);
          B_ymax = std::max( s1b.y+ it1b->y, s2b.y+ it2b->y);
          
          bool BoxCollide = false;
          BoxCollide = TwoBoxCollide(A_xmin, A_xmax, A_ymin, A_ymax, B_xmin, B_xmax, B_ymin, B_ymax);
          if (BoxCollide) {
            return true;
          }
          it1b++;
          it2b++;
        }
        it1a++;
        it2a++;
      }
    return false;
    }

  bool TwoBoxCollide(int A_xmin, int A_xmax, int A_ymin, int A_ymax, int B_xmin, int B_xmax, int B_ymin, int B_ymax) {
    if (A_xmax < B_xmin || A_xmin > B_xmax ) {
      return false;
    }
    if (A_ymax < B_ymin || A_ymin > B_ymax ) {
      return false;
    }
    return true;    
  }

  bool getHeuristic1Conflict(const std::vector<PlanResult<State, Action, float> >& solution,
      Conflict& result_final)
  {
    int max_t = 0;
    for (const auto& sol : solution) { 
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }
    std::vector<Conflict> conflicts_list;
    std::vector<int> heuristics1_list; 
    std::vector<int> time_list; 


    for (int t = 0; t <= max_t; ++t) {  
      for (size_t i = 0; i < solution.size(); ++i) { 
        State state1 = getState(i, solution, t);
        State state1a = getState(i, solution, t);
        State state2a = getState(i, solution, t + 1); 
        std::vector<Location> shape1 = m_shapes_sheet[i][state1.o] ;
        for (size_t j = i + 1; j < solution.size(); ++j) {
          Conflict result;
          State state2 = getState(j, solution, t);
          State state1b = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1); 
          std::vector<Location> shape2 = m_shapes_sheet[j][state2.o] ;
          double robot_distance = std::sqrt(std::pow(state1.x - state2.x, 2) + std::pow(state1.y - state2.y, 2));
          if (robot_distance < 9 ) 
          {
          if (state1.equalExceptTime(state2)) { 
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            result.o1 = state1.o; 
            result.i1 = i;
            result.x2 = state2.x;
            result.y2 = state2.y;
            result.o2 = state2.o;
            result.i2 = j;

            int h1a = m_h1_list[i];
            int h1b = m_h1_list[j];
            int h1 = h1a + m_shapes_sheet[i][1].size() + h1b + m_shapes_sheet[j][1].size();
            conflicts_list.push_back(result);
            heuristics1_list.push_back(h1);
            time_list.push_back(t);
            continue;
            
            // return true;
          }
          bool shape_collide = shape_vertex_collide(state1, state2, shape1, shape2);
          if(shape_collide)
          {
            
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::ShapeV; 
            result.x1 = state1.x;
            result.y1 = state1.y;
            result.o1 = state1.o;
            result.i1 = i;
            result.x2 = state2.x;
            result.y2 = state2.y;
            result.o2 = state2.o;
            result.i2 = j;

            int h1a = m_h1_list[i];
            int h1b = m_h1_list[j];
            int h1 = h1a + m_shapes_sheet[i][1].size() + h1b + m_shapes_sheet[j][1].size();
            conflicts_list.push_back(result);
            heuristics1_list.push_back(h1);
            time_list.push_back(t);
            continue;
            // return true;
          }
          bool shapeEC_temp = shape_edge_collide(state1a, state2a, state1b, state2b, i, j);
          if (shapeEC_temp)
          {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::ShapeE;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.o1 = state1a.o;
            result.i1 = i;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            result.o2 = state1b.o;
            result.i2 = j;
            result.data1[0] = state2a.x;
            result.data1[1] = state2a.y;
            result.data1[2] = state2b.x;
            result.data1[3] = state2b.y;
            int h1a = m_h1_list[i];
            int h1b = m_h1_list[j];
            int h1 = h1a + m_shapes_sheet[i][1].size() + h1b + m_shapes_sheet[j][1].size();
            conflicts_list.push_back(result);
            heuristics1_list.push_back(h1);
            time_list.push_back(t);
            continue;
            // return true;
          }
          if (t < max_t)
          {
            if (state2a.o != state1a.o || state2b.o != state1b.o)
            {
              std::vector<State> RC_constrain_states ; 
              int RC_type ; 
              bool RC_clash = RC_AABB(state1a, state2a, state1b, state2b, i, j, RC_constrain_states, RC_type) ;
              if (RC_clash)
              {
                if (RC_type == 1)
                {
                result.time1 = t  ;
                result.agent1 = i;
                result.agent2 = j;
                result.type = Conflict::RotateC;
                result.x1 = state1a.x;
                result.y1 = state1a.y;
                result.o1 = state1a.o;
                result.i1 = i;
                result.time2 = t ;
                result.x2 = state1b.x;
                result.y2 = state1b.y;
                result.o2 = state1b.o; 
                result.i2 = j;
                result.data1[0] = RC_constrain_states[0].x;
                result.data1[1] = RC_constrain_states[0].y;
                result.data1[2] = RC_constrain_states[0].o;
                result.rc_type = RC_type;
                }
                if (RC_type == 2)
                {
                result.time1 = t  ;
                result.agent1 = i;
                result.agent2 = j;
                result.type = Conflict::RotateC;
                result.x1 = state1a.x;
                result.y1 = state1a.y;
                result.o1 = state1a.o;
                result.i1 = i;
                result.time2 = t ;
                result.x2 = state1b.x;
                result.y2 = state1b.y;
                result.o2 = state1b.o; 
                result.i2 = j;
                result.data1[0] = RC_constrain_states[1].x;
                result.data1[1] = RC_constrain_states[1].y;
                result.data1[2] = RC_constrain_states[1].o;
                result.rc_type = RC_type;
                }
                if (RC_type == 3)
                {
                  result.time1 = t  ;
                  result.agent1 = i;
                  result.agent2 = j;
                  result.type = Conflict::RotateC;
                  result.x1 = state1a.x;
                  result.y1 = state1a.y;
                  result.o1 = state1a.o;
                  result.i1 = i;
                  result.time2 = t ;
                  result.x2 = state1b.x;
                  result.y2 = state1b.y;
                  result.o2 = state1b.o; 
                  result.i2 = j;
                  result.data1[0] = RC_constrain_states[0].x;
                  result.data1[1] = RC_constrain_states[0].y;
                  result.data1[2] = RC_constrain_states[0].o;
                  result.data1[3] = RC_constrain_states[1].x;
                  result.data1[4] = RC_constrain_states[1].y;
                  result.data1[5] = RC_constrain_states[1].o;
                  result.rc_type = RC_type;
                }
                int h1a = m_h1_list[i];
                int h1b = m_h1_list[j];
                int h1 = h1a + m_shapes_sheet[i][1].size() + h1b + m_shapes_sheet[j][1].size();
                conflicts_list.push_back(result);
                heuristics1_list.push_back(h1);
                time_list.push_back(t);
                continue;
                // return true;
              }
            }
          }
          } 
        }
      }
    }
    if (conflicts_list.size() > 0)
    {
      int target_index = std::max_element(heuristics1_list.begin(), heuristics1_list.end()) - heuristics1_list.begin();
      result_final =  conflicts_list[target_index];
      return true;
    }
    return false;
  }

  int TouchSpotsNum(std::vector<std::vector<Location>> & shapes_sheet)
  {
      std::vector<Eigen::Vector3d> temp_list(0);
      for (size_t j = 0; j < shapes_sheet[1].size(); j++)
      {
          for (int k = 1; k < 5; k++)
          {
          Eigen::Vector3d touch_temp ;
          touch_temp(0) = shapes_sheet[1][j].x * 1.0 + 0.5 * ( sin(1.5*k) > 0 ? 1 : -1 ) ;
          touch_temp(1) = shapes_sheet[1][j].y * 1.0 + 0.5 * ( cos(1.5*k) > 0 ? 1 : -1 ) ;
          touch_temp(2) = 0.0 ;
          bool found = false;
          for (auto it = temp_list.begin(); it != temp_list.end(); ++it) 
          {
              if (it->isApprox(touch_temp)) {
              temp_list.erase(it);
              found = true;
              break;
              }
          }
              if (!found) {
                  temp_list.push_back(touch_temp);
              }
          }
      }
      return temp_list.size() ;
  }

  bool getFirstConflict( 
      const std::vector<PlanResult<State, Action, float> >& solution,
      Conflict& result) {
    int max_t = 0;
    for (const auto& sol : solution) {  
      max_t = std::max<int>(max_t, sol.states.size() - 1);
    }
    for (int t = 0; t <= max_t; ++t) {  
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) { 
        State state1 = getState(i, solution, t);
        State state1a = getState(i, solution, t);
        State state2a = getState(i, solution, t + 1); 
        std::vector<Location> shape1 = m_shapes_sheet[i][state1.o] ;
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          State state1b = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1); 
          std::vector<Location> shape2 = m_shapes_sheet[j][state2.o] ;
          double robot_distance = std::sqrt(std::pow(state1.x - state2.x, 2) + std::pow(state1.y - state2.y, 2));
          if (robot_distance < 9 ) 
          {
          if (state1.equalExceptTime(state2)) { 
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            result.o1 = state1.o; 
            result.i1 = i;
            result.x2 = state2.x;
            result.y2 = state2.y;
            result.o2 = state2.o;
            result.i2 = j;
            return true;
          }

          bool shape_collide = shape_vertex_collide(state1, state2, shape1, shape2);
          if(shape_collide)
          {
            
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::ShapeV; 
            result.x1 = state1.x;
            result.y1 = state1.y;
            result.o1 = state1.o;
            result.i1 = i;
            result.x2 = state2.x;
            result.y2 = state2.y;
            result.o2 = state2.o;
            result.i2 = j;
            return true;
          }
          bool shapeEC_temp = shape_edge_collide(state1a, state2a, state1b, state2b, i, j);
          if (shapeEC_temp)
          {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::ShapeE;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.o1 = state1a.o;
            result.i1 = i;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            result.o2 = state1b.o;
            result.i2 = j;
            result.data1[0] = state2a.x;
            result.data1[1] = state2a.y;
            result.data1[2] = state2b.x;
            result.data1[3] = state2b.y;
            return true;
          }
          if (t < max_t)
          {
            if (state2a.o != state1a.o || state2b.o != state1b.o)
            {
              std::vector<State> RC_constrain_states ; 
              int RC_type ; 
              bool RC_clash = RC_AABB(state1a, state2a, state1b, state2b, i, j, RC_constrain_states, RC_type) ;
              if (RC_clash)
              {
                if (RC_type == 1)
                {
                result.time1 = t  ;
                result.agent1 = i;
                result.agent2 = j;
                result.type = Conflict::RotateC;
                result.x1 = state1a.x;
                result.y1 = state1a.y;
                result.o1 = state1a.o;
                result.i1 = i;
                result.time2 = t ;
                result.x2 = state1b.x;
                result.y2 = state1b.y;
                result.o2 = state1b.o; 
                result.i2 = j;
                result.data1[0] = RC_constrain_states[0].x;
                result.data1[1] = RC_constrain_states[0].y;
                result.data1[2] = RC_constrain_states[0].o;
                result.rc_type = RC_type;
                }
                if (RC_type == 2)
                {
                result.time1 = t  ;
                result.agent1 = i;
                result.agent2 = j;
                result.type = Conflict::RotateC;
                result.x1 = state1a.x;
                result.y1 = state1a.y;
                result.o1 = state1a.o;
                result.i1 = i;
                result.time2 = t ;
                result.x2 = state1b.x;
                result.y2 = state1b.y;
                result.o2 = state1b.o; 
                result.i2 = j;
                result.data1[0] = RC_constrain_states[1].x;
                result.data1[1] = RC_constrain_states[1].y;
                result.data1[2] = RC_constrain_states[1].o;
                result.rc_type = RC_type;
                }
                if (RC_type == 3)
                {
                  result.time1 = t  ;
                  result.agent1 = i;
                  result.agent2 = j;
                  result.type = Conflict::RotateC;
                  result.x1 = state1a.x;
                  result.y1 = state1a.y;
                  result.o1 = state1a.o;
                  result.i1 = i;
                  result.time2 = t ;
                  result.x2 = state1b.x;
                  result.y2 = state1b.y;
                  result.o2 = state1b.o; 
                  result.i2 = j;
                  result.data1[0] = RC_constrain_states[0].x;
                  result.data1[1] = RC_constrain_states[0].y;
                  result.data1[2] = RC_constrain_states[0].o;
                  result.data1[3] = RC_constrain_states[1].x;
                  result.data1[4] = RC_constrain_states[1].y;
                  result.data1[5] = RC_constrain_states[1].o;
                  result.rc_type = RC_type;
                }

                return true;
              }
            }
          }
          } 
        }
      }
    }
    return false;
  }

  void createConstraintsFromConflict(  
      const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x1, conflict.y1, conflict.o1, conflict.i1));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x2, conflict.y2, conflict.o2, conflict.i2));
      constraints[conflict.agent2] = c2;
    }
    else if (conflict.type == Conflict::ShapeV) {
            Constraints c1; 
            c1.vertexConstraints.emplace(VertexConstraint(
            conflict.time, conflict.x1, conflict.y1, conflict.o1, conflict.i1 ));
            constraints[conflict.agent1] = c1;
            Constraints c2;
            c2.vertexConstraints.emplace(VertexConstraint(
            conflict.time, conflict.x2, conflict.y2, conflict.o2, conflict.i2 ));
            constraints[conflict.agent2] = c2;  
    }

      else if (conflict.type == Conflict::RotateC) {

            if (conflict.rc_type == 1) {
              Constraints c1;
              c1.edgeConstraints.emplace(EdgeConstraint(
              conflict.time1, conflict.x1, conflict.y1, conflict.o1, conflict.data1[0], conflict.data1[1], conflict.data1[2]));
              constraints[conflict.agent1] = c1;
              Constraints c2;
              c2.vertexConstraints.emplace(VertexConstraint(
              conflict.time2, conflict.x2, conflict.y2, conflict.o2, conflict.i2 ));
              constraints[conflict.agent2] = c2;
            }
            else if (conflict.rc_type == 2) {
              Constraints c1;
              c1.vertexConstraints.emplace(VertexConstraint(
              conflict.time1, conflict.x1, conflict.y1, conflict.o1, conflict.i1 ));
              constraints[conflict.agent1] = c1;
              Constraints c2;
              c2.edgeConstraints.emplace(EdgeConstraint(
              conflict.time2, conflict.x2, conflict.y2, conflict.o2, conflict.data1[0], conflict.data1[1], conflict.data1[2]));
              constraints[conflict.agent2] = c2;
            }
            else if (conflict.rc_type == 3) {
              // Two edge constraints for rotation type 3
              Constraints c1;
              c1.edgeConstraints.emplace(EdgeConstraint(
              conflict.time1, conflict.x1, conflict.y1, conflict.o1, conflict.data1[0], conflict.data1[1], conflict.data1[2]));
              constraints[conflict.agent1] = c1;
              Constraints c2;
              c2.edgeConstraints.emplace(EdgeConstraint(
              conflict.time2, conflict.x2, conflict.y2, conflict.o2, conflict.data1[3], conflict.data1[4], conflict.data1[5]));
              constraints[conflict.agent2] = c2;
            }

            

      }
      else if (conflict.type == Conflict::ShapeE) {
          Constraints c1;
          c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x1, conflict.y1, conflict.o1, conflict.data1[0], conflict.data1[1],  conflict.o1));
          constraints[conflict.agent1] = c1;
          Constraints c2;
          c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x2, conflict.y2, conflict.o2, conflict.data1[2], conflict.data1[3], conflict.o2));
          constraints[conflict.agent2] = c2;          
        }  
    
  }

  void createSYMMultiConstraintsFromConflict(
    const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
      
    if (conflict.type == Conflict::Vertex || conflict.type == Conflict::ShapeV) {

      Location p = FindConflictPoint(conflict);  
      std::vector<State> unableStates1;
      unableStates1 = ConflictPointStateSearch(p, m_shapes_sheet[conflict.i1]);
      
      Constraints c1;
      for (auto& unableState : unableStates1) {
        c1.vertexConstraints.emplace(
            VertexConstraint(conflict.time, unableState.x, unableState.y, unableState.o, conflict.i1));
      }
      constraints[conflict.agent1] = c1;

      std::vector<State> unableStates2;
      unableStates2 = ConflictPointStateSearch(p, m_shapes_sheet[conflict.i2]);

      Constraints c2;
      for (auto& unableState : unableStates2) {
        c2.vertexConstraints.emplace(
            VertexConstraint(conflict.time, unableState.x, unableState.y, unableState.o, conflict.i2));
      }
      constraints[conflict.agent2] = c2;

    }

    else if (conflict.type == Conflict::ShapeE || conflict.type == Conflict::Edge) {
                
      std::vector<Location> p = FindConflictPointPair(conflict);  
      std::vector< std::pair<State, State> > unableStatesPair1 ;
      int agentType1 = 1; 
      unableStatesPair1 = ConflictPointStatePairSearch(p, m_shapes_sheet[conflict.i1], agentType1) ;

      Constraints c1;
      for (auto &unableStatesPair : unableStatesPair1) {
        c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, unableStatesPair.first.x, unableStatesPair.first.y, unableStatesPair.first.o, unableStatesPair.second.x, unableStatesPair.second.y,  unableStatesPair.first.o));
      }

      constraints[conflict.agent1] = c1;

      std::vector< std::pair<State, State> > unableStatesPair2 ;
      int agentType2 = 2; 
      unableStatesPair2 = ConflictPointStatePairSearch(p, m_shapes_sheet[conflict.i2], agentType2) ;
      
      Constraints c2;
      for (auto &unableStatesPair : unableStatesPair2) {
        c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, unableStatesPair.first.x, unableStatesPair.first.y, unableStatesPair.first.o, unableStatesPair.second.x, unableStatesPair.second.y,  unableStatesPair.first.o));
      }
      constraints[conflict.agent2] = c2;
    }
    else if (conflict.type == Conflict::RotateC) {
    
      if (conflict.rc_type == 1) 
      {
        Location p = FindConflictPoint_RCtypeOneTwo(conflict);
          std::vector<State> unableStates1;
          unableStates1 = ConflictPointStateSearch_RCtypeOneTwo(conflict); 
          Constraints c1;
          for (auto &unableState : unableStates1) {
            c1.edgeConstraints.emplace(EdgeConstraint(
              conflict.time1, conflict.x1, conflict.y1, conflict.o1, unableState.x, unableState.y, unableState.o));
          }
          c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time1, conflict.x1, conflict.y1, conflict.o1, conflict.data1[0], conflict.data1[1], conflict.data1[2]));
          constraints[conflict.agent1] = c1;
        
          std::vector<State> unableStates2;
          unableStates2 = ConflictPointStateSearch(p, m_shapes_sheet[conflict.i2]);
          Constraints c2;
          for (auto& unableState : unableStates2) {
            c2.vertexConstraints.emplace(
                VertexConstraint(conflict.time2, unableState.x, unableState.y, unableState.o, conflict.i2));
          }
          c2.vertexConstraints.emplace(VertexConstraint(
                conflict.time2, conflict.x2, conflict.y2, conflict.o2, conflict.i2 ));
          constraints[conflict.agent2] = c2;
        
      }
        
      
      else if (conflict.rc_type == 2) 
      {
        Location p = FindConflictPoint_RCtypeOneTwo(conflict);
          std::vector<State> unableStates1;
          unableStates1 = ConflictPointStateSearch(p, m_shapes_sheet[conflict.i1]);
          Constraints c1;
          for (auto& unableState : unableStates1) {
            c1.vertexConstraints.emplace(
                VertexConstraint(conflict.time1, unableState.x, unableState.y, unableState.o, conflict.i1));
          }
          c1.vertexConstraints.emplace(VertexConstraint(
                conflict.time1, conflict.x1, conflict.y1, conflict.o1, conflict.i1 ));
                constraints[conflict.agent1] = c1;
          constraints[conflict.agent1] = c1;
          std::vector<State> unableStates2;
          unableStates2 = ConflictPointStateSearch_RCtypeOneTwo(conflict);
          Constraints c2;
          for (auto& unableState : unableStates2) {
            c2.edgeConstraints.emplace(EdgeConstraint(
                conflict.time2, conflict.x2, conflict.y2, conflict.o2, unableState.x, unableState.y, unableState.o));
          }
          c2.edgeConstraints.emplace(EdgeConstraint(
                conflict.time2, conflict.x2, conflict.y2, conflict.o2, conflict.data1[0], conflict.data1[1], conflict.data1[2]));

          constraints[conflict.agent2] = c2;
      }
      else if (conflict.rc_type == 3) //
      {
        Constraints c1;
        c1.edgeConstraints.emplace(EdgeConstraint(
        conflict.time1, conflict.x1, conflict.y1, conflict.o1, conflict.data1[0], conflict.data1[1], conflict.data1[2]));
        constraints[conflict.agent1] = c1;
        Constraints c2;
        c2.edgeConstraints.emplace(EdgeConstraint(
        conflict.time2, conflict.x2, conflict.y2, conflict.o2, conflict.data1[3], conflict.data1[4], conflict.data1[5]));
        constraints[conflict.agent2] = c2; 
        State s1a = State(conflict.time1, conflict.x1, conflict.y1, conflict.o1);
        State s1b = State(conflict.time2, conflict.x2, conflict.y2, conflict.o2);
        State s2a = State(conflict.time1+1, conflict.data1[0], conflict.data1[1], conflict.data1[2]);
        State s2b = State(conflict.time2+1, conflict.data1[3], conflict.data1[4], conflict.data1[5]);


        std::vector<State> unableStates1;
        unableStates1 = ConflictStateSearch_RCtypeThree(s1a, s1b, s2b, conflict.agent1, conflict.agent2);
        for (auto &unableState : unableStates1) {
            c1.edgeConstraints.emplace(EdgeConstraint(
              conflict.time1, conflict.x1, conflict.y1, conflict.o1, unableState.x, unableState.y, unableState.o));
        }
        c1.edgeConstraints.emplace(EdgeConstraint(
        conflict.time1, conflict.x1, conflict.y1, conflict.o1, conflict.data1[0], conflict.data1[1], conflict.data1[2]));
        constraints[conflict.agent1] = c1;

        std::vector<State> unableStates2;
        unableStates2 = ConflictStateSearch_RCtypeThree(s1b, s1a, s2a, conflict.agent2, conflict.agent1);
        for (auto& unableState : unableStates2) {
            c2.edgeConstraints.emplace(EdgeConstraint(
                conflict.time2, conflict.x2, conflict.y2, conflict.o2, unableState.x, unableState.y, unableState.o));
        }
        constraints[conflict.agent2] = c2;
      }
  
    }
  }

  std::vector<State> ConflictStateSearch_RCtypeThree(State A1, State B1, State B2, int agentA, int agentB) {
    std::vector<State> unableStates;
    int s[4] = {0, 1, 0, -1};
    int c[4] = {1, 0, -1, 0};
    std::vector<Location> shapeA1 = m_shapes_sheet[agentA][A1.o];
    std::vector<Location> shapeB1 = m_shapes_sheet[agentB][B1.o];
    std::vector<Location> shapeB2 = m_shapes_sheet[agentB][B2.o];

    for (int i = 0; i < 4; i++) {
      State A2(A1.time +1, A1.x + s[i], A1.y + c[i], o_turn_left(A1.o) );
      std::vector<Location> shapeA2 = m_shapes_sheet[agentA][A2.o] ;
      State A3(A1.time +1, A1.x + s[i], A1.y + c[i], o_turn_right(A1.o) );
      std::vector<Location> shapeA3 = m_shapes_sheet[agentA][A3.o] ;
      if (TwoRotateAABBCollide(A1, A2, B1, B2, shapeA1, shapeA2, shapeB1, shapeB2)) {
        unableStates.emplace_back(A2);
      }
      if (TwoRotateAABBCollide(A1, A3, B1, B2, shapeA1, shapeA3, shapeB1, shapeB2)) {
        unableStates.emplace_back(A3);
      }
    }
    return unableStates;
  }

  bool isAgentAtGoal(int x, int y, int idx)
  {
    return (x == m_goals[idx].x && y == m_goals[idx].y);
  }

  std::vector<State> ConflictPointStateSearch_RCtypeOneTwo(const Conflict& conflict) {
    std::vector<State> unableStates;
    int s[4] = {0, 1, 0, -1};
    int c[4] = {1, 0, -1, 0};
    
    if (conflict.rc_type == 1) {
      State A1(conflict.time1, conflict.x1, conflict.y1, conflict.o1);
      std::vector<Location> shapeA1 = m_shapes_sheet[conflict.i1][conflict.o1] ;
      std::vector<Location> shapeB1 = m_shapes_sheet[conflict.i2][conflict.o2] ;
      State B1(conflict.time2, conflict.x2, conflict.y2, conflict.o2);
      for (int i = 0; i < 4; i++) {
        State A2(conflict.time1 +1, conflict.x1 + s[i], conflict.y1 + c[i], o_turn_left(conflict.o1) );
        std::vector<Location> shapeA2 = m_shapes_sheet[conflict.i1][A2.o] ;
        State A3(conflict.time1 +1, conflict.x1 + s[i], conflict.y1 + c[i], o_turn_right(conflict.o1) );
        std::vector<Location> shapeA3 = m_shapes_sheet[conflict.i1][A3.o] ;
        std::vector<Location>:: iterator itb1 ;
        for(size_t j = 0; j < shapeA1.size() ; j++)
        {
          bool tag_temp = false;
          std::vector<int> boxA2  = Box_boundray(A1, A2, shapeA1, shapeA2, j); // xmin xmax ymin ymax
          for (itb1 = shapeB1.begin(); itb1 != shapeB1.end(); itb1++) {
            if (itb1->x + B1.x >= boxA2[0] && itb1->x + B1.x <= boxA2[1] && itb1->y + B1.y >= boxA2[2] && itb1->y + B1.y <= boxA2[3]) {
                unableStates.push_back(A2);
                tag_temp = true;
                break;
            }     
          }
          if (tag_temp) {break;}
        }
        for(size_t j = 0; j < shapeA1.size() ; j++)
        {
          bool tag_temp = false;
          std::vector<int> boxA3  = Box_boundray(A1, A3, shapeA1, shapeA3, j); // xmin xmax ymin ymax
          for (itb1 = shapeB1.begin(); itb1 != shapeB1.end(); itb1++) {
          if (itb1->x + B1.x >= boxA3[0] && itb1->x + B1.x <= boxA3[1] && itb1->y + B1.y >= boxA3[2] && itb1->y + B1.y <= boxA3[3]) {
              unableStates.push_back(A3);
              tag_temp = true;
              break;
            } 
          }
          if (tag_temp) {break;}
        }

      }
    return unableStates;
    }

    if (conflict.rc_type == 2){
      State B1(conflict.time2, conflict.x2, conflict.y2, conflict.o2);
      std::vector<Location> shapeB1 = m_shapes_sheet[conflict.i2][conflict.o2] ;
      for (int i = 0; i < 4; i++) {
        State B2(conflict.time2 +1, conflict.x2 + s[i], conflict.y2 + c[i], o_turn_left(conflict.o2) );
        std::vector<Location> shapeB2 = m_shapes_sheet[conflict.i2][B2.o] ;
        State B3(conflict.time2 +1, conflict.x2 + s[i], conflict.y2 + c[i], o_turn_right(conflict.o2) );
        std::vector<Location> shapeB3 = m_shapes_sheet[conflict.i2][B3.o] ;
        std::vector<Location> shapeA1 = m_shapes_sheet[conflict.i1][conflict.o1] ;
        State A1(conflict.time1, conflict.x1, conflict.y1, conflict.o1);
        std::vector<Location>:: iterator ita1;
        for(size_t j = 0; j < shapeA1.size() ; j++)
        {
          bool tag_temp = false;
          std::vector<int> boxB2  = Box_boundray(B1, B2, shapeB1, shapeB2, j); // xmin xmax ymin ymax
          for (ita1 =shapeA1.begin();  ita1 != shapeA1.end(); ita1++) {
          if (ita1->x + A1.x >= boxB2[0] && ita1->x + A1.x <= boxB2[1] && ita1->y + A1.y >= boxB2[2] && ita1->y + A1.y <= boxB2[3] ) {
              unableStates.push_back(B2);
              tag_temp = true;
              break;
            }
          }
          if (tag_temp) {break;}
        }
        for(size_t j = 0; j < shapeA1.size() ; j++)
        {
          bool tag_temp = false;
          std::vector<int> boxB3  = Box_boundray(B1, B3, shapeB1, shapeB3, j); // xmin xmax ymin ymax
          for (ita1 =shapeA1.begin(); ita1 != shapeA1.end(); ita1++) {
            if (ita1->x + A1.x >= boxB3[0] && ita1->x + A1.x <= boxB3[1] && ita1->y + A1.y >= boxB3[2] && ita1->y + A1.y <= boxB3[3] ) {
                unableStates.push_back(B3);
                tag_temp = true;
                break;
            }
          }
          if (tag_temp) {break;}
        }
      }
    return unableStates;
    }

    std::cout << "--- Function ConflictPointStateSearch_RCtypeOneTwo error! --- in environment.hpp " << std::endl;
    std::vector<State> result_bad;
    result_bad.push_back(State(0, 0, 0, 0));
    return result_bad;
  
  }
    
  std::vector<int> Box_boundray(
     State& A1, State& A2, std::vector<Location>& shapeA1, std::vector<Location>& shapeA2, size_t j) {

     int A_xmin = std::min( A1.x + shapeA1[j].x, A2.x + shapeA2[j].x ) ;
     int A_xmax = std::max( A1.x + shapeA1[j].x, A2.x + shapeA2[j].x ) ;
     int A_ymin = std::min( A1.y + shapeA1[j].y, A2.y + shapeA2[j].y );
     int A_ymax = std::max( A1.y + shapeA1[j].y, A2.y + shapeA2[j].y );
 
    return {A_xmin, A_xmax, A_ymin, A_ymax};
  }

  Location FindConflictPoint_RCtypeOneTwo(const Conflict& conflict) {
    if (conflict.rc_type == 1) {
      int A_xmin = 999;
      int A_xmax = 0;
      int A_ymin = 999;
      int A_ymax = 0;
      std::vector<Location> shape1a = m_shapes_sheet[conflict.i1][conflict.o1] ;
      std::vector<Location> shape2a = m_shapes_sheet[conflict.i1][conflict.data1[2]] ;
      std::vector<Location>:: iterator it1a = shape1a.begin();
      std::vector<Location>:: iterator it2a = shape2a.begin();
      std::vector<Location> shape1b = m_shapes_sheet[conflict.i2][conflict.o2] ;
      while (it1a != shape1a.end() && it2a != shape2a.end())
      {
        A_xmin = std::min( conflict.x1+ it1a->x, conflict.data1[0]+ it2a->x) ;
        A_xmax = std::max( conflict.x1+ it1a->x, conflict.data1[0]+ it2a->x) ;
        A_ymin = std::min( conflict.y1+ it1a->y, conflict.data1[1]+ it2a->y)  ;
        A_ymax = std::max( conflict.y1+ it1a->y, conflict.data1[1]+ it2a->y)  ;
        std::vector<Location>:: iterator it1b = shape1b.begin();
        while (it1b != shape1b.end())
        {
          if (conflict.x2 + it1b->x >= A_xmin && conflict.x2 + it1b->x <= A_xmax && conflict.y2 + it1b->y >= A_ymin && conflict.y2 + it1b->y <= A_ymax) {
            return Location(conflict.x2 + it1b->x, conflict.y2 + it1b->y);
          }
          it1b++;
        }
        it1a++;
        it2a++;
      }    
    }
    if (conflict.rc_type == 2) {
      int B_xmin = 999;
      int B_xmax = 0;
      int B_ymin = 999;
      int B_ymax = 0;
      std::vector<Location> shape1b = m_shapes_sheet[conflict.i2][conflict.o2] ;
      std::vector<Location> shape2b = m_shapes_sheet[conflict.i2][conflict.data1[2]] ;
      std::vector<Location>:: iterator it1b = shape1b.begin();
      std::vector<Location>:: iterator it2b = shape2b.begin();
      std::vector<Location> shape1a = m_shapes_sheet[conflict.i1][conflict.o1] ;

      while (it1b != shape1b.end() && it2b != shape2b.end())
      {
        B_xmin = std::min( conflict.x2+ it1b->x, conflict.data1[0]+ it2b->x) ;
        B_xmax = std::max( conflict.x2+ it1b->x, conflict.data1[0]+ it2b->x) ;
        B_ymin = std::min( conflict.y2+ it1b->y, conflict.data1[1]+ it2b->y) ;
        B_ymax = std::max( conflict.y2+ it1b->y, conflict.data1[1]+ it2b->y) ;
        std::vector<Location>:: iterator it1a = shape1a.begin();
        while (it1a != shape1a.end())
        {
          if (conflict.x1 + it1a->x >= B_xmin && conflict.x1 + it1a->x <= B_xmax && conflict.y1 + it1a->y >= B_ymin && conflict.y1 + it1a->y <= B_ymax) {
            return Location(conflict.x1 + it1a->x, conflict.y1 + it1a->y);
          }
          it1a++;
        }
        it1b++;
        it2b++;
      }
      
    }

    std::cout << "------Function FindConflictPoint_RCtypeOneTwo error! in environment.hpp------" << std::endl; 
    return Location(0, 0);

  }

  std::vector< std::pair<State, State> > ConflictPointStatePairSearch(
    const std::vector<Location>& p, const std::vector< std::vector<Location> >& shape_sheet, int agentType ) {
    
    std::vector< std::pair<State, State> > unableStatesPair;
    if (agentType == 1) {
      for (size_t i = 1; i < shape_sheet.size() ; i++) {
        for (size_t j = 0; j < shape_sheet[i].size(); j++) {
        State temp1(0, p[0].x - shape_sheet[i][j].x, p[0].y - shape_sheet[i][j].y, i );
        State temp2(0, p[1].x - shape_sheet[i][j].x, p[1].y - shape_sheet[i][j].y, i );
        std::pair<State, State> temp3(temp1, temp2);
        
        if (unableStatesPair.empty()) {
          unableStatesPair.push_back(temp3);
        }
        else{
          auto it = std::find(unableStatesPair.begin(), unableStatesPair.end(), temp3);
          if (it == unableStatesPair.end()) {
              unableStatesPair.push_back(temp3);
          }
        }
        }
      }
    }

    else if (agentType == 2) {
       for (size_t i = 1; i < shape_sheet.size() ; i++) {
        for (size_t j = 0; j < shape_sheet[i].size(); j++) {
        State temp1(0, p[0].x - shape_sheet[i][j].x, p[0].y - shape_sheet[i][j].y, i );
        State temp2(0, p[1].x - shape_sheet[i][j].x, p[1].y - shape_sheet[i][j].y, i );
        std::pair<State, State> temp3(temp2, temp1);
        
        if (unableStatesPair.empty()) {
          unableStatesPair.push_back(temp3);
        }
        else{
          auto it = std::find(unableStatesPair.begin(), unableStatesPair.end(), temp3);
          if (it == unableStatesPair.end()) {
              unableStatesPair.push_back(temp3);
          }
        }
        }
      }     
    }
    return unableStatesPair;
  }

  std::vector<Location> FindConflictPointPair(const Conflict& conflict) {
    Location s1a(conflict.x1, conflict.y1);
    Location s1b(conflict.x2, conflict.y2);
    Location s2a(conflict.data1[0], conflict.data1[1]);
    Location s2b(conflict.data1[2], conflict.data1[3]);
    std::vector<Location> FCPP;
    for (size_t i = 0; i < m_shapes_sheet[conflict.i1][conflict.o1].size(); i++)
    {
      State agentA_before(conflict.time, s1a.x + m_shapes_sheet[conflict.i1][conflict.o1][i].x, s1a.y + m_shapes_sheet[conflict.i1][conflict.o1][i].y, conflict.o1);
      State agentA_after(conflict.time + 1, s2a.x + m_shapes_sheet[conflict.i1][conflict.o1][i].x, s2a.y + m_shapes_sheet[conflict.i1][conflict.o1][i].y, conflict.o1);
      for (size_t j = 0; j < m_shapes_sheet[conflict.i2][conflict.o2].size(); j++)
      {
        State agentB_before(conflict.time, s1b.x + m_shapes_sheet[conflict.i2][conflict.o2][j].x, s1b.y + m_shapes_sheet[conflict.i2][conflict.o2][j].y, conflict.o2);
        State agentB_after(conflict.time + 1, s2b.x + m_shapes_sheet[conflict.i2][conflict.o2][j].x, s2b.y + m_shapes_sheet[conflict.i2][conflict.o2][j].y, conflict.o2);
        if (agentA_before.equalExceptTime(agentB_after) && agentA_after.equalExceptTime(agentB_before) )
        {
          FCPP.push_back(Location(agentA_before.x, agentA_before.y));
          FCPP.push_back(Location(agentB_before.x, agentB_before.y));
          return FCPP;
        }
      }
    }
    std::cout << "------Function FindConflictPointPair error! in environment.hpp------" << std::endl;
    FCPP.push_back(Location(0, 0));
    FCPP.push_back(Location(0, 0));
    return FCPP;
  }

  Location FindConflictPoint(const Conflict& conflict) {
   Location p1(conflict.x1, conflict.y1);
   Location p2(conflict.x2, conflict.y2);
   for (size_t i = 0; i < m_shapes_sheet[conflict.i1][conflict.o1].size(); i++) {
     for (size_t j = 0; j < m_shapes_sheet[conflict.i2][conflict.o2].size(); j++) {
      if ( p1.x + m_shapes_sheet[conflict.i1][conflict.o1][i].x == p2.x + m_shapes_sheet[conflict.i2][conflict.o2][j].x &&
           p1.y + m_shapes_sheet[conflict.i1][conflict.o1][i].y == p2.y + m_shapes_sheet[conflict.i2][conflict.o2][j].y
      )
      {
        return Location(p1.x + m_shapes_sheet[conflict.i1][conflict.o1][i].x, p1.y + m_shapes_sheet[conflict.i1][conflict.o1][i].y);
      }
     }
   }
   std::cout << "------Function FindConlictPoint error! in environment.hpp------" << std::endl;
   return Location(0, 0);
  }

  std::vector<State> ConflictPointStateSearch(
    const Location& p, const std::vector< std::vector<Location> >& shape_sheet) {
    
    std::vector<State> unableStates;
    for (size_t i = 1; i < shape_sheet.size() ; i++) {
      for (size_t j = 0; j < shape_sheet[i].size(); j++) {
        State temp(0, p.x - shape_sheet[i][j].x, p.y - shape_sheet[i][j].y, i );
        if (unableStates.empty()) {
          unableStates.push_back(temp);
        }
        else {
          auto it = std::find(unableStates.begin(), unableStates.end(), temp) ;
          if (it == unableStates.end()) {
            unableStates.push_back(temp);
          }
        }
        // num++;
        }          
    }
    return unableStates;
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }
  
    bool shapeValid(const State& s, size_t IDX)
  {
    bool shape_valid = true;
    std::vector<Location> shape_temp = m_shapes_sheet[IDX][s.o]; 
    Location s_temp(0, 0);
    Location s0(s.x, s.y) ;
    if (m_obstacles.find(s0+s_temp) != m_obstacles.end())
    { return false; }
    for (const auto& s1: shape_temp)
    {
      if (m_obstacles.find(s0+s1) != m_obstacles.end())
      { return false; }
    }
    int x_max = 0;
    int y_max = 0;
    int x_min = 20000;
    int y_min = 20000;
    for (const auto& s1 : shape_temp) {
        if (s1.x > x_max) {
            x_max = s1.x;
        }
        if (s1.y > y_max) {        
            y_max = s1.y;
        }
        if (s1.x < x_min) {
            x_min = s1.x;
        }
        if (s1.y < y_min) {
            y_min = s1.y;    
        }
    }
    shape_valid = (s.x + x_max <= m_dimx ) && (s.x + x_min >= 0) && (s.y + y_max <= m_dimy) && (s.y + y_min >= 0);

    return shape_valid;
  }
  std::vector<Location> move_shape(int o, std::vector<Location> shape_ )
  {
    std::vector<Location> shape_temp;
    Location s_(0, 0) ;

    switch (o)
    {
    case 1:
      shape_temp = shape_;
      break;
    
    case 2:
      for (const auto& s1: shape_)
      {
        s_.x = -s1.y;
        s_.y = s1.x;
        shape_temp.emplace_back(s_);
      }
      break;

    case 3:
      for (const auto& s1: shape_)
      {
        s_.x = -s1.x;
        s_.y = -s1.y;
        shape_temp.emplace_back(s_);
      }
      break;

    case 4:
      for (const auto& s1: shape_)
      {
        s_.x = s1.y;
        s_.y = -s1.x;
        shape_temp.emplace_back(s_);
      }
      break;
    default:
    shape_temp = shape_; 
      break;
    }
    return shape_temp;
  }

  bool AABB_check(const State& s1, const State& s2)
  {
    bool check_ = true;
    std::vector<Location>::iterator iter1;
    std::vector<Location>::iterator iter2;
    std::vector<Location> shape_temp1 = m_shapes_sheet[m_agentIdx][s1.o] ;
    std::vector<Location> shape_temp2 = m_shapes_sheet[m_agentIdx][s2.o];
    iter1 = shape_temp1.begin();
    iter2 = shape_temp2.begin();
    Location s_temp(0, 0);
    while(iter1 != shape_temp1.end() && iter2 != shape_temp2.end())
    {
        int x_min = std::min(s1.x + iter1->x, s2.x + iter2->x);
        int x_max = std::max(s1.x + iter1->x, s2.x + iter2->x);
        int y_min = std::min(s1.y + iter1->y, s2.y + iter2->y);
        int y_max = std::max(s1.y + iter1->y, s2.y + iter2->y);
        for (int i = x_min; i <= x_max; i++)
        {
          for (int j = y_min; j <= y_max; j++)
          {
            s_temp.x = i;
            s_temp.y = j;
            if (m_obstacles.find(s_temp) != m_obstacles.end())
            {
              check_ = false;
              return check_;
            }
          }
        }
        iter1++;
        iter2++;
    }
    return check_;
  }

  bool shape_vertex_collide(const State& s1, const State& s2, const std::vector<Location>& shape1, const std::vector<Location>& shape2)
  {
    std::vector<Location>::iterator iter1;
    std::vector<Location>::iterator iter2;
    std::vector<Location> shape_temp1 = shape1;
    std::vector<Location> shape_temp2 = shape2;

    iter1 = shape_temp1.begin();
    while(iter1 != shape_temp1.end())
    {
      iter2 = shape_temp2.begin();
      if (s1.x + iter1->x == s2.x && s1.y + iter1->y == s2.y) {return true;}
      while(iter2 != shape_temp2.end())
      {
        if (s1.x == s2.x + iter2->x && s1.y == s2.y + iter2->y) {return true;}
        if (s1.x + iter1->x == s2.x + iter2->x && s1.y + iter1->y == s2.y + iter2->y)
        {
          return true;
        }
        iter2++;
      }
      
      iter1++;
    }
    return false;
  }

  bool shape_edge_collide(const State& s1a, const State& s2a, const State& s1b, const State& s2b, int i, int j)
  {
      if (s1a.o == s2a.o && s1b.o == s2b.o) {
          std::vector<Location> shape1a = m_shapes_sheet[i][s1a.o] ;
          std::vector<Location> shape1b = m_shapes_sheet[j][s1b.o] ;
          std::vector<Location>::iterator iter1 = shape1a.begin();
          while (iter1 != shape1a.end())
          {
            std::vector<Location>::iterator iter2 = shape1b.begin();
            State agentA_before(s1a.time, s1a.x + iter1->x, s1a.y + iter1->y, s1a.o);
            State agnetA_after(s2a.time, s2a.x + iter1->x, s2a.y + iter1->y, s2a.o);
            while (iter2 != shape1b.end())
            {
                State agentB_before(s1b.time, s1b.x + iter2->x, s1b.y + iter2->y, s1b.o);
                State agentB_after(s2b.time, s2b.x + iter2->x, s2b.y + iter2->y, s2b.o);
                if (agentA_before.equalExceptTime(agentB_after) && agnetA_after.equalExceptTime(agentB_before) )
                {
                    return true;
                }
                iter2++;
            }
            iter1++;
          }
      }
      return false;
  }


 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, float> >& solution,
                 size_t t) {
    assert(agentIdx < solution.size());
    
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    if (m_disappearAtGoal) {
      Location obs_temp(solution[agentIdx].states.back().first.x, solution[agentIdx].states.back().first.y);
      m_obstacles.emplace(obs_temp);
      std::vector<Location> shape_add2_obs = m_shapes_sheet[agentIdx][solution[agentIdx].states.back().first.o] ;
      for (size_t i = 0; i < shape_add2_obs.size(); i++)
      {
        Location obs_temp(shape_add2_obs[i].x, shape_add2_obs[i].y);
        m_obstacles.emplace(obs_temp);
      }
      return State(-10 * (agentIdx + 1), -20 * (agentIdx + 1), -1, 1);  
    }
    return solution[agentIdx].states.back().first;
  }

  bool stateValid(const State& s, size_t agentIdx) {
    assert(m_constraints); 
    const auto& con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
           con.find(VertexConstraint(s.time, s.x, s.y, s.o, agentIdx)) == con.end() && shapeValid(s, m_agentIdx);
  }

  

  bool transitionValid(const State& s1, const State& s2) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return ( con.find(EdgeConstraint(s1.time, s1.x, s1.y,s1.o, s2.x, s2.y, s2.o )) ==
           con.end() );
  }



 private:
  int m_dimx;
  int m_dimy;
  std::unordered_set<Location> m_obstacles;
  std::vector<Location> m_goals;
  // std::vector< std::vector<int> > m_heuristic;
  size_t m_agentIdx;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  bool m_disappearAtGoal;
  std::vector< std::vector<Location> > m_shapes;
  std::vector< std::vector< std::vector<Location> > > m_shapes_sheet;
  std::vector<int> m_h1_list;
};
