#pragma once

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/heap/d_ary_heap.hpp>

typedef struct Action {
  double dphi;
  double dheight;
  double step_length;
  double x_dot;
  double z_apex;
  Action(const double _dphi = 0, const double _dhg = 0,
         const double _step_length = 0, const double _x_dot = 0,
         const double _z_apex = 0) {
    dphi = _dphi;
    dheight = _dhg;
    step_length = _step_length;
    x_dot = _x_dot;
    z_apex = _z_apex;
  }
} Action;

template <typename T>
class Primitives {
 public:
  std::vector<T> step_lengths;

  std::vector<std::vector<double>> path;

  double s;

  Primitives() {}

  double NextPhi(T phase);

  double CurrentPhi() const { return action_list[action_ls_pos].dphi * s; }

  std::vector<Action> step_actions(int ns_ind);

  Action step_action(int ns_ind, double _phi);

  double CurrentHeight(T phase) const {
    if (phase == 0) return 0;
    return action_list[action_ls_pos].dheight;
  }  // added on 8/14

  std::vector<double> CurrentCoord(T phase) const {
    if (phase == 0) return {0, 0, 0};
    return path[action_ls_pos];
  }
  double CurrentLength(T phase) const {
    if (phase == 0) return 0;
    return action_list[action_ls_pos].step_length;
  }

  void add_to_step_lengths(T value) { step_lengths.push_back(value); }

  std::vector<T> get_step_lengths() { return step_lengths; }

  void InitActionList(std::vector<Action> list) { action_list = list; }

  void SetS(double val) { s = val; }

  double GetS() { return s; }

  bool IsEnd() { return end; }

  void CreateRegMap(std::vector<int> reg_ind,
                    std::vector<std::string> reg_type);

 private:
  unsigned int action_ls_pos = 0;
  std::map<int, std::string> reg_map;
  std::vector<Action> action_list;
  bool end = false;
  std::map<std::string, double> step = {
      {"large", 0.6}, {"medium", 0.5}, {"small", 0.4}};
  std::map<std::string, double> vel = {
      {"large", 0.8}, {"medium", 0.6}, {"small", 0.4}};
  std::map<std::string, double> height = {
      {"large", 0.9}, {"medium", 0.8}, {"small", 0.7}};
  std::vector<int> yaw = {-1, 0, 1};
  std::vector<int> hgt;
};

template <typename T>
void Primitives<T>::CreateRegMap(std::vector<int> reg_ind,
                                 std::vector<std::string> reg_type) {
  for (unsigned i = 0; i < reg_ind.size(); i++)
    reg_map[reg_ind[i]] = reg_type[i];
}

template <typename T>
double Primitives<T>::NextPhi(T phase) {
  if (phase == 0) return 0;
  if (action_ls_pos < action_list.size() - 1) {
    action_ls_pos += 1;
  } else
    end = true;
  return action_list[action_ls_pos].dphi * s;
}

template <typename T>
std::vector<Action> Primitives<T>::step_actions(int ns_ind) {
  // set the compass gait to stop
  // stair: step small, height large, vel small
  // ground to lower level: step small, height small,vel small
  // to ground: height med: vel large, step large, step heigh med, vel high
  // to upper level: vel med, step med, height med
  std::string next_state = reg_map[ns_ind];

  Action temp;

  if (next_state == "stop") {
    temp.step_length = 0;
    temp.x_dot = 0;
    temp.z_apex = 0;
  } else if (next_state == "stair") {
    temp.step_length = step["small"];
    temp.x_dot = vel["small"];
    temp.z_apex = height["large"];
  } else if (next_state == "ground") {
    temp.step_length = step["large"];
    temp.x_dot = vel["large"];
    temp.z_apex = height["medium"];
  } else if (next_state == "lower") {
    temp.step_length = step["small"];
    temp.x_dot = vel["small"];
    temp.z_apex = height["small"];
  } else {
    temp.step_length = step["medium"];
    temp.x_dot = vel["medium"];
    temp.z_apex = height["medium"];
  }

  std::vector<Action> result(yaw.size(), temp);
  for (unsigned i = 0; i < yaw.size(); i++) {
    result[i].dphi = yaw[i];
  }
  return result;
}

template <typename T>
Action Primitives<T>::step_action(int ns_ind, double _phi) {
  // set the compass gait to stop
  // stair: step small, height large, vel small
  // ground to lower level: step small, height small,vel small
  // to ground: height med: vel large, step large, step heigh med, vel high
  // to upper level: vel med, step med, height med

  std::string next_state = reg_map[ns_ind];
  Action temp;

  if (next_state == "stop") {
    temp.step_length = 0;
    temp.x_dot = 0;
    temp.z_apex = 0;
  } else if (next_state == "stair") {
    temp.step_length = step["small"];
    temp.x_dot = vel["small"];
    temp.z_apex = height["large"];
  } else if (next_state == "ground") {
    temp.step_length = step["large"];
    temp.x_dot = vel["large"];
    temp.z_apex = height["medium"];
  } else if (next_state == "lower") {
    temp.step_length = step["small"];
    temp.x_dot = vel["small"];
    temp.z_apex = height["small"];
  } else {
    temp.step_length = step["medium"];
    temp.x_dot = vel["medium"];
    temp.z_apex = height["medium"];
  }

  temp.dphi = _phi;
  return temp;
}

template <class state>
struct ComparePair {
  bool operator()(const std::shared_ptr<state> p1,
                  const std::shared_ptr<state> p2) const {
    return (p1->g + p1->h) > (p2->g + p2->h);
  }
};

template <class state>
using priorityQueue =
    boost::heap::d_ary_heap<std::shared_ptr<state>, boost::heap::mutable_<true>,
                            boost::heap::arity<2>,
                            boost::heap::compare<ComparePair<state>>>;

typedef struct State {
  //{[x,y,z,phi,automationstate,primitives]}
  std::vector<double> coord;
  std::shared_ptr<State> parent;
  typename priorityQueue<State>::handle_type heapkey;
  float g = std::numeric_limits<float>::infinity();
  float h;
  int closed_iteration = 0;
  Action action;
  unsigned switch_num = 0;
  State(const std::vector<double>& crd) : coord(crd) {}
} State;
