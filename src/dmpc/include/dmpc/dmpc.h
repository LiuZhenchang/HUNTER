#ifndef DMPC_H
#define DMPC_H

#include <search_env/search_environment.h>
#include <iomanip>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include "dmpc/cvt_system_polygon.h"
#include "openGA.hpp"

class DMPC {
 public:
  DMPC();
  ~DMPC();

  void initDmpc(rclcpp::Node::SharedPtr node);
  void init(rclcpp::Node::SharedPtr node);
  std::vector<PredictState> RH_GA();
  std::pair<double, double> jA(const std::vector<PredictState> &predict_states);
  std::pair<double, double> jB(const std::vector<PredictState> &predictStates, double initial_yaw);
  double jC(const std::vector<PredictState> &predict_states);
  double jD(const std::vector<PredictState> &position, const Eigen::Vector2d &current_position, double wr_base);

  void setAgentsStateHistory(std::vector<std::deque<AgentState>> *agents_history) {
    agents_state_history_ = agents_history;
  }

  void setAgentParam(AgentParam *agent_param) {
    agent_param_ = agent_param;
  }

  void setEnvironment(std::shared_ptr<SearchEnvironment> env) {
    env_ = env;
  }

  void setStartPositonMode(bool use_next_target, const PredictState *next_target) {
    use_next_target_ = use_next_target;
    next_target_ = next_target;
  }

  void setCurrentRealtimePosition(const Eigen::Vector3d &position, double yaw);
  bool getActiveTargetPosition(Eigen::Vector2d &target_position) const;
  void markTargetTaskCompleted();

  // Check whether the path is safe
  bool isPathSafe(const Eigen::Vector2d &start, const Eigen::Vector2d &end, const std::vector<Obstacle> &obstacles);

  // Compute the role direction
  Eigen::Vector2d calculateRoleDirection(int agent_id, int agent_num);

  double computeRoleCost(const Eigen::Vector2d &position, const Eigen::Vector2d &current_position,
                         const Eigen::Vector2d &role_direction, double wr);

  // Initialize the CVT system
  void initCVTSystem();

  std::vector<PredictState> solve();

  Eigen::Vector2d getAgentPosition(int agent_id) const;

  // Getlatesttarget information
  const std::vector<Target> &getLatestTargetsInfo() const;

  // expert system
  bool Expert_System();

  struct GA_Params {
    int population_size;
    int generation_max;
    double crossover_fraction;
    double mutation_rate;
    int best_stall_max;
    int elite_count;
    int max_steps;
  };

 private:
  // dynamic multi-target assignment
  bool isTargetTracker(int agent_id, int agent_num, int target_num);
  int getAssignedTargetIndex(int agent_id, int agent_num, int target_num);
  std::vector<std::pair<int, int>> computeDynamicTargetAssignment(int agent_num, int target_num);
  Eigen::Vector2d calculateExplorationDirection(int agent_id, int agent_num, int target_num,
                                                const Eigen::Vector2d &current_pos, const std::vector<Target> &targets);
  double computeAssignmentCost(const std::vector<std::pair<int, int>> &assignment,
                               std::vector<Eigen::Vector2d> agent_pos, std::vector<Target> target_pos);
  std::vector<std::pair<int, int>> lastTargetAssignment_;
  rclcpp::Time last_update_time_;
  bool is_first_assignment_ = true;
  double hysteresis_ratio_ = 0.85;
  double update_period_ = 3.0;
  WeightedCVTPolygon *cvt_system_;
  std::unordered_map<int, Eigen::Vector2d> initial_agent_positions_;
  bool use_initial_task_assignment = false;
  std::vector<int> explorer_ids_;    // explorer UAV ID list
  std::vector<int> cvt_assignment_;  // CVT index assigned to each explorer UAV
  bool cvt_assigned_ = false;        // CVTwhether assigned
 private:
  std::vector<PredictState> getCurrentPositionTrajectory();
  Eigen::Vector3d current_realtime_position_;
  double current_realtime_yaw_;
  std::mutex realtime_position_mutex_;

 private:
  AgentParam *agent_param_ = nullptr;
  std::vector<std::deque<AgentState>> *agents_state_history_ = nullptr;
  std::vector<PredictState> predict_state_;
  bool use_next_target_ = false;
  const PredictState *next_target_ = nullptr;
  std::shared_ptr<SearchEnvironment> env_;

  rclcpp::Node::SharedPtr node_;
  GA_Params ga_params_;
  int path_points, target_num;
  int agent_num, explore_uav_num;
  double v_n, map_resolution;
  Eigen::Vector2d role_direction;
  bool role_direction_initialized_ = false;
  Eigen::Vector2d role_target_position_;
  bool role_target_initialized_ = false;
  bool target_task_completed_ = false;
  double target_switch_distance_ = 80.0;
  double target_arrival_distance_ = 20.0;
  std::vector<Eigen::Vector2d> explorer_positions;
  std::vector<Eigen::Vector2d> region_vertices_;
  std::vector<std::vector<Eigen::Vector2d>> no_fly_zones_;  // multiple no-fly zones
  bool enable_no_fly_zone_ = true;

 private:
  // optimization cache
  struct GridCache {
    std::vector<int> covered_grids;
    std::pair<double, double> benefit;
    bool valid = false;
  };

  std::unordered_map<std::string, GridCache> grid_cache_;
  std::string last_trajectory_hash_;
  std::mutex grid_cache_mutex_;
  std::shared_mutex shared_mutex_;
  std::string generateTrajectoryHash(const std::vector<PredictState> &trajectory);
  rclcpp::Logger getLogger() const;

 private:
  struct MySolution {
    std::vector<int> actions;  // action sequence
    std::string to_string() const {
      std::string s = "[";
      for (size_t i = 0; i < actions.size(); ++i) {
        s += std::to_string(actions[i]);
        if (i != actions.size() - 1)
          s += ",";
      }
      return s;
    }
  };

  struct MyMiddleCost {
    double total_reward;  // total reward
    std::vector<PredictState> predict_states;

    bool reward_breakdown_valid = false;
    double raw_jA1 = 0.0;
    double raw_jA2 = 0.0;
    double raw_jB_turn = 0.0;
    double raw_jB_explored = 0.0;
    double raw_jC = 0.0;
    double raw_jD = 0.0;
    double norm_jA1 = 0.0;
    double norm_jA2 = 0.0;
    double norm_jB = 0.0;
    double norm_jC = 0.0;
    double norm_jD = 0.0;
    double contribution_jA1 = 0.0;
    double contribution_jA2 = 0.0;
    double contribution_jB = 0.0;
    double contribution_jC = 0.0;
    double contribution_jD = 0.0;
  };

  typedef EA::Genetic<MySolution, MyMiddleCost> GA_Type;
  typedef EA::GenerationType<MySolution, MyMiddleCost> Generation_type;

  void init_genes(MySolution &p, const std::function<double(void)> &rnd01);
  bool evaluateFitness(const MySolution &p, MyMiddleCost &c, double dt = 2.0,
                       bool use_large_angles = false);  // fitness function
  std::vector<PredictState> generatePredictState(
      std::vector<double> yaw_seq, double dt = 2.0, bool use_next_target = false,
      const PredictState *next_target = nullptr);  // Generate the predicted state sequence
  MySolution mutate(const MySolution &X_base, const std::function<double(void)> &rnd01, double shrink_scale);
  MySolution crossover(const MySolution &p1, const MySolution &p2, const std::function<double(void)> &rnd01);
  double calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X);
  void SO_report_generation(int generation_number, const Generation_type &last_generation,
                            const MySolution &best_genes);  // Report each generation
};

#endif  // DMPC_H
