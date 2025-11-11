#ifndef HEADWAY_PLANNER_MILP_SCHEDULER_HPP
#define HEADWAY_PLANNER_MILP_SCHEDULER_HPP

#include "headway_planner/scheduler/headway_scheduler.hpp"

namespace headway_planner {

/**
 * @brief MILP-based headway scheduler
 *
 * Formulation:
 *   Variables: phi_i >= 0 for each drone i
 *              y_ijk in {0, 1} for each event k=(i,j)
 *
 *   Minimize: T_max (makespan)
 *
 *   Subject to:
 *     For each event k=(i,j):
 *       t_j - t_i >= delta_t_req - M*(1 - y_ijk)
 *       t_i - t_j >= delta_t_req - M*y_ijk
 *       where t_i = phi_i + s_i/v_des
 *
 *     0 <= phi_i <= T_max
 *     T_max >= phi_i + L_i/v_des  for all i
 *
 * Solvers: CBC (open-source), SCIP, Gurobi (commercial)
 *
 * Note: MILP gives optimal solution but slower than greedy.
 * Recommended: Use greedy first, then MILP to optimize if needed.
 */
class MILPScheduler : public HeadwayScheduler {
public:
  explicit MILPScheduler(const PlannerConfig& config);
  ~MILPScheduler() override;

  /**
   * @brief Solve using MILP
   */
  bool solve(
    std::vector<DroneInfo>& drones,
    const std::vector<CollisionEvent>& events
  ) override;

  /**
   * @brief Set solver type
   * @param solver "cbc", "scip", "gurobi"
   */
  void setSolverType(const std::string& solver) { solver_type_ = solver; }

  /**
   * @brief Set time limit for solver
   */
  void setTimeLimit(double seconds) { time_limit_ = seconds; }

  /**
   * @brief Set warm start (initial solution)
   *
   * Provide initial headways (e.g., from greedy)
   * to speed up MILP solving.
   */
  void setWarmStart(const std::vector<double>& initial_phi);

  /**
   * @brief Enable/disable warm start
   */
  void enableWarmStart(bool enable) { use_warm_start_ = enable; }

private:
  std::string solver_type_;
  double time_limit_;
  bool use_warm_start_;
  std::vector<double> warm_start_phi_;

  /**
   * @brief Build and solve MILP problem
   *
   * TODO: Implement with CBC library
   * Reference: https://github.com/coin-or/Cbc
   */
  bool solveCBC(
    std::vector<DroneInfo>& drones,
    const std::vector<CollisionEvent>& events
  );

  /**
   * @brief Compute big-M value
   *
   * M should be large enough but not too large (numerical issues)
   * M = 2 * T_max is usually sufficient
   */
  double computeBigM() const;
};

} // namespace headway_planner

#endif // HEADWAY_PLANNER_MILP_SCHEDULER_HPP
