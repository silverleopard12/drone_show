#ifndef RL_PLANNER_MLP_POLICY_HPP
#define RL_PLANNER_MLP_POLICY_HPP

#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <random>

namespace rl_planner {

/**
 * @brief Configuration for MLP policy
 */
struct MLPPolicyConfig {
  int state_dim;
  int action_dim;
  std::vector<int> hidden_dims = {256, 256};  // Hidden layer sizes

  double learning_rate = 3e-4;
  double clip_ratio = 0.2;        // PPO clip ratio
  double entropy_coef = 0.01;     // Entropy coefficient
  double value_coef = 0.5;        // Value loss coefficient

  bool use_layer_norm = true;
  bool use_orthogonal_init = true;

  // Action space
  Eigen::Vector3d action_min = Eigen::Vector3d(-3.0, -3.0, -3.0);
  Eigen::Vector3d action_max = Eigen::Vector3d(3.0, 3.0, 3.0);

  MLPPolicyConfig() : state_dim(0), action_dim(0) {}
};

/**
 * @brief Simple MLP weights for C++ implementation
 */
struct MLPWeights {
  // Actor network (policy)
  std::vector<Eigen::MatrixXd> actor_weights;
  std::vector<Eigen::VectorXd> actor_biases;

  // Critic network (value function)
  std::vector<Eigen::MatrixXd> critic_weights;
  std::vector<Eigen::VectorXd> critic_biases;

  // Action distribution parameters
  Eigen::MatrixXd action_mean_layer;
  Eigen::VectorXd action_mean_bias;
  Eigen::VectorXd action_logstd;  // Learnable log std
};

/**
 * @brief Action distribution output
 */
struct ActionDistribution {
  Eigen::Vector3d mean;
  Eigen::Vector3d std;

  /**
   * @brief Sample action from distribution
   */
  Eigen::Vector3d sample(std::mt19937& rng) const;

  /**
   * @brief Compute log probability
   */
  double logProb(const Eigen::Vector3d& action) const;

  /**
   * @brief Compute entropy
   */
  double entropy() const;
};

/**
 * @brief Multi-Layer Perceptron Policy Network
 *
 * Implements an actor-critic architecture for RL:
 * - Actor: Outputs action distribution (mean and std)
 * - Critic: Outputs value estimate
 */
class MLPPolicy {
public:
  using Ptr = std::shared_ptr<MLPPolicy>;

  explicit MLPPolicy(const MLPPolicyConfig& config);
  ~MLPPolicy();

  /**
   * @brief Get action from policy (deterministic or stochastic)
   */
  Eigen::Vector3d getAction(const Eigen::VectorXd& state, bool deterministic = false);

  /**
   * @brief Get action distribution
   */
  ActionDistribution getActionDistribution(const Eigen::VectorXd& state);

  /**
   * @brief Get value estimate
   */
  double getValue(const Eigen::VectorXd& state);

  /**
   * @brief Evaluate actions (for training)
   * Returns: values, log_probs, entropy
   */
  struct EvaluateResult {
    std::vector<double> values;
    std::vector<double> log_probs;
    std::vector<double> entropies;
  };

  EvaluateResult evaluate(
    const std::vector<Eigen::VectorXd>& states,
    const std::vector<Eigen::Vector3d>& actions
  );

  /**
   * @brief Save model to file
   */
  bool saveModel(const std::string& filepath);

  /**
   * @brief Load model from file
   */
  bool loadModel(const std::string& filepath);

  /**
   * @brief Get model info
   */
  std::string getModelInfo() const;

  /**
   * @brief Initialize weights
   */
  void initializeWeights();

private:
  MLPPolicyConfig config_;
  MLPWeights weights_;
  std::mt19937 rng_;

  /**
   * @brief Forward pass through actor network
   */
  Eigen::VectorXd actorForward(const Eigen::VectorXd& state);

  /**
   * @brief Forward pass through critic network
   */
  double criticForward(const Eigen::VectorXd& state);

  /**
   * @brief Activation functions
   */
  Eigen::VectorXd relu(const Eigen::VectorXd& x);
  Eigen::VectorXd tanh(const Eigen::VectorXd& x);
  double sigmoid(double x);

  /**
   * @brief Layer normalization
   */
  Eigen::VectorXd layerNorm(const Eigen::VectorXd& x);

  /**
   * @brief Orthogonal initialization
   */
  void orthogonalInit(Eigen::MatrixXd& matrix, double gain = 1.0);
};

} // namespace rl_planner

#endif // RL_PLANNER_MLP_POLICY_HPP
