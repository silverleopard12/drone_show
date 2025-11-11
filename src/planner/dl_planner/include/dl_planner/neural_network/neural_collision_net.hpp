#ifndef DL_PLANNER_NEURAL_COLLISION_NET_HPP
#define DL_PLANNER_NEURAL_COLLISION_NET_HPP

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <memory>

namespace dl_planner {

/**
 * @brief Configuration for neural network
 */
struct NeuralNetConfig {
  std::string model_path;           // Path to PyTorch model
  int input_dim = 12;                // Input dimension (2 drones * 6 states)
  int hidden_dim = 64;               // Hidden layer dimension
  int output_dim = 1;                // Output dimension (collision probability)
  double collision_threshold = 0.5;  // Threshold for collision prediction
  bool use_gpu = false;              // Use GPU for inference

  NeuralNetConfig() = default;
};

/**
 * @brief Drone state for neural network input
 */
struct DroneState {
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  double timestamp;
  int drone_id;

  DroneState() : position(Eigen::Vector3d::Zero()),
                 velocity(Eigen::Vector3d::Zero()),
                 timestamp(0.0),
                 drone_id(-1) {}

  // Convert to feature vector
  Eigen::VectorXd toFeatureVector() const {
    Eigen::VectorXd features(6);
    features << position, velocity;
    return features;
  }
};

/**
 * @brief Neural network for collision prediction
 *
 * This class wraps a PyTorch model for predicting collision probabilities
 * between drones based on their current states.
 */
class NeuralCollisionNet {
public:
  using Ptr = std::shared_ptr<NeuralCollisionNet>;

  explicit NeuralCollisionNet(const NeuralNetConfig& config);
  ~NeuralCollisionNet();

  /**
   * @brief Load pre-trained model from file
   */
  bool loadModel(const std::string& model_path);

  /**
   * @brief Predict collision probability between two drones
   *
   * @param drone1 State of first drone
   * @param drone2 State of second drone
   * @return Collision probability [0, 1]
   */
  double predictCollisionProbability(
    const DroneState& drone1,
    const DroneState& drone2
  );

  /**
   * @brief Predict control action to avoid collision
   *
   * @param ego_state Current state of ego drone
   * @param other_state State of other drone
   * @return Recommended velocity adjustment
   */
  Eigen::Vector3d predictControlAction(
    const DroneState& ego_state,
    const DroneState& other_state
  );

  /**
   * @brief Batch prediction for multiple drone pairs
   *
   * @param states Vector of drone states
   * @return Matrix of pairwise collision probabilities
   */
  Eigen::MatrixXd batchPredict(const std::vector<DroneState>& states);

  /**
   * @brief Check if collision is predicted
   */
  bool isCollisionPredicted(double probability) const {
    return probability > config_.collision_threshold;
  }

  /**
   * @brief Get model information
   */
  std::string getModelInfo() const;

  /**
   * @brief Check if model is loaded
   */
  bool isModelLoaded() const { return model_loaded_; }

private:
  NeuralNetConfig config_;
  bool model_loaded_;

  // Simple MLP implementation (placeholder for PyTorch/LibTorch)
  struct SimpleMLPWeights {
    Eigen::MatrixXd W1;  // First layer weights
    Eigen::VectorXd b1;  // First layer bias
    Eigen::MatrixXd W2;  // Second layer weights
    Eigen::VectorXd b2;  // Second layer bias
    Eigen::MatrixXd W3;  // Output layer weights
    Eigen::VectorXd b3;  // Output layer bias
  };

  SimpleMLPWeights weights_;

  /**
   * @brief Initialize random weights (for testing without trained model)
   */
  void initializeRandomWeights();

  /**
   * @brief Forward pass through network
   */
  double forwardPass(const Eigen::VectorXd& input);

  /**
   * @brief ReLU activation
   */
  Eigen::VectorXd relu(const Eigen::VectorXd& x);

  /**
   * @brief Sigmoid activation
   */
  double sigmoid(double x);

  /**
   * @brief Normalize input features
   */
  Eigen::VectorXd normalizeInput(const Eigen::VectorXd& input);

  /**
   * @brief Create input feature vector from two drone states
   */
  Eigen::VectorXd createInputFeatures(
    const DroneState& drone1,
    const DroneState& drone2
  );
};

/**
 * @brief Factory for creating neural network
 */
class NeuralCollisionNetFactory {
public:
  static NeuralCollisionNet::Ptr create(const NeuralNetConfig& config) {
    return std::make_shared<NeuralCollisionNet>(config);
  }

  static NeuralCollisionNet::Ptr createFromFile(const std::string& model_path) {
    NeuralNetConfig config;
    config.model_path = model_path;
    auto net = std::make_shared<NeuralCollisionNet>(config);
    net->loadModel(model_path);
    return net;
  }
};

} // namespace dl_planner

#endif // DL_PLANNER_NEURAL_COLLISION_NET_HPP
