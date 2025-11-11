#include "dl_planner/neural_network/neural_collision_net.hpp"
#include <iostream>
#include <fstream>
#include <cmath>
#include <random>

namespace dl_planner {

NeuralCollisionNet::NeuralCollisionNet(const NeuralNetConfig& config)
  : config_(config),
    model_loaded_(false)
{
  // Initialize with random weights for testing
  initializeRandomWeights();
}

NeuralCollisionNet::~NeuralCollisionNet() {
}

bool NeuralCollisionNet::loadModel(const std::string& model_path) {
  std::cout << "[NeuralCollisionNet] Attempting to load model from: "
            << model_path << std::endl;

  // TODO: Implement actual PyTorch/LibTorch model loading
  // For now, use placeholder implementation

  std::ifstream file(model_path);
  if (!file.good()) {
    std::cerr << "[NeuralCollisionNet] Model file not found: "
              << model_path << std::endl;
    std::cout << "[NeuralCollisionNet] Using randomly initialized weights for testing."
              << std::endl;
    initializeRandomWeights();
    model_loaded_ = true;
    return true;
  }

  // If file exists, try to load (placeholder)
  std::cout << "[NeuralCollisionNet] Model file found. Loading weights..." << std::endl;

  // TODO: Parse model file and load actual weights
  // For now, just use random initialization
  initializeRandomWeights();

  model_loaded_ = true;
  std::cout << "[NeuralCollisionNet] Model loaded successfully (placeholder)." << std::endl;

  return true;
}

double NeuralCollisionNet::predictCollisionProbability(
    const DroneState& drone1,
    const DroneState& drone2)
{
  if (!model_loaded_) {
    std::cerr << "[NeuralCollisionNet] Warning: Model not loaded. "
              << "Using random initialization." << std::endl;
    initializeRandomWeights();
    model_loaded_ = true;
  }

  // Create input features
  Eigen::VectorXd input = createInputFeatures(drone1, drone2);

  // Normalize input
  input = normalizeInput(input);

  // Forward pass
  double probability = forwardPass(input);

  return probability;
}

Eigen::Vector3d NeuralCollisionNet::predictControlAction(
    const DroneState& ego_state,
    const DroneState& other_state)
{
  // Predict collision probability
  double collision_prob = predictCollisionProbability(ego_state, other_state);

  // If collision likely, generate avoidance action
  if (collision_prob > config_.collision_threshold) {
    // Compute repulsive velocity away from other drone
    Eigen::Vector3d relative_pos = ego_state.position - other_state.position;
    double distance = relative_pos.norm();

    if (distance < 1e-3) {
      // Too close, use random direction
      std::random_device rd;
      std::mt19937 gen(rd());
      std::normal_distribution<> d(0.0, 1.0);
      relative_pos = Eigen::Vector3d(d(gen), d(gen), d(gen));
      relative_pos.normalize();
    } else {
      relative_pos.normalize();
    }

    // Scale by collision probability
    double avoidance_gain = 2.0;  // m/s
    Eigen::Vector3d avoidance_vel = relative_pos * collision_prob * avoidance_gain;

    return avoidance_vel;
  }

  // No avoidance needed
  return Eigen::Vector3d::Zero();
}

Eigen::MatrixXd NeuralCollisionNet::batchPredict(
    const std::vector<DroneState>& states)
{
  int n = states.size();
  Eigen::MatrixXd collision_matrix = Eigen::MatrixXd::Zero(n, n);

  // Compute pairwise collision probabilities
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      double prob = predictCollisionProbability(states[i], states[j]);
      collision_matrix(i, j) = prob;
      collision_matrix(j, i) = prob;  // Symmetric
    }
  }

  return collision_matrix;
}

std::string NeuralCollisionNet::getModelInfo() const {
  std::stringstream ss;
  ss << "Neural Collision Network Info:\n";
  ss << "  Model path: " << config_.model_path << "\n";
  ss << "  Input dim: " << config_.input_dim << "\n";
  ss << "  Hidden dim: " << config_.hidden_dim << "\n";
  ss << "  Output dim: " << config_.output_dim << "\n";
  ss << "  Collision threshold: " << config_.collision_threshold << "\n";
  ss << "  Model loaded: " << (model_loaded_ ? "Yes" : "No") << "\n";
  return ss.str();
}

void NeuralCollisionNet::initializeRandomWeights() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> d(0.0, 0.1);

  // Initialize weights with small random values
  // Layer 1: input_dim -> hidden_dim
  weights_.W1 = Eigen::MatrixXd(config_.hidden_dim, config_.input_dim);
  weights_.b1 = Eigen::VectorXd(config_.hidden_dim);

  for (int i = 0; i < config_.hidden_dim; ++i) {
    for (int j = 0; j < config_.input_dim; ++j) {
      weights_.W1(i, j) = d(gen);
    }
    weights_.b1(i) = d(gen);
  }

  // Layer 2: hidden_dim -> hidden_dim
  weights_.W2 = Eigen::MatrixXd(config_.hidden_dim, config_.hidden_dim);
  weights_.b2 = Eigen::VectorXd(config_.hidden_dim);

  for (int i = 0; i < config_.hidden_dim; ++i) {
    for (int j = 0; j < config_.hidden_dim; ++j) {
      weights_.W2(i, j) = d(gen);
    }
    weights_.b2(i) = d(gen);
  }

  // Layer 3: hidden_dim -> output_dim
  weights_.W3 = Eigen::MatrixXd(config_.output_dim, config_.hidden_dim);
  weights_.b3 = Eigen::VectorXd(config_.output_dim);

  for (int i = 0; i < config_.output_dim; ++i) {
    for (int j = 0; j < config_.hidden_dim; ++j) {
      weights_.W3(i, j) = d(gen);
    }
    weights_.b3(i) = d(gen);
  }

  std::cout << "[NeuralCollisionNet] Initialized random weights." << std::endl;
}

double NeuralCollisionNet::forwardPass(const Eigen::VectorXd& input) {
  // Layer 1
  Eigen::VectorXd h1 = weights_.W1 * input + weights_.b1;
  h1 = relu(h1);

  // Layer 2
  Eigen::VectorXd h2 = weights_.W2 * h1 + weights_.b2;
  h2 = relu(h2);

  // Output layer
  Eigen::VectorXd output = weights_.W3 * h2 + weights_.b3;

  // Apply sigmoid to get probability
  return sigmoid(output(0));
}

Eigen::VectorXd NeuralCollisionNet::relu(const Eigen::VectorXd& x) {
  return x.cwiseMax(0.0);
}

double NeuralCollisionNet::sigmoid(double x) {
  return 1.0 / (1.0 + std::exp(-x));
}

Eigen::VectorXd NeuralCollisionNet::normalizeInput(const Eigen::VectorXd& input) {
  // Simple normalization: assume positions in [-50, 50], velocities in [-10, 10]
  Eigen::VectorXd normalized(config_.input_dim);

  // Normalize positions (first 6 dimensions)
  for (int i = 0; i < 6; ++i) {
    normalized(i) = input(i) / 50.0;
  }

  // Normalize velocities (last 6 dimensions)
  for (int i = 6; i < 12; ++i) {
    normalized(i) = input(i) / 10.0;
  }

  return normalized;
}

Eigen::VectorXd NeuralCollisionNet::createInputFeatures(
    const DroneState& drone1,
    const DroneState& drone2)
{
  Eigen::VectorXd features(config_.input_dim);

  // Concatenate both drone states
  features << drone1.position,
              drone1.velocity,
              drone2.position,
              drone2.velocity;

  return features;
}

} // namespace dl_planner
