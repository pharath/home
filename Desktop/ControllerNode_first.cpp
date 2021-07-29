#include <cstdio>
#include "ControllerNode.hpp"
//#include "util.hpp"

#include <control/pose.hpp>
#include <galaxis/node_ros_pubsub_wrapper.hpp>

#include <utils/logger.hpp>

// Onboarding: Your own inlcudes
#include <vector>
#include <utils/file.hpp>
#include <galaxis/controllerSettings.hpp>
#include <control/control.hpp>

using namespace galaxis::control;
using namespace galaxis::ros;
using namespace galaxis::utils;

using std::placeholders::_1;


ControllerNode::ControllerNode() :
    GalaxisNode("OnboardingControllerNode") {

  log("[OnboardingControllerNode] Starting...");

  // Create a ROS2 Subscription for the trajectory polynoms
  galaxis::ros::create_subscription(this,
                                    m_pathPolynomSubSPtr,
                                    std::bind(&ControllerNode::on_pathpolynom_callback, this, _1));

  // Create a ROS2 Publisher to send vehicle control information (velocity, steering angle)
  rclcpp::QoS qosParams             = get_qos_profile(QOS_BEST_EFFORT);
  m_simulationVehicleControlPubSPtr = this->create_publisher<SimulationVehicleControlType>("galaxis/simulation/remotecontrol", qosParams);

  log("[OnboardingControllerNode] Waiting for messages...");
}

void ControllerNode::on_pathpolynom_callback(const PathpolynomType::SharedPtr msg) {
  log("[OnboardingControllerNode] #New Pathpolynom received.");

  // Extract Polynom from message (format: a*x^0 + b*x^1 + c*x^2 + d*x^3) (in meter)
  std::vector<float> trajectoryPolynom{ static_cast<float>(msg->c[0]), static_cast<float>(msg->c[1]), static_cast<float>(msg->c[2]), static_cast<float>(msg->c[3]) };
  
  // Current Pose of the vehicle (origin), which looks into the direction of the x-axis
  std::shared_ptr<CarPose> currentPoseSPtr = std::make_shared<CarPose>();
  currentPoseSPtr->position[0] = 0; // x in meter
  currentPoseSPtr->position[1] = 0; // y in meter
  currentPoseSPtr->orientation = 0; // orientation in radians


  // ####################### TODO BEGIN #######################
  // # Your task is to write a short piece of code, which     #
  // # calculates the current turn angle of the vehicle.      #
  // # Feel free to use the given functions in util.hpp. :)   #
  // #                                                        #
  // # HINT: The polynoms are updated very often within a     #
  // #       second. Therefore you only have to respect the   #
  // #       first centimeters of the trajectory polynom.     #
  // ##########################################################

  std::shared_ptr<std::vector<float>> m_turnAnglesSPtr;
  std::shared_ptr<galaxis::control::TwoAndAHalfCarrots> m_controlTwoAndAHalfCarrotsSPtr;
  std::shared_ptr<PolySpline> m_currentReferenceSplineSPtr;

  // Configuration
  m_turnAnglesSPtr                   = galaxis::utils::read_from_file_to_float("settings/control_turnAngles.csv");
  float horizonStepSize              = 0.5f;
  int m_errorAccuracyStepsPerHorizon = 4;
  float carWheelbase                 = galaxis::controllerSettings::carWheelbaseSimulationDrDrift; 
  float errorIncreaseForFirstHorizon = 4;

  // Further Config from file
  std::string configPath = "settings/controller.conf";
  auto configMap = galaxis::utils::read_configuration(configPath);
  horizonStepSize = std::stof(configMap->at("horizonStepTimeSec"));
  errorIncreaseForFirstHorizon = std::stof(configMap->at("errorIncreaseForFirstHorizon"));
  m_errorAccuracyStepsPerHorizon = std::stof(configMap->at("errorAccuracyStepsPerHorizon"));

  m_controlTwoAndAHalfCarrotsSPtr = std::make_shared<TwoAndAHalfCarrots>(TwoAndAHalfCarrots(m_turnAnglesSPtr, horizonStepSize, 2 /* horizon */, m_errorAccuracyStepsPerHorizon, carWheelbase, errorIncreaseForFirstHorizon));

  m_currentReferenceSplineSPtr = std::make_shared<PolySpline>();
  //m_currentReferenceSplineSPtr->add_polynom(-100, 100, galaxis::polynom{ 0, 0, 0, 0 });

  m_currentReferenceSplineSPtr->clear();
  m_currentReferenceSplineSPtr->add_polynom(-100, 100, trajectoryPolynom);

  float currentFrontAxisTurnAngle = m_controlTwoAndAHalfCarrotsSPtr->calculate_best_yaw_rate(m_currentReferenceSplineSPtr, 1.f); // front turn angle
  //std::get<1>(currentFrontAxisTurnAngle) = 0; // rear turn angle

  // ######################## TODO END ########################


  // Send Vehicle Control Message to Simulation Adapter
  auto message = std::make_unique<SimulationVehicleControlType>();
  message->car_speed       = 1.f; // velocity set to 1 m/s
  message->turnangle_front = currentFrontAxisTurnAngle;
  message->reset           = false;
  m_simulationVehicleControlPubSPtr->publish(std::move(message));

  log("[OnboardingControllerNode] New Turnangle send: ", currentFrontAxisTurnAngle);
}
