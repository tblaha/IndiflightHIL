/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 * Copyright (C) 2019 Open Source Robotics Foundation
 * Copyright (C) 2022 Benjamin Perseghetti, Rudis Laboratories
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "MulticopterMotorModelTB.hh"

#include <mutex>
#include <string>
#include <chrono>

#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/mc_actuator_state.pb.h>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/JointAxis.hh"
#include "gz/sim/components/JointVelocity.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/components/JointEffortLimitsCmd.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/Events.hh"


// from rotors_gazebo_plugins/include/rotors_gazebo_plugins/common.h
/// \brief    This class can be used to apply a first order filter on a signal.
///           It allows different acceleration and deceleration time constants.
/// \details
///           Short reveiw of discrete time implementation of first order system
///           Laplace:
///             X(s)/U(s) = 1/(tau*s + 1)
///           continous time system:
///             dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
///           discretized system (ZoH):
///             x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k) // NOLINT
template <typename T>
class FirstOrderFilter {
  public:
  FirstOrderFilter(double _timeConstantUp, double _timeConstantDown, T _initialState): // NOLINT
      timeConstantUp(_timeConstantUp),
      timeConstantDown(_timeConstantDown),
      previousState(_initialState) {}

  /// \brief    This method will apply a first order filter on the _inputState.
  T UpdateFilter(T _inputState, double _samplingTime)
  {
    T outputState;
    if (_inputState > previousState) {
      // Calcuate the outputState if accelerating.
      double alphaUp = exp(-_samplingTime / timeConstantUp);
      // x(k+1) = Ad*x(k) + Bd*u(k)
      outputState = alphaUp * previousState + (1 - alphaUp) * _inputState;
    }
    else
    {
      // Calculate the outputState if decelerating.
      double alphaDown = exp(-_samplingTime / timeConstantDown);
      outputState = alphaDown * previousState + (1 - alphaDown) * _inputState;
    }
    previousState = outputState;
    return outputState;
  }

  ~FirstOrderFilter() = default;

  protected:
  double timeConstantUp;
  double timeConstantDown;
  T previousState;
};

// adapted from rotors_gazebo_plugins/include/rotors_gazebo_plugins/common.h
/// \brief    This class can be used to apply a first order filter on a signal.
///           It allows different acceleration and deceleration time constants
///           and limiting the rate of change
/// \details
///           Short reveiw of discrete time implementation of first order system
///           Laplace:
///             X(s)/U(s) = 1/(tau*s + 1)
///           continous time system:
///             dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
///             u(t) = x(t) + tau * dx(t)
///             u_max = x(t) + tau * rateLimitUp
///             u_min = x(t) - tau * rateLimitDown
///           discretized system (ZoH):
///             x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k) // NOLINT
template <typename T>
class FirstOrderRateLimitedFilter {
  public:
  FirstOrderRateLimitedFilter(double _timeConstantUp, double _timeConstantDown, double _rateLimitUp, double _rateLimitDown, T _initialState): // NOLINT
      timeConstantUp(_timeConstantUp),
      timeConstantDown(_timeConstantDown),
      previousState(_initialState),
      rate(0)
      {
        duMax = _timeConstantUp * _rateLimitUp;
        duMin = -_timeConstantDown * _rateLimitDown;
      }

  /// \brief    This method will apply a first order filter on the _inputState.
  T UpdateFilter(T _inputState, double _samplingTime)
  {
    T outputState;
    _inputState = rateLimitInput(_inputState);
    if (_inputState > previousState) {
      // Calcuate the outputState if accelerating.
      double alphaUp = exp(-_samplingTime / timeConstantUp);
      outputState = alphaUp * previousState + (1 - alphaUp) * _inputState;
    }
    else
    {
      // Calculate the outputState if decelerating.
      double alphaDown = exp(-_samplingTime / timeConstantDown);
      outputState = alphaDown * previousState + (1 - alphaDown) * _inputState;
    }
    previousState = outputState;
    return outputState;
  }

  T getInitialRate(T _inputState) {
    _inputState = rateLimitInput(_inputState);
    T rate;
    if (_inputState > previousState)
        rate = -1/timeConstantUp * previousState + 1/timeConstantUp * _inputState;
    else
        rate = -1/timeConstantDown * previousState + 1/timeConstantDown * _inputState;

    return rate;
  }

  T getCurrent() {return previousState;}

  ~FirstOrderRateLimitedFilter() = default;

  private:
  inline T rateLimitInput(T _inputState) {
    return (_inputState > previousState + duMax) ? previousState + duMax : 
           (_inputState < previousState + duMin) ? previousState + duMin :
           _inputState;
  }

  protected:
  double timeConstantUp;
  double timeConstantDown;
  double duMax;
  double duMin;
  T previousState;
  T rate;
};

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Constants for specifying clockwise (kCw) and counter-clockwise (kCcw)
/// directions of rotation.
namespace turning_direction {
static const int kCcw = 1;
static const int kCw = -1;
}  // namespace turning_direction

/// \brief Type of input command to motor.
enum class MotorType {
  kVelocity,
  kPosition,
  kForce
};

class gz::sim::systems::MulticopterMotorModelPrivateTB
{
  /// \brief Callback for actuator commands.
  public: void OnActuatorMsg(const msgs::Double &_msg);

  /// \brief Apply link forces and moments based on propeller state.
  public: void UpdateForcesAndMoments(const UpdateInfo &_info, EntityComponentManager &_ecm);

  /// \brief Joint Entity
  public: Entity jointEntity;

  /// \brief Joint name
  public: std::string jointName;

  /// \brief Link Entity
  public: Entity linkEntity;

  /// \brief Link name
  public: std::string linkName;

  /// \brief Parent link Entity
  public: Entity parentLinkEntity;

  /// \brief Parent link name
  public: std::string parentLinkName;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Topic namespace.
  public: std::string robotNamespace;

  /// \brief Topic for actuator commands.
  public: std::string statePubTopic;

  /// \brief bool indicating whether to publish state
  public: bool publishState{false};

  /// \brief Sampling time (from motor_model.hpp).
  public: double samplingTime = 0.01;

  /// \brief Index of motor on multirotor_base.
  public: int motorNumber = 0;

  /// \brief Turning direction of the motor.
  public: int turningDirection = turning_direction::kCw;

  /// \brief Type of input command to motor.
  public: MotorType motorType = MotorType::kVelocity;

  /// \brief Maximum rotational velocity command with units of rad/s.
  /// The default value is taken from gazebo_motor_model.h
  /// and is approximately 8000 revolutions / minute (rpm).
  public: double maxRotVelocity = 838.0;

  /// \brief Moment constant for computing drag torque based on thrust
  /// with units of length (m).
  /// The default value is taken from gazebo_motor_model.h
  public: double momentConstant = 0.016;

  /// \brief Thrust coefficient for propeller with units of N / (rad/s)^2.
  /// The default value is taken from gazebo_motor_model.h
  public: double motorConstant = 8.54858e-06;

  /// \brief Reference input to motor. For MotorType kVelocity, this
  /// is the reference angular velocity in rad/s.
  public: double refMotorInput = 0.0;

  /// \brief Rolling moment coefficient with units of N*m / (m/s^2).
  /// The default value is taken from gazebo_motor_model.h
  public: double rollingMomentCoefficient = 1.0e-6;

  /// \brief Rotor drag coefficient for propeller with units of N / (m/s^2).
  /// The default value is taken from gazebo_motor_model.h
  public: double rotorDragCoefficient = 1.0e-4;

  /// \brief Large joint velocities can cause problems with aliasing,
  /// so the joint velocity used by the physics engine is reduced
  /// this factor, while the larger value is used for computing
  /// propeller thrust.
  /// The default value is taken from gazebo_motor_model.h
  public: double rotorVelocitySlowdownSim = 10.0;

  /// \brief Time constant for rotor deceleration.
  /// The default value is taken from gazebo_motor_model.h
  public: double timeConstantDown = 1.0 / 40.0;

  /// \brief Time constant for rotor acceleration.
  /// The default value is taken from gazebo_motor_model.h
  public: double timeConstantUp = 1.0 / 80.0;

  /// \brief Maximum torque the motor can deliver (independent of speed)
  /// default: could spin prop with inertia 2.65e-5 kgm^2 to 1000rad/s in .1sec
  public: double maxTorqueUp = 0.25;

  /// \brief Maximum torque the motor can deliver (independent of speed)
  /// default: half of maxTorqueUp
  public: double maxTorqueDown = 0.125;

  /// \brief propeller inertia around spin axis
  public: double rotorInertia = 2.65e-5;
  public: double maxRateUp = 0.25/2.65e-5;
  public: double maxRateDown = 0.25/2.65e-5;

  /// \brief bool indicating whether to simulate spin up effects internally
  /// implies slowdownSim of 50 to make sure gz doesnt try to simulate these effects
  public: bool inertiaEffects{true};

  /// \brief Filter on rotor velocity that has different time constants
  /// for increasing and decreasing values.
  public: std::unique_ptr<FirstOrderRateLimitedFilter<double>> rotorVelocityFilter;

  /// \brief Received Actuators message. This is nullopt if no message has been
  /// received.
  public: std::optional<msgs::Double> recvdActuatorsMsg;

  /// \brief Mutex to protect recvdActuatorsMsg.
  public: std::mutex recvdActuatorsMsgMutex;

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Gazebo publisher for states.
  public: transport::Node::Publisher statePub;

  /// \brief time of last state pub
  public: std::chrono::steady_clock::duration statePubPeriod{0};

  /// \brief time of last state pub
  public: std::chrono::steady_clock::duration lastStatePubTime{0};

  public: EventManager *eventMgr{nullptr};
  public: double rt_strictness{0};
  public: int maxDrops{1};
  public: int drops{0};
};

//////////////////////////////////////////////////
MulticopterMotorModelTB::MulticopterMotorModelTB()
  : dataPtr(std::make_unique<MulticopterMotorModelPrivateTB>())
{
}

//////////////////////////////////////////////////
void MulticopterMotorModelTB::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "MulticopterMotorModel plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }

  auto sdfClone = _sdf->Clone();

  this->dataPtr->robotNamespace.clear();

  if (sdfClone->HasElement("robotNamespace"))
  {
    this->dataPtr->robotNamespace =
        sdfClone->Get<std::string>("robotNamespace");
  }
  else
  {
    gzwarn << "No robotNamespace set using entity name.\n";
    this->dataPtr->robotNamespace = this->dataPtr->model.Name(_ecm);
  }

  // Get params from SDF
  if (sdfClone->HasElement("jointName"))
  {
    this->dataPtr->jointName = sdfClone->Get<std::string>("jointName");
  }

  if (this->dataPtr->jointName.empty())
  {
    gzerr << "MulticopterMotorModel found an empty jointName parameter. "
           << "Failed to initialize.";
    return;
  }

  if (sdfClone->HasElement("linkName"))
  {
    this->dataPtr->linkName = sdfClone->Get<std::string>("linkName");
  }

  if (this->dataPtr->linkName.empty())
  {
    gzerr << "MulticopterMotorModel found an empty linkName parameter. "
           << "Failed to initialize.";
    return;
  }

  if (sdfClone->HasElement("motorNumber"))
    this->dataPtr->motorNumber =
      sdfClone->GetElement("motorNumber")->Get<int>();
  else
    gzerr << "Please specify a motorNumber.\n";

  if (sdfClone->HasElement("turningDirection"))
  {
    auto turningDirection =
        sdfClone->GetElement("turningDirection")->Get<std::string>();
    if (turningDirection == "cw")
      this->dataPtr->turningDirection = turning_direction::kCw;
    else if (turningDirection == "ccw")
      this->dataPtr->turningDirection = turning_direction::kCcw;
    else
      gzerr << "Please only use 'cw' or 'ccw' as turningDirection.\n";
  }
  else
  {
    gzerr << "Please specify a turning direction ('cw' or 'ccw').\n";
  }

  if (sdfClone->HasElement("motorType"))
  {
    auto motorType = sdfClone->GetElement("motorType")->Get<std::string>();
    if (motorType == "velocity")
      this->dataPtr->motorType = MotorType::kVelocity;
    else if (motorType == "position")
    {
      this->dataPtr->motorType = MotorType::kPosition;
      gzerr << "motorType 'position' not supported" << std::endl;
    }
    else if (motorType == "force")
    {
      this->dataPtr->motorType = MotorType::kForce;
      gzerr << "motorType 'force' not supported" << std::endl;
    }
    else
    {
      gzerr << "Please only use 'velocity', 'position' or "
               "'force' as motorType.\n";
    }
  }
  else
  {
    gzwarn << "motorType not specified, using velocity.\n";
    this->dataPtr->motorType = MotorType::kVelocity;
  }

  sdfClone->Get<double>("realTimeStrictness",
      this->dataPtr->rt_strictness, this->dataPtr->rt_strictness);
  sdfClone->Get<int>("maxDrops",
      this->dataPtr->maxDrops, this->dataPtr->maxDrops);
  sdfClone->Get<bool>("publishState",
      this->dataPtr->publishState, this->dataPtr->publishState);

  sdfClone->Get<double>("rotorDragCoefficient",
      this->dataPtr->rotorDragCoefficient,
      this->dataPtr->rotorDragCoefficient);
  sdfClone->Get<double>("rollingMomentCoefficient",
      this->dataPtr->rollingMomentCoefficient,
      this->dataPtr->rollingMomentCoefficient);
  sdfClone->Get<double>("maxRotVelocity",
      this->dataPtr->maxRotVelocity, this->dataPtr->maxRotVelocity);
  sdfClone->Get<double>("motorConstant",
      this->dataPtr->motorConstant, this->dataPtr->motorConstant);
  sdfClone->Get<double>("momentConstant",
      this->dataPtr->momentConstant, this->dataPtr->momentConstant);

  sdfClone->Get<double>("timeConstantUp",
      this->dataPtr->timeConstantUp, this->dataPtr->timeConstantUp);
  sdfClone->Get<double>("timeConstantDown",
      this->dataPtr->timeConstantDown, this->dataPtr->timeConstantDown);
  sdfClone->Get<double>("maxTorqueUp",
      this->dataPtr->maxTorqueUp, this->dataPtr->maxTorqueUp);
  sdfClone->Get<double>("maxTorqueDown",
      this->dataPtr->maxTorqueDown, this->dataPtr->maxTorqueDown);
  sdfClone->Get<double>("rotorInertia",
      this->dataPtr->rotorInertia, this->dataPtr->rotorInertia);
  sdfClone->Get<double>("rotorVelocitySlowdownSim",
      this->dataPtr->rotorVelocitySlowdownSim, this->dataPtr->rotorVelocitySlowdownSim);
  sdfClone->Get<bool>("inertiaEffects",
      this->dataPtr->inertiaEffects, this->dataPtr->inertiaEffects);

  if ((this->dataPtr->inertiaEffects) 
        && (this->dataPtr->rotorVelocitySlowdownSim < 50.)) {
    gzwarn << "Internal spin up torque handling requested, increasing"
           << " rotorVelocitySlowdownSim to 50" << std::endl;
    this->dataPtr->rotorVelocitySlowdownSim = 50.;
  }

  if (this->dataPtr->rotorInertia > 0)
  {
    this->dataPtr->maxRateUp = this->dataPtr->maxTorqueUp/this->dataPtr->rotorInertia;
    this->dataPtr->maxRateDown = this->dataPtr->maxTorqueDown/this->dataPtr->rotorInertia;
  } 
  else
  {
    gzerr << "Rotor inertia must be positive!" << std::endl;
  }

  // Create the first order filter.
  this->dataPtr->rotorVelocityFilter =
      std::make_unique<FirstOrderRateLimitedFilter<double>>(
          this->dataPtr->timeConstantUp, this->dataPtr->timeConstantDown,
          this->dataPtr->maxRateUp, this->dataPtr->maxRateDown,
          this->dataPtr->refMotorInput);

  // Subscribe to actuator command messages
  std::string topic = transport::TopicUtils::AsValidTopic(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/actuator_command/" + std::to_string(this->dataPtr->motorNumber));
  if (topic.empty())
  {
    gzerr << "Failed to create topic for [" << this->dataPtr->robotNamespace
           << "]" << std::endl;
    return;
  }
  else
  {
    gzdbg << "Listening to topic: " << topic << std::endl;
  }
  this->dataPtr->node.Subscribe(topic,
      &MulticopterMotorModelPrivateTB::OnActuatorMsg, this->dataPtr.get());

  // Advertise pub topic for actuator states, if requeseted
  if (this->dataPtr->publishState)
  {
    double stateFreq = sdfClone->Get<double>("statePubFrequency", 250).first;
    if (stateFreq > 0)
    {
      std::chrono::duration<double> period{1 / stateFreq};
      this->dataPtr->statePubPeriod = 
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    }

    //sdfClone->Get<std::string>("statePubTopic",
    //  this->dataPtr->statePubTopic, this->dataPtr->statePubTopic);
    this->dataPtr->statePubTopic = "/model/" + this->dataPtr->model.Name(_ecm) + "/actuator_state/" + std::to_string(this->dataPtr->motorNumber);
    std::string statePubTopicValid {transport::TopicUtils::AsValidTopic(
      this->dataPtr->statePubTopic)};

    if (statePubTopicValid.empty())
    {
      gzerr << "Failed to generate statePub topic ["
            << this->dataPtr->statePubTopic << "]" << std::endl;
    }
    else 
    {
      this->dataPtr->statePub = this->dataPtr->node.Advertise<msgs::Double>(
        statePubTopicValid);
      gzmsg << "MulticopterMotorModelTB publishing motor states on ["
            << statePubTopicValid << "]" << std::endl;
    }

    // save the eventMgr pointer in the private object
    this->dataPtr->eventMgr = &_eventMgr;
  }
}

//////////////////////////////////////////////////
void MulticopterMotorModelTB::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("MulticopterMotorModel::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // If the joint or links haven't been identified yet, look for them
  if (this->dataPtr->jointEntity == kNullEntity)
  {
    this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);

    const auto parentLinkName = _ecm.Component<components::ParentLinkName>(
        this->dataPtr->jointEntity);
    this->dataPtr->parentLinkName = parentLinkName->Data();
  }

  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  }

  if (this->dataPtr->parentLinkEntity == kNullEntity)
  {
    this->dataPtr->parentLinkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->parentLinkName);
  }

  if (this->dataPtr->jointEntity == kNullEntity ||
      this->dataPtr->linkEntity == kNullEntity ||
      this->dataPtr->parentLinkEntity == kNullEntity)
    return;

  gz::math::Vector2d limits{};
  if (this->dataPtr->turningDirection == turning_direction::kCcw)
    limits.Set(-(this->dataPtr->maxTorqueDown), this->dataPtr->maxTorqueUp);
  else if (this->dataPtr->turningDirection == turning_direction::kCw)
    limits.Set(-(this->dataPtr->maxTorqueUp), this->dataPtr->maxTorqueDown);

  const auto jointEffortCmd = _ecm.Component<components::JointEffortLimitsCmd>(
      this->dataPtr->jointEntity);
  if (jointEffortCmd)
  {
    *jointEffortCmd = components::JointEffortLimitsCmd({limits});
  }
  else
  {
    _ecm.CreateComponent(this->dataPtr->jointEntity,
        components::JointEffortLimitsCmd({limits}));
  }


  // skip UpdateForcesAndMoments if needed components are missing
  bool doUpdateForcesAndMoments = true;

  const auto jointVelocity = _ecm.Component<components::JointVelocity>(
      this->dataPtr->jointEntity);
  if (!jointVelocity)
  {
    _ecm.CreateComponent(this->dataPtr->jointEntity,
        components::JointVelocity());
    doUpdateForcesAndMoments = false;
  }
  else if (jointVelocity->Data().empty())
  {
    doUpdateForcesAndMoments = false;
  }

  if (!_ecm.Component<components::JointVelocityCmd>(
      this->dataPtr->jointEntity))
  {
    _ecm.CreateComponent(this->dataPtr->jointEntity,
        components::JointVelocityCmd({0}));
    doUpdateForcesAndMoments = false;
  }

  if (!_ecm.Component<components::WorldPose>(this->dataPtr->linkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity, components::WorldPose());
    doUpdateForcesAndMoments = false;
  }
  if (!_ecm.Component<components::WorldLinearVelocity>(
      this->dataPtr->linkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity,
        components::WorldLinearVelocity());
    doUpdateForcesAndMoments = false;
  }

  if (!_ecm.Component<components::WorldPose>(this->dataPtr->parentLinkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->parentLinkEntity,
        components::WorldPose());
    doUpdateForcesAndMoments = false;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->samplingTime =
    std::chrono::duration<double>(_info.dt).count();
  if (doUpdateForcesAndMoments)
  {
    this->dataPtr->UpdateForcesAndMoments(_info, _ecm);
  }
}

//////////////////////////////////////////////////
void MulticopterMotorModelPrivateTB::OnActuatorMsg(
    const msgs::Double &_msg)
{
  std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);
  this->recvdActuatorsMsg = _msg;
}

//////////////////////////////////////////////////
void MulticopterMotorModelPrivateTB::UpdateForcesAndMoments(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("MulticopterMotorModelPrivate::UpdateForcesAndMoments");

  //std::optional<msgs::Actuators> msg;
  //auto actuatorMsgComp =
  //    _ecm.Component<components::Actuators>(this->model.Entity());
  std::optional<msgs::Double> msg;

  // Actuators messages can come in from transport or via a component. If a
  // component is available, it takes precedence.
  //if (actuatorMsgComp)
  //{
  //  msg = actuatorMsgComp->Data();
  //}
  //else
  {
    std::lock_guard<std::mutex> lock(this->recvdActuatorsMsgMutex);
    if (this->recvdActuatorsMsg.has_value())
    {
      msg = *this->recvdActuatorsMsg;
      this->recvdActuatorsMsg.reset();
    }
  }

  if (msg.has_value())
  {
    // verify that external sim is still running real-time
    auto stamp = msg->header().stamp();
    if (this->rt_strictness > 0 && 
          (_info.simTime > (this->rt_strictness * _info.dt
                         + std::chrono::seconds(stamp.sec())
                         + std::chrono::nanoseconds(stamp.nsec()))
          )
       )
    {
      gzerr << "Motor " << this->motorNumber << ": Out of date actuator message ("
                << stamp.sec()+1e-9*stamp.nsec() << ") received at sim_time " 
                << _info.simTime.count()*1e-9 << ". Occurence " << ++this->drops
                << " of " << this->maxDrops+1 << std::endl;

      if (this->drops > this->maxDrops) {
        this->eventMgr->Emit<events::Pause>(true);
        gzerr << "Paused sim" << std::endl;
      }
    }

    //if (this->motorNumber > msg->velocity_size() - 1)
    //{
    //  gzerr << "You tried to access index " << this->motorNumber
    //    << " of the Actuator velocity array which is of size "
    //    << msg->velocity_size() << std::endl;
    //  return;
    //}

    if (this->motorType == MotorType::kVelocity)
    {
      this->refMotorInput = std::min(
          static_cast<double>(msg->data()),
          static_cast<double>(this->maxRotVelocity));
    }
    //  else if (this->motorType == MotorType::kPosition)
    else  // if (this->motorType == MotorType::kForce) {
    {
      this->refMotorInput = 0;
    }
  }

  switch (this->motorType)
  {
    case (MotorType::kPosition):
    {
      // double err = joint_->GetAngle(0).Radian() - this->refMotorInput;
      // double force = pids_.Update(err, this->samplingTime);
      // joint_->SetForce(0, force);
      break;
    }
    case (MotorType::kForce):
    {
      // joint_->SetForce(0, this->refMotorInput);
      break;
    }
    default:  // MotorType::kVelocity
    {
      const auto jointVelocity = _ecm.Component<components::JointVelocity>(
          this->jointEntity);
      double motorRotVel = jointVelocity->Data()[0];
      if (motorRotVel / (2 * GZ_PI) > 1 / (2 * this->samplingTime))
      {
        gzerr << "Aliasing on motor [" << this->motorNumber
              << "] might occur. Consider making smaller simulation time "
                 "steps or raising the rotorVelocitySlowdownSim param.\n";
      }
      double realMotorVelocity = this->rotorVelocityFilter->getCurrent();
          // motorRotVel * this->rotorVelocitySlowdownSim;
      // Get the direction of the rotor rotation.
      //int realMotorVelocitySign =
      //    (realMotorVelocity > 0) - (realMotorVelocity < 0);
      // Assuming symmetric propellers (or rotors) for the thrust calculation.
      double thrust = realMotorVelocity * realMotorVelocity
                      * this->motorConstant;

      using Pose = math::Pose3d;
      using Vector3 = math::Vector3d;

      Link link(this->linkEntity);
      const auto worldPose = link.WorldPose(_ecm);

      // Apply a force to the link.
      link.AddWorldForce(_ecm,
                         worldPose->Rot().RotateVector(Vector3(0, 0, thrust)));

      const auto jointPose = _ecm.Component<components::Pose>(
          this->jointEntity);
      if (!jointPose)
      {
        gzerr << "joint " << this->jointName << " has no Pose"
               << "component" << std::endl;
        return;
      }
      // computer joint world pose by multiplying child link WorldPose
      // with joint Pose
      Pose jointWorldPose = *worldPose * jointPose->Data();

      const auto jointAxisComp = _ecm.Component<components::JointAxis>(
          this->jointEntity);
      if (!jointAxisComp)
      {
        gzerr << "joint " << this->jointName << " has no JointAxis"
               << "component" << std::endl;
        return;
      }

      const auto worldLinearVel = link.WorldLinearVelocity(_ecm);

      Entity windEntity = _ecm.EntityByComponents(components::Wind());
      auto windLinearVel =
          _ecm.Component<components::WorldLinearVelocity>(windEntity);
      Vector3 windSpeedWorld = windLinearVel->Data();

      // Forces from Philppe Martin's and Erwan Salaun's
      // 2010 IEEE Conference on Robotics and Automation paper
      // The True Role of Accelerometer Feedback in Quadrotor Control
      // - \omega * \lambda_1 * V_A^{\perp}
      Vector3 jointAxis =
          jointWorldPose.Rot().RotateVector(jointAxisComp->Data().Xyz());
      Vector3 bodyVelocityWorld = *worldLinearVel;
      Vector3 relativeWindVelocityWorld = bodyVelocityWorld - windSpeedWorld;
      Vector3 bodyVelocityPerpendicular =
          relativeWindVelocityWorld -
          (relativeWindVelocityWorld.Dot(jointAxis) * jointAxis);
      Vector3 airDrag = -std::abs(realMotorVelocity) *
                               this->rotorDragCoefficient *
                               bodyVelocityPerpendicular;

      // Apply air drag to link.
      link.AddWorldForce(_ecm, airDrag);

      // Moments get the parent link, such that the resulting torques can be
      // applied.
      Vector3 parentWorldTorque;
      auto parentWrenchComp =
        _ecm.Component<components::ExternalWorldWrenchCmd>(
          this->parentLinkEntity);
      // gazebo_motor_model.cpp subtracts the GetWorldCoGPose() of the
      // child link from the parent but only uses the rotation component.
      // Since GetWorldCoGPose() uses the link frame orientation, it
      // is equivalent to use WorldPose().Rot().
      Link parentLink(this->parentLinkEntity);
      const auto parentWorldPose = parentLink.WorldPose(_ecm);
      // The transformation from the parent_link to the link_.
      // Pose poseDifference =
      //  parent_links.at(0)->GetWorldCoGPose().Inverse()
      //  * link_->GetWorldCoGPose()
      Pose poseDifference = (*parentWorldPose).Inverse() * (*worldPose);
      Vector3 rotorTorque(
          0, 0, -this->turningDirection * thrust * this->momentConstant);
      double rotorRate = this->rotorVelocityFilter->getInitialRate(
          this->refMotorInput);
      if (this->inertiaEffects) {
        // spinup torque effects (since hard to model with prop inertia)
        rotorTorque.Z() += 
            -this->turningDirection * rotorRate * this->rotorInertia;
      }
      // Transforming the rotor torque into the parent frame to handle
      // arbitrary rotor orientations.
      // NOTE: why not just worldPose->Rot().RotateVector(rotorTorque)?
      Vector3 rotorTorqueParentFrame =
          poseDifference.Rot().RotateVector(rotorTorque);
      parentWorldTorque =
          parentWorldPose->Rot().RotateVector(rotorTorqueParentFrame);

      Vector3 rollingMoment;
      // - \omega * \mu_1 * V_A^{\perp}
      rollingMoment = -std::abs(realMotorVelocity) *
                       this->rollingMomentCoefficient *
                       bodyVelocityPerpendicular;
      parentWorldTorque += rollingMoment;
      if (!parentWrenchComp)
      {
        components::ExternalWorldWrenchCmd wrench;
        msgs::Set(wrench.Data().mutable_torque(), parentWorldTorque);
        _ecm.CreateComponent(this->parentLinkEntity, wrench);
      }
      else
      {
        msgs::Set(parentWrenchComp->Data().mutable_torque(),
          msgs::Convert(parentWrenchComp->Data().torque()) + parentWorldTorque);
      }
      // Apply the filter on the motor's velocity.
      double refMotorRotVel;
      refMotorRotVel = this->rotorVelocityFilter->UpdateFilter(
          this->refMotorInput, this->samplingTime);

      const auto jointVelCmd = _ecm.Component<components::JointVelocityCmd>(
          this->jointEntity);
      *jointVelCmd = components::JointVelocityCmd(
          {this->turningDirection * refMotorRotVel
                              / this->rotorVelocitySlowdownSim});

      //---- publish current filtered velocities
      auto diff = _info.simTime - this->lastStatePubTime;
      if ((!this->publishState) 
          || (this->statePubPeriod > std::chrono::steady_clock::duration::zero() 
              && diff > std::chrono::steady_clock::duration::zero()
              && diff < this->statePubPeriod)
              )
      {
        // no publishing requested, or throttling required to stay above statePubPeriod
        return;
      }
      // build message
      msgs::Header header;
      header.mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));
      auto id = header.add_data();
      id->set_key("motor_id");
      id->add_value(std::to_string(this->motorNumber));

      //msgs::McActuatorState msg;
      //msg.mutable_header()->CopyFrom(header);
      //msg.set_speed(realMotorVelocity);
      //msg.set_rate(rotorRate);
      msgs::Double msg;
      msg.mutable_header()->CopyFrom(header);
      msg.set_data(realMotorVelocity);

      this->lastStatePubTime = _info.simTime;
      if (this->statePub.Valid())
      {
        this->statePub.Publish(msg);
      }
    }
  }
}

GZ_ADD_PLUGIN(MulticopterMotorModelTB,
                    System,
                    MulticopterMotorModelTB::ISystemConfigure,
                    MulticopterMotorModelTB::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(MulticopterMotorModelTB,
                          "gz::sim::systems::MulticopterMotorModelTB")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(MulticopterMotorModelTB,
                          "ignition::gazebo::systems::MulticopterMotorModelTB")
