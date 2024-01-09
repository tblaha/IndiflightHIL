/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_INDIFLIGHT_HH_
#define GZ_SIM_SYSTEMS_INDIFLIGHT_HH_

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <memory>

#include "boost/asio.hpp"
using namespace boost::asio;

using namespace gz;
using namespace sim;
using namespace systems;

typedef struct actuator_s {
    int fdm_id;
    int model_id;
    transport::Node::Publisher commanded_rpm_pub;
    std::string state_topic;
    double kappa;
    double multiplier;
    double input;
    double commanded_rpm;
    double sensor_rpm;
} actuator_t;

typedef struct hilInput_s {
    float gyro[3]; // deg/s
    float acc[3]; // g
    float rpm[4]; // rpm
} hilInput_t;

typedef struct pose_s {
    uint64_t timeUs;
    float x;
    float y;
    float z;
    float qx;
    float qy;
    float qz;
    float qw;
} pose_t;

typedef struct pose_der_s {
    uint64_t timeUs;
    float x;
    float y;
    float z;
    float wx;
    float wy;
    float wz;
} pose_der_t;

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class IndiflightPrivate;

  /// \brief This system applies a thrust force to models with spinning
  /// propellers. See examples/worlds/quadcopter.sdf for a demonstration.
  class Indiflight
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate,
        public ISystemPostUpdate
  {
    /// \brief Constructor
    public: Indiflight();

    /// \brief Destructor
    public: ~Indiflight() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    public: void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;

    private:
      void LoadControlChannels(
        sdf::ElementPtr _sdf,
        gz::sim::EntityComponentManager & _ecm);

  /// \brief Load IMU sensors
    private:
      void LoadImuSensors(
        sdf::ElementPtr _sdf,
        gz::sim::EntityComponentManager & _ecm);

    private:
      bool InitSockets(sdf::ElementPtr _sdf);

      void serialWriter(uint8_t byte);

  /// \brief Update PID Joint controllers.
  /// \param[in] _dt time step size since last update.
    private:
      void ApplyMotorForces(
        const double _dt,
        gz::sim::EntityComponentManager & _ecm);

  /// \brief Send state to ArduPilot
    //private:
    //  void SendState(double _simTime, const gz::sim::EntityComponentManager & _ecm) const;

    /// \brief Calculates odometry and publishes an odometry message.
    /// \param[in] _info System update information.
    /// \param[in] _ecm The EntityComponentManager of the given simulation
    /// instance.
    public: void UpdateOdometry(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm);

    /// \brief Private data pointer
    private: std::unique_ptr<IndiflightPrivate> dataPtr;

  };
  }
}
}
}

#endif
