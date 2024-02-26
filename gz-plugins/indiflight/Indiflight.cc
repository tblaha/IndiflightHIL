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

#include "Indiflight.hh"

#include <mutex>
#include <string>
#include <chrono>
#include <functional>
#include <iostream>
#include <termios.h>
#include <linux/serial.h>

#include <gz/msgs/header.pb.h>
#include <gz/msgs/world_odom.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/double.pb.h>

#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/LinearAcceleration.hh"
#include "gz/sim/components/Pose.hh"
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/Link.hh>
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "gz/sim/Conversions.hh"

#include "pi-protocol.h"
#include "pi-messages.h"

#include "boost/asio.hpp"
using namespace boost::asio;

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::IndiflightPrivate
{
  public:
    IndiflightPrivate() : _io_service(), _socket(_io_service), _io_service2(), _serialPort{_io_service2}
    {
    }
    ~IndiflightPrivate()
    {
        _socket.close();
        _serialPort.close();
    }
  public:
  gz::msgs::IMU imuMsg;
  
  /// \brief Have we received at least one IMU data message?
  
  public:
  bool imuMsgValid{false};
  bool imuInitialized{false};
  
  /// \brief This mutex should be used when accessing imuMsg or imuMsgValid
  
  public:
  std::mutex imuMsgMutex;
  
  /// \brief This subscriber callback latches the most recently received
  ///        IMU data message for later use.

  public: void ImuCb(const gz::msgs::IMU & _msg)
  {
    std::lock_guard<std::mutex> lock(this->imuMsgMutex);
    imuMsg = _msg;
    imuMsgValid = true;
  }

  public:
  double rads[4];
  double input[4];
  bool radsInitialized{false};
  
  public:

  public: void RpmCb0(const gz::msgs::Double & _msg) { rads[0] = _msg.data(); }
  public: void RpmCb1(const gz::msgs::Double & _msg) { rads[1] = _msg.data(); }
  public: void RpmCb2(const gz::msgs::Double & _msg) { rads[2] = _msg.data(); }
  public: void RpmCb3(const gz::msgs::Double & _msg) { rads[3] = _msg.data(); }
  //public: void RpmCb_ptr[4]
  //public: std::function<void(const gz::msgs::Double &)> RpmCb_ptr[4];

  /// \brief Model name
  public: std::string modelName;

  /// \brief Link name
  public: std::string linkName;

  /// \brief Topic for publishing world odometry
  public: std::string topic{"world_odometry"};

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Link interface
  public: Link link{kNullEntity};

  /// \brief Gazebo publisher for actuators.
  public: transport::Node::Publisher pub;

  /// \brief minimum publishing period
  public: std::chrono::steady_clock::duration statePubPeriod{0};

  /// \brief time of last state pub
  public: std::chrono::steady_clock::duration lastStatePubTime{0};

  /// \brief keep track of controller update sim-time.
  public:
    std::chrono::steady_clock::duration lastControllerUpdateTime{0};

  /// \brief Keep track of the time the last servo packet was received.
  public:
    std::chrono::steady_clock::duration lastServoPacketRecvTime{0};

  public:
    std::vector<actuator_t> actuators;
    hilInput_t hilInput;

  public:
    gz::sim::World world{gz::sim::kNullEntity};

  public:
    std::string worldName;

  /// \brief Set true to enforce lock-step simulation
  public:
    bool isLockStep{false};

  /// \brief The name of the IMU sensor
  public:
    std::string imuName;

  /// \brief The entity representing the link containing the imu sensor.
  public:
    gz::sim::Entity imuLink{gz::sim::kNullEntity};

  public:
    hilInput_t hilState;

  public: io_service _io_service;
  public: io_service _io_service2;
  public: ip::udp::socket _socket;
  public: ip::udp::endpoint _endpoint;
  public: uint32_t udp_ac_id;

  public: serial_port _serialPort;
  public: std::array<char, 10000> _readBuffer;
  public: bool startedAsyncRead{false};

  public: int serialFd{-1};
  public: pi_parse_states_t p_hil;
};

//////////////////////////////////////////////////
Indiflight::Indiflight()
  : dataPtr(std::make_unique<IndiflightPrivate>())
{
  //this->dataPtr->_socket = ip::udp::socket(this->dataPtr->_io_service);
  //this->dataPtr->RpmCb_ptr[0] = IndiflightPrivate::RpmCb0;
  //this->dataPtr->RpmCb_ptr[1] = IndiflightPrivate::RpmCb1;
  //this->dataPtr->RpmCb_ptr[2] = IndiflightPrivate::RpmCb2;
  //this->dataPtr->RpmCb_ptr[3] = IndiflightPrivate::RpmCb3;
}

//void Indiflight::startAsyncRead() {
//  this->dataPtr->_serialPort.async_read_some(boost::asio::buffer(this->dataPtr->_readBuffer),
//    [this](const boost::system::error_code& error, std::size_t bytesRead) {
//      if (!error) {
//          // Process the received data
//          //gzerr << "Received from serial port: " << std::string(this->dataPtr->_readBuffer.begin(), this->dataPtr->_readBuffer.begin() + bytesRead) << std::endl;
//
//          // Continue reading for the next packet
//          startAsyncRead();
//      } else {
//          gzerr << "Error reading data: " << error.message() << std::endl;
//      }
//    }
//  );
//}

//////////////////////////////////////////////////
void Indiflight::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  if ((!this->dataPtr->model.Valid(_ecm)))
  {
    gzerr << "Indiflight plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }
  this->dataPtr->modelName = this->dataPtr->model.Name(_ecm);

  this->dataPtr->world = gz::sim::World(
    _ecm.EntityByComponents(gz::sim::components::World()));
  if (!this->dataPtr->world.Valid(_ecm)) {
    gzerr << "World entity not found" << std::endl;
    return;
  }
  if (this->dataPtr->world.Name(_ecm).has_value()) {
    this->dataPtr->worldName = this->dataPtr->world.Name(_ecm).value();
  }

  //gzerr << this->dataPtr->worldName << " " << this->dataPtr->modelName << std::endl;

  // TODO(ahcorde): fix this topic name
  //this->dataPtr->node.Subscribe(
  //  "/world/empty_betaflight_world/model/iris_with_Betaflight/model/iris_with_standoffs/"
  //  "link/imu_link/sensor/air_pressure_sensor/air_pressure",
  //  &IndiflightPrivate::onAirPressureMessageReceived, this->dataPtr.get());

  auto sdfClone = _sdf->Clone();

  // Load sensor params
  this->LoadImuSensors(sdfClone, _ecm);

  // Load control channels
  this->LoadControlChannels(sdfClone, _ecm);

  if (sdfClone->HasElement("odometry_link"))
  {
    sdfClone->Get<std::string>("odometry_link",
      this->dataPtr->linkName, this->dataPtr->linkName);
    this->dataPtr->link = Link(
      this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName));
  }
  else 
  {
    gzerr << "Indiflight must have the <odometry_link> tag defined. Failed to initialize." << std::endl;
    return;
  }

  std::string devName;
  if (sdfClone->HasElement("hil_device"))
  {
    sdfClone->Get<std::string>("hil_device", devName, devName);
  }
  else 
  {
    gzerr << "Indiflight must have the <hil_device> tag defined. Failed to initialize." << std::endl;
    return;
  }

  unsigned int devBaud = sdfClone->Get("hil_baud", static_cast<uint32_t>(921600)).first;
  if (devBaud != 921600) {
    gzerr << "Havent implemented baudrates other than 921600" << std::endl;
    return;
  }

  // boost:asio, slow
  //this->dataPtr->_serialPort = serial_port(this->dataPtr->_io_service2, devName);
  //this->dataPtr->_serialPort.set_option(serial_port_base::baud_rate(devBaud));

  // termios
  int fd = open(devName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd == -1) {
    gzerr << "openSerialPort: Unable to open port" << std::endl;
    return;
  }

  struct termios options;
  tcgetattr(fd, &options);
  options.c_cflag = B921600 | CS8 | CLOCAL | CREAD;
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &options);

  // linux ioctl command to set device in low-latency mode
  // this brings round-trip latency down to around 6ms, even through a USB hub. Mind is blown.
  // https://github.com/Distrotech/setserial/blob/master/setserial.c#L525
  struct serial_struct old_serinfo, new_serinfo;
  if (ioctl(fd, TIOCGSERIAL, &old_serinfo) < 0) {
    gzerr << "Cannot get serial info" << std::endl;
  } else {
    new_serinfo = old_serinfo;
    new_serinfo.flags &= ~ASYNC_LOW_LATENCY; // is this line even needed?
    new_serinfo.flags |= ASYNC_LOW_LATENCY;

    if (ioctl(fd, TIOCSSERIAL, &new_serinfo) < 0) {
      gzerr << "Cannot set serial info" << std::endl;
    }
  }


  this->dataPtr->serialFd = fd;

  // Initialise sockets
  if (!this->InitSockets(sdfClone)) {
    return;
  }

  // Enforce lock-step simulation (has default: false)
  this->dataPtr->isLockStep =
    sdfClone->Get("lock_step", this->dataPtr->isLockStep).first;

  // Get params from SDF
  sdfClone->Get<std::string>("topic",
      this->dataPtr->topic, this->dataPtr->topic);
}

/////////////////////////////////////////////////
void Indiflight::LoadControlChannels(
  sdf::ElementPtr _sdf,
  gz::sim::EntityComponentManager & _ecm)
{
  // per control channel
  sdf::ElementPtr actuatorSDF;
  if (_sdf->HasElement("actuator")) {
    actuatorSDF = _sdf->GetElement("actuator");
  } else {
    gzerr << "Need to add at least 1 <actuator> block to the plugin" << std::endl;
    return;
  }

  actuator_t actuator;

  while (actuatorSDF) {
    if (actuatorSDF->HasAttribute("channel")) {
      actuator.fdm_id = 
        atoi(actuatorSDF->GetAttribute("channel")->GetAsString().c_str());
    } else if (actuatorSDF->HasAttribute("id")) {
      gzwarn << "[" << this->dataPtr->modelName << "] "
             << "please deprecate attribute id, use channel instead.\n";
      actuator.fdm_id =
        atoi(actuatorSDF->GetAttribute("id")->GetAsString().c_str());
    } else {
      gzerr << "[" << this->dataPtr->modelName << "] "
             << "channel attribute not specified" << std::endl;
      return;
    }
    if (actuatorSDF->HasAttribute("motorNumber")) {
      actuator.model_id = 
        atoi(actuatorSDF->GetAttribute("motorNumber")->GetAsString().c_str());
    } else {
      gzerr << "[" << this->dataPtr->modelName << "] "
             << "motorNumber attribute not specified" << std::endl;
      return;
    }

    if (actuatorSDF->HasElement("commandTopic")) {
      std::string topic = actuatorSDF->Get<std::string>("commandTopic") + "/" + std::to_string(actuator.model_id);
      std::string topicValid {transport::TopicUtils::AsValidTopic(topic)};

      // validate topic valid
      if (topicValid.empty())
      {
        gzerr << "Failed to generate command topic [" << topic << "]" << std::endl;
      }
      else
      {
        actuator.commanded_rpm_pub = this->dataPtr->node.Advertise<msgs::Double>(
          topicValid);
      }
    } else {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "commandTopic not specified" << std::endl;
      return;
    }

    if (actuatorSDF->HasElement("actuatorStateTopic")) {
      std::string topic = actuatorSDF->Get<std::string>("actuatorStateTopic") + "/" + std::to_string(actuator.model_id);
      std::string topicValid {transport::TopicUtils::AsValidTopic(topic)};

      // validate topic valid
      if (topicValid.empty())
      {
        gzerr << "Failed to generate actuator state topic [" << topic << "]" << std::endl;
      }
      else
      {
        actuator.state_topic = topicValid;
      }
    } else {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "actuatorStateTopic not specified" << std::endl;
      return;
    }

    actuator.multiplier = actuatorSDF->Get<double>("multiplier", 1.).first;
    actuator.kappa = actuatorSDF->Get<double>("thrustNonlinearity", 0.).first;

    this->dataPtr->actuators.push_back(actuator);
    actuatorSDF = actuatorSDF->GetNextElement("actuator");
  }
}

double inputToRpmNonDimensional(double kappa, double in) {
    return sqrt(kappa * in*in + (1-kappa) * in);
}

/////////////////////////////////////////////////
void Indiflight::LoadImuSensors(
  sdf::ElementPtr _sdf,
  gz::sim::EntityComponentManager & /*_ecm*/)
{
  this->dataPtr->imuName =
    _sdf->Get("imuName", static_cast<std::string>("imu_sensor")).first;
}


//////////////////////////////////////////////////
void Indiflight::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  GZ_PROFILE("Indiflight::PreUpdate");

// This lookup is done in PreUpdate() because in Configure()
  // it's not possible to get the fully qualified topic name we want
  if (!this->dataPtr->imuInitialized) {
    // Set unconditionally because we're only going to try this once.
    this->dataPtr->imuInitialized = true;
    std::string imuTopicName;

    // The model must contain an imu sensor element:
    //  <sensor name="..." type="imu">
    //
    // Extract the following:
    //  - Sensor topic name: to subscribe to the imu data
    //  - Link containing the sensor: to get the pose to transform to
    //    the correct frame for Betaflight

    // try scoped names first
    auto entities = entitiesFromScopedName(
      this->dataPtr->imuName, _ecm, this->dataPtr->model.Entity());

    // fall-back to unscoped name
    //entities = EntitiesFromUnscopedName(
    //  this->dataPtr->imuName, _ecm, this->dataPtr->model.Entity());
    if (entities.empty()) {
      std::vector<gz::sim::Entity> _entities;
      if (this->dataPtr->model.Entity() == gz::sim::kNullEntity) {
        // search everything
        _entities = _ecm.EntitiesByComponents(gz::sim::components::Name(this->dataPtr->imuName));
      } else {
        // search all descendents
        auto descendents = _ecm.Descendants(this->dataPtr->model.Entity());
        for (const auto & descendent : descendents) {
          if (_ecm.EntityHasComponentType(
              descendent,
              gz::sim::components::Name::typeId))
          {
            auto nameComp = _ecm.Component<gz::sim::components::Name>(descendent);
            if (nameComp->Data() == this->dataPtr->imuName) {
              _entities.push_back(descendent);
            }
          }
        }
      }
      entities = std::unordered_set<gz::sim::Entity>(_entities.begin(), _entities.end());
    }

    if (!entities.empty()) {
      if (entities.size() > 1) {
        gzwarn << "Multiple IMU sensors with name ["
               << this->dataPtr->imuName << "] found. "
               << "Using the first one.\n";
      }

      // select first entity
      gz::sim::Entity imuEntity = *entities.begin();

      // validate
      if (!_ecm.EntityHasComponentType(
          imuEntity,
          gz::sim::components::Imu::typeId))
      {
        gzerr << "Entity with name ["
              << this->dataPtr->imuName
              << "] is not an IMU sensor\n";
      } else {
        gzmsg << "Found IMU sensor with name ["
              << this->dataPtr->imuName
              << "]\n";

        // verify the parent of the imu sensor is a link.
        gz::sim::Entity parent = _ecm.ParentEntity(imuEntity);
        if (_ecm.EntityHasComponentType(
            parent,
            gz::sim::components::Link::typeId))
        {
          this->dataPtr->imuLink = parent;

          imuTopicName = gz::sim::scopedName(
            imuEntity, _ecm) + "/imu";

          gzdbg << "Computed IMU topic to be: "
                << imuTopicName << std::endl;
        } else {
          gzerr << "Parent of IMU sensor ["
                << this->dataPtr->imuName
                << "] is not a link\n";
        }
      }
    } else {
      gzerr << "[" << this->dataPtr->modelName << "] "
            << "imu_sensor [" << this->dataPtr->imuName
            << "] not found, abort Betaflight plugin." << "\n";
      return;
    }

    this->dataPtr->node.Subscribe(
      imuTopicName,
      &IndiflightPrivate::ImuCb,
      this->dataPtr.get());

  } else {
    double t =
      std::chrono::duration_cast<std::chrono::duration<double>>(
      _info.simTime).count();
    // Update the control surfaces.
    if (!_info.paused && _info.simTime >
      this->dataPtr->lastControllerUpdateTime)
    {
      // todo: reimplement with HIL through pi thorugh UART

      //if (this->dataPtr->isLockStep) {
      //  while (!this->ReceiveServoPacket(t, _ecm) &&
      //    this->dataPtr->betaflightOnline)
      //  {
      //    // // SIGNINT should interrupt this loop.
      //    // if (this->dataPtr->signal != 0)
      //    // {
      //    //     break;
      //    // }
      //  }
      //  this->dataPtr->lastServoPacketRecvTime = _info.simTime;
      //} else if (this->ReceiveServoPacket(t, _ecm)) {
      //  this->dataPtr->lastServoPacketRecvTime = _info.simTime;
      //}

      //if (this->dataPtr->betaflightOnline) {
      //  double dt =
      //    std::chrono::duration_cast<std::chrono::duration<double>>(
      //    _info.simTime - this->dataPtr->
      //    lastControllerUpdateTime).count();
      //  this->ApplyMotorForces(dt, _ecm);
      //}
    }
  }

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->radsInitialized) {
    for (auto actuator : this->dataPtr->actuators) {
      switch (actuator.model_id) {
        case 0:
          this->dataPtr->node.Subscribe(actuator.state_topic, &IndiflightPrivate::RpmCb0, this->dataPtr.get());
          break;
        case 1:
          this->dataPtr->node.Subscribe(actuator.state_topic, &IndiflightPrivate::RpmCb1, this->dataPtr.get());
          break;
        case 2:
          this->dataPtr->node.Subscribe(actuator.state_topic, &IndiflightPrivate::RpmCb2, this->dataPtr.get());
          break;
        case 3:
          this->dataPtr->node.Subscribe(actuator.state_topic, &IndiflightPrivate::RpmCb3, this->dataPtr.get());
          break;
      }
    }
    this->dataPtr->radsInitialized = true;
  }

  // Create the pose component if it does not exist.
  auto pos = _ecm.Component<components::Pose>(
      this->dataPtr->model.Entity());
  if (!pos)
  {
    _ecm.CreateComponent(this->dataPtr->model.Entity(),
        components::Pose());
  }

  // enable velocity and acceleration checks
  if (auto vel = this->dataPtr->link.WorldLinearVelocity(_ecm); vel == std::nullopt)
  {
    this->dataPtr->link.EnableVelocityChecks(_ecm);
  }
  //if (auto acc = this->dataPtr->link.WorldLinearAcceleration(_ecm); acc == std::nullopt)
  //{
  //  this->dataPtr->link.EnableAccelerationChecks(_ecm);
  //}

  // todo: receive actautor from tty and replace below
  if (piMsgHilOutRxState == PI_MSG_RX_STATE_NONE) {
    //gzerr << "hil is none" << std::endl;
    return;
  }

#define MOTOR_TO_HIL 32767
  this->dataPtr->input[0] = ( (double) piMsgHilOutRx->set_1 ) / MOTOR_TO_HIL;
  this->dataPtr->input[1] = ( (double) piMsgHilOutRx->set_2 ) / MOTOR_TO_HIL;
  this->dataPtr->input[2] = ( (double) piMsgHilOutRx->set_3 ) / MOTOR_TO_HIL;
  this->dataPtr->input[3] = ( (double) piMsgHilOutRx->set_4 ) / MOTOR_TO_HIL;

  for (auto actuator : this->dataPtr->actuators) {
    actuator.input = this->dataPtr->input[actuator.fdm_id];
    //actuator.input = 1.;

    actuator.commanded_rpm = actuator.multiplier * inputToRpmNonDimensional(actuator.kappa, actuator.input);

    msgs::Header header;
    header.mutable_stamp()->CopyFrom(convert<msgs::Time>(_info.simTime));

    msgs::Double msg;
    msg.mutable_header()->CopyFrom(header);
    msg.set_data(actuator.commanded_rpm);

    actuator.commanded_rpm_pub.Publish(msg);
  }
}

void Indiflight::serialWriter(uint8_t byte) {
    write(this->dataPtr->serialFd, &byte, 1);
}

//////////////////////////////////////////////////
void Indiflight::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Indiflight::PostUpdate");
  if (!_info.paused && _info.simTime > this->dataPtr->lastControllerUpdateTime)
    //&& this->dataPtr->betaflightOnline)
  {
    double t =
      std::chrono::duration_cast<std::chrono::duration<double>>(
      _info.simTime).count();
    //this->SendState(t, _ecm);

    // Make a local copy of the latest IMU data (it's filled in
    // on receipt by ImuCb()).
    gz::msgs::IMU imuMsg;
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->imuMsgMutex);
      // Wait until we've received a valid message.
      if (!this->dataPtr->imuMsgValid) {
        return;
      }
      imuMsg = this->dataPtr->imuMsg;
      //this->dataPtr->imuMsg.reset();
    }

    // assemble state. TODO: do this straight in the piMsgTx buffer
#define HIL_TO_DEGS 0.1
#define HIL_TO_G 0.001
#define HIL_TO_RPM 10.
    piMsgHilInTx.gyro_x = (int16_t) (imuMsg.angular_velocity().x() * 57.3 / HIL_TO_DEGS);
    piMsgHilInTx.gyro_y = (int16_t) (imuMsg.angular_velocity().y() * 57.3 / HIL_TO_DEGS);
    piMsgHilInTx.gyro_z = (int16_t) (imuMsg.angular_velocity().z() * 57.3 / HIL_TO_DEGS);
    piMsgHilInTx.acc_x  = (int16_t) (-imuMsg.linear_acceleration().x() / 9.81 / HIL_TO_G);
    piMsgHilInTx.acc_y  = (int16_t) (imuMsg.linear_acceleration().y() / 9.81 / HIL_TO_G);
    piMsgHilInTx.acc_z  = (int16_t) (-imuMsg.linear_acceleration().z() / 9.81 / HIL_TO_G);
    piMsgHilInTx.baro = (int16_t) 0;
    double rads_in_fdm_numbering[4];
    for (auto actuator : this->dataPtr->actuators)
      rads_in_fdm_numbering[actuator.fdm_id] = this->dataPtr->rads[actuator.model_id];

    piMsgHilInTx.rpm_1  = (int16_t) (rads_in_fdm_numbering[0] * 9.55 / HIL_TO_RPM);
    piMsgHilInTx.rpm_2  = (int16_t) (rads_in_fdm_numbering[1] * 9.55 / HIL_TO_RPM);
    piMsgHilInTx.rpm_3  = (int16_t) (rads_in_fdm_numbering[2] * 9.55 / HIL_TO_RPM);
    piMsgHilInTx.rpm_4  = (int16_t) (rads_in_fdm_numbering[3] * 9.55 / HIL_TO_RPM);

    std::string message = "test serial port\n";
    // boost::asio
    //write(this->dataPtr->_serialPort, buffer(message));

    //if (!this->dataPtr->startedAsyncRead) {
    //  this->startAsyncRead();
    //  this->dataPtr->startedAsyncRead = true;
    //}
    //this->dataPtr->_io_service2.run_one();

    // termios
    //write(this->dataPtr->serialFd, &message, 17);
    //uint8_t buf[127];
    //ssize_t numBytes = read(this->dataPtr->serialFd, buf, 127);
    //if (numBytes) {
    //  //for (int i=0; i < numBytes; i++)
    //    //piParse(buffer[i]);
    //    //gzerr << buf[i] << std::endl;
    //}
    uint8_t tx_buf[PI_MAX_PACKET_LEN*2]; // *2 because encoding
    unsigned int tx_bytes = piAccumulateMsg(&piMsgHilInTx, tx_buf);
    write(this->dataPtr->serialFd, &tx_buf, tx_bytes);

    this->dataPtr->lastControllerUpdateTime = _info.simTime;

    uint8_t rx_buf[PI_MAX_PACKET_LEN*2]; // *2 because encoding
    ssize_t numBytes = read(this->dataPtr->serialFd, rx_buf, PI_MAX_PACKET_LEN*2);
    if (numBytes) {
      for (int i=0; i < numBytes; i++) {
        piParse(&this->dataPtr->p_hil, rx_buf[i]);
        //printf("%x", rx_buf[i]);
      }
      //printf("\n");
      //if (piMsgHilOutRxState != PI_MSG_RX_STATE_NONE)
        //gzerr << piMsgHilOutRx->set_1 << std::endl;
    }

    // todo: do this only when messaged changed
    

    this->UpdateOdometry(_info, _ecm);
  }
}

/////////////////////////////////////////////////
bool Indiflight::InitSockets(sdf::ElementPtr _sdf)
{
  std::string addr = _sdf->Get("odometry_streaming_address", static_cast<std::string>("127.0.0.1")).first;
  int port = _sdf->Get("odometry_streaming_port", static_cast<uint32_t>(5005)).first;
  this->dataPtr->udp_ac_id = _sdf->Get("odometry_streaming_ac_id", static_cast<uint32_t>(1)).first;

  double stateFreq = _sdf->Get<double>("odometry_streaming_frequency", 30).first;
  if (stateFreq > 0)
  {
    std::chrono::duration<double> period{1 / stateFreq};
    this->dataPtr->statePubPeriod = 
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }

  this->dataPtr->_endpoint = ip::udp::endpoint(ip::address::from_string(addr), port);
  this->dataPtr->_socket.open(ip::udp::v4());

  return true;
}
//    _sdf->Get("fdm_addr", static_cast<std::string>("127.0.0.1")).first;
//  this->dataPtr->listen_addr =
//    _sdf->Get("listen_addr", static_cast<std::string>("127.0.0.1")).first;
//  this->dataPtr->fdm_port_in =
//    _sdf->Get("fdm_port_in", static_cast<uint32_t>(9002)).first;
//  this->dataPtr->fcu_port_out =
//    _sdf->Get("fcu_port_out", static_cast<uint32_t>(9003)).first;
//
//  if (!this->dataPtr->socket_in.Bind(
//      this->dataPtr->listen_addr.c_str(),
//      this->dataPtr->fdm_port_in))
//  {
//    gzerr << "[" << this->dataPtr->modelName << "] "
//          << "failed to bind with " << this->dataPtr->listen_addr
//          << ":" << this->dataPtr->fdm_port_in << " aborting plugin.\n";
//    return false;
//  }
//
//  if (!this->dataPtr->socket_out.Connect(
//      this->dataPtr->fdm_addr.c_str(),
//      this->dataPtr->fcu_port_out))
//  {
//    gzerr << "[" << this->dataPtr->modelName << "] "
//          << "failed to bind with " << this->dataPtr->fdm_addr
//          << ":" << this->dataPtr->fcu_port_out << " aborting plugin.\n";
//    return false;
//  }
//
//  return true;
//}

void Indiflight::ApplyMotorForces(
    const double _dt,
    gz::sim::EntityComponentManager & _ecm)
{
  for (size_t i = 0; i < this->dataPtr->actuators.size(); i++) {
    // publish command actuator.fdm_id as a single double on actautor.input_rpm_pub
  }
}

void Indiflight::UpdateOdometry(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("Indiflight::UpdateOdometry");

  //---- publish current state, unless still within publishing period
  auto diff = _info.simTime - this->dataPtr->lastStatePubTime;
  if (this->dataPtr->statePubPeriod > std::chrono::steady_clock::duration::zero() 
          && diff > std::chrono::steady_clock::duration::zero()
          && diff < this->dataPtr->statePubPeriod)
  {
    // throttling required to stay above statePubPeriod
    return;
  }

  const math::Pose3d modelPose = worldPose(this->dataPtr->model.Entity(), _ecm);
  const math::Vector3d pos = modelPose.Pos();
  const math::Quaterniond rot = modelPose.Rot();

  const math::Vector3d vel = this->dataPtr->link.WorldLinearVelocity(_ecm).value();

  // already ENU, I thought...
  pose_t pose {
    .timeUs = _info.simTime.count(), // fixme
    .x = (float) -modelPose.Pos().Y(),
    .y = (float) modelPose.Pos().X(),
    .z = (float) modelPose.Pos().Z(),
    .qx = (float) -modelPose.Rot().Y(),
    .qy = (float) modelPose.Rot().X(),
    .qz = (float) modelPose.Rot().Z(),
    .qw = (float) modelPose.Rot().W(),
  };
  pose_der_t pose_der {
    .timeUs = _info.simTime.count(), // fixme
    .x = (float) -vel.Y(),
    .y = (float) vel.X(),
    .z = (float) vel.Z(),
    .wx = 0.,
    .wy = 0.,
    .wz = 0.,
  };

  static constexpr size_t Ni = sizeof(uint32_t);
  static constexpr size_t Np = sizeof(pose_t);
  static constexpr size_t Nd = sizeof(pose_der_t);

  uint8_t buf[Ni + Np + Nd];
  memcpy(buf, &(this->dataPtr->udp_ac_id), Ni);
  memcpy(buf+Ni, &pose, Np);
  memcpy(buf+Ni+Np, &pose_der, Nd);

  boost::system::error_code err;
  auto sent = this->dataPtr->_socket.send_to(buffer(buf, Ni+Np+Nd), this->dataPtr->_endpoint, 0, err);
  if (err.failed())
    gzerr << "udp send failed with error " << err << std::endl;

  gzerr << "sent at:" << _info.simTime.count() << std::endl;

  this->dataPtr->lastStatePubTime = _info.simTime;
}


GZ_ADD_PLUGIN(Indiflight,
                    System,
                    Indiflight::ISystemConfigure,
                    Indiflight::ISystemPreUpdate,
                    Indiflight::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(Indiflight,
                          "gz::sim::systems::Indiflight")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(Indiflight,
                          "ignition::gazebo::systems::Indiflight")
