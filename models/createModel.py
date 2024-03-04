
import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
import re
import numpy as np

def quat_from_two_vectors(u, v):
    # https://stackoverflow.com/questions/1171849/finding-quaternion-representing-the-rotation-from-one-vector-to-another
    k_cos_theta = u @ v
    k = np.sqrt(u @ u  *  v @ v)

    rotq = np.zeros(4) # scalar last!
    if (k_cos_theta / k > -0.9999):
        rotq[3] = k_cos_theta + k
        rotq[:3] = np.cross(u, v)
        rotq /= np.linalg.norm(rotq)
    else:
        rotq[0] = 1. # 180 rotation around x

    return rotq


class UAV(object):
    def writeSdf(self, filename):
        xml_str = ET.tostring(self._r, encoding='UTF-8', method='xml').decode()

        dom = minidom.parseString(xml_str)
        declaration = '<?xml version="1.0" encoding="UTF-8"?>\n'
        pretty_xml_str = re.sub(r'^<\?xml.*\n', declaration,  dom.toprettyxml(indent='  '))

        with open(filename, 'w') as f:
            f.write(pretty_xml_str)

    def __init__(self, name):
        self._name = name
        self._n = 0
        self._mass = 1
        self._I = np.eye(3)
        self._imu = {'rate' : 1000,
                     'pos'  : (0., 0., 0.),
                     'vBias': (0., 0., 0.),
                     'vStd' : (0., 0., 0.),
                     'aBias': (0., 0., 0.),
                     'aStd' : (0., 0., 0.),
                     }
        self._pose = [0.,0.,0.,  0.,0.,0.,1.] # scalar-last

    def setMassProperties(self, mass, Ixx, Iyy, Izz):
        self._mass = mass
        self._I[0,0] = Ixx
        self._I[1,1] = Iyy
        self._I[2,2] = Izz

    def setIMU(self, rate, pos, vBias, vStd, aBias, aStd):
        self._imu['rate']  = rate
        self._imu['pos']   = pos
        self._imu['vBias'] = vBias
        self._imu['vStd']  = vStd
        self._imu['aBias'] = aBias
        self._imu['aStd']  = aStd

    def setPose(self, pos, rotq=[0., 0., 0., 1.]):
        self._pose[:3] = pos
        self._pose[3:] = rotq

    def calculateG1G2(self):
        raise NotImplementedError("To be implemented by children")

    def buildSdf(self):
        self._r = ET.Element("sdf")
        self._r.set("version", "1.9")
        self._m = ET.SubElement(self._r, "model")
        self._m.set("name", self._name)

        pose = ET.SubElement(self._m, "pose")
        poseNED = self._pose
        poseNED[1] *= -1
        poseNED[2] *= -1
        poseNED[4] *= -1
        poseNED[5] *= -1
        pose.text = ' '.join(str(x) for x in self._pose)

        # Add self_collide element
        self_collide = ET.SubElement(self._m, "self_collide")
        self_collide.text = "false"

        # Add static element
        static = ET.SubElement(self._m, "static")
        static.text = "false"

        # Add link element
        link = ET.SubElement(self._m, "link")
        link.set("name", "base_link")

        # Add inertial element
        inertial = ET.SubElement(link, "inertial")
        mass = ET.SubElement(inertial, "mass")
        mass.text = str(self._mass)

        inertia = ET.SubElement(inertial, "inertia")
        inertia_names = ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'] 
        Is = self._I[np.triu_indices(3)]
        for i, name in enumerate(inertia_names):
            elem = ET.SubElement(inertia, name)
            elem.text = str(Is[i])

        # Add gravity element
        gravity = ET.SubElement(link, "gravity")
        gravity.text = "true"

        ET.SubElement(link, "velocity_decay")

        # Add visual element
        visual = ET.SubElement(link, "visual")
        visual.set("name", "base_link_visual")
        pose = ET.SubElement(visual, "pose")
        pose.text = "0 0 0 0 0 0"
        geometry = ET.SubElement(visual, "geometry")
        box = ET.SubElement(geometry, "box")
        size = ET.SubElement(box, "size")
        size.text = "0.13 0.03 0.04"

        # Add collision element
        collision = ET.SubElement(link, "collision")
        collision.set("name", "base_link_collision_0")
        pose = ET.SubElement(collision, "pose")
        pose.text = "0 0 0 0 0 0"
        geometry = ET.SubElement(collision, "geometry")
        box = ET.SubElement(geometry, "box")
        size = ET.SubElement(box, "size")
        size.text = "0.2 0.2 0.1"
        surface = ET.SubElement(collision, "surface")
        contact = ET.SubElement(surface, "contact")
        ode = ET.SubElement(contact, "ode")
        min_depth = ET.SubElement(ode, "min_depth")
        min_depth.text = "0.001"
        max_vel = ET.SubElement(ode, "max_vel")
        max_vel.text = "0"
        friction = ET.SubElement(surface, "friction")
        ET.SubElement(friction, "ode")

        # Add sensor element
        sensor = ET.SubElement(link, "sensor")
        sensor.set("name", "imu_sensor")
        sensor.set("type", "imu")
        always_on = ET.SubElement(sensor, "always_on")
        always_on.text = "1"
        update_rate = ET.SubElement(sensor, "update_rate")
        update_rate.text = str(self._imu['rate'])

        imu = ET.SubElement(sensor, "imu")
        angular_velocity = ET.SubElement(imu, "angular_velocity")
        linear_acceleration = ET.SubElement(imu, "linear_acceleration")

        axis_names = ['x', 'y', 'z']
        for i, axis in enumerate(axis_names):
            elem = ET.SubElement(angular_velocity, axis)
            noise = ET.SubElement(elem, "noise")
            noise.set("type", "gaussian")
            mean = ET.SubElement(noise, "mean")
            mean.text = str(self._imu['vBias'][i])
            stddev = ET.SubElement(noise, "stddev")
            stddev.text = str(self._imu['vStd'][i])

            elem = ET.SubElement(linear_acceleration, axis)
            noise = ET.SubElement(elem, "noise")
            noise.set("type", "gaussian")
            mean = ET.SubElement(noise, "mean")
            mean.text = str(self._imu['aBias'][i])
            stddev = ET.SubElement(noise, "stddev")
            stddev.text = str(self._imu['aStd'][i])

        enable_orientation = ET.SubElement(imu, "enable_orientation")
        enable_orientation.text = "false"

        self._indiPlugin = ET.SubElement(self._m, "plugin")
        self._indiPlugin.set("filename", "Indiflight")
        self._indiPlugin.set("name", "gz::sim::systems::Indiflight")

        odometry_link = ET.SubElement(self._indiPlugin, "odometry_link")
        odometry_link.text = "base_link"

        odometry_streaming_address = ET.SubElement(self._indiPlugin, "odometry_streaming_address")
        odometry_streaming_address.text = "10.0.0.1"

        odometry_streaming_port = ET.SubElement(self._indiPlugin, "odometry_streaming_port")
        odometry_streaming_port.text = "5005"

        odometry_streaming_frequency = ET.SubElement(self._indiPlugin, "odometry_streaming_frequency")
        odometry_streaming_frequency.text = "10"

        odometry_streaming_ac_id = ET.SubElement(self._indiPlugin, "odometry_streaming_ac_id")
        odometry_streaming_ac_id.text = "1"

        hil_device = ET.SubElement(self._indiPlugin, "hil_device")
        hil_device.text = "/dev/indiflight/hil"

        hil_baud = ET.SubElement(self._indiPlugin, "hil_baud")
        hil_baud.text = "921600"

class Rotor(object):
    def __init__(self, id, channel=None):
        self._id = id
        if channel is not None:
            self._channel = channel
        else:
            self._channel = id

        self._pos      = np.zeros(3, dtype=float)
        self._axis     = np.array([0., 0., -1.], dtype=float)
        self._rotq     = np.array([0., 0., 0., 1.], dtype=float) # scalar last
        self._mass     = 0.002
        self._Izz      = 3.7e-7
        self._D        = 3*0.0254
        self._omegaMax = 4100.
        self._Mmax     = (0.05, 0.02)
        self._tau      = (0.0125, 0.0125)
        self._k        = 2.7e-7
        self._cm       = 0.01
        self._kappa    = 0.55
        self._dir      = 1. # RH positive around _axis

    def setPose(self, pos, axis=(0., 0., -1.), direction="ccw"):
        if direction not in ["cw", "ccw"]:
            raise ValueError("Direction must be one of cw, ccw")

        a = np.array(axis, dtype=float)
        l = np.linalg.norm(a)
        if l < 1e-6:
            raise ValueError("axis must not be (close to) zero-length")

        self._pos = np.array(pos, dtype=float)
        self._axis = a / l
        self._rotq = quat_from_two_vectors(np.array([0., 0., -1]), self._axis)
        self._dir = +1 if direction == "ccw" else -1

    def setMassProperties(self, mass, Izz):
        self._mass = mass
        self._Izz = Izz

    def setPerformance(self, D, omegaMax, k, cm, kappa, tau_up, tau_down, Mmax_up, Mmax_down):
        self._D = D
        self._omegaMax = omegaMax
        self._k = k
        self._cm = cm
        self._kappa = kappa
        self._Mmax = (Mmax_down, Mmax_up)
        self._tau = (tau_down, tau_up)

class Multirotor(UAV):
    def __init__(self, name):
        super().__init__(name)
        self._rotors = []

    def addRotor(self, rotor):
        self._n += 1
        self._rotors.append(rotor)

    def buildSdf(self):
        super().buildSdf()

        for i, rotor in enumerate(self._rotors):
            link = ET.SubElement(self._m, "link")
            link.set("name", f"rotor_{i}")

            gravity = ET.SubElement(link, "gravity")
            gravity.text = "true"

            self_collide = ET.SubElement(link, "self_collide")
            self_collide.text = "false"

            ET.SubElement(link, "velocity_decay")

            pose = ET.SubElement(link, "pose")
            posFLU = rotor._pos
            posFLU[1] *= -1
            posFLU[2] *= -1

            rotqFLU = rotor._rotq # scalar last
            rotqFLU[1] *= -1
            rotqFLU[2] *= -1

            pose.text = ' '.join(str(x) for x in posFLU)
            pose.text += ' ' + ' '.join(str(x) for x in rotqFLU)

            inertial = ET.SubElement(link, "inertial")
            mass = ET.SubElement(inertial, "mass")
            mass.text = str(rotor._mass)

            inertia = ET.SubElement(inertial, "inertia")
            ixx = ET.SubElement(inertia, "ixx")
            ixx.text = str(rotor._Izz * 0.05)
            iyy = ET.SubElement(inertia, "iyy")
            iyy.text = str(rotor._Izz)
            izz = ET.SubElement(inertia, "izz")
            izz.text = str(rotor._Izz)

            visual = ET.SubElement(link, "visual")
            visual.set("name", f"rotor_{i}_visual")
            pose = ET.SubElement(visual, "pose")
            pose.text = "-0.00825 -0.0525 -0.006 0 0 0"
            geometry = ET.SubElement(visual, "geometry")
            mesh = ET.SubElement(geometry, "mesh")
            scale = ET.SubElement(mesh, "scale")
            scale.text = "0.3 0.3 0.3"
            uri = ET.SubElement(mesh, "uri")
            turningDirectionString = "ccw" if rotor._dir == +1 else "cw"
            uri.text = f"model://CineRat/meshes/1345_prop_{turningDirectionString}.stl"
            material = ET.SubElement(visual, "material")
            script = ET.SubElement(material, "script")
            name = ET.SubElement(script, "name")
            name.text = "Gazebo/DarkGrey"
            uri = ET.SubElement(script, "uri")
            uri.text = "file://media/materials/scripts/gazebo.material"

            joint = ET.SubElement(self._m, "joint")
            joint.set("name", f"rotor_{i}_joint")
            joint.set("type", "revolute")
            parent = ET.SubElement(joint, "parent")
            parent.text = "base_link"
            child = ET.SubElement(joint, "child")
            child.text = f"rotor_{i}"
            axis = ET.SubElement(joint, "axis")
            xyz = ET.SubElement(axis, "xyz")
            axisNED = rotor._axis
            axisNED[1] *= -1
            axisNED[2] *= -1
            xyz.text = " ".join(str(x) for x in axisNED)
            limit = ET.SubElement(axis, "limit")
            lower = ET.SubElement(limit, "lower")
            lower.text = "-1e16"
            upper = ET.SubElement(limit, "upper")
            upper.text = "1e16"

            plugin = ET.SubElement(self._m, "plugin")
            plugin.set("filename", "MulticopterMotorModelTB")
            plugin.set("name", "gz::sim::systems::MulticopterMotorModelTB")

            jointName = ET.SubElement(plugin, "jointName")
            jointName.text = f"rotor_{i}_joint"

            linkName = ET.SubElement(plugin, "linkName")
            linkName.text = f"rotor_{i}"

            turningDirection = ET.SubElement(plugin, "turningDirection")
            turningDirection.text = turningDirectionString

            timeConstantUp = ET.SubElement(plugin, "timeConstantUp")
            timeConstantUp.text = str(rotor._tau[1])

            timeConstantDown = ET.SubElement(plugin, "timeConstantDown")
            timeConstantDown.text = str(rotor._tau[0])

            maxTorqueUp = ET.SubElement(plugin, "maxTorqueUp")
            maxTorqueUp.text = str(rotor._Mmax[0])

            maxTorqueDown = ET.SubElement(plugin, "maxTorqueDown")
            maxTorqueDown.text = str(rotor._Mmax[0])

            rotorInertia = ET.SubElement(plugin, "rotorInertia")
            rotorInertia.text = str(rotor._Izz)

            inertiaEffects = ET.SubElement(plugin, "inertiaEffects")
            inertiaEffects.text = "true"

            maxRotVelocity = ET.SubElement(plugin, "maxRotVelocity")
            maxRotVelocity.text = str(rotor._omegaMax)

            motorConstant = ET.SubElement(plugin, "motorConstant")
            motorConstant.text = str(rotor._k)

            momentConstant = ET.SubElement(plugin, "momentConstant")
            momentConstant.text = str(rotor._cm)

            motorNumber = ET.SubElement(plugin, "motorNumber")
            motorNumber.text = str(rotor._id)

            rotorDragCoefficient = ET.SubElement(plugin, "rotorDragCoefficient")
            rotorDragCoefficient.text = "0"

            rollingMomentCoefficient = ET.SubElement(plugin, "rollingMomentCoefficient")
            rollingMomentCoefficient.text = "0"

            rotorVelocitySlowdownSim = ET.SubElement(plugin, "rotorVelocitySlowdownSim")
            rotorVelocitySlowdownSim.text = "50"

            motorType = ET.SubElement(plugin, "motorType")
            motorType.text = "velocity"

            publishState = ET.SubElement(plugin, "publishState")
            publishState.text = "true"

            statePubFrequency = ET.SubElement(plugin, "statePubFrequency")
            statePubFrequency.text = "0"

            realTimeStrictness = ET.SubElement(plugin, "realTimeStrictness")
            realTimeStrictness.text = "1.1"

            maxDrops = ET.SubElement(plugin, "maxDrops")
            maxDrops.text = "4"


            actuator = ET.SubElement(self._indiPlugin, "actuator")
            actuator.set("motorNumber", str(i))
            actuator.set("channel", str(rotor._channel))

            commandTopic = ET.SubElement(actuator, "commandTopic")
            commandTopic.text = f"/model/{self._name}/actuator_command"

            actuatorStateTopic = ET.SubElement(actuator, "actuatorStateTopic")
            actuatorStateTopic.text = f"/model/{self._name}/actuator_state"

            multiplier = ET.SubElement(actuator, "multiplier")
            multiplier.text = str(rotor._omegaMax)

            thrustNonlinearity = ET.SubElement(actuator, "thrustNonlinearity")
            thrustNonlinearity.text = str(rotor._kappa)

    def calculateG1G2(self):
        return super().calculateG1G2()


if __name__=="__main__":
    from copy import deepcopy
    m = Multirotor('test')
    m.setPose((0., 0., -.24))
    m.setMassProperties(0.43, 0.00075, 0.0008, 0.0009)

    r_RR = Rotor(0)
    r_RR.setPose((-0.0455, 0.0635, -0.02))

    r_FR = deepcopy(r_RR)
    r_FR._id = r_FR._channel = 1
    r_FR.setPose((0.0455, 0.0635, -0.02))

    r_RL = deepcopy(r_RR)
    r_RL._id = r_RL._channel = 2
    r_RL.setPose((-0.0455, -0.0635, -0.02))

    r_FL = deepcopy(r_RR)
    r_FL._id = r_FL._channel = 3
    r_FL.setPose((0.0455, -0.0635, -0.02))

    m.addRotor(r_RR)
    m.addRotor(r_FR)
    m.addRotor(r_RL)
    m.addRotor(r_FL)

    m.buildSdf()
    m.writeSdf('test.sdf')
