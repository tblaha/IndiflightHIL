from copy import deepcopy
from createModel import Multirotor, Rotor

if __name__=="__main__":
    m = Multirotor('generated')

    # general properties
    m.setPose((0., 0., -.24))
    m.setMassProperties(0.428, 0.0007206, 0.00077, 0.00089)

    # rotors properties
    RR = Rotor(0)
    FR = deepcopy(RR); FR._id = FR._channel = 1
    RL = deepcopy(RR); RL._id = RL._channel = 2
    FL = deepcopy(RR); FL._id = FL._channel = 3

    RR.setPose((-0.0455, 0.0635, -0.02), direction="cw")
    FR.setPose((0.0455, 0.0635, -0.02), direction="ccw")
    RL.setPose((-0.0455, -0.0635, -0.02), direction="ccw")
    FL.setPose((0.0455, -0.0635, -0.02), direction="cw")

    m.addRotor(RR)
    m.addRotor(FR)
    m.addRotor(RL)
    m.addRotor(FL)

    m.buildSdf()

    m.write("models/generated")