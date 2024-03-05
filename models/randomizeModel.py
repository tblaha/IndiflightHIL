from createModel import Multirotor, Rotor

import numpy as np
from numpy import random
from datetime import datetime

if __name__=="__main__":
    m = Multirotor('generated')

    # general properties
    m.setPose((0., 0., -.24))
    m.setMassProperties(0.428, 0.0007206, 0.00077, 0.00089)

    # rotors properties
    N = 4

    offset = np.pi/3. * random.random()
    phi = random.random(N) * np.pi/3  +  np.arange(0, 2.*np.pi, np.pi/2.)
    R = random.random(N) * 0.1 + 0.05

    X = R * np.stack(( np.cos(phi), np.sin(phi), np.zeros(N) ))
    X[2, :] = -0.02

    dirs = ["cw", "ccw", "cw", "ccw"] if random.randint(0,2) else ["ccw", "cw", "ccw", "cw"]

    kappas = random.random(N) * 0.5 + 0.25
    taus = random.random(N) * 0.02 + 0.015
    cms = random.random(N) * 0.01 + 0.01

    #from scipy.spatial.transform import Rotation as R
    #alpha = random.random(N) * 5. * np.pi/180.
    #gamma = random.random(N) * 2. * np.pi
    #alpha *= 0
    #gamma *= 0
    #alpha[0] = 1. * np.pi/180.
    #rotax = ( alpha * np.stack(( np.cos(phi), np.sin(phi), np.zeros(N) )) ).T
    #Rots = R.from_rotvec(rotax)
    #prop_axes = Rots.apply((0., 0., -1.))

    perm = random.permutation(range(N))
    for i, p in enumerate(perm):
        rotor = Rotor(p)
        rotor.setPose(X[:, i], direction=dirs[i])
        #rotor.setPose(X[:, i], axis=prop_axes[i], direction=dirs[i])
        rotor._kappa = kappas[i]
        rotor._tau = (taus[i], taus[i])
        rotor._cm = cms[i]
        m.addRotor(rotor)

    G1, G2, Gs = m.calculateG1G2()

    import pickle
    with open(f'{datetime.now().strftime("%Y-%m-%dT%H:%M:%S")}.pkl', 'wb') as f:
        pickle.dump({
            'G1': G1[:, perm],
            'G2': G2[:, perm],
            'Gs': np.array(Gs)[perm],
            'kappas': kappas[perm],
            'taus': taus[perm],
            'cms': cms[perm],
            }, f)

    m.buildSdf()
    m.write("models/generated")