
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import ode
from mpl_toolkits.mplot3d import Axes3D
from OrbitPropagator import OrbitPropagator as OP
import planetary_data as pd

# change out "sun" here with any other planets in planetary_data
cb = pd.earth

if __name__ == '__main__':
    # initial conditions of orbit parameters
    r_mag = cb['radius'] + cb['radius']*0.5 # km
    v_mag = np.sqrt(cb['mu']/r_mag) # km/s

    # initial position and velocity vectors
    r0 = [r_mag,r_mag*0.01,r_mag*-0.3]
    v0 = [0,v_mag,v_mag*0.6]

    # timespan one day
    tspan = 6*3600*24 # s

    # timestep
    dt = 100 # s

    op = OP(r0,v0,tspan,dt,cb = cb)
    op.propagate_orbit()
    op.plot_3d(show_plot = True)
