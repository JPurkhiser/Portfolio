import numpy as np
import matplotlib.pyplot as plt
from Func import integrateGraph

#Variables
totalMass = 1
dryMass = 0.906
burnTime = 3.5
totalImpulse = 49.6
propellantMass = 0.064
averageThrust = totalImpulse/burnTime
massFlowRate = propellantMass/burnTime
Landed = 0
n = 0

time = np.linspace(0, 10, 100, False)
print('Test')

index = int(np.where(time==burnTime)[0] + 1)
thrust = np.append(np.repeat(averageThrust, index), np.repeat(0, len(time) - index))
mass = np.append(np.repeat(totalMass, index) - time[0:index] * massFlowRate, np.repeat(dryMass, len(time) - index))
acceleration = thrust/mass - 9.81
velocity = integrateGraph(time, acceleration)
displacement = integrateGraph(time, velocity)

while Landed == 0:
    disp = displacement[n]
    if n != 0 and disp <= 0:
        Landed = 1
        airTime = np.linspace(0, n/10, n, False)
    n = n + 1

index = int(np.where(airTime==burnTime)[0] + 1)
thrust = np.append(np.repeat(averageThrust, index), np.repeat(0, len(airTime) - index))
mass = np.append(np.repeat(totalMass, index) - airTime[0:index] * massFlowRate, np.repeat(dryMass, len(airTime) - index))
acceleration = thrust/mass - 9.81
velocity = integrateGraph(airTime, acceleration)
displacement = integrateGraph(airTime, velocity)


plt.subplot(131)
plt.plot(airTime, acceleration, "r")
plt.xlabel("Time")
plt.ylabel("Acceleration")

plt.subplot(132)
plt.plot(airTime, velocity, "b")
plt.xlabel("Time")
plt.ylabel("Velocity")

plt.subplot(133)
plt.plot(airTime, displacement, "g")
plt.xlabel("Time")
plt.ylabel("Displacement")
plt.show()