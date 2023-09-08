import numpy as np
import matplotlib.pyplot as plt

positions = np.load('UTM_position.npy').T

print(positions[1])

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.scatter(positions[0,0:1000], positions[1, 0:1000], positions[2, 0:1000])
ax.scatter(positions[0,1000:], positions[1, 1000:], positions[2, 1000:])

ax.set_xlabel('East (m)')
ax.set_ylabel('North (m)')
ax.set_zlabel('Altitude (m)')

fig2 = plt.figure()
plt.plot(positions[0,:1000], positions[1,:1000])
plt.plot(positions[0,1000:], positions[1,1000:])
plt.xlabel('East (m)')
plt.ylabel('North (m)')

plt.show()
