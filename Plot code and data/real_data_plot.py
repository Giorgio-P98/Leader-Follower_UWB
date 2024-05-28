"""Code to generate the real test plots. Set the WHICH_DATASET variable to the wanted set of data (contained in the real_data folder) """

# IMPORTS
from matplotlib import pyplot as plt
import numpy as np
import os
from scipy.stats import norm


# POLICY CONSTANTS
takeoff_height = 0.4
following_distance = 1.2

# LOAD AND SAVE PATH -> which_dataset specifies the date-time of the test to use
THIS_PATH = os.getcwd()

# Uncomment the wanted dataset (POLICY COSTANTS may vary for the different tests, but those values are only for plot aspect)
# which_dataset = '2024-05-27 16:30:37/'
# which_dataset = '2024-05-27 17:13:45/'
which_dataset = '2024-05-27 17:55:26/'

LOAD_PATH = THIS_PATH+'/real_data/'+which_dataset
SAVE_PATH = THIS_PATH+'/real_data/graphs/'+which_dataset

# Create the SAVE_PATH if not previously generated
if not(os.path.isdir(SAVE_PATH)):
    os.makedirs(SAVE_PATH)

# Load data arrays from saved .csv files
alpha = np.genfromtxt(LOAD_PATH+'angles.csv', delimiter=',')
drone_true_pos = np.genfromtxt(LOAD_PATH+'drone_xy_pos.csv', delimiter=',')
True_range= np.genfromtxt(LOAD_PATH+'ranges.csv', delimiter=',')
target_true_pos = np.genfromtxt(LOAD_PATH+'target_xy_pos.csv', delimiter=',')
time = np.genfromtxt(LOAD_PATH+'time_vec.csv', delimiter=',')
Uwb_aoa = np.genfromtxt(LOAD_PATH+'Uwb_aoa.csv', delimiter=',')
init_range = np.genfromtxt(LOAD_PATH+'Uwb_range.csv', delimiter=',')
yaw_estim = np.genfromtxt(LOAD_PATH+'yaw_estim.csv', delimiter=',')
drone_pos_estim = np.genfromtxt(LOAD_PATH+'drone_xy_pos_estim.csv', delimiter=',')

# Correct the range with the hieght of the antenna w.r.t. the flight controller 
Uwb_range = np.zeros(shape=init_range.shape)
Uwb_range[15:] = np.sqrt(init_range[15:]**2 - 0.24**2) 

# Correct the drone height with the initial drone FC height (The first 15 elements of all the data arrays are zeros due to the node sinc)
drone_height = drone_true_pos[:,2] - drone_true_pos[15,2]

# menage wrong angle values (generated from the use of atan2 in calculating the true angle between drone and target)
alpha = alpha - np.sign(alpha)*np.floor(np.abs(alpha)/300)*360

# calculate the angle between drone and target as the sum of the measured aoa and the FC drone estimated yaw (used for the tag Localization performed by the drone)
theta = (Uwb_aoa*np.pi/180 + yaw_estim)

# Calculate the estimated target position as the sum of the local measurd position w.r.t. drone and his globla position (plus the initial position for comparison with the ground truth)
meas_tag_pos = np.transpose(Uwb_range * np.array([np.cos(theta),np.sin(theta)]))
estim_tag_pos = meas_tag_pos + drone_pos_estim[:,:2] + drone_true_pos[15,:2]

# Calculate the localization error
tag_pos_errx = estim_tag_pos[:,0] - target_true_pos[:,0]
tag_pos_erry = estim_tag_pos[:,1] - target_true_pos[:,1]   


## PLOTS
plt.figure(1)
plt.plot(drone_true_pos[15:-800,0], drone_true_pos[15:-800,1], label='Drone')
plt.plot(target_true_pos[15:-800,0], target_true_pos[15:-800,1], label='Target')
plt.plot(drone_true_pos[15,0], drone_true_pos[15,1], '>', c='tab:blue', markersize=10)
plt.plot(target_true_pos[15,0], target_true_pos[15,1], '>', c='tab:orange', markersize=10)
plt.xlabel('x Position [m]')
plt.ylabel('y Position [m]')
plt.legend()
# plt.title('Drone & Target x-y position',fontweight='bold')
plt.savefig(SAVE_PATH+'drone_target_xy_pos.png', dpi=300)

plt.figure(2)
plt.plot(time[15:-800], -drone_height[15:-800])
plt.axhline(y=takeoff_height, color='r', linestyle='--', linewidth=2)
plt.ylabel('Height [m]')
plt.xlabel('Time [s]')
# plt.title('Drone height',fontweight='bold')
plt.savefig(SAVE_PATH+'drone_height.png', dpi=300)

plt.figure(3)
plt.plot(time[15:-800], True_range[15:-800], label='Mocap')
plt.plot(time[15:-800], Uwb_range[15:-800], label='Uwb')
plt.axhline(y=following_distance, color='r', linestyle='--', linewidth=2)
plt.ylabel('range [m]')
plt.xlabel('Time [s]')
plt.legend()
# plt.title('Range between drone and target',fontweight='bold')
plt.savefig(SAVE_PATH+'range_Uwb_Mocap.png', dpi=300)

plt.figure(4)
plt.plot(time[15:-800], alpha[15:-800], label='Mocap')
plt.plot(time[15:-800], Uwb_aoa[15:-800], label='Uwb')
plt.axhline(y=15, color='r', linestyle='--', linewidth=2)
plt.axhline(y=-15, color='r', linestyle='--', linewidth=2)
plt.ylabel('theta [deg]')
plt.xlabel('Time [s]')
plt.legend()
# plt.title('angle between drone and target',fontweight='bold')
plt.savefig(SAVE_PATH+'aoa_Uwb_Mocap.png', dpi=300)

plt.figure(5)
plt.plot(estim_tag_pos[15:-800,0], estim_tag_pos[15:-800,1], label='Estimated')
plt.plot(target_true_pos[15:-800,0], target_true_pos[15:-800,1], label='True')
plt.ylabel('y [m]')
plt.xlabel('x [m]')
plt.legend()
# plt.title('Target position estimat vs real position',fontweight='bold')
plt.savefig(SAVE_PATH+'tag_estim-real_pos.png', dpi=300)

plt.figure(6,figsize=(10,3))
plt.plot(time[15:-800], tag_pos_errx[15:-800])
plt.ylabel('x error [m]')
plt.xlabel('Time [s]')
# plt.title('Target estimation error in x',fontweight='bold')
plt.savefig(SAVE_PATH+'real_tag_estimation_errx.png', dpi=300)

plt.figure(7,figsize=(10,3))
plt.plot(time[15:-800], tag_pos_erry[15:-800])
plt.ylabel('y error [m]')
plt.xlabel('Time [s]')
# plt.title('Target estimation error in y',fontweight='bold')
plt.savefig(SAVE_PATH+'real_tag_estimation_erry.png', dpi=300)

# X location Error distribution with his fitted gaussian distribution Plot
plt.figure(8, figsize=(6,7))
mu, std = norm.fit(tag_pos_errx[15:-800]) 
plt.hist(tag_pos_errx[15:-800], bins=41, density=True, alpha=0.6, color='b')
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = norm.pdf(x, mu, std)

plt.plot(x, p, 'k', linewidth=2)
plt.title("x error: \u03BC: {:.4f}m, \u03C3: {:.4f}m".format(mu, std))
plt.savefig(SAVE_PATH+'real_tag_estim-hist_errx.png', dpi=300)

# Y location Error distribution with his fitted gaussian distribution Plot
plt.figure(9, figsize=(6,7))
mu, std = norm.fit(tag_pos_erry[15:-800]) 
plt.hist(tag_pos_erry[15:-800], bins=41, density=True, alpha=0.6, color='b')
# Plot the PDF.
xmin, xmax = plt.xlim()
x = np.linspace(xmin, xmax, 100)
p = norm.pdf(x, mu, std)

plt.plot(x, p, 'k', linewidth=2)
plt.title("y error: \u03BC: {:.4f}m, \u03C3: {:.4f}m".format(mu, std))
plt.savefig(SAVE_PATH+'real_tag_estim-hist_erry.png', dpi=300)

plt.show()
plt.close()