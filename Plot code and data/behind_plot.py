""" code to generate the UWB measurements in the entire plane at a distance of 1 meter. used to characterize the behaviour behind the sensor"""

## IMPORTS
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.stats import norm


## FUNCTIONS & UTILITIES
def generate_circle(c_x, c_y, r, s=0.1):
    """
    generetes the set of points of a circle centered in [c_x, c_y], with radius r and stepsize s
    """        

    y = np.arange(c_y - r, c_y + r + s, s)
    x = -np.sqrt(r**2 - y**2)

    # since each x value has two corresponding y-values, duplicate x-axis.
    # [::-1] is required to have the correct order of elements for plt.plot. 
    x = np.concatenate([x,-x[::-1]])

    # concatenate y and flipped y. 
    y = np.concatenate([y,y[::-1]])
    
    return x, y + c_x

# list of colors for plot 
color_list = ['blue','orange','green','red','purple','brown','pink','gray','olive','cyan','darkblue','tan','bisque']

# PATHS for load data and save graph
THIS_PATH = os.getcwd()+'/UWB characterization/misure_CH9'
SAVE_PATH = THIS_PATH + '/Graphs/'
LOAD_PATH_AOA = THIS_PATH + '/Aoas/'
LOAD_PATH_RANGE = THIS_PATH + '/Ranges/'
comnames = ['DIETRO_','RANGE_']

# Create SAVE_PATH if not already present
if not(os.path.isdir(SAVE_PATH)):
    os.makedirs(SAVE_PATH)

for comname in comnames:
    
    # load path for the specific comname
    aoa_fpt = LOAD_PATH_AOA+comname+str(100)+'/'
    range_fpt = LOAD_PATH_RANGE+comname+str(100)+'/'

    # list all the .csv data dile in the directories
    aoa_listdir = sorted(os.listdir(aoa_fpt))
    range_listdir = sorted(os.listdir(range_fpt))

    # initialize void lists for the path of each .csv file
    aoa_file_list, range_file_list = [], []

    # create the list of all the .csv full paths
    aoa_file_list.extend([aoa_fpt+i for i in aoa_listdir])
    range_file_list.extend([range_fpt+i for i in range_listdir])

    # initialize void lists for datas
    all_array_aoa, all_array_range = [], []
    aoa_list= []

    # load all the data in two list of arrays (range & aoa). aoa_list, list the order with which the data are loaded w.r.t. the aoa
    for f in aoa_file_list:
        all_array_aoa.append(np.genfromtxt(f, delimiter=','))
        aoa_list.append(int(f[-7:-4]))

    for f in range_file_list:
        all_array_range.append(np.genfromtxt(f, delimiter=','))
        
    # Plot the data cloud for each angle for the current semi-plane, looping in the loaded data.
    loop = enumerate(aoa_list)

    plt.figure(1)
    if comname == 'RANGE_':
        for i, aoa in loop:
            coords = all_array_range[i]*np.array([np.sin(all_array_aoa[i]*np.pi/180), np.cos(all_array_aoa[i]*np.pi/180)])
            plt.scatter(coords[0],coords[1], marker='+', c=color_list[int(abs(aoa/15))])
            real_coord = 100*np.array([np.sin(aoa*np.pi/180),np.cos(aoa*np.pi/180)])
            plt.scatter(real_coord[0], real_coord[1], marker='s',c=color_list[int(abs(aoa/15))])
            plt.plot([0,real_coord[0]], [0,real_coord[1]], c=color_list[int(abs(aoa/15))])
        adder = 0
    else:
        for i, aoa in loop:
            coords = all_array_range[i]*np.array([np.sin(all_array_aoa[i]*np.pi/180 + np.pi), np.cos(all_array_aoa[i]*np.pi/180+ np.pi)])
            plt.scatter(coords[0],coords[1], marker='+', c=color_list[int(abs(aoa/15))])
            real_coord = 100*np.array([np.sin(aoa*np.pi/180 + np.pi),np.cos(aoa*np.pi/180 + np.pi)])
            plt.scatter(real_coord[0], real_coord[1], marker='s',c=color_list[int(abs(aoa/15))])
            plt.plot([0,real_coord[0]], [0,real_coord[1]], c=color_list[int(abs(aoa/15))])
        adder = 0
x,y = generate_circle(0,0,100, 0.1)
plt.plot(x, y, '--', c='k')
plt.plot(0,0,'-ko')
plt.axis('equal')
plt.savefig(SAVE_PATH+'ch9_behind_measure.png',dpi=300)
plt.show()
plt.close()