""" code to generate the UWB measurements change with height graphs. used to characterize the behaviour of the sensor w.r.t. a tag height change"""

## IMPORT
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.stats import norm


## FUNCTIONS & UTILITIES
def generate_quartercircle(c_x, c_y, r, s=0.1):
    """
    generetes the set of points of a quartercircle centered in [c_x, c_y], with radius r and stepsize s
    """        

    y = np.arange(c_y, c_y + r + s, s)
    x = -np.sqrt(r**2 - y**2)

    # since each x value has two corresponding y-values, duplicate x-axis.
    # [::-1] is required to have the correct order of elements for plt.plot. 
    # x = np.concatenate([x,-x[::-1]])

    # # concatenate y and flipped y. 
    # y = np.concatenate([y,y[::-1]])
    
    return x, y + c_x

# list of colors for plot 
color_list = ['blue','red','green','cyan'] #'purple','brown','gray','olive','cyan','darkblue','tan','bisque']

# PATHS for load data and save graph
THIS_PATH = os.getcwd()+'/UWB caracterization/misure_CH9'
SAVE_PATH = THIS_PATH + '/Graphs/'
LOAD_PATH_AOA = THIS_PATH + '/Aoas/MISURE_ALTEZZE/H_'

# Create SAVE_PATH if not already present
if not(os.path.isdir(SAVE_PATH)):
    os.makedirs(SAVE_PATH)

# alle the heights at which we have take the measure
heights = [46,60,115]
adder = 0                   # for string character extraction purposes in aoa_list generation
for height in heights:
    if height == 115:
        adder = 1
        
    # load path for the specific height
    aoa_fpt = LOAD_PATH_AOA+str(height)+'/'

    # list all the .csv data dile in the directories
    aoa_listdir = sorted(os.listdir(aoa_fpt))

    # initialize void list for the path of each .csv file
    aoa_file_list = []

    # create the list of all the .csv full paths
    aoa_file_list.extend([aoa_fpt+i for i in aoa_listdir])
    
    # initialize void lists for datas
    all_array_aoa = []
    aoa_list = []
    
    # load all aoa data in one array. aoa_list, list the order with which the data are loaded w.r.t. the aoa
    for f in aoa_file_list:
        all_array_aoa.append(np.genfromtxt(f, delimiter=','))
        aoa_list.append(int(f[-13:-11]))
        
        
    # Plot the data at 75 degree first, for aesthetics reasons
    plt.figure(1)
    aoa = 75
    coords = height*np.array([-np.abs(np.sin(all_array_aoa[-1]*np.pi/180)), np.cos(all_array_aoa[-1]*np.pi/180)])
    plt.scatter(coords[0],coords[1], marker='+', c=color_list[int(abs(aoa/30)+0.5)]) #facecolors='none', edgecolors=color_list[int(abs(aoa/15))])
    real_coord = height*np.array([np.sin(-aoa*np.pi/180),np.cos(-aoa*np.pi/180)])
    plt.scatter(real_coord[0], real_coord[1], marker='s',c=color_list[int(abs(aoa/30)+0.5)])
    plt.plot([0,real_coord[0]], [0,real_coord[1]], c=color_list[int(abs(aoa/30)+0.5)])
    plt.plot([0.0, 35.0], [height,height], '--', c='k')
    
    
    # Plot the data cloud for each angle for the current height, looping in the loaded data.
    loop = enumerate(aoa_list[:-1])
    
    for i, aoa in loop:
        if aoa > 50:
            coords = height*np.array([-np.abs(np.sin(all_array_aoa[i]*np.pi/180)), np.cos(all_array_aoa[i]*np.pi/180)])
        else:
            coords = height*np.array([np.sin(all_array_aoa[i]*np.pi/180), np.cos(all_array_aoa[i]*np.pi/180)])
        plt.scatter(coords[0],coords[1], marker='+', c=color_list[int(abs(aoa/30)+0.5)]) #facecolors='none', edgecolors=color_list[int(abs(aoa/15))])
        real_coord = height*np.array([np.sin(-aoa*np.pi/180),np.cos(-aoa*np.pi/180)])
        plt.scatter(real_coord[0], real_coord[1], marker='s',c=color_list[int(abs(aoa/30)+0.5)])
        plt.plot([0,real_coord[0]], [0,real_coord[1]], c=color_list[int(abs(aoa/30)+0.5)])
        plt.plot([0.0, 35.0], [height,height], '--', c='k')
    x,y = generate_quartercircle(0,0,height, 0.1)
    plt.text(8, height+1, 'h = '+str(height), ha='left')
    plt.plot(x, y, '--', c='k')
    adder = 0
plt.axis('equal')
plt.ylim(top=130)
plt.plot(0,0,'-ko')
plt.savefig(SAVE_PATH+'ch9_heights.png',dpi=300)
plt.show()
plt.close()
