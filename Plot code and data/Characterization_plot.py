""" code to generate the UWB measurements semi-plane characterization. Change the variable CHANNEL = [9, 5], in order to plot the graphs of the selected channel. make sure to run this code while inside the '/Plot code and data' folder
"""

## IMPORTS
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.stats import norm


## DATA SELECTION CONSTANTS

# CHANNEL var 9 or 5
CHANNEL = 5

# Select angle and distance of which you want to plot the distributions (both range and aoa). choose a valid distance and a valid angle
WANT_AOA = 0
WANT_DIST = 150


## FUNCTIONS & UTILITIES
def generate_semicircle(c_x:float, c_y:float, r:float, s=0.1):
    """
    generetes the set of points of a semicircle centered in [c_x, c_y], with radius r and stepsize s
    """        

    y = np.arange(c_y, c_y + r + s, s)
    x = np.sqrt(r**2 - y**2)

    # since each x value has two corresponding y-values, duplicate x-axis.
    # [::-1] is required to have the correct order of elements for plt.plot. 
    x = np.concatenate([x,-x[::-1]])

    # concatenate y and flipped y. 
    y = np.concatenate([y,y[::-1]])
    
    return x, y + c_x

def plot_distributions(want_aoa, want_dist, aoas, aoa_array, range_array):
    """
    generates the distributions of both range and angle at the want_aoa angle
    """
    # Extracted the list index at which corresponds the selected angle
    i = np.where(np.array(aoas) == want_aoa)[0]
    
    if len(i) == 0:
        print('the chosen angle is not present in the data for the selected range. Please change the WANT_AOA value accordingly')
        exit()
    else:
        i = i.item()
    
    # Plot of the aoa distribution, with the fitted gaussian
    plt.figure(1)
    mu, std = norm.fit(aoa_array[i]) 
    plt.hist(aoa_array[i], bins=25, density=True, alpha=0.6, color='b')
    # Plot the PDF.
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = norm.pdf(x, mu, std)

    plt.plot(x, p, 'k', linewidth=2)
    plt.title("\u03BC: {:.2f}°, \u03C3: {:.2f}°".format(mu, std))
    plt.savefig(SAVE_PATH+'ch'+str(CHANNEL)+'_aoa_hist__angle='+str(want_aoa)+'_dist='+str(want_dist)+'.png',dpi=300)
    
    # Plot of the range distribution, with the fitted gaussian
    plt.figure(2)
    mu, std = norm.fit(range_array[i]) 
    plt.hist(range_array[i], bins=9, density=True, alpha=0.6, color='b')
    # Plot the PDF.
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = norm.pdf(x, mu, std)
    
    plt.plot(x, p, 'k', linewidth=2)
    plt.title("\u03BC: {:.2f} cm, \u03C3: {:.2f} cm".format(mu, std))
    plt.savefig(SAVE_PATH+'ch'+str(CHANNEL)+'_range_hist__angle='+str(want_aoa)+'_dist'+str(want_dist)+'.png',dpi=300)

# list of colors for plot 
color_list = ['blue','orange','green','red','purple','brown','pink','gray','olive','cyan','darkblue','tan','bisque']


## MAIN CODE

# channel 9 and channel 5 where tested at different sets of distances, error in CHANNEL vars check
if CHANNEL == 9:
    distances = [100,200,300]
elif CHANNEL == 5:
    distances = [150,250]
else:
    print('you have to choose one of the supported channel: please change the variable CHANNEL = 9 or 5')
    exit()
    
# check the distance selected for the distribution plot
if len(np.where(np.array(distances) == WANT_DIST)[0]) == 0:
    print('the chosen distance is not present among the one of the selected channel. please change the WANT_DIST value accordingly, or try another CHANNEL ')
    exit()

# PATHS for load data and save graphs
THIS_PATH = os.getcwd()+'/UWB caracterization/misure_CH'+str(CHANNEL)
SAVE_PATH = THIS_PATH + '/Graphs/'
LOAD_PATH_AOA = THIS_PATH + '/Aoas/'
LOAD_PATH_RANGE = THIS_PATH + '/Ranges/'
comname = 'RANGE_'

# Create SAVE_PATH if not already present
if not(os.path.isdir(SAVE_PATH)):
    os.makedirs(SAVE_PATH)
    
for used_range in distances:
    
    # load path for the specific distance = used_range
    aoa_fpt = LOAD_PATH_AOA+comname+str(used_range)+'/'
    range_fpt = LOAD_PATH_RANGE+comname+str(used_range)+'/'

    # list all the .csv data dile in the directories
    aoa_listdir = sorted(os.listdir(aoa_fpt))
    range_listdir = sorted(os.listdir(range_fpt))

    # initialize void lists for the path of each .csv file
    aoa_file_list, range_file_list = [], []

    # create a list of all the .csv full paths
    aoa_file_list.extend([aoa_fpt+i for i in aoa_listdir])
    range_file_list.extend([range_fpt+i for i in range_listdir])
    
    # initialize void lists for datas
    all_array_aoa, all_array_range = [], []
    aoa_list = []

    # load all the data in two list of arrays (range & aoa). aoa_list, list the order with which the data are loaded w.r.t. the aoa
    for f in aoa_file_list:
        all_array_aoa.append(np.genfromtxt(f, delimiter=','))
        aoa_list.append(int(f[-7:-4]))

    for f in range_file_list:
        all_array_range.append(np.genfromtxt(f, delimiter=','))
    
    
    # PLOT of the selected distribution
    if WANT_DIST == used_range:
        plot_distributions(WANT_AOA, WANT_DIST, aoa_list, all_array_aoa, all_array_range)
        
    # Plot the data cloud for each angle at the current used_range, looping in the loaded data.
    loop = enumerate(aoa_list)

    plt.figure(3,figsize=(16,6))
    for i, aoa in loop:
        coords = all_array_range[i]*np.array([np.sin(all_array_aoa[i]*np.pi/180), np.cos(all_array_aoa[i]*np.pi/180)])
        plt.scatter(coords[0],coords[1], marker='+', c=color_list[int(abs(aoa/15))]) #facecolors='none', edgecolors=color_list[int(abs(aoa/15))])
        real_coord = used_range*np.array([np.sin(aoa*np.pi/180),np.cos(aoa*np.pi/180)])
        plt.scatter(real_coord[0], real_coord[1], marker='s',c=color_list[int(abs(aoa/15))])
        plt.plot([0,real_coord[0]], [0,real_coord[1]], c=color_list[int(abs(aoa/15))])
    x,y = generate_semicircle(0,0,used_range, 0.1)
    plt.plot(x, y, '--', c='k')
plt.plot(0,0,'-ko')
plt.savefig(SAVE_PATH+'ch'+str(CHANNEL)+'_characterization.png',dpi=300)

plt.show()
plt.close()