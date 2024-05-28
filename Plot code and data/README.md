# PLOT EXTRAS

This folder contain extra code sources, used to generate some of the report graphs. Not all the graphs are generated with the files in this folder, some are generated directly in the ROS2 nodes of the simulation.

Inside this folder all the collected data, are organized in folders and subfolders. The name of the folders, must not be changed, in order for the codes to execute. The code if execute generates what follows:
  - `Characterization_plot.py` generate the working half-plane characterization, for the selected channel (to be chosen by changing the `CHANNEL` variable). Moreover, by selecting a proper range and angle (setting `WANT_DIST` and `WANT_AOA' variable), it generate the angle and range measurement distributions. The used data are loaded from the `UWB characterization` folder;
  - `behind_plot.py` generate the graph that compare the working region (the front half-plane) with the behind. This is possible at a fixed distance and for only the channel 9. The used data are loaded from the `UWB characterization` folder;
  - `height_plot.py` generate the graph that compare the UWB sensor performance w.r.t. a tag height change. Also in this case, distance and channel are not changable. The used data are loaded from the `UWB characterization` folder;
  - `real_data_plot.py` generate the graphs of real test. The used data are loaded from the `real_data` folder

To run the codes, open a terminale in the `Plot code and data` directory and run them using `python3` as follow:
```
python3 file_name.py
```
