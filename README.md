README

Some Random Notes:

	For autonomous control of the DJI M100 control authority has to be requested by the ground station through the apropriate ROS service. You will see references to this in the code. Sometimes this doesn't want to behave so a reset of the drone, the ROS DJI SDK or the ROS Node requesting the service is needed. One of those points in the communication chain acts up.


	You will see references to a plot_clock, it is just a device used to be able to set the refresh rate of the ROS callback functions that updates the plots.






			Scripts folder


- pathplanner_DJI (ROS node)

	ROS node that comunicates and sets up remote control from ROS with the DJI SDK.
	It sets up the local refrence frame and makes the drone take off. 
	After that, it reads from the topic "/pathplanner_cmd" and sends the drone to the commanded position through that topic.
	A waypoint mode (instead of positioning wrt local reference frame) is coded but not setup completly.


- ipp_pathplanner (ROS node)

	This note implements ACLOP (outdated version) and sends the commands to the pathplanner_DJI node. However the implementation of ACLOP used in this file is outdated and does not include budget constrains nor utility map construction correctly.

	This node implements the IPP pathplanner. It includes the kernel update, the kernel drawing functions and the grid pathplanning algorithm.


- aclop_ipp

	This code implements the latest version of ACLOP with budget constrains but it does so on a predetermined (parametrized) groundtruth and does not include any of the processing ROS workflow needed to implement it either in simulation nor the real setting.

	This would need to be merged with the "ipp_pathplanner" to test the algorithm in a simulation and real setting.

	Plots can be activated or deactivated via a local variable in the code "drawing" by setting it to True or False.




- send_commands_dji (ROS Node)

	Small cmd interface node that enables control of the drone through cmd line. It allows authority requests and local setpoints and reference frames. 

- simulate_gaden_readings (ROS Node)

	Simulates readings from GADEN by reading the GADEN output file and publishing the gas readings at location x,y (input) in a ROS Topic.





			src/gdm_pack folder

This folder includes libraries and other accesory functions needed for the execution of the nodes.

- matrix_operations and td_kernel_dmvw

 	Kernel code from the GitHub repository. This implements the kernel. The code has been modified to fix some small bugs. (Original code: https://github.com/smueller18/TDKernelDMVW)


- kernel_drawing

 	Implements some function for quick drawing of kernel maps.


- ipp_pathplanner_v1.py

  	Implements the pathplanning algorithm ACLOP. Including the budget constrains.






			Launch folder

- field_test.launch
	
	Launch file for the field test that calls the send_commands_dji (ROS Node)

- gaden_simulation_demo.launch
	
	Launch file that launches a simulation on RVIZ with GADEN visualization. Readings are simulated and read from the GADEN Output file corresponding to the visualization.

- ipp_test.launch

	This launch file is the one used on the real experiments (Launches the ipp_pathplanner node with the outdated ACLOP version).





			Snipets folder


This folder includes all nodes and other snippets of code.

- create_gt_model (Python script)
	
	Create a grid ground truth model from one of the output files from GADEN simulations. It saves the result (in gdm_pack/include) in three numpy array file for the concentration, wind and wind direction. Units are ppm, m/s and degrees by default.

- create_gt_model_joao(_1) (Python script)

	Draft, quick and dirty script to obtain plots from the experiments made with Imke last year.


- path_maker.py (Python script)

	Quick script to get the GPS coordinates of a spiral path. Inputs are gps coordinates of the origin, altitude/s, radius, step and decimation.


# License

Any work done using this code should reference the paper [An Adaptive Informative Path Planning Algorithm for Real-time Air Quality Monitoring Using UAVs](https://ieeexplore.ieee.org/document/9214013)

```

@INPROCEEDINGS{9214013,
  author={O. {Velasco} and J. {Valente} and A. Y. {Mersha}},
  booktitle={2020 International Conference on Unmanned Aircraft Systems (ICUAS)}, 
  title={An Adaptive Informative Path Planning Algorithm for Real-time Air Quality Monitoring Using UAVs}, 
  year={2020},
  volume={},
  number={},
  pages={1121-1130},
  doi={10.1109/ICUAS48674.2020.9214013}}

```
