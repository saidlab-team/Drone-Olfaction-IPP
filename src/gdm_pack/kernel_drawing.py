# coding=utf-8
# Kernel library

from td_kernel_dmvw import TDKernelDMVW
# Drawing Libraries
import matplotlib.pyplot as plt  # In case plot update is too slow, change to pyqtgraph
# plt.style.use('seaborn-white')


# Others
import numpy as np
# from timeit import default_timer as timer, Timer


##############################
#     Start the Kernel       #
##############################

def start_kernel(min_x = 0., min_y = 0., max_x = 20., max_y = 20., cell_size = 0.1, kernel_size = 1., wind_scale = 0.05, time_scale = 0,
                lccz = True, evaluation_radius = 10):

    # Set starting parameters
    # min_x = 0
    # min_y = 0
    # max_x = 20
    # max_y = 20
    # cell_size = 0.1
    # kernel_size = 10 * cell_size
    # wind_scale = 0.05
    # time_scale = 0
    # evaluation_radius = 10*kernel_size

    # Create dummy measurement vectors
    positions_x = []
    positions_y = []
    concentrations = []
    wind_directions = []
    wind_speeds = []
    timestamps = []

    # call Kernel
    kernel = TDKernelDMVW(min_x, min_y, max_x, max_y, cell_size, kernel_size, wind_scale, time_scale,
                          low_confidence_calculation_zero=lccz, evaluation_radius=evaluation_radius)
    kernel.set_measurements(positions_x, positions_y, concentrations, wind_speeds, wind_directions, timestamps)
    # kernel.calculate_maps()

    return kernel


#######################
# Update the Kernel
#######################

def update_kernel(kernel, new_x, new_y, new_conc, new_wspeed, new_wdir, new_timestamp):

    # Push the new measurements and recalculate the maps
    kernel.measurement_positions_x.append(new_x)
    kernel.measurement_positions_y.append(new_y)
    kernel.measurement_concentrations.append(new_conc)
    kernel.measurement_wind_speeds.append(new_wspeed)
    kernel.measurement_wind_directions.append(new_wdir)
    kernel.measurement_timestamps.append(new_timestamp)
    #kernel.calculate_maps()


#######################
# Plot the initial map
#######################

def init_map():

    # Initiate interactive mode
    plt.ion()

    # Create Figure and axes
    fig, axarray = plt.subplots()

    axarray.clear()
    axarray.set_title("Map")
    cs = axarray.contourf(np.zeros((2, 2)), np.zeros((2, 2)), np.zeros((2, 2)))
    fig.colorbar(cs)

    # Now that the colorbar has its own axes we can get the new axis list and return it.
    axarray = fig.get_axes()

    return axarray

#######################
# Update the map
#######################


def update_map(axarray, kernel, map_type):

    # Clear the current axes and colorbar. Get the pointers first to redraw them.
    # markers=axarray[0].get_lines()
    axarray[0].clear()

    # Redraw the plot
    x = kernel.cell_grid_x
    y = kernel.cell_grid_y
    if map_type == 'mean':
        cs = axarray[0].contourf(x, y, kernel.mean_map)
        axarray[0].set_title("Mean map")
    elif map_type == 'confidence':
        cs = axarray[0].contourf(x, y, kernel.confidence_map)
        axarray[0].set_title("Confidence map")
    elif map_type == 'variance':
        cs = axarray[0].contourf(x, y, kernel.variance_map)
        axarray[0].set_title("Variance map")
    else:
        raise NameError('Map type for update_map() is incorrect')

    # Redraw the markers
    # for marker in markers:
    #     axarray[0].add_line(marker)

    # Redraw the colorbar and add the new marker
    plt.colorbar(cs, cax=axarray[1])
    axarray[0].scatter(kernel.measurement_positions_x, kernel.measurement_positions_y, marker='+')



    # plt.ion()
    # plt.draw()
    # plt.show()
    plt.pause(0.00001)



#######################
# Main
#######################

def main():

    """
        gdm_pack:   Kernel object from the Kernel DM module
        axarray:    matplotlib array containing the axis of the figure

    """



    kernel = start_kernel(min_x = 0, min_y = 0, max_x = 10-0.1, max_y = 11-0.1, cell_size = 0.1, kernel_size = 0.1,
                          wind_scale=0.05, time_scale=0,lccz=True, evaluation_radius=1)



    ## Draw Ground Truth

    path = '/home/omar/PycharmProjects/gdm/src/gdm_pack/include'
    Conc = np.load(path + '/Conc45_37.npy')
    plt.imshow(Conc)

    fig, axarray = plt.subplots()

    axarray.clear()
    axarray.set_title("Map")
    cs = axarray.contourf(kernel.cell_grid_x, kernel.cell_grid_y, Conc)
    fig.colorbar(cs)

    # plt.draw()
    # plt.show()
    plt.pause(0.00001)


    # Draw the initial plot and save the axes for each figure
    mean_array = init_map()
    confidence_array = init_map()
    variance_array = init_map()



    #raw_input("Press [enter] to continue.")



    for i in range(0, 99):
        for j in range(0, 99):

            # Update the Kernel with the new values

            update_kernel(kernel, i*0.1 , j*0.1, Conc[j][i], 1, 0, j+i*110 )

            # Redraw the map with the new values
            #update_map(mean_array, kernel, 'mean')
            print(j)
            #raw_input("Press [enter] to continue.")

    kernel.calculate_maps()
    update_map(mean_array, kernel, 'mean')
    update_map(confidence_array, kernel, 'confidence')
    update_map(variance_array, kernel, 'variance')

    # Finishing up
    raw_input("Press [enter] to finish.")


if __name__ == '__main__':
    main()
