# coding=utf-8
#!/usr/bin/env python



import numpy as np
from scipy import signal
from scipy.signal import convolve2d
import matplotlib.pyplot as plt
import timeit
import time
import gdm_pack.ipp_pathplanner_v1 as gdm
import random



if __name__ == '__main__':

    resultsarray = [] # To store results. Only used for debugging and analysis.
    for j in range(1): # Number of iterations for analysis

        # Tunable Parameters
        TotalValue = 0
        curr_cell = gdm.coord(10, 10) # Starting Cell.
        lvl_sizes = np.array([5, 2]) # Subcells and number of levels. E.g for grid size 20x20 (/5)-> 4x4 (/2)-> 2x2
        map_size = 20 # Size of the squared grid eg 20x20
        B = 8 #Budget
        drawing = True

        # Parameters for ground truth simulation
        kx = random.uniform(-1.5,2.5)*0.05#0.7 #0.32
        ky = random.uniform(-1.5, 2.2)*0.05
        sx = random.uniform(-1,2)#1.3
        sy = random.uniform(-1,2)#0

        # Simulating a ground truth
        x = np.repeat([np.arange(0,map_size),],map_size,axis=0).T*kx+sx
        y = np.repeat([np.arange(0,map_size),],map_size,axis=0)*ky+sy

        z = np.sin(5*x)*np.cos(5*y)*x/2+1
        # z = np.sin(x**2)+np.cos(y**2)
        # z = np.sin(10*(x**2 + y**2)) / 10
        # z = np.random.random((20, 20))
        utility_map = z

        # Drawings

        if drawing == True:
            fig, axis = plt.subplots()
            plt.axis('off')
            plt.imshow(z.T,interpolation='none', origin='lower')
            # plt.colorbar(ticks=[])
            plt.pause(0.0000000001)

            axes= []
            axes.append(axis)

        # Initialising stuff

            # Variables to keep track of planned paths and history.
        max_path_array = []
        curr_cell_array= []
        hist_array = []

        lista = np.array([[[curr_cell.x, curr_cell.y]]])
        n_lvl = lvl_sizes.size
        maps = []


        # Plotting the maps!
        if drawing == True:
            i = 0
            # For Drawings. Create the maps for each level.
            maps.append(utility_map)  # The first level does not need convolution
            for lvl in range(1, n_lvl + 1):  # Go through every level upwards

                window = np.full((lvl_sizes[lvl - 1], lvl_sizes[lvl - 1]), 1. / (lvl_sizes[lvl - 1] * lvl_sizes[lvl - 1]))
                maps.append(convolve2d(maps[lvl - 1], window, boundary='fill', mode='valid')[::lvl_sizes[lvl - 1],
                            ::lvl_sizes[lvl - 1]])

                # Drawing
                if i == 0:
                    fig, axis = plt.subplots()
                    axes.append(axis)
                    plt.axis('off')
                    plt.imshow(maps[lvl].T, interpolation='none', origin='lower')


                    plt.pause(0.0000000001)

        # raw_input(" Press Enter to finish")





        # THE ACTUAL ACLOP IMPLEMENTATION


        aclopdata = []
        #Init stuff
        visited = np.zeros([map_size, map_size])
        TotalValue = 0
        max_path_array = []
        curr_cell_array = []
        hist_array = []
        curr_cell = gdm.coord(10, 10)

        lista = np.array([[[curr_cell.x, curr_cell.y]]])
        n_lvl = lvl_sizes.size
        maps = []


        start = time.time() # For tracking execution time only

        # The actual cycle.
        for i in range(0,B,1):

            #Planning
            next_cell = gdm.stack_planner(z, curr_cell, lvl_sizes, B=B, visited=visited)
            # visited[curr_cell.x, curr_cell.y] -= 1 #Uncomment if want to mark starting cell as visited. BF does not.

            curr_cell, [maps, curr_cell_array, max_path_array] = next_cell
            visited[curr_cell.x, curr_cell.y] -= 1 # Mark the cell as visited. The path planner will not visit this cell again (In any of the further executions of ACLOP this run).  # Uncomment if you want to allow revisiting this cell in the next for loop iteration.
            TotalValue += z[curr_cell.x, curr_cell.y]

            print i, TotalValue
            #raw_input(" Press Enter to finish")




            # Drawing
            if drawing == True:

                for lvl in range(0, len(axes)):

                    if i == 0:
                        hist_array.append([[curr_cell_array[lvl].x, curr_cell_array[lvl].y]])
                    else:
                        hist_array[lvl].append([curr_cell_array[lvl].x, curr_cell_array[lvl].y])

                    plt.axis('off')
                    axes[lvl].clear()
                    axes[lvl].imshow(maps[lvl].T, interpolation='none', origin='lower')
                    axes[lvl].scatter([row[0] for row in hist_array[lvl]], [row[1] for row in hist_array[lvl]], marker='+')
                    max_path_x = gdm.getList2Array(max_path_array[lvl])[:, 0] if (gdm.getList2Array(max_path_array[lvl]) != []) else []
                    max_path_y = gdm.getList2Array(max_path_array[lvl])[:, 1] if (gdm.getList2Array(max_path_array[lvl]) != []) else []
                    a = 10
                    axes[lvl].scatter(max_path_x, max_path_y, s=[10 * a, 7 * a, 5 * a, 3 * a, 2 * a, 1 * a], marker='o', c='r')
                    axes[lvl].scatter(curr_cell_array[lvl].x, curr_cell_array[lvl].y, s=5 * 15, c='lime', marker='*')
                    # axes[lvl].set_xlim(0,map_size-1)
                    # axes[lvl].set_ylim(0, map_size-1)
                    plt.pause(0.00000001)
            else:
                for lvl in range(0, lvl_sizes.size+1):

                    if i == 0:
                        hist_array.append([[curr_cell_array[lvl].x, curr_cell_array[lvl].y]])
                    else:
                        hist_array[lvl].append([curr_cell_array[lvl].x, curr_cell_array[lvl].y])

            B -= 1
            # raw_input(" Press Enter to finish")

        totaltime = time.time()- start

        aclopdata.append([i+1, TotalValue, hist_array, totaltime])


    results = [z, aclopdata]
    resultsarray.append(results)


    from datetime import datetime
    path = '/home/omar/PycharmProjects/gdm/src/gdm_pack/Results/npresults/resultsarray/structured54' + datetime.now().strftime(
            '%Y-%m-%d %H:%M:%S')

    # Uncomment if the results need to be saved. Check the path!
    # np.save(path, resultsarray)
    # print results

    raw_input(" Press Enter to finish")