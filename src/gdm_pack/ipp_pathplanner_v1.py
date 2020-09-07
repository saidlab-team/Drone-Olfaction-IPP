# coding=utf-8
#!/usr/bin/env python



import numpy as np
from scipy import signal
from scipy.signal import convolve2d
import matplotlib.pyplot as plt
import timeit



    ### Classes ###

# Coordinate class to help with the matrix coordinate movements and cell operations
class coord:

    def __init__(self,x ,y):
        self.x = x
        self.y = y

    # Gives a new coordinate position (x+dx, y+dy) for a cell given dx and dy
    def moveCoord(self, cell):
        return coord( self.x + cell.x, self.y + cell.y)

    def getArray(self):
        aux = np.array([self.x, self.y])
        return aux

# IPP Planner Instance
class planner:

    def __init__(self, matrix, directions = None):

        # Utility matrix
        self.matrix = np.array(matrix, copy=True)

        # Marks cell locations as visited and how many times it has been visited
        self.visited = np.zeros(matrix.shape)


        # Available directions for the planner.
        if directions is None:
            self.directions = np.array([coord( 0,  0),
                                        coord(-1, -1), coord(-1, 0), coord(-1, 1),
                                        coord( 0, -1)              , coord( 0, 1),
                                        coord( 1, -1), coord( 1, 0), coord( 1, 1)])
        else:
            self.directions = directions.copy()


    ## Methods

    def update_matrix(self, matrix):

        # Utility matrix
        self.matrix = np.array(matrix, copy=True)

        # Marks cell locations as visited and how many times it has been visited
        self.visited = np.zeros(matrix.shape)

    def update_visited(self, visited):
        if visited is not None:
            self.visited = np.array(visited, copy=True)


    def update_directions(self, directions):
        self.directions = directions.copy()

    def isLegalPosition(self, cell):

        # Check limits of the matrix
        if ( cell.x < 0 or cell.x >= self.matrix.shape[0]):
            return False

        # Check limits of the matrix
        if ( cell.y < 0 or cell.y >= self.matrix.shape[1]):
            return False

        # Check if the matrix has been visited once (-1) or twice (-2) already.
        return self.visited[cell.x][cell.y] > -1

    # Implementation of the brute force algorithm. Based on recurssion.
    def findMax_kConstrained(self, k, currPosition, currDepth = 0, currSum = 0, currPath_ = None):

        # End the recursion if we have reached depth k
        if currDepth == k:
            return currSum, currPath_

        # Find the greatest number in the available directions. Current max value is initialized ot None.

        currMax = None
        tempPath = []
        if currPath_ is None:
            currPath = []
        else:
            currPath=list(currPath_)


        for cell in self.directions:

            # Get the new cell that will be evaluated
            newCell = currPosition.moveCoord(cell)

            # Check if the cell is legal. If it is then explore it adding another depth level.
            if self.isLegalPosition(newCell):

                # mark the cell as visited
                utility = self.matrix[newCell.x, newCell.y]
                self.visited[newCell.x, newCell.y] -= 1


                currPath.append(newCell)
                newMax, newPath = self.findMax_kConstrained(k, newCell, currDepth + 1, currSum + utility, currPath)

                # Restore the visited map for upper level of recursion
                self.visited[newCell.x, newCell.y] += 1

                # Calculate the new max. If this is the first time we pass trough here use newMax.
                # If is not, actualize the value currMax if it is smaller than newMax.


                if newMax != None:
                    if currMax == None:
                        currMax = newMax
                        tempPath = list(newPath)

                    elif newMax > currMax:
                        currMax = newMax
                        tempPath = list(newPath)

                    elif newMax == currMax:
                        for newCell, tempCell in zip(newPath,tempPath):

                            newValue = self.matrix[newCell.x, newCell.y]
                            tempValue = self.matrix[tempCell.x, tempCell.y]

                            if newValue <> tempValue:
                                if newValue > tempValue:
                                    currMax = newMax
                                    tempPath = list(newPath)

                                break



                    else:
                        currMax = currMax
                        # tempPath = tempPath

                currPath.pop()



        return  currMax, tempPath


    ### Aux Functions ###

def getList2Array(list):

    if list == []:
        return []

    aux = None
    if list is not None:

        for item in list:
            if aux is None:
                aux = np.array([item.getArray()])
            else:
                aux = np.append(aux, [item.getArray()], axis=0)


    return aux

# Returns the correct planner which is a function of the relative position of the two input cells: next cell(nx, ny) and current cell (cx, cy). The cells have to be at the same map level.
def select_planner(nx, ny, cx, cy, without_zero = False):

    if without_zero:

        switch = {}
        switch[(-1,-1)] = planner_1
        switch[( 0,-1)] = planner_2
        switch[( 1,-1)] = planner_3
        switch[(-1, 0)] = planner_4
        switch[( 0, 0)] = planner_5
        switch[( 1, 0)] = planner_6
        switch[(-1, 1)] = planner_7
        switch[( 0, 1)] = planner_8
        switch[( 1, 1)] = planner_9

        # TODO : Clean this up, get the returns and switchs out of the ifs. Get the switchs as global variables.
        # print 'planner ' + str((nx - cx, ny - cy))

        if (nx - cx, ny - cy) == (0, 0):
            planner_is_5 = False
        else:
            planner_is_5 = True

        return switch[(nx - cx, ny - cy)], planner_is_5

    else:

        switch = {}
        switch[(-1, -1)] = planner_10
        switch[(0, -1)] = planner_20
        switch[(1, -1)] = planner_30
        switch[(-1, 0)] = planner_40
        switch[(0, 0)] = planner_50
        switch[(1, 0)] = planner_60
        switch[(-1, 1)] = planner_70
        switch[(0, 1)] = planner_80
        switch[(1, 1)] = planner_90

        # print 'planner ' + str((nx - cx, ny - cy))

        if (nx - cx, ny - cy) == (0, 0):
            planner_is_5 = False
        else:
            planner_is_5 = True


        return switch[(nx - cx, ny - cy)], planner_is_5

def get_cell_index(cx, cy, lvl, lvl_size):

    nx = int(cx / lvl_size[:lvl].prod())
    ny = int(cy / lvl_size[:lvl].prod())
    cell = coord(nx, ny)

    return cell


    ### Constants ###

# coord(0, 0) should always  be at the start of the array. atm
coord_1 = np.array([coord(0, 0), coord(-1, -1), coord(-1,  0), coord( 0, -1)])
coord_2 = np.array([coord(0, 0), coord(-1, -1), coord( 0, -1), coord( 1, -1)])
coord_3 = np.array([coord(0, 0), coord( 0, -1), coord( 1, -1), coord( 1,  0)])
coord_4 = np.array([coord(0, 0), coord(-1, -1), coord(-1,  0), coord(-1,  1)])
coord_5 = np.array([coord(0, 0), coord(-1, -1), coord(-1,  0), coord(-1,  1), coord( 0, -1),
                    coord(0, 1), coord( 1, -1), coord( 1,  0), coord( 1,  1)])
coord_6 = np.array([coord(0, 0), coord( 1, -1), coord( 1,  0), coord( 1,  1)])
coord_7 = np.array([coord(0, 0), coord(-1,  0), coord(-1,  1), coord( 0,  1)])
coord_8 = np.array([coord(0, 0), coord(-1,  1), coord( 0,  1), coord( 1,  1)])
coord_9 = np.array([coord(0, 0), coord( 0,  1), coord( 1,  0), coord( 1,  1)])

z = np.empty(1)

planner_10 = planner(z, coord_1)
planner_20 = planner(z, coord_2)
planner_30 = planner(z, coord_3)
planner_40 = planner(z, coord_4)
planner_50= planner(z)
planner_60 = planner(z, coord_6)
planner_70 = planner(z, coord_7)
planner_80 = planner(z, coord_8)
planner_90 = planner(z, coord_9)

planner_1 = planner(z, np.delete(coord_1,0))
planner_2 = planner(z, np.delete(coord_2,0))
planner_3 = planner(z, np.delete(coord_3,0))
planner_4 = planner(z, np.delete(coord_4,0))
planner_5 = planner(z, np.delete(coord_5,0))
planner_6 = planner(z, np.delete(coord_6,0))
planner_7 = planner(z, np.delete(coord_7,0))
planner_8 = planner(z, np.delete(coord_8,0))
planner_9 = planner(z, np.delete(coord_9,0))

def stack_planner(utility_map, curr_cell, lvl_sizes, B = 50, visited = None):
    '''

    :param utility_map: Utility map used for the mission planning.
    :param curr_cell: Current cell position, as a coord object.
    :param lvl_sizes: Level decimation size, as a np.array
    :return: next cell position, as a coord object.
    '''

    # Initialising stuff

    n_lvl = lvl_sizes.size
    maps = []
    max_path_array = []
    curr_cell_array = []
    hist_array = []


    # Create the maps for each level.
    maps.append(utility_map)  # The first level does not need convolution
    for lvl in range(1, n_lvl + 1):  # Go through every level upwards

        window = np.full((lvl_sizes[lvl - 1], lvl_sizes[lvl - 1]), 1. / (lvl_sizes[lvl - 1] * lvl_sizes[lvl - 1]))
        maps.append(convolve2d(maps[lvl - 1], window, boundary='fill', mode='valid')[::lvl_sizes[lvl - 1],
                    ::lvl_sizes[lvl - 1]])

        # Drawing
        # plt.subplots()
        # plt.imshow(maps[lvl].T, interpolation='none', origin='lower')
        # plt.pause(0.0000000001)

    # The UPPERMOST level always uses the same planner:
    lvl = lvl_sizes.size
    planner_lvl = planner(maps[-1])
    curr_cell_lvl = get_cell_index(curr_cell.x, curr_cell.y, lvl, lvl_sizes)
    max_utility, max_path = planner_lvl.findMax_kConstrained(max(maps[-1].shape), curr_cell_lvl)
    next_cell = max_path[0]

    # Only consider staying in the same cell if the upper level planned to stay in the upper cell
    if (next_cell.x - curr_cell_lvl.x, next_cell.y - curr_cell_lvl.y) == (0, 0):
        without_zero = False
    else:
        without_zero = True

    # Keep track of the planned paths and positions for each lvl
    max_path_array.insert(0, max_path)
    curr_cell_array.insert(0, curr_cell_lvl)

    # print 'lvl 2: ' + str(next_cell.getArray())

    for lvl in range(n_lvl - 1, -1, -1):  # Go through every level downwards, skip the uppermost level.

        # If this is the last level, do not include the same cell in the planner.
        if lvl == 0:
            without_zero = True

        # Select the next planner based on the above level, then update the current cell to current lvl.
        planner_lvl, without_zero = select_planner(next_cell.x, next_cell.y, curr_cell_lvl.x, curr_cell_lvl.y,
                                                   without_zero)
        curr_cell_lvl = get_cell_index(curr_cell.x, curr_cell.y, lvl, lvl_sizes)
        # print 'current cell lvl' + str(curr_cell_lvl.getArray())
        # Update the planner with the current map.
        planner_lvl.update_matrix(maps[lvl])

        if lvl == 0 and visited is not None:
            planner_lvl.update_visited(visited)

        # Calculate next cell.
        max_utility, max_path = planner_lvl.findMax_kConstrained(lvl_sizes[lvl],
                                                                 curr_cell_lvl)  # k is equal to the size of the level above
        #in case there are no legal moves, plan in every direction:
        if max_utility is None:
            planner_50.update_matrix(maps[lvl])
            planner_50.update_visited(visited)
            max_utility, max_path = planner_50.findMax_kConstrained(lvl_sizes[lvl], curr_cell_lvl)

        next_cell = max_path[0]

        # Keep track of the planned paths and positions for each lvl
        max_path_array.insert(0, max_path)
        curr_cell_array.insert(0, curr_cell_lvl)

        # print 'lvl ' + str(lvl) + ': ' + str(next_cell.getArray())

        # Drawing
        # lista = np.append(lista, [[next_cell.x, next_cell.y]], axis=0)
        # axis.imshow(z.T, interpolation='none', origin='lower')
        # axis.scatter(lista[:, 0], lista[:, 1], marker='+')
        # plt.pause(0.00000001)

    weights = CalculateWeights(lvl_sizes)
    costofpath = CostOfPath(weights = weights,lvl_sizes=lvl_sizes, k_vector=lvl_sizes, total = True)
    if costofpath > B:
        next_cell, [maps, curr_cell_array, max_path_array] = CalculateBudget(B, maps, curr_cell_array, max_path_array, utility_map, curr_cell, lvl_sizes, weights,visited)


    return next_cell, [maps, curr_cell_array, max_path_array]


    ### BUDGET CONSTRAINS CODE ###

def CostOfPath(weights, lvl_sizes, k_vector, total = False):

    costofpath = 0
    for i in range(0,k_vector.size): # It is sum of from i=0 to L-1
        costofpath += weights[i]*k_vector[i]

    if total is True: costofpath += weights[lvl_sizes.size]

    return costofpath


def CalculateWeights(lvl_sizes):
    #Calculate weights
    w0 = 1
    weights = [w0]
    for i in range(1,lvl_sizes.size+1):
        weights.append(w0*reduce(lambda x,y: x*y, lvl_sizes[0:i]))

    return weights


def CalculateBudget(B, maps, curr_cell_array, max_path_array, utility_map, curr_cell, lvl_sizes, weights, visited):

    k_vector = np.array(lvl_sizes)

    # Calculate k_vector
    while True:

        costofpath = CostOfPath(weights, lvl_sizes, k_vector, total = False)
        # print costofpath,k_vector
        if costofpath <= B:
            break

        k_vector[-1] = k_vector[-1] - 1
        if k_vector[-1] == 0:
            k_vector = k_vector[:-1]
            if k_vector.size == 0:
                break

        if k_vector.size == 0:
            break


    # Once calculated:

    if k_vector.size == 0:
        max_path_array = [ [] for i in range(len(max_path_array)) ]
        return curr_cell, [maps, curr_cell_array, max_path_array]

    # If first level. plan directly
    if k_vector.size == 1:
        planner_lvl0 = planner(utility_map, coord_5)
        planner_lvl0.update_visited(visited)
        max_utility, max_path = planner_lvl0.findMax_kConstrained(k_vector[0], curr_cell)

        next_cell = max_path[0]
        i = 0
        for path in max_path_array:
            if i == 0 :
                max_path_array[i] = list(max_path)
            else:
                max_path_array[i] = list()
            i += 1

        return next_cell, [maps, curr_cell_array, max_path_array]

    # If not first level use stack planner:
    next_cell, max_path_array_budget = smallstack_planner(curr_cell_array, k_vector, maps, visited)

    i = 0
    for path in max_path_array:
        if i < len(max_path_array_budget):
            max_path_array[i]= list(max_path_array_budget[i])
        else:
            max_path_array[i] = list()
        i += 1


    return next_cell, [maps, curr_cell_array, max_path_array]


# Instance of the Brute Force Algorithm but adapted for lower levels (smaller stacks) after the budget calculations.
def smallstack_planner(curr_cell_array, k_vector, maps, visited = None):
    '''

    :param utility_map: Utility map used for the mission planning.
    :param curr_cell: Current cell position, as a coord object.
    :param lvl_sizes: Level decimation size, as a np.array
    :return: next cell position, as a coord object.
    '''

    # Initialising stuff

    n_lvl = k_vector.size
    max_path_array_budget = []

    # The UPPERMOST level always uses the same planner:
    lvl = k_vector.size-1
    planner_lvl = planner(maps[lvl])
    curr_cell_lvl = curr_cell_array[lvl]
    max_utility, max_path = planner_lvl.findMax_kConstrained(k_vector[lvl], curr_cell_lvl)
    next_cell = max_path[0]

    # Only consider staying in the same cell if the upper level planned to stay in the upper cell
    if (next_cell.x - curr_cell_lvl.x, next_cell.y - curr_cell_lvl.y) == (0, 0):
        without_zero = False
    else:
        without_zero = True

    # Keep track of the planned paths and positions for each lvl
    max_path_array_budget.insert(0, max_path)

    # Start planning for the below levels
    for lvl in range(n_lvl - 2, -1, -1):  # Go through every level downwards, skip the uppermost level.

        # If this is the last level, do not include the same cell in the planner.
        if lvl == 0:
            without_zero = True

        # Select the next planner based on the above level, then update the current cell to current lvl.
        planner_lvl, without_zero = select_planner(next_cell.x, next_cell.y, curr_cell_lvl.x, curr_cell_lvl.y,
                                                   without_zero)
        curr_cell_lvl = curr_cell_array[lvl]
        # Update the planner with the current map.
        planner_lvl.update_matrix(maps[lvl])

        if lvl == 0 and visited is not None:
            planner_lvl.update_visited(visited)

        # Calculate next cell.
        max_utility, max_path = planner_lvl.findMax_kConstrained(k_vector[lvl],
                                                                 curr_cell_lvl)  # k is equal to the size of the level above
        #In case there is no possible solution.
        # in case there are no legal moves, plan in every direction:
        if max_utility is None:
            planner_50.update_matrix(maps[lvl])
            planner_50.update_visited(visited)
            max_utility, max_path = planner_50.findMax_kConstrained(k_vector[lvl], curr_cell_lvl)
        next_cell = max_path[0]

        # Keep track of the planned paths and positions for each lvl
        max_path_array_budget.insert(0, max_path)


    return next_cell, max_path_array_budget





# This main function is just for  testing the functionallity of the above functions!
if __name__ == '__main__':

    # Simulating a ground truth

    x = np.repeat([np.arange(0,40),],40,axis=0).T*0.07
    y = np.repeat([np.arange(0,40),],40,axis=0)*0.05

    z = np.sin(5*x)*np.cos(5*y)*x/2
    # z = np.sin(x**2)+np.cos(y**2)
    # z = np.sin(10*(x**2 + y**2)) / 10



    fig, axis = plt.subplots()
    plt.imshow(z.T,interpolation='none', origin='lower')
    plt.pause(0.0000000001)

    # Drawings
    axes= []
    axes.append(axis)

    # Variables to keep track of planned paths and history.
    max_path_array = []
    curr_cell_array= []
    hist_array = []



    # Initialising stuff

    curr_cell = coord(30, 30)
    lvl_sizes = np.array([5, 4])

    lista = np.array([[[curr_cell.x, curr_cell.y]]])
    n_lvl = lvl_sizes.size
    maps = []

    utility_map = z



    i = 0
    while True:

        print 'iteration ' + str(i)
        print 'current location' + str(curr_cell.getArray())




        # Create the maps for each level.
        maps.append(utility_map)  # The first level does not need convolution
        for lvl in range(1, n_lvl + 1):  # Go through every level upwards

            window = np.full((lvl_sizes[lvl - 1], lvl_sizes[lvl - 1]), 1. / (lvl_sizes[lvl - 1] * lvl_sizes[lvl - 1]))
            maps.append(convolve2d(maps[lvl - 1], window, boundary='fill', mode='valid')[::lvl_sizes[lvl - 1],
                        ::lvl_sizes[lvl - 1]])

            # Drawing
            if i == 0:
                fig, axis = plt.subplots()
                axes.append(axis)
                plt.imshow(maps[lvl].T, interpolation='none', origin='lower')
                plt.pause(0.0000000001)


        # The UPPERMOST level always uses the same planner:
        lvl = lvl_sizes.size
        planner_lvl = planner(maps[-1])
        # planner_lvl.update_directions(planner_lvl.directions[:-1])
        curr_cell_lvl = get_cell_index(curr_cell.x, curr_cell.y, lvl, lvl_sizes)
        max_utility, max_path = planner_lvl.findMax_kConstrained(max(maps[-1].shape), curr_cell_lvl)
        next_cell = max_path[0]

        # Only consider staying in the same cell if the upper level planned to stay in the upper cell
        if (next_cell.x - curr_cell_lvl.x, next_cell.y - curr_cell_lvl.y) == (0, 0):
            without_zero = False
        else:
            without_zero = True

        # Keep track of the planned paths and positions for each lvl
        max_path_array.insert(0, max_path)
        curr_cell_array.insert(0, curr_cell_lvl)

        print 'lvl 2: ' + str(next_cell.getArray())


        for lvl in range(n_lvl-1,-1,-1): # Go through every level downwards, skip the uppermost level.

            # If this is the last level, do not include the same cell in the planner.
            if lvl == 0:
                without_zero = True

            # Select the next planner based on the above level, then update the current cell to current lvl.
            planner_lvl, without_zero  = select_planner(next_cell.x, next_cell.y, curr_cell_lvl.x, curr_cell_lvl.y, without_zero) #without_zero)
            curr_cell_lvl = get_cell_index(curr_cell.x, curr_cell.y, lvl, lvl_sizes)
            print 'current cell lvl' + str(curr_cell_lvl.getArray())
            # Update the planner with the current map.
            planner_lvl.update_matrix(maps[lvl])

            # Calculate next cell.
            max_utility, max_path = planner_lvl.findMax_kConstrained(lvl_sizes[lvl], curr_cell_lvl)  # k is equal to the size of the level above
            next_cell = max_path[0]

            # Keep track of the planned paths and positions for each lvl
            max_path_array.insert(0, max_path)
            curr_cell_array.insert(0, curr_cell_lvl)

            print 'lvl ' + str(lvl) + ': ' + str(next_cell.getArray())



        # Drawing
        for lvl in range(0, len(axes)):

            if i == 0:
                hist_array.append([[curr_cell_array[lvl].x, curr_cell_array[lvl].y]])
            else:
                hist_array[lvl].append([curr_cell_array[lvl].x, curr_cell_array[lvl].y])

            axes[lvl].clear()
            axes[lvl].imshow(maps[lvl].T, interpolation='none', origin='lower')
            axes[lvl].scatter([row[0] for row in hist_array[lvl]], [row[1] for row in hist_array[lvl]], marker='+')
            max_path_x = getList2Array(max_path_array[lvl])[:, 0]
            max_path_y = getList2Array(max_path_array[lvl])[:, 1]
            a=10
            axes[lvl].scatter(max_path_x, max_path_y, s=[10*a, 7*a, 5*a, 3*a, 2*a, 1*a], marker='o', c='r')
            axes[lvl].scatter(curr_cell_array[lvl].x, curr_cell_array[lvl].y, s=5*15, c='g', marker='*')
            plt.pause(0.00000001)



        #raw_input()


        # Debug
        curr_cell = next_cell
        i += 1
        if i == 30: break



    raw_input(" Press Enter to finish")