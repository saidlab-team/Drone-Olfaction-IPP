


import numpy as np
import utm

conversion_factor = 0.00001/1.1132 # 0.00001 degrees of longitude are 1.1132 meters
m1 = 111132.92		# latitude calculation term 1
m2 = -559.82		#// latitude calculation term 2
m3 = 1.175		#// latitude calculation term 3
m4 = -0.0023		#// latitude calculation term 4
p1 = 111412.84		#// longitude calculation term 1
p2 = -93.5			#// longitude calculation term 2
p3 = 0.118			#// longitude calculation term 3




class waypoint:

    def __init__(self, latitude, longitude, altitude):

        self.AMSLAltAboveTerrain = "null"
        self.altitude = altitude
        self.altitudeMode = 1
        self.autoContinue = "true"
        self.command = 16
        self.doJumpID = 1
        self.frame = 3
        self.params = [0, 0, 0, "null", latitude, longitude, altitude]
        self.type = "SimpleItem"




    def print_waypoint(self):

        print("{")
        print "\t\"AMSLAltAboveTerrain\": " + str(self.AMSLAltAboveTerrain) + ","
        print '\t"Altitude": ' + str(self.altitude) +','
        print '\t"AltitudeMode": ' + str(self.altitudeMode) + ','
        print '\t"autoContinue": ' + str(self.autoContinue) + ','
        print '\t"command": ' + str(self.command) + ','
        print '\t"doJumpID": ' + str(self.doJumpID) + ','
        print '\t"frame": ' + str(self.frame) + ','
        print '\t"params": ['
        print '\t\t' + str(self.params[0]) + ','
        print '\t\t' + str(self.params[1])+','
        print '\t\t' + str(self.params[2])+','
        print '\t\t' + str(self.params[3])+','
        print '\t\t' + str(self.params[4])+','
        print '\t\t' + str(self.params[5])+','
        print '\t\t' + str(self.params[6])
        print '\t],'
        print '\t"type": "' + str(self.type) +'"'
        print "},"


def draw_spiral(latitude, longitude, altitude, radius = 5, step = 1, decimation = 10):

    # height or heights for the spiral
    # radius: Total radius of the spiral
    # step: step of the spiral each revolution
    # decimation: number of points per revolution

    wp_list = []

    # // Calculate the length of a degree of latitude and longitude in meters
    latlen = m1 + (m2 * np.cos(2 * latitude)) + (m3 * np.cos(4 * latitude)) + \
    (m4 * np.cos(6 * latitude))

    longlen = (p1 * np.cos(latitude)) + (p2 * np.cos(3 * latitude)) + \
    (p3 * np.cos(5 * latitude))


    for alt in altitude:
        for phi in range(0, int(360*radius/step), int(360/decimation)):

            # Radial to cartesian coordinates
            deg = np.deg2rad(phi)
            spiral_x = ((step*deg)/(2*np.pi))*np.cos(deg)
            spiral_y = ((step*deg)/(2*np.pi))*np.sin(deg)

            # Cartesian to WSG84 decimal degrees
            spiral_long  = spiral_x*conversion_factor*np.cos(np.deg2rad(latitude))
            spiral_lat = spiral_y*conversion_factor*np.cos(np.deg2rad(latitude))

            spiral_long = (spiral_x/longlen)/3
            spiral_lat = (spiral_y/latlen)

            wp_list.append(waypoint(latitude + spiral_lat, longitude + spiral_long, alt))

    print_list(wp_list)

    return wp_list


## Print a waypoint mission from a waypoint list. Changes the first node to a take off node.
def print_list(wp_list):

    # print takeoff wp

    wp_list[0].command = 22
    wp_list[0].doJumpID = 1
    wp_list[0].params[0]=15

    index = 1
    for wp in wp_list:
        wp.doJumpID = index
        wp.print_waypoint()
        index = index + 1

def print_gps(wp_list):

    for wp in wp_list:
        print(str(wp.params[4])+", " + str(wp.params[5])+", " + str(wp.params[6]))


if __name__ == '__main__':

    # height or heights for the spiral
    # radius: Total radius of the spiral
    # step: step of the spiral each revolution
    # decimation: number of points per revolution

    # latitude, longitude, altitude or list of altitudes, radius, step of each revolution, decimation (# of points per revolution)
    wp_list = draw_spiral(51.9903086, 5.66452737, [6], 20, 5, 10)

    print_gps(wp_list)