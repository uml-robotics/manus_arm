#!/usr/bin/env python

'''

Generates a connectivity map by simulating growth of seeded neuron cells. 

The growth simulation is a three-stage process:
1. Determine the cell density for each region of the MEA.
2. Assign cells to locations based on density. 
3. Connect the cells based on a connectivity function. 

Cell density is assigned based on random midpoint displacement fractals, aka "plasma fractals". 
This approach was chosen because it provides a stochastic approach that is computationally 
efficient. If it is not a good model of the actual distribution of neurons, a different algorithm
can be used. 

Cells are probabilistically assigned to locations based on the cell density (actually cell probability)
at that point. Cells are modeled as point locations. 

For each pair of cells, the probability of their connection is determined based on the distance 
between the cells and the cells are connected accordingly. Note that this results in N(N-1) comparisons, 
so roughly n^2 connections. Put that in your NP pipe and smoke it. 

'''

import roslib; roslib.load_manifest('neuro_recv')
import rospy
import math
import numpy as np
import networkx as nx
import pygame
from pickle import Pickler
import datetime
from brian import *
import random
from pprint import pprint
import Image

'''
Data structure for a MEA channel pad
    neurons: list of neurons within range of the pad
    total_weight: combined weights of all the neurons
'''
class Channel():
    def __init__(self):
        self.neurons = []
        self.total_weight = 0.0
        
    def add(self, neuron):
        self.neurons.append(neuron)
        self.total_weight += neuron.weight
        
    def size(self):
        return len(self.data)

'''
Data structure for a neuron that is close to a channel pad.
    id: unique id of the neuron
    weight: distance from the center of the pad
'''
class CloseNeuron():
    def __init__(self, id, dist):
        self.id = id
        self.weight = 40.0 - dist

'''
Take a 2-D Numpy array and fill it in with a density map. The map values are in the range 
0-1 and describe the probability of there being a cell soma located at that location. 
1 is a cell, 0 is no cell. 
'''
def genPCell(density_map):
    #Assign a random value to each corner of the array
    max_x, max_y = density_map.shape
    random.seed()
    
    #Set corners to random values
    density_map[0][0] = random.random()
    density_map[max_x -1][0] = random.random()
    density_map[0][max_y - 1] = random.random()
    density_map[max_x-1][max_y-1] = random.random()
    
    #perform recursive plasma fractal generation
    squarePlasma(density_map)
    
def displace(size, originalSize):
    max = (float(size)/float(originalSize)) * 3.00
    displacement = (random.random() - 0.5) * max
    return displacement

def diSqPlasma(array):
    pass

def squarePlasma(array):
    if array.shape == (2,2):
        return
    
    #Calculate midpoint values
    height, width = array.shape
    array[height/2][width-1] = (array[0][width-1] + array[height-1][width-1])/2
    array[height/2][0] = (array[0][0] + array[height-1][0])/2
    array[0][width/2] = (array[0][0] + array[0][width-1])/2
    array[height-1][width/2] = (array[height-1][0] + array[height-1][width-1])/2
    
    #Calculate center value
    center = 0
    center += array[height/2][width-1]
    center += array[height/2][0]
    center += array[0][width/2]
    center += array[height-1][width/2]
    center = center/4
    #Add displacement
    center += displace(height+width, 257*2)
    if center > 1:
        center = 1
    if center < 0:
        center = 0;
    #Set center value 
    array[height/2][width/2] = center
    
    #perform this step on sub-arrays
    squarePlasma(array[0:height/2+1,0:width/2+1])
    squarePlasma(array[0:height/2+1,width/2:width])
    squarePlasma(array[height/2:height,0:width/2+1])
    squarePlasma(array[height/2:height,width/2:width])
    
'''
Find the next highest power of two, to get a shape that's good for
plasma fractal generation 
'''      
def nextPowTwo(start):
    y = math.floor(math.log(start,2))
    return int(math.pow(2, y+1))

'''
Calculate the probability of a connection between two neurons, 
based on their distance apart.  
Generates a value from 0 to 1, centered at 
'''
def gaussConnect(distance, center):
    return math.e - ((distance - center)**2)/(2*center/2)**2

'''
Use pygame to render an array. The result is an image of the same dimensionality as the
array, with each pixel set to 255*the value of the corresponding array location. 
'''
def renderMap(densityData, title):
    if densityData.ndim != 2:
        print "Can only render 2-D arrays"
    
    
    #Display the image
    win = pygame.display.set_mode(densityData.shape)
    win.fill((0,0,0))
    pygame.display.set_caption(title)
    screen = pygame.display.get_surface()
    color = densityData * 255
    color = color.astype(int)
    pygame.surfarray.blit_array(screen, color)
    pygame.display.flip()
    
    #Write the array to an image
    pic = Image.fromarray(color.astype(float))
    pic.convert('RGB').save("{0}.png".format(title), "PNG")
    
    while 1:
        for event in pygame.event.get(): 
            if event.type == pygame.QUIT:
                return

def grimReap(density_map, survival_rate):
    #For each cell, give it a survival_rate % chance of getting killed
    x_max, y_max = density_map.shape
    for ii in xrange(0,x_max):
        for jj in xrange(0, y_max):
            if random.random() > survival_rate/100.00:
                density_map[ii][jj] = 0
                
def densityReap(density_map, expected_cells):
    #Count the cells
    total_cells = 0
    for cell in density_map.flat:
        total_cells += cell
    #Decrease the surplus population
    surplus = total_cells - expected_cells
    print "total: ", total_cells
    print "expected: ", expected_cells
    print "surplus: ", surplus
    x_max, y_max = density_map.shape
    #Pick a random location in the dish, and if there is a cell there,
    #kill it. Repeat until the surplus population is gone
    while surplus > 0:
        #pick a random location
        rand_X = random.randint(0,x_max-1)
        rand_Y = random.randint(0,y_max-1)
        if density_map[rand_X][rand_Y] == 1:
            density_map[rand_X][rand_Y] = 0
            surplus -= 1

def countCells(density_map):
    #Count the cells
    total_cells = 0
    for cell in density_map.flat:
        total_cells += cell
    return total_cells
        
if __name__ == '__main__':
    # Make this a ROS node so it can be run like other nodes even though it
    # doesn't do any ROS stuff
    rospy.init_node("seed_mea")
    
    #Set up some defaults, measurements in um
    soma_dia = 30
    dish_width = 2500 
    dendrite_max = 180
    #Distance of maximum connections
    axon_growth = 220
    #Axons cannot reach out of dish
    axon_max = dish_width
    #Cell survival rate, as percentage of plated cells
    survival_rate = 65
    #Cell connectivity, as percentage of total connections
    connectivity_rate = 55
    #Cell density in cells/mm^2
    cell_density = 900
    
    #Assume neurons are square, don't pile up
    #Create a list of lists representing possible cell locations
    cells_per_edge = int(dish_width/soma_dia)
    
    #Bump edge length up to make plasma fractal calculation easy
    edge_len = nextPowTwo(cells_per_edge) + 1
    
    #Calculate cell distribution probability
    density_map = np.zeros((edge_len, edge_len))
    genPCell(density_map)
    renderMap(density_map, "Plasma")
    
    #Convert probabilities of cells into presence or absence
    #Iterating the data (e.g "for row in density_map") gets 
    #copies, iterating the indexes gets references
    for row in xrange(edge_len):
        for col in xrange(edge_len):
            if random.random() < density_map[row][col]:
                density_map[row][col] = 1
            else:
                density_map[row][col] = 0
    renderMap(density_map, "Cells")
    
    #Reap cells to configured density
    #Convert to mm^2 and multiply by density to get expected value
    expected_cells = (dish_width/1000)**2 * cell_density
    densityReap(density_map, expected_cells)
    renderMap(density_map, "Density Corrected")
                
    #Kill off some percentage of the cells to reflect cell death. 
    #Can lose 45-55% of the cells by 17 days in vitro, which is about the mature level
    grimReap(density_map, survival_rate)
    renderMap(density_map, "Cull the Weak")
    
    #At this point, each element of the density map represents 
    #a soma-sized patch of space that either does or does not contain
    #the soma of a cell.
    print "Cells remaining:", countCells(density_map)
    
    #Calculate connections
    #Connections are stored as a list of tuples, (from neuron, to neuron)
    print "Building connectivity...",
    neuron_list = []
    neuron_id = 0
    for row_1 in xrange(edge_len):
        for col_1 in xrange(edge_len):
            if density_map[row_1][col_1] == 1:
                #If there is a neuron at the current position store its location
                #and an ID, incrementing the ID
                neuron_list.append(((row_1, col_1), neuron_id))
                neuron_id += 1
    #Now have a list of neurons, their locations, and their IDs. Run through it 
    #and connect the neurons based on pairwise probability of connection
    conn_list = []
    for from_neuron in neuron_list:
        for to_neuron in neuron_list:
            #Calculate distance. Grid units are soma diameters, so multiply by that to get real distance
            distance = math.sqrt((from_neuron[0][0] - to_neuron[0][0])**2 + (from_neuron[0][1] - to_neuron[0][1])**2)
            distance *= soma_dia
            if random.random() < gaussConnect(distance, axon_growth):
                #Just note that they are connected
                conn_list.append((from_neuron[1], to_neuron[1]))
                
    print "done."
    
    #Done building terrible huge list with a time consuming process. Pickle the results.
    print "Saving connectivity...", 
    now = datetime.datetime.now()
    file_date = "{0}-{1}-{2}-{3}:{4}:{5}".format(now.year, now.month, now.day, now.hour, now.minute, now.second) 
    
    #Build a networkx graph
    conn_graph = nx.DiGraph()
    for connection in conn_list:
        conn_graph.add_edge(connection[0], connection[1])
    
    #Save the networkx graph
    with open("nx_graph_{0}.pickle".format(file_date), "w") as outfile:
        Pickler(outfile, 0).dump(conn_graph)
        outfile.close()

    #Save the connectivity list
    with open("connections_{0}.pickle".format(file_date), "w") as outfile:
        Pickler(outfile, 0).dump(conn_list)
        outfile.close()
        
    print "done."
    
    #Generate a Brian environment based on the connectivity 
          
    #LIF model, different time constants for excitory and inhibitory
    eqs = '''
    dv/dt = (ge+gi-(v+49*mV))/(20*ms) : volt
    dge/dt = -ge/(5*ms) : volt
    dgi/dt = -gi/(10*ms) : volt
    '''
    
    #Set up neuron population and initial voltage
    neuron_count = int(countCells(density_map))
    P = NeuronGroup(neuron_count, eqs, threshold=-50*mV, reset=-60*mV)
    P.v = -60*mV + 15 * mV * rand(len(P)) 
    
    #Partition population based on assumption of 25% inhibitory, 75% excitatory
    excitory_count = int(neuron_count * 0.75)
    inhib_count = neuron_count - excitory_count
    #Pe = P.subgroup(excitory_count)
    #Pi = P.subgroup(inhib_count)
    
    #Set up connections. First, pick a random set of inhibitory neuron ids. 
    #Everything else is excitatory. Then create the connection objects and 
    #populate them according to the connectivity list
    inhib_neurons = random.sample(xrange(neuron_count), inhib_count)
    Ce = Connection (P, P, 'ge')
    Ci = Connection (P, P, 'gi')
    for connection in conn_list:
        if connection[0] in inhib_neurons:
            #Need to subtract the count because first parameter is its index in Pi, not P
            Ci[connection[0], connection[1]] = -9*mV
        else:
            #print "{0} -> {1}".format(connection[0], connection[1])
            Ce[connection[0], connection[1]] = 1.62*mV
    
    #Determine which neurons have MEA pads under them. 
    #These are the ones that will be monitored when the simulation runs. 
    #For each pad, calculate the location of the pad in grid coordinates
    pad_rows = 8
    pad_cols = 8
    ignore_corners = True #Corner electrodes are frequently used as reference points, not recorded
    pad_dia = 30 #In um
    pad_spacing = 200 #again, in um, center to center
    pad_threshold = 40 #In um; how far a neuron can be from the center of a pad
    
    #For each pad, check the neurons to find all that are within a soma diameter
    #of the pad. Those neurons are used to record at that pad when the
    #simulation runs.
    pad_neuron_map = {}
    for pad_x in xrange(0,pad_rows):
        for pad_y in xrange (0,pad_cols):
            if ignore_corners:
                if pad_x == 0 and pad_y == 0:
                    continue
                elif pad_x == 0 and pad_y == (pad_cols - 1):
                    continue
                elif pad_x == (pad_rows - 1) and pad_y == 0:
                    continue
                elif pad_x == (pad_rows - 1) and pad_y == (pad_cols - 1):
                    continue 
                
            #Calculate the pad location in um
            pad_x_loc = pad_x * pad_spacing
            pad_y_loc = pad_y * pad_spacing
             
            #For each neuron, determine how close it is to the pad
            close_neurons = Channel()
            for neuron in neuron_list:
                #Neuron grid is in soma diameters, convert to um
                neuron_x_loc = neuron[0][0] * soma_dia
                neuron_y_loc = neuron[0][1] * soma_dia
                #Calculate distance from center of pad
                dist = math.sqrt((neuron_x_loc - pad_x_loc)**2 + (neuron_y_loc - pad_y_loc)**2)
                #
                if dist <= pad_threshold:
                    #Close enough to influence pad
                    close_neurons.add(neuron[1], dist)
                    
            if close_neurons.size() == 0:
                pad_neuron_map[(pad_x, pad_y)] = None
            else:
                pad_neuron_map[(pad_x, pad_y)] = close_neurons
    pprint(pad_neuron_map)
    
    #Save the pad neuron map list
    with open("pad_{0}.pickle".format(file_date), "w") as outfile:
        Pickler(outfile, 0).dump(pad_neuron_map)
        outfile.close()
    
    '''
    #Set up and run the simulation
    M = SpikeMonitor(P)
    #Monitor the voltages of the neurons that land on pads
    vMonitor = StateMonitor(P, "v", record=[neuron for neuron in pad_neuron_map.values() if neuron != None])
    print "Running simulation...",
    run(10*second)
    subplot(211)
    raster_plot(M, newfigure=False)
    subplot(212)
    vMonitor.plot()
    xlabel('Time (seconds)')
    ylabel('Membrane potential (volts)')
    #legend(('Neuron #100', 'Neuron #500', 'Neuron #1000', 'Neuron #1500' ), 'upper right')
    show()
    show()
    '''
    
    print "done."
