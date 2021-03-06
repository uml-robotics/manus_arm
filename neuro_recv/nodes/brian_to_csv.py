#!/usr/bin/env python

'''
brian_to_csv.py
Copyright 2013 University of Massachusetts Lowell
Author: Jonathan Hasenzahl, Abraham Shultz
'''

## @package brian_to_csv
#
# This is a ROS node that is used by itself to generate CSV data files by running
# Brian simulations. Use this node to generate a CSV file, and then use the
# csv_recv node to read the generated file.

# Note: The CSV file path is currently hardcoded at line 133; it would be nice
#       to make this a server parameter at some point.
#
# @copyright Copyright 2013 University of Massachusetts Lowell
# @author Jonathan Hasenzahl
# @author Abraham Shultz

import roslib; roslib.load_manifest('neuro_recv')
import rospy
from brian import *
from pickle import Unpickler
import random


## @briefData structure for a MEA channel pad
#  neurons: list of neurons within range of the pad
#  total_weight: combined weights of all the neurons
class Channel():
    def __init__(self):
        self.neurons = []
        self.total_weight = 0.0
        
    def __str__(self):
        name = 'Weight:' + str(self.total_weight)
        if len(self.neurons) == 0:
            name += '[No neurons]'
        else:
            for neuron in self.neurons:
                name += '[' + str(neuron) + ']'
        return name
        
    def add(self, neuron):
        self.neurons.append(neuron)
        self.total_weight += neuron.weight
        
    def size(self):
        return len(self.neurons)


## @brief Data structure for a neuron that is close to a channel pad.
#  data: unique id of the neuron
#  weight: distance from the center of the pad
class CloseNeuron():
    def __init__(self, data, dist):
        self.data = data
        self.weight = 40.0 - dist
        
    def __str__(self):
        return str(self.data) + ':' + str(self.weight)

## Runs a Brian simulation "P" and records and publishes voltages captured by
#  the multielectrode array "channels"
def brianRecv(connections, channels):
    # LIF model, different time constants for excitory and inhibitory
    eqs = Equations('''
          dv/dt = (ge+gi-(v+49*mV))/(20*ms) : volt
          dge/dt = -ge/(5*ms)               : volt
          dgi/dt = -gi/(10*ms)              : volt
          ''')

    # Get the neuron count. This is kind of hacky, as it depends on the
    # links being stored in increasing order. The +1 is because they are indexed
    # from zero. 
    neuron_count = connections[-1][0] + 1
    
    # Set up the population based on the count and equations
    P = NeuronGroup(neuron_count, eqs, threshold=-50*mV, reset=-60*mV)
    P.v = -60*mV + (15 * rand(len(P))) * mV  
    
    # Partition population based on assumption of 25% inhibitory, 75% excitatory
    excitory_count = int(neuron_count * 0.75)
    inhib_count = neuron_count - excitory_count
    #Pe = P.subgroup(excitory_count)
    #Pi = P.subgroup(inhib_count)
    
    # Set up connections. First, pick a random set of inhibitory neuron ids. 
    # Everything else is excitatory. Then create the connection objects and 
    # populate them according to the connectivity list
    inhib_neurons = random.sample(xrange(neuron_count), inhib_count)
    Ce = Connection (P, P, 'ge')
    Ci = Connection (P, P, 'gi')
    for connection in connections:
        prune = random.randint(0,100)
        #if prune < percent_connected:
        if connection[0] in inhib_neurons:
            Ci[connection[0], connection[1]] = -9*mV
        else:
            Ce[connection[0], connection[1]] = 1.62*mV

    # Create a list of all the neurons to record
    print 'Recorded neurons:',
    recorded_neurons = []
    for channel in channels:
        if channel != None:
            for neuron in channel.neurons:
                recorded_neurons.append(neuron.data)
                print neuron.data,
    print
    print 'Length of recorded neurons array', len(recorded_neurons)
    
    # Create a monitor to record the voltages of each neuron every 1 ms.
    # If timestep = 1, then 10 actions are recorded every ms.
    # If timestep = 10, then 1 action is recorded every ms.
    # Setting timestep to 1 is in effect 30 s of simulation in just 3 s.
    M = StateMonitor(P, "v", record=recorded_neurons, timestep=10)

    # Get running time parameter
    try:
        running_time = rospy.get_param('brian_running_time')
    except KeyError:
        rospy.logerr('Could not get brian_running_time parameter, default will be used')
        running_time = 30.0
        
    # Run the simulation
    # Divide seconds by 10 if timestep=1 above
    rospy.loginfo("Running simulation...")
    run(running_time * second)
    rospy.loginfo("Simulation finished")    
        
    rospy.loginfo('Recording dish states...')
        
    # Open CSV file for recording
    # TODO: Make this a server parameter
    log = open('/home/jon/ros_workspace/other_stuff/csv/brian.csv', 'w')
    log.write('index,')
    for index in range(60):
        log.write('channel_' + str(index) + ',')
    log.write('\n')
       
    # Write the dishes
    for current_dish in range(len(M[recorded_neurons[0]])):    
        log.write(str(current_dish) + ',')
        for index in range(60):
            volt = 0.0
            
            # Get the weighted average of the volts of the neurons on this 
            # channel 
            if channels[index] != None:
                for neuron in channels[index].neurons:
                    weight = neuron.weight / channels[index].total_weight
                    weighted = M[neuron.data][current_dish] * weight
                    volt += weighted
                    #print '    Neuron: ', neuron.data, 'Weight:', weight, 'State:', M[neuron.data][current_dish]
                    #print '    Weighted State: ', weighted, 'Total:', d.samples[index]
            log.write(str(volt) + ',')
        
        log.write('\n')        
        
    log.close()
    
    rospy.loginfo('Recording finished')

## Populates a 60-channel list using data from an x,y map
def channelizer(pad_neuron_map):
    channels = []
    for row in range(8):
        for col in range(8):
            if row == 0 and col == 0:
                continue
            elif row == 0 and col == 7:
                continue
            elif row == 7 and col == 0:
                continue
            elif row == 7 and col == 7:
                continue
            channels.append(pad_neuron_map[(row, col)])
    return channels
        
if __name__ == '__main__':
    rospy.init_node('brian_to_csv')
    
    # Get the file name parameters
    try:
        connections_file_name = rospy.get_param('brian_connections_file_path')
        pad_neuron_map_file_name = rospy.get_param('brian_pad_neuron_map_file_path')
    except KeyError:
        rospy.logfatal('Could not load file name parameters')
    else:
        # Load the files
        try:
            connections_file = open(connections_file_name)
            pad_neuron_map_file = open(pad_neuron_map_file_name)
        except IOError:
            rospy.logfatal('Could not open pickle files')
        else:
            connections = Unpickler(connections_file).load()
            pad_neuron_map = Unpickler(pad_neuron_map_file).load()
            connections_file.close()
            pad_neuron_map_file.close()
            
            # Get list of neurons for each channel from the pad neuron map
            channels = channelizer(pad_neuron_map)
            
            # Run the ROS node
            try:
                brianRecv(connections, channels)
            except rospy.ROSInterruptException: pass