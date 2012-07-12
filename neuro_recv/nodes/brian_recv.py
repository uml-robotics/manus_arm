#!/usr/bin/env python
import roslib; roslib.load_manifest('neuro_recv')
import rospy
from neuro_recv.msg import dish_state
from brian import *
from pickle import Unpickler
from pprint import pprint
import random
import argparse

def brianRecv(connections, channels):
    # Initialize ROS and setup the dish state publisher
    rospy.init_node('brian_recv')
    pub = rospy.Publisher('dish_states', dish_state)
    
    # Wait for a subscriber before publishing
    rospy.loginfo('Waiting for subscriber...')
    while pub.get_num_connections() < 1 and not rospy.is_shutdown():
        pass
    rospy.loginfo('Subscriber found. Continuing...')
    
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
        
    # Initialize timestamp offset
    offset = rospy.Time.now() - rospy.Time(0)

    # Set a loop rate of 200 Hz
    loop_rate = rospy.Rate(200)

    # Main loop
    while not rospy.is_shutdown():
        # Run the simulation for 1 ms (1000 updates/sec)
        run(1 * ms)
        
        # Initialize a new dish state
        d = dish_state()
        d.header.stamp = rospy.Time.now() - offset
        
        for index in range(60):
            if channels[index] == None:
                d.samples[index] = 0.0
            elif len(channels[index]) == 1:
                d.samples[index] = P.state("v")[channels[index][0]]
            else:
                sum = 0.0
                count = 0
                for n in channels[index]:
                    sum += P.state("v")[n]
                    count += 1
                    #print 'V: ', P.state("v")[n], ' Sum: ', sum, ' Count: ', count
                d.samples[index] = sum / count
            #print index, ': ', d.samples[index]
                
        #rospy.loginfo(d)
        pub.publish(d)
        print 'Published'
        #rospy.signal_shutdown("For testing")
        loop_rate.sleep()

def channelizer(pad_neuron_map):
    channels = list()
    index = 0
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
            index += 1
    return channels
        
if __name__ == '__main__':
    # Get the connections filename
    #argparser = argparse.ArgumentParser("Load and run an existing saved network")
    #argparser.add_argument("conn_path", type=str, nargs=1, help="the connectivity pickle file to load and run")
    #argparser.add_argument("pad_path", type=str, nargs=1, help="the pad neuron map pickle file to load and run")
    #args = argparser.parse_args()
    
    # Load the files
    with open('/home/jon/Python/connections_2012-7-12-16:14:11.pickle') as infile:
        connections = Unpickler(infile).load()
    with open('/home/jon/Python/pad_2012-7-12-16:14:11.pickle') as infile:
        pad_neuron_map = Unpickler(infile).load()
        
    # Get list of neurons for each channel from the pad neuron map
    channels = channelizer(pad_neuron_map)
    
    # Run the ROS node
    try:
        brianRecv(connections, channels)
    except rospy.ROSInterruptException: pass