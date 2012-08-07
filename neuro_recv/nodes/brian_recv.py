#!/usr/bin/env python
import roslib; roslib.load_manifest('neuro_recv')
import rospy
from neuro_recv.msg import dish_state
from brian import *
from pickle import Unpickler
import random

# Runs a Brian simulation "P" and records and publishes voltages captured by
# the multielectrode array "channels"
def brianRecv(P, channels):
    pub = rospy.Publisher('dish_states', dish_state)
    
    # Wait for a subscriber before publishing
    rospy.loginfo('Waiting for subscriber...')
    while pub.get_num_connections() < 1 and not rospy.is_shutdown():
        pass
    rospy.loginfo('Subscriber found. Continuing...')

    # Create a list of all the neurons to record
    print 'Recorded neurons:',
    recorded_neurons = []
    for channel in channels:
        if channel != None:
            for neuron in channel.neurons:
                recorded_neurons.append(neuron.id)
                print neuron,
    print
    print 'Length of recorded neurons array', len(recorded_neurons)
    
    # Create a monitor to record the voltages of each neuron every 1 ms.
    # If timestep = 1, then 10 actions are recorded every ms.
    # If timestep = 10, then 1 action is recorded every ms.
    # Setting timestep to 1 is in effect 30 s of simulation in just 3 s.
    M = StateMonitor(P, "v", record=recorded_neurons, timestep=1)

    # Get running time parameter
    try:
        running_time = rospy.get_param('brian_running_time')
    except KeyError:
        rospy.logerr('Could not get brian_running_time parameter, default will be used')
        running_time = 30.0
        
    # Run the simulation
    rospy.loginfo("Running simulation...")
    run(running_time / 10 * second)
    rospy.loginfo("Simulation finished")    

    # Get loop rate parameter
    try:
        rate = rospy.get_param('loop_rate')
    except KeyError:
        rospy.logerr('Could not get loop_rate parameter, default will be used')
        rate = 200
    loop_rate = rospy.Rate(rate)

    rospy.loginfo('Publishing dish states...')
    
    # Open log file for debugging
    log = open('/home/jon/brian_log.csv', 'w')
    log.write(',')
    for i in range(60):
        log.write(str(i) + ',')
    log.write('\n')
    
    # Initialize timestamp offset
    offset = rospy.Time.now() - rospy.Time(0)    
    
    # For each dish state in the record
    for current_dish in range(len(M[recorded_neurons[0]])):
        #log.write(str(current_dish) + ',')
        # Initialize a new dish state
        d = dish_state()
        d.header.stamp = rospy.Time.now() - offset
        
        # For each channel in a single dish state:
        for index in range(60):
            d.samples[index] = 0.0
            #print 'Index', index, ':',
            
            # Get the weighted average of the volts of the neurons on this 
            # channel 
            if channels[index] != None:
                for neuron in channels[index].neurons:
                    d.samples[index] += M[neuron.id][current_dish] * neuron.weight / channels[index].total_weight
                    #print
                    #print 'Len:', len(channels[index]), 'Neuron:', neuron, 'State:', M[neuron][current_dish], 'Sum:', sum
                d.samples[index] = sum / len(channels[index])
                #print 'Average: ', d.samples[index]
            #log.write(str(d.samples[index]) + ',')
        
        #log.write('\n')        
        pub.publish(d)
        loop_rate.sleep()
        
    rospy.loginfo('Publishing finished in ' + str((rospy.Time.now() - offset).to_sec()) + 's')
    #log.close()

# Creates and returns a Brian simulation based on "connections" array
def createSimulation(connections):
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
    
    return P    

# Populates a 60-channel list using data from an x,y map
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
    rospy.init_node('brian_recv')
    
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
            
            # Create simulation
            P = createSimulation(connections)
            
            # Run the ROS node
            try:
                brianRecv(P, channels)
            except rospy.ROSInterruptException: pass