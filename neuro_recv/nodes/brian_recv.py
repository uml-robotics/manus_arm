#!/usr/bin/env python
import roslib; roslib.load_manifest('neuro_recv')
import rospy
from neuro_recv.msg import dish_state
from brian import *
from pickle import Unpickler
import random

'''
Runs a Brian simulation, then creates and publishes dish_states using
the generated data.
'''
class BrianSimulation():
    def __init__(self, connections, channels):
        self.connections = connections
        self.channels = channels
        
        # Get parameters
        self.getParams()
        
        # Initialize publishers
        self.initPubs()
        
        # Run the Brian simulation
        self.runSimulation()
        
        # Publish buffer dishes
        self.publishBuffer()
        
        # Initialize time stamp offset
        self.offset = rospy.Time.now() - rospy.Time(0)
        
        # Publish the rest of the dishes
        self.publish()
        
    def getParams(self):
        # Get brian_running_time parameter
        try:
            self.running_time = rospy.get_param('brian_running_time')
        except KeyError:
            rospy.logerr('Could not get brian_running_time parameter, default is 30.0')
            self.running_time = 30.0
            
        # Get do_volt_distr parameter
        try:
            self.do_volt_distr = rospy.get_param('do_volt_distr')
        except KeyError:
            rospy.logerr('Could not get do_volt_distr parameter, default is True')
            self.do_volt_distr = True
            
        # Get do_burst_calc parameter
        try:
            self.do_burst_calc = rospy.get_param('do_burst_calc')
        except KeyError:
            rospy.logerr('Could not get do_burst_calc parameter, default is True')
            self.do_burst_calc = True
            
        # Get buffer_size parameter
        try:
            self.buffer_size = rospy.get_param('buffer_size')
        except KeyError:
            rospy.logerr('Could not get buffer_size parameter, default is 1000')
            self.buffer_size = 1000
        
        # Get loop_rate parameter
        try:
            self.loop_rate = rospy.get_param('loop_rate')
        except KeyError:
            rospy.logerr('Could not get loop_rate parameter, default is 200')
            self.loop_rate = 200
            
    def initPubs(self):
        rospy.loginfo('Waiting for subscribers...')
        
        if self.do_volt_distr == True:
            # Advertise and wait for a subscriber
            self.dish_pub_volt = rospy.Publisher('dish_states_to_volt_distr', dish_state)
            while self.dish_pub_volt.get_num_connections() < 1 and not rospy.is_shutdown():
                pass
            
        if self.do_burst_calc == True:
            # Advertise and wait for subscribers
            self.dish_pub_viz = rospy.Publisher('dish_states_to_dish_viz', dish_state)
            self.dish_pub_burst = rospy.Publisher('dish_states_to_burst_creator', dish_state)
            while self.dish_pub_viz.get_num_connections() < 1 and self.dish_pub_burst.get_num_connections() < 1 and not rospy.is_shutdown():
                pass
            
    def runSimulation(self):            
        # LIF model, different time constants for excitory and inhibitory
        eqs = Equations('''
              dv/dt = (ge+gi-(v+49*mV))/(20*ms) : volt
              dge/dt = -ge/(5*ms)               : volt
              dgi/dt = -gi/(10*ms)              : volt
              ''')
    
        # Get the neuron count. This is kind of hacky, as it depends on the
        # links being stored in increasing order. The +1 is because they are indexed
        # from zero. 
        neuron_count = self.connections[-1][0] + 1
        
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
        for connection in self.connections:
            prune = random.randint(0,100)
            #if prune < percent_connected:
            if connection[0] in inhib_neurons:
                Ci[connection[0], connection[1]] = -9*mV
            else:
                Ce[connection[0], connection[1]] = 1.62*mV
    
        # Create a list of all the neurons to record
        #print 'Recorded neurons:',
        self.recorded_neurons = []
        for channel in self.channels:
            if channel != None:
                for neuron in channel.neurons:
                    self.recorded_neurons.append(neuron.data)
                    #print neuron.data,
        #print
        #print 'Length of recorded neurons array:', len(self.recorded_neurons)
        
        # Create a monitor to record the voltages of each neuron every 1 ms.
        # If timestep = 1, then 10 actions are recorded every ms.
        # If timestep = 10, then 1 action is recorded every ms.
        # Setting timestep to 1 is in effect 30 s of simulation in just 3 s.
        self.M = StateMonitor(P, "v", record=self.recorded_neurons, timestep=10)
     
        # Run the simulation
        # Divide seconds by 10 if timestep=1 above
        rospy.loginfo("Running simulation...")
        run(self.running_time * second)
        rospy.loginfo("Simulation finished")
        
    def publishBuffer(self):
        rospy.loginfo("Publishing buffer dishes...")
        loop_rate = rospy.Rate(self.loop_rate)
        
        for current_dish in range(self.buffer_size):
            # Initialize a new dish state
            d = dish_state()
            
            # For each channel in the dish state
            for channel in range(60):
                d.samples[channel] = 0.0
                #print 'Channel', channel, ':'
                
                # If there are no recorded neurons on this channel, then the voltage is 0.0
                # Otherwise, get the weighted average of the volts of the neurons on this channel 
                if self.channels[channel] != None:
                    for neuron in self.channels[channel].neurons:
                        weight = neuron.weight / self.channels[channel].total_weight
                        weighted = self.M[neuron.data][current_dish] * weight
                        d.samples[channel] += weighted
                        #print '    Neuron: ', neuron.data, 'Weight:', weight, 'State:', M[neuron.data][current_dish]
                        #print '    Weighted State: ', weighted, 'Total:', d.samples[channel]
    
            # If we are publishing to burst_calc, then publish the dish_state now
            # Otherwise, do nothing with it
            if self.do_burst_calc == True:
                self.dish_pub_burst.publish(d)
                loop_rate.sleep()
                        
    def publish(self):
        rospy.loginfo("Publishing dishes...")
        loop_rate = rospy.Rate(self.loop_rate)
        current_dish = self.buffer_size
        
        while current_dish < len(self.M[self.recorded_neurons[0]]) and not rospy.is_shutdown():
            # Initialize a new dish state
            d = dish_state()
            
            # For each channel in the dish state
            for channel in range(60):
                d.samples[channel] = 0.0
                #print 'Channel', channel, ':'
                
                # If there are no recorded neurons on this channel, then the voltage is 0.0
                # Otherwise, get the weighted average of the volts of the neurons on this channel 
                if self.channels[channel] != None:
                    for neuron in self.channels[channel].neurons:
                        weight = neuron.weight / self.channels[channel].total_weight
                        weighted = self.M[neuron.data][current_dish] * weight
                        d.samples[channel] += weighted
                        #print '    Neuron: ', neuron.data, 'Weight:', weight, 'State:', M[neuron.data][current_dish]
                        #print '    Weighted State: ', weighted, 'Total:', d.samples[channel]
    
            # Add the time stamp
            d.header.stamp = rospy.Time.now() - self.offset
    
            # Publish the dish state
            if self.do_volt_distr == True:
                self.dish_pub_volt.publish(d)
            if self.do_burst_calc == True:
                self.dish_pub_burst.publish(d)
                self.dish_pub_viz.publish(d)
                
            # Sleep and move on to the next dish
            current_dish += 1    
            loop_rate.sleep()
            
        # Publish a final dish to let the nodes know to finish up    
        end = dish_state()
        end.last_dish = True
        if self.do_volt_distr == True:
            self.dish_pub_volt.publish(end)
        if self.do_burst_calc == True:
            self.dish_pub_burst.publish(end)
            
        rospy.loginfo('Publishing finished')        
            
'''
Data structure for a multi electrode array channel pad
    neurons      : list of neurons within range of the pad
    total_weight : combined weights of all the neurons
'''
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

'''
Data structure representing a neuron that is within range of a channel pad
    self.data   : unique id of the neuron
    self.weight : how much weight this neuron has in calculating average
                  activity for its channel
'''
class CloseNeuron():
    def __init__(self, data, dist):
        self.data = data
        self.weight = 40.0 - dist
        
    def __str__(self):
        return str(self.data) + ':' + str(self.weight)

'''
Channelizer takes data from an 8x8 map and populates a 60-channel list.
'''
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
        # Got file names ok, now load the files
        try:
            connections_file = open(connections_file_name)
            pad_neuron_map_file = open(pad_neuron_map_file_name)
        except IOError:
            rospy.logfatal('Could not open pickle files')
        else:
            # Opened files okay, now unpickle and run the node
            connections = Unpickler(connections_file).load()
            pad_neuron_map = Unpickler(pad_neuron_map_file).load()
            connections_file.close()
            pad_neuron_map_file.close()
            
            # Get list of neurons for each channel from the pad neuron map
            channels = channelizer(pad_neuron_map)
            
            # Run the ROS node
            try:
                brian = BrianSimulation(connections, channels)
            except rospy.ROSInterruptException:
                pass