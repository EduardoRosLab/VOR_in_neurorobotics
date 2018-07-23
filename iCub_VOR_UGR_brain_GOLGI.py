# -*- coding: utf-8 -*-
"""
This file contains the setup of the neuronal network running the iCub experiment in a VOR task
"""
# pragma: no cover

__author__ = 'Francisco Naveros, Jesus Garrido, Niceto Luque, Eduardo Ros'

### The following can be removed when PyNN 0.8 has been established or we have a more elegant
### solution
import pyNN.nest as sim
import nest
import numpy as np
import logging

from pyNN.random import RandomDistribution, NumpyRNG

logger = logging.getLogger(__name__)

# Synapsis parameters
gc_pc_weights = 0.005 
mf_vn_weights = 0.001 
pc_vn_weights = 0.00002
io_pc_weights = 0.0
mf_gc_weights = 0.0006
go_gc_weights = -0.0002
input_weights = 0.00025
mf_go_weights = 0.0006 


# Network parameters
num_MF_neurons = 100
num_GC_neurons = 2000
num_GOC_neurons = 100
num_PC_neurons = 200
num_VN_neurons = 200
num_IO_neurons = 200


sim.setup(timestep=0.5, threads=4, min_delay=0.5, max_delay=100.0)

def create_brain():
	"""
	Initializes PyNN with the neuronal network that has to be simulated for the experiment
	"""

	GR_PARAMS = {'cm': 0.002,
                 'v_rest': -70.0,
                 'tau_m': 100.0,
                 'e_rev_E': 0.0,
                 'e_rev_I': -75.0,
                 'v_reset': -70.0,
                 'v_thresh': -40.0,
                 'tau_refrac': 1.0,
                 'tau_syn_E': 0.5,
                 'tau_syn_I': 2.0}

	GO_PARAMS = {'cm': 0.002,
                 'v_rest': -70.0,
                 'tau_m': 100.0,
                 'e_rev_E': 0.0,
                 'e_rev_I': -75.0,
                 'v_reset': -70.0,
                 'v_thresh': -40.0,
                 'tau_refrac': 1.0,
                 'tau_syn_E': 0.5,
                 'tau_syn_I': 2.0}

	PC_PARAMS = {'C_m': 0.314,
                 'g_L': 0.012,
                 'E_L': -70.0,
                 'E_ex': 0.0,
                 'E_in': -75.0,
                 'e_cs': 0.0,
                 'V_reset': -70.0,
                 'V_th': -52.0,
                 't_ref': 1.0,
                 'tau_syn_ex': 0.85,
                 'tau_syn_in': 5.45,
                 'tau_syn_cs': 0.85}

	VN_PARAMS = {'C_m': 0.002,
                 'g_L': 0.0002, 
                 'E_L': -70.0,
                 'E_ex': 0.0,
                 'E_in': -80.0,
                 'e_ts': 0.0,
                 'V_reset': -70.5,
                 'V_th': -40.0,
                 't_ref': 1.0,
                 'tau_syn_ex': 0.5,
                 'tau_syn_in': 7.0,
                 'tau_syn_ts': 0.85,
                 'tau_cos': 10.0,
                 'exponent': 2.0}
    
	##THIS MODULE CAN BE DOWNLOADED FROM https://github.com/jgarridoalcazar/SpikingCerebellum/
	#try:
	#	nest.Install('cerebellummodule')
	#except nest.NESTError:
	#	pass 
    
	
	parrot_neuron = sim.native_cell_type('parrot_neuron')
	
	# Create MF population 
	MF_population = sim.Population(num_MF_neurons,parrot_neuron,{},label='MFLayer')

	# Create GOC population    
	GOC_population = sim.Population(num_GOC_neurons,sim.IF_cond_alpha(**GO_PARAMS),label='GOCLayer')
	
	# Create MF-GO connections
	mf_go_connections = sim.Projection(MF_population,
                                           GOC_population,
                                           sim.OneToOneConnector(),
                                           sim.StaticSynapse(delay=1.0, weight=mf_go_weights))



	# Create GrC population
	GC_population = sim.Population(num_GC_neurons,sim.IF_cond_alpha(**GR_PARAMS),label='GCLayer')

	# Random distribution for synapses delays and weights
	delay_distr = RandomDistribution('uniform', (1.0, 10.0), rng=NumpyRNG(seed=85524))
	weight_distr_MF = RandomDistribution('uniform', (mf_gc_weights*0.8, mf_gc_weights*1.2), rng=NumpyRNG(seed=85524))
	weight_distr_GO = RandomDistribution('uniform', (go_gc_weights*0.8, go_gc_weights*1.2), rng=NumpyRNG(seed=24568))


	# Create MF-GC and GO-GC connections
	float_num_MF_neurons = float (num_MF_neurons)
	for i in range (num_MF_neurons):
		GC_medium_index = int(round((i / float_num_MF_neurons) * num_GC_neurons))
		GC_lower_index = GC_medium_index - 40
		GC_upper_index = GC_medium_index + 60
		if(GC_lower_index < 0):
			GC_lower_index = 0

		elif(GC_upper_index > num_GC_neurons):
			GC_upper_index = num_GC_neurons

		if(GC_lower_index < GC_medium_index):
			GO_GC_con1 = sim.Projection(sim.PopulationView(GOC_population, range(i, i+1)),
                                      sim.PopulationView(GC_population, range(GC_lower_index, GC_medium_index)),
                                      sim.AllToAllConnector(),
                                      sim.StaticSynapse(delay=delay_distr, weight=weight_distr_GO))

			MF_GC_con2 = sim.Projection(sim.PopulationView(MF_population, range(i, i+1)),
                                      sim.PopulationView(GC_population, range(GC_medium_index, GC_medium_index + 20)),
                                      sim.AllToAllConnector(),
                                      sim.StaticSynapse(delay=delay_distr, weight=weight_distr_MF))

		if((GC_medium_index + 20) < GC_upper_index):
			GO_GC_con3 = sim.Projection(sim.PopulationView(GOC_population, range(i, i+1)),
                                      sim.PopulationView(GC_population, range(GC_medium_index + 20, GC_upper_index)),
                                      sim.AllToAllConnector(),
                                      sim.StaticSynapse(delay=delay_distr, weight=weight_distr_GO))


	# Create PC population (THIS MODEL HAS BEEN DEFINED IN THE CEREBELLUMMODULE PACKAGE: https://github.com/jgarridoalcazar/SpikingCerebellum/)
	pc_neuron = sim.native_cell_type('iaf_cond_exp_cs')
	PC_population = sim.Population(num_PC_neurons,pc_neuron(**PC_PARAMS),label='PCLayer')

	# Create VN population (THIS MODEL HAS BEEN DEFINED IN THE CEREBELLUMMODULE PACKAGE: https://github.com/jgarridoalcazar/SpikingCerebellum/)
	vn_neuron = sim.native_cell_type('iaf_cond_exp_cos')
	VN_population = sim.Population(num_VN_neurons,vn_neuron(**VN_PARAMS),label='VNLayer')

	# Create IO population
	IO_population = sim.Population(num_IO_neurons,parrot_neuron,{},label='IOLayer')



	# Create MF-VN learning rule (THIS MODEL HAS BEEN DEFINED IN THE CEREBELLUMMODULE PACKAGE: https://github.com/jgarridoalcazar/SpikingCerebellum/)
	stdp_cos = sim.native_synapse_type('stdp_cos_synapse')(**{'weight':mf_vn_weights,
                                                              'delay':1.0,
                                                              'exponent': 2.0,
                                                              'tau_cos': 5.0,
                                                              'A_plus': 0.0000009,
                                                              'A_minus': 0.00001,
                                                              'Wmin': 0.0005,
                                                              'Wmax': 0.007})

    	# Create MF-VN connections
	mf_vn_connections = sim.Projection(MF_population,
					VN_population,
					sim.AllToAllConnector(),
					receptor_type='AMPA',
	#				synapse_type = sim.StaticSynapse(delay=1.0, weight=0.007))
					synapse_type = stdp_cos)



	# Create PC-VN connections
	pc_vn_connections = sim.Projection(PC_population,
                                       VN_population,
                                       sim.OneToOneConnector(),
                                       receptor_type='GABA',
                                       synapse_type = sim.StaticSynapse(delay=1.0, weight=pc_vn_weights))

	# This second synapse with "receptor_type=TEACHING_SIGNAL" propagates the learning signals that drive the plasticity mechanisms in MF-VN synapses
	pc_vn_connections = sim.Projection(PC_population,
                                       VN_population,
                                       sim.OneToOneConnector(),
                                       receptor_type='TEACHING_SIGNAL',
                                       synapse_type = sim.StaticSynapse(delay=1.0, weight=0.0))




	
	# Create MF-VN learning rule (THIS MODEL HAS BEEN DEFINED IN THE CEREBELLUMMODULE PACKAGE: https://github.com/jgarridoalcazar/SpikingCerebellum/)
	stdp_syn = sim.native_synapse_type('stdp_sin_synapse')(**{'weight':gc_pc_weights,
                                                              'delay':1.0,
                                                              'exponent': 10,
                                                              'peak': 100.0,
                                                              'A_plus': 0.000014,
                                                              'A_minus': 0.00008,
                                                              'Wmin': 0.000,
                                                              'Wmax': 0.010})
      
	# Create GC-PC connections
	gc_pc_connections = sim.Projection(GC_population,
					PC_population,
					sim.AllToAllConnector(),
					receptor_type='AMPA',
	#				synapse_type = sim.StaticSynapse(delay=1.0, weight=0.000))
					synapse_type = stdp_syn)


	# Create IO-PC connections. This synapse with "receptor_type=COMPLEX_SPIKE" propagates the learning signals that drive the plasticity mechanisms in GC-PC synapses
	io_pc_connections = sim.Projection(IO_population,
                                       PC_population,
                                       sim.OneToOneConnector(),
                                       receptor_type='COMPLEX_SPIKE',
                                       synapse_type = sim.StaticSynapse(delay=1.0, weight=io_pc_weights)) 




	# Group all neural layers
	population = MF_population + GOC_population + GC_population + PC_population + VN_population + IO_population 

	# Set Vm to resting potential
	# sim.initialize(PC_population, V_m=PC_population.get('E_L'))
	# sim.initialize(VN_population, V_m=VN_population.get('E_L'))
	
	return population


circuit = create_brain()
