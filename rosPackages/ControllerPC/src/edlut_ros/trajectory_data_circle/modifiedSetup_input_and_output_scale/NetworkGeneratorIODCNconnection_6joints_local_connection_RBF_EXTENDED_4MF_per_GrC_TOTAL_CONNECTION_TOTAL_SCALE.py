#!/usr/bin/env python

def create_network():
    import numpy
    import os

    #networkfile = os.path.expanduser('/home/baxter/NetworkGeneratorDelays/Network_6_joints_10MF_per_RBF_4MF_per_GrC_100DCN.cfg')
    #weightfile = os.path.expanduser('/home/baxter/NetworkGeneratorDelays/Weights_6_joints_10MF_per_RBF_4MF_per_GrC_100DCN.cfg')
    networkfile = os.path.expanduser('/home/portatil-linux/Desktop/modifiedSetup_input_and_output_scale/Network_12_120.cfg')
    weightfile = os.path.expanduser('/home/portatil-linux/Desktop/modifiedSetup_input_and_output_scale/Weights_12_120.cfg')


    num_of_joints = 6

    scale_factor = 1.2

    num_mf_neurons_per_joint = int(4*10*scale_factor)
    num_mf_neurons_per_RBF = num_mf_neurons_per_joint / 4
    num_grc_neurons_per_joint = (num_mf_neurons_per_RBF)*(num_mf_neurons_per_RBF)*(num_mf_neurons_per_RBF)*(num_mf_neurons_per_RBF)
    num_pc_neurons_per_joint = int(2*50*scale_factor)
    num_io_neurons_per_joint = int(2*50*scale_factor)
    num_dcn_neurons_per_joint = int(2*50*scale_factor)



    num_mf_neurons = num_of_joints*num_mf_neurons_per_joint
    num_grc_neurons = num_of_joints*num_grc_neurons_per_joint
    num_pc_neurons = num_of_joints*num_pc_neurons_per_joint
    num_io_neurons = num_of_joints*num_io_neurons_per_joint
    num_dcn_neurons = num_of_joints*num_dcn_neurons_per_joint

    num_of_input_mf_per_grc = 4
    num_of_input_grc_per_pc = num_grc_neurons
    num_of_target_pc_per_io = num_pc_neurons/num_io_neurons
    num_of_input_pc_per_dcn = num_dcn_neurons/num_pc_neurons
    num_of_input_mf_per_dcn = num_mf_neurons
    num_of_target_dcn_per_io = num_dcn_neurons/num_io_neurons

    mfgrc_weights = 0.18 # GrC average firing: 28.41Hz
    mfdcn_weights = 0.1 # Max DCN average firing: 507.66Hz
    grcpc_weights = 2.2 #2.0 Max PC average firing: 279.61Hz
    grcpc_weights_fast = 2.0 #2.0 Max PC average firing: 279.61Hz
    pcdcn_weights = 1.0 # Min DCN average firing: 0.13Hz
    iodcn_AMPA_weights = 0.5
    iodcn_NMDA_weights = 0.25

    grcpc_max_weights = 5.0
    mfgrc_max_weights = 2.0

    grcpc_ltd = -0.0008
    grcpc_ltp = 0.002
    grcpc_peak = 0.150
    grcpc_init = 0.120


    num_mfgrc_connections = num_grc_neurons*num_of_input_mf_per_grc
    num_grcpc_connections = num_pc_neurons*num_of_input_grc_per_pc
    num_iopc_connections = num_io_neurons*num_of_target_pc_per_io
    num_pcdcn_connections = num_dcn_neurons*num_of_input_pc_per_dcn
    num_mfdcn_connections = num_dcn_neurons*num_of_input_mf_per_dcn
    num_iodcn_connections = num_io_neurons*num_of_target_dcn_per_io

    total_connections = num_mfgrc_connections + num_grcpc_connections + num_iopc_connections + num_pcdcn_connections + num_mfdcn_connections + num_iodcn_connections*2
    total_neurons = num_mf_neurons + num_grc_neurons + num_pc_neurons + num_io_neurons + num_dcn_neurons

    print ('Writing network file in',networkfile)
    # Create the network file
    with open(networkfile, 'w') as net_file:
        net_file.write('// Created with python NetworkGenerator function\n\n')

        net_file.write('// NB OF DIFFERENT TYPES OF NEURONS\n')
        net_file.write('4\n\n') # Granular layer and Purkinje neurons

        net_file.write('// TOTAL NUMBER OF NEURONS\n')
        net_file.write(str(total_neurons)+'\n\n')

        net_file.write('// DEFINITION OF LAYERS\n')
        net_file.write('// Mossy Fibers\n')
        net_file.write(str(num_mf_neurons)+' InputNeuronModel Null 0 0\n')
        net_file.write('// Climbing Fibers\n')
        net_file.write(str(num_io_neurons)+' InputNeuronModel Null 0 0\n')
        net_file.write('// Granule cells\n')
        net_file.write(str(num_grc_neurons)+' LIFTimeDrivenModel_1_1_GPU data/_LIFTimeDriven_Granule_1_1_RK2_4MF 1 0\n')
        net_file.write('// Purkinje cells\n')
        net_file.write(str(num_pc_neurons)+' LIFTimeDrivenModel_1_1 data/_LIFTimeDriven_Purkinje_1_1_RK4 1 0\n')
        net_file.write('// DCN cells\n')
        net_file.write(str(num_dcn_neurons)+' LIFTimeDrivenModel_1_4 data/_LIFTimeDriven_DCN_1_4_RK4 1 0\n\n')

        net_file.write('// NUMBER OF LEARNING RULES\n')
        net_file.write(str(num_of_joints)+'\n\n')
        net_file.write('// LEARNING RULES\n')
        net_file.write('ExpOptimisedBufferedAdditiveKernel '+str(grcpc_peak)+' '+str(grcpc_ltp)+' '+str(grcpc_ltd)+' '+str(grcpc_init)+'\n')
        net_file.write('ExpOptimisedBufferedAdditiveKernel '+str(grcpc_peak)+' '+str(grcpc_ltp)+' '+str(grcpc_ltd)+' '+str(grcpc_init)+'\n')
        net_file.write('ExpOptimisedBufferedAdditiveKernel '+str(grcpc_peak)+' '+str(grcpc_ltp)+' '+str(grcpc_ltd)+' '+str(grcpc_init)+'\n')
        net_file.write('ExpOptimisedBufferedAdditiveKernel '+str(grcpc_peak)+' '+str(grcpc_ltp)+' '+str(grcpc_ltd)+' '+str(grcpc_init)+'\n')
        net_file.write('ExpOptimisedBufferedAdditiveKernel '+str(grcpc_peak)+' '+str(grcpc_ltp)+' '+str(grcpc_ltd)+' '+str(grcpc_init)+'\n')
        net_file.write('ExpOptimisedBufferedAdditiveKernel '+str(grcpc_peak)+' '+str(grcpc_ltp)+' '+str(grcpc_ltd)+' '+str(grcpc_init)+'\n')

        net_file.write('// TOTAL NUMBER OF CONNECTIONS\n')
        net_file.write(str(int(total_connections))+'\n\n')

        net_file.write('// DEFINITION OF CONNECTIONS\n')
        net_file.write('// MF-DCN connections\n')
        net_file.write(str(0)+' '+str(num_mf_neurons)+' '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons)+' '+str(num_dcn_neurons)+' 1 0.001 0 0 '+str(mfdcn_weights)+' -1\n')


        net_file.write('// MF-GrC connections\n')
        grc_index = num_mf_neurons + num_io_neurons
        for joint in range (0, num_of_joints):
            for index1 in range (0, num_mf_neurons_per_RBF):
                net_file.write(str(joint*num_mf_neurons_per_joint+index1)+' 1 '+str(grc_index)+' '+str(num_mf_neurons_per_RBF*num_mf_neurons_per_RBF*num_mf_neurons_per_RBF)+' 1 0.001 0 0 '+str(mfgrc_max_weights)+' -1\n')
                for index2 in range (num_mf_neurons_per_RBF, 2*num_mf_neurons_per_RBF):
                    net_file.write(str(joint*num_mf_neurons_per_joint+index2)+' 1 '+str(grc_index)+' '+str(num_mf_neurons_per_RBF*num_mf_neurons_per_RBF)+' 1 0.001 0 0 '+str(mfgrc_max_weights)+' -1\n')
                    for index3 in range (2*num_mf_neurons_per_RBF, 3*num_mf_neurons_per_RBF):
                        net_file.write(str(joint*num_mf_neurons_per_joint+index3)+' 1 '+str(grc_index)+' '+str(num_mf_neurons_per_RBF)+' 1 0.001 0 0 '+str(mfgrc_max_weights)+' -1\n')
                        for index4 in range (3*num_mf_neurons_per_RBF, 4*num_mf_neurons_per_RBF):
                            net_file.write(str(joint*num_mf_neurons_per_joint+index4)+' 1 '+str(grc_index)+' 1 1 0.001 0 0 '+str(mfgrc_max_weights)+' -1\n')
                            grc_index+=1

        net_file.write('// GrC-PC connections\n')
        net_file.write(str(num_mf_neurons+num_io_neurons)+' '+str(num_grc_neurons)+' '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*0)+' '+str(num_pc_neurons_per_joint)+' 1 0.001 0 0 '+str(grcpc_max_weights)+' 0\n')
        net_file.write(str(num_mf_neurons+num_io_neurons)+' '+str(num_grc_neurons)+' '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*1)+' '+str(num_pc_neurons_per_joint)+' 1 0.001 0 0 '+str(grcpc_max_weights)+' 1\n')
        net_file.write(str(num_mf_neurons+num_io_neurons)+' '+str(num_grc_neurons)+' '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*2)+' '+str(num_pc_neurons_per_joint)+' 1 0.001 0 0 '+str(grcpc_max_weights)+' 2\n')
        net_file.write(str(num_mf_neurons+num_io_neurons)+' '+str(num_grc_neurons)+' '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*3)+' '+str(num_pc_neurons_per_joint)+' 1 0.001 0 0 '+str(grcpc_max_weights)+' 3\n')
        net_file.write(str(num_mf_neurons+num_io_neurons)+' '+str(num_grc_neurons)+' '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*4)+' '+str(num_pc_neurons_per_joint)+' 1 0.001 0 0 '+str(grcpc_max_weights)+' 4\n')
        net_file.write(str(num_mf_neurons+num_io_neurons)+' '+str(num_grc_neurons)+' '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*5)+' '+str(num_pc_neurons_per_joint)+' 1 0.001 0 0 '+str(grcpc_max_weights)+' 5\n')


        net_file.write('// IO-PC connections\n')
        net_file.write(str(num_mf_neurons+num_io_neurons_per_joint*0)+' 1 '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*0)+' 1 '+str(num_io_neurons_per_joint)+' 0.001 0 0 0.0 t0\n')
        net_file.write(str(num_mf_neurons+num_io_neurons_per_joint*1)+' 1 '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*1)+' 1 '+str(num_io_neurons_per_joint)+' 0.001 0 0 0.0 t1\n')
        net_file.write(str(num_mf_neurons+num_io_neurons_per_joint*2)+' 1 '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*2)+' 1 '+str(num_io_neurons_per_joint)+' 0.001 0 0 0.0 t2\n')
        net_file.write(str(num_mf_neurons+num_io_neurons_per_joint*3)+' 1 '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*3)+' 1 '+str(num_io_neurons_per_joint)+' 0.001 0 0 0.0 t3\n')
        net_file.write(str(num_mf_neurons+num_io_neurons_per_joint*4)+' 1 '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*4)+' 1 '+str(num_io_neurons_per_joint)+' 0.001 0 0 0.0 t4\n')
        net_file.write(str(num_mf_neurons+num_io_neurons_per_joint*5)+' 1 '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons_per_joint*5)+' 1 '+str(num_io_neurons_per_joint)+' 0.001 0 0 0.0 t5\n')


        net_file.write('// IO-DCN connections\n')
        net_file.write(str(num_mf_neurons)+' 1 '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons)+' 1 '+str(num_io_neurons)+' 0.001 0 0 100.0 -1\n')

        net_file.write('// IO-DCN connections\n')
        net_file.write(str(num_mf_neurons)+' 1 '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons)+' 1 '+str(num_io_neurons)+' 0.001 0 1 100.0 -1\n')

        net_file.write('// PC-DCN connections\n')
        net_file.write(str(num_mf_neurons+num_io_neurons+num_grc_neurons)+' 1 '+str(num_mf_neurons+num_io_neurons+num_grc_neurons+num_pc_neurons)+' 1 '+str(num_pc_neurons)+' 0.001 0 2 5.0 -1\n')


    print ('Network file created')

    print ('Writing weight file in',weightfile)
    # Create the weight file
    with open(weightfile, 'w') as net_file:
        net_file.write('// Created with python create_network function\n\n')

        net_file.write('// MF-DCN connections\n')
        net_file.write(str(int(num_mfdcn_connections))+' '+str(mfdcn_weights)+'\n\n')

        net_file.write('// MF-GrC connections\n')
        net_file.write(str(int(num_mfgrc_connections))+' '+str(mfgrc_weights)+'\n\n')

        net_file.write('// GrC-PC connections\n')
        net_file.write('//'+str(int(num_grcpc_connections))+' '+str(grcpc_weights)+'\n\n')
        net_file.write(str(int(num_grcpc_connections))+' '+str(grcpc_weights_fast)+'\n\n')

        net_file.write('// IO-PC connections\n')
        net_file.write(str(int(num_iopc_connections))+' 0.0\n\n')

        net_file.write('// IO-DCN connections\n')
        net_file.write(str(int(num_iodcn_connections))+' '+str(iodcn_AMPA_weights)+'\n\n')

        net_file.write('// IO-DCN connections\n')
        net_file.write(str(int(num_iodcn_connections))+' '+str(iodcn_NMDA_weights)+'\n\n')

        net_file.write('// PC-DCN connections\n')
        net_file.write(str(int(num_pcdcn_connections))+' '+str(pcdcn_weights)+'\n\n')

    print ('Weight file created')

    return

if __name__ == '__main__':
    create_network()
