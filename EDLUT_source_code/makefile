
# this is the directory the library will be installed if you issue a
# 'make install' command:
# headers to $(installdir)/$(packagename)/include
# library to $(installdir)/lib
# 'make install-exe' command:
# executable to $(installdir)/bin/
installdir  := /usr/local

# this is the name of the package. i.e if it is 'cvlab' the executable
# will be named as 'cvlab' and if this is a library its name will be
# 'libcvlab.a'
packagename := edlutkernel

version     := 2.0

# you can write a short description here about the package
description := EDLUT Kernel Library

# i'm for gpl but you can edit it yourself
license     := GPL v3.0

# the external libraries and sources are managed (included, linked)
# using the pkg-config program. if you don't have it, you cannot use
# this template to include/link libraries. get it from
# http://pkg-config.freedesktop.org/wiki

compiler 	:= g++
ccompiler	:= gcc

# if matlab support -> Matlab paths, includes and flags.
#matlabsupport	:= false
matlabsupport	:= false

# matlab root dir : Folder where Matlab is installed. The root folder
# in order to find the matlab include folder.If you don't need matlab 
# compatibility (mex files), this option won't be used.
#matlabrootdir	:= /Applications/MATLAB_R2013a.app
matlabrootdir	:= 


# matlab include folder : Path where matlab include files are stored.
# If you don't need matlab compatibility (mex files), this option won't
# be used.
#matlabinclude	:= ${matlabrootdir}/extern/include
matlabinclude	:= /extern/include

# if simulink support -> Simulink paths, includes and flags.
#simulinksupport	:= false
simulinksupport	:= false

# simulink include folder : Path where simulink include files are stored.
# If you don't need simulink compatibility (simulink files), this option won't
# be used.
#simulinkinclude	:= ${matlabrootdir}/simulink/include
simulinkinclude	:= /simulink/include

# matlab mex file suffix : Suffix of generated mex files. E.g. mexglx in Linux 32 bits,
# mex64 in Linux 64 bits, mexmac in MAC OS X 32 bits, mexmaci64 in MAC OS X 64 bits, 
# mexw32 in windows 32 bits, mexw64 in windows 64 bits,... See mex documentation for more
# detailed information.
#mex32suffix	:= mexglx
#mex64suffix	:= mexa64
mexsuffix	:= 

# if generate_robot -> robot source codes and library will be builded.
generate_robot    := false

# if optimized -> no debug info is produced --> applies -O3 flag if
# set to true
#optimize := true
optimize    := true


# do you want openmp support ? if you've never heard of it say 'false'
# parallelize := false - Not needed anymore. Just link OPENMP_CFLAGS or OPENMP_CXXFLAGS

# generate profiler data if true.  
#   ! set the optimize = false if you want annotation support.  
#  !! if you don't compile libraries with this flag, profiler won't be 
#      able to make measurements for those libraries.  
# !!! after running your program, you can see the results with
#      'make gflat' and 'make gcall'
profile := false


# If you want to optimize the software to run on GPU with CUDA set: cuda_enable->true
#cuda_enabled := false
cuda_enabled := true

# If you want to optimize the software to run on multicores with OPENMP set: openmp_enabled->true
openmp_enabled := true


# Set the installation path of CUDA (where nvcc is located)
#CUDA_INSTALL_PATH := /usr/local/cuda
cudarootdir := /usr/local/cuda-10.2

cudalib		:= /usr/local/cuda-10.2/lib64

# Set the architecture of CUDA
cudaversion := 7.5
cudamajor 	:= 7
cudaminor	:= 5

# do not change for linux /usr type directory structures. this structure means
# .cpp .cc files reside in ./srcdir and .h files reside in ./include/$(packagename)/
# if you are building a library, the lib$(packagename).a will be in ./lib file.
# libdir      := lib
# srcdir      := src
# includedir  := include
# If you'd like to have everything in the main directory
libdir      	:= ./lib
srcdir      	:= ./src
includedir  	:= ./include
bindir			:= ./bin
mexdir			:= ./mex
sfunctiondir	:= ./sfunction

# default file to be included when the executable build is required
exe-source-file := ${srcdir}/EDLUTKernel.cpp
rtexe-source-file := ${srcdir}/RealTimeEDLUTKernel.cpp
step-source-file := ${srcdir}/StepByStep.cpp
prec-source-file := ${srcdir}/PrecisionTest.cpp
robot-source-file := ${srcdir}/Feedback_control_loop_VOR.cpp


# default file to be included when the mex build is required
mex-source-file := ${srcdir}/EDLUTMatlab.cpp

# default file to be included when the sfunction build is required
sfunction-source-file := ${srcdir}/EDLUTSfunction.cpp

# what to compile ? include .cpp and .c files here in your project
# if you don't have a main() function in one of the sources, you'll get an error
# if you're building an executable. for a library, it won't complain for anything.
communication-sources	:= $(srcdir)/communication/ArrayInputCurrentDriver.cpp \
			$(srcdir)/communication/ArrayInputSpikeDriver.cpp \
			$(srcdir)/communication/ArrayOutputSpikeDriver.cpp \
			$(srcdir)/communication/CdSocket.cpp \
			$(srcdir)/communication/ClientSocket.cpp \
			$(srcdir)/communication/CommunicationDevice.cpp \
			$(srcdir)/communication/ConnectionException.cpp \
			$(srcdir)/communication/FileInputCurrentDriver.cpp \
			$(srcdir)/communication/FileInputSpikeDriver.cpp \
			$(srcdir)/communication/FileOutputSpikeDriver.cpp \
			$(srcdir)/communication/FileOutputWeightDriver.cpp \
			$(srcdir)/communication/InputBooleanArrayDriver.cpp \
			$(srcdir)/communication/InputCurrentDriver.cpp \
			$(srcdir)/communication/InputSpikeDriver.cpp \
			$(srcdir)/communication/OutputBooleanArrayDriver.cpp \
			$(srcdir)/communication/OutputSpikeDriver.cpp \
			$(srcdir)/communication/OutputWeightDriver.cpp \
			$(srcdir)/communication/ServerSocket.cpp \
			$(srcdir)/communication/TCPIPInputOutputSpikeDriver.cpp \
			$(srcdir)/communication/TCPIPInputSpikeDriver.cpp \
			$(srcdir)/communication/TCPIPOutputSpikeDriver.cpp


integration-sources	:=	$(srcdir)/integration_method/BiFixedStep.cpp \
			$(srcdir)/integration_method/FixedStep.cpp \
			$(srcdir)/integration_method/FixedStepSRM.cpp \
			$(srcdir)/integration_method/IntegrationMethod.cpp
			
ifeq ($(cuda_enabled),true)
integration-sources	+= $(srcdir)/integration_method/BDFn_GPU.cu \
			$(srcdir)/integration_method/Bifixed_Euler_GPU.cu \
			$(srcdir)/integration_method/Bifixed_RK2_GPU.cu \
			$(srcdir)/integration_method/Bifixed_RK4_GPU.cu \
			$(srcdir)/integration_method/BiFixedStep_GPU.cu \
			$(srcdir)/integration_method/Euler_GPU.cu \
			$(srcdir)/integration_method/FixedStep_GPU.cu \
			$(srcdir)/integration_method/IntegrationMethod_GPU.cu \
			$(srcdir)/integration_method/RK2_GPU.cu \
			$(srcdir)/integration_method/RK4_GPU.cu
endif
			   

interface-sources	:= $(srcdir)/interface/C_Interface.cpp \
			$(srcdir)/interface/C_interface_for_robot_control.cpp

learning_rules-sources	:=	$(srcdir)/learning_rules/AdditiveKernelChange.cpp \
			$(srcdir)/learning_rules/BufferedActivityTimes.cpp \
			$(srcdir)/learning_rules/ConnectionState.cpp \
			$(srcdir)/learning_rules/CosState.cpp \
			$(srcdir)/learning_rules/CosWeightChange.cpp \
			$(srcdir)/learning_rules/ExpState.cpp \
			$(srcdir)/learning_rules/ExpWeightChange.cpp \
			$(srcdir)/learning_rules/ExpOptimisedBufferedWeightChange.cpp \
			$(srcdir)/learning_rules/LearningRule.cpp \
			$(srcdir)/learning_rules/SimetricCosBufferedWeightChange.cpp \
			$(srcdir)/learning_rules/SimetricCosSinState.cpp \
			$(srcdir)/learning_rules/SimetricCosSinSTDPState.cpp \
			$(srcdir)/learning_rules/SimetricCosSinSTDPWeightChange.cpp \
			$(srcdir)/learning_rules/SimetricCosSinWeightChange.cpp \
			$(srcdir)/learning_rules/SimetricCosState.cpp \
			$(srcdir)/learning_rules/SimetricCosSTDPState.cpp \
			$(srcdir)/learning_rules/SimetricCosSTDPWeightChange.cpp \
			$(srcdir)/learning_rules/SimetricCosWeightChange.cpp \
			$(srcdir)/learning_rules/SinBufferedWeightChange.cpp \
			$(srcdir)/learning_rules/SinOptimisedBufferedWeightChange.cpp \
			$(srcdir)/learning_rules/SinState.cpp \
			$(srcdir)/learning_rules/SinWeightChange.cpp \
			$(srcdir)/learning_rules/STDPLSState.cpp \
			$(srcdir)/learning_rules/STDPLSWeightChange.cpp \
			$(srcdir)/learning_rules/STDPState.cpp \
			$(srcdir)/learning_rules/STDPWeightChange.cpp \
			$(srcdir)/learning_rules/WithoutPostSynaptic.cpp \
			$(srcdir)/learning_rules/WithPostSynaptic.cpp
 

neuron_model-sources	:= $(srcdir)/neuron_model/AdExTimeDrivenModel.cpp \
			$(srcdir)/neuron_model/CompressNeuronModelTable.cpp \
			$(srcdir)/neuron_model/CompressSynchronousTableBasedModel.cpp \
			$(srcdir)/neuron_model/CompressTableBasedModel.cpp \
			$(srcdir)/neuron_model/CurrentSynapseModel.cpp \
			$(srcdir)/neuron_model/EgidioGranuleCell_TimeDriven.cpp \
			$(srcdir)/neuron_model/EventDrivenInputDevice.cpp \
			$(srcdir)/neuron_model/EventDrivenNeuronModel.cpp \
			$(srcdir)/neuron_model/HHTimeDrivenModel.cpp \
			$(srcdir)/neuron_model/InputCurrentNeuronModel.cpp \
			$(srcdir)/neuron_model/InputSpikeNeuronModel.cpp \
			$(srcdir)/neuron_model/IzhikevichTimeDrivenModel.cpp \
			$(srcdir)/neuron_model/Lethargic_LIFTimeDrivenModel_1_1.cpp \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_1.cpp \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_2.cpp \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_2_NEW.cpp \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_3.cpp \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_3_NEW.cpp \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_4.cpp \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_4_NEW.cpp \
			$(srcdir)/neuron_model/NeuronModel.cpp \
			$(srcdir)/neuron_model/NeuronModelTable.cpp \
			$(srcdir)/neuron_model/NeuronState.cpp \
			$(srcdir)/neuron_model/OriginalHHTimeDrivenModel.cpp \
			$(srcdir)/neuron_model/SRMTableBasedModel.cpp \
			$(srcdir)/neuron_model/SRMTimeDrivenModel.cpp \
			$(srcdir)/neuron_model/SynchronousTableBasedModel.cpp \
			$(srcdir)/neuron_model/TableBasedModel.cpp \
			$(srcdir)/neuron_model/TimeDrivenInferiorOliveCell.cpp \
			$(srcdir)/neuron_model/TimeDrivenInputDevice.cpp \
			$(srcdir)/neuron_model/TimeDrivenModel.cpp \
			$(srcdir)/neuron_model/TimeDrivenNeuronModel.cpp \
			$(srcdir)/neuron_model/TimeDrivenPurkinjeCell.cpp \
			$(srcdir)/neuron_model/TimeDrivenPurkinjeCell_IP.cpp \
			$(srcdir)/neuron_model/TimeDrivenSinCurrentGenerator.cpp \
			$(srcdir)/neuron_model/VectorBufferedState.cpp \
			$(srcdir)/neuron_model/VectorNeuronState.cpp \
			$(srcdir)/neuron_model/VectorSRMState.cpp
ifeq ($(cuda_enabled),true)
neuron_model-sources	+= $(srcdir)/neuron_model/AdExTimeDrivenModel_GPU.cu \
			$(srcdir)/neuron_model/EgidioGranuleCell_TimeDriven_GPU.cu \
			$(srcdir)/neuron_model/HHTimeDrivenModel_GPU.cu \
			$(srcdir)/neuron_model/IzhikevichTimeDrivenModel_GPU.cu \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_1_GPU.cu \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_2_GPU.cu \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_2_GPU_NEW.cu \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_3_GPU.cu \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_4_GPU.cu \
			$(srcdir)/neuron_model/LIFTimeDrivenModel_1_4_GPU_NEW.cu \
			$(srcdir)/neuron_model/TimeDrivenNeuronModel_GPU.cu \
			$(srcdir)/neuron_model/TimeDrivenPurkinjeCell_GPU.cu \
			$(srcdir)/neuron_model/VectorNeuronState_GPU.cu
endif


simulation-sources	:= $(srcdir)/simulation/CommunicationEvent.cpp \
			$(srcdir)/simulation/EndSimulationEvent.cpp \
			$(srcdir)/simulation/Event.cpp \
			$(srcdir)/simulation/EventQueue.cpp \
			$(srcdir)/simulation/ExponentialTable.cpp \
			$(srcdir)/simulation/MediananFilter.cpp \
			$(srcdir)/simulation/ParameterException.cpp \
			$(srcdir)/simulation/ParamReader.cpp \
			$(srcdir)/simulation/RandomGenerator.cpp \
			$(srcdir)/simulation/RealTimeRestriction.cpp \
			$(srcdir)/simulation/SaveWeightsEvent.cpp \
			$(srcdir)/simulation/StopSimulationEvent.cpp \
			$(srcdir)/simulation/SynchronizeActivityEvent.cpp \
			$(srcdir)/simulation/SynchronizeSimulationEvent.cpp \
			$(srcdir)/simulation/TimeEventUpdateNeurons.cpp \
			$(srcdir)/simulation/TrigonometricTable.cpp \
			$(srcdir)/simulation/Utils.cpp 
ifeq ($(cuda_enabled),true)
simulation-sources	+= $(srcdir)/simulation/Simulation_GPU.cu 
else
simulation-sources	+= $(srcdir)/simulation/Simulation.cpp
endif


spike-sources		:= $(srcdir)/spike/Current.cpp \
			$(srcdir)/spike/EDLUTException.cpp \
			$(srcdir)/spike/EDLUTFileException.cpp \
			$(srcdir)/spike/EndRefractoryPeriodEvent.cpp \
			$(srcdir)/spike/InputCurrent.cpp \
			$(srcdir)/spike/InputSpike.cpp \
			$(srcdir)/spike/Interconnection.cpp \
			$(srcdir)/spike/InternalSpike.cpp \
			$(srcdir)/spike/Neuron.cpp \
			$(srcdir)/spike/NeuronModelPropagationDelayStructure.cpp \
			$(srcdir)/spike/NeuronPropagationDelayStructure.cpp \
			$(srcdir)/spike/PropagatedCurrent.cpp \
			$(srcdir)/spike/PropagatedSpike.cpp \
			$(srcdir)/spike/PropagatedSpikeGroup.cpp \
			$(srcdir)/spike/Spike.cpp \
			$(srcdir)/spike/SynchronousTableBasedModelEvent.cpp \
			$(srcdir)/spike/SynchronousTableBasedModelInternalSpike.cpp \
			$(srcdir)/spike/TimeDrivenInternalSpike.cpp 
ifeq ($(cuda_enabled),true)
spike-sources		+= $(srcdir)/spike/Network_GPU.cu
else
spike-sources		+= $(srcdir)/spike/Network.cpp
endif

robot-arm-sources	:= $(srcdir)/arm_robot_simulator/ArmRobotSimulation.c \
			$(srcdir)/arm_robot_simulator/MatrixOperations.c \
			$(srcdir)/arm_robot_simulator/RobotRecursiveInverseDynamics.c 


ifeq ($(cuda_enabled),true)
openmp-sources		:= $(srcdir)/openmp/openmp_GPU.cu
else
openmp-sources		:= $(srcdir)/openmp/openmp.cpp
endif

vor-model-sources	:= $(srcdir)/vor_model/CommunicationAvatarVOR.cpp \
			$(srcdir)/vor_model/EntrySignal_VOR.cpp \
			$(srcdir)/vor_model/Individual_VOR.cpp \
			$(srcdir)/vor_model/RK4_VOR.cpp \





sources			:= $(communication-sources) \
			$(integration-sources) \
			$(interface-sources) \
			$(neuron_model-sources) \
			$(learning_rules-sources) \
			$(simulation-sources) \
			$(spike-sources)\
			$(openmp-sources)\
			$(vor-model-sources)
			
ifeq ($(generate_robot),true)
sources		+= $(robot-arm-sources)
endif




               

################################################################################
####################### LOAD PRESET SETTINGS ###################################
################################################################################

# these are the magic files that this interface depends.

# some temp operations.
include static-variables.makefile

# flag settings for gcc like CXXFLAGS, LDFLAGS...  to see the active
# flag definitions, issue 'make flags' command
include flags.makefile

# rules are defined here. to see a list of the available rules, issue 'make rules'
# command
include rules.makefile
