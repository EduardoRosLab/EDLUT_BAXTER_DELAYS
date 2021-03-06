################################################################################
########################### - MAKEFILE FLAGS - #################################
################################################################################

CCFLAGS = -I$(includedir) -fPIC
CXXFLAGS = -I$(includedir) -fPIC
NVCCFLAGS = -Xcompiler -fPIC
LDFLAGS = -lm

ifeq ($(optimize),true)
  CCFLAGS += -Wall -O3 -DHAVE_INLINE
  CXXFLAGS += -Wall -O3 -DHAVE_INLINE
else
  CCFLAGS += -g -Wall
  CXXFLAGS += -g -Wall
  NVCCFLAGS	+= -g -G
endif

OPENMP_FLAGS = -fopenmp

CCFLAGS += $(OPENMP_FLAGS)
CXXFLAGS += $(OPENMP_FLAGS)
LDFLAGS  += $(OPENMP_FLAGS)
NVCCFLAGS += -Xcompiler $(OPENMP_FLAGS)

ifeq ($(profile),true)
  LDFLAGS+= -lprofiler
endif

ARCH 		:= $(shell getconf LONG_BIT)
UNAME_S 	:= $(shell uname -s)

ifeq ($(OS),Windows_NT)
    OS_SYSTEM := WINNT
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        OS_SYSTEM := LINUX
    endif
    ifeq ($(UNAME_S),Darwin)
        OS_SYSTEM := OSX
    endif
endif


ifeq ($(cuda_enabled),true)
  CCFLAGS	+= -I$(cudarootdir)/include -I$(cudarootdir)/samples/common/inc/
  CXXFLAGS	+= -I$(cudarootdir)/include -I$(cudarootdir)/samples/common/inc/
  NVCCFLAGS	+= -I$(cudarootdir)/include -I$(cudarootdir)/samples/common/inc/ -arch='sm_$(cudamajor)$(cudaminor)' -m=$(ARCH)
  LDFLAGS 	+= -L$(cudalib) -lcudart
endif

CCFLAGS += -fno-strict-aliasing
CXXFLAGS += -fno-strict-aliasing

ifeq ($(matlabsupport),true)
  CCFLAGS	+= -I$(matlabinclude) -DMATLAB_MEX_FILE
  CXXFLAGS	+= -I$(matlabinclude) -DMATLAB_MEX_FILE
  MEXFLAGS	:= -cxx CC='$(ccompiler)' CXX='$(compiler)' LD='$(compiler)'
  ifeq ($(OS_SYSTEM),OSX)
  	## Adding -undefined dynamic_loop -bundle to fix the MAC OS X bug as reported in http://www.mathworks.com/matlabcentral/newsreader/view_thread/299757
  	MEXFLAGS	+= LDFLAGS='$(LDFLAGS) -undefined dynamic_lookup -bundle'
  else
  	MEXFLAGS	+= LDFLAGS='$(LDFLAGS)'
  endif
  
  ifeq ($(simulinksupport),true)
  	CCFLAGS	+= -I$(simulinkinclude)
  	CXXFLAGS	+= -I$(simulinkinclude)
  endif
endif

ifeq ($(generate_robot),true)
  ifeq ($(OS_SYSTEM),OSX)
    # MAC OS X 64 bits architecture 
    LDFLAGS	+= -L$(matlabrootdir)/bin/maci64
  else
  	ifeq ($(OS_SYSTEM),LINUX)
  	  # LINUX 64 bits architecture 
  	  LDFLAGS	+= -L$(matlabrootdir)/bin/glnxa64 -lrt
  	else
  	  # WINNT 32 and 64 bits architecture
  	  LDFLAGS += -L$(matlabrootdir)/bin/win$(ARCH)
  	endif
  endif
  LDFLAGS	+= -lmat -lmx -lmex
endif
  	
  	
  	
