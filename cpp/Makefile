#---------------------------------------------------------------------
# ROBOTIS Makefile template for projects using DXL SDK
#---------------------------------------------------------------------

# *** ENTER THE TARGET NAME HERE ***
TARGET    = main

# important directories used by assorted rules and other variables
DIR_DXL   = dynamixel_sdk
DIR_OBJS  = .objects

# compiler options
CX        = g++
CXFLAGS   = -O2 -O3 -DLINUX -D_GNU_SOURCE -Wall $(INCLUDES) $(FORMAT) -g
FORMAT    = -m64

#---------------------------------------------------------------------
# Core components (all of these are likely going to be needed)
#---------------------------------------------------------------------
INCLUDES  += -I $(DIR_DXL)/include/dynamixel_sdk
INCLUDES  += -I Eigen
INCLUDES  += -I leap_hand_utils/include
LIBRARIES += -ldxl_x64_cpp
LIBRARIES += -lrt

#---------------------------------------------------------------------
# Files
#---------------------------------------------------------------------
SOURCES = $(TARGET).cpp \
		  leap_hand_utils/src/dynamixel_client.cpp \
		  leap_hand_utils/src/leap_hand_utils.cpp

OBJECTS = $(addsuffix .o,$(addprefix $(DIR_OBJS)/,$(basename $(notdir $(SOURCES)))))

#---------------------------------------------------------------------
# Compiling Rules
#---------------------------------------------------------------------
$(TARGET): make_directory $(OBJECTS)
	$(CX) $(CXFLAGS) $(OBJECTS) -o $(TARGET) $(LIBRARIES)

all: $(TARGET)

clean:
	rm -rf $(TARGET) $(DIR_OBJS) core *~ *.a *.so *.lo

make_directory:
	mkdir -p $(DIR_OBJS)/

$(DIR_OBJS)/main.o: main.cpp
	$(CX) -c $? -o $@ $(INCLUDES)

$(DIR_OBJS)/dynamixel_client.o: leap_hand_utils/src/dynamixel_client.cpp
	$(CX) -c $? -o $@ $(INCLUDES)

$(DIR_OBJS)/leap_hand_utils.o: leap_hand_utils/src/leap_hand_utils.cpp
	$(CX) -c $? -o $@ $(INCLUDES)

# $(DIR_OBJS)/%.o: ../src/%.cpp ../main.cpp
# 	$(CX) $(CXFLAGS) -c $? -o $@

#---------------------------------------------------------------------
# End of Makefile
#---------------------------------------------------------------------