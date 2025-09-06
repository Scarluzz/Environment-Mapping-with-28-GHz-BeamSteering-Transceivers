SRC := $(filter-out rfnoc_copy.cpp, $(wildcard *.cpp))
OBJ=$(basename $(SRC))
CXXFLAGS=-std=gnu++11 -Wall -Wextra -O2
LDLIBS=-luhd -lboost_program_options -lboost_thread -lserial -lpthread -lboost_thread -lboost_system
LDLIBS += -ltlkcore_lib -lboost_filesystem
.PHONY: all clean
all: $(OBJ)
clean:
	rm -f $(OBJ)
