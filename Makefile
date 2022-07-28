# Get list of .cpp files to compile
SRCS = $(wildcard *.cpp) 
SRCS += $(wildcard src/**/.cpp include/**/*.cpp) 
SRCS += $(wildcard include/**/*.cpp) 
SRCS += $(wildcard firmware_headers/**/*.cpp)

# Get list of .o (object) files to link
OBJS = $(patsubst %.cpp,%.o,$(SRCS))

# Compiler flags
CFLAGS = -fno-strict-aliasing -fwrapv -Wall -Wextra  -Wno-unused-parameter

# Dir where to store build result
dir = api

# Specify paths and "os based" commands
ifeq ($(OS),Windows_NT)
	PATH_PREFIX = C:/arm-gcc/bin/
	COMPILER_PREFIX = arm-evcpp-linux-gnueabi-
	CLEAR_COMMAND = del /Q /F /S  .\src\*.o .\firmware_headers\*.o .\api\*.a .\*.o
endif

COMPILER = $(PATH_PREFIX)$(COMPILER_PREFIX)gcc
AR = $(PATH_PREFIX)$(COMPILER_PREFIX)ar

.DEFAULT: evcppapi.a
$(dir)/evcppapi.a: $(OBJS)
	$(AR) rcs $@ $^

%.o: %.cpp
	$(COMPILER) -Os $(CFLAGS) -c $< -o $@

clean:
	$(CLEAR_COMMAND)