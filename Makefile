# platform based constants and comands
ifeq ($(OS),Windows_NT)
	PATH_PREFIX = C:/arm-gcc/bin/
	COMPILER_PREFIX = arm-evcpp-linux-gnueabi-
	CLEAN_COMMAND = del /Q /F /S  .\src\*.o .\firmware_headers\*.o .\api\*.a .\*.o
endif

COMPILER = $(PATH_PREFIX)$(COMPILER_PREFIX)g++
LINKER = $(PATH_PREFIX)$(COMPILER_PREFIX)ar

SRCS = $(wildcard *.cpp) 
SRCS += $(wildcard src/**/*.cpp include/**/*.cpp src/*.cpp) 
SRCS += $(wildcard include/**/*.cpp) 
SRCS += $(wildcard firmware_headers/**/*.cpp)

OBJS = $(patsubst %.cpp,%.o,$(SRCS))

COMPILER_FLAGS = -fno-strict-aliasing -fwrapv -Wall -Wextra  -Wno-unused-parameter

# CFLAGS += -Wpointer-sign
# CFLAGS += -fpermissive

.DEFAULT: evcppapi.a
api/evcppapi.a: $(OBJS)
	$(LINKER) rcs $@ $^

%.o: %.cpp
	$(COMPILER) -Os $(COMPILER_FLAGS) -c $< -o $@

.PHONY: clean
clean:
	$(CLEAN_COMMAND)
