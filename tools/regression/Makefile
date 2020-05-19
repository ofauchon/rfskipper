# Recursively get all *.cpp in this directory and any sub-directories

SRCDIR = ../../src/decoder.cpp ../../src/utils.cpp ../../src/plugin.cpp ../../src/plugin ./src
SOURCES = $(shell find ${SRCDIR} -name *.c -o -name *.cpp | grep -v somfy)


OBJECTS := $(filter %.o,$(SOURCES:.c=.o))
OBJECTS += $(filter %.o,$(SOURCES:.cpp=.o))

FLAGS = -I ../../include -I include -DWITH_REGRESSION=1

#Tells make your binary is called artifact_name_here and it should be in bin/
BIN = regression


all: $(BIN)

#$(BIN) is dependent on objects
$(BIN): $(OBJECTS)
	g++ $(FLAGS) $(OBJECTS) -o $@ 

#each object file is dependent on its source file, and whenever make needs to create
# an object file, to follow this rule:
%.o: %.c 
	g++ $(FLAGS) -c  $< -o $@

%.o: %.cpp 
	g++ $(FLAGS) -c  $< -o $@

info:
	@echo $(SOURCES)
	@echo $(OBJECTS)

clean:
	rm -v $(OBJECTS)
