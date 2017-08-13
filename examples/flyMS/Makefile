# ------------------------------------------------
# Generic Makefile
#
# Author: yanick.rochon@gmail.com
# Date  : 2011-08-10
#
# Changelog :
#   2010-11-05 - first version
#   2011-08-10 - added structure : sources, objects, binaries
#                thanks to http://stackoverflow.com/users/128940/beta
# ------------------------------------------------

# project name (generate executable with this name)
TARGET   = flyMS

CC       = g++
# compiling flags here
CFLAGS	:= -c -Wall -g  
LFLAGS	:= -lm -lrt -lpthread -lroboticscape
# linking flags here
LINKER   = g++ -o

INSTALL_DIR = /usr/bin/


# change these to proper directories where each file should be
SRCDIR   = src
INCDIR   = include
OBJDIR   = obj
BINDIR   = bin

SOURCES  := $(wildcard $(SRCDIR)/*.cxx)
INCLUDES := $(wildcard $(INCDIR)/*.h)
OBJECTS  := $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

SOURCESCPP := $(wildcard $(SRCDIR)/*.cpp)
INCLUDESCPP := $(wildcard $(INCDIR)/*.h)
OBJECTSCPP  := $(SOURCESCPP:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

rm       = rm -f


$(BINDIR)/$(TARGET): $(OBJECTS)
	@$(LINKER)	$@	$(LFLAGS)	$(OBJECTS)
	@echo "Linking complete!"

$(OBJECTS): $(OBJDIR)/%.o:$(SRCDIR)/%.c
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo "Compiled "$<" successfully!"


$(OBJECTSCPP): $(OBJDIR)/%.o:$(SRCDIR)/%.c
	@$(CC) $(CFLAGS) -c $< -o $@
	@echo "Compiled "$<" successfully!"

	
install: $(all)
	@$(MAKE)
	@install -m 0755 $(BINDIR)/$(TARGET) $(INSTALL_DIR)
	@echo
	@echo "Project "$(BINDIR)/$(TARGET)" installed to $(INSTALL_DIR)"
	@echo

.PHONY: clean
clean:
	@$(rm) $(OBJECTS)
	@$(rm) $(BINDIR)/$(TARGET)
	@echo "Cleanup complete!"

.PHONY: remove
remove: clean
	@$(rm) $(BINDIR)/$(TARGET)
	@echo "Executable removed!"