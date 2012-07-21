# Relay module makefile

# There are two operation levels for the module: Main and Keyboard
MAIN		= main  # Main level
KEYBOARD	= keyboard # Keyboard level

INCLUDEDIRS	+= -Iinclude
LIBDIRS		+= -Llib

ODIR		= bin
SDIR		= src

_OBJS_MAIN		= main.o zigclient.o
OBJS_MAIN		= $(patsubst %,$(ODIR)/%,$(_OBJS_MAIN))
_SRCS_MAIN		= $(OBJS:.o=.c)
SRCS_MAIN		= $(patsubst %,$(SDIR)/%,$(_SRCS_MAIN))

_OBJS_KEYBOARD		= main.o zigclient.o
OBJS_KEYBOARD		= $(patsubst %,$(ODIR)/%,$(_OBJS_KEYBOARD))
_SRCS_KEYBOARD		= $(OBJS:.o=.c)
SRCS_KEYBOARD		= $(patsubst %,$(SDIR)/%,$(_SRCS_KEYBOARD))


CFLAGS		= $(INCLUDEDIRS) -W -Wall -O2
LDFLAGS		= $(LIBDIRS) -lm -lzgb -lncurses

CC			= gcc

all:	$(MAIN) $(KEYBOARD)

$(MAIN): $(OBJS_MAIN)
	$(CC) -o $@ $^ $(LDFLAGS)
	
$(KEYBOARD): $(OBJS_KEYBOARD)
	$(CC) -o $@ $^ $(LDFLAGS)
	
$(ODIR)/%.o: $(SDIR)/%.c
	$(CC) -c -o $@ $< $(CFLAGS)
	
clean:
	rm -f $(OBJS_MAIN) $(MAIN)
	rm -f $(OBJS_KEYBOARD) $(KEYBOARD)
	@echo "file deleted."

dep:
	gccmakedep $(SRCS_MAIN) $(SRCS_KEYBOARD)

