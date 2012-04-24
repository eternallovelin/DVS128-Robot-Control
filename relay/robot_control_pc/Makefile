# This makefile is used for the main relay module

TARGET		= main

INCLUDEDIRS	+= -I../include
LIBDIRS		+= -L../lib
ODIR		= bin
SDIR		= src

_OBJS		= main.o tcp_client.o
OBJS		= $(patsubst %,$(ODIR)/%,$(_OBJS))

_SRCS		= $(OBJS:.o=.c)
SRCS		= $(patsubst %,$(SDIR)/%,$(_SRCS))

CFLAGS		= $(INCLUDEDIRS) -W -Wall -O2
LDFLAGS		= $(LIBDIRS) -lm -lzgb

CC			= gcc

all:	$(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	
$(ODIR)/%.o: $(SDIR)/%.c
	$(CC) -c -o $@ $< $(CFLAGS)
	
clean:
	rm -f $(OBJS) $(TARGET)
	@echo "file deleted."

dep:
	gccmakedep $(SRCS)

