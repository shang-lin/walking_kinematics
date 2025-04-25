#CC=g++ -g -Wall
#LFLAGS=-I/opt/X11/include -L/opt/X11/lib -lglut -lGL -lGLU -lXmu -lXext -lXi -lX11 -lm

include include.mk

all: walk

walk: state.h state.cpp walk.cpp
	$(CC) state.cpp walk.cpp -o walk $(INCLUDES) $(LDFLAGS)

clean:
	-rm -f *.o
