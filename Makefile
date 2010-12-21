all: zbik3d

CXXFLAGS?=-Wall -O2 -g

zbik3d: zbik3d.o irp6.o polycrank.o 
	$(CXX) $(LDFLAGS) $^ -lplibpu -lplibfnt -lplibul -lplibnet -lpthread -lglut -lGLU -o $@

clean:
	rm -f *.o zbik3d
