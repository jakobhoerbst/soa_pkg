CC=g++
CFLAGS+=`pkg-config --cflags opencv` -std=c++11
LDFLAGS+=`pkg-config --libs opencv`
PROG=DFS
SRC=DFS
OBJS=$(SRC).o
.PHONY: clean
$(PROG): $(OBJS)
	$(CC) -o $(PROG) $(OBJS) $(LDFLAGS)
%.o: %.cpp
	$(CC) -c $(CFLAGS) $<
clean:
	rm -f $(OBJS) $(PROG) *.out


