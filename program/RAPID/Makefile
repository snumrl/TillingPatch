CFLAGS		= -O -I. -I..

LDFLAGS		= -L. -L..

.SUFFIXES: .C

SRCS		= RAPID.C build.C collide.C overlap.C 

OBJECTS		= RAPID.o build.o collide.o overlap.o 

CLEAN		= $(OBJECTS) *.a *.o *~ sample_client

default: library test

install: libRAPID.a
library: libRAPID.a

libRAPID.a: $(OBJECTS)
	/bin/rm -f libRAPID.a
	ar ruv libRAPID.a $(OBJECTS)

.C.o:
	$(CC) ${CFLAGS} -c $<

.c.o:
	$(CC) ${CFLAGS} -c $<

test:
	$(CC) -o sample_client sample_client.C -L. -lRAPID -lm

clean:
	rm -f $(CLEAN)

box.o: matvec.H
