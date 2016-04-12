CFLAGS = -O

VC_TOP     = ..

INCLUDE_PATH = -I. -I$(VC_TOP)/RAPID

OBJS = NBody.o PairData.o VInternal.o VCollide.o

TARGET = libVCollide.a

$(TARGET) : $(OBJS) 
	ar ruv $(TARGET) $(OBJS)

.C.o :
	$(CC) $(CFLAGS) $(INCLUDE_PATH) -c $<

install: $(TARGET)
	mv $(TARGET) $(VC_TOP)/lib
	cp VCollide.H $(VC_TOP)/include

clean: 
	rm -f *.o $(TARGET)


