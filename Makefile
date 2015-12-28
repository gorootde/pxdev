.PHONY: all clean install


pixtend/pixtend.so: LDLIBS = -lwiringPi
pixtend/pixtend.o: CFLAGS = -fpic
pixtendtool/pixtendtool: LDLIBS = -lpixtend -lwiringPi
pxauto/pxauto: LDLIBS = -lmenu -lform -lpanel -lncurses -lrt -lpixtend -lwiringPi

all: pixtend/pixtend.a pixtend/pixtend.so pixtendtool/pixtendtool pxauto/pxauto

clean:
	rm -rf pixtend/*.o pixtend/*.so pixtend/*.a pixtendtool/*.a pixtendtool/pixtendtool pxauto/pxauto pxauto/*.o

install: pixtend/pixtend.so
	cp pixtend/pixtend.so /usr/lib/libpixtend.so
	
%.o: %.c
	$(CC) $(LDLIBS) $(CFLAGS) -c $< -o $@

%.a: %.o
	$(AR) crs $@ $<

%.so: %.o
	$(CC) $(LDLIBS) -shared $< -o $@