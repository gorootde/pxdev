.PHONY: all clean


pixtendtool/pixtendtool: LDLIBS = -lpixtend -lwiringPi
pxauto/pxauto: LDLIBS = -lmenu -lform -lpanel -lncurses -lrt -lpixtend -lwiringPi

all: pixtend/pixtend.a pixtend/pixtend.so pixtendtool/pixtendtool pxauto/pxauto

clean:
	rm -rf pixtend/*.o pixtend/*.so pixtend/*.a pixtendtool/*.a pixtendtool/pixtendtool pxauto/pxauto pxauto/*.o

%.o: %.c
	$(CC) -c -fPIC $< -o $@

%.a: %.o
	$(AR) crs $@ $<

%.so: %.o
	$(CC) $< -shared -o $@