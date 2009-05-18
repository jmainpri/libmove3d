SWIG = swig

make: jerk.so
	cp jerk.so ../tcl/jerk.so

jerk_wrap.c : jerk.i
	$(SWIG) -tcl -o $@ $<

jerk.o : jerk.c jerk.h
	gcc -c -g -fPIC -I /usr/local/tcl-8.4/include -o $@ $<

jerk_wrap.o : jerk_wrap.c jerk.h
	gcc -c -g -fPIC -I /usr/local/tcl-8.4/include -o $@ $<

jerk.so : jerk.o jerk_wrap.o
	exec gcc -g -shared -L/home/daniel/mytmp/lib -lBLT24 jerk.o jerk_wrap.o -o $@


clean:
	rm -f *.o jerk_wrap* *.so
