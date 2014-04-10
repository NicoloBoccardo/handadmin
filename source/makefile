# compiler
COMPILER = gcc

# flags
#CFLAGS = -c -Wall
CFLAGS = -c
LMFLAGS = -lm
# PFLAGS = -framework Phidget21 -I /Library/Frameworks/Phidget21.framework/Headers

# files
#SOURCES = qbmove.c qbmoveadmin.c qbinit.c qbtest.c qb_pos_stiff_demo.c qbbackup.c


all:qbmove qbmoveadmin qbparam

    
qbmove:objs/qbmove.o objs/qbmove_communications.o bin
	$(COMPILER) $(PFLAGS) objs/qbmove.o objs/qbmove_communications.o -o bin/qbmove $(LMFLAGS)

qbmoveadmin:objs/qbmoveadmin.o objs/qbmove_communications.o bin
	$(COMPILER) objs/qbmoveadmin.o objs/qbmove_communications.o -o bin/qbmoveadmin $(LMFLAGS)

qbparam:objs/qbparam.o objs/qbmove_communications.o bin
	$(COMPILER) objs/qbparam.o objs/qbmove_communications.o -o bin/qbparam $(LMFLAGS)



objs/qbmove_communications.o:qbmoveAPI/qbmove_communications.c objs
	$(COMPILER) $(CFLAGS) qbmoveAPI/qbmove_communications.c -o objs/qbmove_communications.o

objs/qbmove.o:qbmove.c objs
	$(COMPILER) $(CFLAGS) $(PFLAGS) qbmove.c -o objs/qbmove.o 

objs/qbmoveadmin.o:qbmoveadmin.c objs
	$(COMPILER) $(CFLAGS) qbmoveadmin.c -o objs/qbmoveadmin.o

objs/qbparam.o:qbparam.c objs
	$(COMPILER) $(CFLAGS) qbparam.c -o objs/qbparam.o



clean:
ifeq "$(OS)"  "Windows_NT"
	erase /Q /S objs
	erase /Q /S bin
else
	rm -rf *.o bin objs
endif

objs:
	mkdir objs

bin:
	mkdir bin
