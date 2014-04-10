# compiler
COMPILER = gcc

# flags
#CFLAGS = -c -Wall
CFLAGS = -c
LMFLAGS = -lm
# PFLAGS = -framework Phidget21 -I /Library/Frameworks/Phidget21.framework/Headers



all:handmove handmoveadmin handparam

    
handmove:objs/handmove.o objs/qbmove_communications.o bin
	$(COMPILER) $(PFLAGS) objs/handmove.o objs/qbmove_communications.o -o bin/handmove $(LMFLAGS)

handmoveadmin:objs/handmoveadmin.o objs/qbmove_communications.o bin
	$(COMPILER) objs/handmoveadmin.o objs/qbmove_communications.o -o bin/handmoveadmin $(LMFLAGS)

handparam:objs/handparam.o objs/qbmove_communications.o bin
	$(COMPILER) objs/handparam.o objs/qbmove_communications.o -o bin/handparam $(LMFLAGS)



objs/qbmove_communications.o:qbmoveAPI/qbmove_communications.c objs
	$(COMPILER) $(CFLAGS) qbmoveAPI/qbmove_communications.c -o objs/qbmove_communications.o

objs/handmove.o:handmove.c objs
	$(COMPILER) $(CFLAGS) $(PFLAGS) handmove.c -o objs/handmove.o 

objs/handmoveadmin.o:handmoveadmin.c objs
	$(COMPILER) $(CFLAGS) handmoveadmin.c -o objs/handmoveadmin.o

objs/handparam.o:handparam.c objs
	$(COMPILER) $(CFLAGS) handparam.c -o objs/handparam.o



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
