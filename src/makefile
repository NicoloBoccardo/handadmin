# compiler
COMPILER = g++

# flags
CFLAGS = -c -Wall
LMFLAGS = -lm

# folders
BIN_FOLDER = ../bin
OBJS_FOLDER = ../objs
LIB_FOLDER = ../../qbAPI/lib



all:handmove handmoveadmin handparam

   
handmove:$(OBJS_FOLDER)/handmove.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/handmove.o      $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/handmove      $(LMFLAGS)

handmoveadmin:$(OBJS_FOLDER)/handmoveadmin.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/handmoveadmin.o $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/handmoveadmin $(LMFLAGS)

handparam:$(OBJS_FOLDER)/handparam.o $(BIN_FOLDER)
	$(COMPILER) $(OBJS_FOLDER)/handparam.o     $(LIB_FOLDER)/libqbmove_comm.a -o $(BIN_FOLDER)/handparam     $(LMFLAGS)




$(OBJS_FOLDER)/handmove.o:handmove.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) $(PFLAGS) handmove.c -o $(OBJS_FOLDER)/handmove.o 

$(OBJS_FOLDER)/handmoveadmin.o:handmoveadmin.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) handmoveadmin.c -o $(OBJS_FOLDER)/handmoveadmin.o

$(OBJS_FOLDER)/handparam.o:handparam.c $(OBJS_FOLDER)
	$(COMPILER) $(CFLAGS) handparam.c -o $(OBJS_FOLDER)/handparam.o



clean:
ifeq "$(OS)"  "Windows_NT"
	erase /Q /S ..\objs
	erase /Q /S ..\bin
else
	rm -rf $(BIN_FOLDER) $(OBJS_FOLDER)
endif

$(OBJS_FOLDER):
	mkdir $(OBJS_FOLDER)

$(BIN_FOLDER):
	mkdir $(BIN_FOLDER)
