TARGET = BarelangFC

#INCLUDE_DIRS = -I/usr/local/include/cvblobs
INCLUDE_DIRS = -I ../Lib_Barelang-FC/minIni

CXX = g++ -std=c++11
#CXXFLAGS += `pkg-config --cflags opencv` -O2 -DLINUX -DOPENCV -Wall #$(INCLUDE_DIRS)
#CXXFLAGS += -O2 -DDEBUG -DLINUX -Wall $(INCLUDE_DIRS)
CXXFLAGS += -DLINUX -Wall $(INCLUDE_DIRS)
#LFLAGS += -lpthread  -lrt
#LFLAGS += -lpthread -ljpeg -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_objdetect -lrt
LFLAGS += -lpthread -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_objdetect -lrt -lstdc++

#OBJECTS =   main.o BarelangFC-Vision.o BarelangFC-Receiver.o BarelangFC-Strategy.o BarelangFC-GameControllerClient.o BarelangFC-RobotCoordination.o BarelangFC-Odometry.o BarelangFC-Monitor.o
OBJECTS =   main.o BarelangFC-Receiver.o BarelangFC-Strategy.o BarelangFC-GameControllerClient.o BarelangFC-RobotCoordination.o BarelangFC-Odometry.o BarelangFC-Monitor.o
#OBJECTS =   main.o BarelangFC-Vision.o BarelangFC-Strategy.o BarelangFC-GameControllerClient.o BarelangFC-RobotCoordination.o
#OBJECTS =   main.o BarelangFC-Vision.o BarelangFC-Strategy.o BarelangFC-GameControllerClient.o
#OBJECTS =   main.o BarelangFC-Vision.o BarelangFC-Strategy.o BarelangFC-RobotCoordination.o

all: $(TARGET)
clean:
	rm -f *.a *.o $(TARGET) core *~ *.so *.lo

Barelang-FC.a:
	make -C ../Lib_Barelang-FC

$(TARGET): Barelang-FC.a $(OBJECTS)
	$(CXX) $(CFLAGS) $(OBJECTS) ../Lib_Barelang-FC/build/Barelang-FC.a -o $(TARGET) $(LFLAGS)
#	$(CXX) $(CFLAGS) $(OBJECTS) -o $(TARGET) $(LFLAGS)
	chmod 755 $(TARGET)

# useful to make a backup "make tgz"
#tgz: clean
#	mkdir -p backups
#	tar czvf ./backups/camera_`date +"%Y_%m_%d_%H.%M.%S"`.tgz --exclude backups *
