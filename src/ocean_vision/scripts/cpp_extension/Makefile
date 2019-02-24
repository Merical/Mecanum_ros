CC := g++
CFLAGS := -std=c++11 -std=gnu++11

main: hc_sensor.cpp
	$(CC) hc_sensor.cpp jetsonGPIO.c -o hc_sensor.so -shared -fPIC
	

clean:
	rm hc_sensor.so
