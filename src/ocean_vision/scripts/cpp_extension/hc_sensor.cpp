#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include "jetsonGPIO.h"
using namespace std;


char dist_message[1024];

double GetTickCount()
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

std::string doubleToString(double num)
{
	char str[256];
	sprintf(str,"%.4lf",num);
	std::string result = str;
	return result;
}


int getkey() {
    int character;
    struct termios orig_term_attr;
    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);

    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}


double checkdist(jetsonTX1GPIONumber pin_out, jetsonTX1GPIONumber pin_in)
{
	//cout << "LCH: Checkdist start!" << endl;
	unsigned int in_value = low;
	gpioSetValue(pin_out, high);
	usleep(15);
	gpioSetValue(pin_out, low);
	gpioGetValue(pin_in, &in_value);
	while (in_value != high)
	{	
		gpioGetValue(pin_in, &in_value);
	}
//	long double tim1 = GetTickCount()/1000;
	clock_t tim1 = clock();
	//cout << "LCH: The tim1 is " << tim1 << endl;
	while (in_value != low)
	{	
		gpioGetValue(pin_in, &in_value);
	}
//	long double tim2 = GetTickCount()/1000;
	clock_t tim2 = clock();
	double delta_time = (double)(tim2-tim1)/CLOCKS_PER_SEC;
	//cout << "LCH: The tim2 is " << tim2 << endl;
	double distance = delta_time * 34000/2;
	return distance;

}

extern "C"
{
	char* hc_sensor()
	{
		string dist_string = "200";

		jetsonTX1GPIONumber hc_out = gpio219 ;     // Ouput
		jetsonTX1GPIONumber hc_in = gpio38 ; // Input

		gpioExport(hc_out);
		gpioExport(hc_in);
		gpioSetDirection(hc_in,inputPin);
		gpioSetDirection(hc_out,outputPin);;
		dist_string = doubleToString(checkdist(hc_out, hc_in));
		strcpy(dist_message, dist_string.c_str());
		gpioUnexport(hc_in);     // unexport the LED
		gpioUnexport(hc_out);      // unexport the push button
		return dist_message;
	}
}
