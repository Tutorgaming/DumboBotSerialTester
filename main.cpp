/*********************************************************************
DUMBO BOT INTERFACE
Author : Theppasith Nisitsukcharoen
<theppasith@gmail.com , Theppasith.N@Student.chula.ac.th>
Department of Computer Engineering , Chulalongkorn University
*********************************************************************/
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <string>
#include <sys/types.h>
#include <iostream>
#include <termios.h>

#define DUMBO_MAX_SPEED 150
#define DUMBO_MIN_SPEED 50

//FOR MicroSleep
#include <unistd.h>

//#include <ros/ros.h>
#include "serial/serial.h"


#ifndef MIN
#define MIN(a,b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a > b) ? (a) : (b))
#endif

std::string port_name_;

serial::Serial ser;

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int16_t calculateChecksum_int16_t(unsigned char *buffer , unsigned int buffer_size){
    int16_t sum =0;
    for(unsigned int i = 2 ; i < buffer_size ; i++ ){
        std::cout << "Read ["<<i<<"] = " << (int)buffer[i] <<std::endl;
        sum += (int)buffer[i];
    }
    return sum;
}

char calculateChecksum(unsigned char *buffer , unsigned int buffer_size){
    int16_t sum =0;
    for(unsigned int i = 2 ; i < buffer_size ; i++ ){
        sum += (int)buffer[i];
    }
    return ((char)(sum & 0xFF)^0xFF);
}


// *****************************************************************************
// Open the serial port
int openSerialPort(std::string port){

    try{
        ser.setPort(port);
        ser.setBaudrate(9600);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e){
        std::cout << "Unable to open port " << std::endl;
        return -1;
    }

	return(0);

}


int send_readEncoder(){
    /*
        //255 255 001 004 002 024 010 214
    */
	// Compose Command
		unsigned char cmd_buffer[8];
    // Headers
		cmd_buffer[0] = (char)0xFF; //255
		cmd_buffer[1] = (char)0xFF; //255
		cmd_buffer[2] = (char)0x01; //001
	// counter
		cmd_buffer[3] = (char)0x04; //007
	// Error Bit
		cmd_buffer[4] = (char)0x02; //003
	// Starting Address
		cmd_buffer[5] = (char)0x18; // ('1' x 16 ) + ('4') = 020
	// Direction  			V - Right Motor
		cmd_buffer[6] = (char)0x0A;
	// Velocity Amount	 	V - Right Motor
		//cmd_buffer[7] = (char)0xD6;
		cmd_buffer[7] = calculateChecksum(cmd_buffer,7);

    // Send Command
		ser.write(cmd_buffer,8);
        return 0;
}

// *****************************************************************************
// Driving Commands

void send_forward(int speed)
{
	// Limit velocity
	int16_t sent_speed_int 	= MAX(speed, DUMBO_MIN_SPEED);  //MINIMUM at 50
			sent_speed_int 	= MIN(sent_speed_int, DUMBO_MAX_SPEED);  //MAXIMUM at 100
    std::cout << "FORWARD COMMAND WITH SPEED (BOUNDED) = " << sent_speed_int << std::endl;
	//Driving Command Protocol to Arduino Mega
	/*
		Address For Commands
		20 = direction Vr (1 = Forw, 2 = Backw)
		21 = Vr
		22 = direction Vl (1 = Forw, 2 = Backw)
		23 = Vl

		Ex. "Forward" = 255 255 001 007 003 020 001 255 001 255 224

		255 255 001 "Header"
		007 'Count From Here 7 Packet'
		003 "error bit"
		020 "Start Position of Register"
		001 " Write 001 at @020"  = Direction FW R
		255 V Forward R
		001 " Write 001 at @022"  = Direction FW L
		255 V Forward L
		224 "Checksum (bitwise of sum)"
	*/

	// Compose Driving Command
		unsigned char cmd_buffer[11];
	// Headers
		cmd_buffer[0] = (char)0xFF; //255
		cmd_buffer[1] = (char)0xFF; //255
		cmd_buffer[2] = (char)0x01; //001
	// counter
		cmd_buffer[3] = (char)0x07; //007
	// Error Bit
		cmd_buffer[4] = (char)0x03; //003
	// Starting Address
		cmd_buffer[5] = (char)0x14; // ('1' x 16 ) + ('4') = 020
	// Direction  			V - Right Motor
		cmd_buffer[6] = (char)0x02;
	// Velocity Amount	 	V - Right Motor
		cmd_buffer[7] = (char)(sent_speed_int& 0xFF);
	// Direction  			V - Left Motor
		cmd_buffer[8] = (char)0x02;
	// Velocity Amount	 	V - Left Motor
		cmd_buffer[9] = (char)(sent_speed_int& 0xFF);
	// CHECKSUM
//        int16_t checkSum = calculateChecksum(cmd_buffer,10);
//        std::cout << "checksum = " << (int) ((char)(checkSum & 0xFF)^0xFF)  <<std::endl;
		cmd_buffer[10] = calculateChecksum(cmd_buffer,10);//((char)(checkSum & 0xFF)^0xFF);

    // Send Packages
		ser.write(cmd_buffer,11);

}


void send_backward(int speed)
{
	// Limit velocity
	int16_t sent_speed_int 	= MAX(speed, DUMBO_MIN_SPEED);  //MINIMUM at 50
			sent_speed_int 	= MIN(sent_speed_int, DUMBO_MAX_SPEED);  //MAXIMUM at 100
    std::cout << "BACKWARD COMMAND WITH SPEED (BOUNDED) = " << sent_speed_int << std::endl;
	//Driving Command Protocol to Arduino Mega
	// Compose Driving Command
		unsigned char cmd_buffer[11];
	// Headers
		cmd_buffer[0] = (char)0xFF; //255
		cmd_buffer[1] = (char)0xFF; //255
		cmd_buffer[2] = (char)0x01; //001
	// counter
		cmd_buffer[3] = (char)0x07; //007
	// Error Bit
		cmd_buffer[4] = (char)0x03; //003
	// Starting Address
		cmd_buffer[5] = (char)0x14; // ('1' x 16 ) + ('4') = 020
	// Direction  			V - Right Motor
		cmd_buffer[6] = (char)0x01;
	// Velocity Amount	 	V - Right Motor
		cmd_buffer[7] = (char)(sent_speed_int& 0xFF);
	// Direction  			V - Left Motor
		cmd_buffer[8] = (char)0x01;
	// Velocity Amount	 	V - Left Motor
		cmd_buffer[9] = (char)(sent_speed_int& 0xFF);
	// CHECKSUM
		cmd_buffer[10] = calculateChecksum(cmd_buffer,10);

    // Send Packages
		ser.write(cmd_buffer,11);

}


void send_turnright(int speed)
{
	// Limit velocity
	int16_t sent_speed_int 	= MAX(speed, DUMBO_MIN_SPEED);  //MINIMUM at 50
			sent_speed_int 	= MIN(sent_speed_int, DUMBO_MAX_SPEED);  //MAXIMUM at 100
    std::cout << "BACKWARD COMMAND WITH SPEED (BOUNDED) = " << sent_speed_int << std::endl;
	//Driving Command Protocol to Arduino Mega
	// Compose Driving Command
		unsigned char cmd_buffer[11];
	// Headers
		cmd_buffer[0] = (char)0xFF; //255
		cmd_buffer[1] = (char)0xFF; //255
		cmd_buffer[2] = (char)0x01; //001
	// counter
		cmd_buffer[3] = (char)0x07; //007
	// Error Bit
		cmd_buffer[4] = (char)0x03; //003
	// Starting Address
		cmd_buffer[5] = (char)0x14; // ('1' x 16 ) + ('4') = 020
	// Direction  			V - Right Motor
		cmd_buffer[6] = (char)0x02;
	// Velocity Amount	 	V - Right Motor
		cmd_buffer[7] = (char)(sent_speed_int& 0xFF);
	// Direction  			V - Left Motor
		cmd_buffer[8] = (char)0x01;
	// Velocity Amount	 	V - Left Motor
		cmd_buffer[9] = (char)(sent_speed_int& 0xFF);
	// CHECKSUM
		cmd_buffer[10] = calculateChecksum(cmd_buffer,10);

    // Send Packages
		ser.write(cmd_buffer,11);

}

void send_turnleft(int speed)
{
	// Limit velocity
	int16_t sent_speed_int 	= MAX(speed, DUMBO_MIN_SPEED);  //MINIMUM at 50
			sent_speed_int 	= MIN(sent_speed_int, DUMBO_MAX_SPEED);  //MAXIMUM at 100
    std::cout << "BACKWARD COMMAND WITH SPEED (BOUNDED) = " << sent_speed_int << std::endl;
	//Driving Command Protocol to Arduino Mega
	// Compose Driving Command
		unsigned char cmd_buffer[11];
	// Headers
		cmd_buffer[0] = (char)0xFF; //255
		cmd_buffer[1] = (char)0xFF; //255
		cmd_buffer[2] = (char)0x01; //001
	// counter
		cmd_buffer[3] = (char)0x07; //007
	// Error Bit
		cmd_buffer[4] = (char)0x03; //003
	// Starting Address
		cmd_buffer[5] = (char)0x14; // ('1' x 16 ) + ('4') = 020
	// Direction  			V - Right Motor
		cmd_buffer[6] = (char)0x01;
	// Velocity Amount	 	V - Right Motor
		cmd_buffer[7] = (char)(sent_speed_int& 0xFF);
	// Direction  			V - Left Motor
		cmd_buffer[8] = (char)0x02;
	// Velocity Amount	 	V - Left Motor
		cmd_buffer[9] = (char)(sent_speed_int& 0xFF);
	// CHECKSUM
		cmd_buffer[10] = calculateChecksum(cmd_buffer,10);

    // Send Packages
		ser.write(cmd_buffer,11);

}


void send_stop(){

//255 255 001 007 003 020 000 255 000 255 226
    	// Compose Driving Command
		unsigned char cmd_buffer[11];
	// Headers
		cmd_buffer[0] = (char)0xFF; //255
		cmd_buffer[1] = (char)0xFF; //255
		cmd_buffer[2] = (char)0x01; //001
	// counter
		cmd_buffer[3] = (char)0x07; //007
	// Error Bit
		cmd_buffer[4] = (char)0x03; //003
	// Starting Address
		cmd_buffer[5] = (char)0x14; // ('1' x 16 ) + ('4') = 020
	// Direction  			V - Right Motor
		cmd_buffer[6] = (char)0x00;
	// Velocity Amount	 	V - Right Motor
		cmd_buffer[7] = (char)0xFF;
	// Direction  			V - Left Motor
		cmd_buffer[8] = (char)0x00;
	// Velocity Amount	 	V - Left Motor
		cmd_buffer[9] = (char)0xFF;
	// CHECKSUM
		cmd_buffer[10] = (char)0xE2;

    // Send Packages
		ser.write(cmd_buffer,11);
}

// *****************************************************************************
// Close the serial port
int closeSerialPort()
{
	send_stop();
		ser.close();
	return(0);
}

// *****************************************************************************
// Close the serial port

int main()
{
    port_name_ = "/dev/ttyACM1";

    std::cout << "DumboBot Tester v0.1" << std::endl;
	std::cout << "Creating Connection to " << port_name_ << std::endl;

	if( openSerialPort(port_name_) == -1 ) {
		std::cout << "@An Error Occurred" <<std::endl;
		return -1;
	}

    //Reader Buffer
    std::string buff;

        //Read Things in Buffer First
        ser.read(buff,13);
        //CHECK IF INCOMING BUFFER HAS SOME VALUE TO SHOW
        if(ser.available()){
            int i;
            for (i = 0; i < 13; i++) {
                printf("%02x ", buff[i]);
                std::cout << std::endl;
            }
        }

        send_stop();
        //usleep(5000);



    //Keyboard Teleop
    char kb = 's';
    while(true){
        kb = getch();
        std::cout <<std::endl;
        if(kb == 's'){
          send_stop();
        }
        if(kb == 'w'){
          send_forward(100);
        }
        if(kb == 'x'){
          send_backward(100);
        }
        if(kb == 'a'){
          send_turnleft(100);
        }
        if(kb == 'd'){
          send_turnright(100);
        }
        if(kb == 27){
         send_stop();
         break;
        }
    }

	return 0;
}
