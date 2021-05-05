#include "ros/ros.h"

#include <chrono>
#include <thread>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>


int rxCounter = 0;


struct Motor{
	int16_t angle,velocity,I;
	uint8_t temperature;
	int16_t realAngle;
    int32_t position;
    int16_t NumOfTurns;

	int32_t targetPosition;
	int16_t targetVelocity;

	int16_t angleDifference;
	int16_t angleLast;

};

Motor motor[4];

void printfSingleMotor(Motor motor){
	printf("angle is %d,realAngle is %d,position is %d,velocity is %d,I is %d\r\n",
	motor.angle,motor.realAngle,motor.position,motor.velocity,motor.I);

}

void printfMultiMotorAngle(void){
	printf("angle is %d,%d,%d,%d;\r\n",motor[0].angle,motor[1].angle,motor[2].angle,motor[3].angle);
}

void printfMultiMotorPosition(void){
	printf("position is %d,%d,%d,%d;\r\n",motor[0].position,motor[1].position,motor[2].position,motor[3].position);
}

void printfMultiMotorVelocity(void){
	printf("velocity is %d,%d,%d,%d;\r\n",motor[0].velocity,motor[1].velocity,motor[2].velocity,motor[3].velocity);
}

void printfMultiMotorI(void){
	printf("I is %d,%d,%d,%d;\r\n",motor[0].I,motor[1].I,motor[2].I,motor[3].I);
}


void rxThread(int s) 
{
	int ID;
	int i;
	struct can_frame frame;
	int nbytes;
    for (i = 0;; i++) 
    {
		
		nbytes = read(s, &frame, sizeof(struct can_frame));
		if (nbytes < 0) 
		{
			perror("Read");
			break;
		}
		ID = int(frame.can_id-0x200)-1;
		motor[ID].angle = (frame.data[0] << 8)+ frame.data[1];
		motor[ID].realAngle = motor[ID].angle*360/8191;
		motor[ID].velocity = (frame.data[2] <<8) + frame.data[3];
		motor[ID].I = (frame.data[4] <<8) + frame.data[5];
		motor[ID].temperature = frame.data[6];
		rxCounter++;
		if(i%4==0)
		{
			printfMultiMotorAngle();
			printfMultiMotorVelocity();
		}
		std::this_thread::sleep_for(std::chrono::nanoseconds(100000)); 
    } 
    
}

void txThread(int s)
{
    struct can_frame frame;
	frame.can_id = 0x200;
	frame.can_dlc = 8;
	for (int j = 0; j < 8; j++) 
	{
		frame.data[j] = 0x00;
	}
	int nbytes;

    for (int i = 0;; i++) 
	{
        nbytes = write(s, &frame, sizeof(struct can_frame));
        if (nbytes == -1) {
			printf("send error\n");
        }
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000000)); 
    }
    
}



int main(int argc, char** argv) {

	ros::init(argc,argv,"canTest1");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	int s;
	struct sockaddr_can addr;
	struct ifreq ifr;

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Error while opening socket");
		return -1;
	}

	strcpy(ifr.ifr_name, "can0");
	ioctl(s, SIOCGIFINDEX, &ifr);

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	printf("%s at index %d\n", ifr.ifr_name, ifr.ifr_ifindex);

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Error in socket bind");
		return -2;
	}

	//std::thread canTx(txThread, s);
	std::thread canRx1(rxThread, s);


	while (ros::ok())
    {
		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
