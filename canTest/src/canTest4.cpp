
#include "ros/ros.h"

#include <unistd.h>

#include <chrono>
#include <thread>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>


int8_t mode;
int flag;

int rxCounter;

int limit[4];
int order[5];
int countrLimit;

ros::Publisher positionPub;
ros::Publisher velocityPub;
ros::Publisher IPub;
ros::Publisher sendIPub;
ros::Publisher targetPositionPub;
ros::Publisher limitPub;

ros::Subscriber PISub;
ros::Subscriber PSub;
ros::Subscriber targetPositionSub;
ros::Subscriber targetVelocitySub;
ros::Subscriber targetPositionAllSub;
ros::Subscriber modeSub;
ros::Subscriber orderSub;

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

	int16_t sendI;

	int32_t positionDifference;
	int32_t positionDifferenceSum;

	int16_t velocityDifference;
	int32_t velocityDifferenceSum;
	
	float Ki;
	float Kp;
	
	float K;

	int16_t limit_0;
	int16_t limit_1;


	int16_t relativePosition;
	int16_t targetRelativePosition;

};
Motor motor[4];

void motorInit(void){
	flag = 0;
	for(int i = 0;i<4;i++)
	{
		motor[i].targetPosition = 0;
		motor[i].targetVelocity = 0;
		motor[i].Kp = 13;
		motor[i].Ki = 6;
		motor[i].K = 15;
		motor[i].limit_0 = 0;
		motor[i].limit_1 = 0;

		limit[i] = 0;
		order[i] = 0;

	}
	order[4] = 0;
	order[0] = 1;
	countrLimit = 0;
}


void PI_Callback(std_msgs::Float32MultiArray PIMessage)
{
	int ID = 0;
    motor[ID].Kp = PIMessage.data[0];
    motor[ID].Ki = PIMessage.data[1];
}

void P_Callback(std_msgs::Float32 PMessage)
{
	int ID = 0;
	motor[ID].K = PMessage.data;
}

void targetPosition_Callback(std_msgs::Float32 targetPositionMessage){
	int ID = 0;
	motor[ID].targetPosition = targetPositionMessage.data;

}

void targetPositionAll_Callback(std_msgs::Float32MultiArray targetPositionAllMessage){
	for(int i = 0;i<4;i++) motor[i].targetPosition = targetPositionAllMessage.data[i];
	
}

void targetVelocity_Callback(std_msgs::Float32 targetVelocityMessage){
	int ID = 0;
	motor[ID].targetVelocity = targetVelocityMessage.data;
}

void mode_Callback(std_msgs::Int8 modeMessage)
{
	mode = modeMessage.data;
    motorInit();

	rxCounter = 0;
}

void order_Callback(std_msgs::Int8 orderMessage)
{
	order[0] = 1;
}


void printfMultiMotorVelocity(void){
	printf("velocity is %d,%d,%d,%d;",motor[0].velocity,motor[1].velocity,motor[2].velocity,motor[3].velocity);
}

std_msgs::Int32MultiArray positionMessage,velocityMessage,IMessage,sendIMessage,targetPositionMessage;
std_msgs::Int16MultiArray limitMessage;

void rxThread(int s)
{
	int ID;
	int i;
	int j;
	struct can_frame frame;
	int nbytes;
	rxCounter= 0;

	positionMessage.data.resize(4);
	velocityMessage.data.resize(4);
	IMessage.data.resize(4);
	targetPositionMessage.data.resize(4);
	limitMessage.data.resize(8);

    
	for (j = 0;j<4;j++)
	{
		motor[j].angleDifference = 0;
		motor[j].NumOfTurns = 0;
	}

    for (i = 0;; i++)
    {
		ros::spinOnce();
		nbytes = read(s, &frame, sizeof(struct can_frame));
		if (nbytes < 0)
		{
			perror("Read");
			break;
		}

		ID = int(frame.can_id-0x200)-1;

		rxCounter++;

        motor[ID].angle = (frame.data[0] << 8)+ frame.data[1];
		motor[ID].realAngle = motor[ID].angle*360/8191;
		motor[ID].velocity = (frame.data[2] <<8) + frame.data[3];
		motor[ID].I = (frame.data[4] <<8) + frame.data[5];
		motor[ID].temperature = frame.data[6];

        if (i>4){
            motor[ID].angleDifference = motor[ID].angle - motor[ID].angleLast;
        }

        if(motor[ID].angleDifference<-4000)
        {
            motor[ID].NumOfTurns++;
        }
        if(motor[ID].angleDifference>4000)
        {
            motor[ID].NumOfTurns--;
        }

        motor[ID].position = 8192*motor[ID].NumOfTurns+motor[ID].angle;
        motor[ID].angleLast = motor[ID].angle;


		positionMessage.data[ID] = motor[ID].position;
		velocityMessage.data[ID] = motor[ID].velocity;
		IMessage.data[ID] = motor[ID].I;

		if(i%4==0)
		{
			switch (mode)
			{
				case 0:
					printf("mode is %d; ",mode);
					printf("angle is %d,%d,%d,%d\n",motor[0].angle,motor[1].angle,motor[2].angle,motor[3].angle);
					break;
				case 1:
					printf("mode is %d; ",mode);
					printf("kp is %f; ki is %f; vel is %d; tv is %d; vel_d is %d; vel_d_sum = %d; send_I is %d; I is %d\n",
					motor[0].Kp,motor[0].Ki,motor[0].velocity,motor[0].targetVelocity,motor[0].velocityDifference,motor[0].velocityDifferenceSum,
					motor[0].sendI,motor[0].I);
					break;

				case 2:
					printf("mode is %d; ",mode);
					printf("k %f; p is %d; tp is %d; send_I is %d; I is %d\n",
					motor[0].K,motor[0].position,motor[0].targetPosition,
					motor[0].sendI,motor[0].I);

					break;

				case 3:
					printf("mode is %d; ",mode);
					printf("vel is %d; tv is %d; vel_d is %d; vel_d_sum = %d; send_I is %d; I is %d\n",
					motor[0].velocity,motor[0].targetVelocity,motor[0].velocityDifference,motor[0].velocityDifferenceSum,
					motor[0].sendI,motor[0].I);

					break;

				case 4:
					printf("mode is %d; ",mode);
					printf("p is %d; tp is %d; send_I is %d; I is %d\n",
					motor[0].position,motor[0].targetPosition,
					motor[0].sendI,motor[0].I);

					break;
				case 5:
					printf("mode is %d; ",mode);
					printf("position is %d,%d,%d,%d",motor[0].position,motor[1].position,motor[2].position,motor[3].position);
					printf("target position is %d,%d,%d,%d;",motor[0].targetPosition,motor[1].targetPosition,motor[2].targetPosition,motor[3].targetPosition);
					printf("sendI is %d,%d,%d,%d;",motor[0].sendI,motor[1].sendI,motor[2].sendI,motor[3].sendI);
					printf("I is %d,%d,%d,%d;",motor[0].I,motor[1].I,motor[2].I,motor[3].I);
					printf("\n");
					break;
					
				case 6:
					printf("mode is %d; ",mode);
					printf("order is %d;%d;%d;%d;%d;",order[0],order[1],order[2],order[3],order[4]);
					printf("limitmotor0 is %d %d;",motor[0].limit_0,motor[0].limit_1);
					printf("limitmotor1 is %d %d;",motor[1].limit_0,motor[1].limit_1);
					printf("limitmotor2 is %d %d;",motor[2].limit_0,motor[2].limit_1);
					printf("limitmotor3 is %d %d\n;",motor[3].limit_0,motor[3].limit_1);
					break;
				default:
					mode = 0;
					printf("请设定正常的mode,现在已经更改mode为0\n");
			}
			positionPub.publish(positionMessage);
			velocityPub.publish(velocityMessage);
			IPub.publish(IMessage);
			for(j = 1;j<4;j++) {
				limitMessage.data[2*j] = motor[j].limit_0;
				limitMessage.data[2*j+1] = motor[j].limit_1;
			}
			limitPub.publish(limitMessage);


		}
		if(rxCounter == 32&& mode == 5)
		{
			flag = 1;
			for (j = 0;j<4;j++)
			{
				targetPositionMessage.data[j] = motor[j].position;
				motor[j].targetPosition = motor[j].position;
			}
			
		}
		if(rxCounter>32) targetPositionPub.publish(targetPositionMessage);
		std::this_thread::sleep_for(std::chrono::nanoseconds(100000));
    }

}

void controlP_calSendI_PI(int ID){
	motor[ID].positionDifference = motor[ID].targetPosition - motor[ID].position;
	motor[ID].positionDifferenceSum = motor[ID].positionDifference + motor[ID].positionDifferenceSum;

	motor[ID].sendI = motor[ID].Kp*motor[ID].positionDifference + motor[ID].Ki*motor[ID].positionDifferenceSum;
}

void controlV_calSendI_PI(int ID){
	int velocityDifferenceSumLimit = 16667;
	limit[ID] = 0;
	motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
	if(motor[ID].velocityDifference > 1000) motor[ID].velocityDifference = 1000;
	if(motor[ID].velocityDifference < -1000) motor[ID].velocityDifference = -1000;
	motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;

	if(motor[ID].velocityDifferenceSum>velocityDifferenceSumLimit) {motor[ID].velocityDifferenceSum = velocityDifferenceSumLimit;limit[ID] = 1;}
	if(motor[ID].velocityDifferenceSum<(0-velocityDifferenceSumLimit)) {motor[ID].velocityDifferenceSum = (0-velocityDifferenceSumLimit);limit[ID] = 1;}

	motor[ID].sendI = motor[ID].Kp*motor[ID].velocityDifference + motor[ID].Ki/10*motor[ID].velocityDifferenceSum;
}

void controlP_calSendI_PPI(int ID){
	int velocityDifferenceSumLimit = 16667;
	limit[ID] = 0;

	motor[ID].positionDifference = motor[ID].targetPosition - motor[ID].position;
	if(motor[ID].positionDifference > 4000) motor[ID].positionDifference = 4000;
	if(motor[ID].positionDifference < -4000) motor[ID].positionDifference = -4000;

	motor[ID].targetVelocity = motor[ID].K/100*(motor[ID].positionDifference);
	motor[ID].velocityDifference = motor[ID].targetVelocity - motor[ID].velocity;
	motor[ID].velocityDifferenceSum = motor[ID].velocityDifference + motor[ID].velocityDifferenceSum;

	if(motor[ID].velocityDifferenceSum>velocityDifferenceSumLimit) {motor[ID].velocityDifferenceSum = velocityDifferenceSumLimit;limit[ID] = 1;}
	if(motor[ID].velocityDifferenceSum<(0-velocityDifferenceSumLimit)) {motor[ID].velocityDifferenceSum = (0-velocityDifferenceSumLimit);limit[ID] = 1;}

	motor[ID].sendI = motor[ID].Kp*motor[ID].velocityDifference + motor[ID].Ki/10*motor[ID].velocityDifferenceSum;
}

void testLimit(int ID){
	switch (order[ID])
	{
		case 0:
			break;

		case 1:
			motor[ID].targetVelocity = 100;
			controlV_calSendI_PI(ID);
			
			if (limit[ID] == 1){
				countrLimit++;
				if(countrLimit >100)
				{
					motor[ID].limit_0 = motor[ID].position;
					order[ID] = 2;
					countrLimit = 0;
				}
			}
			else{
				countrLimit = 0;
			}
			break;

		case 2:
			motor[ID].targetVelocity = -100;
			controlV_calSendI_PI(ID);
			if (limit[ID] == 1){
				countrLimit++;
				if(countrLimit >100)
				{
					motor[ID].limit_1 = motor[ID].position;
					order[ID] = 3;
					countrLimit = 0;
				}
			}
			else{
				countrLimit = 0;
			}
			break;

		case 3:
			order[ID+1] = 1;
			order[ID] = 0;
			motor[ID].sendI = 0;
			break;

		default:
		printf("这个不可能被打印出来！");
	}

}


void txThread(int s)
{
    struct can_frame frame;
	frame.can_id = 0x200;
	frame.can_dlc = 8;
	int j;
	sendIMessage.data.resize(4);
	for (j = 0; j < 4; j++)
	{
		motor[j].sendI = 0;
		frame.data[2*j] = motor[j].sendI << 8;
		frame.data[2*j+1] = motor[j].sendI << 0;
	}

	int nbytes;

    for (int i = 0;; i++)
	{
		ros::spinOnce();
		switch (mode)
		{
			case 0:
				break;
			case 1:
				controlV_calSendI_PI(0);
				break;
			case 2:
				controlP_calSendI_PPI(0);
				break;
			case 3:
				controlV_calSendI_PI(0);
				break;
			case 4:
				controlP_calSendI_PPI(0);
				break;
			case 5:
				if (flag == 1){
					controlP_calSendI_PPI(0);
					controlP_calSendI_PPI(1);
					controlP_calSendI_PPI(2);
					controlP_calSendI_PPI(3);
				}
				break;
			case 6:
				testLimit(0);
				testLimit(1);
				testLimit(2);
				testLimit(3);
				break;
			default:
				mode = 0;
				printf("请设定正常的mode,现在已经更改mode为0\n");

		}

		
		for (j = 0;j<4;j++)
		{
			if (motor[j].sendI >15000) {
				motor[j].sendI = 15000;
			}
			if (motor[j].sendI <-15000) {
				motor[j].sendI = -15000;
			}
			frame.data[2*j] = motor[j].sendI>>8;
			frame.data[2*j+1] = motor[j].sendI>>0;
			sendIMessage.data[j] = motor[j].sendI;

		}
		sendIPub.publish(sendIMessage);
        nbytes = write(s, &frame, sizeof(struct can_frame));
        if (nbytes == -1) {
			printf("send error\n");
			printf("Please check if the power switch is turned on\n");
			exit (0);

        }
		std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
    }

}



int main(int argc, char** argv) {

	mode = 0;
	motorInit();


	ros::init(argc,argv,"canTest4");
	ros::NodeHandle n;
	ros::Rate loop_rate(100);

	

    positionPub = n.advertise<std_msgs::Int32MultiArray>("position",100);
    velocityPub = n.advertise<std_msgs::Int32MultiArray>("velocity",100);
    IPub = n.advertise<std_msgs::Int32MultiArray>("I",100);
	sendIPub = n.advertise<std_msgs::Int32MultiArray>("sendI",100);
	targetPositionPub = n.advertise<std_msgs::Int32MultiArray>("targetPositionFromPosition",100);
	limitPub = n.advertise<std_msgs::Int16MultiArray>("limit",100);


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

	std::thread canRx(rxThread, s);
	sleep(0.5);
	std::thread canTx(txThread, s);
    PISub = n.subscribe("PI", 10, PI_Callback);
	PSub = n.subscribe("P",10,P_Callback);
	targetVelocitySub = n.subscribe("targetVelocity", 10, targetVelocity_Callback);
	targetPositionSub = n.subscribe("targetPosition", 10, targetPosition_Callback);
    targetPositionAllSub = n.subscribe("targetPositionAll", 10, targetPositionAll_Callback);
	modeSub = n.subscribe("mode", 10, mode_Callback);
	orderSub = n.subscribe("order",10,order_Callback);

	while (ros::ok())
    {

		ros::spinOnce();
		loop_rate.sleep();
	}

    return 0;
}
