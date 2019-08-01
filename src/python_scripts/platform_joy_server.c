/* 
   Author: Madhukar Thalore
   Date: Thursday, July 1st, 2019 - Tuesday, July 19th, 2019

   This code contains server side implementation of UDP client-server model for receiving Twist messages from Joystick and converting them to wheel velocities.
   It implements a real time task that samples from the CAN BUS in every loop to read the sensors from each ASOC.
*/

//Standard includes
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <getopt.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/uio.h> 
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>

//Xenomai includes
#include <rtdm/can.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/pipe.h>

//Custom includes
#include "platform-can-interface.h"
#include "kinematics.h"

#define PORT	8080 
#define MAXLINE 4096 

// Driver code 
int main() 
{ 	
	//Lock memory to prevent swapping
	mlockall(MCL_CURRENT | MCL_FUTURE);
	
	int sockfd; 
	char buffer[MAXLINE]; 
	struct sockaddr_in servaddr, cliaddr; 

	// Creating socket file descriptor 
	if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ) 
	{ 
		perror("socket creation failed"); 
		exit(EXIT_FAILURE); 
	} 
	
	memset(&servaddr, 0, sizeof(servaddr)); 
	memset(&cliaddr, 0, sizeof(cliaddr)); 
	
	// Filling server information 
	servaddr.sin_family = AF_INET; // IPv4 
	servaddr.sin_addr.s_addr = INADDR_ANY; 
	servaddr.sin_port = htons(PORT); 
	
	// Bind the socket with the server address 
	if ( bind(sockfd, (const struct sockaddr *)&servaddr, 
			sizeof(servaddr)) < 0 ) 
	{ 
		perror("bind failed"); 
		exit(EXIT_FAILURE); 
	} 

	printf("Finished setting up server. Listening for incoming UDP data.\r\n");

	printf("Setting up Kinematics of platform.\r\n");
	float alpha_l, alpha_r;
	float p_ll, p_lr, p_rl, p_rr;
	int p_ll_raw, p_lr_raw, p_rl_raw, p_rr_raw;
	Resi_t resi;
	float Vx, Vy, w;

	int alphaL, alphaR;
	int p1, p2, p3, p4; 
	float vll, vlr, vrl, vrr;

	//Set up the the CAN Bus
	initialize_can_bus();

	//Enable the motors
	enable_motors();

	//Set all wheel velocities to zero
	set_velocities(0, 0, 0, 0);

	while (1)
	{ 	
		int len = sizeof(servaddr), n; 	
		char *p;
		int i=0;
		float twist[6], q;
		float vx, vy, vz, wx, wy, wz; 

		//Receiving Twist data from client end
		n = recvfrom(sockfd, (char *)buffer, MAXLINE, MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len);

		p = strtok(buffer," ");
		while (p!= NULL)
		{
			q = atof(p);
			twist[i] = q;
			i++;
			p = strtok(NULL," ");
		}
		//Angular and Linear components
		vx = twist[0]; vy = twist[1]; vz = twist[2];
		wx = twist[3]; wy = twist[4]; wz = twist[5];
		
		//Read the sensors
		read_wheel_encoders_raw(&p1, &p2, &p3, &p4, &alphaL, &alphaR);

		//Kinematics
		resi = Caculate(vx, vy, wz, alphaL, alphaR);
		vll = (float)resi.w[0];
		vlr = (float)resi.w[1];
		vrl = (float)resi.w[2];
		vrr = (float)resi.w[3];

		if(vz <= 0.5)
		{
			//Set desired velocities
			set_velocities(vll, vlr, vrl, vrr);
      printf("Linear X: %f \t Linear Y: %f \t Angular Z: %f \n",vx,vy,wz);
			printf("Velocities of wheels: %f %f %f %f \n",vll,vlr,vrl,vrr);
		}
   else
   {
     set_velocities(0,0,0,0);
   }
		
	}
	return 0; 
} 


