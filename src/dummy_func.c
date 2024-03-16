#include "gsNetPoll_tcp.h"
#include <clips.h>
// #include <ros/ros.h>

void ROS_INFO(){}
void ROS_WARN(){}



int CONDOR_make_contact_client() {
	ROS_INFO("Function %s has been deprecated", __func__);

	/* check for exactly one argument */
	if(ArgCountCheck("CONDOR_make_contact_client", EXACTLY, 1) == -1)
		return -1;

	/* get the value for the 1st argument */
	// int s = RtnDouble(1);

	return 1;
}



/* */
/* */
/**
 * Opens a network conection, with signals, to receive data
 * Format: (CONDOR_open_conection_receive_signal TIME_OPTION PORT BUFFER_SIZE)
 * @remark Dummy function
 * @return Always 1
 */
int CONDOR_open_conection_receive_signal() {
	ROS_INFO("Function %s has been deprecated", __func__);
	// char var_port[100], directory[100];

	/* check for exactly 3 arguments */
	if(ArgCountCheck("CONDOR_open_conection_receive_signal", EXACTLY, 3) == -1)
		return -1;

	// /* get the value for the 1st argument */
	// int time_option = RtnDouble(1);
	// /* get the value for the 2nd argument */
	// int port = RtnDouble(2);
	// /* get the value for the 3rd argument */
	// int buffer_size = RtnDouble(3);


	// /* Sets the function to catch the external signal */
	// signal(SIGALRM, sigcatcher);

	// /* Puts the pid number to a file to be used by the sender */
	// int pid = getpid();
	// sprintf(var_port, "/tmp/PORT%d", port);
	// FILE *fp = fopen(var_port, "w");
	// if(fp == NULL){
	// 	printf("\n File %s can not be created", var_port);
	// 	printf("\n Check that directory /tmp exists\n");
	//  	return 0;
	// }
	// /* it opens the network conection */
	// GSNPS* port_signal = open_network_conection("localhost", port, buffer_size, "r");
	// if((port_signal) == NULL){
	// 	fclose(fp);
	// 	return 0;
	// }

	// fprintf(fp, "%d", pid);
	// fclose(fp);
	return 1;
}



/**
 * Opens the network conection, with signals, to send data
 * Format: (CONDOR_open_conection_send_signal PORT BUFFER_SIZE)
 * @remark Dummy function
 * @return Always 1
 */
int CONDOR_open_conection_send_signal() {
	ROS_INFO("Function %s has been deprecated", __func__);

	/* check for exactly one argument */
	if(ArgCountCheck("CONDOR_open_conection_send_signal", EXACTLY, 2) == -1)
		return -1;

	// /* get the value of the port */
	// int port = RtnDouble(1);
	// /* get the value of buffer_size */
	// int buffer_size = RtnDouble(2);

	// /* gets the pid number of the receiver port from a file */
	// char var_port[100];
	// sprintf(var_port, "/tmp/PORT%d", port);
	// FILE *fp = fopen(var_port, "r");
	// if(fp == NULL){
	// 	printf("\n Check that the receiving port is open\n");
	// 	return 0;
	// }
	// /* it opens the network conection */
	// GSNPS* port_network = open_network_conection("localhost", port, buffer_size, "w");
	// if(port_network == NULL){
	// 	return 0;
	// 	fclose(fp);
	// }

	// int pid;
	// fscanf(fp, "%d", &pid);
	// fclose(fp);

	// Link_signals++;
	// if(Link_signals >= NUM_MAX_CONNECTIONS){
	// 	printf("\n No more conections, maximum number %d", NUM_MAX_CONNECTIONS);
	// 	return -1;
	// }

	// Connections_signals[Link_signals] = port_network;
	// Pid_signals[Link_signals] = pid;
	// Port_signals[Link_signals] = port;
	// return Link_signals;
	return 1;
}


/**
 * Opens the server conection
 * @remark Dummy function
 * @return always 1
 */
int CONDOR_open_server(){
	ROS_INFO("Function %s has been deprecated", __func__);

	/* check for exactly one argument */
	if(ArgCountCheck("CONDOR_open_server", EXACTLY, 1) == -1) return -1;

	/* get the value for the 1st argument */
	// char* server_name = RtnLexeme(1);

	return 1;
}



/**
 * Opens client & server conection
 * @remark Dummy function
 * @return Always 1
 */
int CONDOR_open_client_conection(){
	ROS_INFO("Function %s has been deprecated", __func__);

	/* check for exactly two arguments */
	if(ArgCountCheck("CONDOR_open_client_conection", EXACTLY, 2) == -1)
		return -1;

	/* get the values of the first two arguments */
	// char* client = RtnLexeme(1);
	// char* server = RtnLexeme(2);

	return 1;
}



/**
 * Opens the network conection
 * @remark Dummy function
 * @return Always 1
 */
int CONDOR_open_network_conection() {
	ROS_INFO("Function %s has been deprecated", __func__);

	/* check for exactly one argument */
	if(ArgCountCheck("CONDOR_open_network_conection", EXACTLY, 4) == -1)
		return -1;

	/* get the value for the 1st argument */
	// char* address = RtnLexeme(1);
	/* get the value for the 2nd argument */
	// int   port = RtnDouble(2);
	/* get the value for the 3rd argument */
	// int   buffer_size = RtnDouble(3);
	/* get the value for the 4th argument */
	// char* operation = RtnLexeme(4);

	return 1;
}


/*
 */
/**
 * Sends data to the network using handshaking with signals
 * Format: (CONDOR_send_data_signal NUM_CONNECTION DATA)
 * @remark Dummy function
 * @return Always 1
 */
int CONDOR_send_data_signal() {
	ROS_INFO("Function %s has been deprecated", __func__);

	/* check for exactly two arguments */
	if(ArgCountCheck("CONDOR_send_data_signal", EXACTLY, 2) == -1)
		return -1;

	/* get the values for the 1st, 2rd */
	// int c = RtnDouble(1);
	// char* message = RtnLexeme(2);
	// int ipd = Pid_signals[c];
	// if(ipd == 0){
	// 	printf("\n receiver is not available \n");
	// 	return 0;
	// }

	/* It sends the data */
	// send_data_client_network(Connections_signals[c], message);
	// if ( kill(ipd, SIGALRM) == -1){
	// 	/*printf("\n Receiver process is dead \n");*/
	// 	printf("Start receiver\nWaiting ... \n");
	// 	wait_start_receiver(c);
	// 	send_data_client_network(Connections_signals[c], message);
	// 	send_data_client_network(Connections_signals[c], message);
	// }

	return 1;
}



/**
 * Opens the network server conection
 * @param  address     [description]
 * @param  buffer_size [description]
 * @param  port        [description]
 * @param  operation   [description]
 * @return             [description]
 */
GSNPS *open_network_conection(char* address, int buffer_size, int port, char* operation) {
	// GSNPS *connector;
	// connector = gsNetPollOpen(address, port, buffer_size, operation, NULL);
	// return connector;
	return NULL;
}


/**
 * Opens the server conection
 * @param s      [description]
 * @param server [description]
 */
void open_server(int *s, char *server) {
	// struct sockaddr sock_desc;
	// int c, stringlen;
	// char string[16];
	// char line[256];


	// /* Delete the previous server */
	// sprintf(line, "rm -f %s", server);
	// system(line);
	// strcpy(line, server);

	// printf("\n open server %s", line);


	// /* set up listening socket s */
	// printf("Crando el socket desde socketlib.c\n");
	// *s = socket(AF_INET, SOCK_STREAM, 0);
	// if(*s<0){
	// 	perror("server:socket");
	// 	exit(1);
	// }

	// sock_desc.sa_family = AF_INET;
	// strcpy(sock_desc.sa_data, line);
	// 	if( bind(*s, &sock_desc, sizeof(sock_desc))== -1) {
	// 	perror("server:bind");
	// 	exit(1);
	// }
}



void make_contact_client(int s, int *ns) {
	// int stringlen;
	// /*char string[16];*/
	// struct sockaddr *string;

	// /* Makes the connection */
	// printf("\n Waits to make connection \n");
	// /* Wait to make conctact with the client */
	// listen(s, 100);

	// /* accept connection request */
	// stringlen = sizeof(string) -1;
	// *ns = accept(s, string, &stringlen);
	// if(*ns<0){
	// 	perror("server:accept");
	// 	unlink("rversoc"); /* if necesary */
	// 	exit(1);
	// }

	// printf(" Connection ready \n");
}



/**
 * It opens client & server conection
 * @param s      [description]
 * @param client [description]
 * @param server [description]
 */
void open_client_conection(int* s, char* client, char* server) {
	// char line[256], cl[256], srv[256];
	// struct sockaddr sock_desc;
	// struct sockaddr sock_server;

	// /* Delete the previous server */
	// sprintf(line, "rm -f %s", client);
	// system(line);

	// strcpy(cl, client);
	// strcpy(srv, server);

	// /* set up listening socket s */
	// printf("Creando el socket del servidor desde socketlib.c\n");
	// *s = socket(AF_UNIX, SOCK_STREAM, 0);

	// sock_desc.sa_family = AF_UNIX;
	// strcpy(sock_desc.sa_data, cl);
	// if( bind(*s, &sock_desc, sizeof(sock_desc))== -1) {
	// 	perror("client:bind");
	// 	exit(1);
	// }

	// /* request connection to serversoc */
	// sock_server.sa_family = AF_UNIX;
	// strcpy(sock_server.sa_data, srv);

	// if(connect(*s, &sock_server, sizeof(sock_server)) == -1){
	// 	perror("client:connect");
	// 	unlink("ientsoc"); /* if necesary */
	// 	exit(1);
	// }
}



void close_network_conection(GSNPS *s) {
	// gsNetPollClose(s);
}



/* receive the data from the client */
/**
 * [receive_data_client description]
 * @param  ns [description]
 * @return    [description]
 */
char *receive_data_client(int ns) {
	// /* data transfer on connected socket ns */
	// int c = read(ns, buf_mena, sizeof(buf_mena));
	// printf(" Message from client '%s' status %d\n", buf_mena, c);

	// return buf_mena;
}


/**
 * Sends data to client
 * @param  ns [description]
 * @return    [description]
 */
void send_data_client(int s, char *message) {
	// char output[256];

	// strcpy(output, message);
	// write(s, output, sizeof(output));
}



/**
 * Sends data to client in the network
 * @param  ns [description]
 * @return    [description]
 */
void send_data_client_network(GSNPS *s, char *message) {
	// int j, size;
	// char output[2000];
	// size = strlen(message);
	// for(j = 0;j<1999;j++)output[j]=0;

	// strcpy(output, message);
	// strcat(output, "  ");
	// gsNetPollWrite(s, output);
}



/**
 * Receives data from the network
 * @param  ns [description]
 * @return    [description]
 */
char *receive_data_client_network(GSNPS *s) {
	// char input[1000];
	// int i, j;

	// bcopy(gsNetPollRead(s), input, 512);
	// for(i = 511;i> -1; i--){
	// 	/*printf("\n input %d %x", i,input[i]);*/
	// 	if(input[i] != ' ') break;
	// }
	// strncpy(out_mena, input, i+1);
	// out_mena[i+1]='\0';

	// return out_mena;
}



/**
 * Sends data to the server
 * @param  ns [description]
 * @return    [description]
 */
void send_data_server(int s, char *message) {
	// char output[256];

	// strcpy(output, message);
	// write(s, output, sizeof(output));
}



/**
 * Receive the data from the server
 * @param  ns [description]
 * @return    [description]
 */
char *receive_data_server(int ns) {
	// /* data transfer on connected socket ns */
	// int c = read(ns, buf_mena1, sizeof(buf_mena1));
	// /*printf(" Message from server '%s' \n", buf_mena1);*/

	// return buf_mena1;
}


