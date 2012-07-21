#define HOSTNAME    "127.0.0.1"
#define PORT        3333
#define DEFAULT_DEVICEINDEX	0
#define TIMEOUT_TIME		5
#define DEBUG				0
#define NO_ZIGBEE			0  // No Zigbee
#define DATA_SIZE			5

#include <stdint.h>

/** Connects to the Workstation-Side Controller (WSC) */
int connect_to_server();

/** Forwards a message to the WSC */
int send_to_server(int* _send);

/** Receives a message from the WSC to be forwarded to the robot */
int receive_from_server(unsigned int* _recv);

/** Closes connection to the WSC */
int close_server_connection();

/** The selector is used for polling both comm channels for incoming data. Used instead of continuously looping and waiting for data */
int init_selector();

/* Checks whether data is available over any of the two comm channels */
int check_select();

/** Initializes Zigbee connection with the robot */
int initialize();

/* Initialize the keyboard resources */
void keyboard();

/** Initialize in keyboard mode */
int initialize_with_keyboard();

/** Sends a message to the robot */
int zgb_send16(uint16_t data);

/** Assemble a 16 bit message. Using uint16 to ensure that the message is 16 bits long */
uint16_t assemble16(uint16_t time16, uint16_t speed16, uint16_t cmd16);

/** Recevies a messsage from the robot. The selector is used to check whether a message is pending */
int zgb_recv();

/** Closes the Zigbee connection */
void close_zig_conn();
