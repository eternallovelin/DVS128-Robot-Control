// This is the main header file for the Zigbee keyboard module: used to manually control the robot
// Although some code is the same as in the main file, it's better to use a different header and .c files so that stuff doesn't break
#define DEFAULT_DEVICEINDEX	0
#define TIMEOUT_TIME		5

// No Zigbee connection establishment
#define DEBUG				1

#include <stdint.h>

/** Initilize the control: keyboard + zigbee */
int init();

/** Initializes the keyboard function. Uses ncurses for managing user IO */
void init_keyboard();

/** Send message to robot */
void zgb_send(int output);

/** Assemble a 16 bit message. Using uint16 to ensure that the message is 16 bits long */
uint16_t assemble16(uint16_t time16, uint16_t speed16, uint16_t cmd16);

/** Receive messages from the robot. Uses a selector mechanism */
void receive();

/** Initialize the selector */
void init_selector();

/** Check if data is incoming */
int chk_select(unsigned int timeout_seconds);

/* Close Zigbee */
void zig_close();
