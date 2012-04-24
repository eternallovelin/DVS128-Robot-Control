/// Control table address
#define MOVING_SPEED_L			32
#define MOVING_SPEED_H			33
#define GOAL_POSITION_L			30
#define GOAL_POSITION_H			31
#define CW_ANGLE_LIMIT_L		6
#define CW_ANGLE_LIMIT_H		7
#define CCW_ANGLE_LIMIT_L		8
#define CCW_ANGLE_LIMIT_H		9
#define MOVING					38
#define PRESENT_POSITION_L		36
#define PRESENT_POSITION_H		37
#define TORQUE_ENABLE			24

// Default settings
#define DEFAULT_BAUDNUM			34 // 1Mbps
#define DXL_FAIL				-1
#define INIT_SUCCESS			1
#define CW_ANGLE_FAIL			-1
#define CCW_ANGLE_FAIL			-2
#define CW_CCW_ANGLE_SUCCESS	1

// Servo motor ids
#define FRONT_LEFT				10
#define FRONT_RIGHT				2
#define BACK_LEFT				4
#define BACK_RIGHT				6
#define TURN_WEIGHT				0.7
#define TURN_WEIGHT				0.7

// Failure flags
#define WHEEL_MODE_ERROR		2

// Speeds
#define CW_SPEED_L				1024
#define CW_SPEED_H				2047
#define CCW_SPEED_L				0
#define CCW_SPEED_H				1023

// LED data
#define LED_BAT 0x01
#define LED_TxD 0x02
#define LED_RxD 0x04
#define LED_AUX 0x08
#define LED_MANAGE 0x10
#define LED_PROGRAM 0x20
#define LED_PLAY 0x40

// Data masks
#define CMD_MASK 0xF
#define SPEED_MASK 0xFFF
#define TIME_MASK 0xFF

// Will only echo back received data
#define DEBUG		1

// A perfect way to debug :)
#define LED_BAT 0x01
#define LED_TxD 0x02
#define LED_RxD 0x04
#define LED_AUX 0x08
#define LED_MANAGE 0x10
#define LED_PROGRAM 0x20
#define LED_PLAY 0x40

/** Initialize Dynamixel */
int dyn_init(void);
/** Set servo motors in wheel mode */
int set_wheel_mode(int id);
/** Execute command */
int dyn_exec(unsigned int cmd,unsigned int speed,unsigned int time);
/** Stop execution */
void dyn_stop(void);
/** Move forward */
void dyn_forward(unsigned int speed,unsigned int time);
/** Move backward */
void dyn_backward(unsigned int speed,unsigned int time);
/** Turn left */
void dyn_turn_left(int direction, unsigned int speed,unsigned int time);
/** Turn right */
void dyn_turn_right(int direction, unsigned int speed,unsigned int time);
/** Check if the robot is moving. Was a bit unstable and currently not used. The controller simply ACKs whenever done */
int check_if_moving();
/** Checks the remaining time to execute a command, if time has been specified */
void check_delay(int* delays, unsigned int time);
/** Delay for specific execution time, uses _delay_ms */
void delay(unsigned int time);
/** When cancelling the current command, remember to stop the delay  */
void set_stop_delay(int flag);
/** Cancel a command */
void cancel_command();
/** Send ACK */
void confirm();
