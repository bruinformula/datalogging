
#define BLUE_LED 4

#define BOARD_INDEX 0

#if BOARD_INDEX == 0
  #define CAN_ID_FRAME      0xC000000
#elif BOARD_INDEX == 1
  #define CAN_ID_FRAME      0xC100000
#elif BOARD_INDEX == 2
  #define CAN_ID_FRAME      0xC200000
#else
  #define CAN_ID_FRAME      0xC300000
#endif


#define PIN_RS485_RX 1
#define PIN_RS485_TX 2

#define CAN_STR_GAG_ID_1 0x01
#define CAN_STR_GAG_ID_2 0x02
#define CAN_STR_GAG_ID_3 0x03
#define STR_GAG_PIN_1 8
#define STR_GAG_PIN_2 10
#define STR_GAG_PIN_3 13
#define STR_GAG_READ_INT 10

#define CAN_SHOCK_POT_ID 0x0001000
#define SHOCK_POT_PIN 33
#define SHOCK_POT_READ_INT 10

#define WHEEL_SPEED_PIN 1
#define WHEEL_SPEED_ID 0x05
#define WHEEL_SPEED_READ_INT 10


#define BRAKE_TEMP_READ_INT 10
#define BRAKE_TEMP_ID 0x06

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
#define WHEEL_TEMP_READ_INT 10
#define WHEEL_TEMP_ID_START 0x07

uint8_t str_gag_range = 2;
double str_gag_zero = 0;

uint8_t shock_pot_range = 5;
double shock_pot_zero = 0;

uint16_t start_pixel = 360;
uint16_t end_pixel = 383;


#define PIN_WIRE_SDA         (29u)
#define PIN_WIRE_SCL         (31u)

#define CAN_TX         (4u)
#define CAN_RX         (2u)

#define TA_SHIFT 8 //Default shift for MLX90640 in open air



void sendCanMsg(uint32_t id, uint8_t msg[]);
