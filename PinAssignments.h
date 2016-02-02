//Pin assignments for DCM_2017

//Status LEDs (GPIOB)
#define STATUS_R_PIN					GPIO_Pin_7
#define STATUS_G_PIN					GPIO_Pin_8
#define STATUS_B_PIN					GPIO_Pin_9

//Throttles (GPIOA)
#define THROTTLE_1 						GPIO_Pin_1
#define THROTTLE_2 						GPIO_Pin_2

//I2C Channels (GPIOA)
#define I2C2_SCL 							GPIO_Pin_9
#define I2C2_SDA 							GPIO_Pin_10

//Gyro Data Ready interrupt pin 
#define GYRO_DRDY 						GPIO_Pin_0






//Constants	
#define OWN_I2C_ADDRESS				0x0
#define GYRO_I2C_ADDRESS			0x6B