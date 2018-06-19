#ifndef DEFINES_H_
#define DEFINES_H_

//LEDS:

#define LED_GREEN 37
#define LED_YELLOW 98

//Encoders pinout:

#define ENK_1_A 4
#define ENK_1_B 5
#define ENK_2_A 23
#define ENK_2_B 24
#define ENK_3_A 40
#define ENK_3_B 42
#define ENK_4_A 53
#define ENK_4_B 54
#define ENK_5_A 59
#define ENK_5_B 60
#define ENK_6_A 63
#define ENK_6_B 64
#define ENK_7_A 90
#define ENK_7_B 91

//Motors current measurement:

#define MOTOR_1_CURRENT 18
#define MOTOR_2_CURRENT 25
#define MOTOR_3_CURRENT 26
#define MOTOR_4_CURRENT 15
#define MOTOR_5_CURRENT 16
#define MOTOR_6_CURRENT 17
#define BM_CURRENT 30

//Motors direction pins:

#define MOTOR_1_INA 1
#define MOTOR_1_INB 2
#define MOTOR_2_INA 8
#define MOTOR_2_INB 7
#define MOTOR_3_INA 83
#define MOTOR_3_INB 9
#define MOTOR_4_INA 82
#define MOTOR_4_INB 81
#define MOTOR_5_INA 88
#define MOTOR_5_INB 85
#define MOTOR_6_INA 3
#define MOTOR_6_INB 97

//Motors PWM outputs:

#define MOTOR_1_PWM 95
#define MOTOR_2_PWM 25
#define MOTOR_3_PWM 26
#define MOTOR_4_PWM 77
#define MOTOR_5_PWM 89
#define MOTOR_6_PWM 96

//High power motor pinout:

#define BM_L_PWM 48
#define BM_R_PWM 47

#define BM_L_ENABLE 45
#define BM_R_ENABLE 46

//Source voltage measurement:

#define VIN_MEASURE 29

//I2C pinout:

#define I2C1_SCL 92
#define I2C1_SDA 93

//SPI pinout:

#define SPI3_SCK 78
#define SPI3_MISO 79
#define SPI3_MOSI 80

//CAN pinout:

#define CAN_RX 70
#define CAN_TX 71

//UART 1 & 2 pinout:

#define UART1_TX 68
#define UART1_RX 69

#define UART2_TX 86
#define UART2_RX 87

//Temperature measurement:

#define TEMP_1 36
#define TEMP_2 35
#define TEMP_3 34
#define TEMP_4 33


//Digital input pinout:

#define IN_1 38
#define IN_2 39
#define IN_3 41
#define IN_4 43
#define IN_5 44
#define IN_6 51
#define IN_7 52
#define IN_8 55
#define IN_9 56
#define IN_10 57
#define IN_11 58
#define IN_12 61

//Digital input pinout:

#define OUT_1 62
#define OUT_2 66
#define OUT_3 65
#define OUT_4 67



#endif /* DEFINES_H_ */
