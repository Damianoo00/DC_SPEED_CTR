/**
 * @file main.cpp
 * @author Damian Płaskowicki (damian.plaskowicki.stud@pw.edu.pl)
 * @brief DC Drive control with Speed and Current regulatorn on Arduino
 * @version 0.1
 * @date 2022-05-31
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "../include/control.h"
#include "../include/PWM.h"
#include "../include/sensors.h"
#include "../include/uart.h"
#include "../include/i2c.h"
#include <Arduino.h>

/**
 * @brief define or remove correct line to switch any mode {#define [param]}
 * @param LOG define to enabel logging
 * @param SET_VALUES define to enable getting current and speed values from UART
 * @param WORK define to enable getting current and speed values from sensors
 *
 */

#define LOG
#define SET_VALUES

/***** POUT *****/
#define CURR_PORT A0
#define PWM_PORT 10

/*** UART params***/
#define BAUD 115200
#define TIMEOUT 10

/*** I2C slave id ***/
#define ENCODER_ID 8

/** Electric Params **/
constexpr int In = 1; // A
constexpr int Vs = 6; // V

/*** REG I params ***/
const float Ts = 0.0001f;
const float Kr_i = 2584.44f;
const float Tr_i = 0.0004f;
const int8_t max_i = Vs;
const int8_t min_i = 0;

/*** REG V params ***/
const float Kr_v = 79.6296f;
const float Tr_v = 0.0012f;
const float max_v = 1.2f * In;
const float min_v = -1.2f * In;

struct PICTRL PIctrl_curr;
struct PICTRL PIctrl_speed;

static int curr_sensor = 0;
static int speed_sensor = 0;

/* REF speed value [rad/s] */
const int speed_ref = 1900;

void setup()
{
  uart_begin(BAUD, TIMEOUT);
  i2c_begin_master();

  PWM_begin(PWM_PORT);

  InitPIctrl(&PIctrl_speed, Ts, Kr_v, Tr_v, max_v, min_v);
  InitPIctrl(&PIctrl_curr, Ts, Kr_i, Tr_i, max_i, min_i);
}

void loop()
{

#ifdef LOG
  /************************** Set header and params to log **********************************/
  const String header = "time,speed_ref,speed,current_max,current,ctr_sig";
  const long log_parametrs[] = {millis(), speed_ref, speed_sensor, (long)(max_v * 1000), curr_sensor, (long)(PIctrl_curr.y * 1000)};
  /********************************************************************************************/

  const int NumOfParams = sizeof(log_parametrs) / sizeof(log_parametrs[0]);
  log_uart(header, log_parametrs, NumOfParams);
#endif

#ifdef WORK
  constexpr int ShutResistance = 500; // m Ohm
  curr_sensor = GetCurrent(CURR_PORT, ShutResistance);

  constexpr int NumOfBytes = 4;
  speed_sensor = i2c_get_value_from_slave(ENCODER_ID, NumOfBytes);
#endif

#ifdef SET_VALUES
  uart_recive_2_params(&curr_sensor, &speed_sensor);
#endif
  CalcPIctrl(&PIctrl_speed, (float)speed_ref - (float)speed_sensor);
  CalcPIctrl(&PIctrl_curr, PIctrl_speed.y - (float)curr_sensor / 1000.0f);

  PWM_write(VoltageToDuty(PIctrl_curr.y, Vs));
}