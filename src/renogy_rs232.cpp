/* Reference: https://github.com/1Ghasthunter1/Helium_RMS */
/*
    For use with Renogy Wanderer. Other Renogy charge controllers may be supported, but are untested
    A system to communicate with Renogy solar charge controllers often used in Off-Grid Helium Hotspots.
    - Battery, solar, and load statistics
    - Error code monitoring
    - Remote load control

    The controller sends out data requests via the built-in RS232 RJ12 port on the Renogy controller 
    and receives information like solar charge, load status, and error messages

    Interfacing provided by Serial1 via the MIKROE-1582 RS232 Click module via the RAK1920 adapter
*/

#include "Arduino.h"
#include "app.h"

#ifdef ENABLE_RS232
#include <ModbusMaster.h>

// instantiate ModbusMaster object
ModbusMaster node;

renogy_data_s g_renogy_data;
uint8_t recvd_renogy_downlink = 0;

static byte* func_out;
//static byte bitmask_chargestatus = 0b01111111;
String temp_str;
char temp[50];

void init_renogy_rs232(void)
{
	Serial1.begin(9600);
  node.begin(1, Serial1);
}

const uint16_t dataStartRegister = 0x100;
const int numDataRegisters = 10;//30;
const uint16_t infoStartRegister = 0x00A;
const int numInfomRegisters = 17;
const uint16_t errorStartRegister = 0x121;
const int numErrorRegisters = 2;

void renogySetData(uint16_t *data)
{
  int i = 0;
  g_renogy_data.batt_capacity.val16 = data[i++];
  g_renogy_data.batt_voltage.val16 = data[i++];
  g_renogy_data.batt_charge_current.val16 = data[i++];
  g_renogy_data.temp.val16 = data[i++];
  g_renogy_data.load_voltage.val16 = data[i++];
  g_renogy_data.load_current.val16 = data[i++];
  g_renogy_data.load_power.val16 = data[i++];
  g_renogy_data.panel_voltage.val16 = data[i++];
  g_renogy_data.panel_current.val16 = data[i++];
  g_renogy_data.panel_power.val16 = data[i++];
}

void renogySetError(uint16_t *data)
{
  g_renogy_data.error_status_1.val16 = data[0];
  g_renogy_data.error_status_2.val16 = data[1];
}

void renogyPollRs232Errors(void)
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[numErrorRegisters];
  
  i++;
  
  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  node.setTransmitBuffer(0, lowWord(i));
  
  // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
  node.setTransmitBuffer(1, highWord(i));
  
  // slave: read (17) 16-bit registers starting at register 0x100 to RX buffer
  result = node.readHoldingRegisters(errorStartRegister, numErrorRegisters);
  
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < numErrorRegisters; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
    renogySetError(data);
  }
  else
  {
    MYLOG("RS232","Modbus error %d", result);
  }
  
}

void renogyPollRs232Data(void)
{
  static uint32_t i;
  uint8_t j, result;
  uint16_t data[numDataRegisters];
  
  i++;
  
  // set word 0 of TX buffer to least-significant word of counter (bits 15..0)
  node.setTransmitBuffer(0, lowWord(i));
  
  // set word 1 of TX buffer to most-significant word of counter (bits 31..16)
  node.setTransmitBuffer(1, highWord(i));
  
  // slave: read (17) 16-bit registers starting at register 0x100 to RX buffer
  result = node.readHoldingRegisters(dataStartRegister, numDataRegisters);
  
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < numDataRegisters; j++)
    {
      data[j] = node.getResponseBuffer(j);
    }
    renogySetData(data);
  }
  else
  {
    MYLOG("RS232","Modbus error %d", result);
  }
  
}

void renogyPrintStatus(void)
{
  MYLOG("RS232","Battery Capacity: %d", g_renogy_data.batt_capacity.val16);
  MYLOG("RS232","Battery Voltage: %f", ((float)g_renogy_data.batt_voltage.val16 * 0.1));
  MYLOG("RS232","Battery Charge Current: %f", ((float)g_renogy_data.batt_charge_current.val16 * 0.01));
  MYLOG("RS232","Battery Temperature: %d", (g_renogy_data.temp.val8[0]));
  MYLOG("RS232","Control Temperature: %d", (g_renogy_data.temp.val8[1]));
  MYLOG("RS232","Load Voltage: %f", ((float)g_renogy_data.load_voltage.val16 * 0.1));
  MYLOG("RS232","Load Current: %f", ((float)g_renogy_data.load_current.val16 * 0.01));
  MYLOG("RS232","Load Power: %d", g_renogy_data.load_power.val16);
  MYLOG("RS232","Panel Voltage: %f", ((float)g_renogy_data.panel_voltage.val16 * 0.1));
  MYLOG("RS232","Panel Current: %f", ((float)g_renogy_data.panel_current.val16 * 0.01));
  MYLOG("RS232","Panel Power: %d", g_renogy_data.panel_power.val16);
  MYLOG("RS232","Error Status 1: 0x%02X", g_renogy_data.error_status_1.val16);
  MYLOG("RS232","Error Status 2: 0x%02X", g_renogy_data.error_status_2.val16);
}

/* 
Renogy Wanderer Error Codes

Number	Description
E0		No error detected
E01		Battery over-discharged
E02		Battery over-voltage
E04		Load short circuit
E05		Load overloaded
E06		Controller over-temperature
E08		PV input over-current
E10		PV over-voltage
E13		PV reverse polarity
E14		Battery reverse polarity

Read more: https://manuals.plus/renogy/solar-charge-controller-manual#ixzz7baexKyl6
*/

#define RENOGY_MAX_ERROR_CODE 14
const char *renogyErrorCodes[] = {
    "No error detected",
    "Battery over-discharged",
    "Battery over-voltage",
    "Unknown error",
    "Load short circuit",
    "Load overloaded",
    "Controller over-temperature",
    "Unknown error",
    "PV input over-current",
    "Unknown error",
    "PV over-voltage",
    "Unknown error",
    "Unknown error",
    "PV reverse polarity",
    "Battery reverse polarity",
};

const char *renogyDecodeErrorStatus(void)
{
#if 0 //TBD
#if MY_DEBUG == 1
    MYLOG("RS232", "error_status = 0x%02X", g_renogy_data.error_status.val16);
#endif 
    if(g_renogy_data.error_status_2 > RENOGY_MAX_ERROR_CODE)
        return "Unknown error";
    else
        return renogyErrorCodes[g_renogy_data.error_status_2];
#endif
}

#endif // ENABLE_RS232
