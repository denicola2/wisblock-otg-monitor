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

    RJ12   <---> DB9M RS232 pin mapping
    P1(TX) <---> P3 (TX)
    P2(RX) <---> P2 (RX)
    P3(GND)<---> P5 (GND)

*/

#include "Arduino.h"
#include "app.h"

#ifdef ENABLE_RS232

renogy_data_s g_renogy_data;
uint8_t recvd_renogy_downlink = 0;

/* Settings to read from the charge controller */
static byte panel_voltage_msg[8] = {0xFF, 0x03, 0x01, 0x07, 0x00, 0x01};
static byte panel_current_msg[8] = {0xFF, 0x03, 0x01, 0x08, 0x00, 0x01};
static byte panel_power_msg[8] = {0xFF, 0x03, 0x01, 0x09, 0x00, 0x01};
static byte batt_voltage_msg[8] = {0xFF, 0x03, 0x01, 0x01, 0x00, 0x01};
static byte batt_current_msg[8] = {0xFF, 0x03, 0x01, 0x02, 0x00, 0x01};
static byte batt_percent_msg[8] = {0xFF, 0x03, 0x01, 0x00, 0x00, 0x01};
static byte load_voltage_msg[8] = {0xFF, 0x03, 0x01, 0x04, 0x00, 0x01};
static byte load_current_msg[8] = {0xFF, 0x03, 0x01, 0x05, 0x00, 0x01};
static byte load_power_msg[8] = {0xFF, 0x03, 0x01, 0x06, 0x00, 0x01};
static byte load_status_msg[8] = {0xFF, 0x03, 0x01, 0x20, 0x00, 0x01};
static byte error_status_msg[8] = {0xFF, 0x03, 0x01, 0x21, 0x00, 0x02};

static byte load_on[8] = {0xFF, 0x06, 0x01, 0x0A, 0x00, 0x01};
static byte load_off[8] = {0xFF, 0x06, 0x01, 0x0A, 0x00, 0x00};

static byte *func_out;
// static byte bitmask_chargestatus = 0b01111111;
String temp_str;
char temp[50];

void init_renogy_rs232(void)
{
  // If our RS232 Target is 5V, we need to enable WB_IO5 to enable 5V on the RAK1920
#ifdef ENABLE_RS232_5V
  pinMode(WB_IO5, OUTPUT);
  digitalWrite(WB_IO5, HIGH);
#endif
  Serial1.begin(9600);
}

static uint16_t ModRTU_CRC(byte message[], int sizeOfArray)
{
  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < sizeOfArray - 2; pos++)
  {
    crc ^= (uint16_t)message[pos]; // XOR byte into least sig. byte of crc

    for (int i = 8; i != 0; i--)
    { // Loop over each bit
      if ((crc & 0x0001) != 0)
      {            // If the LSB is set
        crc >>= 1; // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else         // Else LSB is not set
        crc >>= 1; // Just shift right
    }
  }

  // byte MSB = (crc >> 8) & 0xFF;
  // byte LSB = crc & 0xFF;

  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;
}

static byte recvd_data[16] = {};
static byte *querySlave(byte b[], int sizeOfArray)
{
  uint16_t crc = ModRTU_CRC(b, sizeOfArray);
  byte LSB = crc & 0xFF;        // comes first
  byte MSB = (crc >> 8) & 0xFF; // then second
  b[sizeOfArray - 2] = LSB;
  b[sizeOfArray - 1] = MSB;

  Serial1.write(b, sizeOfArray);
  int counter = 0;
  while ((Serial1.available() < 1) && (counter < 10))
  {
    counter += 1;
    delay(5);
  }

  counter = 0;
  while (Serial1.available() > 0)
  {
    recvd_data[counter] = Serial1.read();
    counter++;
  }

#if MY_DEBUG == 1
  MYLOG("RS232", "Read %d bytes from Renogy", counter);
  int i;
  for (i = 0; i < counter; i++)
  {
    MYLOG("RS232", "[%d] %02X", i, recvd_data[i]);
  }
#endif

  return recvd_data;
}

void renogyDownLinkDataHandle(uint8_t *buffer)
{
  if (buffer[0] == 0x00 && buffer[1] == 0x69)
  { // Turns on load
    querySlave(load_on, sizeof(load_on));
  }
  else if (buffer[0] == 0x00 && buffer[1] == 0x70)
  { // Turns on load
    querySlave(load_off, sizeof(load_off));
  }
  recvd_renogy_downlink = 1;
}

void renogyPollRs232(void)
{
  // Sends 9 basic metrics out for each of the nine below functions
  func_out = querySlave(panel_voltage_msg, sizeof(panel_voltage_msg));
  g_renogy_data.panel_voltage.val8[0] = func_out[3];
  g_renogy_data.panel_voltage.val8[1] = func_out[4];

  func_out = querySlave(panel_current_msg, sizeof(panel_current_msg));
  g_renogy_data.panel_current.val8[0] = func_out[3];
  g_renogy_data.panel_current.val8[1] = func_out[4];

  func_out = querySlave(panel_power_msg, sizeof(panel_power_msg));
  g_renogy_data.panel_power.val8[0] = func_out[3];
  g_renogy_data.panel_power.val8[1] = func_out[4];

  func_out = querySlave(batt_voltage_msg, sizeof(batt_voltage_msg));
  g_renogy_data.batt_voltage.val8[0] = func_out[3];
  g_renogy_data.batt_voltage.val8[1] = func_out[4];

  func_out = querySlave(batt_current_msg, sizeof(batt_current_msg));
  g_renogy_data.batt_current.val8[0] = func_out[3];
  g_renogy_data.batt_current.val8[1] = func_out[4];

  func_out = querySlave(batt_percent_msg, sizeof(batt_percent_msg));
  g_renogy_data.batt_percent.val8[0] = func_out[3];
  g_renogy_data.batt_percent.val8[1] = func_out[4];

  func_out = querySlave(load_voltage_msg, sizeof(load_voltage_msg));
  g_renogy_data.load_voltage.val8[0] = func_out[3];
  g_renogy_data.load_voltage.val8[1] = func_out[4];

  func_out = querySlave(load_current_msg, sizeof(load_current_msg));
  g_renogy_data.load_current.val8[0] = func_out[3];
  g_renogy_data.load_current.val8[1] = func_out[4];

  func_out = querySlave(load_power_msg, sizeof(load_power_msg));
  g_renogy_data.load_power.val8[0] = func_out[3];
  g_renogy_data.load_power.val8[1] = func_out[4];

  // gathers information on load status as well as battery charging mode
  func_out = querySlave(load_status_msg, sizeof(load_status_msg));
  g_renogy_data.load_status.val8[0] = func_out[3];
  g_renogy_data.load_status.val8[1] = func_out[4];

  func_out = querySlave(error_status_msg, sizeof(error_status_msg));
  g_renogy_data.error_status.val8[0] = func_out[3];
  g_renogy_data.error_status.val8[1] = func_out[4];

  g_renogy_data.recvd_downlink = recvd_renogy_downlink;
  recvd_renogy_downlink = 0;
}

void renogyPrintStatus(void)
{
  MYLOG("RS232", "====== Renogy Status: ======");
  MYLOG("RS232", "Panel Voltage: %f V", (float)g_renogy_data.panel_voltage.val16 * 0.1);
  MYLOG("RS232", "Panel Current: %f A", (float)g_renogy_data.panel_current.val16 * 0.01);
  MYLOG("RS232", "Panel Power: %d W", g_renogy_data.panel_power.val16);
  MYLOG("RS232", "Battery Voltage: %f V", (float)g_renogy_data.batt_voltage.val16 * 0.1);
  MYLOG("RS232", "Battery Current: %f A", (float)g_renogy_data.batt_current.val16 * 0.01);
  MYLOG("RS232", "Battery Percent: %d %", g_renogy_data.batt_percent.val16);
  MYLOG("RS232", "Load Voltage: %f V", (float)g_renogy_data.load_voltage.val16 * 0.1);
  MYLOG("RS232", "Load Current: %f A", (float)g_renogy_data.load_current.val16 * 0.01);
  MYLOG("RS232", "Load Power: %d W", g_renogy_data.load_power.val16);
  MYLOG("RS232", "Load Status: %d", g_renogy_data.load_status.val16);
  MYLOG("RS232", "Error Status: 0x%X ", g_renogy_data.error_status.val16);
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
#if MY_DEBUG == 1
  MYLOG("RS232", "error_status = 0x%02X", g_renogy_data.error_status.val16);
#endif
#if 0
    if(g_renogy_data.error_status_2 > RENOGY_MAX_ERROR_CODE)
        return "Unknown error";
    else
        return renogyErrorCodes[g_renogy_data.error_status_2];
#endif
  return "No error detected";
}

#endif // ENABLE_RS232