/**
   @file RAK1901_Temperature_Humidity_SHTC3.ino
   @author rakwireless.com
   @brief Setup and read values from a SHTC3 temperature and humidity sensor
   @version 0.1
   @date 2020-12-28
   @copyright Copyright (c) 2020
**/

#include "app.h"
#include "SparkFun_SHTC3.h" 		//Click here to get the library: http://librarymanager/All#SparkFun_SHTC3

#ifdef ENABLE_ENV_MON

SHTC3 g_shtc3;						      // Declare an instance of the SHTC3 class

// The errorDecoder function prints "SHTC3_Status_TypeDef" resultsin a human-friendly way
const char *errorDecoder(SHTC3_Status_TypeDef message)   
{
  switch (message)
  {
    case SHTC3_Status_Nominal:
      return "Nominal";
      break;
    case SHTC3_Status_Error:
      return "Error";
      break;
    case SHTC3_Status_CRC_Fail:
      return "CRC Fail";
      break;
    default:
      return "Unknown return code";
      break;
  }
}

void shtc3_read_data(void)
{
	float Temperature = 0;
	float Humidity = 0;
	
	g_shtc3.update();
	if (g_shtc3.lastStatus == SHTC3_Status_Nominal) // You can also assess the status of the last command by checking the ".lastStatus" member of the object
	{

		Temperature = g_shtc3.toDegC();			          // Packing LoRa data
		Humidity = g_shtc3.toPercent();
		
		MYLOG("ENV","RH = %f %\n", g_shtc3.toPercent()); // "toPercent" returns the percent humidity as a floating point number		
		if (g_shtc3.passRHcrc)  // Like "passIDcrc" this is true when the RH value is valid from the sensor (but not necessarily up-to-date in terms of time)
		{
			MYLOG("ENV","Checksum: pass\n");
		}
		else
		{
			MYLOG("ENV","Checksum: fail\n");
		}
		
        MYLOG("ENV","T = %f C\n", g_shtc3.toDegC()); // "toDegF" and "toDegC" return the temperature as a flaoting point number in deg F and deg C respectively	
		if (g_shtc3.passTcrc) // Like "passIDcrc" this is true when the T value is valid from the sensor (but not necessarily up-to-date in terms of time)
		{
			MYLOG("ENV","Checksum: pass\n");
		}
		else
		{
			MYLOG("ENV","Checksum: fail\n");
		}

        int16_t temp_int = (int16_t)(Temperature * 10.0);
	    uint16_t humid_int = (uint16_t)(Humidity * 2);

        g_tracker_data.humid_1 = (uint8_t)(humid_int);
	    g_tracker_data.temp_1 = (uint8_t)(temp_int >> 8);
	    g_tracker_data.temp_2 = (uint8_t)(temp_int);
	}
	else
	{
        MYLOG("ENV","Update failed, error: %s\n", errorDecoder(g_shtc3.lastStatus));
	}
}

int init_shtc3(void)
{
	MYLOG("ENV", "Beginning shtc3 sensor. Result = %s", errorDecoder(g_shtc3.begin())); // Most SHTC3 functions return a variable of the type "SHTC3_Status_TypeDef" to indicate the status of their execution

	if (g_shtc3.passIDcrc)                      // Whenever data is received the associated checksum is calculated and verified so you can be sure the data is true
	{					   						                    // The checksum pass indicators are: passIDcrc, passRHcrc, and passTcrc for the ID, RH, and T readings respectively
		MYLOG("ENV", "ID Passed Checksum. ");
		MYLOG("ENV", "Device ID: 0b%0b", g_shtc3.ID); // The 16-bit device ID can be accessed as a member variable of the object
	}
	else
	{
		Serial.println("ID Checksum Failed. ");
	}

    return !!g_shtc3.passIDcrc;
}

#endif // ENABLE_ENV_MON