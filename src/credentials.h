/**
 * Optional hard-coded LoRaWAN credentials for OTAA and ABP.
 * It is strongly recommended to avoid duplicated node credentials
 * Options to setup credentials are
 * - over USB with AT commands
 * - over BLE with My nRF52 Toolbox
 */
//#define NODE_DEVICE_EUI {0x00, 0x0D, 0x75, 0xE6, 0x56, 0x4D, 0xC1, 0xF3} //!< Device EUI (8 bytes)
//#define NODE_APP_EUI    {0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02, 0x01, 0xE1} //!< Application EUI (8 bytes)
//#define NODE_APP_KEY    {0x2B, 0x84, 0xE0, 0xB0, 0x9B, 0x68, 0xE5, 0xCB, 0x42, 0x17, 0x6F, 0xE7, 0x53, 0xDC, 0xEE, 0x79} //!< Application key (16 bytes)

//DEV_EUI 02336A90DAD49DBF
//APP_EUI F18A54A56680658B
//APP_KEY A9253295B1CC1AD05E27C308319C766E

#define NODE_DEVICE_EUI {0x02, 0x33, 0x6a, 0x90, 0xda, 0xd4, 0x9d, 0xbf}
#define NODE_APP_EUI    {0xf1, 0x8a, 0x54, 0xa5, 0x66, 0x80, 0x65, 0x8b}
#define NODE_APP_KEY    {0xa9, 0x25, 0x32, 0x95, 0xb1, 0xcc, 0x1a, 0xd0, 0x5e, 0x27, 0xc3, 0x08, 0x31, 0x9c, 0x76, 0x6e}

#define NODE_NWS_KEY    {0x32, 0x3D, 0x15, 0x5A, 0x00, 0x0D, 0xF3, 0x35, 0x30, 0x7A, 0x16, 0xDA, 0x0C, 0x9D, 0xF5, 0x3F}
#define NODE_APPS_KEY   {0x3F, 0x6A, 0x66, 0x45, 0x9D, 0x5E, 0xDC, 0xA6, 0x3C, 0xBC, 0x46, 0x19, 0xCD, 0x61, 0xA1, 0x1E}