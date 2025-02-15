enum HK_COLOR
{
  TAN,
  GOLD,
  SILVER,
  BLACK
};

enum lockStates
{
  UNLOCKED,
  LOCKED,
  JAMMED,
  UNKNOWN,
  UNLOCKING,
  LOCKING
};

//Miscellaneous
#define HOMEKEY_COLOR BLACK
#define SETUP_CODE "46637726"  // HomeKit Setup Code (only for reference, has to be changed during WiFi Configuration or from WebUI)
#define OTA_PWD "homespan-ota" //custom password for ota
#define DEVICE_NAME "MIAU" //Device name
#define MDNS_NAME "miau"
#define GPIO_ACTION_ENABLE true
#define GPIO_ACTION_PIN 2
#define GPIO_ACTION_LOCK_STATE LOW
#define GPIO_ACTION_UNLOCK_STATE HIGH
#define GPIO_ACTION_MOMENTARY_STATE 0
#define GPIO_ACTION_MOMENTARY_TIMEOUT 5000

// WebUI
#define WEB_AUTH_ENABLED true
#define WEB_AUTH_USERNAME "ursem"
#define WEB_AUTH_PASSWORD "mobileIGNITION"
