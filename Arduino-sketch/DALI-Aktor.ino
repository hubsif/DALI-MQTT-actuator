/*
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
   MA 02110-1301  USA
*/

/*

  DOCUMENTATION
  -------------

  A DALI-Controller and Actuator with switch inputs, MQTT network interface and integrated DALI power supply.

  This device can be used to control DALI slaves and relay outputs per wall switches and per MQTT. It complies to
  the MQTT Homie convention, simplifying integration into several smarthome platforms. It supports DALI Commissioning
  (automatic short address assignment).

  On boot it scans the DALI bus and publishes homie information on all slaves found and groups set on these slaves.


  MQTT Homie topology:
  --------------------
  homie/<deviceid>/
    $name → "HDA-63"
    $homie → "4.0.0"
    $state → ["ready", "disconnected", "lost" (LWT)]
    $nodes → "config,button<X>..button<X>,relay<X>..relay<X>,slave<X>..slave<X>,group<X>..group<X>

    -- for each found slave and group:
    slave<X>/, group<X>/
      $name → "DALI Slave <X>" / "DALI Group <X>"
      $type" → "DALI Short Address" / "DALI Group Address"
      $properties" → "arc,cmd"

      arc/
        $name" → "Brightness Slave <X>" / "Brightness Group <X>"
        $datatype" → "string"
        $settable" → "true"
        -- publish format: "<ARC>"
        --                  ARC: current arc level (0 to 254)
        -- set format: "<ARC> [<FADETIME>]"
        --              ARC: arc level to send to slave
        --              FADETIME: (optional) fade time to (temporarily) use for setting arc level

      cmd/
        $name" → "Command Slave <X>" / "Command Group <X>"
        $datatype" → "integer"
        $format" → "0:254"
        $settable" → "true"

    -- for each relay
    relay<X>/
      $name → "Relay <X>"
      $type → "Relay"
      $properties → "on"

      on/
        $name → "Relay State <X>"
        $datatype → "boolean"
        $settable → "true"

    -- for each button
    button<X>/
      $name → "Button"
      $type → "Button"
      $properties → "config,1clicks,2clicks,3clicks,4clicks,5clicks"

      -- for each click count
      <Y>clicks/
        $name → "Button <X> - <X> Clicks"
        $datatype → "boolean"

    -- device configuration
    config/
      $name → "Config"
      $type → "config"
      $properties → "network,mqttbroker,deviceid"

      network/
        $name → "Network Config"
        $datatype → "string"
        $settable → "true"

      mqttbroker/
        $name → "MQTT Broker IP"
        $datatype → "string"
        $settable → "true"

      deviceid/
        $name → "Device ID"
        $datatype → "string"
        $settable → "true"

      -- TODO: unpublished yet:
      commission/
      save/
      reset/
      reboot/

    -- TODO: unpublished yet:
    broadcast/
      spc/



  // TODO:
  - add "scan bus" command (not only at reboot)?
  - when using group, publish arc of all slaves


  short / group addresses, scenes, relays:
  group addresses are converted to come after short addresses by adding 64
  which results in addresses being 0-63 for short addresses and 64-79 for group addresses
  80 to 95 are reserved for scenes support
  relays are addressed starting from 96, i.e. REL1=96, REL2=97, REL3=98

*/


// #define NO_DHCP  // saves about 5KB flash and 160B RAM

// ==========
// INCLUSIONS
// ==========

#include <Dali.h>          // DALI library
#include <Ethernet.h>      // Ethernet library for W5500
#include <PubSubClient.h>  // MQTT library
#include <EEPROM.h>        // EEPROM library for config persistence
#include <Timer.h>         // Timer library (from http://www.doctormonk.com/2012/01/arduino-timer-library.html)
#include <avr/wdt.h>       // Watchdog support


// ================
// CONSTANTS/CONFIG
// ================

// firmware version
const char FW_VERSION[] = "0.3";

// ---------------
// HARDWARE CONFIG
// ---------------

// DALI rx and tx pins
const byte DALI_TX_PIN = 1;
const byte DALI_RX_PIN = 0;

// relay output pins
const byte RELAY_PINS[] = { A2, A1, A0 };
const byte RELAY_COUNT = (sizeof(RELAY_PINS) / sizeof(byte));

// button input pins
const byte INPUT_PINS[] = { 3, 4, 5, 6, 7, 8 };
const byte BUTTON_COUNT = (sizeof(INPUT_PINS) / sizeof(byte));

// eeprom start address for config - to avoid conflict with bootloader
const int EEPROM_START_ADDRESS = 256;


// ---------------
// GLOBAL SETTINGS
// ---------------

// switch read sampling rate in milliseconds
const int BUTTON_SAMPLING_RATE = 50;

// max time between clicks and time after button is considered long pressed
const int BUTTON_CHANGE_TIME = 400;

// repeat time for dim commands (must be <200ms as this is how long one DALI dim step takes)
const int DIM_REPEAT_TIME =  150;

// arc level check count - dim observation interval to get and send current arc level
const int DIM_UPDATE_INTERVAL = 500;
// arc level check count depending on fadetime (0-15)
const int DIM_UPDATE_COUNT[] = { 2, 3, 3, 4, 5, 7, 9, 13, 17, 24, 33, 47, 65, 92, 129, 184 };

// highest DALI short address to scan for. limited to save space
const int MAX_SHORT_ADDRESS = 9;


// --------------
// NETWORK CONFIG
// --------------

// ethernet mac address
const byte MAC_ADDRESS[] = { 0xDE, 0xAD, 0xBE, 0x00, 0x02, 0x02 };

// network link check interval
const int NETWORK_CHECK_INTERVAL = 5000;


// ---------------------
// MQTT / HOMIE SETTINGS
// ---------------------

// MQTT connection check interval in milliseconds
const int  MQTT_CHECK_INTERVAL = 30000;

// mqtt client id (based on mac address)
const char MQTT_CLIENT_ID[] = "HDA63_DEADBE000202";

// homie device friendly name
const char HOMIE_DEVICE_NAME_P[] PROGMEM = "HDA-63";

// MQTT broker default values
const byte MQTT_DEFAULT_BROKER_IP_P[4] PROGMEM = { 192, 168, 178, 10 };
const int  MQTT_DEFAULT_BROKER_PORT = 1883;

// mqtt homie default device id (based on mac address)
const char HOMIE_DEFAULT_DEVICEID_P[] PROGMEM = "hda63-0202";
const byte HOMIE_DEVICEID_MAXLENGTH = 15;

// mqtt homie base topic
const char HOMIE_BASE_TOPIC_P[] PROGMEM = "homie/";
const byte HOMIE_BASE_TOPIC_LENGTH = 6;  // strlen_P is not compiler optimized, set base topic length manually

// mqtt length limits
const byte MQTT_TOPICSUFFIX_MAXLENGTH = 30;
const byte MQTT_PAYLOAD_MAXLENGTH = 20;
const int  MQTT_TOPIC_MAXLENGTH = HOMIE_BASE_TOPIC_LENGTH + HOMIE_DEVICEID_MAXLENGTH + MQTT_TOPICSUFFIX_MAXLENGTH;
//           - "config"  ^= 7 byte           =>  7 bytes
// 6 buttons - "buttonX" ^= 8 byte           => 48 bytes
// 3 relays  - "relayX"  ^= 7 byte           => 21 bytes
// 10 slaves - "slaveXX" ^= 8 byte (7+comma) => 80 bytes
// 4 groups  - "groupX"  ^= 7 byte           => 28 bytes
const byte HOMIE_NODES_MAXLENGTH = 7 + BUTTON_COUNT * 8 + RELAY_COUNT * 7 + (MAX_SHORT_ADDRESS + 1) * 8 + 4 * 7;
//const byte HOMIE_NODES_MAXLENGTH = 180;


// strings in program memory
const char HOMIE_STATE_P[] PROGMEM = "$state";
const char HOMIE_HOMIE_P[] PROGMEM = "$homie";
const char HOMIE_NAME_P[] PROGMEM = "$name";
const char HOMIE_TYPE_P[] PROGMEM = "$type";
const char HOMIE_NODES_P[] PROGMEM = "$nodes";
const char HOMIE_PROPERTIES_P[] PROGMEM = "$properties";
const char HOMIE_DATATYPE_P[] PROGMEM = "$datatype";
const char HOMIE_FORMAT_P[] PROGMEM = "$format";
const char HOMIE_SETTABLE_P[] PROGMEM = "$settable";
const char HOMIE_INTEGER_P[] PROGMEM = "integer";
const char HOMIE_BOOLEAN_P[] PROGMEM = "boolean";
const char HOMIE_STRING_P[] PROGMEM = "string";

const char SLAVE_P[] PROGMEM = "slave";
const char GROUP_P[] PROGMEM = "group";
const char BUTTON_P[] PROGMEM = "button";
const char RELAY_P[] PROGMEM = "relay";

const char CONFIG_P[] PROGMEM = "config";
const char DEVICEID_P[] PROGMEM = "deviceid";
const char MQTTBROKER_P[] PROGMEM = "mqttbroker";
const char NETWORK_P[] PROGMEM = "network";


// ================
// GLOBAL VARIABLES
// ================

// button mapping to action (relays, dali short or dali group)
//  0-63 = DALI Short Address 0-63
// 63-79 = DALI Group Address 0-15
// 80-95 = reserved for DALI Scene 0-15
// 96-98 = Relays 0-2
byte buttonMappings[BUTTON_COUNT][5];

// timer ID storage for button dimming
char buttonTimers[BUTTON_COUNT];

// slave settings
struct slaveConf {
  boolean present;
  word groups;
} slaves[MAX_SHORT_ADDRESS];

// dim direction (0 = up, 1 = down) for each slave
byte dimDirection[MAX_SHORT_ADDRESS];

// ethernet variables
byte netIP[4], netMask[4], netGW[4];

// mqtt variables
byte mqttBrokerIP[4];
int mqttBrokerPort;
char deviceID[HOMIE_DEVICEID_MAXLENGTH];
int mqttTimer = -1;
int dimUpdateTimer[MAX_SHORT_ADDRESS];

EthernetClient mqttEthClient;
PubSubClient mqttClient(mqttEthClient);

Timer timer;

bool ethHasBeenDown = false;



/* DECLARATIONS */
void setArc(byte address, byte value, int fadetime = -1);
void eepromWriteString(int &address, char * str, byte maxLength = 25);


/* ***********************************************************************
   MAIN ROUTINES
 * *********************************************************************** */

void setup() {

  for (byte i = 0; i < MAX_SHORT_ADDRESS; i++)
    dimUpdateTimer[i] = -1;

  initPins(); // initialize input and output pins

  initConfig(false); // load config from eeprom

  wdt_enable(WDTO_8S); // enable watchdog

  Dali.begin(DALI_TX_PIN, DALI_RX_PIN); // start DALI

  timer.every(BUTTON_SAMPLING_RATE, read_buttons_timerCallback, (void*) 0); // start button polling

  scanSlaves(); // scan for slaves

  timer.every(NETWORK_CHECK_INTERVAL, checkNetwork_timerCallback, (void*) 0); // check network link and connect MQTT
}

void loop() {
  // Timer tick
  timer.update();

  // MQTT loop
  mqttClient.loop();


#ifndef NO_DHCP
  // presumedly required at least for DHCP
  Ethernet.maintain();
#endif

  // DALI tick (only required for commissioning)
  Dali.commission_tick();

  // reset watchdog with each loop
  wdt_reset();
}


/* ***********************************************************************
   INITIALIZATIONS
 * *********************************************************************** */

void initPins() {
  // set button pins to inputs
  for (byte i = 0; i < BUTTON_COUNT; i++)
    pinMode(INPUT_PINS[i], INPUT);

  // set relay pins to outputs at low state
  for (byte i = 0; i < RELAY_COUNT; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    digitalWrite(RELAY_PINS[i], LOW);
  }
}

void initConfig(bool defaults) {
  if (eepromFresh() || defaults) {
    // initialize variables with default values
    for (byte i = 0; i < 4; i++)
      netIP[i] = netMask[i] = netGW[i] = 0;

    for (byte i = 0; i < BUTTON_COUNT; i++) {
      buttonMappings[i][0] = i;
      buttonMappings[i][1] = buttonMappings[i][2] = buttonMappings[i][3] = buttonMappings[i][4] = 0;
    }

    memcpy_P(mqttBrokerIP, MQTT_DEFAULT_BROKER_IP_P, 4);
    mqttBrokerPort = MQTT_DEFAULT_BROKER_PORT;
    strcpy_P(deviceID, HOMIE_DEFAULT_DEVICEID_P);

    writeConfig();
  }
  else
    readConfig();
}


/* ***********************************************************************
   NETWORK FUNCTIONS
 * *********************************************************************** */
void checkNetwork_timerCallback(void * context) {
  static EthernetLinkStatus ethLink, oldEthLink = LinkOFF;

  ethLink = Ethernet.linkStatus();

  if (ethLink != oldEthLink)
    if (ethLink == LinkON || ethLink == Unknown) {
      // Link changed to active, start networking
      startEthernet();
      startMqtt();
    }
    else
      stopMqtt();

  oldEthLink = ethLink;

  // reestablish network when it has been lost (e.g. due to relays interference)
  if (ethLink == LinkON && Ethernet.localIP() == 0) {
    ethHasBeenDown = true;
    startEthernet();
    startMqtt();
  }

}

void startEthernet() {
  if (netIP[0] != 0)
    Ethernet.begin(MAC_ADDRESS, netIP, netIP, netGW, netMask); // use static network address
#ifndef NO_DHCP
  else
    Ethernet.begin(MAC_ADDRESS);// no static address defined, use DHCP
#endif
}


/* ***********************************************************************
   MQTT FUNCTIONS
 * *********************************************************************** */

/** publish current value for a Homie property
   @param node      node name
   @param property  property name
   @param payload   property value (MQTT payload)
   @param retained  retain MQTT message */
void publishHomiePropertyValue(const char * node, const char * property, const char * value, bool retained = true) {
  char topic[MQTT_TOPIC_MAXLENGTH];
  strcpy_BaseTopic(topic);

  strcat(topic, node); strcat(topic, "/");
  strcat(topic, property);

  mqttClient.publish(topic, value, retained);
}

/** publish Homie property specs
    @param nodes         properties list to append property to
    @param node          node name
    @param property      property name
    @param friendlyName  property friendly name
    @param datatype_P    datatype (in PROGMEM)
    @param settable      settable attribute
    @param format        format attribute */
void publishHomieProperty(char * properties, const char * node, const char * property, const char * friendlyName, const char * datatype_P, const boolean settable = false, const char * format = NULL) {
  char topic[MQTT_TOPIC_MAXLENGTH];
  strcpy_BaseTopic(topic);

  strcat(topic, node); strcat(topic, "/");
  strcat(topic, property); strcat(topic, "/");

  publishMQTT_RP_R(topic, HOMIE_NAME_P, friendlyName);
  publishMQTT_RP_P(topic, HOMIE_DATATYPE_P, datatype_P);
  if (settable) {
    publishMQTT_RP_R(topic, HOMIE_SETTABLE_P, "true");
    if (format != NULL)
      publishMQTT_RP_R(topic, HOMIE_FORMAT_P, format);
  }

  if (strlen(properties) > 0) strcat(properties, ",");
  strcat(properties, property);
}

/** publish Homie node specs
    @param nodes         nodes list to append node to
    @param node          node name
    @param friendlyName  node friendly name
    @param type_P        type string (in PROGMEM)
    @param properties    properties to publish for node */
void publishHomieNode(char * nodes, const char * node, const char * friendlyName, const char * type_P, const char * properties) {
  char topic[MQTT_TOPIC_MAXLENGTH];
  strcpy_BaseTopic(topic);

  strcat(topic, node); strcat(topic, "/");

  publishMQTT_RP_R(topic, HOMIE_NAME_P, friendlyName);
  publishMQTT_RP_P(topic, HOMIE_TYPE_P, type_P);
  publishMQTT_RP_R(topic, HOMIE_PROPERTIES_P, properties);

  if (strlen(nodes) > 0) strcat(nodes, ",");
  strcat(nodes, node);
}

/** check if the MQTT connection is established and if not connect and publish Homie info.
    Called regularly by a timer */
void mqttConnect_timerCallback(void * context) {
  if (!mqttClient.connected()) {                                                                            // check if we're connected to the MQTT Broker
    char topicPrefix[MQTT_TOPIC_MAXLENGTH];                                                                 // char array used for topic creation
    strcpy_BaseTopic_cat_P(topicPrefix, HOMIE_STATE_P);                                                     // set topic to "homie/<device-id>/$state" for LWT

    if (mqttClient.connect(MQTT_CLIENT_ID, topicPrefix, 0, true, "lost")) {                                 // connect to MQTT broker with LWT $state -> "lost" following Homie convention
      char node[8], property[12], friendlyName[20];
      char payload[48];                                                                                     // payload string, longest is network config with 3 IPs, max 15 chars each + 2 spaces
      char properties[47];                                                                                  // properties string, longest is buttons with "config" and 5x "Xclicks" + commas
      char nodes[HOMIE_NODES_MAXLENGTH];                                                                    // string to store nodes payload. due to low memory limited HOMIE_NODES_MAXLENGTH
      nodes[0] = '\0';                                                                                      // initialize empty string for upcoming strcat

      // *** NODE: config ***
      properties[0] = '\0';                                                                                 // initialize empty string for upcoming strcat
      strcpy_P(node, CONFIG_P);                                                                             // set node name to "config"

      // *** PROPERTY: network ***
      payload[0] = '\0';                                                                                    // initialize empty string for upcoming strcat
      strcatIP(payload, netIP); strcat(payload, " ");                                                       // get network configuration ...
      strcatIP(payload, netMask); strcat(payload, " ");                                                     // ... and create string of style ...
      strcatIP(payload, netGW);                                                                             // ... "0.0.0.0 0.0.0.0 0.0.0.0" (IP MSK GW)

      strcpy_P(property, NETWORK_P);                                                                        // set property name
      strcpy_P(friendlyName, PSTR("Network Config"));                                                       // set friendly name

      publishHomiePropertyValue(node, property, payload);                                                   // publish current network configuration
      publishHomieProperty(properties, node, property, friendlyName, HOMIE_STRING_P, true);                 // publish network property specs

      // *** PROPERTY: mqttbroker ***
      payload[0] = '\0';                                                                                    // initialize empty string for upcoming strcat
      strcatIP(payload, mqttBrokerIP);

      strcpy_P(property, MQTTBROKER_P);                                                                     // set property name
      strcpy_P(friendlyName, PSTR("MQTT Broker IP"));                                                       // set friendly name

      publishHomiePropertyValue(node, property, payload);                                                   // publish current MQTT broker IP
      publishHomieProperty(properties, node, property, friendlyName, HOMIE_STRING_P, true);                 // publish network MQTT broker propery specs

      // *** PROPERTY: deviceid ***
      strcpy_P(property, DEVICEID_P);                                                                       // set property name
      strcpy_P(friendlyName, PSTR("Device ID"));                                                            // set friendly name

      publishHomiePropertyValue(node, property, deviceID);                                                  // publish current network configuration
      publishHomieProperty(properties, node, property, friendlyName, HOMIE_STRING_P, true);                 // publish network property specs

      publishHomieNode(nodes, node, "Config", CONFIG_P, properties);                                        // publish config node

      // *** NODE: buttonX ***
      for (byte i = 0; i < BUTTON_COUNT; i++) {                                                             // publish homie info for every button
        properties[0] = '\0';                                                                               // initialize empty string for upcoming strcat
        strcpy_P_catByte99(friendlyName, PSTR("Config Button "), i);                                        // set friendly name
        strcpy_P_catByte99(node, BUTTON_P, i);                                                              // set node name to "buttonX"

        // *** PROPERTY: config ***
        strcpy_P(property, CONFIG_P);
        publishButtonConfig(i);
        publishHomieProperty(properties, node, property, friendlyName, HOMIE_STRING_P, true);               // publish button property specs

        // *** PROPERTY: Xclicks ***
        for (byte j = 0; j < 5; j++) {                                                                      // publish button pushes
          property[0] = '\0';                                                                               // initialize empty string for upcoming strcat
          strcatByte99(property, j + 1);                                                                    // set property ...
          strcat(property, "clicks");                                                                       // to "Xclicks"

          strcpy_P_catByte99(friendlyName, PSTR("Button "), i);                                             // set friendlyName to ...
          strcat(friendlyName, " - ");                                                                      // "Button X - Y Clicks"
          strcatByte99(friendlyName, j);
          strcat_P(friendlyName, PSTR(" Clicks"));

          publishHomieProperty(properties, node, property, friendlyName, HOMIE_BOOLEAN_P, false);           // publish "Xclicks" property
        }

        publishHomieNode(nodes, node, "Button", PSTR("Button"), properties);                                // publish "buttonX" node
      }

      // *** NODE: slaveX ***
      for (byte i = 0; i < MAX_SHORT_ADDRESS; i++)                                                          // for all (allowed) slave addresses ...
        if (slaves[i].present == true)                                                                      // check if slave has been found
          publishHomieSlaveOrGroup(i, Dali.DALI_SHORT_ADDRESS, nodes);                                      // publish Homie specs at "homie/<device-id>/slave<X>"


      // *** NODE: groupX ***
      for (byte i = 0; i < 16; i++) {                                                                       // iterate through all possible group addresses
        bool set = false;                                                                                   // set current group to not being used in any slave
        for (byte j = 0; j < MAX_SHORT_ADDRESS; j++)                                                        // check if in any slave ...
          if (slaves[j].groups & (1 << i)) {                                                                // ... current group is enabled ...
            set = true;
            break;
          }
        if (set)                                                                                            // ... and if so ...
          publishHomieSlaveOrGroup(i, Dali.DALI_GROUP_ADDRESS, nodes);                                      // publish Homie specs at "homie/<device-id>/group<X>"
      }
      /* takes exact same amount of space as above, but has worse readbility?
        word groups = 0;
        for (byte i = 0; i < MAX_SHORT_ADDRESS; i++)
        groups |= slaves[i].groups;
        for (byte i = 0; i < 16; i++)
        if (groups & (1 << i))
          publishHomieSlaveOrGroup(i, Dali.DALI_GROUP_ADDRESS, nodes); */

      // *** NODE: relayX ***
      for (byte i = 0; i < RELAY_COUNT; i++) {                                                              // iterate through all relays
        strcpy_P_catByte99(node, RELAY_P, i);                                                               // set node name to "relayX"

        // *** PROPERTY: on ***
        properties[0] = '\0';                                                                               // initialize empty string for upcoming strcat
        strcpy_P_catByte99(friendlyName, PSTR("Relay State "), i);                                          // set friendly name to "Relay State X"
        publishHomieProperty(properties, node, "on", friendlyName, HOMIE_BOOLEAN_P, true);                  // publish Homie property specs at "homie/<device-id>/relay<X>/on"

        strcpy_P_catByte99(friendlyName, PSTR("Relay "), i);                                                // set friendly name to "Relay X"
        publishHomieNode(nodes, node, friendlyName, PSTR("Relay"), properties);                             // publish Homie node specs at "homie/<device-id>/relay<X>"
      }

      // *** DEVICE ***
      strcpy_BaseTopic(topicPrefix);                                                                        // set prefix to "homie/<device-id>/"
      publishMQTT_RP_P(topicPrefix, HOMIE_STATE_P, PSTR("ready"));                                          // publish "homie/<device-id>/$state" -> "ready"
      publishMQTT_RP_P(topicPrefix, HOMIE_HOMIE_P, PSTR("4.0.0"));                                          // publish "homie/<device-id>/$homie" -> "4.0.0"
      publishMQTT_RP_P(topicPrefix, HOMIE_NAME_P, HOMIE_DEVICE_NAME_P);                                           // publish "homie/<device-id>/$name"  -> "HDA63 DALI"

      strcat_P(topicPrefix, HOMIE_NODES_P);                                                                 // set topic(-prefix) to "homie/<device-id>/$nodes"
      int len = strlen(nodes);                                                                              // store nodes length
      mqttClient.beginPublish(topicPrefix, len, true);                                                      // use begin-/end-Publish so we can send long nodes string
      mqttClient.write(nodes, len);
      mqttClient.endPublish();

      strcpy_BaseTopic_cat_P(topicPrefix, PSTR("+/+/set"));                                                 // subscribe to "homie/<deviceid>/+/+/set"
      mqttClient.subscribe(topicPrefix);

      if (ethHasBeenDown)
        mqttClient.publish("homie/hda63-0201/ethhasbeendown", "");
      ethHasBeenDown = false;
    }
  }
}

/** publish Homie info on slaves and groups
    @param number  slave or group number
    @param type    slave or group
    @param nodes   pointer to nodes list to append slave/group to */
void publishHomieSlaveOrGroup(byte number, DaliClass::daliAddressTypes type, char * nodes) {
  char properties[8];
  properties[0] = '\0';

  char node[8];
  strcpy_P_catByte99(node, (type == Dali.DALI_SHORT_ADDRESS) ? SLAVE_P : GROUP_P, number);

  char friendlyName[20];
  strcpy_P_catByte99(friendlyName, (type == Dali.DALI_SHORT_ADDRESS) ? PSTR("Brightness Slave ") : PSTR("Brightness Group "), number);

  //publishHomieProperty(properties, node, "arc", friendlyName, HOMIE_INTEGER_P, true, "0:254");
  publishHomieProperty(properties, node, "arc", friendlyName, HOMIE_STRING_P, true);

  strcpy_P_catByte99(friendlyName, (type == Dali.DALI_SHORT_ADDRESS) ? PSTR("Command Slave ") : PSTR("Command Group "), number);
  publishHomieProperty(properties, node, "cmd", friendlyName, HOMIE_INTEGER_P, true, "0:254");

  strcpy_P_catByte99(friendlyName, (type == Dali.DALI_SHORT_ADDRESS) ? PSTR("DALI Slave ") : PSTR("DALI Group "), number);
  publishHomieNode(nodes, node, friendlyName, (type == Dali.DALI_SHORT_ADDRESS) ? PSTR("DALI Short Address") : PSTR("DALI Group Address"), properties);
}

void publishButtonConfig(byte buttonNo) {
  char node[8], property[7], payload[20];
  strcpy_P_catByte99(node, BUTTON_P, buttonNo);                                                       // set node name to "buttonX"

  payload[0] = '\0';                                                                                  // initialize empty string for upcoming strcat
  for (byte i = 0; i <= 4; i++) {                                                                     // collect assignments for each click count ...
    if (i > 0) strcat(payload, " ");                                                                  // ... and create string of style ...
    strcatByte99(payload, buttonMappings[buttonNo][i]);                                                      // ... "0 0 0 0 0"
  }

  strcpy_P(property, CONFIG_P);
  publishHomiePropertyValue(node, property, payload);                                                 // publish current button config settings
}

void publishButtonClicked(byte buttonNo, byte clicks) {
  char node[8], property[8];
  strcpy_P_catByte99(node, BUTTON_P, buttonNo);

  property[0] = '\0';
  strcatByte99(property, clicks);
  strcat(property, "clicks");

  publishHomiePropertyValue(node, property, "true", false);

}

void publishDaliResponse(char * topicSuffix, int response) {
  char topic[MQTT_TOPIC_MAXLENGTH], itoabuf[7];
  strcpy_BaseTopic(topic);
  strcat(topic, topicSuffix);
  mqttClient.publish(topic, itoa(response, itoabuf, 10), false);
}

void publishArc(int address, int arc, DaliClass::daliAddressTypes type = Dali.DALI_SHORT_ADDRESS) {
  char node[8];
  strcpy_P_catByte99(node, (type == Dali.DALI_SHORT_ADDRESS) ? SLAVE_P : GROUP_P, address);

  char payload[6];
  itoa(arc, payload, 10);

  publishHomiePropertyValue(node, "arc", payload);
}

void publishRelay(byte relayNo, bool on) {
  char node[7];
  strcpy_P_catByte99(node, RELAY_P, relayNo);

  char payload[6];
  strcpy(payload, (on) ? "true" : "false");

  publishHomiePropertyValue(node, "on", payload);
}

/** set MQTT parameters and (re-)connect timer, which checks every MQTT_CHECK_INTERVAL
    if we're still connected and if not reconnects and publishes Homie info */
void startMqtt() {
  mqttClient.setServer(mqttBrokerIP, mqttBrokerPort);                                                       // set MQTT broker and port to connect to
  mqttClient.setCallback(onMqttMessage);                                                                    // define callback function to be called on incoming subscribed messages

  if (mqttTimer >= 0) timer.stop(mqttTimer);                                                                // stop reconnect timer if it already exists
  mqttConnect_timerCallback((void*)0);
  mqttTimer = timer.every(MQTT_CHECK_INTERVAL, mqttConnect_timerCallback, (void*)0);                        // set reconnect timer
}

/** stop the MQTT connection
    and properly set the Homie device state to "disconnected" */
void stopMqtt() {
  char topic[MQTT_TOPIC_MAXLENGTH];
  strcpy_BaseTopic(topic);                                                                                  // set prefix to "homie/<device-id>"
  publishMQTT_RP_P(topic, HOMIE_STATE_P, PSTR("disconnected"));                                            // publish "homie/<device-id>/$state" -> "disconnected"

  mqttClient.disconnect();                                                                                  // disconnect from MQTT broker
  if (mqttTimer >= 0) {                                                                                     // disable reconnect timer
    timer.stop(mqttTimer);
    mqttTimer = -1;
  }
}

/** MQTT callback function
    called on incoming subscribed messages */
void onMqttMessage(const char * topic, byte * payload, unsigned int length) {

  char mytopic[MQTT_TOPICSUFFIX_MAXLENGTH], mypayload[MQTT_PAYLOAD_MAXLENGTH];

  if (strlen(topic) >= MQTT_TOPIC_MAXLENGTH || length >= MQTT_PAYLOAD_MAXLENGTH) return;                    // validate length of topic and payload so their copies fit into our char arrays
  strcpy(mytopic, topic + HOMIE_BASE_TOPIC_LENGTH + strlen(deviceID) + 1);                                  // copy variable part of topic (everything after "homie/<device-id>/")
  mytopic[strlen(mytopic) - 4] = '\0';                                                                      // we only subscribe to topics ending with "/set", so strip that

  strncpy(mypayload, payload, length);                                                                      // copy ...
  mypayload[length] = '\0';                                                                                 // ... and null-terminate payload

  word address, val[12];                                                                                    // variables to store parsed numbers

  byte slave_or_group = 2;
  if (strncmp_P(mytopic, SLAVE_P, 5) == 0) slave_or_group = Dali.DALI_SHORT_ADDRESS;
  else if (strncmp_P(mytopic, GROUP_P, 5) == 0) slave_or_group = Dali.DALI_GROUP_ADDRESS;
  if (slave_or_group < 2 && parseNumbers(mytopic + 5, &address, 1) == 1 && address < MAX_SHORT_ADDRESS) {   // topic starts with "slaveX/..." or "groupX/..."
    byte ptradd = (address >= 10) ? 1 : 0;
    if (slave_or_group == Dali.DALI_GROUP_ADDRESS)
      address += 64;

    // *** slaveX/arc/set *** : set slave ARC level
    if (strcmp(mytopic + 6 + ptradd, "/arc") == 0) {                                                        // check topic
      byte res = parseNumbers(mypayload, val, 2);                                                           // though we define the homie arc property to be integer, we accept an additional integer as fade time
      byte address2 = address;
      if (res == 1 && val[0] <= 254)                                                                        // validate parsing result, then set arc
        setArc(address, val[0]);
      else if (res == 2 && val[1] <= 15)
        setArc(address, val[0], val[1]);
    }

    // *** slaveX/cmd/set *** : send command to slave
    if (strcmp(mytopic + 6 + ptradd, "/cmd") == 0)                                                          // check topic
      if (parseNumbers(mypayload, val, 1) == 1 && val[0] <= 255) {                                          // parse and validate payload
        int resp = sendCmdWait(address, val[0]);                                                            // send DALI command to the bus
        if (slave_or_group == Dali.DALI_SHORT_ADDRESS)                                                      // group responses are not reliable
          publishDaliResponse(mytopic, resp);                                                               // publish DALI response at "homie/<device-id>/slaveX/cmd"
      }
  }

  if (strncmp(mytopic, "broadcast/", 10) == 0) {                                                            // check topic
    // *** broadcast/cmd/set *** : send broadcast command
    if (strcmp(mytopic + 10, "cmd") == 0 && parseNumbers(mypayload, val, 1) == 1) {                         // parse and validate payload
      int resp = Dali.sendCmdWait(255, val[0], Dali.DALI_GROUP_ADDRESS);                                    // send DALI special command to the bus
      publishDaliResponse(mytopic, resp);                                                                   // publish DALI response at "homie/<device-id>/broadcast/cmd"
    }

    // *** broadcast/spc/set *** : send special command (broadcast)
    if (strcmp(mytopic + 10, "spc") == 0
        && parseNumbers(mypayload, val, 2) == 2 & val[0] >= 256 && val[0] <= 287 && val[1] <= 255) {        // parse and validate payload
      int resp = Dali.sendSpecialCmdWait(val[0], val[1]);                                                   // send DALI special command to the bus
      if (val[0] == 258 || val[0] == 259)
        resp = Dali.sendSpecialCmdWait(val[0], val[1]);                                                     // send DALI special command to the bus again if required twice
      publishDaliResponse(mytopic, resp);                                                                   // publish DALI response at "homie/<device-id>/broadcast/spc"
    }
  }
  /*
    // *** broadcast/spc/set *** : send special command broadcast
    if (strcmp(mytopic, "broadcast/spc") == 0)                                                                // check topic
      if (parseNumbers(mypayload, val, 2) == 2 & val[0] >= 256 && val[0] <= 287 && val[1] <= 255) {           // parse and validate payload
        int resp = Dali.sendSpecialCmdWait(val[0], val[1]);                                                   // send DALI special command to the bus
        if (val[0] == 258 || val[0] == 259)
          resp = Dali.sendSpecialCmdWait(val[0], val[1]);                                                     // send DALI special command to the bus again if required twice
        publishDaliResponse(mytopic, resp);                                                                   // publish DALI response at "homie/<device-id>/broadcast/spc"
      }
  */
  // *** relayX/on/set *** : set relay state
  if (strncmp_P(mytopic, RELAY_P, 5) == 0 && parseNumbers(mytopic + 5, &address, 1) == 1 && address >= 0    // check topic, parse and validate its relay number
      && address <= 3 && strcmp(mytopic + 6, "/on") == 0) {
    if (strcmp(mypayload, "true") == 0)                                                                     // parse boolean payload and set relay
      switchRelay(address, HIGH);
    else if (strcmp(mypayload, "false") == 0)
      switchRelay(address, LOW);
  }

  // *** buttonX/config/set *** : set button configuration
  if (strncmp(mytopic, "button", 6) == 0 && parseNumbers(mytopic + 6, &address, 1) == 1
      && strcmp_P(mytopic + 8, CONFIG_P) == 0) {                                                          // check topic, parse and validate its button number
    byte no = parseNumbers(mypayload, val, 4);                                                              // parse button configuration
    for (byte i = 0; i < no; i++)                                                                           // store parsed result
      buttonMappings[address][i] = val[i];
    publishButtonConfig(address);                                                                           // publish new config
  }


  if (strncmp(mytopic, "config/", 7) == 0) {

    // *** config/commission/set *** : perform DALI commissioning
    if (strcmp(mytopic + 7, "commission") == 0) {                                                          // check topic
      if (strcmp(mypayload, "true") == 0) {                                                                // parse boolean payload
        byte maxCurrentAddress = 0;
        for (byte i = 0; i < MAX_SHORT_ADDRESS; i++)                                                       // for all (allowed) slave addresses ...
          if (slaves[i].present == true)                                                                   // check if slave has been found
            maxCurrentAddress = i;                                                                         // record address
        Dali.commission(maxCurrentAddress + 1, true);                                                      // start commissioning only new (addressless) slaves
      }
      else if (strcmp(mypayload, "false") == 0)
        Dali.commission();                                                                                 // start commissioning all

      char topic[MQTT_TOPIC_MAXLENGTH];
      strcpy_BaseTopic_cat_P(mytopic, PSTR("config/commission"));
      mqttClient.publish(mytopic, "OK", false);
      // TODO: inform when done
    }

    // *** config/save/set *** : save configuration to eeprom
    if (strcmp_P(mytopic + 7, PSTR("save")) == 0)
      writeConfig();

    // *** config/reset/set *** : reset configuration to defaults
    if (strcmp_P(mytopic + 7, PSTR("reset")) == 0)
      initConfig(true);

    // *** config/reboot/set *** : reboot device (e.g. to enter bootloader for firmware update)
    if (strcmp_P(mytopic + 7, PSTR("reboot")) == 0) {                                                       // check topic
      stopMqtt();                                                                                           // disconnect MQTT
      wdt_enable(WDTO_15MS);                                                                                // set watchdog timer to minimum
      for (;;) {}                                                                                           // endless loop to trigger watchdog
    }

    // *** config/network/set *** : set network configuration
    if (strcmp_P(mytopic + 7, NETWORK_P) == 0) {
      byte result = parseNumbers(mypayload, val, 12);
      if (result > 0 && result & 3 == 0) {  // result is multiple of 4
        copyIp(netIP, val);
        if (result >= 8) copyIp(netMask, val[4]);
        if (result == 12) copyIp(netGW, val[8]);
      }
    }

    // *** config/mqttbroker/set *** : set IP of MQTT broker
    if (strcmp_P(mytopic + 7, MQTTBROKER_P) == 0 && parseNumbers(mypayload, val, 4) == 4)
      copyIp(mqttBrokerIP, val);

    // *** config/deviceid/set *** : set Device ID
    if (strcmp_P(mytopic + 7, DEVICEID_P) == 0 && length < HOMIE_DEVICEID_MAXLENGTH) {
      int tmpaddr = EEPROM_START_ADDRESS + 1;
      eepromWriteString(tmpaddr, mypayload);
    }

  }
}


/* ***********************************************************************
   DALI FUNCTIONS
 * *********************************************************************** */

/** scan the DALI bus for slaves by requesting the status from 0 to MAX_SHORT_ADDRESS
*/
void scanSlaves() {
  int response;
  for (byte i = 0; i < MAX_SHORT_ADDRESS; i++) {                                                            // search for slaves from address 0 to MAX_SHORT_ADDRESS
    wdt_reset();                                                                                            // DALI is slow, since this possible high amount of queries takes long, reset WDT
    response = Dali.sendCmdWait(i, Dali.CMD_QUERY_GROUPS_0_7);                                              // query groups from slave
    if (response >= 0) {                                                                                    // check if it answered
      slaves[i].present = true;                                                                             // set slave to be present
      slaves[i].groups = response;

      response = Dali.sendCmdWait(i, Dali.CMD_QUERY_GROUPS_8_15);                                           // query groups of slave from 8-15
      if (response >= 0)                                                                                    // if slave answered ...
        slaves[i].groups |= response << 8;                                                                  // ... add result to groups bitmask
    }
    else
      slaves[i].present = false;                                                                            // slave has not answered, set to not present
  }
}


/* ***********************************************************************
   INPUT BUTTON ROUTINE
 * *********************************************************************** */
void read_buttons_timerCallback(const void *context) {
  static byte old_button_status[BUTTON_COUNT];
  static unsigned long button_press_time[BUTTON_COUNT], button_release_time[BUTTON_COUNT];
  static byte button_click_count[BUTTON_COUNT];

  byte button_status[BUTTON_COUNT];

  // read all inputs
  for (byte i = 0; i < BUTTON_COUNT; i++) {
    button_status[i] = digitalRead(INPUT_PINS[i]);
    // check if button state changed
    if (button_status[i] != old_button_status[i]) {
      // button changed
      if (button_status[i] == HIGH) {
        // button changed to pressed
        button_press_time[i] = millis();
        if (button_click_count[i] <= 5)
          button_click_count[i]++;
      }
      else {
        // button changed to released
        button_release_time[i] = millis();
        if (button_release_time[i] - button_press_time[i] > BUTTON_CHANGE_TIME) {
          // button has been held, end action depending on click count
          button_hold_stop(i, button_click_count[i]);
          button_click_count[i] = 0;
        }
      }
    }
    // check if button is being held down
    else if (button_status[i] == HIGH && millis() - button_press_time[i] > BUTTON_CHANGE_TIME && millis() - button_press_time[i] <= BUTTON_CHANGE_TIME + BUTTON_SAMPLING_RATE) {
      // button held down
      button_hold_start(i, button_click_count[i]);
    }
    // check if button has been released without being pressed again for long enough
    else if (button_click_count[i] != 0 && button_status[i] == LOW && millis() - button_release_time[i] > BUTTON_CHANGE_TIME) {
      // button clicked x-times
      button_clicked(i, button_click_count[i]);
      button_click_count[i] = 0;
    }

    old_button_status[i] = button_status[i];
  }

}


/* ***********************************************************************
   INPUT BUTTON ACTIONS
 * *********************************************************************** */

// called when button <buttonno> is pressed <clickcount> times
void button_clicked(byte buttonno, byte clickcount) {
  byte address = buttonMappings[buttonno][clickcount - 1];
  if (address < 80)
    toggleLamp(address);
  else if (address >= 96)
    toggleRelay(address - 96);
  publishButtonClicked(buttonno, clickcount);
}

// called when button <buttonno> is held down with <clickcount>nd click
void button_hold_start(byte buttonno, byte clickcount) {
  byte address = buttonMappings[buttonno][clickcount - 1];
  if (address >= 80) return;

  byte singleAddress = address;
  if (address >= 64) singleAddress = getFirstOfGroup(address - 64);

  int arc = getArc(singleAddress);
  if (arc == 0) {
    Dali.sendCmdWait((address < 64) ? address : address - 64, Dali.CMD_ON_AND_STEP_UP, (address < 64) ? Dali.DALI_SHORT_ADDRESS : Dali.DALI_GROUP_ADDRESS);
    dimDirection[singleAddress] = 0;
  }
  else if (arc == 254)
    dimDirection[singleAddress] = 1;

  if (dimUpdateTimer[singleAddress] >= 0)
    timer.stop(dimUpdateTimer[singleAddress]);

  dimUpdateTimer[singleAddress] = timer.every(DIM_UPDATE_INTERVAL, getAndSendArc_timerCallback, (void*) address);
  buttonTimers[buttonno] = timer.every(DIM_REPEAT_TIME, dim_timerCallback, (void*) address);
}

// called when button <buttonno> is released with <clickcount>nd click
void button_hold_stop(byte buttonno, byte clickcount) {
  byte address = buttonMappings[buttonno][clickcount - 1];
  if (address >= 80) return;

  byte singleAddress = address;
  if (address >= 64) singleAddress = getFirstOfGroup(address - 64);

  timer.stop(buttonTimers[buttonno]);

  if (dimUpdateTimer[singleAddress] >= 0) {
    timer.stop(dimUpdateTimer[singleAddress]);
    dimUpdateTimer[singleAddress] = -1;
  }

  dimDirection[singleAddress] = !dimDirection[singleAddress];

  getAndSendArc_timerCallback((void*) address);
}


/* ***********************************************************************
   ACTION FUNCTIONS
 * *********************************************************************** */
int getArc(byte address) {
  return Dali.sendCmdWait(address, Dali.CMD_QUERY_ACTUAL_LEVEL);
}

void getAndSendArc_timerCallback(const void * context) {
  byte address = (byte) context;
  bool first = true;

  if (address >= 64) {
    for (byte i = 0; i < MAX_SHORT_ADDRESS; i++)
      if (slaves[i].groups & (1 << (address - 64))) {
        int arc = getArc(i);
        if (arc >= 0 && arc <= 254) {
          publishArc(i, arc, Dali.DALI_SHORT_ADDRESS);
          if (first) {
            publishArc(address - 64, arc, Dali.DALI_GROUP_ADDRESS);
            first = false;
          }
        }
      }
  }
  else {
    int arc = getArc(address);
    if (arc >= 0 && arc <= 254)
      publishArc(address, arc, Dali.DALI_SHORT_ADDRESS);
  }
}

void setArc(byte address, byte value, int fadetime = -1) {
  byte singleAddress = address;
  if (address >= 64)
    singleAddress = getFirstOfGroup(address - 64);

  int oldSpeeds = Dali.sendCmdWait(singleAddress, Dali.CMD_QUERY_FADE_SPEEDS);
  if (oldSpeeds >= 0) {
    byte oldFadetime = (byte)oldSpeeds >> 4;
    if (fadetime < 0) fadetime = oldFadetime;
    if (oldFadetime != fadetime) {
      Dali.sendSpecialCmdWait(Dali.CMD_SET_DTR, fadetime);
      sendCmdWait(address, Dali.CMD_DTR_AS_FADE_TIME);
    }

    if (address < 64)
      Dali.sendArcWait(address, value, Dali.DALI_SHORT_ADDRESS);
    else
      Dali.sendArcWait(address - 64, value, Dali.DALI_GROUP_ADDRESS);

    if (oldFadetime != fadetime) {
      Dali.sendSpecialCmdWait(Dali.CMD_SET_DTR, oldFadetime);
      sendCmdWait(address, Dali.CMD_DTR_AS_FADE_TIME);
    }

    if (dimUpdateTimer[singleAddress] >= 0)
      timer.stop(dimUpdateTimer[singleAddress]);
    dimUpdateTimer[singleAddress] = timer.every(DIM_UPDATE_INTERVAL, getAndSendArc_timerCallback, DIM_UPDATE_COUNT[fadetime], (void*) address);
  }
}

int sendCmdWait(byte address, byte command) {
  if (address < 64)
    return Dali.sendCmdWait(address, command, Dali.DALI_SHORT_ADDRESS);
  else
    return Dali.sendCmdWait(address - 64, command, Dali.DALI_GROUP_ADDRESS);
}

byte getFirstOfGroup(byte group) {
  byte result = group;
  for (byte i = 0; i < MAX_SHORT_ADDRESS; i++)
    if (slaves[i].groups & (1 << group)) {
      result = i;
      break;
    }
  return result;
}

void toggleRelay(byte relayNo) {
  switchRelay(relayNo, !digitalRead(RELAY_PINS[relayNo]));
}

void switchRelay(byte relayNo, byte state) {
  digitalWrite(RELAY_PINS[relayNo], state);
  publishRelay(relayNo, state);
}

void toggleLamp(byte address) {
  byte singleAddress = address;
  if (address >= 64)
    singleAddress = getFirstOfGroup(address - 64);

  int arc = getArc(singleAddress);

  if (arc > 0)
    setArc(address, 0);
  else if (arc == 0)
    setArc(address, 254);
}

void dim_timerCallback(const void * context) {
  byte address = (byte) context;

  if (address >= 64) {
    byte singleAddress = getFirstOfGroup(address - 64);
    Dali.sendCmdWait(address - 64, dimDirection[singleAddress] + 1, Dali.DALI_GROUP_ADDRESS);
  }
  else
    Dali.sendCmdWait(address, dimDirection[address] + 1);
}


/* ***********************************************************************
   EEPROM FUNCTIONS
 * *********************************************************************** */

template<typename T>
void eepromPut(int &address, const T &t) {
  EEPROM.put(address, t);
  address += sizeof(t);
}

template<typename T>
void eepromGet(int &address, T &t) {
  EEPROM.get(address, t);
  address += sizeof(t);
}

void eepromWriteString(int &address, char * str, byte maxLength = 25) {
  int strSize = strlen(str);
  for (int i = 0; i < strSize; i++)
    EEPROM.write(address + i, str[i]);
  EEPROM.write(address + strSize, '\0');
  address += maxLength;
}

void eepromReadString(int &address, char * str, byte maxLength = 25) {
  byte pos = 0;
  char c = EEPROM.read(address);
  while (c != '\0' && pos < maxLength) {
    c = EEPROM.read(address + pos);
    str[pos++] = c;
  }
  str[pos] = '\0';
  address += maxLength;
}

void readConfig() {
  int address = EEPROM_START_ADDRESS + 1;
  eepromReadString(address, deviceID);

  eepromGet(address, netIP);
  eepromGet(address, netMask);
  eepromGet(address, netGW);

  eepromGet(address, mqttBrokerIP);
  eepromGet(address, mqttBrokerPort);

  eepromGet(address, buttonMappings);
}

void writeConfig() {
  int address = EEPROM_START_ADDRESS;
  eepromPut(address, (byte) 0xA1);
  eepromWriteString(address, deviceID);

  eepromPut(address, netIP);
  eepromPut(address, netMask);
  eepromPut(address, netGW);

  eepromPut(address, mqttBrokerIP);
  eepromPut(address, mqttBrokerPort);

  eepromPut(address, buttonMappings);
}

bool eepromFresh() {
  return (EEPROM.read(EEPROM_START_ADDRESS) != 0xA1);
}


/* ***********************************************************************
   HELPER FUNCTIONS
 * *********************************************************************** */

void strcpy_P_catByte99(char * dst, char * src_P, byte i) {
  strcpy_P(dst, src_P);
  strcatByte99(dst, i);
}

void strcatByte99(char * dst, byte i) {
  char str[3];
  byte pos = 0;

  if (i >= 10) {
    str[pos++] = (byte)((i / 10) + '0');
    i %= 10;
  }
  str[pos] = i + '0';
  str[pos + 1] = '\0';

  strcat(dst, str);
}

byte parseNumbers(const char* str, word * results, byte size) {
  byte count = 0, digits = 0;
  results[0] = 0;

  for (byte i = 0; (str[i] != '\0'); ++i) {
    if (str[i] == ' ' || str[i] == '.') {
      if (count >= size) break;
      digits = 0;
      continue;
    }
    else if (isdigit(str[i]) && digits < 3) {
      if (digits == 0) {
        count++;
        results[count - 1] = 0;
      }
      results[count - 1] = results[count - 1] * 10 + (str[i] - '0');
      digits++;
    }
    else {
      break;
    }
  }
  return count;
}

/*
  // takes ~600 bytes more space than variant above
  byte parseNumbers(const char * scan, word * values, byte size) {
  char * ptr = NULL;
  char * endptr = scan;
  byte found = 0;

  while (ptr != endptr && found < size) {
    ptr = endptr;
    if (*ptr == '.') ptr++;  // allow for single preceding ".", for parsing IPs
    unsigned long val = strtoul(ptr, &endptr, 10);
    if (ptr != endptr)
      values[found++] = val;
  }
  return found;
  }
*/

void strcpy_BaseTopic(char * dst) {
  strcpy_P(dst, HOMIE_BASE_TOPIC_P); strcat(dst, deviceID);
  strcat(dst, "/");
}

void strcpy_BaseTopic_cat_P(char * dst, const char * src_P) {
  strcpy_BaseTopic(dst);
  strcat_P(dst, src_P);
}

void publishMQTT_RP_P(const char * topicPrefix, const char * topicSuffix_P, const char * payload_P) {
  char payload[MQTT_PAYLOAD_MAXLENGTH];
  strcpy_P(payload, payload_P);

  publishMQTT_RP_R(topicPrefix, topicSuffix_P, payload);
}

void publishMQTT_RP_R(const char * topicPrefix, const char * topicSuffix_P, const char * payload) {
  char topic[MQTT_TOPIC_MAXLENGTH + 1];
  strcpy(topic, topicPrefix);
  strcat_P(topic, topicSuffix_P);

  mqttClient.publish(topic, payload, true);
}


void copyIp(byte * dst, const word * src) {
  for (byte i = 0; i < 4; i++)
    dst[i] = src[i];
}

void strcatIP(char * dst, const byte * ip) {
  char itoabuf[7];
  for (byte i = 0; i < 4; i++) {
    itoa(ip[i], itoabuf, 10);
    strcat(dst, itoabuf);
    if (i < 3) strcat(dst, ".");
  }
}
