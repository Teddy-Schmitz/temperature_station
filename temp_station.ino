#include <IRremote.h>
#include <SPI.h>
#include <DHT.h>
#include <Ethernet.h>
#include <aREST.h>
#include <avr/wdt.h>

unsigned long
currentTime = 0L;

// Enter a MAC address for your controller below.
byte mac[] = { 0x08, 0x00, 0x28, 0x57, 0x5E, 0xD5 };

// IP address in case DHCP fails
IPAddress ip(192, 168, 3, 246);

// Ethernet server
EthernetServer server(80);


//Arest
#define LISTEN_PORT 80
aREST rest = aREST();

//IR
IRsend irsend;
int khz = 38;
// Cool Low 25
unsigned int Signal_On[] = {8956, 4452, 620, 528, 644, 532, 620, 1680, 620, 560, 616, 1680, 620, 560, 620, 528, 620, 556, 620, 560, 616, 1680, 620, 1680, 620, 1652, 652, 1652, 620, 552, 620, 1684, 620, 556, 620, 556, 620, 532, 620, 552, 620, 560, 620, 552, 624, 556, 620, 528, 620, 556, 624, 552, 620, 560, 620, 552, 620, 532, 620, 556, 620, 556, 620, 556, 620, 560, 620, 528, 620, 556, 620, 552, 620, 560, 620, 556, 620, 528, 620, 560, 620, 556, 620, 552, 620, 1680, 620, 532, 648, 528, 620}; //AnalysIR Batch Export (IRremote) - RA
unsigned int Signal_Off[] = {8960, 4448, 620, 532, 644, 532, 620, 1680, 616, 560, 620, 556, 620, 532, 644, 532, 616, 556, 620, 556, 620, 1684, 620, 1676, 620, 1660, 644, 1656, 592, 1704, 620, 556, 620, 560, 620, 556, 620, 532, 592, 580, 620, 556, 620, 556, 620, 560, 620, 524, 600, 580, 620, 556, 620, 556, 620, 556, 620, 532, 620, 556, 616, 560, 620, 556, 620, 556, 620, 528, 620, 556, 620, 560, 616, 560, 620, 556, 620, 528, 620, 556, 620, 556, 620, 1680, 620, 556, 620, 1652, 624, 556, 616}; //AnalysIR Batch Export (IRremote) - RAW

//Cool High Fan 25
unsigned int Signal_High[] = {8964, 4448, 620, 560, 612, 560, 620, 1652, 648, 532, 620, 1676, 620, 560, 620, 556, 616, 560, 620, 532, 620, 1676, 620, 560, 620, 1676, 620, 1680, 620, 556, 620, 1656, 620, 556, 620, 560, 620, 552, 620, 556, 620, 532, 620, 556, 620, 556, 620, 556, 620, 556, 620, 532, 620, 556, 616, 556, 620, 560, 620, 556, 616, 532, 620, 560, 620, 552, 620, 556, 620, 560, 620, 528, 620, 556, 620, 560, 612, 560, 620, 556, 620, 532, 616, 564, 616, 1680, 616, 1684, 616, 560, 620}; //AnalysIR Batch Export (IRremote) - RAW

//Cool Low Fan 25
unsigned int Signal_Low[] = {8960, 4448, 624, 552, 620, 532, 644, 1656, 620, 556, 620, 1680, 620, 560, 616, 556, 620, 532, 644, 528, 624, 1680, 620, 1680, 620, 1676, 620, 1680, 620, 532, 644, 1656, 620, 556, 620, 560, 620, 552, 620, 532, 644, 528, 624, 556, 620, 556, 620, 556, 620, 532, 644, 532, 620, 552, 620, 560, 620, 552, 624, 528, 648, 532, 620, 552, 620, 556, 620, 560, 620, 528, 644, 532, 620, 560, 620, 552, 620, 556, 620, 532, 648, 524, 620, 560, 620, 1680, 620, 556, 620, 556, 616}; //AnalysIR Batch Export (IRremote) - RAW

//Dry 25
//unsigned int Signal_Dry[] = {8952, 4484, 612, 536, 616, 564, 612, 1684, 616, 564, 612, 1688, 616, 532, 616, 560, 616, 564, 608, 1688, 616, 560, 616, 1656, 644, 1660, 616, 1684, 616, 556, 616, 1688, 616, 560, 616, 536, 640, 532, 616, 560, 616, 564, 616, 556, 616, 536, 640, 540, 612, 560, 616, 560, 616, 564, 616, 532, 640, 536, 616, 556, 616, 564, 616, 560, 616, 532, 644, 536, 616, 560, 616, 560, 612, 564, 616, 532, 640, 536, 616, 564, 616, 556, 616, 560, 616, 536, 644, 532, 616, 560, 616}; //AnalysIR Batch Export (IRremote) - RAW

char sensorPrintout[9];
char tempPrintout[9];

//Sensor
#define DHTTYPE DHT22
#define DHTPIN A0

DHT dht(DHTPIN, DHTTYPE);

bool error = false;
float h;
float t;

#define INTERVAL 300000

unsigned long lastUpdateTime = 0;
int buffer_size = 20;

bool connected = true;

char lf = 10;
int x = 0;
String readString;

void setup() {

  Serial.begin(115200);

  Serial.println(F("Starting up"));

  //Sensor Init
  dht.begin();

  //aREST init
  t = 27;
  h = 40;
  rest.variable("temperature", &t);
  rest.variable("humidity", &h);

  rest.function("turnOn", turnOnAC);
  rest.function("turnOff", turnOffAC);
  rest.function("high", highAC);
  rest.function("low", lowAC);

  rest.set_id("009");
  rest.set_name("autotemp");


  //Wifi Init
  uint32_t ip = 0L, t;

  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    // try to congifure using IP address instead of DHCP:
    Ethernet.begin(mac, ip);
  }
  connected = true;
  server.begin();
  wdt_enable(WDTO_8S);
  Serial.println(F("Setup complete, starting loop"));


}

void loop() {
  h = dht.readHumidity();
  // Read temperature as Celsius
  t = dht.readTemperature();
  String humidity = String(h);
  humidity += " %";
  String temp = String(t);
  temp += " C";
  temp.toCharArray(tempPrintout, 9);
  humidity.toCharArray(sensorPrintout, 9);
  EthernetClient client = server.available();
  rest.handle(client);
  //   Check connection, reset if connection is lost
  wdt_reset();

}

int turnOnAC(String command) {
  irsend.sendRaw(Signal_On, sizeof(Signal_On) / sizeof(int), khz); //AnalysIR Batch Export (IRremote) - RAW
  return 1;
}

int turnOffAC(String command) {
  irsend.sendRaw(Signal_Off, sizeof(Signal_Off) / sizeof(int), khz); //AnalysIR Batch Export (IRremote) - RAW
  return 1;
}

int lowAC(String command) {
  irsend.sendRaw(Signal_Low, sizeof(Signal_Low) / sizeof(int), khz);
  return 1;
}
int highAC(String command) {
  irsend.sendRaw(Signal_High, sizeof(Signal_High) / sizeof(int), khz);
  return 1;
}


