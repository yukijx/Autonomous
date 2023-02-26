/*
     Pin wiring diagram

     [],pin  front   [],pin
     0,2     0-|-0    3,11
               |
     1,9     0-|-0   -4,12
               |
    -2,20    0-|-0   -5,13
     "-" indicaes wheel's polarity needs to be reversed

     gimbal pan: D6
     gimbal tilt: D7

     LED RGB: 19,20,21

     Pins used by Ethernet:

*/

/*
 * STILL NEED TO ADD GPS
 */
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Servo.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(10, 0, 0, 101);

unsigned int localPort = 1001;      // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";        // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;


#define DEBUG 0 // set to 0 to avoid compiling print statements (will save space, don't need to print if running on rover)
int checkSum = 0; // used to check for errors in recieved message.
#define greenPin 29 // LED pins
#define redPin 27
#define bluePin 25
#define LED 0x02
#define WHEEL 0x00
int vertPos = 90; // position of gimbal
int horizPos = 90; 
Servo wheel0, wheel1, wheel2, wheel3, wheel4, wheel5, gimbalVert, gimbalHoriz;
Servo wheels[6] = {wheel0, wheel1, wheel2, wheel3, wheel4, wheel5};
bool wheelReverse[6] = {true, false, false, true, false, true};
// PWM on mega is D2-D13. Using D8-13 for wheels, D7/D6 for gimbal, and D19/D20/D21 for LEDs

unsigned long timeOut = 0; // used to measure time between msgs. If we go a full second without new msgs, stop wheels so rover doesn't run away from us

void setup() {
  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields <---
  //Ethernet.init(5);   // MKR ETH Shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 with Adafruit FeatherWing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit FeatherWing Ethernet
  wheel0.attach(2, 1000, 2000);
  wheel1.attach(3, 1000, 2000);
  wheel2.attach(4, 1000, 2000);
  wheel3.attach(5, 1000, 2000);
  wheel4.attach(6, 1000, 2000);
  wheel5.attach(7, 1000, 2000);

  gimbalVert.attach(8);
  gimbalHoriz.attach(9);

  pinMode(redPin, OUTPUT); // red
  pinMode(greenPin, OUTPUT); // green
  pinMode(bluePin, OUTPUT); // blue

  // start the Ethernet
  Ethernet.begin(mac, ip);

  // Open serial communications and wait for port to open:
  #if DEBUG
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  #endif

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    #if DEBUG
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    #endif
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    #if DEBUG
    Serial.println("Ethernet cable is not connected.");
    #endif
  }

  // start UDP
  Udp.begin(localPort);
}

void updateGimbal(int vertical, int horizontal) {
  // inputs are 0 for counter-clockwise, 1 for nothing, 2 for clockwise
   switch(vertical){
      case 2: 
        if(vertPos >= 180){
          vertPos=179;
        }
        vertPos += 1; 
        break;
      case 0:
        if(vertPos <= 0){
          vertPos=1;
        }
        vertPos -= 1; 
        break;
   }
    switch(horizontal) {
      case 2: 
        if(horizPos >= 180){
          horizPos=179;
        }
        horizPos += 1; 
        break;
      case 0:
        if(horizPos <= 0){
          horizPos=1;
        }
        horizPos -= 1; 
        break;
      }
  gimbalVert.write(vertPos);
  gimbalHoriz.write(horizPos);
}

//LED msg: [0x23, 0x02, red, green, blue]
//WHEEL msg: [0x23, 0x00, w1, w2, w3, w4, w5, w6, checkSum]
void update(char msg[], int msgSize) {
  checkSum = 0;
  #if DEBUG
  //Serial.println("msg recieved:");
  for(int i=0; i<msgSize; ++i) {
    Serial.print(uint8_t(msg[i]));
    Serial.print(", ");
  }
  Serial.println();
  #endif
  
  if(msg[0] == 0x23) {
    /**************LED MSG *****************/
    if (msg[1] == LED) {
      if(msgSize == 5) {
        if(uint8_t(msg[2]) > 0) {
          digitalWrite(redPin, HIGH);
        }
        else {
          digitalWrite(redPin, LOW);
        }
        if(uint8_t(msg[3]) > 0) {
          digitalWrite(greenPin, HIGH);
        }
        else {
          digitalWrite(greenPin, LOW);
        }
        if(uint8_t(msg[4]) > 0) {
          digitalWrite(bluePin, HIGH);
        }
        else {
          digitalWrite(bluePin, LOW);
        }
      } 
      else {
        #if DEBUG
        Serial.println("msg for LEDS was wrong length... ignoring this message.");
        #endif
      }
    }
    /**************WHEEL MSG *****************/
    else if (msg[1] == WHEEL) {
      if(msgSize == 9) {
        for(int i=2; i<8; i++) { 
          checkSum += msg[i];
        }
        checkSum = checkSum & 0xff;
        #if DEBUG
          // Serial.print("Calcuated checksum: ");
          // Serial.println(checkSum);
          // Serial.print("Received checksum: ");
          // Serial.println((int)msg[8]);
        #endif
        if(checkSum == uint8_t(msg[8])) {
          timeOut = millis(); // save time so we know how long it's been between this and next msg
          long int speed;
          for (int i = 0; i < 6; i++) {
            if (wheelReverse[i]) {
              // speed = map(uint8_t(msg[i+2]), 252, 0, 0, 180);
              // wheels[i].write(speed);
              wheels[i].write(map(uint8_t(msg[i+2]), 252, 0, 0, 180));
            } else {
              // speed = map(uint8_t(msg[i+2]), 0, 252, 0, 180);
              // wheels[i].write(speed);
              wheels[i].write(map(uint8_t(msg[i+2]), 0, 252, 0, 180));

            }
            #if DEBUG
              Serial.print(i);
              Serial.print(": ");
              Serial.println(speed);
            #endif
          }          
          //updateGimbal(uint8_t(msg[8]), uint8_t(msg[9]));
        }
        else {
          #if DEBUG
          Serial.println("checkSum for WHEELS was incorrect... ignoring this message.");
          #endif
        }
      }
      else {
        #if DEBUG
        Serial.println("msg for WHEELS was wrong length... ignoring this message.");
        #endif
      }
    }
    else {
      #if DEBUG
      Serial.println("msg was for the wrong device... ignoring this message.");
      #endif
    }
  }
  else {
    #if DEBUG
    Serial.println("startByte was incorrect... ignoring this message.");
    #endif
  }
}


void loop() {
  // if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    #if DEBUG
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remote = Udp.remoteIP();
    for (int i=0; i < 4; i++) {
      Serial.print(remote[i], DEC);
      if (i < 3) {
        Serial.print(".");
      }
    }
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
    #endif
    // read the packet into packetBuffer
    Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
    update(packetBuffer, packetSize);

    // send a reply to the IP address and port that sent us the packet we received
    //Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    //Udp.write(ReplyBuffer);
    //Udp.endPacket();
  }
  if ( millis() - timeOut >= 1000) // if the last good msg recieved was longer than 1 sec ago, stop wheels
  {
    timeOut = millis();
    wheel0.write(90);
    wheel1.write(90);
    wheel2.write(90);
    wheel3.write(90);
    wheel4.write(90);
    wheel5.write(90);
    gimbalVert.write(90);
    gimbalHoriz.write(90);

    #if DEBUG
    Serial.println("Stopped motors");
    #endif
  }
  delay(10);
}
