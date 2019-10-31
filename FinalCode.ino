//#include <lmic.h>
//#include <hal/hal.h>
//#include <SPI.h>





//HB 100 sensor

const int frequencyOut=4;
const int dopplerDivision=44;
int frequency;


//ultrasonic sensor

//Sensor 1
const int trigPin1=13;
const int echoPin1=12;

// sensor 2
const int trigPin2=8;
const int echoPin2=7;

long duration1=0;
long duration2=0;

int direction1Pedestrians=0;
int direction2Pedestrians=0;

int direction1Bicyclist=0;
int direction2Bicyclist=0;

long timeOfMotion;



//static const PROGMEM u1_t NWKSKEY[16] = { 0x6A, 0x64, 0x7D, 0xB7, 0x06, 0x2B, 0x6B, 0x6D, 0xF8, 0xAA, 0x12, 0xBE, 0xBB, 0xA3, 0xEA, 0x3B };
//
//static const u1_t PROGMEM APPSKEY[16] = { 0x80, 0x27, 0x97, 0x8C, 0xA6, 0x41, 0x9A, 0xF4, 0xF3, 0x4A, 0x0F, 0x9A, 0x2F, 0xB4, 0xAF, 0x25 };
//
//static const u4_t DEVADDR = 0x260410CE;
//void os_getArtEui (u1_t* buf) { }
//void os_getDevEui (u1_t* buf) { }
//void os_getDevKey (u1_t* buf) { }
//
//static osjob_t sendjob;
//
//const unsigned TX_INTERVAL = 5;
//
//
//const lmic_pinmap lmic_pins = {
//    .nss = 10,
//    .rxtx = LMIC_UNUSED_PIN,
//    .rst = 9,
//    .dio = {2, 6, 7},
//};
//
//
//void onEvent (ev_t ev) {
//    Serial.print(os_getTime());
//    Serial.print(": ");
//    switch(ev) {
//        case EV_SCAN_TIMEOUT:
//            Serial.println(F("EV_SCAN_TIMEOUT"));
//            break;
//        case EV_BEACON_FOUND:
//            Serial.println(F("EV_BEACON_FOUND"));
//            break;
//        case EV_BEACON_MISSED:
//            Serial.println(F("EV_BEACON_MISSED"));
//            break;
//        case EV_BEACON_TRACKED:
//            Serial.println(F("EV_BEACON_TRACKED"));
//            break;
//        case EV_JOINING:
//            Serial.println(F("EV_JOINING"));
//            break;
//        case EV_JOINED:
//            Serial.println(F("EV_JOINED"));
//            break;
//        case EV_RFU1:
//            Serial.println(F("EV_RFU1"));
//            break;
//        case EV_JOIN_FAILED:
//            Serial.println(F("EV_JOIN_FAILED"));
//            break;
//        case EV_REJOIN_FAILED:
//            Serial.println(F("EV_REJOIN_FAILED"));
//            break;
//            break;
//        case EV_TXCOMPLETE:
//            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
//            if(LMIC.dataLen)
//            {
//        
//                Serial.print(F("Data Received: "));
//                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
//                Serial.println();
//            }
//            
//            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
//            break;
//        case EV_LOST_TSYNC:
//            Serial.println(F("EV_LOST_TSYNC"));
//            break;
//        case EV_RESET:
//            Serial.println(F("EV_RESET"));
//            break;
//        case EV_RXCOMPLETE:
//            
//            Serial.println(F("EV_RXCOMPLETE"));
//            break;
//        case EV_LINK_DEAD:
//            Serial.println(F("EV_LINK_DEAD"));
//            break;
//        case EV_LINK_ALIVE:
//            Serial.println(F("EV_LINK_ALIVE"));
//            break;
//         default:
//            Serial.println(F("Unknown event"));
//            break;
//    }
//}
//
//void do_send(osjob_t* j){
//
//   
//    
//    if (LMIC.opmode & OP_TXRXPEND) 
//    {
//
//        Serial.print("OP_TXRXPEND, not sending; at freq: ");
//        Serial.println(LMIC.freq);        
//    } 
//    else 
//    {
//
//
//        uint32_t leftPedestrians=direction1Pedestrians;
//        uint32_t  leftBicyclist=direction1Bicyclist;
//        
//        
//        uint32_t RightPedestrians=direction2Pedestrians;
//        uint32_t RightBicyclist=direction2Bicyclist;
//
//        uint32_t timeNow=timeOfMotion;
//
//
//        byte payload[10];
//        
//        payload[0] = highByte(leftPedestrians);
//        payload[1] = lowByte(leftPedestrians);
//        payload[2] = highByte(leftBicyclist);
//        payload[3] = lowByte(leftBicyclist); 
//
//        payload[4] = highByte(RightPedestrians);
//        payload[5] = lowByte(RightPedestrians);
//        payload[6] = highByte(RightBicyclist);
//        payload[7] = lowByte(RightBicyclist); 
//
//         payload[8] = highByte(timeNow);
//        payload[9] = lowByte(timeNow); 
//
//        Serial.print(payload[2]);
//         LMIC_setTxData2(1, payload, sizeof(payload), 0);
//       
//        Serial.print(F("Packet queued for freq: "));
//        Serial.println(LMIC.freq);
//    }
//
//}
//   









void setup()
{
  Serial.begin(115200);

  //HB100 sensor

  pinMode(frequencyOut,INPUT);

  //Ultrasonic Sensor

    pinMode(trigPin1, OUTPUT); 
    pinMode(trigPin2, OUTPUT);
    pinMode(echoPin1, INPUT); 
    pinMode(echoPin2, INPUT);

     


  //TTN Code


//   Serial.println(F("Starting"));
//    #ifdef VCC_ENABLE
//   
//    pinMode(VCC_ENABLE, OUTPUT);
//    digitalWrite(VCC_ENABLE, HIGH);
//    delay(1000);
//    #endif
//
//    
//    os_init();
//    LMIC_reset();
//
//   
//    #ifdef PROGMEM
//    
//    uint8_t appskey[sizeof(APPSKEY)];
//    uint8_t nwkskey[sizeof(NWKSKEY)];
//    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
//    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
//    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
//    #else
//    
//    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
//    #endif
//
//    
//    
//   
//    for (int channel=0; channel<8; ++channel) {
//      LMIC_disableChannel(channel);
//    }
//    
//    for (int channel=16; channel<72; ++channel) {
//       LMIC_disableChannel(channel);
//    }
//   
//    LMIC_setLinkCheckMode(0);
//
//   
//    LMIC_setDrTxpow(DR_SF7,14);
//     
//    do_send(&sendjob);

  
    


 
}

void loop()
{
   timeOfMotion=0;

    

    int distance1=0;
    int distance2=0;
  
    digitalWrite(trigPin1, HIGH);
    delayMicroseconds(3000);
    digitalWrite(trigPin1, LOW);
  
    duration1=pulseIn(echoPin1,HIGH);
    distance1=duration1*0.34/2;
  
    digitalWrite(trigPin2, HIGH);
    delayMicroseconds(3000);
    digitalWrite(trigPin2, LOW);
  
    duration2=pulseIn(echoPin2,HIGH);
    distance2=duration2*0.34/2;

   // Serial.println(distance1);



  if(distance1<500)
  {
    int pulseLength=0;
    pulseLength=pulseIn(frequencyOut,HIGH);
    pulseLength+=pulseIn(frequencyOut,LOW);

    int Time=pulseLength;

    int Frequency=1000000/Time;


    int Velocity=Frequency/dopplerDivision;


     if(Velocity>0 && Velocity<5)
     {

      
      digitalWrite(trigPin2,LOW);
      digitalWrite(trigPin1,LOW);
      direction1Pedestrians=direction1Pedestrians+1;
      direction2Pedestrians=direction2Pedestrians;
      direction1Bicyclist=direction1Bicyclist;
      direction2Bicyclist=direction2Bicyclist;

      Serial.print("Direction1Pedestrains ");
      Serial.println(direction1Pedestrians);

      //Serial.println(distance1);


      timeOfMotion=millis();
     

      delay(3000);

      
      }
  


      else if(Velocity>5)
      {
     
      digitalWrite(trigPin2,LOW);
      digitalWrite(trigPin1,LOW);

      direction1Pedestrians=direction1Pedestrians;
      direction2Pedestrians=direction2Pedestrians;
      direction1Bicyclist=direction1Bicyclist+1;
      direction2Bicyclist=direction2Bicyclist;

      Serial.print("direction1Bicyclist ");
      Serial.println(direction1Bicyclist);

     // Serial.println(Velocity);

      timeOfMotion=millis();
     

      delay(3000);
      }
       
 //   do_send(&sendjob);
       //os_runloop_once(); 
  }
        
     
  
      else if(distance2<500)
      {
        long t=millis();
        int pulseLength=0;
         pulseLength=pulseIn(frequencyOut,HIGH);
        pulseLength +=pulseIn(frequencyOut,LOW);
    
        int Time=pulseLength;
    
        int Frequency=1000000/Time;
  
        int Velocity=Frequency/dopplerDivision;
    
        if(Velocity>0 && Velocity<5 ){
          
        
        digitalWrite(trigPin2,LOW);
        digitalWrite(trigPin1,LOW);
        
      direction1Pedestrians=direction1Pedestrians;
      direction2Pedestrians=direction2Pedestrians+1;
      direction1Bicyclist=direction1Bicyclist;
      direction2Bicyclist=direction2Bicyclist;

    
        Serial.print("direction2Pedestrians ");
        Serial.println(direction2Pedestrians);

       // Serial.print(Velocity);

        timeOfMotion=millis();
    
        
    
       
    
        delay(3000);
      }
      
  
  
      else if(Velocity>5){
        
       digitalWrite(trigPin2,LOW);
       digitalWrite(trigPin1,LOW);
    
       direction1Pedestrians=direction1Pedestrians;
      direction2Pedestrians=direction2Pedestrians;
      direction1Bicyclist=direction1Bicyclist;
      direction2Bicyclist=direction2Bicyclist+1;

       Serial.print("direction2Bicyclist ");
       Serial.println(direction2Bicyclist);

     //  Serial.println(Velocity);

       timeOfMotion=millis();

      // timeNow=timeOfMotion;

       
    

  
    
     
    
       delay(3000);
    
  }
   
  
    
    //do_send(&sendjob);
   
  }


//
// os_runloop_once();

  
}
