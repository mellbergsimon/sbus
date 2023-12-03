 
byte outArray[7];        // read data Pelco Command
 unsigned stopcheck;      // For checking when a STOP command is received (257 Decimal)
 int checksum;            // For Calculating Checksum. Sum of the payload bytes (bytes 2 through 6) in the message
 int ByteNumber;
 int PanSpeed;
 int TiltSpeed;
 int panPin = 0;
 int tiltPin = 1;
 int panpos = 0;
 int tiltpos = 0;
 int retning = 0;     //mounting direction, reverses pan and tilt if high
 int focusvalue = 1024;
 int focusStep = 25;
 //sbus stuff
unsigned int SBUS_Channel_Data[18];
byte SBUS_Current_Channel = 0;
byte SBUS_Current_Channel_Bit = 0;
byte SBUS_Current_Packet_Bit = 0;
byte SBUS_Packet_Data[25];
byte SBUS_Packet_Position = 0;

byte SBUS_Failsafe_Active = 0;
byte SBUS_Lost_Frame = 0;
 
 #include <Arduino.h>
 HardwareSerial mySerial(2,4);

void setup(){
 
    mySerial.begin(9600);    // 485-port
    pinMode(3,  OUTPUT); //tiltPWM
    pinMode(11, OUTPUT); //panPWM
    pinMode(12, OUTPUT); //tiltDIR
    pinMode(13, OUTPUT); //panDIR
    pinMode(10, INPUT_PULLUP); //PTZ head mounted up or down

    //sbus stuff
    Serial.begin(100000,SERIAL_8E2);
    SBUS_Channel_Data[0] = 1024;
    SBUS_Channel_Data[1] = 1024;
    SBUS_Channel_Data[2] = 1024;
    SBUS_Channel_Data[3] = 1024;
    SBUS_Channel_Data[4] = 1024;
    SBUS_Channel_Data[5] = 1024;
    SBUS_Channel_Data[6] = 1024;
    SBUS_Channel_Data[7] = 1024;
    SBUS_Channel_Data[8] = 1024;
    SBUS_Channel_Data[9] = 1024;
    SBUS_Channel_Data[10] = 1024;
    SBUS_Channel_Data[11] = 1024;
    SBUS_Channel_Data[12] = 1024;
    SBUS_Channel_Data[13] = 1024;
    SBUS_Channel_Data[14] = 1024;
    SBUS_Channel_Data[15] = 1024;
    SBUS_Channel_Data[16] = 0;
    SBUS_Channel_Data[17] = 0;
    SBUS_Failsafe_Active = 1;
    SBUS_Lost_Frame = 1;
}
void loop(){
//tiltpos = analogRead(tiltPin);
//panpos = analogRead(panPin);
//Serial.println(tiltpos);
//Serial.println(panpos);
retning = digitalRead(10);  //mounting direction selectionswitch

 if ( mySerial.available () > 0) {
      outArray[ByteNumber ++] = mySerial.read();}
 
 if ( ByteNumber > 6){         // process it
      ByteNumber = 0;          // ready for next time

      stopcheck = outArray[0] + outArray[1] + outArray[2] + outArray[3] + outArray[4] + outArray[5] + outArray[6] ;   // Calculate if STOP Command is received
 if ( stopcheck == 257){       // When stopcheck is 257 decimal a STOP command is received
      analogWrite(3, 0);
      analogWrite(11, 0);
      SBUS_Channel_Data[1] = 1024;
      SBUS_Channel_Data[3] = 1024;
     }          // Stop all PTZ Actions
   
 if ( bitRead(outArray[3],0) == 0 ){   // When BIT 0 = 0 command 2 than data is Normal command (PTZ)
     PanSpeed = map (outArray[4],  0, 0x3F, 0, 255);
 }
 if ( bitRead(outArray[3],0) == 0 ){   // When BIT 0 = 0 command 2 than data is Normal command (PTZ)
     TiltSpeed = map (outArray[5],  0, 0x3F, 0, 255);     
 }

// PAN TILT:
 if ( bitRead(outArray[3],1) == 1 ){  //pan right
     analogWrite(11 , PanSpeed);
     if (retning == 1){digitalWrite(13, HIGH); }
     else {digitalWrite(13, LOW);}
     }
     
 if ( bitRead(outArray[3],2) == 1 ){  //pan left
     analogWrite(11, PanSpeed); 
     if (retning == 1){digitalWrite(13, LOW); }
     else {digitalWrite(13, HIGH);}
     }
 
 if ( bitRead(outArray[3],3) == 1 ){  //tilt up
     analogWrite(3, TiltSpeed);
     if (retning == 1){digitalWrite(12, HIGH); }
     else {digitalWrite(12, LOW);}
     }
 
 if ( bitRead(outArray[3],4) == 1 ){  //tilt down
     analogWrite(3, TiltSpeed);
     if (retning == 1){digitalWrite(12, LOW); }
     else {digitalWrite(12, HIGH);}
     }
 

// ZOOM IRIS FOCUS:
 if ( bitRead(outArray[2],2) == 1 ){ //iris close
     SBUS_Channel_Data[1] = 352;
     }
 if ( bitRead(outArray[2],1) == 1 ){ //iris open
     SBUS_Channel_Data[1] = 1696;
     }
 if ( bitRead(outArray[2],0) == 1 ){ //focus near
     if (focusvalue > 352){focusvalue = focusvalue - focusStep;}
     SBUS_Channel_Data[2] = focusvalue;
     }
 if ( bitRead(outArray[3],7) == 1 ){ //focus far
     if (focusvalue < 1696){focusvalue = focusvalue + focusStep;}
     SBUS_Channel_Data[2] = focusvalue;
     }
 if ( bitRead(outArray[3],6) == 1 ){ //zoom wide
     SBUS_Channel_Data[3] = 352;
     }
 if ( bitRead(outArray[3],5) == 1 ){ //zoom near
     SBUS_Channel_Data[3] = 1696;
     } 
 
}                    // Try to decode the Pelco Command

 if ( bitRead(outArray[3],0) == 1 ){   // When BIT 0 = 1 command 2 than data is an Extended command
     
 if ( outArray[2] == 0 ){         // Only continu when Word 3 is 0

 if ( outArray[3] == 0x03 ){      // SET PRESET       
    }    // PRINT Preset. -1 to calculate right preset
 
 if ( outArray[3] == 0x05 ){      // Clear Preset     
    }    // PRINT Preset. -1 to calculate right preset
         
}}             // Try to decode the Extended Pelco Command

SBUS_Build_Packet();
Serial.write(SBUS_Packet_Data, 25);
   
}  // end if full

void SBUS_Build_Packet(void){
  for(SBUS_Packet_Position = 0; SBUS_Packet_Position < 25; SBUS_Packet_Position++) SBUS_Packet_Data[SBUS_Packet_Position] = 0x00;  //Zero out packet data
 
  SBUS_Current_Packet_Bit = 0;
  SBUS_Packet_Position = 0;
  SBUS_Packet_Data[SBUS_Packet_Position] = 0x0F;  //Start Byte
  SBUS_Packet_Position++;
 
  for(SBUS_Current_Channel = 0; SBUS_Current_Channel < 16; SBUS_Current_Channel++)
  {
    for(SBUS_Current_Channel_Bit = 0; SBUS_Current_Channel_Bit < 11; SBUS_Current_Channel_Bit++)
    {
      if(SBUS_Current_Packet_Bit > 7)
      {
        SBUS_Current_Packet_Bit = 0;  //If we just set bit 7 in a previous step, reset the packet bit to 0 and
        SBUS_Packet_Position++;       //Move to the next packet byte
      }
      SBUS_Packet_Data[SBUS_Packet_Position] |= (((SBUS_Channel_Data[SBUS_Current_Channel]>>SBUS_Current_Channel_Bit) & 0x01)<<SBUS_Current_Packet_Bit);  //Downshift the channel data bit, then upshift it to set the packet data byte
      SBUS_Current_Packet_Bit++;
    }
  }
  if(SBUS_Channel_Data[16] > 1023) SBUS_Packet_Data[23] |= (1<<0);  //Any number above 1023 will set the digital servo bit
  if(SBUS_Channel_Data[17] > 1023) SBUS_Packet_Data[23] |= (1<<1);
  if(SBUS_Lost_Frame != 0) SBUS_Packet_Data[23] |= (1<<2);          //Any number above 0 will set the lost frame and failsafe bits
  if(SBUS_Failsafe_Active != 0) SBUS_Packet_Data[23] |= (1<<3);
  SBUS_Packet_Data[24] = 0x00;  //End byte
  delay(3);
}