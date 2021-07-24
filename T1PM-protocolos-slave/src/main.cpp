#include <Arduino.h>

#include <SPI.h>

volatile boolean received;
volatile byte slaveReceived, slaveSend;

void slaveSPI()
{
  if (received) //Logic to SET LED ON OR OFF depending upon the value recerived from master
  {
   Serial.println("Slave received" + slaveReceived);

    slaveSend = 1;
    SPDR = slaveSend; //Sends the x value to master via SPDR
    delay(1000);
  }
}

void setup()
{
  SPCR |= _BV(SPE); //Turn on SPI in Slave Mode
  received = false;

  SPI.attachInterrupt(); //Interuupt ON is set for SPI commnucation
}

ISR(SPI_STC_vect) //Inerrrput routine function
{
  slaveReceived = SPDR; // Value received from master if store in variable slavereceived
  Serial.println("slave received");
  received = true;      //Sets received as True
}

void loop()
{
  slaveSPI();
}