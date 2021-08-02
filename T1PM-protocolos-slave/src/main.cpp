#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#include <SPI.h>
#include <SPI_Send_Receive.h>

volatile boolean received;
volatile byte slaveReceived, slaveSend;

LiquidCrystal_I2C lcd(0x20, 16, 2);

// criar estrutura de transferência de dados por SPI
struct GyroAccelData
{
  char tipo = 'T';
  float acelerometroX, acelerometroY, acelerometroZ;
  float temperatura;
  float giroscopioX, giroscopioY, giroscopioZ;
  bool isFilled;
};

GyroAccelData gyroAccelData;

unsigned int offset = 0;
float inteiro;

unsigned int interruptions = 0;

void slaveSPI()
{
  if (received)
  {
    Serial.println("recebido:");
    Serial.println(gyroAccelData.acelerometroX);
    Serial.println(gyroAccelData.acelerometroY);
    Serial.println(gyroAccelData.acelerometroZ);
    Serial.println(gyroAccelData.temperatura);
    received = false;
    Serial.println("interruptions:");
    Serial.println(interruptions);
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Setting up slave");

  pinMode(MISO, OUTPUT); // have to send on master in, *slave out*
  SPCR |= _BV(SPE);      //Turn on SPI in Slave Mode
  received = false;

  SPI.attachInterrupt(); //Interuupt ON is set for SPI commnucation

  lcd.init();      // Inicializando o LCD
  lcd.backlight(); // Ligando o BackLight do LCD
}

ISR(SPI_STC_vect) //Inerrrput routine function
{
  interruptions++;
  SPI_read(gyroAccelData);
  received = true;
}

void loop()
{
  lcd.print("teste");
  slaveSPI();
}