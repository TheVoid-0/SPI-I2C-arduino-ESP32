#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

#include <SPI.h>
#include <SPI_Send_Receive.h>

volatile boolean received;
volatile byte slaveReceived, slaveSend;

LiquidCrystal_I2C lcd(0x20, 16, 2);

// criar estrutura de transferência de dados por SPI
struct DataWrapper
{
  char *mensagem;
  float numberData;
  bool isFilled;
};

DataWrapper structTest;
unsigned int offset = 0;
float inteiro;

unsigned int interruptions = 0;

void slaveSPI()
{
  if (received)
  {
    Serial.println("recebido:");
    Serial.println(structTest.mensagem);
    Serial.println(structTest.numberData);
    Serial.println(structTest.isFilled);
    received = false;
    Serial.println("interruptions:");
    Serial.println(interruptions);
    delay(1000);
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Setting up slave");

  pinMode(MISO, OUTPUT); // have to send on master in, *slave out*
  SPCR |= _BV(SPE); //Turn on SPI in Slave Mode
  received = false;

  SPI.attachInterrupt(); //Interuupt ON is set for SPI commnucation

  lcd.init();      // Inicializando o LCD
  lcd.backlight(); // Ligando o BackLight do LCD
}

ISR(SPI_STC_vect) //Inerrrput routine function
{
  interruptions++;
  SPI_read(structTest);
  received = true;
}

void loop()
{
  lcd.print("teste");
  slaveSPI();
}