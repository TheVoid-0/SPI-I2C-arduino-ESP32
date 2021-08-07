
#include <Arduino.h>

#include <LiquidCrystal_I2C.h>
#include <SPI.h>

// TODO: adicionar codigo do giroscópio e completar lógica da aplicação

// inclui biblioteca personalizada
#include <SPI_Send_Receive.h>

// criar estrutura de transferência de dados por SPI
struct GyroAccelData
{
  char tipo = 'T';
  float acelerometroX, acelerometroY, acelerometroZ;
  float temperatura;
  float giroscopioX, giroscopioY, giroscopioZ;
};

GyroAccelData gyroAccelData;

GyroAccelData bufferGyroAccel;
byte buffer[sizeof(bufferGyroAccel)];

//flags de controle
bool SCAN_LCD = false;
bool IS_SLAVE = true;

// functions
void findLCDAddress(); // scan do endereço do LCD
void masterSPI();
void slaveSPI();
void slaveSPI(float data);
void getMockData();

// instanciar LCD
LiquidCrystal_I2C lcd(0x3F, 16, 2);

volatile bool locked = false;
volatile int bytes = 0;
volatile int interruptions = 0;

void setup()
{

  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  Serial.begin(9600);
  while (!Serial)
    ; // Leonardo: wait for serial monitor

  Serial.print("Setting up ESP-CODE  - acting as: ");

  lcd.init();      // Inicializando o LCD
  lcd.backlight(); // Ligando o BackLight do LCD
  lcd.print("TESTE");
  if (!IS_SLAVE)
  {
    Serial.println("MASTER");
    // inicializa o SPI para comunicação mestre-escravo -mestre
    SPI.begin();                         //Begins the SPI commnuication
    SPI.setClockDivider(SPI_CLOCK_DIV8); //Sets clock for SPI communication at 8 (16/8=2Mhz)
    digitalWrite(SS, HIGH);              // Setting SlaveSelect as HIGH (So master doesnt connnect with slave)
  }
  else
  {
    Serial.println("SLAVE");
    pinMode(MOSI, OUTPUT);
    SPCR |= _BV(SPE);
    SPCR |= _BV(SPIE);
    SPI.attachInterrupt(); //Interuupt ON is set for SPI commnucation
  }
}

ISR(SPI_STC_vect) //Inerrrput routine function
{
  //buffer = SPDR;
  interruptions++;
  if (!locked)
  {
    locked = true;
    bytes = SPI_read(gyroAccelData, SPDR);
  }
}

void loop()
{
  if (SCAN_LCD)
  {
    findLCDAddress();
  }

  if (!IS_SLAVE)
  {
    getMockData();
    masterSPI();
  }

  if (locked)
  {
    slaveSPI();
    locked = false;
  }
}

void slaveSPI()
{
  Serial.print("recebido:");
  Serial.print(bytes);
  Serial.println(" bytes");
  Serial.println(gyroAccelData.temperatura);
  Serial.println(gyroAccelData.acelerometroX);
  Serial.println(gyroAccelData.acelerometroY);
  Serial.println(gyroAccelData.acelerometroZ);
  Serial.println("interruptions:");
  Serial.println(interruptions);
  if (gyroAccelData.giroscopioX != 0.00)
  {
    digitalWrite(4, HIGH);
    digitalWrite(3, LOW);
    lcd.clear();
    lcd.print(gyroAccelData.temperatura);
    lcd.print("Gx:");
    lcd.print(gyroAccelData.giroscopioX);

    lcd.setCursor(0, 1);

    lcd.print("Gy: ");
    lcd.print(gyroAccelData.giroscopioY);
    lcd.print("Gz: ");
    lcd.print(gyroAccelData.giroscopioZ);
  }
  else
  {

    digitalWrite(4, LOW);
    digitalWrite(3, HIGH);
  }
}

void slaveSPI(float data)
{
  Serial.print("recebido:");
  Serial.print(bytes);
  Serial.println(" bytes");
  Serial.println(data);
  Serial.println("interruptions:");
  Serial.println(interruptions);
}

void masterSPI()
{
  lcd.clear();
  lcd.print("MASTER");
  digitalWrite(SS, LOW); //Starts communication with Slave connected to master
  Serial.println("Master sending");
  Serial.println("struct");
  int bytes = SPI_write(gyroAccelData);
  Serial.println("nro de bytes");
  Serial.println(bytes);
  delay(1000);
}

void getMockData()
{
  gyroAccelData.acelerometroX = 1;
  gyroAccelData.acelerometroY = 2;
  gyroAccelData.acelerometroZ = 3;
  gyroAccelData.temperatura = 4;
  gyroAccelData.giroscopioX = 5;
  gyroAccelData.giroscopioY = 6;
  gyroAccelData.giroscopioZ = 7;
  gyroAccelData.temperatura = 30.85;
}

void findLCDAddress()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++)
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
  {
    SCAN_LCD = false;
    Serial.println("done\n");
  }
  if (nDevices == 2)
  {
    SCAN_LCD = false;
  }
  if (SCAN_LCD)
    delay(5000); // wait 5 seconds for next scan
}