#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <Wire.h>

// biblioteca personalizada
#include <SPI_Send_Receive.h>

//Endereco I2C do MPU6050 (giroscopio e acelerometro)
const int MPU = 0x68;
// criar estrutura de transferência de dados por SPI
struct GyroAccelData
{
  char tipo = 'T';
  int acelerometroX, acelerometroY, acelerometroZ;
  int temperatura;
  int giroscopioX, giroscopioY, giroscopioZ;
};

GyroAccelData gyroAccelData;

volatile boolean received;

LiquidCrystal_I2C lcd(0x20, 16, 2);

unsigned int offset = 0;
float inteiro;

// flags de controle
bool SCAN_I2C = true;
bool IS_MISO = false;

bool send = false;
unsigned int interruptions = 0;

// functions
void getAccelerometerAndGyroscopeData();
void slaveSPI();
void log();
void findI2CAddress();

void setup()
{
  Serial.begin(9600);
  Serial.println("Setting up slave");

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  //Inicializa o MPU-6050
  Wire.write(0);
  Wire.endTransmission(true);

  if (IS_MISO)
  {
    pinMode(MISO, OUTPUT); // have to send on master in, *slave out*
    SPCR |= _BV(SPE);      //Turn on SPI in Slave Mode
    SPI.attachInterrupt(); //Interuupt ON is set for SPI commnucation
  }
  else
  {
    Serial.println("MISO mode off");
    SPI.begin();                         //Begins the SPI commnuication
    SPI.setClockDivider(SPI_CLOCK_DIV8); //Sets clock for SPI communication at 8 (16/8=2Mhz)
    digitalWrite(SS, HIGH);
  }
  received = false;

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
  if (SCAN_I2C)
  {
    findI2CAddress();
  }
  getAccelerometerAndGyroscopeData();
  slaveSPI();
}

void slaveSPI()
{
  if (received)
  {
    received = false;
    Serial.println("interruptions:");
    Serial.println(interruptions);
    delay(1000);
  }

  if (send)
  {
    digitalWrite(SS, LOW); //Starts communication with Slave connected to master
    log();
    int bytes = SPI_write(gyroAccelData);
    Serial.println("nro de bytes");
    Serial.println(bytes);
    send = false;
    digitalWrite(SS, HIGH);

    delay(3000);
  }
}

void getAccelerometerAndGyroscopeData()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU, 14, true);
  //Armazena o valor dos sensores nas variaveis correspondentes
  gyroAccelData.acelerometroX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  gyroAccelData.acelerometroY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  gyroAccelData.acelerometroZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  gyroAccelData.temperatura = Wire.read() << 8 | Wire.read();   //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  gyroAccelData.giroscopioX = Wire.read() << 8 | Wire.read();   //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  gyroAccelData.giroscopioY = Wire.read() << 8 | Wire.read();   //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  gyroAccelData.giroscopioZ = Wire.read() << 8 | Wire.read();   //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  gyroAccelData.temperatura = gyroAccelData.temperatura / 340.00 + 36.53;

  send = true;
}

void log()
{
  Serial.println("Slave sending");
  Serial.println("struct GyroAccelData");
  Serial.print("AX:");
  Serial.println(gyroAccelData.acelerometroX);
  Serial.print("AY:");
  Serial.println(gyroAccelData.acelerometroY);
  Serial.print("AZ:");
  Serial.println(gyroAccelData.acelerometroZ);
  Serial.print("GX:");
  Serial.println(gyroAccelData.giroscopioX);
  Serial.print("GY:");
  Serial.println(gyroAccelData.giroscopioY);
  Serial.print("GZ:");
  Serial.println(gyroAccelData.giroscopioZ);
  Serial.print("T:");
  Serial.println(gyroAccelData.temperatura);
}

void findI2CAddress()
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
    SCAN_I2C = false;
    Serial.println("done\n");
  }
  if (SCAN_I2C)
    delay(5000); // wait 5 seconds for next scan
}