
#include <Arduino.h>
//Carrega a biblioteca Wire

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

//Endereco I2C do MPU6050
const int MPU = 0x68;
//Variaveis para armazenar valores dos sensores
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

//flags de controle
bool SCAN_LCD = true;

// scan do endereço do LCD
void findLCDAddress();

void getCoordinates();

void masterSPI();

// instanciar LCD
LiquidCrystal_I2C lcd(0x38, 16, 2);

void setup()
{
  Serial.begin(9600);

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);

  Serial.begin(9600);
  while (!Serial)
    ; // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner");

  //Inicializa o MPU-6050
  Wire.write(0);
  Wire.endTransmission(true);

  lcd.init();      // Inicializando o LCD
  lcd.backlight(); // Ligando o BackLight do LCD

  // inicializa o SPI para comunicação mestre-escravo -mestre
  SPI.begin();                         //Begins the SPI commnuication
  SPI.setClockDivider(SPI_CLOCK_DIV8); //Sets clock for SPI communication at 8 (16/8=2Mhz)
  digitalWrite(SS, HIGH);              // Setting SlaveSelect as HIGH (So master doesnt connnect with slave)
}

void loop()
{
  if (SCAN_LCD)
  {
    findLCDAddress();
  }
  getCoordinates();

  lcd.clear();
  lcd.print("T=");
  lcd.print(Tmp);
  lcd.setCursor(0, 1);

  masterSPI();
}

void masterSPI()
{
  byte masterSend, masterReceive;
  digitalWrite(SS, LOW); //Starts communication with Slave connected to master

  masterSend = 2;
  masterReceive = SPI.transfer(masterSend);

  Serial.println("Received data: " + masterReceive);

  delay(3000);
}

void getCoordinates()
{
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  //Solicita os dados do sensor
  Wire.requestFrom(MPU, 14, true);
  //Armazena o valor dos sensores nas variaveis correspondentes
  AcX = Wire.read() << 8 | Wire.read(); //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX = Wire.read() << 8 | Wire.read(); //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY = Wire.read() << 8 | Wire.read(); //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ = Wire.read() << 8 | Wire.read(); //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //Envia valor X do acelerometro para a serial e o LCD
  //Serial.print("AcX = ");
  //Serial.print(AcX);

  //Envia valor Y do acelerometro para a serial e o LCD
  //Serial.print(" | AcY = ");
  //Serial.print(AcY);

  //Envia valor Z do acelerometro para a serial e o LCD
  //Serial.print(" | AcZ = ");
  //Serial.print(AcZ);

  //Envia valor da temperatura para a serial e o LCD
  //Calcula a temperatura em graus Celsius
  //Serial.print(" | Tmp = ");
  //Serial.print(Tmp / 340.00 + 36.53);
  Tmp = Tmp / 340.00 + 36.53;

  //Envia valor X do giroscopio para a serial e o LCD
  //Serial.print(" | GyX = ");
  //Serial.print(GyX);

  //Envia valor Y do giroscopio para a serial e o LCD
  //Serial.print(" | GyY = ");
  //Serial.print(GyY);

  //Envia valor Z do giroscopio para a serial e o LCD
  //Serial.print(" | GyZ = ");
  //Serial.println(GyZ);

  //Aguarda 300 ms e reinicia o processo
  delay(300);
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

  delay(5000); // wait 5 seconds for next scan
}