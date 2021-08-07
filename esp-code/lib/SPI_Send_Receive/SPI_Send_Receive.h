template <typename T>
unsigned int SPI_write(const T &value)
{
  const byte *p = (const byte *)&value;
  unsigned int i;
  for (i = 0; i < sizeof (value); i++)
  {
    SPI.transfer(*p++);
  }
  return i;
}

// template <typename T> unsigned int SPI_readAnything_ISR(T& value, byte firstByte)
//   {
//     byte * p = (byte*) &value;
//     unsigned int i;
//     *p++ = SPDR;  // get first byte
//     for (i = 1; i < sizeof value; i++)
//           *p++ = SPI.transfer (0);
//     return i;
//   }  // end of SPI_readAnything_ISR

template <typename T>
unsigned int SPI_read(T &output, byte firstByte)
{
  byte *p = (byte *)&output; // cast qualquer valor para um ponteiro do tipo byte
  unsigned int i;

  *p++ = firstByte;

  for (i = 1; i < sizeof(output); i++)
  {
    *p++ = SPI.transfer(0);
  }

  return i;
}