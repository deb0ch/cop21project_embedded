// Akeru library - http://akeru.cc
//
// copyleft Snootlab, 2014
// this code is public domain, enjoy!

#include <Arduino.h>
#include "Akeru.h"

Akeru_ Akeru;

Akeru_::Akeru_()
  : _serial(5, 4)
{
  _lastSend = -1;     // Since _lastSend is unsigned, this is infinity
}

Akeru_::~Akeru_() {}

void Akeru_::begin()
{
  _serial.begin(9600);

  _serial.write((uint8_t)'\0');  // Remove un-ended commands from TST's buffer
  _serial.write((uint8_t)';');

  while (_serial.available() < 3)  // Wait for the "KO;"
    ;
  _serial.read(); //'K'
  _serial.read(); //'O'
  _serial.read(); //';'
}


bool Akeru_::isReady() {

  // IMPORTANT WARNING. PLEASE READ BEFORE MODIFYING THE CODE
  //
  // The Sigfox network operates on public frequencies. To comply with
  // radio regulation, it can send radio data a maximum of 1% of the time
  // to leave room to other devices using the same frequencies.
  //
  // Sending a message takes about 6 seconds (it's sent 3 times for
  // redundancy purposes), meaning the interval between messages should
  // be 10 minutes.
  //
  // Also make sure your send rate complies with the restrictions set
  // by the particular subscription contract you have with your Sigfox
  // network operator.
  //
  // FAILING TO COMPLY WITH THESE CONSTRAINTS MAY CAUSE YOUR MODEM
  // TO BE BLOCKED BY YOUR SIFGOX NETWORK OPERATOR.
  //
  // You've been warned!

  unsigned long currentTime = millis();

  if (currentTime >= _lastSend && (currentTime - _lastSend) <= 600000)
    return false;

  // Time is ok, ask the modem's status
  _serial.write((uint8_t)'\0');
  _serial.write((uint8_t)'S');
  _serial.write((uint8_t)'F');
  _serial.write((uint8_t)'P');
  _serial.write((uint8_t)';');

  return _nextReturn() == OK;
}

bool Akeru_::send(const void* data, uint8_t len)
{
  uint8_t* bytes = (uint8_t*)data;

  if (!isReady())
    return false;

  _lastSend = millis();  // See comment in isReady()

  _serial.write((uint8_t)'\0');
  _serial.write((uint8_t)'S');
  _serial.write((uint8_t)'F');
  _serial.write((uint8_t)'M');
  _serial.write(len);
  for (uint8_t i = 0; i < len; ++i)
    {
      _serial.write(bytes[i]);
    }
  _serial.write(';');

  uint8_t ok = _nextReturn();

  if (ok == OK)
    {
      _nextReturn(); // SENT
      return true;
    }
  return false;
}

uint8_t Akeru_::getRev()
{
  _serial.write((uint8_t)'\0');
  _serial.write((uint8_t)'S');
  _serial.write((uint8_t)'F');
  _serial.write((uint8_t)'v');
  _serial.write((uint8_t)';');

  while (_serial.available() < 3)
    ;
  if (_serial.peek() == 'K')
    {
      _serial.read(); //'K'
      _serial.read(); //'O'
      _serial.read(); //';'
      return 0;
    }
  else
    {
    while (_serial.available() < 5)
      ;

    uint8_t rev = 10 * (_serial.read() - '0') + (_serial.read() - '0');

    _serial.read(); //'O'
    _serial.read(); //'K'
    _serial.read(); //';'

    return rev;
  }
}

unsigned long Akeru_::getID()
{
  _serial.write((uint8_t)'\0');
  _serial.write((uint8_t)'S');
  _serial.write((uint8_t)'F');
  _serial.write((uint8_t)'I');
  _serial.write((uint8_t)'D');
  _serial.write((uint8_t)';');

  uint8_t response[8] = {0};  // Response is [byte1, byte2, ..., byteN, 'O', 'K']
  uint8_t i = 0;

  while(!_serial.available())
    ;
  while (_serial.peek() != ';')
    {
      response[i] = _serial.read();
      while (!_serial.available())
	;
      ++i;
    }
  _serial.read(); // ';'

  unsigned long id = 0;

  for(uint8_t j = 0; j < i-2; ++j)
    {
      id += response[j] << ((i - 3 - j) * 8);
    }
  return id;
}

// Power value:
// 0 -25 -30 dBm
// 1 0dBm
// 2 14dBm
// 3 16dBm
// 4 18dBm
// 5 Max (18-19dBm)
bool Akeru_::setPower(uint8_t power)
{
  power = power % 6; // It's 0-5
  _serial.write((uint8_t)'\0');
  _serial.write((uint8_t)'S');
  _serial.write((uint8_t)'F');
  _serial.write((uint8_t)'G');
  _serial.write(power);
  _serial.write((uint8_t)';');
  return _nextReturn() == OK;
}

uint8_t Akeru_::_nextReturn()
{
  while (!_serial.available())
    ;

  char fstChar = _serial.read();

  while (_serial.read() != ';')
    ;
  return fstChar;
}
