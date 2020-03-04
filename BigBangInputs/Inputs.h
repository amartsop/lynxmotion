#ifndef _Inputs_h
#define _Inputs_h

class Potentiometer
{

  private:
    int pinnum;

  public:
    Potentiometer(int pot_pin);
    ReadVal();
};


// Set a pin number to the Potentiometer class
Potentiometer::Potentiometer(int pot_pin)
{
  pinnum = pot_pin;
}

//Return value read from the potentiometer
int Potentiometer::ReadVal()
{
  int val = analogRead(pinnum);

  return val;
}

class Button
{

  private:
    int pinnum;

  public:
    Button(int butt_pin);
    ReadVal();
  
};

Button::Button(int butt_pin)
{
  pinMode(butt_pin, INPUT_PULLUP);
  pinnum = butt_pin;
}

int Button::ReadVal()
{
  int val = digitalRead(pinnum);

  return val;
}

#endif
