#include "RotaryEncoder.h"
#include <wiringPi.h>


int RotaryEncoder::encoderCount = 2;
RotaryEncoder* RotaryEncoder::encoders[2] = { NULL, NULL };


RotaryEncoder::RotaryEncoder(int a, int b, int c, int index)
{	
	pin_a = a;
	pin_b = b;
	pin_c = c;
	
	//bcm2835_gpio_fsel(pin_a, BCM2835_GPIO_FSEL_INPT); /* First Edge */
	//bcm2835_gpio_fsel(pin_a, BCM2835_GPIO_FSEL_INPT); /* Second Edge */
	//bcm2835_gpio_fsel(pin_a, BCM2835_GPIO_FSEL_INPT); /* Push Button */ 	
	
	pinMode(pin_a, INPUT);
	pinMode(pin_b, INPUT);
	pinMode(pin_c, INPUT);
	
	wiringPiISR(pin_a, INT_EDGE_BOTH, RotaryEncoder::eventHandler);
	wiringPiISR(pin_b, INT_EDGE_BOTH, RotaryEncoder::eventHandler);
	wiringPiISR(pin_c, INT_EDGE_BOTH, RotaryEncoder::pressHandler);

	
	//bcm2835_gpio_len(pin_a);
	//bcm2835_gpio_hen(pin_a);
	//bcm2835_gpio_len(pin_b);
	//bcm2835_gpio_hen(pin_b);
	//bcm2835_gpio_len(pin_c);
	//bcm2835_gpio_hen(pin_c);
	encoders[index] = this;
}


RotaryEncoder::~RotaryEncoder()
{
}


void RotaryEncoder::eventHandler(void)
{
	for (int i = 0; i < RotaryEncoder::encoderCount; i++)
	{
		RotaryEncoder* enc = encoders[i];
	
		char MSB = digitalRead(enc->pin_a);
		char LSB = digitalRead(enc->pin_b);

		int encoded = (MSB << 1) | LSB;
		int sum = (enc->lastEncoded << 2) | encoded;

		/* need to throw out the encoder ticks that land between detents on the knob...*/
		if (sum == 0b1101 || /*sum == 0b0100 || sum == 0b0010 || */ sum == 0b1011) enc->value++;
		if (sum == 0b1110 ||  sum == 0b0111 /* || sum == 0b0001 || sum == 0b1000*/) enc->value--;

		enc->lastEncoded = encoded;	
	}
}

void RotaryEncoder::pressHandler(void)
{
	for (int i = 0; i < RotaryEncoder::encoderCount; i++)
	{
		RotaryEncoder* enc = encoders[i];
		char cur;
		cur = digitalRead(enc->pin_c);
		if (cur != enc->lastPressState && !cur)
		{
			enc->press++;
		}
		enc->lastPressState = cur;
	}
}


int RotaryEncoder::getValue(void)
{
	return (value);
}


void RotaryEncoder::setValue(int val)
{
	value = val;
}

bool RotaryEncoder::getSinglePress(void)
{
	bool pushOccurred = false;
	if (press != lastPress) pushOccurred = true;
	lastPress = press;
	return pushOccurred;
}


int RotaryEncoder::getPress(bool clear)
{
	int ret = press;
	if (clear)
	{
		press = 0;
		lastPress = press;
	}
	return (ret);
}
