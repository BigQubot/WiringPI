#include <stdio.h>
#include <wiringPi.h>

unsigned char getGpioNum(void)
{
	int model = -1;

	piBoardId (&model);

	switch (model)
	{
		case PI_MODEL_BERRY:
		case PI_MODEL_ZERO:
			return 28;
			break;
		default:
			printf ("Oops - unable to determine board type... model: %d\n", model);
			return -1;
			break;
	}
}

int main (void)
{
	int i = 0;
	unsigned char gpio_num = 0;

	wiringPiSetup();

	gpio_num = getGpioNum();
	if (-1 == gpio_num)
		printf("Failed to get the number of GPIO!\n");

	for (i = 0; i < gpio_num; i++)
		pinMode (i, OUTPUT) ;

	for ( ;; )
	{
		for (i = 0; i < gpio_num; i++)
			digitalWrite (i, HIGH);	// On
		delay (2000);

		for (i = 0; i < gpio_num; i++)
			digitalWrite (i, LOW);	// Off
		delay (2000);
	}

	return 0;
}
