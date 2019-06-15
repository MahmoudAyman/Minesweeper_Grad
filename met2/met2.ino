#include <FreqCounter.h>
#define buzz 3
long int frq;
long int total = 0;
long int lasttotal = 0;

void setup() {
  Serial.begin(9600);                    // connect to the serial port
  Serial.println("Frequency Counter");
  pinMode(buzz, OUTPUT);
}


void loop() 
{

	FreqCounter::f_comp= 8;             // Set compensation to 12
	FreqCounter::start(100);            // Start counting with gatetime of 100ms
	for(int i =0; i<100; i++){
		while (FreqCounter::f_ready == 0);

		frq=FreqCounter::f_freq;            // read result
		total+=frq;
	}
	total/=100;
	if(lasttotal == 0)
	{
		lasttotal=total;
	}
	else
	{
		int diff = total - lasttotal;
		int diffAbs = abs(diff);
                Serial.println(diffAbs);
		if(diffAbs > 10)
		{
			Serial.println("METAL DETEC");
                        digitalWrite(buzz, HIGH);
		}
                else
                {
                  digitalWrite(buzz, LOW);
                }
		lasttotal = total;
	}
	total = 0;
	//Serial.println(total);                // print result
	delay(20);
}
