#include "mbed.h"
#include <stdio.h>
#include <math.h>

#define PI_F	(3.14159265f)
#define LOOP_N	(1000)

DigitalOut CheckPin1(D2);

volatile float buffer[LOOP_N];

int main() {
    while (true) {
		CheckPin1 = 1;
		volatile float fv = 0.0f;
		for (int i = 0; i < LOOP_N; i++) {
			fv += 0.001f;
			//buffer[i] = fv + PI_F;	// Add
			//buffer[i] = fv - PI_F;	// Sub
			//buffer[i] = fv * PI_F;	// Mul
			//buffer[i] = fv / PI_F;	// Div
			buffer[i] = sinf(fv);
			//buffer[i] = cosf(fv);
			//buffer[i] = tanf(fv);
			//buffer[i] = expf(fv);
			//buffer[i] = logf(fv);
			//buffer[i] = powf(2.0f, fv);
		}
		CheckPin1 = 0;
    }
}
