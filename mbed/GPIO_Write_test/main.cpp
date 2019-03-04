#include "mbed.h"

DigitalOut CheckPin1(D2);

int main() {
	while (true) {
		CheckPin1 = 1;
		CheckPin1 = 0;
	}
}
