/*
 * sr595.c:
 *      Shift register test program
 *
 * Copyright (c) 2012-2013 Gordon Henderson. <projects@drogon.net>
 *
 * Compile: gcc -Wall -o sr595 sr595.c -lwiringPi
 *
 * Shift register control, works on 74HC595 and TLC5917 Tri-color LED driver.
 */

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <sr595.h>

int basepin = 64;
int pins = 4;

void loop1()
{
   int i;
   int r;

    for (i = 0 ; i < pins ; i++) {
     r = rand()%pins;
     digitalWrite (basepin+r, 1);
     digitalWrite (basepin+i, 1);
     delay (500) ;
     digitalWrite (basepin+r, 0);
     digitalWrite (basepin+i, 0);
     delay (500) ;
   }

}

void loop()
{
   int i;

    for (i = 0 ; i < pins ; ++i) {
     digitalWrite (basepin+i, 1);
     delay (500) ;
     digitalWrite (basepin+i, 0);
     delay (500) ;
   }

}

int main (void)
{

  wiringPiSetup () ;

// Use wiringPi pins 0, 1 & 2 for data, clock and latch
//sr595Setup (basepin_min_is_64, num_of_pins, data, clock, latch) ;
  sr595Setup (64, pins, 12, 14, 2);

  printf ("Raspberry Pi - Shift Register Test\n") ;

  for (;;)
  {
    loop1();
  }

  return 0 ;
}
