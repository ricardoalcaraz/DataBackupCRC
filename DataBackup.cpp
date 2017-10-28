/*
MIT License

Copyright (c) 2017  Ricardo Alcaraz

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "DataBackup.h"
#include "Arduino.h"

//Constructors
DataBackup::DataBackup(uint8_t number_of_partitions){
	this->partitions = number_of_partitions;
}

DataBackup::DataBackup(){
	this ->partitions = 1;
}

/*
 * Write data to EEPROM, NOTE: interrupts are disabled while writing 
 * @param uiAddress 16 bit interger pointing to the address of the data to write
 * @param ucData 8 bit value signifying the data being written
 */

void DataBackup::EEPROM_write(uint16_t uiAddress, uint8_t ucData) {
  /*Store SREG value before we disable Interrupts*/
  char SREG_save = SREG;
  noInterrupts();
  /* Wait for completion of any Flash Write
        Note:Only necessary if Flash Memory Manipulation is taking place */
  while(SPMCSR &(1<<SPMEN));
  /* Wait for completion of previous write */
  while(EECR & (1<<EEPE));
  /* Set up address and Data Registers */
  EEAR = uiAddress;
  EEDR = ucData;
  /* Write logical one to EEMPE */
  EECR |= (1<<EEMPE);
  /* Start eeprom write by setting EEPE */
  EECR |= (1<<EEPE);
  /*Restore the SREG value*/
  SREG = SREG_save;
}

/*
 * Read data from EEPROM, NOTE: interrupts are disabled while writing 
 * @param uiAddress 16 bit interger pointing to the address of the data to read
 * @return 8 bit value signifying the data that was read
 */
uint8_t DataBackup::EEPROM_read(unsigned int uiAddress) {
  /*Store SREG value before we disable Interrupts*/
  char SREG_save = SREG;
  noInterrupts();
  /* Wait for completion of any Flash Write
        Note:Only necessary if Flash Memory Manipulation is taking place */
  while(SPMCSR &(1<<SPMEN));
  /* Wait for completion of previous write */
  while(EECR & (1<<EEPE));
  /* Set up address register */
  EEAR = uiAddress;
  /* Start eeprom read by writing EERE */
  EECR |= (1<<EERE);
  /*Restore the SREG value*/
  SREG = SREG_save;
  /* Return data from Data Register */
  return EEDR;
}

/* Function to setup the ADC as a voltage reader
 * Sets the ADC inputs to be a bandgap reference and the VCC
 * INPUTS: None
 * OUTPUTS: None
 */
void DataBackup::vccReadSetup(){
  //We want 0b011110 for the mux bits and 0b10 for the Voltage ref bits
  /*Setting voltage refereance to Vcc and ensuring a right adjust*/
  ADMUX &= ~( (1<<REFS1) & (1<<ADLAR) & (1<<MUX0) );
  /*Setting the values for our bandgap reference input*/
  ADMUX |= (1<<REFS0) | (1<<MUX4) | (1<<MUX3) | (1<<MUX2) | (1<<MUX1);
  ADCSRB &= ~( (1<<MUX5) );
  /*Setting the prescaler to clock/128*/
  ADCSRA |= ( (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0) ); 
  /*ADC needs a 1ms settling time before the ADC is ready to be used*/
  delay(1);
}

/* Reads the Vcc voltage by comparing the Vcc input with the bandgap reference
 * Answer is multiplied by 10 to simplify calculations
 * INPUTS: None
 * @return vcc10 - Vcc*10;
 */
uint8_t DataBackup::readVcc(){
  /*pushing the ADC registers to ensure we don't mess anything up*/
  uint8_t ADCA_backup = ADCSRA;
  uint8_t ADCB_backup = ADCSRB;
  uint8_t ADMUX_backup = ADMUX;
  vccReadSetup();
  /*Start Conversion*/
  ADCSRA |= (1<<ADSC);
  /*Wait for conversion to be ready*/
  while( ADCSRA & (1<<ADSC) );

  /*Read the results and concatenate it into a 16 bit value*/
  uint8_t low = ADCL;
  uint8_t high = ADCH;
  uint16_t adc = (high << 8) | low;

  /*Vcc = (1.1V * 1024) / ADC*/
  /*Multiply by 10 to simplify math*/
  /*Vcc10 = (11V * 1024)/ADC */
  uint8_t vcc10 = (uint8_t) ( (11*1024) / adc );
  /*Popping the register backups*/
   ADMUX = ADMUX_backup;
   ADCSRB = ADCB_backup;
   ADCSRA = ADCA_backup;
  /*The data sheet recommends turning the ADC off when not in use when running on battery power
   *The ADC does not automatically turn off when it goes into sleep modes*/
   return vcc10;  
}


