#include "Arduino.h"
#ifndef DataBackup_h
#define DataBackup_h

class DataBackup {
	public:
		void readVcc();
		MazeValues mazeRestore();
		MazeValues mazeBackup();
		void lowPowerBackup();


	private:
		void EEPROM_write(uint16_t address, uint8_t data);
		void EEPROM_read(uint16_t address, uint8_t data);
		void vccReadSetup();


};

#endif
