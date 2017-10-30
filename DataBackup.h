/* DataBackup code is placed under the MIT license
 * Copyright (c) 2017 Ricardo Alcaraz
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */



#include "Arduino.h"
#ifndef DataBackup_h
#define DataBackup_h
#if !defined(FastCRC_h)
#define FastCRC_h
#include "inttypes.h"


// ================= 8-BIT CRC ===================
class FastCRC8 {
	public:
	  FastCRC8();
	  	uint8_t smbus(const uint8_t *data, const uint16_t datalen);		// Alias CRC-8
	  	uint8_t maxim(const uint8_t *data, const uint16_t datalen);		// Equivalent to _crc_ibutton_update() in crc16.h from avr_libc
	  	uint8_t smbus_upd(const uint8_t *data, uint16_t datalen);			// Call for subsequent calculations with previous seed.
	  	uint8_t maxim_upd(const uint8_t *data, uint16_t datalen);			// Call for subsequent calculations with previous seed.
	private:
  		uint8_t seed;

};

template<class T>
class DataBackup {
	public:
		DataBackup(const T &data);
		uint8_t readVcc();
		void lowPowerBackup();
		uint16_t EEPROMBackup( uint16_t index );
		T EEPROMRestore( const T &data, uint16_t index);
		uint8_t getCRC8();

	private:
		uint8_t EEPROM_read(uint16_t uiAddress);
		void EEPROM_write(uint16_t uiAddress, uint8_t ucData);
		void vccReadSetup();
		uint8_t partitions;
		uint8_t signature;
		uint16_t data_address;
		uint16_t data_size;
		uint16_t index;
		FastCRC8 key;
};


#endif
#endif

/* FastCRC library code is placed under the MIT license
 * Copyright (c) 2014,2015 Frank Bösing
 * 
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */