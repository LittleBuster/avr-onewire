/* OneWire AVR library
 *
 * Copyright (C) 2016 Sergey Denisov.
 * Rewritten by Sergey Denisov aka LittleBuster (DenisovS21@gmail.com)
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public Licence
 * as published by the Free Software Foundation; either version 3
 * of the Licence, or (at your option) any later version.
 *
 * Original library written by admin@kibermaster.net (http://kibermaster.net)
 */

#include "onewire.h"
#include <util/delay.h>

#define sbi(reg,bit) reg |= (1<<bit)
#define cbi(reg,bit) reg &= ~(1<<bit)
#define ibi(reg,bit) reg ^= (1<<bit)
#define check_bit(reg,bit) (reg&(1<<bit))


static void onewire_set(uint8_t mode)
{
	if (mode)
		cbi(OW_PORT, OW_BIT_OUT);
	else
		sbi(OW_PORT, OW_BIT_OUT);
}

static uint8_t one_wire_check_in(void)
{
	return check_bit(OW_PIN, OW_BIT_IN);
}

uint8_t onewire_reset(void)
{
	uint8_t status;
	onewire_set(1);
	_delay_us(480);
	onewire_set(0);
	_delay_us(60);
	/* Store line value and wait until the completion of 480uS period */
	status = OW_CheckIn();
	_delay_us(420);
	/* Return the value read from the presence pulse (0=OK, 1=WRONG) */
	return !status;
}

void onewire_write_bit(uint8_t bit)
{
	/* Pull line low for 1uS */
	onewire_set(1);
	_delay_us(1);
	/* If we want to write 1, release the line (if not will keep low) */
	if(bit) onewire_set(0); 
	/* Wait for 60uS and release the line */
	_delay_us(60);
	onewire_set(0);
}

uint8_t onewire_read_bit(void)
{
	uint8_t bit=0;

	onewire_set(1);
	_delay_us(1);
	onewire_set(0);
	_delay_us(14);
	if(ow_check_in())
		bit = 1;
	_delay_us(45);

	return bit;	
}

void onewire_write_byte(uint8_t byte)
{
	for (uint8_t i = 0; i < 8; i++)
		onewire_write_bit(check_bit(byte, i));
}

uint8_t onewire_read_byte(void)
{
	uint8_t n = 0;

	for (uint8_t i = 0; i < 8; i++)
		if (onewire_read_bit())
			sbi(n, i);
	
	return n;
}

static uint8_t onewire_search_rom(uint8_t diff, uint8_t *id )
{ 	
	uint8_t i, j, next_diff;
	uint8_t b;

	if(!onewire_reset()) 
		return OW_PRESENCE_ERR;       // error, no device found

	onewire_write_byte(OW_CMD_SEARCHROM);     // ROM search command
	next_diff = OW_LAST_DEVICE;      // unchanged on last device
	
	i = OW_ROMCODE_SIZE * 8;         // 8 bytes
	do 
	{	
		j = 8;                        // 8 bits
		do 
		{ 
			b = onewire_read_bit();			// read bit
			if( onewire_read_bit() ) 
			{ // read complement bit
				if(b)                 // 11
					return OW_DATA_ERR;  // data error
			}
			else 
			{ 
				if(!b) { // 00 = 2 devices
				if(diff > i || ((*id & 1) && diff != i)) { 
						b = 1;               // now 1
						next_diff = i;       // next pass 0
					}
				}
			}
         onewire_write_bit(b);               // write bit
         *id >>= 1;
         if(b)
         	*id |= 0x80;			// store bit
         i--;
		} 
		while(--j);
		id++;                            // next byte
    } 
	while(i);
	return next_diff;                  // to continue search
}

void onewire_find_rom(uint8_t *diff, uint8_t id[])
{
	while(1)
    {
		*diff = onewire_search_rom( *diff, &id[0] );
    	if ( *diff==OW_PRESENCE_ERR || *diff==OW_DATA_ERR ||
    		*diff == OW_LAST_DEVICE ) return;
    	//if ( id[0] == DS18B20_ID || id[0] == DS18S20_ID ) 
		return;
    }
}

uint8_t onewire_read_rom(uint8_t *buffer)
{
	if (!onewire_reset())
		return 0;
	onewire_write_byte(OW_CMD_READROM);
	for (uint8_t i = 0; i < 8; i++)
	{
		buffer[i] = onewire_read_byte();
	}
 return 1;
}

uint8_t onewire_match_rom(uint8_t *rom)
{
 	if (!onewire_reset())
 		return 0;
	onewire_write_byte(OW_CMD_MATCHROM);	
	for(uint8_t i = 0; i < 8; i++)
		onewire_write_byte(rom[i]);

	return 1;
}
