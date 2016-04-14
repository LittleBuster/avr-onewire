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

#ifndef __ONEWIRE_H__
#define __ONEWIRE_H__

#include <stdio.h>
#include <avr/io.h>

#define MAXDEVICES 10

/* Need to be change for other ports */
#define OW_DDR DDRB
#define OW_PORT PORTB
#define OW_PIN PINB


#define OW_BIT_OUT 1
#define OW_BIT_IN 0

#define OW_CMD_SEARCHROM	0xF0
#define OW_CMD_READROM		0x33
#define OW_CMD_MATCHROM		0x55
#define OW_CMD_SKIPROM		0xCC

#define	OW_SEARCH_FIRST	0xFF
#define	OW_PRESENCE_ERR	0xFF
#define	OW_DATA_ERR	    0xFE
#define OW_LAST_DEVICE	0x00

#define OW_DS1990_FAMILY_CODE	1
#define OW_DS2405_FAMILY_CODE	5
#define OW_DS2413_FAMILY_CODE	0x3A
#define OW_DS1822_FAMILY_CODE	0x22
#define OW_DS2430_FAMILY_CODE	0x14
#define OW_DS1990_FAMILY_CODE	1
#define OW_DS2431_FAMILY_CODE	0x2d
#define OW_DS18S20_FAMILY_CODE	0x10
#define OW_DS18B20_FAMILY_CODE	0x28
#define OW_DS2433_FAMILY_CODE	0x23

/* rom-code size including CRC */
#define OW_ROMCODE_SIZE	8

/*
 * Reset transfer
 */
uint8_t onewire_reset(void);

/**
 * Sending bit to device
 * @bit: sending bit
 */
void onewire_write_bit(uint8_t bit);

/**
 * Reading bit from device
 * 
 * returns: readed bit.
 */
uint8_t onewire_read_bit(void);

/**
 * Sending byte to device
 * @byte: sending byte
 */
uint8_t onewire_write_byte(uint8_t byte);

/*
 * Start reading byte from device
 */
#define onewire_read_byte() onewire_write_byte(0xFF)

/**
 * Finding ROM of device
 * @diff:
 * @id: returns ID of finded device
 */
void onewire_find_rom(uint8_t *diff, uint8_t id[]);

/**
 * Reading ROM from device
 * @buffer: ROM
 */
uint8_t onewire_read_rom(uint8_t *buffer);

/**
 * Compare ROMs
 * @rom: new ROM
 */
uint8_t onewire_match_rom(uint8_t *rom);


#endif