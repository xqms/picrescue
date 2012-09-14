
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include <libucomm/envelope.h>
#include <libucomm/io.h>
#include <libucomm/checksum.h>
#include "proto_picrescue.h"

const uint32_t OPCODE_NOP  = 0;
const uint32_t OPCODE_GOTO = 0x040200UL;

const uint32_t REG_ID = 0xFF0000UL;

void uart_init()
{
	UCSRA = (1 << U2X);
	UCSRB = (1 << RXEN) | (1 << TXEN);
	UBRRL = 16; // 115200 Baud
	UBRRH = 0;
}

void uart_write(uint8_t c)
{
	while(!(UCSRA & (1 << UDRE)));
	UDR = c;
}

int uart_stdio_putc(char c, FILE*)
{
	if(c == '\n')
		uart_write('\r');

	uart_write(c);
	return 0;
}

uint8_t uart_read()
{
	while(!(UCSRA & (1 << RXC)));
	return UDR;
}

class UARTWriter
{
public:
	bool writeChar(uint8_t c)
	{
		uart_write(c);
		return true;
	}
};

UARTWriter g_uartWriter;

typedef uc::EnvelopeWriter< uc::InvertedModSumGenerator, UARTWriter > EnvelopeWriter;
typedef uc::IO< EnvelopeWriter, uc::IO_W > SimpleWriter;
typedef Proto< SimpleWriter > WProto;

typedef uc::EnvelopeReader< uc::InvertedModSumGenerator, 1024 > EnvelopeReader;
typedef uc::IO< EnvelopeReader, uc::IO_R > SimpleReader;
typedef Proto< SimpleReader > RProto;

WProto::Data g_readBuffer;
RProto::WriteData g_writeBuffer;



static FILE mystdout;

void bbDATA(bool on)
{
	if(on)
		DDRA &= ~(1 << 1);
	else
		DDRA |= (1 << 1);
}

void bbCLK(bool on)
{
	if(on)
		DDRA &= ~(1 << 0);
	else
		DDRA |= (1 << 0);
}

void bbMCLR(bool on)
{
	if(on)
		DDRA &= ~(1 << 2);
	else
		DDRA |= (1 << 2);
}

void bbAUX(bool on)
{
	if(on)
		PORTA |= (1 << 3);
	else
		PORTA &= ~(1 << 3);
}

void spi_writebit(bool data)
{
	bbDATA(data);
	_delay_us(100);
	bbCLK(1);
	_delay_us(100);
	bbCLK(0);
}

void spi_write(uint8_t bits, uint32_t value)
{
	for(uint8_t i = 0; i < bits; ++i)
		spi_writebit(!!(value & (1L << (bits - i - 1))));
}

void spi_writeLSBFirst(uint8_t bits, uint32_t value)
{
	for(uint8_t i = 0; i < bits; ++i)
		spi_writebit(value & (1L << i));
}

void icsp_six(uint32_t opcode)
{
	bbAUX(1);
	spi_writeLSBFirst(4, 0); // SIX code
	bbAUX(0);
	_delay_us(30);
	spi_writeLSBFirst(24, opcode);
	_delay_us(30);
}

uint16_t icsp_read()
{
	uint16_t dest = 0;

	// REGOUT code
	spi_writeLSBFirst(4, 1);
	_delay_us(30);

	// CPU idle
	spi_writeLSBFirst(8, 0);
	bbDATA(1); // Input mode
	_delay_us(30);

	for(uint8_t i = 0; i < 16; ++i)
	{
		if(PINA & (1 << 1))
			dest |= (1 << i);
		bbCLK(1);
		_delay_us(30);
		bbCLK(0);
		_delay_us(30);
	}

	return dest;
}

void icsp_init()
{
	bbDATA(0);
	bbCLK(0);
	bbMCLR(0);
	_delay_us(500);

	// Short MCLR pulse
	bbMCLR(1);
	_delay_us(500);
	bbMCLR(0);

	// ICSP key
	spi_write(32, 0x4D434851);
	_delay_ms(1);

	bbMCLR(1);

	// First SIX instruction is NOP and takes 5 additional cycles
	spi_write(5, 0);
	icsp_six(0);
	icsp_six(OPCODE_GOTO);
	icsp_six(0);
}

void icsp_exit()
{
	bbCLK(0);
	_delay_ms(4);
	bbMCLR(0);
	_delay_ms(4);
}

void icsp_readCode(uint32_t addr, uint16_t* dest, uint8_t len)
{
	// MOV #<SourceAddress23:16>, W0
	icsp_six(0x200000 | ((addr & 0xFFFF0000) >> 12));

	// MOV W0, TBLPAG
	icsp_six(0x880190);

	// MOV #<SourceAddress15:0>, W6
	icsp_six(0x200006 | ((addr & 0x0000FFFF) << 4));

	// MOV #VISI, W7
	icsp_six(0x207847);
	icsp_six(0);

	for(uint8_t i = 0; i < len; i += 3)
	{
		uint16_t lsw1, lsw2, msb;

		// TBLRDL [W6], [W7]
		icsp_six(0xBA0B96);
		icsp_six(0);
		icsp_six(0);
		lsw1 = icsp_read();

		// TBLRDH.B [W6++], [W7++]
		icsp_six(0xBADBB6);
		icsp_six(0);
		icsp_six(0);

		// TBLRDH.B [++W6], [W7--]
		icsp_six(0xBAD3D6);
		icsp_six(0);
		icsp_six(0);
		msb = icsp_read();

		// TBLRDL [W6++], [W7]
		icsp_six(0xBA0BB6);
		icsp_six(0);
		icsp_six(0);
		lsw2 = icsp_read();

		dest[i+0] = lsw1;
		dest[i+1] = msb;
		dest[i+2] = lsw2;

		icsp_six(0);
		icsp_six(OPCODE_GOTO);
		icsp_six(0);
	}
}

uint16_t icsp_readWord(uint32_t addr)
{
	icsp_six(0x200000 | ((addr & 0xFF0000) >> 12));
	icsp_six(0x880190);
	icsp_six(0x200006 | ((addr & 0x00FFFF) << 4));
	icsp_six(0x207847);
	icsp_six(0);
	
	icsp_six(0xBA0BB6);
	icsp_six(0);
	icsp_six(0);
	
	uint16_t ret = icsp_read();
	
	icsp_six(OPCODE_NOP);
	icsp_six(0);

	return ret;
}

void icsp_readConfig(uint16_t* config1, uint16_t* config2)
{
	const uint32_t addr = 0x00ABFC;

	icsp_six(0x200000 | ((addr & 0xFF0000) >> 12));
	icsp_six(0x880190);
	icsp_six(0x200006 | ((addr & 0x00FFFF) << 4));
	icsp_six(0x207847);
	icsp_six(0);

	icsp_six(0xBA0BB6);
	icsp_six(0);
	icsp_six(0);

	*config1 = icsp_read();

	icsp_six(0xBA0BB6);
	icsp_six(0);
	icsp_six(0);

	*config2 = icsp_read();

	icsp_six(OPCODE_NOP);
	icsp_six(0);
}

bool icsp_doWrite()
{
	// Initiate the write cycle
	icsp_six(0xA8E761);
	icsp_six(0);
	icsp_six(0);
	
	// Poll WR bit
	for(uint8_t i = 0; i < 256; ++i)
	{
		icsp_six(OPCODE_GOTO);
		icsp_six(0);
		
		icsp_six(0x803B02);
		icsp_six(0x883C22);
		icsp_six(0);
		
		uint16_t nvmcon = icsp_read();
		
		icsp_six(0);
		
		if(!(nvmcon & (1 << 15)))
			return true;
	}

	return false;
}

bool icsp_chipErase(bool include_config)
{
	const int PAGE = include_config ? 0x80 : 0;
	// Set NVMCON
	icsp_six(0x2404FA);
	icsp_six(0x883B0A);

	// Set TBLPAG and perform dummy table write
	icsp_six(0x200000 | PAGE << 4);
	icsp_six(0x880190);
	icsp_six(0x200000);
	icsp_six(0xBB0800);
	icsp_six(0);
	icsp_six(0);

	return icsp_doWrite();
}

bool icsp_writeConfig(uint16_t cw1, uint16_t cw2)
{
	const int addr = 0x00ABFC;
	icsp_six(0x200007 | ((addr & 0x00FFFF) << 4));
	icsp_six(0x24003A);
	icsp_six(0x883B0A);
	icsp_six(0x200000 | ((addr & 0xFF0000) >> 12));
	icsp_six(0x880190);

	icsp_six(0x200006 | (((uint32_t)cw2) << 4));
	icsp_six(0);
	icsp_six(0xBB1B86);
	icsp_six(0);
	icsp_six(0);
	if(!icsp_doWrite())
		return false;
	icsp_six(OPCODE_NOP);
	icsp_six(OPCODE_GOTO);
	icsp_six(OPCODE_NOP);

	icsp_six(0x200006 | (((uint32_t)cw1) << 4));
	icsp_six(0);
	icsp_six(0xBB1B86);
	icsp_six(0);
	icsp_six(0);
	if(!icsp_doWrite())
		return false;
	icsp_six(OPCODE_NOP);
	icsp_six(OPCODE_GOTO);
	icsp_six(OPCODE_NOP);

	return true;
}

bool icsp_writeCode(uint32_t addr, const uint16_t* src)
{
	icsp_six(0x24001A);
	icsp_six(0x883B0A);

	icsp_six(0x200000 | ((addr & 0xFF0000) >> 12));
	icsp_six(0x880190);
	icsp_six(0x200007 | ((addr & 0x00FFFF) << 4));

	for(uint8_t i = 0; i < 16; ++i)
	{
		for(uint8_t j = 0; j < 6; ++j)
			icsp_six(0x200000 | ((uint32_t)src[6*i + j] << 4) | j);

		icsp_six(0xEB0300);
		icsp_six(0);

		icsp_six(0xBB0BB6);
		icsp_six(0);
		icsp_six(0);
		icsp_six(0xBBDBB6);
		icsp_six(0);
		icsp_six(0);
		icsp_six(0xBBEBB6);
		icsp_six(0);
		icsp_six(0);
		icsp_six(0xBB1BB6);
		icsp_six(0);
		icsp_six(0);
		icsp_six(0xBB0BB6);
		icsp_six(0);
		icsp_six(0);
		icsp_six(0xBBDBB6);
		icsp_six(0);
		icsp_six(0);
		icsp_six(0xBBEBB6);
		icsp_six(0);
		icsp_six(0);
		icsp_six(0xBB1BB6);
		icsp_six(0);
		icsp_six(0);
	}

	return icsp_doWrite();
}

void handleMsg(EnvelopeReader* input)
{
	EnvelopeWriter writer(&g_uartWriter);

	switch(input->msgCode())
	{
		case RProto::Ping::MSG_CODE:
			writer << WProto::Pong();
			break;
		case RProto::ReadDeviceID::MSG_CODE:
		{
			icsp_init();
			uint16_t id[3];
			icsp_readCode(REG_ID, id, 3);
			icsp_exit();

			WProto::DeviceIDReply reply;
			reply.devid =  (((uint32_t)id[1] & 0x00FF) << 16) | id[0];
			reply.devrev = (((uint32_t)id[1] & 0xFF00) <<  8) | id[2];

			writer << reply;
		}
			break;
		case RProto::ReadConfig::MSG_CODE:
		{
			WProto::ConfigData reply;
			icsp_init();
			reply.config2 = icsp_readWord(0x00ABFC);
			reply.config1 = icsp_readWord(0x00ABFE);
			icsp_exit();

			writer << reply;
		}
			break;
		case RProto::EraseFlash::MSG_CODE:
		{
			RProto::EraseFlash cmd;
			*input >> cmd;

			icsp_init();
			bool success = icsp_chipErase(cmd.include_config);
			icsp_exit();

			if(success)
				writer << WProto::EraseSuccess();
		}
			break;
		case RProto::ConfigData::MSG_CODE:
		{
			RProto::ConfigData cfg;
			*input >> cfg;

			icsp_init();
			bool success = icsp_writeConfig(cfg.config1, cfg.config2);
			icsp_exit();

			if(success)
				writer << WProto::ConfigSuccess();
		}
			break;
		case RProto::ReadData::MSG_CODE:
		{
			RProto::ReadData cmd;
			*input >> cmd;

			icsp_init();
			icsp_readCode(cmd.addr, g_readBuffer.data, 3*64/2);
			icsp_exit();

			writer << g_readBuffer;
		}
			break;
		case RProto::WriteData::MSG_CODE:
		{
			*input >> g_writeBuffer;

			icsp_init();
			bool success = icsp_writeCode(g_writeBuffer.addr, g_writeBuffer.data);
			icsp_exit();

			if(success)
				writer << WProto::WriteSuccess();
		}
			break;
	}
}

int main()
{
	uart_init();

	PORTA = 0;
	DDRA = (1 << 1) | (1 << 2) | (1 << 3) | (1 << 4);

	mystdout.put = uart_stdio_putc;
	mystdout.get = 0;
	mystdout.flags = _FDEV_SETUP_WRITE;
	mystdout.udata = 0;
	stdout = &mystdout;

	EnvelopeReader input;

	while(1)
	{
		if(input.take(uart_read()) == EnvelopeReader::NEW_MESSAGE)
		{
			handleMsg(&input);
		}
	}
}
