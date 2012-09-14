// PIC programmer host application
// Author: Max Schwarz <max@x-quadraht.de>

#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdint.h>

#include <libucomm/checksum.h>
#include <libucomm/envelope.h>
#include <libucomm/io.h>

#include "proto_picrescue.h"

typedef uc::EnvelopeWriter< uc::InvertedModSumGenerator > EnvelopeWriter;
typedef uc::IO< EnvelopeWriter, uc::IO_W > SimpleWriter;
typedef Proto< SimpleWriter > WProto;

typedef uc::EnvelopeReader< uc::InvertedModSumGenerator, 1024 > EnvelopeReader;
typedef uc::IO< EnvelopeReader, uc::IO_R > SimpleReader;
typedef Proto< SimpleReader > RProto;

class Serial : public uc::CharWriter
{
public:
	Serial()
	 : m_fd(-1)
	{
	}

	bool open(const char* path)
	{
		m_fd = ::open(path, O_RDWR);
		if(m_fd < 0)
		{
			perror("Could not open serial port");
			return false;
		}

		termios ios;
		if(tcgetattr(m_fd, &ios) != 0)
		{
			perror("Could not get serial attributes");
			return false;
		}

		cfmakeraw(&ios);
		cfsetspeed(&ios, B115200);

		if(tcsetattr(m_fd, TCSANOW, &ios) != 0)
		{
			perror("Could not set serial attributes");
			return false;
		}

		return true;
	}

	virtual bool writeChar(uint8_t c)
	{
		if(write(m_fd, &c, 1) != 1)
		{
			perror("Could not write");
			exit(1);
		}
	}

	bool readData(EnvelopeReader* reader);
private:
	int m_fd;
};


bool Serial::readData(EnvelopeReader* reader)
{
	for(int tries = 0; tries < 10;)
	{
		timeval tv;
		tv.tv_sec = 1;
		tv.tv_usec = 0;
		
		fd_set fds;
		FD_ZERO(&fds);
		FD_SET(m_fd, &fds);
		
		int r = select(m_fd+1, &fds, 0, 0, &tv);
		if(r < 0)
		{
			perror("select()");
			exit(1);
		}
		
		if(r == 0)
		{
			tries++;
			continue;
		}
		
		uint8_t c;
		if(read(m_fd, &c, 1) != 1)
		{
			perror("read()");
			exit(1);
		}
		
		if(reader->take(c) == EnvelopeReader::NEW_MESSAGE)
			return true;
	}
	
	return false; // Timeout
}

bool isNothing(uint32_t* b)
{
	for(uint8_t i = 0; i < 64; ++i)
	{
		if(b[i] != 0x00FFFFFF)
			return false;
	}

	return true;
}


int main()
{
	Serial serial;
	EnvelopeWriter writer(&serial);
	EnvelopeReader reader;

	if(!serial.open("/dev/ttyS0"))
		return 1;

	printf("Detecting programmer...\n");

	writer << WProto::Ping();

	if(!serial.readData(&reader) || reader.msgCode() != RProto::Pong::MSG_CODE)
	{
		fprintf(stderr, "No programmer detected.\n");
		return 1;
	}

	printf("Programmer detected.\n");

	printf("Reading device ID...\n");

	writer << WProto::ReadDeviceID();

	if(!serial.readData(&reader) || reader.msgCode() != RProto::DeviceIDReply::MSG_CODE)
	{
		fprintf(stderr, "No answer\n");
		return 1;
	}

	RProto::DeviceIDReply id;
	reader >> id;
	printf("Device ID: 0x%08X\n", id.devid);
	printf("Revision: 0x%08X\n", id.devrev);

	if(id.devid != 0x447)
	{
		fprintf(stderr, "Unknown PIC. Terminating.\n");
		exit(1);
	}

	printf("Reading config words...\n");

	writer << WProto::ReadConfig();

	if(!serial.readData(&reader) || reader.msgCode() != RProto::ConfigData::MSG_CODE)
	{
		fprintf(stderr, "No answer\n");
		return 1;
	}

	RProto::ConfigData config;
	reader >> config;

	printf("Config: CW1: 0x%04X, CW2: 0x%04X\n", config.config1, config.config2);

	printf("Performing chip erase...\n");
	WProto::EraseFlash erase;
	erase.include_config = true;

	writer << erase;
	if(!serial.readData(&reader) || reader.msgCode() != RProto::EraseSuccess::MSG_CODE)
	{
		fprintf(stderr, "No answer\n");
		return 1;
	}

	printf("done\n");

	printf("Writing new config values...\n");
	WProto::ConfigData cfg;
	cfg.config1 = 0x3F7F;
	cfg.config2 = 0xF9DF;

	writer << cfg;
	if(!serial.readData(&reader) || reader.msgCode() != RProto::ConfigSuccess::MSG_CODE)
	{
		fprintf(stderr, "No answer\n");
		return 1;
	}

// 	WProto::WriteData d;
// 	d.addr = 0xABFC;
// 	d.data[0] = 0xF9DF;
// 	d.data[1] = 0xFFFF;
// 	d.data[2] = 0x3F7F;
// 
// 	writer << d;
// 	if(!serial.readData(&reader) || reader.msgCode() != RProto::WriteSuccess::MSG_CODE)
// 	{
// 		fprintf(stderr, "No answer\n");
// 		return 1;
// 	}

	printf("Success\n");

	printf("Reading config words...\n");
	
	writer << WProto::ReadConfig();
	
	if(!serial.readData(&reader) || reader.msgCode() != RProto::ConfigData::MSG_CODE)
	{
		fprintf(stderr, "No answer\n");
		return 1;
	}
	
	reader >> config;
	
	printf("Config: CW1: 0x%04X, CW2: 0x%04X\n", config.config1, config.config2);

// 	return 0;

	printf("Reading flash at addr 0...\n");

	WProto::ReadData readData;
	readData.addr = 0;

	writer << readData;
	if(!serial.readData(&reader) || reader.msgCode() != RProto::Data::MSG_CODE)
	{
		fprintf(stderr, "No answer\n");
		return 1;
	}

	RProto::Data data;
	reader >> data;

	for(uint8_t addr = 0; addr < 64; addr += 2)
	{
		uint8_t base = 3*addr/2;

		uint32_t value1 = (((uint32_t)data.data[base+1] & 0x00FF) << 16) | data.data[base+0];
		uint32_t value2 = (((uint32_t)data.data[base+1] & 0xFF00) <<  8) | data.data[base+2];

		printf("ADDR: 0x%06X; DATA: 0x%06X\n", 0 + addr, value1);
		printf("ADDR: 0x%06X; DATA: 0x%06X\n", 1 + addr, value2);
	}

// 	return 0;
	printf("Performing chip erase...\n");
// 	WProto::EraseFlash erase;
	erase.include_config = 0;

	writer << erase;
	if(!serial.readData(&reader) || reader.msgCode() != RProto::EraseSuccess::MSG_CODE)
	{
		fprintf(stderr, "No answer\n");
		return 1;
	}

	int binfd = open("/home/max/temp/flash.bin", O_RDONLY);
	uint32_t* bindata = (uint32_t*)malloc(0x20000);
	size_t size = read(binfd, bindata, 0x20000) / 4;

	printf("Writing %u words of flash\n", size);

	for(uint32_t b = 0; b < size; b += 64)
	{
		printf("Address 0x%06X (0x%06X)\n", b, 2*b);

		if(isNothing(bindata + b))
		{
			printf("skip\n");
			continue;
		}

		WProto::WriteData wr;
		wr.addr = 2*b;

		for(uint8_t addr = 0; addr < 64; addr += 2)
		{
			uint8_t base = 3*addr/2;

			uint32_t value1 = bindata[b + addr + 0];
			uint32_t value2 = bindata[b + addr + 1];
			/*uint32_t value1 = ((uint32_t)addr << 16) | ((uint32_t)addr << 8) | addr;
			uint32_t value2 = ((uint32_t)(addr+1) << 16) | ((uint32_t)(addr+1) << 8) | (addr+1);*/

			wr.data[base+0] = value1 & 0xFFFF;
			wr.data[base+1] = ((value2 & 0xFF0000) >> 8) | ((value1 & 0xFF0000) >> 16);
			wr.data[base+2] = value2 & 0xFFFF;
		}

		writer << wr;
		if(!serial.readData(&reader) || reader.msgCode() != RProto::WriteSuccess::MSG_CODE)
		{
			fprintf(stderr, "No answer\n");
			return 1;
		}

		printf("Success\n");
// 		return 0;
	}

	return 0;
}
