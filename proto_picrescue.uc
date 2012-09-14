// PIC programmer protocol

// Host -> uC

msg Ping
{
};

msg Pong
{
};



msg ReadDeviceID
{
};

msg DeviceIDReply
{
	uint32_t devid;
	uint32_t devrev;
};


msg ReadConfig
{
};

msg ConfigData
{
	uint16_t config1;
	uint16_t config2;
};

msg ConfigSuccess
{
};


msg EraseFlash
{
	uint8_t include_config;
};

msg EraseSuccess
{
};


msg ReadData
{
	uint32_t addr;
};

msg Data
{
	uint16_t data[3*64/2]; // Packed format for 64 instruction words
};


msg WriteData
{
	uint32_t addr;
	uint16_t data[3*64/2];
};

msg WriteSuccess
{
};

