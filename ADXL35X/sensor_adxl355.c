/*********************************************************************************
Copyright(c) 2017 Analog Devices, Inc. All Rights Reserved.

This software is proprietary and confidential.  By using this software you agree
to the terms of the associated Analog Devices License Agreement.
********************************************************************************/
#include "aduc7026.h"
#include "adi_adxl355.h"
#include "sensor_adxl355.h"

ADI_ADXL355_HANDLE  hADXL355Device;

uint8_t sensorID = NULL;
bool_t bMeasureEnabled = false;
bool_t bTempEnabled = false;
bool_t eTimeOut = false;
int rangeG = 2;
int32_t SpXdata[100];
int32_t SpYdata[100];
int32_t SpZdata[100];
int16_t  temperature = 0;
uint8_t  nSamples = 0;
uint8_t  nCurrentSample = 0;
uint16_t packetCount = 0;
uint8_t SpRegAddr[50];
uint8_t SpRegData[50];
uint8_t  nRegs = 0;
uint8_t  nCurrentReg = 0;
uint8_t  status = 0;
int32_t Xself = 0;
int32_t Yself = 0;
int32_t Zself = 0;

struct PACKET packet;               /*!< Data packet */


//ADUC702x target
void System_Init(void)
{
    PLLKEY1 = 0xAA;
    PLLCON = 0x21;
    PLLKEY2 = 0x55;
    
    POWKEY1 = 0x01;
    POWCON = 0x00;	//41.78MHZ
    //POWCON = 0x01;	//20.89MHZ
    POWKEY2 = 0xF4;   
}

//ADXL355 sensor
int ADXL355_config(void)
{
    //uint8_t RegData = 0;
    //unsigned int delay = 0;
    //unsigned int delay1 = 0;

    if (adi_adxl355_Open(0, &hADXL355Device) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
    
    /* select SPI device and chip select */
    if (adi_adxl355_ConfigSPI(hADXL355Device) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
   
    return ADXL355_Reset();
}

int ADXL355_Reset(void)
{
    uint8_t RegData = 0;
    unsigned int delay = 0;
    unsigned int delay1 = 0;

    /* software reset */
    RegData = 0x52;
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_RESET, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    for(delay=0; delay<10; delay++)
    {
        for(delay1=0; delay1<65535; delay1++);
    }

    /* read the device ID register */
    RegData = 0;
    adi_adxl355_RegisterRead(hADXL355Device, REG_READ0_DEVID_AD, &RegData);
    if (RegData == EXPECTED_ADXL355_DEVID_AD)
    {
				sensorID = RegData;
    }
    else
    {
        return RESULT_ERROR;
    }

    //reset self-test, normal mode
    RegData = 0;
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_SELF_TEST, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
		
		for(delay1=0; delay1<65535; delay1++){}

			/* read interrupt map */
    //RegData = 0;
    //adi_adxl355_RegisterRead(hADXL355Device, REG_USER0_INT_MAP, &RegData);
		
    /* config interrupt map */
    RegData = (BITM_USER_INT_MAP_FULL_EN1 | BITM_USER_INT_MAP_OVR_EN1 | BITM_USER_INT_MAP_ACT_EN2);
    //RegData |= (BITM_USER_INT_MAP_FULL_EN1 | BITM_USER_INT_MAP_OVR_EN1 );
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_INT_MAP, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
		
		for(delay1=0; delay1<65535; delay1++){}

		/* interrupt polarity */
    RegData = (BITM_USER_RANGE_I2C_HS | BITM_USER_RANGE_INT_POL | 0x01);
    //RegData = (BITM_USER_RANGE_I2C_HS | 0x01);
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_RANGE, RegData) != ADI_ADXL355_SUCCESS)
    {
       return RESULT_ERROR;
    }

    /* FIFO limit */
    RegData = 48;  /* initial value (factor of 3) */
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_FIFO_SAMPLES, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

		for(delay1=0; delay1<65535; delay1++){}

		/* output data rate and filter */
    RegData = (0 << BITP_USER_FILTER_HPF_CORNER) | (5 << BITP_USER_FILTER_ODR_LPF);  /* 125 Hz initial value */
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_FILTER, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    for(delay=0; delay<5; delay++)
    {
        for(delay1=0; delay1<65535; delay1++);
    }

     //accel settings
    ADXL355_setRange(0);
		
   return RESULT_SUCCESS;
}

int ADXL355_readStatus(void)
{
    uint8_t Status = 0;
    
    /* read status */
    if (adi_adxl355_RegisterRead(hADXL355Device, REG_READ0_STATUS, &Status) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
    status = Status;
    return RESULT_SUCCESS;
}

int ADXL355_readPCon(void)
{
    
    /* read status */
    if (adi_adxl355_Pcon(hADXL355Device) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
    return RESULT_SUCCESS;
}

int ADXL355_readAccel(void)
{
    uint8_t  slSamples;
    uint8_t  Status;
        

    /* check for interrupt flag */
    if (gXL355Int1 == 0)
    {
        /* FIFO not full */
        return RESULT_ERROR;
    }

    /* FIFO XYZ read */
    if (adi_adxl355_FifoXYZRead(hADXL355Device, &slSamples) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
    
    /* clear the interrupt */    
    if (adi_adxl355_RegisterRead(hADXL355Device, REG_READ0_STATUS, &Status) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    status = Status;

    nSamples = slSamples;
    nCurrentSample = 0;
		
		adi_Clear_IRQ(IRQSTA & XIRQ0_BIT);
		
    /* clear flag */
    gXL355Int1 = 0;
		
		//re-enable interrupt
		adi_Enable_IRQ(XIRQ0_BIT);
		
    return RESULT_SUCCESS;
}

int ADXL355_readAccelXYZ(void)
{
    uint8_t  Status;
    int32_t slXdata;
    int32_t slYdata;
    int32_t slZdata;

        
    /* XYZ read */
    if (adi_adxl355_XYZRead(hADXL355Device, slXdata, slYdata, slZdata) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    status = Status;
    nSamples = 1;
    nCurrentSample = 0;

    return RESULT_SUCCESS;
}

int ADXL355_readTemp(void)
{
    int16_t uncalTemp;

    /* read uncalibrated temperature */
    if (adi_adxl355_TemperatureRead(hADXL355Device, &uncalTemp) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

//    degC = 25.0 + (1852 - uncalTemp)/9.05;
//    degF = (degC * 9.0/5.0) + 32.0;
    
    temperature = uncalTemp;
		
    return RESULT_SUCCESS;
}

int ADXL355_setTempEnable(uint8_t enable){    
    if(enable == 1){
        bTempEnabled = (bool_t)true;
    }
    else{
        bTempEnabled = (bool_t)false;
				temperature = 0;
		}
    
    if(adi_adxl355_EnableTemperature (hADXL355Device, enable) != ADI_ADXL355_SUCCESS){
        return RESULT_ERROR;
    }
    return RESULT_SUCCESS;
}

int ADXL355_setRange(int range)
{
    uint8_t RegData;
    
    if ((range < 0) || (range > 2))
    {
        return RESULT_ERROR;
    }
    rangeG = range + 1;

    if (adi_adxl355_RegisterRead(hADXL355Device, REG_USER0_RANGE, &RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
    
    RegData &= ~BITM_USER_RANGE_RANGE;
    RegData |= rangeG;
        
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_RANGE, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    return RESULT_SUCCESS;
}

int ADXL355_setMeasureEnable(int enable)
{
    uint8_t Status;

    extern volatile unsigned char gXL355Int1;
    
    /* measure or standby */
    if (adi_adxl355_EnableMeasure (hADXL355Device, enable) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    /* clear interrupts */    
    if (adi_adxl355_RegisterRead(hADXL355Device, REG_READ0_STATUS, &Status) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
    
    status = Status;
    bMeasureEnabled = (bool_t)enable;
		
    gXL355Int1 = 0;
    gXL355Int2 = 0;
		
		adi_Clear_IRQ(IRQSTA & XIRQ0_BIT & XIRQ1_BIT);
		
    //enable INT1 and INT2 interrupts
    if (enable == 1){
        adi_Enable_IRQ(XIRQ0_BIT|XIRQ1_BIT);
				IRQEN |= TIMER1_BIT;				// Enable Timer1 IRQ
				T1CON |= 0x80;							// Enabled

		}
    else{
        adi_Clear_IRQ(XIRQ0_BIT|XIRQ1_BIT);
				T1CON &= ~0x80;							// Disabled
				IRQEN &= ~TIMER1_BIT;				// Disable Timer1 IRQ
		}

    return RESULT_SUCCESS;
}


int ADXL355_setSelfTest(void)
{
    uint8_t RegData;
    int32_t Xdata, Ydata, Zdata;
    int32_t XpreAvg, YpreAvg, ZpreAvg;
    int32_t XpostAvg, YpostAvg, ZpostAvg;
    int delay;
    const int avgCount = 20;
    int n=0;
    
        
    //self-test mode
    RegData = 1 << BITP_USER_SELF_TEST_ST1;
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_SELF_TEST, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    delay = 800000;
    while(delay > 0)
    {
        delay--;
    }

    //avg offset values (ST2 low)
    XpreAvg = 0;
    YpreAvg = 0;
    ZpreAvg = 0;
    
    for (n=0; n<avgCount; n++)
    {
        adi_adxl355_XYZRead(hADXL355Device, Xdata, Ydata, Zdata);
        XpreAvg += SpXdata[0];
        YpreAvg += SpYdata[0];
        ZpreAvg += SpZdata[0];
    }

    XpreAvg /= avgCount;
    YpreAvg /= avgCount;
    ZpreAvg /= avgCount;

    //self-test force
    RegData |= 1 << BITP_USER_SELF_TEST_ST2;
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_SELF_TEST, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    delay = 800000;
    while(delay > 0)
    {
        delay--;
    }

    //avg offset values (ST2 high)
    XpostAvg = 0;
    YpostAvg = 0;
    ZpostAvg = 0;

    for (n=0; n<avgCount; n++)
    {
        adi_adxl355_XYZRead(hADXL355Device, Xdata, Ydata, Zdata);
        XpostAvg += SpXdata[0];
        YpostAvg += SpYdata[0];
        ZpostAvg += SpZdata[0];
    }

    XpostAvg /= avgCount;
    YpostAvg /= avgCount;
    ZpostAvg /= avgCount;

    //reset self-test
    RegData = 0;
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_SELF_TEST, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    Xself = XpostAvg-XpreAvg;
    Yself = YpostAvg-YpreAvg;
    Zself = ZpostAvg-ZpreAvg;
    
    return RESULT_SUCCESS;
}


/*
ActivityEnable: Bit mask which indicates which of the XYZ axis are enabled
                for activity detection. When the corresponding bit is set it 
                indicates that the corresponding axis is enabled for the activity
                detection.
                Bit 0 - X axis
                Bit 1 - Y axis
                Bit 2 - Z axis

Threshold    : Threshold for activity detection on XYZ axis.
*/
int ADXL355_ConfigActivityDetection(uint8_t ActivityEnable, uint16_t Threshold, uint8_t Count)
{
    uint8_t RegData;
    
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_ACT_EN, ActivityEnable) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    RegData = (Threshold & 0x00FF);
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_ACT_THRESH_H, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    RegData = (Threshold >> 8);
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_ACT_THRESH_L, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }

    RegData = Count;
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_ACT_COUNT, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
    
  return RESULT_SUCCESS;
}

/*
 * Offset added to the axis after all other signal processing. 
*/
int ADXL355_OffsetAdjust(uint8_t *pOffsets)
{
    uint8_t Reg;
    uint8_t RegData;
    uint8_t *pData = pOffsets;
    int n=0;
    
    //base offset register 
    Reg = REG_USER0_OFFSET_X_H;
    
    for (n=0; n<6; n++)
    {
        RegData = *pData++;
        if (adi_adxl355_RegisterWrite(hADXL355Device, Reg, RegData) != ADI_ADXL355_SUCCESS)
        {
            return RESULT_ERROR;
        }
        Reg++;
    }
    
    return RESULT_SUCCESS;
}

/*
 * Specify parameters for internal high-pass and low-pass filters. 
 */
int ADXL355_ConfigFilter(uint8_t hpf_corner, uint8_t odr_lpf)
{
    uint8_t RegData;
    
    RegData = (hpf_corner << BITP_USER_FILTER_HPF_CORNER) & BITM_USER_FILTER_HPF_CORNER;
    RegData |= (odr_lpf << BITP_USER_FILTER_ODR_LPF) & BITM_USER_FILTER_ODR_LPF;
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_FILTER, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
    
    return RESULT_SUCCESS;
}

/*
 * Specify the number of samples to store in the FIFO. 
 */
int ADXL355_SetFifoSamples(uint8_t samples)
{
    uint8_t RegData;
    
    RegData = samples;
    if (adi_adxl355_RegisterWrite(hADXL355Device, REG_USER0_FIFO_SAMPLES, RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
    
    return RESULT_SUCCESS;
}

/*
 * Set a register value
 */
int ADXL355_setRegister(uint8_t address, uint8_t value)
{ 
    if (adi_adxl355_RegisterWrite(hADXL355Device, address, value) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
    
    return RESULT_SUCCESS;
}

/*
 * Get a register value
 */
int ADXL355_getRegister(uint8_t address)
{ 
    uint8_t RegData;
    
    if (adi_adxl355_RegisterRead(hADXL355Device, address, &RegData) != ADI_ADXL355_SUCCESS)
    {
        return RESULT_ERROR;
    }
    
    nRegs = 1;
    nCurrentReg = 0;
    SpRegAddr[0] = address;
		SpRegData[0] = RegData;
    return RESULT_SUCCESS;
}

/*
 * Get all register values
 */
int ADXL355_getAllRegisters(void)
{ 
    uint8_t RegData;
    uint8_t address;
    int n;

    nRegs = 0;
    nCurrentReg = 0;

    address = REG_READ0_DEVID_AD;
    for (n=0; n <= REG_USER0_RESET; n++)
    {
        //skip registers 0x12 -> 0x1D
        if (((address+n) <= REG_FIFO0_FIFO_DATA) || ((address+n) >= REG_USER0_OFFSET_X_H))
        {
            if (adi_adxl355_RegisterRead(hADXL355Device, address+n, &RegData) != ADI_ADXL355_SUCCESS)
            {
                return RESULT_ERROR;
            }
            SpRegAddr[n] = address+n;
            SpRegData[n] = RegData;
            nRegs++;
        }
    }
        
    return RESULT_SUCCESS;
}


void ADXL355_formatPacketStatus(struct PACKET *pPacket)
{
    uint8_t *pData = pPacket->payload;

    /* the status reg is read when getting accel data */
    *pData++ = status;   
}

void ADXL355_formatPacketTemp(struct PACKET *pPacket)
{
    uint8_t *pData = pPacket->payload;

    /* temperature data */
    *pData++ = (temperature >> 8) & 0x00FF;
    *pData++ = (temperature >> 0) & 0x00FF;    
}

void ADXL355_formatPacketReg(struct PACKET *pPacket)
{
    uint8_t count;
    uint8_t *pData = pPacket->payload;
    int n=0;
    
    if ((nRegs-nCurrentReg) < MAX_PAYLOAD_REGS)
    {
        count = (nRegs-nCurrentReg);
        *pData++ = count;
    }
    else
    {
        count = MAX_PAYLOAD_REGS;
        *pData++ = count;
    }

    //max registers in a packet
    for (n=0; n < count; n++)
    {
        *pData++ = SpRegAddr[nCurrentReg];
        *pData++ = SpRegData[nCurrentReg];

        nCurrentReg++;
    }   
}

void ADXL355_formatPacketSelfTestData(struct PACKET *pPacket)
{
    int32_t value;
    uint8_t *pData = pPacket->payload;

    // XYZ is 20-bits
    value = Xself;
    *pData++ = (value >> 16) & 0x000000FF;
    *pData++ = (value >> 8)  & 0x000000FF;
    *pData++ = (value >> 0)  & 0x000000FF;
    
    value = Yself;
    *pData++ = (value >> 16) & 0x000000FF;
    *pData++ = (value >> 8)  & 0x000000FF;
    *pData++ = (value >> 0)  & 0x000000FF;

    value = Zself;
    *pData++ = (value >> 16) & 0x000000FF;
    *pData++ = (value >> 8)  & 0x000000FF;
    *pData++ = (value >> 0)  & 0x000000FF;   
}

void ADXL355_Format_Serial_Registers(struct PACKET *pPacket)
{
    char buffer[64];
		sprintf(buffer, "%2x%2x\r\n", SpRegAddr[0], SpRegData[0]);

		nSize = 0;
		output[0] = '!';
    output[1] = '-';
    output[2] = 'R';
	
		if (status > 0x08)
		{
				if(status == 0x09)
						output[3] = 0x39;
				else if(status == 0x0A)
						output[3] = 'A';
				else if(status == 0x0A)
						output[3] = 'A';
				else if(status == 0x0B)
						output[3] = 'B';
				else if(status == 0x0C)
						output[3] = 'C';
				else if(status == 0x0D)
						output[3] = 'D';
				else if(status == 0x0E)
						output[3] = 'E';
				else if(status == 0x0F)
						output[3] = 'F';
				else if(status == 0x10)
						output[3] = 'U';
				else
						output[3] = 'U';
		}
		else
				output[3] = status + 0x30;
		
    output[4] = ADI_XMIT_CMD_REG_DATA;
		output[5] = 0x30;
		output[6] = buffer[0];
		output[7] = buffer[1];
		output[8] = buffer[2];
		output[9] = buffer[3];
    output[10] = '\r';
    output[11] = '\n';
		nSize = 12;
}

void ADXL355_Format_Serial_AllRegisters(struct PACKET *pPacket)
{
    char buffer[64];
		sprintf(buffer, "%2x%2x%2x\r\n", nCurrentReg, SpRegAddr[nCurrentReg], SpRegData[nCurrentReg]);

    nSize = 0;
		output[0] = '!';
    output[1] = 'A';
    output[2] = 'R';
	

		if (status > 0x08)
		{
				if(status == 0x09)
						output[3] = 0x39;
				else if(status == 0x0A)
						output[3] = 'A';
				else if(status == 0x0A)
						output[3] = 'A';
				else if(status == 0x0B)
						output[3] = 'B';
				else if(status == 0x0C)
						output[3] = 'C';
				else if(status == 0x0D)
						output[3] = 'D';
				else if(status == 0x0E)
						output[3] = 'E';
				else if(status == 0x0F)
						output[3] = 'F';
				else if(status == 0x10)
						output[3] = 'U';
				else
						output[3] = 'U';
		}
		else
				output[3] = status + 0x30;
		
    output[4] = ADI_XMIT_CMD_REG_DATA;
		output[5] = buffer[0];
		output[6] = buffer[1];
		output[7] = buffer[2];
		output[8] = buffer[3];
		output[9] = buffer[4];
		output[10] = buffer[5];
    output[11] = '\r';
    output[12] = '\n';
		nSize = 13;
}

void ADXL355_Format_Serial_SelfTestData(struct PACKET *pPacket)
{
    int32_t Xdata, Ydata, Zdata;

    char *pOut = (char*)output;
    char buffer[64];
    int len=0;
    
    double factor, x, y, z, degC;

    static uint16_t count = 0, cmd_id = 0;
    
    nSize = 0;
    
		/* XYZ is 20-bits */
		Xdata = Xself;
		Ydata = Yself;
		Zdata = Zself;

		//sign extend 20-bit data to 32-bits
    Xdata <<= 12;
    Xdata >>= 12;

    Ydata <<= 12;
    Ydata >>= 12;

    Zdata <<= 12;
    Zdata >>= 12;
    
    // +/- 2g sensitivity factor 3.9 ug/LSB
    // +/- 4g sensitivity factor 7.8 ug/LSB
    // +/- 8g sensitivity factor 15.6 ug/LSB
    factor = rangeG * 3.9;

		if (rangeG==0x03)
				factor = 15.6;

    //convert data based on range setting (see datasheet)
    x = (Xdata * factor) / 1000000.0;
    y = (Ydata * factor) / 1000000.0;
    z = (Zdata * factor) / 1000000.0;


    degC = -999.99;

    sprintf(buffer, "%6d, %6d, %10.7f, %10.7f, %10.7f, %7.2f,0\r\n", cmd_id, count, x, y, z, degC);
		
		buffer[0] = '!';
    buffer[1] = 'S';
    buffer[2] = 'T';
	
		if (status > 0x08)
		{
				if(status == 0x09)
						buffer[3] = 0x39;
				else if(status == 0x0A)
						buffer[3] = 'A';
				else if(status == 0x0A)
						buffer[3] = 'A';
				else if(status == 0x0B)
						buffer[3] = 'B';
				else if(status == 0x0C)
						buffer[3] = 'C';
				else if(status == 0x0D)
						buffer[3] = 'D';
				else if(status == 0x0E)
						buffer[3] = 'E';
				else if(status == 0x0F)
						buffer[3] = 'F';
				else if(status == 0x10)
						buffer[3] = 'U';
				else
						buffer[3] = 'U';
		}
		else
				buffer[3] = status + 0x30;

    buffer[4] = ADI_XMIT_CMD_SELF_TEST_DATA;
		
    //len = strlen(buffer);    
    //strncpy(pOut, (const char *)buffer, len);     
    //pOut += len;

		while(buffer[len]!= '\n'){
			*pOut++ = buffer[len];
			len++;
		}

    nSize += len;
}

void ADXL355_Format_Serial_Data(struct PACKET *pPacket)
{
    int32_t Xdata, Ydata, Zdata;
    char *pOut = (char*)output;
    char buffer[64];
    int len=0;
    
    double factor, x, y, z, degC;

    static uint16_t count = 0, cmd_id = 0;
    
    nSize = 0;

		Xdata = SpXdata[nCurrentSample];
    Ydata = SpYdata[nCurrentSample];
    Zdata = SpZdata[nCurrentSample];

    // +/- 2g sensitivity factor 3.9 ug/LSB
    // +/- 4g sensitivity factor 7.8 ug/LSB
    // +/- 8g sensitivity factor 15.6 ug/LSB
    factor = rangeG * 3.9;
	
		if (rangeG==0x03)
				factor = 15.6;

    //convert data based on range setting (see datasheet)
    x = (Xdata * factor) / 1000000.0;
    y = (Ydata * factor) / 1000000.0;
    z = (Zdata * factor) / 1000000.0;

    if (temperature > 0)
    {
				degC = 25.0 + (1852 - temperature)/9.05;
    }
    else
    {
				degC = -999.99;
    }

    sprintf(buffer, "%6d, %6d, %10.7f, %10.7f, %10.7f, %7.2f,0\r\n", cmd_id, count, x, y, z, degC);
		
		buffer[0] = '!';
    buffer[1] = 'S';
    buffer[2] = 'D';
	
		if (status > 0x08)
		{
				if(status == 0x09)
						buffer[3] = 0x39;
				else if(status == 0x0A)
						buffer[3] = 'A';
				else if(status == 0x0A)
						buffer[3] = 'A';
				else if(status == 0x0B)
						buffer[3] = 'B';
				else if(status == 0x0C)
						buffer[3] = 'C';
				else if(status == 0x0D)
						buffer[3] = 'D';
				else if(status == 0x0E)
						buffer[3] = 'E';
				else if(status == 0x0F)
						buffer[3] = 'F';
				else if(status == 0x10)
						buffer[3] = 'U';
				else
						buffer[3] = 'U';
		}
		else
				buffer[3] = status + 0x30;

    buffer[4] = ADI_XMIT_CMD_DATA;
		
    //len = strlen(buffer);    
    //strncpy(pOut, (const char *)buffer, len);     
    //pOut += len;

		while(buffer[len]!= '\n'){
			*pOut++ = buffer[len];
			len++;
		}

    nSize += len;
}

void ADXL355_Format_Serial_Temperature(struct PACKET *pPacket)
{
    char *pOut = (char*)output;
    char buffer[64];
    int len=0;
    
    double x, y, z, degC;

    static uint16_t count = 0, cmd_id = 0;
	
    x=y=z=0;
    nSize = 0;
    if (temperature > 0)
    {
				degC = 25.0 + (1852 - temperature)/9.05;
    }
    else
    {
				degC = -999.99;
    }

    sprintf(buffer, "%6d, %6d, %10.7f, %10.7f, %10.7f, %7.2f,0\r\n", cmd_id, count, x, y, z, degC);
		
		buffer[0] = '!';
    buffer[1] = 'T';
    buffer[2] = 'D';
		
	
		if (status > 0x08)
		{
				if(status == 0x09)
						buffer[3] = 0x39;
				else if(status == 0x0A)
						buffer[3] = 'A';
				else if(status == 0x0A)
						buffer[3] = 'A';
				else if(status == 0x0B)
						buffer[3] = 'B';
				else if(status == 0x0C)
						buffer[3] = 'C';
				else if(status == 0x0D)
						buffer[3] = 'D';
				else if(status == 0x0E)
						buffer[3] = 'E';
				else if(status == 0x0F)
						buffer[3] = 'F';
				else if(status == 0x10)
						buffer[3] = 'U';
				else
						buffer[3] = 'U';
		}
		else
				buffer[3] = status + 0x30;
		
    buffer[4] = ADI_XMIT_CMD_TEMP_DATA;
		
		while(buffer[len]!= '\n'){
			*pOut++ = buffer[len];
			len++;
		}
		
    //len = strlen(buffer);    
    //strncpy(pOut, (const char *)buffer, len);     
    //pOut += len;
    nSize += len;
}

