/**
* @file board_i2c.c
* @brief board level i2c
*
* @author Eli Schneider
* @author David Anidjar
*
* @version 0.0.1
* @date 12.07.2014
*/

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <iodev.h>
#include <sysport.h>
#include <freertos.h>
#include <task.h>
#include <semphr.h>
#include <gio.h>
#include <i2c.h>
#include <board.h>
#include <board_i2c.h>
#include <irqhndl.h>
//#include <timebase.h>
#include <dma.h>
#include <membuf.h>
#include "sem.h"

//#define USE_POLLED_I2C0
//#define USE_DMA
#define USE_DMA_I2C2

#ifdef I2C_ASYNC_ENABLE
#define N_I2C_ASYNC_PACKETS ((MAX_I2C_BUSES)*20)
#endif

/* handle to the input and output streams                 */
GIO_Handle  i2cHandle[MAX_I2C_BUSES]  = {NULL};
xSemaphoreHandle i2cLock[MAX_I2C_BUSES] = {NULL};
#ifdef I2C_ASYNC_ENABLE
xSemaphoreHandle	i2cAsyncSignalSem[MAX_I2C_BUSES] = {NULL};
MEMBUF_POOL i2cAsyncMembufPool;
#endif



/* Global I2C init config data structure */
i2c_Params       i2cParams0;
#if 1<MAX_I2C_BUSES
i2c_Params       i2cParams1;
#if 2<MAX_SPI_BUSES
i2c_Params       i2cParams2;
#endif
#endif


#define I2C1_TX_DMAC_ID 0
#define I2C1_TX_STREAM_ID 7
#define I2C1_TX_CHANNEL_ID 1

#define I2C1_RX_DMAC_ID 0
#define I2C1_RX_STREAM_ID 0
#define I2C1_RX_CHANNEL_ID 1

const struct sDmaResSet i2cDma[MAX_I2C_BUSES]=
{
	{
		{
			{
				I2C1_TX_DMAC_ID,
				I2C1_TX_STREAM_ID,
				I2C1_TX_CHANNEL_ID
			},
			{
				I2C1_RX_DMAC_ID,
				I2C1_RX_STREAM_ID,
				I2C1_RX_CHANNEL_ID
			}
		}
	}
};


#ifdef I2C_ASYNC_ENABLE
int i2cMultibufXferCallback(void * arg, int status , void *bufp, size_t size);

static const GIO_AppCallback gioi2cMultibufXferNullCallback = {i2cMultibufXferCallback, NULL};
#endif


void i2cUserInit0()
{
    sti2cInit();
    i2c_initParams0();
}

void i2c_initParams0(void)
{
    i2cParams0 = i2c_PARAMS;

	#if defined(USE_DMA) && defined (USE_DMA_I2C2)
    i2cParams0.opMode = i2c_OpMode_INTERRUPTDMA;
	#elif defined(USE_POLLED_I2C0)
    i2cParams0.opMode = i2c_OpMode_POLLED;
	#else
    i2cParams0.opMode = i2c_OpMode_INTERRUPT;
	#endif

    i2cParams0.outputClkFreq     = 3400000/*30000000*/;
    #if defined(USE_DMA) && defined (USE_DMA_I2C2)
    i2cParams0.hDma        = (void *)&i2cDma[0];
	#else
    i2cParams0.hDma        = NULL;
	#endif

	i2cParams0.i2cHWCfgData.masterOrSlave=i2c_CommMode_MASTER;
	i2cParams0.i2cHWCfgData.i2c_Ack =1;
	i2cParams0.i2cHWCfgData.i2c_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2cParams0.i2cHWCfgData.i2c_ClockSpeed = 340000;
	i2cParams0.i2cHWCfgData.i2c_DutyCycle = 1;
	i2cParams0.i2cHWCfgData.i2c_Mode = I2C_Mode_I2C;
	i2cParams0.i2cHWCfgData.i2c_OwnAddress = 0;
}

struct i2c_slave *i2c_setup_slave(unsigned int bus, unsigned int cs,uint32_t max_hz, unsigned int mode)
{
	struct board_i2c_slave	*ds;

	if (MAX_I2C_BUSES<=bus)
		return NULL;

	ds = (struct board_i2c_slave *)pvPortMalloc(sizeof(struct board_i2c_slave));
	if (!ds)
		return NULL;

	ds->slave.bus = bus;
	ds->slave.cs = cs;
	ds->slave.freq = max_hz;
	ds->handle = i2cHandle[bus];
	ds->freq = max_hz;

	return &ds->slave;
}

void i2c_free_slave(struct i2c_slave *slave)
{
	struct board_i2c_slave *ds = to_board_i2c(slave);

	vPortFree(ds);
}


int i2c_claim_bus(struct i2c_slave *slave)
{
	struct board_i2c_slave *ds = to_board_i2c(slave);

	// obtain bus lock
	if (i2cLock[ds->slave.bus])
	{
		if (xSemaphoreTake(i2cLock[ds->slave.bus], portMAX_DELAY))
		{
			// set bus frequency
			GIO_control(ds->handle, i2c_IOCTL_SET_CLOCK_RATE, &slave->freq);
			return 0;
		}
		else
			return -1;
	}
	else
		return -1;
}


void i2c_release_bus(struct i2c_slave *slave)
{
	// restore bus frequency to default frequency

	// return bus lock
	if (i2cLock[slave->bus])
		xSemaphoreGive(i2cLock[slave->bus]);
}


// Data format 0
int i2c_xfer(struct i2c_slave *slave, unsigned int bitlen, 	const void *dout, void *din, unsigned int flags)
{
	struct board_i2c_slave *ds = to_board_i2c(slave);
	size_t	len;
    i2c_DataParam       dataparam;

	if (bitlen == 0)
		/* Finish any previously submitted transfers */
		goto out;

	/*
	 * It's not clear how non-8-bit-aligned transfers are supposed to be
	 * represented as a stream of bytes...this is a limitation of
	 * the current SPI interface - here we terminate on receiving such a
	 * transfer request.
	 */
	if (bitlen % 8)
	{
		/* Errors always terminate an ongoing transfer */
		flags |= I2C_XFER_END;
		goto out;
	}

	len = bitlen / 8;

    dataparam.inBufLen       = len;
    dataparam.inBuffer     = (uint8_t *)din;
    dataparam.outBuffer    = (uint8_t *)dout;
    // Flags Read or Write Operation
    dataparam.flags        = flags;

    GIO_write(ds->handle, &dataparam, &len);

	return 0;

out:
	if (flags & I2C_XFER_END)
	{
	}

	return 0;
}

#ifdef SPI_ASYNC_ENABLE
int i2c_xfer_async(struct i2c_slave *slave, unsigned int bitlen, const void *dout, void *din, unsigned int flags, void *appCallback)
{
	struct board_i2c_slave *ds = to_board_i2c(slave);
	size_t	len;
	i2c_DataParam		*dataparam;

	if (bitlen == 0)
		/* Finish any previously submitted transfers */
		goto out;


	/*
	 * It's not clear how non-8-bit-aligned transfers are supposed to be
	 * represented as a stream of bytes...this is a limitation of
	 * the current SPI interface - here we terminate on receiving such a
	 * transfer request.
	 */
	if (bitlen % 8) {
		/* Errors always terminate an ongoing transfer */
		flags |= I2C_XFER_END;
		goto out;
	}

	len = bitlen / 8;

	dataparam=getMemBuf(&i2cAsyncMembufPool);

	if (dataparam==NULL)
	{
		goto out;
	}

	dataparam->bufLen	   = len;
	dataparam->inBuffer	   = (uint8_t *)din;
	dataparam->outBuffer    = (uint8_t *)dout;
    dataparam->flags        = ((flags&SPI_CS_HOLD)==0) ? 0 : (flags&SPI_XFER_END) ? Spi_CSHOLD : (Spi_CSHOLD|Spi_CSHOLD_FOR_MULTI_TRANSCEIVE);
	dataparam->chipSelect   = ds->slave.cs;
	dataparam->dataFormat   = 0;


	return GIO_submit(ds->handle,IODEV_WRITE,dataparam,&len,(GIO_AppCallback *)appCallback);

out:
	if (flags & I2C_XFER_END)
	{
	}

	return 0;
}



void i2c_init_async_io()
{
	void *p;

	p=pvPortMalloc((sizeof(i2c_DataParam)+MEM_BUF_HEADER_SIZE)*N_I2C_ASYNC_PACKETS);
	initMemBufPool(&i2cAsyncMembufPool,p,(sizeof(i2c_DataParam)+MEM_BUF_HEADER_SIZE)*N_I2C_ASYNC_PACKETS,sizeof(i2c_DataParam),N_I2C_ASYNC_PACKETS);
}

int i2c_xfer_async_complete(void *iopacket, void **dout, void **din)
{
	//i2c_DataParam *dataparam=((i2c_DataParam *)iopacket);

	if (iopacket==NULL)
	{
		if (dout)
			*dout=NULL;
		if (din)
			*din=NULL;
	}
	else
	{
		if (dout)
			*dout=((i2c_DataParam *)iopacket)->outBuffer;
		if (din)
			*din=((i2c_DataParam *)iopacket)->inBuffer;
		retMemBuf(iopacket);
	}
	return 0;
}

// Data format 0
int i2c_xfer_seg_multibuf_async(struct i2c_slave *slave, unsigned int *bitlen,
		const void **dout, void **din, unsigned int *flags, unsigned int nbuffers)
{
	GIO_AppCallback gioAppCallback;

	unsigned int		nbuf;

	gioAppCallback.fxn=i2cMultibufXferCallback;
	gioAppCallback.arg=i2cAsyncSignalSem[slave->bus];

	// push transfer commands
	for (nbuf=0;nbuf<nbuffers;nbuf++)
	{
		if ((nbuf+1)<nbuffers)
			i2c_xfer_async(slave,bitlen[nbuf],dout[nbuf],din[nbuf],flags[nbuf],(void *)&gioi2cMultibufXferNullCallback);
		else
			i2c_xfer_async(slave,bitlen[nbuf],dout[nbuf],din[nbuf],flags[nbuf],(void *)&gioAppCallback);
	}

	SEM_pend(i2cAsyncSignalSem[slave->bus], portMAX_DELAY);

	return 0;
}

int i2cMultibufXferCallback(void * arg, int status , void *bufp, size_t size)
{

	i2c_xfer_async_complete(bufp,NULL,NULL);

	if (arg)
		return SEM_post(arg);
	else
		return 0;
}
#endif


