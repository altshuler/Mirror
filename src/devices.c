/**
* @file devices.c
* @brief static devices definition
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 12.06.2014
*/

#include <dev.h>
#include <iodev.h>
#include <uart.h>
#include <spi.h>
#include <i2c.h>

extern const IODEV_Fxns Usart_IODEVFXNS;
//extern const IODEV_Fxns Spi_IODEVFXNS;
extern Uart_Params ctlDevParams[];
extern Uart_Params readoutDevParams;
//extern Spi_Params spiParams0;
extern i2c_Params i2cParams0;

/* /Uart2 */
extern void initCtl1DevParams(void);

/* /Uart3 */
extern void initCtl2DevParams(void);

/* /Uart6 */
extern void initReadoutDevParams(void);


/* /Spi0 */
//extern void SpiUserInit0(void);

/* /i2c1 */
extern void i2cUserInit0(void);



const DEV_Tinit _DEV_initFxn[]=
{
	initCtl1DevParams, /* /Uart2 */
	initCtl2DevParams, /* /Uart3 */
	initReadoutDevParams, /* /Uart6 */
	i2cUserInit0		/* /i2c0 */
};


const int _DEV_numStaticDevs = 4;
DEV_TableElem _DEV_staticDevTable[] = 
{
	/* /Uart1 */
	{
		{
			NULL,
			NULL
		},
		{
				"/Uart1",		   /* device name */
				(void *)&Usart_IODEVFXNS,			/* device function table */
				1,			/* device id */
				&ctlDevParams[0],		 /* device parameters */
				//,		   /* type of the device */
				NULL			 /* pointer to device global data */
		}

	},
	/* /Uart2 */
	{
		{
			NULL,
			NULL
		},	
		{
				"/Uart2",		   /* device name */
				(void *)&Usart_IODEVFXNS,			/* device function table */
				2,			/* device id */
				&ctlDevParams[1],		 /* device parameters */
				//,		   /* type of the device */
				NULL			 /* pointer to device global data */
		}
	
	},
	/* /Uart5 */
	{
		{
			NULL,
			NULL
		},	
		{
				"/Uart5",		   /* device name */
				(void *)&Usart_IODEVFXNS,			/* device function table */
				5,			/* device id */
				&readoutDevParams,		 /* device parameters */
				//, 	   /* type of the device */
				NULL			 /* pointer to device global data */
		}
	
	},
	/* /i2c1 */
		{
			{
				NULL,
				NULL
			},
			{
					"/i2c1",		   /* device name */
					(void *)&i2c_IODEVFXNS,			/* device function table */
					0,			/* device id */
					&i2cParams0, 	 /* device parameters */
					//, 	   /* type of the device */
					NULL			 /* pointer to device global data */
			}

		}	
};
	

