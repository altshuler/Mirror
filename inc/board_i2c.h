/**
* @file board_i2c.h
* @brief board level i2c
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 12.06.2014
*/
#ifndef _BOARD_I2C_H
#define _BOARD_I2C_H

#include <stddef.h>
#include <stdint.h>
#include <freertos.h>
#include <semphr.h>
#include <gio.h>

#ifdef __cplusplus
extern "C" {
#endif

/*  I2C buses allocation */
#define MAX_I2C_BUSES 1

/* I2C transfer flags */
#define I2C_XFER_BEGIN  	0x01		/**< Assert CS before transfer */
#define I2C_XFER_END    	0x02        /**< Deassert CS after transfer */
//#define I2C_CS_HOLD		    0x04        /**< Hold CS in the transfer  */

/*-----------------------------------------------------------------------
 * Representation of a I2C slave, i.e. what we're communicating with.
 *
 * Drivers are expected to extend this with controller-specific data.
 *
 *   bus:       ID of the bus that the slave is attached to.
 *   cs:        ID of the chip select connected to the slave.
 *   freq:        slave clock frequency.
 */
struct i2c_slave {
        unsigned int    bus;
        unsigned int    cs;
		uint32_t	freq;
};


struct board_i2c_slave {
	struct i2c_slave slave;
	void		*handle;
	uint32_t freq;	/* I2C clock frequency in Hz */
	/*
	**	mode 0 (CPOL=0,CPHA=0)	= clock idle low, data read on rising edge, changed on falling edge
	**	mode 1 (CPOL=0,CPHA=1)	= clock idle low, data read on falling edge, changed on rising edge
	**	mode 2 (CPOL=1,CPHA=0)	= clock idle high, data read on falling edge, changed on rising edge
	**	mode 3 (CPOL=1,CPHA=1)	= clock idle high, data read on rising edge, changed on falling edge
	*/
	unsigned int mode;
};


#ifndef offsetof
#define offsetof(TYPE, MEMBER) ((uint32_t) &((TYPE *)0)->MEMBER)
#endif

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:        the pointer to the member.
 * @type:       the type of the container struct this is embedded in.
 * @member:     the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) \
		( (type *)( (char *)ptr - offsetof(type,member) ) )

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

static inline struct board_i2c_slave *to_board_i2c(struct i2c_slave *slave)
{
	return container_of(slave, struct board_i2c_slave, slave);
}

void i2c_initParams0(void);
void i2c_initParams1(void);
void i2c_initParams2(void);

int i2c_xfer (struct i2c_slave *slave, unsigned int bitlen, const void *dout, void *din, unsigned int flags);
int i2c_xfer1(struct i2c_slave *slave, unsigned int bitlen, const void *dout, void *din, unsigned int flags);
int i2c_xfer2(struct i2c_slave *slave, unsigned int bitlen, const void *dout, void *din, unsigned int flags);
int i2c_xfer3(struct i2c_slave *slave, unsigned int bitlen, const void *dout, void *din, unsigned int flags);


int i2c_xfer_async (struct i2c_slave *slave, unsigned int bitlen, 	const void *dout, void *din, unsigned int flags, void *appCallback);
int i2c_xfer1_async(struct i2c_slave *slave, unsigned int bitlen, 	const void *dout, void *din, unsigned int flags, void *appCallback);
int i2c_xfer2_async(struct i2c_slave *slave, unsigned int bitlen, 	const void *dout, void *din, unsigned int flags, void *appCallback);
int i2c_xfer3_async(struct i2c_slave *slave, unsigned int bitlen, 	const void *dout, void *din, unsigned int flags, void *appCallback);

void i2c_init_async_io();


int i2c_xfer_seg_multibuf_async(struct i2c_slave *slave, unsigned int *bitlen,
		const void **dout, void **din, unsigned int *flags, unsigned int nbuffers);
int i2c_xfer1_seg_multibuf_async(struct i2c_slave *slave, unsigned int *bitlen,
		const void **dout, void **din, unsigned int *flags, unsigned int nbuffers);
int i2c_xfer2_seg_multibuf_async(struct i2c_slave *slave, unsigned int *bitlen,
		const void **dout, void **din, unsigned int *flags, unsigned int nbuffers);
int i2c_xfer3_seg_multibuf_async(struct i2c_slave *slave, unsigned int *bitlen,
		const void **dout, void **din, unsigned int *flags, unsigned int nbuffers);



struct i2c_slave *i2c_setup_slave(unsigned int bus, unsigned int cs, uint32_t max_hz, unsigned int mode);
void i2c_free_slave(struct i2c_slave *slave);
int i2c_claim_bus(struct i2c_slave *slave);
void i2c_release_bus(struct i2c_slave *slave);

extern GIO_Handle  i2cHandle[];
extern xSemaphoreHandle i2cLock[];
extern xSemaphoreHandle	i2cAsyncSignalSem[];


#ifdef __cplusplus
}
#endif


#endif


