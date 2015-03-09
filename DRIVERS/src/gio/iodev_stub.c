/**
* @file iodev_stub.c
* @brief IODEV io-driver stub for functions not implemented
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 07.05.2012
*/

#include <iodev.h>

int IODEV_iodNotImpl(void)
{
    return (IODEV_ENOTIMPL);
}

