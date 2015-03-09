/**
* @file _dev.h
* @brief DEV private header file
*
* @author Eli Schneider
*
* @version 0.0.1
* @date 08.05.2012
*/
#ifndef __DEV_H
#define __DEV_H


#include <dev.h>


#ifdef __cplusplus
extern "C" {
#endif


extern DEV_Tinit _DEV_initFxn[];
extern int _DEV_numStaticDevs;
extern DEV_TableElem _DEV_staticDevTable[];

extern QUE_Obj DEV_table;



#ifdef __cplusplus
}
#endif


#endif




