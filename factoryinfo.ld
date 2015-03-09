/**********************************************************************
* File: factoryinfo.ld 
* with .factoryinfo section to be placed in FACTORY_INFO.
*
* History: 31.10.2013 - original (E. Schneider)
**********************************************************************/


/**************************************************************/
/* Link .factoryinfo sections  		                           */
/**************************************************************/
SECTIONS
{
	/*** Factory Manufacturing information ***/
	.bootfactoryinfo  (NOLOAD)  : 
 	{
    	*(.bootfactoryinfo)        /* .bootfactoryinfo sections  */
   	} > BOOT_FACTORY_INFO
	.factoryinfo  (NOLOAD)  : 
 	{
    	*(.factoryinfo)        /* .factoryinfo sections  */
   	} > FACTORY_INFO
}

/******************* end of file ************************/

