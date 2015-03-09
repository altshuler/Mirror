/**********************************************************************
* File: bootinfo.ld 
* with .bootinfo section to be placed in BOOT_INFO.
*
* History: 30.10.2013 - original (E. Schneider)
**********************************************************************/


/**************************************************************/
/* Link .bootinfo sections  		                           */
/**************************************************************/
SECTIONS
{
	/*** Application information ***/
	.bootinfo (NOLOAD) : 
 	{
	    	*(.bootinfo)        /* .bootinfo sections  */
   	} > BOOT_INFO
}

/******************* end of file ************************/
