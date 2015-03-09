/**********************************************************************
* File: bootparam.ld 
* with .bootparam section to be placed in BOOT_PARAM.
*
* History: 30.10.2013 - original (E. Schneider)
**********************************************************************/


/**************************************************************/
/* Link .bootparam sections  		                           */
/**************************************************************/
SECTIONS
{
	/*** Application information ***/
	.bootparam (NOLOAD) : 
 	{
	    	*(.bootparam)        /* .bootparam sections  */
   	} > BOOT_PARAM
}

/******************* end of file ************************/
