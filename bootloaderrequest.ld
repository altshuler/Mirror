/**********************************************************************
* File: bootloaderrequest.ld 
* with .bootloaderreq section to be placed in BOOT_LOADER_REQ.
* with .bootloaderrequest section to be placed in RAM.
*
* History: 30.10.2013 - original (E. Schneider)
**********************************************************************/


/**************************************************************/
/* Link .bootinfo sections  		                           */
/**************************************************************/
SECTIONS
{
	bootloaderreq (NOLOAD) :
    {
      . = ALIGN(4);
	    *(.bootloaderreq)        /* .bootloaderreqsig sections  */
      . = ALIGN(4);
    } > BOOT_LOADER_REQ

	/*** Application information ***/
	.bootloaderreqsig (NOLOAD) : 
 	{
	    *(.bootloaderreqsig)        /* .bootloaderreqsig sections  */
   	} > RAM
}

/******************* end of file ************************/
