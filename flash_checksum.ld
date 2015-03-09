/**********************************************************************
* File: Flash_checksum.cmd 
* with .flashcsum section to be placed in FLASH_CSUM.
*
* History: 23.10.2013 - original (E. Schneider)
**********************************************************************/


/**************************************************************/
/* Link .flashcsum sections  		                           */
/**************************************************************/
SECTIONS
{
	/*** Flash memory checksum ***/
	.flashcsum (NOLOAD) : 
  	{
    	*(.flashcsum)        /* .flashcsum sections  */
   	} > FLASH_CSUM
}

/******************* end of file ************************/
