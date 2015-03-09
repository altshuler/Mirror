/**********************************************************************
* File: appinfo.ld 
* with .appinfo section to be placed in APP_INFO.
*
* History: 23.10.2013 - original (E. Schneider)
**********************************************************************/


/**************************************************************/
/* Link .appinfo sections  		                           */
/**************************************************************/
SECTIONS
{
	/*** Application information ***/
	.appinfo (NOLOAD) : 
 	{
	    	*(.appinfo)        /* .appinfo sections  */
   	} > APP_INFO
}

/******************* end of file ************************/
