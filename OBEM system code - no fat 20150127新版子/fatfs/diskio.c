/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2014        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "spi.h"
#include "sdhc.h" 



#define PERIPH_CLOCKRATE 20000000
/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/
int SD_INIT();
int do_sd_initialize (sd_context_t *sdc);
int do_sd_initialize (sd_context_t *sdc)
{
	/* Initialize the SPI controller */
	spi_initialize();
	/* Set the maximum SPI clock rate possible */
	spi_set_divisor(PERIPH_CLOCKRATE/400000);
	/* Initialization OK? */
	if (sd_initialize(sdc) != 1)
		return 0;
        spi_set_divisor(2);   //---- 2011 0905 spi full speed
	return 1;
}

int SD_INIT()
{
        int j, ok = 0;
        
  	/* Set some reasonable values for the timeouts. */
        sdc.timeout_write = PERIPH_CLOCKRATE/8;
        sdc.timeout_read = PERIPH_CLOCKRATE/20;
        sdc.busyflag = 0;    

        for (j=0; j<SD_INIT_TRY && ok != 1; j++)
	{
	    ok = do_sd_initialize(&sdc);
	}
        
        return ok;
}

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	/*DSTATUS stat = 0;
                
           if(SD_INIT()){
             return stat;
           }                 // sd card 初始化     
                
             return STA_NOINIT;		
  */
  return 0;

}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat = 0;
        
	     if(SD_INIT()){
             return stat;
           }                 // sd card 初始化     
                
             return STA_NOINIT;		

}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/
//------使用上注意!! 讀取一次最好不要超過1個buff---- 不然會出現不可預期錯誤
DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	//int result;


                    sd_read_block(&sdc, sector,buff);
                    sd_wait_notbusy (&sdc);
                res = RES_OK;
                return res;

        
       /* 
	switch (pdrv) {
	case ATA :
		// translate the arguments here
		result = ATA_disk_read(buff, sector, count);
		// translate the reslut code here
		return res;
	case MMC :
		// translate the arguments here
		result = MMC_disk_read(buff, sector, count);
		// translate the reslut code here
		return res;
	case USB :
		// translate the arguments here
		result = USB_disk_read(buff, sector, count);
		// translate the reslut code here
		return res;
	}
                
	return RES_PARERR;
                */
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/
//------使用上注意!! 寫入一次最好不要超過1個buff(512byte)---- 不然會出現不可預期錯誤
#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
//	int result;

            sd_write_block (&sdc, sector, buff);
            sd_wait_notbusy (&sdc);
            res = RES_OK;
            return res;
            
            
        /*
	switch (pdrv) {
	case ATA :
		// translate the arguments here

		result = ATA_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;

	case MMC :
		// translate the arguments here

		result = MMC_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;

	case USB :
		// translate the arguments here

		result = USB_disk_write(buff, sector, count);

		// translate the reslut code here

		return res;
	}
        
	return RES_PARERR;*/
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
         res = RES_OK;
            return res;
        /*
        
	int result;

	switch (pdrv) {
	case ATA :

		// Process of the command for the ATA drive

		return res;

	case MMC :

		// Process of the command for the MMC/SD card

		return res;

	case USB :

		// Process of the command the USB drive

		return res;
	}

	return RES_PARERR;*/
            
}
#endif
