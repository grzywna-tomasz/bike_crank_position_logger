/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "sd_card.h"

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status(
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	/* No disk status implementation. This is simple project and I assume "it just works" */
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize(
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	/* Initialization done in task, nothing to do here */
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read(
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT ret_val = RES_OK;
	for (uint32_t i = 0; i < count; i++)
	{
        if (E_OK != SDCard_ReadSingleBlock(sector + i, &buff[i * 512]))
		{
			ret_val = RES_ERROR;
			break;
		}
    }

    return ret_val;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write(
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT ret_val = RES_OK;
	for (uint32_t i = 0; i < count; i++) 
	{
		/* TODO shouldnt it be sector + i*512? */
        if (E_OK != SDCard_WriteSingleBlock(sector + i, &buff[i * 512]))
		{
			ret_val = RES_ERROR;
			break;
		}
    }

    return ret_val;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl(
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT ret_val = RES_OK;

    switch (cmd) 
	{
        case CTRL_SYNC:
		{
            /* Do nothing */
			break;
		}

        case GET_SECTOR_SIZE:
		{
            *(uint32_t*)buff = 512;
			break;
		}

        case GET_BLOCK_SIZE:
		{
            *(uint32_t*)buff = 1;
			break;
		}

        case GET_SECTOR_COUNT:
		{
            *(uint32_t*)buff = SDCard_GetSectorCount();
			break;
		}

        default:
		{
			ret_val = RES_PARERR;
		}
	}

	return ret_val;
}

DWORD get_fattime(void)
{
    /* Just dummy time 20:02:15 24.06.2025 
	Year
	Month
	Day
	Hour
	Minute
	sec/2 (range 0-29)*/

    return (DWORD)((2025 - 1980) << 25) |
           (DWORD)(6 << 21) |
           (DWORD)(24 << 16) |
           (DWORD)(20 << 11) |
           (DWORD)(2 << 5) |
           (DWORD)(15);
}