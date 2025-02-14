#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "SPIFlashInit.h"

enum
{
    eDST_OFFSET,
    eSRC_IMG_BASE,
    eSRC_IMG_SIZE
};

extern uint32_t  NUBL32_IMAGE_BASE, NUBL32_IMAGE_LIMIT, NUBL32_IMAGE_SIZE;
extern uint32_t  NUBL33_IMAGE_BASE, NUBL33_IMAGE_LIMIT, NUBL33_IMAGE_SIZE;

int32_t _WriteFlash(uint32_t addr, uint32_t size, uint32_t *pu32Data, uint32_t u32CmdMask);

uint8_t g_au8DataBuf[DCACHE_ALIGN_LINE_SIZE(SPI_FLASH_PAGE_SIZE)] __ALIGNED(DCACHE_LINE_SIZE) = { 0 };

int32_t SPIM_WriteImage(void)
{
    int32_t  i32RetCode = 0;
    uint32_t aau32ImgInfo[][3] =
    {
        { 0x0, (uint32_t) &NUBL32_IMAGE_BASE, (uint32_t) &NUBL32_IMAGE_SIZE },
        { 0x100000, (uint32_t) &NUBL33_IMAGE_BASE, (uint32_t) &NUBL33_IMAGE_SIZE },
    };
    uint32_t u32StartAddr, u32Offset, i;

    for (i = 0; i < (sizeof(aau32ImgInfo) / sizeof(aau32ImgInfo[0])); i++)
    {
        printf("\n\nSource Image Base: 0x%08X, Source Image Size: 0x%08X\n", aau32ImgInfo[i][eSRC_IMG_BASE], aau32ImgInfo[i][eSRC_IMG_SIZE]);
        u32StartAddr = aau32ImgInfo[i][eDST_OFFSET];

        for (u32Offset = 0; u32Offset < aau32ImgInfo[i][eSRC_IMG_SIZE]; u32Offset += SPI_FLASH_PAGE_SIZE)
        {
            printf("[%s] Program offset: 0x%08X\n", __func__, u32StartAddr + u32Offset);
            memcpy(g_au8DataBuf, (uint32_t *)(aau32ImgInfo[i][eSRC_IMG_BASE] + u32Offset), SPI_FLASH_PAGE_SIZE);
            printf("g_au8DataBuf: 0x%08X\n", *(uint32_t *)g_au8DataBuf);
#if (NVT_DCACHE_ON == 1)
            // Clean the data cache for the buffer to ensure data consistency
            SCB_CleanDCache_by_Addr(g_au8DataBuf, sizeof(g_au8DataBuf));
#endif

            if ((i32RetCode = _WriteFlash(u32StartAddr + u32Offset, SPI_FLASH_PAGE_SIZE, (uint32_t *)g_au8DataBuf, 0)) != 0)
            {
                printf("Write flash failed ! (i32RetCode: %d)\n", i32RetCode);
                break;
            }
        }
    }

    printf("Done\n");

    return 0;
}
