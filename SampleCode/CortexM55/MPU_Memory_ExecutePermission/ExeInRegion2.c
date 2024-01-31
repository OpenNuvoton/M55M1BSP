#include <stdio.h>
#include <stdlib.h>
#include "NuMicro.h"

__attribute__((section(".MPU_region2"))) uint32_t ExeInRegion2(void)
{
    printf("\n+------------------------------------------+\n");
    printf("[%s] enter\n", __func__);
    printf("  Code execution address: 0x%08X\n", (uint32_t)ExeInRegion2);
    printf("[%s] leave\n", __func__);
    printf("+------------------------------------------+\n");

    return (uint32_t)ExeInRegion2;
}
