#ifndef __PMC100_MEM_CONFIG_H__
#define __PMC100_MEM_CONFIG_H__

#include "NuMicro.h"
#include "pmc100.h"

/******************************************************************************/
/*         Helper macros for accessing parameters from the context            */
/******************************************************************************/
#define PMC100_PIDR2 (0x0000000BU | (ctx->params->REVISION << 4))
#define PMC100_PIDR3 (0x00000000U | (ctx->params->REVAND << 4))

#define ADDRCD                  1

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum core_ram_enum
{
    eMEM_IDATA = 0,
    eMEM_DDATA,
    eMEM_CNT
} core_ram_type;

extern const Pmc100MemInfo_type core_mem_pmc100[eMEM_CNT];
extern const Pmc100Params_type core_params_pmc100;

int32_t PMC100_Set_Reg_Zero(const Pmc100Context_type *ctx);
int32_t PMC100_CleanData(YAMIN_PMC100_CFG_Type *psPMC100_Config);

#ifdef __cplusplus
}
#endif

#endif  // __PMC100_MEM_CONFIG_H__
