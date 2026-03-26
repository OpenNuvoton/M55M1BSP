/**************************************************************************//**
 * @file     jacclib.h
 * @version  V1.00
 * @brief    Libjpeg simd porting related header
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef  __JACCLIB_H__
#define  __JACCLIB_H__
#include <stdlib.h>
#include <inttypes.h>
#include "jpeglib.h"

/****************************************************************************
 * Macros Declaration
 ****************************************************************************/
//#define GETJSAMPLE(value)  ((int)(value))
#define MAXJSAMPLE       255
#define CENTERJSAMPLE    128
#define SHIFT_SIZE 0
#define DCTSIZE             8   /* The basic DCT block is 8x8 samples */
#define DCTSIZE2            64  /* DCTSIZE squared; # of elements in a block */
#define NUM_QUANT_TBLS      4   /* Quantization tables are numbered 0..3 */
#define NUM_HUFF_TBLS       4   /* Huffman tables are numbered 0..3 */
#define NUM_ARITH_TBLS      16  /* Arith-coding tables are numbered 0..15 */
#define MAX_COMPS_IN_SCAN   4   /* JPEG limit on # of components in one scan */
#define MAX_SAMP_FACTOR     4   /* JPEG limit on sampling factors */


#define ALIGNMENT_REQ               aligned(16)
/* Form the attributes, alignment is mandatory */
#define ALIGNMENT_ATTRIBUTE   __attribute__((ALIGNMENT_REQ))

/****************************************************************************
 * Data Structure
 ****************************************************************************/
typedef unsigned char JSAMPLE;
typedef unsigned int JDIMENSION;
typedef JSAMPLE *JSAMPROW;      /* ptr to one image row of pixel samples. */
typedef JSAMPROW *JSAMPARRAY;   /* ptr to some rows (a 2-D sample array) */
typedef JSAMPARRAY *JSAMPIMAGE; /* a 3-D sample array: top index is color */
typedef long JLONG;
typedef short JCOEF;
typedef JCOEF JBLOCK[DCTSIZE2];         /* one block of coefficients */
typedef JBLOCK *JBLOCKROW;              /* pointer to one row of coefficient blocks */
typedef JBLOCKROW *JBLOCKARRAY;         /* a 2-D array of coefficient blocks */
typedef JBLOCKARRAY *JBLOCKIMAGE;       /* a 3-D array of coefficient blocks */
typedef JCOEF *JCOEFPTR;                /* useful in a couple of places */
typedef short DCTELEM;                  /* prefer 16 bit with SIMD for parellelism */
typedef unsigned short UDCTELEM;
typedef unsigned int UDCTELEM2;
typedef struct
{
    unsigned short reciprocal;
    short corr;
    unsigned short scale;
    unsigned short shift;
} _sDivisior;

#define MULTIPLIER  int

typedef unsigned char JOCTET;

#if defined(__x86_64__) && defined(__ILP32__)
    typedef unsigned long long bit_buf_type;
#else
    typedef size_t bit_buf_type;
#endif

#if defined(WITH_SIMD) && !(defined(__arm__) || defined(__aarch64__) || \
                            defined(_M_ARM) || defined(_M_ARM64))
typedef unsigned long long simd_bit_buf_type;
#else
typedef bit_buf_type simd_bit_buf_type;
#endif


typedef struct
{
    INT32 put_buffer;       /* current bit-accumulation buffer */
    int put_bits;           /* # of bits now in it */
    int free_bits;
    int last_dc_val[MAX_COMPS_IN_SCAN]; /* last DC coef for each component */
} savable_state_simd;

typedef struct
{
    JOCTET *next_output_byte;   /* => next byte to write in buffer */
    size_t free_in_buffer;      /* # of byte spaces remaining in buffer */
    savable_state_simd cur;     /* Current bit buffer & DC state */
    j_compress_ptr cinfo;       /* dump_buffer needs access to this */
} working_state_simd;


typedef struct
{
    unsigned int ehufco[256];     /* code for each symbol */
    char ehufsi[256];             /* length of code for each symbol */
    /* If no code has been allocated for a symbol S, ehufsi[S] contains 0 */
} c_derived_tbl;
/****************************************************************************
 * Function Declaration
 ****************************************************************************/
#ifdef  __cplusplus
extern  "C" {
#endif
void jsimd_fdct_ifast_Helium(int16_t *data);
void jsimd_convsamp_helium(uint8_t *sample_data, uint8_t start_col, int16_t *processed_data);
void jsimd_quantize_helium(JCOEFPTR coef_block, DCTELEM *divisors, DCTELEM *workspace);
void jsimd_fdct_islow_helium(int16_t *data);
int compute_reciprocal(uint16_t divisor, DCTELEM *dtbl);
void JpegEncode(unsigned char *image, unsigned char *jBuf, unsigned long *jSize, int width, int height, int quality);
void encode_jpeg_to_memory(unsigned char *image, int width, int height, int quality, const char *comment, unsigned long *jpegSize, unsigned char **jpegBuf);
void fdct(short int *blk, int lx);
int flss(uint16_t val);
uint64_t Get_SysTick_Cycle_Count(void);
int compute_reciprocal(UINT16 divisor, DCTELEM *dtbl);
void jpeg_fdct_islow(DCTELEM *data);
void quantize(JCOEFPTR coef_block, DCTELEM *divisors, DCTELEM *workspace);
boolean encode_one_block_simd(working_state_simd *state, JCOEFPTR block, int last_dc_val, c_derived_tbl *dctbl, c_derived_tbl *actbl);
#ifdef  __cplusplus
}
#endif
#endif

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/
