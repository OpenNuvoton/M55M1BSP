#ifndef _XOMAPI_H_
#define _XOMAPI_H_

#ifdef __cplusplus
extern "C"
{
#endif

int32_t XOM_Add(int32_t a, int32_t b);
int32_t XOM_Sub(int32_t a, int32_t b);
int32_t XOM_Mul(int32_t a, int32_t b);
int32_t XOM_Div(int32_t a, int32_t b);
int32_t XOM_Sum(int32_t *pbuf, int32_t n);

#ifdef __cplusplus
}
#endif

#endif //_XOM_API_H_
