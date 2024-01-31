/**************************************************************************//**
 * @file     MainClassify.h
 * @version  V1.00
 * @brief    Machine Learnig model manipulation and data input and output auxillary function header
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#ifndef __MAINCLASSIFY_H__
#define __MAINCLASSIFY_H__

#include <vector>
#include "Model.h"
#include "board_m55M1.h"

#define  VAL_NORMALIZE_MAX        (1572)
#define  VAL_NORMALIZE_MIN         (-2048)
#define  VAL_ANOMALY_IMU_00G_THRESHOLD         (0.39)

typedef struct
{
    float   scale;
    int     offset;
} QuantParams;


class MainClassify
{

public:
    MainClassify(int8_t *pBuffer);

    ~MainClassify() = default;

    void ExtractFeatures();
    void Classify();
    void FillInTensorData();
    void AveragePredictions();
    int GetTopClass(const std::vector<int8_t> &prediction);
    int GetInferenceResult(const std::vector<int8_t> &prediction);
    uint8_t FillSensorData(uint8_t u8Img);
    uint8_t QuantizeInputData(void);
    void InitMainClassify();
    int FeedImgFiletoSensorBuffer(const uint8_t *p_src);
    int GetAnomalyDetectResult(float f_mae);
    int8_t *gsensorBufferPtr;
    std::vector<int8_t> output;
    QuantParams GetTensorQuantParams(TfLiteTensor *tensor);
    uint32_t gsensorBufferIdx;
    int numOutClasses;
    int numInputDims[4];
    int numOutputDims[4];
    float gsensorBuffer[IMU_DATAIN_SIZE];
    QuantParams inQuantParams;//To keep input quantization info
    QuantParams outQuantParams;

protected:
    /** @brief Initialises the model */
    bool _InitModel();
    //void InitMainClassify();
    std::unique_ptr<Model> model;
};


#endif // __MAINCLASSIFY_H__ 
