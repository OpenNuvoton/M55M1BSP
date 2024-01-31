/**************************************************************************//**
 * @file     MainClassify.cc
 * @version  V1.00
 * @brief    Machine Learnig model manipulation and data input and output auxillary function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <float.h>
#include "MainClassify.h"
#include "CNNModel.h"
#include "board_m55m1.h"

bool MainClassify::_InitModel()
{
    this->model = std::unique_ptr<CNNModel>(new CNNModel());

    if (this->model)
    {
        return this->model->Init();
    }

    printf("Failed to allocate memory for the model\r\n");
    return false;
}

MainClassify::MainClassify(int8_t *pBuffer)
{
    if (this->_InitModel())
    {
        this->gsensorBufferIdx = 0;
        this->gsensorBufferPtr = pBuffer;
        this->InitMainClassify();
    }
}

void MainClassify::InitMainClassify()
{
    if (!model->IsInited())
    {
        printf("Warning: model has not been initialised\r\n");
        model->Init();
    }

    numInputDims[0] = model->GetInputShape()->data[0]; //Show input data shape
    numInputDims[1] = model->GetInputShape()->data[1];
    numInputDims[2] = model->GetInputShape()->data[2];
    numInputDims[3] = model->GetInputShape()->data[3];
    numOutputDims[0] = model->GetOutputShape()->data[0];  // Show output data shape
    numOutputDims[1] = model->GetOutputShape()->data[1];
    numOutputDims[2] = model->GetOutputShape()->data[2];
    numOutputDims[3] = model->GetOutputShape()->data[3];

    numOutClasses = model->GetOutputShape()->data[1];

    // Following are for debug purposes.
    printf("Initialising CNN AutoEncoder model..\r\n");
    printf("numInputDims: %d-%d-%d-%d\r\n", numInputDims[0], numInputDims[1], numInputDims[2], numInputDims[3]);
    printf("numOutputDims: %d-%d-%d-%d\r\n", numOutputDims[0], numOutputDims[1], numOutputDims[2], numOutputDims[3]);

    output = std::vector<int8_t>(IMU_DATAIN_SIZE, 0.0);//IMU_DATAIN_SIZE=600

    /*
        Get input and output quantization information
    */
    TfLiteTensor *outputTensor = model->GetOutputTensor();
    TfLiteTensor *inputTensor = model->GetInputTensor();

    /*
        Vww model preprocessing is image conversion from uint8 to [0,1] float values,
        then quantize them with input quantization info.
    */
    inQuantParams = GetTensorQuantParams(inputTensor);
    outQuantParams = GetTensorQuantParams(outputTensor);
}

void MainClassify::FillInTensorData()
{

    // Copy image into the TfLite tensor.
    int8_t *inTensorData = tflite::GetTensorData<int8_t>(model->GetInputTensor());

    // Copy -sensor data into the TfLite tensor.
    memcpy(inTensorData, gsensorBufferPtr, IMU_DATAIN_SIZE * 1 * sizeof(int8_t));
}



void MainClassify::Classify()
{

    // Run inference on this data.
    model->RunInference();

    // Get output from the TfLite tensor.
    int8_t *outTensorData = tflite::GetTensorData<int8_t>(model->GetOutputTensor());
    memcpy(output.data(), outTensorData, IMU_DATAIN_SIZE * 1 * sizeof(int8_t));

}

uint8_t MainClassify::FillSensorData(uint8_t u8Img)
{
    float fBuff[3];
    uint8_t ret = 0;

    if (gsensorBufferIdx < IMU_DATAIN_SIZE)
    {
        MPU6500_readXYZ_mg(fBuff);
        this->gsensorBuffer[gsensorBufferIdx] = ((fBuff[0]) - VAL_NORMALIZE_MIN) / (VAL_NORMALIZE_MAX - VAL_NORMALIZE_MIN) ;
        this->gsensorBuffer[gsensorBufferIdx + 1] = ((fBuff[1]) - VAL_NORMALIZE_MIN) / (VAL_NORMALIZE_MAX - VAL_NORMALIZE_MIN) ;
        this->gsensorBuffer[gsensorBufferIdx + 2] = ((fBuff[2]) - VAL_NORMALIZE_MIN) / (VAL_NORMALIZE_MAX - VAL_NORMALIZE_MIN) ;

        gsensorBufferIdx += 3;
    }
    else
    {
        gsensorBufferIdx = 0;
        ret = 1;
        //printf("FillSensorData FULL!\r\n");
    }

    return ret;
}

uint8_t MainClassify::QuantizeInputData()
{
    uint8_t ret = 0;
    float *fpImg = &this->gsensorBuffer[0];
    uint32_t ii;

    for (ii = 0; ii < IMU_DATAIN_SIZE; ii++)
    {
        /*Quantize input data with input quantization param.*/
        this->gsensorBufferPtr[ii] = (int8_t)((((float)(*fpImg++))) / inQuantParams.scale) + inQuantParams.offset;
    }

    return ret;
}

QuantParams MainClassify::GetTensorQuantParams(TfLiteTensor *tensor)
{
    QuantParams params = {.scale = 0, .offset = 0};

    if (kTfLiteAffineQuantization == tensor->quantization.type)
    {
        auto *quantParams = (TfLiteAffineQuantization *)(tensor->quantization.params);

        if (quantParams && 0 == quantParams->quantized_dimension)
        {
            if (quantParams->scale->size)
            {
                params.scale = quantParams->scale->data[0];
            }

            if (quantParams->zero_point->size)
            {
                params.offset = quantParams->zero_point->data[0];
            }
        }
        else if (tensor->params.scale != 0.0)
        {
            /* Legacy tensorflow quantisation parameters */
            params.scale = tensor->params.scale;
            params.offset = tensor->params.zero_point;
        }
    }

    return params;
}

int MainClassify::GetAnomalyDetectResult(float f_mae)
{
    /*Check value range, mae must >=0*/
    if (f_mae < 0) return -1;

    if (f_mae > VAL_ANOMALY_IMU_00G_THRESHOLD) return 1;//Anomal
    else if (f_mae < (VAL_ANOMALY_IMU_00G_THRESHOLD - 0.01)) return 1; //Anomal
    else return 0; //Normal

}

/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/