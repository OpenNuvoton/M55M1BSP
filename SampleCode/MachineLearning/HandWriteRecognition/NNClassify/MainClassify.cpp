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
    numOutClasses = model->GetOutputShape()->data[1];

    // Following are for debug purposes.
    printf("Initialising MNIST HandWriteRecognition model..\r\n");
    printf("numInputDims: %d-%d-%d-%d\r\n", numInputDims[0], numInputDims[1], numInputDims[2], numInputDims[3]);
    printf("numOutputDims: %d \r\n", numOutClasses);

    output = std::vector<int8_t>(numOutClasses, 0.0);

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

    printf("inQuantParams: scale=%f, offset=%d \r\n", inQuantParams.scale, inQuantParams.offset);
    printf("outQuantParams: scale=%f, offset=%d \r\n", outQuantParams.scale, outQuantParams.offset);

}


uint8_t MainClassify::QuantizeInputData(uint8_t *u8pImg)
{
    uint8_t ret = 0;

    for (uint32_t ii = 0; ii < MNIST_HANDWRITE_WIDTH * MNIST_HANDWRITE_HEIGHT * 1; ii++)
    {
        /*
            MNIST handwrite model preprocessing is each image pixel is normalized to 255,
            then quantize them with input quantization info.
        */
        int8_t i_data_int8 = (int8_t)((((float)(*u8pImg++)) / 255) / inQuantParams.scale) + inQuantParams.offset;

        /* Feed to buffer for inference*/
        this->gsensorBufferPtr[ii] = i_data_int8;

    }

    return ret;
}

void MainClassify::FillInTensorData()
{

    // Copy image into the TfLite tensor.
    int8_t *inTensorData = tflite::GetTensorData<int8_t>(model->GetInputTensor());


    // Copy -sensor data into the TfLite tensor.
    memcpy(inTensorData, gsensorBufferPtr, MNIST_HANDWRITE_WIDTH * MNIST_HANDWRITE_HEIGHT * 1 * sizeof(int8_t));
}

void MainClassify::Classify()
{

    int8_t *inTensorData = tflite::GetTensorData<int8_t>(model->GetInputTensor());

    // Copy -sensor data into the TfLite tensor.
    memcpy(inTensorData, gsensorBufferPtr, MNIST_HANDWRITE_WIDTH * MNIST_HANDWRITE_HEIGHT * sizeof(int8_t));

    // Run inference on this data.
    model->RunInference();

    // Get output from the TfLite tensor.
    int8_t *outTensorData = tflite::GetTensorData<int8_t>(model->GetOutputTensor());
    memcpy(output.data(), outTensorData, numOutClasses * sizeof(int8_t));

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





/*** (C) COPYRIGHT 2023 Nuvoton Technology Corp. ***/