/**
 * Copyright (C) 2023 Texas Instruments Incorporated
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 *  \file   mmw_lvds_stream.c
 *
 *  \brief  Implements LVDS stream functionality.
 */

#ifdef LVDS_STREAM
/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */
/* MMWAVE Demo Include File */
#include <../mmw_common.h>

/* ========================================================================= */
/*                        Static Function Declaration                        */
/* ========================================================================= */

static void allocateEDMAShadowChannel(EDMA_Handle handle, uint32_t *param);static void allocateEDMAChannel(EDMA_Handle handle, uint32_t *dmaCh, \
                                            uint32_t *tcc, uint32_t *param);
static void MmwLVDSStream_EDMAInit(void);
static void MmwLVDSStream_EDMAAllocateCBUFFChannel
            (CBUFF_EDMAInfo* ptrEDMAInfo, CBUFF_EDMAChannelCfg* ptrEDMACfg);
static int32_t MmwLVDSStream_EDMAAllocateCBUFFHwChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
);
static void MmwLVDSStream_EDMAFreeCBUFFHwChannel(CBUFF_EDMAChannelCfg* ptrEDMACfg);
static void MmwLVDSStream_freeDmaChannels(EDMA_Handle edmaHandle);
static void MmwLVDSStream_HwTriggerFrameDone (CBUFF_SessionHandle sessionHandle);
static int32_t MmwLVDSStreamHwConfig (uint8_t subFrameIndx);


/* ========================================================================= */
/*                            Global Variables                               */
/* ========================================================================= */
uint8_t MmwLVDSStream_hwDataHeader[256] __attribute__((aligned(32)));


/* ========================================================================= */
/*                          Function Definitions                             */
/* ========================================================================= */

static void allocateEDMAShadowChannel(EDMA_Handle handle, uint32_t *param)
{
    int32_t             testStatus = SystemP_SUCCESS;

    testStatus = EDMA_allocParam(handle, param);
    DebugP_assert(testStatus == SystemP_SUCCESS);

    return;
}


static void allocateEDMAChannel(EDMA_Handle handle,
    uint32_t *dmaCh,
    uint32_t *tcc,
    uint32_t *param
)
{
    int32_t             testStatus = SystemP_SUCCESS;
    uint32_t            baseAddr, regionId;
    EDMA_Config        *config;
    EDMA_Object        *object;

    config = (EDMA_Config *) handle;
    object = config->object;

    if((object->allocResource.dmaCh[*dmaCh/32] & (1U << *dmaCh%32)) != (1U << *dmaCh%32))
    {
        testStatus = EDMA_allocDmaChannel(handle, dmaCh);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        testStatus = EDMA_allocTcc(handle, tcc);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        testStatus = EDMA_allocParam(handle, param);
        DebugP_assert(testStatus == SystemP_SUCCESS);

        baseAddr = EDMA_getBaseAddr(handle);
        DebugP_assert(baseAddr != 0);

        regionId = EDMA_getRegionId(handle);
        DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

        /* Request channel */
        EDMA_configureChannelRegion(baseAddr, regionId, EDMA_CHANNEL_TYPE_DMA,
            *dmaCh, *tcc, *param, 0);
   }

    return;
}


static void MmwLVDSStream_EDMAInit (void)
{
    uint32_t index = 0;

    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelAllocatorIndex = 0;

    /* Populate the LVDS Stream HW Session EDMA Channel Table: */
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[0].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_0;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[0].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_0;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[1].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_1;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[1].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_1;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[2].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_2;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[2].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_2;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[3].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_3;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[3].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_3;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[4].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_4;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[4].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_4;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[5].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_5;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[5].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_5;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[6].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_6;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[6].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_6;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[7].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_7;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[7].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_7;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[8].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_8;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[8].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_8;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[9].chainChannelsId       = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_9;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[9].shadowLinkChannelsId  = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_9;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[10].chainChannelsId      = MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_10;
    gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[10].shadowLinkChannelsId = MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_10;

    /* Allocate CBUFF HW Session EDMA channels. */
    for (index=0; index < 11; index++)
    {
        allocateEDMAChannel(gMmwMssMCB.edmaHandle, &gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[index].chainChannelsId,
                            &gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[index].chainChannelsId,
                            &gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[index].chainChannelsId);

        allocateEDMAShadowChannel(gMmwMssMCB.edmaHandle, &gMmwMssMCB.lvdsStream.hwSessionEDMAChannelTable[index].shadowLinkChannelsId);
    }

}


static void MmwLVDSStream_EDMAAllocateCBUFFChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    if(ptrEDMAInfo->dmaNum == 0)
    {
        ptrEDMACfg->chainChannelsId      = MMW_LVDS_STREAM_CBUFF_EDMA_CH_0;

        allocateEDMAChannel(gMmwMssMCB.edmaHandle, &ptrEDMACfg->chainChannelsId,
                    &ptrEDMACfg->chainChannelsId, &ptrEDMACfg->chainChannelsId);

        ptrEDMACfg->shadowLinkChannelsId = MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0;

        allocateEDMAShadowChannel(gMmwMssMCB.edmaHandle, &ptrEDMACfg->shadowLinkChannelsId);
    }
    else if(ptrEDMAInfo->dmaNum == 1)
    {
        ptrEDMACfg->chainChannelsId      = MMW_LVDS_STREAM_CBUFF_EDMA_CH_1;

        allocateEDMAChannel(gMmwMssMCB.edmaHandle, &ptrEDMACfg->chainChannelsId,
                        &ptrEDMACfg->chainChannelsId, &ptrEDMACfg->chainChannelsId);

        ptrEDMACfg->shadowLinkChannelsId = MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_1;

        allocateEDMAShadowChannel(gMmwMssMCB.edmaHandle, &ptrEDMACfg->shadowLinkChannelsId);
    }
    else
    {
        /* Max of 2 CBUFF sessions can be configured*/
        MmwDemo_debugAssert (0);
    }
}


/*! \brief This is the registered CBUFF EDMA channel allocation function
 *      which allocates EDMA channels for CBUFF HW Session.
 */
static int32_t MmwLVDSStream_EDMAAllocateCBUFFHwChannel
(
    CBUFF_EDMAInfo*         ptrEDMAInfo,
    CBUFF_EDMAChannelCfg*   ptrEDMACfg
)
{
    int32_t         retVal = MINUS_ONE;
    MmwDemo_LVDSStream_MCB_t *streamMCBPtr =  &gMmwMssMCB.lvdsStream;

    if(ptrEDMAInfo->isFirstEDMAChannel)
    {
        MmwLVDSStream_EDMAAllocateCBUFFChannel(ptrEDMAInfo, ptrEDMACfg);
        retVal = 0;
    }
    else
    {

        /* Sanity Check: Are there sufficient EDMA channels? */
        if (streamMCBPtr->hwSessionEDMAChannelAllocatorIndex >= MMWDEMO_LVDS_STREAM_HW_SESSION_MAX_EDMA_CHANNEL)
        {
            /* Error: All the EDMA channels are allocated */
            test_print ("Error: MmwLVDSStream_EDMAAllocateCBUFFChannel failed. HW channel index=%d\n",
                            streamMCBPtr->hwSessionEDMAChannelAllocatorIndex);
            goto exit;
        }

        /* Copy over the allocated EDMA configuration. */
        memcpy ((void *)ptrEDMACfg,
                (void*)&streamMCBPtr->hwSessionEDMAChannelTable[streamMCBPtr->hwSessionEDMAChannelAllocatorIndex],
                sizeof(CBUFF_EDMAChannelCfg));

        /* Increment the allocator index: */
        streamMCBPtr->hwSessionEDMAChannelAllocatorIndex++;

        /* EDMA Channel allocated successfully */
        retVal = 0;
    }

exit:
    return retVal;
}


/*! \brief This is the registered CBUFF EDMA channel free function which frees
 * EDMA channels which had been allocated for use by a CBUFF HW Session.
 */
static void MmwLVDSStream_EDMAFreeCBUFFHwChannel (CBUFF_EDMAChannelCfg* ptrEDMACfg)
{
    uint8_t    index;
    MmwDemo_LVDSStream_MCB_t *streamMCBPtr =  &gMmwMssMCB.lvdsStream;

    if((ptrEDMACfg->chainChannelsId == MMW_LVDS_STREAM_CBUFF_EDMA_CH_0) ||
       (ptrEDMACfg->chainChannelsId == MMW_LVDS_STREAM_CBUFF_EDMA_CH_1))
    {
        /*This is the CBUFF trigger channel. It is not part of the resource table so
          nothing needs to be done*/
        goto exit;
    }

    for (index = 0U; index < MMWDEMO_LVDS_STREAM_HW_SESSION_MAX_EDMA_CHANNEL; index++)
    {
        /* Do we have a match? */
        if (memcmp ((void*)ptrEDMACfg,
                    (void*)&streamMCBPtr->hwSessionEDMAChannelTable[index],
                    sizeof(CBUFF_EDMAChannelCfg)) == 0)
        {
            /* Yes: Decrement the HW Session index */
            streamMCBPtr->hwSessionEDMAChannelAllocatorIndex--;
            goto exit;
        }
    }

    /* Sanity Check: We should have had a match. An assertion is thrown to indicate that the EDMA channel
     * being cleaned up does not belong to the table*/
    MmwDemo_debugAssert (0);
exit:
    return;
}


static void MmwLVDSStream_freeDmaChannels(EDMA_Handle edmaHandle)
{
    uint32_t index;
    uint32_t dmaCh, tcc, pram, shadow;
    for (index = 0; index < 64; index++)
    {
        dmaCh = index;
        tcc = index;
        pram = index;
        shadow = index;
        DPEDMA_freeEDMAChannel(edmaHandle, &dmaCh, &tcc, &pram, &shadow);
    }
    for (index = 0; index < 128; index++)
    {
        shadow = index;
        DebugP_assert(EDMA_freeParam(edmaHandle, &shadow) == SystemP_SUCCESS);
    }
    return;
}


/*! \brief This is the registered callback function which is invoked after the
 *  frame done interrupt is received for the hardware session.
 */
static void MmwLVDSStream_HwTriggerFrameDone (CBUFF_SessionHandle sessionHandle)
{
    int32_t     errCode;

    /* Increment stats*/
    gMmwMssMCB.lvdsStream.hwFrameDoneCount++;

    if(sessionHandle != NULL)
    {
        /* There are 2 cases to consider:
           Only one subframe configured (legacy frame):
           If there is a software session configured for this subframe, we need to
           deactivate the HW session here.

           More than one subframe configured:
           If there is more than one subframe we need to deactivate the HW
           session here as other subframes may have configured a HW session.
           In this case (more than one subframe), the HW session is always created
           and activated in the application code */
        if((gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames > 1) ||
            (gMmwMssMCB.subFrameCfg[0].lvdsStreamCfg.isSwEnabled == 1))
        {
            if(CBUFF_deactivateSession (sessionHandle, &errCode) < 0)
            {
                /* Error: Unable to deactivate the session. */
                DebugP_assert(0);
                return;
            }
        }
    }
    else
    {
        DebugP_assert(0);
    }

#if 0
    SemaphoreP_post(&gMmwMssMCB.lvdsStream.hwFrameDoneSemHandle);
#endif
}


/*! \brief This is the LVDS streaming config function.
 *  It configures the sessions for the LVDS streaming.
 */
static int32_t MmwLVDSStreamHwConfig (uint8_t subFrameIndx)
{

    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMssMCB.lvdsStream;
    int32_t                   errCode;
    int32_t                   retVal = MINUS_ONE;
    MmwDemo_SubFrameCfg       *subFrameCfg = &gMmwMssMCB.subFrameCfg[subFrameIndx];
    CBUFF_SessionCfg          sessionCfg;
    memset ((void*)&sessionCfg, 0, sizeof(CBUFF_SessionCfg));

    /* Populate the configuration: */
    sessionCfg.executionMode          = CBUFF_SessionExecuteMode_HW;
    sessionCfg.edmaHandle             = gMmwMssMCB.edmaHandle;
    sessionCfg.allocateEDMAChannelFxn = MmwLVDSStream_EDMAAllocateCBUFFHwChannel;
    sessionCfg.freeEDMAChannelFxn     = MmwLVDSStream_EDMAFreeCBUFFHwChannel;
    sessionCfg.frameDoneCallbackFxn   = MmwLVDSStream_HwTriggerFrameDone;
    sessionCfg.dataType               = CBUFF_DataType_REAL;
    sessionCfg.u.hwCfg.dataMode       = (CBUFF_DataMode)subFrameCfg->adcBufCfg.chInterleave;

    /* Populate the HW Session configuration: */
    sessionCfg.u.hwCfg.adcBufHandle      = gMmwMssMCB.adcBufHandle;
    sessionCfg.u.hwCfg.numADCSamples     = subFrameCfg->numAdcSamples;
    sessionCfg.u.hwCfg.numChirpsPerFrame = subFrameCfg->numChirpsPerSubFrame;
    sessionCfg.u.hwCfg.chirpMode         = subFrameCfg->adcBufCfg.chirpThreshold;
    sessionCfg.u.hwCfg.opMode            = CBUFF_OperationalMode_CHIRP;

    switch(subFrameCfg->lvdsStreamCfg.dataFmt)
    {
        case MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC:
            sessionCfg.u.hwCfg.dataFormat = CBUFF_DataFmt_ADC_DATA;
        break;
        case MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ:
            sessionCfg.u.hwCfg.dataFormat = CBUFF_DataFmt_CP_ADC_CQ;
            sessionCfg.u.hwCfg.cqSize[0] = 132;
            sessionCfg.u.hwCfg.cqSize[1] = 132;
            sessionCfg.u.hwCfg.cqSize[2] = 72;
        break;
        default:
            test_print ("Error: lvdsStreamCfg dataFmt %d is invalid\n", subFrameCfg->lvdsStreamCfg.dataFmt);
            MmwDemo_debugAssert(0);
        break;
    }

    if(subFrameCfg->lvdsStreamCfg.isHeaderEnabled)
    {
        MmwDemo_debugAssert(streamMcb->isHwSessionHSIHeaderAllocated == false);

        streamMcb->ptrHwSessionHSIHeader = (HSIHeader*)&MmwLVDSStream_hwDataHeader[0];

        /* Create the HSI Header to be used for the HW Session: */
        if (HSIHeader_createHeader (&sessionCfg, false, streamMcb->ptrHwSessionHSIHeader, &errCode) < 0)
        {
            /* Error: Unable to create the HSI Header; report the error */
            test_print("Error: MmwDemo_LVDSStream_config unable to create HW HSI header with [Error=%d]\n", errCode);
            goto exit;
        }

        streamMcb->isHwSessionHSIHeaderAllocated = true;

        /* Setup the header in the CBUFF session configuration: */
        sessionCfg.header.size    = HSIHeader_getHeaderSize(streamMcb->ptrHwSessionHSIHeader);
        sessionCfg.header.address = (uint32_t)(streamMcb->ptrHwSessionHSIHeader);
    }

    /* Create the HW Session: */
    streamMcb->hwSessionHandle = CBUFF_createSession (gMmwMssMCB.lvdsStream.cbuffHandle, &sessionCfg, &errCode);

    if (streamMcb->hwSessionHandle == NULL)
    {
        /* Error: Unable to create the CBUFF hardware session */
        test_print("Error: MmwDemo_LVDSStream_config unable to create the CBUFF hardware session with [Error=%d]\n", errCode);
        goto exit;
    }

    /* Control comes here implies that the LVDS Stream has been configured successfully */
    retVal = 0;

exit:
    return retVal;
}


 /**
 * This is the LVDS streaming init function.
 * It initializes the necessary modules that implement the streaming.
 */
extern int32_t MmwDemo_LVDSStreamInit (void)
{
    CBUFF_InitCfg           initCfg;
    int32_t                 retVal = MINUS_ONE;
    int32_t                 errCode;
    int32_t                 status = SystemP_SUCCESS;

    /*************************************************************************************
     * Open the CBUFF Driver:
     *************************************************************************************/
    memset ((void *)&initCfg, 0, sizeof(CBUFF_InitCfg));

    /* Populate the configuration: */
    initCfg.enableECC                 = 0U;
    initCfg.crcEnable                 = 1U;
    /* Up to 1 SW session + 1 HW session can be configured for each frame. Therefore max session is 2. */
    initCfg.maxSessions               = 2U;
    initCfg.enableDebugMode           = false;
    initCfg.interface                 = CBUFF_Interface_LVDS;
    initCfg.outputDataFmt             = CBUFF_OutputDataFmt_16bit;
    initCfg.lvdsCfg.crcEnable         = 0U;
    initCfg.lvdsCfg.msbFirst          = 1U;
    /* Enable all lanes available on the platform*/
    initCfg.lvdsCfg.lvdsLaneEnable    = 0x3U;
    initCfg.lvdsCfg.ddrClockMode      = 1U;
    initCfg.lvdsCfg.ddrClockModeMux   = 1U;

    /* Initialize the CBUFF Driver: */
    gMmwMssMCB.lvdsStream.cbuffHandle = CBUFF_open (&initCfg, &errCode);
    if (gMmwMssMCB.lvdsStream.cbuffHandle == NULL)
    {
        /* Error: Unable to initialize the CBUFF Driver */
        test_print("Error: CBUFF_open failed with [Error=%d]\n", errCode);
        goto exit;
    }

    /* Initialize the HSI Header Module: */
    if (HSIHeader_init (&initCfg, &errCode) < 0)
    {
        /* Error: Unable to initialize the HSI Header Module */
        test_print("Error: HSIHeader_init failed with [Error=%d]\n", errCode);
        goto exit;
    }

    /* Populate EDMA resources */
    MmwLVDSStream_EDMAInit();

    /* Initialize semaphores */
    status = SemaphoreP_constructBinary(&gMmwMssMCB.lvdsStream.hwFrameDoneSemHandle, 0);
    DebugP_assert(SystemP_SUCCESS == status);

    retVal = 0;

exit:
    return retVal;
}


/**
* High level API for configuring Hw session. Deletes h/w session if it exists,
* configures desired configuration input and activates the h/w session
*/
extern void MmwDemo_configLVDSHwData(uint8_t subFrameIndx)
{
    int32_t retVal;

    /* Delete previous CBUFF HW session if one was configured */
    if(gMmwMssMCB.lvdsStream.hwSessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteHwSession();
    }

    /* Reset the Done Counter */
    gMmwMssMCB.lvdsStream.hwFrameDoneCount =0;

    /* Configure HW session */
    if (MmwLVDSStreamHwConfig(subFrameIndx) < 0)
    {
        test_print("Failed LVDS stream HW configuration\n");
        MmwDemo_debugAssert(0);
    }

    /* If HW LVDS stream is enabled, start the session here so that ADC samples will be
    streamed out as soon as the first chirp samples land on ADC*/
    if(CBUFF_activateSession(gMmwMssMCB.lvdsStream.hwSessionHandle, &retVal) < 0)
    {
        test_print("Failed to activate CBUFF session for LVDS stream HW. errCode=%d\n",retVal);
        MmwDemo_debugAssert(0);
    }
}


/*! \brief This function deletes the hardware session and any HSI
 *      header associated with it.
 */
extern void MmwDemo_LVDSStreamDeleteHwSession (void)
{
    int32_t     errCode;
    MmwDemo_LVDSStream_MCB_t* streamMcb = &gMmwMssMCB.lvdsStream;

    /* Free EDMA Channels */
    MmwLVDSStream_freeDmaChannels(gMmwMssMCB.edmaHandle);

    /* Delete session*/
    if (CBUFF_close (streamMcb->hwSessionHandle, &errCode) < 0)
    {
        /* Error: Unable to delete the session. */
        test_print ("Error: MmwDemo_LVDSStreamDeleteHwSession CBUFF_close failed. Error code %d\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }

    streamMcb->hwSessionHandle = NULL;

    /* Did we stream out with the HSI Header? */
    if (streamMcb->isHwSessionHSIHeaderAllocated == true)
    {
        /* Delete the HSI Header: */
        if (HSIHeader_deleteHeader (streamMcb->ptrHwSessionHSIHeader, &errCode) < 0)
        {
            /* Error: Unable to delete the HSI Header */
            test_print ("Error: MmwDemo_LVDSStreamDeleteHwSession HSIHeader_deleteHeader failed. Error code %d\n", errCode);
            MmwDemo_debugAssert(0);
            return;
        }
        streamMcb->isHwSessionHSIHeaderAllocated = false;
    }
}

#endif