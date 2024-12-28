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
 *  \file   mmw_dpc.c
 *
 *  \brief  Datapath Chain Implementation
 */

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

/* MCU+SDK Include Files */
#include <kernel/dpl/TaskP.h>
#include <kernel/dpl/HeapP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <kernel/dpl/TimerP.h>
#include <kernel/dpl/AddrTranslateP.h>
#include <drivers/rti.h>

/* mmWave SDK Include Files */
#include <ti/utils/mathutils/mathutils.h>
#include "../mmw_common.h"


/* ========================================================================= */
/*                           Macros & Typedefs                               */
/* ========================================================================= */

/*! \brief This is supplied at command line when application builds this file.
The resource definitions used by this DPC are prefixed by MMW_DPC_ */
#include APP_RESOURCE_FILE
/*! \brief Radar cube data buffer alignment in bytes. */
#define MMW_DPC_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT \
                            DPU_RANGEPROCHWA_RADARCUBE_BYTE_ALIGNMENT_R5F
/*! \brief Maximum size of window RAM. */
#define MMW_DPC_HWA_MAX_WINDOW_RAM_SIZE_IN_SAMPLES \
                                (CSL_DSS_HWA_WINDOW_RAM_U_SIZE >> 3)
/*! \brief Use Symmetric Window for Range DPU. */
#define MMW_DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
/*! \brief Range FFT Wwindow Type. */
#define MMW_DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE MATHUTILS_WIN_HANNING
/*! \brief Interference Mitigation Window Type. */
#define MMW_DPC_DPU_RANGEPROC_INTERFMITIG_WINDOW_TYPE MATHUTILS_WIN_HANNING
/*! \brief Number of interference mitigation window samples. Used 16 as the
size instead of 14 because mathUtils generates the first and the last samples
as 0, which are not useful. */
#define MMW_DPC_INTFMITIG_WIN_SIZE_TOTAL (16U)
/*! \brief Q Format of interference mitigation window. */
#define MMW_DPC_QFORMAT_RANGEPROC_INTERFMITIG_WINDOW (5U)
/*! \brief User defined heap memory and handle. */
#define MMW_DPC_HEAP_MEM_SIZE (sizeof(MmwDPC_Obj) + 1024U)
/*! \brief Buffer Size to store timing info. */
#define MMW_DPC_TIMING_BUFFER_SIZE 32U
/*! \brief R5F CPU Clock Frequency. */
#define MMW_DPC_TIMING_CPU_CLK_FREQ_KHZ 300000
/*! \brief Enable this to print DPC timing info on sensorStop. */
// #define MMW_DPC_PRINT_DPC_TIMING_INFO
/*! \brief Enable this to declare DPC object globally. */
#define MMW_DPC_DBG_DPC_OBJ

/** @addtogroup DPC_ERROR_CODE
 *  Base error code for the DPC is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/*! \brief Error Code: Invalid argument (such as NULL argument pointer) */
#define MMW_DPC_EINVAL (DP_ERRNO_OBJECTDETECTION_BASE-1)

/*! \brief Error Code: Out of general heap memory */
#define MMW_DPC_ENOMEM (DP_ERRNO_OBJECTDETECTION_BASE-2)

/*! \brief Error Code: Out of L3 RAM during radar cube allocation. */
#define MMW_DPC_ENOMEM__L3_RAM_RADAR_CUBE (DP_ERRNO_OBJECTDETECTION_BASE-3)

/*! \brief   Error Code: Out of HWA Window RAM
 */
#define MMW_DPC_ENOMEM_HWA_WINDOW_RAM (DP_ERRNO_OBJECTDETECTION_BASE-4)

/*! \brief   Error Code: Out of Local RAM for range window coefficients. */
#define MMW_DPC_ENOMEM__CORE_LOCAL_RAM_RANGE_HWA_WINDOW \
                                         (DP_ERRNO_OBJECTDETECTION_BASE-5)

/*! \brief   Error Code: In num range bins. */
#define MMW_DPC_RANGE_BINS_ERR (DP_ERRNO_OBJECTDETECTION_BASE-6)


/* MSS RTI-B Macros */
#define CONFIG_TIMER0                            (0u)
#define CONFIG_TIMER0_BASE_ADDR                  (0x2F7A100u)
#define CONFIG_TIMER0_INT_NUM                    (9u)
#define CONFIG_TIMER0_INPUT_CLK_HZ               (150000000u)
#define CONFIG_TIMER0_INPUT_PRE_SCALER           (1u)
#define CONFIG_TIMER0_USEC_PER_TICK              (1000u)
#define CONFIG_TIMER0_NSEC_PER_TICK              (1000000u)
#define CONFIG_TIMER0_NSEC_PER_TICK_ACTUAL       (1000000u)
#define TIMER_NUM_INSTANCES                      (1u)
#define CONFIG_TIMER0_CLOCK_SRC_MUX_ADDR         (0x210002Cu)
#define CONFIG_TIMER0_CLOCK_SRC_SYSCLK           (0x222u)

/**
 * @}
 */


/* ========================================================================= */
/*                         Structures and Enums                              */
/* ========================================================================= */

/*! \brief Memory pool object to manage memory based on
 * @ref MmwDemo_MemCfg_t.
 */
typedef struct MmwDPC_MemPoolObj_t
{
    /*! @brief Memory configuration */
    MmwDemo_MemCfg cfg;

    /*! @brief   Pool running adress.*/
    uintptr_t currAddr;

    /*! @brief   Pool max address. This pool allows setting address to desired
     *           (e.g for rewinding purposes), so having a running maximum
     *           helps in finding max pool usage
     */
    uintptr_t maxCurrAddr;
} MmwDPC_MemPoolObj;

/*! \brief  The structure is used to hold all the relevant information for
 * each of the sub-frames for the DPC.
 */
typedef struct MmwDPC_SubFrameObj_t
{
    /*! @brief   Pointer to hold Range Proc DPU handle */
    DPU_RangeProcHWA_Handle dpuRangeObj;

    /*! @brief   Range DPU configuration storage */
    DPU_RangeProcHWA_Config rangeCfg;
} MmwDPC_SubFrameObj;


/*! \brief Stats structure to store timing and related information. */
typedef struct MmwDemo_statsDPC_t
{
    /*! @brief   interChirpProcess margin in CPU cyctes */
    uint32_t      interChirpProcessingMargin;

    /*! @brief   Counter which tracks the number of frame start interrupt */
    uint32_t      frameStartIntCounter;

    /*! @brief   Counter which tracks the number of subframe start interrupt */
    uint32_t      subframeStartIntCounter;

    /*! @brief   Frame start CPU time stamp */
    uint32_t      frameStartTimeStamp;

    /*! @brief   Inter-frame start CPU time stamp */
    uint32_t      interFrameStartTimeStamp;

    /*! @brief   Inter-frame end CPU time stamp */
    uint32_t      interFrameEndTimeStamp;

    /*! @brief Sub frame preparation cycles. Note when this is reported as part of
     *         the process result reporting, then it indicates the cycles that took
     *         place in the previous sub-frame/frame for preparing to switch to
     *         the sub-frame that is being reported because switching happens
     *         in the processing of DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED,
     *         which is after the DPC process. */
    uint32_t      subFramePreparationCycles;
} MmwDPC_stats;

/**
 * @brief
 *  Millimeter Object Detection DPC object/instance
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  object detection DPC.
 */
typedef struct MmwDPC_Obj_t
{
    /*! @brief   Handle to the hardware accelerator */
    HWA_Handle hwaHandle;

    /*! @brief   Handle to the EDMA driver. */
    EDMA_Handle edmaHandle;

    /*! @brief   Semaphore Object to signal sensor start. */
    SemaphoreP_Object   dpcExecuteSemHandle;

    /*! @brief   Per sub-frame object */
    MmwDPC_SubFrameObj   subFrameObj[RL_MAX_SUBFRAMES];

    /*! @brief Sub-frame index */
    uint8_t       subFrameIndx;

    /*! @brief  Used for checking that inter-frame (inter-sub-frame) processing
     *          finished on time */
    int32_t       interSubFrameProcToken;

    /*! @brief  Indicates the number of times result exported ioctl was received */
    int32_t       numTimesResultExported;

    /*! @brief L3 ram memory pool object */
    MmwDPC_MemPoolObj    L3RamObj;

    /*! @brief Core Local ram memory pool object */
    MmwDPC_MemPoolObj    CoreLocalRamObj;

    /*! @brief  Common configuration storage. */
    MmwDemo_PreStartCommonCfg commonCfg;

    /*! @brief   Stats structure to capture timing and related information. */
    MmwDPC_stats stats;
} MmwDPC_Obj;

/*! \brief As and when frames come in, these variables will store the actual
 * timestamps. As soon as the last frame processing is completed, they will be
 * replaced by time values in milliseconds, with respect to the start of the
 * first frame of the MMW_DPC_TIMING_BUFFER_SIZE set.
 */
typedef struct MmwDPC_TimingInfo_t
{
    uint32_t frameStartTimes[MMW_DPC_TIMING_BUFFER_SIZE];
    uint32_t rangeEndTimes[MMW_DPC_TIMING_BUFFER_SIZE];
    uint32_t resEndTimes[MMW_DPC_TIMING_BUFFER_SIZE];
    uint32_t rangeEndCnt, frameCnt, resEndCnt;
} MmwDPC_TimingInfo;


/* ========================================================================= */
/*                        Static Function Declaration                        */
/* ========================================================================= */

static void MmwDPC_resetMemPool(MmwDPC_MemPoolObj *pool);
static void *MmwDPC_getMemPool(MmwDPC_MemPoolObj *pool);
static uint32_t MmwDPC_getMemPoolMaxUsage(MmwDPC_MemPoolObj *pool);
static void *MmwDPC_allocMemPool(MmwDPC_MemPoolObj *pool, uint32_t size,
                                uint8_t align);
static uint32_t MmwDPC_getRangeWinGenLen(DPU_RangeProcHWA_Config *cfg);
static void MmwDPC_allocEDMAShadowChannel(EDMA_Handle edmaHandle,
                                uint32_t *param);
static int32_t MmwDPC_rangeConfig(MmwDPC_Obj *objDetObj,
                                MmwDPC_SubFrameObj *subFrmObj,
                                MmwDemo_staticDPCCfg *staticCfg,
                                DPIF_RadarCube *radarCube);
static void MmwDPC_edmaAppFooterCb(Edma_IntrHandle intrHandle,
   void *args);
static void MmwDPC_powerDownHWA(void);
static void MmwDPC_powerUpHWA(void);
static void MmwDPC_reconfigHWA();
static void MmwDPC_hwaPGTimer_isr(void *args);

/* ========================================================================= */
/*                            Global Variables                               */
/* ========================================================================= */

/*! \brief DPC Obj instance */
#ifdef MMW_DPC_DBG_DPC_OBJ
MmwDPC_Obj *MmwDPC_obj;
#endif

/*! \brief Heap Memory */
static uint8_t MmwDPC_heapMem[MMW_DPC_HEAP_MEM_SIZE]
                            __attribute__((aligned(HeapP_BYTE_ALIGNMENT)));

/*! \brief Heap Object */
static HeapP_Object MmwDPC_heapObj;
/*! \brief Structure to store the timining info */
MmwDPC_TimingInfo MmwDPC_timingInfo;

/*! \brief To manually trigger the CPSW transfer */
volatile uint32_t payloadReady = 0;

/*! \brief Interrupt Objet for Footer EDMA */
Edma_IntrObject rangProcIntrObj;

/*! \brief Save HWA Paramset configs before powering down and restore when powered up */
DSSHWACCPARAMRegs paramSave[MMW_RES_DPU_RANGE_NUM_HWA_PARAMSETS]__attribute__((aligned(CacheP_CACHELINE_ALIGNMENT)));;

/*! \brief Timer object for HWA Wakeup */
HwiP_Object gTimerHwiObj[TIMER_NUM_INSTANCES];
uint32_t gTimerBaseAddr[TIMER_NUM_INSTANCES];

/* ========================================================================= */
/*                          Function Definitions                             */
/* ========================================================================= */

static void MmwDPC_resetMemPool(MmwDPC_MemPoolObj *pool)
{
    pool->currAddr = (uintptr_t)pool->cfg.addr;
    pool->maxCurrAddr = pool->currAddr;
}


static void *MmwDPC_getMemPool(MmwDPC_MemPoolObj *pool)
{
    return ((void *)pool->currAddr);
}


static uint32_t MmwDPC_getMemPoolMaxUsage(MmwDPC_MemPoolObj *pool)
{
    return ((uint32_t)(pool->maxCurrAddr - (uintptr_t)pool->cfg.addr));
}


static void *MmwDPC_allocMemPool(MmwDPC_MemPoolObj *pool,
                                     uint32_t size,
                                     uint8_t align)
{
    void *retAddr = NULL;
    uintptr_t addr;
    addr = CSL_MEM_ALIGN(pool->currAddr, align);
    if ((addr + size) <= ((uintptr_t)pool->cfg.addr + pool->cfg.size))
    {
        retAddr = (void *)addr;
        pool->currAddr = addr + size;
        pool->maxCurrAddr = CSL_MAX(pool->currAddr, pool->maxCurrAddr);
    }
    return (retAddr);
}


static uint32_t MmwDPC_getRangeWinGenLen(DPU_RangeProcHWA_Config *cfg)
{
    uint16_t numAdcSamples;
    uint32_t winGenLen;

    numAdcSamples = cfg->staticCfg.ADCBufData.dataProperty.numAdcSamples;

#ifdef MMW_DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
    winGenLen = (numAdcSamples + 1) / 2;
#else
    winGenLen = numAdcSamples;
#endif
    return (winGenLen);
}


static void MmwDPC_genRangeWindow(DPU_RangeProcHWA_Config *cfg)
{
    uint8_t idx;
    /* Symmetric window */
    uint32_t interfMitigWindow[MMW_DPC_INTFMITIG_WIN_SIZE_TOTAL >> 1];

    mathUtils_genWindow((uint32_t *)interfMitigWindow,
                        MMW_DPC_INTFMITIG_WIN_SIZE_TOTAL,
                        MMW_DPC_INTFMITIG_WIN_SIZE_TOTAL >> 1,
                        MMW_DPC_DPU_RANGEPROC_INTERFMITIG_WINDOW_TYPE,
                        MMW_DPC_QFORMAT_RANGEPROC_INTERFMITIG_WINDOW);

    /* Only 5 win samples are supported by the HWA */
    for (idx = 0; idx < DPU_RANGEPROCHWA_INTFMITIG_WIN_SIZE; idx++)
    {
        cfg->hwRes.hwaCfg.hwaInterfMitigWindow
        [DPU_RANGEPROCHWA_INTFMITIG_WIN_SIZE - 1 - idx] =
        (uint8_t)interfMitigWindow[(MMW_DPC_INTFMITIG_WIN_SIZE_TOTAL >> 1) \
        - 2 - idx];
    }

    /* Range FFT window */
    if(cfg->staticCfg.isMode2x)
        /* 16 bit real window */
        mathUtils_genWindowQ15((uint16_t *)cfg->staticCfg.window,
                            cfg->staticCfg.ADCBufData.dataProperty.numAdcSamples,
                            MmwDPC_getRangeWinGenLen(cfg),
                            MMW_DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE,
                            15U);
    else
    {
        /* 18 bit real window */
        mathUtils_genWindow((uint32_t *)cfg->staticCfg.window,
                            cfg->staticCfg.ADCBufData.dataProperty.numAdcSamples,
                            MmwDPC_getRangeWinGenLen(cfg),
                            MMW_DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE,
                            17U);
    }
}


static void MmwDPC_allocEDMAShadowChannel(EDMA_Handle edmaHandle,
                                             uint32_t *param)
{
    int32_t testStatus = SystemP_SUCCESS;
    EDMA_Config *config;
    EDMA_Object *object;
    config = (EDMA_Config *)edmaHandle;
    object = config->object;

    if ((object->allocResource.paramSet[*param / 32] & \
            (1U << *param % 32)) != (1U << *param % 32))
    {
        testStatus = EDMA_allocParam(edmaHandle, param);
        DebugP_assert(testStatus == SystemP_SUCCESS);
    }
    return;
}

/**
 *  @b Description
 *  @n
 *     Function calls EDMA param, channel, tcc allocation.
 *     Datapath assumes paramsetNumber = channelNumber = TCC
 *
 *  @param[in]  handle   EDMA handle
 *  @param[in]  chNum    DMA channel number
 *  @param[in]  shadowParam    DMA shadow paramId
 *  @param[in]  eventQueue    Event queue num
 *  @param[out]  chanCfg    Stores channel configuration
 *  @retval   None
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
extern void MmwDPC_configAssistEDMAChannel(EDMA_Handle handle, uint32_t chNum,
                                    uint32_t shadowParam, uint32_t eventQueue,
                                    DPEDMA_ChanCfg *chanCfg)
{

    DebugP_assert(chanCfg != NULL);
    DPEDMA_allocateEDMAChannel(handle, &chNum, &chNum, &chNum);

    chanCfg->channel = chNum;
    chanCfg->tcc = chNum;
    chanCfg->paramId = chNum;
    chanCfg->shadowPramId = shadowParam;

    MmwDPC_allocEDMAShadowChannel(handle, &shadowParam);

    chanCfg->eventQueue = eventQueue;
    return;
}

/**
 *  @b Description
 *  @n
 *     Function calls EDMA param, channel, tcc allocation with 3 link channels.
 *     Datapath assumes paramsetNumber = channelNumber = TCC
 *
 *  @param[in]  handle   EDMA handle
 *  @param[in]  chNum    DMA channel number
 *  @param[in]  shadowParam1    DMA shadow paramId
 *  @param[in]  shadowParam2    DMA shadow paramId
 *  @param[in]  shadowParam3    DMA shadow paramId
 *  @param[in]  eventQueue    Event queue num
 *  @param[out]  chanCfg    Stores channel configuration
 *  @retval   None
 *
 * \ingroup DPU_RangeProcTest__INTERNAL_FUNCTION
 */
extern void MmwDPC_configAssistEDMAChannelThreeLinks(EDMA_Handle handle,
                                uint32_t chNum, uint32_t shadowParam1,
                                uint32_t shadowParam2, uint32_t shadowParam3,
                                uint32_t eventQueue,
                                DPEDMA_3LinkChanCfg *chanCfg)
{

    DebugP_assert(chanCfg != NULL);
    DPEDMA_allocateEDMAChannel(handle, &chNum, &chNum, &chNum);

    chanCfg->channel = chNum;
    chanCfg->tcc = chNum;
    chanCfg->paramId = chNum;

    MmwDPC_allocEDMAShadowChannel(handle, &shadowParam1);
    MmwDPC_allocEDMAShadowChannel(handle, &shadowParam2);
    MmwDPC_allocEDMAShadowChannel(handle, &shadowParam3);

    chanCfg->ShadowPramId[0] = shadowParam1;
    chanCfg->ShadowPramId[1] = shadowParam2;
    chanCfg->ShadowPramId[2] = shadowParam3;
    chanCfg->eventQueue = eventQueue;

    return;
}

static void MmwDPC_edmaAppFooterCb(Edma_IntrHandle intrHandle,
   void *args)
{
    uint32_t baseAddr, regionId;

    payloadReady++;
    if (payloadReady == gMmwMssMCB.objDetCommonCfg.ethPktRdyCnt)
    {
        baseAddr = EDMA_getBaseAddr(gEdmaHandle[CONFIG_EDMA0]);
        DebugP_assert(baseAddr != 0);

        regionId = EDMA_getRegionId(gEdmaHandle[CONFIG_EDMA0]);
        DebugP_assert(regionId < SOC_EDMA_NUM_REGIONS);

        /* Manual trigger EDMA to copy from L3 to DSS-PP buffer */
        EDMA_enableTransferRegion(baseAddr, regionId, MMW_RES_ENET_EDMAL3OUT_CH, EDMA_TRIG_MODE_MANUAL);
    }
}


static int32_t MmwDPC_rangeConfig(MmwDPC_Obj *objDetObj,
                                      MmwDPC_SubFrameObj *subFrmObj,
                                      MmwDemo_staticDPCCfg *staticCfg,
                                      DPIF_RadarCube *radarCube)
{
    int32_t retVal = 0;
    DPU_RangeProcHWA_Config *rangeCfg = &subFrmObj->rangeCfg;
    DPU_RangeProcHWA_HW_Resources *hwRes = &rangeCfg->hwRes;
    DPU_RangeProcHWA_EDMAInputConfig *edmaIn = &hwRes->edmaInCfg;
    DPU_RangeProcHWA_EDMAOutputConfig *edmaOut = &hwRes->edmaOutCfg;
    DPU_RangeProcHWA_EDMAPacketConfig *edmaPacket = &hwRes->edmaPacketCfg;
    DPU_RangeProcHWA_HwaConfig *hwaCfg = &hwRes->hwaCfg;
    int32_t *windowBuffer;
    uint32_t winGenLen;

    /* static configuration */
    rangeCfg->staticCfg.ADCBufData = staticCfg->ADCBufData;
    rangeCfg->staticCfg.appHeaderSize = staticCfg->appHeaderSize;
    rangeCfg->staticCfg.appFooterSize = staticCfg->appFooterSize;
    rangeCfg->staticCfg.numChirpsPerFrame = staticCfg->numChirpsPerFrame;
    rangeCfg->staticCfg.numRangeBins = staticCfg->numRangeBins;
    rangeCfg->staticCfg.numFFTBins = staticCfg->numRangeFFTBins;
    rangeCfg->staticCfg.numTxAntennas = staticCfg->numTxAntennas;
    rangeCfg->staticCfg.numVirtualAntennas = staticCfg->numVirtualAntennas;
    rangeCfg->staticCfg.isMode2x = staticCfg->isMode2x;
    rangeCfg->staticCfg.numPayloads = staticCfg->numPayloads;
    rangeCfg->staticCfg.numPayloadsPerFrame = (staticCfg->numPayloads * staticCfg->numChirpsPerFrame);
    memcpy(&rangeCfg->staticCfg.numChirpsEachIter[0],
            &staticCfg->numChirpsEachIter[0],
           sizeof(staticCfg->numChirpsEachIter));

    if ((rangeCfg->staticCfg.numRangeBins==(rangeCfg->staticCfg.numFFTBins/2))\
                                            || (rangeCfg->staticCfg.isMode2x))
    {
        rangeCfg->staticCfg.isChirpDataReal = 1;
        rangeCfg->staticCfg.sizeOfInputSample = sizeof(int16_t);
    }
    else if (rangeCfg->staticCfg.numRangeBins
                         == rangeCfg->staticCfg.numFFTBins)
    {
        rangeCfg->staticCfg.isChirpDataReal = 0;
        rangeCfg->staticCfg.sizeOfInputSample = sizeof(cmplx16ImRe_t);
    }
    else
    {
        retVal = MMW_DPC_RANGE_BINS_ERR;
        goto exit;
    }

    memcpy(&rangeCfg->staticCfg.compressionCfg,
           &staticCfg->compressionCfg,
           sizeof(DPU_RangeProcHWA_CompressionCfg));
    memcpy(&rangeCfg->staticCfg.intfStatsCfgdB,
           &staticCfg->intfStatsdBCfg,
           sizeof(DPU_RangeProcHWA_intfStatsdBCfg));

    /* static configuration - windows */
    /* Generating 1D window, allocate first */
    winGenLen = MmwDPC_getRangeWinGenLen(rangeCfg);
    rangeCfg->staticCfg.windowSize = (rangeCfg->staticCfg.isMode2x == 1) ? (winGenLen * sizeof(uint16_t)) : (winGenLen * sizeof(uint32_t));
    windowBuffer = (int32_t *)MmwDPC_allocMemPool(&objDetObj->CoreLocalRamObj,\
    rangeCfg->staticCfg.windowSize, sizeof(uint32_t));
    if (windowBuffer == NULL)
    {
        retVal = MMW_DPC_ENOMEM__CORE_LOCAL_RAM_RANGE_HWA_WINDOW;
        goto exit;
    }
    rangeCfg->staticCfg.window = windowBuffer;
    MmwDPC_genRangeWindow(rangeCfg);

    /* radarCube */
    hwRes->radarCube = *radarCube;
    /* hwres - edma */
    hwRes->edmaHandle = objDetObj->edmaHandle;
    hwRes->radarCubeBufSizeL3 = (radarCube->dataSize \
                     / staticCfg->numChirpsPerFrame) \
                     * staticCfg->numChirpsEachIter[0];

    /* EDMA to copy the Payload Header from bookkeep registers to L3. */
    MmwDPC_configAssistEDMAChannelThreeLinks(hwRes->edmaHandle,
                                MMW_RES_DPU_RANGE_EDMAHEADER_CH,
                                MMW_RES_DPU_RANGE_EDMAHEADER_SHADOW1,
                                MMW_RES_DPU_RANGE_EDMAHEADER_SHADOW2,
                                MMW_RES_DPU_RANGE_EDMAHEADER_SHADOW3,
                                MMW_RES_DPU_RANGE_EDMAHEADER_EVENT_QUE,
                                                 &edmaPacket->appHeader);

    /* We have choosen ISOLATE mode, so we have to fill in dataIn */
    MmwDPC_configAssistEDMAChannel(hwRes->edmaHandle,
                                MMW_RES_DPU_RANGE_EDMAIN_CH,
                                MMW_RES_DPU_RANGE_EDMAIN_SHADOW,
                                MMW_RES_DPU_RANGE_EDMAIN_EVENT_QUE,
                                       &edmaIn->dataIn);

    MmwDPC_configAssistEDMAChannel(hwRes->edmaHandle,
                                MMW_RES_DPU_RANGE_EDMAIN_SIG_CH,
                                MMW_RES_DPU_RANGE_EDMAIN_SIG_SHADOW,
                                MMW_RES_DPU_RANGE_EDMAIN_SIG_EVENT_QUE,
                                       &edmaIn->dataInSignature);

    MmwDPC_configAssistEDMAChannel(hwRes->edmaHandle,
                            MMW_RES_DPU_RANGE_EDMAOUT_SIG_CH,
                            MMW_RES_DPU_RANGE_EDMAOUT_SIG_SHADOW,
                            MMW_RES_DPU_RANGE_EDMAOUT_SIG_EVENT_QUE,
                                       &edmaOut->dataOutSignature);

    /* Ping */
    MmwDPC_configAssistEDMAChannelThreeLinks(hwRes->edmaHandle,
                            MMW_RES_DPU_RANGE_EDMAOUT_PING_CH,
                            MMW_RES_DPU_RANGE_EDMAOUT_PING_SHADOW1,
                            MMW_RES_DPU_RANGE_EDMAOUT_PING_SHADOW2,
                            MMW_RES_DPU_RANGE_EDMAOUT_PING_SHADOW3,
                            MMW_RES_DPU_RANGE_EDMAOUT_PING_EVENT_QUE,
                                                 &edmaOut->dataOutPing);

    /* Pong */
    MmwDPC_configAssistEDMAChannelThreeLinks(hwRes->edmaHandle,
                            MMW_RES_DPU_RANGE_EDMAOUT_PONG_CH,
                            MMW_RES_DPU_RANGE_EDMAOUT_PONG_SHADOW1,
                            MMW_RES_DPU_RANGE_EDMAOUT_PONG_SHADOW2,
                            MMW_RES_DPU_RANGE_EDMAOUT_PONG_SHADOW3,
                            MMW_RES_DPU_RANGE_EDMAOUT_PONG_EVENT_QUE,
                                                 &edmaOut->dataOutPong);

    /* EDMA to copy the Payload Footer from RTI Timestamp registers to L3. */
    MmwDPC_configAssistEDMAChannelThreeLinks(hwRes->edmaHandle,
                                MMW_RES_DPU_RANGE_EDMAFOOTER_CH,
                                MMW_RES_DPU_RANGE_EDMAFOOTER_SHADOW1,
                                MMW_RES_DPU_RANGE_EDMAFOOTER_SHADOW2,
                                MMW_RES_DPU_RANGE_EDMAFOOTER_SHADOW3,
                                MMW_RES_DPU_RANGE_EDMAFOOTER_EVENT_QUE,
                                                 &edmaPacket->appFooter);

#ifdef BSS_LOGGER
    edmaPacket->appFooterCallbackFxn = NULL;
#else
    edmaPacket->appFooterCallbackFxn = MmwDPC_edmaAppFooterCb;
    edmaPacket->edmaCompleteIntrObj = &rangProcIntrObj;
#endif


    /* In this case HWA hardware trigger source is equal to HWA param index
    value*/
    hwaCfg->dataInputMode = DPU_RangeProcHWA_InputMode_ISOLATED;

#ifdef MMW_DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
    hwaCfg->hwaWinSym = HWA_FFT_WINDOW_SYMMETRIC;
#else
    hwaCfg->hwaWinSym = HWA_FFT_WINDOW_NONSYMMETRIC;
#endif
    hwaCfg->hwaWinRamOffset = 0;
    if ((hwaCfg->hwaWinRamOffset + winGenLen) >
    MMW_DPC_HWA_MAX_WINDOW_RAM_SIZE_IN_SAMPLES)
    {
        retVal = MMW_DPC_ENOMEM_HWA_WINDOW_RAM;
        goto exit;
    }

    hwaCfg->numParamSet = MMW_RES_DPU_RANGE_NUM_HWA_PARAMSETS;
    hwaCfg->paramSetStartIdx = MMW_RES_DPU_RANGE_PARAMSET_START_IDX;

    retVal = DPU_RangeProcHWA_config(subFrmObj->dpuRangeObj, rangeCfg);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return retVal;
}


extern void MmwDPC_frameStartISR(void* arg)
{
    CSL_mss_ctrlRegs *ptrMssCtrlRegs = (CSL_mss_ctrlRegs *)CSL_MSS_CTRL_U_BASE;
    CSL_dss_rcmRegs *ptrDssRcmRegs = (CSL_dss_rcmRegs *)CSL_DSS_RCM_U_BASE;
    static bool isfirstFrame = true;
    MmwDPC_Obj *objDetObj = (MmwDPC_Obj *)(arg);
    DebugP_assert(objDetObj != NULL);
    objDetObj->stats.frameStartTimeStamp = ClockP_getTimeUsec();
    MmwDPC_timingInfo.frameStartTimes[MmwDPC_timingInfo.frameCnt %
    MMW_DPC_TIMING_BUFFER_SIZE] = CycleCounterP_getCount32();
    MmwDPC_timingInfo.frameCnt++;

#ifdef POWER_MEAS
    if(gMmwMssMCB.powerMeas.mssLoading)
    {
        SemaphoreP_post(&gMmwMssMCB.powerMeas.frameStartSemaphore);
        gMmwMssMCB.powerMeas.frameStartTimeStamp = objDetObj->stats.frameStartTimeStamp;
    }
#endif

    /* Check if previous frame (sub-frame) processing has completed */
    DebugP_assert(objDetObj->interSubFrameProcToken == 0);

    if(isfirstFrame == false)
    {
#ifndef BSS_LOGGER
        /* Check if count of ping-pong switch events of network
        * packet buffer matching with configured number of payloads
        */
        DebugP_assert(ptrMssCtrlRegs->NW_PACKET_COUNT == (objDetObj->subFrameObj[0].rangeCfg.staticCfg.numPayloadsPerFrame * objDetObj->stats.frameStartIntCounter));
#endif

        /* Ungate HWA Clock based on user configuration */
        if(gMmwMssMCB.powerMeas.isHwaGateAfterFrameProc == MMWDEMO_HWA_CLOCK_GATE)
        {
            CSL_FINS(ptrDssRcmRegs->DSS_HWA_CLK_GATE,
                    DSS_RCM_DSS_HWA_CLK_GATE_DSS_HWA_CLK_GATE_GATED,
                    0);
        }
    }

    /* Start timer for HWA Wakeup when Power gated */
    if(gMmwMssMCB.powerMeas.isHwaGateAfterFrameProc == MMWDEMO_HWA_POWER_GATE)
    {
        TimerP_start(gTimerBaseAddr[CONFIG_TIMER0]);
    }

    isfirstFrame = false;

    /* Check the status of ping-pong select bit */
    if(CSL_FEXT(ptrMssCtrlRegs->HW_REG3, MSS_CTRL_HW_REG3_HW_REG3_RO))
    {
        /* update the source address of CPSW EDMA (Ch 14) to read the pong toggle value */
        DPEDMA_updateAddressAndTrigger(gEdmaHandle[CONFIG_EDMA0],
                                    (uint32_t)&gToggleValPong, /* src addr */
                                    0, /* don't update dest addr */
                                    MMW_RES_ENET_TOGGLE_CH, /* param Id */
                                    false); /* do not trigger */
    }
    else
    {
        /* update the source address of CPSW EDMA (Ch 14) to read the pong toggle value */
        DPEDMA_updateAddressAndTrigger(gEdmaHandle[CONFIG_EDMA0],
                                    (uint32_t)&gToggleValPing, /* src addr */
                                    0, /* don't update dest addr */
                                    MMW_RES_ENET_TOGGLE_CH, /* param Id */
                                    false); /* do not trigger */
    }

    /* Reset the payloadReady - this increments in Footer EDMA transfer Callback */
    payloadReady = 0;

    objDetObj->interSubFrameProcToken++;
    /* Increment interrupt counter for debugging, and reporting purpose */
    if (objDetObj->subFrameIndx == 0)
    {
        (objDetObj->stats.frameStartIntCounter)++;
    }

    (objDetObj->stats.subframeStartIntCounter)++;

    /* Notify the DPC to start the execution */
    SemaphoreP_post(&objDetObj->dpcExecuteSemHandle);
}


extern DPC_Handle MmwDPC_init(MmwDemo_initDPCParams *ptrInitCfg, int32_t *errCode)
{
    volatile uint8_t subFrameIdx = 0U;
    MmwDPC_Obj *objDetObj = NULL;
    MmwDPC_SubFrameObj *subFrmObj = NULL;
    DPU_RangeProcHWA_InitParams rangeInitParams;

    /* Sanity Check */
    if (ptrInitCfg == NULL)
    {
        *errCode = MMW_DPC_EINVAL;
        goto exit;
    }

    /* create heap for RangeProc Hwa object. */
    HeapP_construct(&MmwDPC_heapObj, MmwDPC_heapMem, MMW_DPC_HEAP_MEM_SIZE);
    objDetObj = HeapP_alloc(&MmwDPC_heapObj, sizeof(MmwDPC_Obj));
#ifdef MMW_DPC_DBG_DPC_OBJ
    MmwDPC_obj = objDetObj;
#endif
    if (objDetObj == NULL)
    {
        *errCode = MMW_DPC_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)objDetObj, 0, sizeof(MmwDPC_Obj));

    objDetObj->hwaHandle = ptrInitCfg->hwaHandle;
    objDetObj->edmaHandle = ptrInitCfg->edmaHandle;
    objDetObj->L3RamObj.cfg = ptrInitCfg->L3RamCfg;
    objDetObj->CoreLocalRamObj.cfg = ptrInitCfg->CoreLocalRamCfg;

    /* Create binary semaphore to configure the datapath - when all the configurations are available*/
    SemaphoreP_constructBinary(&objDetObj->dpcExecuteSemHandle, 0);

    rangeInitParams.hwaHandle = ptrInitCfg->hwaHandle;
    for (subFrameIdx = 0; subFrameIdx < RL_MAX_SUBFRAMES; subFrameIdx++)
    {
        subFrmObj = &objDetObj->subFrameObj[subFrameIdx];
        if (subFrmObj != NULL)
        {
            subFrmObj->dpuRangeObj = DPU_RangeProcHWA_init(&rangeInitParams, \
            subFrameIdx, errCode);
            if (*errCode != 0)
            {
                goto exit;
            }
        }
    }

exit:
    if (*errCode != 0)
    {
        if (objDetObj != NULL)
        {
            HeapP_free(&MmwDPC_heapObj, objDetObj);
            HeapP_destruct(&MmwDPC_heapObj);
            objDetObj = NULL;
        }
    }
    return ((DPC_Handle)objDetObj);
}


extern void MmwDPC_start(DPC_Handle handle, int32_t *errCode)
{
    MmwDPC_Obj *objDetObj;
    MmwDPC_SubFrameObj *subFrmObj;

    objDetObj = (MmwDPC_Obj *)handle;
    DebugP_assert(objDetObj != NULL);

    objDetObj->stats.frameStartIntCounter = 0;
    objDetObj->stats.subframeStartIntCounter = 0;
    objDetObj->numTimesResultExported = 0;
    memset((void *)&MmwDPC_timingInfo, 0, sizeof(MmwDPC_TimingInfo));

    /* App must issue export of last frame after stop which will switch to
     * sub-frame 0, so start should always see sub-frame indx of 0, check */
    DebugP_assert(objDetObj->subFrameIndx == 0);

    /* Trigger Range DPU, related to reconfig above */
    subFrmObj = &objDetObj->subFrameObj[objDetObj->subFrameIndx];

    // test_print("HWA Triggered!\n");
    *errCode = DPU_RangeProcHWA_control(subFrmObj->dpuRangeObj,
                                        DPU_RangeProcHWA_Cmd_triggerProc,
                                        NULL, 0);
    if (*errCode < 0)
    {
        return;
    }
}


extern void MmwDPC_preStartCommonConfig(DPC_Handle handle,
                        MmwDemo_PreStartCommonCfg *ptrCommonCfg,
                        int32_t *errCode)
{
    MmwDPC_Obj *objDetObj;

    /* Sanity Check */
    if ((handle == NULL) || (ptrCommonCfg == NULL))
    {
        *errCode = MMW_DPC_EINVAL;
        return;
    }
    objDetObj = (MmwDPC_Obj *)handle;

    objDetObj->commonCfg = *ptrCommonCfg;
}


extern void MmwDPC_preStartConfig(DPC_Handle handle,
                        MmwDemo_preStartDPCCfg *ptrPreStartCfg,
                        int32_t *errCode)
{
    MmwDPC_Obj *objDetObj;
    MmwDPC_SubFrameObj *subFrmObj;
    uint8_t subFrameNum;
    DPIF_RadarCube radarCube;
    uint32_t radarCubeBufSizeL3;

    MmwDemo_staticDPCCfg *staticCfg = &ptrPreStartCfg->staticCfg;
    MmwDemo_memUsageDPC *memUsage = &ptrPreStartCfg->memUsage;
    HeapP_MemStats statsStart;
    HeapP_MemStats statsEnd;

    DSSHWACCRegs *ctrlBaseAddr = (DSSHWACCRegs *)CSL_DSS_HWA_CFG_U_BASE;

    /* Sanity Check */
    if ((handle == NULL) || (ptrPreStartCfg == NULL))
    {
        *errCode = MMW_DPC_EINVAL;
        return;
    }

    /* Get system heap size before preStart configuration */
    HeapP_getHeapStats(&MmwDPC_heapObj, &statsStart);

    objDetObj = (MmwDPC_Obj *)handle;
    subFrameNum = ptrPreStartCfg->subFrameNum;
    subFrmObj = &objDetObj->subFrameObj[subFrameNum];

    MmwDPC_resetMemPool(&objDetObj->L3RamObj);
    MmwDPC_resetMemPool(&objDetObj->CoreLocalRamObj);

    /* L3 allocations - radar cube */
    radarCube.dataSize = staticCfg->radarCubeDataSize;
    radarCubeBufSizeL3 = (radarCube.dataSize / staticCfg->numChirpsPerFrame) \
                         * staticCfg->numChirpsEachIter[0];
    radarCube.data = MmwDPC_allocMemPool(&objDetObj->L3RamObj,
                                radarCubeBufSizeL3,
                                MMW_DPC_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT);
    if (radarCube.data == NULL)
    {
        *errCode = MMW_DPC_ENOMEM__L3_RAM_RADAR_CUBE;
        return;
    }
    radarCube.datafmt = DPIF_RADARCUBE_FORMAT_2;

    /* Enable HWA Dyanamic clock gate */
    if(gMmwMssMCB.powerMeas.isHwaDynamicClockGating)
    {
        CSL_FINSR(ctrlBaseAddr->HWA_ENABLE,
                    HWA_ENABLE_HWA_DYN_CLK_EN_END,
                    HWA_ENABLE_HWA_DYN_CLK_EN_START,
                    (bool)(gMmwMssMCB.powerMeas.isHwaDynamicClockGating));
    }

    *errCode = MmwDPC_rangeConfig(objDetObj, subFrmObj, staticCfg, &radarCube);
    if (*errCode != 0)
    {
        return;
    }

    /* Save HWA Pramsets for reconfiguring it when Powered Up*/
    if(gMmwMssMCB.powerMeas.isHwaGateAfterFrameProc == MMWDEMO_HWA_POWER_GATE)
    {
        memcpy((void*)paramSave, (void*)gHwaAttrs[0].paramBaseAddr, sizeof(paramSave));
    }

    /* Get system heap size after preStart configuration */
    HeapP_getHeapStats(&MmwDPC_heapObj, &statsEnd);

    /* Populate system heap usage */
    memUsage->SystemHeapTotal = MMW_DPC_HEAP_MEM_SIZE;
    memUsage->SystemHeapUsed = MMW_DPC_HEAP_MEM_SIZE - \
                             statsEnd.availableHeapSpaceInBytes;
    memUsage->SystemHeapDPCUsed = statsStart.availableHeapSpaceInBytes - \
    statsEnd.availableHeapSpaceInBytes;
    memUsage->L3RamTotal = objDetObj->L3RamObj.cfg.size;
    memUsage->L3RamUsage = MmwDPC_getMemPoolMaxUsage(&objDetObj->L3RamObj);
    memUsage->CoreLocalRamTotal = objDetObj->CoreLocalRamObj.cfg.size;
    memUsage->CoreLocalRamUsage =
                    MmwDPC_getMemPoolMaxUsage(&objDetObj->CoreLocalRamObj);
}


extern void MmwDPC_execute(DPC_Handle handle, int32_t *errCode)
{
    CSL_dss_rcmRegs *ptrDssRcmRegs = (CSL_dss_rcmRegs *)CSL_DSS_RCM_U_BASE;
    MmwDPC_Obj *objDetObj;
    MmwDPC_SubFrameObj *subFrmObj;
    DPU_RangeProcHWA_OutParams outRangeProc;
    objDetObj = (MmwDPC_Obj *)handle;
    DebugP_assert(objDetObj != NULL);

    subFrmObj = &objDetObj->subFrameObj[objDetObj->subFrameIndx];

    /* Wait for frame start event */
    SemaphoreP_pend(&objDetObj->dpcExecuteSemHandle, SystemP_WAIT_FOREVER);

    gMmwMssMCB.subFrameStats[objDetObj->subFrameIndx].outputStats.interFrameCPULoad = TaskP_loadGetTotalCpuLoad() / 100;

    *errCode = DPU_RangeProcHWA_process(subFrmObj->dpuRangeObj, &outRangeProc);
    if (*errCode != 0)
    {
        return;
    }
    MmwDPC_timingInfo.rangeEndTimes[MmwDPC_timingInfo.rangeEndCnt % \
    MMW_DPC_TIMING_BUFFER_SIZE] = CycleCounterP_getCount32();
    MmwDPC_timingInfo.rangeEndCnt++;

    gMmwMssMCB.subFrameStats[objDetObj->subFrameIndx].outputStats.activeFrameCPULoad = TaskP_loadGetTotalCpuLoad() / 100;

    DebugP_assert(outRangeProc.endOfChirp == true);

    /* Trigger Range DPU for the next frame */
    if(gMmwMssMCB.powerMeas.isHwaGateAfterFrameProc != MMWDEMO_HWA_POWER_GATE)
    {
    *errCode = DPU_RangeProcHWA_control(subFrmObj->dpuRangeObj,
                                        DPU_RangeProcHWA_Cmd_triggerProc,
                                        NULL, 0);
    if (*errCode < 0)
    {
        return;
        }
    }

    /* To ensure no overflow is happening beyond a frame */
    objDetObj->interSubFrameProcToken--;

    /* Power Gate or Clock gate HWA based on user configuration */
    if (gMmwMssMCB.powerMeas.isHwaGateAfterFrameProc == MMWDEMO_HWA_CLOCK_GATE)
    {
        /* Gate HWA Peripheral Clock */
        ptrDssRcmRegs->DSS_HWA_CLK_GATE = 0x7;
    }
    else if(gMmwMssMCB.powerMeas.isHwaGateAfterFrameProc == MMWDEMO_HWA_POWER_GATE)
    {
        MmwDPC_powerDownHWA();
        /* Wait till HWA is Powered Down */
        while(ptrDssRcmRegs->DSS_HWA_PD_STATUS != 0x0);
    }


    /* For rangeProcHwa, interChirpProcessingMargin is not available */
    objDetObj->stats.interChirpProcessingMargin = 0;
    objDetObj->stats.interFrameEndTimeStamp = CycleCounterP_getCount32();

    gMmwMssMCB.subFrameStats[objDetObj->subFrameIndx].outputStats.interFrameProcessingTime =
        (objDetObj->stats.interFrameEndTimeStamp - objDetObj->stats.interFrameStartTimeStamp)/(SOC_getSelfCpuClk()/1000000U); /* In micro seconds */
}


extern void MmwDPC_deinit(DPC_Handle handle, int32_t *errCode)
{
    MmwDPC_Obj *objDetObj = (MmwDPC_Obj *)handle;
    MmwDPC_SubFrameObj *subFrmObj;
    int32_t i;

    if (handle == NULL)
    {
        *errCode = MMW_DPC_EINVAL;
        return;
    }

    for (i = 0; i < RL_MAX_SUBFRAMES; i++)
    {
        subFrmObj = &objDetObj->subFrameObj[i];
        *errCode = DPU_RangeProcHWA_deinit(subFrmObj->dpuRangeObj);
        if (*errCode != 0)
        {
            return;
        }
    }

    HeapP_free(&MmwDPC_heapObj, handle);
    HeapP_destruct(&MmwDPC_heapObj);

    return;
}


extern void MmwDPC_stop(DPC_Handle handle, int32_t *errCode)
{
    MmwDPC_Obj *objDetObj;
    uint32_t i, frame0StartTime;

    objDetObj = (MmwDPC_Obj *)handle;
    DebugP_assert(objDetObj != NULL);

    frame0StartTime = MmwDPC_timingInfo.frameStartTimes[
                    (MmwDPC_timingInfo.frameCnt) % MMW_DPC_TIMING_BUFFER_SIZE];
    for (i = 0; i < MMW_DPC_TIMING_BUFFER_SIZE; i++)
    {
        MmwDPC_timingInfo.frameStartTimes[i] -= frame0StartTime;
        MmwDPC_timingInfo.frameStartTimes[i] /=
        MMW_DPC_TIMING_CPU_CLK_FREQ_KHZ;
        MmwDPC_timingInfo.rangeEndTimes[i] -= frame0StartTime;
        MmwDPC_timingInfo.rangeEndTimes[i] /= MMW_DPC_TIMING_CPU_CLK_FREQ_KHZ;
        MmwDPC_timingInfo.resEndTimes[i] -= frame0StartTime;
        MmwDPC_timingInfo.resEndTimes[i] /= MMW_DPC_TIMING_CPU_CLK_FREQ_KHZ;
    }

#ifdef MMW_DPC_PRINT_DPC_TIMING_INFO
    printf("\n");
    printf("----DPU Timing Info (ms)----\n");
    printf("%10s|%10s|%10s\n", "FrameStart", "RangeEnd", "ResEnd");

    for (i = 0; i < MMW_DPC_TIMING_BUFFER_SIZE; i++)
    {
        printf("%10d|%10d|%10d\n", MmwDPC_timingInfo.frameStartTimes[i],
        MmwDPC_timingInfo.rangeEndTimes[i], MmwDPC_timingInfo.resEndTimes[i]);
    }
    printf("-----------\n");
#endif
}

/**
 *  @b Description
 *  @n
 *      HWA Power Up Sequence
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDPC_powerUpHWA(void)
{
    CSL_dss_rcmRegs *ptrDssRcmRegs = (CSL_dss_rcmRegs *)CSL_DSS_RCM_U_BASE;
    ptrDssRcmRegs->DSS_HWA_PD_CTRL     = 0x07007U;    //PONIN
    ptrDssRcmRegs->DSS_HWA_PD_CTRL     = 0x77007U;    //PGOODIN
    ptrDssRcmRegs->DSS_HWA_PD_CTRL     = 0x77077U;    //AONIN
    ptrDssRcmRegs->DSS_HWA_PD_CTRL     = 0x77777;     //AGOODIN
    ptrDssRcmRegs->DSS_HWA_CLK_GATE    = 0x0U;
    ptrDssRcmRegs->DSS_HWA_CLK_GATE    = 0x7U;
    ptrDssRcmRegs->HW_SPARE_RW0        = 0x0U;
    ptrDssRcmRegs->DSS_HWA_PD_CTRL     = 0x77770U; //Deassert ISO
    ptrDssRcmRegs->DSS_HWA_CLK_GATE    = 0x0U;
}

/**
 *  @b Description
 *  @n
 *      HWA Power Down Sequence
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDPC_powerDownHWA(void)
{
    CSL_dss_rcmRegs *ptrDssRcmRegs = (CSL_dss_rcmRegs *)CSL_DSS_RCM_U_BASE;
    ptrDssRcmRegs->DSS_HWA_CLK_GATE = 0x7;
    ptrDssRcmRegs->DSS_HWA_PD_CTRL = 0x77777;
    ptrDssRcmRegs->HW_SPARE_RW0 = 0x70000;
    ptrDssRcmRegs->DSS_HWA_PD_CTRL = 0x77707;
    ptrDssRcmRegs->DSS_HWA_PD_CTRL = 0x77007;
    ptrDssRcmRegs->DSS_HWA_PD_CTRL = 0x70007;
    ptrDssRcmRegs->DSS_HWA_PD_CTRL = 0x00007;
}

/**
 *  @b Description
 *  @n
 *      Reconfigure HWA after Power Up
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDPC_reconfigHWA()
{
    int32_t retVal = 0;
    MmwDPC_SubFrameObj *subFrmObj;
    DSSHWACCRegs *ctrlBaseAddr = (DSSHWACCRegs *)gHwaObjectPtr[0]->hwAttrs->ctrlBaseAddr;
    MmwDPC_Obj *objDetObj = (MmwDPC_Obj *)gMmwMssMCB.dpcHandle;

    subFrmObj = &objDetObj->subFrameObj[objDetObj->subFrameIndx];

    /* unlock the HWA registers */
    ctrlBaseAddr->LOCK0_KICK0 = 0x01234567U;
    ctrlBaseAddr->LOCK0_KICK1 = 0xFEDCBA8U;


    /*disable accelerator and clock*/
    ctrlBaseAddr->HWA_ENABLE = 0x0U;

    /*leave hw_acc disabled but enable the clock*/
    CSL_FINSR(ctrlBaseAddr->HWA_ENABLE,
                HWA_ENABLE_HWA_CLK_EN_END,
                HWA_ENABLE_HWA_CLK_EN_START,
                0x7U);

    /* clear the PARAM_DONE_SET_STATUS_0 and PARAM_DONE_SET_STATUS_1*/
    ctrlBaseAddr->PARAM_DONE_CLR[0] = 0xFFFFFFFFU;

    /* clear the TRIGGER_SET_STATUS_0 and TRIGGER_SET_STATUS_1*/
    ctrlBaseAddr->TRIGGER_SET_IN_CLR[0] = 0xFFFFFFFFU;

    /* clear the FFTCLIP register */
    ctrlBaseAddr->CLR_FFTCLIP = 0x1U;

    /* clear other clip registers*/
    ctrlBaseAddr->CLR_CLIP_MISC = 0x1U;

    /* Reconfigure window RAM*/
    retVal = HWA_configRam(gHwaHandle[0],
                                HWA_RAM_TYPE_WINDOW_RAM,
                                (uint8_t *)subFrmObj->rangeCfg.staticCfg.window,
                                subFrmObj->rangeCfg.staticCfg.windowSize,   /* size in bytes */
                                subFrmObj->rangeCfg.hwRes.hwaCfg.hwaWinRamOffset * sizeof(uint32_t));
    if (retVal != 0)
    {
        DebugP_assert(0);
    }

    /* Restore HWA paramsets */
    memcpy((void*)gHwaAttrs[0].paramBaseAddr, (void*)paramSave, sizeof(paramSave));

    /* Dynamic clocking bit needs to be reconfigured */
    if(gMmwMssMCB.powerMeas.isHwaDynamicClockGating)
    {
        CSL_FINSR(ctrlBaseAddr->HWA_ENABLE,
                    HWA_ENABLE_HWA_DYN_CLK_EN_END,
                    HWA_ENABLE_HWA_DYN_CLK_EN_START,
                    (bool)(gMmwMssMCB.powerMeas.isHwaDynamicClockGating));
    }
    /* Trigger Range DPU*/
    retVal = DPU_RangeProcHWA_control(subFrmObj->dpuRangeObj, DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0);
    if (retVal < 0)
    {
        /* Not Expected */
        test_print("RangeProc DPU control error %d\n", retVal);
        DebugP_assert(0);
    }

}

/**
 *  @b Description
 *  @n
 *      Registered Timer Callback
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDPC_hwaPGTimer_isr(void *args)
{
    CSL_dss_rcmRegs *ptrDssRcmRegs = (CSL_dss_rcmRegs *)CSL_DSS_RCM_U_BASE;
    TimerP_clearOverflowInt(gTimerBaseAddr[CONFIG_TIMER0]);
    HwiP_clearInt(CONFIG_TIMER0_INT_NUM);
    TimerP_stop(gTimerBaseAddr[CONFIG_TIMER0]);

    /* Power Up HWA and reconfigure */
    MmwDPC_powerUpHWA();
    /* Wait till HWA is powered Up */
    while(ptrDssRcmRegs->DSS_HWA_PD_STATUS != 0xF);
    MmwDPC_reconfigHWA();

    CSL_REG32_WR((volatile uint32_t*)(CONFIG_TIMER0_BASE_ADDR + CSL_RTI_RTIUC0), 0U);
    CSL_REG32_WR((volatile uint32_t*)(CONFIG_TIMER0_BASE_ADDR + CSL_RTI_RTIFRC0), 0U);
    CSL_REG32_WR((volatile uint32_t*)(CONFIG_TIMER0_BASE_ADDR + CSL_RTI_RTICOMP0), 1U);
}

/**
 *  @b Description
 *  @n
 *      Initializes RTI Timer B for HWA wakeup.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDPC_hwaPGTimerInit(void)
{
    HwiP_Params  timerHwiParams;
    int32_t       status;
    TimerP_Params timerParams;

    /* set timer clock source */
    SOC_controlModuleUnlockMMR(SOC_DOMAIN_ID_MSS_RCM, 0);
    *(volatile uint32_t*)AddrTranslateP_getLocalAddr(CONFIG_TIMER0_CLOCK_SRC_MUX_ADDR) = CONFIG_TIMER0_CLOCK_SRC_SYSCLK;
    SOC_controlModuleLockMMR(SOC_DOMAIN_ID_MSS_RCM, 0);

    gTimerBaseAddr[CONFIG_TIMER0] = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_TIMER0_BASE_ADDR);

    TimerP_Params_init(&timerParams);
    timerParams.inputPreScaler = CONFIG_TIMER0_INPUT_PRE_SCALER;
    timerParams.inputClkHz     = CONFIG_TIMER0_INPUT_CLK_HZ;
    timerParams.periodInNsec   = ((uint32_t) ((gMmwMssMCB.framePeriod-0.05) * 1000U * 1000U)); // 50us before next frame start
    timerParams.oneshotMode    = 1;
    timerParams.enableOverflowInt = 1;
    timerParams.enableDmaTrigger  = 0;
    TimerP_setup(gTimerBaseAddr[CONFIG_TIMER0], &timerParams);

    HwiP_Params_init(&timerHwiParams);
    timerHwiParams.intNum = CONFIG_TIMER0_INT_NUM;
    timerHwiParams.callback = MmwDPC_hwaPGTimer_isr;
    timerHwiParams.isPulse = 0;
    timerHwiParams.priority = 7U;
    status = HwiP_construct(&gTimerHwiObj[CONFIG_TIMER0], &timerHwiParams);
    DebugP_assertNoLog(status==SystemP_SUCCESS);
}
