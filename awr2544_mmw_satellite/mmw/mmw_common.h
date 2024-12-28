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
 *  \file   mmw_common.h
 *
 *  \brief  Common Header File For MMW Demo
 */


#ifndef MMW_COMMON_H
#define MMW_COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                            Include FIles                                  */
/* ========================================================================= */

/* MCU+SDK Include Files */
#include "FreeRTOS.h"
#include "task.h"

/* mmWave SDK Include Files */
#include <ti/common/syscommon.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/demo/utils/mmwdemo_adcconfig.h>
#include <ti/demo/utils/mmwdemo_monitor.h>
#include <ti/datapath/dpu/rangeprocReal2x/rangeprochwaReal2x.h>
#include <ti/demo/awr2544/mmw/mmw_res.h>
#include <ti/utils/hsiheader/hsiheader.h>

#include <include/per/cpsw.h>

#ifdef LVDS_STREAM
#include <ti/demo/awr2544/mmw/mmw_lvds/mmw_lvds_stream.h>
#endif

/* Sysconfig Generated Include Files */
#include <ti_drivers_config.h>
#include <ti_drivers_open_close.h>
#include <ti_board_open_close.h>
#include <ti_board_config.h>
#include <ti_enet_open_close.h>
#include <ti_enet_config.h>

/* ========================================================================= */
/*                           Macros & Typedefs                               */
/* ========================================================================= */

/**
 * @defgroup configStoreOffsets     Offsets for storing CLI configuration
 * @brief    Offsets of config fields within the parent structures, note these
 * offsets will be unique and hence can be used to differentiate the commands
 * for processing purposes.
 * @{
 */
#define MMWDEMO_GUIMONSEL_OFFSET (offsetof(MmwDemo_SubFrameCfg, guiMonSel))
#define MMWDEMO_ADCBUFCFG_OFFSET (offsetof(MmwDemo_SubFrameCfg, adcBufCfg))
#ifdef LVDS_STREAM
#define MMWDEMO_LVDSSTREAMCFG_OFFSET (offsetof(MmwDemo_SubFrameCfg, \
                                               lvdsStreamCfg))
#endif
#define MMWDEMO_SUBFRAME_STATICCFG_OFFSET (offsetof(MmwDemo_SubFrameCfg, \
                                                    datapathStaticCfg))
#define MMWDEMO_COMPRESSIONCFG_OFFSET MMWDEMO_SUBFRAME_STATICCFG_OFFSET + \
                                     (offsetof(MmwDemo_datapathCfg, \
                                      compressionCfg))
#define MMWDEMO_INTFMITIGCFG_OFFSET MMWDEMO_SUBFRAME_STATICCFG_OFFSET + \
                                    (offsetof(MmwDemo_datapathCfg, \
                                     intfStatsdBCfg))
/** @}*/ /* configStoreOffsets */

/*! \brief Debug Function */
#define MmwDemo_debugAssert(expression) \
                    {_MmwDemo_debugAssert(expression, __FILE__, __LINE__); \
                                                    DebugP_assert(expression);}

/* This value is written to specific register to gate the clock */
#define MMWDEMO_CLOCK_GATE 0x7

/* Bitmap for GPADC read */
#define CONFIG_GPADC_EXT_VOLT_CHANNEL_BITMAP 0x33

/* HWA Power Saving */
#define MMWDEMO_HWA_POWER_GATE 1U
#define MMWDEMO_HWA_CLOCK_GATE 2U

/*! \brief Datapath Processing Chain (DPC) Handle */
typedef void* DPC_Handle;


/* ========================================================================= */
/*                         Structures and Enums                              */
/* ========================================================================= */

/**
 * @brief
 *  Millimeter Wave Demo Sensor State
 *
 * @details
 *  The enumeration is used to define the sensor states used in mmwDemo
 */
typedef enum MmwDemo_SensorState_e
{
    /*!  @brief Inital state after sensor is initialized.
     */
    MmwDemo_SensorState_INIT = 0,

    /*!  @brief Inital state after sensor is post RF init.
     */
    MmwDemo_SensorState_OPENED,

    /*!  @brief Indicates sensor is started */
    MmwDemo_SensorState_STARTED,

    /*!  @brief  State after sensor has completely stopped */
    MmwDemo_SensorState_STOPPED
}MmwDemo_SensorState;

/**
 * @brief
 *  Millimeter Wave Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_MSS_Stats_t
{
    /*! @brief  Counter to track the number of frame trigger events from BSS */
    uint64_t     frameTriggerReady;

    /*! @brief   Counter which tracks the number of failed calibration reports
     *           The event is triggered by an asynchronous event from the BSS.*/
    uint32_t     failedTimingReports;

    /*! @brief   Counter which tracks the number of calibration reports
     *           received. The event is triggered by an asynchronous event
     *           from the BSS.
     */
    uint32_t     calibrationReports;

     /*! @brief  Counter which tracks the number of sensor stop events
      *          received. The event is triggered by an asynchronous event
      *          from the BSS.
      */
    uint32_t     sensorStopped;

}MmwDemo_MSS_Stats;

#ifdef LVDS_STREAM
/**
 * @brief
 *  LVDS streaming configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  for the LVDS streaming.
 */
typedef struct MmwDemo_LvdsStreamCfg_t
{
    /**
     * @brief  HSI Header enabled/disabled flag. Only applicable for HW
     * streaming. Will be ignored for SW streaming which will always have HSI
     * header.
     */
    bool        isHeaderEnabled;

    /*! HW STREAMING DISABLED */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED   0

    /*! ADC */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC        1

    /*! CP_ADC_CQ */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ  4

    /*! HW streaming data format:
        0-HW STREAMING DISABLED
        1-ADC
        2-Reserved
        3-Reserved
        4-CP_ADC_CQ
    */
    uint8_t     dataFmt;

    /**
     * @brief  SW enabled/disabled flag
     */
    bool        isSwEnabled;
}MmwDemo_LvdsStreamCfg;
#endif

/**
 * @brief
 *  Millimeter Wave Demo Gui Monitor Selection
 *
 * @details
 *  The structure contains the selection for what information is placed to
 *  the output packet, and sent out to GUI. Unless otherwise specified,
 *  if the flag is set to 1, information
 *  is sent out. If the flag is set to 0, information is not sent out.
 *
 */
typedef struct MmwDemo_GuiMonSel_t
{
    /*! @brief   if 1: send the compressed FFT output
     *           if 0: Don't send anything */
    uint8_t        radarCube;
} MmwDemo_GuiMonSel;

/**
 * @brief
 *  Millimeter Wave Demo configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used to execute the Millimeter Wave Demo.
 */
typedef struct MmwDemo_Cfg_t
{
    /*! @brief   mmWave Control Configuration. */
    MMWave_CtrlCfg      ctrlCfg;

    /*! @brief   mmWave Open Configuration. */
    MMWave_OpenCfg      openCfg;

    /*! @brief   Datapath output loggerSetting
                 0 (default): MSS UART logger
                 1: DSS UART logger
     */
    uint8_t              dataLogger;
} MmwDemo_Cfg;

/**
 * @brief
 *  Millimeter Wave Demo Datapath configuration
 *
 * @details
 *  The structure is used to hold datapath related configuration.
 */
typedef struct MmwDemo_datapathCfg_t
{
    /*! @brief     Compression Cfg */
    DPU_RangeProcHWA_CompressionCfg compressionCfg;

    /*! @brief Shift/Scale config for Interf Stats Mag Diff in range DPU */
    DPU_RangeProcHWA_intfStatsdBCfg  intfStatsdBCfg;

} MmwDemo_datapathCfg;

/**
 * @brief Common Configuration independent of sub frame.
 */
typedef struct MmwDemo_PreStartCommonCfg_t
{
    /*! @brief   Number of sub-frames */
    uint8_t numSubFrames;

} MmwDemo_PreStartCommonCfg;


/**
 * @brief Common Configuration independent of sub frame.
 */
typedef struct MmwDemo_DPC_ObjDet_CommonCfg_t
{
    /*! @brief  Processing Chain 1-DDM 0: TDM */
    bool procChain;

    /*! @brief Is 2x mode enabled 1- enabled */
    bool is2xMode;

    /*! @brief Number of Chirps after which CPSW is to be triggered */
    uint16_t ethPktRdyCnt;

    /*! @brief Delay (us) to be added before each packet transfer */
    uint16_t ethPerPktDly;

    /*! @brief Network Packet Buffer CRC selection 1 : 32-bit CRC, 0 : 16-bit CRC */
    uint8_t  nwPktCrcSel;

    /*! @brief pre start common config */
    MmwDemo_PreStartCommonCfg   preStartCommonCfg;
} MmwDemo_DPC_ObjDet_CommonCfg;

/**
 * @brief
 *  Millimeter Wave Demo Data Path Information.
 *
 * @details
 *  The structure is used to hold all the relevant information for
 *  the data path.
 */
typedef struct MmwDemo_SubFrameCfg_t
{
    /*! @brief ADC buffer configuration storage */
    MmwDemo_ADCBufCfg adcBufCfg;

#ifdef LVDS_STREAM
    /*! @brief  LVDS stream configuration */
    MmwDemo_LvdsStreamCfg lvdsStreamCfg;
#endif

    /*! @brief GUI Monitor selection configuration storage from CLI */
    MmwDemo_GuiMonSel guiMonSel;

    /*! @brief  Number of range FFT bins, this is at a minimum the next power of 2 of
                numAdcSamples. If range zoom is supported, this can be bigger than
                the minimum. */
    uint16_t    numRangeBins;

    /*! @brief  Number of Doppler FFT bins, this is at a minimum the next power of 2 of
                numDopplerChirps. If Doppler zoom is supported, this can be bigger
                than the minimum. */
    uint16_t    numDopplerBins;

    /*! @brief  ADCBUF will generate chirp interrupt event every this many chirps - chirpthreshold */
    uint8_t     numChirpsPerChirpEvent;

    /*! @brief  Number of bytes per RX channel, it is aligned to 16 bytes as required by ADCBuf driver  */
    uint32_t    adcBufChanDataSize;

    /*! @brief CQ signal & image band monitor buffer size */
    uint32_t    sigImgMonTotalSize;

    /*! @brief CQ RX Saturation monitor buffer size */
    uint32_t    satMonTotalSize;

    /*! @brief  Number of ADC samples */
    uint16_t    numAdcSamples;

    /*! @brief  Number of chirps per sub-frame */
    uint16_t    numChirpsPerSubFrame;

    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas;

    /*! @brief   Datapath static configuration */
    MmwDemo_datapathCfg     datapathStaticCfg;

} MmwDemo_SubFrameCfg;

/*!
 * @brief
 * Structure holds message stats information from data path.
 *
 * @details
 *  The structure holds stats information. This is a payload of the TLV
 *  message item that holds stats information.
 */
typedef struct MmwDemo_output_message_stats_t
{
    /*! @brief   Interframe processing time in usec */
    uint32_t     interFrameProcessingTime;

    /*! @brief   Transmission time of output detection information in usec */
    uint32_t     transmitOutputTime;

    /*! @brief   Interframe processing margin in usec */
    uint32_t     interFrameProcessingMargin;

    /*! @brief   Interchirp processing margin in usec */
    uint32_t     interChirpProcessingMargin;

    /*! @brief   CPU Load (%) during active frame duration */
    uint32_t     activeFrameCPULoad;

    /*! @brief   CPU Load (%) during inter frame duration */
    uint32_t     interFrameCPULoad;
} MmwDemo_output_message_stats;

/*!
 * @brief
 * Structure holds message stats information from data path.
 *
 * @details
 *  The structure holds stats information. This is a payload of the TLV message item
 *  that holds stats information.
 */
typedef struct MmwDemo_SubFrameStats_t
{
    /*! @brief   Frame processing stats */
    MmwDemo_output_message_stats    outputStats;

    /*! @brief   Dynamic CLI configuration time in usec */
    uint32_t                        pendingConfigProcTime;

    /*! @brief   SubFrame Preparation time on MSS in usec */
    uint32_t                        subFramePreparationTime;
} MmwDemo_SubFrameStats;

/**
 * @brief Task handles storage structure
 */
typedef struct MmwDemo_TaskHandles_t
{
    /*! @brief   MMWAVE Control Task Handle */
    TaskHandle_t    mmwCtrlTask;
    StaticTask_t    mmwCtrlTaskObj;

    /*! @brief   DPC Task Handle */
    TaskHandle_t    mmwDPCTask;
    StaticTask_t    mmwDPCTaskObj;

    /*! @brief   Demo Init Task Handle */
    TaskHandle_t    initTask;
    StaticTask_t    initTaskObj;

#ifdef POWER_MEAS
    /*! @brief   Task for loading task for MSS */
    TaskHandle_t    mssLoadingTaskHandle;
    StaticTask_t    mssLoadingTaskObj;
#endif
} MmwDemo_taskHandles;

/*!
 * @brief
 * Structure holds temperature information from Radar front end.
 *
 * @details
 *  The structure holds temperature stats information.
 */
typedef struct MmwDemo_temperatureStats_t
{

    /*! @brief   retVal from API rlRfTempData_t - can be used to know
                 if values in temperatureReport are valid */
    int32_t        tempReportValid;

    /*! @brief   detailed temperature report - snapshot taken just
                 before shipping data over UART */
    rlRfTempData_t temperatureReport;

} MmwDemo_temperatureStats;

/*!
 * @brief
 * Structure holds calibration save configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration save configuration.
 */
typedef struct MmwDemo_calibDataHeader_t
{
    /*! @brief      Magic word for calibration data header */
    uint32_t 	magic;

    /*! @brief      Header length */
    uint32_t 	hdrLen;

    /*! @brief      mmwLink version */
    rlSwVersionParam_t 	linkVer;

    /*! @brief      RadarSS version */
    rlFwVersionParam_t 	radarSSVer;

    /*! @brief      Data length */
    uint32_t 	dataLen;

    /*! @brief      data padding to make sure calib data is 8 bytes aligned */
    uint32_t      padding;
} MmwDemo_calibDataHeader;

/*!
 * @brief
 * Structure holds calibration save configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration save configuration.
 */
typedef struct MmwDemo_calibCfg_t
{
    /*! @brief      Calibration data header for validation read from flash */
    MmwDemo_calibDataHeader    calibDataHdr;

    /*! @brief      Size of Calibraton data size includng header */
    uint32_t 		sizeOfCalibDataStorage;

    /*! @brief      Enable/Disable calibration save process  */
    uint32_t 		saveEnable;

    /*! @brief      Enable/Disable calibration restore process  */
    uint32_t 		restoreEnable;

    /*! @brief      Flash Offset to restore the data from */
    uint32_t 		flashOffset;
} MmwDemo_calibCfg;

/*!
 * @brief
 * Structure holds calibration restore configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration restore configuration.
 */
typedef struct MmwDemo_calibData_t
{
    /*! @brief      Calibration data header for validation read from flash */
    MmwDemo_calibDataHeader    calibDataHdr;

    /*! @brief      Calibration data */
    rlCalibrationData_t               calibData;

    /*! @brief      Phase shift Calibration data */
    rlPhShiftCalibrationData_t     phaseShiftCalibData;

    /* Future: If more fields are added to this structure or RL definitions
        are changed, please add dummy padding bytes here if size of
        MmwDemo_calibData is not multiple of 8 bytes. */
} MmwDemo_calibData;

/*!
 * @brief
 * Structure holds Spread Spectrum configuration.
 *
 * @details
 *  The structure holds Spread Spectrum configuration.
 */
typedef struct MmwDemo_spreadSpectrumConfig_t
{
    /*! @brief      Moduldation depth in percentage */
    float           modDepth;

    /*! @brief      flag to check the valid Config */
    bool            isEnable;

    /*! @brief      Moduldation rate in KHz */
    uint8_t         modRate;

    /*! @brief      downSpread */
    uint8_t         downSpread;
} MmwDemo_spreadSpectrumConfig;
typedef struct MmwDemo_adcDataDithDelayCfg_t
{
    uint8_t     isDelayEn;
    uint8_t     isDitherEn;
    uint16_t    minDelay;
    uint16_t    ditherVal;
}MmwDemo_adcDataDithDelayCfg;

/*!
 * @brief
 * Structure holds power optimization configuration.
 *
 * @details
 *  The structure holds power optimization configuration given through CLI commands.
 */

typedef struct MmwDemo_powerMeas_t
{
    /*! @brief Flag to Gate HWA clock source. */
    uint32_t                 isHwaDynamicClockGating;

    /*! @brief Flag to Gate HWA clock after frame Processing. */
    uint32_t                 isHwaGateAfterFrameProc;

    /*! @brief MSS Core loading Percentage. */
    uint32_t                 mssLoadingPercent;

    /*! @brief Flag to load MSS core loading. */
    uint32_t                 mssLoading;

    /*! @brief Flag to enable run time calibration. */
    uint32_t                 runCalEn;

    /*! @brief Periodic calibration enable flag. */
    uint32_t                 runCalPeriodEn;

    /*! @brief Periodicity in number of frames. */
    uint32_t                 periodicTimeInFrames;

    /*! @brief Semaphore to trigger MSS loading task. */
    SemaphoreP_Object        frameStartSemaphore;

    /*! @brief FrameStart Time Stamp for triggering MSS Loadig task. */
    uint64_t                 frameStartTimeStamp;
} MmwDemo_powerMeas;

/**
 * @brief
 *  Millimeter Wave Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo.
 */
typedef struct MmwDemo_MSS_MCB_t
{
    /*! @brief      Configuration which is used to execute the demo */
    MmwDemo_Cfg                 cfg;

    /*! @brief      UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;

    /*! @brief      This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*! @brief      DPC Handle, used in datapath processing */
    DPC_Handle                  dpcHandle;

    /*! @brief      ADCBuf driver handle */
    ADCBuf_Handle               adcBufHandle;

    /*! @brief   Handle of the EDMA driver, used for CBUFF */
    EDMA_Handle                 edmaHandle;

    /*! @brief      Object Detection DPC common configuration */
    MmwDemo_DPC_ObjDet_CommonCfg objDetCommonCfg;

    /*! @brief      Object Detection DPC subFrame configuration */
    MmwDemo_SubFrameCfg         subFrameCfg[RL_MAX_SUBFRAMES];

    /*! @brief      sub-frame stats */
    MmwDemo_SubFrameStats       subFrameStats[RL_MAX_SUBFRAMES];

    /*! @brief      Demo Stats */
    MmwDemo_MSS_Stats           stats;

    /*! @brief      Task handle storage */
    MmwDemo_taskHandles         taskHandles;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object           demoInitTaskCompleteSemHandle;

    /*! @brief    Sensor state */
    MmwDemo_SensorState         sensorState;

    /*! @brief   Tracks the number of sensor start */
    uint32_t                    sensorStartCount;

    /*! @brief   Tracks the number of sensor sop */
    uint32_t                    sensorStopCount;

    /*! @brief   CQ monitor configuration - Signal Image band data */
    rlSigImgMonConf_t           cqSigImgMonCfg[RL_MAX_PROFILES_CNT];

    /*! @brief   CQ monitor configuration - Saturation data */
    rlRxSatMonConf_t            cqSatMonCfg[RL_MAX_PROFILES_CNT];

    /*! @brief   Analog monitor bit mask */
    MmwDemo_AnaMonitorCfg       anaMonCfg;

#ifdef LVDS_STREAM
    /*! @brief   this structure is used to hold all the relevant information
         for the mmw demo LVDS stream*/
    MmwDemo_LVDSStream_MCB_t    lvdsStream;
#endif

    /*! @brief   Frame Period based on the CFG file. */
    float                     framePeriod;

    /*! @brief   This structure holds Power measurements related inputs
        like HWA Gating, CPU loading flag etc. */
    MmwDemo_powerMeas         powerMeas;

    /*! @brief   this structure is used to hold all the relevant information
     for the temperature report*/
    MmwDemo_temperatureStats  temperatureStats;

    /*! @brief   Calibration cofiguration for save/restore */
    MmwDemo_calibCfg                calibCfg;

    /*! @brief   this structure is used to hold all the relevant information
     for the Core ADPLL SSC Configuration*/
    MmwDemo_spreadSpectrumConfig     coreAdpllSscCfg;

    /*! @brief   this structure is used to hold all the relevant information
     for the PER ADPLL SSC Configuration*/
    MmwDemo_spreadSpectrumConfig     perAdpllSscCfg;

    /*! @brief   ADC data dithering configuration */
    MmwDemo_adcDataDithDelayCfg  adcDataDithDelayCfg;

    /*! @brief antenna indices in increasing order of phase shift value. */
    uint8_t                      ddmPhaseShiftOrder[SYS_COMMON_NUM_TX_ANTENNAS];

    /*! @brief Flag indicating if DDM Phase Shift orger configuration is
     *         pending to issue sensorStart */
    uint8_t                      isDdmPhaseShiftCfgPending;
} MmwDemo_MSS_MCB;

/*
 * @brief Memory Configuration used during init API
 */
typedef struct MmwDemo_MemCfg_t
{
    /*! @brief   Start address of memory provided by the application
     *           from which DPC will allocate.
     */
    void *addr;

    /*! @brief   Size limit of memory allowed to be consumed by the DPC */
    uint32_t size;
} MmwDemo_MemCfg;


/*
 * @brief Configuration for DPC initialization.
 */
typedef struct MmwDemo_initDPCParams_t
{
   /*! @brief   Handle to the hardware accelerator */
   HWA_Handle hwaHandle;

   /*! @brief   Handle to the EDMA driver. */
   EDMA_Handle edmaHandle;

   /*! @brief L3 RAM configuration. DPC will allocate memory from this
    *         as needed and report the amount of memory consumed */
   MmwDemo_MemCfg L3RamCfg;

   /*! @brief Core Local RAM configuration (e.g L2 for R5F).
    *         DPC will allocate memory from this as needed and report the
    *         amount of memory consumed */
   MmwDemo_MemCfg CoreLocalRamCfg;

}MmwDemo_initDPCParams;

/*
 * @brief Static Configuration that is part of the pre-start configuration.
 */
typedef struct MmwDemo_staticDPCCfg_t
{
    /*! @brief      ADCBuf buffer interface */
    DPIF_ADCBufData  ADCBufData;

    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;

    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas;

    /*! @brief  Number of range FFT bins, this is at a minimum the next power
     * of 2 of @ref DPIF_ADCBufProperty_t::numAdcSamples, in case of complex
     * ADC data,and half that value in case of real only data. If range zoom
     * is supported, this can be bigger than the minimum.
     */
    uint16_t    numRangeBins;

    /*! @brief  Number of bins used in Range FFT Calculation. In case of real
     * only samples, this is twice the number of range bins. Else it is equal * to the number of range bins. */
    uint16_t    numRangeFFTBins;

    /*! @brief  Number of chirps per frame */
    uint16_t    numChirpsPerFrame;

    /*! @brief Number of chirps for Doppler computation purposes. */
    uint16_t    numChirps;

    /*! @brief 1 if 2X Mode is enabled for real data */
    uint16_t    isMode2x;

    /*! @brief  Number of Chirps to be Stored in L3RAM in ecah iteration */
    uint16_t    numChirpsEachIter[RANGEPROCHWA_L3REUSE_MAX_ITERATIONS];

    /*! @brief  Application header size per packet*/
    uint32_t    appHeaderSize;

    /*! @brief  Application footer size per packet*/
    uint32_t    appFooterSize;

    /*! @brief  Number of Payloads per chirp*/
    uint32_t    numPayloads;

    /*! @brief  Radar Cube Data Size including headear and footer payload*/
    uint32_t    radarCubeDataSize;

    /*! @brief  Compression Cfg */
    DPU_RangeProcHWA_CompressionCfg compressionCfg;

    /*! @brief Shift/Scale config for Interf Stats Mag Diff in range DPU */
    DPU_RangeProcHWA_intfStatsdBCfg  intfStatsdBCfg;

} MmwDemo_staticDPCCfg;

/**
 * @brief  Structure related to memory usage as part of
 * \ref @MmwDemo_preStartDPCCfg.
 */
typedef struct MmwDemo_memUsageDPC_t
{
    /*! @brief   number of bytes of L3 memory allocated to be used by DPC */
    uint32_t L3RamTotal;

    /*! @brief  number of bytes of L3 memory used by DPC from the allocated
     *           amount indicated through @ref MmwDemo_initDPCParams */
    uint32_t L3RamUsage;

    /*! @brief  number of bytes of Local memory allocated to be used by DPC */
    uint32_t CoreLocalRamTotal;

    /*! @brief  number of bytes of Local memory used by DPC from the allocated
     *           amount indicated through @ref MmwDemo_initDPCParams */
    uint32_t CoreLocalRamUsage;

    /*! @brief Indicates number of bytes of system heap allocated */
    uint32_t SystemHeapTotal;

    /*! @brief number of bytes of heap used at the end of PreStartCfg */
    uint32_t SystemHeapUsed;

    /*! @brief number of bytes of heap used by DPC at the end of PreStartCfg */
    uint32_t SystemHeapDPCUsed;
} MmwDemo_memUsageDPC;

/*
 * @brief Configuration done before starting execution. Unique to each
 * subrframe.
 */
typedef struct MmwDemo_preStartDPCCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
    uint8_t subFrameNum;

    /*! Static configuration */
    MmwDemo_staticDPCCfg staticCfg;

    /*! Memory usage after the preStartCfg is applied */
    MmwDemo_memUsageDPC memUsage;
} MmwDemo_preStartDPCCfg;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern MmwDemo_MSS_MCB gMmwMssMCB;

/* CPSW Toggle register Values */
extern uint32_t gToggleValPing;
extern uint32_t gToggleValPong;

/**************************************************************************
 ************************* Function Declarations **************************
 **************************************************************************/

/* Function to initialize the CLI. */
extern void MmwDemo_CLIInit(uint8_t taskPriority);

/* Functions to handle the actions need to move the sensor state */
extern int32_t MmwDemo_openSensor(bool isFirstTimeOpen);
extern int32_t MmwDemo_configSensor(void);
extern int32_t MmwDemo_startSensor(void);
extern void MmwDemo_stopSensor(void);

/*! \brief  Utility function to apply configuration to specified sub-frame */
extern void MmwDemo_CfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum);
/*! \brief Spread Spectrum Configuration Function*/
extern void MmwDemo_configSSC(void);

/*! Debug Functions */
extern void _MmwDemo_debugAssert(int32_t expression, const char *file,\
                             int32_t line);

/*! \brief Functions related to Datapath Chain (DPC) */
extern void MmwDPC_configAssistEDMAChannel(EDMA_Handle handle, uint32_t chNum,
                                uint32_t shadowParam, uint32_t eventQueue,
                                DPEDMA_ChanCfg *chanCfg);
extern void MmwDPC_configAssistEDMAChannelThreeLinks(EDMA_Handle handle,
                                uint32_t chNum, uint32_t shadowParam1,
                                uint32_t shadowParam2, uint32_t shadowParam3,
                                uint32_t eventQueue,
                                DPEDMA_3LinkChanCfg *chanCfg);
extern void MmwDPC_frameStartISR(void* arg);
extern DPC_Handle MmwDPC_init(MmwDemo_initDPCParams *ptrInitCfg, \
                                int32_t *errCode);
extern void MmwDPC_start(DPC_Handle handle, int32_t *errCode);
extern void MmwDPC_preStartCommonConfig(DPC_Handle handle,\
                        MmwDemo_PreStartCommonCfg *ptrCommonCfg,\
                        int32_t *errCode);
extern void MmwDPC_preStartConfig(DPC_Handle handle, \
                        MmwDemo_preStartDPCCfg *ptrPreStartCfg, \
                        int32_t *errCode);
extern void MmwDPC_execute(DPC_Handle handle, int32_t *errCode);
extern void MmwDPC_deinit(DPC_Handle handle, int32_t *errCode);
extern void MmwDPC_stop(DPC_Handle handle, int32_t *errCode);


/**
 *  @b Description
 *  @n
 *     Initializes RTI Timer B for HWA wakeup.
 *
 *  @retval   None
 */
void MmwDPC_hwaPGTimerInit(void);



#if defined (POWER_MEAS)
/*! \brief
 * Function that executes Matrix multiplication for user
 * provided loading percentage
 */
void MmwPm_MssLoadingTask(void * args);

/**
 *  @b Description
 *  @n
 *     Initializes CPSW interface and and open CPDMA handle for
 *     Tx and Rx channels. Also etablishes DP83867 ethernet PHY.
 *
 *  @retval   None
 */
void MmwEnet_init(void);

/**
 *  @b Description
 *  @n
 *     Configures Tx Buffer descriptors, initializes NW packet buffer
 *     descriptors and enables FHost CPDMA hardware controlled packet
 *     transmission feature
 *
 *  @param[in]  ptrStaticCfg   Pointer to MmwDemo_staticDPCCfg to
 *                             compute Payload size
 *  @param[in]  nwPktCrcSel    Network Packet Buffer CRC selection
 *                             1 - 32-bit CRC
 *                             0 - 16-bit CRC
 *  @retval   None
 */
void MmwEnet_config(MmwDemo_staticDPCCfg *ptrStaticCfg, uint8_t nwPktCrcSel);

/**
 *  @b Description
 *  @n
 *     Free and dequeue Tx buffer descriptors.
 *
 *  @retval   None
 */
void MmwEnet_freeEthTxPkt(void);

/**
 *  @b Description
 *  @n
 *     prints CPSW Host and MAC port stats.
 *
 *  @param[in]  None
 *
 *  @retval   None
 */
void MmwEnet_showCpswStats(void);

#else

/**
 *  @b Description
 *  @n
 *     Main task for initializing TSN stack with 1D-FFT data transfer.
 *
 *  @param[in]  genfSigPeriod  Freq of GenF (Hz) signal equal to frame
 *              periodicity
 *
 *  @param[in]  enableTsnStack Boolean value to enable / disable TSN stack
 *                             true - Initializes TSN stack
 *                             false - Does not initialize TSN stack
 *
 *  @retval   None
 */
void EnetApp_mainTask(const uint32_t genfSigPeriod, const bool enableTsnStack);

/**
 *  @b Description
 *  @n
 *     prints CPSW Host and MAC port stats.
 *
 *  @param[in]  None
 *
 *  @retval   None
 */
void EnetApp_showCpswStats(void);

/**
 *  @b Description
 *  @n
 *     Configures Tx Buffer descriptors, initializes NW packet buffer
 *     descriptors and enables FHost CPDMA hardware controlled packet
 *     transmission feature
 *
 *  @param[in]  ptrStaticCfg   Pointer to MmwDemo_staticDPCCfg to
 *                             compute Payload size
 *  @param[in]  nwPktCrcSel    Network Packet Buffer CRC selection
 *                             1 - 32-bit CRC
 *                             0 - 16-bit CRC
 *  @retval   None
 */
void EnetApp_config(MmwDemo_staticDPCCfg *ptrStaticCfg, uint8_t nwPktCrcSel);

/**
 *  @b Description
 *  @n
 *     Free and dequeue Tx buffer descriptors.
 *
 *  @retval   None
 */
void EnetApp_freeEthTxPkt(void);
#endif


#ifdef __cplusplus
}
#endif

#endif /* #ifndef MMW_COMMON_H_ */
