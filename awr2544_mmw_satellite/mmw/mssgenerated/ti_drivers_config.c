/*
 *  Copyright (C) 2021 Texas Instruments Incorporated
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * Auto generated file
 */

#include "ti_drivers_config.h"

/*
 * QSPI
 */


/* QSPI attributes */
static QSPI_Attrs gQspiAttrs[CONFIG_QSPI_NUM_INSTANCES] =
{
    {
        .baseAddr             = CSL_MSS_QSPI_U_BASE,
        .memMapBaseAddr       = CSL_EXT_FLASH_U_BASE,
        .inputClkFreq         = 75000000U,
        .intrNum              = 35U,
        .intrEnable           = FALSE,
        .dmaEnable            = FALSE,
        .intrPriority         = 4U,
        .rxLines              = QSPI_RX_LINES_QUAD,
        .chipSelect           = QSPI_CS0,
        .csPol                = QSPI_CS_POL_ACTIVE_LOW,
        .dataDelay            = QSPI_DATA_DELAY_0,
        .frmFmt               = QSPI_FF_POL0_PHA0,
        .wrdLen               = 8,
        .baudRateDiv          = 0,
    },
};
/* QSPI objects - initialized by the driver */
static QSPI_Object gQspiObjects[CONFIG_QSPI_NUM_INSTANCES];
/* QSPI driver configuration */
QSPI_Config gQspiConfig[CONFIG_QSPI_NUM_INSTANCES] =
{
    {
        &gQspiAttrs[CONFIG_QSPI0],
        &gQspiObjects[CONFIG_QSPI0],
    },
};

uint32_t gQspiConfigNum = CONFIG_QSPI_NUM_INSTANCES;

/*
 * EDMA
 */
/* EDMA atrributes */
static EDMA_Attrs gEdmaAttrs[CONFIG_EDMA_NUM_INSTANCES] =
{
    {

        .baseAddr           = CSL_DSS_TPCC_A_U_BASE,
        .compIntrNumber     = CSL_MSS_INTR_DSS_TPCC_A_INTAGG,
        .intrAggEnableAddr  = CSL_DSS_CTRL_U_BASE + CSL_DSS_CTRL_DSS_TPCC_A_INTAGG_MASK,
        .intrAggEnableMask  = 0x1FF & (~(2U << 0)),
        .intrAggStatusAddr  = CSL_DSS_CTRL_U_BASE + CSL_DSS_CTRL_DSS_TPCC_A_INTAGG_STATUS,
        .intrAggClearMask   = (2U << 0),
        .initPrms           =
        {
            .regionId     = 0,
            .queNum       = 0,
            .initParamSet = FALSE,
            .ownResource    =
            {
                .qdmaCh      = 0x00U,
                .dmaCh[0]    = 0xFFFFFFFFU,
                .dmaCh[1]    = 0xFFFFFFFFU,
                .tcc[0]      = 0xFFFFFFFFU,
                .tcc[1]      = 0xFFFFFFFFU,
                .paramSet[0] = 0xFFFFFFFFU,
                .paramSet[1] = 0xFFFFFFFFU,
                .paramSet[2] = 0xFFFFFFFFU,
                .paramSet[3] = 0xFFFFFFFFU,
            },
            .reservedDmaCh[0]    = 0x00000000U,
            .reservedDmaCh[1]    = 0x00000000U,
        },
    },
    {

        .baseAddr           = CSL_RSS_TPCC_A_U_BASE,
        .compIntrNumber     = CSL_MSS_INTR_RSS_TPCC_A_INTAGG,
        .intrAggEnableAddr  = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_MASK,
        .intrAggEnableMask  = 0x1FF & (~(2U << 1)),
        .intrAggStatusAddr  = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_RSS_TPCC_A_INTAGG_STATUS,
        .intrAggClearMask   = (2U << 1),
        .initPrms           =
        {
            .regionId     = 1,
            .queNum       = 0,
            .initParamSet = FALSE,
            .ownResource    =
            {
                .qdmaCh      = 0x01U,
                .dmaCh[0]    = 0xFFFFFFFFU,
                .dmaCh[1]    = 0xFFFFFFFFU,
                .tcc[0]      = 0xFFFFFFFFU,
                .tcc[1]      = 0xFFFFFFFFU,
                .paramSet[0] = 0xFFFFFFFFU,
                .paramSet[1] = 0xFFFFFFFFU,
                .paramSet[2] = 0xFFFFFFFFU,
                .paramSet[3] = 0xFFFFFFFFU,
            },
            .reservedDmaCh[0]    = 0x00000001U,
            .reservedDmaCh[1]    = 0x00000000U,
        },
    },
};

/* EDMA objects - initialized by the driver */
static EDMA_Object gEdmaObjects[CONFIG_EDMA_NUM_INSTANCES];
/* EDMA driver configuration */
EDMA_Config gEdmaConfig[CONFIG_EDMA_NUM_INSTANCES] =
{
    {
        &gEdmaAttrs[CONFIG_EDMA0],
        &gEdmaObjects[CONFIG_EDMA0],
    },
    {
        &gEdmaAttrs[CONFIG_EDMA1],
        &gEdmaObjects[CONFIG_EDMA1],
    },
};

uint32_t gEdmaConfigNum = CONFIG_EDMA_NUM_INSTANCES;

/*
 * ADCBUF
 */
/* ADCBUF atrributes */
static ADCBuf_Attrs gADCBufAttrs[CONFIG_ADCBUF_NUM_INSTANCES] =
{
    {
        .baseAddr           = CSL_RSS_CTRL_U_BASE,
        .interruptNum       = 159U,
        .adcbufBaseAddr     = CSL_RSS_ADCBUF_READ_U_BASE,
        .cqbufBaseAddr      = CSL_BSS_DFE_CQ1_U_BASE,
    },
};
/* ADCBUF objects - initialized by the driver */
static ADCBuf_Object gADCBufObjects[CONFIG_ADCBUF_NUM_INSTANCES];
/* ADCBUF driver configuration */
ADCBuf_Config gADCBufConfig[CONFIG_ADCBUF_NUM_INSTANCES] =
{
    {
        &gADCBufAttrs[CONFIG_ADCBUF0],
        &gADCBufObjects[CONFIG_ADCBUF0],
    },
};

uint32_t gADCBufConfigNum = CONFIG_ADCBUF_NUM_INSTANCES;

/*
 * CBUFF
 */
/* CBUFF atrributes */
CBUFF_Attrs gCbuffAttrs[CONFIG_CBUFF_NUM_INSTANCES] =
{
    {
        .baseAddr                   = CSL_DSS_CBUFF_U_BASE,
        .fifoBaseAddr               = CSL_DSS_CBUFF_FIFO_U_BASE,
        .adcBufBaseAddr             = CSL_RSS_ADCBUF_READ_U_BASE,
        .maxLVDSLanesSupported      = 2,
        .errorIntrNum               = 144,
        .intrNum                    = 143,
        .chirpModeStartIndex        = 1,
        .chirpModeEndIndex          = 8,
        .cpSingleChirpInterleavedAddr[0] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4,
        .cpSingleChirpInterleavedAddr[1] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4 + 4,
        .cpSingleChirpInterleavedAddr[2] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4 + 8,
        .cpSingleChirpInterleavedAddr[3] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG4 + 12,
        .cpSingleChirpNonInterleavedAddr[0] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0,
        .cpSingleChirpNonInterleavedAddr[1] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0 + 4,
        .cpSingleChirpNonInterleavedAddr[2] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0 + 8,
        .cpSingleChirpNonInterleavedAddr[3] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CPREG0 + 12,
        .cpMultipleChirpNonInterleavedAddr[0] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH0CPREG0,
        .cpMultipleChirpNonInterleavedAddr[1] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH1CPREG0,
        .cpMultipleChirpNonInterleavedAddr[2] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH2CPREG0,
        .cpMultipleChirpNonInterleavedAddr[3] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH3CPREG0,
        .cpMultipleChirpNonInterleavedAddr[4] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH4CPREG0,
        .cpMultipleChirpNonInterleavedAddr[5] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH5CPREG0,
        .cpMultipleChirpNonInterleavedAddr[6] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH6CPREG0,
        .cpMultipleChirpNonInterleavedAddr[7] = CSL_RSS_CTRL_U_BASE + CSL_RSS_CTRL_CH7CPREG0,
        .cbuffChannelId[0]             = 30,
        .cbuffChannelId[1]             = 31,
        .cbuffChannelId[2]             = 32,
        .cbuffChannelId[3]             = 33,
        .cbuffChannelId[4]             = 34,
        .cbuffChannelId[5]             = 35,
        .cbuffChannelId[6]             = 36,
    },
};

/* CBUFF objects - initialized by the driver */
CBUFF_Object gCbuffObject[CONFIG_CBUFF_NUM_INSTANCES];
/* CBUFF objects - storage for CBUFF driver object handles */
CBUFF_Object *gCbuffObjectPtr[CONFIG_CBUFF_NUM_INSTANCES] = { NULL };
/* CBUFF objects count */
uint32_t gCbuffConfigNum = CONFIG_CBUFF_NUM_INSTANCES;

/*
 * ESM
 */
/* ESM atrributes */
static ESM_Attrs gEsmAttrs[CONFIG_ESM_NUM_INSTANCES] =
{
    {
        .ptrESMRegs             = (CSL_esmRegs *)CSL_MSS_ESM_U_BASE,
        .ptrCtrlRegs            = (CSL_mss_ctrlRegs *)CSL_MSS_CTRL_U_BASE,
        .numGroup1Err           = ESM_NUM_INTR_PER_GROUP,
        .highPrioIntNum         = CSL_MSS_INTR_MSS_ESM_HI,
        .lowPrioIntNum          = CSL_MSS_INTR_MSS_ESM_LO,
        .intrHighPriority       = 15U,
        .intrLowPriority        = 8U,
    },
};
/* ESM objects - initialized by the driver */
static ESM_Object gEsmObjects[CONFIG_ESM_NUM_INSTANCES];
/* ESM driver configuration */
ESM_Config gEsmConfig[CONFIG_ESM_NUM_INSTANCES] =
{
    {
        &gEsmAttrs[CONFIG_ESM0],
        &gEsmObjects[CONFIG_ESM0],
    },
};

uint32_t gEsmConfigNum = CONFIG_ESM_NUM_INSTANCES;

/*
 * HWA
 */
/* HWA atrributes */
HWA_Attrs gHwaAttrs[CONFIG_HWA_NUM_INSTANCES] =
{
    {
        .instanceNum                = 0U,
        .ctrlBaseAddr               = CSL_DSS_HWA_CFG_U_BASE,
        .paramBaseAddr              = CSL_DSS_HWA_PARAM_U_BASE,
        .ramBaseAddr                = CSL_DSS_HWA_WINDOW_RAM_U_BASE,
        .dssBaseAddr                = CSL_DSS_CTRL_U_BASE,
        .numHwaParamSets            = SOC_HWA_NUM_PARAM_SETS,
        .intNum1ParamSet            = CSL_MSS_INTR_DSS_HWA_PARAM_DONE_INTR1,
        .intNumDone                 = CSL_MSS_INTR_DSS_HWA_LOOP_INTR1,
        .intNumLocalRamErr          = CSL_MSS_INTR_DSS_HWA_LOCAL_RAM_ERR,
        .numDmaChannels             = SOC_HWA_NUM_DMA_CHANNEL,
        .accelMemBaseAddr           = CSL_DSS_HWA_DMA0_U_BASE,
        .accelMemSize               = SOC_HWA_MEM_SIZE,
        .isConcurrentAccessAllowed  = true,
    },
};

HWA_CommonConfig gHwaCommonConfig[1] =
{
    {
        .configMask = HWA_COMMONCONFIG_MASK_CMP_LFSRSEED0
                        | HWA_COMMONCONFIG_MASK_DCEST_SCALESHIFT
                        | HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM
                        | HWA_COMMONCONFIG_MASK_INTERFSUM_MAG
                        | HWA_COMMONCONFIG_MASK_INTERFSUM_MAGDIFF
                        | HWA_COMMONCONFIG_MASK_INTERF_MITG_WINDOW_PARAM
                        | HWA_COMMONCONFIG_MASK_LFSRSEED
                        | HWA_COMMONCONFIG_MASK_STATEMACHINE_CFG
                        | HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE
                        | HWA_COMMONCONFIG_MASK_TWIDINCR_DELTA_FRAC,
		.numLoops = 384,
		.paramStartIdx = 0,
		.paramStopIdx = 9,
		.fftConfig.twidDitherEnable = HWA_FEATURE_BIT_ENABLE,
		.fftConfig.lfsrSeed = 11,
		.dcEstimateConfig =
        {
            .scale = 256,
            .shift = 0,
            .scaleProfile2 = 256,
            .shiftProfile2 = 0,
        },
        .interfConfig = {
            .sumMagScale = 64,
            .sumMagShift = 0,
            .sumMagDiffScale = 64,
            .sumMagDiffShift = 0,
            .mitigationWindowParam[0] = 0,
            .mitigationWindowParam[1] = 0,
            .mitigationWindowParam[2] = 0,
            .mitigationWindowParam[3] = 0,
            .mitigationWindowParam[4] = 0,
        },
        .complexMultiplyConfig =
        {
            .twiddleDeltaFrac = 0,
        },
        .compressConfig =
        {
            .EGEKparam[0] = 3,
            .EGEKparam[1] = 4,
            .EGEKparam[2] = 5,
            .EGEKparam[3] = 7,
            .EGEKparam[4] = 9,
            .EGEKparam[5] = 11,
            .EGEKparam[6] = 13,
            .EGEKparam[7] = 15,
            .cmpLfsrSeed0 = 11,
        },
    },
};


HWA_ParamConfig gHwaParamConfig[5] =
{
    {
		.triggerMode = HWA_TRIG_MODE_DMA,
		.triggerSrc = 0,
		.accelMode = HWA_ACCELMODE_NONE,
    },
    {
		.triggerMode = HWA_TRIG_MODE_DMA,
		.triggerSrc = 1,
		.accelMode = HWA_ACCELMODE_FFT,
		.source =
        {
            .srcAddr = 0,
            .srcAcnt = 383,
            .srcAIdx = 8,
            .srcBcnt = 3,
            .srcBIdx = 2,
            .srcAcircShift = 0,
            .srcAcircShiftWrap = 0,
            .srcCircShiftWrap3 = HWA_FEATURE_BIT_DISABLE,
            .srcRealComplex = HWA_SAMPLES_FORMAT_REAL,
            .srcWidth = HWA_SAMPLES_WIDTH_16BIT,
            .srcSign = HWA_SAMPLES_SIGNED,
            .srcConjugate = HWA_FEATURE_BIT_DISABLE,
            .srcScale = 8,
            .srcIQSwap = HWA_FEATURE_BIT_DISABLE,
        },
		.dest =
        {
            .dstAddr = 32768,
            .dstAcnt = 383,
            .dstAIdx = 8,
            .dstBIdx = 2,
            .dstRealComplex = HWA_SAMPLES_FORMAT_REAL,
            .dstWidth = HWA_SAMPLES_WIDTH_16BIT,
            .dstSign = HWA_SAMPLES_SIGNED,
            .dstConjugate = HWA_FEATURE_BIT_DISABLE,
            .dstScale = 0,
            .dstSkipInit = 0,
            .dstIQswap = HWA_FEATURE_BIT_DISABLE,
        },
        .accelModeArgs =
        {
            .fftMode =
            {
                .mode2X = HWA_FEATURE_BIT_DISABLE,
                .fftEn = HWA_FEATURE_BIT_DISABLE,
                .dcEstProfileSelect = HWA_DCEST_PROFILE_SELECT_PROFILE0,
                .preProcCfg =
                {
                    .dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET,
                    .dcSubEnable = HWA_FEATURE_BIT_DISABLE,
                    .interfStat.resetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                    .interfLocalize =
                    {
                        .thresholdEnable = HWA_FEATURE_BIT_DISABLE,
                    },
                    .interfMitigation =
                    {
                        .enable = HWA_FEATURE_BIT_DISABLE,
                    },
                    .complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE,
                },
            },
        },
    },
    {
		.triggerMode = HWA_TRIG_MODE_IMMEDIATE,
		.accelMode = HWA_ACCELMODE_FFT,
		.source =
        {
            .srcAddr = 0,
            .srcAcnt = 383,
            .srcAIdx = 8,
            .srcBcnt = 3,
            .srcBIdx = 2,
            .srcAcircShift = 0,
            .srcAcircShiftWrap = 0,
            .srcCircShiftWrap3 = HWA_FEATURE_BIT_DISABLE,
            .srcRealComplex = HWA_SAMPLES_FORMAT_REAL,
            .srcWidth = HWA_SAMPLES_WIDTH_16BIT,
            .srcSign = HWA_SAMPLES_SIGNED,
            .srcConjugate = HWA_FEATURE_BIT_DISABLE,
            .srcScale = 8,
            .srcIQSwap = HWA_FEATURE_BIT_DISABLE,
        },
		.dest =
        {
            .dstAddr = 32768,
            .dstAcnt = 383,
            .dstAIdx = 8,
            .dstBIdx = 2,
            .dstRealComplex = HWA_SAMPLES_FORMAT_REAL,
            .dstWidth = HWA_SAMPLES_WIDTH_16BIT,
            .dstSign = HWA_SAMPLES_SIGNED,
            .dstConjugate = HWA_FEATURE_BIT_DISABLE,
            .dstScale = 0,
            .dstSkipInit = 0,
            .dstIQswap = HWA_FEATURE_BIT_DISABLE,
        },
        .accelModeArgs =
        {
            .fftMode =
            {
                .mode2X = HWA_FEATURE_BIT_DISABLE,
                .fftEn = HWA_FEATURE_BIT_DISABLE,
                .dcEstProfileSelect = HWA_DCEST_PROFILE_SELECT_PROFILE0,
                .preProcCfg =
                {
                    .dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                    .dcSubEnable = HWA_FEATURE_BIT_ENABLE,
                    .dcSubSelect = HWA_DCSUB_SELECT_DCEST,
                    .interfStat.resetMode = HWA_DCEST_INTERFSUM_RESET_MODE_PARAMRESET,
                    .interfLocalize =
                    {
                        .thresholdEnable = HWA_FEATURE_BIT_DISABLE,
                    },
                    .interfMitigation =
                    {
                        .enable = HWA_FEATURE_BIT_DISABLE,
                    },
                    .complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE,
                },
            },
        },
    },
    {
		.triggerMode = HWA_TRIG_MODE_IMMEDIATE,
		.accelMode = HWA_ACCELMODE_FFT,
		.source =
        {
            .srcAddr = 32768,
            .srcAcnt = 383,
            .srcAIdx = 8,
            .srcBcnt = 3,
            .srcBIdx = 2,
            .srcAcircShift = 0,
            .srcAcircShiftWrap = 0,
            .srcCircShiftWrap3 = HWA_FEATURE_BIT_DISABLE,
            .srcRealComplex = HWA_SAMPLES_FORMAT_REAL,
            .srcWidth = HWA_SAMPLES_WIDTH_16BIT,
            .srcSign = HWA_SAMPLES_SIGNED,
            .srcConjugate = HWA_FEATURE_BIT_DISABLE,
            .srcScale = 8,
            .srcIQSwap = HWA_FEATURE_BIT_DISABLE,
        },
		.dest =
        {
            .dstAddr = 0,
            .dstAcnt = 191,
            .dstAIdx = 16,
            .dstBIdx = 4,
            .dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX,
            .dstWidth = HWA_SAMPLES_WIDTH_16BIT,
            .dstSign = HWA_SAMPLES_SIGNED,
            .dstConjugate = HWA_FEATURE_BIT_DISABLE,
            .dstScale = 0,
            .dstSkipInit = 0,
            .dstIQswap = HWA_FEATURE_BIT_DISABLE,
        },
        .accelModeArgs =
        {
            .fftMode =
            {
                .mode2X = HWA_FEATURE_BIT_DISABLE,
                .fftEn = HWA_FEATURE_BIT_ENABLE,
                .fftSize = 7,
                .butterflyScaling = 7,
                .fftSize3xEn = HWA_FEATURE_BIT_ENABLE,
                .windowEn = HWA_FEATURE_BIT_ENABLE,
                .butterflyScalingFFT3x = HWA_FFT3x_BFLY_SCALING_MSBSATURATED,
                .winSymm = HWA_FEATURE_BIT_ENABLE,
                .windowStart = 0,
                .windowMode = HWA_WINDOW_MODE_16BITREAL,
                .dcEstProfileSelect = HWA_DCEST_PROFILE_SELECT_PROFILE0,
                .preProcCfg =
                {
                    .dcEstResetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                    .dcSubEnable = HWA_FEATURE_BIT_DISABLE,
                    .interfStat.resetMode = HWA_DCEST_INTERFSUM_RESET_MODE_NOUPDATE,
                    .interfLocalize =
                    {
                        .thresholdEnable = HWA_FEATURE_BIT_ENABLE,
                        .thresholdMode = HWA_INTERFTHRESH_MODE_MAGDIFF,
                        .thresholdSelect = HWA_INTERFTHRESH_SELECT_EST_INDIVIDUAL,
                    },
                    .interfMitigation =
                    {
                        .enable = HWA_FEATURE_BIT_ENABLE,
                            .countThreshold = 1,
                            .pathSelect = HWA_INTERFMITIGATION_PATH_WINDOWZEROOUT,
                            .leftHystOrder = 2,
                            .rightHystOrder = 2,
                    },
                    .complexMultiply.cmultMode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE,
                },
            },
        },
    },
    {
		.triggerMode = HWA_TRIG_MODE_IMMEDIATE,
		.accelMode = HWA_ACCELMODE_COMPRESS,
		.source =
        {
            .srcAddr = 0,
            .srcAcnt = 7,
            .srcAIdx = 4,
            .srcBcnt = 23,
            .srcBIdx = 96,
            .srcAcircShift = 0,
            .srcAcircShiftWrap = 0,
            .srcCircShiftWrap3 = HWA_FEATURE_BIT_DISABLE,
            .srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX,
            .srcWidth = HWA_SAMPLES_WIDTH_16BIT,
            .srcSign = HWA_SAMPLES_SIGNED,
            .srcConjugate = HWA_FEATURE_BIT_DISABLE,
            .srcScale = 8,
            .srcIQSwap = HWA_FEATURE_BIT_DISABLE,
        },
		.dest =
        {
            .dstAddr = 32768,
            .dstAcnt = 3,
            .dstAIdx = 4,
            .dstBIdx = 48,
            .dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX,
            .dstWidth = HWA_SAMPLES_WIDTH_16BIT,
            .dstSign = HWA_SAMPLES_UNSIGNED,
            .dstConjugate = HWA_FEATURE_BIT_DISABLE,
            .dstScale = 0,
            .dstSkipInit = 0,
            .dstIQswap = HWA_FEATURE_BIT_DISABLE,
        },
        .accelModeArgs =
        {
            .compressMode =
            {
                .compressDecompress = HWA_CMP_DCMP_COMPRESS,
                .method = HWA_COMPRESS_METHOD_EGE,
                .scaleFactorBW = 4,
                .passSelect = HWA_COMPRESS_PATHSELECT_BOTHPASSES,
                .headerEnable = HWA_FEATURE_BIT_ENABLE,
                .ditherEnable = HWA_FEATURE_BIT_ENABLE,
                .cmpRoundEn = HWA_FEATURE_BIT_DISABLE,
                .selLfsr = HWA_FEATURE_BIT_DISABLE,
                .overflowProtection = HWA_FEATURE_BIT_DISABLE,
                .EGEKidx = 0,
                .EGEKarrayLength = 3,
            },
        },
    },
};
/* HWA RAM atrributes */
HWA_RAMAttrs gHwaRamCfg[HWA_NUM_RAMS] =
{
    {CSL_DSS_HWA_WINDOW_RAM_U_BASE, CSL_DSS_HWA_WINDOW_RAM_U_SIZE}
};

/* HWA objects - initialized by the driver */
HWA_Object gHwaObject[CONFIG_HWA_NUM_INSTANCES];
/* HWA objects - storage for HWA driver object handles */
HWA_Object *gHwaObjectPtr[CONFIG_HWA_NUM_INSTANCES] = { NULL };
/* HWA objects count */
uint32_t gHwaConfigNum = CONFIG_HWA_NUM_INSTANCES;

/*
 * IPC Notify
 */
#include <drivers/ipc_notify.h>
#include <drivers/ipc_notify/v1/ipc_notify_v1.h>



/* This function is called within IpcNotify_init, this function returns core specific IPC config */
void IpcNotify_getConfig(IpcNotify_InterruptConfig **interruptConfig, uint32_t *interruptConfigNum)
{
    /* extern globals that are specific to this core */
    extern IpcNotify_InterruptConfig gIpcNotifyInterruptConfig_r5fss0_0[];
    extern uint32_t gIpcNotifyInterruptConfigNum_r5fss0_0;

    *interruptConfig = &gIpcNotifyInterruptConfig_r5fss0_0[0];
    *interruptConfigNum = gIpcNotifyInterruptConfigNum_r5fss0_0;
}

/* This function is called within IpcNotify_init, this function allocates SW queue */
void IpcNotify_allocSwQueue(IpcNotify_MailboxConfig *mailboxConfig)
{
}


/*
 * UART
 */
#include "drivers/soc.h"

/* UART atrributes */
static UART_Attrs gUartAttrs[CONFIG_UART_NUM_INSTANCES] =
{
    {
        .baseAddr           = CSL_MSS_SCIA_U_BASE,
        .inputClkFreq       = 150000000U,
    },
};
/* UART objects - initialized by the driver */
static UART_Object gUartObjects[CONFIG_UART_NUM_INSTANCES];
/* UART driver configuration */
UART_Config gUartConfig[CONFIG_UART_NUM_INSTANCES] =
{
    {
        &gUartAttrs[CONFIG_UART0],
        &gUartObjects[CONFIG_UART0],
    },
};

uint32_t gUartConfigNum = CONFIG_UART_NUM_INSTANCES;

void Drivers_uartInit(void)
{
    uint32_t i;
    for (i=0; i<CONFIG_UART_NUM_INSTANCES; i++)
    {
        SOC_RcmPeripheralId periphID;
        if(gUartAttrs[i].baseAddr == CSL_MSS_SCIA_U_BASE) {
            periphID = SOC_RcmPeripheralId_MSS_SCIA;
        } else if (gUartAttrs[i].baseAddr == CSL_MSS_SCIB_U_BASE) {
            periphID = SOC_RcmPeripheralId_MSS_SCIB;
        } else {
            continue;
        }
        gUartAttrs[i].inputClkFreq = SOC_rcmGetPeripheralClock(periphID);
    }
    UART_init();
}


void Pinmux_init(void);
void PowerClock_init(void);
void PowerClock_deinit(void);

/*
 * Common Functions
 */



void System_init(void)
{
    /* DPL init sets up address transalation unit, on some CPUs this is needed
     * to access SCICLIENT services, hence this needs to happen first
     */
    Dpl_init();

    
    /* initialize PMU */
    CycleCounterP_init(SOC_getSelfCpuClk());


    PowerClock_init();
    /* Now we can do pinmux */
    Pinmux_init();
    /* finally we initialize all peripheral drivers */
    QSPI_init();
    EDMA_init();
    ADCBuf_init(SystemP_WAIT_FOREVER);
    ESM_init();
    HWA_init();
    /* IPC Notify */
    {
        IpcNotify_Params notifyParams;
        int32_t status;

        /* initialize parameters to default */
        IpcNotify_Params_init(&notifyParams);

        /* specify the priority of IPC Notify interrupt */
        notifyParams.intrPriority = 12U;

        /* specify the core on which this API is called */
        notifyParams.selfCoreId = CSL_CORE_ID_R5FSS0_0;

        /* list the cores that will do IPC Notify with this core
        * Make sure to NOT list 'self' core in the list below
        */
        notifyParams.numCores = 0;

        notifyParams.isMailboxIpcEnabled = 1;

        notifyParams.isCrcEnabled = 0;

        /* initialize the IPC Notify module */
        status = IpcNotify_init(&notifyParams);
        DebugP_assert(status==SystemP_SUCCESS);

        { /* Mailbox driver MUST be initialized after IPC Notify init */
            Mailbox_Params mailboxInitParams;

            Mailbox_Params_init(&mailboxInitParams);
            status = Mailbox_init(&mailboxInitParams);
            DebugP_assert(status == SystemP_SUCCESS);
        }
    }

    Drivers_uartInit();
}

void System_deinit(void)
{
    QSPI_deinit();
    EDMA_deinit();
    ADCBuf_deinit();
    ESM_deinit();
    HWA_deinit();
    IpcNotify_deInit();

    UART_deinit();
    PowerClock_deinit();
    Dpl_deinit();
}
