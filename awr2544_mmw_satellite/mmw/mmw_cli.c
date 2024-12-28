/**
 * Copyright (C) 2023-24 Texas Instruments Incorporated
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
 *  \file   mmw_cli.c
 *
 *  \brief  Mmw (Milli-meter wave) DEMO CLI Implementation
 *
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* MCU + SDK Include Files: */
#include <drivers/uart.h>
#include <drivers/gpadc.h>

/* mmWave SDK Include Files: */
#include <ti/common/syscommon.h>
#include <ti/common/mmwavesdk_version.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>

/* Demo Include Files */
#include <ti/demo/awr2544/mmw/mmw_common.h>
#include <ti/demo/utils/mmwdemo_adcconfig.h>
#include <ti/demo/utils/mmwdemo_rfparser.h>

/* ========================================================================= */
/*                        Static Function Declaration                        */
/* ========================================================================= */

/* CLI Extended Command Functions */
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIGetSubframe (int32_t argc, char* argv[], \
                                    int32_t expectedArgc, int8_t* subFrameNum);
static int32_t MmwDemo_CLICompressionCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIIntfMitigCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIProcChainCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIADCDataDitherCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIChirpQualitySigImgMonCfg (int32_t argc, \
                                                                char* argv[]);
static int32_t MmwDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[]);
#ifdef LVDS_STREAM
static int32_t MmwDemo_CLILvdsStreamCfg (int32_t argc, char* argv[]);
#endif
static int32_t MmwDemo_CLIQueryDemoStatus (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICalibDataSaveRestore(int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISSCConfig (int32_t argc, char* argv[]);
static int32_t MmwDemo_UnusedPerClkGate (int32_t argc, char* argv[]);
static int32_t MmwDemo_measurevoltage (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIVMONDis (int32_t argc, char* argv[]);
static int32_t MmwDemo_digTempRead (int32_t argc, char* argv[]);
static int32_t MmwDemo_anaTempRead (int32_t argc, char* argv[]);
static int32_t MmwDemo_HwaGateAfterFrameProc (int32_t argc, char* argv[]);
static int32_t MmwDemo_HwaDynamicClockGating (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIBSSVMONEn (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIBSSVMONEn (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIVMONEn (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIVMONSelfTest (int32_t argc, char* argv[]);
static void mssVDDs3p3UVSelfTest(uint32_t compSel);
static void mssVDDAOsc1p8vUVSelfTest(uint32_t compSel);
static void mssVDD1p2vOVSelfTest(uint32_t compSel, uint32_t srcSel);
static void mssVDD1p2vUVSelfTest(uint32_t compSel, uint32_t srcSel);
static int32_t MmwDemo_showCpswStats(int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIDDMPhaseShiftOrder (int32_t argc, char* argv[]);

#ifdef POWER_MEAS
static int32_t powerMeas_MssLoading (int32_t argc, char* argv[]);
#endif

/* ========================================================================= */
/*                            Global Variables                               */
/* ========================================================================= */

extern CSL_mss_toprcmRegs* CSL_MSS_TOP_RCM_getBaseAddress (void);


/* COnfig structure for temperature read using GPADC */
GPADC_ConfigType gCfgPtrTempRead=
{
    .triggSrc       = GPADC_TRIGG_SRC_SW,
    .convMode       = GPADC_ONESHOT_CONV_MODE,

        .channelConfig[0] =
        {
            .channelID            = 0,
            .isConfigured         = TRUE,
            .isBufferedMode       = TRUE,
            .useLuTable           = FALSE,
        },
        .channelConfig[1] =
        {
            .channelID            = 1,
            .isConfigured         = TRUE,
            .isBufferedMode       = TRUE,
            .useLuTable           = FALSE,
        },
        .channelConfig[2] =
        {
            .channelID            = 2,
            .isConfigured         = TRUE,
            .isBufferedMode       = TRUE,
            .useLuTable           = FALSE,
        },
};

/* Config structure for reading external voltage using GPADC */
GPADC_ConfigType gCfgPtrExtVoltage=
{
    .triggSrc       = GPADC_TRIGG_SRC_SW,
    .convMode       = GPADC_ONESHOT_CONV_MODE,

        .channelConfig[0] =
            {
                .channelID            = 1 - 1,
                .isConfigured         = TRUE,
                .isBufferedMode       = FALSE,
                .useLuTable           = TRUE,
            },
        .channelConfig[1] =
            {
                .channelID            = 2 - 1,
                .isConfigured         = TRUE,
                .isBufferedMode       = FALSE,
                .useLuTable           = TRUE,
            },
        .channelConfig[2] =
            {
                .channelID            = 5 - 1,
                .isConfigured         = TRUE,
                .isBufferedMode       = FALSE,
                .useLuTable           = TRUE,
            },
        .channelConfig[3] =
            {
                .channelID            = 6 - 1,
                .isConfigured         = TRUE,
                .isBufferedMode       = FALSE,
                .useLuTable           = TRUE,
            },
};


/* ========================================================================= */
/*                          Function Definitions                             */
/* ========================================================================= */

static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[])
{
    bool        doReconfig = true;
    int32_t     retVal = 0;

    /*  Only following command syntax will be supported
        sensorStart
        sensorStart 0
    */
    if (argc == 2)
    {
        doReconfig = (bool) atoi (argv[1]);

        if (doReconfig == true)
        {
            CLI_write ("Error: Reconfig is not supported, only argument of 0 is\n"
                       "(do not reconfig, just re-start the sensor) valid\n");
            return -1;
        }
    }
    else
    {
        /* In case there is no argument for sensorStart, always do reconfig */
        doReconfig = true;
    }

    /*
     * Check for below error condition:
     *      Proc Chain selected as DDM and
     *      DDM Phase Shift order CLI input not available
     */
    if ((gMmwMssMCB.objDetCommonCfg.procChain == 1) &&
         (gMmwMssMCB.isDdmPhaseShiftCfgPending == 0))
    {
        CLI_write ("Error: DDM Proc chain requires ddmPhaseShiftAntOrder CLI input\r\n");
        return -1;
    }

    /***********************************************************************************
     * Spread SPectrum Configuration
     ***********************************************************************************/
    MmwDemo_configSSC();

    /***********************************************************************************
     * Do sensor state management to influence the sensor actions
     ***********************************************************************************/

    /* Error checking initial state: no partial config is allowed
       until the first sucessful sensor start state */
    if ((gMmwMssMCB.sensorState == MmwDemo_SensorState_INIT) ||
         (gMmwMssMCB.sensorState == MmwDemo_SensorState_OPENED))
    {
        MMWave_CtrlCfg ctrlCfg;

        /* need to get number of sub-frames so that next function to check
         * pending state can work */
        CLI_getMMWaveExtensionConfig (&ctrlCfg);
        gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
            MmwDemo_RFParser_getNumSubFrames(&ctrlCfg);
    }

    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: Sensor is already started\n");
        return 0;
    }

    if (doReconfig == true)
    {
        /* User intends to issue sensor start with full config, check if all config
           was issued after stop and generate error if  is the case. */
        MMWave_CtrlCfg ctrlCfg;

        /* need to get number of sub-frames so that next function to check
         * pending state can work */
        CLI_getMMWaveExtensionConfig (&ctrlCfg);
        gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
            MmwDemo_RFParser_getNumSubFrames(&ctrlCfg);
    }

    /***********************************************************************************
     * Retreive and check mmwave Open related config before calling openSensor
     ***********************************************************************************/

    /*  Fill demo's MCB mmWave openCfg structure from the CLI configs*/
    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_INIT)
    {
        /* Get the open configuration: */
        CLI_getMMWaveExtensionOpenConfig (&gMmwMssMCB.cfg.openCfg);
        /* call sensor open */
        retVal = MmwDemo_openSensor(true);
        if(retVal != 0)
        {
            return -1;
        }
        gMmwMssMCB.sensorState = MmwDemo_SensorState_OPENED;
    }
    else
    {
        /* openCfg related configurations like chCfg, lowPowerMode, adcCfg
         * are only used on the first sensor start. If they are different
         * on a subsequent sensor start, then generate a fatal error
         * so the user does not think that the new (changed) configuration
         * takes effect, the board needs to be reboot for the new
         * configuration to be applied.
         */
        MMWave_OpenCfg openCfg;
        CLI_getMMWaveExtensionOpenConfig (&openCfg);
        /* Compare openCfg->chCfg*/
        if(memcmp((void *)&gMmwMssMCB.cfg.openCfg.chCfg, (void *)&openCfg.chCfg,
                          sizeof(rlChanCfg_t)) != 0)
        {
            MmwDemo_debugAssert(0);
        }

        /* Compare openCfg->lowPowerMode*/
        if(memcmp((void *)&gMmwMssMCB.cfg.openCfg.lowPowerMode, (void *)&openCfg.lowPowerMode,
                          sizeof(rlLowPowerModeCfg_t)) != 0)
        {
            MmwDemo_debugAssert(0);
        }
        /* Compare openCfg->adcOutCfg*/
        if(memcmp((void *)&gMmwMssMCB.cfg.openCfg.adcOutCfg, (void *)&openCfg.adcOutCfg,
                          sizeof(rlAdcOutCfg_t)) != 0)
        {
            MmwDemo_debugAssert(0);
        }
    }

    /***********************************************************************************
     * Retrieve mmwave Control related config before calling startSensor
     ***********************************************************************************/
    /* Get the mmWave ctrlCfg from the CLI mmWave Extension */
    if(doReconfig)
    {
        /* if MmwDemo_openSensor has non-first time related processing, call here again*/
        /* call sensor config */
        CLI_getMMWaveExtensionConfig (&gMmwMssMCB.cfg.ctrlCfg);
        retVal = MmwDemo_configSensor();
        if(retVal != 0)
        {
            return -1;
        }
    }
    retVal = MmwDemo_startSensor();
    if(retVal != 0)
    {
        return -1;
    }

    /***********************************************************************************
     * Set the state
     ***********************************************************************************/
    gMmwMssMCB.sensorState = MmwDemo_SensorState_STARTED;
    return 0;
}

static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[])
{
    if ((gMmwMssMCB.sensorState == MmwDemo_SensorState_STOPPED) ||
        (gMmwMssMCB.sensorState == MmwDemo_SensorState_INIT) ||
        (gMmwMssMCB.sensorState == MmwDemo_SensorState_OPENED))
    {
        CLI_write ("Ignored: Sensor is already stopped\r\n");
        return 0;
    }

    MmwDemo_stopSensor();

    gMmwMssMCB.sensorState = MmwDemo_SensorState_STOPPED;
    return 0;
}

static int32_t MmwDemo_CLIGetSubframe (int32_t argc, char* argv[],
                                       int32_t expectedArgc,
                                       int8_t* subFrameNum)
{
    int8_t subframe;

    /* Sanity Check: Minimum argument check */
    if (argc != expectedArgc)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /*Subframe info is always in position 1*/
    subframe = (int8_t) atoi(argv[1]);

    if(subframe >= (int8_t)RL_MAX_SUBFRAMES)
    {
        CLI_write ("Error: Subframe number is invalid\n");
        return -1;
    }

    *subFrameNum = (int8_t)subframe;

    return 0;
}

static int32_t MmwDemo_CLICompressionCfg (int32_t argc, char* argv[])
{
    DPU_RangeProcHWA_CompressionCfg   compressionCfg;
    int8_t                            subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&compressionCfg, 0, sizeof(compressionCfg));

    /* Populate configuration: */
    compressionCfg.isEnabled              = (bool) atoi (argv[2]);
    compressionCfg.compressionMethod      = (uint8_t) atoi (argv[3]);
    compressionCfg.compressionRatio       = (float) atof (argv[4]);
    compressionCfg.rangeBinsPerBlock      = (uint16_t) atoi (argv[5]);
    /* rxAntennasPerBlock will be fixed to the number of Rx antennas */

    /* is it a power of 2 and greater > 1? */
    if ((compressionCfg.rangeBinsPerBlock <=1)||((compressionCfg.rangeBinsPerBlock & (compressionCfg.rangeBinsPerBlock - 1)) != 0))
    {
        CLI_write("Error: rangeBinsPerBlock should be greater than 1 and a power of 2 \n");
        return -1;
    }

    MmwDemo_CfgUpdate((void *)&compressionCfg, MMWDEMO_COMPRESSIONCFG_OFFSET,
                          sizeof(compressionCfg), subFrameNum);

    return 0;
}

static int32_t MmwDemo_CLIIntfMitigCfg (int32_t argc, char* argv[])
{

    DPU_RangeProcHWA_intfStatsdBCfg  intfStatsdBCfg;
    int8_t                                 subFrameNum;

    if(MmwDemo_CLIGetSubframe(argc, argv, 4, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&intfStatsdBCfg, 0, sizeof(intfStatsdBCfg));

    /* Populate configuration: */
    intfStatsdBCfg.intfMitgMagSNRdB               = (uint32_t) atoi (argv[2]);
    intfStatsdBCfg.intfMitgMagDiffSNRdB           = (uint32_t) atoi (argv[3]);

    MmwDemo_CfgUpdate((void *)&intfStatsdBCfg, MMWDEMO_INTFMITIGCFG_OFFSET,
                          sizeof(intfStatsdBCfg), subFrameNum);

    return 0;
}

static int32_t MmwDemo_CLIProcChainCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.objDetCommonCfg.procChain    = (bool) atoi (argv[1]);
    gMmwMssMCB.objDetCommonCfg.is2xMode     = (bool) atoi (argv[2]);
    gMmwMssMCB.objDetCommonCfg.ethPktRdyCnt = (uint16_t) atoi (argv[3]);
    gMmwMssMCB.objDetCommonCfg.ethPerPktDly = (uint16_t) atoi (argv[4]);
    gMmwMssMCB.objDetCommonCfg.nwPktCrcSel  = (uint8_t) atoi (argv[5]);

    return 0;
}

static int32_t MmwDemo_CLIADCDataDitherCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    gMmwMssMCB.adcDataDithDelayCfg.isDelayEn = (bool) atoi (argv[1]);

    if(gMmwMssMCB.adcDataDithDelayCfg.isDelayEn)
    {
        /* 1 LSB in "RSS_CTRL::ADCBUFCFG1_EXTD_ADCBUFINTGENDLY" = 3 Clocks = 20 ns Delay (SYS_CLK = 150MHz)*/
        gMmwMssMCB.adcDataDithDelayCfg.minDelay = 55U; /* MinDelay = 1.1us = 55 LSBs*/
    }
    gMmwMssMCB.adcDataDithDelayCfg.isDitherEn = (bool) atoi (argv[2]);

    if(gMmwMssMCB.adcDataDithDelayCfg.isDitherEn)
    {
        /* Convert the input dither from (us) to units of 3 Clock LSBs*/
        gMmwMssMCB.adcDataDithDelayCfg.ditherVal = (uint16_t)((atof(argv[3])/20.0)*1e3);
        if(gMmwMssMCB.adcDataDithDelayCfg.ditherVal == 0U)
        {
            gMmwMssMCB.adcDataDithDelayCfg.ditherVal = 3;
            CLI_write ("Error: Incorrect Dither Value. \n");
            return -1;
        }
    }

    return 0;
}

static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[])
{
    MmwDemo_ADCBufCfg   adcBufCfg;
    int8_t              subFrameNum;

    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    if(MmwDemo_CLIGetSubframe(argc, argv, 6, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcBufCfg, 0, sizeof(adcBufCfg));

    /* Populate configuration: */
    adcBufCfg.adcFmt          = (uint8_t) atoi (argv[2]);
    adcBufCfg.iqSwapSel       = (uint8_t) atoi (argv[3]);
    adcBufCfg.chInterleave    = (uint8_t) atoi (argv[4]);
    adcBufCfg.chirpThreshold  = (uint8_t) atoi (argv[5]);

    /* This demo is using HWA for 1D processing which does not allow multi-chirp
     * processing */
    if (adcBufCfg.chirpThreshold != 1)
    {
        CLI_write("Error: chirpThreshold must be 1, multi-chirp is not allowed\n");
        return -1;
    }

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&adcBufCfg,
                      MMWDEMO_ADCBUFCFG_OFFSET,
                      sizeof(MmwDemo_ADCBufCfg), subFrameNum);
    return 0;
}

static int32_t MmwDemo_CLIChirpQualityRxSatMonCfg (int32_t argc, char* argv[])
{
    rlRxSatMonConf_t        cqSatMonCfg;

    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 6)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSatMonCfg, 0, sizeof(rlRxSatMonConf_t));

    /* Populate configuration: */
    cqSatMonCfg.profileIndx                 = (uint8_t) atoi (argv[1]);

    if(cqSatMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {

        cqSatMonCfg.satMonSel                   = (uint8_t) atoi (argv[2]);
        cqSatMonCfg.primarySliceDuration        = (uint16_t) atoi (argv[3]);
        cqSatMonCfg.numSlices                   = (uint16_t) atoi (argv[4]);
        cqSatMonCfg.rxChannelMask               = (uint8_t) atoi (argv[5]);

        /* Save Configuration to use later */
        gMmwMssMCB.cqSatMonCfg[cqSatMonCfg.profileIndx] = cqSatMonCfg;

        return 0;
    }
    else
    {
        return -1;
    }
}


static int32_t MmwDemo_CLIChirpQualitySigImgMonCfg (int32_t argc, char* argv[])
{
    rlSigImgMonConf_t       cqSigImgMonCfg;

    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cqSigImgMonCfg, 0, sizeof(rlSigImgMonConf_t));

    /* Populate configuration: */
    cqSigImgMonCfg.profileIndx              = (uint8_t) atoi (argv[1]);

    if(cqSigImgMonCfg.profileIndx < RL_MAX_PROFILES_CNT)
    {
        cqSigImgMonCfg.numSlices            = (uint8_t) atoi (argv[2]);
        cqSigImgMonCfg.timeSliceNumSamples  = (uint16_t) atoi (argv[3]);

        /* Save Configuration to use later */
        gMmwMssMCB.cqSigImgMonCfg[cqSigImgMonCfg.profileIndx] = cqSigImgMonCfg;

        return 0;
    }
    else
    {
        return -1;
    }
}


static int32_t MmwDemo_CLIAnalogMonitorCfg (int32_t argc, char* argv[])
{
    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Save Configuration to use later */
    gMmwMssMCB.anaMonCfg.rxSatMonEn = atoi (argv[1]);
    gMmwMssMCB.anaMonCfg.sigImgMonEn = atoi (argv[2]);
    gMmwMssMCB.anaMonCfg.apllLdoSCMonEn = atoi (argv[3]);
    return 0;
}

#ifdef LVDS_STREAM
static int32_t MmwDemo_CLILvdsStreamCfg (int32_t argc, char* argv[])
{
    MmwDemo_LvdsStreamCfg   cfg;
    int8_t                  subFrameNum;

    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    if(MmwDemo_CLIGetSubframe(argc, argv, 5, &subFrameNum) < 0)
    {
        return -1;
    }

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_LvdsStreamCfg));

    /* Populate configuration: */
    cfg.isHeaderEnabled = (bool)    atoi(argv[2]);
    cfg.dataFmt         = (uint8_t) atoi(argv[3]);
    cfg.isSwEnabled     = (bool)    atoi(argv[4]);

    /* If both h/w and s/w are enabled, HSI header must be enabled, because
     * we don't allow mixed h/w session without HSI header
     * simultaneously with s/w session with HSI header (s/w session always
     * streams HSI header) */
    if ((cfg.isSwEnabled == true) && (cfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED))
    {
        if (cfg.isHeaderEnabled == false)
        {
            CLI_write("Error: header must be enabled when both h/w and s/w streaming are enabled\n");
            return -1;
        }
    }

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&cfg,
                      MMWDEMO_LVDSSTREAMCFG_OFFSET,
                      sizeof(MmwDemo_LvdsStreamCfg), subFrameNum);

    return 0;
}
#endif

static int32_t MmwDemo_CLIQueryDemoStatus (int32_t argc, char* argv[])
{
    CLI_write ("Sensor State: %d\n",gMmwMssMCB.sensorState);
    return 0;
}


static int32_t MmwDemo_CLICalibDataSaveRestore(int32_t argc, char* argv[])
{
    if (gMmwMssMCB.sensorState == MmwDemo_SensorState_STARTED)
    {
        CLI_write ("Ignored: This command is not allowed after sensor has started\n");
        return 0;
    }

    /* Validate inputs */
    if ( ((uint32_t) atoi(argv[1]) == 1) && ((uint32_t) atoi(argv[2] ) == 1))
    {
        CLI_write ("Error: Save and Restore can be enabled only one at a time\n");
        return -1;
    }

    /* Populate configuration: */
    gMmwMssMCB.calibCfg.saveEnable = (uint32_t) atoi(argv[1]);
    gMmwMssMCB.calibCfg.restoreEnable = (uint32_t) atoi(argv[2]);
    sscanf(argv[3], "0x%x", &gMmwMssMCB.calibCfg.flashOffset);

    return 0;
}

static void mssVDD1p2vUVSelfTest(uint32_t compSel, uint32_t srcSel)
{
    CSL_mss_toprcmRegs *ptrMssTopRcmReg = CSL_MSS_TOP_RCM_getBaseAddress();

    uint32_t comp = CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL);
    uint32_t src = CSL_FEXT(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL);

    /* Set Reference for Self Test Mode */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL, srcSel); //0);

    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL, compSel); //3);

    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_UV_SELF_TEST_SEL, 1);

    /* Allow reference to change */
    ClockP_usleep(2);

    /* Enable VMON. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN, 1);

    /* Allow VMON to trip */
    ClockP_usleep(3);

    if(CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_UVDET_LAT))
    {
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN, 0);
        CLI_write("VDD1.2 UVtest Passed\n");
    }
    else
    {
        CLI_write("VDD1.2 UV test Failed\n");
    }

    /* Disable VMON. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN, 0);

    /* Disables Self Test Mode. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_UV_SELF_TEST_SEL, 0);

    /* Set Reference for Normal Mode. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL, comp);

    /* Set Reference for Normal Mode. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL, src);

    /* Allow VMON to reset. */
    ClockP_usleep(2);

    /* Re-enable VMON */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN, 1);

    return;
}


static void mssVDD1p2vOVSelfTest(uint32_t compSel, uint32_t srcSel)
{
    CSL_mss_toprcmRegs *ptrMssTopRcmReg = CSL_MSS_TOP_RCM_getBaseAddress();

    uint32_t comp = CSL_FEXT(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL);
    uint32_t src = CSL_FEXT(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL);

    /* Set Reference for Self Test Mode */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL, compSel); //3);

    /* Set Reference for Self Test Mode */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL, srcSel); //1);

    /* Enable Self test mode. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SELF_TEST_SEL, 1);

    /* Allow reference to change. */
    ClockP_usleep(2);

    /* Enable VMON. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN, 1);

    /* Allow VMON to trip. */
    ClockP_usleep(3);

    if (CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_OVDET_LAT))
    {
        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN, 0);
        CLI_write("VDD1.2 OVtest Passed\n");
    }
    else
    {
        CLI_write("VDD1.2 OVtest Failed\n");
    }

    /* Disable VMON. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN, 0);

    /* Disable Self test mode. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SELF_TEST_SEL, 0);

    // /* Set Reference for Normal Mode. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL, src);

    /* Set Reference for Normal Mode. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL, comp);

    /* Allow VMON to reset. */
    ClockP_usleep(2);

    /* Re-enable VMON. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN, 1);

    return;
}

static void mssVDDAOsc1p8vUVSelfTest(uint32_t compSel)
{
    CSL_mss_toprcmRegs *ptrMssTopRcmReg = CSL_MSS_TOP_RCM_getBaseAddress();

    uint32_t comp = CSL_FEXT(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_IR_DROP_COMP_SEL);

    /* Set Reference for Self Test Mode */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_IR_DROP_COMP_SEL, compSel); //0);

    /* Enable Self Test Mode */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_UV_SELF_TEST_SEL, 1);

    /* Allow reference to change. */
    ClockP_usleep(2);

    /* Enable VMON. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN, 1);

    /* Allow VMON to trip. */
    ClockP_usleep(3);

    if(CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA_OSC_UVDET_LAT))
    {
        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN, 0);
        CLI_write("VDDA_OSC 1.8V UV Test Passed\n");
    }
    else
    {
        CLI_write("VDDA_OSC 1.8V UV Test Failed\n");
    }

    /* Disable VMON. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN, 0);

    /* Disable Self Test Mode. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_UV_SELF_TEST_SEL, 0);

    /* Set Reference for Normal Mode */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_IR_DROP_COMP_SEL, comp);

    /* Allow VMON to reset. */
    ClockP_usleep(2);

    /* Re-enable VMON. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN, 1);

    return;
}

static void mssVDDs3p3UVSelfTest(uint32_t compSel)
{
    CSL_mss_toprcmRegs *ptrMssTopRcmReg = CSL_MSS_TOP_RCM_getBaseAddress();

    uint32_t comp = CSL_FEXT(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_IR_DROP_COMP_SEL);
    /* Set Reference for Self Test Mode */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_IR_DROP_COMP_SEL, compSel); //0);

    /* Set Reference for Self Test Mode */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_UV_SELF_TEST_SEL, 1);

    /* Allow reference to change. */
    ClockP_usleep(2);

    /* Enable VMON. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN, 1);

    /* Allow VMON to trip. */
    ClockP_usleep(3);

    if(CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDS_3P3V_UVDET_LAT))
    {
        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN, 0);
        CLI_write("VDDS33 3.3V UV Test Passed\n");
    }
    else
    {
        CLI_write("VDDS33 3.3V UV Test Failed\n");
    }

    /* Disable VMON. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN, 0);

    /* Disable Self Test Mode */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_UV_SELF_TEST_SEL, 0);

    /* Set Reference for Normal Mode */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_IR_DROP_COMP_SEL, comp);

    /* Allow VMON to reset. */
    ClockP_usleep(2);

    /* Re-enable VMON. */
    CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN, 1);

    return;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for VMON self test
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIVMONSelfTest (int32_t argc, char* argv[])
{

    /* Mask VMON ESM interrupts */
    CSL_REG32_WR((volatile uint32_t *)(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP2_MASK), 0xFFFFFFFFUL);

    uint32_t UV_VMON, OV_VMON, OSC_UV_VMON, VMON_3P3V;
    float UV_THRESH, OV_THRESH, OSC_UV_THRESH, THRESH_3P3V;
    uint32_t compSel, srcSel;

    /* Values from 0.48 to 0.58 V */
    uint32_t compSelArr1p2Ov[11] = {3,2,3,2,1,0,1,3,2,1,0};
    uint32_t srcSelArr1p2Ov[11]  = {3,3,2,2,2,2,1,0,0,0,0};

    /* Values from 0.56 to 0.68 V */
    uint32_t compSelArr1p2Uv[13] = {3,2,1,3,2,1,3,2,1,3,2,1,0};
    uint32_t srcSelArr1p2Uv[13]  = {3,3,3,2,2,2,1,1,1,0,0,0,0};

    /* 0.6, 0.62, 0.64, 0.66 V */
    uint32_t compSelOthers[4] = {3,2,1,0};

    /* Populate configuration: */
    UV_VMON           = atoi (argv[1]);
    UV_THRESH         = (float) atof (argv[2]);
    OV_VMON           = atoi (argv[3]);
    OV_THRESH         = (float) atof (argv[4]);
    OSC_UV_VMON       = atoi (argv[5]);
    OSC_UV_THRESH     = (float) atof (argv[6]);
    VMON_3P3V         = atoi (argv[7]);
    THRESH_3P3V       = (float) atof (argv[8]);

    if(UV_VMON){

        /* Convert V to number between 0 and 10 */
        compSel = compSelArr1p2Uv[(int)floor((UV_THRESH * 100 + 0.5) - 56)];
        srcSel  = srcSelArr1p2Uv[(int)floor((UV_THRESH * 100 + 0.5) - 56)];
        // CLI_write("----------------\nUV_VMON\nREFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL = %d, REFSYS_SPARE_REG_LOWV_VDD_SR_SEL = %d\n", compSel, srcSel);

        /* Self-Test VMON. */
        mssVDD1p2vUVSelfTest(compSel, srcSel);
    }
    if(OV_VMON){

        /* Convert V to number between 0 and 10 */
        compSel = compSelArr1p2Ov[(int)floor((OV_THRESH * 100 + 0.5) - 48)];
        srcSel  = srcSelArr1p2Ov[(int)floor((OV_THRESH * 100 + 0.5) - 48)];
        // CLI_write("----------------\nUV_VMON \nREFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL = %d, REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL = %d\n", compSel, srcSel);

        /* Self-Test VMON. */
        mssVDD1p2vOVSelfTest(compSel, srcSel);
    }
    if(OSC_UV_VMON){

        compSel = compSelOthers[(int)floor((OSC_UV_THRESH * 100 - 60)/2 + 0.5)];
        // CLI_write("----------------\nOSC_UV_VMON\nREFSYS_SPARE_REG_LOWV_VDDA_OSC_IR_DROP_COMP_SEL = %d\n", compSel);

        /* Self-Test VMON. */
        mssVDDAOsc1p8vUVSelfTest(compSel);
    }
    if(VMON_3P3V){

        compSel = compSelOthers[(int)floor((THRESH_3P3V * 100 - 60)/2 + 0.5)];
        // CLI_write("----------------\n3P3V_VMON\nREFSYS_SPARE_REG_LOWV_VDDS_3P3V_IR_DROP_COMP_SEL = %d\n", compSel);

        /* Self-Test VMON. */
        mssVDDs3p3UVSelfTest(compSel);
    }

    /* Unmask VMON ESM interrupts */
    CSL_REG32_WR((volatile uint32_t *)(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP2_MASK), 0xFFF9FFFCUL);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for VMON enable
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIVMONEn (int32_t argc, char* argv[])
{
    CSL_mss_toprcmRegs *ptrMssTopRcmReg = CSL_MSS_TOP_RCM_getBaseAddress();
    uint32_t UV_VMON, OV_VMON, OSC_UV_VMON, VMON_3P3V;
    float UV_THRESH, OV_THRESH, OSC_UV_THRESH, THRESH_3P3V;
    uint32_t compSel, srcSel;

    /* Values from 0.48 to 0.58 V */
    uint32_t srcSelArr1p2Uv[11]  = {3,3,2,2,2,2,1,0,0,0,0};
    uint32_t compSelArr1p2Uv[11] = {3,2,3,2,1,0,1,3,2,1,0};

    /* Values from 0.56 to 0.68 V */
    uint32_t srcSelArr1p2Ov[13]  = {3,3,3,3,2,2,2,1,1,1,0,0,0};
    uint32_t compSelArr1p2Ov[13] = {3,2,1,0,2,1,0,2,1,0,2,1,0};

    /* 0.5, 0.52, 0.54, 0.56 V */
    uint32_t compSelOthers[4] = {3,2,1,0};

    /* Populate configuration: */
    UV_VMON           = atoi (argv[1]);
    UV_THRESH         = (float) atof (argv[2]);
    OV_VMON           = atoi (argv[3]);
    OV_THRESH         = (float) atof (argv[4]);
    OSC_UV_VMON       = atoi (argv[5]);
    OSC_UV_THRESH     = (float) atof (argv[6]);
    VMON_3P3V         = atoi (argv[7]);
    THRESH_3P3V       = (float) atof (argv[8]);

    if(UV_VMON)
    {
        /* Convert V to number between 0 and 10 */
        compSel = compSelArr1p2Uv[(int)floor((UV_THRESH * 100 + 0.5) - 48)];
        srcSel  = srcSelArr1p2Uv[(int)floor((UV_THRESH * 100 + 0.5) - 48)];
        // CLI_write("UV_VMON \nREFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL = %d, REFSYS_SPARE_REG_LOWV_VDD_SR_SEL = %d\n", compSel, srcSel);

        /* Set Reference for Normal Mode. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_IR_DROP_COMP_SEL, compSel);

        /* Set Reference for Normal Mode. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_SR_SEL, srcSel);

        /* Allow VMON to reset. */
        ClockP_usleep(2);

        /* Enable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN, 1);
    }
    if(OV_VMON){

        /* Convert V to number between 0 and 10 */
        compSel = compSelArr1p2Ov[(int)floor((OV_THRESH * 100 + 0.5) - 56)];
        srcSel  = srcSelArr1p2Ov[(int)floor((OV_THRESH * 100 + 0.5) - 56)];
        // CLI_write("OV_VMON\nREFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL = %d, REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL = %d\n", compSel, srcSel);

        /* Set Reference for Normal Mode. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_SR_SEL, srcSel);

        /* Set Reference for Normal Mode. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDD_OV_IR_DROP_COMP_SEL, compSel);

        /* Allow VMON to reset. */
        ClockP_usleep(2);

        /* Enable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN, 1);
    }
    if(OSC_UV_VMON){

        compSel = compSelOthers[(int)floor((OSC_UV_THRESH * 100 - 50)/2 + 0.5)];
        // CLI_write("OSC_UV_VMON\nREFSYS_SPARE_REG_LOWV_VDDA_OSC_IR_DROP_COMP_SEL = %d\n", compSel);

        /* Set Reference for Normal Mode */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDA_OSC_IR_DROP_COMP_SEL, compSel); //2);

        /* Allow VMON to reset. */
        ClockP_usleep(2);

        /* Enable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN, 1);
    }
    if(VMON_3P3V){

        compSel = compSelOthers[(int)floor((THRESH_3P3V * 100 - 50)/2 + 0.5)];
        // CLI_write("3P3V_VMON\nREFSYS_SPARE_REG_LOWV_VDDS_3P3V_IR_DROP_COMP_SEL = %d\n", compSel);

        /* Set Reference for Normal Mode */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_REFSYS_SPARE_REG_LOWV, MSS_TOPRCM_ANA_REG_REFSYS_SPARE_REG_LOWV_VDDS_3P3V_IR_DROP_COMP_SEL, compSel); //2);

        /* Allow VMON to reset. */
        ClockP_usleep(2);

        /* Enable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN, 1);
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for VMON enable (BSS)
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIBSSVMONEn (int32_t argc, char* argv[])
{

    rlVmonMonConf_t cfg;
    uint32_t temp;
    rlReturnVal_t retVal;

    cfg.VddaBb1P8VEn    = (uint8_t) atoi (argv[1]);
    cfg.VddaBb1P8VRef   = (uint8_t) (3 - (((uint32_t)(((float) atof (argv[2])) * 100))/2 - 25));
    cfg.VddaVco1P8VEn   = (uint8_t) atoi (argv[3]);
    cfg.VddaVco1P8VRef  = (uint8_t) (3 - (((uint32_t)(((float) atof (argv[4])) * 100))/2 - 25));
    cfg.VddRf11P0VEn    = (uint8_t) atoi (argv[5]);
    temp = (uint32_t) (((float) atof (argv[6])) * 100);
    cfg.VddRf11P0VRef   = (temp == 52) ? (0) : ( (temp == 51) ? (1) : ((temp == 49) ? (2) : ((temp == 48) ? 3 : -1))  );
    cfg.VddRf21P0VEn    = (uint8_t) atoi (argv[7]);
    temp = (uint32_t) (((float) atof (argv[8])) * 100);
    cfg.VddRf21P0VRef   = (temp == 52) ? (0) : ( (temp == 51) ? (1) : ((temp == 49) ? (2) : ((temp == 48) ? 3 : -1))  );

    retVal = rlRfVmonMonConfig(RL_DEVICE_MAP_INTERNAL_BSS, &cfg);
    if (retVal != 0){
        CLI_write("ERROR!\n");
    }

    return 0;

}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for enabling HWA Dynamic Clock Gating
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HwaDynamicClockGating (int32_t argc, char* argv[])
{

    /* Populate configuration: */
    gMmwMssMCB.powerMeas.isHwaDynamicClockGating          = (uint32_t) atoi (argv[1]);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for enabling HWA
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HwaGateAfterFrameProc (int32_t argc, char* argv[])
{
    /* Populate configuration: */
     gMmwMssMCB.powerMeas.isHwaGateAfterFrameProc          = (uint32_t) atoi (argv[1]);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for querying the front end temperature sensor data
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_anaTempRead (int32_t argc, char* argv[])
{

    rlRfTempData_t temperatureData;
    rlRfGetTemperatureReport(RL_DEVICE_MAP_INTERNAL_BSS, &temperatureData);
    CLI_write("RxTemp = Rx0: %d, Rx1: %d, Rx2: %d, Rx3: %d\r\n", temperatureData.tmpRx0Sens,
        temperatureData.tmpRx1Sens, temperatureData.tmpRx2Sens, temperatureData.tmpRx3Sens);
    CLI_write("TxTemp = Tx0: %d, Tx1: %d, Tx2: %d, Tx3: %d\r\n", temperatureData.tmpTx0Sens,
        temperatureData.tmpTx1Sens, temperatureData.tmpTx2Sens, temperatureData.tmpDig0Sens);
    CLI_write("PM temperature sensor reading: %d\r\n", temperatureData.tmpPmSens);
    CLI_write("All values in degree C\r\n");

    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for querying the MCU, HWA and HSM temperature values
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_digTempRead (int32_t argc, char* argv[])
{
    int32_t status;
    GPADC_TempSensValueType tempValues = {0};
    // GPADC_CodeValueType gpadcCodeValues = {0};
    uint8_t numAverageSamples;

    GPADC_open(&gCfgPtrTempRead);

    GPADC_initTempMeasurement();
    numAverageSamples = 5U;
    // status = GPADC_readTemperature(numAverageSamples,MAX_GPADC_TEMP_SENSORS, &tempValues, &gpadcCodeValues);
    status = GPADC_readTemperature(numAverageSamples, MAX_GPADC_TEMP_SENSORS, &tempValues);

    if(SystemP_SUCCESS == status)
    {
        CLI_write("Temperature read conversion successful\r\n");
    }
    else if(SystemP_FAILURE == status)
    {
        CLI_write("Temperature read conversion unsuccessful\r\n");
    }

    CLI_write("MCU = %d\r\nHWA = %d\r\nHSM = %d\r\n", tempValues.DigMcuTempValue, tempValues.DigHwaTempValue, tempValues.DigHsmTempValue);
    // CLI_write("MCU code= %d\r\nHWA code= %d\r\nHSM code= %d\r\n", gpadcCodeValues.DigMcuCodeValue, gpadcCodeValues.DigHwaCodeValue, gpadcCodeValues.DigHsmCodeValue);

    GPADC_close();

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for VMON disable
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIVMONDis (int32_t argc, char* argv[])
{

    CSL_mss_toprcmRegs *ptrMssTopRcmReg = CSL_MSS_TOP_RCM_getBaseAddress();
    uint32_t UV_VMON, OV_VMON, OSC_UV_VMON, VMON_3P3V;
    /* Populate configuration: */
    UV_VMON           = atoi (argv[1]);
    OV_VMON           = atoi (argv[2]);
    OSC_UV_VMON       = atoi (argv[3]);
    VMON_3P3V         = atoi (argv[4]);

    if(UV_VMON){

        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN, 0);
    }
    if(OV_VMON){

        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN, 0);
    }
    if(OSC_UV_VMON){

        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN, 0);
    }
    if(VMON_3P3V){

        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN, 0);
    }

    return 0;
}
static int32_t MmwDemo_CLISSCConfig (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 9)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Validate inputs */
    /* Modulation rate input */
    if (((bool) atoi(argv[1])) && (((uint32_t) atoi(argv[2]) == 0U) || ((uint32_t) atoi(argv[2]) > 100U)))
    {
        CLI_write ("Error: Core ADPLL modulation rate should be between 1KHz to 100KHz\r\n");
        return -1;
    }

    if (((bool) atoi(argv[5])) && (((uint32_t) atoi(argv[6]) == 0U) || ((uint32_t) atoi(argv[6]) > 100U)))
    {
        CLI_write ("Error: DSP ADPLL modulation rate should be between 1KHz to 100KHz\r\n");
        return -1;
    }

    /* Validate inputs */
    /* Modulation Depth input */
    if (((bool) atoi(argv[1])) && ((float) atof(argv[3]) > 2.0f))
    {
        CLI_write ("Error: Core ADPLL modulation depth should be between 0% to 2%\r\n");
        return -1;
    }

    if (((bool) atoi(argv[5])) && ((float) atof(argv[7]) > 2.0f))
    {
        CLI_write ("Error: DSP ADPLL modulation depth should be between 0% to 2%\r\n");
        return -1;
    }

    /* Populate configuration: CORE ADPLL SSC */
    gMmwMssMCB.coreAdpllSscCfg.isEnable = (bool) atoi(argv[1]);
    gMmwMssMCB.coreAdpllSscCfg.modRate = (uint8_t) atoi(argv[2]);
    gMmwMssMCB.coreAdpllSscCfg.modDepth = (float) atof(argv[3]);
    gMmwMssMCB.coreAdpllSscCfg.downSpread = (uint8_t) atoi(argv[4]);

    /* Populate configuration: PER ADPLL SSC */
    gMmwMssMCB.perAdpllSscCfg.isEnable = (bool) atoi(argv[5]);
    gMmwMssMCB.perAdpllSscCfg.modRate = (uint8_t) atoi(argv[6]);
    gMmwMssMCB.perAdpllSscCfg.modDepth = (float) atof(argv[7]);
    gMmwMssMCB.perAdpllSscCfg.downSpread = (uint8_t) atoi(argv[8]);

    return 0;
}

#ifdef POWER_MEAS
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for loading MSS for a given percentage of frame period
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t powerMeas_MssLoading (int32_t argc, char* argv[])
{
    uint32_t mssLoad;

    /* Populate configuration: */
    mssLoad          =  atoi (argv[1]);
    gMmwMssMCB.powerMeas.mssLoadingPercent          =  atoi (argv[2]);

    /* Populate configuration: */
    gMmwMssMCB.powerMeas.mssLoading = mssLoad;

    return 0;
}

#endif

static int32_t MmwDemo_measurevoltage (int32_t argc, char* argv[])
{
    uint32_t lowerRef = 0;
    uint32_t upperRef = 1800;
    uint32_t channelResolution = 10U;
    uint16_t gpadc_result[9];
    GPADC_channelsGroupSelectType channels;
    GPADC_ConvResultType convRes = GPADC_CONV_ERROR;
    uint8_t index;
    uint32_t adcInMv;

    GPADC_open(&gCfgPtrExtVoltage);

    channels.bits.b9_ChannelSelectionBitMap = (CONFIG_GPADC_EXT_VOLT_CHANNEL_BITMAP & 0x1FF);

	GPADC_setupResultBuffer(&gpadc_result[0]);
	convRes = GPADC_startGroupConversion(channels,MAX_GPADC_MEAS_SOURCES);
	if(GPADC_CONV_DONE == convRes)
	{
		CLI_write ("Channel adc conversion successful\r\n");
	}
	else if(GPADC_CONV_CHANNEL_CONFIG_MISSING == convRes)
	{
		CLI_write ("channel adc configuration missing\r\n");
	}
	else
	{
		CLI_write ("Channel adc conversion error\n");
	}

	CLI_write (" Channel\tHW_CH\t\tADC Value\tVolt\r\n");

	for(index=0; index < MAX_GPADC_MEAS_SOURCES; index++)
	{

		adcInMv = (gpadc_result[index] * (upperRef -lowerRef))/
				(1<<channelResolution);

		CLI_write (
				" %4d\t\tADC_IN%d\t0x%08x\t%04dmV\r\n",
				index, index, gpadc_result[index], adcInMv);
	}

    GPADC_close();

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_UnusedPerClkGate (int32_t argc, char* argv[])
{
    CSL_mss_rcmRegs *ptrMssRcmRegs = (CSL_mss_rcmRegs *)CSL_MSS_RCM_U_BASE;
    CSL_mss_toprcmRegs *ptrMssToprcmRegs = (CSL_mss_toprcmRegs *)CSL_MSS_TOPRCM_U_BASE;

    uint32_t clkGate          =  atoi (argv[1]);

    if(clkGate)
    {
        //MSS RCM
        ptrMssRcmRegs->MSS_WDT_CLK_GATE = MMWDEMO_CLOCK_GATE;
        ptrMssRcmRegs->MSS_SPIB_CLK_GATE = MMWDEMO_CLOCK_GATE;
        ptrMssRcmRegs->MSS_I2C_CLK_GATE = MMWDEMO_CLOCK_GATE;
        ptrMssRcmRegs->MSS_SCIB_CLK_GATE = MMWDEMO_CLOCK_GATE;

        //MSS TOPRCM
        ptrMssToprcmRegs->MCUCLKOUT_CLK_GATE = MMWDEMO_CLOCK_GATE;
        ptrMssToprcmRegs->PMICCLKOUT_CLK_GATE = MMWDEMO_CLOCK_GATE;
        ptrMssToprcmRegs->OBSCLKOUT_CLK_GATE = MMWDEMO_CLOCK_GATE;
        ptrMssToprcmRegs->TRCCLKOUT_CLK_GATE = MMWDEMO_CLOCK_GATE;

    }
    return 0;
}

static int32_t MmwDemo_showCpswStats(int32_t argc, char* argv[])
{

#if defined (POWER_MEAS)
    MmwEnet_showCpswStats();
#else
    EnetApp_showCpswStats();
#endif

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler to arrange antennas in increasing order of phase shift value,
 * if all of them were enabled. For example, a value of {0,2,3,1} would mean that phase shifts
 * for a particular chirp are in the following order of magnitude -
 *        tx0ChirpPhase < tx2ChirpPhase < tx3ChirpPhase < tx1ChirpPhase
 * Even if the user does not intend to use all the tx antennas, the order should be programmed assuming
 * that all the Tx were enabled. The phase shift values for the ones that are not enabled will be
 * configured to 0 by the code.
 *
 * Note that in the DDMA case, the elevation antenna(s) should always come at the end of this array.
 * Basically, phaseShift(azimuth) < phaseShift(elevation) must be ensured.
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   < -1
 */
static int32_t MmwDemo_CLIDDMPhaseShiftOrder (int32_t argc, char* argv[])
{
    uint8_t index = 0;

    /* Sanity Check: Minimum argument check */
    if (argc < (1 + SYS_COMMON_NUM_TX_ANTENNAS))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    for(index = 0; index < SYS_COMMON_NUM_TX_ANTENNAS; index++)
    {
        gMmwMssMCB.ddmPhaseShiftOrder[index] = (uint8_t)atoi(argv[(index + 1)]);
    }

    gMmwMssMCB.isDdmPhaseShiftCfgPending = 1;

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
extern void MmwDemo_CLIInit (uint8_t taskPriority)
{
    CLI_Cfg     cliCfg;
    char        demoBanner[256];
    uint32_t    cnt;

    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0],
                       "******************************************\r\n" \
                       "AWR2544 MMW Demo %02d.%02d.%02d.%02d\r\n"  \
                       "******************************************\r\n",
                        MMWAVE_SDK_VERSION_MAJOR,
                        MMWAVE_SDK_VERSION_MINOR,
                        MMWAVE_SDK_VERSION_BUGFIX,
                        MMWAVE_SDK_VERSION_BUILD
            );

    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "mmwDemo:/>";
    cliCfg.cliBanner                    = demoBanner;
    cliCfg.cliUartHandle                = gMmwMssMCB.commandUartHandle;
    cliCfg.taskPriority                 = taskPriority;
    cliCfg.mmWaveHandle                 = gMmwMssMCB.ctrlHandle;
    cliCfg.enableMMWaveExtension        = 1U;
    cliCfg.usePolledMode                = true;
    cliCfg.overridePlatform             = false;
    cliCfg.overridePlatformString       = "AWR2544";

    cnt=0;
    cliCfg.tableEntry[cnt].cmd            = "sensorStart";
    cliCfg.tableEntry[cnt].helpString     = "[doReconfig(optional, default:enabled)]";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStart;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "sensorStop";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStop;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcbufCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <adcOutputFmt> <SampleSwap> <ChanInterleave> <ChirpThreshold>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIADCBufCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQRxSatMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <satMonSel> <priSliceDuration> <numSlices> <rxChanMask>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIChirpQualityRxSatMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "CQSigImgMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<profile> <numSlices> <numSamplePerSlice>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIChirpQualitySigImgMonCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "analogMonitor";
    cliCfg.tableEntry[cnt].helpString     = "<rxSaturation> <sigImgBand> <apllLdoSCMonEn>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIAnalogMonitorCfg;
    cnt++;

#ifdef LVDS_STREAM
    cliCfg.tableEntry[cnt].cmd            = "lvdsStreamCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <enableHeader> <dataFmt> <enableSW>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLILvdsStreamCfg;
    cnt++;
#endif

    cliCfg.tableEntry[cnt].cmd            = "queryDemoStatus";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIQueryDemoStatus;
    cnt++;

#ifdef POWER_MEAS
    cliCfg.tableEntry[cnt].cmd            = "powerMeasMssLoading";
    cliCfg.tableEntry[cnt].helpString     = "<enable> <percent>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = powerMeas_MssLoading;
    cnt++;
#endif
    cliCfg.tableEntry[cnt].cmd            = "calibData";
    cliCfg.tableEntry[cnt].helpString    = "<save enable> <restore enable> <Flash offset>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICalibDataSaveRestore;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "compressionCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx> <isEnabled> <compressionMethod> <compressionRatio> <rangeBinsPerBlock>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICompressionCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "intfMitigCfg";
    cliCfg.tableEntry[cnt].helpString     = "<subFrameIdx>  <magSNRdB> <magDiffSNRdB>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIIntfMitigCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "coexMSSVMONSelfTest";
    cliCfg.tableEntry[cnt].helpString     = "<UV_VMON> <Thresh> <OV_VMON> <Thresh> <OSC_UV_VMON> <Thresh> <VMON_3P3V> <Thresh>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIVMONSelfTest;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "coexMSSVMONEnable";
    cliCfg.tableEntry[cnt].helpString     = "<UV_VMON> <Thresh> <OV_VMON> <Thresh> <OSC_UV_VMON> <Thresh> <VMON_3P3V> <Thresh>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIVMONEn;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "coexBSSVMONEnable";
    cliCfg.tableEntry[cnt].helpString     = "<VddaBb1P8VEn> <VddaBb1P8VRef> <VddaVco1P8VEn> <VddaVco1P8VRef> <VddRf11P0VEn> <VddRf11P0VRef> <VddRf21P0VEn> <VddRf21P0VRef>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIBSSVMONEn;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "coexMSSVMONDisable";
    cliCfg.tableEntry[cnt].helpString     = "<UV_VMON> <OV_VMON> <OSC_UV_VMON> <VMON_3P3V>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIVMONDis;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "hwaDynamicClockGating";
    cliCfg.tableEntry[cnt].helpString     = "<enable>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_HwaDynamicClockGating;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "anaTempRead";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_anaTempRead;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "digTempRead";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_digTempRead;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "readVoltageSig";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_measurevoltage;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "hwaGateAfterFrameProc";
    cliCfg.tableEntry[cnt].helpString     = "<option>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_HwaGateAfterFrameProc;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "unusedPerClkGate";
    cliCfg.tableEntry[cnt].helpString     = "<enableClkGating>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_UnusedPerClkGate;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "procChainCfg";
    cliCfg.tableEntry[cnt].helpString     = "<procChain (0: TDM; 1: DDM)> <2xMode> <ethPktRdyCnt> <ethPerPktDly> <nwPktCrcSel>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIProcChainCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcDataDitherCfg";
    cliCfg.tableEntry[cnt].helpString     = "<isDelayEn> <isDitherEn> <ditherVal> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIADCDataDitherCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "spreadSpectrumConfig";
    cliCfg.tableEntry[cnt].helpString     = "<coreADPLLEnable> <coreModRate> <coreModDepth> <coreDownSpread> <perADPLLEnable> <perModRate> <PerModDepth> <perDownSpread>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISSCConfig;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "showCpswStats";
    cliCfg.tableEntry[cnt].helpString     = "No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_showCpswStats;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "ddmPhaseShiftAntOrder";
    cliCfg.tableEntry[cnt].helpString     = "<Tx0> <Tx1> ... <TxN>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIDDMPhaseShiftOrder;
    cnt++;

    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        test_print ("Error: Unable to open the CLI\n");
        return;
    }
    test_print ("Debug: CLI is operational\n");
    return;
}
