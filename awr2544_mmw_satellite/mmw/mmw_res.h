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
 *  \file   mmw_res.h
 *
 *  \brief  Defines partitioning of hardware resources (HWA, EDMA etc)
 *          among the Range DPU and other components in the demo.
 */

#ifndef MMW_DEMO_RES_H
#define MMW_DEMO_RES_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*                           Macros & Typedefs                               */
/* ========================================================================= */

/*! \brief EDMA instance used*/
#define MMW_RES_EDMA_DRV_INST_DSS_A                            0
#define MMW_RES_EDMA_NUM_DMA_CHANNELS                          64
#define MMW_RES_EDMA_SHADOW_BASE                 MMW_RES_EDMA_NUM_DMA_CHANNELS

/*************************** Range DPU EDMA Resources ************************/

/*! \brief Range DPU HWA Paramsets */
#define MMW_RES_DPU_RANGE_PARAMSET_START_IDX                   0
#define MMW_RES_DPU_RANGE_NUM_HWA_PARAMSETS                    10

/*! \brief Range DPU EDMA Instance */
#define MMW_RES_DPU_RANGE_EDMA_INST_ID           MMW_RES_EDMA_DRV_INST_DSS_A

/*! \brief Range DPU EDMA Headaer With Three Links */
#define MMW_RES_DPU_RANGE_EDMAHEADER_CH          \
                                      EDMA_DSS_TPCC_A_EVT_DFE_CHIRP_CYCLE_START
#define MMW_RES_DPU_RANGE_EDMAHEADER_SHADOW1     (MMW_RES_EDMA_SHADOW_BASE + 1)
#define MMW_RES_DPU_RANGE_EDMAHEADER_SHADOW2     (MMW_RES_EDMA_SHADOW_BASE + 2)
#define MMW_RES_DPU_RANGE_EDMAHEADER_SHADOW3     (MMW_RES_EDMA_SHADOW_BASE + 3)
#define MMW_RES_DPU_RANGE_EDMAHEADER_EVENT_QUE                 0

/*! \brief Range DPU EDMA Data In From ADC Buffer to HWA */
#define MMW_RES_DPU_RANGE_EDMAIN_CH         EDMA_DSS_TPCC_A_EVT_RSS_ADC_CAPTURE_COMPLETE
#define MMW_RES_DPU_RANGE_EDMAIN_SHADOW         (MMW_RES_EDMA_SHADOW_BASE + 13)
#define MMW_RES_DPU_RANGE_EDMAIN_EVENT_QUE                     0

/*! \brief Range DPU EDMA In Signature */
#define MMW_RES_DPU_RANGE_EDMAIN_SIG_CH     EDMA_DSS_TPCC_A_EVT_CBUFF_DMA_REQ0
#define MMW_RES_DPU_RANGE_EDMAIN_SIG_SHADOW     (MMW_RES_EDMA_SHADOW_BASE + 14)
#define MMW_RES_DPU_RANGE_EDMAIN_SIG_EVENT_QUE                 0

/*! \brief Range DPU EDMA Out Ping With Three Links */
#define MMW_RES_DPU_RANGE_EDMAOUT_PING_CH  EDMA_DSS_TPCC_A_EVT_DSS_HWA_DMA_REQ1
#define MMW_RES_DPU_RANGE_EDMAOUT_PING_SHADOW1   (MMW_RES_EDMA_SHADOW_BASE + 4)
#define MMW_RES_DPU_RANGE_EDMAOUT_PING_SHADOW2   (MMW_RES_EDMA_SHADOW_BASE + 5)
#define MMW_RES_DPU_RANGE_EDMAOUT_PING_SHADOW3   (MMW_RES_EDMA_SHADOW_BASE + 6)
#define MMW_RES_DPU_RANGE_EDMAOUT_PING_EVENT_QUE               0

/*! \brief Range DPU EDMA Out Pong With Three Links */
#define MMW_RES_DPU_RANGE_EDMAOUT_PONG_CH EDMA_DSS_TPCC_A_EVT_DSS_HWA_DMA_REQ16
#define MMW_RES_DPU_RANGE_EDMAOUT_PONG_SHADOW1   (MMW_RES_EDMA_SHADOW_BASE + 7)
#define MMW_RES_DPU_RANGE_EDMAOUT_PONG_SHADOW2   (MMW_RES_EDMA_SHADOW_BASE + 8)
#define MMW_RES_DPU_RANGE_EDMAOUT_PONG_SHADOW3   (MMW_RES_EDMA_SHADOW_BASE + 9)
#define MMW_RES_DPU_RANGE_EDMAOUT_PONG_EVENT_QUE               0


/*! \brief Range DPU EDMA Footer With Three Links */
#define MMW_RES_DPU_RANGE_EDMAFOOTER_CH    EDMA_DSS_TPCC_A_EVT_DSS_HWA_DMA_REQ2
#define MMW_RES_DPU_RANGE_EDMAFOOTER_SHADOW1    (MMW_RES_EDMA_SHADOW_BASE + 10)
#define MMW_RES_DPU_RANGE_EDMAFOOTER_SHADOW2    (MMW_RES_EDMA_SHADOW_BASE + 11)
#define MMW_RES_DPU_RANGE_EDMAFOOTER_SHADOW3    (MMW_RES_EDMA_SHADOW_BASE + 12)
#define MMW_RES_DPU_RANGE_EDMAFOOTER_EVENT_QUE                 0

/*! \brief Range DPU EDMA Out Signature */
#define MMW_RES_DPU_RANGE_EDMAOUT_SIG_CH  EDMA_DSS_TPCC_A_EVT_DSS_HWA_DMA_REQ31
#define MMW_RES_DPU_RANGE_EDMAOUT_SIG_SHADOW    (MMW_RES_EDMA_SHADOW_BASE + 15)
#define MMW_RES_DPU_RANGE_EDMAOUT_SIG_EVENT_QUE                0

/*! \brief
 *  Below listed EDMA channels are configured to transfer Payload data
 *  from L3 memory to DSS Network Packet buffer.
 *
 *  EDMA channel "EDMA_DSS_TPCC_A_EVT_CPSW_PP_TOGGLE" is enabled for HW trigger
 *  As part of the processing chain CPSW consumes payload from MSS NW packet buffer
 *  TOGGLE channel gets triggered to below listed activities:
 *      - Transfer next Payload from L3 to DSS NW packet buffer
 *      - Trigger CPSW channel control for transfering payload over ethernet interface
 *
 * Payload data = (APP_HEADER + 1D-FFT data + APP_FOOTER)
 */
#define MMW_RES_ENET_EDMAL3OUT_CH                     (EDMA_DSS_TPCC_A_EVT_CPSW_PP_TOGGLE)
#define MMW_RES_ENET_EDMAL3OUT_SHADOW1                (MMW_RES_EDMA_SHADOW_BASE + 16)
#define MMW_RES_ENET_EDMAL3OUT_SHADOW2                (MMW_RES_EDMA_SHADOW_BASE + 17)
#define MMW_RES_ENET_EDMAL3OUT_SHADOW3                (MMW_RES_EDMA_SHADOW_BASE + 18)
#define MMW_RES_ENET_EDMAL3OUT_EVENT_QUE              0

#define MMW_RES_ENET_TOGGLE_CH                        (EDMA_DSS_TPCC_A_EVT_FREE0)
#define MMW_RES_ENET_TOGGLE_SHADOW1                   (MMW_RES_EDMA_SHADOW_BASE + 19)
#define MMW_RES_ENET_TOGGLE_SHADOW2                   (MMW_RES_EDMA_SHADOW_BASE + 20)
#define MMW_RES_ENET_TOGGLE_SHADOW3                   (MMW_RES_EDMA_SHADOW_BASE + 21)
#define MMW_RES_ENET_TOGGLE_EVENT_QUE                 (0)

#define MMW_RES_ENET_DELAY_CH                        (EDMA_DSS_TPCC_A_EVT_DSS_HWA_DMA_REQ10)
#define MMW_RES_ENET_DELAY_SHADOW                    (MMW_RES_EDMA_SHADOW_BASE + 22)
#define MMW_RES_ENET_DELAY_EVENT_QUE                 (0)

/****************** LVDS streaming EDMA resources ****************************/

/*EDMA instance used*/
#define MMW_LVDS_STREAM_EDMA_INSTANCE                       EDMA_DRV_INST_RSS_A
/* CBUFF EDMA trigger channels */
#define MMW_LVDS_STREAM_CBUFF_EDMA_CH_0  EDMA_RSS_TPCC_A_EVT_DSS_CBUFF_DMA_REQ0
#define MMW_LVDS_STREAM_CBUFF_EDMA_CH_1  EDMA_RSS_TPCC_A_EVT_DSS_CBUFF_DMA_REQ1

/* HW Session*/
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_0         EDMA_RSS_TPCC_A_EVT_FREE_0
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_1         EDMA_RSS_TPCC_A_EVT_FREE_1
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_2         EDMA_RSS_TPCC_A_EVT_FREE_2
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_3         EDMA_RSS_TPCC_A_EVT_FREE_3
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_4         EDMA_RSS_TPCC_A_EVT_FREE_4
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_5         EDMA_RSS_TPCC_A_EVT_FREE_5
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_6         EDMA_RSS_TPCC_A_EVT_FREE_6
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_7         EDMA_RSS_TPCC_A_EVT_FREE_7
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_8         EDMA_RSS_TPCC_A_EVT_FREE_8
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_9         EDMA_RSS_TPCC_A_EVT_FREE_9
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_CH_10        EDMA_RSS_TPCC_A_EVT_FREE_10

/*shadow*/
/*shadow CBUFF trigger channels*/
#define MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_0  (MMW_RES_EDMA_SHADOW_BASE + 0U)
#define MMW_LVDS_STREAM_CBUFF_EDMA_SHADOW_CH_1      \
                                                (MMW_RES_EDMA_SHADOW_BASE + 1U)

/* HW Session*/
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_0   (MMW_RES_EDMA_SHADOW_BASE + 2U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_1   \
                                                (MMW_RES_EDMA_SHADOW_BASE + 3U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_2   \
                                                (MMW_RES_EDMA_SHADOW_BASE + 4U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_3   \
                                                (MMW_RES_EDMA_SHADOW_BASE + 5U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_4   \
                                                (MMW_RES_EDMA_SHADOW_BASE + 6U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_5   \
                                                (MMW_RES_EDMA_SHADOW_BASE + 7U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_6   \
                                                (MMW_RES_EDMA_SHADOW_BASE + 8U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_7   \
                                                (MMW_RES_EDMA_SHADOW_BASE + 9U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_8   \
                                              (MMW_RES_EDMA_SHADOW_BASE + 10U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_9   \
                                              (MMW_RES_EDMA_SHADOW_BASE + 11U)
#define MMW_LVDS_STREAM_HW_SESSION_EDMA_SHADOW_CH_10  \
                                              (MMW_RES_EDMA_SHADOW_BASE + 12U)

#ifdef __cplusplus
}
#endif

#endif /* MMW_DEMO_RES_H */
