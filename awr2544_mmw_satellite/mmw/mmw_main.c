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
 *  \file   mmw_main.c
 *
 *  \brief  This is the main file which implements the millimeter wave Demo
 *
 */

/** @mainpage Millimeter Wave (mmw) Demo for AWR2544
 * [TOC]
 *  @section intro_sec Introduction
 *
 *   @image html toplevel.png
 *   \n
 *
 *  The millimeter wave demo shows some of the capabilities of the AWR2544 SOC
 *  using the drivers in the mmWave SDK (Software Development Kit).
 *  It allows user to specify the chirping profile and transmit the compressed 1D FFT
 *  data in real-time.
 *
 *  Following is a high level description of the features of this demo:
 *  - Be able to configure chirping profile via command line interface (CLI)
 *    on a UART interface -
 *    that allows user to provide a variety of profile configurations.
 *  - Some sample profile configurations have been provided in the following demo
 *    directory that can be used with CLI directly:
 *    @verbatim mmw/profiles @endverbatim
 *  - Do DC estimation and subtraction, interference estimation and mitigation,
 *    range FFT, and compression to form the compressed radar cube.
 *  - Stream out the compressed radar cube over ethernet in real-time, as seen in picture above.
 *
 *  @section procChain Processing Chains
 *    AWR2544 demo supports both TDM and DDM type of processing chains - configurable via
 *    procChainCfg <procChain> CLI. The details of the CLI can be found in the user guide.
 *  - **TDM** (Time Division Multiplexing): Here, no two transmitters are active
 *    at the same time. At the receiver antennae, there is no ambiguity on the chirp received,
 *    and every received chirp can directly be mapped, without the need of a
 *    separate disambiguation, to the corresponding Tx antenna.
 *  - **DDM** (Doppler Division Multiplexing): Here, all transmitters are active
 *    at the same time with orthogonality in the Doppler spectrum. In time domain, a unique phase
 *    is applied to each TX channel. This phase value is constant per-chirp (slow time) and
 *    increases as the chirp index increases. At the receiver antennae, the received chirps need to
 *    be disambiguated (demodulation) to obtain samples corresponding to each
 *    Tx antenna. The antenna support for this chain is (AzimTx, ElevTx, Rx) =
 *    (3,1,4) for AWR2544. If DDM processing chain is selected via CLI, demo configures the
 *    phase during the sensor configuration. The phase added to each TX channel is modified as per
 *    the equation: \f$\omega_{k} = (2\pi)(k-1)/(N_{t}+2)\f$, where N_{t} = Number of transmitters (4
 *    for AWR2544), and k is the chirp index.
 *
 *  @section DDMP Phase shifters (DDM)
 *   ti/datapath/dpc/objectdetection/objdethwaDDMA/docs/doxygen/html/index.html can be checked for information
 *   on why phase shifts are needed in the DDMA processing chain and what their values will be.
 *   CLI ddmPhaseShiftAntOrder takes antennas indices in increasing order of phase shift value,
 *   if all of them were enabled For example, a value of {0,3,1,2} would mean that phase shifts for a particular chirp are
 *   in the following order of magnitude- tx0ChirpPhase < tx3ChirpPhase < tx1ChirpPhase < tx2ChirpPhase
 *   Even if the user does not intend to use all the tx antennas, the order should be programmed assuming
 *   that all the Tx were enabled. The phase shift values for the ones that are not enabled will be
 *   configured to 0 by the code. \n
 *
 *   Note that in the DDMA case, the elevation antenna(s) should always come at the end of this array.
 *   Basically, phaseShift(azimuth) < phaseShift(elevation) must be ensured. Hence, {0, 1, 2, 3} is configured
 *   for AWR2544 LOP Antenna (Golden Devices), since Tx0, Tx1, Tx2 are azimuth antennas and Tx3 is elevation antenna.
 *
 *  @section systemFlow System Execution Flow
 *  The millimeter wave demo runs on ARM Cortex R5F (MSS) core. Following diagram
 *  shows the system execution flow
 *
 *  @image html system_flow.png "System Execution Flow"
 *
 *  @section tasks Software Tasks
 *    The demo consists of the following (FreeRTOS) tasks running on R5F (MSS):
 *
 *    - @ref MmwDemo_initTask. This task is created/launched by @ref main and
 *      is a one-time active task whose main functionality is to initialize
 *      drivers (\<driver\>_init), MMWave module (MMWave_init), open UART, Enet
 *      and other drivers (EDMA), and create/launch the following tasks
 *      (the @ref CLI_task is launched indirectly by calling @ref CLI_open).
 *    - @ref CLI_task. This command line interface task provides a simplified
 *      'shell' interface which allows the configuration of the BSS via the
 *      mmWave interface (MMWave_config). It parses input CLI configuration
 *      commands like chirp and profile configuration. When sensor start
 *      CLI command is parsed, all actions related to starting sensor and
 *      starting the processing the data path are taken. When sensor stop CLI
 *      command is parsed, all actions related to stopping the sensor and
 *      stopping the processing of the data path are taken.
 *    - @ref MmwDemo_mmWaveCtrlTask. This task is used to provide an execution
 *      context for the mmWave control, it calls in an endless loop the
 *      @ref MMWave_execute API.
 *    - @ref MmwDemo_DPCTask. This task is used to provide an execution
 *      context for DPC (Datapath Chain), it calls in an endless loop the  @ref MmwDPC_execute API.
 *      In this context, DPC initialization, frame start event and chirp available event
 *      registration will take place. In this task, the DPC's execute API
 *      keeps pending for the frame start event. Once the frame start, it waits for the range DPU
 *      output and then triggers the range DPU for the next frame.
 *    - @ref MmwEnet_rxTask. Enet RX task pending on MSS_CPSW_TH_INT interrupt. This task handles the
 *      data received on ethernet interface and free-up receive buffer descriptors.
 *
 *  @section datapath Datapath
 *
 * @image html Demo_Timing.png "Top Level Data Path Timing"
 *
 * The data path processing consists of taking ADC samples as input and generating compressed radar cube to be exported via Ethernet interface to the PC. The algorithm processing is realized using the RangeprocReal2x DPU. The details of the processing can be seen from the following doxygen documentation.
 *  @verbatim ti/datapath/dpu/rangeprocReal2x/docs/doxygen/html/index.html @endverbatim
 *  As shown in the  above figure, compressed radar cube transfer over CPSW (RGMII transfer) starts
 *  in acquistion period itself, which implies transfer happens in parallel with the range processing.
 *
 * @section EnetCfg Enet Configuration
 *    - @ref MmwEnet_init. This function initializes CPSW interface and open CPDMA handle for Tx and Rx channels.
 *      It establishes link with DP83867 Industrial PHY chip. It also configures CPDMA Rx buffer descriptors for
 *      handling packets received on CPSW interface and enables Fhost_ownership feature.
 *    - @ref MmwEnet_config. This function performs below listed configurations:
 *      - Configures EDMA PARAMSET for "EDMA_DSS_TPCC_A_EVT_CPSW_PP_TOGGLE" and "EDMA_DSS_TPCC_A_EVT_FREE0".
 *      - Network packet buffer configuration:
 *          - Enable either 32-bit or 16-bit CRC based on user configuration "procChainCfg <nwPktCrcSel> CLI parameter.
 *          - Configures buffer size for Ping and Pong network buffers (DSS_PP, MSS_PP) based on the payload size.
 *      - Configures CPDMA Tx buffer descriptors. Two buffer descriptors are configured where first descriptor contains
 *       network header (UDP protocol) while second descriptor always points to the MSS network Packet buffer
 *      (containing Payload data) as shown in the diagram below.
 *      - Configures FHost CPDMA hardware controlled packet transmission feature.
 *      - Transmit the buffer descriptors in a circular queue and submit it to the CPDMA peripheral.
 *      - Enables UDP Checksum offloading feature of CPSW interface.
 *
 * Note - Network Packet total data (NW Header + Payload data) should be a multiple of 8 bytes for correct CRC computation. To ensure this, 2 bytes of dummy is added in network header.
 * For example: For a payload data of 1048 bytes (16 bytes Application header + 1024 bytes FFT data + 8 bytes Application Footer (RTI Timestamp)) and Network Header of 46 bytes (42 bytes NW header + 4 bytes of checksum info input to CPDMA), 2 bytes of dummy needs to be added;
 * such that 1048 + 46 + 2 = 1096, becomes a multiple of 8. If UDP checksum is not enabled, then CPDMA reads Network header of 42 bytes. In such a case, 6 dummy bytes are required to ensure a multiple of 8.
 *
 *  @image html CPSW_BufDesc.png "CPDMA TX Buffer Descriptors"
 *
 *
 *  @section CPSW CPSW Transfer of Radar Cube
 *
 *  Following two EDMA Channels are configured to transfer the compressed radar cube over ethernet.
 *
 *  1. EDMA Channel "EDMA_DSS_TPCC_A_EVT_CPSW_PP_TOGGLE":
 *    - On each trigger, this EDMA channel transfers one payload of data from L3 to DSS Network packet buffer.
 *    - EDMA is configured for HW trigger, so gets triggered each time Network packet buffer toggle happens.
 *    - This channel is configured as a three - link EDMA to handle the L3 memory re-use, that is to read back from the
 *      L3 starting location after a certain number of chirps. See section @ref output_hwa for more details.
 *
 *  2. CPSW Trigger EDMA Channel "EDMA_DSS_TPCC_A_EVT_FREE0":
 *    - This channel is chained to EDMA_DSS_TPCC_A_EVT_CPSW_PP_TOGGLE EDMA channel.
 *    - It is a three link EDMA channel. Three links are required for
 *       - First Link: On first trigger, writes the CPSW toggle value to the MSS_CTRL:HW_REG2 register. If the ping-pong select bit of MSS_CTRL:HW_REG3 register is set, then toggle value
 *          = 0x0111 is written. Else, toggle Value = 0x1011 is written.
 *       - Second Link: For next #PayloadsPerFrame number of EDMA triggers, this channel writes 1 (CPDMA TX channel#0) to the MSS_CTRL:CPSW_HW_TRIG_VAL
 *         register to trigger transfer of data from MSS Packet buffer.
 *              - If user configures per packet delay (> 0) using procChainCfg <ethPerPktDly> CLI, then this channel adds delay by transferring dummy data N times
 *                (proportional to <ethPerPktDly>). After the dummy transfers are completed, chained channel writes 1 (CPDMA TX channel#0) to the
 *                MSS_CTRL:CPSW_HW_TRIG_VAL register.
 *              - This delay spreads the packet transfers and may be helpful in the use-cases where lower (< 1Gbps) CPSW transfer rate is required depending on the
 *                capability of Rx side.
 *       - Third link: It is a dummy transfer required to break the infinite loop. An
 *         infinite loop is created here because each time CPSW is triggered, it generates the
 *         CPSW_PP_TOGGLE event, which triggers the CPSW_PP_TOGGLE channel, which in turn triggers this chained
 *         channel. This channel then again writes 1 to the CPSW trigger value register, which will then again
 *         trigger the CPSW_PP_TOGGLE channel and this goes on. Dummy transfer breaks this cycle by not triggering
 *         the CPSW and hence toggle also does not happen.
 *
 *  Note -
 *  1. Total number of packet transfers programmed for CPSW_PP_TOGGLE channel is equal to
 *  #PayloadsPerFrame + 2. These extra two transfers are required to accommodate the #PayloadsPerFrame + 2 transfers
 *  of the chained channel.
 *  2. Payload data = (APP_HEADER + 1D-FFT data + APP_FOOTER). 1D-FFT data referred in this equation is less than or
 *  equal to 1 chirp data, depending on the number of samples per chirp and maximum allowable packet size.
 *
 *  @image html cpsw_dataflow.png Compressed Radar Cube Over CPSW - Dataflow
 *
 *  The above figure shows the data flow for the transfer of compressed radar cube over CPSW. To understand the output data (compressed radar cube) format,
 *  refer @ref rpCompression section. First trigger to the CPSW_PP_TOGGLE channel is given manually. After that, subsequent DSS_PP buffer write and
 *  MSS_PP buffer read happens automatically without CPU intervention. If CPSW transfer rate is faster than L3 populating rate, it needs to be taken care that the
 *  first manual trigger is given after a few (ethPktRdyCnt) chirps of data is available in L3. This is configurable via
 *  CLI (procChainCfg <ethPktRdyCnt>) to ensure correct data gets transmitted data. This argument can be calculated by a simple calculation given by
 *  \n
 *  \f$(N_c - ethPktRdyCnt) * T_c = ((N_c * PacketsPerChirp * PacketSize * 8) / EthernetSpeed + ethPerPktDly).\f$
 *  \n
 *  The time taken to generate the radar cube for (N_c - ethPktRdyCnt) chirps
 *  should be equal to the time taken in exporting the entire compressed radar cube.
 *  Let N_c = 768, T_c = 30us, PacketSize = 1120 (1024+96) bytes, PacketsPerChirp = 1, EthernetSpeed = 900Mbps
 *  This gives ethPktRdyCnt = 514 (approx.). Thus, first trigger should be given after > 514 chirps in this case, keeping the above assumptions in mind.
 *
 *  To transmit 1 payload of data, around 96 bytes of overhead bytes is transmitted by the
 *  ethernet interface as described in the below diagram.
 *
 *  @image html EthernetPktFormat.png "Ethernet Packet Format"
 *
 *  @section resourceAlloc Hardware Resource Allocation
 *    The demo DPC needs to configure the DPU, Enet and LVDS hardware resources (EDMA).
 *    This resource allocation can be seen in the mmw_res.h file.
 *
 * @section LVDSStreamingNotes Streaming data over LVDS
 *
 *    The LVDS streaming feature enables the streaming of HW data (ADC data)
 *    through LVDS interface.
 *    The streaming is done mostly by the CBUFF and EDMA peripherals with minimal CPU intervention.
 *    The streaming is configured through the @ref MmwDemo_LvdsStreamCfg_t CLI command which
 *    allows control of HSI header, enable/disable of HW data and data format choice for the HW data. Note
 *    that only HW data without HSI header is supported as of now.
 *    The choices for data formats for HW data are:
 *      - @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED
 *      - @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC
 *
 *    When HW data LVDS streaming is enabled, the ADC data is streamed per
 *    chirp on every chirp event.
 *    -# For HW data, the inter-chirp duration should be sufficient to stream out the desired amount
 *       of data. For example, if the HW data-format is ADC and HSI header is enabled (note: HSI
 *       header sending is not enabled in this demo),
 *       then the total amount of data generated per chirp
 *       is:\n
 *        (numAdcSamples * numRxChannels * 4 (size of complex sample) +
 *       52 [sizeof(HSIDataCardHeader_t) + sizeof(HSISDKHeader_t)] )
 *       rounded up to multiples of 256 [=sizeof(HSIHeader_t)] bytes.\n
 *       The chirp time Tc in us = idle time + ramp end time in the profile configuration.
 *       For n-lane LVDS with each lane at a maximum of B Mbps,\n
 *       maximum number of bytes that can be send per chirp = Tc * n * B / 8 which
 *       should be greater than the total amount of data generated per chirp i.e\n
 *       Tc * n * B / 8 >= round-up(numAdcSamples * numRxChannels * 4 + 52, 256). \n
 *       E.g if n = 2, B = 600 Mbps, idle time = 7 us, ramp end time = 44 us, numAdcSamples = 512,
 *       numRxChannels = 4, then 7650 >= 8448 is violated so this configuration will not work.
 *       If the idle-time is doubled in the above example, then we have 8700 > 8448, so
 *       this configuration will work.
 *    -# The total amount of data to be transmitted in a HW packet must be greater than the
 *       minimum required by CBUFF, which is 64 bytes or 32 CBUFF Units (this is the definition
 *       CBUFF_MIN_TRANSFER_SIZE_CBUFF_UNITS in the CBUFF driver implementation).
 *       If this threshold condition is violated, the CBUFF driver will return an error during
 *       configuration and the demo will generate a fatal exception as a result.
 *       When HSI header is enabled, the total transfer size is ensured to be at least
 *       256 bytes, which satisfies the minimum. If HSI header is disabled, for the HW session,
 *       this means that numAdcSamples * numRxChannels * 4 >= 64. Although mmwavelink
 *       allows minimum number of ADC samples to be 2, the demo is supported
 *       for numAdcSamples >= 64. So HSI header is not required to be enabled for HW only case.
 *
 *   @subsection lvdsImpl Implementation Notes
 *
 *    -# The LVDS implementation is mostly present in
 *      mmw_lvds_stream.c with calls in mss_main.c. Additionally HSI
 *      clock initialization is done at first time sensor start using @ref MmwDemo_mssSetHsiClk. Also see the
 *      @ MmwDemo_BoardInit function for register configuration related to HSI clock.
 *    -# EDMA channel resources for CBUFF/LVDS are in the global resource file
 *      (mmw_res.h, see @ref resourceAlloc) along with other EDMA resource allocation.
 *    -# Although the CBUFF driver is configured for two sessions (hw and sw),
 *      at any time only one can be active. So depending on the LVDS CLI configuration
 *      and whether advanced frame or not, there is logic to activate/deactivate
 *      HW and SW sessions as necessary.
 *    -# The CBUFF session (HW/SW) configure-create and delete
 *      depends on whether or not re-configuration is required after the
 *      first time configuration.
 *      -# For HW session, re-configuration is done during sub-frame switching to
 *        re-configure for the next sub-frame but when there is no advanced frame
 *        (number of sub-frames = 1), the HW configuration does not need to
 *        change so HW session does not need to be re-created.
 *    -# SW-trigger-based streaming is not supported in the current release.
 *
 *    The following figure shows a timing diagram for the LVDS streaming
 *    (the figure is not to scale as actual durations will vary based on configuration).
 *    Note: SW streaming and advance frame (subframe-switching) is not supported in the current version of the demo.
 *
 *      @image html lvdstiming.png "LVDS timing diagram"
 *
 * @subsection cgStatic Power Reduction Techniques: Clock Gating Static
 *
 * **Clock Gate unused peripherals**
 *
 *    The device includes several peripherals and modules, and several instances of them. Depending on the application,
 *    some of these may be active while the others remain totally unused. The unused peripherals (RTIs, SPIs, I2C, UARTs)
 *    can be clock-gated to save power. Further, not all the HSDIVs and not all outputs of each HSDIV need to be active.
 *
 * SDK feature:
 * - **Clock Gate unused peripherals**:
 *    Provided example clock gates the below peripherals:\n
 *    MSS: SPIB , I2C, SCIB, WDT, RTIB \n
 *    OBSCLKOUT, PMICCLKOUT, TRCCLKOUT, MCUCLKOUT
 *
 *    Reference CLI in the SDK: unusedPerClkGate (Section 3.7); User guide for default HSDIV configuration (Section 3.9.2)
 *
 *  @subsection fsdyn Power Reduction Techniques: Frequency Scaling Dynamic
 *
 * **BSS**
 *
 * The TI Firmware supports dynamic frequency scaling by enabling the "BSS Underclocking feature" . This would require\n
 *  1. performing the necessary clock configurations of the RSS clock and FRC clock sources\n
 *  2. reserving MSS RTIC for BSS use for maintaining ticks across the modes\n
 *  3. handling the functional safety aspects of FRC and WDT\n
 *      a. This can be achieved by performing a "Logical monitoring of the Frame timing" in the application\n
 *  4. Enabling the feature and necessary configurations before unhalting the BSS core\n
 *      a. Refer to the DFP ICD for more details\n
 *      b. Refer to SDK User Guide section Default SBL Clock configuration (Section 3.9.2)
 *
 * SDK Features:
 *
 *  - **BSS Dynamic clocking**: The BSS clock source switches to XTAL(50 MHz) clock when the core is idle. This feature is enabled in the SBL before unhalting BSS.
 *  Hence this feature cannot be configured through CLI. Refer SOC_rcmPopulateBSSControl API in mcu_plus_sdk_awr2544_<ver>\source\drivers\soc\awr2544\soc_rcm.c
 *  to enable or disable BSS Dynamic clocking feature.
 * Note: MSS RTIC is used by the BSS when Dynamic clocking feature is enabled.
 *
 *  @subsection cgDyn Power Reduction Techniques : Clock Gating Dynamic
 *
 * Dyanamic clock gating is divided into 2 categories:
 * 1. Clock gating of processing cores by invocation of WFI instruction\n
 *      - Applicable cores:\n
 *          - MSS ARM Cortex R5F
 *          - HSM ARM Cortex M4F\n
 *      - The HW based clock gating of the logic is supported. Any interrupt would ungate the logic and wakeup the core.
 * 2. Explicit clock gating of modules by the control core\n
 *      - The clock gating and ungating is controlled by a control processing core.
 *
 * SDK Features:
 *
 *  - **HWA Dynamic clock gating**: It enables the capability to clock gate the Radar Accelerator core IPs (FFT datapath, Memory compression)
 *  based on the ParamSet being executed.
 *
 *    Reference CLI in the SDK: hwaDynamicClockGating (Section 3.7);
 *
 *  - **HWA Clock gate after frame processing**: Clock gates HWA after the frame processing is completed. Clock is ungated in frame start ISR.
 *
 *    Reference CLI in the SDK: hwaGateAfterFrameProc (Section 3.7);
 *
 * @subsection pg Power Reduction Techniques: Power Gating
 *
 * @subsubsection hwapg HWA Dynamic power gating
 *
 * In addition to dynamic clock gating, the HWA's power supply can be gated off using a power-switch dedicated to the HWA.
 * In comparison with the dynamic under-clocking explained earlier, dynamic power-gating needs a reconfiguration of the HWA, consumes
 * more state-transition time (a few micro-seconds) but provides some more power saving.
 *
 * SDK feature:
 *
 * - **HWA Power gate after frame processing**: HWA is Powered down after the range processing and compression is completed. In the demo,
 *    RTIB timer is used to wake up HWA and reconfigure 50us before next frame start. With the profile_3d_3Azim_1ElevTx_DDM_awr2544_2Xmode configuration,
 *    HWA Power Up and reconfiguration profiled time is ~25us.
 *    HWA reconfiguration time depends on the profile configuration given by the user. Accordingly, paramset and window RAM size varies.
 *    User has to ensure that HWA is powered up and reconfigured before the first chirp time of the next frame.
 *
 *    Reference CLI in the SDK: hwaGateAfterFrameProc (Section 3.7);
 *
 *  @image html HWA_PowerSaving.png "HWA Power Save Timing Diagram"
 *
 * @subsubsection mss MSS Loading
 *
 *    The feature can be used for benchmarking power consumed and savings from the optimization hooks.\n
 *    The MSS Loading feature creates a low priority task which starts execution at the frame start. Based on the MSS
 *    Loading percentage given by the user through CLI, the MSS loading time is calculated. It performs dummy matrix multiplication until
 *    the MSS Loading Time elapses and then the MSS Loading task ends. MSS enters IDLE state (WFI) after this.
 *
 *    Reference CLI in the SDK: powerMeasMssLoading (Section 3.7);
 *
 * @subsubsection tm Temperature monitoring
 *
 *  While measuring power, the temperature has to be monitored as the power consumed is dependent on the operating temperature. To monitor temperature, the following functions are available:
 *
 *  - **Digital Temperature Read**:
 *    Querying the MCU, HWA and HSM temperature values
 *
 *    Reference CLI in the SDK: digTempRead (Section 3.7);
 *
 *  - **Analog Temperature Read**:
 *    Queries the front end temperature sensor data (Tx and Rx temperature values)
 *
 *    Reference CLI in the SDK: anaTempRead (Section 3.7);
 *
 * @subsubsection vm Voltage monitoring and self tests
 *
 * - Monitoring voltage rails is important to discover faults of the power source. An under voltage (UV) or over voltage (OV) fault may lead to an undefined behavior.\n
 * - The threshold voltage value is set (Reference Voltage). An ESM error is generated when any faults occurs. \n
 * - UV Monitors generate error if the supplied voltage is below this threshold. OV Monitors generate error when supplied voltage goes above threshold.\n
 * - VMONs have a self test mode feature built in. This allows for self diagnostic testing as part of the VMON enablement and/or during the functional operation of the VMONs.\n
 * The intended use case for this test mode is to check whether the output of the VMON can toggle before fully enabling the VMON to be used.\n
 * The test mode can be enabled at anytime and will not reset the device or disturb the performance of the VMON.\n
 *
 * - There are MSS (1.2V UV, 1.2V OV, 1.8V OSC UV, 3.3V UV) and BSS Voltage monitors (VddaBb 1.8V UV, VddaVco 1.8V UV, VddRf1 1V UV, VddRf2 1V UV).
 *
 *    Reference CLIs in the SDK: coexMSSVMONSelfTest, coexMSSVMONEnable, coexBSSVMONEnable, coexMSSVMONDisable (Section 3.7);
 *
 *  @section bypassCLI How to bypass CLI
 *
 *    Re-implement the file mmw_cli.c as follows:
 *
 *    -# @ref MmwDemo_CLIInit should just create a task with input taskPriority.
 *       Lets say the task is called "MmwDemo_sensorConfig_task".
 *    -# All other functions are not needed
 *    -# Implement the MmwDemo_sensorConfig_task as follows:
 *       - Fill gMmwMssMCB.cfg.openCfg
 *       - Fill gMmwMssMCB.cfg.ctrlCfg
 *       - Add profiles and chirps using @ref MMWave_addProfile and @ref MMWave_addChirp functions
 *       - Call @ref MmwDemo_CfgUpdate for every offset in @ref configStoreOffsets
 *         (MMWDEMO_xxx_OFFSET in mmw_mss.h)
 *       - Fill gMmwMssMCB.objDetCommonCfg.preStartCommonCfg
 *       - Call @ref MmwDemo_openSensor
 *       - Call @ref MmwDemo_configSensor
 *       - Call @ref MmwDemo_startSensor (One can use helper function
 *         @ref MmwDemo_isAllCfgInPendingState to know if all dynamic config was provided)
 *    -# The user can also use the CLI_BYPASS API in the CLI library to directly bypass the CLI
 *       command send over UART.
 *
 *  @section error Note on Error Codes
 *
 *    ------------------------------
 *    When demo runs into error conditions, an error code will be generated and printed out.
 *    Error code is defined as a negative integer. It comes from the following categories:
 *    - Drivers
 *    - Control modules
 *    - Data Processing Unit
 *    - Demo
 *
 *    The error code is defined as (Module  error code base - Module specific error code).\n
 *    The base error code for the above modules can be found in @ref mmwave_error.h\n
 *    The base error code for DPC and DPU can be found in @ref dp_error.h
 *
 *    Module specific error code is specified in the module's header file.
 *    Examples:
 *       - UART driver error code is defined in uart.h
 *       - DPU error code is defined in the dpu used in demo
 *
 *  @subsection mmwave_error mmWave module Error Code
 *    Error code from mmWave module is encoded in the following manner:
 *
 *    Bits(31::16)  |  Bits(15::2)   | Bits (1::0)
 *    :-------------|:----------------|:-------------:
 *    mmwave  error  | Subsystem error   | error level
 *
 *    - mmwave error is defined in mmwave.h \n
 *    - Subsystem error is returned from sub-system such as mmwavelink and mailbox driver. \n
 *    - Error level is referred as WARNING level and ERROR level.
 *    - mmWave exposes an API - MMWave_decodeError() that can be used in demo to decode error code
 *
 *  @subsection mmwave_error_example Example
 *    - Here is an example on how to parse the error code - "-30111"\n
 *        -# The error code is from module with error base "-30000", which indicates it is DPU error.\n
 *        -# By referring to @ref dp_error.h, base "-30100" is from range DPU.\n
 *        -# Then find the error code in rageprochwaReal2x.h for error(-11) as DPU_RANGEPROCHWA_ERADARCUBE_DATASIZE
 *
 *    - Another example is with mmWave control module: - "mmWave Config failed [Error code: -3109 Subsystem: 71]"\n
 *        -# The above message indicates the error is from module(-3100 ->mmwave) with error -9(MMWAVE_ECHIRPCFG)\n
 *        -# The subsystem(mmwavelink) error is 71(RL_RET_CODE_CHIRP_TX_ENA_1INVAL_IN) which can be found in mmwavelink.h
 *
 */

/* ========================================================================= */
/*                             Include Files                                 */
/* ========================================================================= */

/* MCU+SDK include files. */
#include <drivers/uart.h>
#include <kernel/dpl/AddrTranslateP.h>

/* mmWave SDK Include Files: */
#include <ti/common/syscommon.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>

/* Demo Include Files */
#include <ti/demo/awr2544/mmw/mmw_common.h>
#include <ti/demo/utils/mmwdemo_rfparser.h>
#include <ti/demo/utils/mmwdemo_flash.h>


/* ========================================================================= */
/*                           Macros & Typedefs                               */
/* ========================================================================= */

/**
 * @brief Task Priority settings:
 * Mmwave task is at higher priority because of potential async messages from
 * BSS that need quick action in real-time. CLI task must be at a lowest
 * priority.
 */
#define MMWDEMO_CLI_TASK_PRIORITY 6
#define MMWDEMO_DPC_TASK_PRIORITY 7
#define MMWDEMO_CTRL_TASK_PRIORITY 9
#define MMWDEMO_INIT_TASK_PRI (3U)

#ifdef POWER_MEAS
#define MMWPM_MSSLOADING_TASK_PRIORITY  (2U)
#endif

#if (MMWDEMO_CLI_TASK_PRIORITY >= MMWDEMO_DPC_TASK_PRIORITY)
#error CLI task priority must be < DPC task priority
#endif

/* FreeRTOS Task declarations. */
#define MMWDEMO_INIT_TASK_STACK_SIZE (1 * 1024U)
#define MMWDEMO_CTRL_TASK_STACK_SIZE (2 * 1024U)
#define MMWDEMO_DPC_TASK_STACK_SIZE (2 * 1024U)

#ifdef POWER_MEAS
#define MMWPM_MSSLOADING_TASK_STACK_SIZE (1 * 1024U)
#endif

/**
 * \brief These address offsets are in bytes, when configure address offset in
 * hardware, these values will be converted to number of 128bits.
 * Buffer at offset 0x0U is reserved by BSS, hence offset starts from 0x200
 */
#define MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET 0x200U
#define MMW_DEMO_CQ_RXSAT_ADDR_OFFSET 0x400U

/*! \brief CQ data is at 16 bytes alignment for multiple chirps */
#define MMW_DEMO_CQ_DATA_ALIGNMENT 16U

#define MAX_MOD_FREQ_DIVIDER_MANTISSA 127U

/*! \brief Each Packet Headaer and Footer Size */
#define MMW_DEMO_APP_HEADER_SIZE 16U
#define MMW_DEMO_APP_FOOTER_SIZE 8U

/*! \brief Maximum Payload Size */
#define MMW_DEMO_MAX_PAYLOAD_SIZE 1536U

/*! \brief For advanced frame config, below define means the configuration
 * given is global at frame level and therefore it is broadcast to all
 * sub-frames. */
#define MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG (-1)

/*! \brief For the ADCBufData.dataProperty.adcBits field. */
#define ADCBUF_DATA_PROPERTY_ADCBITS_16BIT (2)

/*! \brief L3 RAM buffer size in bytes */
/* 5KB less is allocated to handle the configurations which fits
 * exactly in L3 size. The issue CPSW_PP_TOGGLE channel configuration
 * where CCNT = #Payloads + 2.
 */
#define MMWDEMO_L3RAM_SIZE (CSL_DSS_L3_U_SIZE - (5*1024U))

/*! \brief This is required only to store the window buffer for range proc*/
#define MMWDEMO_L2RAM_SIZE (5U * 1024U)

/*! \brief  Calibration Data Save/Restore defines */
#define MMWDEMO_CALIB_FLASH_SIZE 4096
#define MMWDEMO_CALIB_STORE_MAGIC (0x7CB28DF9U)

/*! \brief  Error Codes */
#define MMW_DEMO_EINVAL_FFT_MODE2X  (MMWAVE_ERRNO_MMWDEMO_DPCONFIG_BASE-1)
#define MMW_DEMO_EINVAL_PAYLOAD_SIZE    \
                                    (MMWAVE_ERRNO_MMWDEMO_DPCONFIG_BASE-2)
#define MMW_DEMO_EINVAL_L3REUSE_ITERATIONS    \
                                    (MMWAVE_ERRNO_MMWDEMO_DPCONFIG_BASE-3)


/* ========================================================================= */
/*                        Static Function Declaration                        */
/* ========================================================================= */

/* MmWave demo functions for datapath operation */
static void MmwDemo_dataPathOpen(void);
static void MmwDemo_computePayloadCfg(MmwDemo_staticDPCCfg *staticCfg);
static int32_t MmwDemo_dataPathConfig(void);
static void MmwDemo_dataPathStop(void);

/* Static Utils */
static int32_t MmwDemo_getNumEmptySubBands(uint32_t numTxAntennas);
static int32_t MmwDemo_configPhaseShifterChirps(void);
static uint32_t MmwDemo_compSscFactCtrlVal(const uint32_t refClk,
            const uint16_t dpllM, MmwDemo_spreadSpectrumConfig *ptrdpllCfg);

/* Calibration save/restore APIs */
static int32_t MmwDemo_calibInit(void);
static int32_t MmwDemo_calibSave(MmwDemo_calibDataHeader *ptrCalibDataHdr, \
                                 MmwDemo_calibData *ptrCalibrationData);
static int32_t MmwDemo_calibRestore(MmwDemo_calibData *calibrationData);

/* CQ config function. */
static int32_t MmwDemo_configCQ(MmwDemo_SubFrameCfg *subFrameCfg,
                                uint8_t numChirpsPerChirpEvent,
                                uint8_t validProfileIdx);

static int32_t MmwDemo_eventCallbackFxn(uint8_t devIndex, uint16_t msgId, \
                            uint16_t sbId, uint16_t sbLen, uint8_t *payload);

static void MmwDPC_chirpAvailISR(void* arg);
static int32_t MmwDemo_registerChirpStartInterrupt(void);
static int32_t MmwDemo_registerFrameStartInterrupt(void);

static int32_t MmwDemo_mmWaveCtrlStop(void);
static void MmwDemo_sensorStopEpilog(void);

/* Task Functions */
static void MmwDemo_mmWaveCtrlTask(void *args);
static void MmwDemo_DPCTask(void *args);
static void MmwDemo_initTask(void *args);

static uint32_t MmwDemo_getGenfPeriodicity(bool* enableTsnStack);


/* ========================================================================= */
/*                            Global Variables                               */
/* ========================================================================= */

/*! \brief Application task stack variables */
StackType_t MmwDemoMainTskStack[MMWDEMO_INIT_TASK_STACK_SIZE] \
                                                __attribute__((aligned(32)));
StackType_t MmwDemoCtrlTskStack[MMWDEMO_CTRL_TASK_STACK_SIZE] \
                                                __attribute__((aligned(32)));
StackType_t MmwDemoDPCTskStack[MMWDEMO_DPC_TASK_STACK_SIZE] \
                                                __attribute__((aligned(32)));

#ifdef POWER_MEAS
StackType_t gMmwDemo_MssLoadingTaskStack[MMWPM_MSSLOADING_TASK_STACK_SIZE] \
                                                __attribute__((aligned(32)));
#endif
/*! \brief Global Variable for tracking information required by the mmw Demo */
MmwDemo_MSS_MCB gMmwMssMCB;

/* Memory Buffers */
uint8_t gMmwL3Ram[MMWDEMO_L3RAM_SIZE] __attribute__((section(".bss.dss_l3")));
uint8_t gMmwL2Ram[MMWDEMO_L2RAM_SIZE] __attribute__((section(".bss")));

/*! \brief Calibration Data Storage */
MmwDemo_calibData gCalibDataStorage __attribute__((aligned(8)));

extern MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg;

/*! \brief Frame Start Hardware Interrupt Object */
HwiP_Object MMwDemo_FrameStartHwiObject;

/*! \brief Chirp Available Hardware Interrupt Object */
HwiP_Object MMwDemo_ChirpAvailHwiObject;

/* ========================================================================= */
/*                          Function Definitions                             */
/* ========================================================================= */


CSL_mss_toprcmRegs* CSL_MSS_TOP_RCM_getBaseAddress (void)
{
    return (CSL_mss_toprcmRegs*) CSL_MSS_TOPRCM_U_BASE;
}

static void MmwDemo_dataPathOpen(void)
{
    gMmwMssMCB.adcBufHandle = MmwDemo_ADCBufOpen();
    if (gMmwMssMCB.adcBufHandle == NULL)
    {
        MmwDemo_debugAssert(0);
    }
}


static void MmwDemo_computePayloadCfg(MmwDemo_staticDPCCfg *staticCfg) __attribute__((optnone))
{
    float achievedCompressionRatio;
    uint32_t outputBytesPerBlock, inputBytesPerBlock, samplesPerBlock;
    uint32_t numBlocksPerChirp;
    uint32_t radarCubeCompressedSizeInBytes, dataSizePerChirp, maxChirpsInL3;
    uint32_t totalChirps, iter, maxPayloadSize, numBlocksPerPayload;
    uint32_t numL3ReUseIterations;

    if (staticCfg->compressionCfg.compressionMethod == 1U)
    {
        samplesPerBlock = staticCfg->compressionCfg.rangeBinsPerBlock;
    }
    else
    {
        samplesPerBlock =
                    staticCfg->compressionCfg.numRxAntennaPerBlock * \
                    staticCfg->compressionCfg.rangeBinsPerBlock;
    }

    inputBytesPerBlock = 4U * samplesPerBlock;
    /* 32-bit boundary aligned */
    outputBytesPerBlock = (MATHUTILS_CEILING_POS_FLOAT(inputBytesPerBlock * staticCfg->compressionCfg.compressionRatio / 4.0f)) * 4U;
    achievedCompressionRatio = (float)outputBytesPerBlock / (float)inputBytesPerBlock;
    /* Compressed radar cube size */
    radarCubeCompressedSizeInBytes = staticCfg->numRangeBins *
            staticCfg->numChirpsPerFrame * staticCfg->ADCBufData.dataProperty.numRxAntennas * sizeof(cmplx16ReIm_t) * achievedCompressionRatio;
    /* number of compressed blocks per chirp */
    numBlocksPerChirp = (staticCfg->numRangeBins * staticCfg->ADCBufData.dataProperty.numRxAntennas) / samplesPerBlock;
    /* max data that transmit biffer can accomodate other than header and footer */
    maxPayloadSize = MMW_DEMO_MAX_PAYLOAD_SIZE - (staticCfg->appHeaderSize + staticCfg->appFooterSize);
    numBlocksPerPayload = floor((float)maxPayloadSize / (float)outputBytesPerBlock);

    /* total number of payload required to transmit one chirp data */
    staticCfg->numPayloads = ceil((float)numBlocksPerChirp / numBlocksPerPayload);
    numBlocksPerPayload = numBlocksPerChirp / staticCfg->numPayloads;
    if ((numBlocksPerPayload * outputBytesPerBlock) > maxPayloadSize)
    {
        /* Data Size per Payload exceeds the maximum payload size */
        CLI_write("Error: Payload data config failed with error[%d]\r\n", MMW_DEMO_EINVAL_PAYLOAD_SIZE);
        MmwDemo_debugAssert(0);
    }

    /*radarCube datasize*/
    staticCfg->radarCubeDataSize = radarCubeCompressedSizeInBytes +
                                    ((staticCfg->appHeaderSize + staticCfg->appFooterSize) * staticCfg->numChirpsPerFrame *
                                    staticCfg->numPayloads);

    /* Calculate L3 Re-Use Iterations and Chirps per Iterations */
    numL3ReUseIterations = ceil((float)staticCfg->radarCubeDataSize / (float)MMWDEMO_L3RAM_SIZE);
    dataSizePerChirp = staticCfg->radarCubeDataSize / staticCfg->numChirpsPerFrame;
    /* maxChirps that can allocated in L3; should be a multiple of 2 */
    maxChirpsInL3 = floor(floor(MMWDEMO_L3RAM_SIZE / dataSizePerChirp) / 2U) * 2U;
    if (numL3ReUseIterations > RANGEPROCHWA_L3REUSE_MAX_ITERATIONS)
    {
        /* Limit on Max iterations of L3 because of EDMA Channel linking */
        CLI_write("Error: L3 Reuse Config failed with error[%d]\r\n", MMW_DEMO_EINVAL_L3REUSE_ITERATIONS);
        MmwDemo_debugAssert(0);
    }
    else
    {
        iter = 0U;
        totalChirps = staticCfg->numChirpsPerFrame;
        while (totalChirps > 0U)
        {
            staticCfg->numChirpsEachIter[iter] = (totalChirps > maxChirpsInL3) ? maxChirpsInL3 : totalChirps;
            totalChirps -= staticCfg->numChirpsEachIter[iter++];
        }
    }
}


/*! \brief The function is used to configure the data path based on the chirp
 *      profile. After this function is executed, the data path processing
 *      will ready to go when the ADC buffer starts receiving samples
 *      corresponding to the chirps.
 */
static int32_t MmwDemo_dataPathConfig(void)
{
    int32_t errCode;
    MMWave_CtrlCfg *ptrCtrlCfg;
    MmwDemo_SubFrameCfg *subFrameCfg;
    int8_t subFrameIndx;
    MmwDemo_RFParserOutParams RFparserOutParams;
    MmwDemo_preStartDPCCfg objDetPreStartCfg;
    MmwDemo_memUsageDPC *memUsage;
    MmwDemo_staticDPCCfg *staticCfg;
    float rfFreqScaleFactor;

    /* Get data path object and control configuration */
    ptrCtrlCfg = &gMmwMssMCB.cfg.ctrlCfg;

    memset((void *)&objDetPreStartCfg, 0, sizeof(MmwDemo_preStartDPCCfg));

    staticCfg = &objDetPreStartCfg.staticCfg;

    gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
        MmwDemo_RFParser_getNumSubFrames(ptrCtrlCfg);

    MmwDPC_preStartCommonConfig(gMmwMssMCB.dpcHandle, \
                    &gMmwMssMCB.objDetCommonCfg.preStartCommonCfg, &errCode);
    if (errCode < 0)
    {
        CLI_write("Error: Unable to do common configurations [Error:%d]\r\n",
                    errCode);
        goto exit;
    }

    /* For 77GHz Automotive */
    rfFreqScaleFactor = 3.6;

    /* Insted of configuring in normal order, we configure in reverse order
    *  so that subFrame 0 is configured at the last so that EDMA channel
    *  parameters of first subframe remain configured and not overwritten.
    */
    for (
    subFrameIndx=gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames- 1;
    subFrameIndx >= 0; subFrameIndx--)
    {
        subFrameCfg = &gMmwMssMCB.subFrameCfg[subFrameIndx];

        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        errCode = MmwDemo_RFParser_parseConfig(&RFparserOutParams,
                                               subFrameIndx,
                                               &gMmwMssMCB.cfg.openCfg,
                                               ptrCtrlCfg,
                                               &subFrameCfg->adcBufCfg,
                                               rfFreqScaleFactor,
                                               false, gMmwMssMCB.objDetCommonCfg.procChain);
        if (errCode != 0)
        {
            CLI_write("Error: MmwDemo_RFParser_parseConfig [Error:%d]\r\n",
                        errCode);
            goto exit;
        }

        subFrameCfg->numRangeBins = RFparserOutParams.numRangeBins;
        /* Workaround for range DPU limitation for FFT size 1024 and 12
         * virtual antennas case*/
        if ((RFparserOutParams.numVirtualAntennas == 12) &&
                                (RFparserOutParams.numRangeBins == 1024))
        {
            subFrameCfg->numRangeBins = 1022;
            RFparserOutParams.numRangeBins = 1022;
        }
        subFrameCfg->datapathStaticCfg.compressionCfg.numRxAntennaPerBlock =
                                            RFparserOutParams.numRxAntennas;
        subFrameCfg->numDopplerBins = RFparserOutParams.numDopplerBins;
        subFrameCfg->numChirpsPerChirpEvent =
                                    RFparserOutParams.numChirpsPerChirpEvent;
        subFrameCfg->adcBufChanDataSize = RFparserOutParams.adcBufChanDataSize;
        subFrameCfg->numAdcSamples = RFparserOutParams.numAdcSamples;
        subFrameCfg->numChirpsPerSubFrame =
                                        RFparserOutParams.numChirpsPerFrame;
        subFrameCfg->numVirtualAntennas = RFparserOutParams.numVirtualAntennas;
        gMmwMssMCB.framePeriod = RFparserOutParams.framePeriod;

        /* Initialize RTIB Timer for HWA Wakeup (50us before next frame start) */
        if(gMmwMssMCB.powerMeas.isHwaGateAfterFrameProc == MMWDEMO_HWA_POWER_GATE)
        {
            MmwDPC_hwaPGTimerInit();
        }

        errCode = MmwDemo_ADCBufConfig(gMmwMssMCB.adcBufHandle,
                        gMmwMssMCB.cfg.openCfg.chCfg.rxChannelEn,
                        subFrameCfg->numChirpsPerChirpEvent,
                        subFrameCfg->adcBufChanDataSize,
                        &subFrameCfg->adcBufCfg,
                        &staticCfg->ADCBufData.dataProperty.rxChanOffset[0]);
        if (errCode < 0)
        {
            CLI_write("Error: ADCBuf config failed with error[%d]\r\n", errCode);
            MmwDemo_debugAssert(0);
        }

        errCode = MmwDemo_configCQ(subFrameCfg, \
                    subFrameCfg->numChirpsPerChirpEvent, \
                    RFparserOutParams.validProfileIdx);
        if (errCode < 0)
        {
            CLI_write("Error: CQ config failed with error[%d]\r\n", errCode);
            MmwDemo_debugAssert(0);
        }

        /* DPC pre-start config */
        {
            objDetPreStartCfg.subFrameNum = subFrameIndx;

            /* Fill static configuration */
            staticCfg->ADCBufData.data = (void *)CSL_RSS_ADCBUF_READ_U_BASE;
            staticCfg->ADCBufData.dataProperty.adcBits =
                            ADCBUF_DATA_PROPERTY_ADCBITS_16BIT; /* 16-bit */

            /* only real format supported */
            MmwDemo_debugAssert(subFrameCfg->adcBufCfg.adcFmt == 1);

            staticCfg->ADCBufData.dataProperty.dataFmt =
                                                        DPIF_DATAFORMAT_REAL16;

            if (subFrameCfg->adcBufCfg.chInterleave == 0)
            {
                staticCfg->ADCBufData.dataProperty.interleave =
                                                DPIF_RXCHAN_INTERLEAVE_MODE;
            }
            else
            {
                staticCfg->ADCBufData.dataProperty.interleave =
                                            DPIF_RXCHAN_NON_INTERLEAVE_MODE;
            }
            staticCfg->ADCBufData.dataProperty.numAdcSamples =
                                            RFparserOutParams.numAdcSamples;
            staticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent =
                                    RFparserOutParams.numChirpsPerChirpEvent;
            staticCfg->ADCBufData.dataProperty.numRxAntennas =
                                            RFparserOutParams.numRxAntennas;
            staticCfg->ADCBufData.dataSize = RFparserOutParams.numRxAntennas*\
                       RFparserOutParams.numAdcSamples * sizeof(int16_t);
            staticCfg->numChirpsPerFrame = RFparserOutParams.numChirpsPerFrame;
            staticCfg->numChirps = RFparserOutParams.numDopplerChirps;
            staticCfg->numRangeBins = RFparserOutParams.numRangeBins;
            staticCfg->isMode2x = gMmwMssMCB.objDetCommonCfg.is2xMode;

            if((staticCfg->isMode2x==1) && (RFparserOutParams.adcDataFmtIsReal==0))
            {
                /*2X mode should be enabled only with real only data */
                CLI_write("Error: 2X Mode Config failed with error[%d]\r\n", MMW_DEMO_EINVAL_FFT_MODE2X);
                MmwDemo_debugAssert(0);
            }

            /* Number of range bins are half the number of FFT Bins in case of
             * real only chirp data with 1X mode */
            if ((!RFparserOutParams.adcDataFmtIsReal)||(staticCfg->isMode2x))
            {
                /* FFT size is equal to range bins in case of complex input
                 * or 2x mode */
                staticCfg->numRangeFFTBins = (RFparserOutParams.numRangeBins);
            }
            else
            {
                staticCfg->numRangeFFTBins =
                                        (RFparserOutParams.numRangeBins) * (2);
            }

            staticCfg->numTxAntennas = RFparserOutParams.numTxAntennas;
            staticCfg->numVirtualAntennas =
                                        RFparserOutParams.numVirtualAntennas;

            memcpy(&staticCfg->compressionCfg, \
                            &subFrameCfg->datapathStaticCfg.compressionCfg, \
                            sizeof(DPU_RangeProcHWA_CompressionCfg));
            memcpy(&staticCfg->intfStatsdBCfg,  \
                            &subFrameCfg->datapathStaticCfg.intfStatsdBCfg, \
                            sizeof(DPU_RangeProcHWA_intfStatsdBCfg));

            /* Common Calculation */
            staticCfg->appHeaderSize = MMW_DEMO_APP_HEADER_SIZE;
            staticCfg->appFooterSize = MMW_DEMO_APP_FOOTER_SIZE;

            MmwDemo_computePayloadCfg(staticCfg);

            if(staticCfg->numChirpsEachIter[0] < staticCfg->numChirpsPerFrame)
            {
                /* Check the ethPktRdyCnt : should be less than the max chirps stored in L3 (buffer size) */
                if(gMmwMssMCB.objDetCommonCfg.ethPktRdyCnt >= staticCfg->numChirpsEachIter[0])
                {
                    CLI_write("Error: Incorrect Packet Ready Count Config. \r\n");
                    MmwDemo_debugAssert(0);
                }

                /* calculate the buffer size required for the enet transmit delay configured */
                float packetTxTime = (staticCfg->radarCubeDataSize * 8.0f * 1e-3)/ (staticCfg->numChirpsPerFrame * staticCfg->numPayloads);

                /* Buffer Chirps = (L3 Reading Out Time - L3 Writing Time) / CHirp Interval */
                int16_t bufChirps = ((staticCfg->numChirpsPerFrame * staticCfg->numPayloads * (packetTxTime + gMmwMssMCB.objDetCommonCfg.ethPerPktDly)) / \
                                    (RFparserOutParams.chirpInterval * 1e6) ) - (staticCfg->numChirpsPerFrame - gMmwMssMCB.objDetCommonCfg.ethPktRdyCnt);

                if(bufChirps >= staticCfg->numChirpsEachIter[0])
                {
                    CLI_write("Error: Incorrect Per Packet Delay, try reducing the delay. \r\n");
                    MmwDemo_debugAssert(0);
                }
            }

            DebugP_logInfo("App: Issuing Pre-start Config IOCTL (subFrameIndx = %d)\r\n", subFrameIndx);

            MmwDPC_preStartConfig(gMmwMssMCB.dpcHandle, &objDetPreStartCfg, &errCode);
            if (errCode < 0)
            {
                CLI_write("Error: Unable to configure [Error:%d]\r\n", errCode);
                goto exit;
            }

#ifndef BSS_LOGGER
#ifdef POWER_MEAS
            if(gMmwMssMCB.sensorState == MmwDemo_SensorState_STOPPED)
            {
                /* Free Ethernet Tx packet before re-allocation.
                 * If condition is required here to de-allocate only when
                 * first time allocation is done.
                 */
                MmwEnet_freeEthTxPkt();
            }

            MmwEnet_config(staticCfg, gMmwMssMCB.objDetCommonCfg.nwPktCrcSel);
#else
            if(gMmwMssMCB.sensorState == MmwDemo_SensorState_STOPPED)
            {
                /* Free Ethernet Tx packet before re-allocation.
                 * If condition is required here to de-allocate only when
                 * first time allocation is done.
                 */
                EnetApp_freeEthTxPkt();
            }
            EnetApp_config(staticCfg, gMmwMssMCB.objDetCommonCfg.nwPktCrcSel);
#endif
#endif
            memUsage = &objDetPreStartCfg.memUsage;

            CLI_write("============ Heap Memory Stats ============\r\n");
            CLI_write("%20s %12s %12s %12s %12s\r\n", " ", "Size", "Used", "Free", "DPCUsed");
            CLI_write("%20s %12d %12d %12d %12d\r\n", "System Heap(L2)",
                       memUsage->SystemHeapTotal, memUsage->SystemHeapUsed,
                       memUsage->SystemHeapTotal - memUsage->SystemHeapUsed,
                       memUsage->SystemHeapDPCUsed);

            CLI_write("%20s %12d %12d %12d\r\n", "L3",
                       memUsage->L3RamTotal,
                       memUsage->L3RamUsage,
                       memUsage->L3RamTotal - memUsage->L3RamUsage);

            CLI_write("%20s %12d %12d %12d\r\n", "localRam(L2)",
                       memUsage->CoreLocalRamTotal,
                       memUsage->CoreLocalRamUsage,
                       memUsage->CoreLocalRamTotal - memUsage->CoreLocalRamUsage);
        }
    }
exit:
    return errCode;
}


static void MmwDemo_dataPathStop(void)
{
    int32_t errCode = 0;
    test_print("App: Issuing DPC_stop\r\n");
    MmwDPC_stop(gMmwMssMCB.dpcHandle, &errCode);
    if (errCode < 0)
    {
        CLI_write("DPC_Stop failed[Error code %d]\r\n", errCode);
        MmwDemo_debugAssert(0);
    }
}

static int32_t MmwDemo_getNumEmptySubBands(uint32_t numTxAntennas)
{
    int32_t numBandsEmpty;
    /* Empty subbands */
    switch (numTxAntennas)
    {
    case 2:
        numBandsEmpty = 1;
        break;
    case 3:
        numBandsEmpty = 1;
        break;
    case 4:
        numBandsEmpty = 2;
        break;
    default:
        numBandsEmpty = -1;
        goto exit;
    }

exit:
    return numBandsEmpty;
}


/*! \brief MMW demo helper Function to configure phase shifter chirps for the
 *   DDMA processing chain. For the xth Tx antenna and the kth chirp, the
 *   phase shifter value is given by (k - 1) * (x - 1) / (numTxTotal + 1).
 */
static int32_t MmwDemo_configPhaseShifterChirps(void)
{
    int32_t errCode = 0;
    uint16_t chirpIdx, chirpStartIdx, chirpEndIdx, numTxAntAzim;
    uint16_t numTxTotalDivisor, numTxAntElev, txAntMask, txOrderIdx;
    uint16_t activeTxCnt, chirpPhaseMultiplier;
    int32_t numEmptySubBands;

    /* xth value of this array corresponds to the phase shift multiplier for the xth Tx antenna */
    uint16_t phaseShiftMultiplier[SYS_COMMON_NUM_TX_ANTENNAS];
    rlRfPhaseShiftCfg_t phaseShiftCfg;

    memset((void *)&phaseShiftCfg, 0, sizeof(phaseShiftCfg));

    txAntMask = gMmwMssMCB.cfg.openCfg.chCfg.txChannelEn;
    numTxAntAzim = mathUtils_countSetBits(txAntMask & MmwDemo_RFParserHwCfg.azimTxAntMask);
    numTxAntElev = mathUtils_countSetBits(txAntMask & MmwDemo_RFParserHwCfg.elevTxAntMask);

    numEmptySubBands = MmwDemo_getNumEmptySubBands(numTxAntAzim + numTxAntElev);
    numTxTotalDivisor = numTxAntAzim + numTxAntElev + numEmptySubBands;

    activeTxCnt = 0;
    /* Get the phase multiplier factor for each Tx antenna */
    /* Loop over Tx Antenna Phase order index */
    for (txOrderIdx = 0; txOrderIdx < SYS_COMMON_NUM_TX_ANTENNAS; txOrderIdx++)
    {
        /* Check if the Tx antenna corresponding to the xth index in the phase order is enabled */
        if (1 << gMmwMssMCB.ddmPhaseShiftOrder[txOrderIdx] & txAntMask)
        {
            /* Antenna is enabled, hence compute the phase shift value */
            phaseShiftMultiplier[gMmwMssMCB.ddmPhaseShiftOrder[txOrderIdx]] = activeTxCnt;
            activeTxCnt++;
        }
        else
        {
            /* Antenna is disabled */
            phaseShiftMultiplier[gMmwMssMCB.ddmPhaseShiftOrder[txOrderIdx]] = 0;
        }
    }

    /* Configure Phase Shifter Chirps */
    if (gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode == MMWave_DFEDataOutputMode_FRAME)
    {
        chirpStartIdx = gMmwMssMCB.cfg.ctrlCfg.u.frameCfg[0].frameCfg.chirpStartIdx;
        chirpEndIdx = gMmwMssMCB.cfg.ctrlCfg.u.frameCfg[0].frameCfg.chirpEndIdx;

        /* Phase multipliers have been computed; phase shift for xth chirp = (x-1) * phase multipler */
        for (chirpIdx = chirpStartIdx; chirpIdx <= chirpEndIdx; chirpIdx++)
        {

            chirpPhaseMultiplier = (chirpIdx - chirpStartIdx);

            /* Populate the chirp configuration: */
            phaseShiftCfg.chirpStartIdx = (chirpEndIdx + 1 - chirpPhaseMultiplier) % numTxTotalDivisor;
            phaseShiftCfg.chirpEndIdx = (chirpEndIdx + 1 - chirpPhaseMultiplier) % numTxTotalDivisor;

            /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
            phaseShiftCfg.tx0PhaseShift =
                ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[0]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;

            /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
            phaseShiftCfg.tx1PhaseShift =
                ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[1]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;

            /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
            phaseShiftCfg.tx2PhaseShift =
                ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[2]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;

            /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
            phaseShiftCfg.tx3PhaseShift =
                ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[3]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;

            /* Add the chirp to the profile */
            if (MMWave_addPhaseShiftChirp(gMmwMssMCB.ctrlHandle, &phaseShiftCfg, &errCode) == NULL)
            {
                /* Error: Unable to add the phase shifter chirp. Return the error code. */
                CLI_write("Error: Unable to add the phase shifter chirp.\r\n");
                return errCode;
            }
        }
    }

    else if (gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        uint8_t numOfSubFrames = gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg[0].frameCfg.frameSeq.numOfSubFrames;
        uint8_t subFrameIdx = 0;
        for (subFrameIdx = 0; subFrameIdx < numOfSubFrames; subFrameIdx++)
        {
            chirpStartIdx = gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg[0].frameCfg.frameSeq.subFrameCfg[subFrameIdx].chirpStartIdx;
            chirpEndIdx = gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg[0].frameCfg.frameSeq.subFrameCfg[subFrameIdx].numOfChirps + chirpStartIdx - 1;

            /* Phase multipliers have been computed; phase shift for xth chirp = (x-1) * phase multipler */
            for (chirpIdx = chirpStartIdx; chirpIdx <= chirpEndIdx; chirpIdx++)
            {

                // printf("Chirp %d\r\n", chirpIdx);
                chirpPhaseMultiplier = (chirpIdx - chirpStartIdx);

                /* Populate the chirp configuration: */
                phaseShiftCfg.chirpStartIdx = (chirpEndIdx + 1 - chirpPhaseMultiplier) % numTxTotalDivisor + chirpStartIdx;
                phaseShiftCfg.chirpEndIdx = (chirpEndIdx + 1 - chirpPhaseMultiplier) % numTxTotalDivisor + chirpStartIdx;

                /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
                phaseShiftCfg.tx0PhaseShift =
                    ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[0]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;

                /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
                phaseShiftCfg.tx1PhaseShift =
                    ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[1]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;

                /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
                phaseShiftCfg.tx2PhaseShift =
                    ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[2]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;

                /* 1 LSB of phaseShiftCfg.txPhaseShift = 360/2^6 = 5.625 degrees Valid range: 0 to 63 */
                phaseShiftCfg.tx3PhaseShift =
                    ((uint32_t)MATHUTILS_ROUND_FLOAT(((float)((chirpPhaseMultiplier * phaseShiftMultiplier[3]) % numTxTotalDivisor) / numTxTotalDivisor) * (1U << 6))) << 2;

                /* Add the chirp to the profile */
                if (MMWave_addPhaseShiftChirp(gMmwMssMCB.ctrlHandle, &phaseShiftCfg, &errCode) == NULL)
                {
                    /* Error: Unable to add the phase shifter chirp. Return the error code. */
                    CLI_write("Error: Unable to add the phase shifter chirp.\r\n");
                    return errCode;
                }
            }
        }
    }

    return errCode;
}


/*! \brief This function computes Modulation frequency divider
 *      (7 bit mantissa and 3 bit exponent) and Modulation depth
 *      (3 bit integer and 18 bit fraction) based on user provided inputs
 *      Modulation rate, Modulation depth in %
 */
static uint32_t MmwDemo_compSscFactCtrlVal(const uint32_t refClk,
                const uint16_t dpllM, MmwDemo_spreadSpectrumConfig *ptrdpllCfg)
{
    float modRateSel = 0.0f;
    float deltaMStep = 0.0f;
    uint32_t delatMStepInt = 0U;
    float deltaMStepFrac = 0.0f;
    uint32_t deltaMStepFracInt = 0U;
    uint32_t modFreqDivExponent = 0U;
    uint32_t modFreqDivMantissa = 0U;

    modRateSel = (refClk * 1000.0f) / (4.0f * ptrdpllCfg->modRate);

    modFreqDivExponent = (uint32_t)(floor(modRateSel
                                            / MAX_MOD_FREQ_DIVIDER_MANTISSA));

    modFreqDivMantissa = (uint32_t)(floor(modRateSel
                                            / pow(2.0f, modFreqDivExponent)));

    ptrdpllCfg->modRate = refClk * 1000
                    / (4 * modFreqDivMantissa * (pow(2, modFreqDivExponent)));

    if (modFreqDivExponent <= 3U)
    {
        deltaMStep = (ptrdpllCfg->modDepth * dpllM)
            / (100.0f * modFreqDivMantissa * pow(2.0f, modFreqDivExponent));
    }
    else
    {
        deltaMStep = (ptrdpllCfg->modDepth * dpllM)
                                    / (100.0f * modFreqDivMantissa * 8.0f);
    }

    delatMStepInt = (uint32_t)(deltaMStep + 0.5f);

    deltaMStepFrac = deltaMStep - delatMStepInt;

    deltaMStepFracInt = (uint32_t)ceilf(deltaMStepFrac * (1U << 18U));

    ptrdpllCfg->modDepth = 100 * ((deltaMStepFrac / (1 << 18)) *
                modFreqDivMantissa * (1U << modFreqDivExponent) / dpllM);

    return (deltaMStepFracInt + (delatMStepInt * (1U << 18U)) +
            (modFreqDivMantissa * (1U << 21U)) + (modFreqDivExponent *
            (1U << 28U)) + (ptrdpllCfg->downSpread * (1U << 31U)));
}


/*! \brief Calibration save/restore initialization */
static int32_t MmwDemo_calibInit(void)
{
    int32_t retVal = 0;
    rlVersion_t verArgs;

    /* Initialize verArgs */
    memset((void *)&verArgs, 0, sizeof(rlVersion_t));

    /* Calibration save/restore init */
    gMmwMssMCB.calibCfg.sizeOfCalibDataStorage = sizeof(MmwDemo_calibData);
    gMmwMssMCB.calibCfg.calibDataHdr.magic = MMWDEMO_CALIB_STORE_MAGIC;
    memcpy((void *)&gMmwMssMCB.calibCfg.calibDataHdr.linkVer, (void *)&verArgs.mmWaveLink, sizeof(rlSwVersionParam_t));
    memcpy((void *)&gMmwMssMCB.calibCfg.calibDataHdr.radarSSVer, (void *)&verArgs.rf, sizeof(rlFwVersionParam_t));

    /* Check if Calibration data is over the Reserved storage */
    if (gMmwMssMCB.calibCfg.sizeOfCalibDataStorage <= MMWDEMO_CALIB_FLASH_SIZE)
    {
        gMmwMssMCB.calibCfg.calibDataHdr.hdrLen = sizeof(MmwDemo_calibDataHeader);
        gMmwMssMCB.calibCfg.calibDataHdr.dataLen = sizeof(MmwDemo_calibData) - sizeof(MmwDemo_calibDataHeader);

        /* Resets calibration data */
        memset((void *)&gCalibDataStorage, 0, sizeof(MmwDemo_calibData));

         /* Initialize Flash */
        retVal = mmwDemo_flashInit();
    }
    else
    {
        CLI_write("Error: Calibration data size is bigger than reserved size\r\n");
        retVal = -1;
    }

    return retVal;
}


/*!  \brief This function retrieves the calibration data from front end and
 * saves it in flash. */
static int32_t MmwDemo_calibSave(MmwDemo_calibDataHeader *ptrCalibDataHdr, MmwDemo_calibData *ptrCalibrationData)
{
    uint32_t flashOffset;
    int32_t retVal = 0;

    /* Calculate the read size in bytes */
    flashOffset = gMmwMssMCB.calibCfg.flashOffset;

    /* Copy header  */
    memcpy((void *)&(ptrCalibrationData->calibDataHdr), ptrCalibDataHdr, sizeof(MmwDemo_calibDataHeader));

    /* Flash calibration data */
    retVal = mmwDemo_flashWrite(flashOffset, (uint8_t *)ptrCalibrationData, sizeof(MmwDemo_calibData));
    if (retVal < 0)
    {
        /* Flash Header failed */
        CLI_write("Error: MmwDemo failed flashing calibration data with error[%d].\r\n", retVal);
    }
    return (retVal);
}


/*!  \brief This function reads calibration data from flash and send it to
 * front end through MMWave_open() &*/
static int32_t MmwDemo_calibRestore(MmwDemo_calibData *ptrCalibData)
{
    MmwDemo_calibDataHeader *pDataHdr;
    int32_t retVal = 0;
    uint32_t flashOffset;

    pDataHdr = &(ptrCalibData->calibDataHdr);

    /* Calculate the read size in bytes */
    flashOffset = gMmwMssMCB.calibCfg.flashOffset;

    /* Read calibration data header */
    if (mmwDemo_flashRead(flashOffset, (uint8_t *)pDataHdr, sizeof(MmwDemo_calibData)) < 0)
    {
        /* Error: only one can be enable at at time */
        CLI_write("Error: MmwDemo failed when reading calibration data from flash.\r\n");
        return -1;
    }

    /* Validate data header */
    if ((pDataHdr->magic != MMWDEMO_CALIB_STORE_MAGIC) ||
        (pDataHdr->hdrLen != gMmwMssMCB.calibCfg.calibDataHdr.hdrLen) ||
        (pDataHdr->dataLen != gMmwMssMCB.calibCfg.calibDataHdr.dataLen))
    {
        /* Header validation failed */
        CLI_write("Error: MmwDemo calibration data header validation failed.\r\n");
        retVal = -1;
    }
    /* Matching mmwLink version:
         In this demo, we would like to save/restore with the matching mmwLink and RF FW version.
         However, this logic can be changed to use data saved from previous mmwLink and FW releases,
         as long as the data format of the calibration data matches.
     */
    else if (memcmp((void *)&pDataHdr->linkVer, (void *)&gMmwMssMCB.calibCfg.calibDataHdr.linkVer, sizeof(rlSwVersionParam_t)) != 0)
    {
        CLI_write("Error: MmwDemo failed mmwLink version validation when restoring calibration data.\r\n");
        retVal = -1;
    }
    else if (memcmp((void *)&pDataHdr->radarSSVer, (void *)&gMmwMssMCB.calibCfg.calibDataHdr.radarSSVer, sizeof(rlFwVersionParam_t)) != 0)
    {
        CLI_write("Error: MmwDemo failed RF FW version validation when restoring calibration data.\r\n");
        retVal = -1;
    }
    return (retVal);
}


static int32_t MmwDemo_configCQ(MmwDemo_SubFrameCfg *subFrameCfg,
                                uint8_t numChirpsPerChirpEvent,
                                uint8_t validProfileIdx)
{
    MmwDemo_AnaMonitorCfg *ptrAnaMonitorCfg;
    ADCBuf_CQConf cqConfig;
    rlRxSatMonConf_t *ptrSatMonCfg;
    rlSigImgMonConf_t *ptrSigImgMonCfg;
    int32_t retVal;
    uint16_t cqChirpSize;

    /* Get analog monitor configuration */
    ptrAnaMonitorCfg = &gMmwMssMCB.anaMonCfg;

    /* Config mmwaveLink to enable Saturation monitor - CQ2 */
    ptrSatMonCfg = &gMmwMssMCB.cqSatMonCfg[validProfileIdx];

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        if (ptrSatMonCfg->profileIndx != validProfileIdx)
        {
            CLI_write("Error: Saturation monitoring (globally) enabled but \
                        not configured for profile(%d)\r\n", validProfileIdx);
            MmwDemo_debugAssert(0);
        }

        retVal = mmwDemo_cfgRxSaturationMonitor(ptrSatMonCfg);
        if (retVal != 0)
        {
            CLI_write("Error: rlRfRxIfSatMonConfig returns error = %d for \
            profile(%d)\r\n", retVal, ptrSatMonCfg->profileIndx);
            goto exit;
        }
    }

    /* Config mmwaveLink to enable Saturation monitor - CQ1 */
    ptrSigImgMonCfg = &gMmwMssMCB.cqSigImgMonCfg[validProfileIdx];

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        if (ptrSigImgMonCfg->profileIndx != validProfileIdx)
        {
            CLI_write("Error: Sig/Image monitoring (globally) enabled but \
            not configured for profile(%d)\r\n", validProfileIdx);
            MmwDemo_debugAssert(0);
        }

        retVal = mmwDemo_cfgRxSigImgMonitor(ptrSigImgMonCfg);
        if (retVal != 0)
        {
            CLI_write("Error: rlRfRxSigImgMonConfig returns error = %d for \
            profile(%d)\r\n", retVal, ptrSigImgMonCfg->profileIndx);
            goto exit;
        }
    }

    retVal = mmwDemo_cfgAnalogMonitor(ptrAnaMonitorCfg);
    if (retVal != 0)
    {
        CLI_write("Error: rlRfAnaMonConfig returns error = %d\r\n", retVal);
        goto exit;
    }

    if (ptrAnaMonitorCfg->rxSatMonEn || ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* CQ driver config */
        memset((void *)&cqConfig, 0, sizeof(ADCBuf_CQConf));
        /* 16bit for mmw demo */
        cqConfig.cqDataWidth = 0;
        /* CQ1 starts from the beginning of the buffer */
        cqConfig.cq1AddrOffset = MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET;
        /* Address should be 16 bytes aligned */
        cqConfig.cq2AddrOffset = MMW_DEMO_CQ_RXSAT_ADDR_OFFSET;

        retVal = ADCBuf_control(gMmwMssMCB.adcBufHandle, \
                                ADCBufMMWave_CMD_CONF_CQ, (void *)&cqConfig);
        if (retVal < 0)
        {
            CLI_write("Error: MMWDemoDSS Unable to configure the CQ\r\n");
            MmwDemo_debugAssert(0);
        }
    }

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* This is for 16bit format in mmw demo, signal/image band data has 2
         * bytes/slice. For other format, please check DFP interface document
         */
        cqChirpSize = (ptrSigImgMonCfg->numSlices + 1) * sizeof(uint16_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, \
                                                MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->sigImgMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        /* This is for 16bit format in mmw demo, saturation data has one b
         * byte/slice. For other format, please check DFP interface document
         */
        cqChirpSize = (ptrSatMonCfg->numSlices + 1) * sizeof(uint8_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, \
                                                MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->satMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }

exit:
    return (retVal);
}


/*! \brief Registered event function to mmwave which is invoked when an event
 *      from the BSS is received.
 */
static int32_t MmwDemo_eventCallbackFxn(uint8_t devIndex, uint16_t msgId,
                            uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* Process the received message: */
    switch (msgId)
    {
    case RL_RF_ASYNC_EVENT_MSG:
    {
        /* Received Asychronous Message: */
        switch (asyncSB)
        {
        case RL_RF_AE_CPUFAULT_SB:
        {
            MmwDemo_debugAssert(0);
            break;
        }
        case RL_RF_AE_ESMFAULT_SB:
        {
            MmwDemo_debugAssert(0);
            break;
        }
        case RL_RF_AE_ANALOG_FAULT_SB:
        {
            MmwDemo_debugAssert(0);
            break;
        }
        case RL_RF_AE_INITCALIBSTATUS_SB:
        {
            rlRfInitComplete_t *ptrRFInitCompleteMessage;
            uint32_t calibrationStatus;

            /* Get the RF-Init completion message: */
            ptrRFInitCompleteMessage = (rlRfInitComplete_t *)payload;
            calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0x1FFFU;

            /* Display the calibration status: */
            CLI_write("Debug: Init Calibration Status = 0x%x\r\n", calibrationStatus);
            break;
        }
        case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
        {
            gMmwMssMCB.stats.frameTriggerReady++;
            break;
        }
        case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
        {
            gMmwMssMCB.stats.failedTimingReports++;
            break;
        }
        case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
        {
            gMmwMssMCB.stats.calibrationReports++;
            break;
        }
        case RL_RF_AE_FRAME_END_SB:
        {
            gMmwMssMCB.stats.sensorStopped++;
            DebugP_logInfo("App: BSS stop (frame end) received\r\n");

            MmwDemo_dataPathStop();
            break;
        }
        default:
        {
            CLI_write("Error: Asynchronous Event SB Id %d not handled\r\n", asyncSB);
            break;
        }
        }
        break;
    }
    /* Async Event from MMWL */
    case RL_MMWL_ASYNC_EVENT_MSG:
    {
        switch (asyncSB)
        {
        case RL_MMWL_AE_MISMATCH_REPORT:
        {
            /* link reports protocol error in the async report from BSS */
            MmwDemo_debugAssert(0);
            break;
        }
        case RL_MMWL_AE_INTERNALERR_REPORT:
        {
            /* link reports internal error during BSS communication */
            MmwDemo_debugAssert(0);
            break;
        }
        }
        break;
    }
    /* Async Event from MSS */
    case RL_DEV_ASYNC_EVENT_MSG:
    {
        switch (asyncSB)
        {
        case RL_DEV_AE_MSSPOWERUPDONE_SB:
        {
            CLI_write("Received RL_DEV_AE_MSSPOWERUPDONE_SB\r\n");
        }
        break;
        default:
        {
            CLI_write("Unhandled Async Event msgId: 0x%x, asyncSB:0x%x  \r\n\r\n", msgId, asyncSB);
            break;
        }
        }
        break;
    }
    default:
    {
        CLI_write("Error: Asynchronous message %d is NOT handled\r\n", msgId);
        break;
    }
    }
    return 0;
}

static void MmwDPC_chirpAvailISR(void* arg)
{
    uint32_t delay = 0;
    CSL_rss_ctrlRegs *ptr_rss_ctrl_regs = (CSL_rss_ctrlRegs*)CSL_RSS_CTRL_U_BASE;

    if(gMmwMssMCB.adcDataDithDelayCfg.isDitherEn == 1U)
    {
        /* configure the variable amount of delay */
        delay = rand() % gMmwMssMCB.adcDataDithDelayCfg.ditherVal;
    }

    if(gMmwMssMCB.adcDataDithDelayCfg.isDelayEn == 1U)
    {
        ptr_rss_ctrl_regs->ADCBUFCFG1_EXTD =  delay + gMmwMssMCB.adcDataDithDelayCfg.minDelay;
    }
}

static int32_t MmwDemo_registerChirpStartInterrupt(void)
{
    int32_t retVal = 0;
    int32_t status = SystemP_SUCCESS;
    HwiP_Params hwiPrms;

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = CSL_MSS_INTR_RSS_ADC_CAPTURE_COMPLETE;
    hwiPrms.callback = &MmwDPC_chirpAvailISR;
    hwiPrms.args = (void*)gMmwMssMCB.dpcHandle;
    status = HwiP_construct(&MMwDemo_ChirpAvailHwiObject, &hwiPrms);
    if (SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_MSS_INTR_RSS_ADC_CAPTURE_COMPLETE);
    }
    return retVal;
}

static int32_t MmwDemo_registerFrameStartInterrupt(void)
{
    int32_t retVal = 0;
    int32_t status = SystemP_SUCCESS;
    HwiP_Params hwiPrms;

    /* Register interrupt */
    HwiP_Params_init(&hwiPrms);
    hwiPrms.intNum = CSL_MSS_INTR_DFE_FRAME_START_TO_MSS;
    hwiPrms.callback = &MmwDPC_frameStartISR;
    hwiPrms.args = (void*)gMmwMssMCB.dpcHandle;
    hwiPrms.priority = 3U;
    status = HwiP_construct(&MMwDemo_FrameStartHwiObject, &hwiPrms);

    if (SystemP_SUCCESS != status)
    {
        retVal = SystemP_FAILURE;
    }
    else
    {
        HwiP_enableInt((uint32_t)CSL_MSS_INTR_DFE_FRAME_START_TO_MSS);
    }
    return retVal;
}

/*! \brief The function is used to trigger the Front end to stop
 *  generating chirps. */
static int32_t MmwDemo_mmWaveCtrlStop(void)
{
    int32_t errCode = 0;

    DebugP_logInfo("App: Issuing MMWave_stop\r\n");

    /* Stop the mmWave module: */
    if (MMWave_stop(gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        MMWave_ErrorLevel errorLevel;
        int16_t mmWaveErrorCode;
        int16_t subsysErrorCode;

        /* Error/Warning: Unable to stop the mmWave module */
        MMWave_decodeError(errCode, &errorLevel, &mmWaveErrorCode, \
                            &subsysErrorCode);
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error: Display the error message: */
            CLI_write("Error: mmWave Stop failed [Error code: %d  \
                        Subsystem: %d]\r\n", mmWaveErrorCode, subsysErrorCode);

            /* Not expected */
            MmwDemo_debugAssert(0);
        }
        else
        {
            /* Warning: This is treated as a successful stop. */
            CLI_write("mmWave Stop error ignored [Error code: %d \
                        Subsystem: %d]\r\n", mmWaveErrorCode, subsysErrorCode);
        }
    }
    return errCode;
}


static void MmwDemo_sensorStopEpilog(void)
{
    /* Note data path has completely stopped due to
     * end of frame, so we can do non-real time processing like prints on
     * console */
    CLI_write("Data Path Stopped (last frame processing done)\r\n");
}


/*! \brief The task is used to provide an execution context for the mmWave
 *      control task */
static void MmwDemo_mmWaveCtrlTask(void *args)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute(gMmwMssMCB.ctrlHandle, &errCode) < 0)
        {
            MmwDemo_debugAssert(0);
        }
    }
}


/*! \brief DPC Execution Task which calls the DPU for data processing. */
static void MmwDemo_DPCTask(void *args)
{
    int32_t errCode = 0;
    MmwDemo_initDPCParams dpcInitParams;

    /* Initialize the Datapath */
    dpcInitParams.hwaHandle = gHwaHandle[CONFIG_HWA0];
    dpcInitParams.edmaHandle = gEdmaHandle[CONFIG_EDMA0];
    dpcInitParams.L3RamCfg.addr = (void *)&gMmwL3Ram[0];
    dpcInitParams.L3RamCfg.size = sizeof(gMmwL3Ram);
    dpcInitParams.CoreLocalRamCfg.addr = (void *)&gMmwL2Ram[0];
    dpcInitParams.CoreLocalRamCfg.size = sizeof(gMmwL2Ram);
    gMmwMssMCB.dpcHandle = MmwDPC_init(&dpcInitParams, &errCode);
    if (errCode < 0)
    {
        CLI_write("Error: DPC init failed [Error code %d]\r\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }

    if (MmwDemo_registerChirpStartInterrupt() != 0)
    {
        CLI_write("Error: Failed to register chirp start interrupts\r\n");
        DebugP_assert(0);
    }

    if (MmwDemo_registerFrameStartInterrupt() != 0)
    {
        CLI_write("Error: Failed to register frame start interrupts\r\n");
        DebugP_assert(0);
    }

    while (1)
    {
        MmwDPC_execute(gMmwMssMCB.dpcHandle, &errCode);
        if (errCode < 0)
        {
            CLI_write("Error: DPC execution failed [Error code %d]\r\n", errCode);
            MmwDemo_debugAssert(0);
            return;
        }
    }
}

static uint32_t MmwDemo_getGenfPeriodicity(bool* enableTsnStack)
{
    MMWave_CtrlCfg       tempctrlCfg = {0};

    CLI_getMMWaveExtensionConfig (&tempctrlCfg);

    /* Populate flag to enable TSN stack
     * If CPTS triggering is enabled set below flag to true
     */
    *enableTsnStack = (tempctrlCfg.u.frameCfg[0].frameCfg.triggerSelect == 3)? true : false;

    /* Compute GenF signal periodicity in Hz based on CLI parameter */
    return (uint32_t)(1000U /(tempctrlCfg.u.frameCfg[0].frameCfg.framePeriodicity * 0.000005f));
}


/*! \brief System Initialization Task which initializes the various
 *      components in the system. */
static void MmwDemo_initTask(void *args)
{
    int32_t errCode;
    MMWave_InitCfg initCfg;
    int32_t index;
    MMWave_ErrorLevel errorLevel;
    int16_t mmWaveErrorCode;
    int16_t subsysErrorCode;

    Drivers_open();
    Board_driversOpen();

#if defined (LVDS_STREAM)
    /* Configure HSI Clock source - PLL_PER_CLK (1000MHz). */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL, 0x333);
#else
    /* Configure HSI Clock source - DPLL_CORE_HSDIV0_CLKOUT0 (Disabled HSDIV). */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_HSI_CLK_SRC_SEL, 0x000);
#endif

    /* Debug Message: */
    test_print("**********************************************\r\n");
    test_print("Debug: Launching the MMW Demo on MSS\r\n");
    test_print("**********************************************\r\n");

    /* Debug Message: */
    test_print("Debug: Launched the Initialization Task\r\n");

    /************************************************************************
     * Initialize the mmWave SDK components:
     ************************************************************************/
    gMmwMssMCB.edmaHandle = gEdmaHandle[CONFIG_EDMA1];

    /* Unmask MSS VMON ESM interrupt */
    CSL_REG32_WR((volatile uint32_t *)(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ANALOG_WU_STATUS_REG_GRP2_MASK), 0xFFF9FFFCU);

    /* Unmask APLL VCO LDO SHORT CIRCUIT ESM interrupt */
    CSL_REG32_WR((volatile uint32_t *)(CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_ANALOG_CLK_STATUS_REG_GRP1_MASK), 0xFFFFFFFCU);

#ifdef LVDS_STREAM
    /* Initialize LVDS streaming components */
    if ((errCode = MmwDemo_LVDSStreamInit()) < 0)
    {
        test_print("Error: MMWDemoDSS LVDS stream init failed with Error[%d]\r\n", errCode);
        return;
    }

    /* Configure Pad registers for LVDS. */
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_LVDS_PAD_CTRL0, 0x0);
    HW_WR_REG32(CSL_MSS_TOPRCM_U_BASE + CSL_MSS_TOPRCM_LVDS_PAD_CTRL1, 0x02000000);

    /*The delay below is needed only if the DCA1000EVM is being used to capture the data traces.
      This is needed because the DCA1000EVM FPGA needs the delay to lock to the
      bit clock before they can start capturing the data correctly. */
    ClockP_usleep(12 * 1000);
#endif

    /* initialize cq configs to invalid profile index to be able to detect
     * unconfigured state of these when monitors for them are enabled.
     */
    for (index = 0; index < RL_MAX_PROFILES_CNT; index++)
    {
        gMmwMssMCB.cqSatMonCfg[index].profileIndx = (RL_MAX_PROFILES_CNT + 1);
        gMmwMssMCB.cqSigImgMonCfg[index].profileIndx = (RL_MAX_PROFILES_CNT + 1);
    }

    /* Open the UART Instance */
    gMmwMssMCB.commandUartHandle = gUartHandle[CONFIG_UART0];
    if (gMmwMssMCB.commandUartHandle == NULL)
    {
        MmwDemo_debugAssert(0);
        return;
    }

    DebugP_logInfo("Command UART instance opened");

    /* Create binary semaphore to pend Main task, */
    SemaphoreP_constructBinary(&gMmwMssMCB.demoInitTaskCompleteSemHandle, 0);

    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Initialize the mmWave control init configuration */
    memset((void *)&initCfg, 0, sizeof(MMWave_InitCfg));

    /* Populate the init configuration: */
    initCfg.domain = MMWave_Domain_MSS;
    initCfg.eventFxn = MmwDemo_eventCallbackFxn;
    initCfg.linkCRCCfg.crcBaseAddr = (uint32_t)AddrTranslateP_getLocalAddr(CONFIG_CRC0_BASE_ADDR);
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel = CRC_CHANNEL_1;
    initCfg.cfgMode = MMWave_ConfigurationMode_FULL;

    /* Initialize and setup the mmWave Control module */
    gMmwMssMCB.ctrlHandle = MMWave_init(&initCfg, &errCode);
    if (gMmwMssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        MMWave_decodeError(errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);

        /* Error: Unable to initialize the mmWave control module */
        test_print("Error Level: %s mmWave: %d Subsys: %d\r\n",
                   (errorLevel == MMWave_ErrorLevel_ERROR) ? "Error" : "Warning",
                   mmWaveErrorCode, subsysErrorCode);
        MmwDemo_debugAssert(0);
        return;
    }
    test_print("Debug: mmWave Control Initialization was successful\r\n");

    /* Synchronization: This will synchronize the execution of the control
     * module between the domains. This is a prerequiste and always needs to
     * be invoked. */
    if (MMWave_sync(gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error: Unable to synchronize the mmWave control module */
        test_print("Error: mmWave Control Synchronization failed [Error code %d]\r\n", errCode);
        MmwDemo_debugAssert(0);
        return;
    }
    test_print("Debug: mmWave Control Synchronization was successful\r\n");

    /**************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *************************************************************************/
    gMmwMssMCB.taskHandles.mmwCtrlTask = xTaskCreateStatic(
                                    MmwDemo_mmWaveCtrlTask,
                                    "mmwdemo_ctrl_task",
                                    MMWDEMO_CTRL_TASK_STACK_SIZE,
                                    NULL,
                                    MMWDEMO_CTRL_TASK_PRIORITY,
                                    MmwDemoCtrlTskStack,
                                    &gMmwMssMCB.taskHandles.mmwCtrlTaskObj);

    configASSERT(gMmwMssMCB.taskHandles.mmwCtrlTask != NULL);

#ifdef POWER_MEAS
    /* Create binary Semaphore for triggering MSS Loading Task. */
    SemaphoreP_constructBinary(&gMmwMssMCB.powerMeas.frameStartSemaphore, 0);
    gMmwMssMCB.taskHandles.mssLoadingTaskHandle = xTaskCreateStatic( MmwPm_MssLoadingTask,
                        "MmwPm_MssLoadingTask",
                        MMWPM_MSSLOADING_TASK_STACK_SIZE,
                        NULL,
                        MMWPM_MSSLOADING_TASK_PRIORITY,
                        gMmwDemo_MssLoadingTaskStack,
                        &gMmwMssMCB.taskHandles.mssLoadingTaskObj);
    configASSERT(gMmwMssMCB.taskHandles.mssLoadingTaskHandle != NULL);
#endif

    /* Launch the DPC Task */
    gMmwMssMCB.taskHandles.mmwDPCTask = xTaskCreateStatic(MmwDemo_DPCTask,
                                        "mmwdemo_dpc_task",
                                        MMWDEMO_DPC_TASK_STACK_SIZE,
                                        NULL,
                                        MMWDEMO_DPC_TASK_PRIORITY,
                                        MmwDemoDPCTskStack,
                                        &gMmwMssMCB.taskHandles.mmwDPCTaskObj);

    configASSERT(gMmwMssMCB.taskHandles.mmwDPCTask != NULL);

    /* Calibration save/restore initialization */
    if (MmwDemo_calibInit() < 0)
    {
        test_print("Error: Calibration data initialization failed \r\n");
        MmwDemo_debugAssert(0);
    }

    /* Initialize the Profiler */
    CycleCounterP_reset();

    /* Initialize the CLI Module */
    MmwDemo_CLIInit(MMWDEMO_CLI_TASK_PRIORITY);

    /* Never return for this task. */
    SemaphoreP_pend(&gMmwMssMCB.demoInitTaskCompleteSemHandle, \
                    SystemP_WAIT_FOREVER);

    /* The following line should never be reached. */
    DebugP_assertNoLog(0);
}


/*! \brief mmw demo helper Function to do one time sensor initialization.
 *      User need to fill gMmwMssMCB.cfg.openCfg before calling this function
 */
volatile bool isEnetInitialized = false;
extern int32_t MmwDemo_openSensor(bool isFirstTimeOpen)
{
    int32_t errCode;
    MMWave_ErrorLevel errorLevel;
    int16_t mmWaveErrorCode;
    int16_t subsysErrorCode;
    int32_t retVal;
    MMWave_CalibrationData calibrationDataCfg;
    MMWave_CalibrationData *ptrCalibrationDataCfg;

    /*  Open mmWave module, this is only done once */
    if (isFirstTimeOpen == true)
    {
        /* Open mmWave module, this is only done once */
        /* Setup the calibration frequency:*/
        gMmwMssMCB.cfg.openCfg.freqLimitLow = 760U;
        gMmwMssMCB.cfg.openCfg.freqLimitHigh = 810U;

        /* start/stop async events */
        gMmwMssMCB.cfg.openCfg.disableFrameStartAsyncEvent = false;
        gMmwMssMCB.cfg.openCfg.disableFrameStopAsyncEvent = false;

        /* No custom calibration: */
        gMmwMssMCB.cfg.openCfg.useCustomCalibration = false;
        gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask = 0x0;

        /* calibration monitoring base time unit
         * setting it to one frame duration as the demo doesnt support any
         * monitoring related functionality
         */
        gMmwMssMCB.cfg.openCfg.calibMonTimeUnit = 1;

        if ((gMmwMssMCB.calibCfg.saveEnable != 0) &&
            (gMmwMssMCB.calibCfg.restoreEnable != 0))
        {
            /* Error: only one can be enabled at at time */
            CLI_write("Error: MmwDemo failed with both save and restore enabled.\r\n");
            return -1;
        }

        if (gMmwMssMCB.calibCfg.restoreEnable != 0)
        {
            if (MmwDemo_calibRestore(&gCalibDataStorage) < 0)
            {
                CLI_write("Error: MmwDemo failed restoring calibration data from flash.\r\n");
                return -1;
            }
            /*  Boot calibration during restore: Disable calibration for:
                 - Rx gain,
                 - Rx IQMM,
                 - Tx phase shifer,
                 - Tx Power

                 The above calibration data will be restored from flash. Since they are calibrated in a control
                 way to avoid interfaerence and spec violations.
                 In this demo, other bit fields(except the above) are enabled as indicated in customCalibrationEnableMask to perform boot time
                 calibration. The boot time calibration will overwrite the restored calibration data from flash.
                 However other bit fields can be disabled and calibration data can be restored from flash as well.

                 Note: In this demo, calibration masks are enabled for all bit fields when "saving" the data.
            */
            gMmwMssMCB.cfg.openCfg.useCustomCalibration = true;
            gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask = 0x1F0U;

            calibrationDataCfg.ptrCalibData = &gCalibDataStorage.calibData;
            calibrationDataCfg.ptrPhaseShiftCalibData = &gCalibDataStorage.phaseShiftCalibData;
            ptrCalibrationDataCfg = &calibrationDataCfg;
        }
        else
        {
            ptrCalibrationDataCfg = NULL;
        }

        /* Open the mmWave module: */
        if (MMWave_open(gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.openCfg, ptrCalibrationDataCfg, &errCode) < 0)
        {
            /* Error: decode and Report the error */
            MMWave_decodeError(errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
            CLI_write("Error: mmWave Open failed [Error code: %d Subsystem: %d]\r\n",
                       mmWaveErrorCode, subsysErrorCode);
            return -1;
        }

        /* Initialize CPSW interface */
#ifdef POWER_MEAS
        MmwEnet_init();
#else
        if(!isEnetInitialized)
        {
            bool enableTsnStack = false;
            uint32_t genfSigPeriod = 0U;
            genfSigPeriod = MmwDemo_getGenfPeriodicity(&enableTsnStack);
            EnetApp_mainTask(genfSigPeriod, enableTsnStack);
            isEnetInitialized = true;
        }
#endif
        /* Save calibration data in flash */
        if (gMmwMssMCB.calibCfg.saveEnable != 0)
        {
            retVal = rlRfCalibDataStore(RL_DEVICE_MAP_INTERNAL_BSS, &gCalibDataStorage.calibData);
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Calibration data restore failed */
                CLI_write("MSS demo failed rlRfCalibDataStore with Error[%d]\r\n", retVal);
                return -1;
            }

            /* update txIndex in all chunks to get data from all Tx.
            This should be done regardless of num TX channels enabled in MMWave_OpenCfg_t::chCfg or number of Tx
            application is interested in. Data for all existing Tx channels should be retrieved
            from RadarSS and in the order as shown below.
            RadarSS will return non-zero phase shift values for all the channels enabled via
            MMWave_OpenCfg_t::chCfg and zero phase shift values for channels disabled in MMWave_OpenCfg_t::chCfg */
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[0].txIndex = 0;
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[1].txIndex = 1;
            gCalibDataStorage.phaseShiftCalibData.PhShiftcalibChunk[2].txIndex = 2;

            /* Basic validation passed: Restore the phase shift calibration data */
            retVal = rlRfPhShiftCalibDataStore(RL_DEVICE_MAP_INTERNAL_BSS, &(gCalibDataStorage.phaseShiftCalibData));
            if (retVal != RL_RET_CODE_OK)
            {
                /* Error: Phase shift Calibration data restore failed */
                CLI_write("MSS demo failed rlRfPhShiftCalibDataStore with Error[%d]\r\n", retVal);
                return retVal;
            }

            /* Save data in flash */
            retVal = MmwDemo_calibSave(&gMmwMssMCB.calibCfg.calibDataHdr, &gCalibDataStorage);
            if (retVal < 0)
            {
                return retVal;
            }
        }

        /* Open the datapath modules that runs on MSS */
        MmwDemo_dataPathOpen();
    }
    return 0;
}


/*! \brief MMW demo helper Function to configure sensor. User need to fill
 *  gMmwMssMCB.cfg.ctrlCfg anD add profiles/chirp to mmWave before calling
 *  this function. */
extern int32_t MmwDemo_configSensor(void)
{
    int32_t errCode = 0;

    /* Configure the phase shift of chirps only if processing chain is DDM */
    if(gMmwMssMCB.objDetCommonCfg.procChain)
    {
        errCode = MmwDemo_configPhaseShifterChirps();
        if (errCode != 0)
        {
            goto exit;
        }
    }

    /* Configure the mmWave module: */
    if (MMWave_config(gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel errorLevel;
        int16_t mmWaveErrorCode;
        int16_t subsysErrorCode;

        /* Error: Report the error */
        MMWave_decodeError(errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        CLI_write("Error: mmWave Config failed [Error code: %d Subsystem: %d]\r\n",
                   mmWaveErrorCode, subsysErrorCode);
        goto exit;
    }
    else
    {
        /* Configure the Datapath */
        errCode = MmwDemo_dataPathConfig();
        if (errCode < 0)
        {
            CLI_write("Error: DPC Config failed [Error code %d]\r\n", errCode);
            MmwDemo_debugAssert(0);
            goto exit;
        }
    }
exit:
    return errCode;
}


/*! @brief MMW demo helper Function to start sensor.*/
extern int32_t MmwDemo_startSensor(void)
{
    int32_t errCode;
    MMWave_CalibrationCfg calibrationCfg;

    /* Reset Network Packet Count.
     * Write '1' followed by '0' to MSS_CTRL:NW_PACKET_COUNT_RESET register
     */
    CSL_REG32_WR((CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_NW_PACKET_COUNT_RESET), 1U);

    ClockP_usleep(10);

    CSL_REG32_WR((CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_NW_PACKET_COUNT_RESET), 0U);

    /* Start the Datapath - ready to handle the front-end data now */
    MmwDPC_start(gMmwMssMCB.dpcHandle, &errCode);
    if (errCode < 0)
    {
        CLI_write("Error: DPC Start failed [Error code %d]\r\n", errCode);
        MmwDemo_debugAssert(0);
        return errCode;
    }

#ifdef LVDS_STREAM
    /* Configure HW LVDS stream for the first sub-frame that will start upon
     * start of frame */
    if (gMmwMssMCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
    {
        MmwDemo_configLVDSHwData(0);
    }
#endif

    /*************************************************************************
     * RF :: now start the RF and the real time ticking
     ************************************************************************/
    /* Initialize the calibration configuration: */
    memset((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));
    /* Populate the calibration configuration: */
    calibrationCfg.dfeDataOutputMode = gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration = false;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity = false;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = false;
    calibrationCfg.u.chirpCalibrationCfg.reportEn = 1;

    DebugP_logInfo("App: MMWave_start Issued\r\n");

    CLI_write("Starting Sensor (issuing MMWave_start)\r\n");

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start(gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel errorLevel;
        int16_t mmWaveErrorCode;
        int16_t subsysErrorCode;

        /* Error/Warning: Unable to start the mmWave module */
        MMWave_decodeError(errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        CLI_write("Error: mmWave Start failed [mmWave Error: %d Subsys: %d]\r\n", mmWaveErrorCode, subsysErrorCode);
        /* datapath has already been moved to start state; so either we initiate a cleanup of start sequence or
           assert here and re-start from the beginning. For now, choosing the latter path */
        MmwDemo_debugAssert(0);
        return -1;
    }
    gMmwMssMCB.sensorStartCount++;
    return 0;
}


/*! \brief Stops the RF and datapath for the sensor. Blocks until both
 * operation are completed. Prints epilog at the end. */
extern void MmwDemo_stopSensor(void)
{
    /* Stop sensor RF, data path will be stopped after RF stop is completed */
    MmwDemo_mmWaveCtrlStop();

#ifdef LVDS_STREAM
    int32_t errCode;
    /* Delete any active streaming session */
    if (gMmwMssMCB.lvdsStream.hwSessionHandle != NULL)
    {
        /* Evaluate need to deactivate h/w session:
         * One sub-frame case:
         *   if h/w only enabled, deactivation never happened, hence need to deactivate
         *   if h/w and s/w both enabled, then s/w would leave h/w activated when it is done
         *   so need to deactivate
         *   (only s/w enabled cannot be the case here because we are checking for non-null h/w session)
         * Multi sub-frame case:
         *   Given stop, we must have re-configured the next sub-frame by now which is next of the
         *   last sub-frame i.e we must have re-configured sub-frame 0. So if sub-frame 0 had
         *   h/w enabled, then it is left in active state and need to deactivate. For all
         *   other cases, h/w was already deactivated when done.
         */
        if ((gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames == 1) ||
            ((gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames > 1) &&
             (gMmwMssMCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)))
        {
            if (CBUFF_deactivateSession(gMmwMssMCB.lvdsStream.hwSessionHandle, &errCode) < 0)
            {
                CLI_write("CBUFF_deactivateSession failed with errorCode = %d\r\n", errCode);
                MmwDemo_debugAssert(0);
            }
        }
        MmwDemo_LVDSStreamDeleteHwSession();
    }
#endif

    /* Print epilog */
    MmwDemo_sensorStopEpilog();

    gMmwMssMCB.sensorStopCount++;

    /* print for user */
    CLI_write("Sensor has been stopped: startCount: %d stopCount %d\r\n",
               gMmwMssMCB.sensorStartCount, gMmwMssMCB.sensorStopCount);
}


extern void MmwDemo_CfgUpdate(void *srcPtr, uint32_t offset,
                                             uint32_t size, int8_t subFrameNum)
{
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if (subFrameNum == MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        uint8_t indx;
        for (indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
        {
            memcpy((void *)((uint32_t)&gMmwMssMCB.subFrameCfg[indx] + offset),\
                        srcPtr, size);
        }
    }
    else
    {
        /* Apply configuration to specific subframe (or to position zero for
         * the legacy case where there is no advanced frame config) */
        memcpy((void *)((uint32_t)&gMmwMssMCB.subFrameCfg[subFrameNum] + \
                    offset), srcPtr, size);
    }
}


/**
 * \brief Performs Spread Spectrum Configuration (SSC)
 * SSC reduces the Electromagnetic Interference by spreading it out
 * across frequencies instead of concentrating at a single frequency
 */
extern void MmwDemo_configSSC(void)
{
    uint16_t dpllM = 0U;
    uint16_t dpllN = 0U;
    uint32_t finp = 50U; /* XTAL */
    uint32_t refClk = 0U;

    CSL_mss_toprcmRegs *ptrMssTopRcmRegs =
                            (CSL_mss_toprcmRegs *)CSL_MSS_TOPRCM_U_BASE;

    if (gMmwMssMCB.coreAdpllSscCfg.isEnable)
    {
        dpllM = CSL_FEXT(ptrMssTopRcmRegs->PLL_CORE_MN2DIV,
                         MSS_TOPRCM_PLL_CORE_MN2DIV_PLL_CORE_MN2DIV_M);

        dpllN = CSL_FEXT(ptrMssTopRcmRegs->PLL_CORE_M2NDIV,
                         MSS_TOPRCM_PLL_CORE_M2NDIV_PLL_CORE_M2NDIV_N);

        /** refClk - Pre-divided reference clock input for the ADPLL
         * CLKINP/(N+1) where N is Pre divider and CLKINP is OSC Clock
         */
        refClk = finp / (dpllN + 1);

        ptrMssTopRcmRegs->PLL_CORE_FRACCTRL =
            MmwDemo_compSscFactCtrlVal(refClk, dpllM, &gMmwMssMCB.coreAdpllSscCfg);

        CSL_FINS(ptrMssTopRcmRegs->PLL_CORE_CLKCTRL, \
                    MSS_TOPRCM_PLL_CORE_CLKCTRL_PLL_CORE_CLKCTRL_ENSSC, 1U);
    }

    if (gMmwMssMCB.perAdpllSscCfg.isEnable)
    {
        dpllM = CSL_FEXT(ptrMssTopRcmRegs->PLL_PER_MN2DIV,
                         MSS_TOPRCM_PLL_PER_MN2DIV_PLL_PER_MN2DIV_M);

        dpllN = CSL_FEXT(ptrMssTopRcmRegs->PLL_PER_M2NDIV,
                         MSS_TOPRCM_PLL_PER_M2NDIV_PLL_PER_M2NDIV_N);

        refClk = finp / (dpllN + 1);

        ptrMssTopRcmRegs->PLL_PER_FRACCTRL = MmwDemo_compSscFactCtrlVal(
                                    refClk, dpllM, &gMmwMssMCB.perAdpllSscCfg);
        CSL_FINS(ptrMssTopRcmRegs->PLL_PER_CLKCTRL, \
                        MSS_TOPRCM_PLL_PER_CLKCTRL_PLL_PER_CLKCTRL_ENSSC, 1U);
    }

    return;
}

/**
 *  @b Description
 *  @n
 *      The function is a callback notifier for ESM Group#1 interrupt#123
 *      (ANA_WU_AND_CLK_STATUS_ERR). This
 *
 *  @retval
 *      None
 */
void MmwDemo_anaWuClkStsErrCallBack(void)
{
    CSL_mss_toprcmRegs *ptrMssTopRcmReg = CSL_MSS_TOP_RCM_getBaseAddress();
    volatile uint32_t regVal = 0U;;

    /* Check the cause for the error.*/
    regVal = HW_RD_REG32(ptrMssTopRcmReg->ANA_REG_CLK_STATUS_REG);

    /* Is it APLL VCO LDO SHORT CIRCUIT INDICATOR? */
    if(HW_GET_FIELD(regVal, CSL_MSS_TOPRCM_ANA_REG_CLK_STATUS_REG_APLL_VCO_LDO_SC_OUT))
    {
        /* APLL LDO Output Short Circuit Detected */
        CLI_write ("APLL LDO SC ESM interrupt triggered.\n");
        DebugP_assert(0);
    }

    return;
}


extern void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression)
    {
        CLI_write("Exception: %s, line %d.\r\n", file, line);
    }
}


/* MSS VMON callback function */
extern void MmwDemo_mssVMONCb(void* arg)
{
    CSL_mss_toprcmRegs *ptrMssTopRcmReg = CSL_MSS_TOP_RCM_getBaseAddress();

    if(CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_UVDET_LAT))
    {
        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN, 0);
        CLI_write("VDD1.2 UV Detected!\n");
        DebugP_assert(0);
    }
    else if (CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_CORE_OVDET_LAT))
    {
        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN, 0);
        CLI_write("VDD1.2 OV Detected!\n");
        DebugP_assert(0);
    }
    else if(CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA_OSC_UVDET_LAT))
    {
        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN, 0);
        CLI_write("VDDA_OSC 1.8V UV Detected!\n");
        DebugP_assert(0);
    }
    else if(CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDS_3P3V_UVDET_LAT))
    {
        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN, 0);
        CLI_write("VDDS33 3.3V UV Detected!\n");
        DebugP_assert(0);
    }

    if(CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA18BB_UV_DET_LAT))
    {
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_UV_VMON_EN, 0);
        CLI_write("VDD 18 BB UV Detected!\n");
        DebugP_assert(0);
    }
    if (CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA10RF1_UVDET_LAT))
    {
        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDD_OV_VMON_EN, 0);
        CLI_write("VDDA RF1 UV Detected!\n");
        DebugP_assert(0);
    }
    if(CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_VDDA10RF2_UVDET_LAT))
    {
        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDA_OSC_UV_VMON_EN, 0);
        CLI_write("VDDA RF2 UV Detected!\n");
        DebugP_assert(0);
    }
    if(CSL_FEXT(ptrMssTopRcmReg->ANA_REG_WU_STATUS_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_STATUS_REG_LOWV_APLLVCO18_UVDET_LAT))
    {
        /* Disable VMON. */
        CSL_FINS(ptrMssTopRcmReg->ANA_REG_WU_CTRL_REG_LOWV, MSS_TOPRCM_ANA_REG_WU_CTRL_REG_LOWV_WU_VDDS_3P3V_UV_VMON_EN, 0);
        CLI_write("APLL VCO 18 UV Detected!\n");
        DebugP_assert(0);
    }

    return;
}

/*! \brief Entry point into the Millimeter Wave Demo*/
int32_t main(void)
{
    /* init SOC specific modules */
    System_init();
    Board_init();

    gMmwMssMCB.taskHandles.initTask = xTaskCreateStatic(
                                        MmwDemo_initTask,
                                        "mmwdemo_init_task",
                                        MMWDEMO_INIT_TASK_STACK_SIZE,
                                        NULL,
                                        MMWDEMO_INIT_TASK_PRI,
                                        MmwDemoMainTskStack,
                                        &gMmwMssMCB.taskHandles.initTaskObj);
    configASSERT(gMmwMssMCB.taskHandles.initTask != NULL);

    /* Start the scheduler to start the tasks executing. */
    vTaskStartScheduler();

    /* The following line should never be reached because vTaskStartScheduler()
    will only return if there was not enough FreeRTOS heap memory available to
    create the Idle and (if configured) Timer tasks.  Heap management, and
    techniques for trapping heap exhaustion, are described in the book text. */
    DebugP_assertNoLog(0);
}
