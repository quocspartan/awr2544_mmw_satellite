/*
 *  Copyright (c) Texas Instruments Incorporated 2023-24
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

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */
#include <tsn_combase/combase.h>
#include "dataflow.h"
#include "enetapp_cpsw.h"
#include <ti/demo/awr2544/mmw/mmw_common.h>

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

#define MMWENET_ETHERTYPE_IPV4          (0x0800)

#define MMWENT_IPV4_HDR_VER_IHL         ((0x4 << 4) | 0x5)
#define MMWENET_IPV4_HDR_TOS            (0x00)
#define MMWENET_IPV4_HDR_IPID           (0x28)
#define MMWENET_IPV4_HDR_FLAGFRAFOFFSET (0x0000)
#define MMWENET_IPV4_HDR_TTL            (0xFF)
#define MMWENET_IPV4_HDR_UDP            (0x11)

#define MMWENET_PKT_HDR_SIZE            (ENET_UTILS_ALIGN((sizeof(EthAppUDPHeader) + \
                                        sizeof(EthAppIPv4Header) + \
                                        sizeof(EthFrameHeader)), 128))

#define MMWENET_IP_ADDR_LEN             (4U)

#define MMWENET_ETH_PAD_SIZE            (0U)
#define MMWENET_SIZEOF_ETH_HDR          (14 + MMWENET_ETH_PAD_SIZE)

/* Macros to get struct ip_hdr fields: */
#define MMWENET_IPH_HL(hdr)             ((hdr)->verIHL & 0x0f)
#define MMWENET_IPH_LEN(hdr)            ((hdr)->totalPktLen)
#define MMWENET_IPH_PROTO(hdr)          ((hdr)->protocol)

#define MMWENET_FOLD_U32T(u) ((uint32_t)(((u) >> 16) + ((u) & 0x0000ffffUL)))
#define MMWENET_PP_HTONS(x) ((uint16_t)((((x) & (uint16_t)0x00ffU) << 8) | (((x) & (uint16_t)0xff00U) >> 8)))
#define MMWENET_SWAP_BYTES_IN_WORD(w) (((w) & 0xff) << 8) | (((w) & 0xff00) >> 8)

#define MMWENET_IP4_ADDR_GET_U32(src_ipaddr) ((src_ipaddr)->addr)

/* Network Packet Buffer CRC Macros */
#define MMWENET_NW_PKT_BUF_32_BITCRC     (1U)
#define MMWENET_NW_PKT_BUF_16_BITCRC     (0U)

/* ========================================================================== */
/*                         Structures and Enums                               */
/* ========================================================================== */

struct ip4_addr
{
    uint32_t addr;
};

typedef struct ip4_addr ip4_addr_t;

typedef ip4_addr_t ip_addr_t;
typedef struct
{
    uint8_t  verIHL;
    uint8_t  tos;
    uint16_t totalPktLen;
    uint16_t ipId;
    uint16_t flagFragOffset;
    uint8_t  ttl;
    uint8_t  protocol;
    uint16_t hdrChksum;
    uint8_t  srcIP[MMWENET_IP_ADDR_LEN];
    uint8_t  dstIP[MMWENET_IP_ADDR_LEN];
} __attribute__ ((packed)) EthAppIPv4Header;

typedef struct
{
    uint16_t srcPort;
    uint16_t dstPort;
    uint16_t length;
    uint16_t csum;
    uint16_t dummy;
} __attribute__ ((packed)) EthAppUDPHeader;

typedef struct MmwEnet_AddrInfo_s
{
    uint8_t dstMac[ENET_MAC_ADDR_LEN];
    uint8_t srcIP[MMWENET_IP_ADDR_LEN];
    uint8_t dstIP[MMWENET_IP_ADDR_LEN];
    uint16_t srcPortUDP;
    uint16_t dstPortUDP;
} MmwEnet_AddrInfo;

/* ========================================================================== */
/*                        Static Function Declaration                         */
/* ========================================================================== */
static void EnetApp_initPkt
(
    EnetDma_Pkt *pPktInfo,
    uint32_t payloadSize,
    uint32_t address
);
static void EnetApp_initEthFrameHdr(uint8_t *bufPtr, uint32_t *len);
static void EnetApp_initIPv4Hdr
(
    uint8_t *bufPtr,
    uint32_t *len,
    uint32_t payloadSize
);
static void EnetApp_initUDPHdr
(
    uint8_t  *bufPtr,
    uint32_t *len,
    uint32_t payloadSize
);

static uint16_t EnetApp_ipChkSum_pseudo
(
    uint8_t proto, uint16_t proto_len,
    const ip4_addr_t *src, const ip4_addr_t *dest
);

static uint16_t EnetApp_cksumpSeudoBase
(
    uint8_t     proto,
    uint16_t    proto_len,
    uint32_t    acc
);

static void EnetApp_configEdma
(
    EDMA_Handle          edmaHandle,
    MmwDemo_staticDPCCfg *staticCfg,
    uint32_t             packetSize
);

static uint32_t EnetApp_getChecsumInfo(void* payload);
static uint32_t EnetApp_retrieveFreeTxPkts(void);
static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh);
static uint16_t EnetApp_computeIpChkSum(const void *dataptr, uint32_t bufflen);
static inline void EnetApp_getSrcIp(uint8_t *pIpPkt, ip_addr_t* pIpAddr);
static inline void EnetApp_getDstIp(uint8_t *pIpPkt, ip_addr_t* pIpAddr);
static uint16_t EnetApp_computeIpChkSum(const void *dataptr, uint32_t bufflen);

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */
/* Variables to configure MSS_CTRL:HW_REG2 */
uint32_t gToggleValPing __attribute__ ((aligned(CacheP_CACHELINE_ALIGNMENT))) = 0x1011;
uint32_t gToggleValPong __attribute__ ((aligned(CacheP_CACHELINE_ALIGNMENT))) = 0x0111;
const uint32_t cpswTrigVal __attribute__ ((aligned(CacheP_CACHELINE_ALIGNMENT))) = 0x1U << ENET_DMA_TX_CH0;

const uint32_t delayValSrc __attribute__ ((aligned(CacheP_CACHELINE_ALIGNMENT))) = 0xFU;
uint32_t delayValDst __attribute__ ((aligned(CacheP_CACHELINE_ALIGNMENT)));

static uint8_t gMmwEnetPktHeader[MMWENET_PKT_HDR_SIZE] __attribute__ ((aligned(CacheP_CACHELINE_ALIGNMENT)));
static uint8_t gEnetAppTaskStackRx[ENETAPP_TASK_STACK_SZ] __attribute__ ((aligned(32)));

const MmwEnet_AddrInfo gMmwEnetAddrInfo =
{
    .dstMac = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF},
    .srcIP  = {0xC0,0xA8,0x00,0xC3}, /* 192.168.0.195 */
    .dstIP  = {0xC0,0xA8,0x00,0x88}, /* 192.168.0.136 */
    .srcPortUDP = 8080,
    .dstPortUDP = 8080,
};

extern EnetApp_Cfg gEnetAppCfg;

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */

static inline uint8_t* EnetApp_getIpPktStart(uint8_t* pEthpkt)
{
    const uint32_t ipPacketStartOffset = (MMWENET_SIZEOF_ETH_HDR);

    return &pEthpkt[ipPacketStartOffset];
}

static inline void EnetApp_getSrcIp(uint8_t *pIpPkt, ip_addr_t* pIpAddr)
{
    EthAppIPv4Header* pIpHdr = (EthAppIPv4Header *)pIpPkt;
    memcpy(pIpAddr, &pIpHdr->srcIP, sizeof(pIpHdr->srcIP));
}

static inline void EnetApp_getDstIp(uint8_t *pIpPkt, ip_addr_t* pIpAddr)
{
    EthAppIPv4Header* pIpHdr = (EthAppIPv4Header *)pIpPkt;
    memcpy(pIpAddr, &pIpHdr->dstIP, sizeof(pIpHdr->dstIP));
}

/* Rx Isr for non-gPTP traffic */
static void EnetApp_rxIsrFxn(void *appData)
{
    SemaphoreP_post(&gEnetAppCfg.rxSemObj);
}

static int32_t EnetApp_openDma()
{
    EnetApp_GetDmaHandleInArgs     txInArgs;
    EnetApp_GetTxDmaHandleOutArgs  txChInfo;
    int32_t status = ENET_SOK;

    /* Open the TX channel */
    txInArgs.cbArg   = NULL;
    txInArgs.notifyCb = NULL;

    EnetApp_getTxDmaHandle((ENET_DMA_TX_CH0),
                           &txInArgs,
                           &txChInfo);

    gEnetAppCfg.txChNum = txChInfo.txChNum;
    gEnetAppCfg.hTxCh   = txChInfo.hTxCh;

    if (gEnetAppCfg.hTxCh == NULL)
    {
#if FIX_RM
        /* Free the channel number if open Tx channel failed */
        EnetAppUtils_freeTxCh(gEnetAppCfg.hEnet,
                              gEnetAppCfg.coreKey,
                              gEnetAppCfg.coreId,
                              gEnetAppCfg.txChNum);
#endif
        EnetAppUtils_print("EnetApp_openDma() failed to open TX channel\r\n");
        status = ENET_EFAIL;
        EnetAppUtils_assert(gEnetAppCfg.hTxCh != NULL);
    }

    /* Open the RX flow for Regular frames */
    if (status == ENET_SOK)
    {
        EnetApp_GetDmaHandleInArgs     rxInArgs;
        EnetApp_GetRxDmaHandleOutArgs  rxChInfo;

        rxInArgs.notifyCb = EnetApp_rxIsrFxn;
        rxInArgs.cbArg   = NULL;

        EnetApp_getRxDmaHandle(ENET_DMA_RX_CH0,
                               &rxInArgs,
                               &rxChInfo);
        gEnetAppCfg.rxFlowIdx = rxChInfo.rxChNum;
        gEnetAppCfg.hRxCh  = rxChInfo.hRxCh;
        EnetAppUtils_assert(rxChInfo.numValidMacAddress == 1);
        EnetUtils_copyMacAddr(gEnetAppCfg.macAddr, rxChInfo.macAddr[rxChInfo.numValidMacAddress - 1]);
        EnetAppUtils_print("MAC port addr: ");
        EnetAppUtils_printMacAddr(gEnetAppCfg.macAddr);

        if (gEnetAppCfg.hRxCh == NULL)
        {
            EnetAppUtils_print("EnetApp_openRxCh() failed to open RX flow\r\n");
            status = ENET_EFAIL;
            EnetAppUtils_assert(gEnetAppCfg.hRxCh != NULL);
        }
    }

    /* Submit all ready RX buffers to DMA */
    if (status == ENET_SOK)
    {
        EnetApp_initRxReadyPktQ(gEnetAppCfg.hRxCh);
    }

     return status;
}

static uint16_t EnetApp_cksumpSeudoBase
(
    uint8_t     proto,
    uint16_t    proto_len,
    uint32_t    acc
)
{
    acc += (uint32_t)MMWENET_PP_HTONS((uint16_t)proto);
    acc += (uint32_t)MMWENET_PP_HTONS(proto_len);

    /* Fold 32-bit sum to 16 bits
        calling this twice is probably faster than if statements... */
    acc = MMWENET_FOLD_U32T(acc);
    acc = MMWENET_FOLD_U32T(acc);

    return (uint16_t)~(acc & 0xffffUL);
}

/**
 *  @b Description
 *  @n
 *
 * Calculates the IPv4 pseudo Internet checksum used by TCP and UDP for a pbuf chain.
 * IP addresses are expected to be in network byte order.
 *
 * @param[in] proto       ip protocol (used for checksum of pseudo header)
 * @param[in] proto_len   length of the ip data part (used for checksum of pseudo header)
 * @param[in] src         source ip address (used for checksum of pseudo header)
 * @param[in] dst         destination ip address (used for checksum of pseudo header)
 * @retval checksum to be saved directly in the protocol header
 */
static uint16_t EnetApp_ipChkSum_pseudo
(
    uint8_t proto, uint16_t proto_len,
    const ip4_addr_t *src, const ip4_addr_t *dest
)
{
  uint32_t acc;
  uint32_t addr;

  addr = MMWENET_IP4_ADDR_GET_U32(src);
  acc = (addr & 0xffffUL);
  acc = (uint32_t)(acc + ((addr >> 16) & 0xffffUL));
  addr = MMWENET_IP4_ADDR_GET_U32(dest);
  acc = (uint32_t)(acc + (addr & 0xffffUL));
  acc = (uint32_t)(acc + ((addr >> 16) & 0xffffUL));
  /* fold down to 16 bits */
  acc = MMWENET_FOLD_U32T(acc);
  acc = MMWENET_FOLD_U32T(acc);

  return EnetApp_cksumpSeudoBase(proto, proto_len, acc);
}

static void EnetApp_closeDma()
{
    EnetDma_PktQ fqPktInfoQ;
    EnetDma_PktQ cqPktInfoQ;

    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Close Regular RX channel */
    EnetApp_closeRxDma(ENET_DMA_RX_CH0,
                       gEnetAppCfg.hEnet,
                       gEnetAppCfg.coreKey,
                       gEnetAppCfg.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    /* Close TX channel */
    EnetQueue_initQ(&fqPktInfoQ);
    EnetQueue_initQ(&cqPktInfoQ);

    /* Retrieve any pending TX packets from driver */
    EnetApp_retrieveFreeTxPkts();

    EnetApp_closeTxDma(ENET_DMA_TX_CH0,
                       gEnetAppCfg.hEnet,
                       gEnetAppCfg.coreKey,
                       gEnetAppCfg.coreId,
                       &fqPktInfoQ,
                       &cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&fqPktInfoQ);
    EnetAppUtils_freePktInfoQ(&cqPktInfoQ);

    EnetAppUtils_freePktInfoQ(&gEnetAppCfg.txFreePktInfoQ);
}

static void EnetApp_initRxReadyPktQ(EnetDma_RxChHandle hRxCh)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *pPktInfo;
    uint32_t i;
    int32_t status;
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    EnetQueue_initQ(&rxFreeQ);

    for (i = 0U; i < ENET_DMA_RX_CH0_NUM_PKTS; i++)
    {
        pPktInfo = EnetMem_allocEthPkt(&gEnetAppCfg,
                                       ENETDMA_CACHELINE_ALIGNMENT,
                                       ENET_ARRAYSIZE(scatterSegments),
                                       scatterSegments);
        EnetAppUtils_assert(pPktInfo != NULL);

        ENET_UTILS_SET_PKT_APP_STATE(&pPktInfo->pktState, ENET_PKTSTATE_APP_WITH_FREEQ);

        EnetQueue_enq(&rxFreeQ, &pPktInfo->node);
    }

    /* Retrieve any packets which are ready */
    EnetQueue_initQ(&rxReadyQ);
    status = EnetDma_retrieveRxPktQ(hRxCh, &rxReadyQ);
    EnetAppUtils_assert(status == ENET_SOK);

    /* There should not be any packet with DMA during init */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxReadyQ) == 0U);

    EnetAppUtils_validatePacketState(&rxFreeQ,
                                     ENET_PKTSTATE_APP_WITH_FREEQ,
                                     ENET_PKTSTATE_APP_WITH_DRIVER);

    EnetDma_submitRxPktQ(hRxCh, &rxFreeQ);

    /* Assert here, as during init, the number of DMA descriptors should be equal to
     * the number of free Ethernet buffers available with app */
    EnetAppUtils_assert(EnetQueue_getQCount(&rxFreeQ) == 0U);
}

static uint32_t EnetApp_retrieveFreeTxPkts()
{
    EnetDma_PktQ txFreeQ;
    EnetDma_Pkt *pktInfo;
    uint32_t txFreeQCnt = 0U;
    int32_t status;

    EnetQueue_initQ(&txFreeQ);

    /* Retrieve any packets that may be free now */
    status = EnetDma_retrieveTxPktQ(gEnetAppCfg.hTxCh, &txFreeQ);
    if (status == ENET_SOK)
    {
        txFreeQCnt = EnetQueue_getQCount(&txFreeQ);

        pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        while (NULL != pktInfo)
        {
            EnetDma_checkPktState(&pktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            EnetQueue_enq(&gEnetAppCfg.txFreePktInfoQ, &pktInfo->node);
            pktInfo = (EnetDma_Pkt *)EnetQueue_deq(&txFreeQ);
        }
    }
    else
    {
        EnetAppUtils_print("retrieveFreeTxPkts() failed to retrieve pkts: %d\r\n", status);
    }

    return txFreeQCnt;
}

/* Rx Echo task for non-gPTP traffic */
static void EnetApp_rxTask(void *args)
{
    EnetDma_PktQ rxReadyQ;
    EnetDma_PktQ rxFreeQ;
    EnetDma_Pkt *rxPktInfo;
    // EnetDma_Pkt *txPktInfo;
    EthFrame *rxFrame;
    // EthFrame *txFrame;
    uint32_t totalRxCnt = 0U;
    int32_t status = ENET_SOK;

    EnetAppUtils_print("%s: default RX flow started\r\n",
                       gEnetAppCfg.name);

    while ((ENET_SOK == status))
    {
        /* Wait for packet reception */
        SemaphoreP_pend(&gEnetAppCfg.rxSemObj, SystemP_WAIT_FOREVER);

        /* All peripherals have single hardware RX channel, so we only need to retrieve
         * packets from a single flow.*/
        EnetQueue_initQ(&rxReadyQ);
        EnetQueue_initQ(&rxFreeQ);

        /* Get the packets received so far */
        status = EnetDma_retrieveRxPktQ(gEnetAppCfg.hRxCh, &rxReadyQ);
        if (status != ENET_SOK)
        {
            /* Should we bail out here? */
            EnetAppUtils_print("Failed to retrieve RX pkt queue: %d\r\n", status);
            continue;
        }
#if DEBUG
        EnetAppUtils_print("%s: Received %u packets\r\n", gEnetAppCfg.name, EnetQueue_getQCount(&rxReadyQ));
#endif
        totalRxCnt += EnetQueue_getQCount(&rxReadyQ);

        /* Consume the received packets and send them back */
        rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        while (rxPktInfo != NULL)
        {
            rxFrame = (EthFrame *)SOC_phyToVirt((uint64_t)rxPktInfo->sgList.list[0].bufPtr);
            EnetDma_checkPktState(&rxPktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_DRIVER,
                                    ENET_PKTSTATE_APP_WITH_READYQ);

            EnetDma_checkPktState(&rxPktInfo->pktState,
                                    ENET_PKTSTATE_MODULE_APP,
                                    ENET_PKTSTATE_APP_WITH_READYQ,
                                    ENET_PKTSTATE_APP_WITH_FREEQ);

            /* Release the received packet */
            EnetQueue_enq(&rxFreeQ, &rxPktInfo->node);
            rxPktInfo = (EnetDma_Pkt *)EnetQueue_deq(&rxReadyQ);
        }

        EnetAppUtils_validatePacketState(&rxFreeQ,
                                         ENET_PKTSTATE_APP_WITH_FREEQ,
                                         ENET_PKTSTATE_APP_WITH_DRIVER);

        /* Submit now processed buffers */
        EnetDma_submitRxPktQ(gEnetAppCfg.hRxCh, &rxFreeQ);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("%s: Failed to submit RX pkt queue: %d\r\n", gEnetAppCfg.name, status);
        }
    }

    EnetAppUtils_print("%s: Received %u packets\r\n", gEnetAppCfg.name, totalRxCnt);

    TaskP_exit();
}

static uint32_t EnetApp_getChecsumInfo(void* payload)
{
    uint32_t chkSumInfo     = 0;
    struct eth_hdr *pEthPkt = (struct eth_hdr *) payload;
    EthAppIPv4Header  *pIpPkt  = (EthAppIPv4Header *) EnetApp_getIpPktStart((uint8_t*)pEthPkt);

    const uint32_t ipPktHdrLen     = (MMWENET_IPH_HL(pIpPkt) << 2)  /* multiply by 4 */;
    const uint32_t ipPktPayloadLen = MMWENET_PP_HTONS(MMWENET_IPH_LEN(pIpPkt)) - ipPktHdrLen;
    const uint32_t protocolType    = MMWENET_IPH_PROTO(pIpPkt);
    ip_addr_t srcIp;
    ip_addr_t dstIp;

    uint8_t *pIpPayload   = (uint8_t*)pIpPkt + ipPktHdrLen;

    EnetApp_getSrcIp((uint8_t *)pIpPkt, &srcIp);
    EnetApp_getDstIp((uint8_t *)pIpPkt, &dstIp);

    switch (protocolType)
    {
        case MMWENET_IPV4_HDR_UDP:
        {
            uint8_t csumCoverageStartByte = 0;
            uint8_t csumResultByte = 0;
            uint16_t pseudoIpHdrChkSum = 0;

            if (protocolType == MMWENET_IPV4_HDR_UDP)
            {
                EthAppUDPHeader* pUdpHdr = (EthAppUDPHeader*)pIpPayload;

                if (pUdpHdr->csum == 0U)
                {
                    /* checksum is valid and not computed by stack */

                    csumCoverageStartByte = (uint8_t*)pUdpHdr - (uint8_t*)pEthPkt + 1; /* CPSW cksum info indexing starts from 1 */
                    csumResultByte = (uint8_t*)(&(pUdpHdr->csum)) - (uint8_t*)pEthPkt + 1;
                    pseudoIpHdrChkSum = ~(EnetApp_ipChkSum_pseudo(MMWENET_IPV4_HDR_UDP, ipPktPayloadLen, &srcIp, &dstIp));
                    pUdpHdr->csum = pseudoIpHdrChkSum;
                    ENETDMA_TXCSUMINFO_SET_CHKSUM_BYTECNT(chkSumInfo, ipPktPayloadLen);
                    ENETDMA_TXCSUMINFO_SET_CHKSUM_STARTBYTE(chkSumInfo, csumCoverageStartByte);
                    ENETDMA_TXCSUMINFO_SET_CHKSUM_RESBYTE(chkSumInfo, csumResultByte);
                }
            }
            else
            {
                chkSumInfo = 0;
            }
        }
        break;
        default:
        {
            chkSumInfo = 0;
        }
    }

    return chkSumInfo;
}

static uint16_t EnetApp_computeIpChkSum(const void *dataptr, uint32_t bufflen)
{
    const uint8_t *ptrBuf = (const uint8_t *)dataptr;
    const uint16_t *ps;
    uint16_t t = 0;
    uint32_t chkSum = 0;
    uint32_t isBufAligned = ((uintptr_t)ptrBuf & 1);

    /* align to uint16_t */
    if (isBufAligned && bufflen > 0)
    {
        ((uint8_t *)&t)[1] = *ptrBuf++;
        bufflen--;
    }

    /* Add data bytes for provided buffer length */
    ps = (const uint16_t *)(const void *)ptrBuf;
    while (bufflen > 1)
    {
        chkSum += *ps++;
        bufflen -= 2;
    }

    /* check if left-over bytes are available */
    if (bufflen > 0)
    {
        ((uint8_t *)&t)[0] = *(const uint8_t *)ps;
    }

    /* Add carry bytes */
    chkSum += t;

    /* Fold 32-bit sum to 16 bits */
    chkSum = MMWENET_FOLD_U32T(chkSum);
    chkSum = MMWENET_FOLD_U32T(chkSum);

    /* Swap if alignment was isBufAligned */
    if (isBufAligned)
    {
        chkSum = MMWENET_SWAP_BYTES_IN_WORD(chkSum);
    }

    return (uint16_t)chkSum;
}

static void EnetApp_initUDPHdr
(
    uint8_t  *bufPtr,
    uint32_t *len,
    uint32_t payloadSize
)
{
    EthAppUDPHeader *udpHdr;

    udpHdr = (EthAppUDPHeader *)bufPtr;
    udpHdr->srcPort = Enet_htons(gMmwEnetAddrInfo.srcPortUDP);
    udpHdr->dstPort = Enet_htons(gMmwEnetAddrInfo.dstPortUDP);
    udpHdr->length = Enet_htons((sizeof(EthAppUDPHeader) + payloadSize));
    udpHdr->csum = 0U;
    /* 2 bytes of dummy required to ensure NW header is mutiple of 8 bytes*/
    udpHdr->dummy = 0U;

    *len += sizeof(EthAppUDPHeader);
}

static void EnetApp_initIPv4Hdr(uint8_t *bufPtr, uint32_t *len, uint32_t payloadSize)
{
    EthAppIPv4Header *ipv4Hdr;

    ipv4Hdr = (EthAppIPv4Header *)bufPtr;
    ipv4Hdr->verIHL = MMWENT_IPV4_HDR_VER_IHL;
    ipv4Hdr->tos    = MMWENET_IPV4_HDR_TOS;
    ipv4Hdr->totalPktLen = Enet_htons((payloadSize + sizeof(EthAppIPv4Header) + sizeof(EthAppUDPHeader)));
    ipv4Hdr->ipId = Enet_htons(MMWENET_IPV4_HDR_IPID);
    ipv4Hdr->flagFragOffset = Enet_htons(MMWENET_IPV4_HDR_FLAGFRAFOFFSET);
    ipv4Hdr->ttl = MMWENET_IPV4_HDR_TTL;
    ipv4Hdr->protocol = MMWENET_IPV4_HDR_UDP;
    ipv4Hdr->hdrChksum = 0x00;
    memcpy(&ipv4Hdr->srcIP,gMmwEnetAddrInfo.srcIP,sizeof(ipv4Hdr->srcIP));
    memcpy(&ipv4Hdr->dstIP,gMmwEnetAddrInfo.dstIP,sizeof(ipv4Hdr->dstIP));
    *len += sizeof(EthAppIPv4Header);

    ipv4Hdr->hdrChksum = ~(EnetApp_computeIpChkSum(bufPtr, sizeof(EthAppIPv4Header)));
}

static void EnetApp_initEthFrameHdr(uint8_t *bufPtr, uint32_t *len)
{
    EthFrame *frame;

    frame = (EthFrame *)bufPtr;
    memcpy(frame->hdr.dstMac, gMmwEnetAddrInfo.dstMac, ENET_MAC_ADDR_LEN);
    *len += ENET_MAC_ADDR_LEN;
    memcpy(frame->hdr.srcMac, &gEnetAppCfg.macAddr[0U], ENET_MAC_ADDR_LEN);
    *len += ENET_MAC_ADDR_LEN;
    frame->hdr.etherType = Enet_htons(MMWENET_ETHERTYPE_IPV4);
    *len += sizeof(frame->hdr.etherType);
}

static void EnetApp_initPkt
(
    EnetDma_Pkt *pPktInfo,
    uint32_t payloadSize,
    uint32_t address
)
{
    uint32_t len;

    len = 0;
    EnetApp_initEthFrameHdr(&gMmwEnetPktHeader[len], &len);
    EnetApp_initIPv4Hdr(&gMmwEnetPktHeader[len], &len, payloadSize);
    EnetApp_initUDPHdr(&gMmwEnetPktHeader[len], &len, payloadSize);
    pPktInfo->chkSumInfo = 0U;
    pPktInfo->sgList.list[0].segmentFilledLen = len;
    pPktInfo->appPriv    = &gEnetAppCfg;
    pPktInfo->sgList.numScatterSegments = 2;

    /* CPSW_READ_NW_BUFFER */
    pPktInfo->sgList.list[1].bufPtr = (uint8_t *)address;
    pPktInfo->sgList.list[1].segmentFilledLen = payloadSize;
    pPktInfo->sgList.list[0].bufPtr = &gMmwEnetPktHeader[0];
    pPktInfo->sgList.list[0].disableCacheOps = true;
    CacheP_wbInv(pPktInfo->sgList.list[0].bufPtr, len, CacheP_TYPE_ALLD);
    pPktInfo->sgList.list[1].disableCacheOps = true;
    CacheP_wbInv(pPktInfo->sgList.list[1].bufPtr, (payloadSize), CacheP_TYPE_ALLD);
}

static void EnetApp_nwPktBufCfg(uint32_t payloadSize, uint8_t nwPktCrcSel)
{
    CSL_mss_ctrlRegs* ptrMssCtrlRegs =  (CSL_mss_ctrlRegs*)CSL_MSS_CTRL_U_BASE;

    /* HW_REG2: Config CRC32 / CRC16 for NW packet buffer */
    CSL_FINS(ptrMssCtrlRegs->HW_REG2, MSS_CTRL_HW_REG2_HW_REG2_CRC_MODE, nwPktCrcSel);

    /* HW_REG2: Enable CRC NW packet buffer */
    CSL_FINS(ptrMssCtrlRegs->HW_REG2, MSS_CTRL_HW_REG2_HW_REG2_CRC_ENABLE, 1U);

    /* CPSW_CRC_PING_ADDR: Configure Ping mem buffer size */
    CSL_FINS(ptrMssCtrlRegs->CPSW_CRC_PING_ADDR,
             MSS_CTRL_CPSW_CRC_PING_ADDR_CPSW_CRC_PING_ADDR_UNDEFINED,
             payloadSize);

    /* CPSW_CRC_PONG_ADDR: Configure Pong mem buffer size */
    CSL_FINS(ptrMssCtrlRegs->CPSW_CRC_PONG_ADDR,
             MSS_CTRL_CPSW_CRC_PONG_ADDR_CPSW_CRC_PONG_ADDR_UNDEFINED,
             payloadSize);

    return;
}

/**
 *  @b Description
 *  @n
 *     Function configures EDMA transfer from L3 to CPSW.
 *     Three link channel for L3 to DSS PP and 2 link channel
 *     from MSS PP to CPSW
 *
 *  @param[in]  edmaHandle   EDMA handle
 *  @param[in]  staticCfg    Pointer to DPC static configuration
 *  @param[in]  packetSize   size of the payload
 *  @retval   None
 */
static void EnetApp_configEdma
(
    EDMA_Handle          edmaHandle,
    MmwDemo_staticDPCCfg *staticCfg,
    uint32_t             packetSize
)
{
    int32_t retVal = 0;
    uint32_t baseAddr;
    DPEDMA_3LinkChanCfg dataL3;
    DPEDMA_3LinkChanCfg toggle;
    DPEDMA_ChanCfg delayCh;
    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg syncABCfg, syncABCfg1, syncABCfg2;
    DPEDMA_syncABCfg syncABCfg3, syncABCfg4, syncABCfg5;
    DPEDMA_syncABCfg syncABCfg6;

    baseAddr = EDMA_getBaseAddr(edmaHandle);
    DebugP_assert(baseAddr != 0);

    MmwDPC_configAssistEDMAChannelThreeLinks(
        edmaHandle,
        MMW_RES_ENET_EDMAL3OUT_CH,
        MMW_RES_ENET_EDMAL3OUT_SHADOW1,
        MMW_RES_ENET_EDMAL3OUT_SHADOW2,
        MMW_RES_ENET_EDMAL3OUT_SHADOW3,
        MMW_RES_ENET_EDMAL3OUT_EVENT_QUE,
        &dataL3
    );

    MmwDPC_configAssistEDMAChannelThreeLinks(
        edmaHandle,
        MMW_RES_ENET_TOGGLE_CH,
        MMW_RES_ENET_TOGGLE_SHADOW1,
        MMW_RES_ENET_TOGGLE_SHADOW2,
        MMW_RES_ENET_TOGGLE_SHADOW3,
        MMW_RES_ENET_TOGGLE_EVENT_QUE,
        &toggle
    );

    if(gMmwMssMCB.objDetCommonCfg.ethPerPktDly != 0U)
    {
        MmwDPC_configAssistEDMAChannel(
            edmaHandle,
            MMW_RES_ENET_DELAY_CH,
            MMW_RES_ENET_DELAY_SHADOW,
            MMW_RES_ENET_DELAY_EVENT_QUE,
            &delayCh
        );
    }

    /* 3 link channel config to transfer radar Cube from L3 to DSS_PP buffer */
    syncABCfg.aCount = packetSize;
    syncABCfg.bCount = 1U;
    syncABCfg.cCount = staticCfg->numChirpsEachIter[0]*staticCfg->numPayloads;
    syncABCfg.srcBIdx = 0U;
    syncABCfg.srcCIdx = packetSize;
    syncABCfg.dstBIdx = 0U;
    syncABCfg.dstCIdx = 0U;
    syncABCfg.srcAddress = (uint32_t) CSL_DSS_L3_U_BASE; /* L3 Memory */
    syncABCfg.destAddress= (uint32_t) CSL_CPSW_PP_RAM_WR_U_BASE; /* DSS NW Packet buffer */

    syncABCfg1 = syncABCfg;
    syncABCfg1.cCount = staticCfg->numChirpsEachIter[1]*staticCfg->numPayloads;

    syncABCfg2 = syncABCfg;
    syncABCfg2.cCount = staticCfg->numChirpsEachIter[2]*staticCfg->numPayloads;

    if(syncABCfg1.cCount == 0U)
    {
        syncABCfg.cCount += 2;
    }
    else if(syncABCfg2.cCount == 0U)
    {
        syncABCfg1.cCount += 2;
    }
    else
    {
        syncABCfg2.cCount += 2;
    }

    /* Chain L3 to DPP and toggle channels */
    chainingCfg.chainingChannel = MMW_RES_ENET_TOGGLE_CH;
    chainingCfg.isFinalChainingEnabled = true;
    chainingCfg.isIntermediateChainingEnabled = true;

    retVal = DPEDMA_configSyncAB_ThreeLinks(edmaHandle,
            &dataL3,
            &chainingCfg,
            &syncABCfg,
            &syncABCfg1,
            &syncABCfg2,
            true,    /* isEventTriggered */
            false,   /* isIntermediateTransferCompletionEnabled */
            false,   /* isTransferCompletionEnabled */
            NULL,
            NULL,
            NULL);
    DebugP_assert(retVal==0);

    if(syncABCfg1.cCount == 0U)
    {
        /* No L3 Reuse case, only single iteration of L3. */
        /* Link main channel to the shadow channel and shadow to itself */
        EDMA_linkChannel(baseAddr, dataL3.paramId, dataL3.ShadowPramId[2]);
        EDMA_linkChannel(baseAddr, dataL3.ShadowPramId[2], dataL3.ShadowPramId[2]);
    }
    else if(syncABCfg2.cCount == 0U)
    {
        /* L3 Reuse two iterations */
        /* Link EDMA link1 to the shadow channel and shadow to link 1 */
        EDMA_linkChannel(baseAddr, dataL3.ShadowPramId[0], dataL3.ShadowPramId[2]);
        EDMA_linkChannel(baseAddr, dataL3.ShadowPramId[2], dataL3.ShadowPramId[0]);
    }

    /* Toggle and cpsw channel trigger */
    syncABCfg3.aCount = 4U;
    syncABCfg3.bCount = 1U;
    syncABCfg3.cCount = 1U;
    syncABCfg3.srcBIdx = 0U;
    syncABCfg3.srcCIdx = 0U;
    syncABCfg3.dstBIdx = 0U;
    syncABCfg3.dstCIdx = 0U;
    syncABCfg3.srcAddress = (uint32_t) &gToggleValPing;
    syncABCfg3.destAddress= (uint32_t) (CSL_MSS_CTRL_U_BASE+CSL_MSS_CTRL_HW_REG2);

    syncABCfg4 = syncABCfg3;
    syncABCfg4.cCount = staticCfg->numChirpsPerFrame*staticCfg->numPayloads;

    if(gMmwMssMCB.objDetCommonCfg.ethPerPktDly)
    {
        // amount of delay to be configured
        syncABCfg4.bCount = gMmwMssMCB.objDetCommonCfg.ethPerPktDly * 30U;
        syncABCfg4.srcAddress = (uint32_t) &delayValSrc;
        syncABCfg4.destAddress= (uint32_t) &delayValDst;

        syncABCfg6 = syncABCfg4;
        syncABCfg6.bCount = 1U;
        syncABCfg6.srcAddress = (uint32_t) &cpswTrigVal;
        syncABCfg6.destAddress= (uint32_t) (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_CPSW_HW_TRIG_VAL); //CPSW

        /* Chain delay to DPP and toggle channels */
        chainingCfg.chainingChannel = MMW_RES_ENET_DELAY_CH;
        chainingCfg.isFinalChainingEnabled = true;
        chainingCfg.isIntermediateChainingEnabled = true;
    }
    else
    {
        syncABCfg4.bCount = 1U;
        syncABCfg4.srcAddress = (uint32_t) &cpswTrigVal;
        syncABCfg4.destAddress= (uint32_t) (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_CPSW_HW_TRIG_VAL); //CPSW

        memset((void*)&chainingCfg, 0U, sizeof(chainingCfg));
    }

    /* Dummy Channel to stop the cpsw trigger */
    syncABCfg5 = syncABCfg4;
    syncABCfg5.bCount = 1U;
    syncABCfg5.cCount = 0U;

    retVal = DPEDMA_configSyncAB_ThreeLinks(edmaHandle,
            &toggle,
            &chainingCfg,
            &syncABCfg3,
            &syncABCfg4,
            &syncABCfg5,
            false,    /* isEventTriggered */
            false,    /* isIntermediateTransferCompletionEnabled */
            false,    /* isTransferCompletionEnabled */
            NULL,
            NULL,
            NULL);
    DebugP_assert(retVal==0);

    if(gMmwMssMCB.objDetCommonCfg.ethPerPktDly)
    {
        /* Link1 and Link3 should not be chained */
        uint32_t optVal;
        EDMACCPaRAMEntry *currPaRAM1;

        /* Disable Chaining for link1 */
        currPaRAM1 = (EDMACCPaRAMEntry *)(baseAddr + EDMA_TPCC_OPT(toggle.paramId));
        optVal = HW_RD_REG32((uint32_t) &currPaRAM1->opt);
        optVal &= ~(EDMA_TPCC_OPT_ITCCHEN_MASK | EDMA_TPCC_OPT_TCCHEN_MASK);
        HW_WR_REG32((uint32_t) &currPaRAM1->opt, optVal);

        /* Disable Chaining for link3 */
        currPaRAM1 = (EDMACCPaRAMEntry *)(baseAddr + EDMA_TPCC_OPT(toggle.ShadowPramId[1]));
        optVal = HW_RD_REG32((uint32_t) &currPaRAM1->opt);
        optVal &= ~(EDMA_TPCC_OPT_ITCCHEN_MASK | EDMA_TPCC_OPT_TCCHEN_MASK);
        HW_WR_REG32((uint32_t) &currPaRAM1->opt, optVal);

        /* Disable Chaining for shadow */
        currPaRAM1 = (EDMACCPaRAMEntry *)(baseAddr + EDMA_TPCC_OPT(toggle.ShadowPramId[2]));
        optVal = HW_RD_REG32((uint32_t) &currPaRAM1->opt);
        optVal &= ~(EDMA_TPCC_OPT_ITCCHEN_MASK | EDMA_TPCC_OPT_TCCHEN_MASK);
        HW_WR_REG32((uint32_t) &currPaRAM1->opt, optVal);


        syncABCfg6 = syncABCfg4;
        syncABCfg6.bCount = 1U;
        syncABCfg6.srcAddress = (uint32_t) &cpswTrigVal;
        syncABCfg6.destAddress= (uint32_t) (CSL_MSS_CTRL_U_BASE + CSL_MSS_CTRL_CPSW_HW_TRIG_VAL); //CPSW
        retVal = DPEDMA_configSyncAB(edmaHandle,
                &delayCh,
                NULL,
                &syncABCfg6,
                false,    /* isEventTriggered */
                false,    /* isIntermediateTransferCompletionEnabled */
                false,    /* isTransferCompletionEnabled */
                NULL,
                NULL,
                NULL);
        DebugP_assert(retVal==0);
    }
}

static void EnetApp_initTxFreePktQ(uint32_t payloadSize)
{
    uint32_t scatterSegments[] = { ENET_MEM_LARGE_POOL_PKT_SIZE };

    gEnetAppCfg.pTxPktInfo = EnetMem_allocEthPkt(&gEnetAppCfg,
                                    ENETDMA_CACHELINE_ALIGNMENT,
                                    ENET_ARRAYSIZE(scatterSegments),
                                    scatterSegments);
    EnetAppUtils_assert(gEnetAppCfg.pTxPktInfo != NULL);

    /* Initialize all queues */
    EnetQueue_initQ(&gEnetAppCfg.txFreePktInfoQ);

    ENET_UTILS_SET_PKT_APP_STATE(&gEnetAppCfg.pTxPktInfo->pktState,
                                 ENET_PKTSTATE_APP_WITH_FREEQ);

    EnetApp_initPkt(gEnetAppCfg.pTxPktInfo, payloadSize,
                    (uint32_t)CSL_CPSW_PP_RAM_RD_U_BASE);

    /* Compute pSuedo Checksum */
    gEnetAppCfg.pTxPktInfo->chkSumInfo = EnetApp_getChecsumInfo(gEnetAppCfg.pTxPktInfo->sgList.list[0].bufPtr);

    CacheP_wbInv(&gMmwEnetPktHeader[0], sizeof(gMmwEnetPktHeader), CacheP_TYPE_ALLD);

    EnetQueue_enq(&gEnetAppCfg.txFreePktInfoQ, &gEnetAppCfg.pTxPktInfo->node);
}

static int32_t EnetApp_showAlivePhys(void)
{
    Enet_IoctlPrms prms;
    bool alive = false;
    int32_t status;

    for (uint32_t phyAdd = 0U; phyAdd < ENET_MDIO_PHY_CNT_MAX; phyAdd++)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &phyAdd, &alive);

        ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId,
                    ENET_MDIO_IOCTL_IS_ALIVE, &prms, status);
        if (status == ENET_SOK)
        {
            if (alive == true)
            {
                EnetAppUtils_print("PHY %u is alive\r\n", phyAdd);
            }
        }
        else
        {
            EnetAppUtils_print("Failed to get PHY %u alive status: %d\r\n", phyAdd, status);
        }
    }

    return status;
}

static int32_t EnetApp_waitForLinkUp(void)
{
    Enet_IoctlPrms prms;
    bool linked    = false;
    int32_t status = ENET_SOK;

    ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetAppCfg.macPorts[0], &linked);

    while (!linked)
    {
        ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId,
                   ENET_PER_IOCTL_IS_PORT_LINK_UP, &prms, status);
        if (status != ENET_SOK)
        {
            EnetAppUtils_print("Failed to get port %u's link status: %d\r\n",
                            ENET_MACPORT_ID(gEnetAppCfg.macPorts[0]), status);
            linked = false;
            break;
        }

        if (!linked)
        {
            /* wait for 50 ms and poll again*/
            ClockP_usleep(50000);
        }
    }

    return status;
}

void EnetApp_createRxTask()
{
    TaskP_Params taskParams;
    int32_t status;

    status = EnetApp_openDma();
    DebugP_assert(ENET_SOK == status);

    status = SemaphoreP_constructBinary(&gEnetAppCfg.rxSemObj, 0);
    DebugP_assert(SystemP_SUCCESS == status);
    TaskP_Params_init(&taskParams);
    taskParams.priority       = 4U;
    taskParams.stack          = gEnetAppTaskStackRx;
    taskParams.stackSize      = sizeof(gEnetAppTaskStackRx);
    taskParams.args           = (void*)&gEnetAppCfg;
    taskParams.name           = "Rx Task";
    taskParams.taskMain       = &EnetApp_rxTask;

    status = TaskP_construct(&gEnetAppCfg.rxTaskObj, &taskParams);
    DebugP_assert(SystemP_SUCCESS == status);
}

void EnetApp_destroyRxTask()
{
    SemaphoreP_destruct(&gEnetAppCfg.rxSemObj);
    TaskP_destruct(&gEnetAppCfg.rxTaskObj);
    EnetApp_closeDma();
}

void EnetApp_config(MmwDemo_staticDPCCfg *ptrStaticCfg, uint8_t nwPktCrcSel)
{
    int32_t status = ENET_SOK;
    uint32_t packetSize = (ptrStaticCfg->radarCubeDataSize /
                          (ptrStaticCfg->numChirpsPerFrame *
                           ptrStaticCfg->numPayloads));

    /* Packet size should be a multiple of 8 bytes
     * else the payload CRC will not be accurate.
     */
    DebugP_assert((packetSize % 8U) == 0U);

    if (MMWENET_NW_PKT_BUF_32_BITCRC == nwPktCrcSel)
    {
        /* Configure Network Packet buffers
        * Enable 32-bit CRC
        */
        EnetApp_nwPktBufCfg(packetSize, MMWENET_NW_PKT_BUF_32_BITCRC);

        gToggleValPing = 0x1011U;
        gToggleValPong = 0x0111U;

        /* Initialize CPSW Tx Buffer descriptors */
        /* +4 bytes for 32-bit CRC placeholder */
        EnetApp_initTxFreePktQ(packetSize + 4U);
    }
    else if(MMWENET_NW_PKT_BUF_16_BITCRC == nwPktCrcSel)
    {
        /* Configure Network Packet buffers
        * Enable 16-bit CRC
        */
        EnetApp_nwPktBufCfg(packetSize, MMWENET_NW_PKT_BUF_16_BITCRC);

        gToggleValPing = 0x1010U;
        gToggleValPong = 0x0110U;

        /* Initialize CPSW Tx Buffer descriptors */
        /* +2 bytes for 16-bit CRC placeholder */
        EnetApp_initTxFreePktQ(packetSize + 2U);
    }

    /* Invalidate cache to reflect gToggleValPing and gToggleValPong */
    CacheP_wbInvAll(CacheP_TYPE_ALL);

    /* Configure EDMA channels to transfer Compressed 1D-FFT data from
     * L3 to Network packet buffer with corresponding link channels.
     */
    EnetApp_configEdma(gEdmaHandle[CONFIG_EDMA0], ptrStaticCfg, packetSize);

    /* Show alive PHYs */
    if (status == ENET_SOK)
    {
        status = EnetApp_showAlivePhys();
    }

    /* Wait for link up */
    if (status == ENET_SOK)
    {
        status = EnetApp_waitForLinkUp();
    }

    /* Configure FHost CPDMA Hardware Controlled Packet Transmission */
    EnetSoc_setupHwCtrlTx(ENET_DMA_TX_CH0);

    EnetDma_submitCircularTxPktQ(gEnetAppCfg.hTxCh,
                                 &gEnetAppCfg.txFreePktInfoQ);
}

void EnetApp_showCpswStats(void)
{
    Enet_IoctlPrms prms;
    CpswStats_PortStats portStats;
    int32_t status;

    /* Show host port statistics */
    ENET_IOCTL_SET_OUT_ARGS(&prms, &portStats);
    ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, ENET_STATS_IOCTL_GET_HOSTPORT_STATS, &prms, status);
    if (status == ENET_SOK)
    {
        EnetAppUtils_print("\r\n Port 0 Statistics\r\n");
        EnetAppUtils_print("-----------------------------------------\r\n");
        EnetAppUtils_printHostPortStats2G((CpswStats_HostPort_2g *)&portStats);
        EnetAppUtils_print("\r\n");
    }
    else
    {
        EnetAppUtils_print("Failed to get host stats: %d\r\n", status);
    }

    /* Show MAC port statistics */
    if (status == ENET_SOK)
    {
        ENET_IOCTL_SET_INOUT_ARGS(&prms, &gEnetAppCfg.macPorts[0], &portStats);
        ENET_IOCTL(gEnetAppCfg.hEnet, gEnetAppCfg.coreId, ENET_STATS_IOCTL_GET_MACPORT_STATS, &prms, status);
        if (status == ENET_SOK)
        {
            EnetAppUtils_print("\r\n Port 1 Statistics\r\n");
            EnetAppUtils_print("-----------------------------------------\r\n");
            EnetAppUtils_printMacPortStats2G((CpswStats_MacPort_2g *)&portStats);
            EnetAppUtils_print("\r\n");
        }
        else
        {
            EnetAppUtils_print("Failed to get MAC stats: %d\r\n", status);
        }
    }
}

void EnetApp_freeEthTxPkt(void)
{
    EnetMem_freeEthPkt(gEnetAppCfg.pTxPktInfo);

    EnetCpdma_dequeueTxBd(gEnetAppCfg.hTxCh);
}