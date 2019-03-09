/*****************************************************************************
 * Copyright (C) 2013-2017 MulticoreWare, Inc
 *
 * Authors: Steve Borho <steve@borho.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
 *
 * This program is also available under a commercial proprietary license.
 * For more information, contact us at license @ x265.com.
 *****************************************************************************/

#ifndef X265_ENCODER_H
#define X265_ENCODER_H

#include "common.h"
#include "slice.h"
#include "threading.h"
#include "scalinglist.h"
#include "x265.h"
#include "framedata.h"

struct x265_encoder {};
namespace X265_NS {
// private namespace
extern const char g_sliceTypeToChar[3];

class Entropy;

#define MAX_NUM_REF_IDX 64

struct RefIdxLastGOP
{
    int numRefIdxDefault[2];
    int numRefIdxl0[MAX_NUM_REF_IDX];
    int numRefIdxl1[MAX_NUM_REF_IDX];
};

struct RPSListNode
{
    int idx;
    int count;
    RPS* rps;
    RPSListNode* next;
    RPSListNode* prior;
};

struct cuLocation
{
    bool skipWidth;
    bool skipHeight;
    uint32_t heightInCU;
    uint32_t widthInCU;
    uint32_t oddRowIndex;
    uint32_t evenRowIndex;
    uint32_t switchCondition;

    void init(x265_param* param)
    {
        skipHeight = false;
        skipWidth = false;
        heightInCU = (param->sourceHeight + param->maxCUSize - 1) >> param->maxLog2CUSize;
        widthInCU = (param->sourceWidth + param->maxCUSize - 1) >> param->maxLog2CUSize;
        evenRowIndex = 0;
        oddRowIndex = param->num4x4Partitions * widthInCU;
        switchCondition = 0; // To switch between odd and even rows
    }
};

struct puOrientation
{
    bool isVert;
    bool isRect;
    bool isAmp;

    void init()
    {
        isRect = false;
        isAmp = false;
        isVert = false;
    }
};


class DPB;
class Lookahead;
class ThreadPool;
class FrameData;

class LookEncoder : public x265_encoder
{
public:

    uint16_t           (*m_offsetEmergency)[MAX_NUM_TR_CATEGORIES][MAX_NUM_TR_COEFFS];

    int64_t            m_firstPts;
    int64_t            m_bframeDelayTime;
    int64_t            m_encodeStartTime;

    int                m_pocLast;         // time index (POC)
    int                m_bframeDelay;
    int                m_numPools;

    // weighted prediction
    uint32_t           m_numDelayedPic;

    ThreadPool*        m_threadPool;
    DPB*               m_dpb;
    x265_param*        m_param;
    x265_param*        m_latestParam;     // Holds latest param during a reconfigure
    Lookahead*         m_lookahead;

    bool               m_externalFlush;

    VPS                m_vps;
    SPS                m_sps;
    PPS                m_pps;
    ScalingList        m_scalingList;      // quantization matrix information
    Window             m_conformanceWindow;

    bool               m_emitCLLSEI;
    bool               m_bZeroLatency;     // x265_encoder_encode() returns NALs for the input picture, zero lag
    bool               m_aborted;          // fatal error detected
    bool               m_reconfigure;      // Encoder reconfigure in progress
    bool               m_reconfigureRc;

    /* Begin intra refresh when one not in progress or else begin one as soon as the current
    * one is done. Requires bIntraRefresh to be set.*/
    int                m_bQueuedIntraRefresh;

    /* For optimising slice QP */
    Lock               m_sliceQpLock;
    int64_t            m_iBitsCostSum[QP_MAX_MAX + 1];
    Lock               m_sliceRefIdxLock;
    RefIdxLastGOP      m_refIdxLastGOP;

    Lock               m_rpsInSpsLock;
    /* For HDR*/
    double                m_cB;
    double                m_cR;

    int                     m_bToneMap; // Enables tone-mapping

    x265_sei_payload        m_prevTonemapPayload;

    /* Collect frame level feature data */
    uint64_t*               m_rdCost;
    uint64_t*               m_variance;
    uint32_t*               m_trainingCount;
    int32_t                 m_startPoint;
    Lock                    m_dynamicRefineLock;

    bool                    m_saveCTUSize;

    LookEncoder();
    ~LookEncoder()
    {
    };

    void create();
    void stopJobs();
    void destroy();

    int encode_lookahead(const x265_picture* pic);

    void printSummary();

    void configure(x265_param *param);
    void initRefIdx();

protected:

    void initVPS(VPS *vps);
    void initSPS(SPS *sps);
    void initPPS(PPS *pps);
};
}

#endif // ifndef X265_ENCODER_H
