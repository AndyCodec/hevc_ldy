/*****************************************************************************
 * Copyright (C) 2013-2017 MulticoreWare, Inc
 *
 * Authors: Chung Shin Yee <shinyee@multicorewareinc.com>
 *          Min Chen <chenm003@163.com>
 *          Steve Borho <steve@borho.org>
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

#include "common.h"
#include "frame.h"
#include "framedata.h"
#include "wavefront.h"
#include "param.h"

#include "frameencoder.h"
#include "common.h"
#include "slicetype.h"
#include "nal.h"

namespace X265_NS {
void weightAnalyse(Slice& slice, Frame& frame, x265_param& param);

FrameEncoder::FrameEncoder()
{
    m_prevOutputTime = x265_mdate();
    m_reconfigure = false;
    m_isFrameEncoder = true;
    m_threadActive = true;
    m_slicetypeWaitTime = 0;
    m_activeWorkerCount = 0;
    m_completionCount = 0;
    m_bAllRowsStop = false;
    m_vbvResetTriggerRow = -1;
    m_outStreams = NULL;
    m_backupStreams = NULL;
    m_substreamSizes = NULL;
    m_nr = NULL;
    m_tld = NULL;
    m_rows = NULL;
    //m_top = NULL;
    m_param = NULL;
    m_frame = NULL;
    m_cuGeoms = NULL;
    m_ctuGeomMap = NULL;
    m_localTldIdx = 0;
    memset(&m_rce, 0, sizeof(RateControlEntry));
}

void FrameEncoder::destroy()
{
    if (m_pool)
    {
        if (!m_jpId)
        {
            int numTLD = m_pool->m_numWorkers;
            if (!m_param->bEnableWavefront)
                numTLD += m_pool->m_numProviders;
            for (int i = 0; i < numTLD; i++)
                m_tld[i].destroy();
            delete [] m_tld;
        }
    }
    else
    {
        m_tld->destroy();
        delete m_tld;
    }

    delete[] m_rows;
    delete[] m_outStreams;
    delete[] m_backupStreams;
    X265_FREE(m_sliceBaseRow);
    X265_FREE(m_sliceMaxBlockRow);
    X265_FREE(m_cuGeoms);
    X265_FREE(m_ctuGeomMap);
    X265_FREE(m_substreamSizes);
    X265_FREE(m_nr);

    m_frameFilter.destroy();

    if (m_param->bEmitHRDSEI || !!m_param->interlaceMode)
    {
        delete m_rce.picTimingSEI;
        delete m_rce.hrdTiming;
    }
}

/* Generate a complete list of unique geom sets for the current picture dimensions */
bool FrameEncoder::initializeGeoms()
{
    /* Geoms only vary between CTUs in the presence of picture edges */
    int maxCUSize = m_param->maxCUSize;
    int minCUSize = m_param->minCUSize;
    int heightRem = m_param->sourceHeight & (maxCUSize - 1);
    int widthRem = m_param->sourceWidth & (maxCUSize - 1);
    int allocGeoms = 1; // body
    if (heightRem && widthRem)
        allocGeoms = 4; // body, right, bottom, corner
    else if (heightRem || widthRem)
        allocGeoms = 2; // body, right or bottom

    m_ctuGeomMap = X265_MALLOC(uint32_t, m_numRows * m_numCols);
    m_cuGeoms = X265_MALLOC(CUGeom, allocGeoms * CUGeom::MAX_GEOMS);
    if (!m_cuGeoms || !m_ctuGeomMap)
        return false;

    // body
    CUData::calcCTUGeoms(maxCUSize, maxCUSize, maxCUSize, minCUSize, m_cuGeoms);
    memset(m_ctuGeomMap, 0, sizeof(uint32_t) * m_numRows * m_numCols);
    if (allocGeoms == 1)
        return true;

    int countGeoms = 1;
    if (widthRem)
    {
        // right
        CUData::calcCTUGeoms(widthRem, maxCUSize, maxCUSize, minCUSize, m_cuGeoms + countGeoms * CUGeom::MAX_GEOMS);
        for (uint32_t i = 0; i < m_numRows; i++)
        {
            uint32_t ctuAddr = m_numCols * (i + 1) - 1;
            m_ctuGeomMap[ctuAddr] = countGeoms * CUGeom::MAX_GEOMS;
        }
        countGeoms++;
    }
    if (heightRem)
    {
        // bottom
        CUData::calcCTUGeoms(maxCUSize, heightRem, maxCUSize, minCUSize, m_cuGeoms + countGeoms * CUGeom::MAX_GEOMS);
        for (uint32_t i = 0; i < m_numCols; i++)
        {
            uint32_t ctuAddr = m_numCols * (m_numRows - 1) + i;
            m_ctuGeomMap[ctuAddr] = countGeoms * CUGeom::MAX_GEOMS;
        }
        countGeoms++;

        if (widthRem)
        {
            // corner
            CUData::calcCTUGeoms(widthRem, heightRem, maxCUSize, minCUSize, m_cuGeoms + countGeoms * CUGeom::MAX_GEOMS);

            uint32_t ctuAddr = m_numCols * m_numRows - 1;
            m_ctuGeomMap[ctuAddr] = countGeoms * CUGeom::MAX_GEOMS;
            countGeoms++;
        }
        X265_CHECK(countGeoms == allocGeoms, "geometry match check failure\n");
    }

    return true;
}

void FrameEncoder::threadMain()
{
}

void FrameEncoder::WeightAnalysis::processTasks(int /* workerThreadId */)
{
    Frame* frame = master.m_frame;
    weightAnalyse(*frame->m_encData->m_slice, *frame, *master.m_param);
}


uint32_t getBsLength( int32_t code )
{
    uint32_t ucode = (code <= 0) ? -code << 1 : (code << 1) - 1;

    ++ucode;
    unsigned long idx;
    CLZ( idx, ucode );
    uint32_t length = (uint32_t)idx * 2 + 1;

    return length;
}

void FrameEncoder::encodeSlice(uint32_t sliceAddr)
{
    Slice* slice = m_frame->m_encData->m_slice;
    const uint32_t widthInLCUs = slice->m_sps->numCuInWidth;
    const uint32_t lastCUAddr = (slice->m_endCUAddr + m_param->num4x4Partitions - 1) / m_param->num4x4Partitions;
    const uint32_t numSubstreams = m_param->bEnableWavefront ? slice->m_sps->numCuInHeight : 1;

    SAOParam* saoParam = slice->m_sps->bUseSAO ? m_frame->m_encData->m_saoParam : NULL;
    for (uint32_t cuAddr = sliceAddr; cuAddr < lastCUAddr; cuAddr++)
    {
        uint32_t col = cuAddr % widthInLCUs;
        uint32_t row = cuAddr / widthInLCUs;
        uint32_t subStrm = row % numSubstreams;
        CUData* ctu = m_frame->m_encData->getPicCTU(cuAddr);

        m_entropyCoder.setBitstream(&m_outStreams[subStrm]);

        // Synchronize cabac probabilities with upper-right CTU if it's available and we're at the start of a line.
        if (m_param->bEnableWavefront && !col && row)
        {
            m_entropyCoder.copyState(m_initSliceContext);
            m_entropyCoder.loadContexts(m_rows[row - 1].bufferedEntropy);
        }

        // Initialize slice context
        if (ctu->m_bFirstRowInSlice && !col)
            m_entropyCoder.load(m_initSliceContext);

        if (saoParam)
        {
            if (saoParam->bSaoFlag[0] || saoParam->bSaoFlag[1])
            {
                int mergeLeft = col && saoParam->ctuParam[0][cuAddr].mergeMode == SAO_MERGE_LEFT;
                int mergeUp = !ctu->m_bFirstRowInSlice && saoParam->ctuParam[0][cuAddr].mergeMode == SAO_MERGE_UP;
                if (col)
                    m_entropyCoder.codeSaoMerge(mergeLeft);
                if (!ctu->m_bFirstRowInSlice && !mergeLeft)
                    m_entropyCoder.codeSaoMerge(mergeUp);
                if (!mergeLeft && !mergeUp)
                {
                    if (saoParam->bSaoFlag[0])
                        m_entropyCoder.codeSaoOffset(saoParam->ctuParam[0][cuAddr], 0);
                    if (saoParam->bSaoFlag[1])
                    {
                        m_entropyCoder.codeSaoOffset(saoParam->ctuParam[1][cuAddr], 1);
                        m_entropyCoder.codeSaoOffset(saoParam->ctuParam[2][cuAddr], 2);
                    }
                }
            }
            else
            {
                for (int i = 0; i < (m_param->internalCsp != X265_CSP_I400 ? 3 : 1); i++)
                    saoParam->ctuParam[i][cuAddr].reset();
            }
        }

        // final coding (bitstream generation) for this CU
        m_entropyCoder.encodeCTU(*ctu, m_cuGeoms[m_ctuGeomMap[cuAddr]]);

        if (m_param->bEnableWavefront)
        {
            if (col == 1)
                // Store probabilities of second CTU in line into buffer
                m_rows[row].bufferedEntropy.loadContexts(m_entropyCoder);

            if (col == widthInLCUs - 1)
                m_entropyCoder.finishSlice();
        }
    }

    if (!m_param->bEnableWavefront)
        m_entropyCoder.finishSlice();
}

/* collect statistics about CU coding decisions, return total QP */
int FrameEncoder::collectCTUStatistics(const CUData& ctu, FrameStats* log)
{
    int totQP = 0;
    uint32_t depth = 0;
    for (uint32_t absPartIdx = 0; absPartIdx < ctu.m_numPartitions; absPartIdx += ctu.m_numPartitions >> (depth * 2))
    {
        depth = ctu.m_cuDepth[absPartIdx];
        totQP += ctu.m_qp[absPartIdx] * (ctu.m_numPartitions >> (depth * 2));
    }

    if (m_param->csvLogLevel >= 1 || m_param->rc.bStatWrite)
    {
        if (ctu.m_slice->m_sliceType == I_SLICE)
        {
            depth = 0;
            for (uint32_t absPartIdx = 0; absPartIdx < ctu.m_numPartitions; absPartIdx += ctu.m_numPartitions >> (depth * 2))
            {
                depth = ctu.m_cuDepth[absPartIdx];

                log->totalCu++;
                log->cntIntra[depth]++;

                if (ctu.m_predMode[absPartIdx] == MODE_NONE)
                {
                    log->totalCu--;
                    log->cntIntra[depth]--;
                }
                else if (ctu.m_partSize[absPartIdx] != SIZE_2Nx2N)
                {
                    /* TODO: log intra modes at absPartIdx +0 to +3 */
                    X265_CHECK(ctu.m_log2CUSize[absPartIdx] == 3 && ctu.m_slice->m_sps->quadtreeTULog2MinSize < 3, "Intra NxN found at improbable depth\n");
                    log->cntIntraNxN++;
                    log->cntIntra[depth]--;
                }
                else if (ctu.m_lumaIntraDir[absPartIdx] > 1)
                    log->cuIntraDistribution[depth][ANGULAR_MODE_ID]++;
                else
                    log->cuIntraDistribution[depth][ctu.m_lumaIntraDir[absPartIdx]]++;
            }
        }
        else
        {
            depth = 0;
            for (uint32_t absPartIdx = 0; absPartIdx < ctu.m_numPartitions; absPartIdx += ctu.m_numPartitions >> (depth * 2))
            {
                depth = ctu.m_cuDepth[absPartIdx];

                log->totalCu++;

                if (ctu.m_predMode[absPartIdx] == MODE_NONE)
                    log->totalCu--;
                else if (ctu.isSkipped(absPartIdx))
                {
                    if (ctu.m_mergeFlag[0])
                        log->cntMergeCu[depth]++;
                    else
                        log->cntSkipCu[depth]++;
                }
                else if (ctu.isInter(absPartIdx))
                {
                    log->cntInter[depth]++;

                    if (ctu.m_partSize[absPartIdx] < AMP_ID)
                        log->cuInterDistribution[depth][ctu.m_partSize[absPartIdx]]++;
                    else
                        log->cuInterDistribution[depth][AMP_ID]++;
                }
                else if (ctu.isIntra(absPartIdx))
                {
                    log->cntIntra[depth]++;

                    if (ctu.m_partSize[absPartIdx] != SIZE_2Nx2N)
                    {
                        X265_CHECK(ctu.m_log2CUSize[absPartIdx] == 3 && ctu.m_slice->m_sps->quadtreeTULog2MinSize < 3, "Intra NxN found at improbable depth\n");
                        log->cntIntraNxN++;
                        log->cntIntra[depth]--;
                        /* TODO: log intra modes at absPartIdx +0 to +3 */
                    }
                    else if (ctu.m_lumaIntraDir[absPartIdx] > 1)
                        log->cuIntraDistribution[depth][ANGULAR_MODE_ID]++;
                    else
                        log->cuIntraDistribution[depth][ctu.m_lumaIntraDir[absPartIdx]]++;
                }
            }
        }
    }

    return totQP;
}

/* DCT-domain noise reduction / adaptive deadzone from libavcodec */
void FrameEncoder::noiseReductionUpdate()
{
    static const uint32_t maxBlocksPerTrSize[4] = {1 << 18, 1 << 16, 1 << 14, 1 << 12};

    for (int cat = 0; cat < MAX_NUM_TR_CATEGORIES; cat++)
    {
        int trSize = cat & 3;
        int coefCount = 1 << ((trSize + 2) * 2);

        if (m_nr->nrCount[cat] > maxBlocksPerTrSize[trSize])
        {
            for (int i = 0; i < coefCount; i++)
                m_nr->nrResidualSum[cat][i] >>= 1;
            m_nr->nrCount[cat] >>= 1;
        }

        int nrStrength = cat < 8 ? m_param->noiseReductionIntra : m_param->noiseReductionInter;
        uint64_t scaledCount = (uint64_t)nrStrength * m_nr->nrCount[cat];

        for (int i = 0; i < coefCount; i++)
        {
            uint64_t value = scaledCount + m_nr->nrResidualSum[cat][i] / 2;
            uint64_t denom = m_nr->nrResidualSum[cat][i] + 1;
            m_nr->nrOffsetDenoise[cat][i] = (uint16_t)(value / denom);
        }

        // Don't denoise DC coefficients
        m_nr->nrOffsetDenoise[cat][0] = 0;
    }
}
#if ENABLE_LIBVMAF
void FrameEncoder::vmafFrameLevelScore()
{
    PicYuv *fenc = m_frame->m_fencPic;
    PicYuv *recon = m_frame->m_reconPic;

    x265_vmaf_framedata *vmafframedata = (x265_vmaf_framedata*)x265_malloc(sizeof(x265_vmaf_framedata));
    if (!vmafframedata)
    {
        x265_log(NULL, X265_LOG_ERROR, "vmaf frame data alloc failed\n");
    }

    vmafframedata->height = fenc->m_picHeight;
    vmafframedata->width = fenc->m_picWidth;
    vmafframedata->frame_set = 0;
    vmafframedata->internalBitDepth = m_param->internalBitDepth;
    vmafframedata->reference_frame = fenc;
    vmafframedata->distorted_frame = recon;

    fenc->m_vmafScore = x265_calculate_vmaf_framelevelscore(vmafframedata);

    if (vmafframedata)
    x265_free(vmafframedata);
}
#endif

}
