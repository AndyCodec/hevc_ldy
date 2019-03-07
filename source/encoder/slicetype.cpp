/*****************************************************************************
 * Copyright (C) 2013-2017 MulticoreWare, Inc
 *
 * Authors: Gopu Govindaswamy <gopu@multicorewareinc.com>
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
#include "picyuv.h"
#include "primitives.h"
#include "lowres.h"
#include "mv.h"

#include "slicetype.h"
#include "motion.h"
//#include "ratecontrol.h"

#if DETAILED_CU_STATS
#define ProfileLookaheadTime(elapsed, count) ScopedElapsedTime _scope(elapsed); count++
#else
#define ProfileLookaheadTime(elapsed, count)
#endif

using namespace X265_NS;

namespace {

/* Compute variance to derive AC energy of each block */
inline uint32_t acEnergyVar(Frame *curFrame, uint64_t sum_ssd, int shift, int plane)
{
    uint32_t sum = (uint32_t)sum_ssd;
    uint32_t ssd = (uint32_t)(sum_ssd >> 32);

    curFrame->m_lowres.wp_sum[plane] += sum;
    curFrame->m_lowres.wp_ssd[plane] += ssd;
    return ssd - ((uint64_t)sum * sum >> shift);
}

/* Find the energy of each block in Y/Cb/Cr plane */
inline uint32_t acEnergyPlane(Frame *curFrame, pixel* src, intptr_t srcStride, int plane, int colorFormat, uint32_t qgSize)
{
    if ((colorFormat != X265_CSP_I444) && plane)
    {
        if (qgSize == 8)
        {
            ALIGN_VAR_4(pixel, pix[4 * 4]);
            primitives.cu[BLOCK_4x4].copy_pp(pix, 4, src, srcStride);
            return acEnergyVar(curFrame, primitives.cu[BLOCK_4x4].var(pix, 4), 4, plane);
        }
        else
        {
            ALIGN_VAR_8(pixel, pix[8 * 8]);
            primitives.cu[BLOCK_8x8].copy_pp(pix, 8, src, srcStride);
            return acEnergyVar(curFrame, primitives.cu[BLOCK_8x8].var(pix, 8), 6, plane);
        }
    }
    else
    {
        if (qgSize == 8)
            return acEnergyVar(curFrame, primitives.cu[BLOCK_8x8].var(src, srcStride), 6, plane);
        else
            return acEnergyVar(curFrame, primitives.cu[BLOCK_16x16].var(src, srcStride), 8, plane);
    }
}

} // end anonymous namespace

/* Find the total AC energy of each block in all planes */
uint32_t LookaheadTLD::acEnergyCu(Frame* curFrame, uint32_t blockX, uint32_t blockY, int csp, uint32_t qgSize)
{
    intptr_t stride = curFrame->m_fencPic->m_stride;
    intptr_t cStride = curFrame->m_fencPic->m_strideC;
    intptr_t blockOffsetLuma = blockX + (blockY * stride);
    int hShift = CHROMA_H_SHIFT(csp);
    int vShift = CHROMA_V_SHIFT(csp);
    intptr_t blockOffsetChroma = (blockX >> hShift) + ((blockY >> vShift) * cStride);

    uint32_t var;

    var  = acEnergyPlane(curFrame, curFrame->m_fencPic->m_picOrg[0] + blockOffsetLuma, stride, 0, csp, qgSize);
    if (csp != X265_CSP_I400 && curFrame->m_fencPic->m_picCsp != X265_CSP_I400)
    {
        var += acEnergyPlane(curFrame, curFrame->m_fencPic->m_picOrg[1] + blockOffsetChroma, cStride, 1, csp, qgSize);
        var += acEnergyPlane(curFrame, curFrame->m_fencPic->m_picOrg[2] + blockOffsetChroma, cStride, 2, csp, qgSize);
    }
    x265_emms();
    return var;
}
/* Find the sum of pixels of each block for luma plane */
uint32_t LookaheadTLD::lumaSumCu(Frame* curFrame, uint32_t blockX, uint32_t blockY, uint32_t qgSize)
{
    intptr_t stride = curFrame->m_fencPic->m_stride;
    intptr_t blockOffsetLuma = blockX + (blockY * stride);
    uint64_t sum_ssd;

    if (qgSize == 8)
        sum_ssd = primitives.cu[BLOCK_8x8].var(curFrame->m_fencPic->m_picOrg[0] + blockOffsetLuma, stride);
    else
        sum_ssd = primitives.cu[BLOCK_16x16].var(curFrame->m_fencPic->m_picOrg[0] + blockOffsetLuma, stride);

    x265_emms();
    return (uint32_t)sum_ssd;
}

void LookaheadTLD::calcAdaptiveQuantFrame(Frame *curFrame, x265_param* param)
{
    /* Actual adaptive quantization */
    int maxCol = curFrame->m_fencPic->m_picWidth;
    int maxRow = curFrame->m_fencPic->m_picHeight;
    int blockCount, loopIncr;
    float modeOneConst, modeTwoConst;
    if (param->rc.qgSize == 8)
    {
        blockCount = curFrame->m_lowres.maxBlocksInRowFullRes * curFrame->m_lowres.maxBlocksInColFullRes;
        modeOneConst = 11.427f;
        modeTwoConst = 8.f;
        loopIncr = 8;
    }
    else
    {
        blockCount = widthInCU * heightInCU;
        modeOneConst = 14.427f;
        modeTwoConst = 11.f;
        loopIncr = 16;
    }

    float* quantOffsets = curFrame->m_quantOffsets;
    for (int y = 0; y < 3; y++)
    {
        curFrame->m_lowres.wp_ssd[y] = 0;
        curFrame->m_lowres.wp_sum[y] = 0;
    }

    /* Calculate Qp offset for each 16x16 or 8x8 block in the frame */
    int blockXY = 0;
    int blockX = 0, blockY = 0;
    double strength = 0.f;
    if ((param->rc.aqMode == X265_AQ_NONE || param->rc.aqStrength == 0) || (param->rc.bStatRead && param->rc.cuTree && IS_REFERENCED(curFrame)))
    {
        /* Need to init it anyways for CU tree */
        int cuCount = blockCount;

        if (param->rc.aqMode && param->rc.aqStrength == 0)
        {
            if (quantOffsets)
            {
                for (int cuxy = 0; cuxy < cuCount; cuxy++)
                {
                    curFrame->m_lowres.qpCuTreeOffset[cuxy] = curFrame->m_lowres.qpAqOffset[cuxy] = quantOffsets[cuxy];
                    curFrame->m_lowres.invQscaleFactor[cuxy] = x265_exp2fix8(curFrame->m_lowres.qpCuTreeOffset[cuxy]);
                }
            }
            else
            {
                memset(curFrame->m_lowres.qpCuTreeOffset, 0, cuCount * sizeof(double));
                memset(curFrame->m_lowres.qpAqOffset, 0, cuCount * sizeof(double));
                for (int cuxy = 0; cuxy < cuCount; cuxy++)
                    curFrame->m_lowres.invQscaleFactor[cuxy] = 256;
            }
        }

        /* Need variance data for weighted prediction and dynamic refinement*/
        if (param->bEnableWeightedPred || param->bEnableWeightedBiPred)
        {            
            for (blockY = 0; blockY < maxRow; blockY += loopIncr)
                for (blockX = 0; blockX < maxCol; blockX += loopIncr)                
                    acEnergyCu(curFrame, blockX, blockY, param->internalCsp, param->rc.qgSize);                
        }
    }
    else
    {
        blockXY = 0;
        double avg_adj_pow2 = 0, avg_adj = 0, qp_adj = 0;
        double bias_strength = 0.f;
        if (param->rc.aqMode == X265_AQ_AUTO_VARIANCE || param->rc.aqMode == X265_AQ_AUTO_VARIANCE_BIASED)
        {
            double bit_depth_correction = 1.f / (1 << (2*(X265_DEPTH-8)));
            curFrame->m_lowres.frameVariance = 0;
            uint64_t rowVariance = 0;
            for (blockY = 0; blockY < maxRow; blockY += loopIncr)
            {
                rowVariance = 0;
                for (blockX = 0; blockX < maxCol; blockX += loopIncr)
                {
                    uint32_t energy = acEnergyCu(curFrame, blockX, blockY, param->internalCsp, param->rc.qgSize);
                    rowVariance += energy;
                    qp_adj = pow(energy * bit_depth_correction + 1, 0.1);
                    curFrame->m_lowres.qpCuTreeOffset[blockXY] = qp_adj;
                    avg_adj += qp_adj;
                    avg_adj_pow2 += qp_adj * qp_adj;
                    blockXY++;
                }
                curFrame->m_lowres.frameVariance += (rowVariance / maxCol);
            }
            curFrame->m_lowres.frameVariance /= maxRow;
            avg_adj /= blockCount;
            avg_adj_pow2 /= blockCount;
            strength = param->rc.aqStrength * avg_adj;
            avg_adj = avg_adj - 0.5f * (avg_adj_pow2 - (modeTwoConst)) / avg_adj;
            bias_strength = param->rc.aqStrength;
        }
        else
            strength = param->rc.aqStrength * 1.0397f;

        blockXY = 0;
        for (blockY = 0; blockY < maxRow; blockY += loopIncr)
        {
            for (blockX = 0; blockX < maxCol; blockX += loopIncr)
            {
                if (param->rc.aqMode == X265_AQ_AUTO_VARIANCE_BIASED)
                {
                    qp_adj = curFrame->m_lowres.qpCuTreeOffset[blockXY];
                    qp_adj = strength * (qp_adj - avg_adj) + bias_strength * (1.f - modeTwoConst / (qp_adj * qp_adj));
                }
                else if (param->rc.aqMode == X265_AQ_AUTO_VARIANCE)
                {
                    qp_adj = curFrame->m_lowres.qpCuTreeOffset[blockXY];
                    qp_adj = strength * (qp_adj - avg_adj);
                }
                else
                {
                    uint32_t energy = acEnergyCu(curFrame, blockX, blockY, param->internalCsp,param->rc.qgSize);
                    qp_adj = strength * (X265_LOG2(X265_MAX(energy, 1)) - (modeOneConst + 2 * (X265_DEPTH - 8)));                    
                }

                if (param->bHDROpt)
                {
                    uint32_t sum = lumaSumCu(curFrame, blockX, blockY, param->rc.qgSize);
                    uint32_t lumaAvg = sum / (loopIncr * loopIncr);
                    if (lumaAvg < 301)
                        qp_adj += 3;
                    else if (lumaAvg >= 301 && lumaAvg < 367)
                        qp_adj += 2;
                    else if (lumaAvg >= 367 && lumaAvg < 434)
                        qp_adj += 1;
                    else if (lumaAvg >= 501 && lumaAvg < 567)
                        qp_adj -= 1;
                    else if (lumaAvg >= 567 && lumaAvg < 634)
                        qp_adj -= 2;
                    else if (lumaAvg >= 634 && lumaAvg < 701)
                        qp_adj -= 3;
                    else if (lumaAvg >= 701 && lumaAvg < 767)
                        qp_adj -= 4;
                    else if (lumaAvg >= 767 && lumaAvg < 834)
                        qp_adj -= 5;
                    else if (lumaAvg >= 834)
                        qp_adj -= 6;
                }
                if (quantOffsets != NULL)
                    qp_adj += quantOffsets[blockXY];
                curFrame->m_lowres.qpAqOffset[blockXY] = qp_adj;
                curFrame->m_lowres.qpCuTreeOffset[blockXY] = qp_adj;
                curFrame->m_lowres.invQscaleFactor[blockXY] = x265_exp2fix8(qp_adj);
                blockXY++;
            }
        }
    }

    if (param->rc.qgSize == 8)
    {
        for (int cuY = 0; cuY < heightInCU; cuY++)
        {
            for (int cuX = 0; cuX < widthInCU; cuX++)
            {
                const int cuXY = cuX + cuY * widthInCU;
                curFrame->m_lowres.invQscaleFactor8x8[cuXY] = (curFrame->m_lowres.invQscaleFactor[cuX * 2 + cuY * widthInCU * 4] +
                                                               curFrame->m_lowres.invQscaleFactor[cuX * 2 + cuY * widthInCU * 4 + 1] +
                                                               curFrame->m_lowres.invQscaleFactor[cuX * 2 + cuY * widthInCU * 4 + curFrame->m_lowres.maxBlocksInRowFullRes] +
                                                               curFrame->m_lowres.invQscaleFactor[cuX * 2 + cuY * widthInCU * 4 + curFrame->m_lowres.maxBlocksInRowFullRes + 1]) / 4;
            }
        }
    }

    if (param->bEnableWeightedPred || param->bEnableWeightedBiPred)
    {
        int hShift = CHROMA_H_SHIFT(param->internalCsp);
        int vShift = CHROMA_V_SHIFT(param->internalCsp);
        maxCol = ((maxCol + 8) >> 4) << 4;
        maxRow = ((maxRow + 8) >> 4) << 4;
        int width[3]  = { maxCol, maxCol >> hShift, maxCol >> hShift };
        int height[3] = { maxRow, maxRow >> vShift, maxRow >> vShift };

        for (int i = 0; i < 3; i++)
        {
            uint64_t sum, ssd;
            sum = curFrame->m_lowres.wp_sum[i];
            ssd = curFrame->m_lowres.wp_ssd[i];
            curFrame->m_lowres.wp_ssd[i] = ssd - (sum * sum + (width[i] * height[i]) / 2) / (width[i] * height[i]);
        }
    }

    if (param->bDynamicRefine)
    {
        blockXY = 0;
        for (blockY = 0; blockY < maxRow; blockY += loopIncr)
            for (blockX = 0; blockX < maxCol; blockX += loopIncr)
            {
                curFrame->m_lowres.blockVariance[blockXY] = acEnergyCu(curFrame, blockX, blockY, param->internalCsp, param->rc.qgSize);
                blockXY++;
            }
    }
}


/** 函数功能       ： 计算当前1/2下采样帧的intra SATD值以及最优intra模式
/*  调用范围       ： 只在PreLookaheadGroup::processTasks函数中被调用
* \参数 fenc       ： 当前帧（经过1/2下采样后的数据）
*   返回值         ： null
**/
void LookaheadTLD::lowresIntraEstimate(Lowres& fenc, uint32_t qgSize)
{
    ALIGN_VAR_32(pixel, prediction[X265_LOWRES_CU_SIZE * X265_LOWRES_CU_SIZE]);
    pixel fencIntra[X265_LOWRES_CU_SIZE * X265_LOWRES_CU_SIZE];
    pixel neighbours[2][X265_LOWRES_CU_SIZE * 4 + 1];
    pixel* samples = neighbours[0], *filtered = neighbours[1];

    const int lookAheadLambda = (int)x265_lambda_tab[X265_LOOKAHEAD_QP];
    const int intraPenalty = 5 * lookAheadLambda;
    const int lowresPenalty = 4; /* fixed CU cost overhead */

    const int cuSize  = X265_LOWRES_CU_SIZE;
    const int cuSize2 = cuSize << 1;
    const int sizeIdx = X265_LOWRES_CU_BITS - 2;

    pixelcmp_t satd = primitives.pu[sizeIdx].satd;
    int planar = !!(cuSize >= 8);

    int costEst = 0, costEstAq = 0;

    for (int cuY = 0; cuY < heightInCU; cuY++)
    {
        fenc.rowSatds[0][0][cuY] = 0;

        for (int cuX = 0; cuX < widthInCU; cuX++)
        {
            const int cuXY = cuX + cuY * widthInCU;
            const intptr_t pelOffset = cuSize * cuX + cuSize * cuY * fenc.lumaStride;
            pixel *pixCur = fenc.lowresPlane[0] + pelOffset;

            /* copy fenc pixels */
            primitives.cu[sizeIdx].copy_pp(fencIntra, cuSize, pixCur, fenc.lumaStride);

            /* collect reference sample pixels */
            pixCur -= fenc.lumaStride + 1;
            memcpy(samples, pixCur, (2 * cuSize + 1) * sizeof(pixel)); /* top */
            for (int i = 1; i <= 2 * cuSize; i++)
                samples[cuSize2 + i] = pixCur[i * fenc.lumaStride];    /* left */

            primitives.cu[sizeIdx].intra_filter(samples, filtered);

            int cost, icost = me.COST_MAX;
            uint32_t ilowmode = 0;

            /* DC and planar */
            primitives.cu[sizeIdx].intra_pred[DC_IDX](prediction, cuSize, samples, 0, cuSize <= 16);
            cost = satd(fencIntra, cuSize, prediction, cuSize);
            COPY2_IF_LT(icost, cost, ilowmode, DC_IDX);

            primitives.cu[sizeIdx].intra_pred[PLANAR_IDX](prediction, cuSize, neighbours[planar], 0, 0);
            cost = satd(fencIntra, cuSize, prediction, cuSize);
            COPY2_IF_LT(icost, cost, ilowmode, PLANAR_IDX);

            /* scan angular predictions */
            int filter, acost = me.COST_MAX;
            uint32_t mode, alowmode = 4;
            for (mode = 5; mode < 35; mode += 5)
            {
                filter = !!(g_intraFilterFlags[mode] & cuSize);
                primitives.cu[sizeIdx].intra_pred[mode](prediction, cuSize, neighbours[filter], mode, cuSize <= 16);
                cost = satd(fencIntra, cuSize, prediction, cuSize);
                COPY2_IF_LT(acost, cost, alowmode, mode);
            }
            for (uint32_t dist = 2; dist >= 1; dist--)
            {
                int minusmode = alowmode - dist;
                int plusmode = alowmode + dist;

                mode = minusmode;
                filter = !!(g_intraFilterFlags[mode] & cuSize);
                primitives.cu[sizeIdx].intra_pred[mode](prediction, cuSize, neighbours[filter], mode, cuSize <= 16);
                cost = satd(fencIntra, cuSize, prediction, cuSize);
                COPY2_IF_LT(acost, cost, alowmode, mode);

                mode = plusmode;
                filter = !!(g_intraFilterFlags[mode] & cuSize);
                primitives.cu[sizeIdx].intra_pred[mode](prediction, cuSize, neighbours[filter], mode, cuSize <= 16);
                cost = satd(fencIntra, cuSize, prediction, cuSize);
                COPY2_IF_LT(acost, cost, alowmode, mode);
            }
            COPY2_IF_LT(icost, acost, ilowmode, alowmode);

            icost += intraPenalty + lowresPenalty; /* estimate intra signal cost */

            fenc.lowresCosts[0][0][cuXY] = (uint16_t)(X265_MIN(icost, LOWRES_COST_MASK) | (0 << LOWRES_COST_SHIFT));
            fenc.intraCost[cuXY] = icost;
            fenc.intraMode[cuXY] = (uint8_t)ilowmode;
            /* do not include edge blocks in the 
            frame cost estimates, they are not very accurate */
            const bool bFrameScoreCU = (cuX > 0 && cuX < widthInCU - 1 &&
                                        cuY > 0 && cuY < heightInCU - 1) || widthInCU <= 2 || heightInCU <= 2;
            int icostAq;
            if (qgSize == 8)
                icostAq = (bFrameScoreCU && fenc.invQscaleFactor) ? ((icost * fenc.invQscaleFactor8x8[cuXY] + 128) >> 8) : icost;
            else
                icostAq = (bFrameScoreCU && fenc.invQscaleFactor) ? ((icost * fenc.invQscaleFactor[cuXY] +128) >> 8) : icost;

            if (bFrameScoreCU)
            {
                costEst += icost;
                costEstAq += icostAq;
            }

            fenc.rowSatds[0][0][cuY] += icostAq;
        }
    }

    fenc.costEst[0][0] = costEst;
    fenc.costEstAq[0][0] = costEstAq;
}

/** 函数功能             ：计算两帧（wp.bPresentFlag 为 ref是否加权）之间的SATD值
/*  调用范围             ：只在LookaheadTLD::weightsAnalyse函数中被调用
* \参数 fenc             ：当前帧
* \参数 ref              ：前向帧
* \返回                  ：两帧（wp.bPresentFlag 为 ref是否加权）之间的SATD值 */
uint32_t LookaheadTLD::weightCostLuma(Lowres& fenc, Lowres& ref, WeightParam& wp)
{
    pixel *src = ref.fpelPlane[0];
    intptr_t stride = fenc.lumaStride;

    if (wp.bPresentFlag)
    {
        int offset = wp.inputOffset << (X265_DEPTH - 8);
        int scale = wp.inputWeight;
        int denom = wp.log2WeightDenom;
        int round = denom ? 1 << (denom - 1) : 0;
        int correction = IF_INTERNAL_PREC - X265_DEPTH; // intermediate interpolation depth
        int widthHeight = (int)stride;

        primitives.weight_pp(ref.buffer[0], wbuffer[0], stride, widthHeight, paddedLines,
            scale, round << correction, denom + correction, offset);
        src = fenc.weightedRef[fenc.frameNum - ref.frameNum].fpelPlane[0];
    }

    uint32_t cost = 0;
    intptr_t pixoff = 0;
    int mb = 0;

    for (int y = 0; y < fenc.lines; y += 8, pixoff = y * stride)
    {
        for (int x = 0; x < fenc.width; x += 8, mb++, pixoff += 8)
        {
            int satd = primitives.pu[LUMA_8x8].satd(src + pixoff, stride, fenc.fpelPlane[0] + pixoff, stride);
            cost += X265_MIN(satd, fenc.intraCost[mb]);
        }
    }

    return cost;
}

bool LookaheadTLD::allocWeightedRef(Lowres& fenc)
{
    intptr_t planesize = fenc.buffer[1] - fenc.buffer[0];
    paddedLines = (int)(planesize / fenc.lumaStride);

    wbuffer[0] = X265_MALLOC(pixel, 4 * planesize);
    if (wbuffer[0])
    {
        wbuffer[1] = wbuffer[0] + planesize;
        wbuffer[2] = wbuffer[1] + planesize;
        wbuffer[3] = wbuffer[2] + planesize;
    }
    else
        return false;

    return true;
}

void LookaheadTLD::weightsAnalyse(Lowres& fenc, Lowres& ref)
{
    static const float epsilon = 1.f / 128.f;
    int deltaIndex = fenc.frameNum - ref.frameNum;

    WeightParam wp;
    wp.bPresentFlag = false;

    if (!wbuffer[0])
    {
        if (!allocWeightedRef(fenc))
            return;
    }

    ReferencePlanes& weightedRef = fenc.weightedRef[deltaIndex];
    intptr_t padoffset = fenc.lowresPlane[0] - fenc.buffer[0];
    for (int i = 0; i < 4; i++)
        weightedRef.lowresPlane[i] = wbuffer[i] + padoffset;

    weightedRef.fpelPlane[0] = weightedRef.lowresPlane[0];
    weightedRef.lumaStride = fenc.lumaStride;
    weightedRef.isLowres = true;
    weightedRef.isWeighted = false;

    /* epsilon is chosen to require at least a numerator of 127 (with denominator = 128) */
    float guessScale, fencMean, refMean;
    x265_emms();
    if (fenc.wp_ssd[0] && ref.wp_ssd[0])
        guessScale = sqrtf((float)fenc.wp_ssd[0] / ref.wp_ssd[0]);
    else
        guessScale = 1.0f;
    fencMean = (float)fenc.wp_sum[0] / (fenc.lines * fenc.width) / (1 << (X265_DEPTH - 8));
    refMean = (float)ref.wp_sum[0] / (fenc.lines * fenc.width) / (1 << (X265_DEPTH - 8));

    /* Early termination */
    if (fabsf(refMean - fencMean) < 0.5f && fabsf(1.f - guessScale) < epsilon)
        return;

    int minoff = 0, minscale, mindenom;
    unsigned int minscore = 0, origscore = 1;
    int found = 0;

    wp.setFromWeightAndOffset((int)(guessScale * 128 + 0.5f), 0, 7, true);
    mindenom = wp.log2WeightDenom;
    minscale = wp.inputWeight;

    origscore = minscore = weightCostLuma(fenc, ref, wp);

    if (!minscore)
        return;

    unsigned int s = 0;
    int curScale = minscale;
    int curOffset = (int)(fencMean - refMean * curScale / (1 << mindenom) + 0.5f);
    if (curOffset < -128 || curOffset > 127)
    {
        /* Rescale considering the constraints on curOffset. We do it in this order
        * because scale has a much wider range than offset (because of denom), so
        * it should almost never need to be clamped. */
        curOffset = x265_clip3(-128, 127, curOffset);
        curScale = (int)((1 << mindenom) * (fencMean - curOffset) / refMean + 0.5f);
        curScale = x265_clip3(0, 127, curScale);
    }
    SET_WEIGHT(wp, true, curScale, mindenom, curOffset);
    s = weightCostLuma(fenc, ref, wp);
    COPY4_IF_LT(minscore, s, minscale, curScale, minoff, curOffset, found, 1);

    /* Use a smaller denominator if possible */
    if (mindenom > 0 && !(minscale & 1))
    {
        unsigned long idx;
        CTZ(idx, minscale);
        int shift = X265_MIN((int)idx, mindenom);
        mindenom -= shift;
        minscale >>= shift;
    }

    if (!found || (minscale == 1 << mindenom && minoff == 0) || (float)minscore / origscore > 0.998f)
        return;
    else
    {
        SET_WEIGHT(wp, true, minscale, mindenom, minoff);

        // set weighted delta cost
        fenc.weightedCostDelta[deltaIndex] = minscore / origscore;

        int offset = wp.inputOffset << (X265_DEPTH - 8);
        int scale = wp.inputWeight;
        int denom = wp.log2WeightDenom;
        int round = denom ? 1 << (denom - 1) : 0;
        int correction = IF_INTERNAL_PREC - X265_DEPTH; // intermediate interpolation depth
        intptr_t stride = ref.lumaStride;
        int widthHeight = (int)stride;

        for (int i = 0; i < 4; i++)
            primitives.weight_pp(ref.buffer[i], wbuffer[i], stride, widthHeight, paddedLines,
            scale, round << correction, denom + correction, offset);

        weightedRef.isWeighted = true;
    }
}

/*
* 只在Encoder::create()中调用：m_lookahead = new Lookahead(m_param, lookAheadThreadPool);
*/
Lookahead::Lookahead(x265_param *param, ThreadPool* pool)
{
    m_param = param;
    m_pool  = pool;

    m_lastNonB = NULL;
    m_isSceneTransition = false;
    m_scratch  = NULL;
    m_tld      = NULL;
    m_filled   = false;
    m_outputSignalRequired = false;
    m_isActive = true;
    m_inputCount = 0;
    m_extendGopBoundary = false;
    m_8x8Height = ((m_param->sourceHeight / 2) + X265_LOWRES_CU_SIZE - 1) >> X265_LOWRES_CU_BITS;
    m_8x8Width = ((m_param->sourceWidth / 2) + X265_LOWRES_CU_SIZE - 1) >> X265_LOWRES_CU_BITS;
    m_cuCount = m_8x8Width * m_8x8Height;
    m_8x8Blocks = m_8x8Width > 2 && m_8x8Height > 2 ? (m_cuCount + 4 - 2 * (m_8x8Width + m_8x8Height)) : m_cuCount;

    /* Allow the strength to be adjusted via qcompress, since the two concepts
     * are very similar. */

    m_cuTreeStrength = 5.0 * (1.0 - m_param->rc.qCompress);

    m_lastKeyframe = -m_param->keyframeMax;//标记当前前一个关键帧位置，初始化为-maxKeyframe， 保证第一帧是关键帧
    m_sliceTypeBusy = false;
    m_fullQueueSize = X265_MAX(1, m_param->lookaheadDepth);
    m_bAdaptiveQuant = m_param->rc.aqMode || m_param->bEnableWeightedPred || m_param->bEnableWeightedBiPred || m_param->bAQMotion;

    /* If we have a thread pool and are using --b-adapt 2, it is generally
     * preferable to perform all motion searches for each lowres frame in large
     * batched; this will create one job per --bframe per lowres frame, and
     * these jobs are performed by workers bonded to the thread running
     * slicetypeDecide() */
    m_bBatchMotionSearch = m_pool && m_param->bFrameAdaptive == X265_B_ADAPT_TRELLIS;

    /* It is also beneficial to pre-calculate all possible frame cost estimates
     * using worker threads bonded to the worker thread running
     * slicetypeDecide(). This creates bframes * bframes jobs which take less
     * time than the motion search batches but there are many of them. This may
     * do much unnecessary work, some frame cost estimates are not needed, so if
     * the thread pool is small we disable this feature after the initial burst
     * of work */
    m_bBatchFrameCosts = m_bBatchMotionSearch;

    if (m_param->lookaheadSlices && !m_pool)
    {
        x265_log(param, X265_LOG_WARNING, "No pools found; disabling lookahead-slices\n");
        m_param->lookaheadSlices = 0;
    }

    if (m_param->lookaheadSlices && (m_param->sourceHeight < 720))
    {
        x265_log(param, X265_LOG_WARNING, "Source height < 720p; disabling lookahead-slices\n");
        m_param->lookaheadSlices = 0;
    }

    if (m_param->lookaheadSlices > 1)
    {
        m_numRowsPerSlice = m_8x8Height / m_param->lookaheadSlices;
        m_numRowsPerSlice = X265_MAX(m_numRowsPerSlice, 10);            // at least 10 rows per slice
        m_numRowsPerSlice = X265_MIN(m_numRowsPerSlice, m_8x8Height);   // but no more than the full picture
        m_numCoopSlices = m_8x8Height / m_numRowsPerSlice;
        m_param->lookaheadSlices = m_numCoopSlices;                     // report actual final slice count
    }
    else
    {
        m_numRowsPerSlice = m_8x8Height;
        m_numCoopSlices = 1;
    }
    if (param->gopLookahead && (param->gopLookahead > (param->lookaheadDepth - param->bframes - 2)))
    {
        param->gopLookahead = X265_MAX(0, param->lookaheadDepth - param->bframes - 2);
        x265_log(param, X265_LOG_WARNING, "Gop-lookahead cannot be greater than (rc-lookahead - length of the mini-gop); Clipping gop-lookahead to %d\n", param->gopLookahead);
    }
#if DETAILED_CU_STATS
    m_slicetypeDecideElapsedTime = 0;
    m_preLookaheadElapsedTime = 0;
    m_countSlicetypeDecide = 0;
    m_countPreLookahead = 0;
#endif

    memset(m_histogram, 0, sizeof(m_histogram));
}

#if DETAILED_CU_STATS
void Lookahead::getWorkerStats(int64_t& batchElapsedTime, uint64_t& batchCount, int64_t& coopSliceElapsedTime, uint64_t& coopSliceCount)
{
    batchElapsedTime = coopSliceElapsedTime = 0;
    coopSliceCount = batchCount = 0;
    int tldCount = m_pool ? m_pool->m_numWorkers : 1;
    for (int i = 0; i < tldCount; i++)
    {
        batchElapsedTime += m_tld[i].batchElapsedTime;
        coopSliceElapsedTime += m_tld[i].coopSliceElapsedTime;
        batchCount += m_tld[i].countBatches;
        coopSliceCount += m_tld[i].countCoopSlices;
    }
}
#endif

bool Lookahead::create()
{
    int numTLD = 1 + (m_pool ? m_pool->m_numWorkers : 0);
    m_tld = new LookaheadTLD[numTLD];
    for (int i = 0; i < numTLD; i++)
        m_tld[i].init(m_8x8Width, m_8x8Height, m_8x8Blocks);
    m_scratch = X265_MALLOC(int, m_tld[0].widthInCU);

    return m_tld && m_scratch;
}

/** 函数功能             ： 关闭帧类型决策任务，如果有任务在执行，则等待完毕，再停止任务
/*  调用范围             ： 只在主线程Encoder::stopJobs()中应用
* \返回                  ： null* */
void Lookahead::stopJobs()
{
    if (m_pool && !m_inputQueue.empty())//如果输入列表中还有帧类型未决策，则当前还不该退出
    {
        m_inputLock.acquire();
        m_isActive = false;
        bool wait = m_outputSignalRequired = m_sliceTypeBusy;
        m_inputLock.release();

        if (wait)
            m_outputSignal.wait();
    }
    if (m_pool && m_param->lookaheadThreads > 0)
    {
        for (int i = 0; i < m_numPools; i++)
            m_pool[i].stopWorkers();
    }
}
void Lookahead::destroy()
{
    // these two queues will be empty unless the encode was aborted
    while (!m_inputQueue.empty())
    {
        Frame* curFrame = m_inputQueue.popFront();
        curFrame->destroy();
        delete curFrame;
    }

    while (!m_outputQueue.empty())
    {
        Frame* curFrame = m_outputQueue.popFront();
        curFrame->destroy();
        delete curFrame;
    }

    X265_FREE(m_scratch);
    delete [] m_tld;
    if (m_param->lookaheadThreads > 0)
        delete [] m_pool;
}
/* The synchronization of slicetypeDecide is managed here.  The findJob() method
 * polls the occupancy of the input queue. If the queue is
 * full, it will run slicetypeDecide() and output a mini-gop of frames to the
 * output queue. If the flush() method has been called (implying no new pictures
 * will be received) then the input queue is considered full if it has even one
 * picture left. getDecidedPicture() removes pictures from the output queue and
 * only blocks as a last resort. It does not start removing pictures until
 * m_filled is true, which occurs after *more than* the lookahead depth of
 * pictures have been input so slicetypeDecide() should have started prior to
 * output pictures being withdrawn. The first slicetypeDecide() will obviously
 * still require a blocking wait, but after this slicetypeDecide() will maintain
 * its lead over the encoder (because one picture is added to the input queue
 * each time one is removed from the output) and decides slice types of pictures
 * just ahead of when the encoder needs them */

/* Called by API thread */
/** 函数功能             ： 向输入列表中添加原始帧准备帧类型决策，在buffer满时，触发帧类型决策
/*  调用范围             ： 只在主线程Encoder中应用
* \参数 curFrame         ： 传入的原始帧
* \参数 sliceType        ： 1pass 中为AUTO 2pass中为具体的帧类型
* \返回                  ： null * */
void Lookahead::addPicture(Frame& curFrame, int sliceType)
{
    if (m_param->analysisLoad && m_param->bDisableLookahead)
    {
        if (!m_filled)
            m_filled = true;
        m_outputLock.acquire();
        m_outputQueue.pushBack(curFrame);
        m_outputLock.release();
        m_inputCount++;
    }
    else
    {
        checkLookaheadQueue(m_inputCount);//触发开始进行帧类型决策，仅在此处调用
        curFrame.m_lowres.sliceType = sliceType;
        addPicture(curFrame);
    }
    //printf("addPicture: (m_inputCount.size, m_inputCount) = (%d, %d)\n", m_inputQueue.size(), m_inputCount);
}

void Lookahead::addPicture(Frame& curFrame)
{
    m_inputLock.acquire();
    m_inputQueue.pushBack(curFrame);
    m_inputLock.release();
    m_inputCount++;
}

void Lookahead::checkLookaheadQueue(int &frameCnt)
{
    /* determine if the lookahead is (over) filled enough for frames to begin to
     * be consumed by frame encoders */
    if (!m_filled)
    {
        if (!m_param->bframes & !m_param->lookaheadDepth)
            m_filled = true; /* zero-latency */
        else if (frameCnt >= m_param->lookaheadDepth + 2 + m_param->bframes)
            m_filled = true; /* full capacity plus mini-gop lag */
    }

    m_inputLock.acquire();
    if (m_pool && m_inputQueue.size() >= m_fullQueueSize)//如果输入列表的帧数已经大于搜索的最大帧数
        tryWakeOne();//触发开始进行帧类型决策
    m_inputLock.release();
}

/* Called by API thread */
/** 函数功能             ： 当前已经读取原始帧完毕，往后不用再继续读取，告知lookahead已满
/*  调用范围             ： 只在主线程Encoder::encode中应用
* \返回                  ： null * */
void Lookahead::flush()
{
    /* force slicetypeDecide to run until the input queue is empty */
    m_fullQueueSize = 1;
    m_filled = true;
}

void Lookahead::setLookaheadQueue()
{
    m_filled = false;
    m_fullQueueSize = X265_MAX(1, m_param->lookaheadDepth);
}

/** 函数功能             ： 触发帧类型决策（在threadMain()主动发起，在getDecidedPicture()被动发起，因为当前发现帧类型不可用，被动发起）
/*  调用范围             ： 只在WorkerThread::threadMain()（在addPicture中触发执行）和Lookahead::getDecidedPicture()函数中被调用
* \返回                  ： null * */
void Lookahead::findJob(int /*workerThreadID*/)
{
    bool doDecide;//用于指示是否进行帧类型决策

    m_inputLock.acquire();
    if (m_inputQueue.size() >= m_fullQueueSize && !m_sliceTypeBusy && m_isActive)//如果当前输入列表的帧数大于等于lookachead最大深度&&当前不再进行sliceDecide&&当前lookachead是触发状态
        doDecide = m_sliceTypeBusy = true;//满足条件将其置为true，准备帧类型决策 （其中m_sliceTypeBusy保证系统只有一个线程进行帧类型决策）
    else
        doDecide = m_helpWanted = false;//不满足条件，将其置为false 准备退出
    m_inputLock.release();

    if (!doDecide)
        return;

    ProfileLookaheadTime(m_slicetypeDecideElapsedTime, m_countSlicetypeDecide);
    ProfileScopeEvent(slicetypeDecideEV);

    slicetypeDecide();//核心函数：获取帧类型并计算frame-cost

    m_inputLock.acquire();

    //与event m_outputSignal;配套使用，初始化为false  true表示需要完成sliceDecide，在Lookahead::findJob中完成sliceDecide置为false并触发m_outputSignal，说明完成sliceDecide
    //在Lookahead::getDecidedPicture()函数中会进行检测并阻塞
    if (m_outputSignalRequired)
    {
        m_outputSignal.trigger();//触发帧类型决策完毕，使其不再等待，继续编码
        m_outputSignalRequired = false;//帧类型决策完毕，置为false
    }
    m_sliceTypeBusy = false;//帧类型决策完毕，不再忙
    m_inputLock.release();
}

/* Called by API thread */

/** 函数功能             ： 只在主线程Encoder中应用，获取已经得到帧类型的原始帧
/*  调用范围             ： 只在Encoder::encode函数中被调用
* \返回                  ： 返回当前帧类型决策完毕的待编码帧 * */
Frame* Lookahead::getDecidedPicture()
{
    if (m_filled)
    {
        m_outputLock.acquire();
        Frame *out = m_outputQueue.popFront();
        m_outputLock.release();

        if (out)//有可用帧，直接返回
        {
            m_inputCount--;
            return out;
        }

        if (m_param->analysisLoad && m_param->bDisableLookahead)
            return NULL;

        //没有可用帧，准备findjob调用slicetype并得到可用帧，作为补充，如果获取编码帧快于slicetype的情况，这种情况较少，但是也会进入
        findJob(-1); /* run slicetypeDecide() if necessary */

        m_inputLock.acquire();
        bool wait = m_outputSignalRequired = m_sliceTypeBusy;//如果当前正在进行slicetype 需要等待
        m_inputLock.release();

        if (wait)
            m_outputSignal.wait();//一直等待直到帧类型决策完毕

        out = m_outputQueue.popFront();
        if (out)
            m_inputCount--;
        return out;//抛出可用帧
    }
    else
        return NULL;
}

/* Called by rate-control to calculate the estimated SATD cost for a given
 * picture.  It assumes dpb->prepareEncode() has already been called for the
 * picture and all the references are established */

 /** 函数功能             ： 获取当前帧每个CTU行对应下采样帧的每个8x8的块cost的累计值
 /*  调用范围             ： 只在Encoder::encode函数中被调用
 * \参数 curFrame         ： 传入的原始帧
 * \返回                  ： null * */
void Lookahead::getEstimatedPictureCost(Frame *curFrame)
{
    Lowres *frames[X265_LOOKAHEAD_MAX];//存储相关的下采样图像  存储方式：前向帧  当前帧  后向帧

    // POC distances to each reference
    Slice *slice = curFrame->m_encData->m_slice;//获取当前slice
    int p0 = 0, p1, b;//b：当前帧   P0前向帧  p1后向帧
    int poc = slice->m_poc;
    int l0poc = slice->m_rps.numberOfNegativePictures ? slice->m_refPOCList[0][0] : -1;//获取list0的第一帧的poc
    int l1poc = slice->m_refPOCList[1][0];//获取list1的第一帧的poc

    switch (slice->m_sliceType)
    {
    case I_SLICE:
        frames[p0] = &curFrame->m_lowres;
        b = p1 = 0;
        break;

    case P_SLICE:
        b = p1 = poc - l0poc;//获取当前帧应该存放的位置
        frames[p0] = &slice->m_refFrameList[0][0]->m_lowres;
        frames[b] = &curFrame->m_lowres;
        break;

    case B_SLICE:
        if (l0poc >= 0)
        {
            b = poc - l0poc;//获取当前帧应该存放的位置
            p1 = b + l1poc - poc;//获取后向帧应该存放的位置
            frames[p0] = &slice->m_refFrameList[0][0]->m_lowres;
            frames[b] = &curFrame->m_lowres;
            frames[p1] = &slice->m_refFrameList[1][0]->m_lowres;
        }
        else 
        {
            p0 = b = 0;
            p1 = b + l1poc - poc;
            frames[p0] = frames[b] = &curFrame->m_lowres;
            frames[p1] = &slice->m_refFrameList[1][0]->m_lowres;
        }
        
        break;

    default:
        return;
    }
    if (!m_param->analysisLoad || !m_param->bDisableLookahead)
    {
        X265_CHECK(curFrame->m_lowres.costEst[b - p0][p1 - b] > 0, "Slice cost not estimated\n")
        if (m_param->rc.cuTree && !m_param->rc.bStatRead)//如果应用cutree 并且不是1pass(2pass以上直接获取)
            /* update row satds based on cutree offsets */
            curFrame->m_lowres.satdCost = frameCostRecalculate(frames, p0, p1, b);//如果当前是B帧直接返回其framecost：costEstAq （qpAqOffset加权）否则，重新计算framecost  经过qpCuTreeOffset加权后的数据
        else if (!m_param->analysisLoad || m_param->scaleFactor)
        {
            if (m_param->rc.aqMode)//如果应用自适应量化 获取自适应量化的cost
                curFrame->m_lowres.satdCost = curFrame->m_lowres.costEstAq[b - p0][p1 - b];
            else//获取framecost
                curFrame->m_lowres.satdCost = curFrame->m_lowres.costEst[b - p0][p1 - b];
        }
        if (m_param->rc.vbvBufferSize && m_param->rc.vbvMaxBitrate)//如果应用VBV
        {
            /* aggregate lowres row satds to CTU resolution */
            curFrame->m_lowres.lowresCostForRc = curFrame->m_lowres.lowresCosts[b - p0][p1 - b];//获取对应帧计算framecost时的每个8x8块值
            // lowresRow 遍历下采样的行号, lowresCol 用于遍历每行8x8块的每个8x8块， lowresCuIdx当前CTU行对应下采样8x8行位置的 8x8块的index, sum intraSum 用于累加计算当前8x8行的cost
            uint32_t lowresRow = 0, lowresCol = 0, lowresCuIdx = 0, sum = 0, intraSum = 0; 
            uint32_t scale = m_param->maxCUSize / (2 * X265_LOWRES_CU_SIZE);//因为是下采样图像，底层的8x8块相当于原始帧的16x16 这是CTU与8x8的缩放关系 64/16
            uint32_t numCuInHeight = (m_param->sourceHeight + m_param->maxCUSize - 1) / m_param->maxCUSize;//获取有多少CTU行
            uint32_t widthInLowresCu = (uint32_t)m_8x8Width, heightInLowresCu = (uint32_t)m_8x8Height;//分别获取有多少8x8行和8x8列
            double *qp_offset = 0;//自适应量化的权重系数
            /* Factor in qpoffsets based on Aq/Cutree in CU costs */
            if (m_param->rc.aqMode || m_param->bAQMotion)//如果应用自适应量化，根据情况选择
                qp_offset = (frames[b]->sliceType == X265_TYPE_B || !m_param->rc.cuTree) ? frames[b]->qpAqOffset : frames[b]->qpCuTreeOffset;

            for (uint32_t row = 0; row < numCuInHeight; row++)//遍历CTU行
            {
                lowresRow = row * scale;//对应下采样图像的行号
                for (uint32_t cnt = 0; cnt < scale && lowresRow < heightInLowresCu; lowresRow++, cnt++)//遍历一个CTU行对应的scale个的下采样8x8行
                {
                    sum = 0; intraSum = 0;//用于累加计算当前8x8行的cost
                    int diff = 0;
                    lowresCuIdx = lowresRow * widthInLowresCu;//获取当前CTU行对应下采样8x8行位置的 8x8块的index
                    for (lowresCol = 0; lowresCol < widthInLowresCu; lowresCol++, lowresCuIdx++)//遍历每行每个8x8块的cost
                    {
                        // 获取当前块的8x8块cost  与操作原因：高14位的数字：0 表示 intra  1 表示前向搜索  2表示后向搜索  3 表示bi搜索
                        uint16_t lowresCuCost = curFrame->m_lowres.lowresCostForRc[lowresCuIdx] & LOWRES_COST_MASK;
                        if (qp_offset)//如果应用自适应量化
                        {
                            double qpOffset;
                            if (m_param->rc.qgSize == 8)//取均值
                                qpOffset = (qp_offset[lowresCol * 2 + lowresRow * widthInLowresCu * 4] +
                                qp_offset[lowresCol * 2 + lowresRow * widthInLowresCu * 4 + 1] +
                                qp_offset[lowresCol * 2 + lowresRow * widthInLowresCu * 4 + curFrame->m_lowres.maxBlocksInRowFullRes] +
                                qp_offset[lowresCol * 2 + lowresRow * widthInLowresCu * 4 + curFrame->m_lowres.maxBlocksInRowFullRes + 1]) / 4;
                            else
                                qpOffset = qp_offset[lowresCuIdx];
                            lowresCuCost = (uint16_t)((lowresCuCost * x265_exp2fix8(qpOffset) + 128) >> 8);//重新计算cost，乘以权重系数
                            int32_t intraCuCost = curFrame->m_lowres.intraCost[lowresCuIdx];//获取intracost
                            curFrame->m_lowres.intraCost[lowresCuIdx] = (intraCuCost * x265_exp2fix8(qpOffset) + 128) >> 8;//重新计算cost，乘以权重系数
                        }
                        if (m_param->bIntraRefresh && slice->m_sliceType == X265_TYPE_P)
                            for (uint32_t x = curFrame->m_encData->m_pir.pirStartCol; x <= curFrame->m_encData->m_pir.pirEndCol; x++)
                                diff += curFrame->m_lowres.intraCost[lowresCuIdx] - lowresCuCost;
                        curFrame->m_lowres.lowresCostForRc[lowresCuIdx] = lowresCuCost;//重新获得cost
                        sum += lowresCuCost;//累加cost
                        intraSum += curFrame->m_lowres.intraCost[lowresCuIdx];//累加intracost
                    }
                    curFrame->m_encData->m_rowStat[row].satdForVbv += sum;//每个CTU行对应所有8x8块的cost累加值
                    curFrame->m_encData->m_rowStat[row].satdForVbv += diff;
                    curFrame->m_encData->m_rowStat[row].intraSatdForVbv += intraSum;//每个CTU行对应所有8x8块的intracost累加值
                }
            }
        }
    }
}


/** 函数功能             ： 初始化lowres并进行下采样、扩边、计算qpCuTreeOffset等信息，获取整帧的像素和和AC能量、计算当前1/2下采样帧的intra SATD值以及最优intra模式、并行处理
/*  调用范围             ： 只在WorkerThread::threadMain()和sliceTypeDecide函数中被调用
* \参数 workerThreadID   ： 当前运行的内核号
* \返回                  ： null * */
void PreLookaheadGroup::processTasks(int workerThreadID)
{
    if (workerThreadID < 0)
        workerThreadID = m_lookahead.m_pool ? m_lookahead.m_pool->m_numWorkers : 0;
    LookaheadTLD& tld = m_lookahead.m_tld[workerThreadID];

    m_lock.acquire();
    while (m_jobAcquired < m_jobTotal)
    {
        Frame* preFrame = m_preframes[m_jobAcquired++];
        ProfileLookaheadTime(m_lookahead.m_preLookaheadElapsedTime, m_lookahead.m_countPreLookahead);
        ProfileScopeEvent(prelookahead);
        m_lock.release();
        preFrame->m_lowres.init(preFrame->m_fencPic, preFrame->m_poc); //生成低分辨率图像
        if (m_lookahead.m_bAdaptiveQuant)
            tld.calcAdaptiveQuantFrame(preFrame, m_lookahead.m_param);
        tld.lowresIntraEstimate(preFrame->m_lowres, m_lookahead.m_param->rc.qgSize);
        preFrame->m_lowresInit = true;

        m_lock.acquire();
    }
    m_lock.release();
}

/* called by API thread or worker thread with inputQueueLock acquired */
/** 函数功能             ： 获取帧类型并计算frame-cost
/*  调用范围             ： 只在Lookahead::findJob函数中被调用
* \返回                  ： null * */
void Lookahead::slicetypeDecide()
{
    PreLookaheadGroup pre(*this);//用于初始化lowres的类

    Lowres* frames[X265_LOOKAHEAD_MAX + X265_BFRAME_MAX + 4];
    Frame*  list[X265_BFRAME_MAX + 4];
    memset(frames, 0, sizeof(frames));
    memset(list, 0, sizeof(list));
    int maxSearch = X265_MIN(m_param->lookaheadDepth, X265_LOOKAHEAD_MAX);
    maxSearch = X265_MAX(1, maxSearch);

    {
        ScopedLock lock(m_inputLock);

        Frame *curFrame = m_inputQueue.first();
        int j;
        for (j = 0; j < m_param->bframes + 2; j++)
        {
            if (!curFrame) break;
            list[j] = curFrame;
            curFrame = curFrame->m_next;
        }

        curFrame = m_inputQueue.first();
        frames[0] = m_lastNonB;
        for (j = 0; j < maxSearch; j++)
        {
            if (!curFrame) break;
            frames[j + 1] = &curFrame->m_lowres;

            if (!curFrame->m_lowresInit)
                pre.m_preframes[pre.m_jobTotal++] = curFrame;

            curFrame = curFrame->m_next;
        }

        maxSearch = j;
    }

    /* perform pre-analysis on frames which need it, using a bonded task group */
    //pre.processTasks(-1) 所有数据都在pre操作，无论在threadmain调用还是下面的procesTask都是操作的该对象,多线程并行处理帧初始化
    //初始化lowres并进行下采样、扩边、计算pCuTreeOffset等信息，获取整帧的像素和和AC能量、计算当前1/2下采样帧的intra SATD值以及最优intra模式、并行处理
    if (pre.m_jobTotal)
    {
        if (m_pool)
            pre.tryBondPeers(*m_pool, pre.m_jobTotal);
        pre.processTasks(-1);//使用PreLookaheadGroup完成初始化工作
        pre.waitForExit();
    }

    if (m_lastNonB && !m_param->rc.bStatRead &&
        ((m_param->bFrameAdaptive && m_param->bframes) ||
         m_param->rc.cuTree || m_param->scenecutThreshold ||
         (m_param->lookaheadDepth && m_param->rc.vbvBufferSize)))  //在1pass 并且m_lastNonB不为空会进入； 2pass等情况不会进入
    {
        slicetypeAnalyse(frames, false);//核心函数：确定帧类型（在使用自适应B帧数目或场景检测时，帧类型不是固定的结构）
        bool bIsVbv = m_param->rc.vbvBufferSize > 0 && m_param->rc.vbvMaxBitrate > 0;
        if (m_param->analysisLoad && m_param->scaleFactor && bIsVbv)
        {
            int numFrames;
            for (numFrames = 0; numFrames < maxSearch; numFrames++)
            {
                Lowres *fenc = frames[numFrames + 1];
                if (!fenc)
                    break;
            }
            vbvLookahead(frames, numFrames, true);
        }
    }

    int bframes, brefs;
    //循环功能：根据配置情况，修正当前GOP的帧类型，如IDR帧插入等情况
    for (bframes = 0, brefs = 0;; bframes++)//在1pass中进来的sliceType初始都为X265_TYPE_AUTO，2pass中都会有具体的帧类型
    {
        Lowres& frm = list[bframes]->m_lowres;

        if (frm.sliceType == X265_TYPE_BREF && !m_param->bBPyramid && brefs == m_param->bBPyramid)
        {
            frm.sliceType = X265_TYPE_B;
            x265_log(m_param, X265_LOG_WARNING, "B-ref at frame %d incompatible with B-pyramid\n",
                     frm.frameNum);
        }

        /* pyramid with multiple B-refs needs a big enough dpb that the preceding P-frame stays available.
         * smaller dpb could be supported by smart enough use of mmco, but it's easier just to forbid it. */
        else if (frm.sliceType == X265_TYPE_BREF && m_param->bBPyramid && brefs &&
                 m_param->maxNumReferences <= (brefs + 3))
        {
            frm.sliceType = X265_TYPE_B;
            x265_log(m_param, X265_LOG_WARNING, "B-ref at frame %d incompatible with B-pyramid and %d reference frames\n",
                     frm.sliceType, m_param->maxNumReferences);
        }
        if ((!m_param->bIntraRefresh || frm.frameNum == 0) && frm.frameNum - m_lastKeyframe >= m_param->keyframeMax &&
            (!m_extendGopBoundary || frm.frameNum - m_lastKeyframe >= m_param->keyframeMax + m_param->gopLookahead))
        {
            if (frm.sliceType == X265_TYPE_AUTO || frm.sliceType == X265_TYPE_I)
                frm.sliceType = m_param->bOpenGOP && m_lastKeyframe >= 0 ? X265_TYPE_I : X265_TYPE_IDR;
            bool warn = frm.sliceType != X265_TYPE_IDR;
            if (warn && m_param->bOpenGOP)
                warn &= frm.sliceType != X265_TYPE_I;
            if (warn)
            {
                x265_log(m_param, X265_LOG_WARNING, "specified frame type (%d) at %d is not compatible with keyframe interval\n",
                         frm.sliceType, frm.frameNum);
                frm.sliceType = m_param->bOpenGOP && m_lastKeyframe >= 0 ? X265_TYPE_I : X265_TYPE_IDR;
            }
        }
        if (frm.sliceType == X265_TYPE_I && frm.frameNum - m_lastKeyframe >= m_param->keyframeMin)
        {
            if (m_param->bOpenGOP)
            {
                m_lastKeyframe = frm.frameNum;
                frm.bKeyframe = true;
            }
            else
                frm.sliceType = X265_TYPE_IDR;
        }
        if (frm.sliceType == X265_TYPE_IDR)
        {
            /* Closed GOP */
            m_lastKeyframe = frm.frameNum;
            frm.bKeyframe = true;
            if (bframes > 0 && !m_param->radl)
            {
                list[bframes - 1]->m_lowres.sliceType = X265_TYPE_P;
                bframes--;
            }
        }
        if (m_param->radl && !m_param->bOpenGOP && list[bframes + 1])
        {
            if ((frm.frameNum - m_lastKeyframe) >  (m_param->keyframeMax - m_param->radl - 1) && (frm.frameNum - m_lastKeyframe) <  m_param->keyframeMax)
                frm.sliceType = X265_TYPE_B;
            if ((frm.frameNum - m_lastKeyframe) == (m_param->keyframeMax - m_param->radl - 1))
                frm.sliceType = X265_TYPE_P;
        }

        if (bframes == m_param->bframes || !list[bframes + 1])
        {
            if (IS_X265_TYPE_B(frm.sliceType))
                x265_log(m_param, X265_LOG_WARNING, "specified frame type is not compatible with max B-frames\n");
            if (frm.sliceType == X265_TYPE_AUTO || IS_X265_TYPE_B(frm.sliceType))
                frm.sliceType = X265_TYPE_P;
        }
        if (frm.sliceType == X265_TYPE_BREF)
            brefs++;
        if (frm.sliceType == X265_TYPE_AUTO)
            frm.sliceType = X265_TYPE_B;
        else if (!IS_X265_TYPE_B(frm.sliceType))
            break;
    }

    if (bframes)
        list[bframes - 1]->m_lowres.bLastMiniGopBFrame = true;
    list[bframes]->m_lowres.leadingBframes = bframes;
    m_lastNonB = &list[bframes]->m_lowres;
    m_histogram[bframes]++;

    /* insert a bref into the sequence */
    if (m_param->bBPyramid && bframes > 1 && !brefs)
    {
        list[bframes / 2]->m_lowres.sliceType = X265_TYPE_BREF;
        brefs++;
    }
    /* calculate the frame costs ahead of time for estimateFrameCost while we still have lowres */
    if (m_param->rc.rateControlMode != X265_RC_CQP)
    {
        int p0, p1, b;
        /* For zero latency tuning, calculate frame cost to be used later in RC */
        if (!maxSearch)
        {
            for (int i = 0; i <= bframes; i++)
               frames[i + 1] = &list[i]->m_lowres;
        }

        /* estimate new non-B cost */
        p1 = b = bframes + 1;
        p0 = (IS_X265_TYPE_I(frames[bframes + 1]->sliceType)) ? b : 0;

        CostEstimateGroup estGroup(*this, frames);

        estGroup.singleCost(p0, p1, b);

        if (bframes)
        {
            p0 = 0; // last nonb
            bool isp0available = frames[bframes + 1]->sliceType == X265_TYPE_IDR ? false : true;

            for (b = 1; b <= bframes; b++)
            {
                if (!isp0available)
                    p0 = b;

                if (frames[b]->sliceType == X265_TYPE_B)
                    for (p1 = b; frames[p1]->sliceType == X265_TYPE_B; p1++)
                        ; // find new nonb or bref
                else
                    p1 = bframes + 1;

                estGroup.singleCost(p0, p1, b);

                if (frames[b]->sliceType == X265_TYPE_BREF)
                {
                    p0 = b;
                    isp0available = true;
                }
            }
        }
    }

    m_inputLock.acquire();
    /* dequeue all frames from inputQueue that are about to be enqueued
     * in the output queue. The order is important because Frame can
     * only be in one list at a time */
    int64_t pts[X265_BFRAME_MAX + 1];
    for (int i = 0; i <= bframes; i++)
    {
        Frame *curFrame;
        curFrame = m_inputQueue.popFront();
        pts[i] = curFrame->m_pts;
        maxSearch--;
    }
    m_inputLock.release();

    m_outputLock.acquire();
    /* add non-B to output queue */
    int idx = 0;
    list[bframes]->m_reorderedPts = pts[idx++];
    m_outputQueue.pushBack(*list[bframes]);
    /* Add B-ref frame next to P frame in output queue, the B-ref encode before non B-ref frame */
    if (brefs)
    {
        for (int i = 0; i < bframes; i++)
        {
            if (list[i]->m_lowres.sliceType == X265_TYPE_BREF)
            {
                list[i]->m_reorderedPts = pts[idx++];
                m_outputQueue.pushBack(*list[i]);
            }
        }
    }

    /* add B frames to output queue */
    for (int i = 0; i < bframes; i++)
    {
        /* push all the B frames into output queue except B-ref, which already pushed into output queue */
        if (list[i]->m_lowres.sliceType != X265_TYPE_BREF)
        {
            list[i]->m_reorderedPts = pts[idx++];
            m_outputQueue.pushBack(*list[i]);
        }
    }

    bool isKeyFrameAnalyse = (m_param->rc.cuTree || (m_param->rc.vbvBufferSize && m_param->lookaheadDepth)) && !m_param->rc.bStatRead;
    if (isKeyFrameAnalyse && IS_X265_TYPE_I(m_lastNonB->sliceType))
    {
        m_inputLock.acquire();
        Frame *curFrame = m_inputQueue.first();
        frames[0] = m_lastNonB;
        int j;
        for (j = 0; j < maxSearch; j++)
        {
            frames[j + 1] = &curFrame->m_lowres;
            curFrame = curFrame->m_next;
        }
        m_inputLock.release();

        frames[j + 1] = NULL;
        slicetypeAnalyse(frames, true);
        bool bIsVbv = m_param->rc.vbvBufferSize > 0 && m_param->rc.vbvMaxBitrate > 0;
        if (m_param->analysisLoad && m_param->scaleFactor && bIsVbv)
        {
            int numFrames;
            for (numFrames = 0; numFrames < maxSearch; numFrames++)
            {
                Lowres *fenc = frames[numFrames + 1];
                if (!fenc)
                    break;
            }
            vbvLookahead(frames, numFrames, true);
        }
    }
    m_outputLock.release();
}

/** 函数功能             ： 重新计算framecost经过qpCuTreeOffset加权后的数据并吧相应信息存储到第一个GOP中，并将相应的B帧设置为参考B帧
/*  调用范围             ： 只在slicetypeAnalyse函数中被调用
* \参数 frames           ： 当前搜索的frames列表
* \参数 numframes        ： frames列表帧数
* \参数 keyframe         ： 是否是IDR帧检测
* \返回                  ： null * */
void Lookahead::vbvLookahead(Lowres **frames, int numFrames, int keyframe)
{
    int prevNonB = 0, curNonB = 1, idx = 0;
    while (curNonB < numFrames && frames[curNonB]->sliceType == X265_TYPE_B)
        curNonB++;
    int nextNonB = keyframe ? prevNonB : curNonB;
    int nextB = prevNonB + 1;
    int nextBRef = 0, curBRef = 0;
    if (m_param->bBPyramid && curNonB - prevNonB > 1)
        curBRef = (prevNonB + curNonB + 1) / 2;
    int miniGopEnd = keyframe ? prevNonB : curNonB;
    while (curNonB < numFrames + !keyframe)
    {
        /* P/I cost: This shouldn't include the cost of nextNonB */
        if (nextNonB != curNonB)
        {
            int p0 = IS_X265_TYPE_I(frames[curNonB]->sliceType) ? curNonB : prevNonB;
            frames[nextNonB]->plannedSatd[idx] = vbvFrameCost(frames, p0, curNonB, curNonB);
            frames[nextNonB]->plannedType[idx] = frames[curNonB]->sliceType;

            /* Save the nextNonB Cost in each B frame of the current miniGop */
            if (curNonB > miniGopEnd)
            {
                for (int j = nextB; j < miniGopEnd; j++)
                {
                    frames[j]->plannedSatd[frames[j]->indB] = frames[nextNonB]->plannedSatd[idx];
                    frames[j]->plannedType[frames[j]->indB++] = frames[nextNonB]->plannedType[idx];
                }
            }
            idx++;
        }

        /* Handle the B-frames: coded order */
        if (m_param->bBPyramid && curNonB - prevNonB > 1)
            nextBRef = (prevNonB + curNonB + 1) / 2;

        for (int i = prevNonB + 1; i < curNonB; i++, idx++)
        {
            int64_t satdCost = 0;
            int type = X265_TYPE_B;
            if (nextBRef)
            {
                if (i == nextBRef)
                {
                    satdCost = vbvFrameCost(frames, prevNonB, curNonB, nextBRef);
                    type = X265_TYPE_BREF;
                }
                else if (i < nextBRef)
                    satdCost = vbvFrameCost(frames, prevNonB, nextBRef, i);
                else
                    satdCost = vbvFrameCost(frames, nextBRef, curNonB, i);
            }
            else
                satdCost = vbvFrameCost(frames, prevNonB, curNonB, i);
            frames[nextNonB]->plannedSatd[idx] = satdCost;
            frames[nextNonB]->plannedType[idx] = type;
            /* Save the nextB Cost in each B frame of the current miniGop */

            for (int j = nextB; j < miniGopEnd; j++)
            {
                if (curBRef && curBRef == i)
                    break;
                if (j >= i && j !=nextBRef)
                    continue;
                frames[j]->plannedSatd[frames[j]->indB] = satdCost;
                frames[j]->plannedType[frames[j]->indB++] = type;
            }
        }
        prevNonB = curNonB;
        curNonB++;
        while (curNonB <= numFrames && frames[curNonB]->sliceType == X265_TYPE_B)
            curNonB++;
    }

    frames[nextNonB]->plannedType[idx] = X265_TYPE_AUTO;
}


/** 函数功能             ： 如果当前帧号b是B帧直接返回其framecost：costEstAq （qpAqOffset加权）否则，重新计算framecost  经过qpCuTreeOffset加权后的数据
/*  调用范围             ： 只在Lookahead::vbvLookahead函数中被调用
* \参数 frames           ： 当前搜索的frames列表
* \参数 p0               ： 前向帧
* \参数 p1               ： 后向帧
* \参数 b                ： 当前帧号
* \返回                  ： 返回重新计算的framecost* */
int64_t Lookahead::vbvFrameCost(Lowres **frames, int p0, int p1, int b)
{
    CostEstimateGroup estGroup(*this, frames);
    int64_t cost = estGroup.singleCost(p0, p1, b);

    if (m_param->rc.aqMode || m_param->bAQMotion)
    {
        if (m_param->rc.cuTree)
            return frameCostRecalculate(frames, p0, p1, b);
        else
            return frames[b]->costEstAq[b - p0][p1 - b];
    }

    return cost;
}


/** 函数功能             ： 获取当前GOP的帧类型，获取可参考帧的qpCuTreeOffset值
/*  调用范围             ： 只在sliceTypeDecide函数中被调用 （一般只在1pass中进入）
* \参数 frames           ： 当前搜索的frames列表
* \参数 bKeyframe        ： 是否是IDR帧检测
* \返回                  ： null * */
void Lookahead::slicetypeAnalyse(Lowres **frames, bool bKeyframe)
{
    int numFrames, origNumFrames, keyintLimit, framecnt;
    int maxSearch = X265_MIN(m_param->lookaheadDepth, X265_LOOKAHEAD_MAX);
    int cuCount = m_8x8Blocks;
    int resetStart;
    bool bIsVbvLookahead = m_param->rc.vbvBufferSize && m_param->lookaheadDepth;

    /* count undecided frames */
    for (framecnt = 0; framecnt < maxSearch; framecnt++)
    {
        Lowres *fenc = frames[framecnt + 1];
        if (!fenc || fenc->sliceType != X265_TYPE_AUTO)
            break;
    }

    if (!framecnt)
    {
        if (m_param->rc.cuTree)
            cuTree(frames, 0, bKeyframe);
        return;
    }
    frames[framecnt + 1] = NULL;
    int keyFrameLimit = m_param->keyframeMax + m_lastKeyframe - frames[0]->frameNum - 1;
    if (m_param->gopLookahead && keyFrameLimit <= m_param->bframes + 1)
        keyintLimit = keyFrameLimit + m_param->gopLookahead;
    else
        keyintLimit = keyFrameLimit;

    origNumFrames = numFrames = m_param->bIntraRefresh ? framecnt : X265_MIN(framecnt, keyintLimit);
    if (bIsVbvLookahead)
        numFrames = framecnt;
    else if (m_param->bOpenGOP && numFrames < framecnt)
        numFrames++;
    else if (numFrames == 0)
    {
        frames[1]->sliceType = X265_TYPE_I;
        return;
    }

    if (m_bBatchMotionSearch)
    {
        /* pre-calculate all motion searches, using many worker threads */
        CostEstimateGroup estGroup(*this, frames);
        for (int b = 2; b < numFrames; b++)
        {
            for (int i = 1; i <= m_param->bframes + 1; i++)
            {
                int p0 = b - i;
                if (p0 < 0)
                    continue;

                /* Skip search if already done */
                if (frames[b]->lowresMvs[0][i][0].x != 0x7FFF)
                    continue;

                /* perform search to p1 at same distance, if possible */
                int p1 = b + i;
                if (p1 >= numFrames || frames[b]->lowresMvs[1][i][0].x != 0x7FFF)
                    p1 = b;

                estGroup.add(p0, p1, b);
            }
        }
        /* auto-disable after the first batch if pool is small */
        m_bBatchMotionSearch &= m_pool->m_numWorkers >= 4;
        estGroup.finishBatch();

        if (m_bBatchFrameCosts)
        {
            /* pre-calculate all frame cost estimates, using many worker threads */
            for (int b = 2; b < numFrames; b++)
            {
                for (int i = 1; i <= m_param->bframes + 1; i++)
                {
                    if (b < i)
                        continue;

                    /* only measure frame cost in this pass if motion searches
                     * are already done */
                    if (frames[b]->lowresMvs[0][i][0].x == 0x7FFF)
                        continue;

                    int p0 = b - i;

                    for (int j = 0; j <= m_param->bframes; j++)
                    {
                        int p1 = b + j;
                        if (p1 >= numFrames)
                            break;

                        /* ensure P1 search is done */
                        if (j && frames[b]->lowresMvs[1][j][0].x == 0x7FFF)
                            continue;

                        /* ensure frame cost is not done */
                        if (frames[b]->costEst[i][j] >= 0)
                            continue;

                        estGroup.add(p0, p1, b);
                    }
                }
            }

            /* auto-disable after the first batch if the pool is not large */
            m_bBatchFrameCosts &= m_pool->m_numWorkers > 12;
            estGroup.finishBatch();
        }
    }

    int numBFrames = 0;
    int numAnalyzed = numFrames;
    bool isScenecut = scenecut(frames, 0, 1, true, origNumFrames);
    /* When scenecut threshold is set, use scenecut detection for I frame placements */
    if (m_param->scenecutThreshold && isScenecut)
    {
        frames[1]->sliceType = X265_TYPE_I;
        return;
    }
    if (m_param->gopLookahead && (keyFrameLimit >= 0) && (keyFrameLimit <= m_param->bframes + 1))
    {
        bool sceneTransition = m_isSceneTransition;
        m_extendGopBoundary = false;
        for (int i = m_param->bframes + 1; i < origNumFrames; i += m_param->bframes + 1)
        {
            scenecut(frames, i, i + 1, true, origNumFrames);
            for (int j = i + 1; j <= X265_MIN(i + m_param->bframes + 1, origNumFrames); j++)
            {
                if (frames[j]->bScenecut && scenecutInternal(frames, j - 1, j, true) )
                {
                    m_extendGopBoundary = true;
                    break;
                }
            }
            if (m_extendGopBoundary)
                break;
        }
        m_isSceneTransition = sceneTransition;
    }
    if (m_param->bframes)
    {
        if (m_param->bFrameAdaptive == X265_B_ADAPT_TRELLIS)
        {
            if (numFrames > 1)
            {
                char best_paths[X265_BFRAME_MAX + 1][X265_LOOKAHEAD_MAX + 1] = { "", "P" };
                int best_path_index = numFrames % (X265_BFRAME_MAX + 1);

                /* Perform the frame type analysis. */
                for (int j = 2; j <= numFrames; j++)
                    slicetypePath(frames, j, best_paths);

                numBFrames = (int)strspn(best_paths[best_path_index], "B");

                /* Load the results of the analysis into the frame types. */
                for (int j = 1; j < numFrames; j++)
                    frames[j]->sliceType = best_paths[best_path_index][j - 1] == 'B' ? X265_TYPE_B : X265_TYPE_P;
            }
            frames[numFrames]->sliceType = X265_TYPE_P;
        }
        else if (m_param->bFrameAdaptive == X265_B_ADAPT_FAST)
        {
            CostEstimateGroup estGroup(*this, frames);

            int64_t cost1p0, cost2p0, cost1b1, cost2p1;

            for (int i = 0; i <= numFrames - 2; )
            {
                cost2p1 = estGroup.singleCost(i + 0, i + 2, i + 2, true);
                if (frames[i + 2]->intraMbs[2] > cuCount / 2)
                {
                    frames[i + 1]->sliceType = X265_TYPE_P;
                    frames[i + 2]->sliceType = X265_TYPE_P;
                    i += 2;
                    continue;
                }

                cost1b1 = estGroup.singleCost(i + 0, i + 2, i + 1);
                cost1p0 = estGroup.singleCost(i + 0, i + 1, i + 1);
                cost2p0 = estGroup.singleCost(i + 1, i + 2, i + 2);

                if (cost1p0 + cost2p0 < cost1b1 + cost2p1)
                {
                    frames[i + 1]->sliceType = X265_TYPE_P;
                    i += 1;
                    continue;
                }

// arbitrary and untuned
#define INTER_THRESH 300
#define P_SENS_BIAS (50 - m_param->bFrameBias)
                frames[i + 1]->sliceType = X265_TYPE_B;

                int j;
                for (j = i + 2; j <= X265_MIN(i + m_param->bframes, numFrames - 1); j++)
                {
                    int64_t pthresh = X265_MAX(INTER_THRESH - P_SENS_BIAS * (j - i - 1), INTER_THRESH / 10);
                    int64_t pcost = estGroup.singleCost(i + 0, j + 1, j + 1, true);
                    if (pcost > pthresh * cuCount || frames[j + 1]->intraMbs[j - i + 1] > cuCount / 3)
                        break;
                    frames[j]->sliceType = X265_TYPE_B;
                }

                frames[j]->sliceType = X265_TYPE_P;
                i = j;
            }
            frames[numFrames]->sliceType = X265_TYPE_P;
            numBFrames = 0;
            while (numBFrames < numFrames && frames[numBFrames + 1]->sliceType == X265_TYPE_B)
                numBFrames++;
        }
        else
        {
            numBFrames = X265_MIN(numFrames - 1, m_param->bframes);
            for (int j = 1; j < numFrames; j++)
                frames[j]->sliceType = (j % (numBFrames + 1)) ? X265_TYPE_B : X265_TYPE_P;

            frames[numFrames]->sliceType = X265_TYPE_P;
        }

        /* Check scenecut on the first minigop. */
        for (int j = 1; j < numBFrames + 1; j++)
        {
            if (scenecut(frames, j, j + 1, false, origNumFrames))
            {
                frames[j]->sliceType = X265_TYPE_P;
                numAnalyzed = j;
                break;
            }
        }
        resetStart = bKeyframe ? 1 : X265_MIN(numBFrames + 2, numAnalyzed + 1);
    }
    else
    {
        for (int j = 1; j <= numFrames; j++)
            frames[j]->sliceType = X265_TYPE_P;

        resetStart = bKeyframe ? 1 : 2;
    }
    if (m_param->bAQMotion)
        aqMotion(frames, bKeyframe);

    if (m_param->rc.cuTree)
        cuTree(frames, X265_MIN(numFrames, m_param->keyframeMax), bKeyframe);
    if (m_param->gopLookahead && (keyFrameLimit >= 0) && (keyFrameLimit <= m_param->bframes + 1) && !m_extendGopBoundary)
        keyintLimit = keyFrameLimit;

    if (!m_param->bIntraRefresh)
        for (int j = keyintLimit + 1; j <= numFrames; j += m_param->keyframeMax)
        {
            frames[j]->sliceType = X265_TYPE_I;
            resetStart = X265_MIN(resetStart, j + 1);
        }

    if (bIsVbvLookahead)
        vbvLookahead(frames, numFrames, bKeyframe);
    int maxp1 = X265_MIN(m_param->bframes + 1, origNumFrames);

    /* Restore frame types for all frames that haven't actually been decided yet. */
    for (int j = resetStart; j <= numFrames; j++)
    {
        frames[j]->sliceType = X265_TYPE_AUTO;
        /* If any frame marked as scenecut is being restarted for sliceDecision, 
         * undo scene Transition flag */
        if (j <= maxp1 && frames[j]->bScenecut && m_isSceneTransition)
            m_isSceneTransition = false;
    }
}


/** 函数功能             ： 判断当前是否场景切换（是否需要将p1帧类型设置为I帧）
/*  调用范围             ： 只在Lookahead::slicetypeAnalyse函数中被调用
* \参数 frames           ： 当前搜索的frames列表
* \参数 p0               ： 前一帧 注：p0 p1 相邻
* \参数 p1               ： 后一帧
* \参数 bRealScenecut    ： 当前是否是真正的场景切换判断，而不是预分析
* \参数 numFrames        ： 列表中原始帧个数
* \参数 maxSearch        ： lookachead最大的搜索帧数
* \返回                  ： 返回当前是否场景切换（是否需要将其帧类型设置为I帧） * */
bool Lookahead::scenecut(Lowres **frames, int p0, int p1, bool bRealScenecut, int numFrames)
{
    /* Only do analysis during a normal scenecut check. */
    if (bRealScenecut && m_param->bframes)
    {
        int origmaxp1 = p0 + 1;
        /* Look ahead to avoid coding short flashes as scenecuts. */
        origmaxp1 += m_param->bframes;
        int maxp1 = X265_MIN(origmaxp1, numFrames);
        bool fluctuate = false;
        bool noScenecuts = false;
        int64_t avgSatdCost = 0;
        if (frames[p0]->costEst[p1 - p0][0] > -1)
            avgSatdCost = frames[p0]->costEst[p1 - p0][0];
        int cnt = 1;
        /* Where A and B are scenes: AAAAAABBBAAAAAA
         * If BBB is shorter than (maxp1-p0), it is detected as a flash
         * and not considered a scenecut. */
        for (int cp1 = p1; cp1 <= maxp1; cp1++)
        {
            if (!scenecutInternal(frames, p0, cp1, false))
            {
                /* Any frame in between p0 and cur_p1 cannot be a real scenecut. */
                for (int i = cp1; i > p0; i--)
                {
                    frames[i]->bScenecut = false;
                    noScenecuts = false;
                }
            }
            else if (scenecutInternal(frames, cp1 - 1, cp1, false))
            {
                /* If current frame is a Scenecut from p0 frame as well as Scenecut from
                 * preceeding frame, mark it as a Scenecut */
                frames[cp1]->bScenecut = true;
                noScenecuts = true;
            }

            /* compute average satdcost of all the frames in the mini-gop to confirm 
             * whether there is any great fluctuation among them to rule out false positives */
            X265_CHECK(frames[cp1]->costEst[cp1 - p0][0]!= -1, "costEst is not done \n");
            avgSatdCost += frames[cp1]->costEst[cp1 - p0][0];
            cnt++;
        }

        /* Identify possible scene fluctuations by comparing the satd cost of the frames.
         * This could denote the beginning or ending of scene transitions.
         * During a scene transition(fade in/fade outs), if fluctuate remains false,
         * then the scene had completed its transition or stabilized */
        if (noScenecuts)
        {
            fluctuate = false;
            avgSatdCost /= cnt;
            for (int i = p1; i <= maxp1; i++)
            {
                int64_t curCost  = frames[i]->costEst[i - p0][0];
                int64_t prevCost = frames[i - 1]->costEst[i - 1 - p0][0];
                if (fabs((double)(curCost - avgSatdCost)) > 0.1 * avgSatdCost || 
                    fabs((double)(curCost - prevCost)) > 0.1 * prevCost)
                {
                    fluctuate = true;
                    if (!m_isSceneTransition && frames[i]->bScenecut)
                    {
                        m_isSceneTransition = true;
                        /* just mark the first scenechange in the scene transition as a scenecut. */
                        for (int j = i + 1; j <= maxp1; j++)
                            frames[j]->bScenecut = false;
                        break;
                    }
                }
                frames[i]->bScenecut = false;
            }
        }
        if (!fluctuate && !noScenecuts)
            m_isSceneTransition = false; /* Signal end of scene transitioning */
    }

    if (m_param->csvLogLevel >= 2)
    {
        int64_t icost = frames[p1]->costEst[0][0];
        int64_t pcost = frames[p1]->costEst[p1 - p0][0];
        frames[p1]->ipCostRatio = (double)icost / pcost;
    }

    /* A frame is always analysed with bRealScenecut = true first, and then bRealScenecut = false,
       the former for I decisions and the latter for P/B decisions. It's possible that the first 
       analysis detected scenecuts which were later nulled due to scene transitioning, in which 
       case do not return a true scenecut for this frame */

    if (!frames[p1]->bScenecut)
        return false;
    return scenecutInternal(frames, p0, p1, bRealScenecut);
}


/** 函数功能             ： 判断当前是否场景切换（是否需要将其帧类型设置为I帧）
/*  调用范围             ： 只在Lookahead::scenecut函数中被调用
* \参数 frames           ： 当前搜索的frames列表
* \参数 p0               ： 当前帧 注：p0 p1 不相邻 p1是p0后面某一帧
* \参数 p1               ： 后一帧
* \参数 bRealScenecut    ： 当前是否是真正的场景切换判断，而不是预分析
* \返回                  ： 返回当前是否场景切换（是否需要将其帧类型设置为I帧） * */
bool Lookahead::scenecutInternal(Lowres **frames, int p0, int p1, bool bRealScenecut)
{
    Lowres *frame = frames[p1];

    CostEstimateGroup estGroup(*this, frames);
    estGroup.singleCost(p0, p1, p1);
    int64_t icost = frame->costEst[0][0];
    int64_t pcost = frame->costEst[p1 - p0][0];
    int gopSize = (frame->frameNum - m_lastKeyframe) % m_param->keyframeMax;
    float threshMax = (float)(m_param->scenecutThreshold / 100.0);
    /* magic numbers pulled out of thin air */
    float threshMin = (float)(threshMax * 0.25);
    double bias = m_param->scenecutBias;
    if (bRealScenecut)
    {
        if (m_param->keyframeMin == m_param->keyframeMax)
            threshMin = threshMax;
        if (gopSize <= m_param->keyframeMin / 4 || m_param->bIntraRefresh)
            bias = threshMin / 4;
        else if (gopSize <= m_param->keyframeMin)
            bias = threshMin * gopSize / m_param->keyframeMin;
        else
        {
            bias = threshMin
                + (threshMax - threshMin)
                * (gopSize - m_param->keyframeMin)
                / (m_param->keyframeMax - m_param->keyframeMin);
        }
    }
    bool res = pcost >= (1.0 - bias) * icost;
    if (res && bRealScenecut)
    {
        int imb = frame->intraMbs[p1 - p0];
        int pmb = m_8x8Blocks - imb;
        x265_log(m_param, X265_LOG_DEBUG, "scene cut at %d Icost:%d Pcost:%d ratio:%.4f bias:%.4f gop:%d (imb:%d pmb:%d)\n",
                 frame->frameNum, icost, pcost, 1. - (double)pcost / icost, bias, gopSize, imb, pmb);
    }
    return res;
}


/** 函数功能             ： 计算当前搜索长度下的最优帧类型路径
/*  调用范围             ： 只在Lookahead::slicetypeAnalyse函数中被调用
* \参数 frames           ： 当前搜索的frames列表
* \参数 length           ： 当前搜索frame列表的长度 （1~length）
* \参数 *best_paths      ： 存储最优帧类型路径
* \返回                  ： null * */
void Lookahead::slicetypePath(Lowres **frames, int length, char(*best_paths)[X265_LOOKAHEAD_MAX + 1])
{
    char paths[2][X265_LOOKAHEAD_MAX + 1];
    int num_paths = X265_MIN(m_param->bframes + 1, length);
    int64_t best_cost = 1LL << 62;
    int idx = 0;

    /* Iterate over all currently possible paths */
    for (int path = 0; path < num_paths; path++)
    {
        /* Add suffixes to the current path */
        int len = length - (path + 1);
        memcpy(paths[idx], best_paths[len % (X265_BFRAME_MAX + 1)], len);
        memset(paths[idx] + len, 'B', path);
        strcpy(paths[idx] + len + path, "P");

        /* Calculate the actual cost of the current path */
        int64_t cost = slicetypePathCost(frames, paths[idx], best_cost);
        if (cost < best_cost)
        {
            best_cost = cost;
            idx ^= 1;
        }
    }

    /* Store the best path. */
    memcpy(best_paths[length % (X265_BFRAME_MAX + 1)], paths[idx ^ 1], length);
}


/** 函数功能             ： 计算当前PB帧设置的frame cost 累加值
/*  调用范围             ： 只在Lookahead::slicetypePath函数中被调用
* \参数 frames           ： 当前搜索的frames列表
* \参数 path             ： 当前frame列表的帧类型：如BPBBBPxxxx(x：暂未制定类型 第一个x前一定是P)
* \参数 threshold        ： 当前最优的framecost
* \返回                  ： 返回当前PB帧设置的frame cost 累加值 * */
int64_t Lookahead::slicetypePathCost(Lowres **frames, char *path, int64_t threshold)
{
    int64_t cost = 0;
    int loc = 1;
    int cur_p = 0;

    CostEstimateGroup estGroup(*this, frames);

    path--; /* Since the 1st path element is really the second frame */
    while (path[loc])
    {
        int next_p = loc;
        /* Find the location of the next P-frame. */
        while (path[next_p] != 'P')
            next_p++;

        /* Add the cost of the P-frame found above */
        cost += estGroup.singleCost(cur_p, next_p, next_p);

        /* Early terminate if the cost we have found is larger than the best path cost so far */
        if (cost > threshold)
            break;

        if (m_param->bBPyramid && next_p - cur_p > 2)
        {
            int middle = cur_p + (next_p - cur_p) / 2;
            cost += estGroup.singleCost(cur_p, next_p, middle);

            for (int next_b = loc; next_b < middle && cost < threshold; next_b++)
                cost += estGroup.singleCost(cur_p, middle, next_b);

            for (int next_b = middle + 1; next_b < next_p && cost < threshold; next_b++)
                cost += estGroup.singleCost(middle, next_p, next_b);
        }
        else
        {
            for (int next_b = loc; next_b < next_p && cost < threshold; next_b++)
                cost += estGroup.singleCost(cur_p, next_p, next_b);
        }

        loc = next_p + 1;
        cur_p = next_p;
    }

    return cost;
}
void Lookahead::aqMotion(Lowres **frames, bool bIntra)
{
    if (!bIntra)
    {
        int curnonb = 0, lastnonb = 1;
        int bframes = 0, i = 1;
        while (frames[lastnonb]->sliceType != X265_TYPE_P)
            lastnonb++;
        bframes = lastnonb - 1;
        if (m_param->bBPyramid && bframes > 1)
        {
            int middle = (bframes + 1) / 2;
            for (i = 1; i < lastnonb; i++)
            {
                int p0 = i > middle ? middle : curnonb;
                int p1 = i < middle ? middle : lastnonb;
                if (i != middle)
                    calcMotionAdaptiveQuantFrame(frames, p0, p1, i);
            }
            calcMotionAdaptiveQuantFrame(frames, curnonb, lastnonb, middle);
        }
        else
            for (i = 1; i < lastnonb; i++)
                calcMotionAdaptiveQuantFrame(frames, curnonb, lastnonb, i);
        calcMotionAdaptiveQuantFrame(frames, curnonb, lastnonb, lastnonb);
    }
}

void Lookahead::calcMotionAdaptiveQuantFrame(Lowres **frames, int p0, int p1, int b)
{
    int listDist[2] = { b - p0, p1 - b };
    int32_t strideInCU = m_8x8Width;
    double qp_adj = 0, avg_adj = 0, avg_adj_pow2 = 0, sd;
    for (uint16_t blocky = 0; blocky < m_8x8Height; blocky++)
    {
        int cuIndex = blocky * strideInCU;
        for (uint16_t blockx = 0; blockx < m_8x8Width; blockx++, cuIndex++)
        {
            int32_t lists_used = frames[b]->lowresCosts[b - p0][p1 - b][cuIndex] >> LOWRES_COST_SHIFT;
            double displacement = 0;
            for (uint16_t list = 0; list < 2; list++)
            {
                if ((lists_used >> list) & 1)
                {
                    MV *mvs = frames[b]->lowresMvs[list][listDist[list]];
                    int32_t x = mvs[cuIndex].x;
                    int32_t y = mvs[cuIndex].y;
                    // NOTE: the dynamic range of abs(x) and abs(y) is 15-bits
                    displacement += sqrt((double)(abs(x) * abs(x)) + (double)(abs(y) * abs(y)));
                }
                else
                    displacement += 0.0;
            }
            if (lists_used == 3)
                displacement = displacement / 2;
            qp_adj = pow(displacement, 0.1);
            frames[b]->qpAqMotionOffset[cuIndex] = qp_adj;
            avg_adj += qp_adj;
            avg_adj_pow2 += qp_adj * qp_adj;
        }
    }
    avg_adj /= m_cuCount;
    avg_adj_pow2 /= m_cuCount;
    sd = sqrt((avg_adj_pow2 - (avg_adj * avg_adj)));
    if (sd > 0)
    {
        for (uint16_t blocky = 0; blocky < m_8x8Height; blocky++)
        {
            int cuIndex = blocky * strideInCU;
            for (uint16_t blockx = 0; blockx < m_8x8Width; blockx++, cuIndex++)
            {
                qp_adj = frames[b]->qpAqMotionOffset[cuIndex];
                qp_adj = (qp_adj - avg_adj) / sd;
                if (qp_adj > 1)
                {
                    frames[b]->qpAqOffset[cuIndex] += qp_adj;
                    frames[b]->qpCuTreeOffset[cuIndex] += qp_adj;
                    frames[b]->invQscaleFactor[cuIndex] += x265_exp2fix8(qp_adj);
                }
            }
        }
    }
}

/** 函数功能             ： 计算可参考帧的qpCuTreeOffset值
/*  调用范围             ： 只在slicetypeAnalyse函数中被调用
* \参数 frames           ： 当前搜索的frames列表
* \参数 numframes        ： frames列表帧数
* \参数 bIntra           ： 是否是IDR帧检测
* \返回                  ： null * */
void Lookahead::cuTree(Lowres **frames, int numframes, bool bIntra)
{
    int idx = !bIntra;
    int lastnonb, curnonb = 1;
    int bframes = 0;

    x265_emms();
    double totalDuration = 0.0;
    for (int j = 0; j <= numframes; j++)
        totalDuration += (double)m_param->fpsDenom / m_param->fpsNum;

    double averageDuration = totalDuration / (numframes + 1);

    int i = numframes;

    while (i > 0 && frames[i]->sliceType == X265_TYPE_B)
        i--;

    lastnonb = i;

    /* Lookaheadless MB-tree is not a theoretically distinct case; the same extrapolation could
     * be applied to the end of a lookahead buffer of any size.  However, it's most needed when
     * lookahead=0, so that's what's currently implemented. */
    if (!m_param->lookaheadDepth)
    {
        if (bIntra)
        {
            memset(frames[0]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
            if (m_param->rc.qgSize == 8)
                memcpy(frames[0]->qpCuTreeOffset, frames[0]->qpAqOffset, m_cuCount * 4 * sizeof(double));
            else
                memcpy(frames[0]->qpCuTreeOffset, frames[0]->qpAqOffset, m_cuCount * sizeof(double));
            return;
        }
        std::swap(frames[lastnonb]->propagateCost, frames[0]->propagateCost);
        memset(frames[0]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
    }
    else
    {
        if (lastnonb < idx)
            return;
        memset(frames[lastnonb]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
    }

    CostEstimateGroup estGroup(*this, frames);

    while (i-- > idx)
    {
        curnonb = i;
        while (frames[curnonb]->sliceType == X265_TYPE_B && curnonb > 0)
            curnonb--;

        if (curnonb < idx)
            break;

        estGroup.singleCost(curnonb, lastnonb, lastnonb);

        memset(frames[curnonb]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
        bframes = lastnonb - curnonb - 1;
        if (m_param->bBPyramid && bframes > 1)
        {
            int middle = (bframes + 1) / 2 + curnonb;
            estGroup.singleCost(curnonb, lastnonb, middle);
            memset(frames[middle]->propagateCost, 0, m_cuCount * sizeof(uint16_t));
            while (i > curnonb)
            {
                int p0 = i > middle ? middle : curnonb;
                int p1 = i < middle ? middle : lastnonb;
                if (i != middle)
                {
                    estGroup.singleCost(p0, p1, i);
                    estimateCUPropagate(frames, averageDuration, p0, p1, i, 0);
                }
                i--;
            }

            estimateCUPropagate(frames, averageDuration, curnonb, lastnonb, middle, 1);
        }
        else
        {
            while (i > curnonb)
            {
                estGroup.singleCost(curnonb, lastnonb, i);
                estimateCUPropagate(frames, averageDuration, curnonb, lastnonb, i, 0);
                i--;
            }
        }
        estimateCUPropagate(frames, averageDuration, curnonb, lastnonb, lastnonb, 1);
        lastnonb = curnonb;
    }

    if (!m_param->lookaheadDepth)
    {
        estGroup.singleCost(0, lastnonb, lastnonb);
        estimateCUPropagate(frames, averageDuration, 0, lastnonb, lastnonb, 1);
        std::swap(frames[lastnonb]->propagateCost, frames[0]->propagateCost);
    }

    cuTreeFinish(frames[lastnonb], averageDuration, lastnonb);
    if (m_param->bBPyramid && bframes > 1 && !m_param->rc.vbvBufferSize)
        cuTreeFinish(frames[lastnonb + (bframes + 1) / 2], averageDuration, 0);
}


/** 函数功能             ： 计算当前编码帧每个8x8块对应参考块的传播cost、如果当前编码帧是可被参考帧计算其qpCuTreeOffset值
/*  调用范围             ： 只在Lookahead::cuTree函数中被调用
* \参数 frames           ： 当前搜索的frames列表
* \参数 averageDuration  ： 平均每帧的播放时长 单位秒
* \参数 p0               ： 前向帧
* \参数 p1               ： 后向帧
* \参数 b                ： 当前帧号
* \参数 referenced       ： 当前计算的b号帧是否是可被参考的
* \返回                  ： null * */
void Lookahead::estimateCUPropagate(Lowres **frames, double averageDuration, int p0, int p1, int b, int referenced)
{
    uint16_t *refCosts[2] = { frames[p0]->propagateCost, frames[p1]->propagateCost };
    int32_t distScaleFactor = (((b - p0) << 8) + ((p1 - p0) >> 1)) / (p1 - p0);
    int32_t bipredWeight = m_param->bEnableWeightedBiPred ? 64 - (distScaleFactor >> 2) : 32;
    int32_t bipredWeights[2] = { bipredWeight, 64 - bipredWeight };
    int listDist[2] = { b - p0, p1 - b };

    memset(m_scratch, 0, m_8x8Width * sizeof(int));

    uint16_t *propagateCost = frames[b]->propagateCost;

    x265_emms();
    double fpsFactor = CLIP_DURATION((double)m_param->fpsDenom / m_param->fpsNum) / CLIP_DURATION(averageDuration);

    /* For non-referred frames the source costs are always zero, so just memset one row and re-use it. */
    if (!referenced)
        memset(frames[b]->propagateCost, 0, m_8x8Width * sizeof(uint16_t));

    int32_t strideInCU = m_8x8Width;
    for (uint16_t blocky = 0; blocky < m_8x8Height; blocky++)
    {
        int cuIndex = blocky * strideInCU;
        if (m_param->rc.qgSize == 8)
            primitives.propagateCost(m_scratch, propagateCost,
                       frames[b]->intraCost + cuIndex, frames[b]->lowresCosts[b - p0][p1 - b] + cuIndex,
                       frames[b]->invQscaleFactor8x8 + cuIndex, &fpsFactor, m_8x8Width);
        else
            primitives.propagateCost(m_scratch, propagateCost,
                       frames[b]->intraCost + cuIndex, frames[b]->lowresCosts[b - p0][p1 - b] + cuIndex,
                       frames[b]->invQscaleFactor + cuIndex, &fpsFactor, m_8x8Width);

        if (referenced)
            propagateCost += m_8x8Width;

        for (uint16_t blockx = 0; blockx < m_8x8Width; blockx++, cuIndex++)
        {
            int32_t propagate_amount = m_scratch[blockx];
            /* Don't propagate for an intra block. */
            if (propagate_amount > 0)
            {
                /* Access width-2 bitfield. */
                int32_t lists_used = frames[b]->lowresCosts[b - p0][p1 - b][cuIndex] >> LOWRES_COST_SHIFT;
                /* Follow the MVs to the previous frame(s). */
                for (uint16_t list = 0; list < 2; list++)
                {
                    if ((lists_used >> list) & 1)
                    {
#define CLIP_ADD(s, x) (s) = (uint16_t)X265_MIN((s) + (x), (1 << 16) - 1)
                        int32_t listamount = propagate_amount;
                        /* Apply bipred weighting. */
                        if (lists_used == 3)
                            listamount = (listamount * bipredWeights[list] + 32) >> 6;

                        MV *mvs = frames[b]->lowresMvs[list][listDist[list]];

                        /* Early termination for simple case of mv0. */
                        if (!mvs[cuIndex].word)
                        {
                            CLIP_ADD(refCosts[list][cuIndex], listamount);
                            continue;
                        }

                        int32_t x = mvs[cuIndex].x;
                        int32_t y = mvs[cuIndex].y;
                        int32_t cux = (x >> 5) + blockx;
                        int32_t cuy = (y >> 5) + blocky;
                        int32_t idx0 = cux + cuy * strideInCU;
                        int32_t idx1 = idx0 + 1;
                        int32_t idx2 = idx0 + strideInCU;
                        int32_t idx3 = idx0 + strideInCU + 1;
                        x &= 31;
                        y &= 31;
                        int32_t idx0weight = (32 - y) * (32 - x);
                        int32_t idx1weight = (32 - y) * x;
                        int32_t idx2weight = y * (32 - x);
                        int32_t idx3weight = y * x;

                        /* We could just clip the MVs, but pixels that lie outside the frame probably shouldn't
                         * be counted. */
                        if (cux < m_8x8Width - 1 && cuy < m_8x8Height - 1 && cux >= 0 && cuy >= 0)
                        {
                            CLIP_ADD(refCosts[list][idx0], (listamount * idx0weight + 512) >> 10);
                            CLIP_ADD(refCosts[list][idx1], (listamount * idx1weight + 512) >> 10);
                            CLIP_ADD(refCosts[list][idx2], (listamount * idx2weight + 512) >> 10);
                            CLIP_ADD(refCosts[list][idx3], (listamount * idx3weight + 512) >> 10);
                        }
                        else /* Check offsets individually */
                        {
                            if (cux < m_8x8Width && cuy < m_8x8Height && cux >= 0 && cuy >= 0)
                                CLIP_ADD(refCosts[list][idx0], (listamount * idx0weight + 512) >> 10);
                            if (cux + 1 < m_8x8Width && cuy < m_8x8Height && cux + 1 >= 0 && cuy >= 0)
                                CLIP_ADD(refCosts[list][idx1], (listamount * idx1weight + 512) >> 10);
                            if (cux < m_8x8Width && cuy + 1 < m_8x8Height && cux >= 0 && cuy + 1 >= 0)
                                CLIP_ADD(refCosts[list][idx2], (listamount * idx2weight + 512) >> 10);
                            if (cux + 1 < m_8x8Width && cuy + 1 < m_8x8Height && cux + 1 >= 0 && cuy + 1 >= 0)
                                CLIP_ADD(refCosts[list][idx3], (listamount * idx3weight + 512) >> 10);
                        }
                    }
                }
            }
        }
    }

    if (m_param->rc.vbvBufferSize && m_param->lookaheadDepth && referenced)
        cuTreeFinish(frames[b], averageDuration, b == p1 ? b - p0 : 0);
}


/** 函数功能             ： 计算参考帧qpCuTreeOffset值
/*  调用范围             ： 只在Lookahead::cuTree和Lookahead::estimateCUPropagate函数中被调用
* \参数 frames           ： 当前搜索的frames列表
* \参数 averageDuration  ： 平均每帧的播放时长 单位秒
* \参数 ref0Distance     ： 如果是双向参考则为0，如果当前为单向参考则为当前帧号-前向参考帧号
* \返回                  ： null * */
void Lookahead::cuTreeFinish(Lowres *frame, double averageDuration, int ref0Distance)
{
    int fpsFactor = (int)(CLIP_DURATION(averageDuration) / CLIP_DURATION((double)m_param->fpsDenom / m_param->fpsNum) * 256);
    double weightdelta = 0.0;

    if (ref0Distance && frame->weightedCostDelta[ref0Distance - 1] > 0)
        weightdelta = (1.0 - frame->weightedCostDelta[ref0Distance - 1]);

    if (m_param->rc.qgSize == 8)
    {
        for (int cuY = 0; cuY < m_8x8Height; cuY++)
        {
            for (int cuX = 0; cuX < m_8x8Width; cuX++)
            {
                const int cuXY = cuX + cuY * m_8x8Width;
                int intracost = ((frame->intraCost[cuXY]) / 4 * frame->invQscaleFactor8x8[cuXY] + 128) >> 8;
                if (intracost)
                {
                    int propagateCost = ((frame->propagateCost[cuXY]) / 4 * fpsFactor + 128) >> 8;
                    double log2_ratio = X265_LOG2(intracost + propagateCost) - X265_LOG2(intracost) + weightdelta;
                    frame->qpCuTreeOffset[cuX * 2 + cuY * m_8x8Width * 4] = frame->qpAqOffset[cuX * 2 + cuY * m_8x8Width * 4] - m_cuTreeStrength * (log2_ratio);
                    frame->qpCuTreeOffset[cuX * 2 + cuY * m_8x8Width * 4 + 1] = frame->qpAqOffset[cuX * 2 + cuY * m_8x8Width * 4 + 1] - m_cuTreeStrength * (log2_ratio);
                    frame->qpCuTreeOffset[cuX * 2 + cuY * m_8x8Width * 4 + frame->maxBlocksInRowFullRes] = frame->qpAqOffset[cuX * 2 + cuY * m_8x8Width * 4 + frame->maxBlocksInRowFullRes] - m_cuTreeStrength * (log2_ratio);
                    frame->qpCuTreeOffset[cuX * 2 + cuY * m_8x8Width * 4 + frame->maxBlocksInRowFullRes + 1] = frame->qpAqOffset[cuX * 2 + cuY * m_8x8Width * 4 + frame->maxBlocksInRowFullRes + 1] - m_cuTreeStrength * (log2_ratio);
                }
            }
        }
    }
    else
    {
        for (int cuIndex = 0; cuIndex < m_cuCount; cuIndex++)
        {
            int intracost = (frame->intraCost[cuIndex] * frame->invQscaleFactor[cuIndex] + 128) >> 8;
            if (intracost)
            {
                int propagateCost = (frame->propagateCost[cuIndex] * fpsFactor + 128) >> 8;
                double log2_ratio = X265_LOG2(intracost + propagateCost) - X265_LOG2(intracost) + weightdelta;
                frame->qpCuTreeOffset[cuIndex] = frame->qpAqOffset[cuIndex] - m_cuTreeStrength * log2_ratio;
            }
        }
    }
}

/* If MB-tree changes the quantizers, we need to recalculate the frame cost without
 * re-running lookahead. */

/** 函数功能             ： 如果当前是B帧直接返回其framecost：costEstAq （qpAqOffset加权）否则，重新计算framecost  经过qpCuTreeOffset加权后的数据
/*  调用范围             ： 只在Lookahead::getEstimatedPictureCost和Lookahead::vbvFrameCost函数中被调用
* \参数 frames           ： 当前搜索的frames列表
* \参数 p0               ： 前向帧
* \参数 p1               ： 后向帧
* \参数 b                ： 当前帧号
* \返回                  ： 返回重新计算的framecost* */
int64_t Lookahead::frameCostRecalculate(Lowres** frames, int p0, int p1, int b)
{
    if (frames[b]->sliceType == X265_TYPE_B)
        return frames[b]->costEstAq[b - p0][p1 - b];

    int64_t score = 0;
    int *rowSatd = frames[b]->rowSatds[b - p0][p1 - b];
    double *qp_offset = frames[b]->qpCuTreeOffset;

    x265_emms();
    for (int cuy = m_8x8Height - 1; cuy >= 0; cuy--)
    {
        rowSatd[cuy] = 0;
        for (int cux = m_8x8Width - 1; cux >= 0; cux--)
        {
            int cuxy = cux + cuy * m_8x8Width;
            int cuCost = frames[b]->lowresCosts[b - p0][p1 - b][cuxy] & LOWRES_COST_MASK;
            double qp_adj;
            if (m_param->rc.qgSize == 8)
                qp_adj = (qp_offset[cux * 2 + cuy * m_8x8Width * 4] +
                          qp_offset[cux * 2 + cuy * m_8x8Width * 4 + 1] +
                          qp_offset[cux * 2 + cuy * m_8x8Width * 4 + frames[b]->maxBlocksInRowFullRes] +
                          qp_offset[cux * 2 + cuy * m_8x8Width * 4 + frames[b]->maxBlocksInRowFullRes + 1]) / 4;
            else 
                qp_adj = qp_offset[cuxy];
            cuCost = (cuCost * x265_exp2fix8(qp_adj) + 128) >> 8;
            rowSatd[cuy] += cuCost;
            if ((cuy > 0 && cuy < m_8x8Height - 1 &&
                 cux > 0 && cux < m_8x8Width - 1) ||
                m_8x8Width <= 2 || m_8x8Height <= 2)
            {
                score += cuCost;
            }
        }
    }

    return score;
}


/** 函数功能             ：单线程计算当前帧与前后参考帧之间的最优frame cost
/*  调用范围             ：只在slicetypeDecide()、vbvFrameCost、slicetypeAnalyse、scenecutInternal、slicetypePathCost和cuTree函数中被调用
* \参数 p0               ：前向帧
* \参数 p1               ：后向帧
* \参数 b                ：当前帧号
* \参数 bIntraPenalty    ：会加上intra带来的编码代价;其中在slicetypeAnalyse会传入ture(也可能为false)并且bFrameAdaptive == X265_B_ADAPT_FAST，其它为false
* \返回                  ：当前帧与前后参考帧之间的最优frame cost* */
int64_t CostEstimateGroup::singleCost(int p0, int p1, int b, bool intraPenalty)
{
    LookaheadTLD& tld = m_lookahead.m_tld[m_lookahead.m_pool ? m_lookahead.m_pool->m_numWorkers : 0];
    return estimateFrameCost(tld, p0, p1, b, intraPenalty);
}


/** 函数功能             ： 添加任务，为后面并发执行做准备
/*  调用范围             ： 只在Lookahead::slicetypeAnalyse函数中被调用
* \参数 p0               ： 前向帧
* \参数 p1               ： 后向帧
* \参数 b                ： 当前帧号
* \返回                  ： null * */
void CostEstimateGroup::add(int p0, int p1, int b)
{
    X265_CHECK(m_batchMode || !m_jobTotal, "single CostEstimateGroup instance cannot mix batch modes\n");
    m_batchMode = true;

    Estimate& e = m_estimates[m_jobTotal++];
    e.p0 = p0;
    e.p1 = p1;
    e.b = b;

    if (m_jobTotal == MAX_BATCH_SIZE)
        finishBatch();
}

/** 函数功能             ： 触发并发执行并一直等到所有任务执行完毕：计算每帧与其对应参考帧之间的帧间cost
/*  调用范围             ： 只在Lookahead::slicetypeAnalyse和CostEstimateGroup::add函数中被调用
* \返回                  ： null * */
void CostEstimateGroup::finishBatch()
{
    if (m_lookahead.m_pool)
        tryBondPeers(*m_lookahead.m_pool, m_jobTotal);
    processTasks(-1);
    waitForExit();
    m_jobTotal = m_jobAcquired = 0;
}


/** 函数功能             ： 功能分两个只能执行其中一个：
1. 获取取每个8x8块的帧间cost(SATD + mvcost + 4) 并 获取当前b帧在p0、p1参考帧下的最优帧cost inter(SATD+mvcost+4)*10/(13 +b)或者 intra (SATD+5+4)
2. Lookachead 多slice并行，执行其中一条slice的每个8x8块的帧间cost(SATD + mvcost + 4)
/*  调用范围             ： 只在CostEstimateGroup::finishBatch()和CostEstimateGroup::estimateFrameCost函数中被调用 （分别执行1,2功能）
* \返回                  ： null * */
void CostEstimateGroup::processTasks(int workerThreadID)
{
    ThreadPool* pool = m_lookahead.m_pool;
    int id = workerThreadID;
    if (workerThreadID < 0)
        id = pool ? pool->m_numWorkers : 0;
    LookaheadTLD& tld = m_lookahead.m_tld[id];

    m_lock.acquire();
    while (m_jobAcquired < m_jobTotal)
    {
        int i = m_jobAcquired++;
        m_lock.release();

        if (m_batchMode)
        {
            ProfileLookaheadTime(tld.batchElapsedTime, tld.countBatches);
            ProfileScopeEvent(estCostSingle);

            Estimate& e = m_estimates[i];
            estimateFrameCost(tld, e.p0, e.p1, e.b, false);
        }
        else
        {
            ProfileLookaheadTime(tld.coopSliceElapsedTime, tld.countCoopSlices);
            ProfileScopeEvent(estCostCoop);

            X265_CHECK(i < MAX_COOP_SLICES, "impossible number of coop slices\n");

            int firstY = m_lookahead.m_numRowsPerSlice * i;
            int lastY = (i == m_jobTotal - 1) ? m_lookahead.m_8x8Height - 1 : m_lookahead.m_numRowsPerSlice * (i + 1) - 1;

            bool lastRow = true;
            for (int cuY = lastY; cuY >= firstY; cuY--)
            {
                m_frames[m_coop.b]->rowSatds[m_coop.b - m_coop.p0][m_coop.p1 - m_coop.b][cuY] = 0;

                for (int cuX = m_lookahead.m_8x8Width - 1; cuX >= 0; cuX--)
                    estimateCUCost(tld, cuX, cuY, m_coop.p0, m_coop.p1, m_coop.b, m_coop.bDoSearch, lastRow, i);

                lastRow = false;
            }
        }

        m_lock.acquire();
    }
    m_lock.release();
}


/** 函数功能             ：获取取每个8x8块的帧间cost(SATD + mvcost + 4) 并 获取当前b帧在p0、p1参考帧下的最优帧cost inter(SATD+mvcost+4)*10/(13 +b)或者 intra (SATD+5+4)
/*  调用范围             ：只在CostEstimateGroup::singleCost和CostEstimateGroup::processTasks函数中被调用
* \参数 tld              ：当前线程的tld
* \参数 p0               ：前向帧
* \参数 p1               ：后向帧
* \参数 b                ：当前帧号
* \参数 bIntraPenalty    ：会加上intra带来的编码代价;在CostEstimateGroup::processTasks中为false 在调用CostEstimateGroup::singleCost中的slicetypeAnalyse会传入ture(也可能为false) 并且bFrameAdaptive == X265_B_ADAPT_FAST
* \返回                  ：当前b帧在p0、p1参考帧下的最优帧cost inter(SATD+mvcost+4)*10/(13 +b)或者 intra (SATD+5+4) * */
int64_t CostEstimateGroup::estimateFrameCost(LookaheadTLD& tld, int p0, int p1, int b, bool bIntraPenalty)
{
    Lowres*     fenc  = m_frames[b];
    x265_param* param = m_lookahead.m_param;
    int64_t     score = 0;

    if (fenc->costEst[b - p0][p1 - b] >= 0 && fenc->rowSatds[b - p0][p1 - b][0] != -1)
        score = fenc->costEst[b - p0][p1 - b];
    else
    {
        bool bDoSearch[2];
        bDoSearch[0] = fenc->lowresMvs[0][b - p0][0].x == 0x7FFF;
        bDoSearch[1] = p1 > b && fenc->lowresMvs[1][p1 - b][0].x == 0x7FFF;

#if CHECKED_BUILD
        X265_CHECK(!(p0 < b && fenc->lowresMvs[0][b - p0][0].x == 0x7FFE), "motion search batch duplication L0\n");
        X265_CHECK(!(p1 > b && fenc->lowresMvs[1][p1 - b][0].x == 0x7FFE), "motion search batch duplication L1\n");
        if (bDoSearch[0]) fenc->lowresMvs[0][b - p0][0].x = 0x7FFE;
        if (bDoSearch[1]) fenc->lowresMvs[1][p1 - b][0].x = 0x7FFE;
#endif

        fenc->weightedRef[b - p0].isWeighted = false;
        if (param->bEnableWeightedPred && bDoSearch[0])
            tld.weightsAnalyse(*m_frames[b], *m_frames[p0]);

        fenc->costEst[b - p0][p1 - b] = 0;
        fenc->costEstAq[b - p0][p1 - b] = 0;

        if (!m_batchMode && m_lookahead.m_numCoopSlices > 1 && ((p1 > b) || bDoSearch[0] || bDoSearch[1]))
        {
            /* Use cooperative mode if a thread pool is available and the cost estimate is
             * going to need motion searches or bidir measurements */

            memset(&m_slice, 0, sizeof(Slice) * m_lookahead.m_numCoopSlices);

            m_lock.acquire();
            X265_CHECK(!m_batchMode, "single CostEstimateGroup instance cannot mix batch modes\n");
            m_coop.p0 = p0;
            m_coop.p1 = p1;
            m_coop.b = b;
            m_coop.bDoSearch[0] = bDoSearch[0];
            m_coop.bDoSearch[1] = bDoSearch[1];
            m_jobTotal = m_lookahead.m_numCoopSlices;
            m_jobAcquired = 0;
            m_lock.release();

            tryBondPeers(*m_lookahead.m_pool, m_jobTotal);

            processTasks(-1);

            waitForExit();

            for (int i = 0; i < m_lookahead.m_numCoopSlices; i++)
            {
                fenc->costEst[b - p0][p1 - b] += m_slice[i].costEst;
                fenc->costEstAq[b - p0][p1 - b] += m_slice[i].costEstAq;
                if (p1 == b)
                    fenc->intraMbs[b - p0] += m_slice[i].intraMbs;
            }
        }
        else
        {
            bool lastRow = true;
            for (int cuY = m_lookahead.m_8x8Height - 1; cuY >= 0; cuY--)
            {
                fenc->rowSatds[b - p0][p1 - b][cuY] = 0;

                for (int cuX = m_lookahead.m_8x8Width - 1; cuX >= 0; cuX--)
                    estimateCUCost(tld, cuX, cuY, p0, p1, b, bDoSearch, lastRow, -1);

                lastRow = false;
            }
        }

        score = fenc->costEst[b - p0][p1 - b];

        if (b != p1)
            score = score * 100 / (130 + param->bFrameBias);

        fenc->costEst[b - p0][p1 - b] = score;
    }

    if (bIntraPenalty)
        // arbitrary penalty for I-blocks after B-frames
        score += score * fenc->intraMbs[b - p0] / (tld.ncu * 8);

    return score;
}


/** 函数功能             ：获取每个8x8块的帧间cost(SATD + mvcost + 4)
/*  调用范围             ：只在 CostEstimateGroup::processTasks和CostEstimateGroup::estimateFrameCost函数中被调用 (一般只在CostEstimateGroup::estimateFrameCost)
* \参数 tld              ：当前线程的tld
* \参数 cuX              ：当前帧的8x8 X坐标
* \参数 cuY              ：当前帧的8x8 Y坐标
* \参数 p0               ：前向帧
* \参数 p1               ：后向帧
* \参数 b                ：当前帧号
* \参数 bDoSearch        ：分别表示p0\p1需不需要search
* \参数 lastRow          ：是否是最后一行
* \参数 slice            ：在CostEstimateGroup::processTasks为相应slice  在CostEstimateGroup::estimateFrameCost为-1
* \返回                  ：null * */
void CostEstimateGroup::estimateCUCost(LookaheadTLD& tld, int cuX, int cuY, int p0, int p1, int b, bool bDoSearch[2], bool lastRow, int slice)
{
    Lowres *fref0 = m_frames[p0];
    Lowres *fref1 = m_frames[p1];
    Lowres *fenc  = m_frames[b];

    ReferencePlanes *wfref0 = fenc->weightedRef[b - p0].isWeighted ? &fenc->weightedRef[b - p0] : fref0;

    const int widthInCU = m_lookahead.m_8x8Width;
    const int heightInCU = m_lookahead.m_8x8Height;
    const int bBidir = (b < p1);
    const int cuXY = cuX + cuY * widthInCU;
    const int cuSize = X265_LOWRES_CU_SIZE;
    const intptr_t pelOffset = cuSize * cuX + cuSize * cuY * fenc->lumaStride;

    if (bBidir || bDoSearch[0] || bDoSearch[1])
        tld.me.setSourcePU(fenc->lowresPlane[0], fenc->lumaStride, pelOffset, cuSize, cuSize, X265_HEX_SEARCH, 1);

    /* A small, arbitrary bias to avoid VBV problems caused by zero-residual lookahead blocks. */
    int lowresPenalty = 4;
    int listDist[2] = { b - p0, p1 - b};

    MV mvmin, mvmax;
    int bcost = tld.me.COST_MAX;
    int listused = 0;

    // TODO: restrict to slices boundaries
    // establish search bounds that don't cross extended frame boundaries
    mvmin.x = (int16_t)(-cuX * cuSize - 8);
    mvmin.y = (int16_t)(-cuY * cuSize - 8);
    mvmax.x = (int16_t)((widthInCU - cuX - 1) * cuSize + 8);
    mvmax.y = (int16_t)((heightInCU - cuY - 1) * cuSize + 8);

    for (int i = 0; i < 1 + bBidir; i++)
    {
        int& fencCost = fenc->lowresMvCosts[i][listDist[i]][cuXY];
        int skipCost = INT_MAX;

        if (!bDoSearch[i])
        {
            COPY2_IF_LT(bcost, fencCost, listused, i + 1);
            continue;
        }

        int numc = 0;
        MV mvc[4], mvp;
        MV* fencMV = &fenc->lowresMvs[i][listDist[i]][cuXY];
        ReferencePlanes* fref = i ? fref1 : wfref0;

        /* Reverse-order MV prediction */
#define MVC(mv) mvc[numc++] = mv;
        if (cuX < widthInCU - 1)
            MVC(fencMV[1]);
        if (!lastRow)
        {
            MVC(fencMV[widthInCU]);
            if (cuX > 0)
                MVC(fencMV[widthInCU - 1]);
            if (cuX < widthInCU - 1)
                MVC(fencMV[widthInCU + 1]);
        }
#undef MVC

        if (!numc)
            mvp = 0;
        else
        {
            ALIGN_VAR_32(pixel, subpelbuf[X265_LOWRES_CU_SIZE * X265_LOWRES_CU_SIZE]);
            int mvpcost = MotionEstimate::COST_MAX;

            /* measure SATD cost of each neighbor MV (estimating merge analysis)
             * and use the lowest cost MV as MVP (estimating AMVP). Since all
             * mvc[] candidates are measured here, none are passed to motionEstimate */
            for (int idx = 0; idx < numc; idx++)
            {
                intptr_t stride = X265_LOWRES_CU_SIZE;
                pixel *src = fref->lowresMC(pelOffset, mvc[idx], subpelbuf, stride);
                int cost = tld.me.bufSATD(src, stride);
                COPY2_IF_LT(mvpcost, cost, mvp, mvc[idx]);
                /* Except for mv0 case, everyting else is likely to have enough residual to not trigger the skip. */
                if (!mvp.notZero() && bBidir)
                    skipCost = cost;
            }
        }

        /* ME will never return a cost larger than the cost @MVP, so we do not
         * have to check that ME cost is more than the estimated merge cost */
        fencCost = tld.me.motionEstimate(fref, mvmin, mvmax, mvp, 0, NULL, s_merange, *fencMV, m_lookahead.m_param->maxSlices);
        if (skipCost < 64 && skipCost < fencCost && bBidir)
        {
            fencCost = skipCost;
            *fencMV = 0;
        }
        COPY2_IF_LT(bcost, fencCost, listused, i + 1);
    }

    if (bBidir) /* B, also consider bidir */
    {
        /* NOTE: the wfref0 (weightp) is not used for BIDIR */

        /* avg(l0-mv, l1-mv) candidate */
        ALIGN_VAR_32(pixel, subpelbuf0[X265_LOWRES_CU_SIZE * X265_LOWRES_CU_SIZE]);
        ALIGN_VAR_32(pixel, subpelbuf1[X265_LOWRES_CU_SIZE * X265_LOWRES_CU_SIZE]);
        intptr_t stride0 = X265_LOWRES_CU_SIZE, stride1 = X265_LOWRES_CU_SIZE;
        pixel *src0 = fref0->lowresMC(pelOffset, fenc->lowresMvs[0][listDist[0]][cuXY], subpelbuf0, stride0);
        pixel *src1 = fref1->lowresMC(pelOffset, fenc->lowresMvs[1][listDist[1]][cuXY], subpelbuf1, stride1);
        ALIGN_VAR_32(pixel, ref[X265_LOWRES_CU_SIZE * X265_LOWRES_CU_SIZE]);
        primitives.pu[LUMA_8x8].pixelavg_pp[NONALIGNED](ref, X265_LOWRES_CU_SIZE, src0, stride0, src1, stride1, 32);
        int bicost = tld.me.bufSATD(ref, X265_LOWRES_CU_SIZE);
        COPY2_IF_LT(bcost, bicost, listused, 3);
        /* coloc candidate */
        src0 = fref0->lowresPlane[0] + pelOffset;
        src1 = fref1->lowresPlane[0] + pelOffset;
        primitives.pu[LUMA_8x8].pixelavg_pp[NONALIGNED](ref, X265_LOWRES_CU_SIZE, src0, fref0->lumaStride, src1, fref1->lumaStride, 32);
        bicost = tld.me.bufSATD(ref, X265_LOWRES_CU_SIZE);
        COPY2_IF_LT(bcost, bicost, listused, 3);
        bcost += lowresPenalty;
    }
    else /* P, also consider intra */
    {
        bcost += lowresPenalty;

        if (fenc->intraCost[cuXY] < bcost)
        {
            bcost = fenc->intraCost[cuXY];
            listused = 0;
        }
    }

    /* do not include edge blocks in the frame cost estimates, they are not very accurate */
    const bool bFrameScoreCU = (cuX > 0 && cuX < widthInCU - 1 &&
                                cuY > 0 && cuY < heightInCU - 1) || widthInCU <= 2 || heightInCU <= 2;
    int bcostAq;
    if (m_lookahead.m_param->rc.qgSize == 8)
        bcostAq = (bFrameScoreCU && fenc->invQscaleFactor) ? ((bcost * fenc->invQscaleFactor8x8[cuXY] + 128) >> 8) : bcost;
    else
        bcostAq = (bFrameScoreCU && fenc->invQscaleFactor) ? ((bcost * fenc->invQscaleFactor[cuXY] +128) >> 8) : bcost;

    if (bFrameScoreCU)
    {
        if (slice < 0)
        {
            fenc->costEst[b - p0][p1 - b] += bcost;
            fenc->costEstAq[b - p0][p1 - b] += bcostAq;
            if (!listused && !bBidir)
                fenc->intraMbs[b - p0]++;
        }
        else
        {
            m_slice[slice].costEst += bcost;
            m_slice[slice].costEstAq += bcostAq;
            if (!listused && !bBidir)
                m_slice[slice].intraMbs++;
        }
    }

    fenc->rowSatds[b - p0][p1 - b][cuY] += bcostAq;
    fenc->lowresCosts[b - p0][p1 - b][cuXY] = (uint16_t)(X265_MIN(bcost, LOWRES_COST_MASK) | (listused << LOWRES_COST_SHIFT));
}
