/*****************************************************************************
 * Copyright (C) 2013-2017 MulticoreWare, Inc
 *
 * Authors: Steve Borho <steve@borho.org>
 *          Min Chen <chenm003@163.com>
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
#include "primitives.h"
#include "threadpool.h"
#include "param.h"
#include "frame.h"
#include "framedata.h"
#include "picyuv.h"

#include "bitcost.h"
#include "encoder.h"
#include "slicetype.h"
#include "dpb.h"

#include "x265.h"

#if _MSC_VER
#pragma warning(disable: 4996) // POSIX functions are just fine, thanks
#endif

namespace X265_NS {
const char g_sliceTypeToChar[] = {'B', 'P', 'I'};
}

/* Threshold for motion vection, based on expermental result.
 * TODO: come up an algorithm for adoptive threshold */
#define MVTHRESHOLD (10*10)
#define PU_2Nx2N 1
static const char* defaultAnalysisFileName = "x265_analysis.dat";

using namespace X265_NS;

inline char *strcatFilename(const char *input, const char *suffix)
{
    char *output = X265_MALLOC(char, strlen(input) + strlen(suffix) + 1);
    if (!output)
    {
        x265_log(NULL, X265_LOG_ERROR, "unable to allocate memory for filename\n");
        return NULL;
    }
    strcpy(output, input);
    strcat(output, suffix);
    return output;
}

#if defined(_MSC_VER)
#pragma warning(disable: 4800) // forcing int to bool
#pragma warning(disable: 4127) // conditional expression is constant
#endif


LookEncoder::LookEncoder()
{
    m_aborted = false;
    m_reconfigure = false;
    m_reconfigureRc = false;
    m_pocLast = -1;
    m_lookahead = NULL;
    m_dpb = NULL;
    m_numDelayedPic = 0;
    m_param = NULL;
    m_latestParam = NULL;
    m_threadPool = NULL;
    m_offsetEmergency = NULL;
    MotionEstimate::initScales();

    m_prevTonemapPayload.payload = NULL;
}

void LookEncoder::create()
{
    if (!primitives.pu[0].sad)
    {
        // this should be an impossible condition when using our public API, and indicates a serious bug.
        x265_log(m_param, X265_LOG_ERROR, "Primitives must be initialized before encoder is created\n");
        abort();
    }

    x265_param* p = m_param;
    bool allowPools = !p->numaPools || strcmp(p->numaPools, "none");

    m_numPools = 0;
    if (allowPools)
        m_threadPool = ThreadPool::allocThreadPools(p, m_numPools, 0);
    else
    {
        if (!p->frameNumThreads)
        {
            // auto-detect frame threads
            int cpuCount = ThreadPool::getCpuCount();
            ThreadPool::getFrameThreadsCount(p, cpuCount);
        }
    }

    if (!m_numPools)
    {
        // issue warnings if any of these features were requested
        if (p->bDistributeMotionEstimation)
            x265_log(p, X265_LOG_WARNING, "No thread pool allocated, --pme disabled\n");
        if (p->bDistributeModeAnalysis)
            x265_log(p, X265_LOG_WARNING, "No thread pool allocated, --pmode disabled\n");
        if (p->lookaheadSlices)
            x265_log(p, X265_LOG_WARNING, "No thread pool allocated, --lookahead-slices disabled\n");

    }

    x265_log(p, X265_LOG_INFO, "Slices                              : %d\n", p->maxSlices);

    char buf[128];
    int len = 0;
    if (p->bDistributeModeAnalysis)
        len += sprintf(buf + len, "%spmode", len ? "+" : "");
    if (p->bDistributeMotionEstimation)
        len += sprintf(buf + len, "%spme ", len ? "+" : "");
    if (!len)
        strcpy(buf, "none");

    x265_log(p, X265_LOG_INFO, "frame threads / pool features       : %d / %s\n", p->frameNumThreads, buf);

    if (!m_scalingList.init())// 初始化量化中所需要的几个表格
    {
        x265_log(m_param, X265_LOG_ERROR, "Unable to allocate scaling list arrays\n");
        m_aborted = true;
        return;
    }
    else if (!m_param->scalingLists || !strcmp(m_param->scalingLists, "off"))
        m_scalingList.m_bEnabled = false;
    else if (!strcmp(m_param->scalingLists, "default"))
        m_scalingList.setDefaultScalingList();
    else if (m_scalingList.parseScalingList(m_param->scalingLists))
        m_aborted = true;
    int pools = m_numPools;
    ThreadPool* lookAheadThreadPool = 0;
    if (m_param->lookaheadThreads > 0)
    {
        lookAheadThreadPool = ThreadPool::allocThreadPools(p, pools, 1);
    }
    else
        lookAheadThreadPool = m_threadPool;
    m_lookahead = new Lookahead(m_param, lookAheadThreadPool);//初始化lookachead用于帧类型决策
    if (pools)
    {
        m_lookahead->m_jpId = lookAheadThreadPool[0].m_numProviders++;
        lookAheadThreadPool[0].m_jpTable[m_lookahead->m_jpId] = m_lookahead;
    }
    if (m_param->lookaheadThreads > 0)
        for (int i = 0; i < pools; i++)
            lookAheadThreadPool[i].start();
    m_lookahead->m_numPools = pools;
    m_dpb = new DPB(m_param);
    //m_rateControl = new RateControl(*m_param);
    initVPS(&m_vps);
    initSPS(&m_sps);
    initPPS(&m_pps);

    if (m_param->rc.vbvBufferSize)
    {
        m_offsetEmergency = (uint16_t(*)[MAX_NUM_TR_CATEGORIES][MAX_NUM_TR_COEFFS])X265_MALLOC(uint16_t, MAX_NUM_TR_CATEGORIES * MAX_NUM_TR_COEFFS * (QP_MAX_MAX - QP_MAX_SPEC));
        if (!m_offsetEmergency)
        {
            x265_log(m_param, X265_LOG_ERROR, "Unable to allocate memory\n");
            m_aborted = true;
            return;
        }

        bool scalingEnabled = m_scalingList.m_bEnabled;
        if (!scalingEnabled)
        {
            m_scalingList.setDefaultScalingList();
            m_scalingList.setupQuantMatrices(m_sps.chromaFormatIdc);
        }
        else
            m_scalingList.setupQuantMatrices(m_sps.chromaFormatIdc);

        for (int q = 0; q < QP_MAX_MAX - QP_MAX_SPEC; q++)
        {
            for (int cat = 0; cat < MAX_NUM_TR_CATEGORIES; cat++)
            {
                uint16_t *nrOffset = m_offsetEmergency[q][cat];

                int trSize = cat & 3;

                int coefCount = 1 << ((trSize + 2) * 2);

                /* Denoise chroma first then luma, then DC. */
                int dcThreshold = (QP_MAX_MAX - QP_MAX_SPEC) * 2 / 3;
                int lumaThreshold = (QP_MAX_MAX - QP_MAX_SPEC) * 2 / 3;
                int chromaThreshold = 0;

                int thresh = (cat < 4 || (cat >= 8 && cat < 12)) ? lumaThreshold : chromaThreshold;

                double quantF = (double)(1ULL << (q / 6 + 16 + 8));

                for (int i = 0; i < coefCount; i++)
                {
                    /* True "emergency mode": remove all DCT coefficients */
                    if (q == QP_MAX_MAX - QP_MAX_SPEC - 1)
                    {
                        nrOffset[i] = INT16_MAX;
                        continue;
                    }

                    int iThresh = i == 0 ? dcThreshold : thresh;
                    if (q < iThresh)
                    {
                        nrOffset[i] = 0;
                        continue;
                    }

                    int numList = (cat >= 8) * 3 + ((int)!iThresh);

                    double pos = (double)(q - iThresh + 1) / (QP_MAX_MAX - QP_MAX_SPEC - iThresh);
                    double start = quantF / (m_scalingList.m_quantCoef[trSize][numList][QP_MAX_SPEC % 6][i]);

                    // Formula chosen as an exponential scale to vaguely mimic the effects of a higher quantizer.
                    double bias = (pow(2, pos * (QP_MAX_MAX - QP_MAX_SPEC)) * 0.003 - 0.003) * start;
                    nrOffset[i] = (uint16_t)X265_MIN(bias + 0.5, INT16_MAX);
                }
            }
        }

        if (!scalingEnabled)
        {
            m_scalingList.m_bEnabled = false;
            m_scalingList.m_bDataPresent = false;
            m_scalingList.setupQuantMatrices(m_sps.chromaFormatIdc);
        }
    }
    else
        m_scalingList.setupQuantMatrices(m_sps.chromaFormatIdc);

    if (!m_lookahead->create())//申请lookachead空间用于帧类型决策
        m_aborted = true;

    m_aborted |= parseLambdaFile(m_param);

    m_encodeStartTime = x265_mdate();

    if (m_param->bDynamicRefine)
    {
        /* Allocate memory for 1 GOP and reuse it for the subsequent GOPs */
        int size = (m_param->keyframeMax + m_param->lookaheadDepth) * m_param->maxCUDepth * X265_REFINE_INTER_LEVELS;
        CHECKED_MALLOC_ZERO(m_variance, uint64_t, size);
        CHECKED_MALLOC_ZERO(m_rdCost, uint64_t, size);
        CHECKED_MALLOC_ZERO(m_trainingCount, uint32_t, size);
        return;
    fail:
        m_aborted = true;
    }
}

void LookEncoder::stopJobs()
{
    //if (m_rateControl)
    //    m_rateControl->terminate(); // unblock all blocked RC calls

    if (m_lookahead)
        m_lookahead->stopJobs();//停止帧类型决策任务，等它完毕再停止

    if (m_threadPool)
    {
        for (int i = 0; i < m_numPools; i++)
            m_threadPool[i].stopWorkers();
    }
}

void LookEncoder::destroy()
{
    if (m_param->bDynamicRefine)
    {
        X265_FREE(m_variance);
        X265_FREE(m_rdCost);
        X265_FREE(m_trainingCount);
    }

    // thread pools can be cleaned up now that all the JobProviders are
    // known to be shutdown
    delete[] m_threadPool;

    if (m_lookahead)
    {
        m_lookahead->destroy();//释放帧类型决策类内存
        delete m_lookahead;
    }

    delete m_dpb;

    X265_FREE(m_offsetEmergency);

    if (m_latestParam != NULL && m_latestParam != m_param)
    {
        if (m_latestParam->scalingLists != m_param->scalingLists)
            free((char*)m_latestParam->scalingLists);

        PARAM_NS::x265_param_free(m_latestParam);
    }

    if (m_param)
    {
        if (m_param->csvfpt)
            fclose(m_param->csvfpt);
        /* release string arguments that were strdup'd */
        free((char*)m_param->rc.lambdaFileName);
        free((char*)m_param->rc.statFileName);
        free((char*)m_param->analysisReuseFileName);
        free((char*)m_param->scalingLists);
        free((char*)m_param->csvfn);
        free((char*)m_param->numaPools);
        free((char*)m_param->masteringDisplayColorVolume);
        free((char*)m_param->toneMapFile);
        free((char*)m_param->analysisSave);
        free((char*)m_param->analysisLoad);
        PARAM_NS::x265_param_free(m_param);
    }
}

int LookEncoder::encode_lookahead(const x265_picture* pic_in)
{
#if CHECKED_BUILD || _DEBUG
    if (g_checkFailures)
    {
        x265_log(m_param, X265_LOG_ERROR, "encoder aborting because of internal error\n");
        return -1;
    }
#endif
    if (m_aborted)
        return -1;

    if (pic_in) //如果当前有读入帧 （有可能已经读完原始帧，但是lookachead buffer里面依然有待编码帧）
    {
        if (m_latestParam->forceFlush == 1)
        {
            m_lookahead->setLookaheadQueue();
            m_latestParam->forceFlush = 0;
        }
        if (m_latestParam->forceFlush == 2)
        {
            m_lookahead->m_filled = false;
            m_latestParam->forceFlush = 0;
        }

        x265_sei_payload toneMap;
        toneMap.payload = NULL;

        if (pic_in->bitDepth < 8 || pic_in->bitDepth > 16)//检错像素深度
        {
            x265_log(m_param, X265_LOG_ERROR, "Input bit depth (%d) must be between 8 and 16\n",
                pic_in->bitDepth);
            return -1;
        }

        Frame *inFrame;//即将create 用于存储视频帧
        if (m_dpb->m_freeList.empty())
        {
            inFrame = new Frame;//申请空间
            inFrame->m_encodeStartTime = x265_mdate();
            x265_param* p = (m_reconfigure || m_reconfigureRc) ? m_latestParam : m_param;//选择新的配置文件
            if (inFrame->create(p, pic_in->quantOffsets))//申请frame空间
            {
                static int count = 0;
                //printf("m_dpb->m_freeList.empty()--count = %d-\n", count++);
                /* the first PicYuv created is asked to generate the CU and block unit offset
                * arrays which are then shared with all subsequent PicYuv (orig and recon)
                * allocated by this top level encoder */
                if (m_sps.cuOffsetY)//已经申请过空间，不用再进入else 将encoder offset指针赋值到对应m_fencPic对象中
                {
                    inFrame->m_fencPic->m_cuOffsetY = m_sps.cuOffsetY;
                    inFrame->m_fencPic->m_buOffsetY = m_sps.buOffsetY;
                    if (m_param->internalCsp != X265_CSP_I400)
                    {
                        inFrame->m_fencPic->m_cuOffsetC = m_sps.cuOffsetC;
                        inFrame->m_fencPic->m_buOffsetC = m_sps.buOffsetC;
                    }
                }
                else
                {
                    if (!inFrame->m_fencPic->createOffsets(m_sps))//申请偏移计算空间，一般不进入
                    {
                        m_aborted = true;
                        x265_log(m_param, X265_LOG_ERROR, "memory allocation failure, aborting encode\n");
                        inFrame->destroy();
                        delete inFrame;
                        return -1;
                    }
                    else
                    {
                        //申请内存正常 会进出入此：将encoder offset值置为对应m_fencPic对象中的指针值
                        m_sps.cuOffsetY = inFrame->m_fencPic->m_cuOffsetY;
                        m_sps.buOffsetY = inFrame->m_fencPic->m_buOffsetY;
                        if (m_param->internalCsp != X265_CSP_I400)
                        {
                            m_sps.cuOffsetC = inFrame->m_fencPic->m_cuOffsetC;//空间为一帧LCU个数，按照行列对应色度LCU的pixel地址
                            m_sps.cuOffsetY = inFrame->m_fencPic->m_cuOffsetY;//空间为一帧LCU个数，按照行列对应亮度LCU的pixel地址
                            m_sps.buOffsetC = inFrame->m_fencPic->m_buOffsetC;//空间为一个LCU的part个数（默认256个4x4），为当前色度位置与LCU首地址的偏移地址
                            m_sps.buOffsetY = inFrame->m_fencPic->m_buOffsetY;//空间为一个LCU的part个数（默认256个4x4），为当前亮度位置与LCU首地址的偏移地址
                        }
                    }
                }
            }
            else//报错信息，正常不会进入
            {
                m_aborted = true;
                x265_log(m_param, X265_LOG_ERROR, "memory allocation failure, aborting encode\n");
                inFrame->destroy();
                delete inFrame;
                return -1;
            }
        }
        else
        {
            inFrame = m_dpb->m_freeList.popBack();//直接从m_freeList中获取空间
            inFrame->m_encodeStartTime = x265_mdate();
            /* Set lowres scencut and satdCost here to aovid overwriting ANALYSIS_READ
            decision by lowres init*/
            inFrame->m_lowres.bScenecut = false;
            inFrame->m_lowres.satdCost = (int64_t)-1;
            inFrame->m_lowresInit = false;//标示未初始化
        }

        /* Copy input picture into a Frame and PicYuv, send to lookahead */
        inFrame->m_fencPic->copyFromPicture(*pic_in, *m_param, m_sps.conformanceWindow.rightOffset, m_sps.conformanceWindow.bottomOffset);

        inFrame->m_poc = ++m_pocLast;
        inFrame->m_userData = pic_in->userData;
        inFrame->m_pts = pic_in->pts;
        inFrame->m_forceqp = pic_in->forceqp;
        inFrame->m_param = (m_reconfigure || m_reconfigureRc) ? m_latestParam : m_param;

        int toneMapEnable = 0;
        if (m_bToneMap && toneMap.payload)
            toneMapEnable = 1;
        int numPayloads = pic_in->userSEI.numPayloads + toneMapEnable;
        inFrame->m_userSEI.numPayloads = numPayloads;

        if (inFrame->m_userSEI.numPayloads)
        {
            if (!inFrame->m_userSEI.payloads)
            {
                inFrame->m_userSEI.payloads = new x265_sei_payload[numPayloads];
                for (int i = 0; i < numPayloads; i++)
                    inFrame->m_userSEI.payloads[i].payload = NULL;
            }
            for (int i = 0; i < numPayloads; i++)
            {
                x265_sei_payload input;
                if ((i == (numPayloads - 1)) && toneMapEnable)
                    input = toneMap;
                else
                    input = pic_in->userSEI.payloads[i];
                int size = inFrame->m_userSEI.payloads[i].payloadSize = input.payloadSize;
                inFrame->m_userSEI.payloads[i].payloadType = input.payloadType;
                if (!inFrame->m_userSEI.payloads[i].payload)
                    inFrame->m_userSEI.payloads[i].payload = new uint8_t[size];
                memcpy(inFrame->m_userSEI.payloads[i].payload, input.payload, size);
            }
            if (toneMap.payload)
                x265_free(toneMap.payload);
        }

        if (pic_in->quantOffsets != NULL)
        {
            int cuCount;
            if (m_param->rc.qgSize == 8)
                cuCount = inFrame->m_lowres.maxBlocksInRowFullRes * inFrame->m_lowres.maxBlocksInColFullRes;
            else
                cuCount = inFrame->m_lowres.maxBlocksInRow * inFrame->m_lowres.maxBlocksInCol;
            memcpy(inFrame->m_quantOffsets, pic_in->quantOffsets, cuCount * sizeof(float));
        }

        if (m_pocLast == 0)
            m_firstPts = inFrame->m_pts;
        if (m_bframeDelay && m_pocLast == m_bframeDelay)
            m_bframeDelayTime = inFrame->m_pts - m_firstPts;

        /* Encoder holds a reference count until stats collection is finished */
        ATOMIC_INC(&inFrame->m_countRefEncoders);

        m_lookahead->addPicture(*inFrame, pic_in->sliceType);//将inFrame放入m_inputQueue中，满足条件时会唤醒工作线程

        m_numDelayedPic++;
    }
    else if (m_latestParam->forceFlush == 2) {
        m_lookahead->m_filled = true;
    }
    else {
        m_lookahead->flush();
    }

    int ret = 0;
    Frame* frameLookahead = m_lookahead->getDecidedPicture();
    if (frameLookahead)
    {
        frameLookahead->m_lowres.getData();
        ret = 2;
    }
    return ret;
}

void LookEncoder::printSummary()
{
    if (m_param->logLevel < X265_LOG_INFO)
        return;

    char buffer[200];
    int pWithB = 0;
    for (int i = 0; i <= m_param->bframes; i++)
        pWithB += m_lookahead->m_histogram[i];

    if (pWithB)
    {
        int p = 0;
        for (int i = 0; i <= m_param->bframes; i++)
            p += sprintf(buffer + p, "%.1f%% ", 100. * m_lookahead->m_histogram[i] / pWithB);

        x265_log(m_param, X265_LOG_INFO, "consecutive B-frames: %s\n", buffer);
    }

    general_log(m_param, NULL, X265_LOG_INFO, "\nencoded 0 frames\n");

}

void LookEncoder::initVPS(VPS *vps)
{
    /* Note that much of the VPS is initialized by determineLevel() */
    vps->ptl.progressiveSourceFlag = !m_param->interlaceMode;
    vps->ptl.interlacedSourceFlag = !!m_param->interlaceMode;
    vps->ptl.nonPackedConstraintFlag = false;
    vps->ptl.frameOnlyConstraintFlag = !m_param->interlaceMode;
}

void LookEncoder::initSPS(SPS *sps)
{
    sps->conformanceWindow = m_conformanceWindow;
    sps->chromaFormatIdc = m_param->internalCsp;
    sps->picWidthInLumaSamples = m_param->sourceWidth;
    sps->picHeightInLumaSamples = m_param->sourceHeight;
    sps->numCuInWidth = (m_param->sourceWidth + m_param->maxCUSize - 1) / m_param->maxCUSize;
    sps->numCuInHeight = (m_param->sourceHeight + m_param->maxCUSize - 1) / m_param->maxCUSize;
    sps->numCUsInFrame = sps->numCuInWidth * sps->numCuInHeight;
    sps->numPartitions = m_param->num4x4Partitions;
    sps->numPartInCUSize = 1 << m_param->unitSizeDepth;

    sps->log2MinCodingBlockSize = m_param->maxLog2CUSize - m_param->maxCUDepth;
    sps->log2DiffMaxMinCodingBlockSize = m_param->maxCUDepth;
    uint32_t maxLog2TUSize = (uint32_t)g_log2Size[m_param->maxTUSize];
    sps->quadtreeTULog2MaxSize = X265_MIN((uint32_t)m_param->maxLog2CUSize, maxLog2TUSize);
    sps->quadtreeTULog2MinSize = 2;
    sps->quadtreeTUMaxDepthInter = m_param->tuQTMaxInterDepth;
    sps->quadtreeTUMaxDepthIntra = m_param->tuQTMaxIntraDepth;

    sps->bUseSAO = m_param->bEnableSAO;

    sps->bUseAMP = m_param->bEnableAMP;
    sps->maxAMPDepth = m_param->bEnableAMP ? m_param->maxCUDepth : 0;

    sps->maxTempSubLayers = m_param->bEnableTemporalSubLayers ? 2 : 1;
    sps->maxDecPicBuffering = m_vps.maxDecPicBuffering;
    sps->numReorderPics = m_vps.numReorderPics;
    sps->maxLatencyIncrease = m_vps.maxLatencyIncrease = m_param->bframes;

    sps->bUseStrongIntraSmoothing = m_param->bEnableStrongIntraSmoothing;
    sps->bTemporalMVPEnabled = m_param->bEnableTemporalMvp;
    sps->bEmitVUITimingInfo = m_param->bEmitVUITimingInfo;
    sps->bEmitVUIHRDInfo = m_param->bEmitVUIHRDInfo;
    sps->log2MaxPocLsb = m_param->log2MaxPocLsb;
    int maxDeltaPOC = (m_param->bframes + 2) * (!!m_param->bBPyramid + 1) * 2;
    while ((1 << sps->log2MaxPocLsb) <= maxDeltaPOC * 2)
        sps->log2MaxPocLsb++;

    if (sps->log2MaxPocLsb != m_param->log2MaxPocLsb)
        x265_log(m_param, X265_LOG_WARNING, "Reset log2MaxPocLsb to %d to account for all POC values\n", sps->log2MaxPocLsb);

    VUI& vui = sps->vuiParameters;
    vui.aspectRatioInfoPresentFlag = !!m_param->vui.aspectRatioIdc;
    vui.aspectRatioIdc = m_param->vui.aspectRatioIdc;
    vui.sarWidth = m_param->vui.sarWidth;
    vui.sarHeight = m_param->vui.sarHeight;

    vui.overscanInfoPresentFlag = m_param->vui.bEnableOverscanInfoPresentFlag;
    vui.overscanAppropriateFlag = m_param->vui.bEnableOverscanAppropriateFlag;

    vui.videoSignalTypePresentFlag = m_param->vui.bEnableVideoSignalTypePresentFlag;
    vui.videoFormat = m_param->vui.videoFormat;
    vui.videoFullRangeFlag = m_param->vui.bEnableVideoFullRangeFlag;

    vui.colourDescriptionPresentFlag = m_param->vui.bEnableColorDescriptionPresentFlag;
    vui.colourPrimaries = m_param->vui.colorPrimaries;
    vui.transferCharacteristics = m_param->vui.transferCharacteristics;
    vui.matrixCoefficients = m_param->vui.matrixCoeffs;

    vui.chromaLocInfoPresentFlag = m_param->vui.bEnableChromaLocInfoPresentFlag;
    vui.chromaSampleLocTypeTopField = m_param->vui.chromaSampleLocTypeTopField;
    vui.chromaSampleLocTypeBottomField = m_param->vui.chromaSampleLocTypeBottomField;

    vui.defaultDisplayWindow.bEnabled = m_param->vui.bEnableDefaultDisplayWindowFlag;
    vui.defaultDisplayWindow.rightOffset = m_param->vui.defDispWinRightOffset;
    vui.defaultDisplayWindow.topOffset = m_param->vui.defDispWinTopOffset;
    vui.defaultDisplayWindow.bottomOffset = m_param->vui.defDispWinBottomOffset;
    vui.defaultDisplayWindow.leftOffset = m_param->vui.defDispWinLeftOffset;

    vui.frameFieldInfoPresentFlag = !!m_param->interlaceMode || (m_param->pictureStructure >= 0);
    vui.fieldSeqFlag = !!m_param->interlaceMode;

    vui.hrdParametersPresentFlag = m_param->bEmitHRDSEI;

    vui.timingInfo.numUnitsInTick = m_param->fpsDenom;
    vui.timingInfo.timeScale = m_param->fpsNum;
}

void LookEncoder::initPPS(PPS *pps)
{
    bool bIsVbv = m_param->rc.vbvBufferSize > 0 && m_param->rc.vbvMaxBitrate > 0;

    if (!m_param->bLossless && (m_param->rc.aqMode || bIsVbv || m_param->bAQMotion))
    {
        pps->bUseDQP = true;
        pps->maxCuDQPDepth = g_log2Size[m_param->maxCUSize] - g_log2Size[m_param->rc.qgSize];
        X265_CHECK(pps->maxCuDQPDepth <= 3, "max CU DQP depth cannot be greater than 3\n");
    }
    else
    {
        pps->bUseDQP = false;
        pps->maxCuDQPDepth = 0;
    }

    pps->chromaQpOffset[0] = m_param->cbQpOffset;
    pps->chromaQpOffset[1] = m_param->crQpOffset;
    pps->pps_slice_chroma_qp_offsets_present_flag = m_param->bHDROpt;

    pps->bConstrainedIntraPred = m_param->bEnableConstrainedIntra;
    pps->bUseWeightPred = m_param->bEnableWeightedPred;
    pps->bUseWeightedBiPred = m_param->bEnableWeightedBiPred;
    pps->bTransquantBypassEnabled = m_param->bCULossless || m_param->bLossless;
    pps->bTransformSkipEnabled = m_param->bEnableTransformSkip;
    pps->bSignHideEnabled = m_param->bEnableSignHiding;

    pps->bDeblockingFilterControlPresent = !m_param->bEnableLoopFilter || m_param->deblockingFilterBetaOffset || m_param->deblockingFilterTCOffset;
    pps->bPicDisableDeblockingFilter = !m_param->bEnableLoopFilter;
    pps->deblockingFilterBetaOffsetDiv2 = m_param->deblockingFilterBetaOffset;
    pps->deblockingFilterTcOffsetDiv2 = m_param->deblockingFilterTCOffset;

    pps->numRefIdxDefault[0] = 1;
    pps->numRefIdxDefault[1] = 1;
}

void LookEncoder::configure(x265_param *p)
{
    this->m_param = p;
    if (p->bMVType == AVC_INFO)
        this->m_externalFlush = true;
    else
        this->m_externalFlush = false;

    if (p->bMVType == AVC_INFO && (p->limitTU == 3 || p->limitTU == 4))
    {
        x265_log(p, X265_LOG_WARNING, "limit TU = 3 or 4 with MVType AVCINFO produces inconsistent output\n");
    }

    if (p->bMVType == AVC_INFO && p->minCUSize != 8)
    {
        p->minCUSize = 8;
        x265_log(p, X265_LOG_WARNING, "Setting minCuSize = 8, AVCINFO expects 8x8 blocks\n");
    }

    if (p->keyframeMax < 0)
    {
        /* A negative max GOP size indicates the user wants only one I frame at
        * the start of the stream. Set an infinite GOP distance and disable
        * adaptive I frame placement */
        p->keyframeMax = INT_MAX;
        p->scenecutThreshold = 0;
    }
    else if (p->keyframeMax <= 1)
    {
        p->keyframeMax = 1;

        // disable lookahead for all-intra encodes
        p->bFrameAdaptive = 0;
        p->bframes = 0;
        p->bOpenGOP = 0;
        p->bRepeatHeaders = 1;
        p->lookaheadDepth = 0;
        p->bframes = 0;
        p->scenecutThreshold = 0;
        p->bFrameAdaptive = 0;
        p->rc.cuTree = 0;
        p->bEnableWeightedPred = 0;
        p->bEnableWeightedBiPred = 0;
        p->bIntraRefresh = 0;

        /* SPSs shall have sps_max_dec_pic_buffering_minus1[ sps_max_sub_layers_minus1 ] equal to 0 only */
        p->maxNumReferences = 1;
    }
    if (!p->keyframeMin)
    {
        double fps = (double)p->fpsNum / p->fpsDenom;
        p->keyframeMin = X265_MIN((int)fps, p->keyframeMax / 10);
    }
    p->keyframeMin = X265_MAX(1, p->keyframeMin);

    if (!p->bframes)
        p->bBPyramid = 0;
    if (!p->rdoqLevel)
        p->psyRdoq = 0;

    /* Disable features which are not supported by the current RD level */
    if (p->rdLevel < 3)
    {
        if (p->bCULossless)             /* impossible */
            x265_log(p, X265_LOG_WARNING, "--cu-lossless disabled, requires --rdlevel 3 or higher\n");
        if (p->bEnableTransformSkip)    /* impossible */
            x265_log(p, X265_LOG_WARNING, "--tskip disabled, requires --rdlevel 3 or higher\n");
        p->bCULossless = p->bEnableTransformSkip = 0;
    }
    if (p->rdLevel < 2)
    {
        if (p->bDistributeModeAnalysis) /* not useful */
            x265_log(p, X265_LOG_WARNING, "--pmode disabled, requires --rdlevel 2 or higher\n");
        p->bDistributeModeAnalysis = 0;

        p->psyRd = 0;                   /* impossible */

        if (p->bEnableRectInter)        /* broken, not very useful */
            x265_log(p, X265_LOG_WARNING, "--rect disabled, requires --rdlevel 2 or higher\n");
        p->bEnableRectInter = 0;
    }

    if (!p->bEnableRectInter)          /* not useful */
        p->bEnableAMP = false;

    /* In 444, chroma gets twice as much resolution, so halve quality when psy-rd is enabled */
    if (p->internalCsp == X265_CSP_I444 && p->psyRd)
    {
        p->cbQpOffset += 6;
        p->crQpOffset += 6;
    }

    if (p->bLossless)
    {
        p->rc.rateControlMode = X265_RC_CQP;
        p->rc.qp = 4; // An oddity, QP=4 is more lossless than QP=0 and gives better lambdas
        p->bEnableSsim = 0;
        p->bEnablePsnr = 0;
    }

    if (p->rc.rateControlMode == X265_RC_CQP)
    {
        p->rc.aqMode = X265_AQ_NONE;
        p->rc.bitrate = 0;
        p->rc.cuTree = 0;
        p->rc.aqStrength = 0;
    }

    if (p->rc.aqMode == 0 && p->rc.cuTree)
    {
        p->rc.aqMode = X265_AQ_VARIANCE;
        p->rc.aqStrength = 0.0;
    }

    if (p->lookaheadDepth == 0 && p->rc.cuTree && !p->rc.bStatRead)
    {
        x265_log(p, X265_LOG_WARNING, "cuTree disabled, requires lookahead to be enabled\n");
        p->rc.cuTree = 0;
    }

    if (p->maxTUSize > p->maxCUSize)
    {
        x265_log(p, X265_LOG_WARNING, "Max TU size should be less than or equal to max CU size, setting max TU size = %d\n", p->maxCUSize);
        p->maxTUSize = p->maxCUSize;
    }

    if (p->rc.aqStrength == 0 && p->rc.cuTree == 0)
        p->rc.aqMode = X265_AQ_NONE;

    if (p->rc.aqMode == X265_AQ_NONE && p->rc.cuTree == 0)
        p->rc.aqStrength = 0;

    if (p->totalFrames && p->totalFrames <= 2 * ((float)p->fpsNum) / p->fpsDenom && p->rc.bStrictCbr)
        p->lookaheadDepth = p->totalFrames;
    if (p->bIntraRefresh)
    {
        int numCuInWidth = (m_param->sourceWidth + m_param->maxCUSize - 1) / m_param->maxCUSize;
        if (p->maxNumReferences > 1)
        {
            x265_log(p, X265_LOG_WARNING, "Max References > 1 + intra-refresh is not supported , setting max num references = 1\n");
            p->maxNumReferences = 1;
        }

        if (p->bBPyramid && p->bframes)
            x265_log(p, X265_LOG_WARNING, "B pyramid cannot be enabled when max references is 1, Disabling B pyramid\n");
        p->bBPyramid = 0;


        if (p->bOpenGOP)
        {
            x265_log(p, X265_LOG_WARNING, "Open Gop disabled, Intra Refresh is not compatible with openGop\n");
            p->bOpenGOP = 0;
        }

        x265_log(p, X265_LOG_WARNING, "Scenecut is disabled when Intra Refresh is enabled\n");

        if (((float)numCuInWidth - 1) / m_param->keyframeMax > 1)
            x265_log(p, X265_LOG_WARNING, "Keyint value is very low.It leads to frequent intra refreshes, can be almost every frame."
                "Prefered use case would be high keyint value or an API call to refresh when necessary\n");

    }


    if (p->interlaceMode)
        x265_log(p, X265_LOG_WARNING, "Support for interlaced video is experimental\n");

    if (p->rc.rfConstantMin > p->rc.rfConstant)
    {
        x265_log(m_param, X265_LOG_WARNING, "CRF min must be less than CRF\n");
        p->rc.rfConstantMin = 0;
    }

    if ((p->analysisLoad || p->analysisSave) && (p->bDistributeModeAnalysis || p->bDistributeMotionEstimation))
    {
        x265_log(p, X265_LOG_WARNING, "Analysis load/save options incompatible with pmode/pme, Disabling pmode/pme\n");
        p->bDistributeMotionEstimation = p->bDistributeModeAnalysis = 0;
    }

    if ((p->analysisLoad || p->analysisSave) && p->rc.cuTree)
    {
        x265_log(p, X265_LOG_WARNING, "Analysis load/save options works only with cu-tree off, Disabling cu-tree\n");
        p->rc.cuTree = 0;
    }

    if ((p->analysisLoad || p->analysisSave) && (p->analysisMultiPassRefine || p->analysisMultiPassDistortion))
    {
        x265_log(p, X265_LOG_WARNING, "Cannot use Analysis load/save option and multi-pass-opt-analysis/multi-pass-opt-distortion together,"
            "Disabling Analysis load/save and multi-pass-opt-analysis/multi-pass-opt-distortion\n");
        p->analysisSave = p->analysisLoad = NULL;
        p->analysisMultiPassRefine = p->analysisMultiPassDistortion = 0;
    }
    if (p->scaleFactor)
    {
        if (p->scaleFactor == 1)
        {
            p->scaleFactor = 0;
        }
        else if ((!p->analysisLoad && !p->analysisSave) || p->analysisReuseLevel < 10)
        {
            x265_log(p, X265_LOG_WARNING, "Input scaling works with analysis load/save, analysis-reuse-level 10. Disabling scale-factor.\n");
            p->scaleFactor = 0;
        }
    }

    if (p->intraRefine)
    {
        if (!p->analysisLoad || p->analysisReuseLevel < 10)
        {
            x265_log(p, X265_LOG_WARNING, "Intra refinement requires analysis load, analysis-reuse-level 10. Disabling intra refine.\n");
            p->intraRefine = 0;
        }
    }

    if (p->interRefine)
    {
        if (!p->analysisLoad || p->analysisReuseLevel < 10)
        {
            x265_log(p, X265_LOG_WARNING, "Inter refinement requires analysis load, analysis-reuse-level 10. Disabling inter refine.\n");
            p->interRefine = 0;
        }
    }

    if (p->bDynamicRefine)
    {
        if (!p->analysisLoad || p->analysisReuseLevel < 10)
        {
            x265_log(p, X265_LOG_WARNING, "Dynamic refinement requires analysis load, analysis-reuse-level 10. Disabling dynamic refine.\n");
            p->bDynamicRefine = 0;
        }
        if (p->interRefine)
        {
            x265_log(p, X265_LOG_WARNING, "Inter refine cannot be used with dynamic refine. Disabling refine-inter.\n");
            p->interRefine = 0;
        }
    }
    if (p->scaleFactor && p->analysisLoad && !p->interRefine && !p->bDynamicRefine)
    {
        x265_log(p, X265_LOG_WARNING, "Inter refinement 0 is not supported with scaling. Enabling refine-inter 1.\n");
        p->interRefine = 1;
    }

    if (p->limitTU && (p->interRefine || p->bDynamicRefine))
    {
        x265_log(p, X265_LOG_WARNING, "Inter refinement does not support limitTU. Disabling limitTU.\n");
        p->limitTU = 0;
    }

    if (p->mvRefine)
    {
        if (!p->analysisLoad || p->analysisReuseLevel < 10)
        {
            x265_log(p, X265_LOG_WARNING, "MV refinement requires analysis load, analysis-reuse-level 10. Disabling MV refine.\n");
            p->mvRefine = 0;
        }
        else if (p->interRefine >= 2)
        {
            x265_log(p, X265_LOG_WARNING, "MVs are recomputed when refine-inter >= 2. MV refinement not applicable. Disabling MV refine\n");
            p->mvRefine = 0;
        }
    }

    if ((p->analysisMultiPassRefine || p->analysisMultiPassDistortion) && (p->bDistributeModeAnalysis || p->bDistributeMotionEstimation))
    {
        x265_log(p, X265_LOG_WARNING, "multi-pass-opt-analysis/multi-pass-opt-distortion incompatible with pmode/pme, Disabling pmode/pme\n");
        p->bDistributeMotionEstimation = p->bDistributeModeAnalysis = 0;
    }

    if (p->rc.bEnableGrain)
    {
        x265_log(p, X265_LOG_WARNING, "Rc Grain removes qp fluctuations caused by aq/cutree, Disabling aq,cu-tree\n");
        p->rc.cuTree = 0;
        p->rc.aqMode = 0;
    }

    if (p->bDistributeModeAnalysis && (p->limitReferences >> 1) && 1)
    {
        x265_log(p, X265_LOG_WARNING, "Limit reference options 2 and 3 are not supported with pmode. Disabling limit reference\n");
        p->limitReferences = 0;
    }

    if (p->bEnableTemporalSubLayers && !p->bframes)
    {
        x265_log(p, X265_LOG_WARNING, "B frames not enabled, temporal sublayer disabled\n");
        p->bEnableTemporalSubLayers = 0;
    }

    m_bframeDelay = p->bframes ? (p->bBPyramid ? 2 : 1) : 0;

    p->bFrameBias = X265_MIN(X265_MAX(-90, p->bFrameBias), 100);
    p->scenecutBias = (double)(p->scenecutBias / 100);

    if (p->logLevel < X265_LOG_INFO)
    {
        /* don't measure these metrics if they will not be reported */
        p->bEnablePsnr = 0;
        p->bEnableSsim = 0;
    }
    /* Warn users trying to measure PSNR/SSIM with psy opts on. */
    if (p->bEnablePsnr || p->bEnableSsim)
    {
        const char *s = NULL;

        if (p->psyRd || p->psyRdoq)
        {
            s = p->bEnablePsnr ? "psnr" : "ssim";
            x265_log(p, X265_LOG_WARNING, "--%s used with psy on: results will be invalid!\n", s);
        }
        else if (!p->rc.aqMode && p->bEnableSsim)
        {
            x265_log(p, X265_LOG_WARNING, "--ssim used with AQ off: results will be invalid!\n");
            s = "ssim";
        }
        else if (p->rc.aqStrength > 0 && p->bEnablePsnr)
        {
            x265_log(p, X265_LOG_WARNING, "--psnr used with AQ on: results will be invalid!\n");
            s = "psnr";
        }
        if (s)
            x265_log(p, X265_LOG_WARNING, "--tune %s should be used if attempting to benchmark %s!\n", s, s);
    }
    if (p->searchMethod == X265_SEA && (p->bDistributeMotionEstimation || p->bDistributeModeAnalysis))
    {
        x265_log(p, X265_LOG_WARNING, "Disabling pme and pmode: --pme and --pmode cannot be used with SEA motion search!\n");
        p->bDistributeMotionEstimation = 0;
        p->bDistributeModeAnalysis = 0;
    }

    if (!p->rc.bStatWrite && !p->rc.bStatRead && (p->analysisMultiPassRefine || p->analysisMultiPassDistortion))
    {
        x265_log(p, X265_LOG_WARNING, "analysis-multi-pass/distortion is enabled only when rc multi pass is enabled. Disabling multi-pass-opt-analysis and multi-pass-opt-distortion");
        p->analysisMultiPassRefine = 0;
        p->analysisMultiPassDistortion = 0;
    }
    if (p->analysisMultiPassRefine && p->rc.bStatWrite && p->rc.bStatRead)
    {
        x265_log(p, X265_LOG_WARNING, "--multi-pass-opt-analysis doesn't support refining analysis through multiple-passes; it only reuses analysis from the second-to-last pass to the last pass.Disabling reading\n");
        p->rc.bStatRead = 0;
    }

    /* some options make no sense if others are disabled */
    p->bSaoNonDeblocked &= p->bEnableSAO;
    p->bEnableTSkipFast &= p->bEnableTransformSkip;
    p->bLimitSAO &= p->bEnableSAO;
    /* initialize the conformance window */
    m_conformanceWindow.bEnabled = false;
    m_conformanceWindow.rightOffset = 0;
    m_conformanceWindow.topOffset = 0;
    m_conformanceWindow.bottomOffset = 0;
    m_conformanceWindow.leftOffset = 0;
    /* set pad size if width is not multiple of the minimum CU size */
    if (p->scaleFactor == 2 && ((p->sourceWidth / 2) & (p->minCUSize - 1)) && p->analysisLoad)
    {
        uint32_t rem = (p->sourceWidth / 2) & (p->minCUSize - 1);
        uint32_t padsize = p->minCUSize - rem;
        p->sourceWidth += padsize * 2;

        m_conformanceWindow.bEnabled = true;
        m_conformanceWindow.rightOffset = padsize * 2;
    }
    else if (p->sourceWidth & (p->minCUSize - 1))
    {
        uint32_t rem = p->sourceWidth & (p->minCUSize - 1);
        uint32_t padsize = p->minCUSize - rem;
        p->sourceWidth += padsize;

        m_conformanceWindow.bEnabled = true;
        m_conformanceWindow.rightOffset = padsize;
    }

    if (p->bEnableRdRefine && (p->rdLevel < 5 || !p->rc.aqMode))
    {
        p->bEnableRdRefine = false;
        x265_log(p, X265_LOG_WARNING, "--rd-refine disabled, requires RD level > 4 and adaptive quant\n");
    }

    if (p->bOptCUDeltaQP && p->rdLevel < 5)
    {
        p->bOptCUDeltaQP = false;
        x265_log(p, X265_LOG_WARNING, "--opt-cu-delta-qp disabled, requires RD level > 4\n");
    }

    if (p->limitTU && p->tuQTMaxInterDepth < 2)
    {
        p->limitTU = 0;
        x265_log(p, X265_LOG_WARNING, "limit-tu disabled, requires tu-inter-depth > 1\n");
    }
    bool bIsVbv = m_param->rc.vbvBufferSize > 0 && m_param->rc.vbvMaxBitrate > 0;
    if (!m_param->bLossless && (m_param->rc.aqMode || bIsVbv || m_param->bAQMotion))
    {
        if (p->rc.qgSize < X265_MAX(8, p->minCUSize))
        {
            p->rc.qgSize = X265_MAX(8, p->minCUSize);
            x265_log(p, X265_LOG_WARNING, "QGSize should be greater than or equal to 8 and minCUSize, setting QGSize = %d\n", p->rc.qgSize);
        }
        if (p->rc.qgSize > p->maxCUSize)
        {
            p->rc.qgSize = p->maxCUSize;
            x265_log(p, X265_LOG_WARNING, "QGSize should be less than or equal to maxCUSize, setting QGSize = %d\n", p->rc.qgSize);
        }
    }
    else
        m_param->rc.qgSize = p->maxCUSize;

    if (m_param->dynamicRd && (!bIsVbv || !p->rc.aqMode || p->rdLevel > 4))
    {
        p->dynamicRd = 0;
        x265_log(p, X265_LOG_WARNING, "Dynamic-rd disabled, requires RD <= 4, VBV and aq-mode enabled\n");
    }
    if (m_param->toneMapFile)
    {
        x265_log(p, X265_LOG_WARNING, "--dhdr10-info disabled. Enable HDR10_PLUS in cmake.\n");
        m_bToneMap = 0;
        m_param->toneMapFile = NULL;
    }
    else if (m_param->bDhdr10opt)
    {
        x265_log(p, X265_LOG_WARNING, "Disabling dhdr10-opt. dhdr10-info must be enabled.\n");
        m_param->bDhdr10opt = 0;
    }

    if (p->uhdBluray)
    {
        p->bEnableAccessUnitDelimiters = 1;
        p->vui.aspectRatioIdc = 1;
        p->bEmitHRDSEI = 1;
        int disableUhdBd = 0;

        if (p->levelIdc && p->levelIdc != 51)
        {
            x265_log(p, X265_LOG_WARNING, "uhd-bd: Wrong level specified, UHD Bluray mandates Level 5.1\n");
        }
        p->levelIdc = 51;

        if (!p->bHighTier)
        {
            x265_log(p, X265_LOG_WARNING, "uhd-bd: Turning on high tier\n");
            p->bHighTier = 1;
        }

        if (!p->bRepeatHeaders)
        {
            x265_log(p, X265_LOG_WARNING, "uhd-bd: Turning on repeat-headers\n");
            p->bRepeatHeaders = 1;
        }

        if (p->bOpenGOP)
        {
            x265_log(p, X265_LOG_WARNING, "uhd-bd: Turning off open GOP\n");
            p->bOpenGOP = false;
        }

        if (p->bIntraRefresh)
        {
            x265_log(p, X265_LOG_WARNING, "uhd-bd: turning off intra-refresh\n");
            p->bIntraRefresh = 0;
        }

        if (p->keyframeMin != 1)
        {
            x265_log(p, X265_LOG_WARNING, "uhd-bd: keyframeMin is always 1\n");
            p->keyframeMin = 1;
        }

        int fps = (p->fpsNum + p->fpsDenom - 1) / p->fpsDenom;
        if (p->keyframeMax > fps)
        {
            x265_log(p, X265_LOG_WARNING, "uhd-bd: reducing keyframeMax to %d\n", fps);
            p->keyframeMax = fps;
        }

        if (p->maxNumReferences > 6)
        {
            x265_log(p, X265_LOG_WARNING, "uhd-bd: reducing references to 6\n");
            p->maxNumReferences = 6;
        }

        if (p->bEnableTemporalSubLayers)
        {
            x265_log(p, X265_LOG_WARNING, "uhd-bd: Turning off temporal layering\n");
            p->bEnableTemporalSubLayers = 0;
        }

        if (p->vui.colorPrimaries != 1 && p->vui.colorPrimaries != 9)
        {
            x265_log(p, X265_LOG_ERROR, "uhd-bd: colour primaries should be either BT.709 or BT.2020\n");
            disableUhdBd = 1;
        }
        else if (p->vui.colorPrimaries == 9)
        {
            p->vui.bEnableChromaLocInfoPresentFlag = 1;
            p->vui.chromaSampleLocTypeTopField = 2;
            p->vui.chromaSampleLocTypeBottomField = 2;
        }

        if (p->vui.transferCharacteristics != 1 && p->vui.transferCharacteristics != 14 && p->vui.transferCharacteristics != 16)
        {
            x265_log(p, X265_LOG_ERROR, "uhd-bd: transfer characteristics supported are BT.709, BT.2020-10 or SMPTE ST.2084\n");
            disableUhdBd = 1;
        }
        if (p->vui.matrixCoeffs != 1 && p->vui.matrixCoeffs != 9)
        {
            x265_log(p, X265_LOG_ERROR, "uhd-bd: matrix coeffs supported are either BT.709 or BT.2020\n");
            disableUhdBd = 1;
        }
        if ((p->sourceWidth != 1920 && p->sourceWidth != 3840) || (p->sourceHeight != 1080 && p->sourceHeight != 2160))
        {
            x265_log(p, X265_LOG_ERROR, "uhd-bd: Supported resolutions are 1920x1080 and 3840x2160\n");
            disableUhdBd = 1;
        }
        if (disableUhdBd)
        {
            p->uhdBluray = 0;
            x265_log(p, X265_LOG_ERROR, "uhd-bd: Disabled\n");
        }
    }
    /* set pad size if height is not multiple of the minimum CU size */
    if (p->scaleFactor == 2 && ((p->sourceHeight / 2) & (p->minCUSize - 1)) && p->analysisLoad)
    {
        uint32_t rem = (p->sourceHeight / 2) & (p->minCUSize - 1);
        uint32_t padsize = p->minCUSize - rem;
        p->sourceHeight += padsize * 2;
        m_conformanceWindow.bEnabled = true;
        m_conformanceWindow.bottomOffset = padsize * 2;
    }
    else if (p->sourceHeight & (p->minCUSize - 1))
    {
        uint32_t rem = p->sourceHeight & (p->minCUSize - 1);
        uint32_t padsize = p->minCUSize - rem;
        p->sourceHeight += padsize;
        m_conformanceWindow.bEnabled = true;
        m_conformanceWindow.bottomOffset = padsize;
    }

    if (p->bLogCuStats)
        x265_log(p, X265_LOG_WARNING, "--cu-stats option is now deprecated\n");

    if (p->log2MaxPocLsb < 4)
    {
        x265_log(p, X265_LOG_WARNING, "maximum of the picture order count can not be less than 4\n");
        p->log2MaxPocLsb = 4;
    }

    if (p->maxSlices < 1)
    {
        x265_log(p, X265_LOG_WARNING, "maxSlices can not be less than 1, force set to 1\n");
        p->maxSlices = 1;
    }
    const uint32_t numRows = (p->sourceHeight + p->maxCUSize - 1) / p->maxCUSize;
    //const uint32_t slicesLimit = X265_MIN(numRows, NALList::MAX_NAL_UNITS - 1);
    //if (p->maxSlices > slicesLimit)
    //{
    //    x265_log(p, X265_LOG_WARNING, "maxSlices can not be more than min(rows, MAX_NAL_UNITS-1), force set to %d\n", slicesLimit);
    //    p->maxSlices = slicesLimit;
    //}
    if (p->bHDROpt)
    {
        if (p->internalCsp != X265_CSP_I420 || p->internalBitDepth != 10 || p->vui.colorPrimaries != 9 ||
            p->vui.transferCharacteristics != 16 || p->vui.matrixCoeffs != 9)
        {
            x265_log(p, X265_LOG_ERROR, "Recommended Settings for HDR: colour primaries should be BT.2020,\n"
                "                                            transfer characteristics should be SMPTE ST.2084,\n"
                "                                            matrix coeffs should be BT.2020,\n"
                "                                            the input video should be 10 bit 4:2:0\n"
                "                                            Disabling offset tuning for HDR videos\n");
            p->bHDROpt = 0;
        }
    }

    if (m_param->toneMapFile || p->bHDROpt || p->bEmitHDRSEI)
    {
        if (!p->bRepeatHeaders)
        {
            p->bRepeatHeaders = 1;
            x265_log(p, X265_LOG_WARNING, "Turning on repeat-headers for HDR compatibility\n");
        }
    }

    p->maxLog2CUSize = g_log2Size[p->maxCUSize];
    p->maxCUDepth = p->maxLog2CUSize - g_log2Size[p->minCUSize];
    p->unitSizeDepth = p->maxLog2CUSize - LOG2_UNIT_SIZE;
    p->num4x4Partitions = (1U << (p->unitSizeDepth << 1));

    if (p->radl && (p->keyframeMax != p->keyframeMin))
    {
        p->radl = 0;
        x265_log(p, X265_LOG_WARNING, "Radl requires fixed gop-length (keyint == min-keyint). Disabling radl.\n");
    }
}
