/*****************************************************************************
 * Copyright (C) 2013-2017 MulticoreWare, Inc
 *
 * Author: Shazeb Nawaz Khan <shazeb@multicorewareinc.com>
 *         Steve Borho <steve@borho.org>
 *         Kavitha Sampas <kavitha@multicorewareinc.com>
 *         Min Chen <chenm003@163.com>
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
#include "picyuv.h"
#include "lowres.h"
#include "slice.h"
#include "mv.h"
#include "bitstream.h"
#include "threading.h"

using namespace X265_NS;
namespace {
struct Cache
{
    const int * intraCost;
    int         numPredDir;
    int         csp;
    int         hshift;
    int         vshift;
    int         lowresWidthInCU;
    int         lowresHeightInCU;
};
/** 函数功能             ： 获取头信息占用的cost
/*  调用范围             ： 只在weightAnalyse函数中被调用
* \参数 w                ： 权重系数
* \参数 lambda           ： QP=12下的lambda
* \参数 bChroma          ： 如果是色度为true 否则为false
* \返回                  ： 头信息占用的cost * */
int sliceHeaderCost(WeightParam *w, int lambda, int bChroma)
{
    /* 4 times higher, because chroma is analyzed at full resolution. */
    if (bChroma)
        lambda *= 4;//色度为全搜素所以乘以四(1/2下采样 ，对应就是4块)
    int denomCost = bs_size_ue(w[0].log2WeightDenom) * (2 - bChroma);//获得指数哥伦布码占用的cost 
    return lambda * (10 + denomCost + 2 * (bs_size_se(w[0].inputWeight) + bs_size_se(w[0].inputOffset)));//返回头信息占用cost
}

/* make a motion compensated copy of lowres ref into mcout with the same stride.
 * The borders of mcout are not extended */
 /** 函数功能             ： 按照MV的信息将参考帧的块copy到mcout去
 /*  调用范围             ： 只在weightAnalyse函数中被调用
 * \参数 mcout            ： 存储MC buffer
 * \参数 ref              ： 参考帧对应的下采样视频
 * \参数 mvs              ： 8x8块搜索的MV
 * \返回                  ： null * */
void mcLuma(pixel* mcout, Lowres& ref, const MV * mvs)
{
    intptr_t stride = ref.lumaStride;
    const int mvshift = 1 << 2;
    const int cuSize = 8;
    MV mvmin, mvmax;

    int cu = 0;

    for (int y = 0; y < ref.lines; y += cuSize)
    {
        intptr_t pixoff = y * stride;
        mvmin.y = (int16_t)((-y - 8) * mvshift);
        mvmax.y = (int16_t)((ref.lines - y - 1 + 8) * mvshift);

        for (int x = 0; x < ref.width; x += cuSize, pixoff += cuSize, cu++)
        {
            ALIGN_VAR_16(pixel, buf8x8[8 * 8]);
            intptr_t bstride = 8;
            mvmin.x = (int16_t)((-x - 8) * mvshift);
            mvmax.x = (int16_t)((ref.width - x - 1 + 8) * mvshift);

            /* clip MV to available pixels */
            MV mv = mvs[cu];
            mv = mv.clipped(mvmin, mvmax);
            pixel *tmp = ref.lowresMC(pixoff, mv, buf8x8, bstride);
            primitives.cu[BLOCK_8x8].copy_pp(mcout + pixoff, stride, tmp, bstride);
        }
    }
}

/* use lowres MVs from lookahead to generate a motion compensated chroma plane.
 * if a block had cheaper lowres cost as intra, we treat it as MV 0 */
void mcChroma(pixel *      mcout,
              pixel *      src,
              intptr_t     stride,
              const MV *   mvs,
              const Cache& cache,
              int          height,
              int          width)
{
    /* the motion vectors correspond to 8x8 lowres luma blocks, or 16x16 fullres
     * luma blocks. We have to adapt block size to chroma csp */
    int csp = cache.csp;
    int bw = 16 >> cache.hshift;
    int bh = 16 >> cache.vshift;
    const int mvshift = 1 << 2;
    MV mvmin, mvmax;

    for (int y = 0; y < height; y += bh)
    {
        /* note: lowres block count per row might be different from chroma block
         * count per row because of rounding issues, so be very careful with indexing
         * into the lowres structures */
        int cu = y * cache.lowresWidthInCU;
        intptr_t pixoff = y * stride;
        mvmin.y = (int16_t)((-y - 8) * mvshift);
        mvmax.y = (int16_t)((height - y - 1 + 8) * mvshift);

        for (int x = 0; x < width; x += bw, cu++, pixoff += bw)
        {
            if (x < cache.lowresWidthInCU && y < cache.lowresHeightInCU)
            {
                MV mv = mvs[cu]; // lowres MV
                mv <<= 1;        // fullres MV
                mv.x >>= cache.hshift;
                mv.y >>= cache.vshift;

                /* clip MV to available pixels */
                mvmin.x = (int16_t)((-x - 8) * mvshift);
                mvmax.x = (int16_t)((width - x - 1 + 8) * mvshift);
                mv = mv.clipped(mvmin, mvmax);

                intptr_t fpeloffset = (mv.y >> 2) * stride + (mv.x >> 2);
                pixel *temp = src + pixoff + fpeloffset;

                int xFrac = mv.x & 7;
                int yFrac = mv.y & 7;
                if (!(yFrac | xFrac))
                {
                    primitives.chroma[csp].pu[LUMA_16x16].copy_pp(mcout + pixoff, stride, temp, stride);
                }
                else if (!yFrac)
                {
                    primitives.chroma[csp].pu[LUMA_16x16].filter_hpp(temp, stride, mcout + pixoff, stride, xFrac);
                }
                else if (!xFrac)
                {
                    primitives.chroma[csp].pu[LUMA_16x16].filter_vpp(temp, stride, mcout + pixoff, stride, yFrac);
                }
                else
                {
                    ALIGN_VAR_16(int16_t, immed[16 * (16 + NTAPS_CHROMA - 1)]);
                    primitives.chroma[csp].pu[LUMA_16x16].filter_hps(temp, stride, immed, bw, xFrac, 1);
                    primitives.chroma[csp].pu[LUMA_16x16].filter_vsp(immed + ((NTAPS_CHROMA >> 1) - 1) * bw, bw, mcout + pixoff, stride, yFrac);
                }
            }
            else
            {
                primitives.chroma[csp].pu[LUMA_16x16].copy_pp(mcout + pixoff, stride, src + pixoff, stride);
            }
        }
    }
}

/* Measure sum of 8x8 satd costs between source frame and reference
 * frame (potentially weighted, potentially motion compensated). We
 * always use source images for this analysis since reference recon
 * pixels have unreliable availability */
 /** 函数功能             ： 获取与运动补偿后参考帧间的SATD值 （如果WeightParam 不为null 则获取与加权运动补偿后参考帧间的SATD值）
 /*  调用范围             ： 只在weightAnalyse函数中被调用
 * \参数 fenc             ： 亮度下采样帧 或者 色度原始帧
 * \参数 ref              ： 运动补偿后的参考帧
 * \参数 weightTemp       ： 如果WeightParam 不为null 则会计算加权帧放入此区域  并用加权参考帧计算SATD
 * \参数 stride           ： 步长
 * \参数 cache            ： 存储当前帧对应下采样帧信息
 * \参数 width            ： 宽度
 * \参数 height           ： 高度
 * \参数 WeightParam      ： 可能为NULL或者待搜索的量化参数
 * \参数 bLuma            ： 亮度为true 否则为false
 * \返回                  ： 原始块与补偿块的SATD值 * */
uint32_t weightCost(pixel *         fenc,
                    pixel *         ref,
                    pixel *         weightTemp,
                    intptr_t        stride,
                    const Cache &   cache,
                    int             width,
                    int             height,
                    WeightParam *   w,
                    bool            bLuma)
{
    if (w)
    {
        /* make a weighted copy of the reference plane */
        int offset = w->inputOffset << (X265_DEPTH - 8);
        int weight = w->inputWeight;
        int denom = w->log2WeightDenom;
        int round = denom ? 1 << (denom - 1) : 0;
        int correction = IF_INTERNAL_PREC - X265_DEPTH; /* intermediate interpolation depth */
        int pwidth = ((width + 31) >> 5) << 5;
        primitives.weight_pp(ref, weightTemp, stride, pwidth, height,
                             weight, round << correction, denom + correction, offset); //获得P帧加权参考帧
        ref = weightTemp;
    }

    uint32_t cost = 0;
    pixel *f = fenc, *r = ref;

    if (bLuma)
    {
        int cu = 0;
        for (int y = 0; y < height; y += 8, r += 8 * stride, f += 8 * stride)
        {
            for (int x = 0; x < width; x += 8, cu++)
            {
                int cmp = primitives.pu[LUMA_8x8].satd(r + x, stride, f + x, stride);
                cost += X265_MIN(cmp, cache.intraCost[cu]);
            }
        }
    }
    else if (cache.csp == X265_CSP_I444)
        for (int y = 0; y < height; y += 16, r += 16 * stride, f += 16 * stride)
            for (int x = 0; x < width; x += 16)
                cost += primitives.pu[LUMA_16x16].satd(r + x, stride, f + x, stride);
    else
        for (int y = 0; y < height; y += 8, r += 8 * stride, f += 8 * stride)
            for (int x = 0; x < width; x += 8)
                cost += primitives.pu[LUMA_8x8].satd(r + x, stride, f + x, stride);

    return cost;
}
}

namespace X265_NS {
/** 函数功能             ： 分析加权信息(每个list的第一帧分析加权与否，其它不加权)
/*  调用范围             ： 只在WeightAnalysis::processTasks和FrameEncoder::compressFrame()函数中被调用
* \参数 slice            ： 当前帧的slice
* \参数 frame            ： 当前编码帧
* \参数 param            ： 当前配置参数
* \返回                  ： null * */
void weightAnalyse(Slice& slice, Frame& frame, x265_param& param)
{
    WeightParam wp[2][MAX_NUM_REF][3];//分别是list  refs  yuv
    PicYuv *fencPic = frame.m_fencPic;
    Lowres& fenc    = frame.m_lowres;//获取对应下采样视频帧

    Cache cache;

    memset(&cache, 0, sizeof(cache));
    cache.intraCost = fenc.intraCost;//每个8x8块的intracost
    cache.numPredDir = slice.isInterP() ? 1 : 2;//获取当前list的个数
    cache.lowresWidthInCU = fenc.width >> 3;
    cache.lowresHeightInCU = fenc.lines >> 3;
    cache.csp = param.internalCsp;
    cache.hshift = CHROMA_H_SHIFT(cache.csp);
    cache.vshift = CHROMA_V_SHIFT(cache.csp);

    /* Use single allocation for motion compensated ref and weight buffers */
    //申请空间为两帧大小 用于存储运动补偿后的参考帧（经参考帧按照MV位置copy到相应区域）以及存储加权运动补偿块位置
    pixel *mcbuf = X265_MALLOC(pixel, 2 * fencPic->m_stride * fencPic->m_picHeight);
    if (!mcbuf)
    {
        slice.disableWeights();
        return;
    }
    pixel *weightTemp = mcbuf + fencPic->m_stride * fencPic->m_picHeight;

    int lambda = (int)x265_lambda_tab[X265_LOOKAHEAD_QP];//获取lamda 对应QP 12
    int curPoc = slice.m_poc;
    const float epsilon = 1.f / 128.f;

    int chromaDenom, lumaDenom, denom;
    chromaDenom = lumaDenom = 7;
    int numpixels[3];
    int w16 = ((fencPic->m_picWidth  + 15) >> 4) << 4;
    int h16 = ((fencPic->m_picHeight + 15) >> 4) << 4;
    numpixels[0] = w16 * h16;
    numpixels[1] = numpixels[2] = numpixels[0] >> (cache.hshift + cache.vshift);


    //功能: 获取当前编码帧参考帧是否加权的信息
    //      1. 获取当前帧与参考帧（当前list的第一帧）的平均值 方差比值的开方 
    //      2. 确定chromaDenom值，保证 guessScale[1、2] < 127/（1<<chromaDenom）
    //      3. 确定当前list的第一个参考帧是否需要加权，并给出加权信息
    //      4. 如果当前亮度加权  色度分量加权情况不一致，则将色度全置成为加权
    //      5. 将当前list的其他帧（非第一帧）设置为 不加权
    for (int list = 0; list < cache.numPredDir; list++)
    {
        WeightParam *weights = wp[list][0];//获取当前list的第一帧的加权信息
        Frame *refFrame = slice.m_refFrameList[list][0];//获取当前list的第一帧
        Lowres& refLowres = refFrame->m_lowres;//获取当前参考帧的下采样帧
        int diffPoc = abs(curPoc - refFrame->m_poc);

        /* prepare estimates */
        float guessScale[3], fencMean[3], refMean[3];
        for (int plane = 0; plane < (param.internalCsp != X265_CSP_I400 ? 3 : 1); plane++)
        {
            SET_WEIGHT(weights[plane], false, 1, 0, 0); // 设置WeightParam类数据
            uint64_t fencVar = fenc.wp_ssd[plane] + !refLowres.wp_ssd[plane];//当前帧的Σ(x^2) - (Σx * Σx)/n = n*Variance (n倍的方差) 后面保证最小为1 不是0
            uint64_t refVar  = refLowres.wp_ssd[plane] + !refLowres.wp_ssd[plane];
            guessScale[plane] = sqrt((float)fencVar / refVar);//两方差比值的开方
            fencMean[plane] = (float)fenc.wp_sum[plane] / (numpixels[plane]) / (1 << (X265_DEPTH - 8));//获取平均值
            refMean[plane]  = (float)refLowres.wp_sum[plane] / (numpixels[plane]) / (1 << (X265_DEPTH - 8));//参考帧的平均值
        }

        /* make sure both our scale factors fit */
        //确定chromaDenom值，保证 guessScale[1、2] < 127/（1<<chromaDenom）  ,保证加权后值小于127
        while (!list && chromaDenom > 0)
        {
            float thresh = 127.f / (1 << chromaDenom);
            if (guessScale[1] < thresh && guessScale[2] < thresh)
                break;
            chromaDenom--;
        }

        SET_WEIGHT(weights[1], false, 1 << chromaDenom, chromaDenom, 0);//设置色度参数
        SET_WEIGHT(weights[2], false, 1 << chromaDenom, chromaDenom, 0);//设置色度参数

        MV *mvs = NULL;//存储下采样8x8块的mv信息

        //功能： 确定当前list的第一个参考帧是否需要加权，并给出加权信息
        //       1.如果当前是色度，并且亮度判断为不加权，直接退出
        //       2.如果平均值相近，纹理特性相近，不加权，直接退出
        //       3.设置亮度，色度权重系数
        //       4.将参考视频帧的原始帧进行扩边
        //       5.获取运动补偿数据（亮度（下采样帧）色度（原始帧））
        //       6.计算原始块与补偿块的SATD值，如果为0 不加权 退出
        //       7.在一定区间内搜索并获取当前最优的scale(inputWeight) 最优cost 最优inputOffset 是否加权
        //       8.如果是lsit0 并且是亮度  更新mindenom 
        //       9.设置当前list当前分量下的加权信息
        for (int plane = 0; plane < (param.internalCsp != X265_CSP_I400 ? 3 : 1); plane++)
        {
            denom = plane ? chromaDenom : lumaDenom;
            if (plane && !weights[0].bPresentFlag)//1.如果当前是色度，并且亮度判断为不加权，直接退出
                break;

            /* Early termination */
            x265_emms();//清除MMX寄存器中的内容，即初始化（以避免和浮点数操作发生冲突）。
            //2.如果编码帧与参考帧的平均值绝对值差小于0.5 并且 1-√(原始帧方差/参考帧方差） < 1/128 前者表示平均值相近  后者表示纹理特性相近  则判断不加权
            if (fabsf(refMean[plane] - fencMean[plane]) < 0.5f && fabsf(1.f - guessScale[plane]) < epsilon)
            {
                SET_WEIGHT(weights[plane], 0, 1 << denom, denom, 0);
                continue;
            }
            //3.设置亮度，色度权重系数
            if (plane)//色度信息
            {
                int scale = x265_clip3(0, 255, (int)(guessScale[plane] * (1 << denom) + 0.5f));
                if (scale > 127)
                    continue;
                weights[plane].inputWeight = scale;
            }
            else
            {
                weights[plane].setFromWeightAndOffset((int)(guessScale[plane] * (1 << denom) + 0.5f), 0, denom, !list);
            }

            int mindenom = weights[plane].log2WeightDenom;
            int minscale = weights[plane].inputWeight;//获取当前权重系数  后面会在 (minscale-4,minscale+4)范围内搜索最优
            int minoff = 0;

            //功能：将参考视频帧的原始帧进行扩边
            if (!plane && diffPoc <= param.bframes + 1)//如果当前是亮度并且编码帧与参考帧不能相隔太远(太远没有数据)
            {
                mvs = fenc.lowresMvs[list][diffPoc];

                /* test whether this motion search was performed by lookahead */
                if (mvs[0].x != 0x7FFF)
                {
                    /* reference chroma planes must be extended prior to being
                     * used as motion compensation sources */
                    if (!refFrame->m_bChromaExtended && param.internalCsp != X265_CSP_I400 && frame.m_fencPic->m_picCsp != X265_CSP_I400)//是否没有扩边过，已经扩边无须扩边
                    {
                        refFrame->m_bChromaExtended = true;
                        PicYuv *refPic = refFrame->m_fencPic;
                        int width = refPic->m_picWidth >> cache.hshift;
                        int height = refPic->m_picHeight >> cache.vshift;
                        extendPicBorder(refPic->m_picOrg[1], refPic->m_strideC, width, height, refPic->m_chromaMarginX, refPic->m_chromaMarginY);//将视频帧进行扩边，便于插值和ME搜索
                        extendPicBorder(refPic->m_picOrg[2], refPic->m_strideC, width, height, refPic->m_chromaMarginX, refPic->m_chromaMarginY);
                    }
                }
                else
                    mvs = 0;
            }

            /* prepare inputs to weight analysis */
            pixel *orig;
            pixel *fref;
            intptr_t stride;
            int    width, height;
            //功能:获取运动补偿数据（亮度（下采样帧）色度（原始帧））
            switch (plane)
            {
            case 0:
                orig = fenc.lowresPlane[0];//获取当前帧的下采样帧
                stride = fenc.lumaStride;
                width = fenc.width;
                height = fenc.lines;
                fref = refLowres.lowresPlane[0];//运动补偿数据
                if (mvs)
                {
                    mcLuma(mcbuf, refLowres, mvs);//按照MV的信息将参考帧的块copy到mcbuf去
                    fref = mcbuf;
                }
                break;

            case 1:
                orig = fencPic->m_picOrg[1];
                stride = fencPic->m_strideC;
                fref = refFrame->m_fencPic->m_picOrg[1];

                /* Clamp the chroma dimensions to the nearest multiple of
                 * 8x8 blocks (or 16x16 for 4:4:4) since mcChroma uses lowres
                 * blocks and weightCost measures 8x8 blocks. This
                 * potentially ignores some edge pixels, but simplifies the
                 * logic and prevents reading uninitialized pixels. Lowres
                 * planes are border extended and require no clamping. */
                width =  ((fencPic->m_picWidth  >> 4) << 4) >> cache.hshift;
                height = ((fencPic->m_picHeight >> 4) << 4) >> cache.vshift;
                if (mvs)
                {
                    mcChroma(mcbuf, fref, stride, mvs, cache, height, width);
                    fref = mcbuf;
                }
                break;

            case 2:
                orig = fencPic->m_picOrg[2];
                stride = fencPic->m_strideC;
                fref = refFrame->m_fencPic->m_picOrg[2];
                width =  ((fencPic->m_picWidth  >> 4) << 4) >> cache.hshift;
                height = ((fencPic->m_picHeight >> 4) << 4) >> cache.vshift;
                if (mvs)
                {
                    mcChroma(mcbuf, fref, stride, mvs, cache, height, width);
                    fref = mcbuf;
                }
                break;

            default:
                slice.disableWeights();//关闭加权预测
                X265_FREE(mcbuf);
                return;
            }
            //计算原始块与补偿块的SATD值
            uint32_t origscore = weightCost(orig, fref, weightTemp, stride, cache, width, height, NULL, !plane);
            if (!origscore)
            {
                SET_WEIGHT(weights[plane], 0, 1 << denom, denom, 0);//如果SATD为0，不用加权 退出
                continue;
            }

            uint32_t minscore = origscore;//存储当前最小的COST  SATD+bitcost
            bool bFound = false;

            /* x264 uses a table lookup here, selecting search range based on preset */
            static const int scaleDist = 4;
            static const int offsetDist = 2;

            int startScale = x265_clip3(0, 127, minscale - scaleDist);
            int endScale   = x265_clip3(0, 127, minscale + scaleDist);
            //功能:在一定区间内搜索并获取当前最优的scale(inputWeight) 最优cost 最优inputOffset 是否加权
            //     1. 获取当前的权重系数绝对值是否大于127 是直接搜索下一个
            //     2. 计算偏移offset ,如果offset过大，先clip offset 再重新求scalse
            //     3. 获取当前最优的offset(整帧的偏移值)
            for (int scale = startScale; scale <= endScale; scale++)
            {
                int deltaWeight = scale - (1 << mindenom);//判断偏移量
                if (deltaWeight > 127 || deltaWeight <= -128)
                    continue;

                x265_emms();
                int curScale = scale;//获取当前scale
                int curOffset = (int)(fencMean[plane] - refMean[plane] * curScale / (1 << mindenom) + 0.5f);// 可以理解为 求offset: enc-ref*w  w就是前面√(原始帧方差/参考帧方差） (在此是修正的值)
                if (curOffset < -128 || curOffset > 127)//一般不进入
                {
                    /* Rescale considering the constraints on curOffset. We do it in this order
                     * because scale has a much wider range than offset (because of denom), so
                     * it should almost never need to be clamped. */
                    curOffset = x265_clip3(-128, 127, curOffset);
                    curScale = (int)((1 << mindenom) * (fencMean[plane] - curOffset) / refMean[plane] + 0.5f);
                    curScale = x265_clip3(0, 127, curScale);
                }

                int startOffset = x265_clip3(-128, 127, curOffset - offsetDist);
                int endOffset   = x265_clip3(-128, 127, curOffset + offsetDist);
                for (int off = startOffset; off <= endOffset; off++)//功能:获取当前最优的offset(整帧的偏移值)
                {
                    WeightParam wsp;//暂存当前搜索的权重系数
                    SET_WEIGHT(wsp, true, curScale, mindenom, off);//设置当前搜索的权重参数
                    uint32_t s = weightCost(orig, fref, weightTemp, stride, cache, width, height, &wsp, !plane) +
                                 sliceHeaderCost(&wsp, lambda, !!plane);//获取与加权运动补偿后参考帧间的SATD值 + slice头信息cost
                    COPY4_IF_LT(minscore, s, minscale, curScale, minoff, off, bFound, true);//选取最小cost

                    /* Don't check any more offsets if the previous one had a lower cost than the current one */
                    if (minoff == startOffset && off != startOffset)//若前一个的cost比当前的更小，就直接退出
                        break;
                }
            }

            /* Use a smaller luma denominator if possible */
            if (!(plane || list))//如果是lsit0 并且是亮度  更新mindenom 
            {
                if (mindenom > 0 && !(minscale & 1))
                {
                    unsigned long idx;
                    CTZ(idx, minscale);
                    int shift = X265_MIN((int)idx, mindenom);
                    mindenom -= shift;
                    minscale >>= shift;
                }
            }

            if (!bFound || (minscale == (1 << mindenom) && minoff == 0) || (float)minscore / origscore > 0.998f)//不加权情况：前面搜索获得结果不加权   加权与否cost变化不大
            {
                SET_WEIGHT(weights[plane], false, 1 << denom, denom, 0);
            }
            else
            {
                SET_WEIGHT(weights[plane], true, minscale, mindenom, minoff);
            }
        }

        if (weights[0].bPresentFlag)//如果当前亮度加权  色度分量加权情况不一致，则将色度全置成为加权
        {
            // Make sure both chroma channels match
            if (weights[1].bPresentFlag != weights[2].bPresentFlag)
            {
                if (weights[1].bPresentFlag)
                    weights[2] = weights[1];
                else
                    weights[1] = weights[2];
            }
        }

        lumaDenom = weights[0].log2WeightDenom;
        chromaDenom = weights[1].log2WeightDenom;

        /* reset weight states */
        for (int ref = 1; ref < slice.m_numRefIdx[list]; ref++)//将当前list的其他帧（非第一帧）设置为 不加权
        {
            SET_WEIGHT(wp[list][ref][0], false, 1 << lumaDenom, lumaDenom, 0);
            SET_WEIGHT(wp[list][ref][1], false, 1 << chromaDenom, chromaDenom, 0);
            SET_WEIGHT(wp[list][ref][2], false, 1 << chromaDenom, chromaDenom, 0);
        }
    }

    X265_FREE(mcbuf);

    memcpy(slice.m_weightPredTable, wp, sizeof(WeightParam) * 2 * MAX_NUM_REF * 3);//获取参考帧加权状态信息
    //打印log  如果加权则打印加权信息
    //样例：x265 [full]: poc: 24 weights: [L0:R0 Y{63/64+1}U{98/128+25}V{123/128+6}]  w[0].inputWeight/1 << w[0].log2WeightDenom    w[0].inputOffset   加权信息  w* ref  + offset
    if (param.logLevel >= X265_LOG_FULL)
    {
        char buf[1024];
        int p = 0;
        bool bWeighted = false;

        p = sprintf(buf, "poc: %d weights:", slice.m_poc);
        int numPredDir = slice.isInterP() ? 1 : 2;
        for (int list = 0; list < numPredDir; list++)
        {
            WeightParam* w = &wp[list][0][0];
            if (w[0].bPresentFlag || w[1].bPresentFlag || w[2].bPresentFlag)
            {
                bWeighted = true;
                p += sprintf(buf + p, " [L%d:R0 ", list);
                if (w[0].bPresentFlag)
                    p += sprintf(buf + p, "Y{%d/%d%+d}", w[0].inputWeight, 1 << w[0].log2WeightDenom, w[0].inputOffset);
                if (w[1].bPresentFlag)
                    p += sprintf(buf + p, "U{%d/%d%+d}", w[1].inputWeight, 1 << w[1].log2WeightDenom, w[1].inputOffset);
                if (w[2].bPresentFlag)
                    p += sprintf(buf + p, "V{%d/%d%+d}", w[2].inputWeight, 1 << w[2].log2WeightDenom, w[2].inputOffset);
                p += sprintf(buf + p, "]");
            }
        }

        if (bWeighted)
        {
            if (p < 80) // pad with spaces to ensure progress line overwritten
                sprintf(buf + p, "%*s", 80 - p, " ");
            x265_log(&param, X265_LOG_FULL, "%s\n", buf);
        }
    }
}
}
