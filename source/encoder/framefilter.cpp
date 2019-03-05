/*****************************************************************************
 * Copyright (C) 2013-2017 MulticoreWare, Inc
 *
 * Authors: Chung Shin Yee <shinyee@multicorewareinc.com>
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
#include "frame.h"
#include "framedata.h"
#include "framefilter.h"
#include "wavefront.h"

using namespace X265_NS;

namespace X265_NS
{
    static void integral_init4h_c(uint32_t *sum, pixel *pix, intptr_t stride)
    {
        int32_t v = pix[0] + pix[1] + pix[2] + pix[3];
        for (int16_t x = 0; x < stride - 4; x++)
        {
            sum[x] = v + sum[x - stride];
            v += pix[x + 4] - pix[x];
        }
    }

    static void integral_init8h_c(uint32_t *sum, pixel *pix, intptr_t stride)
    {
        int32_t v = pix[0] + pix[1] + pix[2] + pix[3] + pix[4] + pix[5] + pix[6] + pix[7];
        for (int16_t x = 0; x < stride - 8; x++)
        {
            sum[x] = v + sum[x - stride];
            v += pix[x + 8] - pix[x];
        }
    }

    static void integral_init12h_c(uint32_t *sum, pixel *pix, intptr_t stride)
    {
        int32_t v = pix[0] + pix[1] + pix[2] + pix[3] + pix[4] + pix[5] + pix[6] + pix[7] +
            pix[8] + pix[9] + pix[10] + pix[11];
        for (int16_t x = 0; x < stride - 12; x++)
        {
            sum[x] = v + sum[x - stride];
            v += pix[x + 12] - pix[x];
        }
    }

    static void integral_init16h_c(uint32_t *sum, pixel *pix, intptr_t stride)
    {
        int32_t v = pix[0] + pix[1] + pix[2] + pix[3] + pix[4] + pix[5] + pix[6] + pix[7] +
            pix[8] + pix[9] + pix[10] + pix[11] + pix[12] + pix[13] + pix[14] + pix[15];
        for (int16_t x = 0; x < stride - 16; x++)
        {
            sum[x] = v + sum[x - stride];
            v += pix[x + 16] - pix[x];
        }
    }

    static void integral_init24h_c(uint32_t *sum, pixel *pix, intptr_t stride)
    {
        int32_t v = pix[0] + pix[1] + pix[2] + pix[3] + pix[4] + pix[5] + pix[6] + pix[7] +
            pix[8] + pix[9] + pix[10] + pix[11] + pix[12] + pix[13] + pix[14] + pix[15] +
            pix[16] + pix[17] + pix[18] + pix[19] + pix[20] + pix[21] + pix[22] + pix[23];
        for (int16_t x = 0; x < stride - 24; x++)
        {
            sum[x] = v + sum[x - stride];
            v += pix[x + 24] - pix[x];
        }
    }

    static void integral_init32h_c(uint32_t *sum, pixel *pix, intptr_t stride)
    {
        int32_t v = pix[0] + pix[1] + pix[2] + pix[3] + pix[4] + pix[5] + pix[6] + pix[7] +
            pix[8] + pix[9] + pix[10] + pix[11] + pix[12] + pix[13] + pix[14] + pix[15] +
            pix[16] + pix[17] + pix[18] + pix[19] + pix[20] + pix[21] + pix[22] + pix[23] +
            pix[24] + pix[25] + pix[26] + pix[27] + pix[28] + pix[29] + pix[30] + pix[31];
        for (int16_t x = 0; x < stride - 32; x++)
        {
            sum[x] = v + sum[x - stride];
            v += pix[x + 32] - pix[x];
        }
    }

    static void integral_init4v_c(uint32_t *sum4, intptr_t stride)
    {
        for (int x = 0; x < stride; x++)
            sum4[x] = sum4[x + 4 * stride] - sum4[x];
    }

    static void integral_init8v_c(uint32_t *sum8, intptr_t stride)
    {
        for (int x = 0; x < stride; x++)
            sum8[x] = sum8[x + 8 * stride] - sum8[x];
    }

    static void integral_init12v_c(uint32_t *sum12, intptr_t stride)
    {
        for (int x = 0; x < stride; x++)
            sum12[x] = sum12[x + 12 * stride] - sum12[x];
    }

    static void integral_init16v_c(uint32_t *sum16, intptr_t stride)
    {
        for (int x = 0; x < stride; x++)
            sum16[x] = sum16[x + 16 * stride] - sum16[x];
    }

    static void integral_init24v_c(uint32_t *sum24, intptr_t stride)
    {
        for (int x = 0; x < stride; x++)
            sum24[x] = sum24[x + 24 * stride] - sum24[x];
    }

    static void integral_init32v_c(uint32_t *sum32, intptr_t stride)
    {
        for (int x = 0; x < stride; x++)
            sum32[x] = sum32[x + 32 * stride] - sum32[x];
    }

    void setupSeaIntegralPrimitives_c(EncoderPrimitives &p)
    {
        p.integral_initv[INTEGRAL_4] = integral_init4v_c;
        p.integral_initv[INTEGRAL_8] = integral_init8v_c;
        p.integral_initv[INTEGRAL_12] = integral_init12v_c;
        p.integral_initv[INTEGRAL_16] = integral_init16v_c;
        p.integral_initv[INTEGRAL_24] = integral_init24v_c;
        p.integral_initv[INTEGRAL_32] = integral_init32v_c;
        p.integral_inith[INTEGRAL_4] = integral_init4h_c;
        p.integral_inith[INTEGRAL_8] = integral_init8h_c;
        p.integral_inith[INTEGRAL_12] = integral_init12h_c;
        p.integral_inith[INTEGRAL_16] = integral_init16h_c;
        p.integral_inith[INTEGRAL_24] = integral_init24h_c;
        p.integral_inith[INTEGRAL_32] = integral_init32h_c;
    }
}

void FrameFilter::destroy()
{
    X265_FREE(m_ssimBuf);

    if (m_parallelFilter)
    {
        if (m_param->bEnableSAO)
        {
            for(int row = 0; row < m_numRows; row++)
                m_parallelFilter[row].m_sao.destroy((row == 0 ? 1 : 0));
        }

        delete[] m_parallelFilter;
        m_parallelFilter = NULL;
    }
}

void FrameFilter::start(Frame *frame, Entropy& initState)
{
    m_frame = frame;

    // Reset Filter Data Struct
    if (m_parallelFilter)
    {
        for(int row = 0; row < m_numRows; row++)
        {
            if (m_param->bEnableSAO)
                m_parallelFilter[row].m_sao.startSlice(frame, initState);

            m_parallelFilter[row].m_lastCol.set(0);
            m_parallelFilter[row].m_allowedCol.set(0);
            m_parallelFilter[row].m_lastDeblocked.set(-1);
            m_parallelFilter[row].m_encData = frame->m_encData;
        }

        // Reset SAO common statistics
        if (m_param->bEnableSAO)
            m_parallelFilter[0].m_sao.resetStats();
    }
}

/* restore original YUV samples to recon after SAO (if lossless) */
static void restoreOrigLosslessYuv(const CUData* cu, Frame& frame, uint32_t absPartIdx)
{
    const int size = cu->m_log2CUSize[absPartIdx] - 2;
    const uint32_t cuAddr = cu->m_cuAddr;

    PicYuv* reconPic = frame.m_reconPic;
    PicYuv* fencPic  = frame.m_fencPic;

    pixel* dst = reconPic->getLumaAddr(cuAddr, absPartIdx);
    pixel* src = fencPic->getLumaAddr(cuAddr, absPartIdx);

    primitives.cu[size].copy_pp(dst, reconPic->m_stride, src, fencPic->m_stride);

    if (cu->m_chromaFormat != X265_CSP_I400)
    {
        pixel* dstCb = reconPic->getCbAddr(cuAddr, absPartIdx);
        pixel* srcCb = fencPic->getCbAddr(cuAddr, absPartIdx);
        pixel* dstCr = reconPic->getCrAddr(cuAddr, absPartIdx);
        pixel* srcCr = fencPic->getCrAddr(cuAddr, absPartIdx);

        const int csp = fencPic->m_picCsp;
        primitives.chroma[csp].cu[size].copy_pp(dstCb, reconPic->m_strideC, srcCb, fencPic->m_strideC);
        primitives.chroma[csp].cu[size].copy_pp(dstCr, reconPic->m_strideC, srcCr, fencPic->m_strideC);
    }
}

/* Original YUV restoration for CU in lossless coding */
static void origCUSampleRestoration(const CUData* cu, const CUGeom& cuGeom, Frame& frame)
{
    uint32_t absPartIdx = cuGeom.absPartIdx;
    if (cu->m_cuDepth[absPartIdx] > cuGeom.depth)
    {
        for (int subPartIdx = 0; subPartIdx < 4; subPartIdx++)
        {
            const CUGeom& childGeom = *(&cuGeom + cuGeom.childOffset + subPartIdx);
            if (childGeom.flags & CUGeom::PRESENT)
                origCUSampleRestoration(cu, childGeom, frame);
        }
        return;
    }

    // restore original YUV samples
    if (cu->m_tqBypass[absPartIdx])
        restoreOrigLosslessYuv(cu, frame, absPartIdx);
}

void FrameFilter::ParallelFilter::copySaoAboveRef(const CUData *ctu, PicYuv* reconPic, uint32_t cuAddr, int col)
{
    // Copy SAO Top Reference Pixels
    int ctuWidth  = ctu->m_encData->m_param->maxCUSize;
    const pixel* recY = reconPic->getPlaneAddr(0, cuAddr) - (ctu->m_bFirstRowInSlice ? 0 : reconPic->m_stride);

    // Luma
    memcpy(&m_sao.m_tmpU[0][col * ctuWidth], recY, ctuWidth * sizeof(pixel));
    X265_CHECK(col * ctuWidth + ctuWidth <= m_sao.m_numCuInWidth * ctuWidth, "m_tmpU buffer beyond bound write detected");

    // Chroma
    if (m_frameFilter->m_param->internalCsp != X265_CSP_I400)
    {
        ctuWidth  >>= m_sao.m_hChromaShift;

        const pixel* recU = reconPic->getPlaneAddr(1, cuAddr) - (ctu->m_bFirstRowInSlice ? 0 : reconPic->m_strideC);
        const pixel* recV = reconPic->getPlaneAddr(2, cuAddr) - (ctu->m_bFirstRowInSlice ? 0 : reconPic->m_strideC);
        memcpy(&m_sao.m_tmpU[1][col * ctuWidth], recU, ctuWidth * sizeof(pixel));
        memcpy(&m_sao.m_tmpU[2][col * ctuWidth], recV, ctuWidth * sizeof(pixel));

        X265_CHECK(col * ctuWidth + ctuWidth <= m_sao.m_numCuInWidth * ctuWidth, "m_tmpU buffer beyond bound write detected");
    }
}

// NOTE: MUST BE delay a row when Deblock enabled, the Deblock will modify above pixels in Horizon pass
void FrameFilter::ParallelFilter::processPostCu(int col) const
{
    // Update finished CU cursor
    m_frameFilter->m_frame->m_reconColCount[m_row].set(col);

    // shortcut path for non-border area
    if ((col != 0) & (col != m_frameFilter->m_numCols - 1) & (m_row != 0) & (m_row != m_frameFilter->m_numRows - 1))
        return;

    PicYuv *reconPic = m_frameFilter->m_frame->m_reconPic;
    const uint32_t lineStartCUAddr = m_rowAddr + col;
    const int realH = getCUHeight();
    const int realW = m_frameFilter->getCUWidth(col);

    const uint32_t lumaMarginX = reconPic->m_lumaMarginX;
    const uint32_t lumaMarginY = reconPic->m_lumaMarginY;
    const uint32_t chromaMarginX = reconPic->m_chromaMarginX;
    const uint32_t chromaMarginY = reconPic->m_chromaMarginY;
    const int hChromaShift = reconPic->m_hChromaShift;
    const int vChromaShift = reconPic->m_vChromaShift;
    const intptr_t stride = reconPic->m_stride;
    const intptr_t strideC = reconPic->m_strideC;
    pixel *pixY = reconPic->getLumaAddr(lineStartCUAddr);
    // // MUST BE check I400 since m_picOrg uninitialize in that case
    pixel *pixU = (m_frameFilter->m_param->internalCsp != X265_CSP_I400) ? reconPic->getCbAddr(lineStartCUAddr) : NULL;
    pixel *pixV = (m_frameFilter->m_param->internalCsp != X265_CSP_I400) ? reconPic->getCrAddr(lineStartCUAddr) : NULL;
    int copySizeY = realW;
    int copySizeC = (realW >> hChromaShift);

    if ((col == 0) | (col == m_frameFilter->m_numCols - 1))
    {
        // TODO: improve by process on Left or Right only
        primitives.extendRowBorder(reconPic->getLumaAddr(m_rowAddr), stride, reconPic->m_picWidth, realH, reconPic->m_lumaMarginX);

        if (m_frameFilter->m_param->internalCsp != X265_CSP_I400)
        {
            primitives.extendRowBorder(reconPic->getCbAddr(m_rowAddr), strideC, reconPic->m_picWidth >> hChromaShift, realH >> vChromaShift, reconPic->m_chromaMarginX);
            primitives.extendRowBorder(reconPic->getCrAddr(m_rowAddr), strideC, reconPic->m_picWidth >> hChromaShift, realH >> vChromaShift, reconPic->m_chromaMarginX);
        }
    }

    // Extra Left and Right border on first and last CU
    if ((col == 0) | (col == m_frameFilter->m_numCols - 1))
    {
        copySizeY += lumaMarginX;
        copySizeC += chromaMarginX;
    }

    // First column need extension left padding area and first CU
    if (col == 0)
    {
        pixY -= lumaMarginX;
        pixU -= chromaMarginX;
        pixV -= chromaMarginX;
    }

    // Border extend Top
    if (m_row == 0)
    {
        for (uint32_t y = 0; y < lumaMarginY; y++)
            memcpy(pixY - (y + 1) * stride, pixY, copySizeY * sizeof(pixel));

        if (m_frameFilter->m_param->internalCsp != X265_CSP_I400)
        {
            for (uint32_t y = 0; y < chromaMarginY; y++)
            {
                memcpy(pixU - (y + 1) * strideC, pixU, copySizeC * sizeof(pixel));
                memcpy(pixV - (y + 1) * strideC, pixV, copySizeC * sizeof(pixel));
            }
        }
    }

    // Border extend Bottom
    if (m_row == m_frameFilter->m_numRows - 1)
    {
        pixY += (realH - 1) * stride;
        pixU += ((realH >> vChromaShift) - 1) * strideC;
        pixV += ((realH >> vChromaShift) - 1) * strideC;
        for (uint32_t y = 0; y < lumaMarginY; y++)
            memcpy(pixY + (y + 1) * stride, pixY, copySizeY * sizeof(pixel));

        if (m_frameFilter->m_param->internalCsp != X265_CSP_I400)
        {
            for (uint32_t y = 0; y < chromaMarginY; y++)
            {
                memcpy(pixU + (y + 1) * strideC, pixU, copySizeC * sizeof(pixel));
                memcpy(pixV + (y + 1) * strideC, pixV, copySizeC * sizeof(pixel));
            }
        }
    }
}

void FrameFilter::computeMEIntegral(int row)
{
    int lastRow = row == (int)m_frame->m_encData->m_slice->m_sps->numCuInHeight - 1;
    if (m_frame->m_lowres.sliceType != X265_TYPE_B)
    {
        /* If WPP, other than first row, integral calculation for current row needs to wait till the
        * integral for the previous row is computed */
        if (m_param->bEnableWavefront && row)
        {
            while (m_parallelFilter[row - 1].m_frameFilter->integralCompleted.get() == 0)
            {
                m_parallelFilter[row - 1].m_frameFilter->integralCompleted.waitForChange(0);
            }
        }

        int stride = (int)m_frame->m_reconPic->m_stride;
        int padX = m_param->maxCUSize + 32;
        int padY = m_param->maxCUSize + 16;
        int numCuInHeight = m_frame->m_encData->m_slice->m_sps->numCuInHeight;
        int maxHeight = numCuInHeight * m_param->maxCUSize;
        int startRow = 0;

        if (m_param->interlaceMode)
            startRow = (row * m_param->maxCUSize >> 1);
        else
            startRow = row * m_param->maxCUSize;

        int height = lastRow ? (maxHeight + m_param->maxCUSize * m_param->interlaceMode) : (((row + m_param->interlaceMode) * m_param->maxCUSize) + m_param->maxCUSize);

        if (!row)
        {
            for (int i = 0; i < INTEGRAL_PLANE_NUM; i++)
                memset(m_frame->m_encData->m_meIntegral[i] - padY * stride - padX, 0, stride * sizeof(uint32_t));
            startRow = -padY;
        }

        if (lastRow)
            height += padY - 1;

        for (int y = startRow; y < height; y++)
        {
            pixel    *pix = m_frame->m_reconPic->m_picOrg[0] + y * stride - padX;
            uint32_t *sum32x32 = m_frame->m_encData->m_meIntegral[0] + (y + 1) * stride - padX;
            uint32_t *sum32x24 = m_frame->m_encData->m_meIntegral[1] + (y + 1) * stride - padX;
            uint32_t *sum32x8 = m_frame->m_encData->m_meIntegral[2] + (y + 1) * stride - padX;
            uint32_t *sum24x32 = m_frame->m_encData->m_meIntegral[3] + (y + 1) * stride - padX;
            uint32_t *sum16x16 = m_frame->m_encData->m_meIntegral[4] + (y + 1) * stride - padX;
            uint32_t *sum16x12 = m_frame->m_encData->m_meIntegral[5] + (y + 1) * stride - padX;
            uint32_t *sum16x4 = m_frame->m_encData->m_meIntegral[6] + (y + 1) * stride - padX;
            uint32_t *sum12x16 = m_frame->m_encData->m_meIntegral[7] + (y + 1) * stride - padX;
            uint32_t *sum8x32 = m_frame->m_encData->m_meIntegral[8] + (y + 1) * stride - padX;
            uint32_t *sum8x8 = m_frame->m_encData->m_meIntegral[9] + (y + 1) * stride - padX;
            uint32_t *sum4x16 = m_frame->m_encData->m_meIntegral[10] + (y + 1) * stride - padX;
            uint32_t *sum4x4 = m_frame->m_encData->m_meIntegral[11] + (y + 1) * stride - padX;

            /*For width = 32 */
            primitives.integral_inith[INTEGRAL_32](sum32x32, pix, stride);
            if (y >= 32 - padY)
                primitives.integral_initv[INTEGRAL_32](sum32x32 - 32 * stride, stride);
            primitives.integral_inith[INTEGRAL_32](sum32x24, pix, stride);
            if (y >= 24 - padY)
                primitives.integral_initv[INTEGRAL_24](sum32x24 - 24 * stride, stride);
            primitives.integral_inith[INTEGRAL_32](sum32x8, pix, stride);
            if (y >= 8 - padY)
                primitives.integral_initv[INTEGRAL_8](sum32x8 - 8 * stride, stride);
            /*For width = 24 */
            primitives.integral_inith[INTEGRAL_24](sum24x32, pix, stride);
            if (y >= 32 - padY)
                primitives.integral_initv[INTEGRAL_32](sum24x32 - 32 * stride, stride);
            /*For width = 16 */
            primitives.integral_inith[INTEGRAL_16](sum16x16, pix, stride);
            if (y >= 16 - padY)
                primitives.integral_initv[INTEGRAL_16](sum16x16 - 16 * stride, stride);
            primitives.integral_inith[INTEGRAL_16](sum16x12, pix, stride);
            if (y >= 12 - padY)
                primitives.integral_initv[INTEGRAL_12](sum16x12 - 12 * stride, stride);
            primitives.integral_inith[INTEGRAL_16](sum16x4, pix, stride);
            if (y >= 4 - padY)
                primitives.integral_initv[INTEGRAL_4](sum16x4 - 4 * stride, stride);
            /*For width = 12 */
            primitives.integral_inith[INTEGRAL_12](sum12x16, pix, stride);
            if (y >= 16 - padY)
                primitives.integral_initv[INTEGRAL_16](sum12x16 - 16 * stride, stride);
            /*For width = 8 */
            primitives.integral_inith[INTEGRAL_8](sum8x32, pix, stride);
            if (y >= 32 - padY)
                primitives.integral_initv[INTEGRAL_32](sum8x32 - 32 * stride, stride);
            primitives.integral_inith[INTEGRAL_8](sum8x8, pix, stride);
            if (y >= 8 - padY)
                primitives.integral_initv[INTEGRAL_8](sum8x8 - 8 * stride, stride);
            /*For width = 4 */
            primitives.integral_inith[INTEGRAL_4](sum4x16, pix, stride);
            if (y >= 16 - padY)
                primitives.integral_initv[INTEGRAL_16](sum4x16 - 16 * stride, stride);
            primitives.integral_inith[INTEGRAL_4](sum4x4, pix, stride);
            if (y >= 4 - padY)
                primitives.integral_initv[INTEGRAL_4](sum4x4 - 4 * stride, stride);
        }
        m_parallelFilter[row].m_frameFilter->integralCompleted.set(1);
    }
}

