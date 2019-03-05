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