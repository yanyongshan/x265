/*****************************************************************************
* Copyright (C) 2013-2020 MulticoreWare, Inc
*
* Authors: Deepthi Nandakumar <deepthi@multicorewareinc.com>
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

#ifndef X265_PREDICT_H
#define X265_PREDICT_H

#include "common.h"
#include "frame.h"
#include "quant.h"
#include "shortyuv.h"
#include "yuv.h"

namespace X265_NS {

class CUData;
class Slice;
struct CUGeom;
//预测单元
struct PredictionUnit
{
    //当前CTU的地址
    uint32_t     ctuAddr;      // raster index of current CTU within its picture
    //当前CU在CTU中的索引
    uint32_t     cuAbsPartIdx; // z-order offset of current CU within its CTU
    //当前PU在CU中的索引
    uint32_t     puAbsPartIdx; // z-order offset of current PU with its CU
    //PU大小
    int          width;
    int          height;

    PredictionUnit(const CUData& cu, const CUGeom& cuGeom, int puIdx);
};

class Predict
{
public:

    enum { ADI_BUF_STRIDE = (2 * MAX_CU_SIZE + 1 + 15) }; // alignment to 16 bytes

    /* Weighted prediction scaling values built from slice parameters (bitdepth scaled) */
    struct WeightValues
    {
        int w, o, offset, shift, round;
    };

    //帧内预测：用于存储周边块的可用标记等信息
    struct IntraNeighbors
    {
        //统计当前预测块周边多少可用4x4相邻块
        int      numIntraNeighbor;
        //周边4x4块个数  左下、左边、左上角1个、上边、右上
        int      totalUnits;
        //上边4x4个数：上边 右上
        int      aboveUnits;
        //左边4x4个数：左边、左下
        int      leftUnits;
        //当前最小块宽度，4
        int      unitWidth;
        //当前最小块高度，4
        int      unitHeight;
        //当前PU-TU深度
        int      log2TrSize;
        //空间大小为65  存储周边标记是否可用  存储方式： 左下、左边(左下和左边按照离左上角由远及近存储0：左下左下像素点....左上角4x4块像素点）、左上角一点、上边、右上
        bool     bNeighborFlags[4 * MAX_NUM_SPU_W + 1];
    };

    ShortYuv  m_predShortYuv[2]; /* temporary storage for weighted prediction */

    // Unfiltered/filtered neighbours of the current partition.
    // 过滤前和过滤后相邻参考像素4*N+1（第一维是参考数据，第二维是经过滤波后的数据）
    // 以最大PU：64*64分配空间，分别为0:左上，1~64：上方，65~129：右上，129~193：左方，193~257：左下
    pixel     intraNeighbourBuf[2][258];

    /* Slice information */
    int       m_csp;
    int       m_hChromaShift;
    int       m_vChromaShift;

    Predict();
    ~Predict();

    bool allocBuffers(int csp);

    // motion compensation functions
    /***
     * 帧间亮度预测
     * @param pu
     * @param dstYuv
     * @param refPic
     * @param mv 运动向量
     */
    void predInterLumaPixel(const PredictionUnit& pu, Yuv& dstYuv, const PicYuv& refPic, const MV& mv) const;
    void predInterChromaPixel(const PredictionUnit& pu, Yuv& dstYuv, const PicYuv& refPic, const MV& mv) const;

    void predInterLumaShort(const PredictionUnit& pu, ShortYuv& dstSYuv, const PicYuv& refPic, const MV& mv) const;
    void predInterChromaShort(const PredictionUnit& pu, ShortYuv& dstSYuv, const PicYuv& refPic, const MV& mv) const;

    void addWeightBi(const PredictionUnit& pu, Yuv& predYuv, const ShortYuv& srcYuv0, const ShortYuv& srcYuv1, const WeightValues wp0[3], const WeightValues wp1[3], bool bLuma, bool bChroma) const;
    void addWeightUni(const PredictionUnit& pu, Yuv& predYuv, const ShortYuv& srcYuv, const WeightValues wp[3], bool bLuma, bool bChroma) const;

    void motionCompensation(const CUData& cu, const PredictionUnit& pu, Yuv& predYuv, bool bLuma, bool bChroma);

    /* Angular Intra */
    /***
     * 按照当前的亮度预测方向获取预测值
     * @param dirMode  当前亮度方向
     * @param pred PU预测块地址
     * @param stride  原始块步长
     * @param log2TrSize 当前PU的最大变换大小
     */
    void predIntraLumaAng(uint32_t dirMode, pixel* pred, intptr_t stride, uint32_t log2TrSize);
    void predIntraChromaAng(uint32_t dirMode, pixel* pred, intptr_t stride, uint32_t log2TrSizeC);
    void initAdiPattern(const CUData& cu, const CUGeom& cuGeom, uint32_t puAbsPartIdx, const IntraNeighbors& intraNeighbors, int dirMode);
    void initAdiPatternChroma(const CUData& cu, const CUGeom& cuGeom, uint32_t puAbsPartIdx, const IntraNeighbors& intraNeighbors, uint32_t chromaId);

    /* Intra prediction helper functions */
    static void initIntraNeighbors(const CUData& cu, uint32_t absPartIdx, uint32_t tuDepth, bool isLuma, IntraNeighbors *IntraNeighbors);
    static void fillReferenceSamples(const pixel* adiOrigin, intptr_t picStride, const IntraNeighbors& intraNeighbors, pixel dst[258]);
    /***
     * 返回当前左上角像素点是否可用
     * @tparam cip 帧内预测是否受限（不能参考inter块）
     * @param cu 当前编码的CU
     * @param partIdxLT 当前PU左上角像素点在CTU中的zigzag标号
     * @return 返回当前左上角像素点是否可用
     */
    template<bool cip>
    static bool isAboveLeftAvailable(const CUData& cu, uint32_t partIdxLT);
    template<bool cip>
    static int  isAboveAvailable(const CUData& cu, uint32_t partIdxLT, uint32_t partIdxRT, bool* bValidFlags);
    template<bool cip>
    static int  isLeftAvailable(const CUData& cu, uint32_t partIdxLT, uint32_t partIdxLB, bool* bValidFlags);
    template<bool cip>
    static int  isAboveRightAvailable(const CUData& cu, uint32_t partIdxRT, bool* bValidFlags, uint32_t numUnits);
    template<bool cip>
    static int  isBelowLeftAvailable(const CUData& cu, uint32_t partIdxLB, bool* bValidFlags, uint32_t numUnits);
};
}

#endif // ifndef X265_PREDICT_H
