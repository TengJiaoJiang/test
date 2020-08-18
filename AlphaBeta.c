/*
 * AlphaBeta.c
 *
 *  Created on: 2019��11��6��
 *      Author: 1
 */
#include <stdio.h>
#include <math.h>

#include "AlphaBeta.h"
#include "DataStructure.h"

extern FILE *fWrite;                         ////////////////////////////////


extern uint16_t ScanGap;
#define    SCANNUMX   (19)   // alphabeta ����
#define    SCANNUMY   (20)   // alphabeta ����


void FilterAlphaBeta(TwiceDataKalman* trace, OnceData* Point)
{
        int i = 0, j = 0, k = 0;
        float alphaX = 0, betaX = 0;
		float alphaY = 0, betaY = 0;
        //Zk
        float ZkdArrayX[1][1];
        float ZkdArrayY[1][1];

        float ynow = trace->XkArray_yfit[0][0]; // 20200422 change

        float xNext = 0;
        float yNext = 0;

        if (Point == NULL)
        {
            ZkdArrayX[0][0] = trace->XkArray_xnxt[0][0];
            ZkdArrayY[0][0] = trace->XkArray_ynxt[0][0];
        }
        else
        {
            ZkdArrayX[0][0] = Point->x;
            ZkdArrayY[0][0] = Point->y;
        }

        //������ʼ��
        if ((*trace).TraceFlag == 0)
        {
			fprintf(fWrite, "		trace(ID,flag, v,x,y) %d %d %f %f %f init\n", (*trace).ID, (*trace).TraceFlag * 1, (*trace).DPLVel, (*trace).XkArray_xfit[0][0], (*trace).XkArray_yfit[0][0]);

			fprintf(fWrite, "				x0 = %f, x1 = %f, y0 = %f, y1 = %f, TimeGap = %f, trace->iScanNum = %d ", trace->XkArray_xfit[0][0], ZkdArrayX[0][0], trace->XkArray_yfit[0][0], ZkdArrayY[0][0], TimeGap, trace->iScanNum);
            trace->XkArray_xfit[1][0] = (ZkdArrayX[0][0] - trace->XkArray_xfit[0][0]) / (TimeGap * trace->iScanNum);        // xv
            trace->XkArray_xfit[0][0] = ZkdArrayX[0][0];                                                // x

            trace->XkArray_yfit[1][0] = (ZkdArrayY[0][0] - trace->XkArray_yfit[0][0]) / (TimeGap * trace->iScanNum);        // yv
            trace->XkArray_yfit[0][0] = ZkdArrayY[0][0];                                                // y

            //Ŀ����ʵ�ٶ�
            trace->Vel = sqrt(trace->XkArray_xfit[1][0] * trace->XkArray_xfit[1][0] + trace->XkArray_yfit[1][0] * trace->XkArray_yfit[1][0]);

			fprintf(fWrite, " xf = %f  yf = %f  xv = %f, yv = %f\n", trace->XkArray_xfit[0][0], trace->XkArray_yfit[0][0], trace->XkArray_xfit[1][0], trace->XkArray_yfit[1][0]);

        }
        else
        {
            CaculAlphaBetaX(trace->iScanNum , &alphaX, &betaX);
			CaculAlphaBetaY(trace->iScanNum, &alphaY, &betaY);

            //xf = xpre + alpha * (z - xpre)
            //x'f = x'pre + beta * (z - xpre) / tgap
            xNext = trace->XkArray_xfit[0][0] + trace->XkArray_xfit[1][0] * (TimeGap * ScanGap);
            trace->XkArray_xfit[0][0] = xNext + alphaX * (ZkdArrayX[0][0] - xNext);
            trace->XkArray_xfit[1][0] = trace->XkArray_xfit[1][0] + betaX * (ZkdArrayX[0][0] - xNext) / (TimeGap * ScanGap);

            //Y�ᴦ��
            yNext = trace->XkArray_yfit[0][0] + trace->XkArray_yfit[1][0] * (TimeGap * ScanGap);                                                                                    //���½ṹ��
            trace->XkArray_yfit[0][0] = yNext + alphaY * (ZkdArrayY[0][0] - yNext);
            trace->XkArray_yfit[1][0] = trace->XkArray_yfit[1][0] + betaY * (ZkdArrayY[0][0] - yNext) / (TimeGap * ScanGap);

            //Ŀ����ʵ�ٶ�
            trace->Vel = sqrt(trace->XkArray_xfit[1][0] * trace->XkArray_xfit[1][0] + trace->XkArray_yfit[1][0] * trace->XkArray_yfit[1][0]);

            //// 20200422 change
            //if (ynow * trace->XkArray_yfit[0][0] < 0)
            //{
            //    trace->passY0 = 1;
            //}
			fprintf(fWrite, " xf = %f  yf = %f  xv = %f, yv = %f\n", trace->XkArray_xfit[0][0], trace->XkArray_yfit[0][0], trace->XkArray_xfit[1][0], trace->XkArray_yfit[1][0]);

        }
}


void FilterAlphaBetaInit(TwiceDataKalman* trace, OnceData* Point)
{
    ;
}



void CaculAlphaBetaX(int ScanNum, float *Alpha, float *Beta)
{
    *Alpha = 2.0 * (2.0 * ScanNum - 1) / (ScanNum * ScanNum + ScanNum);
    *Beta = 6.0 / (ScanNum * ScanNum + ScanNum);
    if (ScanNum >= SCANNUMX)
    {
        *Alpha = 2.0 * (2 * SCANNUMX - 1) / (SCANNUMX * SCANNUMX + SCANNUMX);
        *Beta = 6.0 / (SCANNUMX * SCANNUMX + SCANNUMX);
    }
}

void CaculAlphaBetaY(int ScanNum, float *Alpha, float *Beta)
{
	*Alpha = 2.0 * (2.0 * ScanNum - 1) / (ScanNum * ScanNum + ScanNum);
	*Beta = 6.0 / (ScanNum * ScanNum + ScanNum);
	if (ScanNum >= SCANNUMY)
	{
		*Alpha = 2.0 * (2 * SCANNUMY - 1) / (SCANNUMY * SCANNUMY + SCANNUMY);
		*Beta = 6.0 / (SCANNUMY * SCANNUMY + SCANNUMY);
	}
}