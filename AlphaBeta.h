/*
 * AlphaBeta.h
 *
 *  Created on: 2019Äê11ÔÂ6ÈÕ
 *      Author: 1
 */

#ifndef ALPHABETA_H_
#define ALPHABETA_H_


#include "DataStructure.h"

extern void CaculAlphaBetaX(int ScanNum, float *Alpha, float *Beta);
extern void CaculAlphaBetaY(int ScanNum, float *Alpha, float *Beta);
extern void FilterAlphaBeta(TwiceDataKalman* trace, OnceData* Point);
extern void FilterAlphaBetaInit(TwiceDataKalman* trace, OnceData* Point);


#endif /* ALPHABETA_H_ */
