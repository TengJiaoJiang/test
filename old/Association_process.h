/*
 * Association_process.h
 *
 *  Created on: 2019��9��30��
 *      Author: 1
 */

#ifndef ASSOCIATION_PROCESS_H_
#define ASSOCIATION_PROCESS_H_

#include "DataStructure.h"

extern char AssociationJudge(TwiceDataKalman trace, OnceData point, float CarrierVel, float CarrierAng);
extern void Mapping(float xf, float yf, float CarrierVel, float CarrierAng, float *x, float *y);
extern char InGate(TwiceDataKalman trace, OnceData point, float CarrierVel);
extern float GetDistance(float x, float y, float X, float Y);
extern float GetDistanceVEL(float tVEL, float pVEL);
extern char SamDPL(TwiceDataKalman trace, OnceData point);
extern char SamAmp(TwiceDataKalman trace, OnceData point);
extern char SamVel(TwiceDataKalman trace, OnceData point);

extern char InRectangularGate(float x, float y, float X, float Y);
extern float AngGap(float PointAng, float TraceAng);
extern float AngRation(float PointAng, float TraceAng);
extern char SamVelXY(TwiceDataKalman trace, OnceData point);
extern char JudgeSAME(TwiceDataKalman trace, OnceData point);
extern char JudgeReason(TwiceDataKalman trace, OnceData point);
extern float XT2DA(float x, float y);

#endif /* ASSOCIATION_PROCESS_H_ */
