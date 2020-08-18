/*
 * Tracking_process.h
 *
 *  Created on: 2019��9��30��
 *      Author: 1
 */

#ifndef TRACKING_PROCESS_H_
#define TRACKING_PROCESS_H_

//#include <trackApp/DataStructure.h>
//#include "mss_mmw.h"

#include "DataStructure.h"
#include "detected_obj.h"
#include <stdbool.h>


typedef struct objectAfterEndProcess_track_parameter_t
{
    /*�Ƿ���Ҫ�����˲� ,false��ʾ�����ã�true��ʾ����*/
    bool trackEnable;
    /*�������*/
    //float detectedPeriodTime;
    /*�жϵ��ƴ���*/
    uint8_t breakEstimateMax;
    /*�ж��ݴ����*/
    uint8_t breakDeleteMax;
    /*�켣��ʼƥ�����*/
    uint8_t startTrackMatchMax;
    /*�켣��ʼ�ж��ݴ����*/
    uint8_t startTrackBreakDelMax;
    /*��������X��Χ��Сֵ*/
    float trackXmin;
    /*��������X��Χ���ֵ*/
    float trackXmax;
    /*��������Y��Χ��Сֵ*/
    float trackYmin;
    /*��������y����*/
    float rY;
    /*��������x������ʼֵ*/
    float rXStart;
    /*��������x����б��*/
    float rXK;
    /*������������x*/
    float qX;
    /*������������x*/
    float qY;
    /*�Ƿ���ԭʼ�㼣*/
    bool orignalPointSaveFlag;
    /*������ԭʼ�㼣��ΧYmin*/
    float oriPointYmin;
    /*������ԭʼ�㼣��ΧYmax*/
    float oriPointYmax;
}objectAfterEndProcessTrackPara,*pObjectAfterEndProcessTrackPara;



extern int32_t TRACEINIT();
extern int32_t TRACEEXIT();
extern uint32_t TRACKING(MmwDemo_DSS_DataPathObj *obj, float CarrierVel, float CarrierAng);
//extern uint32_t TRACKING(MmwDemo_DataPathState *obj, float CarrierVel, float CarrierAng);
extern void TrackPrediction(TwiceDataKalman* trace, float CarrierVel, float CarrierAng, int ScanGap);
extern void CancelTrace(int index, int type);
extern int  Merge(OnceDataProPressObj* ProPressonceTemp);
extern char ToBeorNotToBe(TwiceDataKalman* trace);
extern int16_t LastUpdataNum(uint16_t n, uint16_t TraceState);

extern int16_t OneFlag(int n);
extern float AngleCacul(float x, float y);
extern char InBSDZero(TwiceDataKalman trace);
extern char InBSDZeroNew(TwiceDataKalman trace);
extern char PointInBSDZero(OnceData point);
extern char BSDUPJudegeDW(TwiceDataKalman trace);
extern char XTrendJudegeDW(TwiceDataKalman trace);
extern char FixTrace(TwiceDataKalman* trace);
#endif /* TRACKING_PROCESS_H_ */
