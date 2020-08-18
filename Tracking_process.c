#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "Association_process.h"
#include "DataStructure.h"
#include "AlphaBeta.h"
#include "Tracking_process.h"




extern FILE *fWrite;



#define   DISTENCELIMIT  (0.7)
#define   NUMSAM         (3) 
#define   OncePointNum   (50)
#define   TraceConfNum   (20)       //(30)  
#define   TraceTempNum   (30)       //(30)
#define   PI             (3.141592654)
#define   LOSTNUMBER     (7)        //  4
#define   LOSTNUMBERConfirm (7)    //  20


extern uint8_t  g_carSpeedThreshold; 
#define   VEL_GATE			 (15.0) //   KM/H		V_car < 20 KM/H
#define   VEL_GATE_ONE       (15.0) //   KM/H		V_car > 20 KM/H &&	V_car < 30 KM/H
#define   VEL_GATE_TWO       (15.0) //   KM/H		V_car > 30 KM/H &&   V_car < 100 KM/H
#define   VEL_GATE_THR       (20.0) //   KM/H		V_car > 100 KM/H

#define   SCANNUMBER        (16)    //(8) (6)       
#define   CONFNUMBER        (6)     //(5) (4)    

#define   MergeAngle        (1)    
#define   MergeRangeIdx     (1)
#define   MergeDopplerIdx   (1)


// #define   MergeAngleBSD        (0.001)     //
// #define   MergeRangeIdxBSD     (0.001)     // 
// #define   MergeDopplerIdxBSD   (10)        // 

// #define   MergeXBSD            (0.001)     //
// #define   MergeYBSD            (0.001)     // 
// #define   MergeVBSD            (10)        //

#define   TarVelMax            (130.0)        //

TwiceDataKalman *TraceConf;
TwiceDataKalman *TraceTemp;
BSDWARING *BSDWARING_obj;                       //  alongside
BSDWARING *BSDWARING_objysml0;                  //  alongside
BSDWARINGcc *BSDWARING_ccobj;                   //  overtake

char BSDWARINGFLAG = 0;                         //  waring BSD
char BSDWARINGFLAGbj = 0;                       //  waring alongside
char BSDWARINGFLAGbjysml0 = 0;                       //  waring alongside
char BSDWARINGFLAGcc = 0;                       //  waring alongside
//OnceDataAssociation* tempAcc;

uint16_t ScanGap = 0;

OnceData *OnceDataArr;

static int16_t TARGETNUM = 0;
static int16_t PointIDEX = -1;

static int16_t TARGETTEMPNUM = 0;
static int16_t PointTEMPIDEX = -1;


#define   BSD_X_left     (0.5)
#define   BSD_X_right    (3.5)

#define   BSD_Y_left     (-2.0)
#define   BSD_Y_right    (5.0)


#define   NJZeroWx   (1.5)   
#define   NJZeroWy   (4.5)
#define   NJZeroWv   (1.5)





/************************************************************************/
/*

 */
/************************************************************************/
int32_t TRACEINIT()
{
	BSDWARING_obj = (BSDWARING *)malloc(sizeof(BSDWARING));
	BSDWARING_objysml0 = (BSDWARING *)malloc(sizeof(BSDWARING));
    BSDWARING_ccobj = (BSDWARINGcc *)malloc(sizeof(BSDWARINGcc));
    TraceConf = (TwiceDataKalman *)malloc(TraceConfNum * sizeof(TwiceDataKalman));
    TraceTemp = (TwiceDataKalman *)malloc(TraceTempNum * sizeof(TwiceDataKalman));
    OnceDataArr = (OnceData *)malloc(OncePointNum * sizeof(OnceData));
    if (TraceConf == NULL || TraceTemp == NULL || OnceDataArr == NULL)
    {
        return -1;
    }

    memset(TraceConf, 0, sizeof(TwiceDataKalman) * TraceConfNum);
    memset(TraceTemp, 0, sizeof(TwiceDataKalman) * TraceTempNum);
    memset(OnceDataArr, 0, sizeof(OnceData) * OncePointNum);
	memset(BSDWARING_obj, 0, sizeof(BSDWARING));
	memset(BSDWARING_objysml0, 0, sizeof(BSDWARING));
    memset(BSDWARING_ccobj, 0, sizeof(BSDWARINGcc));
    return 0;
}


/********************************************************************
 *                                                                                       *
 *********************************************************************/
//uint32_t TRACKING(MmwDemo_DataPathState *obj, float CarrierVel, float CarrierAng)
uint32_t TRACKING(MmwDemo_DSS_DataPathObj *obj, float CarrierVel, float CarrierAng)
{
    int16_t OncePointNumber = 0;
    int16_t i = 0, j=0, k = 0;
    //    int16_t Track_NUM = 0;
    int16_t ScanGapOld = 0;                 //change 20191127
    int16_t OncePointNumberClearn = 0;
    int16_t tempNum = 0;
    int16_t indexBest = -1;
    float disNear = 0;
	float disNearVel = 0;
    OnceDataAssociation tempAcc[NUMSAM];
    OnceData tempAccBest;
    uint8_t  FarShortFlag = 0;

	uint8_t  VBSDbj = 0; // target num whice  abs(val) < 10.0 km/h
	uint8_t  VBSDbjybig0 = 0; // 
	uint8_t  VBSDbjysml0 = 0; //
    uint8_t  VBSDbjvbig0 = 0;
    uint8_t  VBSDbjvsml0 = 0;

	uint8_t  VBSDbjysml0vsml7 = 0;

    float xTempTemp = 0;
    float yTempTemp = 0;
	float vTempTemp = 0;
    float disTempTemp = 0;                   //change 20200331
    float angTempTemp = 0;                   //change 20200331

	float disRes = 0;
	float velRes = 0;
	float VEL_GATEChose = 0;

	if (CarrierVel > 100)
	{
		VEL_GATEChose = VEL_GATE_THR;
	}
	else if(CarrierVel > 30)
	{
		VEL_GATEChose = VEL_GATE_TWO;
	}
	else if (CarrierVel > 20)
	{
		VEL_GATEChose = VEL_GATE_ONE;
	}
	else
	{
		VEL_GATEChose = VEL_GATE;
	}

    OnceDataProPressObj ProPressonceTemp;
    ProPressonceTemp.Num = 0;

    ScanGap = obj->xyzQFormat;           
    FarShortFlag = obj->subFramIndx;
	
	if (FarShortFlag * 1 == 1)
	{
		disRes = 0.335;
		velRes = 0.285;
	}
	else
	{
		disRes = 0.815;
		velRes = 0.285;
	}
    OncePointNumber = obj->numDetObj;    
    if (OncePointNumber > OncePointNum)
    {
        OncePointNumber = OncePointNum;
    }

    memset(OnceDataArr, 0, sizeof(OnceData) * OncePointNum);

	fprintf(fWrite, "vel_Car(km/h) %f  FarShortFlag  = %d  ScanGap = %d OncePointNumber = %d VEL_GATEChose %f\n", CarrierVel, FarShortFlag * 1, ScanGap, OncePointNumber, VEL_GATEChose);
	
    for (i = 0; i< OncePointNumber; i++)
    {
        //if (FarShortFlag * 1 == 1)
        //{
        //    disTempTemp = obj->detObj2D[i].rangeIdx * 0.335;
        //    vTempTemp = obj->detObj2D[i].dopplerIdx * 0.2853;
        //}
        //else
        //{
        //    disTempTemp = obj->detObj2D[i].rangeIdx * 0.8156;
        //    vTempTemp = obj->detObj2D[i].dopplerIdx * 0.2853;
        //}

		disTempTemp = obj->detObj2D[i].rangeIdx * disRes;
		vTempTemp = obj->detObj2D[i].dopplerIdx * velRes;

        angTempTemp = obj->detObj2D[i].angle * 0.1;
        // xTempTemp = -1.0 * disTempTemp * sinf((float)(PI * angTempTemp / 180.0));
        // yTempTemp = disTempTemp * cosf((float)(PI * angTempTemp / 180.0));

        xTempTemp = -1.0 * (obj->detObj2D[i].x) / 10.0;

        yTempTemp = obj->detObj2D[i].y / 10.0;

		//vTempTemp = obj->detObj2D[i].dopplerIdx * 0.315;
		//fprintf(fWrite, "__  v(kmh) = %f x = %f, y = %f ,  ang = %f, dis = %f, fd = %d\n", vTempTemp * 3.6, xTempTemp, yTempTemp,  angTempTemp, disTempTemp, obj->detObj2D[i].dopplerIdx);
		
		//if (xTempTemp < -4 || xTempTemp > 8)
		//if (xTempTemp < LaneRightAfterLeft || xTempTemp > LaneCloseNearRight)
		if (xTempTemp < LaneRightAfterLeft || xTempTemp > 10.0)
        {
			//fprintf(fWrite, "LC PASS \n");
            continue;
        }

        if (yTempTemp > -2 && yTempTemp < 5)
        {
            if ( xTempTemp > 5.0)
            {
				//fprintf(fWrite,"LC PASS \n");
                continue;
            }
			if (xTempTemp > 0.5 && xTempTemp < 3.5)//BSD
			{
				
				if (fabs(vTempTemp) > 50)
				{
					//fprintf(fWrite,">50m/s PASS \n");
					continue;
				}
			}
        }

        if (xTempTemp > LaneRightAfterLeft && xTempTemp < LaneRightAfterRight && yTempTemp <= 2.0)
        {
            continue; 
        }

		if (xTempTemp > LaneRightAfterLeft && xTempTemp < LaneRightAfterRight && yTempTemp <= 5.0)
		{
			if (fabs(vTempTemp * 3.6) > 120)
			{
				continue;
			}
			
		}

		if(yTempTemp > 5)
		{
			if(vTempTemp * 3.6 < -10.0)
			{
				continue;	
			}
		}

        if (yTempTemp <= 0)
        {
			if (fabs(CarrierVel - vTempTemp * 3.6) > TarVelMax)
			{
				continue;
			}
            
        }
		else
		{
			if (fabs(CarrierVel + vTempTemp * 3.6) > TarVelMax)
			{
				continue;
			}
			
		}

		if (xTempTemp > 3.5 && yTempTemp < 0)
		{
			//fprintf(fWrite,"LC PASS \n");
			continue;
		}


		//fprintf(fWrite, "_ v(km/h) x,y R rindex-> %f  %f %f %f %d\n", vTempTemp * 3.6, xTempTemp, yTempTemp, disTempTemp, obj->detObj2D[i].rangeIdx);

		//fprintf(fWrite,"LC good \n");
        ProPressonceTemp.Prodata[ProPressonceTemp.Num].v	= vTempTemp;   //m/s
        ProPressonceTemp.Prodata[ProPressonceTemp.Num].dis	= disTempTemp; //obj->detObj2D[i].rangeIdx * obj->rangeResolution;
        ProPressonceTemp.Prodata[ProPressonceTemp.Num].ang	= angTempTemp; //obj->detObj2D[i].angle * 0.1;
        ProPressonceTemp.Prodata[ProPressonceTemp.Num].x	= xTempTemp;  //-1.0 * ProPressonceTemp.Prodata[ProPressonceTemp.Num].dis * sinf((float)(PI * ProPressonceTemp.Prodata[ProPressonceTemp.Num].ang / 180.0));     //m
        ProPressonceTemp.Prodata[ProPressonceTemp.Num].y	= yTempTemp; //ProPressonceTemp.Prodata[ProPressonceTemp.Num].dis * cosf((float)(PI * ProPressonceTemp.Prodata[ProPressonceTemp.Num].ang / 180.0));     //m

        ProPressonceTemp.Prodata[ProPressonceTemp.Num].a	= obj->detObj2D[i].peakVal;
        ProPressonceTemp.Prodata[ProPressonceTemp.Num].used = 0;

        ProPressonceTemp.usless[ProPressonceTemp.Num]       = 0;
        ProPressonceTemp.Num                                = ProPressonceTemp.Num + 1;
    }

    //change20200423
    //y<0 Vtar = Vcar - Vmea    fabs|Vtar| < Gate
    //y>0 Vtar = Vcar + Vmea    fabs|Vtar| < Gate
    for (i = 0; i< ProPressonceTemp.Num; i++)
    {
		//fprintf(fWrite, "v = %f \n", ProPressonceTemp.Prodata[i].v * 3.6);
        if (ProPressonceTemp.Prodata[i].y < 0)  //
        {
            if (fabs(CarrierVel / 3.6 - ProPressonceTemp.Prodata[i].v) < (VEL_GATEChose / 3.6)) 
            {
                ProPressonceTemp.usless[i] = 1;
            }

			if ((CarrierVel / 3.6 - ProPressonceTemp.Prodata[i].v) < 0.0) 
			{
				ProPressonceTemp.usless[i] = 1;
			}
        }
        else //y>=0
        {
            if (fabs(CarrierVel / 3.6 + ProPressonceTemp.Prodata[i].v) < (VEL_GATEChose / 3.6)) 
            {
                ProPressonceTemp.usless[i] = 1;
				//fprintf(fWrite, " chesu %f  PASS y > 0\n", CarrierVel / 3.6);
            }

			if ((CarrierVel / 3.6 + ProPressonceTemp.Prodata[i].v) < 0.0) 
			{
				ProPressonceTemp.usless[i] = 1;
				//fprintf(fWrite, "PASS y > 0\n");
			}
        }
    }

    //Merge(&ProPressonceTemp);    

    for (i = 0; i< ProPressonceTemp.Num; i++)
    {
        if (ProPressonceTemp.usless[i] == 0)
        {
            OnceDataArr[OncePointNumberClearn].ang  = ProPressonceTemp.Prodata[i].ang;
            OnceDataArr[OncePointNumberClearn].dis  = ProPressonceTemp.Prodata[i].dis;
            OnceDataArr[OncePointNumberClearn].v    = ProPressonceTemp.Prodata[i].v;
            OnceDataArr[OncePointNumberClearn].x    = ProPressonceTemp.Prodata[i].x;
            OnceDataArr[OncePointNumberClearn].y    = ProPressonceTemp.Prodata[i].y;
            OnceDataArr[OncePointNumberClearn].a    = ProPressonceTemp.Prodata[i].a;
            OncePointNumberClearn                   = OncePointNumberClearn + 1;
        }
    }


	fprintf(fWrite, "\n\n_ focus num = %d\n", OncePointNumberClearn);

	for ( i = 0; i < OncePointNumberClearn; i++)
	{
		fprintf(fWrite, "__ beffocus(v(km/h),x,y) %f %f %f\n", OnceDataArr[i].v * 3.6, OnceDataArr[i].x, OnceDataArr[i].y);
	}

    //alongsideprocess BSDWARING_obj
	for (i = 0; i< ScanGap - 1; i++)
	{
		BSDWARING_obj->TraceState   = BSDWARING_obj->TraceState << 1;
		BSDWARING_obj->iLostNum     = BSDWARING_obj->iLostNum + 1;
		BSDWARING_obj->iScanNum     = BSDWARING_obj->iScanNum + 1;
		BSDWARING_obj->iysmal0 = BSDWARING_obj->iysmal0 << 1;
		BSDWARING_obj->iybig0 = BSDWARING_obj->iybig0 << 1;
	}

	for (i = 0; i < OncePointNumberClearn; i++)
	{
		if (PointInBSDZero(OnceDataArr[i]) == 1)
		{
			if (OnceDataArr[i].v * 3.6 < 0 && OnceDataArr[i].y < 0 && OnceDataArr[i].v * 3.6 > -35.0)
			{
				fprintf(fWrite, "__ (v(km/h),x,y) %f %f %f &&&&& <0 &&&&&&&\n", OnceDataArr[i].v * 3.6, OnceDataArr[i].x, OnceDataArr[i].y);
				VBSDbj = VBSDbj + 1;
				VBSDbjysml0 = VBSDbjysml0 + 1;
			}
			//if (OnceDataArr[i].v * 3.6 > 0 && OnceDataArr[i].y > 0)
            if(OnceDataArr[i].v * 3.6 > 0 && OnceDataArr[i].y > 0 && OnceDataArr[i].v * 3.6 < 35.0)
            {
				fprintf(fWrite, "__ (v(km/h),x,y) %f %f %f &&&&& >0 &&&&&&&\n", OnceDataArr[i].v * 3.6, OnceDataArr[i].x, OnceDataArr[i].y);
                VBSDbj = VBSDbj + 1;
				VBSDbjybig0 = VBSDbjybig0 + 1;
            }

			if (OnceDataArr[i].y < 0.0 && OnceDataArr[i].v * 3.6 < 7.2 && OnceDataArr[i].v * 3.6 > 0)
			{
				VBSDbjysml0vsml7 = VBSDbjysml0vsml7 + 1;
				//fprintf(fWrite, " __________________________________________________near have bingjian data \n", OnceDataArr[i].v * 3.6, OnceDataArr[i].x, OnceDataArr[i].y);
			}
		}
	}

	if (VBSDbjysml0 > 0)
	{
		BSDWARING_obj->iysmal0 = BSDWARING_obj->iysmal0 << 1;
		BSDWARING_obj->iysmal0 = BSDWARING_obj->iysmal0 + 1;
	}
	else
	{
		BSDWARING_obj->iysmal0 = BSDWARING_obj->iysmal0 << 1;
	}

	if (VBSDbjybig0 > 0)
	{
		BSDWARING_obj->iybig0 = BSDWARING_obj->iybig0 << 1;
		BSDWARING_obj->iybig0 = BSDWARING_obj->iybig0 + 1;
	}
	else
	{
		BSDWARING_obj->iybig0 = BSDWARING_obj->iybig0 << 1;
	}


	if (VBSDbjysml0 >= 2 || VBSDbjybig0 >= 2 || (VBSDbjysml0 >= 1 && VBSDbjybig0 >= 1) /*|| VBSDbjysml0vsml7 >= 1*/)
	{
		
		if (VBSDbjysml0 >= 2)
		{
			fprintf(fWrite, "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&BINGJIAN&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&    A\n");
		}
		if (VBSDbjybig0 >= 2)
		{
			fprintf(fWrite, "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&BINGJIAN&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&    B\n");
		}
		if ((VBSDbjysml0 >= 1 && VBSDbjybig0 >= 1))
		{
			fprintf(fWrite, "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&BINGJIAN&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&    C\n");
		}
		if (VBSDbjysml0vsml7 >= 1)
		{
			fprintf(fWrite, "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&BINGJIAN&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&    D\n");
		}
		BSDWARING_obj->TraceState = BSDWARING_obj->TraceState << 1;
		BSDWARING_obj->TraceState = BSDWARING_obj->TraceState + 1;
		BSDWARING_obj->iLostNum = 0;
		BSDWARING_obj->iScanNum = BSDWARING_obj->iScanNum + 1;
		BSDWARING_obj->iUpdaNum = BSDWARING_obj->iUpdaNum + 1;
	}
	else
	{
		if (BSDWARINGFLAGbj == 1)
		{
			if (VBSDbjysml0 >= 1 || VBSDbjybig0 >= 1 || VBSDbjysml0vsml7 >= 1)
			{
				if (VBSDbjysml0 >= 1)
				{
					fprintf(fWrite, "----------------------------------BINGJIAN only 1 y<0 v<0----------------------------------\n");
				}
				if (VBSDbjybig0 >= 1)
				{
					fprintf(fWrite, "----------------------------------BINGJIAN only 1  y>0 v>0----------------------------------\n");
				}
				if (VBSDbjysml0vsml7 >= 1)
				{
					fprintf(fWrite, "----------------------------------BINGJIAN only 1  y<0 7>v>0 ----------------------------------\n");
				}
				BSDWARING_obj->TraceState = BSDWARING_obj->TraceState << 1;
				BSDWARING_obj->TraceState = BSDWARING_obj->TraceState + 1;
				BSDWARING_obj->iLostNum = 0;
				BSDWARING_obj->iScanNum = BSDWARING_obj->iScanNum + 1;
				BSDWARING_obj->iUpdaNum = BSDWARING_obj->iUpdaNum + 1;
			}
			else
			{
				fprintf(fWrite, "----------------------------------BINGJIAN-------%d-----only 1------lost----------\n", BSDWARING_obj->iLostNum);
				BSDWARING_obj->TraceState = BSDWARING_obj->TraceState << 1;
				BSDWARING_obj->iLostNum = BSDWARING_obj->iLostNum + 1;
				BSDWARING_obj->iScanNum = BSDWARING_obj->iScanNum + 1;
			}
		}
		else
		{
			//fprintf(fWrite, "----------------------------------BINGJIAN-------%d-------lost--------------\n", BSDWARING_obj->iLostNum);
			BSDWARING_obj->TraceState = BSDWARING_obj->TraceState << 1;
			BSDWARING_obj->iLostNum = BSDWARING_obj->iLostNum + 1;
			BSDWARING_obj->iScanNum = BSDWARING_obj->iScanNum + 1;
		}
	}

	
    if (LastUpdataNum(10, BSDWARING_obj->TraceState) >= 5 /*&& LastUpdataNum(12, BSDWARING_obj->iysmal0) > 1*/) // 5/4        8/6
	{
		BSDWARING_obj->Waring = 1;
		BSDWARINGFLAGbj = 1;
		fprintf(fWrite, "BSD alongside ******************************\n");

	}
	fprintf(fWrite, "-BINGJIAN-------%d-lost\n", BSDWARING_obj->iLostNum);
	fprintf(fWrite, "-BINGJIAN-------%d-upda\n", LastUpdataNum(10, BSDWARING_obj->TraceState));
	fprintf(fWrite, "-BINGJIAN-------%d-stat\n", BSDWARING_obj->TraceState * 1);
	fprintf(fWrite, "-BINGJIAN-------%d-flag\n", BSDWARINGFLAGbj * 1);
	fprintf(fWrite, "-BINGJIAN-------%d-|v|<7.2\n", VBSDbjysml0vsml7 * 1);

	if (BSDWARING_obj->Waring == 1 && BSDWARING_obj->iLostNum <= 33) // 5/4
	{
		fprintf(fWrite, "BSD alongside *********************************\n");

	}

	if (BSDWARING_obj->Waring == 1 && (VBSDbjysml0 >= 2 || VBSDbjybig0 >= 2 || VBSDbjysml0vsml7 >= 1))
	{
		BSDWARING_obj->iLostNum = 0;
	}

	if (BSDWARING_obj->Waring == 0)
	{
		fprintf(fWrite, "BSD wu  alongside *********************************\n");
	}

	if (BSDWARING_obj->Waring == 1 && BSDWARING_obj->iLostNum >= 33)
	{
		BSDWARING_obj->Waring = 0;
		BSDWARINGFLAGbj = 0;
	}


    // overtake pracess  BSDWARING_ccobj
    for (i = 0; i< ScanGap - 1; i++)
	{
        //fprintf(fWrite, "** ()()()(())()()()()()()()  \n");
		BSDWARING_ccobj->TraceState   = BSDWARING_ccobj->TraceState << 1;
		BSDWARING_ccobj->iLostNum     = BSDWARING_ccobj->iLostNum + 1;
		BSDWARING_ccobj->iScanNum     = BSDWARING_ccobj->iScanNum + 1;
	}

	for (i = 0; i < OncePointNumberClearn; i++)
	{
		if (PointInBSDZero(OnceDataArr[i]) == 1)
		{
			if (OnceDataArr[i].v * 3.6 < 0 && OnceDataArr[i].y > 0)
			{
				VBSDbjvsml0 = VBSDbjvsml0 + 1;
			}
            if(OnceDataArr[i].v * 3.6 > 0 && OnceDataArr[i].y < 0)
            {
                VBSDbjvbig0 = VBSDbjvbig0 + 1;
            }
		}
	}

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////change 20200727
	if ((VBSDbjvbig0 > 0 && VBSDbjvsml0 > 0) || (VBSDbjvsml0 >= 2))
	{
		BSDWARING_ccobj->TraceState = BSDWARING_ccobj->TraceState << 1;
		BSDWARING_ccobj->TraceState = BSDWARING_ccobj->TraceState + 1;
		BSDWARING_ccobj->iLostNum   = 0;
		BSDWARING_ccobj->iScanNum   = BSDWARING_ccobj->iScanNum + 1;
		BSDWARING_ccobj->iUpdaNum   = BSDWARING_ccobj->iUpdaNum + 1;
	}
	else
	{
		BSDWARING_ccobj->TraceState = BSDWARING_ccobj->TraceState << 1;
		BSDWARING_ccobj->iLostNum   = BSDWARING_ccobj->iLostNum + 1;
		BSDWARING_ccobj->iScanNum   = BSDWARING_ccobj->iScanNum + 1;
	}

	fprintf(fWrite, "** (BSD-cc) v>0 = %d  v<0 = %d\n", VBSDbjvbig0, VBSDbjvsml0);

	fprintf(fWrite, "** (gap=%d scan=%d,lost=%d,updata=%d, last5=%d BSDWARING_obj->TraceState =%d)\n", ScanGap, BSDWARING_ccobj->iScanNum, BSDWARING_ccobj->iLostNum, BSDWARING_ccobj->iUpdaNum, LastUpdataNum(10, BSDWARING_ccobj->TraceState), (BSDWARING_ccobj->TraceState) * 1);
	
    
    if (BSDWARING_ccobj->Waring == 1 && BSDWARING_ccobj->iLostNum <= 33) 
	{
		fprintf(fWrite, "BSD tackover *********************************\n");
	}
    
    if (LastUpdataNum(14, BSDWARING_ccobj->TraceState) >= 6) // 5/3
	{
		BSDWARING_ccobj->Waring = 1;
		BSDWARINGFLAGcc = 1;
		fprintf(fWrite, "BSD tackover ******************************\n");
	}

	if (BSDWARING_ccobj->Waring == 0)
	{
		fprintf(fWrite, "BSD wu tackover *********************************\n");
	}

	//if (BSDWARING_obj->Waring == 1 && BSDWARING_obj->iLostNum >= 33)
	//{
	//	BSDWARING_obj->Waring = 0;
	//	BSDWARINGFLAGcc = 0;
	//}

	if (BSDWARING_ccobj->Waring == 1 && BSDWARING_ccobj->iLostNum >= 33)
	{
		BSDWARING_ccobj->Waring = 0;
		BSDWARINGFLAGcc = 0;
	}

    if(BSDWARINGFLAGcc == 1 || BSDWARINGFLAGbj == 1)
    {
        if(BSDWARINGFLAGcc == 1)
        {
            fprintf(fWrite, "BSD tackover ******************************\n");
        }
        if(BSDWARINGFLAGbj == 1)
        {
            fprintf(fWrite, "BSD alongside ******************************\n");
        }
        BSDWARINGFLAG = 1;
        fprintf(fWrite, "BSD waring ******************************\n");
    }

    if(BSDWARINGFLAGcc == 0 && BSDWARINGFLAGbj == 0 /*&& BSDWARINGFLAGbjysml0 ==0*/)
    {
        BSDWARINGFLAG = 0;
        fprintf(fWrite, "BSD no waring ******************************\n");
    }


	Merge(&ProPressonceTemp);
	for (i = 0; i < ProPressonceTemp.Num; i++)
	{
		if (ProPressonceTemp.usless[i] == 0)
		{
			if (fabs(ProPressonceTemp.Prodata[i].y) < 0.05)
			{
				ProPressonceTemp.usless[i] = 1;
			}
		}
	}

	memset(OnceDataArr, 0, sizeof(OnceData) * OncePointNum);
	OncePointNumberClearn = 0;
	for (i = 0; i< ProPressonceTemp.Num; i++)
	{
		if (ProPressonceTemp.usless[i] == 0)
		{
			OnceDataArr[OncePointNumberClearn].ang = ProPressonceTemp.Prodata[i].ang;
			OnceDataArr[OncePointNumberClearn].dis = ProPressonceTemp.Prodata[i].dis;
			OnceDataArr[OncePointNumberClearn].v = ProPressonceTemp.Prodata[i].v;
			OnceDataArr[OncePointNumberClearn].x = ProPressonceTemp.Prodata[i].x;
			OnceDataArr[OncePointNumberClearn].y = ProPressonceTemp.Prodata[i].y;
			OnceDataArr[OncePointNumberClearn].a = ProPressonceTemp.Prodata[i].a;
			OncePointNumberClearn = OncePointNumberClearn + 1;
		}
	}



	fprintf(fWrite, "\n\n_ focus num = %d\n", OncePointNumberClearn);

	for (i = 0; i < OncePointNumberClearn; i++)
	{
		fprintf(fWrite, "__  ** AFTER(v(km/h),x,y) %f %f %f\n", OnceDataArr[i].v * 3.6, OnceDataArr[i].x, OnceDataArr[i].y);
	}

    {

        //confirm trace process
        for (i = 0; i < TraceConfNum; i++)
        {
            if (TraceConf[i].Used * 1 == 1)
            {
                TrackPrediction(&TraceConf[i], CarrierVel, CarrierAng, ScanGap);
				fprintf(fWrite, "(ID,flag, v,x,y) %d %d %f %f %f find point\n", TraceConf[i].ID * 1, TraceConf[i].TraceFlag * 1, TraceConf[i].DPLVel, TraceConf[i].XkArray_xfit[0][0], TraceConf[i].XkArray_yfit[0][0]);
                memset((char *)&tempAcc, 0, sizeof(OnceDataAssociation) * NUMSAM);
                memset((char *)&tempAccBest, 0, sizeof(OnceData));
                tempNum = 0;
                for (j = 0; j < OncePointNumberClearn; j++)
                {
                    if (OnceDataArr[j].used == 0)
                    {
                        if (AssociationJudge(TraceConf[i], OnceDataArr[j], CarrierVel, CarrierAng) * 1 == 1)
                        {
                            tempAcc[tempNum].index = j;
                            tempAcc[tempNum].dis = OnceDataArr[j].dis;
                            tempAcc[tempNum].ang = OnceDataArr[j].ang;
                            tempAcc[tempNum].v = OnceDataArr[j].v;
                            tempAcc[tempNum].x = OnceDataArr[j].x;
                            tempAcc[tempNum].y = OnceDataArr[j].y;
                            tempAcc[tempNum].a = OnceDataArr[j].a;
                            tempAcc[tempNum].used = OnceDataArr[j].used;
                            tempNum = tempNum + 1;
							
                            if (tempNum >= NUMSAM)
                            {
                                break;
                            }
                        }
                    }
                }

                if (tempNum >0 )
                {
                    indexBest = 0;
                    //disNear = GetDistance(TraceConf[i].XkArray_xnxt[0][0], TraceConf[i].XkArray_ynxt[0][0], tempAcc[indexBest].x, tempAcc[indexBest].y);
					disNearVel = GetDistanceVEL(TraceConf[i].DPLVel, tempAcc[indexBest].v);
					fprintf(fWrite, "		(ID, v) %d %f Point v = %f disNearVel = %f\n", TraceConf[i].ID, TraceConf[i].DPLVel, tempAcc[indexBest].v, disNearVel);
					for (k = 1; k < tempNum; k++)
                    {
                        //if (disNear > GetDistance(TraceConf[i].XkArray_xnxt[0][0], TraceConf[i].XkArray_ynxt[0][0], tempAcc[k].x, tempAcc[k].y))
						if (disNearVel > GetDistanceVEL(TraceConf[i].DPLVel, tempAcc[k].v))
						{
                            //disNear = GetDistance(TraceConf[i].XkArray_xnxt[0][0], TraceConf[i].XkArray_ynxt[0][0], tempAcc[k].x, tempAcc[k].y);
							disNearVel = GetDistanceVEL(TraceConf[i].DPLVel, tempAcc[k].v);
							indexBest = k;
							fprintf(fWrite, "		(ID, v) %d %f Point v = %f disNearVel = %f\n", TraceConf[i].ID, TraceConf[i].DPLVel, tempAcc[indexBest].v, disNearVel);
							
                        }
                    }
                    OnceDataArr[tempAcc[indexBest].index].used = 1;

                    tempAccBest.dis = tempAcc[indexBest].dis;
                    tempAccBest.ang = tempAcc[indexBest].ang;
                    tempAccBest.v = tempAcc[indexBest].v;
                    tempAccBest.x = tempAcc[indexBest].x;
                    tempAccBest.y = tempAcc[indexBest].y;
                    tempAccBest.a = tempAcc[indexBest].a;
                    tempAccBest.used = tempAcc[indexBest].used;
					fprintf(fWrite, "		find BEST (v,x,y) %f %f %f\n", tempAccBest.v * 3.6, tempAccBest.x, tempAccBest.y);
					for (k = 0; k<ScanGap - 1; k++)//201911251601
					{
						
						TraceConf[i].TraceState = TraceConf[i].TraceState << 1;
						TraceConf[i].iScanNum = TraceConf[i].iScanNum + 1;
					}
					TraceConf[i].iScanNum = TraceConf[i].iScanNum + 1;

                    FilterAlphaBeta(&TraceConf[i], &tempAccBest);

                    //change 20191128 start
                    TraceConf[i].Xpast[0] = TraceConf[i].Xpast[1];
                    TraceConf[i].Xpast[1] = TraceConf[i].Xpast[2];
                    TraceConf[i].Xpast[2] = TraceConf[i].Xpast[3];
                    TraceConf[i].Xpast[3] = TraceConf[i].Xpast[4];
                    TraceConf[i].Xpast[4] = tempAccBest.x;
                    //TraceConf[i].Xpast[4] = TraceConf[i].XkArray_xfit[0][0];

                    //TraceConf[i].Ypast[0] = TraceConf[i].Ypast[1];
                    //TraceConf[i].Ypast[1] = TraceConf[i].Ypast[2];
                    //TraceConf[i].Ypast[2] = TraceConf[i].Ypast[3];
                    //TraceConf[i].Ypast[3] = TraceConf[i].Ypast[4];
                    //TraceConf[i].Ypast[4] = TraceConf[i].XkArray_yfit[0][0];
                    //change 20191128 end

                    TraceConf[i].dis = tempAccBest.dis;             //
                    TraceConf[i].ang = tempAccBest.ang;             //

                    TraceConf[i].DPLVel = tempAccBest.v;            //20191112

                    //
                    TraceConf[i].TraceState = TraceConf[i].TraceState << 1;
                    TraceConf[i].TraceState = TraceConf[i].TraceState + 1;
					TraceConf[i].Corred = 1;
					TraceConf[i].x_corr = tempAccBest.x;
					TraceConf[i].y_corr = tempAccBest.y;
                    TraceConf[i].Updata = 1;
                    TraceConf[i].iLostNum = 0;

                    TraceConf[i].iUpdaNum = TraceConf[i].iUpdaNum + 1;   //20191122
					fprintf(fWrite, "			(ID,flag, v,x,y) %d %d %f %f %f state = %d  lostnum = %d scan num = %d updata num = %d\n", TraceConf[i].ID, TraceConf[i].TraceFlag * 1, TraceConf[i].DPLVel, TraceConf[i].XkArray_xfit[0][0], TraceConf[i].XkArray_yfit[0][0], TraceConf[i].TraceState * 1, TraceConf[i].iLostNum, TraceConf[i].iScanNum, TraceConf[i].iUpdaNum);

                }
                else
                {
					fprintf(fWrite, "						-----------------find None point\n");
                }
            }
        }

        //
        for (i = 0; i < TraceConfNum; i++)
        {
            if (TraceConf[i].Used == 1)
            {
                if (TraceConf[i].iLostNum >= LOSTNUMBERConfirm)
                {
                    CancelTrace(TraceConf[i].ID - 1, 1);
                }

                if (TraceConf[i].Updata == 1)
                {
                    TraceConf[i].Updata = -1;
                }
                else
                {
                    
					for (k = 0; k<ScanGap; k++)//201911251601
					{
						TraceConf[i].TraceState = TraceConf[i].TraceState << 1;
						TraceConf[i].iScanNum = TraceConf[i].iScanNum + 1;
					}
					TraceConf[i].iLostNum = TraceConf[i].iLostNum + ScanGap;
					fprintf(fWrite, "			(ID,flag, v,x,y) %d %d %f %f %f state = %d  lostnum = %d scan num = %d updata num = %d\n", TraceConf[i].ID, TraceConf[i].TraceFlag * 1, TraceConf[i].DPLVel, TraceConf[i].XkArray_xfit[0][0], TraceConf[i].XkArray_yfit[0][0], TraceConf[i].TraceState * 1, TraceConf[i].iLostNum, TraceConf[i].iScanNum, TraceConf[i].iUpdaNum);

                    FilterAlphaBeta(&TraceConf[i], NULL);

					FixTrace(&TraceConf[i]);

                    //change 20191128 start
                    TraceConf[i].Xpast[0] = TraceConf[i].Xpast[1];
                    TraceConf[i].Xpast[1] = TraceConf[i].Xpast[2];
                    TraceConf[i].Xpast[2] = TraceConf[i].Xpast[3];
                    TraceConf[i].Xpast[3] = TraceConf[i].Xpast[4];
                    TraceConf[i].Xpast[4] = TraceConf[i].XkArray_xfit[0][0];
					TraceConf[i].Corred = 0;
                    //TraceConf[i].Ypast[0] = TraceConf[i].Ypast[1];
                    //TraceConf[i].Ypast[1] = TraceConf[i].Ypast[2];
                    //TraceConf[i].Ypast[2] = TraceConf[i].Ypast[3];
                    //TraceConf[i].Ypast[3] = TraceConf[i].Ypast[4];
                    //TraceConf[i].Ypast[4] = TraceConf[i].XkArray_yfit[0][0];
                    //change 20191128 end
                }
            }
        }

        
        for (i = 0; i < TraceTempNum; i++)
        {
            if (TraceTemp[i].Used * 1 == 1)
            {
				
                memset((char *)&tempAcc, 0, sizeof(OnceDataAssociation) * NUMSAM);
                memset((char *)&tempAccBest, 0, sizeof(OnceData));
                tempNum = 0;

                TrackPrediction(&TraceTemp[i], CarrierVel, CarrierAng, ScanGap);

				fprintf(fWrite, "(ID,flag, v,x,y) %d %d %f %f %f find point\n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0]);
                for (j = 0; j < OncePointNumberClearn; j++)
                {
                    if (OnceDataArr[j].used == 0)
                    {
						if (TraceTemp[i].TraceFlag * 1 == 0 && InBSDZero(TraceTemp[i]) * 1 == 1) 
						{
							if (fabs(OnceDataArr[j].y - TraceTemp[i].XkArray_yfit[0][0]) / ((ScanGap + TraceTemp[i].iLostNum) * TimeGap) > 25.0)
							{
								fprintf(fWrite, "(cacul = %f  gate = %f not cool \n", fabs(OnceDataArr[j].y - TraceTemp[i].XkArray_yfit[0][0]) / (ScanGap * TimeGap), 25.0);
								continue;
							}
							else
							{
								fprintf(fWrite, "(cacul = %f  gate = %f cool \n", fabs(OnceDataArr[j].y - TraceTemp[i].XkArray_yfit[0][0]) / (ScanGap * TimeGap), 25.0);

							}
							if (fabs(OnceDataArr[j].x - TraceTemp[i].XkArray_xfit[0][0]) / ((ScanGap + TraceTemp[i].iLostNum) * TimeGap) > 13.5)
							{
								fprintf(fWrite, "(cacul = %f  gate = %f not cool \n", fabs(OnceDataArr[j].x - TraceTemp[i].XkArray_xfit[0][0]) / ((ScanGap + TraceTemp[i].iLostNum) * TimeGap), 13.5);
								continue;
							}
							else
							{
								fprintf(fWrite, "(cacul = %f  gate = %f cool \n", fabs(OnceDataArr[j].x - TraceTemp[i].XkArray_xfit[0][0]) / ((ScanGap + TraceTemp[i].iLostNum) * TimeGap), 13.5);
							}
						}
						if (TraceTemp[i].TraceFlag * 1 == 0 && InBSDZero(TraceTemp[i]) * 1 != 1) 
						{
							if (fabs(OnceDataArr[j].y - TraceTemp[i].XkArray_yfit[0][0]) / ((ScanGap + TraceTemp[i].iLostNum) * TimeGap) > 45.0)
							{
								//fprintf(fWrite, "(ID,flag, v,x,y) %d %d %f %f %f Yv %f gate %f \n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0], fabs(OnceDataArr[j].y - TraceTemp[i].XkArray_yfit[0][0]) / (ScanGap * TimeGap), 45);
								continue;
							}
							else
							{
								//fprintf(fWrite, "(ID,flag, v,x,y) %d %d %f %f %f Yv  gate %f \n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0], fabs(OnceDataArr[j].y - TraceTemp[i].XkArray_yfit[0][0]) / (ScanGap * TimeGap),45);

							}
							if (fabs(OnceDataArr[j].x - TraceTemp[i].XkArray_xfit[0][0]) / ((ScanGap + TraceTemp[i].iLostNum) * TimeGap) > 15.0)
							{
								//fprintf(fWrite, "(ID,flag, v,x,y) %d %d %f %f %f xv %f gate %f ScanGap = %d xdif = %f lost = %d\n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0], fabs(OnceDataArr[j].x - TraceTemp[i].XkArray_xfit[0][0]) / (ScanGap * TimeGap), ScanGap, OnceDataArr[j].x - TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].iLostNum);

								continue;
							}
							else
							{
								//fprintf(fWrite, "(ID,flag, v,x,y) %d %d %f %f %f xv %f gate %f \n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0], fabs(OnceDataArr[j].x - TraceTemp[i].XkArray_xfit[0][0]) / (ScanGap * TimeGap), 15);

							}
						}


						if (TraceTemp[i].TraceFlag * 1 == 1/* && InBSDZero(TraceTemp[i]) * 1 == 1*/)
						{
							if (fabs(OnceDataArr[j].y - TraceTemp[i].XkArray_yfit[0][0]) / ((ScanGap + TraceTemp[i].iLostNum) * TimeGap) > 25.0)
							{
								fprintf(fWrite, "(ID,flag, v,x,y) %d %d %f %f %f first corr  Yv  gate %f \n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0], fabs(OnceDataArr[j].y - TraceTemp[i].XkArray_yfit[0][0]) / (ScanGap * TimeGap));
								continue;
							}
							else
							{
								fprintf(fWrite, "(ID,flag, v,x,y) %d %d %f %f %f first corr  Yv  gate %f \n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0], fabs(OnceDataArr[j].y - TraceTemp[i].XkArray_yfit[0][0]) / (ScanGap * TimeGap));

							}
							if (fabs(OnceDataArr[j].x - TraceTemp[i].XkArray_xfit[0][0]) / ((ScanGap + TraceTemp[i].iLostNum) * TimeGap) > 15.0)
							{
								fprintf(fWrite, "(ID,flag, v,x,y) %d %d %f %f %f  xv  gate %f ScanGap = %d xdif = %f lost = %d\n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0], fabs(OnceDataArr[j].x - TraceTemp[i].XkArray_xfit[0][0]) / (ScanGap * TimeGap), ScanGap, OnceDataArr[j].x - TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].iLostNum);

								continue;
							}
							else
							{
								fprintf(fWrite, "(ID,flag, v,x,y) %d %d %f %f %f  xv  gate %f \n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0], fabs(OnceDataArr[j].x - TraceTemp[i].XkArray_xfit[0][0]) / (ScanGap * TimeGap));

							}
						}

						if (InBSDZero(TraceTemp[i]) * 1 == 1)
						{
							fprintf(fWrite, "&&&&&&&&&&&&&(ID,flag, v,x,y) %d %d %f %f %f in BSD \n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0]);
							fprintf(fWrite, "&&&&&&&&&&&&& %f %f %f \n", OnceDataArr[j].x, OnceDataArr[j].y, OnceDataArr[j].v);

							//if (fabs(fabs(OnceDataArr[j].y - TraceTemp[i].y_corr) / ((ScanGap + TraceTemp[i].iLostNum) * TimeGap) - fabs(TraceTemp[i].DPLVel)) > 8.0)
							//{
							//	fprintf(fWrite, "ѡ�񲻺��� TraceTemp[i].DPLVel = %f  ����������ٶ� = %f\n", TraceTemp[i].DPLVel, fabs(OnceDataArr[j].y - TraceTemp[i].y_corr) / ((ScanGap + TraceTemp[i].iLostNum) * TimeGap));
							//	continue;
							//}
							//else
							//{
							//	fprintf(fWrite, "���� TraceTemp[i].DPLVel = %f  ����������ٶ� = %f\n", TraceTemp[i].DPLVel, fabs(OnceDataArr[j].y - TraceTemp[i].y_corr) / ((ScanGap + TraceTemp[i].iLostNum) * TimeGap));
							//}

							if (fabs(OnceDataArr[j].v * 3.6) > 15 && fabs(OnceDataArr[j].y - TraceTemp[i].y_corr)< 0.01)
							{
								fprintf(fWrite, " TraceTemp[i].DPLVel = %f  yDif = %f\n", TraceTemp[i].DPLVel, OnceDataArr[j].y - TraceTemp[i].y_corr);
								continue;
							}

						}
                        if (AssociationJudge(TraceTemp[i], OnceDataArr[j], CarrierVel, CarrierAng) * 1 == 1)
                        {
							fprintf(fWrite, "		find (v,x,y) %f %f %f\n", OnceDataArr[j].v * 3.6, OnceDataArr[j].x, OnceDataArr[j].y);
                            tempAcc[tempNum].index = j;
                            tempAcc[tempNum].dis = OnceDataArr[j].dis;
                            tempAcc[tempNum].ang = OnceDataArr[j].ang;
                            tempAcc[tempNum].v = OnceDataArr[j].v;
                            tempAcc[tempNum].x = OnceDataArr[j].x;
                            tempAcc[tempNum].y = OnceDataArr[j].y;
                            tempAcc[tempNum].a = OnceDataArr[j].a;
                            tempAcc[tempNum].used = OnceDataArr[j].used;

                            tempNum = tempNum + 1;
                            if (tempNum >= NUMSAM)
                            {
                                break;
                            }
                        }
                    }
                }


                if (tempNum > 0)
                {

                    
                    indexBest = 0;
                    //disNear = GetDistance(TraceTemp[i].XkArray_xnxt[0][0], TraceTemp[i].XkArray_ynxt[0][0], tempAcc[indexBest].x, tempAcc[indexBest].y);
					disNearVel = GetDistanceVEL(TraceTemp[i].DPLVel, tempAcc[indexBest].v);
					fprintf(fWrite, "		(ID, v) %d %f Point v = %f disNearVel = %f\n", TraceTemp[i].ID, TraceTemp[i].DPLVel, tempAcc[indexBest].v, disNearVel);
                    for (k = 1; k < tempNum; k++)
                    {
                        //if (disNear > GetDistance(TraceTemp[i].XkArray_xnxt[0][0], TraceTemp[i].XkArray_ynxt[0][0], tempAcc[k].x, tempAcc[k].y))
						if (disNearVel > GetDistanceVEL(TraceTemp[i].DPLVel, tempAcc[k].v))
						{
							//disNear = GetDistance(TraceTemp[i].XkArray_xnxt[0][0], TraceTemp[i].XkArray_ynxt[0][0], tempAcc[k].x, tempAcc[k].y);
							disNearVel = GetDistanceVEL(TraceTemp[i].DPLVel, tempAcc[k].v);
							indexBest = k;
							fprintf(fWrite, "		(ID, v) %d %f Point v = %f disNearVel = %f\n", TraceTemp[i].ID, TraceTemp[i].DPLVel, tempAcc[indexBest].v, disNearVel);
                            
                        }
                    }
                    OnceDataArr[tempAcc[indexBest].index].used = 1;

                    tempAccBest.dis = tempAcc[indexBest].dis;
                    tempAccBest.ang = tempAcc[indexBest].ang;
                    tempAccBest.v = tempAcc[indexBest].v;
                    tempAccBest.x = tempAcc[indexBest].x;
                    tempAccBest.y = tempAcc[indexBest].y;
                    tempAccBest.a = tempAcc[indexBest].a;
                    tempAccBest.used = tempAcc[indexBest].used;

					fprintf(fWrite, "		find BEST (v,x,y) %f %f %f\n", tempAccBest.v * 3.6, tempAccBest.x, tempAccBest.y);

                    if( fabs(tempAccBest.x - TraceTemp[i].x_corr) > 0.01 || fabs(tempAccBest.y - TraceTemp[i].y_corr) > 0.01)
                    {

                        for (k = 0; k<ScanGap - 1; k++)//201911251601
                        {
                            
                            TraceTemp[i].TraceState = TraceTemp[i].TraceState << 1;
                            TraceTemp[i].iScanNum = TraceTemp[i].iScanNum + 1;
                        }
                        TraceTemp[i].iScanNum = TraceTemp[i].iScanNum + 1;
                        //FilterKalman(&TraceTemp[i], &tempAccBest);
                        FilterAlphaBeta(&TraceTemp[i], &tempAccBest);

                        //change 20191128 start
                        TraceTemp[i].Xpast[0] = TraceTemp[i].Xpast[1];
                        TraceTemp[i].Xpast[1] = TraceTemp[i].Xpast[2];
                        TraceTemp[i].Xpast[2] = TraceTemp[i].Xpast[3];
                        TraceTemp[i].Xpast[3] = TraceTemp[i].Xpast[4];
                        TraceTemp[i].Xpast[4] = tempAccBest.x;
                        //TraceTemp[i].Xpast[4] = TraceTemp[i].XkArray_xfit[0][0];

                        TraceTemp[i].Ypast[0] = TraceTemp[i].Ypast[1];
                        TraceTemp[i].Ypast[1] = TraceTemp[i].Ypast[2];
                        TraceTemp[i].Ypast[2] = TraceTemp[i].Ypast[3];
                        TraceTemp[i].Ypast[3] = TraceTemp[i].Ypast[4];
                        TraceTemp[i].Ypast[4] = tempAccBest.y;

                        TraceTemp[i].x_corr = tempAccBest.x;
                        TraceTemp[i].y_corr = tempAccBest.y;

                        TraceTemp[i].DPLVelpast[0] = TraceTemp[i].DPLVelpast[1];
                        TraceTemp[i].DPLVelpast[1] = TraceTemp[i].DPLVelpast[2];
                        TraceTemp[i].DPLVelpast[2] = TraceTemp[i].DPLVelpast[3];
                        TraceTemp[i].DPLVelpast[3] = TraceTemp[i].DPLVelpast[4];
                        TraceTemp[i].DPLVelpast[4] = tempAccBest.v;
                        //change 20191128 end

                        fprintf(fWrite, "			(ID) vpast %f  %f  %f %f %f\n", TraceTemp[i].DPLVelpast[0], TraceTemp[i].DPLVelpast[1], TraceTemp[i].DPLVelpast[2], TraceTemp[i].DPLVelpast[3], TraceTemp[i].DPLVelpast[4]);

                        TraceTemp[i].DPLVel = tempAccBest.v;   //20191112


                        //
                        TraceTemp[i].TraceState = TraceTemp[i].TraceState << 1;
                        TraceTemp[i].TraceState = TraceTemp[i].TraceState + 1;
                        TraceTemp[i].Updata = 1;
                        TraceTemp[i].iLostNum = 0;

                        TraceTemp[i].iUpdaNum = TraceTemp[i].iUpdaNum + 1;               //20191122

                        TraceTemp[i].TraceFlag = 1;
                        fprintf(fWrite, "			(ID,flag, v,x,y) %d %d %f %f %f  state = %d  lostnum = %d scan num = %d updata num = %d\n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0], TraceTemp[i].TraceState * 1, TraceTemp[i].iLostNum, TraceTemp[i].iScanNum, TraceTemp[i].iUpdaNum);
                        
                        //if (TraceTemp[i].TraceFlag == 1 && LastUpdataNum(SCANNUMBER, TraceTemp[i].TraceState) == SCANNUMBER)
                        fprintf(fWrite, " SCANNUMBER = %d, TraceTemp[i].TraceState = %d  %d \n", SCANNUMBER, TraceTemp[i].TraceState, LastUpdataNum(SCANNUMBER, TraceTemp[i].TraceState));
                        if (TraceTemp[i].TraceFlag == 1 && LastUpdataNum(SCANNUMBER, TraceTemp[i].TraceState) == CONFNUMBER)   // change  20191125
                        {

							//航迹判断
							//fprintf(fWrite, "  $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$cal = %f   cor = %f \n", fabs(TraceTemp[i].Ypast[0] - TraceTemp[i].Ypast[5]) / 0.03 / 4, fabs(TraceTemp[i].DPLVel));
							//if (fabs(fabs(TraceTemp[i].Ypast[0] - TraceTemp[i].Ypast[5]) / 0.03 / 4 - fabs(TraceTemp[i].DPLVel)) > 10)
							//{
							//	CancelTrace(TraceTemp[i].ID - 1, 0);
							//	fprintf(fWrite, "  ZW  trace Judge \n");
							//	continue;
							//}


                            TraceTemp[i].YEng = tempAccBest.y;
                            fprintf(fWrite, "TraceTemp[i].XkArray_xfit[0][0] = %f, TraceTemp[i].XkArray_yfit[0][0] = %f \n", TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0]);
                            ////////////////////////////////////////////////////////////////////////////////////
                            //if (TraceTemp[i].XkArray_xfit[0][0] > 0.1 &&  TraceTemp[i].XkArray_xfit[0][0] < 4.0 && TraceTemp[i].XkArray_yfit[0][0] > -2.0 && TraceTemp[i].XkArray_yfit[0][0] < 5.0) //�жϵ���  (ֻ��BSD�����жϣ�Ϊȡ���̶�������)
                            if (TraceTemp[i].XkArray_xfit[0][0] > 0.5 &&  TraceTemp[i].XkArray_xfit[0][0] < 4.0 && TraceTemp[i].XkArray_yfit[0][0] > -2.0 && TraceTemp[i].XkArray_yfit[0][0] < 5.0) //�жϵ���  (ֻ��BSD�����жϣ�Ϊȡ���̶�������)
                            {
                                if (BSDWARINGFLAG == 0)
                                {
                                    //if ((LastUpdataNum(SCANNUMBER, TraceTemp[i].TraceState) == (CONFNUMBER + 1)))
                                    {
                                        fprintf(fWrite, " ^^^^^^ID %d  %d \n", TraceTemp[i].ID, LastUpdataNum(SCANNUMBER, TraceTemp[i].TraceState));
                                        if (ToBeorNotToBe(&TraceTemp[i]) * 1 == 0)
                                        {
                                            CancelTrace(TraceTemp[i].ID - 1, 0);
                                            fprintf(fWrite, "  ZW  \n");
                                            //TraceTemp[i].ToBe = 1;
                                            continue;
                                        }

                                        if(XTrendJudegeDW(TraceTemp[i]) == -1)
                                        {
                                            CancelTrace(TraceTemp[i].ID - 1, 0);
                                            fprintf(fWrite, "  GUAIWAN&&&&&&&&  \n");
                                            continue;
                                        }    

                                        if (ToBeorNotToBe(&TraceTemp[i]) * 1 == 2)
                                        {
                                            fprintf(fWrite, "  MB  \n");
                                            TraceTemp[i].ToBe = 2;
                                        }
                                    }
                                }
                                else
                                {
                                    TraceTemp[i].ToBe = 2;
                                }
                            }
                            else 
                            {
                                TraceTemp[i].ToBe = 2;
                            }

                            if (TARGETNUM >= TraceConfNum)
                            {
                                break;
                            }
                            else
                            {
                                if (TraceTemp[i].ToBe == 2)
                                {
                                    while (1)
                                    {
                                        PointIDEX = PointIDEX + 1;
                                        if (PointIDEX >= TraceConfNum)
                                        {
                                            PointIDEX = 0;
                                        }
                                        if (TraceConf[PointIDEX].Used == 0)
                                        {
                                            break;
                                        }
                                    }
                                    fprintf(fWrite, "			temp ID = %d tobe confirm ID =%d, lane = %d\n", TraceTemp[i].ID,  PointIDEX + 1, TraceTemp[i].LaneFlag);
                                    TARGETNUM = TARGETNUM + 1;
                                    memcpy((char *)& TraceConf[PointIDEX], (char *)&TraceTemp[i], sizeof(TwiceDataKalman));
                                    CancelTrace(TraceTemp[i].ID - 1, 0);
                                    TraceConf[PointIDEX].ID = PointIDEX + 1;
                                    TraceConf[PointIDEX].TraceFlag = 2;
                                    TraceConf[PointIDEX].Used = 1;
                                    TraceConf[PointIDEX].Updata = 0;
                                    TraceConf[PointIDEX].Corred = 1;
                                }
                            }
                        }

                        if (TraceTemp[i].TraceFlag == 0 && ScanGap < 3)    // change 20191127
                        {
                            TraceTemp[i].TraceFlag = 1;
                        }
                    }
                    else
                    {
                            
                        if(InBSDZero(TraceTemp[i]))
                        {
                            CancelTrace(TraceTemp[i].ID - 1, 0);
                        }
                        else
                        {
                            fprintf(fWrite, "						------------------Find ERROR point \n");
                            for(k = 0; k<ScanGap; k++)//201911251601
                            {
                                TraceTemp[i].TraceState = TraceTemp[i].TraceState << 1;
                                TraceTemp[i].iScanNum = TraceTemp[i].iScanNum + 1;
                            }
                            TraceTemp[i].iLostNum = TraceTemp[i].iLostNum + ScanGap;                  //20191122
                            fprintf(fWrite, "			(ID,flag, v,x,y) %d %d %f %f %f state = %d  lostnum = %d scan num = %d updata num = %d\n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0], TraceTemp[i].TraceState * 1, TraceTemp[i].iLostNum, TraceTemp[i].iScanNum, TraceTemp[i].iUpdaNum);
                        }  
                            
                    }
                }
                else
                {
					fprintf(fWrite, "						------------------Find none point \n");
                    for(k = 0; k<ScanGap; k++)//201911251601
                    {
                        //״̬��־����һλ
                        TraceTemp[i].TraceState = TraceTemp[i].TraceState << 1;
						TraceTemp[i].iScanNum = TraceTemp[i].iScanNum + 1;
					}
					TraceTemp[i].iLostNum = TraceTemp[i].iLostNum + ScanGap;                  //20191122
					fprintf(fWrite, "			(ID,flag, v,x,y) %d %d %f %f %f state = %d  lostnum = %d scan num = %d updata num = %d\n", TraceTemp[i].ID, TraceTemp[i].TraceFlag * 1, TraceTemp[i].DPLVel, TraceTemp[i].XkArray_xfit[0][0], TraceTemp[i].XkArray_yfit[0][0], TraceTemp[i].TraceState * 1, TraceTemp[i].iLostNum, TraceTemp[i].iScanNum, TraceTemp[i].iUpdaNum);
                }
            }
        }

        ///change 20191127 start
        j = 0;
        for (i = 0; i < TraceConfNum; i++)
        {
            //change 20191128 start
            
            if (TraceConf[i].Used == 1)
            {
                

				if (InBSDZero(TraceConf[i]) * 1 == 1)
				{
					if (TraceConf[i].Corred == 1)
					{
						obj->detObj2D[j].rangeIdx = (int)(TraceConf[i].dis / disRes * 10 + 0.5);
						obj->detObj2D[j].x = (int)(TraceConf[i].XkArray_xfit[0][0] * 10 + 0.5);
                        obj->detObj2D[j].y = (int)(TraceConf[i].XkArray_yfit[0][0] * 10 + 0.5);
						obj->detObj2D[j].angle = (int)(10 * AngleCacul(obj->detObj2D[j].x, obj->detObj2D[j].y));
						obj->detObj2D[j].dopplerIdx = (int)(TraceConf[i].DPLVel * 10); 
						//obj->detObj2D[j].peakVal = TraceConf[i].AMP;
						obj->detObj2D[j].peakVal = TraceConf[i].Corred * 1;
						obj->detObj2D[j].z = TraceConf[i].ID;

						j = j + 1;
					}
				}
				else
				{
					obj->detObj2D[j].rangeIdx = (int)(TraceConf[i].dis * 10 / disRes + 0.5);
					obj->detObj2D[j].x = TraceConf[i].XkArray_xfit[0][0] * 10 + 0.5;
                    obj->detObj2D[j].y = TraceConf[i].XkArray_yfit[0][0] * 10 + 0.5;
					obj->detObj2D[j].angle = 10 * AngleCacul(obj->detObj2D[j].x, obj->detObj2D[j].y);
					obj->detObj2D[j].dopplerIdx = (int)(TraceConf[i].DPLVel * 10); 
					//obj->detObj2D[j].peakVal = TraceConf[i].AMP;
					obj->detObj2D[j].peakVal = TraceConf[i].Corred * 1;
					obj->detObj2D[j].z = TraceConf[i].ID;

					j = j + 1;
				}
                
            }
            //change 20191128 end
        }
        obj->numDetObj = j;

        
        for (i = 0; i < TraceTempNum; i++)
        {
            if (TraceTemp[i].Used == 1)
            {
                if (TraceTemp[i].TraceFlag == 0)                
                {
                    if (TraceTemp[i].iLostNum >= 2)
                    {
                        CancelTrace(TraceTemp[i].ID - 1, 0);
                    }
                }
                else                                        
                {
                    //if (TraceTemp[i].iLostNum == LOSTNUMBER) 
                    if (TraceTemp[i].iLostNum >= LOSTNUMBER) 
                    {
                        CancelTrace(TraceTemp[i].ID - 1, 0);
                    }
                    else                                     
                    {
                        if (TraceTemp[i].Updata == 1)
                        {
                            TraceTemp[i].Updata = -1;
                        }
                        else
                        {
                            //TraceTemp[i].iScanNum = TraceTemp[i].iScanNum + 1;
                            
                            FilterAlphaBeta(&TraceTemp[i], NULL);

                            //change 20191128 start
                            // TraceTemp[i].Xpast[0] = TraceTemp[i].Xpast[1];
                            // TraceTemp[i].Xpast[1] = TraceTemp[i].Xpast[2];
                            // TraceTemp[i].Xpast[2] = TraceTemp[i].Xpast[3];
                            // TraceTemp[i].Xpast[3] = TraceTemp[i].Xpast[4];
                            // TraceTemp[i].Xpast[4] = TraceTemp[i].XkArray_xfit[0][0];
							TraceTemp[i].Corred = 0;
                            //TraceTemp[i].Ypast[0] = TraceTemp[i].Ypast[1];
                            //TraceTemp[i].Ypast[1] = TraceTemp[i].Ypast[2];
                            //TraceTemp[i].Ypast[2] = TraceTemp[i].Ypast[3];
                            //TraceTemp[i].Ypast[3] = TraceTemp[i].Ypast[4];
                            //TraceTemp[i].Ypast[4] = TraceTemp[i].XkArray_yfit[0][0];

                            //change 20191128 end
                        }
                    }
                }
            }
        }

		for (i = 0; i < TraceTempNum; i++)
		{
			if (TraceTemp[i].Used == 1)
			{
				
				if (InBSDZeroNew(TraceTemp[i]) * 1 == 1)
				{
					if (TraceTemp[i].iScanNum == 5 && TraceTemp[i].iUpdaNum == 2)
					{
						CancelTrace(TraceTemp[i].ID - 1, 0);
					}
				}
			}

            if (InBSDZeroNew(TraceTemp[i]) * 1 == 1)
            {
                if (TraceTemp[i].iScanNum == 8 && TraceTemp[i].iUpdaNum < 4)
                {
                    CancelTrace(TraceTemp[i].ID - 1, 0);
                }
            }
		}

       
        for (j = 0; j < OncePointNumberClearn; j++)
        {
            if (OnceDataArr[j].used == 0)
            {
                if(OnceDataArr[j].x > 0.1 && OnceDataArr[j].x < 4)
                {
                    if(OnceDataArr[j].y > 0 && OnceDataArr[j].y <5.0)
                    {
                            continue;
                    }
                }
                if (TARGETTEMPNUM >= TraceTempNum)
                {
                    break;
                }
                else
                {
                    while (1)
                    {
                        PointTEMPIDEX = PointTEMPIDEX + 1;
                        if (PointTEMPIDEX >= TraceTempNum)
                        {
                            PointTEMPIDEX = 0;
                        }
                        if (TraceTemp[PointTEMPIDEX].Used == 0)
                        {
                            break;
                        }
                    }


                    TARGETTEMPNUM = TARGETTEMPNUM + 1;
                    TraceTemp[PointTEMPIDEX].AMP = OnceDataArr[j].a;
                    TraceTemp[PointTEMPIDEX].dis = OnceDataArr[j].dis;
                    TraceTemp[PointTEMPIDEX].ang = OnceDataArr[j].ang;
                    TraceTemp[PointTEMPIDEX].ID = PointTEMPIDEX + 1;
                    TraceTemp[PointTEMPIDEX].TraceFlag = 0;
                    TraceTemp[PointTEMPIDEX].Used = 1;
                    TraceTemp[PointTEMPIDEX].DPLVel = OnceDataArr[j].v;
                    TraceTemp[PointTEMPIDEX].fHead = 0;
                    TraceTemp[PointTEMPIDEX].iLostNum = 0;
                    TraceTemp[PointTEMPIDEX].iScanNum = 1;
                    TraceTemp[PointTEMPIDEX].iUpdaNum = 1;
                    TraceTemp[PointTEMPIDEX].TraceState = 1;
                    TraceTemp[PointTEMPIDEX].Updata = -1;

                    TraceTemp[PointTEMPIDEX].XkArray_xbef[0][0] = OnceDataArr[j].x;
                    TraceTemp[PointTEMPIDEX].XkArray_xbef[1][0] = 0;
                    TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0] = OnceDataArr[j].x;
                    TraceTemp[PointTEMPIDEX].XkArray_xfit[1][0] = 0;
                    TraceTemp[PointTEMPIDEX].XkArray_xnxt[0][0] = OnceDataArr[j].x;
                    TraceTemp[PointTEMPIDEX].XkArray_xnxt[1][0] = 0;

                    TraceTemp[PointTEMPIDEX].XkArray_ybef[0][0] = OnceDataArr[j].y;
                    TraceTemp[PointTEMPIDEX].XkArray_ybef[1][0] = 0;
                    TraceTemp[PointTEMPIDEX].XkArray_yfit[0][0] = OnceDataArr[j].y;
                    TraceTemp[PointTEMPIDEX].XkArray_yfit[1][0] = 0;
                    TraceTemp[PointTEMPIDEX].XkArray_ynxt[0][0] = OnceDataArr[j].y;
                    TraceTemp[PointTEMPIDEX].XkArray_ynxt[1][0] = 0;

                    TraceTemp[PointTEMPIDEX].PkArray_x[0][0] = 0;
                    TraceTemp[PointTEMPIDEX].PkArray_x[0][1] = 0;
                    TraceTemp[PointTEMPIDEX].PkArray_x[1][0] = 0;
                    TraceTemp[PointTEMPIDEX].PkArray_x[1][1] = 0;

                    TraceTemp[PointTEMPIDEX].PkArray_y[0][0] = 0;
                    TraceTemp[PointTEMPIDEX].PkArray_y[0][1] = 0;
                    TraceTemp[PointTEMPIDEX].PkArray_y[1][0] = 0;
                    TraceTemp[PointTEMPIDEX].PkArray_y[1][1] = 0;

                    //change 20191128 start
                    TraceTemp[PointTEMPIDEX].Xpast[0] = TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0];
                    TraceTemp[PointTEMPIDEX].Xpast[1] = TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0];
                    TraceTemp[PointTEMPIDEX].Xpast[2] = TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0];
                    TraceTemp[PointTEMPIDEX].Xpast[3] = TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0];
                    TraceTemp[PointTEMPIDEX].Xpast[4] = TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0];

                    TraceTemp[PointTEMPIDEX].Ypast[0] = TraceTemp[PointTEMPIDEX].XkArray_yfit[0][0];
                    TraceTemp[PointTEMPIDEX].Ypast[1] = TraceTemp[PointTEMPIDEX].XkArray_yfit[0][0];
                    TraceTemp[PointTEMPIDEX].Ypast[2] = TraceTemp[PointTEMPIDEX].XkArray_yfit[0][0];
                    TraceTemp[PointTEMPIDEX].Ypast[3] = TraceTemp[PointTEMPIDEX].XkArray_yfit[0][0];
                    TraceTemp[PointTEMPIDEX].Ypast[4] = TraceTemp[PointTEMPIDEX].XkArray_yfit[0][0];

					TraceTemp[PointTEMPIDEX].DPLVelpast[0] = OnceDataArr[j].v;
					TraceTemp[PointTEMPIDEX].DPLVelpast[1] = OnceDataArr[j].v;
					TraceTemp[PointTEMPIDEX].DPLVelpast[2] = OnceDataArr[j].v;
					TraceTemp[PointTEMPIDEX].DPLVelpast[3] = OnceDataArr[j].v;
					TraceTemp[PointTEMPIDEX].DPLVelpast[4] = OnceDataArr[j].v;
					fprintf(fWrite, "			(ID) vpast %f  %f  %f %f %f\n", TraceTemp[PointTEMPIDEX].DPLVelpast[0], TraceTemp[PointTEMPIDEX].DPLVelpast[1], TraceTemp[PointTEMPIDEX].DPLVelpast[2], TraceTemp[PointTEMPIDEX].DPLVelpast[3], TraceTemp[PointTEMPIDEX].DPLVelpast[4]);

					TraceTemp[PointTEMPIDEX].YBeg = OnceDataArr[j].y;
					TraceTemp[PointTEMPIDEX].ScanGapTotal = 0;
                    //change 20191128 end
                    //TraceTemp[PointTEMPIDEX].passY0 = 0;  // 20200422 change
					TraceTemp[PointTEMPIDEX].ToBe = 0;
					TraceTemp[PointTEMPIDEX].Corred = 1;
					TraceTemp[PointTEMPIDEX].x_corr = OnceDataArr[j].x;
					TraceTemp[PointTEMPIDEX].y_corr = OnceDataArr[j].y;

					if (TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0] >= LaneRightAfterLeft && TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0] < LaneRightAfterRight)
					{
						TraceTemp[PointTEMPIDEX].LaneFlag = 0;
					}
					if (TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0] >= LaneCloseLeft && TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0] < LaneCloseRight)
					{
						TraceTemp[PointTEMPIDEX].LaneFlag = 1;
					}
					if (TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0] >= LaneCloseNearLeft && TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0] <= LaneCloseNearRight)
					{
						TraceTemp[PointTEMPIDEX].LaneFlag = 2;
					}
					fprintf(fWrite, "(v(km/h),x,y) %f %f %f    ID = %d state = %d  lostnum = %d scan num = %d updata num = %d Lane = %d iScanNum = %d   to be trace head\n", OnceDataArr[j].v * 3.6, OnceDataArr[j].x, OnceDataArr[j].y, TraceTemp[PointTEMPIDEX].ID, TraceTemp[PointTEMPIDEX].TraceState * 1, TraceTemp[PointTEMPIDEX].iLostNum, TraceTemp[PointTEMPIDEX].iScanNum, TraceTemp[PointTEMPIDEX].iUpdaNum, TraceTemp[PointTEMPIDEX].LaneFlag * 1, TraceTemp[PointTEMPIDEX].iScanNum);
					fprintf(fWrite, "ID = %d  TraceFlag = %d  x0 = %f xv0 = %f\n", TraceTemp[PointTEMPIDEX].ID, TraceTemp[PointTEMPIDEX].TraceFlag * 1, TraceTemp[PointTEMPIDEX].XkArray_xfit[0][0], TraceTemp[PointTEMPIDEX].XkArray_xbef[1][0]);

				}
            }
        }
    }
    return (obj->numDetObj);
}



/************************************************************************/
/*

 */
/************************************************************************/
int32_t TRACEEXIT()
{

    free(TraceConf);
    free(TraceTemp);
    free(OnceDataArr);
	free(BSDWARING_obj);
    free(BSDWARING_ccobj);
    TraceConf = NULL;
    TraceTemp = NULL;;
    OnceDataArr = NULL;;
	BSDWARING_obj = NULL;
    BSDWARING_ccobj = NULL;
    return 1;
}



/************************************************************************/
/*

 */
/************************************************************************/
void CancelTrace(int index, int type)
{
    if (type == 0)
    {
        
		fprintf(fWrite, "			CANCEL temp (ID,flag) %d %d\n", TraceTemp[index].ID, TraceTemp[index].TraceFlag * 1);
        TraceTemp[index].Used = 0;
		TraceTemp[index].ToBe = 0;
        TraceTemp[index].Updata = -1;
        TARGETTEMPNUM = TARGETTEMPNUM - 1;
		

    }
    if (type == 1)//ȷ�Ϻ���
    {
        
		fprintf(fWrite, "			CANCEL confirm (ID,flag) %d %d\n", TraceConf[index].ID, TraceConf[index].TraceFlag * 1);
        TraceConf[index].Used = 0;
        TraceConf[index].Updata = -1;
        TARGETNUM = TARGETNUM - 1;

    }
}


/************************************************************************/
/*

 */
/************************************************************************/
int16_t LastUpdataNum(uint16_t n, uint16_t TraceState)
{
    int16_t num = 0;
    uint16_t Flag = 1;
    int16_t i = 0;
    uint16_t StartTraceFlag = 0;
    uint16_t data = 0;
    for (i = 0; i < n - 1; i++)
    {
        Flag = Flag << 1;
        Flag = Flag + 1;
    }

    StartTraceFlag = TraceState & Flag;

  
    for (i = 1; i <= n; i++)
    {
        data = OneFlag(i);

        if ((StartTraceFlag & data) > 0)
        {
            num = num + 1;
        }
    }
    return num;
}




/************************************************************************/
/*

 */
/************************************************************************/
int16_t OneFlag(int n)
{
    int i = 0;
	int16_t val = 1;
    if (n < 1)
    {
        return 0;
    }
    for (i = 1; i < n; i++)
    {
        val = val << 1;
    }
    return val;
}


/************************************************************************/
/*

 */
/************************************************************************/
void TrackPrediction(TwiceDataKalman* trace, float CarrierVel, float CarrierAng, int ScanGap)
{

    float VelTar = 0; 
    float VelSel = 0; 

    float xTarOrg = 0; 
    float yTarOrg = 0;

    float xSelOrg = 0;
    float ySelOrg = 0;


    float xTarNxt = 0;
    float yTarNxt = 0;

    float xSelNxt = 0;
    float ySelNxt = 0;
	
    if ((*trace).TraceFlag * 1 == 0)
    {

		fprintf(fWrite, "\n\n(ID,flag, v,x,y) trace %d %d %f %f %f Prediction\n", (*trace).ID, (*trace).TraceFlag * 1, (*trace).DPLVel, (*trace).XkArray_xfit[0][0], (*trace).XkArray_yfit[0][0]);
        VelTar = (*trace).XkArray_yfit[1][0];
        VelSel = CarrierVel / 3.6;

        
        xTarOrg = (*trace).XkArray_xfit[0][0];  //Ŀ������ x
        yTarOrg = (*trace).XkArray_yfit[0][0];  //Ŀ������ y


        xTarNxt = xTarOrg + 0.0 * TimeGap * ScanGap;

        if ((*trace).XkArray_yfit[0][0] < 0)
        {
            yTarNxt = yTarOrg + (*trace).DPLVel * TimeGap * ScanGap;
        }
        else
        {
            yTarNxt = yTarOrg - (*trace).DPLVel * TimeGap * ScanGap;
        }

        xSelNxt = xSelOrg;
        ySelNxt = ySelOrg;

        (*trace).XkArray_xnxt[0][0] = xTarNxt - xSelNxt;
        (*trace).XkArray_xnxt[1][0] = (*trace).XkArray_xfit[1][0];

        (*trace).XkArray_ynxt[0][0] = yTarNxt - ySelNxt;
        (*trace).XkArray_ynxt[1][0] = (*trace).XkArray_yfit[1][0];

		(*trace).ScanGapTotal = (*trace).ScanGapTotal + ScanGap;

		fprintf(fWrite, "(ID,flag, v,x,y)  %d %d %f %f %f \n", (*trace).ID, (*trace).TraceFlag * 1, (*trace).DPLVel, (*trace).XkArray_xnxt[0][0], (*trace).XkArray_ynxt[0][0]);
    }
    else
    {
		fprintf(fWrite, "\n\n (ID,flag, v,x,y)%d %d %f %f %f \n", (*trace).ID, (*trace).TraceFlag * 1, (*trace).DPLVel, (*trace).XkArray_xfit[0][0], (*trace).XkArray_yfit[0][0]);
        VelTar = (*trace).XkArray_yfit[1][0];
        VelSel = CarrierVel / 3.6;

        xTarOrg = (*trace).XkArray_xfit[0][0];
        yTarOrg = (*trace).XkArray_yfit[0][0];

        xTarNxt = xTarOrg + (*trace).XkArray_xfit[1][0] * TimeGap * ScanGap;
        yTarNxt = yTarOrg + (*trace).XkArray_yfit[1][0] * TimeGap * ScanGap;

        xSelNxt = xSelOrg;
        ySelNxt = ySelOrg;

        (*trace).XkArray_xnxt[0][0] = xTarNxt - xSelNxt;
        (*trace).XkArray_xnxt[1][0] = (*trace).XkArray_xfit[1][0];

        (*trace).XkArray_ynxt[0][0] = yTarNxt - ySelNxt;
        (*trace).XkArray_ynxt[1][0] = (*trace).XkArray_yfit[1][0];

		(*trace).ScanGapTotal = (*trace).ScanGapTotal + ScanGap;
		fprintf(fWrite, " (ID,flag, v,x,y) %d %d %f %f %f \n", (*trace).ID, (*trace).TraceFlag * 1, (*trace).DPLVel, (*trace).XkArray_xnxt[0][0], (*trace).XkArray_ynxt[0][0]);
    }
}


float AngleCacul(float x, float y)
{
    return atanf(x / y) * 180 / PI;
}

//int Merge(OnceDataProPressObj* ProPressonceTemp)
//{
//    int16_t i, j, k;
//    int16_t total;
//    float x, y, v;
//    float xmin, ymin, vmin;
//    for (i = 0; i < ProPressonceTemp->Num; i++)
//    {
//		if (ProPressonceTemp->Prodata[i].y > 22.0)
//		{
//			fprintf(fWrite, " (v,x,y)%f %f %f out range \n", ProPressonceTemp->Prodata[i].v * 3.6, ProPressonceTemp->Prodata[i].x, ProPressonceTemp->Prodata[i].y);
//			continue;
//		}
//        total = 1;
//        x = ProPressonceTemp->Prodata[i].x;
//        y = ProPressonceTemp->Prodata[i].y;
//        v = ProPressonceTemp->Prodata[i].v;
//        if (ProPressonceTemp->usless[i] == 0)
//        {
//			fprintf(fWrite, " (v,x,y)%f %f %f out range  commpare with ", v*3.6,x, y);
//            for (j = i + 1; j < ProPressonceTemp->Num; j++)
//            {
//				if (ProPressonceTemp->Prodata[j].y > 22.0)
//				{
//					continue;
//				}
//                if (ProPressonceTemp->usless[j] == 0)
//                {
//					fprintf(fWrite, " (v,x,y)%f %f %f out range \n", ProPressonceTemp->Prodata[j].v * 3.6, ProPressonceTemp->Prodata[j].x, ProPressonceTemp->Prodata[j].y);
//                    if (fabs(ProPressonceTemp->Prodata[i].x - ProPressonceTemp->Prodata[j].x) < NJZeroWx && fabs(ProPressonceTemp->Prodata[i].y - ProPressonceTemp->Prodata[j].y) < NJZeroWy && (ProPressonceTemp->Prodata[i].y * ProPressonceTemp->Prodata[j].y) > 0)
//                    {
//						fprintf(fWrite, " loc ok\n");
//
//                        if (ProPressonceTemp->Prodata[i].v * ProPressonceTemp->Prodata[j].v > 0 && fabs(ProPressonceTemp->Prodata[i].v - ProPressonceTemp->Prodata[j].v) < NJZeroWv)
//                        {
//							fprintf(fWrite, " vel ok\n");
//                            ProPressonceTemp->usless[j] = 1;
//                            if (ProPressonceTemp->Prodata[i].v < ProPressonceTemp->Prodata[j].v)
//							//if (ProPressonceTemp->Prodata[i].y > ProPressonceTemp->Prodata[j].y)
//                            {
//                                ProPressonceTemp->Prodata[i].v = ProPressonceTemp->Prodata[i].v * total + ProPressonceTemp->Prodata[j].v;
//                                ProPressonceTemp->Prodata[i].y = ProPressonceTemp->Prodata[i].y * total + ProPressonceTemp->Prodata[j].y;
//                                ProPressonceTemp->Prodata[i].x = ProPressonceTemp->Prodata[i].x * total + ProPressonceTemp->Prodata[j].x;
//                                total = total + 1;
//                                ProPressonceTemp->Prodata[i].v = 1.0 * ProPressonceTemp->Prodata[i].v / total;
//                                ProPressonceTemp->Prodata[i].y = 1.0 * ProPressonceTemp->Prodata[i].y / total;;
//                                ProPressonceTemp->Prodata[i].x = 1.0 * ProPressonceTemp->Prodata[i].x / total;;
//                            }
//                        }
//						else
//						{
//							fprintf(fWrite, " vel not ok\n");
//						}
//                    }
//					else
//					{
//						fprintf(fWrite, " loc not ok\n");
//					}
//                }
//            }
//        }
//    }
//    return 0;
//}


int Merge(OnceDataProPressObj* ProPressonceTemp)
{
	int16_t i, j, k;
	int16_t total;
	float x, y, v;
	float xmin, ymin, vmin;
	for (i = 0; i < ProPressonceTemp->Num; i++)
	{
		if (ProPressonceTemp->Prodata[i].y > 22.0)
		{
			fprintf(fWrite, " (v,x,y)%f %f %f out range \n", ProPressonceTemp->Prodata[i].v * 3.6, ProPressonceTemp->Prodata[i].x, ProPressonceTemp->Prodata[i].y);
			continue;
		}
		total = 1;
		x = ProPressonceTemp->Prodata[i].x;
		y = ProPressonceTemp->Prodata[i].y;
		v = ProPressonceTemp->Prodata[i].v;
		if (ProPressonceTemp->usless[i] == 0)
		{
			fprintf(fWrite, " (v,x,y)%f %f %f out range  commpare with ", v*3.6, x, y);
			for (j = i + 1; j < ProPressonceTemp->Num; j++)
			{
				if (ProPressonceTemp->Prodata[j].y > 22.0)
				{
					continue;
				}
				if (ProPressonceTemp->usless[j] == 0)
				{
					fprintf(fWrite, " (v,x,y)%f %f %f out range \n", ProPressonceTemp->Prodata[j].v * 3.6, ProPressonceTemp->Prodata[j].x, ProPressonceTemp->Prodata[j].y);
					if (fabs(ProPressonceTemp->Prodata[i].x - ProPressonceTemp->Prodata[j].x) < NJZeroWx && fabs(ProPressonceTemp->Prodata[i].y - ProPressonceTemp->Prodata[j].y) < NJZeroWy && (ProPressonceTemp->Prodata[i].y * ProPressonceTemp->Prodata[j].y) > 0)
					{
						fprintf(fWrite, " loc ok\n");

						if (ProPressonceTemp->Prodata[i].v * ProPressonceTemp->Prodata[j].v > 0 && fabs(ProPressonceTemp->Prodata[i].v - ProPressonceTemp->Prodata[j].v) < NJZeroWv)
						{
							fprintf(fWrite, " vel ok\n");
							ProPressonceTemp->usless[j] = 1;
							//if (ProPressonceTemp->Prodata[i].v < ProPressonceTemp->Prodata[j].v)
							if (ProPressonceTemp->Prodata[i].y > ProPressonceTemp->Prodata[j].y)
							{
								ProPressonceTemp->Prodata[i].v =  ProPressonceTemp->Prodata[j].v;
								ProPressonceTemp->Prodata[i].y =  ProPressonceTemp->Prodata[j].y;
								ProPressonceTemp->Prodata[i].x =  ProPressonceTemp->Prodata[j].x;
							}
						}
						else
						{
							fprintf(fWrite, " vel not ok\n");
						}
					}
					else
					{
						fprintf(fWrite, " loc not ok\n");
					}
				}
			}
		}
	}
	return 0;
}

// JUST for belongsaide
char ToBeorNotToBe(TwiceDataKalman* trace)
{
	//  zawu     0:
	//  temp     1:  
	//  confirm  2:  

	char flag = 0;
	float timespend = (*trace).ScanGapTotal * TimeGap;
	float DeltY = (*trace).YEng - (*trace).YBeg;
	float DeltYv = 0;

	fprintf(fWrite, "@@@@@@@ ToBeorNotToBe@@@@@@@@@@\n");

	
	
	if (((*trace).iUpdaNum == 5 ) &&((*trace).Ypast[0] < 0) && ((*trace).Ypast[1] < 0) && ((*trace).Ypast[2] < 0) && ((*trace).Ypast[3] < 0) && ((*trace).Ypast[4] < 0)) // ��ʷ��y<0
	{
		flag = BSDUPJudegeDW(*trace);
	}

	
	if (((*trace).YBeg <= 0) && ((*trace).YBeg * (*trace).YEng < 0))
	{
		DeltY = DeltY;
		DeltYv = DeltY / timespend;
		fprintf(fWrite, "(ID,)  YEng YBeg vy %d %d %f %f %f\n", (*trace).ID, (*trace).ScanGapTotal, (*trace).YEng, (*trace).YBeg, (*trace).DPLVel);
		if (DeltY * (*trace).DPLVel < 0 && fabs(fabs(DeltYv) - fabs((*trace).DPLVel)) < 5.0) //�ٶȷ����෴
		{
			flag = 2;
		}
		if (DeltY * (*trace).DPLVel > 0) 
		{
			fprintf(fWrite, "ERROR \n");;
		}
		if (fabs(fabs(DeltYv) - fabs((*trace).DPLVel)) > 5.0) 
		{
			fprintf(fWrite, " ERROR \n");;
		}

	}

	if (((*trace).YBeg > 0) && ((*trace).YBeg * (*trace).YEng > 0))
	{
		DeltY = -1.0 * DeltY;
		DeltYv = DeltY / timespend;
		fprintf(fWrite, "(ID,)  YEng YBeg vy %d %d %f %f DeltYv=%f vy = %f\n", (*trace).ID, (*trace).ScanGapTotal, (*trace).YEng, (*trace).YBeg, DeltYv, (*trace).DPLVel);
		if (DeltY * (*trace).DPLVel > 0 && fabs(DeltYv - (*trace).DPLVel) < 5.0) 
		{
			flag = 2;
		}
	}
	//if (((*trace).YBeg > 0) && ((*trace).YBeg * (*trace).YEng < 0))
	//{
	//	DeltY = -1.0 * DeltY;
	//	DeltYv = DeltY / timespend;
	//	if (DeltY * (*trace).DPLVel > 0 && fabs(DeltYv - (*trace).DPLVel) < 5.0) //�ٶȷ����෴
	//	{
	//		flag = 1;
	//	}
	//}
	fprintf(fWrite, "DeltYv = %f, (*trace).ScanGapTotal = %d, DeltY = %f, (*trace).YEng = %f, (*trace).YBeg = %f,  ����0 ����1 Ŀ��2 ���ۣ� %d\n", DeltYv, (*trace).ScanGapTotal, DeltY, (*trace).YEng, (*trace).YBeg, flag * 1);
	return flag;
}


char InBSDZero(TwiceDataKalman trace)
{
	char flag = 0;
	if (trace.XkArray_xfit[0][0] >= 0.5 && trace.XkArray_xfit[0][0] <= 3.5)
	{
		if (trace.XkArray_yfit[0][0] >= -2.0 && trace.XkArray_yfit[0][0] <= 5.0)
		{
			flag = 1;
		}
	}
	return flag;
}

char InBSDZeroNew(TwiceDataKalman trace)
{
	char flag = 0;
	if (trace.XkArray_xfit[0][0] <= 3.5)
	{
		if (trace.XkArray_yfit[0][0] >= -2.0 && trace.XkArray_yfit[0][0] <= 5.0)
		{
			flag = 1;
		}
	}
	return flag;
}

char PointInBSDZero(OnceData point)
{
	char flag = 0;
	if (point.x > 0.5 && point.x < 3.5)
	{
		if (point.y > -2.0 && point.y < 5.0)
		{
			flag = 1;
		}
	}
	return flag;
}



char BSDUPJudegeDW(TwiceDataKalman trace)
{

	fprintf(fWrite, "@@@@@@@ BSDUPJudegeDW @@@@@@@@@@\n");
	char flag = 0;
	int i = 0;
	char havebig = 0;
	char havesml = 0;
	for (i = 0; i < 5; i++)
	{
		if (trace.DPLVelpast[i] >= 0)
		{
			havebig = havebig + 1;
		}
		else
		{
			havesml = havesml + 1;
		}
	}
	//if ((trace.iUpdaNum == 5) && (trace.Ypast[0] < 0) && (trace.Ypast[1] < 0) && (trace.Ypast[2] < 0) && (trace.Ypast[3] < 0) && (trace.Ypast[4] < 0)) //
	{
		if (havebig == 5) 
		{
			fprintf(fWrite, "@@@@@@@ have trend FIVE CHAOCHE @@@@@@@@@@\n");
			if ((trace.Ypast[1] - trace.Ypast[0] >= 0.001) && (trace.Ypast[2] - trace.Ypast[1] >= 0.001) && (trace.Ypast[3] - trace.Ypast[2] >= 0.001) && (trace.Ypast[4] - trace.Ypast[3] >= 0.001))// ���˶�����
			{
				fprintf(fWrite, "@@@@@@@ confirm @@@@@@@@@@\n");
				flag = 2;   // Ŀ��
			}
			else
			{
				fprintf(fWrite, "@@@@@@@ not confirm @@@@@@@@@@\n");
				flag = 0; // ���� ����
			}
		}
		if (havesml == 5) //�ٶ�ȫС��0
		{
			fprintf(fWrite, "@@@@@@@ have trend FIVE beichao @@@@@@@@@@\n");
			if ((trace.Ypast[1] - trace.Ypast[0] <= 0.001) && (trace.Ypast[2] - trace.Ypast[1] <= 0.001) && (trace.Ypast[3] - trace.Ypast[2] <= 0.001) && (trace.Ypast[4] - trace.Ypast[3] <= 0.001))// ���˶�����
			{
				fprintf(fWrite, "@@@@@@@ confirm @@@@@@@@@@\n");
				flag = 2;   // Ŀ��
			}
			else
			{
				fprintf(fWrite, "@@@@@@@ not confirm @@@@@@@@@@\n");
				flag = 0; // ���� ����
			}
		}
		
		
		if (havebig >= 1 && havesml >= 1)
		{
			fprintf(fWrite, "@@@@@@@  havesml = %d@@@@@@@@@@\n", havesml);
			if ((trace.Ypast[1] - trace.Ypast[0] >= 0.001) && (trace.Ypast[2] - trace.Ypast[1] >= 0.001) && (trace.Ypast[3] - trace.Ypast[2] >= 0.001) && (trace.Ypast[4] - trace.Ypast[3] >= 0))// ���˶�����
			{
				fprintf(fWrite, "@@@@@@@ @@@@@@@@@@\n");
				flag = 2;   
			}
		}
	}
	fprintf(fWrite, "@@@@@@@ BSDUPJudegeDW flag = %d@@@@@@@@@@\n", flag * 1);
	return flag;
}



extern char XTrendJudegeDW(TwiceDataKalman trace)
{


    fprintf(fWrite, "!!!!!!!XTrend!!!!!!!!!\n");
    char trean = 1;
    int num = 0;
    int i = 0;

    for(i = 4; i >= 1; i--)
    {
        fprintf(fWrite, " i = %d trace.Xpast[i] = %f\n", i, trace.Xpast[i]);
    }
    fprintf(fWrite, " i = %d trace.Xpast[i] = %f\n", 0, trace.Xpast[0]);

    if(trace.Xpast[4] - trace.Xpast[3] > 0)
    {
        for(i = 4; i >= 1; i--)
        {
            if(trace.Xpast[i] - trace.Xpast[i-1] > 0.5)
            {
                num = num + 1;
            }
        }
    }
    else
    {
        for(i = 4; i >= 1; i--)
        {
            if(trace.Xpast[i] - trace.Xpast[i-1] <  -0.5)
            {
                num = num + 1;
            }
        }
    }
    
    

    if(num >=3)
    {
        trean = -1;
    }
    fprintf(fWrite, "num = %d\n", num);
    return trean;

}



extern char FixTrace(TwiceDataKalman* trace)
{
	fprintf(fWrite, "LaneFlag = %d\n", trace->LaneFlag);
	if (0 == trace->LaneFlag) //right behind
	{
		if (trace->XkArray_xfit[0][0] <= LaneRightAfterLeft)
		{
			trace->XkArray_xfit[0][0] = LaneRightAfterLeft + 1.2;
		}
		if (trace->XkArray_xfit[0][0] > LaneRightAfterRight)
		{
			trace->XkArray_xfit[0][0] = LaneRightAfterRight - 1.2;
		}
	}
	if (1 == trace->LaneFlag) //close
	{
		if (trace->XkArray_xfit[0][0] <= LaneCloseLeft) 
		{
			trace->XkArray_xfit[0][0] = LaneCloseLeft + 1.2;
		}
		if (trace->XkArray_xfit[0][0] > LaneCloseRight)
		{
			trace->XkArray_xfit[0][0] = LaneCloseRight - 1.2;
		}
	}
	if (2 == trace->LaneFlag)//close close
	{
		if (trace->XkArray_xfit[0][0] <= LaneCloseNearLeft)
		{
			trace->XkArray_xfit[0][0] = LaneCloseNearLeft + 1.2;
		}
		if (trace->XkArray_xfit[0][0] > LaneCloseNearRight)
		{
			trace->XkArray_xfit[0][0] = LaneCloseNearRight - 1.2;
		}

	}
	fprintf(fWrite, " xf = %f  yf = %f  xv = %f, yv = %f\n", trace->XkArray_xfit[0][0], trace->XkArray_yfit[0][0], trace->XkArray_xfit[1][0], trace->XkArray_yfit[1][0]);

	return 0;
}
