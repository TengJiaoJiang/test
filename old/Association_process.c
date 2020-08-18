/*
 * Association_process.c
 *
 *  Created on: 2019��9��30��
 *      Author: 1
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "Association_process.h"
#include "Tracking_process.h"
//#include "common_Funx.h"

extern FILE *fWrite;

#define ParameterGateBeg       (1.0F)            
#define DIATENCEGATE           (0.5F)           
#define ParameterGateNom       (6.0F)            

#define RectangularAngleX       (3.5F)            
#define RectangularHorizontalAxis   (0.09223752844312615F)        
#define RectangularX            (3.50F)                             
#define RectangularY            (5.00F)            
#define DISTENCTBLOCK           (40)

#define VelocityMax            (100.0F)         
#define DPLVELGATE             (4.20F)  
#define DPLVELGATEBSD          (3.00F)          
#define AMPGATE                (3000.0F)           
#define VELDIFFERENTIALGATE    (2.0F)           
#define XVELDIFFERENTIALGATE   (10.0F)            
#define YVELDIFFERENTIALGATE   (10.0F)          

#define XDIFFERENT             (5.0F)
#define YDIFFERENT             (6.0F)

//const float TimeGap = 0.02F;                  
extern uint16_t ScanGap;


#define RAD_000             (0.0)
#define RAD_002             (0.0349065850398865915384738153697723)
#define RAD_005             (0.0872664625997164788461845384244306)
#define RAD_010             (0.174532925199432957692369076848861)
#define RAD_015             (0.261799387799149436538553615273292)
#define RAD_020             (0.349065850398865915384738153697723)
#define RAD_025             (0.436332312998582394230922692122153)
#define RAD_030             (0.523598775598298873077107230546584)
#define RAD_040             (0.698131700797731830769476307395445)
#define RAD_045             (0.785398163397448309615660845819876)
#define RAD_050             (0.872664625997164788461845384244306)
#define RAD_060             (1.04719755119659774615421446109317)
#define RAD_075             (1.30899693899574718269276807636646)
#define RAD_090             (1.57079632679489661923132169163975)
#define RAD_180             (3.1415926535897932384626433832795)
#define RAD_270             (4.71238898038468985769396507491925)
#define RAD_360             (6.28318530717958647692528676655901)

/************************************************************************/
/*
 */
/************************************************************************/
char AssociationJudge(TwiceDataKalman trace, OnceData point, float CarrierVel, float CarrierAng)
{



    int32_t same = -1;
    int32_t ingate = -1;
    int32_t samDPL = -1;
    int32_t samAmp = -1;
    int32_t samVel = -1;
    int32_t samLocation = -1;
	int32_t Reason = -1;
 

    if (InGate(trace, point, CarrierVel)*1 == 1)
    {
        ingate = 1;
    }

    if (SamDPL(trace, point) * 1 == 1)                    
    {
        samDPL = 1;
    }

    if (SamAmp(trace, point) * 1 == 1)                     
    {
        samAmp = 1;
    }

    if (SamVelXY(trace, point) * 1 == 1)
        //    if (SamVel(trace, point) * 1 == 1)
    {
        samVel = 1;
    }

    if (JudgeSAME(trace, point) * 1 == 1)
    {
        samLocation = 1;
    }

	if (JudgeReason(trace, point) * 1 == 1)
    {
        Reason = 1;
    }

    if ((ingate * 1 == 1) && (samDPL * 1 == 1) && (samAmp * 1 == 1) && (Reason * 1 == 1)/*&& (samVel * 1 == 1)*/)
    {
        same = 1;
    }
	fprintf(fWrite, "AssociationJudge %d %d %d %d\n", ingate * 1, samDPL * 1, samAmp * 1, Reason * 1);
    return same;
}


/************************************************************************/
/*


 */
/************************************************************************/

void Mapping(float xf, float yf, float CarrierVel, float CarrierAng, float *x, float *y)
{

    float tx = xf;
    float ty = yf;


    float xCarrier = CarrierVel * (TimeGap * ScanGap) * sinf(CarrierAng);
    float yCarrier = CarrierVel * (TimeGap * ScanGap) * cosf(CarrierAng);

    *x = tx - xCarrier;
    *y = ty - yCarrier;

}

char InGate(TwiceDataKalman trace, OnceData point, float CarrierVel)
{



	char ingate = 0;
	float xt = point.x - trace.XkArray_xnxt[0][0];
	float yt = point.y - trace.XkArray_ynxt[0][0];

	fprintf(fWrite, "xt = %f point.x = %f trace.XkArray_xnxt[0][0] = %f \n", xt, point.x, trace.XkArray_xnxt[0][0]);
	fprintf(fWrite, "yt = %f point.y = %f trace.XkArray_ynxt[0][0] = %f \n", yt, point.y, trace.XkArray_ynxt[0][0]);
	float gatX = 0;
	float gatY = 0;
	float coefficientX = 0;
	float coefficientY = 0;

	if (trace.LaneFlag == 0)
	{
		if (fabs(point.x - LaneRightAfterMidle) > (LaneWidth / 2))
		{
			fprintf(fWrite, "lane %d , x %f\n", trace.LaneFlag, point.x);
			return  -1;
		}
	}
	if (trace.LaneFlag == 1)
	{
		if (fabs(point.x - LaneCloseMidle) > (LaneWidth / 2))
		{
			fprintf(fWrite, "lane %d , x %f\n", trace.LaneFlag, point.x);
			return  -1;
		}
	}
	if (trace.LaneFlag == 2)
	{
		if (fabs(point.x - LaneCloseNearMidle) > (LaneWidth / 2))
		{
			fprintf(fWrite, "lane  %d , x %f\n", trace.LaneFlag, point.x);
			return  -1;
		}
	}



	if (CarrierVel < 20.0)
	{
		coefficientX = 1.0;
		coefficientY = 1.0;
	}
	if (CarrierVel >= 20.0 && CarrierVel < 60.0)
	{
		coefficientX = 1; //1.7
		coefficientY = 1.7;
	}
	if (CarrierVel >= 60.0 && CarrierVel < 90.0)
	{
		coefficientX = 1.1;//2.0
		coefficientY = 2.0;
	}
	else
	{
		coefficientX = 1.1;//2.7
		coefficientY = 2.5;
	}

	//BSD 
	if (trace.XkArray_xfit[0][0] >= 0.5 &&  trace.XkArray_xfit[0][0] <= 3.5 && trace.XkArray_yfit[0][0] >= -2.0 && trace.XkArray_yfit[0][0] <= 5.0)
	{
		fprintf(fWrite, "BSD  gate X = %f, Y =%f\n", 2.0, 2.0);
		if ((fabs(xt) < 2.0) && (fabs(yt) < 2.0))
		{
			fprintf(fWrite, "BSD In gate  \n\n");
			ingate = 1;
		}
		else
		{
			fprintf(fWrite, "BSD  gate X = %f, Y =%f small \n\n", 2.0, 2.0);
		}
	}
	else//BSD ������
	{
		if (trace.dis > DISTENCTBLOCK)
		{
			fprintf(fWrite, "out BSD out 40 gate X= RectangularX * coefficientX * 0.8 = %f, Y=RectangularY * coefficientY =%f\n", RectangularX * coefficientX * 0.8, RectangularY * coefficientY);
			//if ((fabs(xt) < RectangularX * coefficientX * 0.8) && (fabs(yt) < RectangularY * coefficientY))
			if ((fabs(xt) < RectangularX * coefficientX) && (fabs(yt) < RectangularY * coefficientY))
			{
				fprintf(fWrite, "out BSD out 40 In gate  \n\n");
				ingate = 1;
			}
			else
			{
				fprintf(fWrite, "out BSD out 40 gate X= RectangularX * coefficientX * 0.8 = %f, Y=RectangularY * coefficientY =%f   samall\n", RectangularX * coefficientX * 0.8, RectangularY * coefficientY);
			}
		}
		else
		{
			fprintf(fWrite, "out BSD in 40  gate X= RectangularX * coefficientX = %f, Y=RectangularY * coefficientY =%f\n", RectangularX * coefficientX, RectangularY * coefficientY);
			if ((fabs(xt) < RectangularX * coefficientX) && (fabs(yt) < RectangularY * coefficientY))                                     //change
			{
				fprintf(fWrite, "out BSD in 40 In gate  \n\n");
				ingate = 1;
			}
			else
			{
				fprintf(fWrite, "out BSD in 40  gate X= RectangularX * coefficientX = %f, Y=RectangularY * coefficientY =%f SMALL \n", RectangularX * coefficientX, RectangularY * coefficientY);
			}
		}
	}

	return ingate;
}


/************************************************************************/
/*


 */
/************************************************************************/

float GetDistance(float x, float y, float X, float Y)
{
    float xt = x - X;
    float yt = y - Y;

    return sqrtf(xt * xt + yt * yt);
}

float GetDistanceVEL(float tVEL, float pVEL)
{
	return fabs(pVEL - tVEL);
}


char InRectangularGate(float x, float y, float X, float Y)
{
    char flag = 0;
    float xt = x - X;
    float yt = y - Y;
    if ((fabs(xt) < RectangularX) && (fabs(yt) < RectangularY))
    {
        flag = 1;
    }
    return flag;
}

/************************************************************************/
/*


 */
/************************************************************************/

char SamDPL(TwiceDataKalman trace, OnceData point)
{
	fprintf(fWrite, "SamDPL JUDGE trace.DPLVel = %f point.v = %f ty = %f py = %f^^^^\n", trace.DPLVel, point.v, trace.XkArray_yfit[0][0], point.y);
    int32_t samDPL = -1;
    //if (trace.XkArray_xfit[0][0] > 0.5 && trace.XkArray_xfit[0][0] < 3.5 && trace.XkArray_yfit[0][0] > -2.0 && trace.XkArray_yfit[0][0] < 5.0)
	if (trace.XkArray_xfit[0][0] > 0.3 && trace.XkArray_xfit[0][0] < 4.0 && trace.XkArray_yfit[0][0] > -2.0 && trace.XkArray_yfit[0][0] < 5.0)
	{
		fprintf(fWrite, "IN BSD \n");
        if (fabs(trace.XkArray_yfit[0][0])>= 3.0) // 2
        {
			fprintf(fWrite, "IN BSD AAAAAA\n");
			fprintf(fWrite, "In BSD &&& ID = %d, flag = %d, XkArray_yfit = %f DPLgate = %f\n", trace.ID, trace.TraceFlag * 1, trace.XkArray_yfit[0][0], DPLVELGATE);
            if (fabs(trace.DPLVel - point.v) < DPLVELGATE)
            {
                samDPL = 1;
            }
        }//��BSD����
        else
        {
            fprintf(fWrite, "IN BSD BBBBBBB\n");
            //if (fabs(fabs(trace.DPLVel) - fabs(point.v)) < DPLVELGATE * 5)
			if (trace.XkArray_yfit[0][0] * point.y > 0)
			{
				
				fprintf(fWrite, "In BSD sudu tonghao ID = %d, flag = %d, XkArray_yfit = %f DPLgate = %f\n", trace.ID, trace.TraceFlag * 1, trace.XkArray_yfit[0][0], DPLVELGATE);
				if (fabs(trace.DPLVel - point.v) < DPLVELGATEBSD)
				{
					samDPL = 1;
				}
			}
			else
			{
				fprintf(fWrite, "In BSD sudu yihao ID = %d, flag = %d, XkArray_yfit = %f DPLgate = %f\n", trace.ID, trace.TraceFlag * 1, trace.XkArray_yfit[0][0], DPLVELGATE * 5);
				if (fabs(fabs(trace.DPLVel) - fabs(point.v)) < DPLVELGATEBSD)
				{
					samDPL = 1;
				}
			}
			
        }
    }
    else
    {
		fprintf(fWrite, " NOT IN BSD\n");
        if (fabs(trace.DPLVel - point.v) < DPLVELGATE * 1.2)
        {
            samDPL = 1;
        }
    }
	if (trace.DPLVel < 0 && trace.XkArray_yfit[0][0] > 0 && point.v > 0 && point.y < 0)
	{
		samDPL = -1;
	}

    return samDPL;
}

/************************************************************************/
/*


 */
/************************************************************************/
char SamAmp(TwiceDataKalman trace, OnceData point)
{
    int32_t samAmp = -1;

	fprintf(fWrite, "samAmp JUDGE trace.AMP = %f   point.a = %f\n", trace.AMP, point.a);
    if (fabs(trace.AMP - point.a) < AMPGATE)
    {
        samAmp = 1;
    }

    return samAmp;
}



/************************************************************************/
/*

 */
/************************************************************************/

char SamVel(TwiceDataKalman trace, OnceData point)
{
    char samVel = -1;
    float Vx = 0;
    float Vy = 0;
    //float DifAng = point.ang - trace.ang;
    float DifAng = AngGap(point.ang, trace.ang);
    float angRation = AngRation(point.ang, trace.ang);

    Vx = point.dis * (angRation) * cosf(point.ang) + point.v * sinf(point.ang);
    Vy = point.v * cosf(point.ang) - point.dis * (angRation) * sinf(point.ang);
    if (fabs(Vx - trace.XkArray_xfit[1][0])<  XVELDIFFERENTIALGATE &&  fabs(Vy - trace.XkArray_yfit[1][0])<YVELDIFFERENTIALGATE)
    {
        samVel = 1;
    }

    return samVel;
}

/************************************************************************/
/*


 */
/************************************************************************/
char JudgeSpecial(TwiceDataKalman trace, OnceData point)
{
    char sam = -1;
    float Vy = 0;
    float DifY = point.y - trace.XkArray_yfit[0][0];

    Vy = DifY / TimeGap / ScanGap;
    if (fabs(trace.XkArray_yfit[0][0]) > 3.5)
    {
        if (trace.XkArray_yfit[0][0] > 0)
        {
            if (fabs(-1 * Vy - point.v) < 10)
            {
                sam = 1;
            }
        }
        else
        {
            if (fabs( Vy - point.v) < 10)
            {
                sam = 1;
            }
        }
    }
    else
    {
        sam = 1;
    }

    return sam;
}


/************************************************************************/
/*


 */
/************************************************************************/
char JudgeSAME(TwiceDataKalman trace, OnceData point)
{

    char sam = -1;

    if (trace.TraceFlag == 0)
    {
        if (fabs(point.x - trace.XkArray_xfit[0][0]) < 0.0001 && fabs(point.y - trace.XkArray_yfit[0][0]) < 0.0001 && fabs(point.v - trace.DPLVel) < 0.0001)
        {
            if (fabs(point.v) > 10.0)
            {
                sam = -1;
            }
        }
	}
	else
	{
		sam = 1;
	}

	return sam;
}


float AngGap(float PointAng, float TraceAng)
{
	float gap = 0; //�Ƕȱ仯

	if (fabs(PointAng - TraceAng) > 3.141592653589793) //��ͬ������
	{
		if (PointAng >= 0 && PointAng < 3.141592653589793)
		{
			gap = PointAng + (6.283185307179586 - TraceAng);
		}
		if (PointAng > 3.1415926 && PointAng < 6.283185307179586)
		{
			gap = PointAng - 6.283185307179586 - TraceAng;
		}
	}
	else                                        //ͬ����
	{
		gap = PointAng - TraceAng;
	}

	return  gap;
}


float AngRation(float PointAng, float TraceAng)
{
	float gap = AngGap(PointAng, TraceAng); //�Ƕȱ仯
	float angVel = 0;

	angVel = gap / (TimeGap * ScanGap);

	return angVel;
}

float XT2DA(float x, float y)
{
	float x1 = fabs(x);
	float y1 = fabs(y);
	float a = 0;

	if (x == 0.0 && y == 0.0)   a = RAD_000;

	else if (x == 0.0f)
	{
		if (y > 0.0f) a = RAD_000;
		if (y < 0.0f) a = RAD_180;
	}
	else if (y == 0.0f)
	{
		if (x > 0.0f) a = RAD_090;
		if (x < 0.0f) a = RAD_270;
	}
	else if (x > 0.0&&y > 0.0)  a = RAD_090 - atanf(y1 / x1); //��һ����
	else if (x > 0.0&&y < 0.0)  a = RAD_090 + atanf(y1 / x1);    //�ڶ�����
	else if (x < 0.0&&y < 0.0)  a = RAD_270 - atanf(y1 / x1);    //��������
	else if (x<0.0&&y>0.0)  a = RAD_270 + atanf(y1 / x1);    //��������
	else a = RAD_000;

	return a;
}



//char SamVelXY(TwiceDataKalman trace, OnceData point)
//{
//    char samVel = -1;
//    float Vx = 0;
//    float Vy = 0;
//
//    Vx = (point.x - trace.XkArray_xfit[0][0]) / (TimeGap * ScanGap);
//    Vy = (point.y - trace.XkArray_yfit[0][0]) / (TimeGap * ScanGap);
//    if (fabs(Vx)<  XVELDIFFERENTIALGATE * 1.5 &&  fabs(Vy)<YVELDIFFERENTIALGATE * 1.5)
//    {
//        samVel = 1;
//    }
//
//    return samVel;
//}


char SamVelXY(TwiceDataKalman trace, OnceData point)
{
	char samVel = 1;
	float Vx = 0;
	float Vy = 0; //km

	//Vx = (point.x - trace.XkArray_xfit[0][0]) / (TimeGap * ScanGap);
	Vy = (point.y - trace.XkArray_yfit[0][0]) / (TimeGap * (trace.iLostNum + 1)) * 3.6;

	if (trace.YBeg < 0 && fabs(fabs(Vy) - fabs(point.v * 3.6) > 5.5))
	{
		samVel = -1;
	}

	return samVel;
}

char JudgeReason(TwiceDataKalman trace, OnceData point)
{

	char reanflag = 1;
	if(trace.XkArray_yfit[0][0] > 0)
	{
		if(trace.DPLVel > 0 && point.v > 0)// go near
		{
			if(point.y > (trace.y_corr))
			{
				reanflag = -1;
			}
		}
		if(trace.DPLVel < 0 && point.v < 0)// go faraway
		{
			if(point.y < trace.y_corr)
			{
				reanflag = -1;
			}
		}

	}
	else
	{
		if(trace.DPLVel > 0 && point.v > 0)// go near
		{
			if(point.y < trace.y_corr)
			{
				reanflag = -1;
			}
		}
		if(trace.DPLVel < 0 && point.v < 0)// go faraway
		{
			if(point.y > trace.y_corr)
			{
				reanflag = -1;
			}
		}
	}
	return reanflag;
}
	