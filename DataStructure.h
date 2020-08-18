#ifndef DATASTRUCTURE_H_
#define DATASTRUCTURE_H_

#include <stdint.h>

//#include "io_interface/detected_obj.h"
#include "detected_obj.h"

#define MMW_MAX_OBJ_OUT1    (50)
#define MMW_MAX_OBJ_OUT_BSD (7)
#define TimeGap             (0.03)
#define SCANGAPLISTNUM      (6)


#define   LaneWidth  (3.3)
#define   LaneRightAfterLeft	(0.7 - LaneWidth)
#define   LaneRightAfterRight	(0.7)
#define   LaneCloseLeft			(0.7)
#define   LaneCloseRight		(0.7 + LaneWidth)
#define   LaneCloseNearLeft		(0.7 + LaneWidth)
#define   LaneCloseNearRight	(0.7 + LaneWidth * 2)

#define   LaneRightAfterMidle	(LaneRightAfterLeft + LaneWidth / 2)
#define   LaneCloseMidle		(LaneCloseLeft + LaneWidth / 2)
#define   LaneCloseNearMidle	(LaneCloseNearLeft + LaneWidth / 2)




typedef volatile struct DetectedObj
{
    float   d;              /*!< ����     */
    float   ang;            /*!< �Ƕ�     ���ε㼣�� ��XY����   -180 ~ 180  */
    float   v;              /*!< �����ٶ�       ���ε㼣 ������ֱ�Ӹ��� */
    float   x;              /*!< X ����   */
    float   y;              /*!< Y ����   */
    float   a;              /*!< ����     */
    float   z;              /*!< һ�ε�� �Ƿ������  ���ε�� ID     */
} DetectedObject;

//typedef volatile struct MmwDemo_detectedObj_t_float
//{
//    float   range;     /*!< @brief Range index */
//    float   doppler2Vel;   /*!< @brief Doppler index. Note that it is changed
//                          to signed integer in order to handle extended maximum velocity.
//                          Neagative values correspond to the object moving toward
//                          sensor, and positive values correspond to the
//                          object moving away from the sensor */
//
//
//                          // unsigned short  peakVal;      /*!< @brief Peak value */            //   FIFFERENT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
//    float  peakVal;      /*!< @brief Peak value */
//    float  x;             /*!< @brief x - coordinate in meters. Q format depends on the range resolution */
//    float  y;             /*!< @brief y - coordinate in meters. Q format depends on the range resolution */
//    float  z;             /*!< @brief z - coordinate in meters. Q format depends on the range resolution */
//    float  angle;             /*!< @brief angle */
//} MmwDemo_detectedObjFloat;

typedef struct {
    MmwDemo_detectedObj dataCF[20];
    int num;
}DataQC;

//20200419change
typedef struct {
    MmwDemo_detectedObjFloat dataCF[20];
    int num;
}DataQCFloat;

//��������
typedef struct DSPData     //MmwDemo_DataPathState_t
{

    int16_t     numDetObj;    /*! @brief   Number of detected objects */

    DetectedObject Detecte[MMW_MAX_OBJ_OUT];

} DSPDataInfo;   //MmwDemo_DataPathState;


//һ�ε㼣
typedef struct OnceDataObj
{
    float x; //λ�� x
    float y; //λ�� y

    float a; //����
    float v; //�ٶ� DPL

    float ang; //�Ƕ�
    float dis; //����

    int16_t used; //������־

}OnceData;

//һ�ε㼣
typedef struct OnceDataProPress
{
    int16_t Num;
    char usless[MMW_MAX_OBJ_OUT1];
    OnceData Prodata[MMW_MAX_OBJ_OUT1];
}OnceDataProPressObj;

//һ�ε㼣
typedef struct OnceDataObjAssociation
{
    float x; //λ�� x
    float y; //λ�� y

    float a; //����
    float v; //�ٶ� DPL

    float ang; //�Ƕ�
    float dis; //����

    int16_t used; //������־
    int16_t index;

}OnceDataAssociation;


//���ε㼣
typedef struct TwiceDataKalmanObj
{
	uint16_t TraceState;		    //
    int8_t  Updata;                 //     
    int8_t  Used;                   //     
    int8_t  TraceFlag;              //     
	int8_t  ToBe;				    //	
	uint8_t ScanGapTotal;           //    
	uint8_t LaneFlag;			    //     

    uint8_t ID;          //
    //uint8_t passY0;      //
	uint8_t Corred;        //
                               //X bef
    float XkArray_xbef[2][1];
    float XkArray_ybef[2][1];

    //X 
    float XkArray_xfit[2][1];
    float XkArray_yfit[2][1];

    //X nex
    float XkArray_xnxt[2][1];
    float XkArray_ynxt[2][1];


    //Pk 
    float PkArray_x[2][2];
    float PkArray_y[2][2];

    float AMP;                 //
    float DPLVel;              //

    float Vel;                 //

    int16_t   iLostNum;            //
    int16_t   iScanNum;            //
    int16_t   iUpdaNum;            //

    float x_corr;              //
    float y_corr;              //

    float fHead;               //

    float ang;                  //
    float dis;                  //
    float Xpast[5];             //change 20191128   
    float Ypast[5];             //change 20191128   
	float DPLVelpast[5];             //change 20191128   

	float YBeg;             //
	float YEng;             //
}TwiceDataKalman;



typedef struct BSDWARINGObj
{
	uint16_t  TraceState;			    //
	uint16_t     iysmal0;               //
	uint16_t     iybig0;                //
	int8_t    Waring;				    //	
	int16_t   iLostNum;                 //
	int16_t   iScanNum;                 //
	int16_t   iUpdaNum;                 //
}BSDWARING;

typedef struct BSDWARINGOcc
{
	uint16_t  TraceState;			    //
	int8_t    Waring;				    //	
	int16_t   iLostNum;                 //
	int16_t   iScanNum;                 //
	int16_t   iUpdaNum;                 //
}BSDWARINGcc;


#endif /* DATASTRUCTURE_H_ */
