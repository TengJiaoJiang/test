#pragma once

#define MMW_MAX_OBJ_OUT 100

typedef volatile struct MmwDemo_detectedObj_t
{
	unsigned short   rangeIdx;     /*!< @brief Range index */
	short   dopplerIdx;   /*!< @brief Doppler index. Note that it is changed
						  to signed integer in order to handle extended maximum velocity.
						  Neagative values correspond to the object moving toward
						  sensor, and positive values correspond to the
						  object moving away from the sensor */


						  // unsigned short  peakVal;      /*!< @brief Peak value */            //   FIFFERENT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	short  peakVal;      /*!< @brief Peak value */
	short  x;             /*!< @brief x - coordinate in meters. Q format depends on the range resolution */
	short  y;             /*!< @brief y - coordinate in meters. Q format depends on the range resolution */
	short  z;             /*!< @brief z - coordinate in meters. Q format depends on the range resolution */
	short  angle;             /*!< @brief angle */
} MmwDemo_detectedObj;

typedef volatile struct MmwDemo_detectedObj_t_float
{
	float   range;     /*!< @brief Range index */
	float   doppler2Vel;   /*!< @brief Doppler index. Note that it is changed
						   to signed integer in order to handle extended maximum velocity.
						   Neagative values correspond to the object moving toward
						   sensor, and positive values correspond to the
						   object moving away from the sensor */


						   // unsigned short  peakVal;      /*!< @brief Peak value */            //   FIFFERENT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

	float  peakVal;      /*!< @brief Peak value */
	float  x;             /*!< @brief x - coordinate in meters. Q format depends on the range resolution */
	float  y;             /*!< @brief y - coordinate in meters. Q format depends on the range resolution */
	float  z;             /*!< @brief z - coordinate in meters. Q format depends on the range resolution */
	float  angle;             /*!< @brief angle */
} MmwDemo_detectedObjFloat;


//ԭʼ�㼣
typedef struct MmwDemo_DSS_DataPathObj_t     //MmwDemo_DataPathState_t
{
	/*! @brief   Number of detected objects */
	unsigned short     numDetObj;

	/*! @brief   Q format of detected objects x/y/z coordinates */
	unsigned short     xyzQFormat;   //

									 /*! @brief   subFramIndx*/
	unsigned char      subFramIndx;

	/*! @brief   rang Resolution */
	float        rangeResolution;

	/*! @brief   doppler Resolution */
	float        dopplerResolution;

	/*! @brief Detected objects after second pass in Range direction */
	MmwDemo_detectedObj detObj2D[MMW_MAX_OBJ_OUT];

} MmwDemo_DSS_DataPathObj;   //MmwDemo_DataPathState;

