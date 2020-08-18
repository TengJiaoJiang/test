#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "DataStructure.h"
#include "Tracking_process.h"
//#include "Tracking_process.c"
#include "detected_obj.h"

extern char BSDWARINGFLAG;
extern char BSDWARINGFLAGbj;               
extern char BSDWARINGFLAGcc; 
#define MAX_LINE 1024

FILE *fWrite = NULL;
FILE *fWriteTemp = NULL;
long start = 0;
long finish = 0;
long tspend = 0;
const int nunMult = 1;
int main()
{
	int i = 0, j = 0, k = 0;
	// FILE *fp = fopen("/home/robot/CopyFolder/0627/CANData0513ZX/CANData0513/0624/left/c-1COVERonceFangzhen.txt", "r");
	//FILE *fp = fopen("/home/robot/CopyFolder/0627/CANData0513ZX/CANData0513/0624/left/bing-1COVERonceFangzhen.txt", "r");
	
	FILE *fp = fopen("/home/robot/PycharmProjects/charmlearn/0731/XIAWU-XUJING-1COVERonceFangzhen.txt", "r");
	FILE *fpout = NULL;
	MmwDemo_DSS_DataPathObj DSPDataInfomation;
	MmwDemo_DSS_DataPathObj DSPDataInfomationEXT;
	MmwDemo_DSS_DataPathObj DSPDataInfomationOUT;
	char buf[MAX_LINE];			
	int len = 0;				
	int num = 0;				
	char *revbuf[8] = { 0 };	
	char *token;				
	const char *sep = ": ";		
	float valData = 0;			
	float LineData[1024] = { 0 };
	int index = 0;
	int SCANNUM = 0;
	char init = 0;
	float VelCar = 0;
	int frame = 0;

	if (fp  == NULL)//   F:\LA1001\ManagerZX\CANData0513ZX\CANData0513\zawuAna
	{
		printf("A  The data file can not be opened.\n");
		return -1;
	}

	
	if ((fWrite = fopen("TwiceData.txt","w+")) == NULL)
	{
		printf("B   The Twice file can not be opened.\n");
		return -1;
	}

	if ((fWriteTemp = fopen("TwiceDataTemp.txt", "w+")) == NULL)
	{
		printf("C   The Twice file can not be opened.\n");
		return -1;
	}

	if ((fpout = fopen("TwiceDataOUT.txt", "w+")) == NULL)
	{
		printf("D   The Twice file out can not be opened.\n");
		return -1;
	}


	init = TRACEINIT();



	if (init < 0)
	{
		printf("Init Faile\n ");
		return -1;
	}


	while (fgets(buf, MAX_LINE, fp) != NULL)
	{
		frame = frame + 1;
		SCANNUM = SCANNUM + 1;

		//if (SCANNUM == 106)
		{
			
			fprintf(fWrite, "\n\n__ _____________%d__________________________\n\n ", SCANNUM);
			//printf("\n_ _____________%d__________________________\n\n ", SCANNUM);
			memset(LineData, 0, sizeof(float) * 1024);
			memset(&DSPDataInfomation, 0, sizeof(MmwDemo_DSS_DataPathObj));
			memset(&DSPDataInfomationEXT, 0, sizeof(MmwDemo_DSS_DataPathObj));
			memset(&DSPDataInfomationOUT, 0, sizeof(MmwDemo_DSS_DataPathObj));
			len = strlen(buf);
			buf[len - 1] = '\0';  
			token = strtok(buf, sep);
			index = 0;
			while (token)
			{
				//printf("%s -> ", token);
				valData = atof(token); 
				LineData[index] = valData;
				index = index + 1;
				//printf("%f \n", valData);
				token = strtok(NULL, sep);
			}

			VelCar = LineData[0];							
			DSPDataInfomation.xyzQFormat = (int)(LineData[1] + 0.5);             
			//DSPDataInfomation.xyzQFormat  = 1;										
			DSPDataInfomation.numDetObj = LineData[2];                          
			DSPDataInfomation.subFramIndx = LineData[3];



			for (i = 0; i < DSPDataInfomation.numDetObj; i++)
			{
				DSPDataInfomation.detObj2D[i].rangeIdx = LineData[i * 6 + 4];
				DSPDataInfomation.detObj2D[i].angle = LineData[i * 6 + 5];
				DSPDataInfomation.detObj2D[i].dopplerIdx = LineData[i * 6 + 6];
				DSPDataInfomation.detObj2D[i].x = LineData[i * 6 + 7] * 100;
				DSPDataInfomation.detObj2D[i].y = LineData[i * 6 + 8] * 100;
				DSPDataInfomation.detObj2D[i].peakVal = LineData[i * 6 + 9];
			}

			fprintf(fWriteTemp, "# %d\n", SCANNUM);
			fprintf(fpout, "# %d\n", SCANNUM);
			TRACKING(&DSPDataInfomation, VelCar, 0);
			
			for (i = 0; i < DSPDataInfomation.numDetObj; i++)
			{
				//if (DSPDataInfomation.detObj2D[i].dopplerIdx / 10.0 > 0)
				//{
				//	continue;
				//}
				fprintf(fpout, " %d  ", DSPDataInfomation.detObj2D[i].z * 1);
				fprintf(fpout, " %f  ", DSPDataInfomation.detObj2D[i].rangeIdx / 10.0);
				fprintf(fpout, " %f  ", DSPDataInfomation.detObj2D[i].angle / 10.0);
				fprintf(fpout, " %f  ", DSPDataInfomation.detObj2D[i].dopplerIdx / 10.0);
				fprintf(fpout, " %f  ", DSPDataInfomation.detObj2D[i].x / 10.0);
				fprintf(fpout, " %f  ", DSPDataInfomation.detObj2D[i].y / 10.0);
				fprintf(fpout, " %f", DSPDataInfomation.detObj2D[i].peakVal / 10.0);

				if ((DSPDataInfomation.detObj2D[i].x / 10.0) >= 0.5 && (DSPDataInfomation.detObj2D[i].x / 10.0) <= 3.5 && (DSPDataInfomation.detObj2D[i].y / 10.0) >= -2.0 && (DSPDataInfomation.detObj2D[i].y / 10.0) <= 5.0)
				{
					fprintf(fpout, "@!!!!!!\n");
				}
				else
				{
					fprintf(fpout, "\n");
				}
			}
			if (BSDWARINGFLAG == 1)
			{
				fprintf(fpout, "BSD WARING BSD WARING BSD WARING BSD WARING BSD WARING BSD WARING BSD WARING@!!!!!!\n"); 
				if(BSDWARINGFLAGbj == 1)
				{
					fprintf(fpout, "BSD WARING alongbsbeside \n"); 
				}
				if(BSDWARINGFLAGcc == 1)
				{
					fprintf(fpout, "BSD WARING tackover \n"); 
				}
			}
			// fprintf(fWriteTemp, "\n", SCANNUM);
			// fprintf(fpout, "\n", SCANNUM);


		}
		
	}
	fclose(fp); 
	fclose(fWrite); 
	fclose(fpout); 
	fclose(fWriteTemp);
	TRACEEXIT();



	//printf("\n\n%f, %f\n", AngleCacul(1.0, 1.0), AngleCacul(-1.0, 1.0));


	//getchar();
	return 0;
}
