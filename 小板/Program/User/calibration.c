#include "calibration.h"
#include "main.h"


void Calibration_Init(void)
{
	
}

void mat_invert3(float src[3][3], float dst[3][3])
{
	float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
							src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
							src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

	dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;//4857
	dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;//5638
	dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
	dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;//2718
	dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
	dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
	dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
	dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
	dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;


}

