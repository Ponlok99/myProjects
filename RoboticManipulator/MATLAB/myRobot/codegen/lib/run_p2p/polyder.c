/*
 * File: polyder.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "polyder.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double u[7]
 *                double a_data[]
 *                int a_size[2]
 * Return Type  : void
 */
void b_polyder(const double u[7], double a_data[], int a_size[2])
{
  int k;
  int nlead0;
  int ny_tmp;
  nlead0 = 0;
  k = 0;
  while ((k < 5) && (u[k] == 0.0)) {
    nlead0++;
    k++;
  }
  ny_tmp = 6 - nlead0;
  a_size[0] = 1;
  a_size[1] = 6 - nlead0;
  for (k = 0; k < ny_tmp; k++) {
    a_data[k] = u[k + nlead0];
  }
  for (k = 0; k <= ny_tmp - 2; k++) {
    a_data[k] *= (double)(5 - (nlead0 + k)) + 1.0;
  }
  if (rtIsInf(u[6]) || rtIsNaN(u[6])) {
    a_data[5 - nlead0] = rtNaN;
  }
}

/*
 * Arguments    : const double u[8]
 *                double a_data[]
 *                int a_size[2]
 * Return Type  : void
 */
void polyder(const double u[8], double a_data[], int a_size[2])
{
  int k;
  int nlead0;
  int ny_tmp;
  nlead0 = 0;
  k = 0;
  while ((k < 6) && (u[k] == 0.0)) {
    nlead0++;
    k++;
  }
  ny_tmp = 7 - nlead0;
  a_size[0] = 1;
  a_size[1] = 7 - nlead0;
  for (k = 0; k < ny_tmp; k++) {
    a_data[k] = u[k + nlead0];
  }
  for (k = 0; k <= ny_tmp - 2; k++) {
    a_data[k] *= (double)(6 - (nlead0 + k)) + 1.0;
  }
  if (rtIsInf(u[7]) || rtIsNaN(u[7])) {
    a_data[6 - nlead0] = rtNaN;
  }
}

/*
 * File trailer for polyder.c
 *
 * [EOF]
 */
