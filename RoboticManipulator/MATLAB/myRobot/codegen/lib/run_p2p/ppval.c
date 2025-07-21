/*
 * File: ppval.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "ppval.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double pp_breaks[4]
 *                double pp_coefs[6][3][3]
 *                double v[101][3]
 * Return Type  : void
 */
void b_ppval(const double pp_breaks[4], double pp_coefs[6][3][3],
             double v[101][3])
{
  int ix;
  int low_ip1;
  for (ix = 0; ix < 101; ix++) {
    double d;
    double d1;
    double d2;
    double xloc;
    int high_i;
    int icp;
    int iv0;
    int low_i;
    iv0 = ix * 3;
    low_i = 1;
    low_ip1 = 2;
    high_i = 4;
    while (high_i > low_ip1) {
      icp = (low_i + high_i) >> 1;
      if (0.01 * (double)ix >= pp_breaks[icp - 1]) {
        low_i = icp;
        low_ip1 = icp + 1;
      } else {
        high_i = icp;
      }
    }
    icp = (low_i - 1) * 3;
    xloc = 0.01 * (double)ix - pp_breaks[low_i - 1];
    d = (&pp_coefs[0][0][0])[icp];
    d1 = (&pp_coefs[0][0][0])[icp + 1];
    d2 = (&pp_coefs[0][0][0])[icp + 2];
    for (low_ip1 = 0; low_ip1 < 5; low_ip1++) {
      high_i = icp + (low_ip1 + 1) * 9;
      d = xloc * d + (&pp_coefs[0][0][0])[high_i];
      d1 = xloc * d1 + (&pp_coefs[0][0][0])[high_i + 1];
      d2 = xloc * d2 + (&pp_coefs[0][0][0])[high_i + 2];
    }
    (&v[0][0])[iv0 + 2] = d2;
    (&v[0][0])[iv0 + 1] = d1;
    (&v[0][0])[iv0] = d;
  }
}

/*
 * Arguments    : const double pp_breaks[4]
 *                double pp_coefs[4][3][3]
 *                double v[101][3]
 * Return Type  : void
 */
void c_ppval(const double pp_breaks[4], double pp_coefs[4][3][3],
             double v[101][3])
{
  int ix;
  int low_ip1;
  for (ix = 0; ix < 101; ix++) {
    double d;
    double d1;
    double d2;
    double xloc;
    int high_i;
    int icp;
    int iv0;
    int low_i;
    iv0 = ix * 3;
    low_i = 1;
    low_ip1 = 2;
    high_i = 4;
    while (high_i > low_ip1) {
      icp = (low_i + high_i) >> 1;
      if (0.01 * (double)ix >= pp_breaks[icp - 1]) {
        low_i = icp;
        low_ip1 = icp + 1;
      } else {
        high_i = icp;
      }
    }
    icp = (low_i - 1) * 3;
    xloc = 0.01 * (double)ix - pp_breaks[low_i - 1];
    d = (&pp_coefs[0][0][0])[icp];
    d1 = (&pp_coefs[0][0][0])[icp + 1];
    d2 = (&pp_coefs[0][0][0])[icp + 2];
    for (low_ip1 = 0; low_ip1 < 3; low_ip1++) {
      high_i = icp + (low_ip1 + 1) * 9;
      d = xloc * d + (&pp_coefs[0][0][0])[high_i];
      d1 = xloc * d1 + (&pp_coefs[0][0][0])[high_i + 1];
      d2 = xloc * d2 + (&pp_coefs[0][0][0])[high_i + 2];
    }
    (&v[0][0])[iv0 + 2] = d2;
    (&v[0][0])[iv0 + 1] = d1;
    (&v[0][0])[iv0] = d;
  }
}

/*
 * Arguments    : const double pp_breaks[6]
 *                const double pp_coefs_data[]
 *                const int pp_coefs_size[3]
 *                const double x[101]
 *                double v_data[]
 *                int v_size[2]
 * Return Type  : void
 */
void ppval(const double pp_breaks[6], const double pp_coefs_data[],
           const int pp_coefs_size[3], const double x[101], double v_data[],
           int v_size[2])
{
  int coefStride_tmp;
  int elementsPerPage_tmp;
  int ic;
  int ix;
  int low_i;
  elementsPerPage_tmp = pp_coefs_size[0];
  coefStride_tmp = pp_coefs_size[0] * 5;
  v_size[0] = pp_coefs_size[0];
  v_size[1] = 101;
  if (pp_coefs_size[0] == 1) {
    for (ix = 0; ix < 101; ix++) {
      double xloc;
      if (rtIsNaN(x[ix])) {
        xloc = rtNaN;
      } else {
        int high_i;
        int low_ip1;
        low_i = 0;
        low_ip1 = 2;
        high_i = 6;
        while (high_i > low_ip1) {
          int icp;
          icp = ((low_i + high_i) + 1) >> 1;
          if (x[ix] >= pp_breaks[icp - 1]) {
            low_i = icp - 1;
            low_ip1 = icp + 1;
          } else {
            high_i = icp;
          }
        }
        xloc = x[ix] - pp_breaks[low_i];
        xloc = xloc * (xloc * pp_coefs_data[low_i] +
                       pp_coefs_data[low_i + coefStride_tmp]) +
               pp_coefs_data[low_i + (coefStride_tmp << 1)];
      }
      v_data[ix] = xloc;
    }
  } else {
    for (ix = 0; ix < 101; ix++) {
      int iv0;
      iv0 = ix * elementsPerPage_tmp;
      if (rtIsNaN(x[ix])) {
        for (low_i = 0; low_i < elementsPerPage_tmp; low_i++) {
          v_data[iv0 + low_i] = x[ix];
        }
      } else {
        double xloc;
        int high_i;
        int icp;
        int low_ip1;
        low_i = 1;
        low_ip1 = 2;
        high_i = 6;
        while (high_i > low_ip1) {
          icp = (low_i + high_i) >> 1;
          if (x[ix] >= pp_breaks[icp - 1]) {
            low_i = icp;
            low_ip1 = icp + 1;
          } else {
            high_i = icp;
          }
        }
        icp = (low_i - 1) * elementsPerPage_tmp;
        xloc = x[ix] - pp_breaks[low_i - 1];
        for (low_i = 0; low_i < elementsPerPage_tmp; low_i++) {
          v_data[iv0 + low_i] = pp_coefs_data[icp + low_i];
        }
        for (ic = 0; ic < 2; ic++) {
          low_ip1 = icp + (ic + 1) * coefStride_tmp;
          for (low_i = 0; low_i < elementsPerPage_tmp; low_i++) {
            high_i = iv0 + low_i;
            v_data[high_i] =
                xloc * v_data[high_i] + pp_coefs_data[low_ip1 + low_i];
          }
        }
      }
    }
  }
}

/*
 * File trailer for ppval.c
 *
 * [EOF]
 */
