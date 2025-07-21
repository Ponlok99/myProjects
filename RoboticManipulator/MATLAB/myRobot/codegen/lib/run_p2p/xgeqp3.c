/*
 * File: xgeqp3.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "xgeqp3.h"
#include "rt_nonfinite.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "xgerc.h"
#include "xnrm2.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Function Declarations */
static int div_nde_s32_floor(int numerator);

/* Function Definitions */
/*
 * Arguments    : int numerator
 * Return Type  : int
 */
static int div_nde_s32_floor(int numerator)
{
  int quotient;
  if ((numerator < 0) && (numerator % 6 != 0)) {
    quotient = -1;
  } else {
    quotient = 0;
  }
  quotient += numerator / 6;
  return quotient;
}

/*
 * Arguments    : double A_data[]
 *                const int A_size[2]
 *                double tau_data[]
 *                int jpvt_data[]
 *                int jpvt_size[2]
 * Return Type  : int
 */
int b_xgeqp3(double A_data[], const int A_size[2], double tau_data[],
             int jpvt_data[], int jpvt_size[2])
{
  double work_data[49];
  int i;
  int ix0;
  int iy;
  int k;
  int n_tmp;
  int tau_size;
  n_tmp = A_size[1];
  if (A_size[1] >= 6) {
    tau_size = 6;
  } else {
    tau_size = A_size[1];
  }
  if (tau_size - 1 >= 0) {
    memset(&tau_data[0], 0, (unsigned int)tau_size * sizeof(double));
  }
  if ((A_size[1] == 0) || (tau_size < 1)) {
    jpvt_size[0] = 1;
    jpvt_size[1] = n_tmp;
    for (iy = 0; iy < n_tmp; iy++) {
      jpvt_data[iy] = iy + 1;
    }
  } else {
    double vn1_data[49];
    double vn2_data[49];
    double absxk;
    double scale;
    double smax;
    double t;
    int kend;
    jpvt_size[0] = 1;
    jpvt_size[1] = n_tmp;
    for (k = 0; k < n_tmp; k++) {
      jpvt_data[k] = k + 1;
      work_data[k] = 0.0;
      vn1_data[k] = 0.0;
      vn2_data[k] = 0.0;
      ix0 = k * 6;
      smax = 0.0;
      scale = 3.3121686421112381E-170;
      kend = ix0 + 6;
      for (iy = ix0 + 1; iy <= kend; iy++) {
        absxk = fabs(A_data[iy - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          smax = smax * t * t + 1.0;
          scale = absxk;
        } else {
          t = absxk / scale;
          smax += t * t;
        }
      }
      absxk = scale * sqrt(smax);
      vn1_data[k] = absxk;
      vn2_data[k] = absxk;
    }
    for (i = 0; i < tau_size; i++) {
      int b_i;
      int ii;
      int ip1;
      int nmi;
      int pvt;
      ip1 = i + 2;
      ii = i * 6 + i;
      nmi = n_tmp - i;
      if (nmi < 1) {
        kend = -1;
      } else {
        kend = 0;
        if (nmi > 1) {
          smax = fabs(vn1_data[i]);
          for (k = 2; k <= nmi; k++) {
            scale = fabs(vn1_data[(i + k) - 1]);
            if (scale > smax) {
              kend = k - 1;
              smax = scale;
            }
          }
        }
      }
      pvt = i + kend;
      if (pvt != i) {
        kend = pvt * 6;
        iy = i * 6;
        for (k = 0; k < 6; k++) {
          ix0 = kend + k;
          smax = A_data[ix0];
          b_i = iy + k;
          A_data[ix0] = A_data[b_i];
          A_data[b_i] = smax;
        }
        kend = jpvt_data[pvt];
        jpvt_data[pvt] = jpvt_data[i];
        jpvt_data[i] = kend;
        vn1_data[pvt] = vn1_data[i];
        vn2_data[pvt] = vn2_data[i];
      }
      if (i + 1 < 6) {
        t = A_data[ii];
        ix0 = ii + 2;
        tau_data[i] = 0.0;
        smax = b_xnrm2(5 - i, A_data, ii + 2);
        if (smax != 0.0) {
          absxk = A_data[ii];
          scale = fabs(absxk);
          smax = fabs(smax);
          if (scale < smax) {
            scale /= smax;
            smax *= sqrt(scale * scale + 1.0);
          } else if (scale > smax) {
            smax /= scale;
            smax = scale * sqrt(smax * smax + 1.0);
          } else if (rtIsNaN(smax)) {
            smax = rtNaN;
          } else {
            smax = scale * 1.4142135623730951;
          }
          if (absxk >= 0.0) {
            smax = -smax;
          }
          if (fabs(smax) < 1.0020841800044864E-292) {
            kend = 0;
            b_i = (ii - i) + 6;
            do {
              kend++;
              for (k = ix0; k <= b_i; k++) {
                A_data[k - 1] *= 9.9792015476736E+291;
              }
              smax *= 9.9792015476736E+291;
              t *= 9.9792015476736E+291;
            } while ((fabs(smax) < 1.0020841800044864E-292) && (kend < 20));
            scale = fabs(t);
            smax = fabs(b_xnrm2(5 - i, A_data, ii + 2));
            if (scale < smax) {
              scale /= smax;
              smax *= sqrt(scale * scale + 1.0);
            } else if (scale > smax) {
              smax /= scale;
              smax = scale * sqrt(smax * smax + 1.0);
            } else if (rtIsNaN(smax)) {
              smax = rtNaN;
            } else {
              smax = scale * 1.4142135623730951;
            }
            if (t >= 0.0) {
              smax = -smax;
            }
            tau_data[i] = (smax - t) / smax;
            scale = 1.0 / (t - smax);
            for (k = ix0; k <= b_i; k++) {
              A_data[k - 1] *= scale;
            }
            for (k = 0; k < kend; k++) {
              smax *= 1.0020841800044864E-292;
            }
            t = smax;
          } else {
            tau_data[i] = (smax - absxk) / smax;
            scale = 1.0 / (absxk - smax);
            b_i = (ii - i) + 6;
            for (k = ix0; k <= b_i; k++) {
              A_data[k - 1] *= scale;
            }
            t = smax;
          }
        }
        A_data[ii] = t;
      } else {
        tau_data[5] = 0.0;
      }
      if (i + 1 < n_tmp) {
        int lastv;
        t = A_data[ii];
        A_data[ii] = 1.0;
        k = ii + 7;
        if (tau_data[i] != 0.0) {
          bool exitg2;
          lastv = 6 - i;
          kend = (ii - i) + 5;
          while ((lastv > 0) && (A_data[kend] == 0.0)) {
            lastv--;
            kend--;
          }
          nmi -= 2;
          exitg2 = false;
          while ((!exitg2) && (nmi + 1 > 0)) {
            int exitg1;
            kend = (ii + nmi * 6) + 6;
            ix0 = kend;
            do {
              exitg1 = 0;
              if (ix0 + 1 <= kend + lastv) {
                if (A_data[ix0] != 0.0) {
                  exitg1 = 1;
                } else {
                  ix0++;
                }
              } else {
                nmi--;
                exitg1 = 2;
              }
            } while (exitg1 == 0);
            if (exitg1 == 1) {
              exitg2 = true;
            }
          }
        } else {
          lastv = 0;
          nmi = -1;
        }
        if (lastv > 0) {
          if (nmi + 1 != 0) {
            if (nmi >= 0) {
              memset(&work_data[0], 0,
                     (unsigned int)(nmi + 1) * sizeof(double));
            }
            b_i = (ii + 6 * nmi) + 7;
            for (iy = k; iy <= b_i; iy += 6) {
              smax = 0.0;
              pvt = iy + lastv;
              for (ix0 = iy; ix0 < pvt; ix0++) {
                smax += A_data[ix0 - 1] * A_data[(ii + ix0) - iy];
              }
              kend = div_nde_s32_floor((iy - ii) - 7);
              work_data[kend] += smax;
            }
          }
          if (!(-tau_data[i] == 0.0)) {
            kend = ii;
            for (iy = 0; iy <= nmi; iy++) {
              absxk = work_data[iy];
              if (absxk != 0.0) {
                smax = absxk * -tau_data[i];
                b_i = kend + 7;
                pvt = lastv + kend;
                for (ix0 = b_i; ix0 <= pvt + 6; ix0++) {
                  A_data[ix0 - 1] += A_data[((ii + ix0) - kend) - 7] * smax;
                }
              }
              kend += 6;
            }
          }
        }
        A_data[ii] = t;
      }
      for (iy = ip1; iy <= n_tmp; iy++) {
        kend = i + (iy - 1) * 6;
        absxk = vn1_data[iy - 1];
        if (absxk != 0.0) {
          smax = fabs(A_data[kend]) / absxk;
          smax = 1.0 - smax * smax;
          if (smax < 0.0) {
            smax = 0.0;
          }
          scale = absxk / vn2_data[iy - 1];
          scale = smax * (scale * scale);
          if (scale <= 1.4901161193847656E-8) {
            if (i + 1 < 6) {
              absxk = b_xnrm2(5 - i, A_data, kend + 2);
              vn1_data[iy - 1] = absxk;
              vn2_data[iy - 1] = absxk;
            } else {
              vn1_data[iy - 1] = 0.0;
              vn2_data[iy - 1] = 0.0;
            }
          } else {
            vn1_data[iy - 1] = absxk * sqrt(smax);
          }
        }
      }
    }
  }
  return tau_size;
}

/*
 * Arguments    : emxArray_real_T *A
 *                emxArray_real_T *tau
 *                emxArray_int32_T *jpvt
 * Return Type  : void
 */
void xgeqp3(emxArray_real_T *A, emxArray_real_T *tau, emxArray_int32_T *jpvt)
{
  emxArray_real_T *vn1;
  emxArray_real_T *vn2;
  emxArray_real_T *work;
  double *A_data;
  double *tau_data;
  double *vn1_data;
  double *vn2_data;
  double *work_data;
  int b_i;
  int i;
  int ia;
  int k;
  int knt;
  int m_tmp;
  int minmana;
  int n_tmp;
  int nmi;
  int *jpvt_data;
  bool guard1;
  A_data = A->data;
  m_tmp = A->size[0];
  n_tmp = A->size[1];
  knt = A->size[0];
  minmana = A->size[1];
  if (knt <= minmana) {
    minmana = knt;
  }
  i = tau->size[0];
  tau->size[0] = minmana;
  emxEnsureCapacity_real_T(tau, i);
  tau_data = tau->data;
  for (i = 0; i < minmana; i++) {
    tau_data[i] = 0.0;
  }
  emxInit_real_T(&work, 1);
  emxInit_real_T(&vn1, 1);
  emxInit_real_T(&vn2, 1);
  guard1 = false;
  if ((A->size[0] == 0) || (A->size[1] == 0)) {
    guard1 = true;
  } else {
    knt = A->size[0];
    minmana = A->size[1];
    if (knt <= minmana) {
      minmana = knt;
    }
    if (minmana < 1) {
      guard1 = true;
    } else {
      double smax;
      int minmn;
      i = jpvt->size[0] * jpvt->size[1];
      jpvt->size[0] = 1;
      jpvt->size[1] = n_tmp;
      emxEnsureCapacity_int32_T(jpvt, i);
      jpvt_data = jpvt->data;
      for (k = 0; k < n_tmp; k++) {
        jpvt_data[k] = k + 1;
      }
      knt = A->size[0];
      minmn = A->size[1];
      if (knt <= minmn) {
        minmn = knt;
      }
      i = work->size[0];
      work->size[0] = n_tmp;
      emxEnsureCapacity_real_T(work, i);
      work_data = work->data;
      i = vn1->size[0];
      vn1->size[0] = n_tmp;
      emxEnsureCapacity_real_T(vn1, i);
      vn1_data = vn1->data;
      i = vn2->size[0];
      vn2->size[0] = n_tmp;
      emxEnsureCapacity_real_T(vn2, i);
      vn2_data = vn2->data;
      for (knt = 0; knt < n_tmp; knt++) {
        work_data[knt] = 0.0;
        smax = xnrm2(m_tmp, A, knt * m_tmp + 1);
        vn1_data[knt] = smax;
        vn2_data[knt] = smax;
      }
      for (b_i = 0; b_i < minmn; b_i++) {
        double s;
        double temp2;
        int ii;
        int ii_tmp;
        int ip1;
        int mmi;
        int pvt;
        ip1 = b_i + 2;
        ii_tmp = b_i * m_tmp;
        ii = ii_tmp + b_i;
        nmi = n_tmp - b_i;
        mmi = m_tmp - b_i;
        if (nmi < 1) {
          minmana = -1;
        } else {
          minmana = 0;
          if (nmi > 1) {
            smax = fabs(vn1_data[b_i]);
            for (k = 2; k <= nmi; k++) {
              s = fabs(vn1_data[(b_i + k) - 1]);
              if (s > smax) {
                minmana = k - 1;
                smax = s;
              }
            }
          }
        }
        pvt = b_i + minmana;
        if (pvt + 1 != b_i + 1) {
          minmana = pvt * m_tmp;
          for (k = 0; k < m_tmp; k++) {
            knt = minmana + k;
            smax = A_data[knt];
            i = ii_tmp + k;
            A_data[knt] = A_data[i];
            A_data[i] = smax;
          }
          minmana = jpvt_data[pvt];
          jpvt_data[pvt] = jpvt_data[b_i];
          jpvt_data[b_i] = minmana;
          vn1_data[pvt] = vn1_data[b_i];
          vn2_data[pvt] = vn2_data[b_i];
        }
        if (b_i + 1 < m_tmp) {
          temp2 = A_data[ii];
          minmana = ii + 2;
          tau_data[b_i] = 0.0;
          if (mmi > 0) {
            smax = xnrm2(mmi - 1, A, ii + 2);
            if (smax != 0.0) {
              s = fabs(A_data[ii]);
              smax = fabs(smax);
              if (s < smax) {
                s /= smax;
                smax *= sqrt(s * s + 1.0);
              } else if (s > smax) {
                smax /= s;
                smax = s * sqrt(smax * smax + 1.0);
              } else if (rtIsNaN(smax)) {
                smax = rtNaN;
              } else {
                smax = s * 1.4142135623730951;
              }
              if (A_data[ii] >= 0.0) {
                smax = -smax;
              }
              if (fabs(smax) < 1.0020841800044864E-292) {
                knt = 0;
                i = ii + mmi;
                do {
                  knt++;
                  for (k = minmana; k <= i; k++) {
                    A_data[k - 1] *= 9.9792015476736E+291;
                  }
                  smax *= 9.9792015476736E+291;
                  temp2 *= 9.9792015476736E+291;
                } while ((fabs(smax) < 1.0020841800044864E-292) && (knt < 20));
                s = fabs(temp2);
                smax = fabs(xnrm2(mmi - 1, A, ii + 2));
                if (s < smax) {
                  s /= smax;
                  smax *= sqrt(s * s + 1.0);
                } else if (s > smax) {
                  smax /= s;
                  smax = s * sqrt(smax * smax + 1.0);
                } else if (rtIsNaN(smax)) {
                  smax = rtNaN;
                } else {
                  smax = s * 1.4142135623730951;
                }
                if (temp2 >= 0.0) {
                  smax = -smax;
                }
                tau_data[b_i] = (smax - temp2) / smax;
                s = 1.0 / (temp2 - smax);
                for (k = minmana; k <= i; k++) {
                  A_data[k - 1] *= s;
                }
                for (k = 0; k < knt; k++) {
                  smax *= 1.0020841800044864E-292;
                }
                temp2 = smax;
              } else {
                tau_data[b_i] = (smax - A_data[ii]) / smax;
                s = 1.0 / (A_data[ii] - smax);
                i = ii + mmi;
                for (k = minmana; k <= i; k++) {
                  A_data[k - 1] *= s;
                }
                temp2 = smax;
              }
            }
          }
          A_data[ii] = temp2;
        } else {
          tau_data[b_i] = 0.0;
        }
        if (b_i + 1 < n_tmp) {
          temp2 = A_data[ii];
          A_data[ii] = 1.0;
          ii_tmp = (ii + m_tmp) + 1;
          if (tau_data[b_i] != 0.0) {
            bool exitg2;
            pvt = mmi;
            minmana = (ii + mmi) - 1;
            while ((pvt > 0) && (A_data[minmana] == 0.0)) {
              pvt--;
              minmana--;
            }
            knt = nmi - 1;
            exitg2 = false;
            while ((!exitg2) && (knt > 0)) {
              int exitg1;
              minmana = ii_tmp + (knt - 1) * m_tmp;
              ia = minmana;
              do {
                exitg1 = 0;
                if (ia <= (minmana + pvt) - 1) {
                  if (A_data[ia - 1] != 0.0) {
                    exitg1 = 1;
                  } else {
                    ia++;
                  }
                } else {
                  knt--;
                  exitg1 = 2;
                }
              } while (exitg1 == 0);
              if (exitg1 == 1) {
                exitg2 = true;
              }
            }
          } else {
            pvt = 0;
            knt = 0;
          }
          if (pvt > 0) {
            if (knt != 0) {
              for (nmi = 0; nmi < knt; nmi++) {
                work_data[nmi] = 0.0;
              }
              nmi = 0;
              i = ii_tmp + m_tmp * (knt - 1);
              for (k = ii_tmp; m_tmp < 0 ? k >= i : k <= i; k += m_tmp) {
                smax = 0.0;
                minmana = k + pvt;
                for (ia = k; ia < minmana; ia++) {
                  smax += A_data[ia - 1] * A_data[(ii + ia) - k];
                }
                work_data[nmi] += smax;
                nmi++;
              }
            }
            xgerc(pvt, knt, -tau_data[b_i], ii + 1, work, A, ii_tmp, m_tmp);
            A_data = A->data;
          }
          A_data[ii] = temp2;
        }
        for (knt = ip1; knt <= n_tmp; knt++) {
          minmana = b_i + (knt - 1) * m_tmp;
          smax = vn1_data[knt - 1];
          if (smax != 0.0) {
            s = fabs(A_data[minmana]) / smax;
            s = 1.0 - s * s;
            if (s < 0.0) {
              s = 0.0;
            }
            temp2 = smax / vn2_data[knt - 1];
            temp2 = s * (temp2 * temp2);
            if (temp2 <= 1.4901161193847656E-8) {
              if (b_i + 1 < m_tmp) {
                smax = xnrm2(mmi - 1, A, minmana + 2);
                vn1_data[knt - 1] = smax;
                vn2_data[knt - 1] = smax;
              } else {
                vn1_data[knt - 1] = 0.0;
                vn2_data[knt - 1] = 0.0;
              }
            } else {
              vn1_data[knt - 1] = smax * sqrt(s);
            }
          }
        }
      }
    }
  }
  if (guard1) {
    i = jpvt->size[0] * jpvt->size[1];
    jpvt->size[0] = 1;
    jpvt->size[1] = n_tmp;
    emxEnsureCapacity_int32_T(jpvt, i);
    jpvt_data = jpvt->data;
    for (knt = 0; knt < n_tmp; knt++) {
      jpvt_data[knt] = knt + 1;
    }
  }
  emxFree_real_T(&vn2);
  emxFree_real_T(&vn1);
  emxFree_real_T(&work);
}

/*
 * File trailer for xgeqp3.c
 *
 * [EOF]
 */
