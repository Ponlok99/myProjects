/*
 * File: asin.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "asin.h"
#include "atan2.h"
#include "rt_nonfinite.h"
#include "sqrt.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Definitions */
/*
 * Arguments    : creal_T *x
 * Return Type  : void
 */
void b_asin(creal_T *x)
{
  creal_T u;
  creal_T v;
  if ((x->im == 0.0) && (!(fabs(x->re) > 1.0))) {
    x->re = asin(x->re);
    x->im = 0.0;
  } else {
    double absim;
    double absre;
    double b_absre;
    double ci;
    double sai;
    double sar;
    double sbi;
    double sbr;
    double t1;
    double t2;
    double tmp;
    bool b;
    bool b1;
    bool finiteScale;
    bool xneg;
    v.re = x->re + 1.0;
    v.im = x->im;
    b_sqrt(&v);
    u.re = 1.0 - x->re;
    u.im = 0.0 - x->im;
    b_sqrt(&u);
    if ((u.im == 0.0) && (v.im == 0.0)) {
      tmp = u.re * v.re;
    } else {
      t1 = u.re * v.re;
      t2 = u.im * v.im;
      tmp = t1 - t2;
      xneg = rtIsNaN(tmp);
      b = rtIsInf(tmp);
      if ((b || xneg) && (!rtIsNaN(u.re)) && (!rtIsNaN(u.im)) &&
          (!rtIsNaN(v.re)) && (!rtIsNaN(v.im))) {
        absre = fabs(u.re);
        absim = fabs(u.im);
        if (absre > absim) {
          if (u.re < 0.0) {
            sar = -1.0;
          } else {
            sar = 1.0;
          }
          sai = u.im / absre;
        } else if (absim > absre) {
          sar = u.re / absim;
          if (u.im < 0.0) {
            sai = -1.0;
          } else {
            sai = 1.0;
          }
          absre = absim;
        } else {
          if (u.re < 0.0) {
            sar = -1.0;
          } else {
            sar = 1.0;
          }
          if (u.im < 0.0) {
            sai = -1.0;
          } else {
            sai = 1.0;
          }
        }
        b_absre = fabs(v.re);
        absim = fabs(v.im);
        if (b_absre > absim) {
          if (v.re < 0.0) {
            sbr = -1.0;
          } else {
            sbr = 1.0;
          }
          sbi = v.im / b_absre;
        } else if (absim > b_absre) {
          sbr = v.re / absim;
          if (v.im < 0.0) {
            sbi = -1.0;
          } else {
            sbi = 1.0;
          }
          b_absre = absim;
        } else {
          if (v.re < 0.0) {
            sbr = -1.0;
          } else {
            sbr = 1.0;
          }
          if (v.im < 0.0) {
            sbi = -1.0;
          } else {
            sbi = 1.0;
          }
        }
        b1 = rtIsInf(absre);
        if ((!b1) && (!rtIsNaN(absre)) &&
            ((!rtIsInf(b_absre)) && (!rtIsNaN(b_absre)))) {
          finiteScale = true;
        } else {
          finiteScale = false;
        }
        if (xneg || (b && finiteScale)) {
          tmp = sar * sbr - sai * sbi;
          if (tmp != 0.0) {
            tmp = tmp * absre * b_absre;
          } else if ((b1 && ((v.re == 0.0) || (v.im == 0.0))) ||
                     (rtIsInf(b_absre) && ((u.re == 0.0) || (u.im == 0.0)))) {
            if (rtIsNaN(t1)) {
              t1 = 0.0;
            }
            if (rtIsNaN(t2)) {
              t2 = 0.0;
            }
            tmp = t1 - t2;
          }
        }
      }
    }
    if ((-u.im == 0.0) && (v.im == 0.0)) {
      ci = 0.0;
    } else {
      t1 = u.re * v.im;
      t2 = -u.im * v.re;
      ci = t1 + t2;
      xneg = rtIsNaN(ci);
      b = rtIsInf(ci);
      if ((b || xneg) && (!rtIsNaN(u.re)) && (!rtIsNaN(-u.im)) &&
          (!rtIsNaN(v.re)) && (!rtIsNaN(v.im))) {
        absre = fabs(u.re);
        absim = fabs(-u.im);
        if (absre > absim) {
          if (u.re < 0.0) {
            sar = -1.0;
          } else {
            sar = 1.0;
          }
          sai = -u.im / absre;
        } else if (absim > absre) {
          sar = u.re / absim;
          if (-u.im < 0.0) {
            sai = -1.0;
          } else {
            sai = 1.0;
          }
          absre = absim;
        } else {
          if (u.re < 0.0) {
            sar = -1.0;
          } else {
            sar = 1.0;
          }
          if (-u.im < 0.0) {
            sai = -1.0;
          } else {
            sai = 1.0;
          }
        }
        b_absre = fabs(v.re);
        absim = fabs(v.im);
        if (b_absre > absim) {
          if (v.re < 0.0) {
            sbr = -1.0;
          } else {
            sbr = 1.0;
          }
          sbi = v.im / b_absre;
        } else if (absim > b_absre) {
          sbr = v.re / absim;
          if (v.im < 0.0) {
            sbi = -1.0;
          } else {
            sbi = 1.0;
          }
          b_absre = absim;
        } else {
          if (v.re < 0.0) {
            sbr = -1.0;
          } else {
            sbr = 1.0;
          }
          if (v.im < 0.0) {
            sbi = -1.0;
          } else {
            sbi = 1.0;
          }
        }
        b1 = rtIsInf(absre);
        if ((!b1) && (!rtIsNaN(absre)) &&
            ((!rtIsInf(b_absre)) && (!rtIsNaN(b_absre)))) {
          finiteScale = true;
        } else {
          finiteScale = false;
        }
        if (xneg || (b && finiteScale)) {
          ci = sar * sbi + sai * sbr;
          if (ci != 0.0) {
            ci = ci * absre * b_absre;
          } else if ((b1 && ((v.re == 0.0) || (v.im == 0.0))) ||
                     (rtIsInf(b_absre) && ((u.re == 0.0) || (-u.im == 0.0)))) {
            if (rtIsNaN(t1)) {
              t1 = 0.0;
            }
            if (rtIsNaN(t2)) {
              t2 = 0.0;
            }
            ci = t1 + t2;
          }
        }
      }
    }
    xneg = (ci < 0.0);
    if (xneg) {
      ci = -ci;
    }
    if (ci >= 2.68435456E+8) {
      ci = log(ci) + 0.69314718055994529;
    } else if (ci > 2.0) {
      ci = log(2.0 * ci + 1.0 / (sqrt(ci * ci + 1.0) + ci));
    } else {
      t1 = ci * ci;
      ci += t1 / (sqrt(t1 + 1.0) + 1.0);
      t1 = fabs(ci);
      if ((t1 > 4.503599627370496E+15) || (rtIsInf(ci) || rtIsNaN(ci))) {
        ci++;
        ci = log(ci);
      } else if (!(t1 < 2.2204460492503131E-16)) {
        ci = log(ci + 1.0) * (ci / ((ci + 1.0) - 1.0));
      }
    }
    if (xneg) {
      ci = -ci;
    }
    x->re = b_atan2(x->re, tmp);
    x->im = ci;
  }
}

/*
 * File trailer for asin.c
 *
 * [EOF]
 */
