/*
 * File: trapveltraj.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "trapveltraj.h"
#include "linspace.h"
#include "ppval.h"
#include "rt_nonfinite.h"
#include "run_p2p_rtwutil.h"
#include "rt_nonfinite.h"
#include <math.h>
#include <string.h>

/* Type Definitions */
#ifndef typedef_cell_wrap_54
#define typedef_cell_wrap_54
typedef struct {
  double f1[4];
} cell_wrap_54;
#endif /* typedef_cell_wrap_54 */

#ifndef struct_emxArray_real_T_9x3
#define struct_emxArray_real_T_9x3
struct emxArray_real_T_9x3 {
  double data[27];
  int size[2];
};
#endif /* struct_emxArray_real_T_9x3 */
#ifndef typedef_emxArray_real_T_9x3
#define typedef_emxArray_real_T_9x3
typedef struct emxArray_real_T_9x3 emxArray_real_T_9x3;
#endif /* typedef_emxArray_real_T_9x3 */

#ifndef typedef_cell_wrap_55
#define typedef_cell_wrap_55
typedef struct {
  emxArray_real_T_9x3 f1;
} cell_wrap_55;
#endif /* typedef_cell_wrap_55 */

/* Function Definitions */
/*
 * Arguments    : double wayPoints[2][3]
 *                double q[101][3]
 *                double qd[101][3]
 *                double qdd[101][3]
 * Return Type  : void
 */
void b_trapveltraj(double wayPoints[2][3], double q[101][3], double qd[101][3],
                   double qdd[101][3])
{
  cell_wrap_54 breaksCell[3];
  cell_wrap_55 coeffsCell[3];
  double c_tmp_data[303];
  double t[101];
  double coeffMat[3][9];
  double parameterMat[6][1][3];
  double breakMat[4][3];
  double evalPointVector[3];
  double d;
  double s0;
  int ppdd_coefs_size[3];
  int b_i;
  int cellSelection;
  int deltaSign;
  int i;
  int i1;
  int indivPolyDim;
  int jj;
  int numComputedPolynomials;
  int partialTrueCount;
  signed char tmp_data[9];
  bool exitg1;
  bool hasMultipleBreaks;
  for (i = 0; i < 101; i++) {
    q[i][0] = 0.0;
    qd[i][0] = 0.0;
    qdd[i][0] = 0.0;
    q[i][1] = 0.0;
    qd[i][1] = 0.0;
    qdd[i][1] = 0.0;
    q[i][2] = 0.0;
    qd[i][2] = 0.0;
    qdd[i][2] = 0.0;
  }
  memset(&coeffMat[0][0], 0, 27U * sizeof(double));
  for (i = 0; i < 4; i++) {
    breakMat[i][0] = 0.0;
    breakMat[i][1] = 0.0;
    breakMat[i][2] = 0.0;
  }
  for (b_i = 0; b_i < 3; b_i++) {
    double coefs[3][3];
    double d1;
    double sF;
    double segATime;
    double segAcc;
    double segVel;
    bool coefIndex[9];
    d = wayPoints[0][b_i];
    s0 = d;
    d1 = wayPoints[1][b_i];
    sF = d1;
    deltaSign = 1;
    if (d1 < d) {
      s0 = d1;
      sF = d;
      deltaSign = -1;
    }
    segVel = 1.5 * (sF - s0);
    segATime = ((s0 - sF) + segVel) / segVel;
    segAcc = segVel / segATime;
    if (s0 == sF) {
      segAcc = 0.0;
      segVel = 0.0;
      segATime = 0.33333333333333331;
    }
    segVel *= (double)deltaSign;
    segAcc *= (double)deltaSign;
    parameterMat[0][0][b_i] = d;
    parameterMat[1][0][b_i] = d1;
    parameterMat[2][0][b_i] = segVel;
    parameterMat[3][0][b_i] = segAcc;
    parameterMat[4][0][b_i] = segATime;
    parameterMat[5][0][b_i] = 1.0;
    for (i = 0; i < 3; i++) {
      coefs[i][0] = 0.0;
      coefs[i][1] = 0.0;
      coefs[i][2] = 0.0;
    }
    if (segVel == 0.0) {
      coefs[2][0] = d;
      coefs[2][1] = d;
      coefs[2][2] = d;
    } else {
      coefs[0][0] = segAcc / 2.0;
      coefs[1][0] = 0.0;
      coefs[2][0] = d;
      coefs[0][1] = 0.0;
      coefs[1][1] = segVel;
      s0 = segAcc / 2.0 * (segATime * segATime);
      coefs[2][1] = s0 + d;
      coefs[0][2] = -segAcc / 2.0;
      coefs[1][2] = segVel;
      coefs[2][2] = (d1 + s0) - segVel * segATime;
    }
    for (deltaSign = 0; deltaSign < 9; deltaSign++) {
      coefIndex[deltaSign] = false;
    }
    for (i = 0; i < 3; i++) {
      coefIndex[(signed char)((b_i + 3 * i) + 1) - 1] = true;
    }
    partialTrueCount = 0;
    cellSelection = 0;
    for (deltaSign = 0; deltaSign < 9; deltaSign++) {
      if (coefIndex[deltaSign]) {
        tmp_data[partialTrueCount] = (signed char)deltaSign;
        cellSelection = partialTrueCount + 1;
        partialTrueCount++;
      }
    }
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < cellSelection; i1++) {
        coeffMat[i][tmp_data[i1]] = (&coefs[0][0])[i1 + cellSelection * i];
      }
    }
    s0 = breakMat[0][b_i];
    breakMat[1][b_i] = segATime + s0;
    breakMat[2][b_i] = (1.0 - segATime) + s0;
    breakMat[3][b_i] = s0 + 1.0;
  }
  hasMultipleBreaks = false;
  for (b_i = 0; b_i < 2; b_i++) {
    double z1[4];
    bool y;
    z1[0] = fabs(breakMat[0][b_i] - breakMat[0][b_i + 1]);
    z1[1] = fabs(breakMat[1][b_i] - breakMat[1][b_i + 1]);
    z1[2] = fabs(breakMat[2][b_i] - breakMat[2][b_i + 1]);
    z1[3] = fabs(breakMat[3][b_i] - breakMat[3][b_i + 1]);
    y = false;
    partialTrueCount = 0;
    exitg1 = false;
    while ((!exitg1) && (partialTrueCount < 4)) {
      if (z1[partialTrueCount] > 2.2204460492503131E-16) {
        y = true;
        exitg1 = true;
      } else {
        partialTrueCount++;
      }
    }
    if (y || hasMultipleBreaks) {
      hasMultipleBreaks = true;
    } else {
      hasMultipleBreaks = false;
    }
  }
  if (hasMultipleBreaks) {
    coeffsCell[0].f1.size[0] = 3;
    coeffsCell[0].f1.size[1] = 3;
    breaksCell[0].f1[0] = breakMat[0][0];
    breaksCell[0].f1[1] = breakMat[1][0];
    breaksCell[0].f1[2] = breakMat[2][0];
    breaksCell[0].f1[3] = breakMat[3][0];
    coeffsCell[1].f1.size[0] = 3;
    coeffsCell[1].f1.size[1] = 3;
    breaksCell[1].f1[0] = breakMat[0][1];
    breaksCell[1].f1[1] = breakMat[1][1];
    breaksCell[1].f1[2] = breakMat[2][1];
    breaksCell[1].f1[3] = breakMat[3][1];
    coeffsCell[2].f1.size[0] = 3;
    coeffsCell[2].f1.size[1] = 3;
    for (i = 0; i < 3; i++) {
      coeffsCell[0].f1.data[3 * i] = coeffMat[i][0];
      i1 = 3 * i + 1;
      coeffsCell[0].f1.data[i1] = coeffMat[i][3];
      deltaSign = 3 * i + 2;
      coeffsCell[0].f1.data[deltaSign] = coeffMat[i][6];
      coeffsCell[1].f1.data[3 * i] = coeffMat[i][1];
      coeffsCell[1].f1.data[i1] = coeffMat[i][4];
      coeffsCell[1].f1.data[deltaSign] = coeffMat[i][7];
      coeffsCell[2].f1.data[3 * i] = coeffMat[i][2];
      coeffsCell[2].f1.data[i1] = coeffMat[i][5];
      coeffsCell[2].f1.data[deltaSign] = coeffMat[i][8];
    }
    breaksCell[2].f1[0] = breakMat[0][2];
    breaksCell[2].f1[1] = breakMat[1][2];
    breaksCell[2].f1[2] = breakMat[2][2];
    breaksCell[2].f1[3] = breakMat[3][2];
  } else {
    coeffsCell[0].f1.size[0] = 9;
    coeffsCell[0].f1.size[1] = 3;
    breaksCell[0].f1[0] = breakMat[0][0];
    breaksCell[0].f1[1] = breakMat[1][0];
    breaksCell[0].f1[2] = breakMat[2][0];
    breaksCell[0].f1[3] = breakMat[3][0];
    coeffsCell[1].f1.size[0] = 9;
    coeffsCell[1].f1.size[1] = 3;
    breaksCell[1].f1[0] = breakMat[0][0];
    breaksCell[1].f1[1] = breakMat[1][0];
    breaksCell[1].f1[2] = breakMat[2][0];
    breaksCell[1].f1[3] = breakMat[3][0];
    coeffsCell[2].f1.size[0] = 9;
    coeffsCell[2].f1.size[1] = 3;
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < 9; i1++) {
        d = coeffMat[i][i1];
        deltaSign = i1 + 9 * i;
        coeffsCell[0].f1.data[deltaSign] = d;
        coeffsCell[1].f1.data[deltaSign] = d;
        coeffsCell[2].f1.data[deltaSign] = d;
      }
    }
    breaksCell[2].f1[0] = breakMat[0][0];
    breaksCell[2].f1[1] = breakMat[1][0];
    breaksCell[2].f1[2] = breakMat[2][0];
    breaksCell[2].f1[3] = breakMat[3][0];
  }
  evalPointVector[0] = (&parameterMat[0][0][0])[15];
  evalPointVector[1] = (&parameterMat[0][0][0])[16];
  evalPointVector[2] = (&parameterMat[0][0][0])[17];
  if (!rtIsNaN(evalPointVector[0])) {
    deltaSign = 1;
  } else {
    deltaSign = 0;
    partialTrueCount = 2;
    exitg1 = false;
    while ((!exitg1) && (partialTrueCount <= 3)) {
      if (!rtIsNaN(evalPointVector[partialTrueCount - 1])) {
        deltaSign = partialTrueCount;
        exitg1 = true;
      } else {
        partialTrueCount++;
      }
    }
  }
  if (deltaSign == 0) {
    s0 = evalPointVector[0];
  } else {
    s0 = evalPointVector[deltaSign - 1];
    i = deltaSign + 1;
    for (partialTrueCount = i; partialTrueCount < 4; partialTrueCount++) {
      d = evalPointVector[partialTrueCount - 1];
      if (s0 < d) {
        s0 = d;
      }
    }
  }
  linspace(0.0, s0, t);
  if (hasMultipleBreaks) {
    numComputedPolynomials = 3;
    indivPolyDim = 1;
  } else {
    numComputedPolynomials = 1;
    indivPolyDim = 3;
  }
  ppdd_coefs_size[0] = indivPolyDim;
  ppdd_coefs_size[1] = 5;
  ppdd_coefs_size[2] = 3;
  for (jj = 0; jj < numComputedPolynomials; jj++) {
    double dCoeffs_data[45];
    double ddCoeffs_data[45];
    double newCoefs_data[45];
    double pp_coefs_data[45];
    double ppd_coefs_data[45];
    double coefsWithFlatStart_data[36];
    double valueAtEnd_data[12];
    double newSegmentCoeffs_data[9];
    double newBreaks[6];
    double breaksWithFlatStart[5];
    int tmp_size[2];
    int loop_ub_tmp;
    int rowSelection_size_idx_1;
    signed char b_tmp_data[3];
    signed char rowSelection_data[3];
    if (hasMultipleBreaks) {
      rowSelection_size_idx_1 = 1;
      rowSelection_data[0] = (signed char)(jj + 1);
      cellSelection = jj;
    } else {
      rowSelection_size_idx_1 = 3;
      rowSelection_data[0] = 1;
      rowSelection_data[1] = 2;
      rowSelection_data[2] = 3;
      cellSelection = 0;
    }
    for (b_i = 0; b_i < indivPolyDim; b_i++) {
      evalPointVector[b_i] =
          (coeffsCell[cellSelection].f1.data[b_i] * 0.0 +
           coeffsCell[cellSelection]
                   .f1.data[b_i + coeffsCell[cellSelection].f1.size[0]] *
               0.0) +
          coeffsCell[cellSelection]
              .f1.data[b_i + coeffsCell[cellSelection].f1.size[0] * 2];
    }
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < indivPolyDim; i1++) {
        newSegmentCoeffs_data[i1 + indivPolyDim * i] = 0.0;
      }
    }
    for (i = 0; i < indivPolyDim; i++) {
      newSegmentCoeffs_data[i + indivPolyDim * 2] = evalPointVector[i];
    }
    partialTrueCount = coeffsCell[cellSelection].f1.size[0];
    loop_ub_tmp = partialTrueCount + indivPolyDim;
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < loop_ub_tmp; i1++) {
        coefsWithFlatStart_data[i1 + loop_ub_tmp * i] = 0.0;
      }
    }
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < indivPolyDim; i1++) {
        coefsWithFlatStart_data[i1 + loop_ub_tmp * i] =
            newSegmentCoeffs_data[i1 + indivPolyDim * i];
      }
    }
    i = loop_ub_tmp - indivPolyDim;
    for (i1 = 0; i1 < 3; i1++) {
      for (deltaSign = 0; deltaSign < partialTrueCount; deltaSign++) {
        coefsWithFlatStart_data[(indivPolyDim + deltaSign) + loop_ub_tmp * i1] =
            coeffsCell[cellSelection]
                .f1.data[deltaSign + coeffsCell[cellSelection].f1.size[0] * i1];
      }
    }
    breaksWithFlatStart[0] = breaksCell[cellSelection].f1[0] - 1.0;
    breaksWithFlatStart[1] = breaksCell[cellSelection].f1[0];
    breaksWithFlatStart[2] = breaksCell[cellSelection].f1[1];
    breaksWithFlatStart[3] = breaksCell[cellSelection].f1[2];
    breaksWithFlatStart[4] = breaksCell[cellSelection].f1[3];
    s0 = breaksWithFlatStart[4] - breaksWithFlatStart[3];
    evalPointVector[0] = rt_powd_snf(s0, 2.0);
    evalPointVector[1] = rt_powd_snf(s0, 1.0);
    evalPointVector[2] = rt_powd_snf(s0, 0.0);
    if (i + 1 > loop_ub_tmp) {
      i = 0;
      i1 = 0;
    } else {
      i1 = loop_ub_tmp;
    }
    deltaSign = i1 - i;
    for (b_i = 0; b_i < deltaSign; b_i++) {
      partialTrueCount = i + b_i;
      valueAtEnd_data[b_i] =
          (coefsWithFlatStart_data[partialTrueCount] * evalPointVector[0] +
           coefsWithFlatStart_data[partialTrueCount + loop_ub_tmp] *
               evalPointVector[1]) +
          coefsWithFlatStart_data[partialTrueCount + loop_ub_tmp * 2] *
              evalPointVector[2];
    }
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < indivPolyDim; i1++) {
        newSegmentCoeffs_data[i1 + indivPolyDim * i] = 0.0;
      }
    }
    for (i = 0; i < indivPolyDim; i++) {
      newSegmentCoeffs_data[i + indivPolyDim * 2] = valueAtEnd_data[i];
    }
    partialTrueCount = loop_ub_tmp + indivPolyDim;
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < partialTrueCount; i1++) {
        newCoefs_data[i1 + partialTrueCount * i] = 0.0;
      }
      for (i1 = 0; i1 < loop_ub_tmp; i1++) {
        newCoefs_data[i1 + partialTrueCount * i] =
            coefsWithFlatStart_data[i1 + loop_ub_tmp * i];
      }
    }
    if (loop_ub_tmp + 1 > partialTrueCount) {
      loop_ub_tmp = 0;
      i = 0;
    } else {
      i = partialTrueCount;
    }
    deltaSign = i - loop_ub_tmp;
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < deltaSign; i1++) {
        newCoefs_data[(loop_ub_tmp + i1) + partialTrueCount * i] =
            newSegmentCoeffs_data[i1 + indivPolyDim * i];
      }
    }
    for (i = 0; i < 5; i++) {
      newBreaks[i] = breaksWithFlatStart[i];
    }
    newBreaks[5] = breaksWithFlatStart[4] + 1.0;
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < partialTrueCount; i1++) {
        dCoeffs_data[i1 + partialTrueCount * i] = 0.0;
      }
    }
    for (b_i = 0; b_i < 2; b_i++) {
      for (i = 0; i < partialTrueCount; i++) {
        dCoeffs_data[i + partialTrueCount * (b_i + 1)] =
            ((3.0 - ((double)b_i + 2.0)) + 1.0) *
            newCoefs_data[i + partialTrueCount * b_i];
      }
    }
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < partialTrueCount; i1++) {
        ddCoeffs_data[i1 + partialTrueCount * i] = 0.0;
      }
    }
    for (b_i = 0; b_i < 2; b_i++) {
      for (i = 0; i < partialTrueCount; i++) {
        ddCoeffs_data[i + partialTrueCount * (b_i + 1)] =
            ((3.0 - ((double)b_i + 2.0)) + 1.0) *
            dCoeffs_data[i + partialTrueCount * b_i];
      }
    }
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < 5; i1++) {
        for (deltaSign = 0; deltaSign < indivPolyDim; deltaSign++) {
          pp_coefs_data[(deltaSign + indivPolyDim * i1) +
                        indivPolyDim * 5 * i] =
              newCoefs_data[(deltaSign + indivPolyDim * i1) +
                            indivPolyDim * 5 * i];
        }
      }
    }
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < 5; i1++) {
        for (deltaSign = 0; deltaSign < indivPolyDim; deltaSign++) {
          ppd_coefs_data[(deltaSign + indivPolyDim * i1) +
                         indivPolyDim * 5 * i] =
              dCoeffs_data[(deltaSign + indivPolyDim * i1) +
                           indivPolyDim * 5 * i];
        }
      }
    }
    for (i = 0; i < 3; i++) {
      for (i1 = 0; i1 < 5; i1++) {
        for (deltaSign = 0; deltaSign < indivPolyDim; deltaSign++) {
          newCoefs_data[(deltaSign + indivPolyDim * i1) +
                        indivPolyDim * 5 * i] =
              ddCoeffs_data[(deltaSign + indivPolyDim * i1) +
                            indivPolyDim * 5 * i];
        }
      }
    }
    for (i = 0; i < rowSelection_size_idx_1; i++) {
      b_tmp_data[i] = (signed char)(rowSelection_data[i] - 1);
    }
    ppval(newBreaks, pp_coefs_data, ppdd_coefs_size, t, c_tmp_data, tmp_size);
    for (i = 0; i < 101; i++) {
      for (i1 = 0; i1 < rowSelection_size_idx_1; i1++) {
        q[i][b_tmp_data[i1]] = c_tmp_data[i1 + tmp_size[0] * i];
      }
    }
    ppval(newBreaks, ppd_coefs_data, ppdd_coefs_size, t, c_tmp_data, tmp_size);
    for (i = 0; i < 101; i++) {
      for (i1 = 0; i1 < rowSelection_size_idx_1; i1++) {
        qd[i][b_tmp_data[i1]] = c_tmp_data[i1 + tmp_size[0] * i];
      }
    }
    ppval(newBreaks, newCoefs_data, ppdd_coefs_size, t, c_tmp_data, tmp_size);
    for (i = 0; i < 101; i++) {
      for (i1 = 0; i1 < rowSelection_size_idx_1; i1++) {
        qdd[i][b_tmp_data[i1]] = c_tmp_data[i1 + tmp_size[0] * i];
      }
    }
  }
}

/*
 * File trailer for trapveltraj.c
 *
 * [EOF]
 */
