/*
 * File: computePolyCoefAndTimeOfArrival.c
 *
 * MATLAB Coder version            : 24.2
 * C/C++ source code generated on  : 20-Jul-2025 23:35:10
 */

/* Include Files */
#include "computePolyCoefAndTimeOfArrival.h"
#include "constructM.h"
#include "mldivide.h"
#include "rt_nonfinite.h"
#include "run_p2p_emxutil.h"
#include "run_p2p_types.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : double constraints[3][10]
 *                double pp[3][10][1]
 *                emxArray_real_T *timeOfArrival
 * Return Type  : void
 */
void computePolyCoefAndTimeOfArrival(double constraints[3][10],
                                     double pp[3][10][1],
                                     emxArray_real_T *timeOfArrival)
{
  static const double QPrime[10][10] = {
      {164945.45454634493, 82472.727273407741, 16952.727273084369,
       1603.6363637262439, 38.181818183151677, -164945.45454634493,
       82472.727272021119, -16952.727272467688, 1603.6363635764574,
       -38.181818179417633},
      {82472.727273220662, 42356.363636730763, 9036.36363654217,
       901.81818186372311, 22.424242424964149, -82472.727273220662,
       40116.36363600404, -7916.3636362323305, 701.81818178707908,
       -15.757575756330738},
      {16952.727272849064, 9036.363636450973, 2036.3636364018639,
       221.81818182809093, 5.7575757577444548, -16952.727272849064,
       7916.3636362846592, -1476.3636363350379, 121.81818181117706,
       -2.4242424239618146},
      {1603.6363636531751, 901.81818182951429, 221.81818182243251,
       29.090909092028397, 0.78787878790033794, -1603.6363636531751,
       701.81818180912524, -121.818181814946, 9.0909090900727278,
       -0.12121212117852131},
      {38.181818183021278, 22.424242424976, 5.7575757577923667,
       0.78787878793502664, 0.050505050506447935, -38.181818183021278,
       15.757575757085306, -2.4242424240666196, 0.12121212116215929,
       0.0050505050524097816},
      {-164945.45454634493, -82472.727273407741, -16952.727273084369,
       -1603.6363637262439, -38.181818183151677, 164945.45454634493,
       -82472.727272021119, 16952.727272467688, -1603.6363635764574,
       38.181818179417633},
      {82472.727273124037, 40116.363636676921, 7916.3636365422135,
       701.81818186252076, 15.7575757581875, -82472.727273124037,
       42356.363636017079, -9036.3636362354737, 901.81818178939284,
       -22.424242423086923},
      {-16952.727272800868, -7916.3636364240519, -1476.3636364018839,
       -121.81818182748952, -2.4242424243561373, 16952.727272800868,
       -9036.3636362910038, 2036.3636363365949, -221.81818181231574,
       5.7575757573399},
      {1603.6363636431342, 701.81818182403367, 121.81818182240909,
       9.0909090919083724, 0.12121212122065916, -1603.6363636431342,
       901.81818181032213, -221.81818181523886, 29.090909090300556,
       -0.787878787854198},
      {-38.181818182074039, -15.757575757819609, -2.4242424244499148,
       -0.12121212125931535, 0.005050505050019749, 38.181818182074039,
       -22.424242423856867, 5.7575757574322779, -0.78787878785091436,
       0.050505050503915849}};
  static const double b_AInv[10][10] = {
      {1.0, 0.0, 0.0, 0.0, 0.0, -126.00000000005578, 420.00000000019594,
       -540.00000000026034, 315.0000000001553, -70.000000000035072},
      {0.0, 1.0, 0.0, 0.0, 0.0, -70.000000000028734, 224.00000000010124,
       -280.00000000013472, 160.00000000008041, -35.000000000018161},
      {0.0, 0.0, 0.5, 0.0, 0.0, -17.500000000006349, 52.500000000022553,
       -63.000000000030141, 35.000000000018019, -7.5000000000040723},
      {0.0, 0.0, 0.0, 0.16666666666666666, 0.0, -2.5000000000007154,
       6.6666666666692667, -7.5000000000035358, 4.000000000002129,
       -0.83333333333381487},
      {0.0, 0.0, 0.0, 0.0, 0.041666666666666664, -0.20833333333336226,
       0.41666666666678787, -0.41666666666685243, 0.20833333333344822,
       -0.041666666666693275},
      {0.0, 0.0, 0.0, 0.0, 0.0, 126.00000000005578, -420.00000000019594,
       540.00000000026034, -315.0000000001553, 70.000000000035072},
      {0.0, 0.0, 0.0, 0.0, 0.0, -56.000000000027043, 196.0000000000947,
       -260.00000000012562, 155.00000000007489, -35.000000000016911},
      {0.0, 0.0, 0.0, 0.0, 0.0, 10.500000000005503, -38.500000000019284,
       53.000000000025594, -32.500000000015262, 7.500000000003447},
      {0.0, 0.0, 0.0, 0.0, 0.0, -1.0000000000005647, 3.833333333335323,
       -5.5000000000026406, 3.5000000000015747, -0.83333333333368909},
      {0.0, 0.0, 0.0, 0.0, 0.0, 0.041666666666691769, -0.16666666666675478,
       0.25000000000011707, -0.16666666666673657, 0.041666666666682471}};
  c_robotics_core_internal_Damped solver;
  emxArray_real_T *y;
  double *timeOfArrival_data;
  int R_size[2];
  int b_i;
  int dimIdx;
  int i;
  int i1;
  int i2;
  int k;
  signed char tmp_data[10];
  emxInit_real_T(&y, 1);
  c_emxInitStruct_robotics_core_i(&solver);
  for (dimIdx = 0; dimIdx < 3; dimIdx++) {
    double M[10][10];
    double Q[10][10];
    double R_data[100];
    double b_M[10][10];
    double constraints_data[20];
    double AInv[10];
    double b_Q[10];
    double d;
    int b_y;
    int inner;
    int partialTrueCount;
    int trueCount;
    bool numConstraints_tmp[10];
    for (i = 0; i < 10; i++) {
      numConstraints_tmp[i] = !rtIsNaN(constraints[dimIdx][i]);
    }
    b_y = numConstraints_tmp[0];
    for (k = 0; k < 9; k++) {
      b_y += numConstraints_tmp[k + 1];
    }
    b_constructM(&constraints[dimIdx][0], M);
    for (i = 0; i < 10; i++) {
      for (i1 = 0; i1 < 10; i1++) {
        Q[i][i1] = M[i1][i];
        d = 0.0;
        for (i2 = 0; i2 < 10; i2++) {
          d += M[i2][i] * QPrime[i1][i2];
        }
        b_M[i1][i] = d;
      }
    }
    for (i = 0; i < 10; i++) {
      for (i1 = 0; i1 < 10; i1++) {
        d = 0.0;
        for (i2 = 0; i2 < 10; i2++) {
          d += b_M[i2][i] * Q[i1][i2];
        }
        M[i1][i] = d;
      }
    }
    if ((double)b_y + 1.0 > 10.0) {
      i = 0;
      i1 = 0;
      i2 = 0;
    } else {
      i = b_y;
      i1 = 10;
      i2 = b_y;
    }
    if (b_y < 1) {
      inner = 0;
    } else {
      inner = b_y;
    }
    if ((double)b_y + 1.0 > 10.0) {
      b_y = 0;
      k = 0;
    } else {
      k = 10;
    }
    trueCount = 0;
    partialTrueCount = 0;
    for (b_i = 0; b_i < 10; b_i++) {
      if (numConstraints_tmp[b_i]) {
        trueCount++;
        tmp_data[partialTrueCount] = (signed char)b_i;
        partialTrueCount++;
      }
    }
    partialTrueCount = k - b_y;
    k = y->size[0];
    y->size[0] = partialTrueCount;
    emxEnsureCapacity_real_T(y, k);
    timeOfArrival_data = y->data;
    for (b_i = 0; b_i < partialTrueCount; b_i++) {
      timeOfArrival_data[b_i] = 0.0;
    }
    for (k = 0; k < inner; k++) {
      for (b_i = 0; b_i < partialTrueCount; b_i++) {
        timeOfArrival_data[b_i] +=
            M[b_y + b_i][k] * constraints[dimIdx][tmp_data[k]];
      }
    }
    partialTrueCount = i1 - i;
    R_size[0] = partialTrueCount;
    R_size[1] = partialTrueCount;
    for (i1 = 0; i1 < partialTrueCount; i1++) {
      for (k = 0; k < partialTrueCount; k++) {
        R_data[k + partialTrueCount * i1] = -M[i2 + i1][i + k];
      }
    }
    emxReserve_real_T(y);
    timeOfArrival_data = y->data;
    b_mldivide(R_data, R_size, (double *)y->data, &(*(int(*)[1])y->size)[0]);
    for (i = 0; i < trueCount; i++) {
      constraints_data[i] = constraints[dimIdx][tmp_data[i]];
    }
    partialTrueCount = y->size[0];
    for (i = 0; i < partialTrueCount; i++) {
      constraints_data[i + trueCount] = timeOfArrival_data[i];
    }
    for (i = 0; i < 10; i++) {
      d = 0.0;
      for (i1 = 0; i1 < 10; i1++) {
        d += Q[i1][i] * constraints_data[i1];
      }
      b_Q[i] = d;
    }
    for (i = 0; i < 10; i++) {
      d = 0.0;
      for (i1 = 0; i1 < 10; i1++) {
        d += b_AInv[i1][i] * b_Q[i1];
      }
      AInv[i] = d;
    }
    for (i = 0; i < 10; i++) {
      pp[dimIdx][i][0] = AInv[9 - i];
    }
  }
  i = timeOfArrival->size[0] * timeOfArrival->size[1];
  timeOfArrival->size[0] = 1;
  timeOfArrival->size[1] = 2;
  emxEnsureCapacity_real_T(timeOfArrival, i);
  timeOfArrival_data = timeOfArrival->data;
  timeOfArrival_data[0] = 0.0;
  timeOfArrival_data[1] = 1.0;
  emxFree_real_T(&y);
  c_emxFreeStruct_robotics_core_i(&solver);
}

/*
 * File trailer for computePolyCoefAndTimeOfArrival.c
 *
 * [EOF]
 */
