/*
 * RigidBodyTreeUtils.c
 *
 * Code generation for function 'RigidBodyTreeUtils'
 *
 */

/* Include files */
#include "RigidBodyTreeUtils.h"
#include "allOrAny.h"
#include "mod.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"
#include "blas.h"
#include "mwmathutil.h"
#include <emmintrin.h>
#include <stddef.h>
#include <string.h>

/* Variable Definitions */
static emlrtRSInfo ds_emlrtRSI = {
    113,                           /* lineNo */
    "RigidBodyTreeUtils/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeUtils.m" /* pathName */
};

static emlrtRSInfo es_emlrtRSI = {
    120,                           /* lineNo */
    "RigidBodyTreeUtils/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeUtils.m" /* pathName */
};

static emlrtRSInfo fs_emlrtRSI = {
    121,                           /* lineNo */
    "RigidBodyTreeUtils/distance", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeUtils.m" /* pathName */
};

static emlrtECInfo bb_emlrtECI = {
    -1,                            /* nDims */
    119,                           /* lineNo */
    13,                            /* colNo */
    "RigidBodyTreeUtils/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeUtils.m" /* pName */
};

static emlrtRTEInfo nb_emlrtRTEI = {
    36,        /* lineNo */
    23,        /* colNo */
    "vecnorm", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\matfun\\vecnorm.m" /* pName
                                                                           */
};

static emlrtBCInfo fc_emlrtBCI = {
    -1,                            /* iFirst */
    -1,                            /* iLast */
    119,                           /* lineNo */
    27,                            /* colNo */
    "",                            /* aName */
    "RigidBodyTreeUtils/distance", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\robotics\\robotmanip\\+robotics\\+manip\\+"
    "internal\\RigidBodyTreeUtils.m", /* pName */
    0                                 /* checkKind */
};

/* Function Definitions */
int32_T RigidBodyTreeUtils_distance(const emlrtStack *sp, real_T config1,
                                    const real_T config2_data[],
                                    const int32_T config2_size[2],
                                    real_T dist_data[])
{
  ptrdiff_t incx_t;
  ptrdiff_t n_t;
  emlrtStack b_st;
  emlrtStack st;
  real_T configDiff_data[192];
  real_T thetaWrap_data[192];
  real_T y_data[192];
  real_T b_y_data[32];
  int32_T outsize[2];
  int32_T thetaWrap_size[2];
  int32_T b_loop_ub;
  int32_T dist_size;
  int32_T i;
  int32_T i1;
  int32_T loop_ub;
  int32_T scalarLB;
  int32_T vectorUB;
  int8_T b_iv[6];
  boolean_T x_data[192];
  boolean_T y;
  st.prev = sp;
  st.tls = sp->tls;
  b_st.prev = &st;
  b_st.tls = st.tls;
  if (config2_size[0] < 1) {
    dist_size = 0;
    loop_ub = config2_size[1];
  } else if (config2_size[0] > 1) {
    st.site = &ds_emlrtRSI;
    b_loop_ub = config2_size[0];
    dist_size = config2_size[0];
    loop_ub = config2_size[1];
    for (i = 0; i < loop_ub; i++) {
      scalarLB = (config2_size[0] / 2) << 1;
      vectorUB = scalarLB - 2;
      for (i1 = 0; i1 <= vectorUB; i1 += 2) {
        _mm_storeu_pd(
            &configDiff_data[i1 + dist_size * i],
            _mm_sub_pd(_mm_loadu_pd(&config2_data[i1 + config2_size[0] * i]),
                       _mm_set1_pd(config1)));
      }
      for (i1 = scalarLB; i1 < b_loop_ub; i1++) {
        configDiff_data[i1 + dist_size * i] =
            config2_data[i1 + config2_size[0] * i] - config1;
      }
    }
  } else {
    dist_size = 1;
    b_loop_ub = config2_size[1];
    loop_ub = config2_size[1];
    scalarLB = (config2_size[1] / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      _mm_storeu_pd(
          &configDiff_data[i],
          _mm_sub_pd(_mm_loadu_pd(&config2_data[i]), _mm_set1_pd(config1)));
    }
    for (i = scalarLB; i < b_loop_ub; i++) {
      configDiff_data[i] = config2_data[i] - config1;
    }
  }
  for (i = 0; i < 6; i++) {
    if (i + 1 > loop_ub) {
      emlrtDynamicBoundsCheckR2012b(i + 1, 1, loop_ub, &fc_emlrtBCI,
                                    (emlrtConstCTX)sp);
    }
    b_iv[i] = (int8_T)i;
  }
  st.site = &es_emlrtRSI;
  thetaWrap_size[0] = dist_size;
  thetaWrap_size[1] = 6;
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < dist_size; i1++) {
      thetaWrap_data[i1 + dist_size * i] =
          configDiff_data[i1 + dist_size * b_iv[i]];
    }
  }
  b_loop_ub = dist_size * 6;
  for (scalarLB = 0; scalarLB < b_loop_ub; scalarLB++) {
    y_data[scalarLB] = muDoubleScalarAbs(
        configDiff_data[scalarLB % dist_size +
                        dist_size * b_iv[(int32_T)((uint32_T)scalarLB /
                                                   (uint32_T)dist_size)]]);
  }
  for (i = 0; i < b_loop_ub; i++) {
    x_data[i] = (y_data[i] > 3.1415926535897931);
  }
  y = allOrAny_anonFcn1(x_data, b_loop_ub);
  if (y) {
    __m128d r;
    b_st.site = &dq_emlrtRSI;
    scalarLB = (dist_size / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i < 6; i++) {
      for (i1 = 0; i1 <= vectorUB; i1 += 2) {
        r = _mm_loadu_pd(&configDiff_data[i1 + dist_size * b_iv[i]]);
        _mm_storeu_pd(&y_data[i1 + dist_size * i],
                      _mm_add_pd(r, _mm_set1_pd(3.1415926535897931)));
      }
      for (i1 = scalarLB; i1 < dist_size; i1++) {
        y_data[i1 + dist_size * i] =
            configDiff_data[i1 + dist_size * b_iv[i]] + 3.1415926535897931;
      }
    }
    scalarLB = dist_size * 6;
    for (i = 0; i < scalarLB; i++) {
      thetaWrap_data[i] = b_mod(y_data[i]);
    }
    for (vectorUB = 0; vectorUB < scalarLB; vectorUB++) {
      i = dist_size * 6;
      if ((thetaWrap_data[vectorUB] == 0.0) && (y_data[vectorUB] > 0.0)) {
        if (vectorUB > i - 1) {
          emlrtDynamicBoundsCheckR2012b(vectorUB, 0, i - 1, &gc_emlrtBCI,
                                        &b_st);
        }
        thetaWrap_data[vectorUB] = 6.2831853071795862;
      }
    }
    b_loop_ub = dist_size * 6;
    thetaWrap_size[1] = 6;
    scalarLB = (b_loop_ub / 2) << 1;
    vectorUB = scalarLB - 2;
    for (i = 0; i <= vectorUB; i += 2) {
      r = _mm_loadu_pd(&thetaWrap_data[i]);
      _mm_storeu_pd(&thetaWrap_data[i],
                    _mm_sub_pd(r, _mm_set1_pd(3.1415926535897931)));
    }
    for (i = scalarLB; i < b_loop_ub; i++) {
      thetaWrap_data[i] -= 3.1415926535897931;
    }
  }
  outsize[0] = dist_size;
  outsize[1] = 6;
  emlrtSubAssignSizeCheckR2012b(&outsize[0], 2, &thetaWrap_size[0], 2,
                                &bb_emlrtECI, (emlrtCTX)sp);
  for (i = 0; i < 6; i++) {
    for (i1 = 0; i1 < dist_size; i1++) {
      configDiff_data[i1 + dist_size * b_iv[i]] =
          thetaWrap_data[i1 + thetaWrap_size[0] * i];
    }
  }
  st.site = &fs_emlrtRSI;
  for (i = 0; i < dist_size; i++) {
    for (i1 = 0; i1 < loop_ub; i1++) {
      thetaWrap_data[i1 + loop_ub * i] = configDiff_data[i + dist_size * i1];
    }
  }
  if (((loop_ub != 1) || (dist_size != 1)) && (loop_ub == 1)) {
    emlrtErrorWithMessageIdR2018a(&st, &nb_emlrtRTEI,
                                  "Coder:toolbox:autoDimIncompatibility",
                                  "Coder:toolbox:autoDimIncompatibility", 0);
  }
  for (scalarLB = 0; scalarLB < dist_size; scalarLB++) {
    if (loop_ub < 1) {
      b_y_data[scalarLB] = 0.0;
    } else {
      n_t = (ptrdiff_t)loop_ub;
      incx_t = (ptrdiff_t)1;
      b_y_data[scalarLB] =
          dnrm2(&n_t, &thetaWrap_data[scalarLB * loop_ub], &incx_t);
    }
  }
  if (dist_size - 1 >= 0) {
    memcpy(&dist_data[0], &b_y_data[0], (uint32_T)dist_size * sizeof(real_T));
  }
  return dist_size;
}

/* End of code generation (RigidBodyTreeUtils.c) */
