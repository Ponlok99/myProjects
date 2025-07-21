/*
 * repmat.c
 *
 * Code generation for function 'repmat'
 *
 */

/* Include files */
#include "repmat.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"

/* Variable Definitions */
static emlrtMCInfo c_emlrtMCI = {
    53,       /* lineNo */
    5,        /* colNo */
    "repmat", /* fName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pName
                                                                         */
};

static emlrtRSInfo sv_emlrtRSI = {
    53,       /* lineNo */
    "repmat", /* fcnName */
    "C:\\Program "
    "Files\\MATLAB\\R2024b\\toolbox\\eml\\lib\\matlab\\elmat\\repmat.m" /* pathName
                                                                         */
};

/* Function Declarations */
static void b_error(const emlrtStack *sp, const mxArray *m,
                    emlrtMCInfo *location);

/* Function Definitions */
static void b_error(const emlrtStack *sp, const mxArray *m,
                    emlrtMCInfo *location)
{
  const mxArray *pArray;
  pArray = m;
  emlrtCallMATLABR2012b((emlrtConstCTX)sp, 0, NULL, 1, &pArray, "error", true,
                        location);
}

void repmat(const emlrtStack *sp, const real_T a_data[],
            const int32_T a_size[2], real_T b_data[], int32_T b_size[2])
{
  static const int32_T b_iv[2] = {1, 15};
  static const char_T u[15] = {'M', 'A', 'T', 'L', 'A', 'B', ':', 'p',
                               'm', 'a', 'x', 's', 'i', 'z', 'e'};
  emlrtStack st;
  const mxArray *m;
  const mxArray *y;
  int32_T itilerow;
  int32_T jcol;
  int32_T k;
  int32_T nrows;
  int32_T outsize_idx_0_tmp;
  st.prev = sp;
  st.tls = sp->tls;
  outsize_idx_0_tmp = a_size[0] << 1;
  if ((int8_T)outsize_idx_0_tmp != outsize_idx_0_tmp) {
    y = NULL;
    m = emlrtCreateCharArray(2, &b_iv[0]);
    emlrtInitCharArrayR2013a((emlrtConstCTX)sp, 15, m, &u[0]);
    emlrtAssign(&y, m);
    st.site = &sv_emlrtRSI;
    b_error(&st, y, &c_emlrtMCI);
  }
  b_size[0] = (int8_T)outsize_idx_0_tmp;
  b_size[1] = 3;
  nrows = a_size[0];
  for (jcol = 0; jcol < 3; jcol++) {
    int32_T iacol;
    int32_T ibmat;
    iacol = jcol * nrows;
    ibmat = jcol * outsize_idx_0_tmp - 1;
    for (itilerow = 0; itilerow < 2; itilerow++) {
      int32_T ibcol;
      ibcol = ibmat + itilerow * nrows;
      for (k = 0; k < nrows; k++) {
        b_data[(ibcol + k) + 1] = a_data[iacol + k];
      }
    }
  }
}

/* End of code generation (repmat.c) */
