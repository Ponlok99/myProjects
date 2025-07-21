/*
 * strcmp.c
 *
 * Code generation for function 'strcmp'
 *
 */

/* Include files */
#include "strcmp.h"
#include "rt_nonfinite.h"
#include "run_p2p_data.h"

/* Function Definitions */
boolean_T b_strcmp(const char_T a_data[], const int32_T a_size[2])
{
  static const char_T b_cv[5] = {'f', 'i', 'x', 'e', 'd'};
  boolean_T b_bool;
  b_bool = false;
  if (a_size[1] == 5) {
    int32_T kstr;
    kstr = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (kstr < 5) {
        if (a_data[kstr] != b_cv[kstr]) {
          exitg1 = 1;
        } else {
          kstr++;
        }
      } else {
        b_bool = true;
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return b_bool;
}

/* End of code generation (strcmp.c) */
