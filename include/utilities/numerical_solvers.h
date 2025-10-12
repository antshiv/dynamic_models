/* Numerical integration utilities */
#ifndef DYNAMIC_MODELS_UTILITIES_NUMERICAL_SOLVERS_H
#define DYNAMIC_MODELS_UTILITIES_NUMERICAL_SOLVERS_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*dm_derivative_fn)(const void* state,
                                 const void* inputs,
                                 void* state_dot,
                                 void* user_context);

void dm_integrate_euler(void* state,
                        const void* inputs,
                        double dt,
                        size_t state_bytes,
                        dm_derivative_fn derivative,
                        void* user_context,
                        void* scratch);

void dm_integrate_rk4(void* state,
                      const void* inputs,
                      double dt,
                      size_t state_bytes,
                      dm_derivative_fn derivative,
                      void* user_context,
                      void* scratch_k1,
                      void* scratch_k2,
                      void* scratch_k3,
                      void* scratch_k4,
                      void* scratch_temp);

#ifdef __cplusplus
}
#endif

#endif /* DYNAMIC_MODELS_UTILITIES_NUMERICAL_SOLVERS_H */
