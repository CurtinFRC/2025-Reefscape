#pragma once

#include <iostream>
#include "admm.hpp"

#ifdef _WIN32
#ifdef BUILD_DLL
#define TINY_API __declspec(dllexport)
#else
#define TINY_API __declspec(dllimport)
#endif
#else
#define TINY_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

TINY_API int tiny_setup(TinySolver** solverp,
                tinyMatrix Adyn, tinyMatrix Bdyn, tinyMatrix Q, tinyMatrix R, 
                tinytype rho, int nx, int nu, int N,
                tinyMatrix x_min, tinyMatrix x_max, tinyMatrix u_min, tinyMatrix u_max,
                int verbose);
TINY_API int tiny_precompute_and_set_cache(TinyCache *cache, 
                                    tinyMatrix Adyn, tinyMatrix Bdyn, tinyMatrix Q, tinyMatrix R,
                                    int nx, int nu, tinytype rho, int verbose);
TINY_API int tiny_solve(TinySolver *solver);

TINY_API int tiny_update_settings(TinySettings* settings,
                            tinytype abs_pri_tol, tinytype abs_dua_tol, 
                            int max_iter, int check_termination, 
                            int en_state_bound, int en_input_bound);
TINY_API int tiny_set_default_settings(TinySettings* settings);

TINY_API int tiny_set_x0(TinySolver* solver, tinyVector x0);
TINY_API int tiny_set_x_ref(TinySolver* solver, tinyMatrix x_ref);
TINY_API int tiny_set_u_ref(TinySolver* solver, tinyMatrix u_ref);


#ifdef __cplusplus
}
#endif