/**
 * @file app.hpp
 * @brief Application header file for C++ code
 */

#ifndef APP_HPP_
#define APP_HPP_

#ifdef __cplusplus
extern "C" {
#endif

/* C-compatible function declarations (callable from main.c) */
void App_Init(void);
void App_Main(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
/* C++ only declarations go here */

namespace App {
    void init();
    void run();
}

#endif /* __cplusplus */

#endif /* APP_HPP_ */
