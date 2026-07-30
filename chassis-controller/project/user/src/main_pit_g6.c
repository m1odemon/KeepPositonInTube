/*
 * Build wrapper for the existing application entry point.
 *
 * The extension-board servo outputs B4/B5 use TIMA1. The original main.c
 * assigned the chassis 5 ms PIT to TIMA1, so this translation unit maps only
 * that PIT and its interrupt priority to the unused TIMG6 peripheral.
 */
#include "zf_common_headfile.h"

#define PIT_TIM_A1     PIT_TIM_G6
#define TIMA1_INT_IRQn TIMG6_INT_IRQn
#include "main.c"
