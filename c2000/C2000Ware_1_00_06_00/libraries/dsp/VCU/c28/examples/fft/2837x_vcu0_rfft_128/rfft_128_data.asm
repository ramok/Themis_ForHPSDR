;******************************************************************************
;******************************************************************************
; 
; FILE: rfft_128_data.asm
; 
; DESCRIPTION: Input test data for the FFT
; 
;******************************************************************************
;  $TI Release: C28x VCU Library V2.10.00.00 $
;  $Release Date: Oct 18, 2018 $
;  $Copyright: Copyright (C) 2018 Texas Instruments Incorporated -
;              http://www.ti.com/ ALL RIGHTS RESERVED $
;******************************************************************************
;  This software is licensed for use with Texas Instruments C28x
;  family DSCs.  This license was provided to you prior to installing
;  the software.  You may review this license by consulting a copy of
;  the agreement in the doc directory of this library.
; ------------------------------------------------------------------------
;******************************************************************************
        ;.cdecls   C,LIST,"fft.h"
;############################################################################
;
;/*! \page RFFT_128_DATA (Input test data to the FFT)
;
; The input test data is a two tone function. We run the fft on this 
; data and compare to the expected output.
;*/
;############################################################################
	    .sect .econst
        .align  128
        .global _RFFT16_128p_in_data,_RFFT16_128p_out_data
        
        ; FFT input data, two-tone test          
_RFFT16_128p_in_data: 
        .word   2232, 1930,  1165, 286,  -347, -560,  -445, -286
        .word   -373, -809,  -1440, -1930,  -1957, -1406,  -445, 560
        .word   1237, 1406,  1165, 809,  648, 809,  1165, 1406
        .word   1237, 560,  -445, -1406,  -1957, -1930,  -1440, -809
        .word   -373, -286,  -445, -560,  -347, 286,  1165, 1930
        .word   2232, 1930,  1165, 286,  -347, -560,  -445, -286
        .word   -373, -809,  -1440, -1930,  -1957, -1406,  -445, 560
        .word   1237, 1406,  1165, 809,  648, 809,  1165, 1406
        .word   1237, 560,  -445, -1406,  -1957, -1930,  -1440, -809
        .word   -373, -286,  -445, -560,  -347, 286,  1165, 1930
        .word   2232, 1930,  1165, 286,  -347, -560,  -445, -286
        .word   -373, -809,  -1440, -1930,  -1957, -1406,  -445, 560
        .word   1237, 1406,  1165, 809,  648, 809,  1165, 1406
        .word   1237, 560,  -445, -1406,  -1957, -1930,  -1440, -809
        .word   -373, -286,  -445, -560,  -347, 286,  1165, 1930
        .word   2232, 1930,  1165, 286,  -347, -560,  -445, -286
		
; FFT output data
_RFFT16_128p_out_data: 
        .word   0, 31,  11, 31,  22, 34,  39, 37
        .word   66, 45,  130, 64,  502, 183,  -360, -97
        .word   -143, -27,  -92, -11,  -70, -4,  -56, 0
        .word   -47, 2,  -40, 3,  -37, 5,  -33, 6
        .word   -29, 401,  -27, 6,  -24, 7,  -23, 7
        .word   -21, 8,  -20, 7,  -19, 8,  -17, 9
        .word   -17, 9,  -15, 8,  -15, 9,  -13, 9
        .word   -13, 9,  -12, 8,  -12, 10,  -12, 9
        .word   -11, 9,  -11, 9,  -10, 9,  -9, 9
        .word   -9, 9,  -8, 9,  -8, 9,  -7, 10
        .word   -7, 9,  -6, 9,  -6, 10,  -6, 10
        .word   -5, 9,  -6, 10,  -4, 9,  -4, 10
        .word   -4, 10,  -4, 9,  -4, 9,  -3, 10
        .word   -3, 10,  -3, 9,  -3, 9,  -2, 9
        .word   -2, 10,  -2, 10,  -1, 10,  -2, 10
        .word   -2, 9,  -1, 10,  -1, 9,  0, 10
        
