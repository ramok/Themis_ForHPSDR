;;#############################################################################
;; FILE: CLAcosPU.asm
;; 
;; DESCRIPTION: CLA cosine Per Unit function
;; 
;; Group:            C2000
;; Target Family:    C28x+CLA
;;
;;#############################################################################
;; $TI Release: CLA Math Library 4.02.02.00 $
;; $Release Date: Oct 18, 2018 $
;; $Copyright: Copyright (C) 2018 Texas Instruments Incorporated -
;;             http://www.ti.com/ ALL RIGHTS RESERVED $
;;#############################################################################

    .cdecls C,LIST,"CLAmath.h"
	.include "CLAeabi.asm"

;;----------------------------------------------------------------------------
;; Description: Implement Cosine using taylor series expansion:
;;              
;;              radPU = K + X
;;
;;    rad=radPU * 2 PI
;;
;;              Cos(rad) = Cos(K) - Sin(K)*X 
;;                                - Cos(K)*X^2/2! 
;;                                + Sin(K)*X^3/3! 
;;                                + Cos(K)*X^4/4! 
;;                                - Sin(K)*X^5/5!
;;
;;                       = Cos(K) + X*(-1.0*Sin(K) 
;;                                + X*(-0.5*Cos(K) 
;;                                + X*(0.166666*Sin(K) 
;;                                + X*(0.04166666*Cos(K) 
;;                                + X*(-0.00833333*Sin(K))))))
;;
;;                       = Cos(K) + X*(-Sin(K) 
;;                                + X*(Coef0*Cos(K) 
;;                                + X*(Coef1_pos*Sin(K) 
;;                                + X*(Coef2*Cos(K) 
;;                                + X*(Coef3_neg*Sin(K))))))
;;
;;
;; Equation:    y = Cos(rad)
;;
;; Regs Used:   MR0, MR1, MR2
;;
;; Input:   rad              f32 value in memory
;;
;; // TABLE_SIZE = 128
;;    CLAsincosTable.Sin0  =  0.0;               // sin(  0 * 2*pi/TABLE_SIZE)                  
;;    CLAsincosTable.Sin1  =  0.04906767432742;  // sin(  1 * 2*pi/TABLE_SIZE) 
;;    ...
;;    CLAsincosTable.Sin31 =  0.9987954562052;   // sin( 31 * 2*pi/TABLE_SIZE) 
;;    CLAsincosTable.Cos0  =  1.0;               // sin( 32 * 2*pi/TABLE_SIZE) 
;;    CLAsincosTable.Sin33 =  0.9987954562052;   // sin( 33 * 2*pi/TABLE_SIZE) 
;;    ...
;;    CLAsincosTable.Sin63 =  0.04906767432742;  // sin( 63 * 2*pi/TABLE_SIZE) 
;;    CLAsincosTable.Sin64 =  0.0;               // sin( 64 * 2*pi/TABLE_SIZE)                  
;;    CLAsincosTable.Sin65 = -0.04906767432742;  // sin( 65 * 2*pi/TABLE_SIZE) 
;;    ... 
;;    CLAsincosTable.Sin95 = -0.9987954562052;   // sin( 95 * 2*pi/TABLE_SIZE) 
;;    CLAsincosTable.Sin96 = -1.0;               // sin( 96 * 2*pi/TABLE_SIZE) 
;;    CLAsincosTable.Sin97 = -0.9987954562052;   // sin( 97 * 2*pi/TABLE_SIZE) 
;;    ...
;;    CLAsincosTable.Sin127 = -0.04906767432742; // sin(127 * 2*pi/TABLE_SIZE) 
;;    CLAsincosTable.Cos96  =  0.0;              // sin(  0 * 2*pi/TABLE_SIZE)                  
;;    CLAsincosTable.Cos97  =  0.04906767432742; // sin(  1 * 2*pi/TABLE_SIZE) 
;;    ... 
;;    CLAsincosTable.Cos127 =  0.9987954562052;  // sin( 31 * 2*pi/TABLE_SIZE) 
;;    CLAsincosTable.Cos128 =  1.0;              // sin( 32 * 2*pi/TABLE_SIZE)
;;
;;    // TABLE_SIZE/Pi           
;;    // 2*Pi/TABLE_SIZE
;;    // TABLE_MASK = (TABLE_SIZE*2 - 1) & 0xFFFFFFFE;; 
;;
;;    CLAsincosTable.TABLE_SIZE     =  128.0;      
;;    CLAsincosTable.TwoPiDivTABLE_SIZE  =  0.04908738521234;    
;;    CLAsincosTable.TABLE_MASK          =  0x000000FE;          
;;    CLAsincosTable.Coef0               = -0.5;
;;    CLAsincosTable.Coef1               = -0.1666666666666;
;;    CLAsincosTable.Coef1_pos           =  0.1666666666666;
;;    CLAsincosTable.Coef2               =  4.1666666666666e-2;
;;    CLAsincosTable.Coef3               =  8.3333333333333e-3;
;;    CLAsincosTable.Coef3_neg           = -8.3333333333333e-3;
;;
;; Output:  y                f32 value in memory
;;          MR0 = y          f32 result
;;
;; Benchmark:   Cycles = 28
;;              Instructions = 28
;;
;; Scratchpad Usage: (Local Function Scratchpad Pointer (SP))
;;
;;   |_______|<- MR3                               (SP+0)
;;
;;---------------------------------------------------------------------------- 

    
    .def    _CLAcosPU
    .sect   "Cla1Prog:_CLAcosPU"
    .align  2
	.def    __cla_CLAcosPU_sp
__cla_CLAcosPU_sp   .usect ".scratchpad:Cla1Prog:_CLAcosPU",2,0,1

_CLAcosPU:
    .asmfunc
    .asg    __cla_CLAcosPU_sp + 0, _save_MR3
; Context Save
    MMOV32    @_save_MR3, MR3

; MR0 = radPU
; MR0=MR0*MR3
; MR1 = TABLE_SIZE*Pi/(2*Pi)
; MR1 = radPU*TABLE_SIZE*Pi/(2*Pi)
; MR2 = TABLE_MASK
    MMOV32    MR1,@_CLAsincosTable_TABLE_SIZE                         
    MMPYF32   MR1,MR0,MR1                      
 || MMOV32    MR2,@_CLAsincosTable_TABLE_MASK         

; MR3 = K = integer(radPU*TABLE_SIZE*Pi/(2*Pi))
; MR3 = K & TABLE_MASK
; MR3 = K * 2
; MAR0 = address of Sin(K)
; MR1 = frac(TABLE_SIZE*radPU*Pi/2*Pi)
; MR0 = 2*Pi/TABLE_SIZE
; MR1 = X = frac(TABLE_SIZE*radPU*Pi/2*Pi) * (2*Pi/TABLE_SIZE)
; MR0 = Coef3_neg
    MF32TOI32 MR3,MR1                          
    MAND32    MR3,MR3,MR2                      
    MLSL32    MR3,#1                           
    MMOV16    MAR0,MR3,#_CLAsincosTable_Sin0          
    MFRACF32  MR1,MR1                          
    MMOV32    MR0,@_CLAsincosTable_TwoPiDivTABLE_SIZE 
    MMPYF32   MR1,MR1,MR0                      
 || MMOV32    MR0,@_CLAsincosTable_Coef3_neg          

; MR2 = Sin(K)
; MR3 = Coef3_neg*Sin(K)
; MR2 = Cos(K)
; MR3 = X*Coef3_neg*Sin(K)
    MMOV32    MR2,*MAR0[#+64]++                
    MMPYF32   MR3,MR0,MR2                      
 || MMOV32    MR2,*MAR0[#-64]++                
    MMPYF32   MR3,MR3,MR1                      
 || MMOV32    MR0,@_CLAsincosTable_Coef2

; MR2 = Coef2*Cos(K)
; MR3 = Coef2*Cos(K) + X*Coef3_neg*Sin(K)
; MR2 = Sin(K)
; MR3 = X*Coef2*Cos(K) + X^2*Coef3_neg*Sin(K)
    MMPYF32   MR2,MR0,MR2                      
    MADDF32   MR3,MR3,MR2                      
 || MMOV32    MR2,*MAR0[#+64]++                
    MMPYF32   MR3,MR3,MR1                      
 || MMOV32    MR0,@_CLAsincosTable_Coef1_pos

; MR2 = Coef1_pos*Sin(K)
; MR3 = Coef1_pos*Sin(K) + X*Coef2*Cos(K) + X^2*Coef3_neg*Sin(K)
; MR2 = Cos(K)
; MR3 = X*Coef1_pos*Sin(K) + X^2*Coef2*Cos(K) + X^3*Coef3_neg*Sin(K)
    MMPYF32   MR2,MR0,MR2                      
    MADDF32   MR3,MR3,MR2                      
 || MMOV32    MR2,*MAR0[#-64]++                
    MMPYF32   MR3,MR3,MR1                      
 || MMOV32    MR0,@_CLAsincosTable_Coef0

; MR2 = Coef0*Cos(K)
; MR3 = Coef0*Cos(K) + X*Coef1_pos*Sin(K) + X^2*Coef2*Cos(K) 
;       + X^3*Coef3_neg*Sin(K)
;
; MR2 = Sin(K)
; MR3 = X*Coef0*Cos(K) + X^2*Coef1_pos*Sin(K) 
;       + X^3*Coef2*Cos(K) + X^4*Coef3_neg*Sin(K)
    MMPYF32   MR2,MR0,MR2                      
    MADDF32   MR3,MR3,MR2                      
 || MMOV32    MR2,*MAR0[#+64]++                
    MMPYF32   MR3,MR3,MR1                      

; MR3 = -Sin(K) + X^1*Coef0*Cos(K) + X^2*Coef1_pos*Sin(K) 
;       + X^3*Coef2*Cos(K) + X^4*Coef3_neg*Sin(K)
;
; MR3 = -X*Sin(K) + X^2*Coef0*Cos(K) + X^3*Coef1_pos*Sin(K) 
;       + X^4*Coef2*Cos(K) + X^5*Coef3_neg*Sin(K)
;
; MR2 = Cos(K)
    MSUBF32  MR3,MR3,MR2                      
    MMPYF32  MR3,MR3,MR1                      
 || MMOV32   MR2,*MAR0[#0]++                  

; Context Restore and Final Operations
    MRCNDD   UNC
; MR3 = Cos(K) - X*Sin(K) + X^2*Coef0*Cos(K) + X^3*Coef1_pos*Sin(K) 
;       + X^4*Coef2*Cos(K) + X^5*Coef3_neg*Sin(K)
; Store Y = CosPU(radPU)
    MADDF32  MR3,MR2,MR3                       
    MMOV32   MR0,MR3
    MMOV32   MR3,@_save_MR3 
    .unasg  _save_MR3
    .endasmfunc
    
;; End of File
 
