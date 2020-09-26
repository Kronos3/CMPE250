DIVU        PROC {R2-R6}
            PUSH {R2-R6}
            CMP  R1,#0
            BEQ  DIVU_0     ; Don't try to divide by zero

LEFT_MASK   EQU  0x80000000 ; Used to align the bits to the left
            ; Compute N / D
            ; Use binary long division
            ; R = 0 (Use R2)
            ; Q = 0 (Use R3)
            ; for (i = 31; i >= 0; i--) {
            ;   R = R << 1
            ;   R |= (N & LEFT_MASK) >> 31
            ;   N = N << 1
            ;   if R >= D {
            ;       R = R - D
            ;       Q |= 1 << i
            ;   }
            ; }
            MOVS R2,#0       ; Init Remainder
            MOVS R3,#0       ; Init the Quotient
            MOVS R4,#31      ; Init the iterator (i)
            MOVS R6,#1       ; Used for Q = Q | (1 << i)
DIV_FOR     CMP  R4,#0       ; if (i >= 0)
            BLT  DIVU_FINISH ; Finished loop
            LSLS R2,R2,#1    ; R = R << 1
            ANDS R5,R0,#LEFT_MASK    ; N & LEFT_MASK
            LSRS R5,R5,#31   ; (N & LEFT_MASK) >> 31
            ORRS R2,R2,R5    ; R |= (N & LEFT_MASK) >> 31
            LSLS R0,R0,#1    ; N = N << 1
            CMP  R2,R1       ; if (R >= D)
            BLO  DIV_ITER    ; continue;
            SUBS R2,R2,R1    ; R = R - D
            LSLS R5,R6,R4    ; R5 = 1 << i
            ORRS R3,R3,R5    ; Q = Q | (1 << i)
DIV_ITER    SUBS R4,R4,#1    ; i--
            BL   DIV_FOR     ; For loop
DIVU_FINISH MOVS R0,R3       ; Set the outputs
            MOVS R1,R2       ; 
            BL   DIVU_STOP
DIVU_0      MOVS R0,#0
            MOVS R1,#0
DIVU_STOP   POP {R2-R6}
            BX LR
            ENDP