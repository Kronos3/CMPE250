; Question 1
; if (R0 <= 0)
;    R0 = 0;
; else
;    R0 = R0 + R1
        CMP     R0,#0       ; Flags <- (R0 - 0)
        BGT     R0B1        ; if (R0 > 0) goto R0B1
        MOVS    R0,#0       ; R0 = 0
        BL      END         ; Skip over other branch instruction
R0B1    ADDS    R0,R0,R1    ; R0 = R0 + R1
END1    B       .

; Question 2
; do {
;    R0 = R0 + 1
; } while (R0 < MaxCount)
        ; Load value of maxcount into R1
        LDR     R1,=MaxCount
        LDR     R1,[R1,#0]
LOOP    ADDS    R0,R0,#1 ; R0 = R0 + 1
        CMP     R0,R1
        BLO     LOOP    ; Repeat the statement until R0 >= Maxcount
                        ; Uses unsigned comparison

; Question 3
; Time = 20
; while (Time < 150)
;    Time = Time + 1
        EQU     time_start,#20
        EQU     time_max,#150

        LDR     R0,=Time        ; R0 = &Time (Keep address to write to later)
        MOVS    R1,time_start   ; Init R1
        STR     R1,[R0,#0]      ; *R0 = R1 same as Time = 20
WHILE   CMP     R1,time_max
        BGT     END2            ; If Time >= 150 goto end
        ADDS    R1,R1,#1
        STR     R1,[R0,#0]      ; Time = Time + 1 (Write to time)
        BL      WHILE
END2    B       .

; Question 4
; N,R0 are unsigned
; for (R1 = 0; R1 < N; R1++)
;    R0 = (R0 >> 1) XOR R1
        LDR     R2,=N
        LDR     R2,[R2,#0]  ; Get value of N into R2
        MOVS    R1,#0       ; For loop first statement
FOR     CMP     R1,R2
        BHS     END3        ; Exit loop if statement is false
        LSRS    R0,R0,#1    ; R0 = R0 >> 1
        EORS    R0,R0,R1    ; R0 = R0 XOR R1
        ADDS    R1,R1,#1    ; R1++
        BL      FOR         ; Loop back and compare again
END3    B       .
