; Question 1-A
; R0 = 0
; for (R1 = 0; R1 < 100; R1++)
;   R0 = R0 + Array[R1]
; In C:
; int out = 0;
; char* array = SOMEPOINTER;
; for (int i = 0; i < 100; i++)
;   out += array[i];
ArrayLength EQU  100
            MOVS R0,#0      ; Init R0
            MOVS R1,#0      ; Init iterator
            LDR  R2,=Array  ; Keep this address for memory access
Bcond1      CMP  R1,#ArrayLength
            BGE  Q1AStop
            LDRB R3,[R2,R1] ; This is a byte array so load a single byte
            ADDS R0,R0,R3   ; R0 += array[R1]
            ADDS R1,R1,#1   ; R1++
            B    Bcond1
Q1AStop     B .

; Question 1-B
ArrayLength EQU 100
            MOVS R0,#0
            LDR  R1,=Array      ; Init the offset register
            MOVS R2,R1
            ADDS R2,R2,#ArrayLength ; Hold the end pointer
Bcond2      CMP  R1,R2
            BGE  Q1BStop
            LDRB R3,[R1,#0]     ; R3 = *R1
            ADDS R0,R0,R3       ; R0 += R3
            ADDs R1,R1,#1       ; R1++
            B    Bcond2
Q1BStop     B    .


; Question 2-A
; R0 Total(int* R1, size_t R2)
Total       PROC {R1-R3}
            PUSH {R1-R3}
            MOVS R0,#0 ; Init the output variable
tcond1      CMP  R2,#0
            BLE  tfinish1       ; Down counter condition
            LDR  R3,[R1,#0]     ; R3 = *R1
            ADDS R0,R0,R3       ; R0 += *R1
            ADDS R1,R1,#4       ; R1++ (word array offset)
            SUBS R2,R2,#1       ; R2-- decrement down counter
            B    tcond1
tfinish1    POP  {R1-R3}
            BX   LR             ; No need to push/pop LR/PC (no embedded subroutine)
            ENDP

; Question 2-B
Total       PROC {R1-R3}
            PUSH {R1-R3}
            MOVS R0,#0 ; Init the output variable
tcond2      CMP  R2,#0
            BLE  tfinish2       ; Down counter condition
            LDM  R1!,{R3}       ; R3 = *R1
                                ; R1 += 4
            ADDS R0,R0,R3       ; R0 += *R1
            SUBS R2,R2,#1       ; R2-- decrement down counter
            B    tcond2
tfinish2    POP  {R1-R3}
            BX   LR             ; No need to push/pop LR/PC (no embedded subroutine)
            ENDP
