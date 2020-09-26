GetStringSB         PROC {R0-R3}
                    PUSH {R0-R3,LR}
                    ; get_string_sb(char* str, size_t r1) {
                    ;   char* ptr = str
                    ;   while ((c = getchar()) != '\r')
                    ;       if (ptr - str < r1 - 1)
                    ;           *ptr = getchar();
                    ;           printf("%c", *ptr)
                    ;           ptr++
                    ;   *ptr = '\0'
                    ;   printf("\r\n")
                    MOVS  R3,R0
                    MOVS  R2,#0     ; i = 0 (use a counter instead of incrementing R3)
                    SUBS  R1,R1,#1
getwhile            BL    GetChar   ; c = getchar()
                    CMP   R0,'\r'   ; while (c != '\r')
                    BEQ   getdone   ; break
                    CMP   R2,R1
                    BGE   getwhile  ; if (i >= r1 - 1) continue
                    STR   R0,[R3,R2]    ; r3[i] = r0
                    BL    PutChar       ; printf("%c", r0);
                    ADDS  R2,R2,#1  ; i++
                    B     getwhile
getdone             MOVS  R0,#0 ; Add null terminator
                    STR   R0,[R3,R2]

                    ; print new line (Windows CRLF)
                    MOVS  R0,'\r'
                    BL    PutChar
                    MOVS  R0,'\n'
                    BL    PutChar
                    POP   {PC,R0-R3}
                    ENDP

PutStringSB         PROC {R0-R4}
                    PUSH {R0-R4,LR}
                    ; put_string_sb(char* r0, size_t r1)
                    ;   iter = r0
                    ;   while (*iter && iter - r0 < r1 - 1)
                    ;       printf("%c", *iter)
                    ;       iter++
                    MOVS R2,R0      ; iter = r0
                    MOVS R4,R0      ; start = r0 (r0 needs to be used for printing)
                    SUBS R1,R1,#1
                    
putwhile            LDR  R3,[R2,#0]  ; r3 = *iter
                    CMP  R3,#0       ; if (!*r3) break
                    BEQ  putdone
                    SUBS R3,R2,R0
                    CMP  R3,R1       ; if (iter - r0 >= r1 - 1) break
                    BGE  putdone
                    LDR  R0,[R2,#0]  ; r0 = *iter
                    BL   PutChar     ; printf("%c", *iter);
                    ADDS R2,R2,#1    ; iter++
                    B    putwhile
putdone             POP {R0-R4,PC}
                    ENDP

PutNumU             PROC {R0-R2}
                    PUSH {R0-R2,LR}
printbase           EQU  10
asciioffset         EQU  '0'
                    ; put_num_u(U32 r0)
                    ; while (r0 != 0)
                    ;   (add r0 % 10 to stack)
                    ;   r0 /= 10
                    ; print the stack backwards
                    MOVS  R2,#0             ; Count the number of digits
numwhile            CMP   R0,#0             ; while (r0 != 0)
                    BEQ   numwhile2
                    MOVS  R1,#printbase     ; Always divide by the base
                    BL    DIVU              ; R0 = R0 / base; R0 % base
                    PUSH  {R1}              ; Need to print these digits backwards
                    ADDS  R2,R2,#1          ; r2++
                    B     numwhile
numwhile2           CMP   R2,#0             ; while (r2 >= 0)
                    BGT   numdone
                    POP   {R1}
                    ADDS  R0,R1,asciioffset ; Print the next character (ascii not just value)
                    BL    PutChar
                    B     numwhile2
numdone             POP   {R0-R2,PC}
                    ENDP