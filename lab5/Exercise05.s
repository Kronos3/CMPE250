      TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
;Secure string I/O and printing decimal numbers
;Name:  Andrei Tumbar
;Date:  9/21/2020
;Class:  CMPE-250
;Section:  5, Tuesday, 11 AM
;---------------------------------------------------------------
;Keil Simulator Template for KL46
;R. W. Melton
;January 5, 2018
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;EQUates
;Standard data masks
BYTE_MASK         EQU  0xFF
NIBBLE_MASK       EQU  0x0F
;Standard data sizes (in bits)
BYTE_BITS         EQU  8
NIBBLE_BITS       EQU  4
;Architecture data sizes (in bytes)
WORD_SIZE         EQU  4  ;Cortex-M0+
HALFWORD_SIZE     EQU  2  ;Cortex-M0+
;Architecture data masks
HALFWORD_MASK     EQU  0xFFFF
;Return                 
RET_ADDR_T_MASK   EQU  1  ;Bit 0 of ret. addr. must be
                          ;set for BX, BLX, or POP
                          ;mask in thumb mode
;---------------------------------------------------------------
;Vectors
VECTOR_TABLE_SIZE EQU 0x000000C0  ;KL46
VECTOR_SIZE       EQU 4           ;Bytes per vector
;---------------------------------------------------------------
;CPU CONTROL:  Control register
;31-2:(reserved)
;   1:SPSEL=current stack pointer select
;           0=MSP (main stack pointer) (reset value)
;           1=PSP (process stack pointer)
;   0:nPRIV=not privileged
;        0=privileged (Freescale/NXP "supervisor") (reset value)
;        1=not privileged (Freescale/NXP "user")
CONTROL_SPSEL_MASK   EQU  2
CONTROL_SPSEL_SHIFT  EQU  1
CONTROL_nPRIV_MASK   EQU  1
CONTROL_nPRIV_SHIFT  EQU  0
;---------------------------------------------------------------
;CPU PRIMASK:  Interrupt mask register
;31-1:(reserved)
;   0:PM=prioritizable interrupt mask:
;        0=all interrupts unmasked (reset value)
;          (value after CPSIE I instruction)
;        1=prioritizable interrrupts masked
;          (value after CPSID I instruction)
PRIMASK_PM_MASK   EQU  1
PRIMASK_PM_SHIFT  EQU  0
;---------------------------------------------------------------
;CPU PSR:  Program status register
;Combined APSR, EPSR, and IPSR
;----------------------------------------------------------
;CPU APSR:  Application Program Status Register
;31  :N=negative flag
;30  :Z=zero flag
;29  :C=carry flag
;28  :V=overflow flag
;27-0:(reserved)
APSR_MASK     EQU  0xF0000000
APSR_SHIFT    EQU  28
APSR_N_MASK   EQU  0x80000000
APSR_N_SHIFT  EQU  31
APSR_Z_MASK   EQU  0x40000000
APSR_Z_SHIFT  EQU  30
APSR_C_MASK   EQU  0x20000000
APSR_C_SHIFT  EQU  29
APSR_V_MASK   EQU  0x10000000
APSR_V_SHIFT  EQU  28
;----------------------------------------------------------
;CPU EPSR
;31-25:(reserved)
;   24:T=Thumb state bit
;23- 0:(reserved)
EPSR_MASK     EQU  0x01000000
EPSR_SHIFT    EQU  24
EPSR_T_MASK   EQU  0x01000000
EPSR_T_SHIFT  EQU  24
;----------------------------------------------------------
;CPU IPSR
;31-6:(reserved)
; 5-0:Exception number=number of current exception
;      0=thread mode
;      1:(reserved)
;      2=NMI
;      3=hard fault
;      4-10:(reserved)
;     11=SVCall
;     12-13:(reserved)
;     14=PendSV
;     15=SysTick
;     16=IRQ0
;     16-47:IRQ(Exception number - 16)
;     47=IRQ31
;     48-63:(reserved)
IPSR_MASK             EQU  0x0000003F
IPSR_SHIFT            EQU  0
IPSR_EXCEPTION_MASK   EQU  0x0000003F
IPSR_EXCEPTION_SHIFT  EQU  0
;----------------------------------------------------------
PSR_N_MASK           EQU  APSR_N_MASK
PSR_N_SHIFT          EQU  APSR_N_SHIFT
PSR_Z_MASK           EQU  APSR_Z_MASK
PSR_Z_SHIFT          EQU  APSR_Z_SHIFT
PSR_C_MASK           EQU  APSR_C_MASK
PSR_C_SHIFT          EQU  APSR_C_SHIFT
PSR_V_MASK           EQU  APSR_V_MASK
PSR_V_SHIFT          EQU  APSR_V_SHIFT
PSR_T_MASK           EQU  EPSR_T_MASK
PSR_T_SHIFT          EQU  EPSR_T_SHIFT
PSR_EXCEPTION_MASK   EQU  IPSR_EXCEPTION_MASK
PSR_EXCEPTION_SHIFT  EQU  IPSR_EXCEPTION_SHIFT
;----------------------------------------------------------
;Stack
SSTACK_SIZE EQU  0x00000100
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  GetChar
            IMPORT  InitIO
            IMPORT  LengthStringSB
            IMPORT  PutChar
            IMPORT  TestIO
Reset_Handler  PROC {}
main
;---------------------------------------------------------------
;Initialize registers R0-R12
MAX_STRING  EQU  79
CR          EQU  13
LF          EQU  10
;>>>>> begin main program code <<<<<
            BL   InitIO

            MOVS R4,R0      ; Number of iterations we need to do
main_iter   CMP  R4,#0      ; Iterations left
            BEQ  finish
            SUBS R4,R4,#1   ; i--;
            
            ; Print the prompt string
            LDR  R0,=prompt
            LDR  R1,=promptend
            SUBS R1,R1,R0   ; Get the size of R0
            BL   PutStringSB
            
            ; Get input line
            LDR  R0,=buffer
            MOVS R1,#MAX_STRING
            BL   GetStringSB
            
            ; Print the '<' character
            LDR  R0,=promptend
            MOVS R1,#2
            BL   PutStringSB
            
            ; Print the string we got
            LDR  R0,=buffer
            MOVS R1,#MAX_STRING
            BL   PutStringSB
            
            ; Get the length we just printed
            BL   LengthStringSB
            MOVS R2,R0  ; temporarily store it here
            
            ; Print the '<' character
            LDR  R0,=promptend
            MOVS R1,#2
            BL   PutStringSB
            
            ; Print a new line
            LDR  R0,=newline
            MOVS R1,#3
            BL   PutStringSB
            
            ; Print "Length:"
            LDR  R0,=length
            LDR  R1,=len_end
            SUBS R1,R1,R0
            BL   PutStringSB
            
            ; Print the number
            MOVS R0,R2  ; Print the length of the last getstring
            BL   PutNumU
            
            ; Print a new line
            LDR  R0,=newline
            MOVS R1,#3
            BL   PutStringSB
            
            B    main_iter
            
finish      MOVS R0,#1  ; Extra credit
            BL   TestIO
;>>>>> end main program code <<<<<
            B    .
            ENDP
;---------------------------------------------------------------

; Recieve a string from the command line terminated by CR. 
; This should null terminate the string and shouldn't 
; overflow over the maxmimum size specified in R1.
; SUBROUTINES USED
;   - GetChar (recieve input)
;   - PutChar (Print each character as we get them)
; PARAMS
;   INPUT R0: pointer to the destination buffer (word address)
;   INPUT R1: size of the destination buffer    (unsigned word value)
;   OUTPUT  : String buffer stored in memory at R0
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
                    CMP   R0,#CR    ; while (c != '\r')
                    BEQ   getdone   ; break
                    CMP   R2,R1
                    BGE   getwhile  ; if (i >= r1 - 1) continue
                    STRB  R0,[R3,R2]    ; r3[i] = r0
                    BL    PutChar       ; printf("%c", r0);
                    ADDS  R2,R2,#1  ; i++
                    B     getwhile
getdone             MOVS  R0,#0 ; Add null terminator
                    STRB  R0,[R3,R2]

                    ; print new line (Windows CRLF)
                    MOVS  R0,#CR
                    BL    PutChar
                    MOVS  R0,#LF
                    BL    PutChar
                    POP   {PC,R0-R3}
                    ENDP

; Print a NULL terminated string
; SUBROUTINES USED
;   - PutChar (Print each character)
; PARAMS
;   INPUT R0: pointer to the target string (word address)
;   INPUT R1: size of the string buffer so that to not overflow (unsigned word value)
;   OUTPUT  : NONE
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
                    
putwhile            LDRB R3,[R2,#0]  ; r3 = *iter
                    CMP  R3,#0       ; if (!*r3) break
                    BEQ  putdone
                    SUBS R3,R2,R4
                    CMP  R3,R1       ; if (iter - start >= r1 - 1) break
                    BGE  putdone
                    LDRB R0,[R2,#0]  ; r0 = *iter
                    BL   PutChar     ; printf("%c", *iter);
                    ADDS R2,R2,#1    ; iter++
                    B    putwhile
putdone             POP {R0-R4,PC}
                    ENDP

; Print a number in decimal using the ascii
; characters
; SUBROUTINES USED
;   - PutChar (Print each digit)
;   - DIVU
; PARAMS
;   INPUT R0: Number to print (unsigned word value)
;   OUTPUT  : NONE
PutNumU             PROC {R0-R2}
                    PUSH {R0-R2,LR}
printbase           EQU  10
asciioffset         EQU  '0'
                    ; put_num_u(U32 r0)
                    ; while (r0 != 0)
                    ;   (add r0 % 10 to stack)
                    ;   r0 /= 10
                    ; print the stack backwards
                    CMP R0,#0  ; Just print a '0'
                    BEQ numzero
                    
                    MOVS  R2,#0             ; Count the number of digits
numwhile            CMP   R0,#0             ; while (r0 != 0)
                    BEQ   numwhile2
                    MOVS  R1,#printbase     ; Always divide by the base
                    BL    DIVU              ; R0 = R0 / base; R0 % base
                    PUSH  {R1}              ; Need to print these digits backwards
                    ADDS  R2,R2,#1          ; r2++
                    B     numwhile
numwhile2           CMP   R2,#1             ; while (r2 >= 1)
                    BLT   numdone
                    POP   {R1}
                    MOVS  R0,R1
                    ADDS  R0,R0,#asciioffset ; Print the next character (ascii not just value)
                    SUBS  R2,R2,#1
                    BL    PutChar
                    B     numwhile2
numzero             MOVS  R0,#'0'
                    BL    PutChar
numdone             POP   {R0-R2,PC}
                    ENDP

; Calculate the quotient and remainder of
; two unsigned word values.
; SUBROUTINES: None
; PARAMS:
;   INPUT R0: dividend (unsigned word)
;   INPUT R1: divisor  (unsigned word)
;   OUTPUT R0: quotient (unsigned word)
;   OUTPUT R1: remainder (unsigned word)
DIVU        PROC {R0-R7}
            PUSH {LR}
            PUSH {R2-R7}
            CMP  R1,#0
            BEQ  DIVU_0     ; Don't try to divide by zero
LEFT_MASK   EQU  0x80 ; Used to get most significant bit of a byte
SHIFT32     EQU  31
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
            MOVS R4,#SHIFT32 ; Init the iterator (i)
            MOVS R6,#1       ; Used for Q = Q | (1 << i)
DIV_FOR     CMP  R4,#0       ; if (i >= 0)
            BLT  DIVU_FINISH ; Finished loop
            LSLS R2,R2,#1    ; R = R << 1
            LSRS R5,R0,#SHIFT32   ; R5 = most significant bit in N
            MOVS R6,#1
            ANDS R5,R5,R6    ; R5 = (N & LEFT_MASK) >> 31
            ORRS R2,R2,R5    ; R |= (N & LEFT_MASK) >> 31
            LSLS R0,R0,#1    ; N = N << 1
            CMP  R2,R1       ; if (R >= D)
            BLO  DIV_ITER    ; continue;
            SUBS R2,R2,R1    ; R = R - D
            LSLS R6,R6,R4    ; R6 = 1 << i
            MOVS R5,R6
            ORRS R3,R3,R5    ; Q = Q | (1 << i)
DIV_ITER    SUBS R4,R4,#1    ; i--
            B    DIV_FOR     ; For loop
DIVU_FINISH MOVS R0,R3       ; Set the outputs
            MOVS R1,R2       ;
            ; Clear the carry flag
            MRS  R2,APSR
            MOVS R3,#0x20
            LSLS R3,R3,#24
            BICS R2,R2,R3
            MSR  APSR,R2
            B    DIVU_STOP
DIVU_0      MOVS R3,#1       ; Init R3 as 1
            ; Set the carry flag
            MRS  R2,APSR
            MOVS R3,#0x20
            LSLS R3,R3,#24
            ORRS R2,R2,R3
            MSR  APSR,R2
DIVU_STOP   POP  {R2-R7}
            POP  {PC}
            ENDP

;****************************************************************
;Initializes register n to value 0xnnnnnnnn, for n in 
;{0x0-0xC,0xE}
;****************************************************************
;Put return on stack
RegInit     PROC {}
            PUSH    {LR}
;Initialize registers
            LDR     R1,=0x11111111
            ADDS    R2,R1,R1
            ADDS    R3,R2,R1
            ADDS    R4,R3,R1
            ADDS    R5,R4,R1
            ADDS    R6,R5,R1
            ADDS    R7,R6,R1
            ADDS    R0,R7,R1
            MOV     R8,R0
            ADDS    R0,R0,R1
            MOV     R9,R0
            ADDS    R0,R0,R1
            MOV     R10,R0
            ADDS    R0,R0,R1
            MOV     R11,R0
            ADDS    R0,R0,R1
            MOV     R12,R0
            ADDS    R0,R0,R1
            ADDS    R0,R0,R1
            MOV     R14,R0
            MOVS    R0,#0
            POP     {PC}
            ENDP
;---------------------------------------------------------------
;>>>>> begin subroutine code <<<<<
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;reset vector
            SPACE  (VECTOR_TABLE_SIZE - (2 * VECTOR_SIZE))
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
prompt      DCB     "Enter a string:\r\n>\0"
promptend   DCB     "<\0"
newline     DCB     "\r\n\0"
length      DCB     "Length:\0"
len_end
;>>>>>   end constants here <<<<<
;****************************************************************
            AREA    |.ARM.__at_0x1FFFE000|,DATA,READWRITE,ALIGN=3
            EXPORT  __initial_sp
;Allocate system stack
            IF      :LNOT::DEF:SSTACK_SIZE
SSTACK_SIZE EQU     0x00000100
            ENDIF
Stack_Mem   SPACE   SSTACK_SIZE
__initial_sp
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
buffer      SPACE   MAX_STRING
;>>>>>   end variables here <<<<<
            END