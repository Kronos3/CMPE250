            TTL Timer driver
;****************************************************************
;Descriptive comment header goes here.
;Timer driver
;Name:  Andrei Tumbar
;Date:  10/07/20
;Class:  CMPE-250
;Section:  5, Tuesday, 11 AM
;---------------------------------------------------------------
;Keil Template for KL05
;R. W. Melton
;September 13, 2020
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL05Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT--------------------
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << PIT_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at ~24 MHz count rate
;0.01 s * ~24,000,000 Hz = ~240,000
;TSV = ~240,000 - 1
;Clock ticks for 0.01 s at 23,986,176 Hz count rate
;0.01 s * 23,986,176 Hz = 239,862
;TSV = 239,862 - 1
PIT_LDVAL_10ms  EQU  239861
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRL:  timer control register
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGFLLCLK
;MCGFLLCLK is 47972352 Hz ~=~ 48 MHz
;SBR ~=~ 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
;SBR = 47972352 / (9600 * 16) = 312.32 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGFLLCLK
;MCGFLLCLK is 47972352 Hz ~=~ 48 MHz
;SBR ~=~ 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
;SBR = 47972352 / (9600 * 16) = 312.32 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------
;****************************************************************
;EQUates
; Queue management record field offsets
IN_PTR      EQU   0
OUT_PTR     EQU   4
BUF_STRT    EQU   8
BUF_PAST    EQU   12
BUF_SIZE    EQU   16
NUM_ENQD    EQU   17
; Queue structure sizes
Q_BUF_SZ    EQU   4   ;Queue contents
Q_REC_SZ    EQU   18  ;Queue management record
TX_BUF_SZ    EQU   40  ; Queue buffer size for RxBuffer and TxBuffer
RX_BUF_SZ    EQU   5  ; Queue buffer size for RxBuffer and TxBuffer
            
            MACRO
$label      P  $string
            PUSH {R0}
            LDR R0,=$string
            BL  printf
            POP {R0}
            MEND

;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
Reset_Handler  PROC  {}
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL05 system startup with 48-MHz system clock
            BL      Startup
            BL      Init_PIT_IRQ
            BL      Init_UART0_IRQ
            CPSIE   I ; Unmask interrupts
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
            LDR  R1,=StopWatchRecord
__9         P    newline
main_loop   ; Get input
__0         P prompt; Print the prompt string
            BL   GetChar
            BL   HandleCommand
            B    main_loop

;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
            ALIGN
            LTORG
;>>>>> begin subroutine code <<<<<

; Handle a command input
; Subroutines used:
;   Dequene
;   Enqueue
;   printf
;   PutChar
; Input:
;   R0: input command (case insensitive)
;   R1: Pointer to counter structure
; Output (None)
; Modified Registers (None)
HandleCommand PROC {R0-R12}
            PUSH {R0-R4,LR}
AS_L_S      EQU 'a'
AS_L_E      EQU 'z'
AS_U_S      EQU 'A'
AS_U_E      EQU 'Z'
    
            ; Convert the character to upper-case
            MOVS R2,R0
            CMP R2,#AS_U_S
            BLT end_handle
            CMP R2,#AS_U_E
            BLE do_cmd ; Already in uppercase
            ; Convert to upper case
            SUBS R2,R2,#(AS_L_S - AS_U_S)
            B   do_cmd

do_cmd      CMP R2,#'C'
            BEQ do_clear
            CMP R2,#'D'
            BEQ do_display
            CMP R2,#'H'
            BEQ do_help
            CMP R2,#'P'
            BEQ do_pause
            CMP R2,#'T'
            BEQ do_time
            P    newline  ; Invalid character, don't print it
            B   end_handle

do_clear    ; Clear the counter in the
            ; timing record structure
            BL   PutChar ; print the command character
            P    newline
            MOVS R0,#0
            STR R0,[R1,#0]
            B   end_handle

do_display  ; Display the value of the counter
            ; Load the printf arguments
            BL   PutChar ; print the command character
            P    newline
            LDR R0,=display
            LDR R2,[R1,#0]
            PUSH {R2}
            BL  printf
            B   end_handle

do_help     ;
            BL   PutChar ; print the command character
            P    newline
__h         P    help
            B    end_handle

do_pause    ; Stop timing
            BL   PutChar ; print the command character
            P    newline
            MOVS R0,#0
            STRB R0,[R1,#4]
            B    end_handle

do_time     ; Start timing
            BL   PutChar ; print the command character
            P    newline
            MOVS R0,#1
            STRB R0,[R1,#4]
            B    end_handle
end_handle  POP  {R0-R4,PC}
            ENDP

; This subroutine is a printf() clone with the following
; formats defined:
;    %x: hex
;    %d: decimal
;    %s: string
;    %b: single byte decimal
;    %c: char
; Subroutines used:
;    PutChar
;    PutNumHex (%x)
;    PutNumU   (%d)
;    PutStringSB (%s)
;    PutNumUB (%b)
;    printf (recursive call on error)
; Input Registers:
;    R0: Pointer to the format string
;    Push all format arguments to the stack
; Output (None)
; Modified Registers (None)
printf      PROC {R0-R12}
            ; Because the stack is used to pass parameters to
            ; this subroutines. We can't modify the stack pointer
            ; before moving through the format string.
            ; Use the memory as a stack
            PUSH {R5}
            LDR  R5,=printf_static
            STR  R0,[R5,#0]
            STR  R1,[R5,#4]
            STR  R2,[R5,#8]
            STR  R3,[R5,#12]
            STR  R4,[R5,#16]
            POP  {R5}
            
            MOVS R1,R0
            
            ; Store the return pointer
            PUSH {LR}
            POP  {R4}
            
printf_loop LDRB R2,[R1,#0]
            TST  R2,R2
            BEQ  printf_end ;  End of string
            CMP  R2,#'%' ; Format character
            BEQ  printf_fmt
            MOVS R0,R2
            BL   PutChar
            B    printf_continue
printf_fmt  ADDS R1,R1,#1
            LDRB R2,[R1,#0]
            CMP  R2,#'x' ; Print a hex string
            BEQ  printf_h
            CMP  R2,#'d' ; Print a decimal string
            BEQ  printf_d
            CMP  R2,#'s' ; Print a string
            BEQ  printf_s
            CMP  R2,#'c'
            BEQ  printf_c
            CMP  R2,#'b' ; Print a single byte
            BEQ  printf_b
            ; INVALID FORMAT STRING
            ; Print the error string
            PUSH {R5}
            LDR  R1,[R5,#4]
            POP  {R5}
            
            PUSH {R1}
            LDR  R1,=invalid
            BL   printf
            B    . ; Invalid format string (STOP)
printf_h    ; Print the current number in R0
            POP  {R0}
            BL   PutNumHex
            B    printf_continue
printf_d    ; Print the current digit in R0
            POP  {R0}
            BL   PutNumU
            B    printf_continue
printf_s    ; Print the string in R0
            POP  {R0}
            PUSH {R1}
            MOVS R1,#100 ; Max string length of 100
            BL   PutStringSB
            POP  {R1}
            B    printf_continue
printf_b    ; Print the single byte value in R0
            POP  {R0}
            BL   PutNumUB
            B    printf_continue
printf_c    ; Print the single char value in R0
            POP  {R0}
            BL   PutChar
            B    printf_continue
printf_continue ADDS R1,R1,#1
            B  printf_loop
printf_end  ; Restore the link register
            PUSH {R4} ; This will be POP'ed to PC
            
            ; Restore registers from memory
            PUSH {R5}
            LDR  R5,=printf_static
            LDR  R0,[R5,#0]
            LDR  R1,[R5,#4]
            LDR  R2,[R5,#8]
            LDR  R3,[R5,#12]
            LDR  R4,[R5,#16]
            POP  {R5}
            
            POP {PC}
            ENDP

; Initialize a queue structure
; Subroutines Used: (none)
; Input:
;   R0: Pointer to the first address of the circular buffer
;   R1: Pointer to the start of the queue structure
;   R2: Size of the circular buffer pointed to by R0
; Output (None)
; Modified Registers (None)
InitQueue   PROC {R0-R12}
            PUSH {R0-R2}
            STR   R0,[R1,#IN_PTR]
            STR   R0,[R1,#OUT_PTR]
            STR   R0,[R1,#BUF_STRT]
            ADDS  R0,R0,R2
            STR   R0,[R1,#BUF_PAST]
            STRB  R2,[R1,#BUF_SIZE]
            MOVS  R0,#0
            STRB  R0,[R1,#NUM_ENQD]
            POP  {R0-R2}
            BX   LR
            ENDP

; Emplace a character into the queue structure.
; If the queue is already full, indicate failure
; by setting the C flag in the APSR. A cleared C flag
; indicates success.
; Subroutes used: (none)
; Input:  R0:  Character to enqueue
;         R1:  Address of queue record structure
; Output:  PSR C flag:  Success(0) or Failure (1)
; Modify:  APSR
; All other registers remain unchanged on return
Enqueue     PROC {R0-R12}
            PUSH {R2,R3}
            LDRB R2,[R1,#NUM_ENQD]
            LDRB R3,[R1,#BUF_SIZE]
            CMP  R2,R3  ; Check if the queue is full
            BGE  en_full
            LDR  R3,[R1,#IN_PTR]
            STRB R0,[R3,#0]; Put new element at memory location pointed by InPointer
            ; Increment NumberEnqueued
            ADDS R2,R2,#1
            STRB R2,[R1,#NUM_ENQD]

            ; Increment InPointer
            ADDS R3,R3,#1
            LDR  R2,[R1,#BUF_PAST]
            CMP  R3,R2
            BLT  en_str  ; If (R1->in_ptr < R1->BUF_PAST) goto en_str
            ; IN_PTR is past the end of the queue
            ; We need to set it back to the start of the queue
            LDR  R3,[R1,#BUF_STRT]
en_str      STR  R3,[R1,#IN_PTR]
            ; Clear the carry flag  (No error occured)
            MRS  R2,APSR
            MOVS R3,#0x20
            LSLS R3,R3,#24
            BICS R2,R2,R3
            MSR  APSR,R2
            B    en_done ; Don't clear the APSR C-flag
en_full     ; Set the carry flag  (Error occured)
            MRS  R2,APSR
            MOVS R3,#0x20
            LSLS R3,R3,#24
            ORRS R2,R2,R3
            MSR  APSR,R2
en_done     POP  {R2,R3}
            BX   LR
            ENDP

; Remove the next item from the queue structure.
; If the queue is already empty, indicate failure by
; setting the C flag in the APSR. A cleared C flag
; indicates success.
; Subroutines used: (none)
; Input:  R1:  Address of queue record structure 
; Output: R0:  Character dequeued 
;         PSR C flag:  Success(0) or Failure (1)
; Modify: R0
; APSR 
; All other registers remain unchanged on return
Dequeue     PROC {R1-R12}
            PUSH {R2-R4}
            LDRB R2,[R1,#NUM_ENQD] ; R2 = NUM_ENQ
            LDR  R3,[R1,#OUT_PTR] ; R3 = OUT_PTR
            CMP  R2,#0
            BLE  de_empty  ; No more items in the queue
            LDRB R0,[R3,#0] ; Get item at OUT_PTR
            ; Decrement number enqueued
            SUBS R2,R2,#1
            STRB R2,[R1,#NUM_ENQD]

            ; Increment OutPointer
            ADDS R3,R3,#1
            LDR  R4,[R1,#BUF_PAST]
            CMP  R3,R4
            BLT  de_str  ; If (R1->out_ptr < R1->BUF_PAST) goto de_str
            ; OUT_PTR is past the end of the queue
            ; We need to set it back to the start of the queue
            LDR  R3,[R1,#BUF_STRT]
de_str      STR  R3,[R1,#OUT_PTR]
            ; Clear the carry flag (No error occured)
            MRS  R2,APSR
            MOVS R3,#0x20
            LSLS R3,R3,#24
            BICS R2,R2,R3
            MSR  APSR,R2
            B    de_done
de_empty    ; Set the carry flag (Error occured)
            MRS  R2,APSR
            MOVS R3,#0x20
            LSLS R3,R3,#24
            ORRS R2,R2,R3
            MSR  APSR,R2
de_done     POP  {R2-R4}
            BX   LR
            ENDP


; Print a 32-bit number in hex
; Subroutines used:
;   PutDigHex
; Input parameter:
;   R0: number to print in hexadecimal (unsigned word value)
; Output parameter: (none)
PutNumHex   PROC {R0-R12}
RIGHT_NIBBLE_MASK EQU 0x0F
            PUSH {R0-R4,LR}
            ; There are 4 bytes to print
            MOVS R1,#0 ; Use R1 as an iterator
            MOVS R3,#24
            MOVS R4,#RIGHT_NIBBLE_MASK
            RORS R0,R0,R3 ; Move the MSB to the LSB
pnh_loop    CMP  R1,#4
            BGE  pnh_done
            ; Print the most significant nibble
            LSRS R2,R0,#4
            ANDS R2,R2,R4
            BL   PutDigHex ; Print this digit
            ; Print the least significant nibble
            MOVS R2,R0
            ANDS R2,R2,R4
            BL   PutDigHex
            RORS R0,R0,R3 ; Move to the next byte
            ADDS R1,R1,#1 ; i++
            B    pnh_loop
pnh_done    POP  {R0-R4,PC}
            ENDP

; Print a single hex digit
; stored in R2
; Subroutines used:
;   PutChar
; Input parameter:
;   R2 Hex digit to print
PutDigHex   PROC {R0-R12}
ASCII_LETTER EQU 10 ; Any values greater than or equal to this will be a letter A-F
ASCII_DIG_OFF EQU '0'
ASCII_DIG_LAST EQU '9'
ASCII_LET_OFF EQU ('A' - '0' - 10)
            PUSH {R0,R2,R3,LR}
            MOVS R3,#0xFF
            ANDS R2,R2,R3
            ADDS R2,#ASCII_DIG_OFF
            CMP  R2,#ASCII_DIG_LAST
            BLT  put_dig
            ADDS R2,R2,#ASCII_LET_OFF
put_dig     MOVS R0,R2
            BL   PutChar
            POP  {R0,R2,R3,PC}
            ENDP

; Prints to the terminal screen the decimal representation of the
; unsigned byte value in R0
; Subroutines Used:
;  PutNumU
; Input parameter:
;   R0:number to print in decimal (unsigned byte value)
;Output parameter: (none)
PutNumUB    PROC {R0-R12}
            PUSH {R0,R1,LR}
            MOVS R1,#BYTE_MASK
            ANDS R0,R0,R1
            BL   PutNumU
            POP  {R0,R1,PC}
            ENDP

; Print a number in decimal using the ascii
; characters
; SUBROUTINES USED
;   - PutChar (Print each digit)
;   - DIVU
; PARAMS
;   INPUT R0: Number to print (unsigned word value)
;   OUTPUT  : NONE
PutNumU             PROC {R0-R12}
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
DIVU        PROC {R2-R12}
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


; PutChar will print a character to the terminal
; Subroutines used: (none)
; Input: R0 (character to print)
; Output; None
; Register modification list: R1-R3
PutChar     PROC {R0-R12}
            PUSH {R0,R1,LR}
            LDR R1,=TxQueue
putchar_l   CPSID I
            BL Enqueue
            CPSIE I
            BCS putchar_l
            ; Enable the Tx interrupt
            LDR R0,=UART0_BASE
            MOVS R1,#UART0_C2_TI_RI
            STRB R1,[R0,#UART0_C2_OFFSET]
            POP {R0,R1,PC}
            ENDP

; GetChar will real a character from the terminal
; Subroutines used: (none)
; Input: None
; Output; R0 (character read from terminal)
; Register modification list: R1-R3
GetChar     PROC {R1-R12}
            PUSH {R1,LR}
            LDR R1,=RxQueue
getchar_l   CPSID I
            BL  Dequeue
            CPSIE I
            BCS getchar_l
            POP {R1,PC}
            ENDP


; UART0_ISR will handler the UART0 interrupt
; when data is ready to recieved/transmitted
; Subroutines used: Enqueue, Dequeue
; Input: None
; Output: None
; Reigster modification: None
UART0_ISR   PROC {R0-R12}
            PUSH {LR}
            CPSID I ; Don't interrupt in the middle of this one
            LDR  R1,=UART0_BASE
            MOVS R2,#UART0_C2_TIE_MASK
            LDRB R3,[R1,#UART0_C2_OFFSET]
            TST  R2,R3   ; If TIE = 1
            BEQ  check_Rx
            
            ; Check if there if the transmit
            ; register is empty
            MOVS R2,#UART0_S1_TDRE_MASK
            LDRB R3,[R1,#UART0_S1_OFFSET]
            TST  R2,R3
            BEQ  check_Rx ; Tx is not the source
            ; Get the next character from the Tx buffer
            LDR R1,=TxQueue
            BL  Dequeue
            BCS disable_tx
            ; Dequeue successful, write it to
            ; the data register
            LDR  R1,=UART0_BASE
            STRB R0,[R1,#UART0_D_OFFSET]
            B  check_Rx
disable_tx  ; No data to send, disable this interrupt so we
            ; don't cause an infinite loop
            ; Disable Tx interrupt
            LDR R1,=UART0_BASE
            MOVS R2,#UART0_C2_T_RI
            STRB R2,[R1,#UART0_C2_OFFSET]
check_Rx    MOVS R2,#UART0_S1_RDRF_MASK
            LDRB R3,[R1,#UART0_S1_OFFSET]
            TST  R2,R3
            BEQ uart0_isr_done
            
            LDRB R0,[R1,#UART0_D_OFFSET]
            LDR R1,=RxQueue
            BL  Enqueue  ; Put R0 in the RxQueue
uart0_isr_done CPSIE I
            POP{PC}
            ENDP

PIT_ISR     PROC {R0-R12}
            LDR  R0,=StopWatchRecord
            
            ; Check if we are timing
            LDRB R1,[R0,#4]
            TST  R1,R1
            BEQ  pit_finished ; We are not timing, exit
            ; Increment the counter
            LDR  R1,[R0,#0]
            ADDS R1,R1,#1
            STR  R1,[R0,#0]
pit_finished
            ; Clear interrupt
            LDR R0,=PIT_CH0_BASE
            LDR R1,=PIT_TFLG_TIF_MASK
            STR R1,[R0,#PIT_TFLG_OFFSET]
            BX LR
            ENDP

; Print a NULL terminated string
; SUBROUTINES USED
;   - PutChar (Print each character)
; PARAMS
;   INPUT R0: pointer to the target string (word address)
;   INPUT R1: size of the string buffer so that to not overflow (unsigned word value)
;   OUTPUT  : NONE
PutStringSB         PROC {R0-R12}
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

; Initialize the UART0 serial polling with
; 8 databits, no parity, one stop bit
; Subroutines used: (none)
; Input: None
; Output: None
; Register modification: R0-R2
Init_UART0_IRQ PROC {R3-R12}
            PUSH {R0-R2,LR}
            ; Select/configure UART0 sources
SIM_SOPT2_UART0SRC_MCGFLLCLK  EQU  (1 << SIM_SOPT2_UART0SRC_SHIFT)
            ; Set the UART0 clock to 48 MHz
            LDR     R0,=SIM_SOPT2
            LDR     R1,=SIM_SOPT2_UART0SRC_MASK
            LDR     R2,[R0,#0]        ;current SIM_SOPT2value
            BICS    R2,R2,R1          ;only UART0SRCbits cleared
            LDR     R1,=SIM_SOPT2_UART0SRC_MCGFLLCLK
            ORRS    R2,R2,R1          ;only UART0 bits changed
            STR     R2,[R0,#0]        ;update SIM_SOPT2
            
            ; Set SIM_SOPT5 for UART0 External
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  (SIM_SOPT5_UART0ODE_MASK :OR: SIM_SOPT5_UART0RXSRC_MASK :OR: SIM_SOPT5_UART0TXSRC_MASK)
            LDR   R0,=SIM_SOPT5
            LDR   R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
            LDR   R2,[R0,#0];current SIM_SOPT5value
            BICS  R2,R2,R1;only UART0 bits cleared
            STR   R2,[R0,#0];update SIM_SOPT5
            
            ; Enable clocks for UART0 and Port B
            ; Enable UART0 clocks
            LDR   R0,=SIM_SCGC4
            LDR   R1,=SIM_SCGC4_UART0_MASK
            LDR   R2,[R0,#0];current SIM_SCGC4value
            ORRS  R2,R2,R1;only UART0bit set
            STR   R2,[R0,#0];update SIM_SCGC4
            
            ; Set SIM_CGC5 for Port B Clock Enabled
            LDR   R0,=SIM_SCGC5
            LDR   R1,=SIM_SCGC5_PORTB_MASK
            LDR   R2,[R0,#0];current SIM_SCGC5value
            ORRS  R2,R2,R1;only PORTBbit set
            STR   R2,[R0,#0];update SIM_SCGC5
            
            ; Select Port B mux pins to connect to UART0
PORT_PCR_SET_PTB2_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: PORT_PCR_MUX_SELECT_2_MASK)
            
            LDR   R0,=PORTB_PCR2
            LDR   R1,=PORT_PCR_SET_PTB2_UART0_RX
            STR   R1,[R0,#0];Port B pin 2 connects to UART0 Rx
            LDR   R0,=PORTB_PCR1
            LDR   R1,=PORT_PCR_SET_PTB1_UART0_TX
            STR   R1,[R0,#0];Port B pin 1 connects to UART0 Tx
            
            ; Configure UART0 (register initialization)
            ; Load base addr for UART0
            LDR   R0,=UART0_BASE
            
            ; Disable UART0
            MOVS  R1,#UART0_C2_T_R
            LDRB  R2,[R0,#UART0_C2_OFFSET]
            BICS  R2,R2,R1
            STRB  R2,[R0,#UART0_C2_OFFSET]
            
            ; Set UART0 baud rate?BDH before BDL
            MOVS R1,#UART0_BDH_9600
            STRB R1,[R0,#UART0_BDH_OFFSET]
            MOVS R1,#UART0_BDL_9600
            STRB R1,[R0,#UART0_BDL_OFFSET]

            ; Set UART0 character format for serial bit stream and clear flags
            MOVS R1,#UART0_C1_8N1
            STRB R1,[R0,#UART0_C1_OFFSET]
            MOVS R1,#UART0_C3_NO_TXINV
            STRB R1,[R0,#UART0_C3_OFFSET]
            MOVS R1,#UART0_C4_NO_MATCH_OSR_16
            STRB R1,[R0,#UART0_C4_OFFSET]
            MOVS R1,#UART0_C5_NO_DMA_SSR_SYNC
            STRB R1,[R0,#UART0_C5_OFFSET]
            MOVS R1,#UART0_S1_CLEAR_FLAGS
            STRB R1,[R0,#UART0_S1_OFFSET]
            MOVS R1,#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
            STRB R1,[R0,#UART0_S2_OFFSET]
            
            ; Enable UART0
            MOVS R1,#UART0_C2_T_R
            STRB R1,[R0,#UART0_C2_OFFSET]
            
            ;Set UART0 IRQ priority
            LDR R0,=UART0_IPR
            ;LDR R1,=NVIC_IPR_UART0_MASK
            LDR R2,=NVIC_IPR_UART0_PRI_3
            LDR R3,[R0,#0]
            ;BICS R3,R3,R1
            ORRS R3,R3,R2
            STR R3,[R0,#0]
            
            ; Initialize the Rx and Tx queues
            LDR R0,=RxBuffer
            LDR R1,=RxQueue
            MOVS R2,#RX_BUF_SZ
            BL  InitQueue
            
            LDR R0,=TxBuffer
            LDR R1,=TxQueue
            MOVS R2,#TX_BUF_SZ
            BL  InitQueue
            
            ;Clear any pending UART0 interrupts
            LDR R0,=NVIC_ICPR
            LDR R1,=NVIC_ICPR_UART0_MASK
            STR R1,[R0,#0]
            ;Unmask UART0 interrupts
            LDR R0,=NVIC_ISER
            LDR R1,=NVIC_ISER_UART0_MASK
            STR R1,[R0,#0]

            POP {R0-R2,PC}
            ENDP

Init_PIT_IRQ PROC {R0-R12}
            PUSH {R0-R3}
            ; Initialize the timer structure
            LDR R0,=StopWatchRecord
            MOVS R1,#0
            STR R1,[R0,#0]
            STRB R1,[R0,#4]
            
            LDR R0,=SIM_SCGC6
            LDR R1,=SIM_SCGC6_PIT_MASK
            LDR R2,[R0,#0]  ;current SIM_SCGC6 value
            ORRS R2,R2,R1   ;only PIT bit set
            STR R2,[R0,#0]  ;update SIM_SCGC6
            
            ;Disable PIT timer 0
            LDR R0,=PIT_CH0_BASE
            LDR R1,=PIT_TCTRL_TEN_MASK
            LDR R2,[R0,#PIT_TCTRL_OFFSET]
            BICS R2,R2,R1
            STR R2,[R0,#PIT_TCTRL_OFFSET]
            
            ;Set PIT interrupt priority
            LDR R0,=PIT_IPR
            LDR R1,=NVIC_IPR_PIT_MASK
            ;LDR Rk,=NVIC_IPR_PIT_PRI_0
            LDR R3,[R0,#0]
            BICS R3,R3,R1
            ;ORRS Rl,Rl,Rk
            STR R3,[R0,#0]
            
            ; Enable interrupts for PIT on channel 0
            LDR R0,=PIT_CH0_BASE
            MOVS R1,#PIT_TCTRL_CH_IE
            STR R1,[R0,#PIT_TCTRL_OFFSET]
            
            ;Clear any pending PIT interrupts 
            LDR R0,=NVIC_ICPR
            LDR R1,=NVIC_ICPR_PIT_MASK
            STR R1,[R0,#0]
            
            ; Interrupt every 10ms
            LDR R0,=PIT_CH0_BASE
            LDR R1,=PIT_LDVAL_10ms
            STR R1,[R0,#PIT_LDVAL_OFFSET]
            
            ; Enable PIT module
            LDR R0,=PIT_BASE
            LDR R1,=PIT_MCR_EN_FRZ
            STR R1,[R0,#PIT_MCR_OFFSET]
            
            ;Unmask PIT interrupts
            LDR R0,=NVIC_ISER
            LDR R1,=NVIC_ISER_PIT_MASK
            STR R1,[R0,#0]
            
            
            POP{R0-R3}
            BX LR
            ENDP

;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendSV (PendableSrvReq)
                                      ;   pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 transfer 
                                      ;   complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:FTFA command complete/
                                      ;   read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:(reserved)
            DCD    Dummy_Handler      ;26:SPI0
            DCD    Dummy_Handler      ;27:(reserved)
            DCD    UART0_ISR          ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:(reserved)
            DCD    Dummy_Handler      ;30:(reserved)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:(reserved)
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    PIT_ISR            ;38:PIT
            DCD    Dummy_Handler      ;39:(reserved)
            DCD    Dummy_Handler      ;40:(reserved)
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:(reserved)
            DCD    Dummy_Handler      ;46:PORTA
            DCD    Dummy_Handler      ;47:PORTB
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
initial_p   DCB     "Press key for stopwatch command (C,D,H,P,T)\r\n\0"
prompt      DCB     ">\0"
display     DCB     "D:   %d x 0.01s\r\n\0"
status_s    DCB     "Status\0"
success     DCB     "Success\0"
failure     DCB     "Failure\0"
newline     DCB     "\r\n\0"
invalid     DCB     "\r\n\r\nInvalid format string '%s'\r\n\0"
invalid_in  DCB     "Invalid command '%c'\r\n\0"
help        DCB     ">H: C(lear),D(isplay),H(elp),P(ause),T(ime)\r\n\0"
test_invalid_printf  DCB  "Test string %jdsoka\0"  ; Just used to test the printf() function
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
RxQueue     SPACE   Q_REC_SZ
            ALIGN
TxQueue     SPACE   Q_REC_SZ
            ALIGN
printf_static SPACE 20
StopWatchRecord SPACE 5
RxBuffer    SPACE   RX_BUF_SZ
TxBuffer    SPACE   TX_BUF_SZ
;>>>>>   end variables here <<<<<
            ALIGN
            END