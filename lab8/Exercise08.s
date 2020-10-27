            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
;Add perform arithmetic on n-word unsigned integers
;Name:  Andrei Tumbar
;Date:  10/18/20
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
;EQUates
; UART0 Equates
;UART0 register addresses as well as bit field offsets and masks
;from 'MKL05Z4.s' included by program template
;---------------------------------------------------------------
;UART0_BDH
; 0-> 7:LIN break detect IE (disabled)
; 0-> 6:RxD input active edge IE (disabled)
; 0-> 5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGFLLCLK
;MCGPLLCLK is 47972352 Hz ~=~ 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
;SBR = 47972352 / (9600 * 16) = 312.32 --> 312 = 0x138
UART0_BDH_9600 EQU 0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGFLLCLK
;MCGPLLCLK is 47972352 Hz ~=~ 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
;SBR = 47972352 / (9600 * 16) = 312.32 --> 312 = 0x138
UART0_BDL_9600 EQU 0x38

;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select
; (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1 EQU 0x00
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
UART0_C2_T_R EQU (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)

;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
; 10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
; 10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
; (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV EQU 0x00

;UART0_C4
; 0--> 7:MAEN1=match address mode enable 1 (disabled)
; 0--> 6:MAEN2=match address mode enable 2 (disabled)
; 0--> 5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
; = 1 + OSR for 3 <= OSR <= 31
; = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16 EQU 0x0F
UART0_C4_NO_MATCH_OSR_16 EQU UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
; 0--> 7:TDMAE=transmitter DMA enable (disabled)
; 0--> 6:Reserved; read-only; always 0
; 0--> 5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
; 0--> 1:BOTHEDGE=both edge sampling (rising edge only)
; 0--> 0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC EQU 0x00

;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS EQU (UART0_S1_IDLE_MASK :OR: \
UART0_S1_OR_MASK :OR: \
UART0_S1_NF_MASK :OR: \
UART0_S1_FE_MASK :OR: \
UART0_S1_PF_MASK)

;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
; write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
; write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS EQU \
(UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)

;****************************************************************
            MACRO
$label      P  $string
            PUSH {R1}
            LDR R1,=$string
            BL  printf
            POP {R1}
            MEND

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
            BL      Init_UART0_Polling
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
WORD_COUNT  EQU 4
prompt_start
__1         P    newline
__2         P    prompt_1
            
            ; Get the first number
prompt_fir  LDR  R0,=number_1
            MOVS R1,#WORD_COUNT
            BL   GetHexMulti
            BCS  invalid_input_1
            
__5         P    prompt_2
            
            ; Get the second number
prompt_sec  LDR  R0,=number_2
            MOVS R1,#WORD_COUNT
            BL   GetHexMulti
            BCS  invalid_input_2
            
            ; Add the numbers and print them
            LDR  R0,=result
            LDR  R1,=number_1
            LDR  R2,=number_2
            MOVS R3,#WORD_COUNT
            
            BL   AddIntMultiU
            BCS  add_overflow

__6         P   sum
            MOVS R1,#WORD_COUNT
            BL   PutNumHexMulti
            B    prompt_start

add_overflow
__7         P   sum
__8         P   overflow
            B    prompt_start

invalid_input_1
__3         P prompt_e
            B  prompt_fir
invalid_input_2
__4         P prompt_e
            B  prompt_sec
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP

; Add two arbitarily sized unsigned integers
; Input params:
;    R1: n-word augend address
;    R2: n-word addend address
;    R3: n number of words in addend and augend
; Output params:
;    R0: n-word sum address
;    C: 0 success, 1 failure (overflow)
; Subroutines: None
AddIntMultiU    PROC {R1-R12}
                PUSH {R0-R6}
                ; Use R3 as a down counter of words
                ; Use R4 and R5 as temporary storage for
                ; adding single words

                ; We first need to clear the C-flag
                PUSH {R2,R3}
                MRS  R2,APSR
                MOVS R3,#0x20
                LSLS R3,R3,#24
                BICS R2,R2,R3
                MSR  APSR,R2
                POP {R2,R3}

                MRS R6,APSR
AddLoop         CMP R3,#0
                BLE AddDone
                ; Load the next words into R4 and R5
                LDM R1!,{R4}
                LDM R2!,{R5}
                
                SUBS R3,R3,#1 ; Decrement counter
                MSR APSR,R6
                ADCS R4,R4,R5 ; Add the numbers and set the status flags
                MRS R6,APSR
                STM R0!,{R4}  ; Store the sum in the next address 
                B AddLoop
AddDone         MSR APSR,R6
                POP {R0-R6}
                BX  LR
                ENDP

; GetHexMulti will get an N-word hexidecimal number
; from the terminal and store it in memory
; Input params:
;    R1: n: number of words in the input
; Output params:
;    R0: address to number input from user
;    APSR C-flag: 0 valid, 1 invalid
; Subroutines used:
;    GetStringSB
GetHexMulti     PROC {R1-R12}
                PUSH {R0-R6,LR}
                
                ; Clear the output
                PUSH {R0,R1}
                MOVS R5,#0
ClearLoop       TST  R1,R1   ; Check if R1 is 0
                BEQ  ClearDone
                STR R5,[R0,#0]
                ADDS R0,R0,#4
                SUBS R1,R1,#1  ; Decrement down counter
                B  ClearLoop
ClearDone       POP {R0,R1}
                
                ; Allocate string buffer on the stack
                ; We need at least 8*4 + 1, but the
                ; stack must be moved by multiple of 4
                ; so 8*5 = 40
                MOV  FP,SP      ; Save the position of the stack
                SUB  SP,SP,#40
                MOV  R3,SP

                ; Get string input and store it in R2
                PUSH {R0,R1}
                MOVS R0,R3
                LSLS R1,R1,#3
                ADDS R1,R1,#1
                BL   GetStringSB
                POP {R0,R1}

                ; Go to the end of the string
                MOVS R2,R3   ; Store the start of the string in R2
GetLenLoop      LDRB R4,[R3,#0]
                CMP R4,#0
                BEQ GetHexLen
                ADDS R3,R3,#1
                B   GetLenLoop
GetHexLen       
                ; R3 is now at the end of the string
                ; Read the string backwards and store the result in the output
                ; R0 is the output pointer and will be incremented after 4-bytes are read
                ; Use R4 as a counter for how many characters have been read
                ; R5 will be used to create a bit shift and as a temporary variable
                MOVS R1,#0    ; Use as a counter for the offset of the output word
NextWord        MOVS R4,#0    ; Initialize the counter for characters
NextNibble      SUBS R3,R3,#1
                LSLS R5,R4,#2 ; Create a shift 4*[Characters read]
                LDRB R6,[R3,#0] ; Load the next character from the string
                BL   FromHex    ; Convert R6 to hex value
                BCS  GetHexMultiFail ; If conversion fails go to failure branch
                LSLS R6,R6,R5   ; Shift the variable to the correct position in the word
                LDR  R5,[R0,R1] ; Load the current output word
                ORRS R5,R5,R6   ; Append this nibble to the current word
                STR  R5,[R0,R1] ; Store the nibble
                ADDS R4,R4,#1   ; Increment the character counter
                CMP  R3,R2      ; Check if we just read the first character in the input string
                BEQ  GetHexMultiDone  ; If it was the first, we are done
                CMP  R4,#8      ; Check if we read an entire word yet
                BLO  NextNibble   ; If we haven't continue reading into this word
                ADDS R1,R1,#4   ; Go to the next word
                B    NextWord
GetHexMultiDone MOV  SP,FP ; Restore the stack position
                ; Clear C-flag to indicate success
                PUSH {R2,R3}
                MRS  R2,APSR
                MOVS R3,#0x20
                LSLS R3,R3,#24
                BICS R2,R2,R3
                MSR  APSR,R2
                POP  {R2,R3}
                POP  {R0-R6,PC}
GetHexMultiFail MOV SP,FP   ; Keep the C-flag set
                POP  {R0-R6,PC}
                ENDP

; PutNumHexMulti will take N-word integer and
; print the hexidecimal represenation
; Input params:
;     R0: pointer to the word array
;     R1: Number of words in integer
; Output params: (none)
; Subroutines used: PutNumHex
PutNumHexMulti  PROC {R0-R12}
                PUSH {R0-R2,LR}
                MOVS R2,R0  ; Use R2 as the pointer 
                            ; because R0 is a parameter
                            ; to PutNumHex
                ; Calculate the pointer of the final word
                LSLS R1,R1,#2
                ADDS R1,R1,R0
                SUBS R1,R1,#4
pnhm_loop       CMP  R1,R2  ; Check if we read all the words
                BLT  PutNumHexMultiDone
                LDR  R0,[R1,#0] ; Load the word into R0 and go to the next word
                BL   PutNumHex  ; Print the word
                SUBS R1,R1,#4
                B    pnhm_loop
PutNumHexMultiDone
                POP {R0-R2,PC}
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
            MOVS R3,#0x0F
            ANDS R2,R2,R3
            ADDS R2,#ASCII_DIG_OFF
            CMP  R2,#ASCII_DIG_LAST
            BLE  put_dig
            ADDS R2,R2,#ASCII_LET_OFF
put_dig     MOVS R0,R2
            BL   PutChar
            POP  {R0,R2,R3,PC}
            ENDP

; ToUpper will convert an ASCII character to uppercase
; If the input character is not lowercase it will not change it
; Input Params:
;       R6: Input character
; Ouptut Params:
;       R6: Upper-case character
; Subroutines used: (none)
ToUpper     PROC {R0-R5,R7-R12}
AS_L_S      EQU 'a'
AS_L_E      EQU 'z'
AS_U_S      EQU 'A'
            ; Check if character is lowercase
            CMP R6,#AS_L_E
            BGT ToUpperDone   ; Not a lowercase character
            CMP R6,#AS_L_S
            BLT ToUpperDone ; Already in uppercase or not an alpha character
            ; Convert to upper case
            SUBS R6,R6,#(AS_L_S - AS_U_S)
ToUpperDone BX LR
            ENDP


; FromHex will convert an ASCII hexidecimal character to binary
; If the ASCII character is not a valid hexidecimal value, a
; failure condition will be set (APSR-C)
; Input Params:
;       R6: ascii character
; Output Params:
;       R6: hex digit conversion
;       APSR-C: 0 (success), 1 (failure)
; Subroutines used: ToUpper
FromHex     PROC {R0-R5,R7-R12}
            PUSH {LR}
            BL   ToUpper ; Convert R3 to uppercase
            CMP  R6,#AS_U_S ; Check if A-Z or 0-9
            BLT  FromHex09
            ; R6 is A-Z
            CMP  R6,#'F'
            BGT  FromHexFail     ; Not a valid hexidemical character
            SUBS R6,R6,#(AS_U_S - 10) ; Convert to digit
            B    FromHexDone
FromHex09   CMP  R6,#'0'
            BLT  FromHexFail     ; Not a valid hexidemical character
            CMP  R6,#'9'
            BGT  FromHexFail     ; Not a valid hexidemical character
            SUBS R6,R6,#'0'      ; Convert digit
            B    FromHexDone
FromHexFail ; Set C-flag to indicate failure
            PUSH {R2,R3}
            MRS  R2,APSR
            MOVS R3,#0x20
            LSLS R3,R3,#24
            ORRS R2,R2,R3
            MSR  APSR,R2
            POP {R2,R3,PC}
FromHexDone ; Clear C-flag to indicate Success
            PUSH {R2,R3}
            MRS  R2,APSR
            MOVS R3,#0x20
            LSLS R3,R3,#24
            BICS R2,R2,R3
            MSR  APSR,R2
            POP {R2,R3,PC}
            ENDP

; PutChar will print a character to the terminal
; Subroutines used: (none)
; Input: R0 (character to print)
; Output; None
; Register modification list: R1-R3
PutChar     PROC {R0-R12}
            PUSH {R1-R3}
            
            ; Wait until last character has been sent
            ;Set Z if TDRE=0
            LDR  R1,=UART0_BASE
            MOVS R2,#UART0_S1_TDRE_MASK
putchar_l   LDRB R3,[R1,#UART0_S1_OFFSET]
            TST  R2,R3
            BEQ  putchar_l
            
            ; Write the character to the terminal
            STRB R0,[R1,#UART0_D_OFFSET]
            
            POP {R1-R3}
            BX  LR
            ENDP

; GetChar will real a character from the terminal
; Subroutines used: (none)
; Input: None
; Output; R0 (character read from terminal)
; Register modification list: R1-R3
GetChar     PROC {R1-R12}
            PUSH {R1-R3}
            
            ; Wait until a character is ready
            ;Set Z if RDRF=0
            LDR  R1,=UART0_BASE
            MOVS R2,#UART0_S1_RDRF_MASK
getchar_l   LDRB R3,[R1,#UART0_S1_OFFSET]
            TST  R2,R3
            BEQ  getchar_l
            
            ; Read the value from the data register
            LDRB R0,[R1,#UART0_D_OFFSET]
            
            POP {R1-R3}
            BX  LR
            ENDP

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
                    CMP   R0,#'\r'    ; while (c != '\r')
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
                    MOVS  R0,#'\r'
                    BL    PutChar
                    MOVS  R0,#'\n'
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
Init_UART0_Polling PROC {R3-R12}
            PUSH {R0-R2}
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

            POP {R0-R2}
            BX  LR
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
;    R1: Pointer to the format string
;    N variables on the stack that correspond 
;    to arguments in the format string
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
            DCD    Dummy_Handler      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:(reserved)
            DCD    Dummy_Handler      ;30:(reserved)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:(reserved)
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    Dummy_Handler      ;38:PIT
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


;Constants
            AREA    MyConst,DATA,READONLY
prompt_1    DCB     " Enter first 128-bit hex number:  0x\0"
prompt_2    DCB     "Enter 128-bit hex number to add:  0x\0"
prompt_e    DCB     "      Invalid number--try again:  0x\0"
sum         DCB     "                            Sum:  0x\0"
invalid     DCB     "\r\n\r\nInvalid format string '%s'\r\n\0"
overflow    DCB     "OVERFLOW\0"
newline     DCB     "\r\n\0"
hex_str     DCB     "%x\r\n\0"

;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
number_1        SPACE WORD_COUNT*4
number_2        SPACE WORD_COUNT*4
result          SPACE WORD_COUNT*4
printf_static   SPACE 20 ; Internal memory used by printf
;>>>>>   end variables here <<<<<
            ALIGN
            END
            END