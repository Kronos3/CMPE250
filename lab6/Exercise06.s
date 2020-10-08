            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
;(What does the program do?)
;Name:  <Your name here>
;Date:  <Date completed here>
;Class:  CMPE-250
;Section:  <Your lab section, day, and time here>
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
;from 'MKL05Z4.s� included by program template
;---------------------------------------------------------------
;UART0_BDH
; 0-> 7:LIN break detect IE (disabled)
; 0-> 6:RxD input active edge IE (disabled)
; 0-> 5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGFLLCLK
;MCGPLLCLK is 47972352 Hz ~=~ 48 MHz
;SBR � 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
;SBR = 47972352 / (9600 * 16) = 312.32 --> 312 = 0x138
UART0_BDH_9600 EQU 0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGFLLCLK
;MCGPLLCLK is 47972352 Hz ~=~ 48 MHz
;SBR � 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
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
main_loop   BL      GetChar
            BL      PutChar
            B       main_loop
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<
; PutChar will print a character to the terminal
; Input: R0 (character to print)
; Output; None
; Register modification list: R1-R3
PutChar     PROC {R0-R3}
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
; Input: None
; Output; R0 (character read from terminal)
; Register modification list: R1-R3
GetChar     PROC {R0-R3}
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

; Initialize the UART0 serial polling with
; 8 databits, no parity, one stop bit
; Input: None
; Output: None
; Register modification: R0-R2
Init_UART0_Polling PROC {R0-R2}
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
            
            ; Set UART0 baud rate�BDH before BDL
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
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
;>>>>>   end variables here <<<<<
            ALIGN
            END
