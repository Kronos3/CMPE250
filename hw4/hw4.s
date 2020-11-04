; Question 1
    ; Load the input and output matrices
    LDR R0,=Q
    LDR R1,=P
    LDR R2,=F

    PUSH{R5,R6}
    LDR R6,=0xFFFF  ; Create a mask for the least sig halfword
    ; Load the matrix metadata
    MOVS R3,#Q_ROWS
    LSLS R3,R3,#16  ; Move the rows 
                    ; to the Most significant
                    ; halfword
    MOVS R5,#Q_COLS
    ANDS R5,R5,R6   ; Make sure COLS stays in the halfword
    ORRS R3,R3,R5   ; Put the halfword in the register

    ; Load P_ROWS:P_COLS into R4
    MOVS R4,#Q_ROWS
    LSLS R4,R4,#16  ; Move the rows 
                    ; to the Most significant
                    ; halfword
    MOVS R5,#Q_COLS
    ANDS R5,R5,R6   ; Make sure COLS stays in the halfword
    ORRS R4,R4,R5   ; Put the halfword in the register

    POP {R5,R6}

    BL MatrixMult   ; Compute the matrix product
    BVC Done       ; No error occured, skip error subroutine
    BL Error
Done


Question 2
- Set the UART0 interrupt priority on SIM_SCGC4
- Clear any interrupts currently active on NVIC_ICPR by
   writing NVIC_ICPR_UART0_MASK
- Enable the interrupt by writing NVIC_ISER_UART0_MASK to 
  to NVIC_ISER



Question 3
A subroutine is invoked by a standard CPU instruction (BL). Before the subroutine is entered
the PC is stored in to the link register (LR). To return from execution the link register must
be writen back to the program counter using either the stack or the BX instruction

An ISR is invoked when it is set up in the vector table and both the interrupt has been
enabled by the peripheral device as well as unmasked by the CPU. When the interrupt condition
is set the CPU will push some of the general purpose registers as well as the program counter. 
The link register is set to an address to restore the CPU to its previous execution.

An expection handler is caused by a software fault such as an invalid memory access. It is 
handled in the same way that an interrupt is handled expect it is triggered by the CPU instead
of peripheral hardware.


Question 4
Exiting from an ISR will look the same as a subroutine. LR however is set to a return for the
interrupt handler not the next instruction in the code. This return area will restore pushed 
registers described above and return back to normal program execution.


Question 5
            MACRO
$label      TOTAL $ROUT, $RIN, $RLEN
            PUSH{$RIN,$RLEN}
            MOVS $ROUT, #0  ; Zero out the sum
$label.loop TST $RLEN,$RLEN ; Check we read all the words
            BEQ $label.done
            ; Save the pointer to use RIN as a temporary register
            PUSH{$RIN}
            LDR $RIN,[$RIN,#0] ; Load the current value
            ADDS $ROUT,$ROUT,$RIN ; Add the current value to the sum
            POP{$RIN}
            ADDS $RIN,$RIN,#4
            SUBS $RLEN,$RLEN,#1
            B $label.loop
$label.done POP{$RIN,RLEN}
            MEND
