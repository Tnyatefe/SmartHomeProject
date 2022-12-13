.section Timer, "ax"
.global SysTick_Handler
.type SysTick_Handler, "function"


SysTick_Handler:
.global counter
LDR R0, =counter
LDR R1, [R0]
ADD R1, #1
STR R1, [R0]
BX LR
