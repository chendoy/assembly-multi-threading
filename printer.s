; printer co-rotine, resumed every K step

section .rodata

section .bss
extern CORS_PTR_ARR

section .data

section .text
extern startCo.resume
global printGameBoard 

printGameBoard:

   ;this code should be at the end of the function, it will resume the scheduler
    mov edi,[CORS_PTR_ARR]
    mov ebx,[4+edi]  ; moving to ebx the id of the scheduler co-routine
    call startCo.resume