; drone co-rotine, resumed each time and performs one drone step

section .rodata

section .bss
extern CORS_PTR_ARR

section .data

section .text
extern startCo.resume

global moveDrone 
moveDrone:
    
    ;this code should be at the end of the function, it will resume the scheduler
    mov edi,[CORS_PTR_ARR]
    mov ebx,[edi+4]  ; moving to ebx the id of the scheduler co-routine
    push moveDrone
    jmp startCo.resume

    