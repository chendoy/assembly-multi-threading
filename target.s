; target co-rotine, resumed every time a target is destroyed

section .rodata

section .bss
tar_X_offset equ 0
tar_Y_offset equ 8
extern TARGET_POS
extern CORS_PTR_ARR
extern randomized

section .data

section .text
extern init_target
extern startCo.resume
global createTarget

createTarget:
    call init_target 
    
    mov edi,[CORS_PTR_ARR]
    mov ebx,[edi+4]  ; moving to ebx the id of the scheduler co-routine
    push createTarget
    jmp startCo.resume
