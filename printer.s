; drone co-rotine, resumed each time and performs one drone step

section .rodata

target_str: db "Target location : " ,10,0 ;
drone_str : db " drone state : " ,10,0;
format_print_f: db "%f",10,0  ; print floating
format_print_s: db "%s",10,0  ; for print string
format_print_d: db "%d",10,0  ;

section .bss
extern CORS_PTR_ARR
extern DRONES_ARR
extern TARGET_POS
extern NUMCO
DRONE_STRUC_SIZE equ 16

section .data

section .text
extern startCo.resume
global printGameBoard 
extern printf


printGameBoard:

    pushad
    push target_str
    push format_print_s
    call printf
    add esp,8
    popad
        
        ;this code should be at the end of the function, it will resume the scheduler
    mov edi,[CORS_PTR_ARR]
    mov ebx,[edi+4]  ; moving to ebx the id of the scheduler co-routine
    push printGameBoard
    jmp startCo.resume
      