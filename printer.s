; drone co-rotine, resumed each time and performs one drone step

section .rodata


target_str_format: db "TARGET: %.2f,%.2f",10,0 ;
drone_str_format : db "%d,%.2f,%.2f,%.2f,%d",10,0;

drone_str : db " drone state : " ,10,0;
format_print_f: db "%.2f",10,0  ; print floating
format_print_s: db "%s",10,0  ; for print string
format_print_d: db "%d",10,0  ;

section .bss
tar_X_offset equ 0
tar_Y_offset equ 8
extern CORS_PTR_ARR
extern DRONES_ARR
extern TARGET_POS
extern NUMCO

DRONE_STRUC_SIZE equ 28

section .data

%macro printTarget_Macro 0
    pushad
    push dword [TARGET_POS+tar_Y_offset+4]
    push dword [TARGET_POS+tar_Y_offset]
    push dword [TARGET_POS+tar_X_offset+4]
    push dword [TARGET_POS+tar_X_offset]
    push target_str_format
    call printf
    add esp,16
    popad
%endmacro

%macro printDrone_Macro 1
    pushad 
    mov esi, %1
    mov eax,DRONE_STRUC_SIZE
    mov ebx,%1
    sub ebx,1
    mul ebx   ;eax holda offset
    mov ebx,[DRONES_ARR]
    add ebx,eax ; ebx holds struct pointer

    push dword [ebx+24]
    push dword [ebx+20]
    push dword [ebx+16]
    push dword [ebx+12]
    push dword [ebx+8]
    push dword [ebx+4]
    push dword [ebx]
    push dword esi

    push drone_str_format
    call printf
    add esp,36
    popad
%endmacro

; ------- MACROS: END -------


section .text
extern startCo.resume
global printGameBoard 
extern printf


printGameBoard:

    printTarget_Macro

      mov ecx,[NUMCO] ;ecx holds the num of drones (for loop) 
    .print_drone_state:
        mov esi,[NUMCO]
        sub esi,ecx  ; esi = current drone position in drones arr (offset)
        mov ebx,esi    
        add ebx,1   ; ebx = current loop iteration, start from 1 (first parameter for macro)
         
     printDrone_Macro ebx
        
    loop .print_drone_state,ecx

    mov edi,[CORS_PTR_ARR]
    mov ebx,[edi+4]  ; moving to ebx the id of the scheduler co-routine
    push printGameBoard
    jmp startCo.resume
      