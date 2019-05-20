; main program file

section .rodata

section .bss

section .data

; ------- MACROS: START -------

; ------- MACROS: END -------

section .text
align 16
global main
extern printf
extern fprintf
extern malloc
extern free
extern sscanf

_start:

    pop    dword ecx    ; ecx = argc
    mov    esi,esp      ; esi = argv
    mov     eax,ecx     ; put the number of arguments into eax
    shl     eax,2       ; compute the size of argv in bytes
    add     eax,esi     ; add the size to the address of argv 
    add     eax,4       ; skip NULL at the end of argv
    push    dword eax   ; char *envp[]
    push    dword esi   ; char* argv[]
    push    dword ecx   ; int argc

    call    main        ; int main( int argc, char *argv[], char *envp[] )

    mov     ebx,eax     ; exiting
    mov     eax,1
    int     0x80

main:



; [IN]: coordinates x,y,α of a drone
; [OUT]: TRUE if the caller drone may destroy the target, FALSE otherwise 
mayDestroy:

; [IN]: void
; [OUT]: pseudo-random number in range [0...(2^16)-1]
generateRandom:
