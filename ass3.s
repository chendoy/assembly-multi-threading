; main program file

section .rodata
format_input_d: db "%d",0     ; for sscanf
format_print_d: db "%d",10,0  ; for printf

section .bss

CURR        resd 1          ; current co-rotine pointer
STKSIZE     equ 16*1024     ; drone's stack size - 16kib
COS_ARR     resd 1          ; pointer to drones struct array
SPT         resd 1          ; temporary stack pointer
SPMAIN      resd 1          ; stack pointer of main
struc CO_i                 ; drone struct
    CODEP: resd 1
    SPP:   resd 1
endstruc

CO_STRUCT_SIZE equ 8
NUMCO        resd 1            ; N - # of co-routine
NUM_HITS     resd 1            ; T - number of hits to win
PRINT_STEPS  resd 1            ; K - number of steps until printing
BETA         resd 1            ; β - visibility angle
DISTANCE     resd 1            ; d - visibility distance of a drone
SEED         resd 1            ; seed - LFSR's seed


section .data

; ------- MACROS: START -------

; [ebp-4] must be available for this macro
%macro ALLOC_STACK 0
    pushad
    push STKSIZE
    call malloc
    add esp,4
    add eax,STKSIZE  ; make the pointer point to the END of the co-routine stack
    mov [ebp-4],eax
    popad
    
%endmacro

; ------- MACROS: END -------

section .text
align 16
global main
extern printf
extern fprintf
extern malloc
extern calloc ; CAN USE OR NOT??
extern free
extern sscanf
extern printGameBoard
extern schedule
extern target_func
extern moveDrone

main:

    mov esi, [esp+8]
    push esi
    call getArguments
    add esp,4

    push dword [NUMCO]
    call alloc_coRotines     ; allocate co-routines memory
    add esp,4
    mov dword [COS_ARR], eax
    pushad


    push dword [NUMCO]
    call init_coRotines      ; initializes co-routines
    add esp,4

    mov     ebx,eax     ; exiting
    mov     eax,1
    int     0x80


; [IN]: coordinates x,y,α of a drone
; [OUT]: TRUE if the caller drone may destroy the target, FALSE otherwise 
;mayDestroy:


    mov ebx, [ebp+8]  ; ebx = N
    add ebx,3         ; ebx = N+3
    pushad
    push ebx
    push CO_STRUCT_SIZE
    call calloc
    add esp,8
    mov [ebp-4], eax
    popad

    popad
    mov eax, [ebp-4]
    mov esp, ebp
    pop ebp
    ret

; [IN]: number of drones co rotines
; [OUT]: pointer to a co-rotines allocated 
alloc_coRotines:
    push ebp
    mov ebp,esp
    sub esp,4
    pushad

    mov ebx, [ebp+8]  ; ebx = N
    add ebx,3         ; ebx = N+3
    pushad
    push ebx
    push CO_STRUCT_SIZE
    call calloc
    add esp,8
    mov [ebp-4], eax
    popad

    popad
    mov eax, [ebp-4]
    mov esp, ebp
    pop ebp
    ret


; [IN]: char* argv[]
; [OUT]: void
getArguments:
    push ebp
    mov ebp,esp
    pushad

    mov esi, [ebp+8] ; esi = char* argv[]

    mov dword ebx, [esi+4]

    push NUMCO
    push format_input_d
    push ebx
    call sscanf
    add esp,12

    mov dword ebx, [esi+8]

    push NUM_HITS
    push format_input_d
    push ebx
    call sscanf
    add esp,12

    mov dword ebx, [esi+12]

    push PRINT_STEPS
    push format_input_d
    push ebx
    call sscanf
    add esp,12

    mov dword ebx, [esi+16]

    push BETA
    push format_input_d
    push ebx

    call sscanf
    add esp,12

    mov dword ebx, [esi+20]

    push DISTANCE
    push format_input_d
    push ebx
    call sscanf
    add esp,12

    mov dword ebx, [esi+24]

    push SEED
    push format_input_d
    push ebx
    call sscanf
    add esp,12

    popad
    mov esp,ebp
    pop ebp
    ret


; [IN]: number of co rotines
; [OUT]: void
init_coRotines:
    push ebp
    mov ebp,esp
    sub esp,4
    pushad

    mov ecx, [ebp+8]  ; ecx = N

    ; will now init printer, scheduler, target

    ; printer init

    ALLOC_STACK
    mov edx, [ebp-4]
    mov ebx, [COS_ARR]  ; get first co-routine aka printer
    mov dword [ebx+CODEP], printGameBoard
    mov dword [ebx+SPP], edx
    mov [SPT], esp
    mov esp, [ebx+SPP]
    push printGameBoard
    pushfd
    pushad
    mov [ebx+SPP], esp
    mov esp, [SPT]

    ; scheduler init

    ALLOC_STACK
    mov edx, [ebp-4]
    mov ebx, [COS_ARR]  ; get first co-routine aka printer
    add ebx, CO_STRUCT_SIZE
    mov dword [ebx+CODEP], schedule
    mov dword [ebx+SPP], edx
    mov [SPT], esp
    mov esp, [ebx+SPP]
    push schedule
    pushfd
    pushad
    mov [ebx+SPP], esp
    mov esp, [SPT]

    ; target init

    ALLOC_STACK
    mov edx, [ebp-4]
    mov ebx, [COS_ARR]  ; get first co-routine aka printer
    add ebx,2* CO_STRUCT_SIZE
    mov dword [ebx+CODEP], target_func
    mov dword [ebx+SPP], edx
    mov [SPT], esp
    mov esp, [ebx+SPP]
    push target_func
    pushfd
    pushad
    mov [ebx+SPP], esp
    mov esp, [SPT]

    ; init N drones in a "for loop"

    mov ecx, [ebp+8]  ; ecx = N

    .init_drones:

    ALLOC_STACK
    mov ebx, [COS_ARR]  ; get first co-routine aka printer

    ; adding the current drones offset to ebx

    mov eax, CO_STRUCT_SIZE
    mov edi, 3
    mul edi
    add ebx, eax                    ; ebx = ebx + 3*[STRUCT_SIZE]
    mov eax, [ebp+8]
    sub eax,ecx                     ; eax = N - ecx (offset)
    mov edi, CO_STRUCT_SIZE
    mul edi              ; eax = (N - ecx) * CO_STRUCT_SIZE
    add ebx,eax       

    mov dword [ebx+CODEP], moveDrone
    mov edx, [ebp-4]     ; re-taking allocated co-stack (because mul destroyed it)
    mov dword [ebx+SPP], edx
    mov [SPT], esp
    mov esp, [ebx+SPP]
    push moveDrone
    pushfd
    pushad
    mov [ebx+SPP], esp
    mov esp, [SPT]

    loop .init_drones, ecx

    popad
    mov esp,ebp
    pop ebp
    ret