; main program file
global NUMCO
global CURR
global STKSIZE
global COS_ARR
global CORS_PTR_ARR
global SPT
global SPMAIN
global CO_i
global PRINT_STEPS;

section .rodata
format_input_d: db "%d",0     ; for sscanf
format_print_d: db "%d",10,0  ; for printf
format_print_p: db "%p",10,0  ; for printf
format_print_s: db "%s",10,0  ; for printf
newLine : db "\n",10,0;

section .bss

CURR        resd 1          ; current co-rotine pointer
STKSIZE     equ 16*1024     ; drone's stack size - 16kib
COS_ARR     resd 1          ; pointer to drones struct array
CORS_PTR_ARR resd 1          ;array of pointers to cor structs
SPT         resd 1          ; temporary stack pointer
SPMAIN      resd 1          ; stack pointer of main
struc CO_i                 ; drone struct
    CODEP: resd 1
    SPP:   resd 1
endstruc

CO_STRUCT_SIZE equ 8
CO_PTR_SIZE    equ 4
NUMCO        resd 1            ; N - # of co-routine
NUM_HITS     resd 1            ; T - number of hits to win
PRINT_STEPS  resd 1            ; K - number of steps until printing
BETA         resd 1            ; β - visibility angle
DISTANCE     resd 1            ; d - visibility distance of a drone
SEED         resw 1            ; seed - LFSR's seed
curr_LFSR    resw 1            ; current LFSR's read



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
%macro ptrprint 1
    pushad
    push  dword %1
    push format_print_p
    call printf
    add esp,8
    popad
    
    pushad
    push newLine
    push format_print_s
    call printf
    add esp,8
    popad
%endmacro

; ------- MACROS: END -------

section .text

align 16
global main
global startCo
global startCo.resume;
global startCo.endCo;
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
    pushad    ;  why ?

    mov dx,word [SEED]            ; initializing LFSR with SEED
    mov word [curr_LFSR],dx
    popad
    
    push dword [NUMCO]
    call init_coRotines      ; initializes co-routines
    add esp,4

    
    push 1                    ; first we start the scheduler co-routine 
    call startCo
    add esp,4

                              ; here we are after first round - we need to decide how to proceed.

    mov     ebx,eax     ; exiting
    mov     eax,1
    int     0x80

startCo:

    push ebp                  
    mov ebp,esp                
    pushad                        ;save registers of main
    mov [SPMAIN],esp              ;save esp of main
    mov ebx,[ebp+8]               ; gets ID of  co-routine to activate
    mov edi,[CORS_PTR_ARR]
    mov ebx,[ebx*4+edi]  ;get a pointer to co-routine struct
    jmp .do_resume
 
 .resume: ; save state of current co-routine
    pushfd
    pushad
    mov edx,[CURR]
    mov [edx+SPP],esp  ;save current esp


 .do_resume: ; load esp for resumed co-routine   
    mov esp,[ebx+SPP] ;esp points now to the co routine stack
    mov [CURR],ebx    ; CURR points now to the struct of the current co-routine
    popad              ;restore resumed co-routine state 
    popfd              ;restore flags
    ret                ; this will jump to the activated function that serve as "return address"
 
 .endCo:
    mov esp,[SPMAIN]   ; restore esp of main
    popad              ; restore registers of main      
    pop ebp
    ret

; [IN]: coordinates x,y,α of a drone
; [OUT]: TRUE if the caller drone may destroy the target, FALSE otherwise 
;mayDestroy:

; [IN]: void
; [OUT]: the next pseudo-random LSFR generated number  
generateNumber:
    push ebp
    mov ebp,esp
    sub esp,4
    pushad

    mov esi,0     ; esi is the xoring result
    mov bx, word [curr_LFSR]
    mov ecx, 16   ; ecx is the loop counter

    .one_shift:

    mov dx,0
    test bx, 0000000000000001b ; checks if the 16-th bits is on
    jz .16_bit_was_not_set
    mov dx,1

    .16_bit_was_not_set:
    mov si,dx  ; saving zero flag aside for now
    mov dx,0
    test bx, 0000000000000100b ; checks if the 14-th bits is on
    jz .14_bit_was_not_set
    mov dx,1

    .14_bit_was_not_set:
    XOR si,dx
    mov dx,0
    test bx, 0000000000001000b ; checks if the 13-th bits is on
    jz .13_bit_was_not_set
    mov dx,1

    .13_bit_was_not_set:
    XOR si,dx
    mov dx,0
    test bx, 0000000000100000b ; checks if the 11-th bits is on
    jz .11_bit_was_not_set
    mov dx,1

    .11_bit_was_not_set:
    XOR si,dx

    shr bx,1 ; shifting
    cmp si,0
    jz .do_not_set
    OR bx, 1000000000000000b ; setting input bit to 1
    .do_not_set:

    loop .one_shift, ecx
    
    mov word [curr_LFSR],bx
    mov [ebp-4], ebx

    ; after loop - return the result in eax

    popad
    mov eax, [ebp-4]
    mov esp,ebp
    pop ebp
    ret




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
    
    pushad
    push ebx
    push CO_PTR_SIZE
    call calloc
    add esp,8
    mov [CORS_PTR_ARR],eax
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
    mov esi, dword[CORS_PTR_ARR]
    mov dword [esi],ebx ;save the pointer on the pointer's Array
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
    mov ebx, [COS_ARR]  ; get second co-routine scheduler
    add ebx, CO_STRUCT_SIZE
    mov esi, [CORS_PTR_ARR]
    mov dword[esi+4],ebx   ;save the pointer on the pointer's Array
    
    mov dword [ebx+CODEP], schedule
    mov dword [ebx+SPP], edx
    mov dword[SPT], esp
    mov esp, [ebx+SPP]
    push schedule
    pushfd
    pushad
    mov dword[ebx+SPP], esp
    mov esp, dword[SPT]
     


    ; target init

 

    ALLOC_STACK
    mov edx, [ebp-4]
    mov ebx, [COS_ARR]  ; get first co-routine aka printer
    add ebx,2* CO_STRUCT_SIZE
    mov esi,dword[CORS_PTR_ARR]
    mov dword[esi+8],ebx ;save the pointer on the pointer's Array
 

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

    mov ebx, [ebp+8]  ; ecx = N

    

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
    mov esi,eax                     ; save the offset in order to find the co-routine later
    mov edi, CO_STRUCT_SIZE
    mul edi              ; eax = (N - ecx) * CO_STRUCT_SIZE
    add ebx,eax       

 
    mov edi,[CORS_PTR_ARR]
    mov [edi+12+(esi*4)],ebx   ;save the pointer on the pointer's Array
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