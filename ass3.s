; main program file

global NUMCO
global CURR
global STKSIZE
global COS_ARR
global CORS_PTR_ARR
global SPT
global SPMAIN
global CO_i
global PRINT_STEPS
global TARGET_POS
global DRONES_ARR
global randomized
global BETA
global DISTANCE
global NUM_HITS


section .rodata
format_input_d: db "%d",0       ; for sscanf
format_print_d: db "%d",10,0    ; for printf
format_print_s: db "%s",10,0    ; for printf
format_input_f: db "%lf",0     ; for sscanf
format_print_f: db "%.2f",10,0  ; for printf
format_print_p: db "%p",10,0  ; for printf
drone_str_format : db "%.2f,%.2f,%.2f,%d",10,0;
newLine : db "\n",10,0;
MAX_INT_16: dd 0xffff


section .bss

randomized resq 1           ; randomized scaled FP
CURR        resd 1          ; current co-rotine pointer
STKSIZE     equ 16*1024     ; drone's stack size - 16kib
COS_ARR     resd 1          ; pointer to drones struct array
CORS_PTR_ARR resd 1         ; array of pointers to cor structs
DRONES_ARR   resd 1         ; array of drone state structs (x,y,α,score)
SPT         resd 1          ; temporary stack pointer
SPMAIN      resd 1          ; stack pointer of main
struc CO_i                  ; drone struct
    CODEP: resd 1
    SPP:   resd 1
endstruc

struc DRONE
    X:     resq 1
    Y:     resq 1
    ALPHA: resq 1
    SCORE: resd 1
endstruc

TARGET_POS:
    tar_X: resq 1
    tar_Y: resq 1

tar_X_offset equ 0
tar_Y_offset equ 8
CO_STRUCT_SIZE equ 8
CO_PTR_SIZE    equ 4
DRONE_STRUC_SIZE equ 28
NUMCO        resd 1            ; N - # of co-routine
NUM_HITS     resd 1            ; T - number of hits to win
PRINT_STEPS  resd 1            ; K - number of steps until printing
BETA         resq 1            ; β - visibility angle
DISTANCE     resq 1            ; d - visibility distance of a drone
SEED         resw 1            ; seed - LFSR's seed
curr_LFSR    resw 1            ; current LFSR's read
offset       resd 1            ;
neg_flag     resd 1            ; tells whether the generated random was negative or not
STK_PTR_ARR resd 1;

section .data
 cur_Number dd 2;
; ------- MACROS: START -------

; [ebp-4] must be available for this macro
%macro ALLOC_STACK 1
    mov dword[ebp-4],%1
    pushad
    push STKSIZE
    call malloc
    mov edx,[STK_PTR_ARR]
    add esp,4
    mov esi,[ebp-4]
    mov [edx+4*esi],eax
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

%macro print_qword_macro 1

    pushad
    push dword [%1+4]
    push dword [%1]
    push format_print_f
    call printf
    add esp,12
    popad

%endmacro

; ------- MACROS: END -------

section .text

align 16
global generateScaled
global generateRandom
global main
global startCo
global startCo.resume;
global startCo.endCo;
extern printf
extern fprintf
extern malloc
extern calloc
extern free
extern sscanf
extern printGameBoard
extern schedule
extern target_func
extern moveDrone
global init_target
extern createTarget

main:

    mov esi, [esp+8]
    push esi
    call getArguments
    add esp,4

    finit             ; initializes FPU stack

    push dword [NUMCO]      
    call alloc_coRotines     ; allocate co-routines memory
    add esp,4
    mov dword [COS_ARR], eax

    mov dx,word [SEED]            ; initializing LFSR with SEED
    mov word [curr_LFSR],dx

    call init_target

     
    pushad
    mov eax,[NUMCO]
    add eax,3
    push eax
    call malloc
    mov [STK_PTR_ARR],eax
    add esp,4
    popad
    
    push dword [NUMCO]
    call init_coRotines      ; initializes co-routines
    add esp,4

    push 1                    ; first we start the scheduler co-routine 
    call startCo
    add esp,4

    call freeMem

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
    ret                ; this will jump to the activated function that serves as "return address"
 
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
generateRandom:
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

; [IN]: two integers, A and B
; [OUT]: generated random scaled to range [A,B]

;           (B-A)*x
; f(x) = -------------- + A   
;            0xffff

generateScaled:
    push ebp
    mov ebp,esp
    pushad

    mov esi, [ebp+8]  ; A
    mov edi, [ebp+12] ; B

    call generateRandom
    mov ebx, eax              ; ebx = x (generated random)
    cmp ebx,0
    jg .do_not_set_flag
    .set_neg_flag:
    mov dword [neg_flag], 1   ; turns on the negative random flag. used to make safe signed addition
    neg ebx
    .do_not_set_flag:
    sub edi,esi       ; edi = B - A
    mov [ebp-4], edi  ; [ebp-4] = B - A
    fild dword [ebp-4]
    mov [ebp-4], ebx
    fimul dword [ebp-4]
    fidiv dword [MAX_INT_16]
    mov [ebp-4], esi
    fiadd dword [ebp-4]
    cmp dword [neg_flag],0
    je .do_not_change_sign
    fchs                             ; change sign
    mov dword [neg_flag], 0          ; inits the neg flag for next time
    .do_not_change_sign:
    fstp qword [randomized]          ; eax = fp of ((B-A)*x)/0xffff)

    popad
    mov esp,ebp
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

    pushad
    sub ebx,3 ; set ebx = N
    push ebx
    push DRONE_STRUC_SIZE
    call calloc
    add esp,8
    mov [DRONES_ARR], eax
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
    push format_input_f
    push ebx
    call sscanf
    add esp,12


    mov dword ebx, [esi+20]

    push DISTANCE
    push format_input_f
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

    ALLOC_STACK 0
    mov edx, [ebp-4]
    mov ebx, [COS_ARR]  ; get second co-routine scheduler
    mov esi, [CORS_PTR_ARR]
    mov dword [esi],ebx   ;save the pointer on the pointer's Array
    
    mov dword [ebx+CODEP], printGameBoard
    mov dword [ebx+SPP], edx
    mov dword[SPT], esp
    mov esp, [ebx+SPP]
    push printGameBoard
    pushfd
    pushad
    mov dword [ebx+SPP], esp
    mov esp, dword[SPT]


    ; scheduler init

    ALLOC_STACK 1
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

    ALLOC_STACK 2
    mov edx, [ebp-4]
    mov ebx, [COS_ARR]  ; get first co-routine aka printer
    add ebx,2* CO_STRUCT_SIZE
    mov esi,dword[CORS_PTR_ARR]
    mov dword[esi+8],ebx ;save the pointer on the pointer's Array
 

    mov dword [ebx+CODEP], createTarget
    mov dword [ebx+SPP], edx
    mov [SPT], esp
    mov esp, [ebx+SPP]
    push createTarget
    pushfd
    pushad
    mov [ebx+SPP], esp
    mov esp, [SPT]


    ; init N drones in a "for loop"

    mov ebx, [ebp+8]  ; ecx = N

    .init_drones:
    mov esi,[cur_Number]
    add esi,1
    mov [cur_Number],esi
    ALLOC_STACK dword esi
    mov ebx, [COS_ARR]  ; get first co-routine aka printer

    ; adding the current drones offset to ebx

    mov eax, CO_STRUCT_SIZE
    mov edi, 3
    mul edi
    add ebx, eax                    ; ebx = ebx + 3*[STRUCT_SIZE]
    mov eax, [ebp+8]
    sub eax,ecx                     ; eax = N - ecx (offset)
    mov esi,eax                     ; save the offset in order to find the co-routine later
    mov [offset],esi
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

    .init_drones_states:

    ; initializing the Drone[i] with initial parmeters for (x,y,alpha,scores)

    mov ebx,[DRONES_ARR]
    mov eax,[offset]  ;ecx holds the number of the drone 0 to n-1
    mov edx, DRONE_STRUC_SIZE
    mul edx            ; edx = DRONE_STRUC_SIZE * offset
    add ebx,eax        ; ebx holds pointer to current drone's struct

    pushad
    mov eax, 100
    mov ebx, 0
    push eax
    push ebx
    call generateScaled
    add esp,8
    popad

    mov esi, dword [randomized+4]
    mov [ebx+4],esi

    mov esi,dword [randomized]
    mov [ebx],esi

    pushad
    mov eax, 100
    mov ebx, 0
    push eax
    push ebx
    call generateScaled
    add esp,8
    popad

    mov esi,dword [randomized+4]
    mov [ebx+12],esi

    mov esi,dword[randomized]
    mov [ebx+8],esi


    pushad
    mov eax, 360
    mov ebx, 0
    push eax
    push ebx
    call generateScaled ;[randomized] holds random number at scale
    add esp,8
    popad
   
    mov esi,dword[randomized+4]
    mov [ebx+20],esi

    mov esi,dword[randomized]
    mov [ebx+16],esi

    mov dword [ebx+24],0  ; initial scores ([ebx+12]) = 0


    dec ecx
    jnz .init_drones
 
    popad
    mov esp,ebp
    pop ebp
    ret


init_target:
    push ebp
    mov ebp,esp
    pushad

    mov eax,100
    mov ebx,0
    push eax 
    push ebx   
    call generateScaled
    add esp,8

    mov esi, dword [randomized+4]
    mov [TARGET_POS+tar_X_offset+4], esi

    mov esi, dword [randomized]
    mov [TARGET_POS+tar_X_offset],esi

    mov eax,100
    mov ebx,0
    push eax 
    push ebx   
    call generateScaled
    add esp,8

    mov esi, dword [randomized+4]
    mov [TARGET_POS+tar_Y_offset+4], esi

    mov esi, dword [randomized]
    mov [TARGET_POS+tar_Y_offset],esi

    popad
    mov esp,ebp
    pop ebp
    ret

freeMem:
    push ebp
    mov ebp,esp
    pushad
    
    mov edx,[STK_PTR_ARR]
    mov edx,[edx]
    push edx
    call free
    add esp,4
        
    mov edx,[STK_PTR_ARR]
    mov edx,[edx+4]
    push edx
    call free
    add esp,4

    mov edx,[STK_PTR_ARR]
    mov edx,[edx+8]
    push edx
    call free
    add esp,4

    mov ecx,0
    .delete_stack:
     mov edx,[STK_PTR_ARR]
     mov edx,[edx+12+4*ecx]
     pushad
     push edx
     call free
     add esp,4
     popad
     add ecx,1
     cmp ecx,[NUMCO]
     jz .cont
     jmp .delete_stack

    .cont:
    push dword [STK_PTR_ARR]
    call free
    add esp,4
    
    push dword [DRONES_ARR]
    call free
    add esp,4

    push dword [CORS_PTR_ARR]
    call free
    add esp,4

    push dword [COS_ARR]
    call free
    add esp,4

    popad
    mov esp,ebp
    pop ebp
    ret
