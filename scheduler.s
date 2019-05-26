; scheduler co-rotine, resumed every step and performs round-robin resuming manner

section .rodata

section .bss
extern NUMCO;
extern CURR;
extern PRINT_STEPS
extern CORS_PTR_ARR
extern COS_ARR


section .data
 steps_counter dd 0

section .text
extern startCo.resume
extern startCo.endCo
global schedule

schedule:
    mov esi,[NUMCO] ;esi=NUMCO
    mov ecx,esi ; 
    .round_robin_schsule: ; looping on the number of the co-routines
        

        mov edx,esi   
        sub edx,ecx    
        add edx,3
        mov ebx,[CORS_PTR_ARR]
        mov ebx,[ebx+4*edx]; ebx = N-ecx +3
        call startCo.resume
        inc dword [steps_counter] ; increment steps

        mov eax,[steps_counter]
        mov edi, [PRINT_STEPS]
        cmp eax,edi  ;checks if need to print now
        jnz .done 
        
        .printSteps_co:   ; it's time to print
        mov dword [steps_counter],1  ;init steps counter
        mov ebx,[CORS_PTR_ARR] ; ebx holds reference to print coroutine now (its index is 0)
        call startCo.resume
       
    
        .done:
    loop .round_robin_schsule, ecx

   jmp schedule ; one round has ended, start another one

   ;jmp startCo.endCo

   
