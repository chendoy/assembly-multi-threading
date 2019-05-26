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

        ;mov eax,[steps_counter]
        ;mov ebx, [PRINT_STEPS]
        ;cmp eax,ebx  ;checks if need to print now
        ;jnz .contiune_next_drone ; activate next drone co-routine
        
        ;.printSteps_co:
        ;mov dword [steps_counter],0  ;init steps counter
        ;mov ebx,[CORS_PTR_ARR] ; ebx holds reference to print coroutine now (its index is 0)
        ;call startCo.resume   

        .contiune_next_drone:
        inc dword [steps_counter] ;else inc steps_counter and activate next drone co-routine
        mov edx,esi   
        sub edx,ecx    
        add edx,3
        mov ebx,[CORS_PTR_ARR]
        mov ebx,[ebx+4*edx]; ebx = N-ecx +3
        call startCo.resume

        


    loop .round_robin_schsule, ecx

    
    jmp startCo.endCo   ;ending round, resuming main(). should call it from drone ? after winning ?
