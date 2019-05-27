; drone co-rotine, resumed each time and performs one drone step

extern DRONES_ARR
extern startCo.resume
extern CORS_PTR_ARR
extern CURR
extern randomized
global moveDrone 

section .rodata
format_print_f: db "%.2f",10,0  ; for printf

section .bss
extern currentDrone_index
DRONE_STRUC_SIZE equ 28
curr_alpha resq 1
drone_ptr resd 1
angle_360 resq 1

delta_alpha    resq 1           ; reserved for ∆α
delta_distance resq 1           ; reserved for ∆d

section .text
extern printf
extern generateScaled
moveDrone:
    ;saves the angle 360 label (in qword)
    mov dword[angle_360],360

    ; calculating ∆α and ∆d

    mov eax,60
    mov ebx,0
    push eax
    push ebx
    call generateScaled
    add esp,8
    
    mov esi, dword [randomized+4]
    mov [delta_alpha+4], esi

    mov esi, dword [randomized]
    mov [delta_alpha],esi

    pushad
    push dword [delta_alpha+4]
    push dword [delta_alpha]
    push format_print_f
    call printf
    add esp, 12
    popad

    mov eax,50
    mov ebx,0
    push eax
    push ebx
    call generateScaled
    add esp,8
    
    mov esi, dword [randomized+4]
    mov [delta_distance+4], esi

    mov esi, dword [randomized]
    mov [delta_distance],esi


    ; calculating drone offset and his current α

    mov eax,DRONE_STRUC_SIZE
    mov edx,[currentDrone_index]
    mul edx                 ; eax = offset
    mov ebx,[DRONES_ARR]
    add ebx,eax ; ebx holds curr drone struct
    mov dword [drone_ptr], ebx ; saving current drone struct ptr
    mov edx,dword[ebx+20]
    mov dword[curr_alpha+4],edx
    mov edx,dword[ebx+16]
    mov dword[curr_alpha],edx
    
    ; drone(α) = curr(α) + ∆α

    fild qword [curr_alpha]
    fadd qword [delta_alpha]
    fstp qword [ebx+16]


    mov eax,dword[ebx+16]
    mov edx,dword[angle_360]
    cmp eax,edx
    jg .wraparounding
    jmp .continue
    
    .wraparounding:  ; wraparounding α if needed
    fild qword [ebx+16]
    fsub qword [angle_360]
    fstp qword [ebx+16]

    .continue:



    ;this code should be at the end of the function, it will resume the scheduler
    mov edi,[CORS_PTR_ARR]
    mov ebx,[edi+4]  ; moving to ebx the id of the scheduler co-routine
    push moveDrone
    jmp startCo.resume

    