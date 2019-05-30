; drone co-rotine, resumed each time and performs one drone step

extern DRONES_ARR
extern startCo.resume
extern CORS_PTR_ARR
extern CURR
extern randomized
global moveDrone 

section .rodata
format_print_f: db "%.2f",10,0  ; for printf
format_print_e: db "%e",10,0  ; for printf

section .bss
extern currentDrone_index
DRONE_STRUC_SIZE equ 28
curr_alpha     resq 1
drone_ptr      resd 1
angle_360      resd 1
delta_alpha    resq 1           ; reserved for ∆α
delta_distance resq 1           ; reserved for ∆d
delta_x        resq 1           ; reserved for ∆x
delta_y        resq 1           ; reserved for ∆y
degrees_180    resq 1           ; reserved for 180 degrees constant
alpha_rad      resq 1           ; reserved for α conversion to radians
one_hundred    resd 1           ; reserved for 100 constant

section .data
; ------- MACROS: START -------

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
extern printf
extern generateScaled
extern generateRandom
moveDrone:
    ; initializing constants

    mov dword [angle_360],360
    mov dword [degrees_180],180
    mov dword [one_hundred],100

    ; calculating ∆α and ∆d


    mov eax,60
    mov ebx,-60
    push eax
    push ebx
    call generateScaled
    add esp,8

    mov esi, [randomized+4]
    mov [delta_alpha+4], esi

    mov esi, [randomized]
    mov [delta_alpha],esi

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

    .debug:
    
    ; drone(α) = curr(α) + ∆α

    fld qword [curr_alpha]
    fadd qword [delta_alpha]
    fstp qword [ebx+16]      ; extracting from x87 to the right offset of the drone

    fild dword [angle_360] ; st1
    fld qword [ebx+16]     ; st0
    fcomi
    fstp
    ja .wraparound

    fldz                   ; st1
    fld qword [ebx+16]     ; st0
    fcomi
    fstp
    jb .negative_angle

    jmp .alpha_updated
    
    .wraparound:           ; wraparounding α if needed
    fild dword [angle_360]
    fld qword [ebx+16]
    fprem
    fstp qword [ebx+16]

    jmp .alpha_updated

    .negative_angle:
    fild dword [angle_360] ; st1
    fld qword [ebx+16]     ; st0
    fadd
    fstp qword [ebx+16]

    .alpha_updated:

     ; convering updated α to radians first
     push dword [ebx+20]
     push dword [ebx+16]
     call toRadians
     add esp,8

    ; ∆x = cos(α) * ∆d

    fld qword [alpha_rad]        ; loading x
    fcos
    fld qword [delta_distance]
    fmulp
    fstp qword [delta_x]

    ;print_qword_macro delta_x

    ; ∆y = sin(α) * ∆d

    fld qword [alpha_rad]        ; loading y
    fsin
    fld qword [delta_distance]
    fmulp
    fstp qword [delta_y]

    ; x' = x + ∆x

    fld qword [delta_x]
    fld qword [ebx]
    fadd
    fstp qword [ebx]

    ; y' = y + ∆y

    fld qword [delta_y]
    fld qword [ebx+8]
    fadd
    fstp qword [ebx+8]

    .corners_wrapping:

    finit

    ; x wrapping

    fild dword [one_hundred] ; st1
    fld qword [ebx]          ; st0
    fcomiq
    fstp
    ja .above_hundrend_x

    fldz                ; st1
    fld qword [ebx]     ; st0
    fcomi
    jb .below_zero_x

    jmp .x_updated

    .above_hundrend_x:

    fild dword [one_hundred]
    fld qword [ebx]
    fprem
    fstp qword [ebx]
    jmp .x_updated

    .below_zero_x:
    fild dword [one_hundred] ; st1
    fld qword [ebx]          ; st0
    faddp
    fstp qword [ebx]
    jmp .x_updated

    .x_updated:

    ; y wrapping

    fild dword [one_hundred] ; st1
    fld qword [ebx+8]     ; st0
    fcomi
    ja .above_hundrend_y

    fldz                   ; st1
    fld qword [ebx+8]      ; st0
    fcomi
    fstp
    ;fstp
    jb .below_zero_y

    jmp .x_updated

    .above_hundrend_y:

    fild dword [one_hundred]
    fld qword [ebx+8]
    fprem
    fstp qword [ebx+8]
    fstp
    jmp .x_updated

    .below_zero_y:
    fild dword [one_hundred]  ; st1
    fld qword [ebx+8]         ; st0
    faddp
    fstp qword [ebx+8]

    .y_updated:

    ;this code should be at the end of the function, it will resume the scheduler
    mov edi,[CORS_PTR_ARR]
    mov ebx,[edi+4]  ; moving to ebx the id of the scheduler co-routine
    push moveDrone
    jmp startCo.resume

; [IN]: α(degrees) in qword
; [OUT]: α(radians) in qword
toRadians:
    push ebp
    mov ebp,esp
    pushad

    fld qword [ebp+8]  ; load α
    fldpi              ; load pi
    fmulp
    fild qword [degrees_180]
    fdivp
    fstp qword [alpha_rad]
    fstp

    popad
    mov esp,ebp
    pop ebp
    ret

    