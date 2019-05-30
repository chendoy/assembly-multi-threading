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
format_print_d: db "%d",10,0  ; for printf
drone_winning_str : db "Drone id <%d>: I am a winner",10,0;

section .bss
drone_X_offset equ 0
drone_Y_offset equ 8
drone_Alpha_offset equ 16
drone_Score_offset equ 24
target_X_offset equ 0
target_Y_offset equ 8
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
extern TARGET_POS ; (X2,Y2)
angle_gamma resq 1
Y_differnce resq 1
X_differnce resq 1
extern BETA
extern DISTANCE
extern NUM_HITS
SPP equ 4

section .data
; ------- MACROS: START -------

%macro printWinnerDrone 0
    pushad
    mov esi,dword[currentDrone_index]
    add esi,1
    push esi
    push drone_winning_str
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
extern printf
extern generateScaled
extern generateRandom
extern startCo.endCo
extern init_target
extern createTarget

moveDrone:
    ; initializing constants

    finit

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
    fstp st0
    fstp st0
    ja .wraparound

    fldz                   ; st1
    fld qword [ebx+16]     ; st0
    fcomi
    fstp st0
    fstp st0
    jb .negative_angle

    jmp .alpha_updated
    
    .wraparound:           ; wraparounding α if needed
    fild dword [angle_360]
    fld qword [ebx+16]
    fprem
    fstp st0
    fstp st0
    fstp qword [ebx+16]

    jmp .alpha_updated

    .negative_angle:
    fild dword [angle_360] ; st1
    fld qword [ebx+16]     ; st0
    faddp
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
    faddp
    fstp qword [ebx]

    ; y' = y + ∆y

    fld qword [delta_y]
    fld qword [ebx+8]
    faddp
    fstp qword [ebx+8]

    .corners_wrapping:

    ; x wrapping

    fild dword [one_hundred] ; st1
    fld qword [ebx]          ; st0
    fcomi
    fstp st0
    fstp st0
    ja .above_hundrend_x

    fldz                ; st1
    fld qword [ebx]     ; st0
    fcomi
    fstp st0
    fstp st0
    jb .below_zero_x

    jmp .x_updated

    .above_hundrend_x:

    fild dword [one_hundred]
    fld qword [ebx]
    fprem
    fstp qword [ebx]
    fstp st0
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
    fstp st0
    fstp st0
    ja .above_hundrend_y

    fldz                   ; st1
    fld qword [ebx+8]      ; st0
    fcomi
    fstp st0
    fstp st0
    jb .below_zero_y

    jmp .y_updated

    .above_hundrend_y:

    fild dword [one_hundred]
    fld qword [ebx+8]
    fprem
    fstp qword [ebx+8]
    fstp st0

    jmp .y_updated

    .below_zero_y:
    fild dword [one_hundred]  ; st1
    fld qword [ebx+8]         ; st0
    faddp
    fstp qword [ebx+8]

   
    call mayDestroy ; if eax=1 then can destory ,eax=0 can't destroy
        cmp eax,0
        jnz .destroy_target
        jmp .do_not_destroy
        
    .do_not_destroy:
        jmp .end_fucntion1
            
    .destroy_target:                        ; destory the target
      mov ebx,[drone_ptr]                   ;increse drone's scores
      mov esi,dword [ebx+drone_Score_offset]
      add esi,1 ; esi =  new drone's score
      mov dword [ebx+drone_Score_offset],esi
      mov edi,[NUM_HITS]                    ; edi = NUM_HITS
        cmp esi,edi                         ; cmp drone's scores >= NUM_HITS?
        jge .droneWin_endGame 
        jmp .resume_targetCoroutine 
      
        .droneWin_endGame:
            printWinnerDrone
            call startCo.endCo; end the game retuning to main

        .resume_targetCoroutine:
            push moveDrone
            pushfd
            pushad
            mov edx,[CURR]
            mov [edx+SPP],esp  ;save current esp of drone
            .do_resume_target:
             mov ebx,[CORS_PTR_ARR] 
             mov ebx,[ebx+8] ; target co- routine ptr
             mov esp,[ebx+SPP]  ;esp points now to the co routine stack
             mov [CURR],ebx     ;CURR points now to the struct of the current co-routine
             popad              ;restore resumed co-routine state 
             popfd              ;restore flags
             ret                ; this will jump to the activated function that serves as "return address"


    .y_updated:

    .end_fucntion1:

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

    fld qword [ebp+8]   ; load α
    fldpi               ; load pi
    fmulp
    fild qword [degrees_180]
    fdivp
    fstp qword [alpha_rad]

    popad
    mov esp,ebp
    pop ebp
    ret

    mayDestroy:
    push ebp                  
    mov ebp,esp
    sub esp,4 ; [ebp-4]=0 => can't destory , [ebp-4]=1 => can destroy
    pushad
    mov dword[ebp-4],0 ;  default = can't destroy

 ;<< gamma = arctan2(y2-y1, x2-x1) >>
    
    mov ebx,[drone_ptr]
    fld qword [ebx+drone_Y_offset]                  ; ST(1)=Y1 (drone's Y)  
    fld qword [TARGET_POS+target_Y_offset]          ; ST(0)=Y2 (Target Y)
    fsubp                                           ; ST(0)=Y2-Y1
    fst qword [Y_differnce]                         ; Y_differnce= Y2-Y1

    fld qword [TARGET_POS+target_X_offset]          ; ST(1)=X2 (Target X)
    mov ebx,[drone_ptr]
    fld qword [ebx+drone_X_offset]                  ; ST(0)=X1 (drone's X)  
    fsubp                                           ; ST(0)=X2-X1, ST(1)=Y2-Y1
    fst qword [X_differnce]                         ; X_differnce= X2-X1
    

    fpatan ; ST(1)=Arctan(y2-y1,x2-x1) 
    ;converting the reuslt to degrees from radians ST(1)=pi
    fldpi  ; ST(0)= pi (for converting to degrees)
    fdivp ; ST(0) = Arctan(y2-y1,x2-x1)\pi
    fild qword [degrees_180] ; ST(1)= 180 deg
    fmulp ;ST(0)= Arctan(y2-y1,x2-x1)\pi *180 = ANGLE_GAMMA
    fst qword [angle_gamma]

    ;checking conditions 
    .check_1st_Cond:                                      ;abs(alpha-gamma) < beta) ?
    mov ebx,[drone_ptr] 
    fld qword [ebx+drone_Alpha_offset]                    ;ST(0)=Alpha
    ;fld qword [angle_gamma]                              ;ST(0)=gamma
    fsubp                                                 ;ST(0) = alpha-gamma
    fabs                                                  ; ST(1) = ABS(alpha-gamma)
    fild qword[BETA]                                      ; ST(0) = BETA 
    fcomip                                                ; comapring 
    ja .check_2nd_Cond
    ;free stack
    FSTP ST0
    FSTP ST0
    FSTP ST0
    FSTP ST0
    FSTP ST0
    jmp .end_function ;condition failed

    .check_2nd_Cond:            ; sqrt((y2-y1)^2+(x2-x1)^2) < d
     fld qword [Y_differnce]    ; ST(1) = (Y2-Y1)
     fld qword [Y_differnce]    ; ST(0) = (Y2-Y1)
     FMULP ; ST(1)=(Y2-Y1)^2
     fld qword[X_differnce]     ; ST(0)=(X2-X1)
     fld qword[X_differnce]     ; ST(1)=(X2-X1)
     FMULP                      ; ST(0)=(X2-X1)^2
     faddP                      ; ST(0)=(Y2-Y1)^2 + (X2-X1)^2

     fsqrt ; ST(1)= SQRT ((Y2-Y1)^2 + (X2-X1)^2)
     fild qword [DISTANCE]      ; ST(0)= d
     fcomip 
     ja .can_destory            ; if both conditions implmented, then can destory target
     jmp .end_function
     .can_destory: 
        mov dword[ebp-4],1
    ;free stack
    FSTP ST0
    FSTP ST0
    FSTP ST0
    FSTP ST0
    FSTP ST0
    FSTP ST0

    .end_function:
    popad
    mov eax, [ebp-4]
    mov esp,ebp
    pop ebp
    ret

