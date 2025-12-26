.code
TTC_SwitchContext PROC
    push rbx
    push rbp
    push rdi
    push rsi
    push r12
    push r13
    push r14
    push r15
    sub rsp, 160
    movups [rsp + 0],   xmm6
    movups [rsp + 16],  xmm7
    movups [rsp + 32],  xmm8
    movups [rsp + 48],  xmm9
    movups [rsp + 64],  xmm10
    movups [rsp + 80],  xmm11
    movups [rsp + 96],  xmm12
    movups [rsp + 112], xmm13
    movups [rsp + 128], xmm14
    movups [rsp + 144], xmm15
    mov [rcx], rsp
    mov rsp, rdx
    movups xmm15, [rsp + 144]
    movups xmm14, [rsp + 128]
    movups xmm13, [rsp + 112]
    movups xmm12, [rsp + 96]
    movups xmm11, [rsp + 80]
    movups xmm10, [rsp + 64]
    movups xmm9,  [rsp + 48]
    movups xmm8,  [rsp + 32]
    movups xmm7,  [rsp + 16]
    movups xmm6,  [rsp + 0]
    add rsp, 160
    pop r15
    pop r14
    pop r13
    pop r12
    pop rsi
    pop rdi
    pop rbp
    pop rbx
    ret
TTC_SwitchContext ENDP
END
