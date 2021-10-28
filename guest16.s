        .code16
        .global code16, code16_end
guest16:
        movb $'H', %al
        outb %al, $0x01
        movb $'e', %al
        outb %al, $0x01
        movb $'l', %al
        outb %al, $0x01
        movb $'l', %al
        outb %al, $0x01
        movb $'o', %al
        outb %al, $0x01
        movb $',', %al
        outb %al, $0x01
        movb $' ', %al
        outb %al, $0x01
        movb $'w', %al
        outb %al, $0x01
        movb $'o', %al
        outb %al, $0x01
        movb $'r', %al
        outb %al, $0x01
        movb $'l', %al
        outb %al, $0x01
        movb $'d', %al
        outb %al, $0x01
        movb $'.', %al
        outb %al, $0x01
        movb $'\n', %al
        outb %al, $0x01
        hlt
guest16_end:
