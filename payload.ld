SECTIONS
{
    .payload16 0 : {
        guest16 = .;
        guest16.o(.text)
        guest16_end = .;
    }
}
