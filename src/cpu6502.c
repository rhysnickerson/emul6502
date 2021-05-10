//
// Created by rhys on 2021-05-10.
//

#include "cpu6502.h"
#define flags(c) c->REG.flags.f
#define F_ZERO(v) flags(CPU).Z = (v==0)
#define F_NEG(v) flags(CPU).N = (v>>7)==1

c_6502* c_init(){
    c_6502* CPU = malloc(sizeof(c_6502));
    m_reset(CPU);
    r_reset(&CPU->REG);
    return CPU;
}
void c_reset(c_6502* CPU){
    m_reset(CPU);
    r_reset(&CPU->REG);
}
void c_destroy(c_6502* CPU){
    free(CPU);
}
void m_reset(c_6502* CPU){
    for(uint16_t  i = 0; i < 0xFFFF; i ++) CPU->MEM[i]=0x00;
}
void r_reset(r_6502* REG){
    REG->PC = PROGRAM_START; //TODO: CHANGE TO RESET TO RELAVENT POINT
    REG->A = REG->X = REG->Y = 0x00;
    PROC_FLAGS* flags = &REG->flags.f;
    flags->C = flags->Z = flags->I = flags->D = flags->B = flags->O = flags->N = 0;
}
uint8_t  fetch_byte(c_6502* CPU){
    uint8_t byte = CPU->MEM[CPU->REG.PC];
    CPU->REG.PC++;
    return byte;
}
uint16_t fetch_word(c_6502* CPU){
    uint16_t word = CPU->MEM[CPU->REG.PC];
    CPU->REG.PC++;
    word |= (CPU->MEM[CPU->REG.PC] << 8);
    CPU->REG.PC++;
    return word;
}
uint8_t load_byte(c_6502* CPU, uint16_t addr){
    uint8_t byte = CPU->MEM[addr];
    return byte;
}
uint16_t load_word(c_6502* CPU, uint16_t addr){
    uint8_t lo = CPU->MEM[addr];
    uint8_t hi = CPU->MEM[addr+1];
    return (hi << 8) | lo;
}
void store_byte(c_6502* CPU, uint16_t addr, uint8_t byte){
    CPU->MEM[addr]=byte;
}
void store_word(c_6502* CPU, uint16_t addr, uint16_t word){
    uint8_t lo = word & (0xff);
    uint8_t hi = word >> 8;
    CPU->MEM[addr] = lo;
    CPU->MEM[addr+1] = hi;
}
//Addressing Modes
uint16_t get_addr(c_6502* CPU, ADDR_MD AM){
    switch (AM){
        case (AM_ZP): return fetch_byte(CPU);
        case (AM_ZPX): return (fetch_byte(CPU) + CPU->REG.X);
        case (AM_ZPY): return (fetch_byte(CPU) + CPU->REG.Y);
        case (AM_ABS): return fetch_word(CPU);
        case (AM_ABSX): return fetch_word(CPU) + CPU->REG.X;
        case (AM_ABSY): return fetch_word(CPU) + CPU->REG.Y;
        //TODO: implement indirection
        default:
            return 0x0000; //if implicit, immediate or accumulator no relevant address, default to 0x0000 (this is undefined behaviour)
    }
}
uint8_t get_byte(c_6502* CPU, ADDR_MD AM){
    switch(AM){
        case AM_IMP: return 0x00; //TODO: raise error, this is undefined behaviour - should not access memory with implicit type
        case AM_ACC: return CPU->REG.A;
        case AM_IMM: return fetch_byte(CPU);
        default:
            return load_byte(CPU, get_addr(CPU,AM));

    }
}

/*Generic Load instruction - used to implement LDA, LDX, LDY*/
void gen_LD(c_6502* CPU, uint8_t* REG, ADDR_MD AM){
    uint8_t byte = get_byte(CPU,AM);
    *REG = byte;
    F_NEG(*REG);
    F_ZERO(*REG);
}
//generic store instruction
void gen_ST(c_6502* CPU, uint8_t* REG, ADDR_MD AM){
    uint16_t addr = get_addr(CPU,AM);
    store_byte(CPU,addr,*REG);
}
//generic transfer instruction
void gen_TRANS(c_6502* CPU, uint8_t* REG_D, uint8_t* REG_T){
    *REG_D = *REG_T;
    F_ZERO(*REG_D);
    F_NEG(*REG_D);
}


/*TODO:
 * Using: http://www.obelisk.me.uk/6502/instructions.html
 * Load and Store - add all addressing modes
 * Register transfers DONE
 * Stack operations DONE
 * logical AND EOR ORA BIT
 * arithmetic
 * increments decrements
 * shifts
 * jumps & calls
 * branches
 * status flag calls
 * system functions
 */
int execute(c_6502* CPU){
    uint8_t instr = fetch_byte(CPU);

    switch (instr) {
        /* LDA - Load Accumulator */
        case 0xA9: //LDA_I - Load accumulator immediate
            gen_LD(CPU,&CPU->REG.A, AM_IMM);
            break;
        case 0xA5: // zero paged
            gen_LD(CPU,&CPU->REG.A, AM_ZP);
            break;
        case 0xB5: //zero paged X
            gen_LD(CPU,&CPU->REG.A,AM_ZPX);
            break;
        case 0xAD:  //absolute
            gen_LD(CPU,&CPU->REG.A, AM_ABS);
            break;
        case 0xBD: //absolute X
            gen_LD(CPU,&CPU->REG.A,AM_ABSX);
            break;
        case 0xB9: //absolute Y
            gen_LD(CPU,&CPU->REG.A,AM_ABSY);
            break;

        /*LDX - Load X */
        case 0xA2: //zero paged
            gen_LD(CPU,&CPU->REG.X,AM_IMM);
            break;
        case 0xA6:  // zero paged
            gen_LD(CPU,&CPU->REG.X, AM_ZP);
            break;
        case 0xB6: //zero paged Y
            gen_LD(CPU,&CPU->REG.X,AM_ZPY);
            break;
        case 0xAE:  //absolute
            gen_LD(CPU,&CPU->REG.X, AM_ABS);
            break;
        case 0xBE: //absolute Y
            gen_LD(CPU,&CPU->REG.X,AM_ABSY);
            break;
        /*LDY - Load Y */
        case 0xA0: //zero paged
            gen_LD(CPU,&CPU->REG.Y,AM_IMM);
            break;
        case 0xA4:  // zero paged
            gen_LD(CPU,&CPU->REG.Y, AM_ZP);
            break;
        case 0xB4:  // zero paged X
            gen_LD(CPU,&CPU->REG.Y, AM_ZPX);
            break;
        case 0xAC:  //absolute
            gen_LD(CPU,&CPU->REG.Y, AM_ABS);
            break;
        case 0xBC: //absolute X
            gen_LD(CPU,&CPU->REG.Y,AM_ABSX);
            break;


        /* STA - Store Accumulator */
        case 0x85: //zero paged
            gen_ST(CPU, &CPU->REG.A, AM_ZP);
            break;
        case 0x95: //zero paged X
            gen_ST(CPU, &CPU->REG.A, AM_ZPX);
            break;
        case  0x8D: //absolute address
            gen_ST(CPU, &CPU->REG.A, AM_ABS);
            break;
        case  0x9D: //absolute address X
            gen_ST(CPU, &CPU->REG.A, AM_ABSX);
            break;
        case  0x99: //absolute address Y
            gen_ST(CPU, &CPU->REG.A, AM_ABSY);
            break;

        /* STX - Store Accumulator */
        case 0x86: //zero paged
            gen_ST(CPU, &CPU->REG.X, AM_ZP);
            break;
        case 0x96: //zero paged Y
            gen_ST(CPU, &CPU->REG.X, AM_ZPY);
            break;
        case  0x8E: //absolute address
            gen_ST(CPU, &CPU->REG.X, AM_ABS);
            break;
        /* STY - Store Accumulator */
        case 0x84: //zero paged
            gen_ST(CPU, &CPU->REG.Y, AM_ZP);
            break;
        case 0x94: //zero paged X
            gen_ST(CPU, &CPU->REG.Y, AM_ZPX);
            break;
        case  0x8C: //absolute address
            gen_ST(CPU, &CPU->REG.Y, AM_ABS);
            break;


        /*TAX - transfer acc to X */
        case 0xAA:
            gen_TRANS(CPU, &CPU->REG.X, &CPU->REG.A);
            break;
        /*TAY - transfer acc to Y */
        case 0xA8:
            gen_TRANS(CPU, &CPU->REG.Y, &CPU->REG.A);
            break;
        /*TSX - transfer stack to X */
        case 0xBA:
            gen_TRANS(CPU, &CPU->REG.X, &CPU->REG.S);
            break;
        /*TXA - transfer X to acc */
        case 0x8A:
            gen_TRANS(CPU, &CPU->REG.A, &CPU->REG.X);
            break;
        /*TXS - transfer X to stack */
        case 0x9A:
            gen_TRANS(CPU, &CPU->REG.S, &CPU->REG.X);
            break;
        /*TYA - transfer Y to acc */
        case 0x98:
            gen_TRANS(CPU, &CPU->REG.A, &CPU->REG.Y);
            break;

       /*PHA - push accumulator */
        case 0x48:
            CPU->REG.S --;
            store_byte(CPU, (0x0100 | CPU->REG.S), CPU->REG.A);
            break;
        /*PHP - push processor status */
        case 0x08:
            CPU->REG.S --;
            store_byte(CPU, (0x0100 | CPU->REG.S), CPU->REG.flags.v);
            break;
        /*PLA - pul accumulator */
        case 0x68:
            CPU->REG.A = load_byte(CPU, CPU->REG.S);
            CPU->REG.S ++;
            F_ZERO(CPU->REG.A);
            F_NEG(CPU->REG.A);
            break;
        /*PLP - pull processor status */
        case 0x28:
            CPU->REG.flags.v = load_byte(CPU, CPU->REG.S);
            CPU->REG.S ++;
            break;
        default:
            return 1;
    }
    return 0;
}
