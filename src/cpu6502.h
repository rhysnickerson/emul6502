//
// Created by rhys on 2021-05-10.
//


#ifndef EMUL6502_CPU6502_H

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#define EMUL6502_CPU6502_H
#define STACK_START 0x0100

typedef enum{
    AM_IMP, //implicit
    AM_ACC, //accumulator
    AM_IMM, //immediate
    AM_ZP, //Zero paged
    AM_ZPX, //zero paged X
    AM_ZPY, //zero paged Y
    AM_REL, //relative
    AM_ABS, //absolute
    AM_ABSX, //absolute X
    AM_ABSY, //absolute Y
    AM_IND, //Indirect
    AM_INDX, //indexed indirect
    AM_INDY //indirect indexed
}ADDR_MD;
typedef enum{
    OP_AND,
    OP_EOR,
    OP_ORA
}LOGICAL_OP;
typedef struct{
    //PROCESSOR FLAG STATES
    /*Addressing modes:
    * Implicit : when source and dest are implicit to the function or not relevant (eg. Clear Carry Flag CLC or Return from Subroutine RTS)
    * Immediate: uses directly specified 8 bit constant within instruction (load LO load HI)
    * Zero Page: uses zero page and 0x0000-0x00ff, from immediate or relevant register
    * Relative : uses 8 bit offset (-127 - 126) which is added to current PC value (used in branches)
    * Absolute : use full 16 bit address to identify target location (used in jumps)
    * Indirect : use 16 bit address , identifies location of least significant byte of another 16 bit memory address, uses that as real target
    */
    uint8_t  C : 1; //carry flag
    uint8_t  Z : 1; //zero flag
    uint8_t  I : 1; //interrupt disable
    uint8_t  D : 1; //decimal mode
    uint8_t  B : 1; //break mode
    uint8_t  V : 1; //overflow flag
    uint8_t  N : 1; //negative flag
}PROC_FLAGS;
typedef struct {

    uint16_t PC; //program counter
    /* 16 bit register points to next instruction to be executed*/

    uint8_t  S; //stack pointer
    /* Points to 256 byte stack from 0x0100 to 0x01ff, stores lowest 8 bits of stack pointer
     * ie. Stack address = 0x0100 || SP */

    uint8_t  A; //accumulator
    /* 8 bit accumulator used in arithmetic and logical operations excluding inc and dec */

    uint8_t  X,Y; //index registers
    /* Index registers used in operations, can be read to/written from memory
     * X register can copy address of SP */
    union{
        PROC_FLAGS f;
        uint8_t v;
    } flags;

}r_6502;

typedef struct{
    uint16_t ROM_HEADER;
    uint8_t MEM[0xFFFF]; // processor memory
    /* 64kB indexed from 0x0000 to 0xffff
     * 0x0100-0x01ff STACK
     * 0x0600-0x0
     *
     */

    r_6502 REG;
}c_6502;
c_6502* c_init(uint16_t);
void c_reset(c_6502*);
void m_reset(c_6502*);
void r_reset(c_6502*);
void c_destroy(c_6502*);
uint8_t fetch_byte(c_6502*);
uint16_t fetch_word(c_6502*);
uint8_t load_byte(c_6502* CPU, uint16_t addr);
uint16_t load_word(c_6502* CPU, uint16_t addr);
void store_byte(c_6502* CPU, uint16_t addr, uint8_t byte);
void store_word(c_6502* CPU, uint16_t addr, uint16_t word);
int execute (c_6502* CPU);



#endif //EMUL6502_CPU6502_H
