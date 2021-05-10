#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "cpu6502.h"

void readProgram(char* file, c_6502* CPU, uint16_t addr){ //TODO: transfer to external file
    FILE* f = fopen(file,"r");
    if (f == NULL){
        printf("Cannot find file");
        exit(1);
    }
    char val[2] = "";
    int i = 0;
    char c;
    while(!feof(f)){
        c = fgetc(f);
        if((c >= '0' && c<='9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <='F')){
           val[i] = c;
           i++;
           if(i >= 2){
               CPU->MEM[addr]=strtol(val,NULL,16);
               addr++;
               i = 0;
           }
        }
    }
}
//TODO: proper file loading
int main(int argv, char* argc[]){
    c_6502* CPU = c_init();
    readProgram("/home/rhys/CLionProjects/emul6502/src/program",CPU,0x0000);
    while(execute(CPU)!=1);
    c_destroy(CPU);

}