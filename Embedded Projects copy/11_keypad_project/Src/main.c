/*
 * note: Embedded-C courses this belongs to. Keypad project. 
 * 	First course in embedded-C udemy courses.
 */

#include<stdint.h>
#include<stdio.h>
#include<stdlib.h>

void delay(void)
{
	for(uint32_t i =0 ; i < 300000 ; i++);

}

int main(void)
{
	//peripheral register addresses
	uint32_t volatile *const pGPIODModeReg  =  (uint32_t*)(0x40020C00);
	uint32_t volatile *const pInPutDataReg  =  (uint32_t*)(0x40020C00+0x10);
	uint32_t volatile *const pOutPutDataReg =  (uint32_t*)(0x40020C00+0x14);
	uint32_t volatile *const pClockCtrlReg  =  (uint32_t*)(0x40023800+0x30);
	uint32_t volatile *const pPullupDownReg =  (uint32_t*)(0x40020C00 + 0x0C);

	*pClockCtrlReg |= ( 1 << 3);

	*pGPIODModeReg &= ~(0xFF); 
	*pGPIODModeReg |= 0x55;   


    *pGPIODModeReg &= ~(0xFF << 16);

    *pPullupDownReg &= ~(0xFF << 16);
    *pPullupDownReg |=  (0x55 << 16);

while(1)
{
    *pOutPutDataReg |= 0x0f;

    *pOutPutDataReg &= ~( 1 << 0);

    if(!(*pInPutDataReg & ( 1 << 8))){
    	delay();
    	printf("1\n");
    }

    if(!(*pInPutDataReg & ( 1 << 9))){
    	delay();
    	printf("2\n");
    }

    if(!(*pInPutDataReg & ( 1 << 10))){
    	delay();
    	printf("3\n");
    }

    if(!(*pInPutDataReg & ( 1 << 11))){
    	delay();
    	printf("A\n");
    }


     *pOutPutDataReg |= 0x0f;
    *pOutPutDataReg &= ~( 1 << 1);

    if(!(*pInPutDataReg & ( 1 << 8))){
    	delay();
    	printf("4\n");
    }

    if(!(*pInPutDataReg & ( 1 << 9))){
    	delay();
    	printf("5\n");
    }

    if(!(*pInPutDataReg & ( 1 << 10))){
    	delay();
    	printf("6\n");
    }

    if(!(*pInPutDataReg & ( 1 << 11))){
    	delay();
    	printf("B\n");
    }

     *pOutPutDataReg |= 0x0f;
    *pOutPutDataReg &= ~( 1 << 2);

    if(!(*pInPutDataReg & ( 1 << 8))){
    	delay();
    	printf("7\n");
    }

    if(!(*pInPutDataReg & ( 1 << 9))){
    	delay();
    	printf("8\n");
    }

    if(!(*pInPutDataReg & ( 1 << 10))){
    	delay();
    	printf("9\n");
    }

    if(!(*pInPutDataReg & ( 1 << 11))){
    	delay();
    	printf("C\n");
    }

     *pOutPutDataReg |= 0x0f;
    *pOutPutDataReg &= ~( 1 << 3);

    if(!(*pInPutDataReg & ( 1 << 8))){
    	delay();
    	printf("*\n");
    }

    if(!(*pInPutDataReg & ( 1 << 9))){
    	delay();
    	printf("0\n");
    }

    if(!(*pInPutDataReg & ( 1 << 10))){
    	delay();
    	printf("#\n");
    }

    if(!(*pInPutDataReg & ( 1 << 11))){
    	delay();
    	printf("D\n");
    }

}

}
