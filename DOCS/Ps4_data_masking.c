#include <stdio.h>
#include <stdio.h>
#include <stdint.h>

int main()
{
    uint8_t arr[6] = {69, 20, -124, 250, 32};
    uint8_t temp_1 = arr[3];
    uint8_t temp_2 = arr[4];

    uint8_t status = 0;
    uint8_t i;

    for (int i = 0; i < 8; i++)
    {
        status = temp_1 & (0x01);
        printf("The bit at %d position is %d\n", i, status);
        temp_1 = (temp_1 >> 1);
    }
    return 0;
}

uint8_t getStatus(uint8_t i, uint8_t temp_1)
{
    temp_1 = temp_1 >> i;
    uint8_t status = temp_1 & 0x01;
    return 0;
}