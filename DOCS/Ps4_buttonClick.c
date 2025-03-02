#include <stdio.h>
#include <stdio.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

bool getClick(uint8_t i, uint8_t temp_1)
{
    i = 7 - i;
    uint8_t prev_Status[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    temp_1 = temp_1 >> i;
    uint8_t status = temp_1 & 0x01;
    if (prev_Status[i] ^ status)
    {
        if (status == 1)
        {
            prev_Status[i] = 1;
            return 1;
        }
        if (status == 0)
        {
            prev_Status[i] = 0;
            return 0;
        }
    }
}
int main()
{
    uint8_t rcv_buf[6] = {1, 45, 68, 181, 69};
    uint8_t button_val = rcv_buf[3];
    uint8_t temp_1 = button_val;
    printf("%d\n", getClick(1, temp_1));
}