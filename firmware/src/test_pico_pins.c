#include "pico/stdlib.h"

// Skips GP4 and GP5
#define TEST_MASK ((1u<<2)|(1u<<3)|(1u<<6)|(1u<<7)|(1u<<8)|(1u<<9)| \
                   (1u<<10)|(1u<<11)|(1u<<12)|(1u<<13)|(1u<<14)|(1u<<15)| \
                   (1u<<18)|(1u<<19)|(1u<<20)|(1u<<21))

int main() {
    gpio_init_mask(TEST_MASK);
    gpio_set_dir_out_masked(TEST_MASK);
    while (true) {
        gpio_xor_mask(TEST_MASK);
        sleep_ms(1000);
    }
}
