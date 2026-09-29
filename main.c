/**
 * main.c - LifeLink entry point
 *
 * All initialisation and the main loop live in lifelink_init() / lifelink_loop().
 * main() just calls them in order.
 */
#include "app/lifelink/lifelink.h"

int main(void) {
    lifelink_init();
    lifelink_loop();
    return 0; 
}
  