

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "config.h"

int config_tx_clock_multiplier = 1; // this isn't really configured (yet?)
//!!! put other hardware-specific configuration items here and in the .h file

// Read environment variables to set up configuration of Dialogus.
// Exit the program with helpful messages if this fails.
void configure_dialogus(void) {
    FILE *fp;
    char buf[1000];
    bool found = false;
    
    fp = popen("fw_printenv", "r");
    while (fgets(buf, sizeof(buf), fp) != NULL) {
        if (strncmp(buf, "dialogus=pluto", 14) == 0) {
            printf("Dialogus is configured for ADALM PLUTO hardware.\n");
            found = true;
            //!!! apply Pluto settings here
            break;
        } else if (strncmp(buf, "dialogus=libre", 14) == 0) {
            printf("Dialogus is configured for LibreSDR hardware.\n");
            found = true;
            //!!! apply LibreSDR settings here
            break;
        } 
    }
    pclose(fp);

    if (!found) {
        printf("You must fw_setenv dialogus to your hardware type.\n");
        exit(1);
    }

	sleep(1);	// Give user a chance to read the configuration before proceeding.
}

