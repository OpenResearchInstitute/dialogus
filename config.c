

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "config.h"

int config_tx_clock_multiplier = 1; // this isn't really configured (yet?)

long long config_tx_channel_center = 0;
long long config_rx_channel_center = 0;

//!!! put other hardware-specific configuration items here and in the .h file


bool some_configuration_found = false;

static bool handle_config_string(char *cfgstring) {
    char *value;
    char *endptr;
    char cfg[1000];
    strncpy(cfg, cfgstring, 1000);

    char *keyword = strtok(cfg, "=");

    if (keyword != NULL) {
        if (strcasecmp(keyword, "rxfreq") == 0) {
            value = strtok(NULL, "=");
            if (value == NULL) {
                printf("Configuration syntax error: RXFREQ requires a value in Hz, like RXFREQ=905050000 (no spaces)\n");
                return false;
            }
            config_rx_channel_center = strtoll(value, &endptr, 10);
            if ((config_rx_channel_center == LLONG_MAX || config_rx_channel_center == LLONG_MIN) && errno == ERANGE) {
                printf("Configuration value error: RXFREQ must be a valid long long integer. %s is not.\n", value);
                return false;
            } else if (value == endptr) {
                printf("Configuration syntax error: RXFREQ cannot take an empty value.\n");
                return false;
            } else {
                some_configuration_found = true;
            }
        } else if (strcasecmp(keyword, "txfreq") == 0) {
            value = strtok(NULL, "=");
            if (value == NULL) {
                printf("Configuration syntax error: TXFREQ requires a value in Hz, like TXFREQ=905050000 (no spaces)\n");
                return false;
            }
            config_tx_channel_center = strtoll(value, &endptr, 10);
            if ((config_tx_channel_center == LLONG_MAX || config_tx_channel_center == LLONG_MIN) && errno == ERANGE) {
                printf("Configuration value error: TXFREQ must be a valid long long integer. %s is not.\n", value);
                return false;
            } else if (value == endptr) {
                printf("Configuration syntax error: TXFREQ cannot take an empty value.\n");
                return false;
            } else {
                some_configuration_found = true;
            }
        } else if (strcasecmp(keyword, "freq") == 0) {
            value = strtok(NULL, "=");
            if (value == NULL) {
                printf("Configuration syntax error: FREQ requires a value in Hz, like FREQ=905050000 (no spaces)\n");
                return false;
            }
            config_rx_channel_center = strtoll(value, &endptr, 10);
            config_tx_channel_center = config_rx_channel_center;
            if ((config_rx_channel_center == LLONG_MAX || config_rx_channel_center == LLONG_MIN) && errno == ERANGE) {
                printf("Configuration value error: FREQ must be a valid long long integer. %s is not.\n", value);
                return false;
            } else if (value == endptr) {
                printf("Configuration syntax error: FREQ cannot take an empty value.\n");
                return false;
            } else {
                some_configuration_found = true;
            }
        } else {
            printf("Unknown keyword in configuration string: %s\n", keyword);
            return false;
        }
    }

    return true;
}

// Read environment variables to set up configuration of Dialogus.
// Exit the program with helpful messages if this fails.
static void configure_dialogus_from_environment(void) {
    FILE *fp;
    char buf[1000];
    char params[1000];
    bool found = false;
    bool found_params = false;
    
    fp = popen("fw_printenv", "r");
    while (fgets(buf, sizeof(buf), fp) != NULL) {
        if (strncmp(buf, "dialogus=pluto", 14) == 0) {
            printf("CONFIG: Dialogus is configured for ADALM PLUTO hardware.\n");
            found = true;
            if (strlen(buf) > 15) {
                found_params = true;
                strncpy(params, buf+14, 1000);
            }
            //!!! apply fixed Pluto settings here
            break;
        } else if (strncmp(buf, "dialogus=libre", 14) == 0) {
            printf("CONFIG: Dialogus is configured for LibreSDR hardware.\n");
            found = true;
            if (strlen(buf) > 15) {
                found_params = true;
                strncpy(params, buf+14, 1000);
            }
            //!!! apply fixed LibreSDR settings here
            break;
        } 
    }
    pclose(fp);

    if (!found) {
        printf("CONFIG: You must fw_setenv dialogus to your hardware type.\n");
        exit(1);
    }

    if (found_params) {
        char *token = strtok(params, " \n");
        while (token != NULL) {
            if (!handle_config_string(token)) {
                printf("CONFIG: Invalid configuration string in environment: %s\n", token);
                exit(2);
            }
            some_configuration_found = true;
            token = strtok(NULL, " \n");
        }
    }
}


static void configure_dialogus_from_args(int argc, char **argv) {
    for (int i = 1; i < argc; i++) {
        if (!handle_config_string(argv[i])) {
            printf("CONFIG: Invalid configuration string on command line: %s\n", argv[i]);
            exit(3);
        } else {
            some_configuration_found = true;
        }
    }
}

static void configure_dialogus_from_file(void) {
    //!!! don't know how to configure from a file just yet.
}


static void check_dialogus_configuration(void) {

    if (!some_configuration_found) {
        printf("CONFIG: No configuration found in a file, in the environment, or on the command line.\n");
        exit(1);
    }

    if (config_rx_channel_center == 0) {
        printf("CONFIG: No RXFREQ has been set.\n");
        exit(1);
    } else {
        printf("CONFIG: The RXFREQ value is set to %lld\n", config_rx_channel_center);
    }

    if (config_tx_channel_center == 0) {
        printf("CONFIG: No TXFREQ has been set.\n");
        exit(1);
    } else {
        printf("CONFIG: The TXFREQ value is set to %lld\n", config_tx_channel_center);
    }
}

void configure_dialogus(int argc, char **argv) {

    // Order is important. Later config sources override earlier ones.
    
    configure_dialogus_from_file();

    configure_dialogus_from_environment();

    configure_dialogus_from_args(argc, argv);

    check_dialogus_configuration();

    sleep(1);	// Give user a chance to read the configuration before proceeding.
}
