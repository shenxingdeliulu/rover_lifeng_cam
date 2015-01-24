#ifndef GP2D12_H
#define GP2D12_H

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/fs.h>
#include <errno.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>

#include "settings.h"

// #define INFRARED_M  (0.0000243f)
// #define INFRARED_B  (-0.00042f)
// #define INFRARED_K  (4.0f)

extern pthread_mutex_t lock_read_gp2d12;
extern pthread_cond_t ready_read_gp2d12;

extern char flag_read_gp2d12;

int ir_init();
void  *read_ir();

#endif