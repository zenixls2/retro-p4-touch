#ifndef RETRO_TOUCH_FATFS_EXFAT_OVERRIDE_H
#define RETRO_TOUCH_FATFS_EXFAT_OVERRIDE_H

#include "ffconf.h"

#undef FF_USE_FASTSEEK
#ifdef CONFIG_FATFS_USE_FASTSEEK
#define FF_USE_FASTSEEK 1
#else
#define FF_USE_FASTSEEK 0
#endif

#undef FF_USE_LABEL
#ifdef CONFIG_FATFS_USE_LABEL
#define FF_USE_LABEL 1
#else
#define FF_USE_LABEL 0
#endif

#undef FF_USE_DYN_BUFFER
#ifdef CONFIG_FATFS_USE_DYN_BUFFERS
#define FF_USE_DYN_BUFFER 1
#else
#define FF_USE_DYN_BUFFER 0
#endif

#undef FF_FS_EXFAT
#define FF_FS_EXFAT 1

#endif
