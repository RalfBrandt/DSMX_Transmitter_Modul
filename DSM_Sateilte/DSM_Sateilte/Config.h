#ifndef CONFIG_H
#define CONFIG_H
#include "E2PROM_1.h"

typedef struct
{
BYTE	buf0[16];
BYTE	buf1[16];
}BUFX4;

typedef union
{
BYTE buf[32];
BUFX4 b;
}RAM_D;


#define CONFIG_ID 0x21
#define OPTION_DSMX 	(1<<0)
#define OPTION_11MS 	(1<<1)
#define OPTION_11BIT	(1<<2)

void SaveConfig(BYTE options,BYTE Nr);
BYTE ReadConfig(BYTE Nr);

#endif //CONFIG_H