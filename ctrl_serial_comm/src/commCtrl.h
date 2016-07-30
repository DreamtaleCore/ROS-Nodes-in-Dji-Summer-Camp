#ifndef COMMCTRL_H
#define COMMCTRL_H

#include "../include/sysHeaders.h"
#include "../include/stdHeaders.h"

int commWrite(string data, string port);

int commRead(string& data, string port);

static int isOpened = 0;
static int fd;

#endif // COMMCTRL_H

