#ifndef __LINUX_ADCTS_KDB_H
#define __LINUX_ADCTS_KDB_H

struct ts_key {
	int code;	/* input key code */
	unsigned char *name;
	int x0;
	int y0;
	int x1;
	int y1;
};

#define TS_KEY_MAX 10

#endif

