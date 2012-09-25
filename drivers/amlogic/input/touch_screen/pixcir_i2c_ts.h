#ifndef	_PIXCIR_I2C_TS_H
#define	_PIXCIR_I2C_TS_H

#define	TRUE		1
#define	ATTB_PIN_LOW	0

/*
struct pixcir_i2c_ts_platform {
	int (*attb_read_val) (void);
	int ts_x_max;
	int ts_y_max;
};*/

enum {
    PIXCIR_CALIBRATE_UNKNOW,
    PIXCIR_CALIBRATING,
    PIXCIR_CALIBRATE_SUCCESS,
    PIXCIR_CALIBRATE_FAILED,
};

#define MAX_CALIBRATE_CHECKTIME (20*HZ)

#endif
