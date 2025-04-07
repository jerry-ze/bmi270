/*---------------------------------------------*/
#ifndef RTROBOT_BMI323_H
#define RTROBOT_BMI323_H

#include "bmi2_defs.h"
#include <zephyr/kernel.h>

#define READ_WRITE_LEN UINT8_C(8)

void bmi2_interface_init(struct bmi2_dev* dev, int8_t intf);
void bmi2_error_codes_print_result(int8_t rslt);


#endif //RTROBOT_BMI323_H
