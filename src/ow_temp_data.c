/*
 * DallasSensorData.cpp
 *
 *  Created on: 28 вер. 2015 р.
 *      Author: sd
 */

#include "ow_temp_data.h"

#include <stdio.h>

const char *ow_temp_data_get_temp_str(double *_temp) {
    static char buff[8];

    sprintf(buff, "%3.1f", _temp);
    return buff;
}
