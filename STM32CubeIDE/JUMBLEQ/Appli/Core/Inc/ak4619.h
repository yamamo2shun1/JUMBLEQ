/*
 * ak4619.h
 *
 *  Created on: 2026/01/24
 *      Author: shuni
 */

#ifndef INC_AK4619_H_
#define INC_AK4619_H_

#include <stdint.h>

#define AK4619_MIC_GAIN_CH1   0U
#define AK4619_MIC_GAIN_CH2   1U
#define AK4619_MIC_GAIN_DB_0  0U
#define AK4619_MIC_GAIN_DB_27 27U

void ak4619_init(uint32_t hz);

void ak4619_set_mic_amp_gain(uint8_t ch, uint8_t gain_db);

#endif /* INC_AK4619_H_ */
