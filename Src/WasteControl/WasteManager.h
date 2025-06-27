/*
 * WASTEMANAGER.h
 *
 *  Created on: 2021年10月11日
 *      Author: liwenqin
 */

#ifndef SRC_WASTEMANAGER_H_
#define SRC_WASTEMANAGER_H_

#include "Common/Types.h"

void WasteManager_Init(void);
void WasteManager_Cmd(Uint8 cmd, Uint16 reg, Uint16 param);

#endif /* SRC_WASTEMANAGER_H_ */
