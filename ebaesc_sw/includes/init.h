/*
 * init.h
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#ifndef INIT_H_
#define INIT_H_

void InitSysVars(Int16 initial);
UWord32 EEPROMStorei16(UWord32 uw32CurrIndex, Int16 i16Data);
UWord32 EEPROMStoref16(UWord32 uw32CurrIndex, Frac16 f16Data);
UWord32 EEPROMStoreui16(UWord32 uw32CurrIndex, UInt16 ui16Data);
UWord32 EEPROMStorew16(UWord32 uw32CurrIndex, Word16 w16Data);
UWord32 EEPROMReadi16(UWord32 uw32CurrIndex, Int16 *i16Data);
UWord32 EEPROMReadf16(UWord32 uw32CurrIndex, Frac16 *f16Data);
UWord32 EEPROMReadui16(UWord32 uw32CurrIndex, UInt16 *ui16Data);
UWord32 EEPROMReadw16(UWord32 uw32CurrIndex, Word16 *w16Data);
Int16 LoadEEPROM();
Int16 StoreEEPROM();

#endif /* INIT_H_ */
