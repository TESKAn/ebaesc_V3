/*
 * init.h
 *
 *  Created on: Mar 29, 2015
 *      Author: Jure
 */

#ifndef INIT_H_
#define INIT_H_

void InitSysVars(Int16 initial);
UWord32 EEPROMStorei8(UWord32 uw32CurrIndex, Int8 i8Data);
UWord32 EEPROMStoreui8(UWord32 uw32CurrIndex, UInt8 ui8Data);
UWord32 EEPROMStorei16(UWord32 uw32CurrIndex, Int16 i16Data);
UWord32 EEPROMStoref16(UWord32 uw32CurrIndex, Frac16 f16Data);
UWord32 EEPROMStoreui16(UWord32 uw32CurrIndex, UInt16 ui16Data);
UWord32 EEPROMStorew16(UWord32 uw32CurrIndex, Word16 w16Data);
UWord32 EEPROMStoref(UWord32 uw32CurrIndex, float fData);
UWord32 EEPROMReadi8(UWord32 uw32CurrIndex, Int8 *i8Data);
UWord32 EEPROMReadui8(UWord32 uw32CurrIndex, UInt8 *ui8Data);
UWord32 EEPROMReadi16(UWord32 uw32CurrIndex, Int16 *i16Data);
UWord32 EEPROMReadf16(UWord32 uw32CurrIndex, Frac16 *f16Data);
UWord32 EEPROMReadui16(UWord32 uw32CurrIndex, UInt16 *ui16Data);
UWord32 EEPROMReadw16(UWord32 uw32CurrIndex, Word16 *w16Data);
UWord32 EEPROMReadf(UWord32 uw32CurrIndex, float *fData);
Int16 LoadEEPROM();
Int16 StoreEEPROM();
Int16 CheckEEPROM();

#endif /* INIT_H_ */
