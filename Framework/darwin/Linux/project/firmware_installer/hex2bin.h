#ifndef _HEX2BIN_H_
#define _HEX2BIN_H_


#ifdef __cplusplus
extern "C" {
#endif

#define MEMORY_MAXSIZE		256*1024		/* size in bytes */
bool hex2bin(char *hexFile, unsigned char *pBinBuffer, long *StartAddress, long *pBufSize);

#ifdef __cplusplus
}
#endif


#endif
