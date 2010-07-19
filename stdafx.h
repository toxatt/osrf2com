
#ifndef STDAFX_H
#define STDAFX_H

//#include <ioavr.h>

#include <intrinsics.h>
//#include <pgmspace.h> /* compiler extra options must be added "--string_literals_in_flash" */

/*****************************************************************************
Types definitions
******************************************************************************/

typedef unsigned char UREG;
typedef signed char REG;

/* byte */
typedef char CHAR;
typedef signed char SCHAR;
typedef unsigned char UCHAR;
typedef signed char INT8;
typedef UCHAR UINT8;
typedef SCHAR SINT8;
#define MAXUINT8	((UINT8)(-1))

/* word */
typedef short INT16;
typedef signed short SINT16;
typedef unsigned short UINT16;
#define MAXUINT16	((UINT16)(-1))

/* long word */
typedef long INT32;
typedef signed long SINT32;
typedef unsigned long UINT32;
#define MAXUINT32	((UINT32)(-1))

/* utils */
/*#define MAKEUINT16(byte_h, byte_l)	((UINT16)(((UINT16)(byte_h) << 8)|(UINT16)(byte_l)))
#define HIBYTE(word)				((UCHAR)((UINT16)(word) >> 8))
#define LOBYTE(word)				((UCHAR)((UINT16)(word) & 0xff))

#define MAKEUINT32(word_h, word_l)	((UINT32)(((UINT32)(word_h) << 16)|(UINT32)(word_l)))
#define HIWORD(longv)				((UINT16)((UINT32)(longv) >> 16))
#define LOWORD(longv)				((UINT16)((UINT32)(longv) & 0xffff))*/

#define SWAP16(n16) ((((UINT16)(n16))<<8)|(((UINT16)(n16))>>8))
/*#define SWAP32(n32)					(MAKEUINT32(SWAP16(LOWORD(n32)), SWAP16(HIWORD(n32))))*/

#define htons(VAR) __reverse((unsigned int)(VAR))
#define htonl(VAR) __reverse((unsigned long)(VAR))
#define ntohs(VAR) __reverse((unsigned int)(VAR))
#define ntohl(VAR) __reverse((unsigned long)(VAR))

#endif//STDAFX_H

