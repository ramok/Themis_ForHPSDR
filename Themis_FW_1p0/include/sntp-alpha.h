// this is taken from NTP distribution and is included here for test purposes only.
// SNTP and NTP use the same packet format.
// 


#ifndef NTP_H
#define NTP_H

#include "FreeRTOS.h"
#include "inc/hw_types.h"
#include <stddef.h>
#include <math.h>

typedef unsigned int u_int32;
typedef unsigned char u_char;
typedef signed char s_char;


/*
 * NTP uses two fixed point formats.  The first (l_fp) is the "long"
 * format and is 64 bits long with the decimal between bits 31 and 32.
 * This is used for time stamps in the NTP packet header (in network
 * byte order) and for internal computations of offsets (in local host
 * byte order). We use the same structure for both signed and unsigned
 * values, which is a big hack but saves rewriting all the operators
 * twice. Just to confuse this, we also sometimes just carry the
 * fractional part in calculations, in both signed and unsigned forms.
 * Anyway, an l_fp looks like:
 *
 *    0			  1		      2			  3
 *    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |			       Integral Part			     |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |			       Fractional Part			     |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 */
typedef struct {
	union {
		u_int32 Xl_ui;
		int32 Xl_i;
	} Ul_i;
	u_int32	l_uf;
} l_fp;

/*
 * The second fixed point format is 32 bits, with the decimal between
 * bits 15 and 16.  There is a signed version (s_fp) and an unsigned
 * version (u_fp).  This is used to represent synchronizing distance
 * and synchronizing dispersion in the NTP packet header (again, in
 * network byte order) and internally to hold both distance and
 * dispersion values (in local byte order).  In network byte order
 * it looks like:
 *
 *    0			  1		      2			  3
 *    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *   |		  Integer Part	     |	   Fraction Part	     |
 *   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 *
 */
typedef int32 s_fp;
typedef u_int32 u_fp;


/*
 * NTP packet format.  The mac field is optional.  It isn't really
 * an l_fp either, but for now declaring it that way is convenient.
 * See Appendix A in the specification.
 *
 * Note that all u_fp and l_fp values arrive in network byte order
 * and must be converted (except the mac, which isn't, really).
 */
typedef struct pkt {
	u_char	li_vn_mode;	/* peer leap indicator */
	u_char	stratum;	/* peer stratum */
	u_char	ppoll;		/* peer poll interval */
	s_char	precision;	/* peer clock precision */
	u_fp	rootdelay;	/* roundtrip delay to primary source */
	u_fp	rootdisp;	/* dispersion to primary source*/
	u_int32	refid;		/* reference id */
	l_fp	reftime;	/* last update time */
	l_fp	org;		/* originate time stamp */
	l_fp	rec;		/* receive time stamp */
	l_fp	xmt;		/* transmit time stamp */

#define	MIN_V4_PKT_LEN	(12 * sizeof(u_int32))	/* min header length */
#define	LEN_PKT_NOMAC	(12 * sizeof(u_int32))	/* min header length */
#define	MIN_MAC_LEN	(1 * sizeof(u_int32))	/* crypto_NAK */
#define	MAX_MD5_LEN	(5 * sizeof(u_int32))	/* MD5 */
#define	MAX_MAC_LEN	(6 * sizeof(u_int32))	/* SHA */
#define	KEY_MAC_LEN	sizeof(u_int32)		/* key ID in MAC */
#define	MAX_MDG_LEN	(MAX_MAC_LEN-KEY_MAC_LEN) /* max. digest len */

	/*
	 * The length of the packet less MAC must be a multiple of 64
	 * with an RSA modulus and Diffie-Hellman prime of 256 octets
	 * and maximum host name of 128 octets, the maximum autokey
	 * command is 152 octets and maximum autokey response is 460
	 * octets. A packet can contain no more than one command and one
	 * response, so the maximum total extension field length is 864
	 * octets. But, to handle humungus certificates, the bank must
	 * be broke.
	 *
	 * The different definitions of the 'exten' field are here for
	 * the benefit of applications that want to send a packet from
	 * an auto variable in the stack - not using the AUTOKEY version
	 * saves 2KB of stack space. The receive buffer should ALWAYS be
	 * big enough to hold a full extended packet if the extension
	 * fields have to be parsed or skipped.
	 */
#ifdef AUTOKEY
	u_int32	exten[(NTP_MAXEXTEN + MAX_MAC_LEN) / sizeof(u_int32)];
#else	/* !AUTOKEY follows */
	u_int32	exten[(MAX_MAC_LEN) / sizeof(u_int32)];
#endif	/* !AUTOKEY */
} ntp_pkt;

#endif

// *eof
