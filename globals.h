#include "pico/stdlib.h"
#include <stdlib.h>
#include <stdio.h>
#include "port_common.h"
#include "wizchip_conf.h"
#include "wizchip_spi.h"
#include "dhcp.h"
#include "dns.h"
#include "socket.h"
#include "timer.h"
#include "hardware/watchdog.h"
///#include "tusb.h"

#include <time.h>

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "string.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include <ctype.h>

// WinterHill includes

#include "errors.h"
#include "main.h"
#include "nim.h"
#include "rpi2c.h"
#include "stv0910.h"
#include "stv0910_regs.h"
#include "stv0910_utils.h"
#include "stv6120.h"
#include "stv6120_regs.h"
#include "stv6120_utils.h"
#include "stvvglna.h"
#include "stvvglna_utils.h"

typedef int32_t					int32 ;
typedef signed short			int16 ;
typedef unsigned short			uint16 ;
typedef unsigned char			uint8 ;
typedef char					int8 ;
typedef int64_t					int64 ;
typedef uint32_t				uint32 ; 								///BBB

extern uint32	GLOBALNIM ;												// 1 = NIM_A, 2 = NIM_B
extern bool		nimspresent		[5] ;
extern bool		xlnaspresent	[5] ;

extern void		lmprint 		(char*) ;								// used to intercept print output from LM 

// Raspberry Pi Pico IO assignments

/*
    RX1         NIM_A   TS2
    RX2         NIM_A   TS1
*/

#define NIMDAT_R1           	13										
#define NIMCLK_R1           	14	
#define NIMSS_R1				15
#define EXECPIN_R1 				NIMSS_R1
#define BASEPIN_R1          	NIMDAT_R1	
#define NIMDAT_R1_INDEX			(NIMDAT_R1-BASEPIN_R1)
#define NIMCLK_R1_INDEX			(NIMCLK_R1-BASEPIN_R1)
#define NIMSS_R1_INDEX			(NIMSS_R1-BASEPIN_R1)
#define BASEPIN_R1_INDEX     	(NIMDAT_R1-BASEPIN_R1)	
#define EXECPIN_R1_INDEX    	(EXECPIN_R1-BASEPIN_R1)	

#define NIMDAT_R2           	10									
#define NIMCLK_R2           	11										
#define NIMSS_R2				12										
#define EXECPIN_R2 				NIMSS_R2
#define BASEPIN_R2          	NIMDAT_R2	
#define NIMDAT_R2_INDEX			(NIMDAT_R2-BASEPIN_R2)
#define NIMCLK_R2_INDEX			(NIMCLK_R2-BASEPIN_R2)
#define NIMSS_R2_INDEX			(NIMSS_R2-BASEPIN_R2)
#define BASEPIN_R2_INDEX     	(NIMDAT_R2-BASEPIN_R2)	
#define EXECPIN_R2_INDEX    	(EXECPIN_R2-BASEPIN_R2)	

#define	I2C1DAT					26
#define	I2C1CLK					27
#define ILED					25										// on the Pico board
#define	VGXEN					6
#define	VGXSEL					9
#define	VGYEN					7
#define	VGYSEL					8
#define BOARDRESET  			2
#define PLUS4					28										// increases base ip address by 4
#define REGULATOR_ENABLE		22
#define WIZRESET				20

#define GP7						7
#define GP8						8

#define	GP0						0
#define	GP1						1
#define	SPARE_GP3				3
#define	SPARE_GP4				4
#define	SPARE_GP5				5
