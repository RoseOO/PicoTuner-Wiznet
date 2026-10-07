#/* ================================================================================================== */
/* picotunewh.c                                                                                       */
/* Derived from the WinterHill receiver main.c                                                        */
/*    - an implementation of the Serit NIM controlling software for the G4EML PicoTouner Hardware     */
/* Copyright 2020, 2024 Brian Jordan, G4EWJ                                                           */
/* ================================================================================================== */
/* 
	PicoTuneWH conversion
	
    This file is part of PicoTuneWH

    WinterHill is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    WinterHill is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with WinterHill.  If not, see <https://www.gnu.org/licenses/>.

	This software is based on LongMynd, copyright 2019 by Heather Lomond, M0HMO
	Modified by Brian Jordan, G4EWJ, 2020 for the WinterHill dual serial NIM project.
	Modified by Brian Jordan, G4EWJ, 2024 for the PicoTuner dual serial NIM project.
*/

/*
// =======================================================================================================

	General Operation
	-----------------

	Built with the WIZnet version of the Raspberry Pi Pico SDK

	Receiver parameters are held in struct rxcontrol: rcv[1], rcv[2]

	struct packetblock holds 7 TS packets and other data
	packetblocks are organised into linked chains
		freechain for available packetblocks
		the DMA routines add packetblocks to rcv[x].rxchain to await processing and Ethernet transmission
		packetblocks are also used for inter core communication

	Core 0 handles all initialisation (including I2C usage). Afer initialisation:
	 
	Core 0:
		Ethernet comms
		TS input using 2 ping-pong DMA controllers per receiver inputting 7 packets at a time
	
 	Core 1:
 		All I2C usage
 		Tuning command parsing
		NIM control
		NIM status and info stream generation
		
	Inter core communication:
		Ethernet tuning requests are received by Core0 and passed to Core1 in tur01e (struct packetblock)
		Info stream data is passed from Core1 to Core0 in esr10 (struct packetblock) for Ethernet tx

	inicommand_loop, tsproc_loop, info_loop were originally threads in WinterHill
	inicommand_loop is called by the initialisation code in Core0 to pass initial tuning commands to Core1
	tsproc_loop is called by Core0 mainloop to process TS data from the rxchains
	info_loop is called by Core1 to get NIM status and build info streams

	LongMynd modules call lmprint, which is currently patched out

// =======================================================================================================
*/
		
#define VERSIONX	"0v3" 		// main version ID
#define VERSIONX2	"e"			// sub version ID

#define CR		13
#define LF		10
#define TAB		9
#define OFF		0
#define ON		1

#include "globals.h"
#include "pico/unique_id.h"
#include "picotunewh.pio.h"
#include "errno.h"
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"
#include "hardware/watchdog.h"
#include "pico/bootrom.h"

/*
    Default operation for baseipport = 9900 
    baseipport can be changed to any even number xx00 - xx14
    
    Port    Direction   Function
    ----------------------------------------------------------------------
    9901    OUT         send text info stream for all receivers
	9902	OUT			send 2 line rx status info 
    9903    OUT         same as 9901
	9904	OUT			same as 9902
    9921    IN          listen for QuickTune commands for receiver 1         
    9922    IN          listen for QuickTune commands for receiver 2        
    9941    OUT         send transport stream for receiver 1    
    9942    OUT         send transport stream for receiver 2    

	The WH form of tuning commands may be used:
	
		[to@wh] freq=10491500 offset=9750000 srate=1500 fplug=a rcv=1 (to port 9921)
		[to@wh] freq=10491500 offset=9750000 srate=1500 fplug=a rcv=2 (to port 9922)

		If the base IP port has been changed to 9904:

			[to@wh] freq=10491500 offset=9750000 srate=1500 fplug=a rcv=5 (to port 9925)
			[to@wh] freq=10491500 offset=9750000 srate=1500 fplug=a rcv=6 (to port 9926)

		WH form commands may also sent to port 9920, irrespective of the base IP port setting.

	Other commands that may be sent to port 9920, irrespective of the base IP port setting:

        [to@wh] reboot=147		reset any changed settings and reboot
        [to@wh] reboot=258		reboot
        [to@wh] bootsel=369		go into BOOTSEL program loading mode

		These settings will survive a reboot, but are lost at power down:
	
    	    [to@wh] bip=9908	set the base IP port - PT will reboot
        	[to@wh] tsflash=0	activity LEDs are steady on, when receiving a signal

*/ 

#define EIT_PID				18
#define ESC					27
#define EVENTID				0						// for EIT
#define INFOPERIOD      	500 	               	// time in ms between info outputs 
#define MAXFREQ				2600000
#define MAXFREQSTOSCAN		16
#define MAXINFOS			100
#define MAXINICOMMANDS		16						// maximum number of commands in the ini file
#define MAXPACKETBLOCKS		128
#define MAXPIDS				8						// numbers of allowed pids for a program
#define MAXSRSTOSCAN		16						// number of SRs to scan
#define	MAXNIMS				1
#define MAXRECEIVERS       	(MAXNIMS * 2)
#define MINFREQ				144000
#define MAXSR				45000
#define MINSR				25
#define NETWORK				0						// not needed by VLC for EIT
#define NULL_PID			8191
#define NULL2_PID			8190					// fake null packet insert by some modulators
#define	ON					1
#define OFF					0
#define PAT_PID				0
#define PORTINFOLMEX		1						// textual status for all receivers
#define PORTINFOMULTIRX		2						// 4 line receiver summary
#define PORTINFOLMEX2		3						// copy of 1
#define PORTINFOMULTIRX2	4						// copy of 2
#define PORTINFOLMBASE		60						// LongMynd textual status for receivers
#define PORTLISTENBASE		20						// listen for receive commands on this + RX number (1-4)
#define PORTTSBASE			40						// output TS to this + RX number
#define PORTBROADCAST		9997
#define PORTMULTICAST		9998
#define QO100NO				0
#define QO100BAND			1
#define QO100BEACON			2
#define QTHEADER			1						// QuickTune command header seen
#define SDT_PID				17
#define SERVICE_H262		0x02		
#define SERVICE_H264		0x1b
#define SERVICE_H265		0x24
#define SERVICE_H266		0x33
#define SERVICE_MPA			0x03			
#define SERVICE_AAC			0x0f
#define SERVICE_AC3			0x77	// ???
#define TSID				0						// not needed by VLC for EIT
#define DAY0 				0xc957 					// 31 December 1999 in Julian days
#define WHHEADER			2						// WinterHill command header seen

#define MODE_ANYWHERE		0						// TS is sent to where the command came from
#define MODE_MULTICAST		1						// TS is sent to the multicast address
#define MODE_LOCAL			2						// TS is sent to the address supplied at startup
#define MODE_FIXED			3						// TS is sent to a fixed address

#define IP_OFFNET			0
#define IP_MYPC				1
#define IP_MYNET			2
#define IP_MULTI			3


#define TSPACKETSIZE		188						// TS packet size
#define PPB					7						// TS packets per UDP transmission block

#define	EXPINFOREC			1
#define SUMINFOREC			2
#define LMINFOREC			3
#define	EITREC				5
#define GENINFOREC			6
#define TS7REC				7	
#define	REBOOTREC			8
#define USBTUNREC			9						// tuning request from USB

#define ZLED				ILED					// use external LED

extern  volatile     		uint32	MY_RAM_0 [64] ;
#define	RAM_BIP				2
#define	RAM_TSFLASH			4
#define DEFAULT_BIP			9900

// packetblock structure holding up to 7 packets 
// used for TS and also inter core communication
// organised into linked chains

typedef struct
{																		
	volatile	uint16		status ;
	volatile	uint32		nextpointer ;								// for packetblock chaining
	volatile	uint16		typex ;										// binary
	volatile	int16		receiver ;									// 0, 1
	volatile	uint16		receivedonport ;	
	union
	{
	  volatile	uint8		address [4] ;								// ETH IP address
	  volatile	uint32		address32 ;
	} ;
	volatile	uint16		port ;										// ETH destination port; 
	volatile	uint16		packetsinblock ;
	volatile	uint32		length ;									// number of bytes following
	union
	{
				uint8		buffer [PPB*TSPACKETSIZE] ;
				uint8		packets [PPB] [TSPACKETSIZE] ;
	} ;			
}  packetblock_t ;


struct eitx
{
    uint                sync            :8  ;
	
    uint                pid1208         :5  ;
    uint                tspriority      :1  ;
    uint                payloadstart    :1  ;    
    uint                tserror         :1  ;
	
    uint                pid0700         :8  ;

    uint                continuity      :4  ;
    uint                adaption        :2  ;
    uint                scrambling      :2  ;
	
    uint                pointer         :8  ;

    uint8               tableid             ;

    uint                filler1         :4  ;                
    uint                reserved0       :2  ;
    uint                filler2         :1  ;
    uint                syntax          :1  ;
	
    uint8               sectionlength       ;

    uint                servicehigh     :8  ;   // upper byte of service id
    uint                servicelow      :8  ;

    uint                currentnext     :1  ;
    uint                version         :5  ;
    uint                reserved1       :2  ;
	
    uint                section         :8  ;
    uint                lastsection     :8  ; 

    uint                tsidhigh        :8  ;   // upper byte of ts id
    uint                tsidlow	        :8  ;

    uint                networkhigh     :8  ;   // upper byte of network id
    uint                networklow      :8  ;

    uint                lastsegment     :8  ;
    uint                lasttable       :8  ;
    
    uint8               loop                ;

    uint8               filler0 [168]       ;
} ;

 
struct modinfo
{
    char    modtext [16] ;								// MODCOD gives modulation and FEC
    int32 	minmer ;									// required MER threshold for decode in tenths
} ;

 
const struct modinfo    modinfo_S [9] =
{
    {"",0},
    {"QPSK 1/2",17},
    {"QPSK 2/3",33},
    {"QPSK 3/4",42},
    {"",0},
    {"QPSK 5/6",51},
    {"QPSK 6/7",55},
    {"QPSK 7/8",58},
    {"",0}
} ;


const struct modinfo    modinfo_S2 [32] =
{                                                                                                                                                   
    {"",0},
    {"QPSK 1/4",   -23},
    {"QPSK 1/3",   -12},
    {"QPSK 2/5",    -3},
    {"QPSK 1/2",    10},
    {"QPSK 3/5",    22},
    {"QPSK 2/3",    31},
    {"QPSK 3/4",    40},
    {"QPSK 4/5",    47},
    {"QPSK 5/6",    52},
    {"QPSK 8/9",    62},
    {"QPSK 9/10",   64},
    {"8PSK 3/5",    55},
    {"8PSK 2/3",    66},
    {"8PSK 3/4",    79},
    {"8PSK 5/6",    94},
    {"8PSK 8/9",   107},
    {"8PSK 9/10",  110},
    {"16APSK 2/3",  90},
    {"16APSK 3/4", 102},
    {"16APSK 4/5", 110},
    {"16APSK 5/6",116},
    {"16APSK 8/9",129,},
    {"16APSK 9/10",131},
    {"32APSK 3/4",127},
    {"32APSK 4/5",136},
    {"32APSK 5/6",143},
    {"32APSK 8/9",157},
    {"32APSK 9/10",160},
    {"",0},
    {"",0},
    {"",0}
} ;

 
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@  global data 
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

			char				output   	  [1024] ;
			char				outputonnet   [1024] ;					// rx status output 
			char				outputoffnet  [1024] ;

			int 				packetcount = 0 ;
volatile	uint8				inhibit_lmprint ;
			uint32				lastcommandcheck_ms ;
volatile	uint8				ethernet_ready ; 						// set when an IP address has been set

volatile	uint32				freechainempty ;
	
            char        		baseinterfaceaddress	[16] ;
            char				baseipaddress 			[16] ;       	// the IP address   for all I/O
            uint16				baseipport ;            	         	// the base IP port for all I/O
            char                zbuff     		        [1024] ;
            char                commandreplybuff        [320] ;
            char                commandrxbuff  			[256] ;
            char*               commandrxbuff2 ;
            char                commandrxbuff3 			[256] ;
const 		uint32 				daysinmonth 			[]   = {0,31,28,31,30,31,30,31,31,30,31,30,31} ;
			char				expandedtextinfo 		[1400] ;
			struct eitx     	eit ;    				// EIT packet
			uint32				eitinsert ;				// an EIT packet with info is inserted into the TS
			uint32				eitremove ;				// EIT packets are removed from the incoming TS
            uint32				GLOBALNIM ; 
volatile	int32				lminfoutenabled ;		// enable info sending
		    uint32      		lastinfotime ;			// time of last info transmission (ms)
			uint32				h265max ;				// maximum number of VLC windows to use the hardware decoder
			uint32				idletime ;				// receiver is disabled after this many seconds of inactivity
volatile	uint32				inicommandcount ;		// number of commands in the ini file
volatile	uint32				inicommandtimer ;		// number of commands in the ini file
volatile	uint32				inicommandnext ;		// the next command to be processed
volatile	int32				inicommandenabled ;		// enable ini command sending
			char				inicommands[16][80] ;	// commands in the ini file
			uint32				modex ;					// program and TS distribution mode
			uint32				null8190 ;				// PID 8190 is treated as a NULL packet
			uint32				nullremove ;			// NULL packets are not sent to VLC
			uint32				offnettime ;			// off net transmission is stopped after this many seconds
														// of no commands when sending off net
            uint32              peripherals_virtual_address ;	// virtual address of the peripherals
			int32				rxbase ;				// the 4 receivers are numbered starting at this value
volatile	uint32				terminate ;
volatile    uint32             	txpbindexin ;           // indexes for the UDP sending ring buffer       
volatile    uint32            	txpbindexout ;                                    
volatile	int32				tsprocenabled ;			// enable UDP packet sending
			uint32				qo100beaconfreq ;		// nominal frequency of the beacon
			uint32				vgxpresent ;	
			uint32				vgxtone ;				// voltage generator X tone
			uint32				vgxen ;					// voltage generator X enable				
			uint32				vgxsel ;				// voltage generator X low / high select	
			uint32				vgypresent ;	
			uint32				vgytone ;				// voltage generator Y tone
			uint32				vgyen ;					// voltage generator Y enable				
			uint32				vgysel ;				// voltage generator Y low / high select	
			int					whfd ;


// DMA variables

volatile	uint32				dmatoggle				[MAXRECEIVERS+1]    = {0,0,0} ;
volatile	uint32				last_dmatoggle			[MAXRECEIVERS+1]    = {0,0,0} ;
volatile	uint32				last_dmatoggle_time 	[MAXRECEIVERS+1]    = {0,0,0} ;
			int8				activity_ledgp			[MAXRECEIVERS+1]	= {-1, -1, -1} ;

volatile	uint32				broadcast_sequence ;
volatile	uint32				last_broadcast_time ;;

volatile	char				lastincomingcommand [256] ;
volatile	char				debuginfo			[256] ;

			char				versionx3U			[16] ;
			char				versionx3L			[16] ;
			
			 
// DHCP state machine

#define	MAC_SIZE						6
#define DHCPOUT_PORT					67
#define DHCPIN_PORT						68
#define	DHCP_MESSAGE_TYPE				53
#define	DHCP_MESSAGE					56
#define	DHCP_MESSAGE_SUBNET				1
#define	DHCP_MESSAGE_GATEWAY			3
#define	DHCP_MESSAGE_DNS				6
#define	DHCP_MESSAGE_OFFER				2
#define	DHCP_MESSAGE_ACK				5
#define	DHCP_MESSAGE_NAK				6
#define DHCP_MESSAGE_SERVER				54
#define DHCP_MESSAGE_LEASETIME			51
#define DHCP_MESSAGE_BROADCAST			28
#define DHCP_MESSAGE_REQUEST			53
#define DHCP_MESSAGE_CLIENTID			61
#define DHCP_MESSAGE_REQUESTEDIP		50
#define DHCP_MESSAGE_HOSTNAME			12
#define DHCP_MESSAGE_PARAMETERS			55

#define	DHCP_IDLE_STATE_0				0
#define DHCP_SENDING_DISCOVER_STATE_1	1
#define	DHCP_WAITING_OFFER_STATE_2		2
#define	DHCP_WAITING_ACK_STATE_3		3
#define DHCP_WAITING_TO_RETRY_STATE_4	4
#define DHCP_OK_STATE_5					5
#define DHCP_AWAITING_LINK_STATE_6		6
#define DHCP_SENDING_REQUEST_STATE_7	7
#define DHCP_SUCCESS					73					
#define DHCP_FAIL						88										


struct dhcp_discover
{
	uint8	boot_request ;									// 1
	uint8	hardware_type ;									// 1 = Ethernet
	uint8	mac_length ;									// 6
	uint8	hops ;											// 0
	uint8	transaction_id 				[4] ;				// random
	uint8	seconds_elapsed 			[2] ;				// 0
	uint8	bootp_flags 				[2] ;				// 0x80, 0x00
	uint8	client_ip 					[4] ;				// 0
	uint8	your_ip 					[4] ;				// 0
	uint8	next_server					[4] ;				// 0
	uint8	relay_ip 					[4] ;				// 0
	uint8	mac2 						[MAC_SIZE] ;		// my mac
	uint8	mac2_padding				[16 - MAC_SIZE] ;	// 0
	uint8	server_name					[64] ;				// 0
	uint8	boot_file_name				[128] ;				// 0
	uint8	magic_cookie				[4] ;				// 0x63, 0x82, 0x53, 0x63
	union
	{
		uint8	options [1] ;
		uint8	discover_type ;						// 0x35
	} ;
	uint8	discover_length ;						// 1
	uint8	discover ;								// 1
	uint8	client_id_type ;						// 0x3d
	uint8	client_id_length ;						// 7
	uint8	client_id_hardware ;					// 1 = Ethernet
	uint8	client_id_mac				[6] ;		// my mac
	uint8	host_name_type ;						// 0x0c
	uint8	host_name_length ;						// 18
	union
	{
		uint8		host_name			[18] ;
		struct
		{
			uint8	host_name_fixed		[12] ;
			uint8	host_name_mac		[6] ;
		} ;
	} ;
	uint8	parameter_request_type ;					// 0x37
	uint8	parameter_request_length ;					// 6
	uint8	parameter_request			[6] ;			// 1,3,6,15,58,59
	uint8	end_list ;									// 0xff
	uint8	padding						[272] ;			// 0
	uint8	end_packet ;
} ;	

	int		socket0 ;
	int		socket1 ;
	int		socket2 ;
	int		socket3 ;
	int		dhcp_socket ;
	int		broadcast_socket ;


struct dhcp_discover	dhcpdisc =
						{
							.boot_request 				= 1,
							.hardware_type				= 1,		// Ethernet
							.mac_length					= 6,
							.hops						= 0,
							.transaction_id				= {1,2,3,4},				
							.seconds_elapsed			= 0,
							.bootp_flags				= {0x80, 0},
							.client_ip					= {0},
							.your_ip					= {0},
							.next_server				= {0},
							.relay_ip					= {0},
							.mac2						= {0},
							.mac2_padding				= {0},
							.server_name				= {0},
							.boot_file_name				= {0},
							.magic_cookie				= {0x63, 0x82, 0x53, 0x63},
							.discover_type				= 0x35,
							.discover_length			= 1,
							.discover					= 1,
							.client_id_type				= 0x3d,
							.client_id_length			= 7,
							.client_id_hardware			= 1,		// Ethernet
							.client_id_mac				= {0},	
							.host_name_type				= 0x0c,
							.host_name_length			= 18,
							.host_name_fixed			= {'P','i','c','o','T','u','n','e','r','W','H','-'},
							.host_name_mac				= {0},
							.parameter_request_type		= 0x37,
							.parameter_request_length 	= 6,
							.parameter_request			= {0x01, 0x03, 0x06, 0x0f, 0x3a, 0x3b},
							.end_list					= 0xff,
							.padding					= 0							 
						} ;

struct dhcp_discover	dhcpoffer ;

struct dhcp_discover	dhcprequest =
						{
							.boot_request 				= 1,
							.hardware_type				= 1,		// Ethernet
							.mac_length					= 6,
							.bootp_flags				= {0x80, 0},
							.magic_cookie				= {0x63, 0x82, 0x53, 0x63},
						} ;

struct dhcp_discover	dhcpack ;

typedef struct wiz_NetInfo_ewj_t
{
   	uint8_t 		mac[6] ;  	/// < Source Mac Address
   	uint8_t 		ip[4] ;   	/// < Source IP Address
   	uint8_t 		sn[4] ;   	/// < Subnet Mask
   	uint8_t 		gw[4] ;   	/// < Gateway IP Address
   	uint8_t 		dns[4] ;  	/// < DNS server IP Address
   	dhcp_mode		dhcp ;		/// < 1 - Static, 2 - DHCP
	uint8_t			rx[4] ;		/// < the address that the offer was received from
	uint8_t			sv[4] ;		/// < DHCP server
} wiz_NetInfo_ewj ;

wiz_NetInfo_ewj netinfo =    			                        // Network
{
        .mac  = {0x28, 0xcd, 0xc1, 0x00, 0x00, 0x00},     	// MAC address (chip does not have one - 0x00, 0x02 may be better)
        .ip   =  {192, 168,  77, 203},                      // IP address
   		.sn   =  {255, 255, 255,   0},                      // Subnet Mask
   		.gw   =  {192, 168,  77,   1},                 		// Gateway
   		.dns  =  {192, 168,  77,   1},                   	// DNS server   		
   		.dhcp =  NETINFO_DHCP,								// mode
   		.sv   =  {192, 168,  77,   1},                      // Gateway
} ;

 	wiz_NetInfo 		netinfo_wiz ;
 	                 
	uint				dhcpoutaddress 	[4] 		= {255, 255, 255, 255} ;	
	uint8				unique_board_id [8] ;
	char				board_type		[64] ;

			int		dhcp_link_state				= 0 ;
			int		dhcp_mark_time				= 0 ;
			int		dhcp_last_time				= 0 ;
			int		dhcp_current_time			= 0 ;
			int		dhcp_machine_state  		= 0 ;
			int		dhcp_debug					= 0 ;
			int		dhcp_active 				= 0 ;
			int		dhcp_success 				= 0 ;	
			int		dhcp_counter				= 0 ;	
			int		dhcp_last_runtime			= 0 ;	
			int		bip_error					= 0 ;
			int		bip_value					= 0 ;
			int		tsflash_value				= 1 ;
volatile	int		reboot_request				= 0 ;
	
	char	dhcp_message 				[64] ;
	char	dhcp_status_message			[64] ;
	uint8	broadcast_address 			[4] 	= {255, 255, 255, 255} ;
	uint8	multicast_address 			[4] 	= {230,   0,   0, 230} ;
	uint32  lease_period ;
	uint32	leased_at ;													// when the DHCP was offered

// packet buffers

volatile packetblock_t			packetblocks [MAXPACKETBLOCKS] ;		// the physical packet locations

// chains

typedef struct
{
	volatile	packetblock_t*	start ;
	volatile	packetblock_t*	end ;
	volatile 	uint32			count ;
} chain_t ;

	chain_t		freechain ;												// the chain of free packets

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//
//	Ethernet variables
//
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

/* Buffer */

#define ETHERNET_BUF_MAX_SIZE (1024 * 2)


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@  control structure for each of the total of 2 receivers in a NIM
//@  there are 3 structures: 0 is used by the system for IP control
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

struct rxcontrol
{
			uint8      		    receiver ;        	        	// 1-4 receiver number in the system
			uint8               nimreceiver ;              		// 1/2 = first/second receiver in each NIM
			uint8               xlnaexists ;            		// not all NIMs have external LNAs
			uint8               nim ;                   		// 1/2 = NIMA/NIMB
			char	            nimtype			[16] ;  		// FTS4334L, FTS4335
			uint8         		active ;						//  /1 = actively receiving or searching
			uint8               antenna ;               		// 1/2 = TOP/BOT
			uint8         		scanstate ;             		// searching/locked etc

			uint32				audiotype ;						// audio service type indicator
			uint32				commandreceivedtime ;			// the time at which the command arrived (ms)
			uint32				debug0 ;
			uint8				sendnullpackets ;	    		//  /1 = send null packets on the output UDP streams
			int32				demodfreq ;						// frequency offset detected by the demodulator
			uint32              enablefreqscan ;        		// scan the 'frequencies' list
			uint32      		frequencies [MAXFREQSTOSCAN] ;	// kHz; multiple frequencies may be scanned
			uint16              freqindex ;             		// the index of the current frequency in 'frequencies'
			uint8				eitcontinuity ;					// sequence number for injected EIT packets
			uint8				eitversion ;					// incrementing version number for injected EIT packets
			uint32              enablesrscan ;          		// scan the 'symbolrates' list           
			uint32				errors_outsequence ;
			uint32				errors_insequence ;
			uint32				errors_restart ;
			uint32				errors_overflow ;
			uint32				errors_sync ;
			uint32				forbidden ;						// cannot send this TS to RPi VLC
			uint32				hardwarefreq ;					// frequency passed to the tuner
			uint32				highsideloc ;					// /1 = LO is on the high side
			uint16      		summaryport ;			    	// port for sending 4 line info 
			int					summarysock ;					// . . (same info for all receivers)
			uint16      		summary2port ;			
			int					summary2sock ;					
			uint16      		lminfoport ;			   		// port for sending original LM $ info   	
			int					lminfosock ;		    			
			uint16      		expinfoport ;			   		// port for sending expanded WH $ info 
			int					expinfosock ;		    		// 	. . (same for all receivers)	
			uint16      		expinfo2port ;			    	// future expansion
			int					expinfo2sock ;		    			 
			union
			{
				uint8 			fromaddress[4] ;
				uint32			fromaddress32 ;
			} ;
			uint16				fromport ;
			uint32				insequence ;					// 4 bit counter inserted by the PIC for each packet
			char	    		interfaceaddress[16] ;			// network interface to use, if more than one is available
			char	    		ipaddress 		[16] ;			// address for all outgoing operations
			uint32				ipchanges ;						// incremented when a command comes from a different IP
			uint32				iptype ;						// destination IP type: MYPC, MYNET, OFFNET, MULTICAST
			uint32				lastmodulation ;				// last modcod / FEC seen
			uint16      		listenport ;	        		// port for incoming commands
			int					listensock ;		    		// 
			uint32				modechanges ;					// increments on new command, new callsign, new codec
			char	    		newipaddress 	[16] ;			// command came from a new IP address
			uint32				programcount ;					// number of programs in the TS
			int32				qo100locerror ;
			uint32				qo100mode ;						// 0 / 1 / 2 = NO / QO-100 band / QO-100 beacon
			uint32				requestedfreq ;					// the frequency in the incoming command
			uint32				requestedloc ;					// the local oscillator the incoming command
			uint32				requestedprog ;					// the program number in the incoming command
			uint32      		symbolrates [MAXSRSTOSCAN] ;	// kS;  multiple symbol rates may be scanned
			uint16              srindex ;               		// the index of the current symbol rate in 'symbolrates'
			uint32				timeoutholdoffcount ;			// info is sent a number of times after timing out
			uint16      		tsport ;			    		// port    for transport stream output
			int					tssock ;		    			// socket  for transport stream output
			uint32				outsequence ;					// 4 bit counter inserted by each PIC 
			uint32				packetcountprogram ;			// total since the program started
			uint32				packetcountrx ;					// total for this reception
			PIO					piogroup ;						// PIO group 0/1
			uint16				piosm ;							// PIO state machine number
			uint16				piooffset ;
			uint16				piodmachan	 		[2] ;		// 2 channels for ping-pong
			dma_channel_config	piodmaconfig 		[2] ;
			packetblock_t*		piodmapacketblock	[2] ;		// address of the packet buffer	
volatile	uint16				piorunning ;					// !0 when active
			uint32				nullpacketcountprogram ;		// total null packets since the program started
			uint32				nullpacketcountrx ;				// total null packets for this reception
			uint16				network ;						// TS network number; 0xffff for beacon; not needed for VLC EIT
			uint32				signalacquiredtime ;			// time when the received signal was first acquired
			uint32				signallosttime ;				// time when the received signal was lost
			uint32				timedouttime ;					// time when the transmission timeout occurred
			uint16				tsid ;							// TS ID; 0xaaaa for beacon; not needed for VLC EIT
			uint16				pmtpid ;						// program map table pid
			uint16				pcrpid ;						
			uint16				serviceid ;						// TS program service ID; 0x0001 for beacon; NEEDED for VLC EIT
			chain_t				rxchain ;						// received packet chain structure 
			uint16				socket	;						// Wiznet socket number 0-3
			union
			{
				uint8			toaddress		[4] ;			// Wiznet IP address - destination of TS - 192,168 . .
				uint32			toaddress32 ;
			} ;
			union
			{
				uint8			newtoaddress	[4] ;			// Wiznet IP address - destination of TS - 192,168 . .
				uint32			newtoaddress32 ;
			} ;
			uint16				toport ;						// Wiznet IP port
			uint16				ethusbpath ;					// 0 / 1 /2 = None / ETH / USB path for TS etc
			uint32				videotype ;						// video service type indicator
			uint32				vlcnextcount ;					// incremented when N command sent to VLC
			uint32				vlcstopcount ;					// incremented when S command sent to VLC
			uint32				vlcstopped ;					// S command sent to VLC
			int32               rawinfos  		[MAXINFOS] ;  	// raw values of the info items  
			char                textinfos 		[MAXINFOS][64]; // formatted items for info or EIT output, indexed by parameter number  
} ; 
			struct rxcontrol	rcv       		[MAXRECEIVERS+1] ;  // receivers 1, 2; 0 is used by the system

			packetblock_t 		tur01e ;						// tuning requests from Core0 to Core1
			packetblock_t		xsr10 ;							// used by Core1 to send to Core0
			packetblock_t*		turp ;							// used by tuning command processor	
		 	

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@  function prototypes
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

			void			xprintf						(char*) ;
			void			dma_interrupts_off			(void) ;
			void			dma_interrupts_on			(void) ;

			void			core1_main					(void) ;
			uint32			calculateCRC32				(uint8*, uint32) ;
			int				info_loop					(void) ;
			void			getdatetime					(char*) ;
			int32			getiptype					(char*) ;
			int				get_link_state				(void) ;
			void 			inicommand_loop 			(void) ;
			void			juliandate					(int32*, int32*) ;
            uint32			monotime_ms					(void) ;
			uint32			reverse						(uint32) ;
			void			setup_eit					(void*, uint32, char*) ;
            int             setup_io_map        		(void) ;
			void			setup_titlebar				(char*, uint32) ;
			int				tsproc_loop					(packetblock_t*) ;
			void			whexit						(int32) ;
			void 			addtochain 					(chain_t *chain, packetblock_t* add) ;
			packetblock_t* 	getfromchain 				(chain_t *chain) ;
			void			mydma_handler				(void) ;
			packetblock_t*	check_received_pid 			(packetblock_t*, chain_t*) ;
			void			lmprint						(char*) ;  
			void			set_activity_led			(int32, uint32) ;
			char*			validate_callsign			(char*) ;		

static 		void	     	wizchip_dhcp_init           (void) ;
static 		void    	 	wizchip_dhcp_assign         (void) ;
static 		void     		wzchip_dhcp_conflict       	(void) ; 
static 		void 			repeating_timer_callback    (void) ;

			int				ethernet_setup				(void) ;
			void			mainloop					(void) ;

			int 			dhcp_scheduler 						(int) ;
			int 			dhcp_idle_state_0					(int) ;
			int 			dhcp_sending_discover_state_1		(int) ;
			int 			dhcp_waiting_offer_state_2			(int) ;
			int 			dhcp_waiting_ack_state_3			(int) ;
			int 			dhcp_waiting_to_retry_state_4		(int) ;
			int 			dhcp_ok_state_5						(int) ;
			int 			dhcp_awaiting_link_state_6			(int) ;
			int 			dhcp_sending_request_state_7		(int) ;

//********************************************************************************************************
	
uint32 get32 (uint8* p)													// get 32 value from 4 low-endian bytes
{
	uint32	temp ;	
	int		x ;

	memcpy ((void*)&temp, (void*)p, 4) ;

	return (temp) ;
}

void xprintf (char* message)											// print to stdout or pass USB record
{
	printf ("%s", message) ;
}

void lmprint (char* xstring)											// intercept print output from LM
{
	return ; 
}    

// WIZnet W6100 low level SPI callbacks and initialisation
// (replaces the modified "ewj" library functions)

static void pt_wizchip_select (void)
{
	gpio_put (PIN_CS, 0) ;
}

static void pt_wizchip_deselect (void)
{
	gpio_put (PIN_CS, 1) ;
}

static uint8_t pt_wizchip_read (void)
{
	uint8_t rx = 0xFF ;
	spi_read_blocking (SPI_PORT, 0xFF, &rx, 1) ;
	return (rx) ;
}

static void pt_wizchip_write (uint8_t wb)
{
	spi_write_blocking (SPI_PORT, &wb, 1) ;
}

static void pt_wizchip_read_buf (uint8_t* rx, datasize_t len)
{
	spi_read_blocking (SPI_PORT, 0xFF, rx, len) ;
}

static void pt_wizchip_write_buf (uint8_t* tx, datasize_t len)
{
	spi_write_blocking (SPI_PORT, tx, len) ;
}

void wizchip_initialize_ewj (void)
{
#if (_WIZCHIP_ == W6100)
	uint8_t memsize [2] [8] = { {2, 4, 4, 2, 1, 1, 1, 1}, {2, 4, 4, 2, 1, 1, 1, 1} } ;
#endif

	reg_wizchip_cs_cbfunc (pt_wizchip_select, pt_wizchip_deselect) ;
#if (_WIZCHIP_ == W6100)
	reg_wizchip_spi_cbfunc (pt_wizchip_read, pt_wizchip_write, pt_wizchip_read_buf, pt_wizchip_write_buf) ;
#else
	reg_wizchip_spi_cbfunc (pt_wizchip_read, pt_wizchip_write) ;
#endif

	if (ctlwizchip (CW_INIT_WIZCHIP, (void*) memsize) == -1)
	{
		printf ("WIZnet init fail\r\n") ;
		return ;
	}
}

void netinfo_to_wiz (void)											// copy ewj netinfo to the WIZnet netinfo struct
{
	memset (&netinfo_wiz, 0, sizeof(netinfo_wiz)) ;
	memcpy (netinfo_wiz.mac, netinfo.mac, 6) ;
	memcpy (netinfo_wiz.ip,  netinfo.ip,  4) ;
	memcpy (netinfo_wiz.sn,  netinfo.sn,  4) ;
	memcpy (netinfo_wiz.gw,  netinfo.gw,  4) ;
	memcpy (netinfo_wiz.dns, netinfo.dns, 4) ;
	netinfo_wiz.dhcp  = netinfo.dhcp ;
#if (_WIZCHIP_ == W6100)
	netinfo_wiz.ipmode = (netinfo.dhcp == NETINFO_DHCP) ? NETINFO_DHCP_V4 : NETINFO_STATIC_V4 ;
#endif
}
    
     
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@  main process
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
 
int __in_flash("my_group_name") main()
{
	int32				argc ;
	char				argv [8] [20] ;
	int					fd ;
	FILE				*ip ;
	uint32				x ;
	uint32				y ;
	int32				z ;
	uint32				r ;
	char*				pos ;
	int32				rx ;
    int32               temp ;
    uint8				tempc0 ;
    uint8				tempc1 ;
	char				temps [256] ;
	int					status ;
	uint8				nimOK ;
	uint8				xlnaOK ;
	uint8				chipid0910 ;
	uint8				devid0910 ;
	uint8				id6120 ;
	uint8*				pointer ;
	uint8				tempc ;
	uint8				err ;
	uint8 				scanstate ;	
	uint8				tempc2 ;

	stdio_init_all() ;
	stdio_flush() ;

	inhibit_lmprint 	= 0 ;
	xsr10.status 		= 0 ;
	terminate 			= 0 ;
	freechainempty 		= 0 ;

// set up gpio
	
	gpio_init 		(WIZRESET);
    gpio_set_dir 	(WIZRESET, GPIO_OUT);
    gpio_put 		(WIZRESET, 0) ; 	  								// active low
	
	gpio_init 		(BOARDRESET);
    gpio_set_dir 	(BOARDRESET, GPIO_OUT);
    gpio_put 		(BOARDRESET, 0) ;   								// active low

    gpio_init 		(ILED);												// on Pico board, for testing
    gpio_set_dir 	(ILED, GPIO_OUT);
    gpio_put 		(ILED, 0) ;   
	
// set I2C as inputs for the moment

    gpio_init 		(I2C1CLK) ;
    gpio_set_dir 	(I2C1CLK, GPIO_IN) ;
    gpio_set_pulls 	(I2C1CLK, 1, 0) ;									// pull up

    gpio_init 		(I2C1DAT) ;
    gpio_set_dir 	(I2C1DAT, GPIO_IN) ;
    gpio_set_pulls 	(I2C1DAT, 1, 0) ;									// pull up

// set NIM receiver inputs and pull up
		
    gpio_init 		(NIMDAT_R1) ;					
    gpio_set_dir 	(NIMDAT_R1, GPIO_IN) ;		
    gpio_set_pulls 	(NIMDAT_R1, 1, 0) ;									// pull up

    gpio_init 		(NIMCLK_R1) ;
    gpio_set_dir 	(NIMCLK_R1, GPIO_IN) ;
    gpio_set_pulls 	(NIMCLK_R1, 1, 0) ;									// pull up
	
    gpio_init 		(NIMSS_R1) ;				
    gpio_set_dir 	(NIMSS_R1, GPIO_IN) ;
    gpio_set_pulls 	(NIMSS_R1, 1, 0) ;									// pull up
	
    gpio_init 		(NIMDAT_R2) ;					
    gpio_set_dir 	(NIMDAT_R2, GPIO_IN) ;		
    gpio_set_pulls 	(NIMDAT_R2, 1, 0) ;									// pull up

    gpio_init 		(NIMCLK_R2) ;
    gpio_set_dir 	(NIMCLK_R2, GPIO_IN) ;
    gpio_set_pulls 	(NIMCLK_R2, 1, 0) ;									// pull up
	
    gpio_init 		(NIMSS_R2) ;				
    gpio_set_dir 	(NIMSS_R2, GPIO_IN) ;
    gpio_set_pulls 	(NIMSS_R2, 1, 0) ;									// pull up

// set other pins

    gpio_init 		(REGULATOR_ENABLE) ;
    gpio_set_dir 	(REGULATOR_ENABLE, GPIO_IN) ;		// input initially, for presence sensing
    gpio_set_pulls 	(REGULATOR_ENABLE, 1, 0) ;			// pull up
	
    gpio_init 		(VGXEN) ;
    gpio_set_dir 	(VGXEN, GPIO_OUT) ;
	gpio_put		(VGXEN, 0) ;

    gpio_init 		(VGXSEL) ;
    gpio_set_dir 	(VGXSEL, GPIO_IN) ;					// input initially, for presence sensing
    gpio_set_pulls 	(VGXSEL, 1, 0) ;					// pull up
	
    gpio_init 		(VGYEN) ;
    gpio_set_dir 	(VGYEN, GPIO_OUT) ;
	gpio_put		(VGYEN, 0) ;

    gpio_init 		(VGYSEL) ;
    gpio_set_dir 	(VGYSEL, GPIO_IN) ;					// input initially, for presence sensing
    gpio_set_pulls 	(VGYSEL, 1, 0) ;					// pull up

    gpio_init 		(PLUS4) ;
    gpio_set_dir 	(PLUS4, GPIO_IN) ;
    gpio_set_pulls 	(PLUS4, 1, 0) ;												// pull up

    gpio_init 		(GP0);														// LED on v0.3beta
    gpio_set_dir 	(GP0, GPIO_IN);
    gpio_set_pulls 	(GP0, 1, 0) ;									

    gpio_init 		(GP1);														// LED on v0.3beta
    gpio_set_dir 	(GP1, GPIO_IN);
    gpio_set_pulls 	(GP1, 1, 0) ;									

    gpio_init 		(SPARE_GP3);												// on Pico board, for testing
    gpio_set_dir 	(SPARE_GP3, GPIO_IN);
    gpio_set_pulls 	(SPARE_GP3, 1, 0) ;									

    gpio_init 		(SPARE_GP4);												// on Pico board, for testing
    gpio_set_dir 	(SPARE_GP4, GPIO_IN);
    gpio_set_pulls 	(SPARE_GP4, 1, 0) ;									

    gpio_init 		(SPARE_GP5);												// on Pico board, for testing
    gpio_set_dir 	(SPARE_GP5, GPIO_IN);
    gpio_set_pulls 	(SPARE_GP5, 1, 0) ;									

	sleep_ms (100) ;	

	vgxpresent	= (gpio_get (VGXSEL) & 1) ^ 1 ;
	vgypresent	= (gpio_get (VGYSEL) & 1) ^ 1 ;

	if (gpio_get (REGULATOR_ENABLE))											// doesn't appear to work reliably, so don't broadcast
	{
		strcpy (board_type, "MT MK2 + adapter") ;
	}
	else
	{
		strcpy (board_type, "BATC PicoTuner") ;
	}

	gpio_init 		(REGULATOR_ENABLE);
    gpio_set_dir 	(REGULATOR_ENABLE, GPIO_OUT);
    gpio_put 		(REGULATOR_ENABLE, 0) ;   							 
	
    gpio_init 		(VGXSEL) ;
    gpio_set_dir 	(VGXSEL, GPIO_OUT) ;
	gpio_put		(VGXSEL, 0) ;

    gpio_init 		(VGYSEL) ;
    gpio_set_dir 	(VGYSEL, GPIO_OUT) ;
	gpio_put		(VGYSEL, 0) ;

	activity_ledgp [1] = 0 ;
	activity_ledgp [2] = 1 ;

	for (x = 1 ; x <= 2 ; x++)
	{
		if (activity_ledgp[x] >= 0)
		{
			gpio_init 			(activity_ledgp[x]) ;
    		gpio_set_dir 		(activity_ledgp[x], GPIO_OUT) ;
    		set_activity_led	(x, OFF) ;  			
		}
	}

// send HI in morse

#define K	100

	gpio_put (ZLED, 1) ;
	set_activity_led	(1, ON) ;  				 
	set_activity_led	(2, ON) ;  				 
	sleep_ms (K) ;	
	gpio_put (ZLED, 0) ;
	set_activity_led	(1, OFF) ;  				 
	set_activity_led	(2, OFF) ;  				 
	sleep_ms (K) ;	
	gpio_put (ZLED, 1) ;
	set_activity_led	(1, ON) ;  				 
	set_activity_led	(2, ON) ;  				 
	sleep_ms (K) ;	
	gpio_put (ZLED, 0) ;
	set_activity_led	(1, OFF) ;  				 
	set_activity_led	(2, OFF) ;  				 
	sleep_ms (K) ;	
	gpio_put (ZLED, 1) ;
	set_activity_led	(1, ON) ;  				 
	set_activity_led	(2, ON) ;  				 
	sleep_ms (K) ;	
	gpio_put (ZLED, 0) ;
	set_activity_led	(1, OFF) ;  				 
	set_activity_led	(2, OFF) ;  				 
	sleep_ms (K) ;	
	gpio_put (ZLED, 1) ;
	set_activity_led	(1, ON) ;  				 
	set_activity_led	(2, ON) ;  				 
	sleep_ms (K) ;	
	gpio_put (ZLED, 0) ;
	set_activity_led	(1, OFF) ;  				 
	set_activity_led	(2, OFF) ;  				 
	sleep_ms (K) ;	
	sleep_ms (K) ;	
	sleep_ms (K) ;	
	gpio_put (ZLED, 1) ;
	set_activity_led	(1, ON) ;  				 
	set_activity_led	(2, ON) ;  				 
	sleep_ms (K) ;	
	gpio_put (ZLED, 0) ;
	set_activity_led	(1, OFF) ;  				 
	set_activity_led	(2, OFF) ;  				 
	sleep_ms (K) ;	
	gpio_put (ZLED, 1) ;
	set_activity_led	(1, ON) ;  				 
	set_activity_led	(2, ON) ;  				 
	sleep_ms (K) ;	
	gpio_put (ZLED, 0) ;
	set_activity_led	(1, OFF) ;  				 
	set_activity_led	(2, OFF) ;  				 
	sleep_ms (K) ;	
	sleep_ms (100) ;	

	if (_WIZCHIP_ == W6100) 
	{
		strcpy (versionx3U, "-W6100") ;
		strcpy (versionx3L, "-w6100") ;
	}
	else 
	{
		strcpy (versionx3U, "") ;
		strcpy (versionx3L, "") ;
	}

	printf ("\r\n") ;
	printf ("ptwh%s%s%s.uf2 is starting \r\n", VERSIONX, VERSIONX2, versionx3L) ;

	gpio_put (REGULATOR_ENABLE, 1) ;
	sleep_ms (250) ;

	gpio_put (WIZRESET, 1) ; 											// enable WIZ chip
	sleep_ms (100) ;
    wizchip_spi_initialize() ;
    wizchip_cris_initialize() ;
    wizchip_reset() ;
    wizchip_initialize_ewj() ;

	temp = 0 ;

#if (_WIZCHIP_ == W6100)
    if (getCIDR() == 0x6100)
    {
    	temp = 1 ;
    }
#endif

	gpio_put (WIZRESET, 0) ; 				// disable WIZ chip

	if (temp == 0)
	{
		sleep_ms (1000) ;

// repeatedly send NO in morse

		while (1)
		{
			printf ("ptwh%s%s%s.uf2: wrong Pico module\r\n", VERSIONX, VERSIONX2, versionx3L ) ;
			gpio_put (ZLED, 1) ;
			set_activity_led	(1, ON) ;  				 
			set_activity_led	(2, ON) ;  				 
			sleep_ms (K * 3) ;	
			gpio_put (ZLED, 0) ;
			set_activity_led	(1, OFF) ;  				 
			set_activity_led	(2, OFF) ;  				 
			sleep_ms (K) ;	
			gpio_put (ZLED, 1) ;
			set_activity_led	(1, ON) ;  				 
			set_activity_led	(2, ON) ;  				 
			sleep_ms (K) ;	
			gpio_put (ZLED, 0) ;
			set_activity_led	(1, OFF) ;  				 
			set_activity_led	(2, OFF) ;  				 
			sleep_ms (K) ;	
			sleep_ms (K) ;	
			sleep_ms (K) ;	
			gpio_put (ZLED, 1) ;
			set_activity_led	(1, ON) ;  				 
			set_activity_led	(2, ON) ;  				 
			sleep_ms (K * 3) ;	
			gpio_put (ZLED, 0) ;
			set_activity_led	(1, OFF) ;  				 
			set_activity_led	(2, OFF) ;  				 
			sleep_ms (K) ;	
			gpio_put (ZLED, 1) ;
			set_activity_led	(1, ON) ;  				 
			set_activity_led	(2, ON) ;  				 
			sleep_ms (K * 3) ;	
			gpio_put (ZLED, 0) ;
			set_activity_led	(1, OFF) ;  				 
			set_activity_led	(2, OFF) ;  				 
			sleep_ms (K) ;	
			gpio_put (ZLED, 1) ;
			set_activity_led	(1, ON) ;  				 
			set_activity_led	(2, ON) ;  				 
			sleep_ms (K * 3) ;	
			gpio_put (ZLED, 0) ;
			set_activity_led	(1, OFF) ;  				 
			set_activity_led	(2, OFF) ;  				 
			sleep_ms (K) ;	

			sleep_ms (K * 6) ;
		}
	}

	baseipport = DEFAULT_BIP ;
	if (MY_RAM_0 [RAM_BIP + 1] == 123456789)
	{
		baseipport = (MY_RAM_0 [RAM_BIP + 0] & ~1) & 0xffff ;
	}
	else if (gpio_get(PLUS4) == 0)
	{
		baseipport	+= 4 ;							// add 4 to hard coded baseipport if PLUS4 grounded
	}

	rxbase = baseipport % 100 ;
	if (rxbase < 0 || rxbase > 14 || rxbase & 1)	//Base IP Port must be even and xx00 to xx14
	{
		baseipport 	= 9900 ;
		rxbase		= 0 ;
	}

	if (MY_RAM_0 [RAM_TSFLASH + 1] == 123456789)
	{
		tsflash_value = MY_RAM_0 [RAM_TSFLASH + 0] ;
	}

	xprintf ("\r\n") ;
    xprintf ("=======================================================================================================\r\n") ;                         
    xprintf ("=======================================================================================================\r\n") ;                         
    sprintf (temps, "PicoTuner Dual Channel Digital TV Receiver - WinterHill mode - Version=ptwh%s%s%s.uf2 \r\n", VERSIONX, VERSIONX2, versionx3L) ;
    xprintf (temps) ;
    xprintf ("=======================================================================================================\r\n") ;                         

	whfd			= -1 ;
	eitremove		= 1 ;							// winterhill.ini file items
	eitinsert		= 0 ;
	null8190		= 1 ;
	nullremove		= 0 ;
	h265max			= 1 ;							// one H.265 hardware decoder allowed by default on RPi
    qo100beaconfreq	= 10491500 ;
    offnettime		= 900 ;							// 15 minute hour timeout when sending off net
    idletime		= 0 ;							// switch off receivers after ~ inactivity
	inicommandcount = 0 ;
	inicommandtimer = 0 ;
	inicommandnext  = 0 ;
	memset (inicommands, 0, sizeof(inicommands)) ;

	sprintf ((void*)lastincomingcommand, "") ;
	sprintf ((void*)debuginfo, "") ;

    uint16 					expinfoport ; 
    int32 					expinfosock ; 
	int32					dhcpsock ;
	
	argc = 8 ;
    
///    sscanf ("winterhill 0 9900 0 0 0 0 0", "%s %s %s %s %s %s %s %s", argv[0],argv[1],argv[2],argv[3],argv[4],argv[5],argv[6],argv[7]) ;
    

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@  clear data    
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    
    memset ((void*)&rcv,0,sizeof(rcv)) ;                    // clear the receiver control structures

    tsprocenabled	    = 0 ;
    lminfoutenabled     = 0 ;
	vgxen			    = 0 ;								// voltage generators
	vgxsel			    = 0 ;	
	vgxtone				= 0 ;
	vgyen			    = 0 ;								// voltage generators
	vgysel			    = 0 ;	
	vgytone				= 0 ;
	modex 				= 0 ;

	if (vgxpresent)																// VGX/Y stored in rcv[0]
	{
		strcpy (rcv[0].textinfos[STATUS_VGX_STATE], "OFF") ;
	}
	else
	{
		strcpy (rcv[0].textinfos[STATUS_VGX_STATE], "absent") ;
	}

	if (vgypresent)
	{
		strcpy (rcv[0].textinfos[STATUS_VGY_STATE], "OFF") ;
	}
	else
	{
		strcpy (rcv[0].textinfos[STATUS_VGY_STATE], "absent") ;
	}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@  apply the supplied parameters
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    memset ((void*)baseipaddress,0,sizeof(baseipaddress)) ;
    strncpy (baseipaddress,argv[1],sizeof(baseipaddress)-1) ;               // base IP address
    if (atoi(baseipaddress) == 0)
    {
        strcpy (baseipaddress,"") ;	                		        		// no interface address
		modex = MODE_ANYWHERE ;												// TS sent to command IP
    }
	else 
	{
		temp = getiptype (baseipaddress) ;
		if (temp < 0)
		{
			sprintf (temps, "IP address is invalid \r\n") ;
			xprintf (temps) ;
			whexit (3) ;
		}
		else if (temp == IP_MULTI)
		{
	        modex = MODE_MULTICAST ;                                    	// TS sent to multicast address
		}
		else if (temp == IP_MYPC)
		{
	    	modex = MODE_LOCAL ;											// TS sent to the local address
		}
		else
		{
	    	modex = MODE_FIXED ;											// TS sent to a fixed address
		}
	}		

    memset ((void*)baseinterfaceaddress,0,sizeof(baseinterfaceaddress)) ;
    strncpy (baseinterfaceaddress,argv[3],sizeof(baseinterfaceaddress)-1) ; // default interface

    if (atoi(baseinterfaceaddress) == 0)
    {
        strcpy (baseinterfaceaddress,"") ;                 		        	// no interface address
    }
    else
    {
		temp = getiptype (baseinterfaceaddress) ;
		if (temp < 0)
		{
			sprintf (temps, "Interface IP address is invalid \r\n") ;
			xprintf (temps) ;
			whexit (3) ;
		}
		else if (temp != IP_MYPC)
		{	
			sprintf (temps, "Interface IP address must be local \r\n") ;
			xprintf (temps) ;
			whexit (9) ;
		}
	}

	sleep_ms (100) ;


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@  detect and initialise the chips in the NIMs
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

	rpi2c_init (100000) ;

	memset (nimspresent,  0, sizeof(nimspresent)) ;			// declared in nim.c
	memset (xlnaspresent, 0, sizeof(xlnaspresent)) ;		// "

    GLOBALNIM = NIM_A ;

   	nimspresent  [NIM_A] = true ;							// assume presence for the moment
   	xlnaspresent [NIM_A] = true ;

    status = nim_init (&nimOK, &xlnaOK, &chipid0910, &devid0910, &id6120) ;

    if (status == 0 && nimOK)
    {
    	nimspresent [NIM_A] = true ;

		rcv[1].nim			= NIM_A ;
		rcv[2].nim			= NIM_A ;
		rcv[1].receiver		= 1 ;							// receiver in system
		rcv[2].receiver		= 2 ;
		rcv[1].nimreceiver	= 1 ;							// receiver in NIM
		rcv[2].nimreceiver	= 2 ;
		rcv[1].scanstate	= STATE_IDLE ;
		rcv[2].scanstate	= STATE_IDLE ;

		sprintf (temps,"%s","") ;

		sprintf 
		(
			temps + strlen(temps),
			"stv0910chipid = 0x%02X, stv0910devid = 0x%02X, stv6120id = %d, external lna = ",
			chipid0910, devid0910, id6120
		) ;

		if (xlnaOK == 0)
		{
	    	xlnaspresent [NIM_A] = false ;

			sprintf (temps+strlen(temps),"NO") ;
			
			strcpy (rcv[1].nimtype,"FTS4335") ;
			strcpy (rcv[2].nimtype,"FTS4335") ;
		}
		else
		{
	    	xlnaspresent [NIM_A] = true ;

			strcpy (rcv[1].nimtype,"FTS4334L") ;
			strcpy (rcv[2].nimtype,"FTS4334L") ;

			if (xlnaOK & 2) 
			{
		    	rcv[1].xlnaexists = 1 ;
				sprintf (temps+strlen(temps),"TOP") ;
			}
			if (xlnaOK & 1) 
			{
    			rcv[2].xlnaexists = 1 ;
    			if (xlnaOK & 2)
    			{
    				sprintf (temps+strlen(temps),"/") ;
    			}
				sprintf (temps+strlen(temps),"BOT") ;
			}
		}
		strcat (temps, "\r\n") ;
///		xprintf (temps) ;
		
		sprintf (temps, "+++ NIM_A %s initialised +++ \r\n", rcv[1].nimtype) ;
		xprintf (temps) ;
	}
    else
    {
    	nimspresent  [NIM_A] = false ;
    	xlnaspresent [NIM_A] = false ;
    	sprintf (temps, "--- NIM_A could not be initialised --- \r\n") ;
		xprintf (temps) ;
    }

    xprintf ("=======================================================================================================\r\n") ;                         

	sleep_ms (100) ;

// set up the packet chaining structure

	memset ((void*)&freechain, 0, sizeof(freechain)) ;
        
	memset ((void*)packetblocks,0,sizeof(packetblocks)) ;							
	for (x = 0 ; x < MAXPACKETBLOCKS ; x++)									// add all the packet buffers to the free chain
	{
		addtochain (&freechain, (packetblock_t*) &packetblocks[x]) ;
	}


// set up PIO

    for (rx = 1 ; rx <= MAXRECEIVERS ; rx++) 
	{
		rcv[rx].piogroup		= pio0 ;                                       		// Choose which PIO instance to use (there are two instances)   
		rcv[rx].piosm	  		= pio_claim_unused_sm (rcv[rx].piogroup, true) ;	// Find a free state machine on our chosen PIO (erroring if there are none).  
		if (rx == 1)
		{
			rcv[rx].piooffset	= pio_add_program (rcv[rx].piogroup, &RX_pio_program) ;	// Our assembled program needs to be loaded into this PIO's instruction
		}
		else
		{
			rcv[rx].piooffset	= rcv[1].piooffset ;								// Use the same program
		}
	}																				// memory. This SDK function will find a location (offset) in the                                                                        // instruction memory where there is enough space for our program. We need                                                                      // to remember this location!    


// set up DMA channels for each PIO for ping-pong

	for (rx = 1 ; rx <= MAXRECEIVERS ; rx++)
    {								
		rcv[rx].piodmachan[0]	= dma_claim_unused_channel (true) ; 	// Get a free channel, panic() if there are none
		rcv[rx].piodmachan[1]	= dma_claim_unused_channel (true) ; 	// Get a free channel, panic() if there are none

		for (y = 0 ; y < 2 ; y++)								
		{			
			rcv[rx].piodmaconfig[y] = dma_channel_get_default_config (rcv[rx].piodmachan[y]) ; 	// get the default DMA configuration    
			
			channel_config_set_transfer_data_size	(&rcv[rx].piodmaconfig[y], DMA_SIZE_32) ;	// 32 bit transfers
			channel_config_set_read_increment       (&rcv[rx].piodmaconfig[y], false) ;      	// always read from the PIO fifo
			channel_config_set_write_increment      (&rcv[rx].piodmaconfig[y], true) ;       	// increment the receive buffer address    
			channel_config_set_dreq                 (&rcv[rx].piodmaconfig[y], pio_get_dreq(rcv[rx].piogroup, rcv[rx].piosm, false)) ;
			channel_config_set_chain_to             (&rcv[rx].piodmaconfig[y], rcv[rx].piodmachan[y^1]);	// ping-pong
			channel_config_set_bswap                (&rcv[rx].piodmaconfig[y], true) ;       	// swap the order of each 4 bytes   

			rcv[rx].piodmapacketblock[y] = getfromchain (&freechain) ;
			
			dma_channel_configure
			(
				rcv[rx].piodmachan[y],                                  // Channel to be configured
				&rcv[rx].piodmaconfig[y],                               // The configuration we just created
				(void*) &rcv[rx].piodmapacketblock[y]->buffer,         	// The initial write address
				&pio0_hw->rxf [rcv[rx].piosm],    	                   	// The fixed read address of the PIO rx fifo
				PPB * TSPACKETSIZE / 4,                                 // Number of transfers; in this case each is 32 bits 
				false                                                   // Don't start immediately
			) ;
		}
	}

	sleep_ms (100) ;
	
	ethernet_setup() ;	
   
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@	 set up default IP configuration
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

    for (rx = 0 ; rx <= MAXRECEIVERS ; rx++)
    {
     	if (rx == 0)
		{
			rcv[rx].toaddress[0]	= 127 ;
			rcv[rx].toaddress[1]	= 0 ;
			rcv[rx].toaddress[2]	= 0 ;
			rcv[rx].toaddress[3]	= 1 ;
		}	

		rcv[rx].iptype 			= getiptype (rcv[rx].ipaddress) ;
	
		rcv[rx].toport 			= baseipport 		+ PORTTSBASE + rx ;		// send TS
		rcv[rx].summaryport  	= (baseipport & ~2) + PORTINFOMULTIRX ;		// send multi rx summary	
		rcv[rx].summary2port  	= (baseipport & ~2) + PORTINFOMULTIRX2 ;	// duplicate
		rcv[rx].expinfoport  	= (baseipport & ~2) + PORTINFOLMEX ;		// send expanded LM $ info
		rcv[rx].expinfo2port  	= (baseipport & ~2) + PORTINFOLMEX2 ;		// duplicate	
		rcv[rx].lminfoport  	= PORTINFOLMBASE + rx ;						// send original LM $ info	
///     rcv[rx].listenport 		= (baseipport & ~2) + PORTLISTENBASE + rx ;	// listen for commands
    }

// set up the initial tuning commands

	sprintf 
	(
		inicommands[inicommandcount++], 
		"[to@wh] rcv=1 fplug=a offset=9750000, freq=10491500 srate=1500 vgx=HI vgy=OFF "
	) ;
	sprintf (inicommands[inicommandcount++], "2") ;
	sprintf 
	(
		inicommands[inicommandcount++], 
		"[to@wh] rcv=2 fplug=a offset=9750000 freq=10491500 srate=1500 "
	) ;

	multicore_launch_core1 (core1_main) ;                           	// start the second core for NIM interface

	sleep_ms (100) ;	

// enable PIO DMA

    irq_set_exclusive_handler (DMA_IRQ_0, mydma_handler) ;              // enable DMA interrupts
    irq_set_enabled (DMA_IRQ_0, true) ;
    
	RX_pio_program_init (rcv[1].piogroup, rcv[1].piosm, rcv[1].piooffset, BASEPIN_R1, EXECPIN_R1_INDEX) ;	// Configure the PIO program
	RX_pio_program_init (rcv[2].piogroup, rcv[2].piosm, rcv[2].piooffset, BASEPIN_R2, EXECPIN_R2_INDEX) ;	// Configure the PIO program
	
	mainloop() ;
}

int get_link_state()
{
	return ((getPHYSR() & PHYSR_LNK) ? 1 : 0) ;
}


char* validate_callsign (char* buff)
{
	int		x ;

    if (strcmp(buff, "GSE!"))                                   
    {
        for (x = 0 ; x < (int) strlen(buff) ; x++)
        {
            if (!isalnum(buff[x]) && buff[x] != '/')
            {
                buff[x] = 0 ;
                break ;
            }
        }
    }
    return (buff) ;
}
		

void form_broadcast_message (char *buff)
{
	char	temps [64] ;
	int		x ;
	int		y ;
	int32	temp32 ;

	
	sprintf (buff, "\r\n") ;
	sprintf (buff+strlen(buff), " =======================================\r\n") ;
	sprintf (buff+strlen(buff), "        PicoTuner Broadcast\r\n") ;
	sprintf (buff+strlen(buff), " ---------------------------------------\r\n") ;

	sprintf (buff+strlen(buff), "        Sequence   %d \r\n", ++broadcast_sequence) ;
	sprintf (buff+strlen(buff), "        Run time   %ds \r\n", monotime_ms() / 1000) ;

	sprintf (buff+strlen(buff), "  Broadcast port   %d\r\n", PORTBROADCAST) ;

	sprintf (buff+strlen(buff), "     Pico serial   ") ;
	for (x = 0 ; x < sizeof(unique_board_id) ; x++)
	{
		sprintf (buff+strlen(buff), "%02X", unique_board_id[x]) ;
	}
	sprintf (buff+strlen(buff), "\r\n") ;

	sprintf (buff+strlen(buff), "    Net hardware   ") ;
	sprintf (buff+strlen(buff), "WIZnet ") ;
	if (_WIZCHIP_ == W6100)
	{
		sprintf (buff+strlen(buff), "W6100") ; 
	}
	sprintf (buff+strlen(buff), "\r\n") ;

	sprintf (buff+strlen(buff), "             MAC   ") ;
	for (x = 0 ; x < 6 ; x++)
	{
		sprintf (buff+strlen(buff), "%02X", netinfo.mac[x]) ;
		if (x != 5)
		{
			sprintf (buff+strlen(buff), ":") ;
		}
	}
	sprintf (buff+strlen(buff), "\r\n") ;

	memset (temps, 0, sizeof(temps)) ;
	memcpy (temps,  dhcpdisc.host_name, sizeof(dhcpdisc.host_name)) ;
	sprintf (buff+strlen(buff), "       Host name   %s \r\n", temps) ;

	sprintf (buff+strlen(buff), "      DHCP state   %s\r\n", dhcp_status_message) ;	

	if (netinfo.dhcp == NETINFO_DHCP)
	{
		temp32 = leased_at + lease_period - monotime_ms() / 1000 ;
		sprintf (buff+strlen(buff), " Lease remaining   ") ;
		if (lease_period == 0 || lease_period == 0xffffffff)
		{
			sprintf (buff+strlen(buff), "infinite") ;
		}
		else if (temp32 < 0)
		{
			sprintf (buff+strlen(buff), "expired (%ds) ", abs(temp32)) ;
		}
		else
		{
			sprintf (buff+strlen(buff), "%ds", temp32) ;
		}
		sprintf (buff+strlen(buff), "\r\n") ;
	}

	sprintf (buff+strlen(buff), "      IP address   ") ;
	for (x = 0 ; x < 4 ; x++)
	{
		sprintf (buff+strlen(buff), "%d", netinfo.ip[x]) ;
		if (x != 3)
		{
			sprintf (buff+strlen(buff), ".") ;
		}
	}
	sprintf (buff+strlen(buff), "\r\n") ;

///	sprintf (buff+strlen(buff), "      Board type   %s \r\n", board_type) ;		// detection not reliable

	sprintf (buff+strlen(buff), "        NIM type   ") ;
	if (nimspresent[NIM_A])
	{
		sprintf (buff+strlen(buff), "%s", rcv[1].nimtype) ;
	}
	else
	{
		sprintf (buff+strlen(buff), "absent") ;
	}
	sprintf (buff+strlen(buff), "\r\n") ;	

	sprintf (buff+strlen(buff), "    LNB supply X   ") ;
	if (vgxpresent == 0)
	{
		sprintf (buff+strlen(buff), "absent") ;
	}
	else 
	{
		sprintf (buff+strlen(buff), rcv[0].textinfos[STATUS_VGX_STATE]) ;
	}
	sprintf (buff+strlen(buff), "\r\n") ;

	sprintf (buff+strlen(buff), "    LNB supply Y   ") ;
	if (vgypresent == 0)
	{
		sprintf (buff+strlen(buff), "absent") ;
	}
	else 
	{
		sprintf (buff+strlen(buff), rcv[0].textinfos[STATUS_VGY_STATE]) ;
	}
	sprintf (buff+strlen(buff), "\r\n") ;
	
	sprintf (buff+strlen(buff), "        Software   ptwh%s%s%s.uf2 \r\n", VERSIONX, VERSIONX2, versionx3L) ;
	
	sprintf (buff+strlen(buff), "            Mode   WinterHill \r\n") ;	

	sprintf (buff+strlen(buff), "    Base IP port   %d ", baseipport) ;	
	if (bip_error)
	{
		sprintf (buff+strlen(buff), "(%d bad) ", bip_value) ;
	}
	sprintf (buff+strlen(buff), "\r\n") ;


	for (x = 1 ; x <= 2 ; x++)
  	{
  		sprintf (temps, "RX%d   ", x + rxbase) ;
		sprintf (buff+strlen(buff), "%19s%9s", temps, rcv[x].textinfos[STATUS_CARRIER_FREQUENCY]) ;
		sprintf (buff+strlen(buff), "%c", rcv[x].textinfos[STATUS_ANTENNA][0]) ;		
  		sprintf (buff+strlen(buff), " ") ;
		sprintf (temps, "") ;
		if (rcv[x].scanstate == STATE_DEMOD_S2 || rcv[x].scanstate == STATE_DEMOD_S)
		{
			sprintf (temps+strlen(temps), "%s", rcv[x].textinfos[STATUS_SERVICE_NAME]) ;
			validate_callsign (temps) ;
		}
		else
		{
			sprintf (temps+strlen(temps), "%s", rcv[x].textinfos[STATUS_STATE]) ;
		}
		temps [10] = 0 ;
		sprintf (buff+strlen(buff), "%s\r\n", temps) ;
  	}

  	for (x = 1 ; x <= 2 ; x++)
  	{
  		sprintf (temps, "TS target RX%d   ", x + rxbase) ;
  		sprintf (buff+strlen(buff), "%19s", temps) ;
		for (y = 0 ; y < 4 ; y++)
		{
			sprintf (buff+strlen(buff), "%d", rcv[x].toaddress[y]) ;
			if (y != 3)
			{
				sprintf (buff+strlen(buff), ".") ;
			}
		}
		sprintf (buff+strlen(buff), ":%d", rcv[x].toport) ;
		sprintf (buff+strlen(buff), "\r\n") ;
  	}
	sprintf (buff+strlen(buff), " =======================================\r\n") ;
	sprintf (buff+strlen(buff), "\r\n") ;
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@	 main process
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

void mainloop()
{

	uint32				rx ;
	int32				temp ;
	int					x ;
	int					y ;
	int					status ;
	uint32				nowms ;
	uint32				tempu ;
	packetblock_t*		packetblockptr ;
	char				temps [256] ;
	int 				maxr ;
	int 				maxc ;

/*
	xprintf ("Listening for commands\r\n") ;
    xprintf ("=======================================================================================================\r\n") ;                         
    xprintf ("=======================================================================================================\r\n") ;                         
///	xprintf ("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\r\n\r\n\r\n\r\n\r\n\r\n\r\n") ;
*/

	lastinfotime 	 	= monotime_ms() ;
	lminfoutenabled  	= 1 ; 								// enable the 4 line status display
	inicommandenabled 	= 1 ;								// enable thread to send ini commands

// main loop

	lastcommandcheck_ms = monotime_ms() ;
	last_broadcast_time = monotime_ms() ;	

	inhibit_lmprint = 1 ;

	while (1)
    { 
    	if (reboot_request)
    	{
    		for (x = 0 ; x < 10 ; x++)
    		{
				sleep_ms (100) ;
				if (xsr10.status == 1)
				{
					xsr10.status = 2 ;					// ignore send requests from core1
				}
    		}
			sleep_ms (1000) ;
			irq_set_enabled (DMA_IRQ_0, false) ; 		// disable DMA interrupts
			multicore_reset_core1() ;
			set_activity_led (1, OFF) ;
			set_activity_led (2, OFF) ;
			gpio_put (ZLED, 0) ;
			gpio_put (WIZRESET, 0) ;
			sleep_ms (200) ;
			if (reboot_request == 4)
			{
				printf ("BASE IP PORT command has been received \r\n") ;
				printf ("Pico is rebooting\r\n\r\n") ;
				watchdog_reboot (0, 0, 200) ;
				while (1) ;
			}
			if (reboot_request == 3)
			{
				printf ("BOOTSEL command has been received \r\n") ;
				printf ("Pico is going into BOOTSEL mode\r\n\r\n") ;
				sleep_ms (200) ;
				reset_usb_boot (0, 0) ;
				while (1) ;
			}
			else if (reboot_request == 2)
			{
				printf ("REBOOT command has been received \r\n") ;
				printf ("Pico is rebooting\r\n\r\n") ;
				watchdog_reboot (0, 0, 200) ;
				while (1) ;
    		}
			else if (reboot_request == 1)
			{
				printf ("RESET command has been received \r\n") ;
				printf ("Pico is rebooting\r\n\r\n") ;
				watchdog_reboot (0, 0, 200) ;
				while (1) ;
    		}
    	}   	

		inicommand_loop() ;													// send ini commands one by one
		status = dhcp_scheduler (4) ;
		
		if (freechainempty > 1)
		{
			if (monotime_ms() - freechainempty >= 1000)
			{
				reboot_request = 2 ;
				continue ;
			}
		}
    	else if (freechainempty == 1)
    	{
			freechainempty = monotime_ms() ;
			
			sprintf (temps,"\r\nFree:%d\r\n",freechain.count) ;
			xprintf (temps) ;
			for (rx = 1 ; rx <= MAXRECEIVERS ; rx++)
			{
				sprintf (temps,"RX%d:%d\r\n",rx,rcv[rx].rxchain.count) ;
				xprintf (temps) ;
			}
    	}

// activity LEDs

		if (dhcp_debug == 0)
		{
			for (x = 1 ; x <= 2 ; x++)
			{
				if (tsflash_value == 0)									// activity LEDs static on when receiving
				{
					if (last_dmatoggle[x] != dmatoggle[x])
					{				
						last_dmatoggle[x] 	   = dmatoggle[x] ;
						last_dmatoggle_time[x] = monotime_ms() ;
						set_activity_led (x, ON) ;
					}
					else if (monotime_ms() - last_dmatoggle_time[x] >= 100)
					{
						set_activity_led (x, OFF) ;
					}
				}
				else
				{
				   	if (monotime_ms() - last_dmatoggle_time[x] >= 1000)
	    			{
						last_dmatoggle_time[x] = monotime_ms() ;
	    				if (last_dmatoggle[x] == dmatoggle[x])
		    			{
							dmatoggle[x] 		= 0 ;
							last_dmatoggle[x]	= 0 ;
	    				}
						last_dmatoggle[x] = dmatoggle[x] ;
					}
					tempu = dmatoggle[x] % 36 ;
					if (dhcp_success)
					{
						if (tempu == 0)
						{
							set_activity_led (x, ON) ;
						}
						else if (tempu == 18)
						{
							set_activity_led (x, OFF) ;
						}
					}
				}
			}	
		}
		
		if (xsr10.status == 1)											// there is a send request from core 1
		{
			rx = xsr10.receiver ;

			if (rcv[rx].ethusbpath == 1)								// ETH output
			{
				if (rcv[rx].toaddress32)
				{
					status = sendto (rcv[rx].socket, (void*)(&xsr10.buffer), xsr10.length, rcv[rx].toaddress, xsr10.port) ;
				}
			}
			xsr10.status = 2 ;											// command processed
		}

 		for (rx = 1 ; rx <= MAXRECEIVERS ; rx++)
		{		
			if (rcv[rx].piorunning == 1)
			{
				pio_sm_set_enabled (rcv[rx].piogroup, rcv[rx].piosm, false) ;				
				rcv[rx].piorunning = 0 ;
				for (y = 0 ; y < 2 ; y++)
				{
					dma_channel_set_irq0_enabled (rcv[rx].piodmachan[y], false);	// disable DMA interrupts for this channel
					dma_channel_abort (rcv[rx].piodmachan[y]) ;                       
				}
				for (y = 0 ; y < 2 ; y++)
				{
					dma_hw->ch[rcv[rx].piodmachan[y]].write_addr = (uint32) &rcv[rx].piodmapacketblock[y]->buffer ;					// save the buffer address
					dma_hw->ints0 = (((uint) 1) << rcv[rx].piodmachan[y]) ;		// clear any interrupts
					dma_channel_set_irq0_enabled (rcv[rx].piodmachan[y], true);	// enable DMA interrupts for this channel
				}		
				dma_channel_start  	(rcv[rx].piodmachan[0]) ;                       
				pio_sm_clear_fifos  (rcv[rx].piogroup, rcv[rx].piosm) ;
				pio_sm_restart      (rcv[rx].piogroup, rcv[rx].piosm) ;					
				pio_sm_set_enabled	(rcv[rx].piogroup, rcv[rx].piosm, true);				
				rcv[rx].piorunning = 2 ;
			}
		}

// send multi TS blocks via Ethernet 

// find receiver with most data waiting to be transmitted

		maxr = 0 ;
		maxc = 0 ;
 		for (rx = 1 ; rx <= MAXRECEIVERS ; rx++)
		{
			temp = rcv[rx].rxchain.count ;
			if (temp > maxc)
			{
				maxc = temp ;
				maxr = rx ;
			}
		}

		if (maxr)
		{
			rx = maxr ;
			if (freechainempty == 0)
			{				
				dma_interrupts_off() ; 									// avoid chaining conflicts
				packetblockptr = getfromchain (&rcv[rx].rxchain) ;		// get the first block in the chain	
				dma_interrupts_on() ;
				
				packetblockptr->packetsinblock 	= PPB ;
				packetblockptr->receiver 		= rx ;
				tsproc_loop ((void*)packetblockptr) ;					// extract info from the packets

				if (rcv[rx].ethusbpath == 1)							// ETH
				{
					if (rcv[rx].toaddress[0] && packetblockptr->packetsinblock)
					{
						packetblockptr->length = packetblockptr->packetsinblock * TSPACKETSIZE ;
						status = 	sendto 
									(
										rcv[rx].socket, 
										(void*)&packetblockptr->buffer, 
										packetblockptr->length, 
										rcv[rx].toaddress, 
										rcv[rx].toport
									) ;				
					}
				}
				packetcount += packetblockptr->packetsinblock ;

				dma_interrupts_off() ;
				addtochain (&freechain, packetblockptr) ;				// free the packet block
				dma_interrupts_on() ;
			}
		}
	
// check for Ethernet commands received and place into buffer for core 1        

		if (tur01e.status != 1 && monotime_ms() - lastcommandcheck_ms >= 250)
		{
			lastcommandcheck_ms = monotime_ms() ;
			for (rx = 0 ; rx <= MAXRECEIVERS ; rx++)
			{
				status = 0 ;
				status = recvfrom 
				(
					rcv[rx].socket, 
					(void*) &tur01e.buffer, 
					sizeof  (tur01e.buffer) - 1, 
					(void*) &(tur01e.address), 
					(void*) &tur01e.port
				) ;

				if (status > 0)
				{
					if (rx == 0)
					{
						tur01e.receivedonport = (baseipport / 100) * 100 + PORTLISTENBASE ;
					}
					else
					{
						tur01e.receivedonport = baseipport + PORTLISTENBASE + rx ;
					}
///					rcv[rx].ethusbpath 		= 1 ;						// use ETH for TS output
					tur01e.buffer [status] 	= 0 ;						// terminating zero
					tur01e.receiver 		= rx ;
					tur01e.status			= 1 ;						// signal ready										
					break ;
				}

// check for timeout for TS leaving the building

				if (rcv[rx].active)
				{
					if (rcv[rx].scanstate != STATE_TIMEOUT && rcv[rx].scanstate != STATE_IDLE)
					{
						if (rcv[rx].iptype == IP_OFFNET) 				// sending off my subnet	
						{
							tempu = offnettime * 1000 ;
							if ((monotime_ms() - rcv[rx].commandreceivedtime >= tempu) && offnettime)
							{
								rcv[rx].scanstate = STATE_TIMEOUT ;		
								rcv[rx].timeoutholdoffcount = 4 ;		// send info 4 times after timeout
								rcv[rx].commandreceivedtime = 0 ;
								rcv[rx].timedouttime = monotime_ms() ;	// time of timeout

// send a null command to turn off the receiver

								rcv[rx].scanstate = STATE_TIMEOUT ;
								memset (&tur01e, 0, sizeof(tur01e)) ;
								sprintf 
								(
									tur01e.buffer, 
									"[to@wh] rcv=%d freq=0 srate=%d offset=0 fplug=%c",
									rx,
									rcv[rx].symbolrates[0],
									rcv[rx].antenna + '@'									
								) ; 
 				               	tur01e.status = 1 ;                 	// signal ready to Core1
                                break ;
							}												
						}
					}
				}
			}
		}
        
// check for signal loss and set idle mode

        nowms = monotime_ms() ;
      	for (rx = 1 ; rx <= MAXRECEIVERS ; rx++)
		{
			temp = 0 ;
       		if (rcv[rx].active && rcv[rx].scanstate != STATE_IDLE)
       		{
       			if (rcv[rx].scanstate == STATE_LOST)
       			{
       				if (nowms - rcv[rx].signallosttime >= idletime * 1000)
       				{
						temp++ ;										// can go to idle (turn off receiver) 
					}
       			}
       			if (rcv[rx].scanstate == STATE_SEARCH)
       			{
       				if (nowms - rcv[rx].commandreceivedtime >= idletime * 1000)
       				{
						temp++ ;										// can go to idle
       				}
       			}	
       			if (rcv[rx].scanstate == STATE_TIMEOUT)
       			{
       				if (nowms - rcv[rx].timedouttime >= idletime * 1000)
       				{
						temp++ ;										// can go to idle
       				}
       			}	
       			if (temp && idletime)
       			{
					rcv[rx].scanstate   			= STATE_IDLE ;		
					rcv[rx].rawinfos[STATUS_STATE] 	= STATE_IDLE ;		
    	   			rcv[rx].signallosttime 			= 0 ;       			
					rcv[rx].timeoutholdoffcount 	= 4 ;	// send info 4 times after timeout
					GLOBALNIM = rcv[rx].nim ;
					stv6120_init 							// configure receiver
					(
						rcv[rx].nimreceiver, 0,				// turn off the receiver
						rcv[rx].antenna,	 rcv[rx].symbolrates[0]				
					) ;												
    	   		}
    	   	}
       	}	               
    }	
}


void set_activity_led (int32 ledno, uint32 state)
{
	if ((ledno == 1 || ledno == 2))
	{
		if (activity_ledgp [ledno] >= 0)
		{
			gpio_put (activity_ledgp [ledno], state) ;
		}
	}
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@	 get the type of an IP address
//@
//@	 Calling:	IP address (string)
//@
//@	 Return:	
//@				-1  error
//@				 0  IP_OFFNET
//@				 1  IP_MYPC
//@				 2  IP_MYNET
//@				 3  IP_MULTI
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

int32 getiptype (char *testipaddress)
{ 
	uint32		test ;
	uint32		myip ;
	uint32		mymask ;
	uint32		x ;
	int			typex ;
	
	myip   	= *((uint32*) &netinfo.ip) ; 
	mymask 	= *((uint32*) &netinfo.sn) ; 
	test 	= *((uint32*) testipaddress) ; 

	if (test == 0 || test == myip)
	{
		typex = IP_MYPC ;
	}
	else if ((test & mymask) == (myip & mymask))
	{
		typex = IP_MYNET ;
	}
	else
	{
		typex = IP_OFFNET ;
	}
		
	return (typex) ;
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@	 Get monotonic time in milliseconds
//@
//@	 Calling:	
//@
//@	 Return:	time in milliseconds
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

uint32 monotime_ms() 
{ 
	absolute_time_t		timex ;
	uint32				timems ;			
	
	timex  = get_absolute_time() ;
	timems = to_ms_since_boot (timex) ;
	return (timems) ;
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@	 send commands from the .ini file        
//@
//@	 Calling:	
//@
//@	 Return:	
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

void inicommand_loop()
{
	uint32		x ;
	uint32		y ;
	uint32		tempu ;
	int32		status ;
	char*		pos ;
	char		temps [256] ;
	
	if (inicommandnext < inicommandcount)
	{
		x = inicommandnext ;
   		if ((strlen(inicommands[x]) == 1) && isdigit(inicommands[x][0]))	// single digit
   		{
			tempu = inicommands[x][0] & 0xf ;
			if (inicommandtimer == 0)
			{
				inicommandtimer = monotime_ms() ;
				sprintf (temps, "Delaying %d seconds\r\n", tempu) ;
///				xprintf (temps) ;
			}
			else if (monotime_ms() - inicommandtimer >= tempu * 1000) 
			{
				inicommandtimer = 0 ;
				inicommandnext++ ;
			}
		}	
		else
		{
			while (tur01e.status == 1) ;							// wait for structure free
			memcpy ((void*)&tur01e.buffer, (void*)&inicommands[x], strlen(inicommands[x]) + 1) ;
			tur01e.address32	= 0 ;  
			tur01e.port 		= baseipport ;
			tur01e.port 		= 0 ;
			pos = strstr (strupr(inicommands[x]),"RCV=") ;			// inicommand must have rcv=
			if (pos)
			{
				tur01e.receiver 	= -1 ;					
				tur01e.status		= 1 ;							// ready
			}
			inicommandnext++ ;
///			xprintf ("Sending ini file command\r\n") ;
   		}
   	}
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@	 tsproc
//@
//@	 extract info from a packetblock of PPB packets
//@
//@	 Calling:	
//@
//@	 Return:	
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
    	
int tsproc_loop (packetblock_t* pbp)
{
		int			x ;
		uint8		tempc ;
		uint32		tempu ;
		char		temps  [256] ;
		char		temps2 [256] ;
		uint32		index ;
		uint32		rx ;
		int32		status ;
		uint8		*sdtp ;
		uint16		pid ;
		uint32		length ;
		uint32		length2 ;
		uint32		y ;
		uint32		sectionlength ;
		uint32		servicetype ;
		uint32		audioservicetype ;
		uint32		videoservicetype ;
		uint32		prognumber ;
		uint32		pmtpid ;
		uint32		programcount ;
		uint32		changedetected ;
		uint8*		pp ;												// individual packet pointer		
		uint32		validpacketindex ;
		
		rx = pbp->receiver ;											// receivers are numbered 1-4						
		validpacketindex = 0 ;

		for (x = 0 ; x < PPB ; x++)
		{
			pp = pbp->packets[x] ;
			if 
			(
				(rcv[rx].active == 0) || 
				(rcv[rx].scanstate != STATE_DEMOD_S && rcv[rx].scanstate != STATE_DEMOD_S2)) 
			{
			}
	       	else if (pp[0] != 0x47)										// sync byte missing
	       	{
				rcv[rx].errors_sync++ ;		       						
				rcv[rx].packetcountprogram++ ;							
				return (0) ;
	      	}
	       	else 
	       	{	       		       				
				pid = (pp[1] & 0x1f) * 0x100 + pp[2] ;

				if (x != validpacketindex)
				{
					memcpy (pbp->packets[validpacketindex], pbp->packets[x], TSPACKETSIZE) ;
				}
				validpacketindex++ ;

				rcv[rx].packetcountprogram++ ;				
				rcv[rx].packetcountrx++ ;				
				if (pid == NULL_PID || (pid == NULL2_PID && null8190))
				{
					rcv[rx].nullpacketcountprogram++ ;				
					rcv[rx].nullpacketcountrx++ ;				
				}
					
// select the first PMT with program number > 0

			if (pid == PAT_PID)
			{
				index = 5 ;
				sectionlength = ((pp[index+1] & 0xf) << 8) + pp[index+2] ;
				if (sectionlength <= 1 || sectionlength >= 188)
				{
					sectionlength = 0 ;
				}
				if (sectionlength)
				{
					tempu = calculateCRC32 ((uint8*)&pp[index],sectionlength-1) ;
					tempu = reverse (tempu) ;							// reverse byte order
					index += sectionlength - 1 ;
					if (tempu == get32(&(pp[index])))					// CRC32 OK
					{
						pmtpid = 0 ; 									// PMT to use
						programcount = 0 ;								// number of programs
						index = 0x0d ;
						while (index < 0x08 + sectionlength - 4)
						{
							prognumber = pp [index] * 0x100 + pp [index + 1] ;
							index += 2 ;								// point to PMT pid
							tempu = (pp [index] & 0x1f) * 0x100 + pp [index + 1] ;
							if (prognumber != 0) 
							{ 
								if ((rcv[rx].requestedprog == 0) || (prognumber == rcv[rx].requestedprog))									
								{
									if (pmtpid == 0)
									{
										pmtpid = (pp [index] & 0x1f) * 0x100 + pp [index + 1] ;
									}
								}	
								programcount++ ;
							}
							index += 2 ;
						}
						rcv[rx].pmtpid = pmtpid ;
						rcv[rx].programcount = programcount ;
					}
				}
			}

// extract the video and audio service types from the PMT

			else if ((pid == rcv[rx].pmtpid) && rcv[rx].pmtpid)			// the program we want
			{	
				index = 5 ;
				sectionlength = ((pp[index+1] & 0xf) << 8) + pp[index+2] ;
				if (sectionlength <= 1 || sectionlength >= 188)
				{
					sectionlength = 0 ;
				}
				audioservicetype = 0 ;
				videoservicetype = 0 ;
				servicetype      = 0 ;								
				if (sectionlength)
				{
					tempu = calculateCRC32 ((uint8*)&pp[index],sectionlength-1) ;
					tempu = reverse (tempu) ;							// reverse byte order
					index += sectionlength - 1 ;
					if (tempu == get32(&(pp[index])))					// CRC32 OK
					{					
						index = 0x0d ;										// point to first entry
// get the PCR pid
						tempu = (pp [index] & 0x1f) * 0x100 + pp [index + 1] ;
						rcv[rx].pcrpid = tempu ;
						index += 2 ;
						tempu = (pp [index] & 0x0f) * 0x100 + pp [index + 1] ; // info length
						index += tempu + 2 ;							// skip over the info										

						audioservicetype = 0 ;
						videoservicetype = 0 ;
						servicetype      = 0 ;								

// look for service types
						while (index < 0x08 + sectionlength - 4)			
						{							
							servicetype = pp[index] ;					// H264, AAC etc	
							index++ ;
							index += 2 ;
							tempu = (pp [index] & 0x0f) * 0x100 + pp [index + 1] ; // info length
							index += tempu + 2 ;								// skip over the info																

							switch (servicetype)
							{
								case SERVICE_H262:
								case SERVICE_H264:
								case SERVICE_H265:
								case SERVICE_H266:
									videoservicetype = servicetype ;
								break ;
								case SERVICE_MPA:
								case SERVICE_AAC:
								case SERVICE_AC3:
									audioservicetype = servicetype ;
								break ;
							}	
						}
					}							
							
					changedetected = 0 ;
					y = STATUS_VIDEO_TYPE ;
					if 
					(
						((int)videoservicetype != rcv[rx].rawinfos[y]) && 
						videoservicetype
					)
					{
						if (rcv[rx].rawinfos[y] != 0 || rcv[rx].vlcstopped)
						{
							 changedetected++ ;					
						}			
						rcv[rx].rawinfos[y] = videoservicetype ;
						switch (videoservicetype)	
						{					
							case SERVICE_H262:
								sprintf (rcv[rx].textinfos[y], "H262") ; break ;
							break ;
							case SERVICE_H264:
								sprintf (rcv[rx].textinfos[y], "H264") ; break ;
							break ;
							case SERVICE_H265:
								sprintf (rcv[rx].textinfos[y], "H265") ; break ;
							break ;
							case SERVICE_H266:
								sprintf (rcv[rx].textinfos[y], "H266") ; break ;
							break ;
							default:
								sprintf (rcv[rx].textinfos[y], "%s", "") ; break ;
							break ;											
						} ;
					}

					y = STATUS_AUDIO_TYPE ;
					if 
					(
						((int)audioservicetype != rcv[rx].rawinfos[y]) && 
						audioservicetype
					)
					{
						if (rcv[rx].rawinfos[y] != 0 || rcv[rx].vlcstopped)
						{
							 changedetected++ ;					
						}			
						rcv[rx].rawinfos[y] = audioservicetype ;
						switch (audioservicetype)
						{
							case SERVICE_MPA:
								sprintf (rcv[rx].textinfos[y], "MPA") ; break ;
							break ;
							case SERVICE_AAC:
								sprintf (rcv[rx].textinfos[y], "AAC") ; break ;
							break ;
							case SERVICE_AC3:
								sprintf (rcv[rx].textinfos[y], "AC3") ; break ;
							break ;
							default:
								sprintf (rcv[rx].textinfos[y], "%s", "") ; break ;
							break ;
						}					
					}

					if (changedetected)
					{										
						rcv[rx].vlcstopped = 0 ;
						y = STATUS_VIDEO_TYPE ;
						rcv[rx].rawinfos[y]= videoservicetype ;
						y = STATUS_AUDIO_TYPE ;
						rcv[rx].rawinfos[y] = audioservicetype ;
						rcv[rx].modechanges++ ;							// count a mode change
						rcv[rx].vlcnextcount++ ;
					}
				}
			}
			else if (pid == SDT_PID)									// service descriptor
			{
				index = 5 ;
				sectionlength = ((pp[index+1] & 0xf) << 8) + pp[index+2] ;
				if (sectionlength <= 1 || sectionlength >= 188)
				{
					sectionlength = 0 ;
				}
				if (sectionlength)
				{
					tempu = calculateCRC32 ((uint8*)&pp[index],sectionlength-1) ;
					tempu = reverse (tempu) ;							// reverse byte order
					index += sectionlength - 1 ;
					if (tempu == get32(&(pp[index])))
					{
						sdtp = (uint8*) pp ;
						sdtp += 16 ;									// point to the first entry
						rcv[rx].serviceid  = *sdtp++ * 0x100 ;			// 16 bit service ID, high/low
						rcv[rx].serviceid += *sdtp++ ;
						sdtp += 3 ;										// point to the descriptor tag
						if (*sdtp == 0x48)								// service descriptor
						{					
							sdtp += 3 ;									// point to the provider length
							length = *sdtp++ ;							// provider length
							if (length <=  15)
							{
								length2 = length ;
							}
								else
							{
								length2 = 15 ;
							}
							y = STATUS_SERVICE_PROVIDER_NAME ; 
							strncpy (rcv[rx].textinfos[y], (void*)sdtp, length2) ; // copy the provider name
							rcv[rx].textinfos[y][length2] = 0 ;			// terminate the string
							sdtp += length ;							// point to the service name length 
							length = *sdtp++ ;							// service name length 
							if (length <=  15)
							{
								length2 = length ;
							}
								else
							{
								length2 = 15 ;
							}
							y = STATUS_SERVICE_NAME ; 					// callsign
							if (rcv[rx].programcount != 1 && rcv[rx].textinfos[y][0])
							{
								strcpy (temps, "+") ;
								strncat (temps, (void*)sdtp, length2) ; // copy the service name
								temps [length2+1] = 0 ;					// terminate the string	
							}
							else
							{
								strcpy (temps, "") ;
								strncat (temps, (void*)sdtp, length2) ; 	// copy the service name
								temps [length2] = 0 ;						// terminate the string	
							}
							if (strcmp(temps,rcv[rx].textinfos[y]) != 0)
							{
								rcv[rx].modechanges++ ;					// count a mode change			
								rcv[rx].packetcountrx  	   		= 0 ;	// clear packet count		
								rcv[rx].nullpacketcountrx  		= 0 ;	// clear null packet count	
								if (rcv[rx].textinfos[y][0] || rcv[rx].vlcstopped)
								{
									rcv[rx].vlcstopped = 0 ;
									rcv[rx].vlcnextcount++ ;
									rcv[rx].rawinfos[STATUS_VLCNEXTS] = rcv[rx].vlcnextcount ;
								}
							}
							strcpy (rcv[rx].textinfos[y], temps) ;		// copy the service name
						}
						sdtp += length ;								
					}
				}
			}				
		}		    	   
	} 
	return (1) ;
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@	 setup_titlebar
//@
//@	 create a string to put into the VLC title bar
//@
//@	 Calling:	buffer to populate
//@				receiver number
//@
//@	 Return:	
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

void setup_titlebar (char *output, uint32 rx)
{
	int				x ;
	int				y ;
	char			temps  [256] ;
	char			*pos ;
	
	sprintf (output, "%d:", rx + rxbase) ;

// print status	
	
	if (rcv[rx].scanstate == STATE_IDLE)
	{
		sprintf (output+strlen(output), " *idle*") ;
	}
	else
	{
		if (rcv [rx].scanstate != STATE_DEMOD_S2 && rcv[rx].scanstate != STATE_DEMOD_S)
		{
			y = STATUS_STATE ;
			sprintf (temps, "%s", rcv[rx].textinfos[y]) ;
			for (x = 0 ; x < (int)strlen(temps) ; x++)
			{
				temps[x] = tolower(temps[x]) ;
			}
			sprintf (output+strlen(output), " *%s*", temps) ;
		}
	
// display callsign, truncate at non alphanumeric

        y = STATUS_SERVICE_NAME ;                                       // callsign
        sprintf (temps, "%s", rcv[rx].textinfos[y]) ;
		validate_callsign (temps) ;
        sprintf (output+strlen(output)," %s",temps) ;

// display MER		
		
		if (rcv[rx].scanstate == STATE_DEMOD_S2 || rcv[rx].scanstate == STATE_DEMOD_S)
		{
			y = STATUS_MER ;
			if (rcv[rx].textinfos[y][0])
			{
				sprintf (output+strlen(output)," M%s",rcv[rx].textinfos[y]) ;
			}
			y = STATUS_DNUMBER ;
			if (rcv[rx].textinfos[y][0])
			{
				sprintf (output+strlen(output)," D%s",rcv[rx].textinfos[y]) ;
			}
		}
			
// display TS type and video type
			
		if (rcv[rx].scanstate == STATE_DEMOD_S2 || rcv[rx].scanstate == STATE_DEMOD_S)
		{
			y = STATUS_STATE ;
			if (rcv[rx].forbidden)
			{
			sprintf (output+strlen(output), "%s", " *") ;
			}
			else
			{
				sprintf (output+strlen(output), "%s", " ") ;
			}
			sprintf (temps, "%s", rcv[rx].textinfos[y]) ;
			if (strlen(temps))
			{
				if (strncmp(temps, "DVB-", 4) == 0)
				{
					strcpy (temps,temps+4) ;						// S or S2
				}		
				sprintf (output+strlen(output), "%s", temps) ;
			}	
			y = STATUS_VIDEO_TYPE ;								// H265 etc
			sprintf (temps, "%s", rcv[rx].textinfos[y]) ;
			if (strlen(temps) >= 4)
			{
				sprintf (output+strlen(output), "%c%c", temps[0], temps[3]) ;
			}
			if (rcv[rx].forbidden)
			{
				sprintf (output+strlen(output),"%s","*") ;
			}			
		}
		
// display modulation and FEC			

		if (rcv[rx].scanstate == STATE_DEMOD_S2 || rcv[rx].scanstate == STATE_DEMOD_S)
		{
			y = STATUS_MODCOD ;
			sprintf (temps,"%s",rcv[rx].textinfos[y]) ;
			pos = strstr (temps,"QPSK") ;
			if (pos)
			{
				strcpy (pos+2, pos+5) ; 
			}
			else 
			{
				pos = strstr (temps,"8PSK") ;
				if (pos)
				{
					strcpy (pos+2, pos+5) ; 
				}
				else
				{
					pos = strstr (temps,"APSK") ;
					if (pos)
					{
						strcpy (pos+1, pos+5) ; 
					}
				}
			}
			sprintf (output+strlen(output)," %s",temps) ;
		}
		
// display SR and frequency
		
		if (rcv[rx].scanstate == STATE_DEMOD_S2 || rcv[rx].scanstate == STATE_DEMOD_S)
		{
/*
			sprintf (temps,"%0.3fM", (float)rcv[rx].frequencies[0] / 1000) ;
			pos = strstr (temps,".") ;
			if (pos && (pos != temps))
			{
				pos-- ;
			}
			else
			{
				pos = temps ;
			}
			sprintf (output+strlen(output), " %d", rcv[rx].symbolrates[0]) ;
			sprintf (output+strlen(output), " %s", pos) ;	
*/
			sprintf (output+strlen(output), " %s",rcv[rx].textinfos [STATUS_SYMBOL_RATE]) ;
			
			sprintf (temps, " %s", rcv[rx].textinfos [STATUS_CARRIER_FREQUENCY]) ;
			pos = strstr (temps,".") ;
			if (pos && (pos != temps))
			{
				pos-- ;
			}
			else
			{
				pos = temps ;
			}
			sprintf (output+strlen(output), " %sM", pos) ;
		}
		else
		{
			sprintf (output+strlen(output), " SR%d", rcv[rx].symbolrates[0]) ;
			sprintf (output+strlen(output), " %0.3fMHz", (float)rcv[rx].frequencies[0] / 1000) ;
		}
		
// display antenna
		
		y = STATUS_ANTENNA ;
		sprintf (output+strlen(output), " %c", rcv[rx].textinfos[y][0]) ;		

// extra terminating zero		

		sprintf (output+strlen(output), "%c%c", 0, 0) ;				
	}	
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@	 create an EIT packet that will appear in the VLC title bar
//@
//@	 Calling:	buffer to populate
//@				receiver number
//@				text string to insert
//@
//@	 Return:	
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

void setup_eit (void *packet, uint32 rx, char* info)
{
    int				y ;
    uint8       	*p, *q ;
    uint        	utempi ;    
    char        	temps  [256] ;
	int32			jday ;
	int32			jhour ;	
	struct eitx		*eitp ;

	eitp = (struct eitx*) packet ;

    memset ((void*)eitp, 0, sizeof(*eitp)) ;				

    eitp->sync = 0x47 ;
    eitp->payloadstart 	= 1 ;
    eitp->adaption 		= 1 ;
    eitp->pid1208 		= EIT_PID >> 8 ;
    eitp->pid0700 		= EIT_PID & 0xff ;

	eitp->continuity = rcv[rx].eitcontinuity++ & 0xf ;	// increment continuity
     
    eitp->tableid 		= 0x4e ;               				// EIT 'current program' descriptor

    eitp->reserved0 	= 3 ;
    eitp->filler2 		= 1 ;
    eitp->syntax 		= 1 ;

	eitp->sectionlength = 0 ;								// fill in later

    eitp->servicehigh 	= (uint8) (rcv[rx].serviceid >> 8) ;						
    eitp->servicelow  	= (uint8) (rcv[rx].serviceid & 0xff) ;						

    eitp->currentnext 	= 1 ;
    eitp->reserved1   	= 3 ;

    eitp->version 	 	= rcv[rx].eitversion & 0x1f ;		// table table version

    eitp->tsidhigh  	= (uint8) (TSID >> 8) ;				// TSID = 0; not needed by VLC for EIT					
    eitp->tsidlow   	= (uint8) (TSID & 0xff) ;						
    eitp->networkhigh 	= (uint8) (NETWORK >> 8) ;			// NETWORK = 0; not needed by VLC for EIT
    eitp->networklow  	= (uint8) (NETWORK & 0xff) ;
    eitp->lasttable 	= 0x4e ;                 			// service descriptor
    
    p = (uint8*) &eitp->loop ;
 
    *p++ = (uint8) (EVENTID >> 8) ;             			// event id = zero (first event)
    *p++ = (uint8) (EVENTID & 0xff) ;

    juliandate (&jday,&jhour) ;
	jhour -= 1 ;											// start at the beginning of the previous hour
	if (jhour < 0)
	{
		jhour += 24 ;
		jday-- ;
	}
    
    *p++ = (uint8) (jday >> 8) ;                			// start date
    *p++ = (uint8) (jday & 0xff) ;

	utempi = (jhour / 10) ;									// convert to BCD
	utempi *= 0x10 ;
	utempi += (jhour % 10) ;

    *p++ = utempi ;            				       			// start time
    *p++ = 0x00 ;
    *p++ = 0x00 ;
    
    *p++ = 0x03 ;                           				// duration 3 hours
    *p++ = 0x00 ;	
    *p++ = 0x00 ;

    *p++ = 0x80  ;    			                        	// status = running, scrambled = no
        
    q = p ;                                     

    *p++ = 0 ;									    		// descriptors loop length - fill in later

    *p++ = 0x4d ;                               			// short descriptor type 
    
    *p++ = 0 ;                                  			// descriptor length - fill in later

   	sprintf ((char*)p, "eng") ;
    p += 3 ;

	strcpy (temps, info) ; 
	
    *p++ = (uint8) strlen (temps) ;
	sprintf ((char*)p, temps) ;
    p += strlen (temps) ;

	y = STATUS_SERVICE_PROVIDER_NAME ;
    strcpy ((char*)temps, rcv[rx].textinfos[y]) ;

    *p++ = (uint8) strlen (temps) ;
	sprintf ((char*)p, temps) ;
    p += strlen (temps) ;

    *(q+2) = (uint8) (p - q - 1 - 2) ;

    *q = (uint8) (p - q - 1) ;
    
    eitp->sectionlength = (uint8) ((uint32)p - (uint) &eitp->sectionlength - 1 + 4) ;
    
    utempi = calculateCRC32 ((uint8*)&eitp->tableid,eitp->sectionlength-1) ;
    *p++ = (uint8) ((utempi >> 24) & 0xff) ;
    *p++ = (uint8) ((utempi >> 16) & 0xff) ;
    *p++ = (uint8) ((utempi >>  8) & 0xff) ;
    *p++ = (uint8) ((utempi >>  0) & 0xff) ;

    memset (p, 0xff, 188-((uint)p-(uint)eitp)) ;	// pad out the rest of the packet
}

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@	 create an EIT packet that will appear in the VLC title bar
//@
//@	 Calling:	buffer to populate
//@				receiver number
//@
//@	 Return:	
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

void setup_eit2 (void *packet, uint32 rx, char* info)
{
    int				y ;
    uint8       	*p, *q ;
    uint        	utempi ;    
    char        	temps  [256] ;
	int32			jday ;
	int32			jhour ;	
	struct eitx		*eitp ;

	eitp = (struct eitx*) packet ;

    memset ((void*)eitp, 0, sizeof(*eitp)) ;				

    eitp->sync = 0x47 ;
    eitp->payloadstart 	= 1 ;
    eitp->adaption 		= 1 ;
    eitp->pid1208 		= EIT_PID >> 8 ;
    eitp->pid0700 		= EIT_PID & 0xff ;

	eitp->continuity = rcv[rx].eitcontinuity++ & 0xf ;	// increment continuity
     
    eitp->tableid 		= 0x4e ;               				// EIT 'current program' descriptor

    eitp->reserved0 	= 3 ;
    eitp->filler2 		= 1 ;
    eitp->syntax 		= 1 ;

	eitp->sectionlength = 0 ;								// fill in later

    eitp->servicehigh 	= (uint8) (rcv[rx].serviceid >> 8) ;						
    eitp->servicelow  	= (uint8) (rcv[rx].serviceid & 0xff) ;						

    eitp->currentnext 	= 1 ;
    eitp->reserved1   	= 3 ;

    eitp->version 	 	= rcv[rx].eitversion & 0x1f ;		// table table version

    eitp->tsidhigh  	= (uint8) (TSID >> 8) ;				// TSID = 0; not needed by VLC for EIT					
    eitp->tsidlow   	= (uint8) (TSID & 0xff) ;						
    eitp->networkhigh 	= (uint8) (NETWORK >> 8) ;			// NETWORK = 0; not needed by VLC for EIT
    eitp->networklow  	= (uint8) (NETWORK & 0xff) ;
    eitp->lasttable 	= 0x4e ;                 			// service descriptor
    
    p = (uint8*) &eitp->loop ;
 
    *p++ = (uint8) (EVENTID >> 8) ;             			// event id = zero (first event)
    *p++ = (uint8) (EVENTID & 0xff) ;

    juliandate (&jday,&jhour) ;
	jhour -= 1 ;											// start at the beginning of the previous hour
	if (jhour < 0)
	{
		jhour += 24 ;
		jday-- ;
	}
    
    *p++ = (uint8) (jday >> 8) ;                			// start date
    *p++ = (uint8) (jday & 0xff) ;

	utempi = (jhour / 10) ;									// convert to BCD
	utempi *= 0x10 ;
	utempi += (jhour % 10) ;

    *p++ = utempi ;            				       			// start time
    *p++ = 0x00 ;
    *p++ = 0x00 ;
    
    *p++ = 0x03 ;                           				// duration 3 hours
    *p++ = 0x00 ;	
    *p++ = 0x00 ;

    *p++ = 0x80  ;    			                        	// status = running, scrambled = no
        
    q = p ;                                     

    *p++ = 0 ;									    		// descriptors loop length - fill in later

    *p++ = 0x4d ;                               			// short descriptor type 
    
    *p++ = 0 ;                                  			// descriptor length - fill in later

   	sprintf ((char*)p, "eng") ;
    p += 3 ;

	strcpy (temps, info) ; 
	
    *p++ = (uint8) strlen (temps) ;
	sprintf ((char*)p, temps) ;
    p += strlen (temps) ;

	y = STATUS_SERVICE_PROVIDER_NAME ;
    strcpy ((char*)temps, rcv[rx].textinfos[y]) ;

    *p++ = (uint8) strlen (temps) ;
	sprintf ((char*)p, temps) ;
    p += strlen (temps) ;

    *(q+2) = (uint8) (p - q - 1 - 2) ;

    *q = (uint8) (p - q - 1) ;
    
    eitp->sectionlength = (uint8) ((uint32)p - (uint) &eitp->sectionlength - 1 + 4) ;
    
    utempi = calculateCRC32 ((uint8*)&eitp->tableid,eitp->sectionlength-1) ;
    *p++ = (uint8) ((utempi >> 24) & 0xff) ;
    *p++ = (uint8) ((utempi >> 16) & 0xff) ;
    *p++ = (uint8) ((utempi >>  8) & 0xff) ;
    *p++ = (uint8) ((utempi >>  0) & 0xff) ;

    memset (p, 0xff, 188-((uint)p-(uint)eitp)) ;	// pad out the rest of the packet
}


void juliandate (int32* julday, int32* julhour)
{
    uint16          utempi ;
    int             x ;
    int             year ;
    int				month ;
    int				day ;
  	time_t 			timex ;
   	struct tm		mytime ;

	timex 	= time (NULL) ;
    mytime 	= *gmtime (&timex) ;

	year 	= mytime.tm_year ;
	while (year >= 100)
	{
		year -= 100 ;										// appears to start from 1900
	}
	month 	= mytime.tm_mon + 1 ;							// returned month = 0-11
	day 	= mytime.tm_mday ;

	printf ("%d %d %d %d %d %d\r\n",mytime.tm_year,mytime.tm_mon,mytime.tm_mday,mytime.tm_hour,mytime.tm_min,mytime.tm_sec) ; 

    utempi 	= DAY0 ;

    for (x = 0 ; x < year ; x++)
    {
        utempi += 365 ;
        if ((x & 3) == 0)
        {
            utempi++ ;
        }
    }

    for (x = 1 ; x < month ; x++)
    {
        utempi += daysinmonth [x] ;
        if (x == 2 && ((year & 3) == 0))
        {
            utempi++ ;
        }
    }

	utempi += day ;

	*julday = utempi ;
	*julhour  = mytime.tm_hour ;
}


uint32 calculateCRC32 (uint8* data, uint32 dataLength)
{
    uint32      crc ;
    uint32      poly, temp, temp2, temp4, bit31 ;
	uint32		x ;

	crc = 0xffffffff ;
	poly = 0x04c11db7 ;

   	while (dataLength-- > 0)
	{
		temp4 = poly ;
		temp2 = (crc >> 24) ^ *data ;
		temp = 0 ;

		for (x = 0 ; x < 8 ; x++)
		{
			if ((temp2 >> x) & 1)
			{
				temp ^= temp4 ;
			}

			bit31 = temp4 >> 31 ;
			temp4 <<= 1 ;
			if (bit31)
			{
				temp4 ^= poly ;
			}
		}
		crc = (crc<<8) ^ temp ;
		data++ ;
   	}
    return crc ;
}

// reverse the byte order in a word

uint32 reverse (uint32 indata)
{
	uint32		tempu ;

	tempu  = 0 ;
	tempu |= ((indata >> 24) & 0xff) <<  0 ;
	tempu |= ((indata >> 16) & 0xff) <<  8 ;
	tempu |= ((indata >>  8) & 0xff) << 16 ;
	tempu |= ((indata >>  0) & 0xff) << 24 ;
	
	return (tempu) ;
}

void getdatetime (char *string)
{
   	struct tm		mt ;
  	time_t 			timex ;

	timex 	= time (NULL) ;
    mt 		= *gmtime (&timex) ;

	sprintf (string, "%04d-%02d-%02d %02d:%02d:%02d", 1900+mt.tm_year, mt.tm_mon+1, mt.tm_mday, mt.tm_hour, mt.tm_min, mt.tm_sec) ;
}


void whexit (int32 reason)
{
	char		temps [256] ;

    printf ("===============================================================================================\r\n") ;                         
	sprintf (temps, "WinterHill stopped: (%d) ", reason) ;
	printf ("%s\r\n", temps) ;
    printf ("===============================================================================================\r\n") ;                         

	printf ("\r\n") ;

	sleep_ms (2000) ;
	exit (reason) ;
}	


// get the address of a packet from the start of a chain; return zero if none available

packetblock_t* __not_in_flash_func (getfromchain) (chain_t *chain)
{
	packetblock_t* 	returnpointer ;
	
	if (chain->start == 0)							// no packets available
	{
		returnpointer = 0 ;
	}
	else
	{
		returnpointer				= (packetblock_t*) chain->start ;
		chain->start				= (packetblock_t*) returnpointer->nextpointer ;
		returnpointer->nextpointer	= 0 ;
		if (chain->start == 0)
		{
			chain->end = 0 ;
		}
		chain->count-- ;
	}
	return (returnpointer) ;	
}

// packet chaining routines

void __not_in_flash_func (addtochain) (chain_t *chain, packetblock_t* add)
{
	if (chain->end == 0)
	{
		add->nextpointer	= 0 ;
		chain->start		= add ;
		chain->end			= add ;
	}
	else
	{
		(chain->end)->nextpointer	= (uint32) add ;
		chain->end					= add ;
		add->nextpointer			= 0 ;
	}
	chain->count++ ;
}


packetblock_t* __not_in_flash_func (check_received_pid) (packetblock_t* packetblockptr, chain_t* chain)
{
	uint16			pid ;
	uint16			ok ;
	packetblock_t*	newpacketblockptr ;	
	int				x ;

	ok = 1 ; 

	if (ok)
	{
		newpacketblockptr = getfromchain (&freechain) ;					// get a free packet
		if (newpacketblockptr == 0)
		{
			if (freechainempty == 0)
			{
				freechainempty = 1 ;									// indicate no more packets available
			}
		}
		else
		{
			addtochain (chain, packetblockptr) ;						// add the received packet to the output chain	
			packetblockptr = newpacketblockptr ;						// return the free packet address
		}
	}		

	return (packetblockptr) ;		
}

			
void __not_in_flash_func (dma_interrupts_off) ()
{
    irq_set_enabled (DMA_IRQ_0, false) ;
}					

				
void __not_in_flash_func (dma_interrupts_on) ()
{
    irq_set_enabled (DMA_IRQ_0, true) ;
}					


// DMA interrupt handler

void __not_in_flash_func (mydma_handler) ()
{   
	uint32			rx ;
	uint32			y ;
    uint32    		temp ;
	packetblock_t*	blockaddress ;
	
	for (rx = 1 ; rx <= MAXRECEIVERS ; rx++)
	{
		for (y = 0 ; y < 2 ; y++)
		{
			temp   = dma_hw->ints0 ;													// DMA channel interrupt flags
			temp  &= (((uint) 1) << rcv[rx].piodmachan[y]) ;
			if (temp)
			{
				dma_hw->ints0 = temp ;                                      			// clear the interrupt
				blockaddress   = rcv[rx].piodmapacketblock[y] ;							// current DMA write address
				blockaddress   = check_received_pid (blockaddress, &rcv[rx].rxchain) ;	// see if PID is required and add it to chain
				dma_hw->ch[rcv[rx].piodmachan[y]].write_addr = (uint32) &(blockaddress->buffer) ;	// write address for next transfer
				rcv[rx].piodmapacketblock[y] = blockaddress ;							// save the buffer address
			    dmatoggle[rx]++ ;
			}
		}
	}
}


int dhcp_scheduler (int command)
{
	int			status ;
	char		buff   [512] ;
	
	dhcp_current_time = monotime_ms() ;

	if (monotime_ms() - last_broadcast_time >= 1000)
	{
		last_broadcast_time = monotime_ms() ;
		if (dhcp_link_state)
		{
			form_broadcast_message (zbuff) ;		
			if (strlen(zbuff))
			{
				status = sendto (broadcast_socket, zbuff, strlen(zbuff) + 1, broadcast_address, PORTBROADCAST) ;
			}	
		}
	}
	
	if (netinfo.dhcp == NETINFO_STATIC)
	{
		if (dhcp_current_time - dhcp_last_runtime >= 125)
		{
			dhcp_last_runtime = dhcp_current_time ;
			dhcp_link_state = get_link_state() ;
			if (dhcp_link_state)
			{
				strcpy (dhcp_status_message, "Static-linked") ;
				gpio_put (ZLED, 1) ;
			}
			else
			{
				strcpy (dhcp_status_message, "Static-unlinked") ;
				gpio_put (ZLED, 0) ;
			}
		}
		return (0) ;
	}

	if (command == 0)
	{
		if (netinfo.dhcp == NETINFO_DHCP)
		{
			memset (&netinfo.ip,  0, sizeof(netinfo.ip)) ;
			memset (&netinfo.sn,  0, sizeof(netinfo.sn)) ;
			memset (&netinfo.gw,  0, sizeof(netinfo.gw)) ;
			memset (&netinfo.dns, 0, sizeof(netinfo.dns)) ;
			memset (&netinfo.rx,  0, sizeof(netinfo.rx)) ;
			memset (&netinfo.sv,  0, sizeof(netinfo.sv)) ;
		}
		
		dhcp_machine_state 	= DHCP_IDLE_STATE_0 ;
		dhcp_active			= 0 ;
		dhcp_success		= 0 ;
		gpio_put 			(ZLED, 0) ;
		set_activity_led 	(1, OFF) ;
		set_activity_led 	(2, OFF) ;
		return (0) ;
	}
	else if (command == 1)
	{
		if (dhcp_debug)
		{
			printf ("DHCP: starting \r\n") ;
		}
		dhcp_machine_state = DHCP_SENDING_DISCOVER_STATE_1 ;
		return (0) ;
	}
	else if (command == 2)
	{
		dhcp_debug = 0 ;
		return (0) ;
	}
	else if (command == 3)
	{
		dhcp_debug = 1 ;
		return (0) ;
	}
	else if (command == 4)
	{
		if (dhcp_current_time - dhcp_last_runtime < 125)
		{
			return (0) ;
		}
		else
		{
			dhcp_counter++ ;
			dhcp_last_runtime = dhcp_current_time ;
		}
		if (dhcp_machine_state == DHCP_IDLE_STATE_0)
		{
			return (0) ;
		}
	}
	
   	if (netinfo.dhcp == NETINFO_DHCP)
	{
		if (dhcp_success)
		{
			gpio_put (ZLED, 1) ;
		}
		else
		{
			if (dhcp_link_state == 0)
			{
				if ((dhcp_counter & 7) == 0)
				{
					gpio_put (ZLED, 1) ;
					set_activity_led (1, ON) ;
					set_activity_led (2, ON) ;
				}
				else
				{
					gpio_put (ZLED, 0) ;
					set_activity_led (1, OFF) ;
					set_activity_led (2, OFF) ;
				}
			}			
			else
			{
				if ((dhcp_counter & 7) == 0 || (dhcp_counter & 7) == 2)
				{
					gpio_put (ZLED, 1) ;
					set_activity_led (1, ON) ;
					set_activity_led (2, ON) ;
				}
				else
				{
					gpio_put (ZLED, 0) ;
					set_activity_led (1, OFF) ;
					set_activity_led (2, OFF) ;
				}
			}			
		}
	}

	dhcp_link_state = get_link_state() ;

	switch (dhcp_machine_state) 
	{
		case DHCP_SENDING_DISCOVER_STATE_1: 	 status = dhcp_sending_discover_state_1	(command) ; break ;	
		case DHCP_WAITING_OFFER_STATE_2: 		 status = dhcp_waiting_offer_state_2	(command) ; break ;
		case DHCP_WAITING_ACK_STATE_3:			 status = dhcp_waiting_ack_state_3		(command) ; break ;
		case DHCP_WAITING_TO_RETRY_STATE_4:		 status = dhcp_waiting_to_retry_state_4	(command) ; break ;
		case DHCP_OK_STATE_5:			 		 status = dhcp_ok_state_5				(command) ; break ;
		case DHCP_AWAITING_LINK_STATE_6: 		 status = dhcp_awaiting_link_state_6	(command) ; break ;
		case DHCP_SENDING_REQUEST_STATE_7: 		 status = dhcp_sending_request_state_7	(command) ; break ;
		default:								 status = dhcp_idle_state_0 			(command) ; break ;
	} ;

	return (status) ;
}


int dhcp_idle_state_0 (int command)
{
	return (0) ;
}


int dhcp_sending_discover_state_1 (int command)
{
	int 		x ;
	int			status ;
	char		temps [32] ;
	uint8		nullx [4] = {0,0,0,0} ;
			
	if (dhcp_link_state == 0)
	{
		if (dhcp_debug)
		{
			printf ("DHCP: unlinked \r\n") ;
		}
		strcpy (dhcp_status_message, "Unlinked") ;
		dhcp_machine_state = DHCP_AWAITING_LINK_STATE_6 ;
		dhcp_mark_time  = dhcp_current_time ;
		dhcp_success	= 0 ;
		return (DHCP_FAIL) ;
	}
	
	strcpy (dhcp_status_message, "Working") ;
	dhcpdisc.transaction_id [0]  = rand() & 0xff ;
	dhcpdisc.transaction_id [1]  = rand() & 0xff ;
	dhcpdisc.transaction_id [2]  = netinfo.mac [4] ;
	dhcpdisc.transaction_id [3]  = netinfo.mac [5] ;

	sprintf (temps, "%02X%02X%02X", netinfo.mac[3], netinfo.mac[4], netinfo.mac[5]) ;

	memcpy (dhcpdisc.host_name_mac, temps, sizeof(dhcpdisc.host_name_mac)) ;
	memcpy (netinfo.sv, broadcast_address, sizeof(broadcast_address)) ;

	memset (netinfo.rx, 0, sizeof(netinfo.rx)) ;

    NETUNLOCK() ;
    setSIPR (nullx) ;
    status = sendto (dhcp_socket, (void*) &dhcpdisc, sizeof(dhcpdisc), broadcast_address, DHCPOUT_PORT) ;
    setSIPR (netinfo.ip) ;
    NETLOCK() ;
 
	if (dhcp_debug)
	{
		printf ("DHCP: DISCOVER sent") ;
		printf (" - My MAC ") ;
		for (x = 0 ; x < 6 ; x++)
		{
			printf ("%02X", netinfo.mac[x]) ;
			if (x != 5)
			{
				printf (":") ;
			}
		}
		printf (" - Transaction ID ") ;
		for (x = 0 ; x < 4 ; x++)
		{
			printf ("%02X", dhcpdisc.transaction_id[x]) ;
		}
		printf ("\r\n") ;
	}

	dhcp_machine_state = DHCP_WAITING_OFFER_STATE_2 ;
	if (dhcp_debug)
	{
		printf ("DHCP: waiting for OFFER \r\n") ;
	}

	dhcp_mark_time = dhcp_current_time ;

	return (0) ;
}


int dhcp_waiting_offer_state_2 (int command)
{
	uint8	received_address [4] ;
	uint16	received_port ;
	int		status ;
	int		x ;
	uint8	*p ;
	uint8	utemp8 ;
	uint8	temps [4] ;
	
	if (dhcp_link_state == 0)
	{
		if (dhcp_debug)
		{
			printf ("DHCP: unlinked\r\n") ;
		}
		strcpy (dhcp_status_message, "Unlinked") ;
		dhcp_machine_state = DHCP_AWAITING_LINK_STATE_6 ;
		dhcp_mark_time = dhcp_current_time ;
		dhcp_success   = 0 ;
		return (DHCP_FAIL) ;
	}

	status = recvfrom (dhcp_socket, (void*)&dhcpoffer, sizeof(dhcpoffer), received_address, &received_port) ;
	if (status > 0)
	{

// check for a valid DHCP offer to us

		if 
		(
			memcmp (dhcpdisc.bootp_flags, dhcpoffer.bootp_flags, 2) == 0		&&
			memcmp (dhcpdisc.mac2, dhcpoffer.mac2, MAC_SIZE) == 0 				&&
			memcmp (dhcpdisc.transaction_id, dhcpoffer.transaction_id, 4) == 0 	&&
			memcmp (dhcpdisc.magic_cookie, dhcpoffer.magic_cookie, 4) == 0		&&
			dhcpoffer.options[0] == DHCP_MESSAGE_TYPE 							&& 
			dhcpoffer.options[2] == DHCP_MESSAGE_OFFER
		)
		{
			memcpy (netinfo.rx, received_address, 4) ;			// the address making the offer
			memset (temps, 0, sizeof(temps)) ;
						
			p = dhcpoffer.options ;
			while (*p != 255 && (int) p < (int) &dhcpoffer.end_packet)
			{
				switch (p[0])
				{
					default: break ;
				} ;
				p += p[1] + 2 ;
			}

			if (dhcp_debug)
			{
				printf ("DHCP: OFFER of ") ;
				for (x = 0 ; x < 4 ; x++)
				{
					printf ("%d", dhcpoffer.your_ip[x]) ;
					if (x != 3)
					{
						printf (".") ;
					}
				}

				printf (" received from ") ;
				for (x = 0 ; x < 4 ; x++)
				{
					printf ("%d", received_address[x]) ;
					if (x != 3)
					{
						printf (".") ;
					}
				}
				printf ("\r\n") ;
			}
			dhcp_machine_state = DHCP_SENDING_REQUEST_STATE_7 ;
			return (0) ;
		}
	}

	if (dhcp_current_time - dhcp_mark_time > 10000)
	{
		if (dhcp_debug)
		{
			printf ("DHCP: timeout - waiting to retry \r\n") ;
		}
		dhcp_mark_time = dhcp_current_time ;
		dhcp_machine_state = DHCP_WAITING_TO_RETRY_STATE_4 ;
		return (0) ;
	}
	return (0) ;
}


int dhcp_sending_request_state_7 (int command)
{
	int		status ;
	uint8	*p ;
	
	memcpy (&dhcprequest.transaction_id, dhcpdisc.transaction_id, 4) ;

	p = &dhcprequest.options[0] ;

	*p++ = DHCP_MESSAGE_REQUEST ;
	*p++ = 1 ;								// length
	*p++ = 3 ;								// request

	*p++ = DHCP_MESSAGE_CLIENTID ;
	*p++ = 7 ;								// length
	*p++ = 1 ;								// Ethernet
	memcpy (p, netinfo.mac, MAC_SIZE) ;
	p += 6 ;		

	*p++ = DHCP_MESSAGE_REQUESTEDIP ;
	*p++ = 4 ;								// length
	memcpy (p, dhcpoffer.your_ip, 4) ;		// offered ip address
	p += 4 ;		

	*p++ = DHCP_MESSAGE_HOSTNAME ;
	*p++ = sizeof (dhcprequest.host_name) ;								// length
	memcpy (p, &dhcpdisc.host_name, sizeof(dhcpdisc.host_name)) ;
	p += sizeof (dhcprequest.host_name) ;		
	
	*p++ = DHCP_MESSAGE_PARAMETERS ;
	*p++ = 8 ;								
	*p++ = 1 ;								
	*p++ = 3 ;								
	*p++ = 6 ;								
	*p++ = 15 ;								
	*p++ = 58 ;								
	*p++ = 59 ;								
	*p++ = 31 ;								
	*p++ = 33 ;								
	*p++ = 0xff ;							// end list

	status = sendto (dhcp_socket, (void*) &dhcprequest, sizeof(dhcprequest), netinfo.sv, DHCPOUT_PORT) ;
	
	if (dhcp_debug)
	{
		printf ("DHCP: REQUEST sent \r\n") ;
		printf ("DHCP: waiting for ACK \r\n") ;
	}

	dhcp_machine_state = DHCP_WAITING_ACK_STATE_3 ;
	dhcp_mark_time = dhcp_current_time ;
	return (0) ;
}


int dhcp_waiting_ack_state_3 (int command)
{
	uint8	received_address [4] ;
	uint16	received_port ;
	int		status ;
	char	message [64] ;
	char	temps [16] ;
	int		x ;
	uint8	*p ;

	if (dhcp_link_state == 0)
	{
		if (dhcp_debug)
		{
			printf ("DHCP: unlinked\r\n") ;
		}
		strcpy (dhcp_status_message, "Unlinked") ;
		dhcp_machine_state = DHCP_AWAITING_LINK_STATE_6 ;
		dhcp_mark_time = dhcp_current_time ;
		dhcp_success   = 0 ;
		return (DHCP_FAIL) ;
	}

	status = recvfrom (dhcp_socket, (void*)&dhcpack, sizeof(dhcpack), received_address, &received_port) ;
	if (status > 0)
	{
		if 
		(
			memcmp (dhcpack.bootp_flags, dhcpdisc.bootp_flags, 2) == 0		&&
			memcmp (dhcpack.mac2, dhcpdisc.mac2, MAC_SIZE) == 0				&&
			memcmp (dhcpack.magic_cookie, dhcpdisc.magic_cookie, 4) == 0	&&
			memcmp (received_address, netinfo.rx, 4) == 0					&&
			dhcpack.options[0] == DHCP_MESSAGE_TYPE 					 					
		)
		{
			if (dhcpack.options[2] == DHCP_MESSAGE_ACK)
			{
				memset (message, 0, sizeof(message)) ;
				memset (temps, 0, sizeof(temps)) ;
				p = dhcpack.options ;
				while (*p != 255 && (int) p < (int) &dhcpack.end_packet)
				{			
					switch (p[0])
					{
						case DHCP_MESSAGE_GATEWAY: 		memcpy (&netinfo.gw, p+2, 4) 		; break ;
						case DHCP_MESSAGE_DNS: 			memcpy (&netinfo.dns, p+2, 4) 		; break ;
						case DHCP_MESSAGE_SUBNET: 		memcpy (&netinfo.sn, p+2, 4) 		; break ;
						case DHCP_MESSAGE: 				strncpy (message, (p+2), p[1]) 		; break ;
						case DHCP_MESSAGE_SERVER: 		memcpy (&netinfo.sv, p+2, 4) ; 		; break ;
						case DHCP_MESSAGE_LEASETIME:	memcpy (temps,p+2, 4) ; break ;
						default: break ;
					} ;
					p += p[1] + 2 ;
				}	
				
				memcpy (&netinfo.sv, received_address, 4) ;		// the address making the offer
				memcpy (&netinfo.ip, dhcpack.your_ip, 4) ;		// the address offered

				netinfo_to_wiz () ;
			    network_initialize (netinfo_wiz) ; 
				
				leased_at 	   = monotime_ms() / 1000 ;
				lease_period   = (temps [0] << 24) | 
							 	 (temps [1] << 16) | 
							 	 (temps [2] <<  8) |
							 	 (temps [3] <<  0) ;					 	 

				if (dhcp_debug)
				{
					printf ("DHCP: ACK received \r\n") ;
					printf ("DHCP: +++ SUCCESS +++ \r\n") ;
					xprintf ("\r\n\r\n\r\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n") ;
				}
				dhcp_debug = 0 ;
				dhcp_success = 1 ; 
				strcpy (dhcp_status_message, "Success") ;
				dhcp_machine_state 	= DHCP_OK_STATE_5 ;
				return (DHCP_SUCCESS) ;				
			}
			else if (dhcpack.options[2] == DHCP_MESSAGE_NAK)
			{
				memset (message, 0, sizeof(message)) ;
				p = dhcpack.options ;
				while (*p != 255 && (int) p < (int) &dhcpack.end_packet)
				{			
					switch (p[0])
					{
						case DHCP_MESSAGE: strncpy (message, (p+2), p[1]) ; break ;
						default: break ;
					} ;
					p += p[1] + 2 ;
				}	
				if (dhcp_debug)
				{
					printf ("DHCP: NAK received (%s) \r\n", message) ;
					printf ("DHCP: --- FAILURE --- \r\n") ;
					printf ("DHCP: waiting to retry \r\n") ;
				}
				strcpy (dhcp_status_message, "Failure") ;
				dhcp_mark_time = dhcp_current_time ;
				dhcp_machine_state = DHCP_WAITING_TO_RETRY_STATE_4 ;
				return (DHCP_FAIL) ;
			}	
			else 
			{
				if (dhcp_debug)
				{
					printf ("DHCP: unknown response \r\n") ;
					printf ("DHCP: --- FAILURE --- \r\n") ;
					printf ("DHCP: waiting to retry \r\n") ;
				}
				strcpy (dhcp_status_message, "FAILURE") ;
				dhcp_mark_time = dhcp_current_time ;
				dhcp_machine_state = DHCP_WAITING_TO_RETRY_STATE_4 ;
				return (DHCP_FAIL) ;
			}
		}	
	}

	if (dhcp_current_time - dhcp_mark_time > 10000)
	{
		if (dhcp_debug)
		{
			printf ("DHCP: timeout - waiting to retry \r\n") ;
		}
		dhcp_mark_time = dhcp_current_time ;
		dhcp_machine_state = DHCP_WAITING_TO_RETRY_STATE_4 ;
		return (DHCP_FAIL) ;
	}

	return (0) ;
}


int dhcp_waiting_to_retry_state_4 (int command)
{
	if (dhcp_link_state == 0)
	{
		if (dhcp_debug)
		{
			printf ("DHCP: unlinked\r\n") ;
		}
		strcpy (dhcp_status_message, "Unlinked") ;
		dhcp_machine_state = DHCP_AWAITING_LINK_STATE_6 ;
		dhcp_mark_time = dhcp_current_time ;
		dhcp_success   = 0 ;
		return (DHCP_FAIL) ;
	}

	if (dhcp_current_time - dhcp_mark_time >= 3000)
	{
		dhcp_machine_state = DHCP_SENDING_DISCOVER_STATE_1 ;
	}
	
	return (0) ;
}


int dhcp_ok_state_5 (int command)
{
	int32	temp32 ;
	
	if (dhcp_link_state == 0)
	{
		dhcp_debug = 1 ;
		gpio_put (ZLED, 0) ;
		set_activity_led (1, OFF) ;
		set_activity_led (2, OFF) ;
		if (dhcp_debug)
		{
			printf ("DHCP: unlinked\r\n") ;
		}
		strcpy (dhcp_status_message, "Unlinked") ;
		dhcp_machine_state = DHCP_AWAITING_LINK_STATE_6 ;
		dhcp_mark_time = dhcp_current_time ;
		dhcp_success   = 0 ;
		return (DHCP_FAIL) ;
	}
	else 
	{
		temp32 = leased_at + lease_period - (monotime_ms() / 1000) ;	// lease time remaining in seconds
		if ((temp32 <= lease_period / 8) && lease_period != 0 && lease_period != 0xffffffff)
		{
			strcpy (dhcp_status_message, "Renewing") ;
			dhcp_debug = 1 ;
			gpio_put (ZLED, 0) ;
			set_activity_led (1, OFF) ;
			set_activity_led (2, OFF) ;
			if (dhcp_debug)
			{
				printf ("DHCP: Requesting new DCHP address\r\n") ;
			}
			dhcp_machine_state = DHCP_SENDING_DISCOVER_STATE_1 ;		// requesting new address
		}
		else if ((temp32 <= lease_period / 2) && lease_period != 0 && lease_period != 0xffffffff)
		{
			strcpy (dhcp_status_message, "Renewing") ;
///			dhcp_debug = 1 ;
///			gpio_put (ZLED, 0) ;
///			set_activity_led (1, OFF) ;
///			set_activity_led (2, OFF) ;
			if (dhcp_debug)
			{
				printf ("DHCP: Renewing DCHP lease\r\n") ;
			}
			dhcp_machine_state = DHCP_SENDING_REQUEST_STATE_7 ;		// renew DHCP lease		
		}
	}
		
	return (0) ;
}


int dhcp_awaiting_link_state_6 (int command)
{
	if (dhcp_link_state)
	{
		if (dhcp_debug)
		{
			printf ("DHCP: linked - waiting to retry\r\n") ;
		}
		strcpy (dhcp_status_message, "Linked") ;
		dhcp_machine_state = DHCP_WAITING_TO_RETRY_STATE_4 ;
		dhcp_mark_time = dhcp_current_time ;
		dhcp_success   = 0 ;
		return (0) ;
	}
	else if (dhcp_current_time - dhcp_last_time >= 1000)
	{
		if (dhcp_debug)
		{
			printf ("DHCP: waiting for Ethernet link - %d \r\n", (dhcp_current_time - dhcp_mark_time) / 1000) ;
		}
		dhcp_last_time = dhcp_current_time ;
	} 
	
	return (0) ;
}


int __in_flash("my_group_name") ethernet_setup()
{
	int			x, y ;
	char		temps [256] ;
	int			status ;
	uint8		received_address [4] = {255, 255, 255, 255} ;;
	uint16		received_port = 0 ;
	uint8		*p ;
	int			temp ;
	int			counter ;

	memset (unique_board_id, 0, sizeof(unique_board_id)) ;
	pico_get_unique_board_id ((void*)&unique_board_id) ;			// Flash ID

	netinfo.mac [3] = unique_board_id [5] ;
	netinfo.mac [4] = unique_board_id [6] ;
	netinfo.mac [5] = unique_board_id [7] ;

	for (x = 0 ; x < 6 ; x++)
	{
		dhcpdisc.mac2 			[x] = netinfo.mac [x] ;
		dhcpdisc.client_id_mac 	[x] = netinfo.mac [x] ;
		netinfo.mac 			[x] = netinfo.mac [x] ;
		netinfo.mac 			[x] = netinfo.mac [x] ;
		dhcprequest.mac2		[x] = netinfo.mac [x] ;
	}

	lease_period = 0 ;
	leased_at 	 = 0 ;

	gpio_put (WIZRESET, 1) ; 											// enable WIZ chip
	sleep_ms (100) ;
    wizchip_spi_initialize() ;
    wizchip_cris_initialize() ;
    wizchip_reset() ;
    wizchip_initialize_ewj() ;

	netinfo_to_wiz () ;
    network_initialize (netinfo_wiz) ; 

    temp  = (baseipport / 100) ;
    temp *= 100 ;

    socket0   = socket (0,  Sn_MR_UDP, temp       + PORTLISTENBASE,     SF_IO_NONBLOCK) ;   // broadcast and c$
    socket1   = socket (1,  Sn_MR_UDP, baseipport + PORTLISTENBASE + 1, SF_IO_NONBLOCK) ;   // listen for RX1 $
    socket2   = socket (2,  Sn_MR_UDP, baseipport + PORTLISTENBASE + 2, SF_IO_NONBLOCK) ;   // listen for RX2 $
    socket3   = socket (3,  Sn_MR_UDP, DHCPIN_PORT,                     SF_IO_NONBLOCK) ;   // DHCP

	broadcast_socket	= socket0 ;
	rcv[1].socket 		= socket1 ;	
	rcv[2].socket 		= socket2 ;	
	dhcp_socket   		= socket3 ;

	counter = 0 ;

	do
	{
		if ((x & 7) == 0)
		{
			gpio_put (ZLED, 1) ;
			set_activity_led (1, ON) ;
			set_activity_led (2, ON) ;
			printf ("Waiting for Ethernet link - %d \r\n", counter++) ;
		}
		else
		{
			gpio_put (ZLED, 0) ;
			set_activity_led (1, OFF) ;
			set_activity_led (2, OFF) ;
		}
		sleep_ms (125) ;
		x++ ;
	} while (get_link_state() == 0) ;
	gpio_put (ZLED, 0) ;
	set_activity_led (1, OFF) ;
	set_activity_led (2, OFF) ;
	printf ("\r\n") ;
	sleep_ms (200) ;
	 
	dhcp_scheduler (0) ;						// reset dhcp
	if (netinfo.dhcp == NETINFO_DHCP)
	{
		dhcp_scheduler (3) ;					// debug on
		dhcp_scheduler (1) ;					// start dhcp
		do 
		{
			status = dhcp_scheduler (4) ;
		} while (status != DHCP_SUCCESS) ;
		dhcp_scheduler (2) ;					// debug off
	}
	else
	{
		strcpy (dhcp_status_message, "Static") ;
		printf ("Using static IP address: ") ;
		strcpy (zbuff, "") ;
		for (x = 0 ; x < 4 ; x++)
		{
			sprintf (zbuff+strlen(zbuff), "%d", netinfo.ip[x]) ;
			if (x != 3)
			{
				sprintf (zbuff+strlen(zbuff), ".") ;
			}
		}
		sprintf (zbuff+strlen(zbuff), "\r\n") ;
		printf (zbuff) ;
	}
   
	ethernet_ready = 1 ;
}


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@
//@	 second core 
//@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

volatile	uint32			thenms ;

void core1_main()
{
	int				x ;
	int				y ;
	uint8*			pointer ;
	packetblock_t*	packetptr ;
	int				packetcount = 0 ;
	int32			rx ;
	char*			buffptr ;
	uint16			buffsize ;	
	uint8			scanstate ;
	char*			pos ;
	uint8			tempc ;
	uint32			temp ;
	uint8			err ;
	char			temps [32] ;

	int32			status ;
	int32			goodx ;
	int32			headerx ;
	int32			rxx ;
	int32           freqx ;
	int32           locx  ;
    int32           srx   ;
    int32		    antx  ;
	int32		    voltx ;
	int32		    volty ;
	int32			reqprogx ;
	int32			bip ;
	int32			tsflash ;
	int32			reboot ;
	int32			bootsel ;
	uint32			vgxtone_ ;				// voltage generator X tone
	uint32			vgxen_ ;				// voltage generator X enable				
	uint32			vgxsel_ ;				// voltage generator X low / high select	
	char			vgxstring_ [8] ;
	uint32			vgytone_ ;				// voltage generator Y tone
	uint32			vgyen_ ;				// voltage generator Y enable				
	uint32			vgysel_ ;				// voltage generator Y low / high select	
	char			vgystring_ [8] ;

	thenms = monotime_ms() ;

	while (1)
	{	
		info_loop() ;							
		
		for (rx = 1 ; rx <= MAXRECEIVERS ; rx++)
		{			
			GLOBALNIM = rcv[rx].nim ;
			stv0910_read_scan_state (rcv[rx].nimreceiver, &scanstate) ;  // get the scan state
			if (scanstate == 2 || scanstate == 3)
			{
				if (rcv[rx].piorunning == 2)
				{
					rcv[rx].piorunning = 3 ;
				}
			}
			else
			{
				if (rcv[rx].piorunning == 3)
				{
					rcv[rx].piorunning = 1 ;
				}
			}
		}
		
// check for commands from core 0		
		
		if (tur01e.status == 1)											// ETH command receiver by core 0 
		{			
			turp = &tur01e ;	
		}
		else
		{
			turp = 0 ;													// pointer to command			
		}
		
		if (turp)														// command received by core 0
		{      
			goodx   		=  0  ;
			headerx 		= -1 ;
			rxx   			= -1 ;
            freqx 			= -1 ;
            locx  			= -1 ;
            srx   			= -1 ;
            antx  			= -1 ;
            voltx 			= -1 ;
            volty 			= -1 ;
            reqprogx		= 0 ;
            bip 			= -1 ;
			tsflash			= -1 ;
			reboot			= -1 ;
			bootsel			= -1 ;			


			commandrxbuff2 = (void*) &(turp->buffer) ;

			sprintf 
			(
				(char*)lastincomingcommand, "Last command (%d): %s", 
				turp->receivedonport, commandrxbuff2
			) ;

			y = 0 ;
			for (x = 0 ; x < strlen(commandrxbuff2) ; x++)						// convert to upper case
			{
				if (commandrxbuff2[x] >= ' ')
				{
					commandrxbuff[y]  = toupper (commandrxbuff2[x]) ;			// upper case with non-printing removed
					commandrxbuff3[y] = commandrxbuff2[x] ;						// non printing removed, for reply
					y++ ;
				}
			}
			commandrxbuff  [y] 	= 0 ;
			commandrxbuff3 [y] 	= 0 ;

			rx = turp->receiver ;

			if  (strncmp (commandrxbuff, "[GLOBALMSG]", 11) == 0)      		// QuickTune command
			{	
				headerx = QTHEADER ;
				rxx = rx ;
			}

			pos = strstr (commandrxbuff, "[TO@WH]") ;
			if (pos)
			{
			  headerx = WHHEADER ;
			  if (rx == 0)
			  {
				pos = strstr (commandrxbuff, "TSFLASH=") ;
				if (pos)
				{
					pos += strlen ("TSFLASH=") ;
					tsflash = atoi (pos) ;
					if (tsflash)
					{
						tsflash = 1 ;
					}
					tsflash_value = tsflash ;
					MY_RAM_0 [RAM_TSFLASH + 0] = tsflash ;
					MY_RAM_0 [RAM_TSFLASH + 1] = 123456789 ;
				}

				pos = strstr (commandrxbuff, "BIP=") ;
				if (pos)
				{
					pos += strlen ("BIP=") ;
					bip = atoi (pos) ;
					if (bip != baseipport)
					{
						if ((bip < 1100 && bip != 0) || (bip > 65400) || ((bip % 100) > 14) || (bip & 1))
						{
							bip_error = 1 ;
							bip_value = bip ;
						}
						else
						{
							bip_error = 0 ;
							bip_value = bip ;
							MY_RAM_0 [RAM_BIP + 0] = bip ;
							if (bip == 0)
							{
								MY_RAM_0 [RAM_BIP + 1] = 0 ;
							}
							else
							{
								MY_RAM_0 [RAM_BIP + 1] = 123456789 ;
							}
							reboot_request = 4 ;
						}
					}
				}

				pos = strstr (commandrxbuff, "RESET=") ;
				if (pos)
				{
					pos += strlen ("RESET=") ;
					reboot = atoi (pos) ;
					if (reboot == 147)
					{
						reboot_request = 1 ;
						MY_RAM_0 [RAM_BIP + 1] 		= 0 ;
						MY_RAM_0 [RAM_TSFLASH + 1] 	= 0 ;
					}			
				}

				pos = strstr (commandrxbuff, "REBOOT=") ;
				if (pos)
				{
					pos += strlen ("REBOOT=") ;
					reboot = atoi (pos) ;
					if (reboot == 258)
					{
						reboot_request = 2 ;
					}			
				}

				pos = strstr (commandrxbuff, "BOOTSEL=") ;
				if (pos)
				{
					pos += strlen ("BOOTSEL=") ;
					bootsel = atoi (pos) ;
					if (bootsel == 369)
					{
						reboot_request = 3 ;
					}							
				}
			  }				

				pos = strstr (commandrxbuff, "RCV=") ;
				if (pos)
				{
					pos += strlen ("RCV=") ;
					rxx = atoi (pos) ;
					if (rx < 0)
					{
						rxx += rxbase ;
					}
					if (rx > 0 && rxx - rxbase != rx)
					{
						headerx = 0 ;
					}
					rx = rxx - rxbase ;
					if (rx <= 0 || rx > MAXRECEIVERS)
					{
						headerx = 0 ;
					}
				}
             	else
 	          	{
					headerx = 0 ;
               	}

///				if (turp->receiver == 0)
				{
///					printf ("%s\r\n",commandrxbuff2) ;
				}
			}
			
// parse the command
			
			pos = strstr (commandrxbuff,"FREQ=") ;
			if (pos)
			{
				pos += strlen ("FREQ=") ;
				freqx = atoi (pos) ;
			}
			pos = strstr (commandrxbuff,"OFFSET=") ;
			if (pos)
			{
				pos += strlen ("OFFSET=") ;
				locx = atoi (pos) ;
			}
			pos = strstr (commandrxbuff,"SRATE=") ;
			if (pos)
			{
				pos += strlen ("SRATE=") ;
				srx = atoi (pos) ;
			}
			pos = strstr (commandrxbuff,"FPLUG=") ;
			if (pos)
			{
				pos += strlen ("FPLUG=") ;
				antx = *pos ;
				if (antx == 'A')
				{
					antx = 1 ;
				}
				else if (antx == 'B')
				{
					antx = 2 ;
				}
				else
				{
					antx = -1 ;
				}
			}
			pos = strstr (commandrxbuff,"PRG=") ;
			if (pos)
			{
				pos += strlen ("PRG=") ;
				reqprogx = atoi (pos) ;
			}
			pos = strstr (commandrxbuff,"VGX=") ;
			if (pos && vgxpresent)
			{	
				pos += strlen ("VGX=") ;
				sscanf (pos, "%4[^, ]", vgxstring_) ;

				if (strcmp(vgxstring_, "OFF") == 0)
				{
					vgxen_ 		= 0 ;									
					vgxsel_  	= 0 ;
					vgxtone_ 	= 0 ;
					voltx   	= 1 ;
				}
				else if (strcmp(vgxstring_, "LO") == 0)
				{
					vgxen_   	= 1 ;									
					vgxsel_  	= 0 ;
					vgxtone_ 	= 0 ;
					voltx   	= 1 ;
				}
				else if (strcmp(vgxstring_, "HI") == 0)
				{
					vgxen_   	= 1 ;									
					vgxsel_  	= 1 ;
					vgxtone_ 	= 0 ;
					voltx   	= 1 ;
				}
				else if (strcmp(vgxstring_, "LOT") == 0)
				{
					vgxen_ 	  	= 1 ;									
					vgxsel_  	= 0 ;
					vgxtone_	= 1 ;
					voltx  	 	= 1 ;
				}
				else if (strcmp(vgxstring_, "HIT") == 0)
				{
					vgxen_   	= 1 ;									
					vgxsel_  	= 1 ;
					vgxtone_ 	= 1 ;
					voltx   	= 1 ;
				}
				else
				{
					voltx   	= 0 ;
				}					
			}
			pos = strstr (commandrxbuff,"VGY=") ;
			if (pos && vgypresent)
			{	
				pos += strlen ("VGY=") ;
				sscanf (pos, "%4[^, ]", vgystring_) ;

				if (strcmp(vgystring_, "OFF") == 0)
				{
					vgyen_	 	= 0 ;									
					vgysel_ 	= 0 ;
					vgytone_ 	= 0 ;
					volty   	= 1 ;
				}
				else if (strcmp(vgystring_, "LO") == 0)
				{
					vgyen_   	= 1 ;									
					vgysel_  	= 0 ;
					vgytone_ 	= 0 ;
					volty   	= 1 ;
				}
				else if (strcmp(vgystring_, "HI") == 0)
				{
					vgyen_   	= 1 ;									
					vgysel_  	= 1 ;
					vgytone_ 	= 0 ;
					volty   	= 1 ;
				}
				else if (strcmp(vgystring_, "LOT") == 0)
				{
					vgyen_   	= 1 ;									
					vgysel_  	= 0 ;
					vgytone_ 	= 1 ;
					volty   	= 1 ;
				}
				else if (strcmp(vgystring_, "HIT") == 0)
				{
					vgyen_   	= 1 ;									
					vgysel_  	= 1 ;
					vgytone_ 	= 1 ;
					volty   	= 1 ;
				}
				else
				{
					volty   = 0 ;
				}					
			}
			pos = strstr (commandrxbuff,"VOLTAGE=") ;
			if (pos)
			{
				pos += strlen ("VOLTAGE=") ;
				voltx = atoi (pos) ;
				if (voltx == 0)
				{
					strcpy (vgxstring_, "OFF") ;
					vgxen_  = 0 ;									
					vgxsel_ = 0 ;
					voltx   = 1 ;
				}
				else if (voltx == 13)
				{
					strcpy (vgxstring_, "LO") ;
					vgxen_  = 1 ;									
					vgxsel_ = 0 ;
					voltx   = 1 ;
				}
				else if (voltx == 18)
				{
					strcpy (vgxstring_, "HI") ;
					vgxen_  = 1 ;									
					vgxsel_ = 1 ;
					voltx   = 1 ;
				}
				else 
				{
					voltx = 0 ;									// error
				}                        
			}

			pos = strstr (commandrxbuff,"22KHZ=") ;
			if (pos)
			{
				pos += strlen ("22KHZ=") ;
				if (strncmp(pos,"OFF",3) == 0)
				{
					vgxtone_ = 0 ;
				}
				else if (strncmp(pos,"ON",2) == 0)
				{
					if (vgxen_ == 1)
					{
						vgxtone_ = 1 ;
						strcat (vgxstring_, "T") ;
					}
				}
				else
				{
					voltx = 0 ;
				}
			}	

// process the parsed command

			if (rx > 0 && headerx > 0 && (freqx == 0 || srx == 0))
			{
               	GLOBALNIM = rcv[rx].nim ;
                stv6120_init (rcv[rx].nimreceiver, 0, rcv[rx].antenna, 0) ;			// disable receiver
                headerx = 0 ;
       		}
			else if ((rxx == -1 && freqx == -1 && srx == -1 && locx == -1 && antx == -1) && (headerx >= 0) && (voltx > 0 || volty > 0))
			{
				if (vgxpresent && voltx > 0)
				{
					strcpy (rcv[0].textinfos[STATUS_VGX_STATE], vgxstring_) ;
					vgxen  	= vgxen_ ;
					vgxsel 	= vgxsel_ ;
					vgxtone = vgxtone_ ;
				}
				if (vgypresent && volty > 0)
				{
					strcpy (rcv[0].textinfos[STATUS_VGY_STATE], vgystring_) ;
					vgyen  	= vgyen_ ;
					vgysel 	= vgysel_ ;
					vgytone = vgytone_ ;
				}
// set LNB psu boards
				gpio_put (VGXSEL, vgxsel) ;				// RT5047 LNB voltage module
				gpio_put (VGXEN,  vgxen) ;
				gpio_put (VGYSEL, vgysel) ;				// RT5047 LNB voltage module
				gpio_put (VGYEN,  vgyen) ;
				GLOBALNIM = NIM_A ;
				if (vgxtone == 0)
				{
					tempc = KHZ22OFF ;					// defined in stv0910.h	
				}
				else
				{
					tempc = KHZ22ON ;										
				}
				err = stv0910_write_reg (RSTV0910_P2_DISTXCFG, tempc) ;		
				GLOBALNIM = NIM_A ;
				if (vgytone == 0)
				{
					tempc = KHZ22OFF ;					// defined in stv0910.h	
				}
				else
				{
					tempc = KHZ22ON ;										
				}
				err = stv0910_write_reg (RSTV0910_P1_DISTXCFG, tempc) ;		
			}        		           
            else if (rx > 0 && headerx > 0 && freqx >= 0 && locx >= 0 && srx >= 0 && antx >= 0 && voltx != 0 && volty != 0)		// voltx=0 = error
			{			
				if (((abs(freqx - locx) >= MINFREQ) && (abs(freqx - locx) <= MAXFREQ)) || freqx == 0)
				{
					if ((srx >= MINSR) && (srx <= MAXSR) && rcv[rx].receiver)
					{
                     	goodx = 1 ;						// good command
						rcv[rx].active  = 1 ;
						
						memset ((void*)&rcv[rx].rawinfos,0,sizeof(rcv[rx].rawinfos)) ;
						memset ((void*)&rcv[rx].textinfos,0,sizeof(rcv[rx].textinfos)) ;

						sprintf (rcv[rx].textinfos[STATUS_FREQ_REQUESTED],   "%.3f", (float) freqx / 1000) ;
						sprintf (rcv[rx].textinfos[STATUS_OFFSET_REQUESTED], "%.3f", (float) locx  / 1000) ;

						if (vgxpresent && voltx > 0)
						{
							strcpy (rcv[0].textinfos[STATUS_VGX_STATE], vgxstring_) ;
							vgxen  	= vgxen_ ;
							vgxsel 	= vgxsel_ ;
							vgxtone = vgxtone_ ;
						}

						if (vgypresent && volty > 0)
						{
							strcpy (rcv[0].textinfos[STATUS_VGY_STATE], vgystring_) ;
							vgyen  	= vgyen_ ;
							vgysel 	= vgysel_ ;
							vgytone = vgytone_ ;
						}

						rcv[rx].ethusbpath 		= 1 ;						// use ETH for TS output
						rcv[rx].fromaddress32 	= turp->address32 ;				
						rcv[rx].fromport 		= turp->port ;
						if (freqx < locx)
						{
							rcv[rx].highsideloc = 1 ;										// LO is on the high side
						}
						else
						{
							rcv[rx].highsideloc = 0 ;										// LO is on the low side
						}
						if (rcv[rx].vlcstopped == 0)
						{
							rcv[rx].vlcstopped = 1 ;
							rcv[rx].vlcstopcount++ ;
							rcv[rx].rawinfos[STATUS_VLCSTOPS] = rcv[rx].vlcstopcount ;
						}
						if ((rx & 1) && (abs(freqx - qo100beaconfreq) <= 200))	
						{
							rcv[rx].qo100mode     = QO100BEACON ;
							freqx 			      = qo100beaconfreq ;
							rcv[rx].qo100locerror = 0 ;
						}
						else if ((freqx >= 10490000) && (freqx < 10500000))			// QO-100
						{
							rcv[rx].qo100mode 	= QO100BAND ;
						}
						else
						{
							rcv[rx].qo100mode   = QO100NO ;
						}
							rcv[rx].requestedfreq   = freqx ;                            
						rcv[rx].requestedloc 	= locx ;
						if (freqx != 0)
						{                            
							rcv[rx].hardwarefreq	= abs (rcv[rx].requestedfreq - rcv[rx].requestedloc) ;
							if (rcv[rx].qo100mode == QO100BAND)
							{
								rcv[rx].hardwarefreq -= rcv[rx].qo100locerror ;
							}
						}
						else
						{
							rcv[rx].hardwarefreq = 0 ;
						}
						rcv[rx].frequencies[0]  = freqx ;
						rcv[rx].freqindex       = 0 ;
						rcv[rx].enablefreqscan  = 0 ;
						rcv[rx].symbolrates[0]  = srx ;
						rcv[rx].srindex         = 0 ;
						rcv[rx].enablesrscan    = 0 ;
						rcv[rx].antenna         = antx ;
						rcv[rx].pmtpid			= 0 ;
						rcv[rx].scanstate       = 0 ;
						rcv[rx].requestedprog	= reqprogx ;
   	            		rcv[rx].forbidden 	    = 0 ;

						{
							rcv[rx].modechanges++ ;									// count a mode change
							y = STATUS_ANTENNA ;
							rcv[rx].rawinfos[y] = antx ;
		
							rcv[rx].signalacquiredtime 		= 0 ;
							rcv[rx].signallosttime 			= 0 ;
							rcv[rx].packetcountrx  	   		= 0 ;					// clear packet count				
							rcv[rx].nullpacketcountrx  		= 0 ;					// clear null packet count				
							rcv[rx].errors_sync 			= 0 ;
							rcv[rx].errors_insequence		= 0 ;
							rcv[rx].lastmodulation			= 0 ;
							temp = ((rx - 1) & 2) + 1 ;								// force 1 or 3 
							rcv[temp].errors_outsequence	= 0 ;
							rcv[temp].errors_restart		= 0 ;
							GLOBALNIM = rcv[rx].nim ;
							stv6120_init 							// configure receiver
							(
								rcv[rx].nimreceiver, rcv[rx].hardwarefreq, 
								rcv[rx].antenna,	 rcv[rx].symbolrates[0]				
							) ;								
							if (rcv[rx].qo100locerror && rcv[rx].qo100mode != QO100NO)
							{
								temp = 1 ;							// in the QO100 band 
							}										// and rx has been calibrated
							else
							{
								temp = 0 ;
							}							
							stv0910_setup_receive 					// configure demodulator
							(
								rcv[rx].nimreceiver, 
								rcv[rx].symbolrates[0], 
								temp								// restrict frequency scan when calibrated
							) ; 
							stv0910_start_scan (rcv[rx].nimreceiver) ;       						// look for a signal
							rcv[rx].piorunning = 1 ;				// core 0 will restart PIO and DMA
// set LNB psu boards
							gpio_put (VGXSEL, vgxsel) ;				// RT5047 LNB voltage module
							gpio_put (VGXEN,  vgxen) ;
							gpio_put (VGYSEL, vgysel) ;				// RT5047 LNB voltage module
							gpio_put (VGYEN,  vgyen) ;
							GLOBALNIM = NIM_A ;
							if (vgxtone == 0)
							{
								tempc = KHZ22OFF ;					// defined in stv0910.h	
							}
							else
							{
								tempc = KHZ22ON ;										
							}
								err = stv0910_write_reg (RSTV0910_P2_DISTXCFG, tempc) ;		
								GLOBALNIM = NIM_A ;
							if (vgytone == 0)
							{
								tempc = KHZ22OFF ;					// defined in stv0910.h	
							}
							else
							{
								tempc = KHZ22ON ;										
							}
							err = stv0910_write_reg (RSTV0910_P1_DISTXCFG, tempc) ;		

// check receiver state
							y = STATUS_STATE ;
							rcv[rx].scanstate  	= STATE_SEARCH ;
							rcv[rx].rawinfos[y] = STATE_SEARCH ;
							tsprocenabled 		= 1 ; 					// enable the packet processing
							lminfoutenabled 	= 1 ;							
						}
					}
				}

///				if (rxx > 0)											// don't reply to scan info
				{
					if (goodx)  
					{
   	    	    		sprintf (commandreplybuff,"[from@wh:GOOD] \"%s\"\r\n",commandrxbuff3) ;
						{
							if (rcv[rx].toaddress32 != rcv[rx].fromaddress32 && rcv[rx].ipchanges == 0)
							{
								rcv[rx].ipchanges++ ;								// count an IP change
								rcv[rx].newtoaddress32 = rcv[rx].fromaddress32 ;	// update the destination address
							}
						}

						rcv[rx].commandreceivedtime  = monotime_ms() ;		// command arrival time
						rcv[rx].timedouttime = 0 ;					// reset time of timeout

// update the command time for other receivers going to the same command IP address

						for (x = 1 ; x <= MAXRECEIVERS ; x++)
						{
							if (rcv[x].active && rcv[x].commandreceivedtime)
							{
								if (rcv[x].fromaddress32 == rcv[rx].fromaddress32)
								{
									if (rcv[x].fromaddress32)
									{
										rcv[x].commandreceivedtime = rcv[rx].commandreceivedtime ;
									}
								}
							}
						}						
					}
					else
					{
	       	    		sprintf (commandreplybuff,"[from@wh:BAD]  \"%s\"\r\n",commandrxbuff3) ;
					}        		
       		   	}
			}
			turp->status = 2 ;											// signal to core 0 that 
		}																//   the command has been processed
	}
}	

int __in_flash("my_group_name") info_loop ()
{
		int				err ;
		uint32			rx ;
		uint32			x ;
		uint32			y ;
		uint32			z ;
		char			titlebar [256] ;
		char			temps    [512] ;
		char			temps2   [256] ;
		char			temps3   [256] ;
		bool			bool1 ;
		bool			bool2 ;
		uint8			tempc ;	
		int32			temp ;
		uint32			tempu ;
		char			command       [256] ;
		int32			status ;
static	uint32			counter ;


	if ((monotime_ms() - thenms) < INFOPERIOD)
	{
		return (0) ;	
	}
	thenms += INFOPERIOD ;

   	thenms = monotime_ms() ;											// current time

    {    
		counter++ ;														// for EIT injection

// status output

		strcpy (output, "") ;

		sprintf (output+strlen(output)," RX  STATUS   CALLSIGN           MER      D  FREQUENCY     SR  ") ;
		sprintf (output+strlen(output),"MODULATION   FPRO  CODECS    ANT  PACKETS  ") ;
		sprintf (output+strlen(output),"%s","\%NUL  NIMTYPE   ") ;
		sprintf (output+strlen(output),"TS DESTINATION      ") ;
		sprintf (output+strlen(output),"\r\n") ;

		sprintf (output+strlen(output)," -----------------------------------------------------------------") ;
		sprintf (output+strlen(output),"-------------------------------------------") ;
		sprintf (output+strlen(output),"-----------------") ;
		sprintf (output+strlen(output),"----------------") ;
		sprintf (output+strlen(output),"\r\n") ;

		strcpy (outputonnet,  output) ;									// for transmission on net
		strcpy (outputoffnet, output) ;									// for transmission off net

		strcpy (expandedtextinfo, "") ;

        for (rx = 1 ; rx <= MAXRECEIVERS ; rx++)			       
        {
			strcpy (output,"") ;										// reset output line
	   		
	        if ((rcv[rx].active == 0)) 
    	    {
				memset ((void*)&rcv[rx].textinfos, 0, sizeof(rcv[rx].textinfos)) ;
				rcv[rx].scanstate              = STATE_IDLE ;
				rcv[rx].rawinfos[STATUS_STATE] = STATE_IDLE ;
				sprintf (rcv[rx].textinfos[STATUS_STATE], "idle") ;	
				continue ;
          	}
          	else
          	{
				y = STATUS_MODECHANGES ;
				rcv[rx].rawinfos[y] = rcv[rx].modechanges ;						// copy mode changes into infos
				sprintf (rcv[rx].textinfos[y], "%d", rcv[rx].rawinfos[y]) ;		
          		if (rcv[rx].ipchanges)											// IP address has changed
          		{
					{
///						if (inicommandcount == 0)									// wait until ini commands processed
						{
							while (xsr10.status == 1) ;
							sprintf (xsr10.buffer, "$0,%d\r\n$99,1\r\n", rx) ;		// create IP changed info
							xsr10.length 	= strlen (xsr10.buffer) + 1 ;
							xsr10.port 		= rcv[rx].expinfoport ;
							xsr10.typex 	= EXPINFOREC ;
							xsr10.receiver	= rx ;
							xsr10.status	= 1 ;									// ready for processing

							while (xsr10.status == 1) ;
							sprintf (xsr10.buffer, "$0,%d\r\n$99,2\r\n", rx) ;		// create IP changed info
							xsr10.length 	= strlen (xsr10.buffer) + 1 ;
							xsr10.port 		= rcv[rx].expinfo2port ;
							xsr10.typex 	= EXPINFOREC ;
							xsr10.receiver	= rx ;
							xsr10.status	= 1 ;
						}
					}					
					rcv[rx].toaddress32 = rcv[rx].newtoaddress32 ;
					rcv[rx].iptype 		= getiptype (rcv[rx].newtoaddress) ;		// see if the commanding IP address is local
          			rcv[rx].ipchanges = 0 ;

					sprintf 
					(
						(void*)debuginfo, 
						"Debug: ME=%08X SN=%08X IN=%08X TY=%d",
						*((uint32*) &netinfo.ip),
						*((uint32*) &netinfo.sn),
						*((uint32*) &rcv[rx].toaddress32),
						rcv[rx].iptype
					) ;
				}							
				
				tempc = rcv[rx].scanstate ;
                GLOBALNIM = rcv[rx].nim ;                                           // set NIM_A or NIM_B
                y = STATUS_STATE ;                                                  // parameter 1
				if (rcv[rx].scanstate != STATE_TIMEOUT && rcv[rx].scanstate != STATE_IDLE)   
				{
    	            stv0910_read_scan_state (rcv[rx].nimreceiver, &tempc) ;        	// get the scan state
                	if (tempc == STATE_SEARCH)
                	{
                		if (rcv[rx].scanstate != STATE_LOST)
                		{
                			if (rcv[rx].signalacquiredtime)
                			{
		   	            		tempc = STATE_LOST ;
		   	            		rcv[rx].signallosttime = monotime_ms() ;
		   	            		rcv[rx].signalacquiredtime = 0 ;					// reset so that next 
		   	            	}														// aquisition will reset NULLs
		   	            }
		   	            else
		   	            {
		   	            	tempc = STATE_LOST ;   				   	            	
						}       	        	
					}
				}
       	        rcv[rx].scanstate 	= tempc ;
           	    rcv[rx].rawinfos[y] = tempc ;				
                switch (tempc)
                {
                    case STATE_IDLE 		: sprintf (rcv[rx].textinfos[y],"idle")    ;  break ;      
                    case STATE_TIMEOUT		: sprintf (rcv[rx].textinfos[y],"timeout") ;  break ;      
                    case STATE_LOST 		: sprintf (rcv[rx].textinfos[y],"lost")    ;  break ;      
                    case STATE_SEARCH 		: sprintf (rcv[rx].textinfos[y],"search")  ;  break ;      
                    case STATE_HEADER_S2	: sprintf (rcv[rx].textinfos[y],"header")  ;  break ;      
                    case STATE_DEMOD_S2 	: sprintf (rcv[rx].textinfos[y],"DVB-S2")  ;  break ;    
                    case STATE_DEMOD_S 		: sprintf (rcv[rx].textinfos[y],"DVB-S")   ;  break ;     
                    default 			    : sprintf (rcv[rx].textinfos[y],"%s","")   ;  break ;   
                }
        
                y = STATUS_CARRIER_FREQUENCY ;                                      // parameter 6 - frequency
                stv0910_read_car_freq (rcv[rx].nimreceiver,&temp) ;                 // get the carrier offset               
				temp /= 1000 ;														// convert to kHz											
				if (temp == 0)
				{
					temp = 1 ;														// prevent zero value
				}
				rcv[rx].demodfreq = temp ;											// offset detected by the demodulator
                if (rcv[rx].scanstate == STATE_DEMOD_S2 || rcv[rx].scanstate  == STATE_DEMOD_S) 
                {
					if (rcv[rx].qo100mode == QO100BEACON)							// update loc error
					{
						tempu 					   = ((rx - 1) & 2) + 1  ;			// 1,2,3,4 --> 1,1,3,3
						rcv[tempu].qo100locerror   = -temp ;						// first receiver
						rcv[tempu+1].qo100locerror = -temp ;						// second receiver
					}                
                	if (rcv[rx].signalacquiredtime == 0)
	                {
						if (rcv[rx].vlcstopped == 0)
						{
							memset ((void*)&rcv[rx].rawinfos,0,sizeof(rcv[rx].rawinfos)) ;
							memset ((void*)&rcv[rx].textinfos,0,sizeof(rcv[rx].textinfos)) ;			
							rcv[rx].vlcstopped = 1 ;
							rcv[rx].vlcstopcount++ ;
							rcv[rx].rawinfos[STATUS_VLCSTOPS] = rcv[rx].vlcstopcount ;
						}               
						rcv[rx].signalacquiredtime = monotime_ms() ;				// signal first acquired
						rcv[rx].signallosttime    = 0 ;								// clear the lost time
						rcv[rx].packetcountrx = 0 ;									// clear null packet count
						rcv[rx].nullpacketcountrx = 0 ;								// clear null packet count
						y = STATUS_VIDEO_TYPE ;
						rcv[rx].rawinfos[y]    	  = 0 ;								// clear the service type
						rcv[rx].textinfos[y][0]	  = 0 ;								
        	        }
        		}        

                y = STATUS_CARRIER_FREQUENCY ;                                      // parameter 6 - frequency
            	rcv[rx].rawinfos[y]      =  rcv[rx].requestedfreq  ;
				if (rcv[rx].qo100mode   != QO100BEACON)
				{
					if (rcv[rx].highsideloc == 0)		
					{
						rcv[rx].rawinfos[y] += rcv[rx].demodfreq ;					// adjust the actual frequency
					}
					else
					{
						rcv[rx].rawinfos[y] -= rcv[rx].demodfreq ;
					}
				}

                y = STATUS_PACKETCOUNT ;                                      		
                rcv[rx].rawinfos[y] = rcv[rx].packetcountrx ;
                sprintf (rcv[rx].textinfos[y], "%d" ,rcv[rx].rawinfos[y]) ;  			    

                y = STATUS_BASEIPPORT ;                                      		
                rcv[rx].rawinfos[y] = baseipport ;
                sprintf (rcv[rx].textinfos[y], "%d" ,rcv[rx].rawinfos[y]) ;  			    
        		
                y = STATUS_TSDESTINATION ;                                      		
				sprintf (rcv[rx].textinfos[y], "") ;
				for (z = 0 ; z < 4 ; z++)
				{
					sprintf (rcv[rx].textinfos[y]+strlen(rcv[rx].textinfos[y]), "%d", rcv[rx].toaddress[z]) ;
					if (z != 3)
					{
						sprintf (rcv[rx].textinfos[y]+strlen(rcv[rx].textinfos[y]), ".") ;
					}
				}
				sprintf (rcv[rx].textinfos[y]+strlen(rcv[rx].textinfos[y]), ":%d", rcv[rx].toport) ;

                y = STATUS_CARRIER_FREQUENCY ;                                      // parameter 6 - frequency
                sprintf (rcv[rx].textinfos[y], "%0.3f" ,(float)rcv[rx].rawinfos[y] / 1000) ;  	// frequency in MHz    

                y = STATUS_SYMBOL_RATE ;                                            // parameter 9
                stv0910_read_sr (rcv[rx].nimreceiver,&tempu) ;                      // get the symbol rate               
                rcv[rx].rawinfos[y] = tempu ;
                sprintf (rcv[rx].textinfos[y], "%d" , (tempu + 500) / 1000) ;    	// symbol rate in kS  

                y = STATUS_SYMBOL_RATE_FULL ;                                       // parameter 90
                rcv[rx].rawinfos[y] = tempu ;
                sprintf (rcv[rx].textinfos[y], "%d" , tempu) ;    					// symbol rate to 1s
                
                y = STATUS_MER ;                                                    // parameter 12  
                stv0910_read_mer (rcv[rx].nimreceiver,&temp) ;                      // get the MER in 0.1dB units               
                rcv[rx].rawinfos[y] = temp ;										
                if (temp > 999)
                {
					temp = 999 ;
                }
                else if (temp < -999)
                {
					temp = -999 ;
                }
                sprintf (rcv[rx].textinfos[y], "%.1f" ,((float) temp) / 10) ;		// MER  

           	    if (rcv[rx].scanstate == STATE_HEADER_S2 || rcv[rx].scanstate == STATE_DEMOD_S2) // DVB-S2 header or lock
                {                    
                    y = STATUS_MODCOD ;                                    			// parameter 18
                    stv0910_read_modcod_and_type 
                    (
                    	rcv[rx].nimreceiver, &tempu, &bool1, &bool2  				// modcod, frametype, pilots
					) ;
					if (tempu == rcv[rx].lastmodulation)							// check for consecutive similar
					{
	                    rcv[rx].rawinfos[y] = tempu ;
    	                sprintf (rcv[rx].textinfos[y], "%s" , modinfo_S2[tempu].modtext) ; 			// qpsk 3/4 etc 
	                    y = STATUS_DNUMBER ; 														// decode threshold
						temp = rcv[rx].rawinfos[STATUS_MER] - modinfo_S2[tempu].minmer ;
	                    rcv[rx].rawinfos[y] = temp ; 
    	                sprintf (rcv[rx].textinfos[y], "%.1f" , (float) temp / 10) ;

	                    y = STATUS_FRAME_TYPE ;                                         			// parameter 19
    	                rcv[rx].rawinfos[y] = (bool1 ? 1 : 0) ;
						if (rcv[rx].rawinfos[y])
						{	
							sprintf (rcv[rx].textinfos[y], "S") ;
						}
						else
						{	
							sprintf (rcv[rx].textinfos[y], "L") ;
						}
	
    	                y = STATUS_PILOTS ;                                             			// parameter 20
	                    rcv[rx].rawinfos[y] = (bool2 ? 1 : 0) ;                                    
						if (rcv[rx].rawinfos[y])
						{	
							sprintf (rcv[rx].textinfos[y], "Y") ;
						}
						else
						{	
							sprintf (rcv[rx].textinfos[y], "N") ;
						}
	
						y = STATUS_ROLLOFF ;
	    	            stv0910_read_rolloff (rcv[rx].nimreceiver,&tempu) ; 		                // get the rolloff               
						rcv[rx].rawinfos[y] = tempu ;
						switch (rcv[rx].rawinfos[y])
						{
							case 0 : sprintf (rcv[rx].textinfos[y],"35") ; break ;
							case 1 : sprintf (rcv[rx].textinfos[y],"25") ; break ;
							case 2 : sprintf (rcv[rx].textinfos[y],"20") ; break ;
							case 3 : sprintf (rcv[rx].textinfos[y],"15") ; break ;
						} ;
					}
					else
					{
						rcv[rx].lastmodulation = tempu ;
					}

                }
                else if (rcv[rx].scanstate == STATE_DEMOD_S)									// DVB-S
                {
                    y = STATUS_MODCOD ;                                             			// parameter 18
					stv0910_read_puncture_rate (rcv[rx].nimreceiver,&tempc) ;
					rcv[rx].rawinfos[y] = 100 + tempc ;
					sprintf (rcv[rx].textinfos[y], "%s", modinfo_S[tempc].modtext) ;

					if (tempc == rcv[rx].lastmodulation)							// check for consecutive similar
					{
	                    y = STATUS_DNUMBER ; 														// decode threshold
						temp = rcv[rx].rawinfos[STATUS_MER] - modinfo_S[tempc].minmer ;
                    	rcv[rx].rawinfos[y] = temp ; 
                    	sprintf (rcv[rx].textinfos[y], "%.1f" , (float) temp / 10) ;
                    }
					else
					{
						rcv[rx].lastmodulation = tempc ;
					}
                }
                else 																			// set some parameters to default
                {
/*                
	                y = STATUS_MODCOD ;                                         				// parameter 18
                    rcv[rx].rawinfos[y] = 0 ;
       	            sprintf (rcv[rx].textinfos[y],"%s","") ;                               
                
	                y = STATUS_FRAME_TYPE ;                                     				// parameter 19
                    rcv[rx].rawinfos[y] = 0 ;
       	            sprintf (rcv[rx].textinfos[y],"%s","") ;                               
                
           	        y = STATUS_PILOTS ;                                         				// parameter 20
               	    rcv[rx].rawinfos[y] = 0 ;
                   	sprintf (rcv[rx].textinfos[y],"%s","") ;                               
*/					                	
                    y = STATUS_MER ;                                             				// parameter 12
                    rcv[rx].rawinfos[y] = 0 ;
                    sprintf (rcv[rx].textinfos[y],"%s","") ;                               
					                	
                    y = STATUS_DNUMBER ;                                             	
                    rcv[rx].rawinfos[y] = 0 ;
                    sprintf (rcv[rx].textinfos[y],"%s","") ;                               
                
                    y = STATUS_SYMBOL_RATE ;                                      				// parameter 9
                    rcv[rx].rawinfos[y] = rcv[rx].symbolrates[rcv[rx].srindex] ;
                    sprintf (rcv[rx].textinfos[y],"%d",rcv[rx].rawinfos[y]) ;                              

	                y = STATUS_CARRIER_FREQUENCY ;                                  			// parameter 6 - frequency
					temp = rcv[rx].frequencies[rcv[rx].freqindex] ;
        	        rcv[rx].rawinfos[y] = temp ;
            	    sprintf (rcv[rx].textinfos[y], "%.3f" ,((float)temp) / 1000) ;   			// frequency in MHz    
                }
                
// put info into VLC header bar

				setup_titlebar (titlebar, rx) ;										// create the VLC title bar
				setup_titlebar (titlebar, rx) ;										// create the VLC title bar
				strcpy (rcv[rx].textinfos[STATUS_TITLEBAR], titlebar) ;				


// send a modified EIT packet to be displayed in the VLC title bar

				if (eitinsert && rcv[rx].iptype != IP_MYPC)															// this function is enabled
				{
					if ((counter & 1) == 0)
					{
						if ((counter & 3) == 0)
						{
							rcv[rx].eitversion++ ;
						}	
						setup_eit ((void*)&command, rx, titlebar) ;
						if 
						(
							(rcv[rx].scanstate != STATE_TIMEOUT && rcv[rx].scanstate != STATE_IDLE) || 
							rcv[rx].timeoutholdoffcount != 0
						)

///						if (inicommandcount == 0)							// wait until ini commands processed
						{
							while (xsr10.status == 1) ;						// wait for core 0 to deal with the last request
							xsr10.length 	= TSPACKETSIZE ;
							memcpy ((void*) &xsr10.buffer, (void*) &command, strlen(command) + 1) ;
							xsr10.port 		= rcv[rx].toport ;
							xsr10.typex 	= EITREC ;
							xsr10.receiver	= rx ;
							xsr10.status	= 1 ;
						}
					}
 				}
            } 

// packet counts

            if (rcv[rx].active == 0)
            {
               	strcpy (temps2, "") ;
            }
            else
            {
				if (rcv[rx].packetcountrx < 10000000)
				{
					sprintf (temps2,"%7d", rcv[rx].packetcountrx) ;
				}
				else
				{
					sprintf (temps2, 	 "%-.6fM", (float) rcv[rx].packetcountrx / 1000000) ;	
					if (strlen(temps2) > 7)
					{
						sprintf (temps2 + 6, "M") ;	
					}
				}
			}

			strncpy (temps3, rcv[rx].textinfos[STATUS_SERVICE_NAME], sizeof(temps3)-1) ;
		    temps3 [15] = 0 ;

			y = STATUS_TS_NULL_PERCENTAGE ;
			sprintf 
			(
				rcv[rx].textinfos[y], 
				"%0.1f", 
				(float) 99.9 * rcv[rx].nullpacketcountrx / (rcv[rx].packetcountrx + 1)
			) ;
			rcv[rx].rawinfos[y] = atof (rcv[rx].textinfos[y]) + 0.5 ; 			
	
			sprintf 
			(
				temps,
				"%1s%1s%2s",									
				rcv[rx].textinfos[STATUS_FRAME_TYPE], 
				rcv[rx].textinfos[STATUS_PILOTS], 
				rcv[rx].textinfos[STATUS_ROLLOFF]
			) ;

// start building the info line for this rx
		     
	       	sprintf 
       		(
       			output + strlen(output),
               	" %2d  %-7s  %-15s  %5s  %5s  %9s  %5s  %-11s  ",  
   	            rx + rxbase, 
       			rcv[rx].textinfos [STATUS_STATE], 	
				temps3,						       				// rcv[rx].textinfos[STATUS_SERVICE_NAME]
       			rcv[rx].textinfos [STATUS_MER],
  				rcv[rx].textinfos [STATUS_DNUMBER],
      			rcv[rx].textinfos [STATUS_CARRIER_FREQUENCY],	
       			rcv[rx].textinfos [STATUS_SYMBOL_RATE],
      			rcv[rx].textinfos [STATUS_MODCOD]
			) ;

			sprintf (temps3, "%s-%s", rcv[rx].textinfos[STATUS_VIDEO_TYPE],rcv[rx].textinfos [STATUS_AUDIO_TYPE]) ;
			if (strlen(temps3) == 1)
			{	
				strcpy (temps3, "") ;
			}

	       	sprintf 
       		(
       			output + strlen(output),
               	"%-4s  %-8s  %3s  %7s  %4s  %-8s  ",    						
				temps,											// FPRO
				temps3,											// video-audio	
      			rcv[rx].textinfos [STATUS_ANTENNA],
				temps2,											// packetcount
      			rcv[rx].textinfos [STATUS_TS_NULL_PERCENTAGE],
                rcv[rx].nimtype
       		) ;

			strcat (outputonnet, output) ;								// add the output line 
			sprintf 
			(
				temps,
				"%d.%d.%d.%d",
				rcv[rx].toaddress[0],
				rcv[rx].toaddress[1],
				rcv[rx].toaddress[2],
				rcv[rx].toaddress[3]
			) ;
			if (rcv[rx].ethusbpath == 1)								// ETH
			{
///				sprintf (temps+strlen(temps), "-E   ") ;
///				sprintf (temps+strlen(temps), "     ") ;
			}
			else
			{
///				sprintf (temps+strlen(temps), "     ") ;
			}
///			temps [17] = 0 ;											// max length		
			while (strlen(temps) < 20)
			{
				strcat (temps, " ") ;
			}
			strcat (outputonnet, temps) ;
	       	sprintf (outputonnet+strlen(outputonnet), "\r\n") ;			

			strcat (outputoffnet, output) ;								// add the output line 			
			sprintf 
			(
				temps,
				"%d.%d.%d.%d",
				rcv[rx].toaddress[0],
				rcv[rx].toaddress[1],
				0,
				0
			) ;
			if (rcv[rx].ethusbpath == 1)								// ETH
			{
				sprintf (temps+strlen(temps), "-E") ;
			}
			else
			{
				sprintf (temps+strlen(temps), "  ") ;
			}
			temps [17] = 0 ;											// max length		
			while (strlen(temps) < 17)
			{
				strcat (temps, " ") ;
			}
			strcat (outputoffnet, temps) ;
	       	sprintf (outputoffnet+strlen(outputoffnet), "\r\n") ;			
	       	
// build the original LM $ text info for an individual receiver
     
        	sprintf (temps, "%s", "") ;
			for (y = 1 ; y <= STATUS_LNB_POLARISATION_H ; y++)
			{
				if (rcv[rx].textinfos[y][0])
				{
					if (y == STATUS_SERVICE_NAME || y == STATUS_SERVICE_PROVIDER_NAME)
					{
						sprintf (temps+strlen(temps), "$%d,%s\r\n", y, rcv[rx].textinfos[y]) ;
					}
					else				
					{
						sprintf (temps+strlen(temps), "$%d,%d\r\n", y, rcv[rx].rawinfos[y]) ;
					}
				}
			}

// send the orginal LM $ text info for this receiver

/*
			if 
			(
				(rcv[rx].scanstate != STATE_TIMEOUT && rcv[rx].scanstate != STATE_IDLE) || 
				rcv[rx].timeoutholdoffcount != 0
			)
			{
///				if (inicommandcount == 0)									// wait until ini commands processed
				{
					while (xsr10.status == 1) ;								// wait for core 0 to deal with the last request
					memcpy ((void*) &xsr10.buffer, (void*) &temps, strlen(temps) + 1) ;
					xsr10.length 	= strlen (xsr10.buffer) + 1 ;
					xsr10.port 		= rcv[rx].lminfoport ;
					xsr10.typex		= LMINFOREC ;
					xsr10.receiver	= rx ;
					xsr10.status	= 1 ;									// signal ready
				}
			}
*/
		
		}
		
// end of the 4 receiver loop

		for (rx = 1 ; rx <= MAXRECEIVERS ; rx++)
		{
			if (rcv[rx].active == 0)
			{
				sprintf (outputonnet+strlen(outputonnet), "\r\n") ;
				sprintf (outputoffnet+strlen(outputoffnet), "\r\n") ;
			}
		}		

// send expanded LM $ text info and 4 line summary

		for (rx = 0 ; rx <= MAXRECEIVERS ; rx++)				// rx0 is used for some system functions
		{
			if 
			(
				rx == 0 ||
				(rcv[rx].scanstate != STATE_TIMEOUT && rcv[rx].scanstate != STATE_IDLE) || 
				rcv[rx].timeoutholdoffcount != 0		// send info a few times after timeout
			)
			{
				if (rcv[rx].antenna == 1)
				{
					sprintf (rcv[rx].textinfos[STATUS_ANTENNA], "TOP") ;
				}
				else if (rcv[rx].antenna == 2)
				{
					sprintf (rcv[rx].textinfos[STATUS_ANTENNA], "BOT") ;
				}
									 
				strcpy (temps, "") ;									// expanded LM info
				sprintf (temps+strlen(temps),"$0,%d\r\n", rx + rxbase) ;
				sprintf (rcv[rx].textinfos[STATUS_VLCSTOPS], "%d", rcv[rx].vlcstopcount) ;
				sprintf (rcv[rx].textinfos[STATUS_VLCNEXTS], "%d", rcv[rx].vlcnextcount) ;
				for (y = 1 ; y < MAXINFOS ; y++)
				{
					if (y == STATUS_VGX_STATE)
					{
						if (vgxpresent)
						{						
							sprintf (temps+strlen(temps), "$%d,%s\r\n", y, rcv[0].textinfos[y]) ;	// stored in rcv[0]
						}
					}
					else if (y == STATUS_VGY_STATE)
					{
						if (vgypresent)
						{						
							sprintf (temps+strlen(temps), "$%d,%s\r\n", y, rcv[0].textinfos[y]) ;	// stored in rcv[0]
						}
					}
					else if (rcv[rx].textinfos[y][0])
					{
						sprintf (temps+strlen(temps), "$%d,%s\r\n", y, rcv[rx].textinfos[y]) ;
					}
				}

// pass to core 0 for sending

				if (rx)
				{
///					if (inicommandcount == 0)									// wait until ini commands processed
					{
						while (xsr10.status == 1) ;								// wait for core 0 to deal with the last request
						memcpy ((void*) &xsr10.buffer, (void*) &temps, strlen(temps) + 1) ;
						xsr10.length 	= strlen (xsr10.buffer) + 1  ; 
						xsr10.port 		= rcv[rx].expinfoport ;
						xsr10.typex		= EXPINFOREC ;
						xsr10.receiver	= rx ;
						xsr10.status	= 1 ;
						while (xsr10.status == 1) ;								// wait for core 0 to deal with the last request
						xsr10.port 		= rcv[rx].expinfo2port ;				// duplicate on another port
						xsr10.status	= 1 ;

// duplicate to rx0 for local output					

/*						
						while (xsr10.status == 1) ;								// wait for core 0 to deal with the last request
						xsr10.receiver	= 0 ;
						xsr10.port 		= rcv[rx].expinfoport ;				// duplicate on another port
						xsr10.status	= 1 ;
						while (xsr10.status == 1) ;								// wait for core 0 to deal with the last request
						xsr10.port 		= rcv[rx].expinfo2port ;				// duplicate on another port
						xsr10.status	= 1 ;
*/
					}
				}
				
// send 4 line summary

				temp = 0 ;
				for (y = 1 ; y < rx ; y++)								// check if already sent to this address
				{
					if 
					(
						(rcv[y].scanstate != STATE_TIMEOUT && rcv[y].scanstate != STATE_IDLE) || 
						rcv[y].timeoutholdoffcount != 0
					)
					{
						if (rcv[y].toaddress32 == rcv[rx].toaddress32)
						{
							temp++ ;									// already sent to this address
						}						
					}
				}

				if ((rx == 0) || (temp == 0))							// haven't already sent to this address
				{
///					if (inicommandcount == 0)							// wait until ini commands processed
					{
						while (xsr10.status == 1) ;						// wait for core 0 to deal with the last request									

						xsr10.typex			= SUMINFOREC ;
						xsr10.receiver		= rx ;

						if (rcv[rx].iptype == IP_OFFNET)				// mask TS destinations for off net
						{
							memcpy ((void*) &xsr10.buffer, (void*) &outputoffnet, strlen(outputoffnet) + 1) ;
							xsr10.length 	= strlen (xsr10.buffer) + 1 ;
						}
						else
						{
							memcpy ((void*) &xsr10.buffer, (void*) &outputonnet, strlen(outputonnet) + 1) ;
							xsr10.length 	= strlen (xsr10.buffer) + 1 ;
						}			
								
						xsr10.port 			= rcv[rx].summaryport ;
						xsr10.status		= 1 ;
						while (xsr10.status == 1) ;	
						xsr10.port 			= rcv[rx].summary2port ;
						xsr10.status		= 1 ;
					}
				}
			}	
			
						   			
   			if (rcv[rx].timeoutholdoffcount)
   			{
	   			rcv[rx].timeoutholdoffcount-- ;		// decrement the holdoff count
   			}
		}

		strcpy (output, "") ;
        sprintf (output+strlen(output), " PICOTUNER (WH mode) Version=ptwh%s%s%s.uf2", VERSIONX, VERSIONX2, versionx3L) ;
   		sprintf (output+strlen(output), "  RunTime=%.0fs", (float)monotime_ms() / 1000) ;
		sprintf (output+strlen(output), "  BasePort=%d", baseipport) ;
		sprintf (output+strlen(output), "  MAC=") ;
		for (x = 0 ; x < MAC_SIZE ; x++)
		{
			sprintf (output+strlen(output), "%02X", netinfo.mac[x]) ;
			if (x != MAC_SIZE - 1)
			{
				sprintf (output+strlen(output), ":") ;
			}
		} 

		sprintf (output+strlen(output), "  IP=") ;
		for (x = 0 ; x < 4 ; x++)
		{
			sprintf (output+strlen(output), "%d", netinfo.ip[x]) ;
			if (x != 3)
			{
				sprintf (output+strlen(output), ".") ;
			}
		} 
		sprintf (output+strlen(output), "  DHCP=%s", dhcp_status_message) ;
		sprintf (output+strlen(output), "%c[0K", ESC) ;
		sprintf (output+strlen(output), "\r\n") ;

		sprintf (output+strlen(output), " -----------------------------------------------------------------") ;
		sprintf (output+strlen(output), "-------------------------------------------") ;
		sprintf (output+strlen(output), "-----------------") ;
		sprintf (output+strlen(output), "----------------") ;
		sprintf (output+strlen(output), "\r\n") ;

		if (dhcp_success == 0 || dhcp_debug)
		{
			return (0) ;
		}

///	if (inicommandcount == 0)
	{
		printf ("%c[10A", ESC) ;

		printf ("==================================================================") ;
		printf ("===========================================") ;
		printf ("=================") ;
		printf ("================") ;
		printf ("\r\n") ;

		printf ("%s", output) ;
		printf ("%s", outputonnet) ;

		printf ("==================================================================") ;
		printf ("===========================================") ;
		printf ("=================") ;
		printf ("================") ;
		printf ("\r\n") ;

		char *pos ;

		pos = strchr((void*)lastincomingcommand, 13) ;
		if (pos)
		{
			*pos = 0 ;
		}
		pos = strchr((void*)lastincomingcommand, 10) ;
		if (pos)
		{
			*pos = 0 ;
		}

		printf (" %s",lastincomingcommand) ;
		printf ("%c[0K\r\n", ESC) ;

		printf ("%c[0K\r\n", ESC) ;
		printf ("%c[0K\r\n", ESC) ;
///		printf ("%c[0K7\r\n", ESC) ;

///		printf ("%c[0K2\r\n%c[0K3\r\n", ESC, ESC) ;

		printf ("%c[1A", ESC) ;
	  }
	}

	return (1) ;
}



