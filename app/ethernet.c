/**
 * @ingroup     Ethernet.c is a module which run the ethernet thread
 * @{
 *
 * @file
 * @brief  implementation of the Ethernet thread and driver
 *         The ETH interrupt, located at stm32f4x7_eth_bsp.c, awakens
 *         the ethernet thread every time it sees a new packet received.
 *         This makes sure that new packets are immediately processed
 *         by the LwIP package.
 *
 * @note
 * @author Nimrod Stoler <nstoler@gmail.com> 12-oct-2015
 */

#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
// NS #include "magStripe.h"
#include "printer.h"
#include "board.h"
#include "mutex.h"
#include "xtimer.h"
#include "thread.h"
#include "sched.h"
#include "msg.h"
#include "lwip/tcp.h"
#include "lwip/ipv4/ip_addr.h"
#include "lwip/netconf.h"
//NS??? #include "lwip/ping.h"
#include "ringbuffer.h"
#include "periph/cpuid.h"

//! FAT-fs files
#include "fatfs_sd_sdio.h"
#include "ff.h"

#include "stm32f4x7_eth.h"          // for debug purposes
#include "ethernet.h"

#define DEBUG_ETH_TIMING
#ifdef DEBUG_ETH_TIMING
#include "cyclog.h"
#endif


#define ETH_USE_MSG

#ifdef DEBUG_TIMING
    extern uint32_t global_dt2, global_dt3, global_dt4, global_dt5, global_dt6, global_t1;
#endif

extern uint8_t haveRxFrame;
static mutex_t ethrx_mutex = MUTEX_INIT;


/**
 * module's static functions
 */
void ethernet_wakeup( void *arg);
err_t ethernet_http_client_connect( char* conn_name,
                                    uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4,
                                    uint16_t port, void* user_parameters);
void ethernet_client_connection_close( struct tcp_pcb *pcb, uint8_t success);
void ethernet_client_error(void *arg, err_t err);
err_t ethernet_client_connected( void *arg, struct tcp_pcb *tpcb, err_t err);
err_t ethernet_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static int ethdemo_connected_get_partial( char *connection_name, char *buffer, uint16_t buffer_length);
err_t ethernet_client_poll_watchdog( void *arg, struct tcp_pcb *pcb);
int ethdemo_connected( char *connection_name, char *buffer, uint16_t buffer_length);
err_t ethdemo_recv( uint8_t *data, int len);
err_t ethdemo_disconnected( void);

err_t ethernet_CAT_poll_watchdog( void *arg, struct tcp_pcb *pcb);
err_t ethernet_CAT_connected( void *arg, struct tcp_pcb *tpcb, err_t err);
err_t ethernet_CAT_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
err_t ethernet_CAT_disconnected( void);
int ethernet_establishCATconnection(
                    int (* connected_cb)( char *connection_name),
                    err_t (* recv_cb)( uint8_t *data, int len),
                    err_t (* disconnected_cb)( void),
                    struct ip_addr *dest_ip,
                    int CATaddrs);
void ethernet_CAT_connect_error(void *arg, err_t err);
int http_parseGetHeader( char *buffer, int len, uint32_t *start_file);


__attribute__((weak)) uint16_t ethernet_client_CreateHeadersCallback( char *connection_name, char *http_command, char *buffer, uint16_t buffer_length);
err_t eth_llmnr_recv( struct ip_addr *ipaddr);          //!callback from llmnr_listen
void eth_llmnr_error( void);                            //!callback from llmnr
void *ethernet_timers( void *arg);


/**
 *   DATA DEFINES
 */
#define MAX_DEMO_RETRIES    300
#define USE_ETH_TIMERS_THREAD
#define LLMNR_RETRIES       100

/**
 * module's static data
 */
static kernel_pid_t eth_pid;
static uint32_t dest_ip, dhcpLedTimer;
static ringbuffer_t file_ringbuf;
static uint8_t run_timers;
uint8_t etht_stack[1524]                 __attribute__ ((section(".ccm")));
static uint32_t retryFromByte;              // retry from byte number using Range...
static uint8_t  availRetries, saveRetries;  // number of available retries
static uint32_t totalBytes;                 // total number of bytes to be read from HTTP server
static struct ip_addr hostIPaddr, defIP;    // host IP address as found by LLMNR protocol
static uint32_t totalRecv;                  // total bytes received in _recv function
static uint32_t memptrRecv;
static bool CATconnectError, isCatConnected, ongoingLoad;
static uint32_t lastCatRetryTime, llmnr_retries, llmnrRetryRequest;
static uint32_t watchDogClient, watchDogCAT, retry_llmnr;
static struct tcp_pcb *mypcb, *catpcb;
//=========================================================
static FIL     *rd_p80;
static char    httpFileName[32], diskFileName[32];
static uint8_t *eth_buff1, *eth_buff2;
//=========================================================


/************************************************/
/*    C A L L B A C K    F U N C T I O N S      */
/************************************************/
int (* client_connected_cb)( char *connection_name, char *buffer, uint16_t buffer_length);
err_t (* client_recv_cb)( uint8_t *data, int len);
err_t (* client_disconnected_cb)( void);
err_t (* client_file_loaded_cb)( void);
void ( *cat_recv_cb)( uint8_t *data, uint len);



/**
 * @brief an easy way to come up with a MAC per device
 * @param[in]   num     number of MAC address byte [0..5]
 * @return unique number as written in the cpuid on the chip
 */
uint8_t ethernet_getMAC( uint8_t num)
{
    uint8_t cpuid[CPUID_ID_LEN];
    cpuid_get( (void *)cpuid);
    if( num == 0) return 0xa0;
    return cpuid[CPUID_ID_LEN - num - 1];
}

/**
 * @brief   checks if load of file from internet server is ongoing now
 * @return  true if ongoing load is in running background
 */
int ethernet_isOngoingDownload( void)
{
    return (int)ongoingLoad;
}


char *ethernet_getLocalIP( void)
{
    return LwIP_getLocalIP();
}


/************************************************/
/*    E T H E R N E T      I N I T S            */
/************************************************/
/**
 * @brief Main Ethernet thread
 *
 * @param arg       pointer to callback function for receive
 */
void ethernet_thread( void *argument)
{
    ETH_Thread_Arg *arg = (ETH_Thread_Arg *)argument;
    cat_recv_cb = arg->cat_recv_cb;
    defIP = arg->default_ip;

    eth_pid = thread_getpid();
    run_timers = watchDogClient = watchDogCAT = 0;
    llmnr_retries = LLMNR_RETRIES;
    retry_llmnr = 5;
    hostIPaddr.addr = 0;

    ongoingLoad = false;
    CATconnectError = isCatConnected = false;
    catpcb = mypcb = NULL;


    /* configure ethernet (GPIOs, clocks, MAC, DMA) */
    ETH_BSP_Config( eth_pid, &ethrx_mutex);

    /* Initilaize the LwIP stack */
    LwIP_Init();
    Llmnr_Init( eth_llmnr_recv, eth_llmnr_error, arg->catHostName /*"NimrodW7x64"*/);    // will be incorporated inside LwIP in future...
    httpd_init( /*1*/);

#ifdef USE_ETH_TIMERS_THREAD // changed to very high priority
    thread_create( etht_stack, sizeof(etht_stack),
                   PRIORITY_MAIN - 3, CREATE_STACKTEST,
                   ethernet_timers, NULL, "eth_tmr");
    puts("Ethernet timers thread created");
#endif

#ifdef DEBUG_ETH_TIMING
    //cyclog_init( /*5 * */  5000);
#endif

    //! setup a queue of messages for communication between interrupt and thread
    msg_t mq[16], m;
    msg_init_queue( mq, 16);
    int counter;

    /* Infinite loop */
    while (1)
    {   //! call me back in 250mSecs or wake-up if have new packet received

#ifdef ETH_USE_MSG
        msg_receive( &m);
#endif
        counter = 0;

#ifdef DEBUG_ETH_TIMING
      cyclog_write( 5, xtimer_now(), 5);
#endif

#ifndef ETH_USE_MSG
      mutex_lock( &ethrx_mutex);

      if( haveRxFrame)
      {
          haveRxFrame = 0;
#endif
          while( ETH_CheckFrameReceived())
          {
              /* process received ethernet packet */
              LwIP_Pkt_Handle();
              //counter++;
              //if( counter > 19) { /*xtimer_usleep( 10000);*/ counter = 0; }
    #ifdef DEBUG_ETH_TIMING
              cyclog_write( 5, 0xFFFFFFFE, 5);
    #endif
          } // endwhile have a frame on hand
    #ifdef DEBUG_ETH_TIMING
          cyclog_write( 5, 0xFFFFFFFF, 5);
#endif

#ifndef ETH_USE_MSG
      } // endif have rx frame from interrupt
      mutex_unlock( &ethrx_mutex);
      xtimer_usleep( 1000);
#endif

    } // endwhile forever
} // endfunc thread


/************************************************/
/* E T H E R N E T   T I M E R S   T H R E A D  */
/************************************************/
/**
 * @brief this is the timers thread of the Ethernet main thread
 * @param arg
 */
void *ethernet_timers( void *arg)
{
uint32_t sentRetryRequest = 0;
uint32_t LocalTime;
int dhcpstate;
dhcpLedTimer = LocalTime = xtimer_now() / 1000;

    llmnrRetryRequest = 0;

    while( true)
    {
        if(( dhcpstate = LwIP_haveDHCP()))
        {
            /* Http client Init */
            //ethernet_http_client_init();
            if( LocalTime - dhcpLedTimer > 500 && isCatConnected)
            {
                gpio_toggle( LED_2);
                dhcpLedTimer = LocalTime;
            }
            if( dhcpstate == _DHCP_ADDRESS_ASSIGNED_FIRST)
            {   // start the Llmnr subprocess
                CATconnectError = false;
                lastCatRetryTime = LocalTime;
            }
            //! if LLMNR response wasn't received
            if( retry_llmnr == 0 && hostIPaddr.addr == 0)
            {
                eth_llmnr_error();
            }
            if( retry_llmnr &&
                ( LocalTime - llmnrRetryRequest > 5000 || !llmnrRetryRequest))
            {
                retry_llmnr--;
                Llmnr_Run( NULL);
                llmnrRetryRequest = LocalTime;
            }
        } // endif has dhcp

        /* handle periodic timers for LwIP */
        LwIP_Periodic_Handle(LocalTime);
        xtimer_usleep( 250000);

        //! is retry been requested
        if( /*retryFromByte != 0 &&*/ availRetries > 0 && LocalTime - sentRetryRequest > 5000)
        {
            /*
            availRetries--;
            //ethdemo_connected_get_partial( "noyb.avi", "noyb.avi", &hostIPaddr);
            ethdemo_getfile_retries( httpFileName, diskFileName, &hostIPaddr);
            saveRetries = availRetries;
            availRetries = 0;
            sentRetryRequest = LocalTime;
            */
        }

        //! retry TCP connection if previous connection failed
        if( (!isCatConnected && dhcpstate == _DHCP_ADDRESS_ASSIGNED) &&
               LocalTime - lastCatRetryTime > 10000 && hostIPaddr.addr != 0)
        {
            ethernet_establishCATconnection( ethernet_CAT_connected, ethernet_CAT_recv, ethernet_CAT_disconnected, NULL, 1);
            CATconnectError = false;
            lastCatRetryTime = LocalTime;
            watchDogCAT = 0;
        }
        else if( isCatConnected)
        {
            lastCatRetryTime = LocalTime;
            if( catpcb->state == CLOSED)
            {
                tcp_abandon( catpcb, 1);
                /* Free pcb */
                //mem_free( catpcb);
                catpcb = NULL;
                isCatConnected = false;
            }
        }
        LocalTime = xtimer_now() / 1000;
    } // endwhile forever

    //! shouldn't reach here
} // endfunc


/**
  * @brief  Status callback function, this function is called on change
  *         of network interface.
  * @param  The network interface
  * @retval None
  */
void ETH_status_callback(struct netif *netif)
{
}
/**
  * @brief  Link callback function, this function is called on change of link status.
  * @param  The network interface
  * @retval None
  */
void ETH_link_callback(struct netif *netif) {

    if( netif_is_link_up( netif))
        gpio_set( LED_1);
    else
        gpio_clear( LED_1);

#if 0
    __IO uint32_t timeout = 0;
    uint32_t tmpreg;
    struct ip_addr ipaddr;
    struct ip_addr netmask;
    struct ip_addr gw;

    if (netif_is_link_up(netif)) {
        /* Restart the auto-negotiation */

#if 0
        if (ETH_InitStructure.ETH_AutoNegotiation != ETH_AutoNegotiation_Disable)
        {
            /* Reset Timeout counter */
            timeout = 0;

            /* Enable auto-negotiation */
            ETH_WritePHYRegister(DP83848_PHY_ADDRESS, PHY_BCR, PHY_AutoNegotiation);

            /* Wait until the auto-negotiation will be completed */
            do {
                timeout++;
            } while (!(ETH_ReadPHYRegister(DP83848_PHY_ADDRESS, PHY_BSR) & PHY_AutoNego_Complete) && (timeout < (uint32_t)PHY_READ_TO));

            /* Reset Timeout counter */
            timeout = 0;

            /* Get speed and duplex mode from PHY */
            /* This is different for every PHY */
            ETH_EXTERN_GetSpeedAndDuplex(DP83848_PHY_ADDRESS, &ETH_InitStructure);

            /*------------------------ ETHERNET MACCR Re-Configuration --------------------*/
            /* Get the ETHERNET MACCR value */
            tmpreg = ETH->MACCR;

            /* Set the FES bit according to ETH_Speed value */
            /* Set the DM bit according to ETH_Mode value */
            tmpreg |= (uint32_t)(ETH_InitStructure.ETH_Speed | ETH_InitStructure.ETH_Mode);

            /* Write to ETHERNET MACCR */
            ETH->MACCR = (uint32_t)tmpreg;

            _eth_delay_(ETH_REG_WRITE_DELAY);
            tmpreg = ETH->MACCR;
            ETH->MACCR = tmpreg;
        }
#endif

        /* Restart MAC interface */
        ETH_Start();

#ifdef USE_DHCP
        ipaddr.addr = 0;
        netmask.addr = 0;
        gw.addr = 0;
        DHCP_state = DHCP_START;
#else
        IP4_ADDR(&ipaddr, TM_ETHERNET.ip_addr[0], TM_ETHERNET.ip_addr[1], TM_ETHERNET.ip_addr[2], TM_ETHERNET.ip_addr[3]);
        IP4_ADDR(&netmask, TM_ETHERNET.netmask[0], TM_ETHERNET.netmask[1], TM_ETHERNET.netmask[2], TM_ETHERNET.netmask[3]);
        IP4_ADDR(&gw, TM_ETHERNET.gateway[0], TM_ETHERNET.gateway[1], TM_ETHERNET.gateway[2], TM_ETHERNET.gateway[3]);
#endif /*ETHERNET_USE_DHCP */

        /* Set address */
        netif_set_addr( netif, &ipaddr , &netmask, &gw);

        /* When the netif is fully configured this function must be called.*/
        netif_set_up( netif);

        /* Call user function */
        TM_ETHERNET_INT_LinkIsUpCallback();

        EthLinkStatus = 0;
    } else {
        ETH_Stop();
#ifdef USE_DHCP
        DHCP_state = DHCP_LINK_DOWN;
        dhcp_stop(netif);
#endif /* ETHERNET_USE_DHCP */

        /*  When the netif link is down this function must be called. */
        netif_set_down( netif);

        /* Call user function */
        TM_ETHERNET_INT_LinkIsDownCallback();
    }
#endif
}

/************************************************/
/*         L L M N R    M A N A G E R           */
/************************************************/
/**
 * @brief when llmnr succeeds, the host ip address will be transferred here
 * @param ipaddr
 * @return
 */
err_t eth_llmnr_recv( struct ip_addr *ipaddr)
{
    retry_llmnr = 0;
    //Llmnr_Down();
    hostIPaddr = *ipaddr;

    // what was was, was was... ethernet_establishCATconnection( ethernet_CAT_connected, ethernet_CAT_recv, ethernet_CAT_disconnected, ipaddr, 1);
    isCatConnected = false;
    CATconnectError = true;
    lastCatRetryTime = 0;

    // this is now called from CAT---> ethdemo_getfile( "ffmpeg.exe", "ffmpeg.exe", ipaddr);
    return 0;
}

/**
 * @brief   handles llmnr error - when no reply is returned after a timeout?
 */
void eth_llmnr_error( void)
{
    retry_llmnr = 0;
    Llmnr_Down();
    struct ip_addr addr;
    IP4_ADDR( &addr, 192, 168, 1, 12);
    hostIPaddr = addr;
    if( defIP.addr != 0)
        hostIPaddr = defIP;

    // what was was, was was... start connecting to CAT
    isCatConnected = false;
    CATconnectError = true;
    lastCatRetryTime = 0;
}




#ifdef TOBEDELETED
/**
 * @brief Initializes the http client which loads files from the web server
 * @return
 */
int ethernet_http_client_init( void)
{
    ethernet_http_client_connect( "connection1", 192, 168, 2, 2, 80, NULL);
}
#endif


/************************************************/
/*         F I L E   D O W N L O A D            */
/* FROM PORT 80 INTERNET SERVER                 */
/************************************************/
/**
 * @brief starts a new tcp connection with a HTTP server in order to read a file
 *        directly from the HTTP server
 *
 * @param[in]   client_connected_cb - callback function called after TCP is connected
 * @param[in]   client_recv_cb - callback function for received data
 * @param[in]   client_disconnected_cb - callback function, called when TCP is disconnected
 * @param[in]   dest_ip - pointer to destination IP address structure
 *
 * @return -1 if no IP has been assigned yet by DHCP server (or link is down)
 *          0 if OK
 *          otherwise error in TCP (e.g. no more TCP sockets available)
 */
int ethernet_getfile( int (* connected_cb)( char *connection_name, char *buffer, uint16_t buffer_length),
                      err_t (* recv_cb)( uint8_t *data, int len),
                      err_t (* disconnected_cb)( void),
                      err_t (* file_loaded_cb)( void),
                      struct ip_addr *dest_ip)
{
    if( !LwIP_haveDHCP())
        return -1;

    // maybe should wait here for LLMNR also???

    client_connected_cb = connected_cb;
    client_recv_cb      = recv_cb;
    client_disconnected_cb = disconnected_cb;
    client_file_loaded_cb = file_loaded_cb;


    if( dest_ip == NULL)
        dest_ip = &hostIPaddr;

    return( ethernet_http_client_connect( "connection1",
                                          ip4_addr1( dest_ip)/*192*/,
                                          ip4_addr2( dest_ip)/*168*/,
                                          ip4_addr3( dest_ip)/*2*/,
                                          ip4_addr4( dest_ip)/*2*/, 80, NULL));
    //return 0;
}


/* Connects to the TCP echo server */
/**
 * @brief Connects to a TCP http server located at a specific IP and port
 * @param[in]   conn_name   connection name
 * @param[in]   ip1         IP(1) of destination
 * @param[in]   ip2         IP(2) of destination
 * @param[in]   ip3         IP(3) of destination
 * @param[in]   ip4         IP(4) of destination
 * @param[in]   port        port of destination http server (usually 80)
 * @param[in]   user_parameters
 * @return      err_t
 */
err_t ethernet_http_client_connect( char* conn_name,
                                    uint8_t ip1, uint8_t ip2, uint8_t ip3, uint8_t ip4,
                                    uint16_t port, void* user_parameters)
{
    struct ip_addr DestIPaddr;
    //struct tcp_pcb *pcb;

    if( mypcb != NULL)
    {
        tcp_abandon( mypcb, 1);
        mypcb = NULL;
        //ethdemo_disconnected();
        //ongoingLoad = false;
        //return ERR_CONN;
    }

    if( mypcb != NULL)
        while(1);

    /* create new tcp pcb */
    mypcb = tcp_new();
    if( mypcb == catpcb)
        while(1);

    if( mypcb != NULL) {
        /* Set IP address */
        IP4_ADDR( &DestIPaddr, ip1, ip2, ip3, ip4);

        /* Set parameters */
        tcp_arg( mypcb, (void *)conn_name);
        mypcb->flags |= TF_NODELAY /*| TF_ACK_NOW*/; // ack now is a test 01-05-17, removed 04-05 due to too many acks sent

        /* Set error function */
        tcp_err( mypcb, ethernet_client_error);

        /* connect to destination address/port */
        tcp_connect( mypcb, &DestIPaddr, port, ethernet_client_connected);

        /* set the poll callback function, for when the line is idle */
        tcp_poll( mypcb, ethernet_client_poll_watchdog, 10);

        /* Return OK */
        return ERR_OK;
    } else {
        /* Deallocate - what? */
        ongoingLoad = false;
    }
    /* Return error */
    return ERR_CONN;
}


////////////////////////////////////////////////////////////////////////////////
int ethdemo_getfile( char *http_filename, char *filename_disk,
                     err_t( *success_cb)( void),
                     struct ip_addr *addr)
{
FRESULT fr;
int ret = 0;


    if( eth_buff1 != NULL)
    {
        while(1);
    }

    /* allocate memory for 1st buffer */
    eth_buff1 = malloc( 1460 * 17 /*512 * 17*/);
    if( eth_buff1 == (uint8_t *)NULL)
    {
        return -1;
    }
#ifdef USE_TWO_BUFFER2
    eth_buff2 = malloc( 1460 * 12 /*512 * 17*/);
    if( eth_buff2 == (uint8_t *)NULL)
    {
        free( eth_buff1);
        return -1;
    }
#else
    eth_buff2 = NULL;
#endif
    //ringbuffer_init(&file_ringbuf, (char *)eth_buff1, 1460*12);

    rd_p80    = malloc( sizeof( FIL));
    fr = f_open_adj( rd_p80, filename_disk, FA_CREATE_ALWAYS | FA_WRITE);
    if( fr != FR_OK)
    {
        free( eth_buff2);
        free( eth_buff1);
        free( rd_p80);
        return -1;      // file could not be created
    }
    memcpy( httpFileName, http_filename, sizeof( httpFileName));
    memcpy( diskFileName, filename_disk, sizeof( diskFileName));

    ongoingLoad = true;
    ret = ethernet_getfile( ethdemo_connected,
                            ethdemo_recv,
                            ethdemo_disconnected,
                            success_cb, addr);
    if( ret == -1)
    {
        ongoingLoad = false;

        f_close( rd_p80);
        free( rd_p80);
        free( eth_buff1);
        free( eth_buff2);
        rd_p80 = NULL;
        eth_buff1 = NULL;
    }
    return ret;
} // endfunc


////////////////////////////////////////////////////////////////////////////////
int ethdemo_getfile_retries( char *http_filename, char *filename_disk, struct ip_addr *addr)
{
FRESULT fr;
int ret = 0;


    /* allocate memory for 1st buffer */
    eth_buff1 = malloc( 1460 * 12 /*512 * 17*/);
    if( eth_buff1 == (uint8_t *)NULL)
    {
        ongoingLoad = false;
        return -1;
    }
    eth_buff2 = malloc( 1460 * 12 /*512 * 17*/);
    if( eth_buff2 == (uint8_t *)NULL)
    {
        ongoingLoad = false;
        free( eth_buff1);
        return -1;
    }
    //ringbuffer_init(&file_ringbuf, (char *)eth_buff1, 1460*12);

    rd_p80    = malloc( sizeof( FIL));
    fr = f_open( rd_p80, filename_disk, /*FA_CREATE_ALWAYS |*/ FA_WRITE);
    if( fr != FR_OK)
    {
        ongoingLoad = false;
        free( rd_p80);
        return -1;      // file could not be created
    }
    // seek to last position
    f_lseek( rd_p80, rd_p80->fsize);
    memcpy( httpFileName, http_filename, sizeof( httpFileName));
    ret = ethernet_getfile( ethdemo_connected_get_partial,
                            ethdemo_recv,
                            ethdemo_disconnected,
                            client_file_loaded_cb, addr);

    if( ret == -1)
    {
        ongoingLoad = false;
        f_close( rd_p80);
        free( rd_p80);
        free( eth_buff1);
        free( eth_buff2);
    }
    return ret;
} // endfunc



int ethdemo_connected( char *connection_name, char *buffer, uint16_t buffer_length)
{
    //! check size of buffer if can hold so many bytes as requested
    if( buffer_length < strlen( httpFileName) + strlen( connection_name) + 30)
        return 0;

    saveRetries = MAX_DEMO_RETRIES;
    memptrRecv = totalRecv = 0;

    //! this seem to work OK
    sprintf( buffer, "GET %s%s HTTP/1.1\r\nHost: %s\r\n\r\n", /*WEBPAGE*/"/", httpFileName, connection_name);
    printf( "HTTP GET string: \n%s\n", buffer);

#ifdef DEBUG_TIMING
    //! write start to cyclic logger
    cyclog_write( 80, 0, 5);
#endif
    /* Return number of characters */
    return strlen( buffer);
}


/**
 * @brief   Gets partial file from internet server using HTTP
 * @param connection_name
 * @param buffer
 * @param buffer_length
 * @return
 */
static int ethdemo_connected_get_partial( char *connection_name, char *buffer, uint16_t buffer_length)
{
    //! check size of buffer if can hold so many bytes as requested
    if( buffer_length < strlen( httpFileName) + strlen( connection_name) + 30)
        return 0;

    memptrRecv = totalRecv = 0;

    //! this seem to work OK
    sprintf( buffer, "GET %s%s HTTP/1.1\r\nHost: %s\r\nRange: Bytes=%d-\r\n\r\n", /*WEBPAGE*/"/", httpFileName, connection_name, /*retryFromByte*/rd_p80->fsize);
    printf( "HTTP GET string: \n%s\n", buffer);

#ifdef DEBUG_TIMING
    //! write start to cyclic logger
    cyclog_write( 80, 0, 5);
#endif
    /* Return number of characters */
    return strlen( buffer);
}


/**
 * @brief   This function handles data reception using two buffers. Currently
 *          the write to SD functions wait until all data is writen before
 *          returning (blocking behavior) so there is no need for the two
 *          buffers.
 *
 * @param data
 * @param len
 * @return
 */
err_t ethdemo_recv( uint8_t *data, int len)
{
static uint32_t start_file, lastpct = 0, checksum;
static int file_size;
static uint8_t *ethBuff;
uint32_t bwritten;
FRESULT fr;
#define MAX_BUF_ETH         (512*/*34*/48)

    uint32_t t1 = xtimer_now();
    uint32_t copy_len = 0;

// time testing
//    return ERR_OK;


    if( eth_buff1 == NULL || rd_p80 == NULL)
    {
        tcp_abort( mypcb);
        ongoingLoad = false;
        return ERR_ABRT;
    }


    while( true)
    {
        if( memptrRecv + len >= MAX_BUF_ETH)
        {
            copy_len = MAX_BUF_ETH - memptrRecv;

            memcpy( &ethBuff[memptrRecv], data, copy_len);

            //tcp_recved( mypcb, copy_len);
#if 1
#if 0
            // calculate a (silly) check sum over the data
            uint32_t *cs = ( uint32_t *)&ethBuff[start_file];
            for( int x = 0; x < MAX_BUF_ETH / 4; x++)
                checksum += cs[x];
#endif
           uint32_t seekplc = rd_p80->fptr;
           uint32_t sst = xtimer_now();
           fr = f_fastwrite( rd_p80, &ethBuff[start_file], MAX_BUF_ETH/*memptr - start_file + copy_len*/, &bwritten);
           uint32_t est = xtimer_now() - sst;

           cyclog_write( 0x20, xtimer_now(), 5);
           cyclog_write( 0x21, est, 5);


           if( fr != FR_OK)
           {
#if 1
               for( int retries = 0; retries < 5; retries++)
               {
                   f_close( rd_p80);
                   f_open( rd_p80, diskFileName, FA_WRITE);
                   f_lseek( rd_p80, /*rd_p80->fsize*/seekplc);
                   fr = f_fastwrite( rd_p80, &ethBuff[start_file], MAX_BUF_ETH/*memptr - start_file + copy_len*/, &bwritten);
                   if( fr == FR_OK)
                       break;
                   if( retries == 4)
                   { //! major error -- abadon tcp connection and try again
                       tcp_abort( mypcb);
                       //! CLOSE THIS CONNECTION ??? need this here???
                       ethernet_client_connection_close( mypcb, 1);

                       mypcb = NULL;
                       return ERR_ABRT;
                   }
               } // endfor
#else
               tcp_abort( mypcb);
               //! CLOSE THIS CONNECTION ??? need this here???
               ethernet_client_connection_close( mypcb, 1);
               mypcb = NULL;
               return ERR_ABRT;
#endif
           } // endif
#else
            f_write( rd_p80, &eth_buff1[start_file], MAX_BUF_ETH/*memptr - start_file + copy_len*/, &bwritten);
#endif
            memptrRecv = 0;
            // should be copied to second buffer because this memcpy
            // treads over data to be copied by DMA to SD card
            //==========================
            // CHANGE TO 2nd BUFFER
            //==========================

#ifdef USE_TWO_BUFFER2
            if( ethBuff == eth_buff2)
                ethBuff = eth_buff1;
            else
                ethBuff = eth_buff2;
#endif
            memcpy( &ethBuff[memptrRecv], &data[copy_len], len - copy_len);
            memptrRecv      += len - copy_len;

// show progress will be handled in the future by another callback
//#define DEBUG_SHOW_PROGRESS
#ifdef DEBUG_SHOW_PROGRESS
            uint32_t pct = f_tell( rd_p80) / (totalBytes / 100);
            if(/*( pct % 10) == 0 &&*/ lastpct != pct)
            {
                printf( "%d%%\n", (int)pct);
                lastpct = pct;
            }
#endif
        }
        else
        {
            if( totalRecv == 0)
            { /** note that len includes the http header
                 so I am adding start to the file size
                */
                file_size = (int)http_parseGetHeader( data, len, &start_file) + start_file;
                totalBytes = file_size - start_file;
                //! always start with 1st buffer
                ethBuff = eth_buff1;
                checksum = 0;
            }
            memcpy( &ethBuff[memptrRecv], &data[start_file], len - start_file- copy_len);
            memptrRecv      += len - start_file;
        }
        file_size   -= len;
        totalRecv   += len;
        start_file = 0;
        break;
    }
    /**********************/
    /*   END OF FILE??    */
    /**********************/
    if( file_size <= 0)
    {   /**
         * if not previously written
         * memory to file - write the reminder now
         */
        if( memptrRecv != 0)
        {
            uint32_t written;
#if 0
            // calculate a (silly) check sum over the data
            uint32_t *cs = ( uint32_t *)&ethBuff[start_file];
            for( int x = 0; x < (memptrRecv - start_file) / 4; x++)
                checksum += cs[x];
#endif
#if 1
            fr = f_fastwrite( rd_p80, &ethBuff[start_file], memptrRecv - start_file, &written);
            // no need for that right now.... f_wait_for_dma_write();
#else
            f_write( rd_p80, &eth_buff1[start_file], memptr - start_file, &written);
#endif
        }
        //! CLOSE THIS CONNECTION
        ethernet_client_connection_close( mypcb, 1);

        /* notify client that the loading is done */
        if( client_file_loaded_cb != NULL && file_size == 0) client_file_loaded_cb();
    } // endif

#if 0
    ///////////////////////////////////////////////
    //! first packet contains HTTP GET header
    //! parse it
    if( total == 0)
    { /** note that len includes the http header
         so I am adding start to the file size
        */
        file_size = http_parseGetHeader( eth_buff1, len, &start_file) + start_file;
    }
    file_size -= len;
    total     += len;
    memptr    += len;

    //! testing writing to the new file on card
    if( memptr > 512*/*17*/34)
    {
    }
    if( file_size <= 0)
    {
        /**
         * if not previously written
         * memory to file - write the reminder now
         */
        if( memptr != 0)
        {
            uint32_t written;
            fr = f_fastwrite( rd_p80, &eth_buff1[start_file], memptr - start_file, &written);
        }
    }
#endif
#if 0
    // if first block received
    if( f_tell( rd_p80) == 0)
    {
        http_parseGetHeader( data, len, &start_file);
/*
        fr = f_fastwrite( rd, &data[start_file], len - start_file, &bwritten);
*/
        fr = f_write( rd, &data[start_file], len - start_file, &bwritten);
    }
    else
        fr = f_write( rd, data, len, &bwritten);
        /*
        fr = f_fastwrite( rd, data, len, &bwritten);
        */

#endif
    uint32_t t2 = xtimer_now() - t1;
    cyclog_write( 0x22, xtimer_now(), 5);
    cyclog_write( 0x23, t2, 5);
    return fr;
}

uint32_t ethernet_getUploadSize( void)
{
    return totalRecv;
}
uint32_t ethernet_getTotalUploadSize( void)
{
    return totalBytes;
}

/**
 * @brief   called by LwIP framework when connection is disconnected?
 * @return
 */
err_t ethdemo_disconnected( void)
{
uint32_t bwritten;

    //! wait until DMA finishes writing all data to file before closing it...
    // no need for that right now.... f_wait_for_dma_write();
    f_close( rd_p80);
    free( rd_p80);
    free( eth_buff1);
    free( eth_buff2);
    eth_buff1 = eth_buff2 = NULL;
    rd_p80 = NULL;
    return ERR_OK;
}


/************************************************/
/*     C A T    C O M M U N I C A T I O N S     */
/************************************************/
/**
 * @brief       sends a TCP packet to CAT host
 * @param testval
 * @param rx
 * @return
 */
err_t ethernet_CAT_send( uint8_t *testval, int rx)
{
    /* Allocate pbuf */
    struct pbuf *p_tx = pbuf_alloc(PBUF_TRANSPORT, rx, PBUF_POOL);

    /* If we have memory for buffer */
    if( p_tx) {
        /* send data */
        /* enqueue data for transmission */
        err_t wr_err = tcp_write( catpcb, testval, rx, 1);
        if( wr_err == ERR_OK && catpcb != NULL)
            tcp_output( catpcb);

        /* free pbuf: will free pbufs up to es->p (because es->p has a reference count > 0) */
        pbuf_free( p_tx);

        /* Return OK */
        return wr_err;
    }

}

/**
 * @brief starts a new tcp connection with a HTTP server in order to read a file
 *        directly from the HTTP server
 *
 * @param[in]   client_connected_cb - callback function called after TCP is connected
 * @param[in]   client_recv_cb - callback function for received data
 * @param[in]   client_disconnected_cb - callback function, called when TCP is disconnected
 * @param[in]   dest_ip - pointer to destination IP address structure
 *
 * @return -1 if no IP has been assigned yet by DHCP server (or link is down)
 *          0 if OK
 *          otherwise error in TCP (e.g. no more TCP sockets available)
 */
int ethernet_establishCATconnection(
                    int (* connected_cb)( char *connection_name),
                    err_t (* recv_cb)( uint8_t *data, int len),
                    err_t (* disconnected_cb)( void),
                    struct ip_addr *dest_ip,
                    int CATaddrs)
{
    if( !LwIP_haveDHCP())
        return -1;

    // maybe should wait here for LLMNR also???
/*
    client_connected_cb = connected_cb;
    client_recv_cb      = recv_cb;
    client_disconnected_cb = disconnected_cb;
*/

    if( dest_ip == NULL)
        dest_ip = &hostIPaddr;

    if( catpcb != NULL)
    {
        tcp_abandon( catpcb, 1);
        catpcb = NULL;
    }

    /* create new tcp pcb */
    catpcb = tcp_new();

    if( catpcb != NULL) {
        /* Set parameters */
        tcp_arg( catpcb, (void *)"eCAT");
        catpcb->flags |= TF_NODELAY /*| TF_ACK_DELAY*/;

        /* Set error function */
        tcp_err( catpcb, ethernet_CAT_connect_error);
        tcp_poll( catpcb, ethernet_CAT_poll_watchdog, 10);


        /* connect to destination address/port */
        tcp_connect( catpcb, dest_ip, 1050 + CATaddrs, connected_cb);

        /* Return OK */
        return ERR_OK;
    } else {
        /* Deallocate - what? */
    }
    /* Return error */
    return ERR_CONN;
}


/**
 * @brief Function called when CAT TCP connection is established
 * @param[in]   arg     name of connection
 * @param[in]   tpcb    pcb of connection
 * @param[in]   err     err_t structure connection status
 * @return err_t
 *
 * @note        Don't do anything after connected, Wait for PC to
 *              contact us.
 */
err_t ethernet_CAT_connected( void *arg, struct tcp_pcb *tpcb, err_t err)
{
    uint16_t length;

    if (err == ERR_OK) {
        /********************/
        /* We are connected */
        /********************/
        /* initialize LwIP tcp_recv callback function */
        tcp_recv( tpcb, ethernet_CAT_recv);

        isCatConnected = true;

        /* initialize LwIP tcp_sent callback function */
        //tcp_sent( tpcb, ethernet_client__sent);

        /* initialize LwIP tcp_poll callback function */
        //tcp_poll( tpcb, ethernet_client_poll, 1);

        /* Set new error handler */
        tcp_err( tpcb, ethernet_CAT_connect_error);

        /* Return OK */
        return ERR_OK;
    } else {
        /* close connection */
        ethernet_client_connection_close( tpcb, 0);
        isCatConnected = false;
    }
    return err;
} // endfunc


/**
 * @brief   called when the TCP connection gets disconnected
 * @return
 */
err_t ethernet_CAT_disconnected( void)
{
    CATconnectError = false;
    isCatConnected = false;
    // was tcp_close( catpcb);              // NS 09-10-16
    tcp_abandon( catpcb, 0);


    return ERR_OK;
}
/**
 * @brief tcp_receive callback
 * @param arg
 * @param tpcb
 * @param p
 * @param err
 * @return
 */
err_t ethernet_CAT_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    struct pbuf* point_pbuf;
    err_t ret_err;

    /* Check all next buffers and pointers */
    if (p != NULL && p->tot_len <= 1460)
    {
        for (point_pbuf = p; point_pbuf != NULL; point_pbuf = point_pbuf->next)
        {
            if (point_pbuf->len == 0) continue;

            /* Call user function if defined to respond to incoming data */
            //client_recv_cb( (uint8_t *) point_pbuf->payload, point_pbuf->len);
            cat_recv_cb( (uint8_t *) point_pbuf->payload, point_pbuf->len);
        } // endfor

        watchDogCAT = 0;

    } // endif have data

    /* if we receive an empty tcp frame from server => close connection */
    if (p == NULL) {
        /* we're done sending, close connection */
        //ethernet_CAT_connection_close( tpcb, 1);
        ret_err = ERR_OK;
    } else if (err != ERR_OK) {/* else : a non empty frame was received from echo server but for some reason err != ERR_OK */
        /* free received pbuf*/
        pbuf_free(p);
        ret_err = err;
    } else {
        /* Acknowledge data reception */
        tcp_recved( tpcb, p->tot_len);

        /* free pbuf and do nothing */
        pbuf_free( p);
        ret_err = ERR_OK;
    }
    /* Return */
    return ret_err;
} // endfunc







#if 0
/* Client based callbacks */
/**
 * @brief
 *
 *
 */
__attribute__((weak)) uint16_t ethernet_client_CreateHeadersCallback( char *connection_name, char *http_command, char *buffer, uint16_t buffer_length)
{
    //strcpy((char *)buffer, "GET /rom009.bmp HTTP/1.1\r\nHost: \r\n");

    sprintf( buffer, "GET %s%s HTTP/1.1\r\nHost: %s\r\n\r\n", /*WEBPAGE*/"/", "rom009.bmp", connection_name);

    /* Return number of characters */
    return strlen(buffer);
}
#endif


/**
  * @brief tcp_receiv callback
  */
err_t ethernet_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    struct pbuf* point_pbuf;
    err_t ret_err;

    /* Check all next buffers and pointers */
    if (p != NULL && p->tot_len <= 1460)
    {
        for (point_pbuf = p; point_pbuf != NULL; point_pbuf = point_pbuf->next)
        {
            if (point_pbuf->len == 0) continue;

            /* Call user function if defined to respond to incoming data */
            client_recv_cb( (uint8_t *) point_pbuf->payload, point_pbuf->len);


            //TM_ETHERNETCLIENT_ReceiveDataCallback(client, (uint8_t *) point_pbuf->payload, point_pbuf->len, point_pbuf->tot_len);
        } // endfor

        //! reset the watch dog for the polling
        watchDogClient = 0;

    } // endif have data

    /* if we receive an empty tcp frame from server => close connection */
    if (p == NULL) {
        /* we're done sending, close connection */
        ethernet_client_connection_close( tpcb, 1);
        ret_err = ERR_OK;
    } else if (err != ERR_OK) {/* else : a non empty frame was received from echo server but for some reason err != ERR_OK */
        /* free received pbuf*/
        pbuf_free(p);
        ret_err = err;
    } else {
        /* Acknowledge data reception */
        tcp_recved( tpcb, p->tot_len);

        /* free pbuf and do nothing */
        pbuf_free( p);
        ret_err = ERR_OK;
    }
    /* Return */
    return ret_err;
}



/**
  * @brief This function is used to close the tcp connection with server
  */
void ethernet_client_connection_close( struct tcp_pcb *pcb, uint8_t success)
{
    // 05-10-16 forget about retries for now...
    retryFromByte = 0;
    availRetries = 0;
    if( pcb != NULL) {
        tcp_recv( pcb, NULL);
        tcp_sent( pcb, NULL);
        tcp_poll( pcb, NULL, 0);

        /* Close tcp connection */
        if( tcp_close( pcb) == ERR_OK)
            pcb = NULL;
    }
    ethdemo_disconnected();
    ongoingLoad = false;
    return;




    //! it seems like I am getting FIN packet before the end of the file
    //! is reached, after only 30% is downloaded (128MB)
    //! in that case I should retry...
    uint32_t curpos = f_tell( rd_p80);
    //! should retry??
    if( curpos < totalBytes)
    {
        retryFromByte = curpos;
        availRetries = MAX_DEMO_RETRIES;
    }
    else
    {
        ongoingLoad = false;
        retryFromByte = 0;
        availRetries = 0;
    }


    /* Remove callbacks from PCB */
    if( pcb != NULL) {
        tcp_recv( pcb, NULL);
        tcp_sent( pcb, NULL);
        tcp_poll( pcb, NULL, 0);
    }

    /* Close tcp connection */
    if( pcb != NULL) {
        tcp_close( pcb);
    }
    /* Free pcb */
    //mem_free( pcb);
    mypcb = NULL;

    /* Call user function if defined */
    client_disconnected_cb();
    //TM_ETHERNETCLIENT_ConnectionClosedCallback(client, success);
}



err_t ethernet_client_poll_watchdog( void *arg, struct tcp_pcb *pcb)
{
    watchDogClient++;
    if( watchDogClient == 2)
    {
        tcp_abandon( pcb, 1);
        // ????? tcp_close( pcb);
        /* Free pcb */
        //mem_free( pcb);
        mypcb = NULL;           // pcb has been freed

        watchDogClient = 0;
        availRetries = 0;
        ongoingLoad = false;
    }
    return ERR_OK;
}


static uint last_poll_time = 0;
err_t ethernet_CAT_poll_watchdog( void *arg, struct tcp_pcb *pcb)
{
    watchDogCAT++;
    if( watchDogCAT >= 2 && xtimer_now() - last_poll_time > 10000000)
    {
        tcp_poll( pcb, NULL, 0);
        tcp_close( pcb);
        /* Free pcb */
        //mem_free( pcb);

        // ???? tcp_abandon( pcb, 1);               // ??????? pcb should have been released
        catpcb = NULL;
        watchDogCAT = 0;
        CATconnectError = true;
        isCatConnected = false;
        last_poll_time = xtimer_now();
    }
}

/**
  * @brief Function called when TCP connection error
  */
void ethernet_CAT_connect_error(void *arg, err_t err) {
    /* Call user function */
    //TM_ETHERNETCLIENT_ErrorCallback(client);

    /* Close connection */
    //tcp_echoclient_connection_close(client, 0);

    printf( "CAT TCP ERROR %d\n", err);

    // if we encounter an error try to abort...
    // no need.... tcp_abort( catpcb);
    //????????? tcp_close( catpcb);
    /* Free pcb */
    //mem_free( catpcb);

    catpcb = NULL;
    CATconnectError = true;
    isCatConnected = false;
    //lastCatRetryTime = 0;           // try a retry fast
}


/**
  * @brief Function called when TCP connection error
  */
void ethernet_client_error(void *arg, err_t err) {
    /* Call user function */
    //TM_ETHERNETCLIENT_ErrorCallback(client);

    /* Close connection */
    //tcp_echoclient_connection_close(client, 0);

    printf( "TCP ERROR %d\n", err);

    // move this to a callback sometime soon
    if( rd_p80 != NULL)
    {
#if 0
        uint32_t curpos = f_tell( rd_p80);
        //! should retry??
        if( curpos < totalBytes)
        {
            retryFromByte = curpos;
            availRetries = saveRetries;
        }
        else
#endif
        {
            ongoingLoad = false;
            retryFromByte = 0;
            // was .... availRetries = 0;
            availRetries = saveRetries;
        }
        f_close( rd_p80);
        free( rd_p80);
        free( eth_buff1);
        free( eth_buff2);
        rd_p80 = NULL;
        eth_buff1 = eth_buff2 = NULL;

        // if error function called - the pcb is long gone...
        mypcb = NULL;
    }
}

#if 0
/**
 * @brief  gets a file through HTTP from a remote HTTP server
 * @param filename - the file name to get
 * @return -1 on error, 0 OK
 */
int http_getFile( char *filename_disk, char *filename_web, uint32_t ipAddrs, int withWrite)
{
FIL     *rd    = malloc( sizeof( FIL));
FRESULT fr;
int ret = 0;


    if( filename_web == NULL)
        filename_web = filename_disk;

    if( withWrite)
    {
        fr = f_open( rd, filename_disk, FA_CREATE_ALWAYS | FA_WRITE);
        if( fr != FR_OK)
        {
            free( rd);
            return fr;      // file could not be created
        }
    }
    printf( "Creating socket ... ");
    ethernet_http_client_connect( "connection1", 192, 168, 2, 12, 80, NULL);
    return 1;
}
#endif

/**
  * @brief Function called when TCP connection established
  */
/**
 * @brief Function called when TCP connection is established
 * @param[in]   arg     name of connection
 * @param[in]   tpcb    pcb of connection
 * @param[in]   err     err_t structure connection status
 * @return err_t
 */
err_t ethernet_client_connected( void *arg, struct tcp_pcb *tpcb, err_t err)
{
    uint16_t length;

    if (err == ERR_OK) {
        /* We are connected */
        /* Get HTTP request from user */
        char *ipbuf = ip_ntoa( &tpcb->remote_ip);
        char buffer_out[128];
        strcpy( buffer_out, (char *)arg);

        char snd[100];

        // call callback
        length = client_connected_cb( ipbuf, buffer_out, sizeof( buffer_out));

        //length = ethernet_client_CreateHeadersCallback( ipbuf, "12121.dat", buffer_out, sizeof( buffer_out));

        /* Check if length = 0 */
        if (length == 0) {
            /* Close connection */
            ethernet_client_connection_close( tpcb, 1);

            /* Return */
            return ERR_CONN;
        }
#if 1
        /* Allocate pbuf */
        struct pbuf *p_tx = pbuf_alloc(PBUF_TRANSPORT, strlen((char *)buffer_out), PBUF_POOL);

        /* If we have memory for buffer */
        if( p_tx) {
            /* Call user function */
            //TM_ETHERNETCLIENT_ConnectedCallback(client);

            /* copy data to pbuf */
            pbuf_take( p_tx, (char *)buffer_out, length);

            /* initialize LwIP tcp_recv callback function */
            tcp_recv( tpcb, ethernet_client_recv);

            /* initialize LwIP tcp_sent callback function */
            //tcp_sent( tpcb, ethernet_client__sent);

            /* initialize LwIP tcp_poll callback function */
            //tcp_poll( tpcb, ethernet_client_poll, 1);

            /* Set new error handler */
            tcp_err( tpcb, ethernet_client_error);

            /* send data */
            /* enqueue data for transmission */
            err_t wr_err = tcp_write( tpcb, buffer_out, length, 1);

            /* free pbuf: will free pbufs up to es->p (because es->p has a reference count > 0) */
            pbuf_free( p_tx);

            /* Return OK */
            return wr_err;
        }
    } else {
        /* close connection */
        ethernet_client_connection_close( tpcb, 0);
#endif
    }
    return err;
}
#if 0
    #define MAX_PACKET_SIZE   1600
    #define WEBSITE      "192.168.1.12"
    #define WEBPAGE      "/extras/"
    char snd[100];
    uint32_t total = 0, memcnt = 0;

    sprintf( snd, "GET %s%s HTTP/1.1\r\nHost: %s\r\n\r\n", WEBPAGE, filename_web, WEBSITE);

    Status = socket_Send( SockID, snd, strlen( snd), 0 );
    if( Status <= 0 )
    {
        free( rd);
        socket_Close( SockID);
        printf(" [TCP Client] Data send Error \n\r");
        /*
        Status = socket_Close( SockID);
        SockID = socket_Open( SL_AF_INET, SL_SOCK_STREAM, 0);
        Status = socket_Connect( SockID, ( SlSockAddr_t *)&Addr, AddrSize);
        Status = socket_Send( SockID, snd, strlen( snd), 0 );
        if( Status <= 0) while( 1);
        */
        return -1;
    }
    /* allocate memory for 1st buffer */
    uint8_t *buff1 = malloc( 1460 * 12 /*512 * 17*/);
    if( buff1 == (uint8_t *)0xFFFFFFFF)
    {
        free( rd);
        socket_Close( SockID);
        return -1;
    }
#if 1
    /* allocate memory for 2nd buffer */
    uint8_t *buff2 = malloc( 1460 * 12);
    if( buff2 == (uint8_t *)0xFFFFFFFF)
    {
        free( buff1);
        free( rd);
        socket_Close( SockID);
        return -1;
    }
#else
    int8_t *buff2 = buff1;
#endif
    uint32_t tt1 = xtimer_now(), written;
  /**
   * READ FROM SOCKET LOOP
   */
  uint32_t start = 0, file_size;
  uint8_t *buff = buff1;
  while( g_Status & ( wlanConnected | HasDHCP))
  {
      int gred = socket_Recv( SockID, &buff[memcnt], 1460, 0);
      cc3100_background( 0);        // testing...

      if( gred < 0)
      { /* ERROR ! */
          ret = -1;
          break;
      }

      //! first packet contains HTTP GET header
      //  parse it
      if( total == 0)
      { /** note that gred includes the http header
           so I am adding start to the file size
          */
          file_size = http_parseGetHeader( buff, 1460, &start) + start;
      }
      file_size -= gred;

      total += gred;
      if( fr == FR_OK)
          memcnt += gred;

      //! testing writing to the new file on card
      if( /*fr == FR_OK &&*/ memcnt > 512*/*17*/34)
      {
          // dump all indexes to file
          //f_write( rd, &buff[start], 1460*6 - start, &written);
          if( withWrite)
          {
              fr = f_fastwrite( rd, &buff[start], memcnt /*1460*6*/ - start, &written);
              if( fr != FR_OK)
              {
                 ret = fr;
                 break;
              }
              if( buff == buff1)
                  buff = buff2;
              else
                  buff = buff1;
          }
          start = memcnt = 0;
      }

      if( file_size <= 0)
      {
          /**
           * if not previously written
           * memory to file - write the reminder now
           */
          if( memcnt != 0 && withWrite)
          {
              uint32_t written;
              fr = f_fastwrite( rd, &buff[start], memcnt - start, &written);
              if( fr != FR_OK)
              {
                 ret = fr;
                 break;
              }
          }
          ret = 0;
          break;
      }
  } // endwhile forever
  /*----- Finally, close the SD card DMA -----*/
  if( withWrite)
      f_wait_for_dma_write();

  int tt2 = xtimer_now() - tt1;

  socket_Close( SockID);
  fr = f_close( rd);
  free( rd);
  free( buff1);
  free( buff2);
  return ret;
} // endfunc http get
#endif


/**
 * @brief takes the first received HTTP GET string and
 *        parses it in order to find the content length.
 *
 * @param buffer
 * @param len
 *
 * @return
 */
int http_parseGetHeader( char *buffer, int len, uint32_t *start_file)
{
    /*
     * HTTP/1.1 200 OK
        Date: Fri, 15 May 2015 09:37:00 GMT
        Server: Apache/2.4.12 (Win64) OpenSSL/1.0.1l PHP/5.5.21
        X-Distributed-by: AHC
        Last-Modified: Fri, 18 Jan 2013 20:13:04 GMT
        ETag: "11dc50f-4d395bd28d400"
        Accept-Ranges: bytes
        Content-Length: 18728207
     *
     */
    // start by checking HTTP/x.x 200 OK
    if( memcmp( &buffer[9], "200 OK", 6))
        return 0;// NOT OK


    char lookfor[] = "Content-Length: ";
    int p;
    for( p = 0; p < len; p++)
    {
        if( !memcmp( &buffer[p], lookfor, strlen( lookfor)))
        {
           p = p + strlen( lookfor);
           break;
        }
    }
    if( p == len)
        return 0;// not found?!

    //! find the end of the header by the two CR+LFs
    for( int l = p; l < len; l++)
    {
        if( buffer[l] == 0x0D && buffer[l + 1] == 0x0A
            && buffer[l + 2] == 0x0D && buffer[l + 3] == 0x0A)
        {
            printf( "HTTP header end found at %d\n", l + 4);
            *start_file = l + 4;        // 2 LF and 2 CR = 4 bytes
            break;
        }
    } // endfor look for string


    for( int l = 0; l < 100; l++)
    {
        if( buffer[l + p] == 0x0D && buffer[l + p + 1] == 0x0A
            /* && buffer[l + p + 2] == 0x0D && buffer[l + p + 3] == 0x0A*/)
        {
            char tmp[32];
            memcpy( tmp, &buffer[p], l);
            tmp[l] = 0;

            printf( "HTTP file size is %s\n", tmp);
            //*start_file = p + l + 4;        // 2 LF and 2 CR = 4 bytes
            return atoi( tmp);
        }
    } // endfor look for string
    return 0;
} // endfunc


#if 0
uint8_t *cyclog_buf;
ringbuffer_t cyclog;
/**
 * @brief Inits a cyclic logger entity
 * @param[in]   size - byte size for the entire logger
 * @return      -1 if error (not enough memory)
 */
int cyclog_init( uint32_t size)
{
    cyclog_buf = malloc( size);
    if( cyclog_buf == NULL)
        return -1;
    ringbuffer_init( &cyclog, (char *)cyclog_buf, size);
    return 0;   // all is OK
}
/**
 * @brief writes a stream of data to the cyclic logger
 * @param buf
 * @param length
 * @return
 */
void cyclog_write( uint8_t type, uint32_t tme, uint32_t length)
{
    uint8_t buf[length];
    buf[0] = type;
    buf[1] = (tme & 0xff000000) >> 24;
    buf[2] = (tme & 0x00ff0000) >> 16;
    buf[3] = (tme & 0x0000ff00) >> 8;
    buf[4] = (tme & 0x000000ff);
    ringbuffer_add( &cyclog, buf, length);
}

void cyclog_dump( int element_size)
{
    uint8_t buf[element_size];

    while( 1)
    {
        int n = ringbuffer_get( &cyclog, buf, element_size);
        if( n > 0)
        {
            uint8_t type  = buf[0];
            uint32_t data = buf[1] << 24 | buf[2] << 16 | buf[3] << 8 | buf[4];
            printf( ">%X:%u\n", (int)type, (int)data);
        }
        else
            break;
    } // endwhile
}
#endif
