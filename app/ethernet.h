#ifndef ethernet_h
#define ethernet_h


#ifdef __cplusplus
extern "C" {
#endif

#include "lwip/ipv4/ip_addr.h"
#include "lwip/err.h"


typedef struct
{
    void ( *cat_recv_cb)( uint8_t *data, uint len); // callback for CAT receive function
    struct ip_addr default_ip;                      // default IP when LLMNR isn't answering...
    const char *catHostName;                        // Computer Name of CAT Host

} ETH_Thread_Arg;


/**
 * @brief  Default device name to be shown to your router when requesting IP address with DHCP
 */
#ifndef ETHERNET_HOSTNAME_DHCP
#define ETHERNET_HOSTNAME_DHCP              "ETH-Device-1"
#endif


/* MII and RMII mode selection, for STM324xG-EVAL Board(MB786) RevB ***********/
#define RMII_MODE    // User have to provide the 50 MHz clock by soldering a 50 MHz
                     // oscillator (ref SM7745HEV-50.0M or equivalent) on the U3
                     // footprint located under CN3 and also removing jumper on JP5.
                     // This oscillator is not provided with the board.
                     // For more details, please refer to STM3240G-EVAL evaluation
                     // board User manual (UM1461).


//#define MII_MODE

/* Uncomment the define below to clock the PHY from external 25MHz crystal (only for MII mode) */
#ifdef  MII_MODE
 #define PHY_CLOCK_MCO
#endif

/* STM324xG-EVAL jumpers setting
    +==========================================================================================+
    +  Jumper |       MII mode configuration            |      RMII mode configuration         +
    +==========================================================================================+
    +  JP5    | 2-3 provide 25MHz clock by MCO(PA8)     |  Not fitted                          +
    +         | 1-2 provide 25MHz clock by ext. Crystal |                                      +
    + -----------------------------------------------------------------------------------------+
    +  JP6    |          2-3                            |  1-2                                 +
    + -----------------------------------------------------------------------------------------+
    +  JP8    |          Open                           |  Close                               +
    +==========================================================================================+
  */



/************************************************/
/*    E T H E R N E T      I N I T S            */
/************************************************/
/**
 * @brief Ethernet thread
 * @param arg
 */
void ethernet_thread( void *arg);

/**
  * @brief  Link callback function, this function is called on change of link status.
  * @param  The network interface
  * @retval None
  */
void ETH_link_callback(struct netif *netif);
/**
  * @brief  Status callback function, this function is called on change
  *         of network interface.
  * @param  The network interface
  * @retval None
  */
void ETH_status_callback(struct netif *netif);

/**
 * @brief       sends a TCP packet to CAT host
 * @param testval
 * @param rx
 * @return
 */
err_t ethernet_CAT_send( uint8_t *testval, int rx);


/**
 * @brief       start reading process of file from internet server (port 80)
 *              using HTTP protocol
 *
 * @param http_filename - name of file on server to download
 * @param filename_disk - name of file to save locally
 * @param addr
 * @return
 */
int ethdemo_getfile( char *http_filename, char *filename_disk,
                      err_t( *success_cb)( void), struct ip_addr *addr);


/**
 * @brief   checks if load of file from internet server is ongoing now
 * @return  true if ongoing load is in running background
 */
int ethernet_isOngoingDownload( void);
/**
 * @brief   returns the number of bytes loaded so far from internet (port 80)
 *          host
 *
 * @return
 */
uint32_t ethernet_getUploadSize( void);
/**
 * @brief   returns the total number of bytes to be uploaded from internet (port 80)
 *          host
 *
 * @return
 */
uint32_t ethernet_getTotalUploadSize( void);

/**
 * @brief   returns the ASCII representation of the IP address received from
 *          the DHCP server or NULL if DHCP process is not yet complete
 * @return
 */
char *ethernet_getLocalIP( void);


#ifdef __cplusplus
}
#endif



#endif // ethernet_h
