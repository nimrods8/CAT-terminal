/**
 * @{
 *
@file
 * @brief  Implementation of the Domestic CAT (Customer Activated Terminal)
 *         device rev D document from August 2002
 *
 * @note   CAT device is a slave unit. It only responds to messages from the
 *         POS master.
 *
 * @author Nimrod Stoler <nstoler@gmail.com>
 */

#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include "time.h"
#include "board.h"
//NS does not exist anymore.... #include "magStripe.h"
#include "printer.h"
#include "ethernet.h"
#include "mutex.h"
#ifdef HWTIMER
#include "hwtimer.h"
#include "vtimer.h"
#else
#include "xtimer.h"
#endif
#include "periph/uart.h"
#include "thread.h"
#include "sched.h"
#include "msg.h"
#include "ringbuffer.h"
#include "ps.h"
#include "ugui.h"
#include "stm32f429i_discovery_sdram.h"
#include "../../app/aviriff.h"
#include "periph/dac.h"
#include "periph/cpuid.h"
#include "rtcbkp.h"
#include "stm32f4xx_rtc.h"
#include "rtcbkp.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_ltdc.h"
#include "stm32f4xx_dac.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_tim.h"
#include "timex.h"
#include "periph/gpio.h"
#include "periph/adc.h"
#include "lwip/ipv4/ip_addr.h"

#include "ps.h"

#include "crypto/bigInteger.hpp"

//! FAT-fs files
#include "fatfs_sd_sdio.h"
#include "ff.h"

#include "shell_commands.h"
#include "board_uart0.h"

#include "gt811.h"

#include "clip.h"
#include "riffparser.h"
#include "display.h"
#include "creator.h"
#include "emvCard.hpp"
#include "epp.h"
//#include "httpd.h"
#include "CAT.hpp"
#include "ashProtocol.hpp"

#include  "cyclog.h"
#include  "swuart.h"
#include "microRWD.h"
#include "mt188.h"
#include "sw3501b.h"


// for version control
#include "../../app/version.h"



/********************
 *  MAIN SWITCHES   *
 ********************/
#define CAT_DEFAULT_ADDRESS         1
#define USE_ETHERNET
//#define USE_SWIPE
#ifndef USE_SWIPE
#define USE_CREATOR_READER
#endif
//#define PLAY_WAVE

////////////////////////////////
// removing this switch would
// lower the stack requirements
// considerably!!
#define DEBUG_CAT
////////////////////////////////
#define FIRST_LINE      TXB_ID_0
#define SECOND_LINE     TXB_ID_1

//extern GUI_BITMAP bmscr_complete800x480;

void CAT_keyPressed( char buttonID);

/**
 * module's static functions
 */
static void CAT_inject( char *data, int len);
uint16_t CAT_CRC16( uint8_t data, uint16_t crc);
static void CAT_rxbyte( void *arg, char data);
static int CAT_txbyte( void *arg);
static void CAT_initialize( void);
static FRESULT CAT_mountDisk( void);
FRESULT CAT_findFile( char *filename);
static uint32_t CAT_decompress( char *ucmsg, uint32_t *uclen, char *cmsg,
        uint32_t clen, char DLEchar, char cchar);
int CAT_buildKeypadBuf( unsigned inx, char *tempBuffer, char *keybuffer,
        char zero, char decimalpt);
kernel_pid_t CAT_runClipThread( char *filename);
uint32_t CAT_buildStatus( void);
uint8_t CAT_showROMframe( uint8_t flashNum);
// dac tests
void *test_stopSound( void *arg);
static void DAC2_Config( void);
static void TIM6_Config( void);

bool CAT_checkIfMagStripeConnected( void);
void CAT_soundBuzzer( uint32_t msec);
void CAT_notifyClipThreadDone( void);

/**
 * @brief makes sure that the keypad is displayed
 */
void CAT_showKeypad( void);
void CAT_scanForCommandsAndShow( char *Text, uint8_t cnt, int id);
int test_startSound( uint32_t msec);
void CAT_recv( uint8_t *data, uint len);
uint8_t CAT_showShortClip( const char *clipName);
int CAT_runShortClip( uint16_t *len, uint8_t shouldReply);
void CAT_stopShortClip( void);
int CAT_buildStatusString( void);
err_t CAT_loadClipSuccess( void);
FRESULT CAT_renameClip( void);
void CAT_calcFileChecksum( void *arg);

int CAT_reset( uint16_t *len, uint8_t shouldReply);
static int CAT_softwareStatusTest( uint16_t *length, uint8_t shouldReply);
static int CAT_prepareTxDataAndSend( uint8_t *buff, uint32_t length);
int CAT_printText( uint16_t *length, uint8_t shouldReply);
int CAT_getTrack1( uint16_t *length, uint8_t shouldReply);
int CAT_getTrack2( uint16_t *length, uint8_t shouldReply);
int CAT_getTrack3( uint16_t *length, uint8_t shouldReply);
int CAT_getRFID( uint16_t *length, uint8_t shouldReply);
int CAT_getConfiguration( uint16_t *length, uint8_t shouldReply);
int CAT_getEightBitStatus( uint16_t *length, uint8_t shouldReply);
int CAT_get16BitStatus( uint16_t *length, uint8_t shouldReply);
int CAT_get24BitStatus( uint16_t *length, uint8_t shouldReply);
int CAT_get40BitStatus( uint16_t *length, uint8_t shouldReply); // my new own function

int CAT_storeToRAM( uint16_t *length, uint8_t shouldReply);
int CAT_beep( uint16_t *length, uint8_t shouldReply);
int CAT_setBeepDuration( uint16_t *length, uint8_t shouldReply);
int CAT_magStripeOff( uint16_t *length, uint8_t shouldReply);
int CAT_magStripeOn( uint16_t *length, uint8_t shouldReply);
int CAT_returnCardStatus( uint16_t *length, uint8_t shouldReply);
int CAT_waitForCardBufferFull( uint16_t *length, uint8_t shouldReply);
int CAT_waitForCardBufferFullPlus( uint16_t *length, uint8_t shouldReply);
int CAT_executeRAM( uint16_t *length, uint8_t shouldReply);
int CAT_commandExecute( int cmd, uint16_t *cnt, uint8_t shouldReply);
int CAT_defineKeypad( uint16_t *len, uint8_t shouldReply);
int CAT_configureKeypad( uint16_t *len, uint8_t shouldReply);
int CAT_displayAndActivateKeypad( uint16_t *len, uint8_t shouldReply);
int CAT_activateKeypadDisplay( uint16_t *len, uint8_t shouldReply);
int CAT_waitForKeypadLock( uint16_t *len, uint8_t shouldReply);
int CAT_clearKeyBuffer( uint16_t *len, uint8_t shouldReply);
int CAT_waitTime( uint16_t *len, uint8_t shouldReply);
int CAT_setGraphicsMode( uint16_t *len, uint8_t shouldReply);
int CAT_unsetGraphicsMode( uint16_t *len, uint8_t shouldReply);
int CAT_lockKeypad( uint16_t *len, uint8_t shouldReply);
int CAT_returnKeyBuffer( uint16_t *len, uint8_t shouldReply);
int CAT_stopCommandExecution( uint16_t *len, uint8_t shouldReply);
int CAT_GetRamExecutionState( uint16_t *len, uint8_t shouldReply);
int CAT_printGraphics( uint16_t *len, uint8_t shouldReply);
int CAT_defineVerifoneFrame( uint16_t *len, uint8_t shouldReply);
int CAT_downloadVerifoneFrame( uint16_t *len, uint8_t shouldReply);
int CAT_printFlashGraphics( uint16_t *len, uint8_t shouldReply);
int CAT_displayMultiple( uint16_t *len, uint8_t shouldReply);
int CAT_initializeBitmapFrame( uint16_t *len, uint8_t shouldReply);
int CAT_downloadCompressedFrame( uint16_t *len, uint8_t shouldReply);
int CAT_clearPowerFlag( uint16_t *length, uint8_t shouldReply);
int CAT_displayROMFrame( uint16_t *len, uint8_t shouldReply);
int CAT_setMulticast( uint16_t *len, uint8_t shouldReply);
int CAT_runClip( uint16_t *len, uint8_t shouldReply);
int CAT_loadClip( uint16_t *len, uint8_t shouldReply);
int CAT_messageBox( uint16_t *len, uint8_t shouldReply);
int CAT_fullMessageBox( uint16_t *len, uint8_t shouldReply);

int CAT_displayMultipleFrames( uint16_t *len, uint8_t shouldReply);
int CAT_getBoardConfiguration( uint16_t *len, uint8_t shouldReply);
int CAT_setTimeDate( uint16_t *len, uint8_t shouldReply);
int CAT_emvStart( uint16_t *len, uint8_t shouldReply);
int CAT_emvContinue( uint16_t *len, uint8_t shouldReply);
int CAT_emvTagData( uint16_t *length, uint8_t shouldReply);
int CAT_getEmvReturnCode( uint16_t *len, uint8_t shouldReply);
int CAT_setEmvTagData( uint16_t *length, uint8_t shouldReply);
int CAT_emvFinalize( uint16_t *len, uint8_t shouldReply);
int CAT_injectKey( uint16_t *len, uint8_t shouldReply);

int CAT_showNoCommClip( void);
int CAT_restartNoCommClip( void);
int CAT_stopNoCommClip( void);


char wave_playback( const char *FileName);
int usb_main( void);
void runCommandLoop( uint16_t cnt);

/********************
 *   DATA DEFINES   *
 *   CAT PROTOCOL   *
 ********************/
#define SYNC        0xFE                    // SYNC - start of frame
#define SF          0xFD                    // Stop Flag
#define DLE         0xFC                    // Data Link Escape
#define MULTICAST_ADDRESS   0xFF            // address for multicast
#define RESET_SAFETY_CHAR   0xE0            // safety before resetting

#define CAT_TX_BUFFER_SZ            100

/* command execution returns */
#define I_WILL_DEAL_WITH_RETURN     -1
#define IN_COMMAND_EXECUTION        -2
#define NEXT_COMMANDS_ABORT         -3
#define COMMAND_NOT_FOUND           -4
#define FINISHED_EXECUTING_RAM      -5
#define NOTHING_EXECUTE             -999

/* for status bits returned */
#define GET_STATUS_LOW_8BIT(dat)       ( dat & 0xFF)
#define GET_STATUS_MID_8BIT(dat)       (( dat & 0xFF00) >> 8)
#define GET_STATUS_HIGH_8BIT(dat)      (( dat & 0xFF0000) >> 16)

//state machine:
typedef enum
{
    STATE_RESTART,
    STATE_IDLE,
    STATE_GOT_SYNC,
    STATE_MY_ADDRESS,
    STATE_GET_CRC1,
    STATE_GET_CRC2
} enumCAT;
////////////////////////////////////////////////////////////////////////////
//! REMOVES cnt bytes from rx ring buffer
#define REMOVE_FROM_RX( cnt)        \
    {                               \
        mutex_lock(&cmd_mutex);     \
        ringbuffer_remove(&cmd_ringbuf, cnt);   \
        mutex_unlock(&cmd_mutex);   \
    }
/**
 * @brief Signature for the command function
 *
 * @param[in] arg
 *
 * @return
 * @return
 */
typedef int (*CAT_command_t)( uint16_t *length, uint8_t shouldReply);

typedef enum
{
    RAM_ONLY, LINK_ONLY, BOTH
} enumType_t;

// CAT commands
typedef struct
{
    uint8_t cmdnum;
    CAT_command_t cmd_cb;
    enumType_t type;
} CATcmds_t;

static const CATcmds_t commands[] =
{
{ 0x00, CAT_softwareStatusTest, LINK_ONLY },
{ 0x01, CAT_configureKeypad, RAM_ONLY },
{ 0x03, CAT_activateKeypadDisplay, BOTH },
{ 0x04, CAT_displayAndActivateKeypad, BOTH },
{ 0x05, CAT_executeRAM, BOTH },
{ 0x07, CAT_lockKeypad, BOTH },
{ 0x08, CAT_printText, BOTH },
{ 0x0A, CAT_getTrack1, LINK_ONLY },
{ 0x0B, CAT_getTrack2, LINK_ONLY },
{ 0x0C, CAT_getTrack3, LINK_ONLY },
{ 0x0D, CAT_getConfiguration, LINK_ONLY },
{ 0x0E, CAT_returnKeyBuffer, LINK_ONLY },
{ 0x0F, CAT_getEightBitStatus, LINK_ONLY },
{ 0x10, CAT_beep, BOTH },
{ 0x11, CAT_stopCommandExecution, BOTH },
{ 0x12, CAT_storeToRAM, BOTH },
{ 0x13, CAT_magStripeOff, BOTH },
{ 0x14, CAT_magStripeOn, BOTH },
{ 0x17, CAT_waitForCardBufferFull, RAM_ONLY },
{ 0x18, CAT_waitForKeypadLock, RAM_ONLY },
{ 0x1A, CAT_waitTime, RAM_ONLY },
{ 0x1B, CAT_GetRamExecutionState, LINK_ONLY },
{ 0x1C, CAT_clearKeyBuffer, BOTH },
{ 0x1E, CAT_clearPowerFlag, BOTH },
{ 0x1F, CAT_returnCardStatus, BOTH },
{ 0x20, CAT_reset, BOTH },
{ 0x21, CAT_get16BitStatus, LINK_ONLY },
{ 0x22, CAT_defineKeypad, RAM_ONLY },
{ 0x23, CAT_setBeepDuration, RAM_ONLY },
{ 0x28, CAT_displayMultiple, BOTH },
{ 0x29, CAT_waitForCardBufferFullPlus, RAM_ONLY },
{ 0x2E, CAT_setGraphicsMode, BOTH },
{ 0x2F, CAT_unsetGraphicsMode, BOTH },
{ 0x30, CAT_initializeBitmapFrame, BOTH },
{ 0x31, CAT_downloadCompressedFrame, BOTH },
{ 0x32, CAT_displayROMFrame, BOTH },
{ 0x37, CAT_printGraphics, BOTH },
{ 0x39, CAT_get24BitStatus, LINK_ONLY },
{ 0x3A, CAT_get40BitStatus, LINK_ONLY },          // new function, not in manual

{ 0x65, CAT_defineVerifoneFrame, BOTH },
{ 0x66, CAT_downloadVerifoneFrame, BOTH },
{ 0x67, CAT_printFlashGraphics, BOTH },
////////////////////////////////////////////////////////////
{ 0x70, CAT_runClip, BOTH },
{ 0x71, CAT_loadClip, LINK_ONLY },
{ 0x72, CAT_runShortClip, BOTH },
////////////////////////////////////////////////////////////
//
// E   M   V
//
{ 0x73, CAT_emvStart, LINK_ONLY },
{ 0x74, CAT_emvTagData, LINK_ONLY },
{ 0x75, CAT_getEmvReturnCode, LINK_ONLY },
{ 0x76, CAT_emvContinue, LINK_ONLY },
{ 0x77, CAT_setEmvTagData, LINK_ONLY },
{ 0x78, CAT_emvFinalize, LINK_ONLY },
////////////////////////////////////////////////////////////
{ 0x80, CAT_messageBox, BOTH },
{ 0x81, CAT_fullMessageBox, BOTH },
{ 0x82, CAT_displayMultipleFrames, BOTH },
{ 0x83, CAT_getBoardConfiguration, LINK_ONLY },
{ 0x84, CAT_setTimeDate, LINK_ONLY },          // BCD: HH,MM,SS,DD,MM,YY
{ 0x85, CAT_getRFID, LINK_ONLY },              // returns 16 bytes of data read from RFID reader (microRWD)
////////////////////////////////////////////////////////////
{ 0x86, CAT_injectKey, LINK_ONLY },               // to inject a key from PC (for testing purposes)
////////////////////////////////////////////////////////////
{ 0xFE, CAT_setMulticast, LINK_ONLY } };
/**
 * This is the primary definition of the 'RAM' of the CAT
 * According to the definition, each block should not be
 * of more than 240 bytes...
 */
#define MAX_RAM_SLOTS               (256+1) // the extra slot is used for internal
// ram execution during, e.g. wait for funcs
typedef struct
{
    uint8_t *data;
    uint16_t length;
    uint16_t belongsToCmd;                  // which command # owns this data?
    uint16_t currentInx;                    // execution index inside data
} RAM_t;

/**
 * CAT error codes
 */
#define CAT_RX_OVERFLOW             0x01
#define CAT_NOT_CONNECTED           0x02
#define CAT_SELFTEST_FAIL           0x04
#define CAT_PROTOCOL_CRC_ERR        0x08
#define CAT_PROTOCOL_MULTICAST      0x10
#define CAT_PROTOCOL_RECEIVED_OK    0x20
#define CAT_PROTOCOL_OK             0x00
#define CAT_GENERAL_ERROR           0x03

#define CAT_TIMEOUT                 1000            // in milliseconds

#define FRAME_FILENAME              "frm"
#define FRAME_EXTENSION             "dat"
#define ROM_FILENAME                "rom"
#define ROM_EXTENSION               "bmp"

#define KEYPAD_BUF_SIZE             64
#define CAT_RX_BUFSIZE              364             // was 64????

#define NO_COMM_FIRST_TIME  0xff
#define NO_COMM_TIMEOUT_SECS        20

/**
 * Static Variables
 */
static enumCAT rxstate;
char rxBuffer[CAT_RX_BUFSIZE] __attribute__ ((section(".ccm")));
char txBuffer[CAT_TX_BUFFER_SZ] __attribute__ ((section(".ccm")));
RAM_t ramData[MAX_RAM_SLOTS] __attribute__ ((section(".ccm")));
uint8_t saveRamLoc[12] __attribute__ ((section(".ccm")));
uint8_t keypadBuffer[KEYPAD_BUF_SIZE] __attribute__ ((section(".ccm")));
uint8_t magstripe_stack[512 * 2 + 256] __attribute__ ((section(".ccm")));
uint8_t clip_stack[3048] __attribute__ ((section(".ccm")));
uint8_t eth_stack[2548] __attribute__ ((section(".ccm")));
uint8_t checksum_stack[800 + 256] __attribute__ ((section(".ccm")));
uint8_t displaymb_stack[800 + 256] __attribute__ ((section(".ccm")));
uint8_t microrwd_stack[800 + 256]  __attribute__ ((section(".ccm")));

int catstate,     nextCatState      __attribute__ ((section(".ccm")));
int r_catstate                      __attribute__ ((section(".ccm")));

static uint16_t crc, calcCRC, rxError;
static uint8_t /*shouldReply*/isMulticast, myAddress, DLEreceived,
        powerFailFlag, runningFromRAM, stopShortClip, noCommCounter;
static uint8_t cardRetryLimitReached, cardMaxRetries, cardIgnore1Err,
        cardRetries, doneFileLoading, stopNCShortClip;
static uint8_t cardIgnore2Err, cardIgnore3Err, cardInsertedMsg,
        cardInsertedTimeoutReached;
static uint8_t ramSuspended = 0, ramLoc256[2];
kernel_pid_t cat_handler_pid = KERNEL_PID_UNDEF, clip_pid = KERNEL_PID_UNDEF;
static ringbuffer_t tx_ringbuf, cmd_ringbuf, keypad_ringbuf;
static mutex_t cmd_mutex = MUTEX_INIT, sync_display = MUTEX_INIT, sync_state = MUTEX_INIT;
static uint32_t ramLoc, beepDuration;
//! FAT FS pointer
static FATFS *FatFs;
static DIR *dj; /* Directory search object */
static FIL *rd; /*< file handle */
static uint32_t diskerror, shortClip_offset, *shortClip_pInx,
        lastReportedStatus;
static emvCard *emv;
static int emvReturnCode;
static uint8_t nc_frameNum;                 // no comm frame number and first time designator
static uint8_t menuStatus;

//static uint8_t emvRunning;
static cardReader_cb_TypeDef *card;         // card reader call back functions

/**
 * HEAP MEMORY USED FOR FRAMES AND DECOMPERESSION
 */
uint8_t _HeapMem[1024 * 20] __attribute__((section(".HeapMemSection")));

#if 0
int test_sendNext( void)
{
    swuart_sendByte( 0, 0xAA);
}
static int cbaaa = 0;
void test_rxByte( uint8_t b)
{
    if( b != 0x31)
    {
        cbaaa++;
    }
}
#endif

/**********************************************/
/*  P U B L I C     F U N C T I O N S         */
/**********************************************/
void CAT_main( void)
{
    //msg_t m;
    uint16_t cnt;
    int ret/*, cmd*/;
    uint written;
    diskerror = 0;

#define DEBUG_GUI_SCREEN
#ifdef DEBUG_GUI_SCREEN
    SDRAM_Init();

    if( CAT_mountDisk() != FR_OK)
        diskerror = 1;

    display_main(&CAT_keyPressed, &sync_display);
#endif

    // for the console
    display_fill( 0, C_WHITE);
    display_showConsole();

    char buf[64];
    sprintf( buf, "Starting Telak CAT v%02d.%02d\n", MAJOR_VERSION, MINOR_VERSION);
    display_consoleWrite( buf);

    uint32_t cpuid = *(uint32_t *)(0xE000ED00);
    char cpuname[4], part[12];
    if(( cpuid & 0xFF000000) == 0x41000000)
        strcpy( cpuname, "ARM");
    else
        strcpy( cpuname, "-");
    int var = ( cpuid & 0x00F00000) >> 20;
    int pt = ( cpuid & 0x0000FFF0) >> 4;
    if( pt == 0xC24)
       strcpy( part, "Cortex-M4");
    else if( pt == 0xC27)
       strcpy( part, "Cortex-M7");
    else
       strcpy( part, "unknown");

    int pat = ( cpuid & 0x0000000F);
    display_consoleWrite( "You are running ROS on a WaveShare board.\n");
    sprintf( buf, "CPU id: %s %s rev %Xp%X\n", cpuname, part, var, pat);
    display_consoleWrite( buf);

    display_consoleWrite( "RTC Init...");
    rtcbkp_init();
    display_consoleWriteColor( "OK\n", C_LAWN_GREEN);
    display_consoleWrite( "CAT initialize\n");
    CAT_initialize();
#if 0
    rtcbkp_init();

    RTC_TimeTypeDef RTC_TimeStructure;
    RTC_DateTypeDef RTC_DateStructure;
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

#endif

    {
        //RCC_PCLK2Config( RCC_HCLK_Div1);
        RCC_ClocksTypeDef RCC_Clocks;
        RCC_GetClocksFreq(&RCC_Clocks);
    }
    emv = NULL;
#if 0
    struct tm _tm;
    timex_get_localtime( &_tm);
#endif
    /*
     if( CAT_checkIfMagStripeConnected())
     card = magStripe_getCallBacks();
     else
     */

//////////////////////////
#define DEBUG_MT188
//////////////////////////

    display_consoleWrite( "Card Reader initialization...");

#ifndef  DEBUG_MT188
    card = creator_getCallBacks();
#else
    // check creator, if OK, take it, if not creator, grab mt188
    card = creator_getCallBacks();
    if( card == NULL)
        card = mt188_getCallBacks();

    display_consoleWriteColor( (const char *)card->getModel(), C_YELLOW);
    display_consoleWrite( "\n");
#endif

    display_consoleWrite( "LED Init\n");

    gpio_init_out( LED_1, GPIO_PULLDOWN);
    gpio_init_out( LED_2, GPIO_PULLDOWN);
    gpio_init_out( LED_3, GPIO_PULLDOWN);
    gpio_init_out( LED_4, GPIO_PULLDOWN);
    gpio_clear( LED_1);
    gpio_set( LED_2);
    gpio_clear( LED_3);
    gpio_set( LED_4);

    // restart the Flash Disk (SD Card)
#if 1
    // create the ashProtocol object
    ashProtocol::create();
#endif

#ifndef DEBUG_GUI_SCREEN
    SDRAM_Init();
#endif

    display_consoleWrite( "Trying to mount SD card...");


#if 1 // can't do that now because I2C3 is taken ??!?!?!?!?!?
/*
    if( CAT_mountDisk() != FR_OK)
        diskerror = 1;

    else
    */
    if( diskerror == 0)
    {
//////////////////
#ifdef TEST_FSEEK
// testing 123
        FRESULT fr = f_open_adj( rd, "video_2.avi", FA_READ);
        DWORD clmt[50];
        //! prepare the cluster table for FAST_SEEK
        /*! Using fast seek feature */
        rd->cltbl = clmt; /* Enable fast seek feature (cltbl != NULL) */
        clmt[0] = sizeof( clmt) / sizeof( DWORD); /* Set table size */
        fr = f_lseek( rd, CREATE_LINKMAP); /* Create CLMT */
#endif
//////////////////

        if( CAT_findFile(( char *)"ssetup.xml") == FR_NO_FILE)
        {
            f_open(rd, "ssetup.xml", FA_CREATE_NEW | FA_READ | FA_WRITE);
            f_write(rd, "[DEVICE_ADDRESS=1]\r\n", 20, &written);
            f_close(rd);
        }
        else
            ashProtocol::init("ssetup.xml");

        display_consoleWriteColor( "OK\n", C_LAWN_GREEN);

        int inx = ashProtocol::indexByKey("DEVICE_ADDRESS", NULL);
        if( inx >= 0)
        {
            const char *addr;
            ashProtocol::getParam(inx, "DEVICE_ADDRESS", &addr);
            myAddress = atoi( addr);
            display_consoleWriteColor( "CAT address is ", C_LAWN_GREEN);
            display_consoleWriteColor( addr, C_LAWN_GREEN);
            display_consoleWrite( "\n");
        }
        else
            display_consoleWriteColor( "could not read CAT address. Default to 1\n", C_RED);

    }
    else
        display_consoleWriteColor( " *** FAIL ***\n", C_RED);


    display_setPumpNumber( myAddress);
#endif

    display_consoleWrite( "Starting Card Reader Thread...\n");

#if 1
//#define DEBUG_MT188
#ifdef USE_CREATOR_READER
    thread_create(( char *)magstripe_stack, ( int)sizeof(magstripe_stack),
    PRIORITY_MAIN /* was - 1 NS 02-10 */, CREATE_STACKTEST,
            ( thread_task_func_t)card->Init, ( void *)NULL, "creator_thread");
    puts("swipe_thread created");
#endif
#endif
    // initialize display & touch screen tests

#if 0// debugging issues with sdram
    display_main(&CAT_keyPressed, &sync_display);
#endif

    if( diskerror)
    {
        display_setupMessageBox("Error", "SD Card Mount Error", 2);
        display_consoleWrite( "SD Card Mount Error!\n");
    }


#ifdef PLAY_WAVE
    wave_playback( "audio.wav");
#endif

#if 1 // in collision with SPI1-CLK --- change the SPI1
    CAT_soundBuzzer(100);
    //xtimer_usleep( 300000);
    CAT_soundBuzzer(100);
#endif
    /* Init Touch */
#ifdef USE_SWIPE
    thread_create( magstripe_stack, sizeof(magstripe_stack),
            PRIORITY_MAIN + 1, CREATE_STACKTEST,
            magStripe_main, NULL, "swipe_thread");
    puts("swipe_thread created");
#endif


    //emvParamsTable tbl = emvParamsTable();
    {
        //ashProtocol ash = ashProtocol();
        ashProtocol::init("pinpad.xml");
        ashProtocol::init("strings.xml");
    }
    // test

    //////////////////////////////////////////////////////////////////////////////
#ifdef USE_ETHERNET // changed to very high priority

    ETH_Thread_Arg arg;

    int inx = ashProtocol::indexByKey("DEFAULT_HOST_IP", NULL);
    const char *addrStr = "0.0.0.0";
    if( inx >= 0)
    {
        ashProtocol::getParam(inx, "DEFAULT_HOST_IP", &addrStr);
    }
    inx = ashProtocol::indexByKey("DEFAULT_HOST_NAME", NULL);
    arg.catHostName = "ANDROID-1C73BAD";
    if( inx >= 0)
        ashProtocol::getParam(inx, "DEFAULT_HOST_NAME", &arg.catHostName);

    arg.cat_recv_cb = CAT_recv;
    arg.default_ip.addr = ipaddr_addr( addrStr /* "192.168.2.1" */);

    display_consoleWrite( "Starting Ethernet Thread...");
    display_consoleWriteColor( "Host IP: ", C_YELLOW);
    display_consoleWriteColor( addrStr, C_YELLOW);
    display_consoleWrite( "\n");
    display_setHostIP( addrStr);



    thread_create(( char *)eth_stack, ( int)sizeof(eth_stack),
                     PRIORITY_MAIN - 4 /* was - 2 *//* worked with + 1 */, CREATE_STACKTEST,
                      ( thread_task_func_t)ethernet_thread, ( void *)&arg, "eth_thread");
    puts("ethernet thread created");
#endif

    display_consoleWrite( "Printer Initialization...");

    printer_init();

    cyclog_init(1024*2);

    // HAVE ERROR?
    if( printer_getError() != 0)
    {
        gpio_set( LED_3);
        //display_consoleWrite( "Printer Init Error!\n");
        display_consoleWriteColor( "Init Error!\n", C_RED);
    }
    else
        display_consoleWriteColor( "OK\n", C_LAWN_GREEN);

    //printer_testGraphics();

#ifndef USE_ETHERNET
    uart_init( CAT_COMM_UART, CAT_BAUDRATE, &CAT_rxbyte, &CAT_txbyte, 0);
#endif

    cat_handler_pid = thread_getpid();

    ///////////////////////////////////////////
    // testing GT811 touch screen controller
    ///////////////////////////////////////////
    display_consoleWrite( "GT811 Touch Controller Init...");
    if( GT811_Init() == 0)
    {
        display_setupMessageBox("Error", "GT811 Touch Controller not found!", 2);
        display_consoleWriteColor( "GT811 Error!\n", C_RED);
    }
    else
        display_consoleWriteColor( "OK\n", C_LAWN_GREEN);


    // restart state of no comm clip
    CAT_restartNoCommClip();

    int countEpp = 0;
    int eppStatus;

    display_consoleWrite( "Starting EPP interface...");

    while( countEpp < 3 && epp_isSmartEpp())
    {
        /* Load the EPP module */
        eppStatus = epp_init();
        if( eppStatus == EPP_OK)
        {
            display_consoleWriteColor( "Smart EPP Found\n", C_LAWN_GREEN);
            break;
        }
        xtimer_usleep( 500000);
        countEpp++;
    }
    if( !epp_isSmartEpp())
    {
        /* Load the EPP module */
        display_consoleWriteColor( "Trying to Init SW3501B\n", C_LAWN_GREEN);
        eppStatus = epp_init();
    }



//#define DEBUG_SDRAM
#ifdef DEBUG_SDRAM
    SDRAM_Init();
    {
        uint8_t g = 0;
        for( uint32_t ii = 0xd0100000; ii < 0xd0200000; ii += 4)
        {
            *( uint32_t *)ii = ( 0xaa00bb00 | g);
            g++;
        }

        while( true)
        {
            int sdram_error = 0;
            uint8_t g = 0;
            for( uint32_t ii = 0xd0100000; ii < 0xd0200000; ii += 4)
            {
                if( *( uint32_t *)ii != ( 0xaa00bb00 | g))
                    sdram_error++;
                g++;
            }
            xtimer_usleep( 100000);
        }
    }
#endif

    if( epp_isSmartEpp())
    {
        if( eppStatus == EPP_NO_KEYS)
            display_setupMessageBox("Error", "EPP Keys Do Not Match!", 2);
        else if( eppStatus != EPP_OK)
            display_setupMessageBox("Error", "EPP Was Not Detected!", 2);
    }

#ifdef DEBUG_CAT

#if 0
    display_showText( "Select App. Remove Card to Cancel", 33, 0);
    const char *selName[4] = { "1. visa1", "2. visa2", "3. visa3", "4. Mastercard" };
    int appcnt = 4;
    ret = display_windowWithButtons( "Select App. Remove Card to Cancel", appcnt, (uint32_t *)selName);
#endif

    display_consoleWriteColor( "Starting CAT application\n", C_WHITE);

//#define DEBUG_CLIP
#ifdef DEBUG_CLIP
    //debug
    CAT_runClipThread( "video_2.avi");
    //display_dataGui();
    //display_window3();
    while( true)
    {
        display_periodic();
        xtimer_usleep( 100000);
    }
#endif



    // printf all threads
    puts("\nSTART APP\n");
#if 0
   thread_print_all();
   emvsw_callbacks *esc = emvGenerate( true);
   thread_print_all();
#endif

   // just to make sure to everybody that the app is working
   gpio_set( LED_1);


#if 0
    xtimer_usleep( 1000000);
    xtimer_usleep( 1000000);
    xtimer_usleep( 1000000);
    xtimer_usleep( 1000000);
    xtimer_usleep( 1000000);
    xtimer_usleep( 1000000);
    xtimer_usleep( 1000000);
    xtimer_usleep( 1000000);
    xtimer_usleep( 1000000);
    xtimer_usleep( 1000000);

    ethdemo_getfile( "ffmpeg.exe", "ffmpeg.exe", CAT_loadClipSuccess, NULL);
    TS_StateTypeDef ts;

    while( true)
    {
        GT811_GetState( &ts);
        xtimer_usleep( 1000000);
    }
#else
#if 1
    /////////////////////////
    // DEBUG DEBUG DEBUG
    //LTDC_Cmd( DISABLE);

    display_consoleWrite( "SW UART Initialization\n");

    swuart_init();
    //swuart_create( 9600, GPIO_7, GPIO_8, test_rxByte, test_sendNext);

    display_consoleWrite( "Starting microRWD Thread...");
    //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // microRwd HiTag 2 RFID reader/writer
    thread_create(( char *)microrwd_stack, ( int)sizeof(microrwd_stack),
                    PRIORITY_MAIN + 1, CREATE_STACKTEST, ( thread_task_func_t)microRWD_thread, ( void *)NULL, "mrwd_thread");
    puts("microRWD thread created");

    int timeout = 30;
    while( !microRWD_isInitialized())
    {
        xtimer_usleep( 100000);
        timeout--;
        if( timeout == 0) break;
    }
    if( microRWD_getError() == RWD_TIMEOUT || timeout == 0)
        display_consoleWriteColor( "** FAIL **\n", C_RED);
    else
        display_consoleWriteColor( "OK\n", C_LAWN_GREEN);

    // have local IP from DHCP server?
    timeout = 50;
    while( timeout > 0)
    {
        char *lll = ethernet_getLocalIP();
        if( lll != NULL)
        {
            display_consoleWriteColor( "DHCP OK, Local IP: ", C_LAWN_GREEN);
            display_consoleWriteColor( lll, C_BLUE);
            display_consoleWriteColor( "\n", C_BLUE);
            break;
        }
        timeout--;
        xtimer_usleep( 100000);
    }

#endif

#if 0
    TS_StateTypeDef ts;

    while( true)
    {
        uint frameNum = 0, counter = 0, __show = 0, seccounter = 0;
        while( true)
        {
            if( __show == 0)
            CAT_showShortClip( "vid997.avi");
            else
            CAT_showShortClip( "vid998.avi");

            AVISTREAMHEADER *avihead = ( AVISTREAMHEADER *)riffParser_GetStreamHeader( 0);
            while( counter < 20)
            {
                display_shortClip( frameNum++, shortClip_pInx, shortClip_offset, (counter==0));
                if( avihead->dwLength / avihead->dwScale <= frameNum)
                frameNum = 0;
                xtimer_usleep( 125000);
                counter++;
            }
            display_free( ( void *)shortClip_offset);
            free( shortClip_pInx);
            shortClip_pInx = NULL;
            shortClip_offset = NULL;
            stopShortClip = 0;
            display_endShortClip();

            counter = 0;
            __show ^= 1;
            seccounter++;
            if( seccounter == 2)
            break;
        }
        char txt[32];
        CAT_runClipThread( "video_2.avi");
        while( thread_getstatus( clip_pid) != STATUS_NOT_FOUND)
        {
            xtimer_usleep( /*500000*/8000);

            GT811_GetState( &ts);

            sprintf( txt, "X:%03d Y:%03d [%d]", ts.touchX[0], ts.touchY[0], ts.touchDetected);
            display_updateTextW3( txt, 0);

            swuart_sendByte( 0, 0xA5);

            //! not working??? UG_Update();
        }
    }
#endif
#endif
#endif // DEBUG_CAT

    catstate = r_catstate = -1;

//#define DEBUG_INJECT
#ifdef DEBUG_INJECT
//    CAT_inject( "\xfe\x01\x12\x11\x30\x00\x00\x90\x00\x00\x01\x00\x30\x00\x02\x91\x00\x01\x00\x07\x37\x01\x01\x08\x38\x01\x02\x09\x39\x02\x00\x04\x34\x02\x01\x05\x35\x02\x02\x06\x36\x03\x00\x01\x31\x03\x01\x02\x32\x03\x02\x03\x33\xfd\x98\x67", 56);
//    CAT_inject( "\xfe\x01\x12\x12\x19\x0a\x00\xf1\x00\x02\xf3\x00\x03\x00\x04\x01\x03\x01\x04\x02\x03\x02\x04\x03\x03\x03\x04\xf4\x00\x00\xfd\x22\x58", 31 + 2);
//    CAT_inject( "\xfe\x01\x22\x11\xfd\x61\xf3", 5 + 2);
//    CAT_inject( "\xfe\x01\x01\x12\xfd\x01\x01", 5 + 2);

//    CAT_inject("\xfe\x01\x32\x01\xfd\x00\x00", 5 + 2); // display ROM frame #1
#if 0
    CAT_inject("\xfe\x01\x31\x01\xfb\x10\x00\x00\x09\x10\xfb\x00\xfb\x02\x05\x10\xfb\x41\xfd\x00\x00", 19 + 2); // bitmap compressed

    CAT_inject( "\xfe\x01\x12\x11\x0a\x29\x13\x14\xd0\x07\x30\x75\x13\xd0\x07\xfd\x00\x00", 16 + 2);// command 0x29
    CAT_inject( "\xfe\x01\x05\x11\xfd\x00\x00", 5 + 2);// execute macro

    CAT_inject( "\xfe\x01\x1B\xfd\x00\x00", 4 + 2);// get ram execution state
#endif
    /*
     CAT_inject("\xfe\x01\x12\x20\x10Nimrod Stoler123\xfd\x00\x74", 22 + 2); // retry message
     CAT_inject("\xfe\x01\x12\x11\x03\x17\x20\x12\xfd\x00\x00", 9 + 2); // command 0x29
     CAT_inject("\xfe\x01\x05\x11\xfd\x00\x00", 5 + 2);          // execute macro
     */
#if 0
    CAT_inject( "\xfe\x01\x12\x11\x0a\x29\x13\x14\xd0\x07\x30\x75\x13\xd0\x07\xfd\x00\x00", 16 + 2); // command 0x29
    CAT_inject( "\xfe\x01\x05\x11\xfd\x00\x00", 5 + 2);// execute macro
#endif

#if 0
    CAT_inject( "\xfe\x01\x0f\xfd\xa4\x7d", 4 + 2);       // get eightbit status
    CAT_inject( "\xfe\x01\x12\x14\x02\x08\x10\xfd\xa1\x49", 8 + 2);// setup print macro
    CAT_inject( "\xfe\x01\x12\x10\x13\x30\x31\x11\x32\x33\x34\x12GHI\x10\xb0\xb1\xb2\x09\xb0\xb1\xb2\x1f\xfd\xcd\x61", 25 + 2);// setup print macro
    CAT_inject( "\xfe\x01\x05\x14\xfd\xa4\x7d", 5 + 2);// execute macro
    CAT_inject( "\xfe\x01\x12\x11\x0a\x29\x13\x14\xd0\x07\x30\x75\x13\xd0\x07\xfd\x00\x00", 16 + 2);
    CAT_inject( "\xfe\x01\x05\x11\xfd\x00\x00", 5 + 2);// execute macro
    CAT_inject( "\xfe\x01\x04\x00\x10Nimrod Stoler123\xfd\x00\x74", 22 + 2);
    CAT_inject( "\xfe\x01\x18\xfd\xab\x8d", 4 + 2);// wait for keypad lock
    CAT_inject( "\xfe\x01\x0e\xfd\xab\x8d", 4 + 2);// get keypad buffer
#endif
    //CAT_inject( "\xfe\x01\x31\x01\xFB\x10\x00\x00\x09\x10\xfb\x00\xfb\x02\x05\x10\xfb\x41\xfd\x34\xd7", 19 + 2);

    /* or use only system shell commands */
    shell_t sys_shell;
    shell_init(&sys_shell, NULL, 500, uart0_readc, putchar);
    shell_run(&sys_shell);
#endif

    //httpd_init( myAddress);

//#define DEBUG_EMV
#ifdef DEBUG_EMV
    while( true)
    {
        // start the emv card object
        if( creator_isCpuCardInserted())
        {
            const char *termType = NULL, *capability = NULL, *add = NULL;
            int inx = ashProtocol::indexByKey( "EMV", NULL);
            if( inx >= 0)
            {
                ashProtocol::getParam( inx, "TerminalType", &termType);
                ashProtocol::getParam( inx, "TerminalCapabilities", &capability);
                ashProtocol::getParam( inx, "AddTerminalCapabilities", &add);
            }

            thread_print_all();

            emv = new emvCard();
            emvReturnCode = emv->start( termType, capability, add);
            /*
             thread_print_all();
             delete emv;
             thread_print_all();
             */
            emvReturnCode = emv->select( 0);

            thread_print_all();

            delete emv;

            thread_print_all();

        }
        xtimer_usleep( 500000);
    } // endwhile forever debug
#endif // DEBUG_EMV

int faster = 0;
uint32_t lastxtimer = 0;

    noCommCounter = 0;


#define DEBUG_RUN
    while( 1)
    {
        // sync display init with CAT loop
        mutex_lock(&sync_display);

        CAT_buildStatusString();
        if( display_periodic( /*NULL*/) == MENU_REQUEST && catstate == 0x72 /* run short clip */)
        {
            // ???????
        }
        if( display_isMainMenuShown())
            menuStatus = 1;
        else
            menuStatus = 0;
        //-----------------------------------------------------
/*
        if( faster == 0)
            noCommCounter++;                        // comm simple watch dog counter
*/
        if( xtimer_now() - lastxtimer > 250000 && menuStatus == 0)
        {
            if( noCommCounter < 240)
                noCommCounter++;
            lastxtimer = xtimer_now();
        }
        if( noCommCounter == NO_COMM_TIMEOUT_SECS * 4 /* 20 seconds for no comm watchdog */)
            CAT_stopShortClip();            // this requests to stop the "normal" short clips only

        if( noCommCounter > NO_COMM_TIMEOUT_SECS * 4 && noCommCounter <= 240)
        {
            if(( shortClip_offset == NULL || nc_frameNum != NO_COMM_FIRST_TIME)
                  && thread_getstatus( clip_pid) == STATUS_NOT_FOUND)
            {
                CAT_showNoCommClip();
                noCommCounter = 240;
            }
        }
        //
        // a new try with the interrupt responding
        mutex_lock( &sync_state);
        if( r_catstate != -1)
        {
            if( catstate == -1)
            {
                catstate = r_catstate;
            }
            else if( nextCatState == -1 && catstate != r_catstate)
            {   // give priority to the new entering command
                nextCatState = catstate;
                catstate = r_catstate;
            }
            else
                r_catstate = 0;
            r_catstate = -1;
        }
        mutex_unlock( &sync_state);

#if 0
        //-----------------------------------------------------
        // start of copied block from isr
        if( ringbuffer_peek_one(&cmd_ringbuf) != -1)
        {
            /*****************************
             * execute command from LINK *
             * received during interrupt *
             *****************************/
            if( catstate >= 0)
                ramSuspended = 1;        //! suspended RAM execution because of LINK cmd
                                         //! cnt should be positive...

            //! Do we have something pending in the rx buffer?
            uint8_t cmd = ringbuffer_peek_one(&cmd_ringbuf);
            if( cmd == 254)
                while( 1)
                    ;
            cnt = 1;
            int ret = CAT_commandExecute(cmd, &cnt, !isMulticast);
            REMOVE_FROM_RX(cnt)
#if 0
            //DEBUG
            if( ringbuffer_get_free(&cmd_ringbuf) != CAT_RX_BUFSIZE)
            {
                ringbuffer_remove(&cmd_ringbuf, CAT_RX_BUFSIZE);
                while( 1)
                    ;
            }
#endif
            //END DEBUG

            ramSuspended = 0;                     // RAM execution not suspended
            switch( ret)
            {
            case COMMAND_NOT_FOUND:                 //! ERROR, don't know what to do???
                break;

            case I_WILL_DEAL_WITH_RETURN:
                break;

            // set the catstate in the foreground (non=isr) ONLY!
            case IN_COMMAND_EXECUTION:
                if( catstate == -1) // we have currently only one level of catstate, sorry
                    catstate = cmd;
                else if( nextCatState == -1)
                { // 25-10 give priority to the new command entering...
                    nextCatState = catstate;
                    catstate = cmd;
                }
                else
                {
                }
                break;

            default:                                //!returned a positive number
                if( !isMulticast)
                {
                    uint8_t data[2];
                    data[0] = cmd;
                    data[1] = ret;
                    CAT_prepareTxDataAndSend(data, 2);
                }
                break;
            } // endswitch
        } // endof block copied from isr

        /**if no new commands on the LINK check if should run RAM or other**/
        else
#endif
        if( catstate >= 0 && !ramSuspended)
        {
            cnt = 0;
            ret = CAT_commandExecute( catstate, &cnt, !isMulticast);
            if( ret == COMMAND_NOT_FOUND)
                ret = NEXT_COMMANDS_ABORT;

            if( ret == IN_COMMAND_EXECUTION)
            {
                // sync display init with CAT loop
                mutex_unlock(&sync_display);
                xtimer_usleep(25000);              // no reason to be in a hurry
                faster = 1;
                continue;
            }
            else if( ret == NEXT_COMMANDS_ABORT || ret == FINISHED_EXECUTING_RAM) // abort ALL
            { //! reset all executeRAM variables
                catstate = -1;
                runningFromRAM = 0;
                for( int i = 0; i < MAX_RAM_SLOTS; i++)
                    ramData[i].currentInx = 0;

                if( nextCatState != -1)
                {
                    catstate = nextCatState;
                    nextCatState = -1;
                }
                //
                // sync display init with CAT loop
                mutex_unlock( &sync_display);
                faster = 1;
                continue;
            }
            else // if ends command execution
            {   // in case a function received from LINK stopped a previous
                // RAM execution command, continue to that command once the
                // command ended
                if( runningFromRAM > 0)
                {
                    catstate = 5;          // next state is running from RAM

                    // sync display init with CAT loop
                    mutex_unlock(&sync_display);

                    faster = 1;
                    continue;
                }
            }
        } // endif if in running state
        else if( nextCatState != -1 && catstate == -1)
        {
            catstate = nextCatState;
            nextCatState = -1;
        }
        // sync display init with CAT loop
        mutex_unlock(&sync_display);


        // if have more data in rx
        if( ringbuffer_get_free(&cmd_ringbuf) != CAT_RX_BUFSIZE && !ramSuspended)
        {
            //ringbuffer_remove(&cmd_ringbuf, CAT_RX_BUFSIZE);
            //while( 1)

            faster = 1;
            xtimer_usleep(100000U);
            continue; // without waiting for the next command
        }


#ifdef DEBUG_RUN
        faster = 0;
        xtimer_usleep(100000U);
#endif

#if 0
        if( doneFileLoading)
        {
            // Copy new file ONLY if not showing LONG clip now
            if( thread_getstatus( clip_pid) == STATUS_NOT_FOUND)
            {
                CAT_renameClip();
                doneFileLoading = false;
            }
        }
#endif

    } // endwhile forever

} // end thread

/**
 * @brief initializes the SD card disk
 * @return FRESULT (FR_OK is good, others are bad)
 */
static FRESULT CAT_mountDisk( void)
{
    FRESULT fr; /* Return value */

    //! can't have FatFs on CCM (DMA'ing to it from SD card)
    FatFs = ( FATFS *)malloc(sizeof(FATFS));
    dj = ( DIR *)malloc(sizeof(DIR));
    rd = ( FIL *)malloc(sizeof(FIL));
    if( rd != NULL) memset( rd, 0, sizeof(FIL));

    //! Mount drive with adjusting clk speed
    FRESULT f = f_mount_adj(FatFs, "", 1);
    if( f != FR_OK)
    {
        free( FatFs);
        free( dj);
        free( rd);
        FatFs = NULL;
        dj = NULL;
        rd = NULL;
    }
    return f;
} // endfunc mount disk
#if 0
/** must prepare lfn prior to working with these
 *  functions... funny!
 */
FILINFO fno, fno2; /* File information */
#if _USE_LFN
char lfn[_MAX_LFN + 1];
fno.lfname = lfn;
fno.lfsize = sizeof lfn;
#endif //_USE_LFN
#endif

/**
 * @brief looks for a specific file name in root directory
 * @param filename - can use wildcards (e.g. v*.avi)
 * @return FR_EXIST if such filename exists
 *         FR_NO_FILE if no such filename exists in the root directory
 *         other FRESULT in case of error
 */
FRESULT CAT_findFile( char *filename)
{
    FILINFO fno; /* File information */

    FRESULT fr = f_findfirst(dj, &fno, "", filename);
    if( fno.fname[0] && fr == FR_OK)
        return FR_EXIST;
    else if( fr == FR_OK)
        return FR_NO_FILE;
    else
        return fr;
} // endfunc

/**
 * @brief This function finds and executes the commands sent from master or
 *        previously written in RAM
 * @param cmd - command number
 * @param cnt - number of bytes of command
 * @param shouldReply
 * @return the next command to execute
 */
int CAT_commandExecute( int cmd, uint16_t *cnt, uint8_t shouldReply)
{
    //locate the command by the number
    uint32_t found = 0, pp;

    for( pp = 0; pp < sizeof(commands) / sizeof(CATcmds_t); pp++)
    {
        if( commands[pp].cmdnum == cmd)
        {
            found = 1;
            break;
        }
    } // endfor scan all commands

    // if not found the correct command - return panic!!!!
    if( found == 0)
    {
        *cnt = CAT_RX_BUFSIZE;
        return COMMAND_NOT_FOUND;
    }

    int ret = commands[pp].cmd_cb(cnt, shouldReply);
    return ret;
} // endfunc

/**
 * @brief This function goes through the DATA ringbuffer in order to fulfill
 *        all the commands of the master or RAM as implemented in the CAT protocol.
 *
 * @param myAddress - own address
 * @param frame - pointer to buffer to hold the incoming data (allocate max
 *                250 bytes)
 * @param count - pointer to uint8_t to receive the total count of bytes received
 * @return CAT_PROTOCOL_OK
 *         CAT_PROTOCOL_MULTICAST
 *         CAT_PROTOCOL_CRC_ERR
 *         CAT_GENERAL_ERROR
 *
 *
 */
//int CAT_getFrame( uint8_t myAddress, uint8_t *frame, uint8_t *count)

/**
 * @brief Called from ethernet interface with data and length with received data
 *
 * @param data
 * @param len
 */
void CAT_recv( uint8_t *data, uint len)
{
    for( uint x = 0; x < len; x++)
        CAT_rxbyte( NULL, ( char)data[x]);

    // testing 123
    //CAT_prepareTxDataAndSend( (uint8_t *)"\x11\x12\x13", 3);
    // end testing
}

/**********************************************/
/*    P R I V A T E     F U N C T I O N S     */
/**********************************************/
/**
 * @brief Receive a new character from the UART and put it into the receive buffer
 */
/**
 * @brief This function reads from STDIO (?) all incoming characters
 *        and implements the CAT protocol.
 *
 * @return CAT_PROTOCOL_OK
 *         CAT_PROTOCOL_MULTICAST
 *         CAT_PROTOCOL_CRC_ERR
 *         CAT_GENERAL_ERROR
 *
 *
 */
static void CAT_rxbyte( void *arg, char _ch)
{
    msg_t msg;
    static uint8_t buf[CAT_RX_BUFSIZE], ch;
    static uint32_t rxInx;

    ch = ( uint8_t)_ch;

    //! if DLE received - just drop it on the floor...
    if( ch == DLE && !DLEreceived)
    {
        DLEreceived = 1;
        return;
    } // endif DLE received

    /**
     * restart state machine to receive new frame
     * or if somehow, we see a SYNC without a DLE prefix, then we should
     * restart the whole state machine, and start receiving data all over again
     */
    if( rxstate == STATE_RESTART || (!DLEreceived && ch == SYNC))
    {
        rxstate = STATE_IDLE;
        calcCRC = 0xFFFF;
        //shouldReply = 0;
        rxInx = 0;
    }
    /**
     * Calculate CRC on all incoming bytes excluding CRC bytes
     */
    if( rxstate != STATE_GET_CRC1 && rxstate != STATE_GET_CRC2)
        calcCRC = CAT_CRC16(( uint8_t)ch, calcCRC);

    switch( rxstate)
    {
    case STATE_IDLE:
        if( ch == SYNC)
            rxstate = STATE_GOT_SYNC;
        else
            rxstate = STATE_RESTART;
        break;
    case STATE_GOT_SYNC:
        if( ch == myAddress)
        {
            rxstate = STATE_MY_ADDRESS;
            //shouldReply = 1;
        }
        else if( ch == MULTICAST_ADDRESS && isMulticast)
        {
            //shouldReply = 0;
            rxstate = STATE_MY_ADDRESS;
        }
        else
            rxstate = STATE_RESTART;
        break;
    case STATE_MY_ADDRESS:
        if( ch == SF && !DLEreceived)
        {
            rxstate = STATE_GET_CRC1;
            break;
        }
        // place in temporary buffer until CRC is checked
        buf[rxInx++] = ( uint8_t)ch;

        // overflow in temp buffer...
        if( rxInx >= sizeof(buf))
        {
            rxInx--;
            rxError |= CAT_RX_OVERFLOW;
        }
        break;
    case STATE_GET_CRC1:
        crc = ch;
        rxstate = STATE_GET_CRC2;
        break;
    case STATE_GET_CRC2:
        rxstate = STATE_RESTART;
        crc |= ( uint16_t)(ch << 8);
        if( crc != calcCRC)
        { // just dump received frame
            rxError |= CAT_PROTOCOL_CRC_ERR;
        }
        else
        {   // just to be on the safe side
            DLEreceived = 0;
            noCommCounter = 0;                  // zero no comm watch dog

            CAT_stopNoCommClip();

            //! add new data to the command buffer
            mutex_lock(&cmd_mutex);
            ringbuffer_add(&cmd_ringbuf, ( char *)buf, rxInx);
            mutex_unlock(&cmd_mutex);
#ifndef USE_ETHERNET
            msg.content.value = ( uint32_t)rxInx;
            msg.type = ( uint16_t)isMulticast;
            msg_send(&msg, cat_handler_pid);
#else
            runCommandLoop(rxInx);          // disgest and send a quick response
#endif
        }
        break;
    } // endswitch

    DLEreceived = 0;

} // end of rx isr

/**
 * @brief       digest request from HOST and send a __quick__ response
 * @param[in]   cnt - number of bytes in receive buffer
 */
uint8_t lastCmd;
void runCommandLoop( uint16_t cnt)
{
#if 1
    /*****************************
     * execute command from LINK *
     *****************************/
    if( catstate >= 0)
        ramSuspended = 1;        //! suspended RAM execution because of LINK cmd
                                 //! cnt should be positive...

    //! Do we have something pending in the rx buffer?
    uint8_t cmd = ringbuffer_peek_one(&cmd_ringbuf);
    //////////////////
    lastCmd = cmd;
    //////////////////
    if( cmd == 254)
        while( 1)
            ;

    int ret = CAT_commandExecute(cmd, &cnt, !isMulticast);
    REMOVE_FROM_RX(cnt)

    //DEBUG
    if( ringbuffer_get_free(&cmd_ringbuf) != CAT_RX_BUFSIZE)
    {
        ringbuffer_remove(&cmd_ringbuf, CAT_RX_BUFSIZE);
        while( 1)
            ;
    }
    //END DEBUG

    ramSuspended = 0;                     // RAM execution not suspended
    switch( ret)
    {
    case COMMAND_NOT_FOUND:                 //! ERROR, don't know what to do???
        break;

    case I_WILL_DEAL_WITH_RETURN:
        break;

    // set the catstate in the foreground (non=isr) ONLY!
    case IN_COMMAND_EXECUTION:
        /*
        int r_catstate = catstate;
        if( r_catstate == -1) // we have currently only one level of catstate, sorry
        */
        mutex_lock( &sync_state);
        r_catstate = cmd;
        mutex_unlock( &sync_state);
        /*
        else if( set_nextCatState == -1)
        { // 25-10 give priority to the new command entering...
            set_nextCatState = r_catstate;
            set_catstate = cmd;
        }
        else
        {
        }
        */
        break;

    default:                                //!returned a positive number
        if( !isMulticast)
        {
            uint8_t data[2];
            data[0] = cmd;
            data[1] = ret;
            CAT_prepareTxDataAndSend(data, 2);
        }
        break;
    } // endswitch
#endif
} // endfunc

/**
 * @brief Called by UART ISR when Tx buffer is empty, thus ready for new byte
 *        to be sent (or sended as Roberto used to say...)
 * @param arg
 */
static int CAT_txbyte( void *arg)
{
    int ret = ringbuffer_get_one(&tx_ringbuf);
    if( ret != -1)
    {
        char data = ( char)ret;
#ifndef USE_ETHERNET
        uart_write( CAT_COMM_UART, data);
#endif
    }
    if( ringbuffer_empty(&tx_ringbuf))
        return 0;
    else
        return 1;
}

/**
 * @brief reads number of bytes from command ring buffer
 * @param data  - buffer to copy data to
 * @param length- number of bytes to copy
 * @return -1 if not enough bytes stored in ring buffer to read,
 *         otherwise the number of bytes read and copied
 *
 * @note:  This function does NOT remove the bytes from the ringbuffer
 */
unsigned CAT_readCmdRingBuf( uint8_t *data, uint8_t length)
{
    if( runningFromRAM == 0 || ramSuspended)
        return (ringbuffer_peek(&cmd_ringbuf, ( char *)data, length));
    else
    { // if running from RAM just copy the RAM locations to data
        uint32_t raminx = ramData[ramLoc].currentInx;
        memcpy(data, &ramData[ramLoc].data[raminx], length);
        return (length);
    }
}

/**
 * @brief sets the multicast mode command 0xfe
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_setMulticast( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2];

    *len = 2;
    CAT_readCmdRingBuf(data, 2);

    if( data[1] == 1)
        isMulticast = true;
    else
        isMulticast = false;

    return 0x00;
}

/**
 * @brief this is command 0x20 which should reset the entire CAT device
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_reset( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2];

    *len = 2;
    CAT_readCmdRingBuf(data, 2);

    if( data[1] != RESET_SAFETY_CHAR)
        return 0x01;
    else
    {
        data[1] = 0x00;
        if( shouldReply)
            CAT_prepareTxDataAndSend(data, 2);
        *len = 0;
        // beep a couple of times...
        //beep_on( beepDuration);
        CAT_soundBuzzer(beepDuration);
        CAT_initialize();
    }
    return I_WILL_DEAL_WITH_RETURN;
}

/**
 * @brief this command stops any currently executing RAM command
 * @param length
 * @param shouldReply
 * @return
 */
int CAT_stopCommandExecution( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2];

    *len = 1;
    CAT_readCmdRingBuf(data, 1);
    data[1] = 0x00;
    if( shouldReply)
        CAT_prepareTxDataAndSend(data, 2);
    return ( NEXT_COMMANDS_ABORT);
} // endfunc

/**
 * THIS IS FUNCTION #08
 * @brief This function returns the bytes sent to it as a software and comm test
 * @param length - length of received frame
 * @param shouldReply
 * @return
 */
static int CAT_softwareStatusTest( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[3];

    CAT_readCmdRingBuf(data, 3);
    if( shouldReply)
        CAT_prepareTxDataAndSend(data, 3 /**length*/);

    // return how many bytes this message takes in the data stream
    *length = 3;

    return I_WILL_DEAL_WITH_RETURN;
}

/**
 * @brief returns to master the current RAM location currently executing
 *        and the index within it.
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_GetRamExecutionState( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[4];
    *len = 1;

    CAT_readCmdRingBuf(data, 1);
    data[1] = 0x00;                 // no error!
    data[2] = ramLoc;
    data[3] = ramData[ramLoc].currentInx;

    if( shouldReply)
        CAT_prepareTxDataAndSend(data, 4);

    return I_WILL_DEAL_WITH_RETURN;
}

/**
 * @brief this function executes commands from RAM. It halts any current command
 *        sequence and begins executing commands specified by RAM number
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_executeRAM( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2], cmd;

    if( *len != 0)
    {
        CAT_readCmdRingBuf(data, 2);

        *len = 2;
        uint32_t _ramLoc = ( uint32_t)data[1];

        if( _ramLoc < 0x10 || _ramLoc > 0xFF || ramData[_ramLoc].data == 0
                || ramData[_ramLoc].length == 0)
            return 0x40;    //RAM reference number empty or illegal

        data[1] = 0x00;                         // return in process to master
        ///////////////////////////////////////////////////////
        // if already running inside a previous executeRAM
        if( ramLoc != 0 && runningFromRAM > 0)
        {
            if( runningFromRAM > sizeof(saveRamLoc))
                while( 1)
                    ;                      // STACK FAULT!!!
            saveRamLoc[runningFromRAM - 1] = ramLoc;
            ramData[ramLoc].currentInx += 2;    // add extra 2 bytes for 05 xx
            *len = 0;
        }
        runningFromRAM++;
        ramLoc = _ramLoc;

        if( shouldReply)
            CAT_prepareTxDataAndSend(data, 2);
        return IN_COMMAND_EXECUTION;       // so that next time it returns to me
    } // endif len != 0

    *len = 2;                                   // just to make sure it works OK

    if( ramData[ramLoc].currentInx >= ramData[ramLoc].length)
        return FINISHED_EXECUTING_RAM;

    int inx = ramData[ramLoc].currentInx;
    //! remove the ram suspension (if present) because next commands will
    //! be executed from RAM ONLY!
    ramSuspended = 0;
    int ret = CAT_commandExecute(ramData[ramLoc].data[inx], len, 0);
    if( ret != IN_COMMAND_EXECUTION)
        ramData[ramLoc].currentInx += *len;

    *len = 0;
    if( ramData[ramLoc].currentInx >= ramData[ramLoc].length)
    {
        ramData[ramLoc].currentInx = 0;
        runningFromRAM--;
        // firstly, check if have another ramloc waiting
        if( runningFromRAM > 0)
            ramLoc = saveRamLoc[runningFromRAM - 1];
        else
        {
            ramLoc = 0;
            return FINISHED_EXECUTING_RAM;
        }
    }
    return IN_COMMAND_EXECUTION;           // so that next time it returns to me

} // endfunc

/**
 * @brief Prints RAM stored text to printer, command 0x08
 * @param length - length of received frame
 * @param shouldReply
 * @return response's error code to return to POS manager (master)
 */
int CAT_printText( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2];

    CAT_readCmdRingBuf(data, 2);

    // return how many bytes this message takes in the data stream
    *len = 2;

    uint32_t loc = ( uint32_t)data[1];
    if( loc < 0x10 || loc > 0xFF || ramData[loc].data == 0
            || ramData[loc].length == 0)
        return 0x40;    //RAM reference number empty or illegal

    uint32_t ret = printer_print(( char *)ramData[loc].data,
            ramData[loc].length);
    if( ret != ramData[loc].length)
        return 0x01;    // ERROR - buffer is full
    else
        return 0x00;    // return OK if should...
} // endfunc print text

/**
 * @brief Prints RAM stored graphics to printer, command 0x37
 * @param length - length of received frame
 * @param shouldReply
 * @return response's error code to return to POS manager (master)
 */
int CAT_printGraphics( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2];

    CAT_readCmdRingBuf(data, 1);

    // return how many bytes this message takes in the data stream
    *len = 1;

    printer_printDownloadedGraphics();
    return 0x00;    // return OK if should...

}

/**
 * @brief Prints SD Card stored graphics to printer, command 0x67
 * @param length - length of received frame
 * @param shouldReply
 * @return response's error code to return to POS manager (master)
 *
 * #note  this frame is simply dropped to the printer as is
 *        compared to printer_printText, which has some translation
 *        going on. All graphics commands should be embedded inside
 *        the frame.
 */
int CAT_printFlashGraphics( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2];
    char buffer[32];
    static FIL *printer_rd = NULL;

    if( *len != 0)
    {
        CAT_readCmdRingBuf(data, 2);

        // return how many bytes this message takes in the data stream
        *len = 2;

        uint8_t flashNum = data[1];
        sprintf(buffer, "%s%03d.%s", FRAME_FILENAME, flashNum, FRAME_EXTENSION);

        // if have the file opened already - come back in a little while
        if( printer_rd != NULL)
            return 0x06;

        printer_rd = ( FIL *)malloc(sizeof(FIL));
        if( printer_rd == NULL)
            return 0x03;            // NO FREE MEMORY!

        FRESULT fr = f_open_adj(printer_rd, buffer, FA_READ);
        if( fr == FR_OK)
        {
            data[1] = 0x00;         // NO ERROR, CONTINUE
            if( shouldReply)
                CAT_prepareTxDataAndSend(data, 2);
        }
        else
        {
            free(printer_rd);
            printer_rd = NULL;

            //! if error, stop all and return error to master
            return 0x02;
        }

        printer_startGfxFrame(printer_rd->fsize, NORMAL_MODE);
        return IN_COMMAND_EXECUTION;       // so that next time it returns to me
    } // endif first time
    else
    {
        //!@!@!@!@ WILL NOT WORK ON CCM !!!!!!
        //uint8_t dat[256];
        uint32_t red;
        uint8_t *dat = ( uint8_t *)malloc(256);

        f_read(printer_rd, dat, 256, ( uint *)&red);
        if( red != 0)
        {
            printer_dumpFrame(dat, red);
            free(dat);
            //xtimer_usleep( 500000);         // ns 20-10
            return IN_COMMAND_EXECUTION; // call me again please until I'm finished dumping
        }
        else
        {
            f_close(printer_rd);

            free(printer_rd);
            free(dat);
            printer_rd = NULL;
            return FINISHED_EXECUTING_RAM; // time to wrap it up...
        }
    }
} // endfunc

/**
 * @brief This command is taken from the Verifone manual. It takes
 *        a number of frame and (optionally) name and saves the data
 *        after uncompressing it, to flash card
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_defineVerifoneFrame( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[12];
    char buffer[32];

    CAT_readCmdRingBuf(data, 12);

    // return how many bytes this message takes in the data stream
    *len = 12;

    uint8_t flashNum = data[1];
    uint16_t size = data[10] | (data[11] << 8);

    //! if file is still open by CAT_printFlashGraphics return WAIT!
    if( rd->fs != 0)
        return 0x06;            //! WAIT some time then send again

    sprintf(buffer, "%s%03d.%s", FRAME_FILENAME, flashNum, FRAME_EXTENSION);
    if( CAT_findFile(buffer) == FR_EXIST)
        return 0x10;            //! ERROR, frame already taken...

    //! just create the file and get out... wait for graphics data
    FRESULT fr = f_open_adj(rd, buffer, FA_CREATE_NEW);
    if( fr == FR_OK)
    {
        f_close(rd);
        return 0x00;
    }
    else
        return 0x01;           //! other error
} // endfunc define printable frame

/**
 * @brief This command is taken from the Verifone manual. It takes
 *        a number of frame and (optionally) name and saves the data
 *        after uncompressing it, to flash card
 *        command 0x66
 * @param len
 * @param shouldReply
 * @return
 *
 * From verifone's manual, p. 39 (42 on pdf)
 * 102 (66 hex) Download Printable Flash Frame
 * NOTE OP4100 will simulate a valid response
 * Byte 0 Command Code (102 decimal / 66 hex)
 * Byte 1 Flash Frame Number (1 – 215 decimal / 01 – D7 hex)
 * Byte 2 LSB Address Within Frame
 * Byte 3 MSB Address Within Frame
 * Byte 4 Compressed Data Byte Count (1 – 240 decimal / 01 – F0 hex)
 * Bytes 5 – n Compressed Data
 *
 */
int CAT_downloadVerifoneFrame( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[256], cnt;
    char buffer[32];
    unsigned int written;

    CAT_readCmdRingBuf(data, 5);
    cnt = data[4];

    // return how many bytes this message takes in the data stream
    *len = 5 + cnt;

    uint8_t flashNum = data[1];
    uint16_t add = data[2] | (data[3] << 8);
    sprintf(buffer, "%s%03d.%s", FRAME_FILENAME, flashNum, FRAME_EXTENSION);
    /**
     * DELETE THIS FRAME OUT!
     */
    if( cnt == 0)
    {
        f_unlink(buffer);
        return 0x00;
    }
    CAT_readCmdRingBuf(data, 5 + cnt);
#if 0
    uint32_t uclen = sizeof( _HeapMem);
    // decompress
    CAT_decompress( ( char *)_HeapMem, &uclen, ( char *)&data[7], clen, DLEchar, cchar);
#endif

#if 1
    FIL *frmrd = ( FIL *)malloc(sizeof(FIL));
    //! just create the file and get out... wait for graphics data
    FRESULT fr = f_open_adj(frmrd, buffer, FA_WRITE | FA_CREATE_NEW);
    if( fr == FR_EXIST)
        fr = f_open(frmrd, buffer, FA_WRITE);
#else
    FRESULT fr = f_open_adj(rd, buffer, FA_WRITE/* | FA_CREATE_NEW*/);
    if( fr == FR_NO_FILE)
    fr = f_open_adj(rd, buffer, FA_WRITE | FA_CREATE_NEW);
#endif
    if( fr == FR_OK)
    {
        f_lseek(frmrd, add);
        fr = f_write(frmrd, &data[5], cnt, &written);
        f_close(frmrd);
        free(frmrd);

        if( written != cnt)
            return 0x06;        // error
        else
            return 0x00;        // OK
    }
    else
    {
        free(frmrd);
        return 0x03;            //! illegal command
    }
} // endfunc

/**
 * @brief CAT_setTimeDate - Command 0x84
 *        This function sets RTC time and date
 *
 * @param len
 * @param shouldReply
 * @return 00 if OK, 01 if error occured
 */

static int elapsedSecondsPC, elapsedSecondsTerminal;

int CAT_setTimeDate( uint16_t *len, uint8_t shouldReply)
{
    static RTC_TimeTypeDef RTC_TimeStructure = { 255, 255, 255, 255 };
    uint8_t data[12];

    CAT_readCmdRingBuf(data, 7);

    // return to remove from ring buffer
    *len = 7;

    RTC_DateTypeDef RTC_DateStructure;

    // if have previously set the time
    if( RTC_TimeStructure.RTC_Hours != 255)
    {
        uint8_t h = data[1];
        if( h < RTC_TimeStructure.RTC_Hours)
            h += 24;

        elapsedSecondsPC = h * 3600 + data[2] * 60 + data[3];
        elapsedSecondsPC -= RTC_TimeStructure.RTC_Hours * 3600 +
                             RTC_TimeStructure.RTC_Minutes * 60 + RTC_TimeStructure.RTC_Seconds;

        RTC_TimeTypeDef RTC_CurTimeStructure;
        RTC_GetTime(RTC_Format_BIN, &RTC_CurTimeStructure);

        int ch = RTC_CurTimeStructure.RTC_Hours;
        if( ch < RTC_TimeStructure.RTC_Hours)
            ch += 24;
        elapsedSecondsTerminal = ch * 3600 +
                                  RTC_CurTimeStructure.RTC_Minutes * 60 +
                                   RTC_CurTimeStructure.RTC_Seconds;
        elapsedSecondsTerminal -= RTC_TimeStructure.RTC_Hours * 3600 +
                             RTC_TimeStructure.RTC_Minutes * 60 + RTC_TimeStructure.RTC_Seconds;
        if( elapsedSecondsPC - elapsedSecondsTerminal > 5 ||
             elapsedSecondsTerminal - elapsedSecondsPC > 5)
        {
            return 0x00;
        }

    }
    RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);
    RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);

    RTC_TimeStructure.RTC_Hours = data[1];
    RTC_TimeStructure.RTC_Minutes = data[2];
    RTC_TimeStructure.RTC_Seconds = data[3];
    RTC_DateStructure.RTC_Date = data[4];
    RTC_DateStructure.RTC_Month = data[5];
    RTC_DateStructure.RTC_Year = data[6];

    ErrorStatus ret1 = RTC_SetTime( RTC_Format_BIN, &RTC_TimeStructure);
    ErrorStatus ret2 = RTC_SetDate( RTC_Format_BIN, &RTC_DateStructure);
    if( ret1 != SUCCESS || ret2 != SUCCESS)
        return 0x01;
    else
        return 0x00;
} // endfunc

/**
 * @brief Command 0x30
 *        This function simply deletes the frame from SD card to prepare for
 *        command 0x31 below
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_initializeBitmapFrame( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[3];
    char buffer[32];

    CAT_readCmdRingBuf(data, 3);

    // return to remove from ring buffer
    *len = 3;

    uint8_t flashNum = data[1];

    //! if file is still open by CAT_printFlashGraphics return WAIT!
    if( rd->fs != 0)
        return 0x04;            //! WAIT some time then send again

    sprintf(buffer, "%s%03d.%s", FRAME_FILENAME, flashNum, FRAME_EXTENSION);
    FRESULT fr = f_unlink(buffer);
    return 0x00;
} // endfunc

/**
 * @brief Command 0x31
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_downloadCompressedFrame( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[256];
    char buffer[32];
    unsigned int written;

    CAT_readCmdRingBuf(data, 7);

    uint32_t clen = ( uint32_t)data[6];

    // return to remove from ring buffer
    *len = 7 + clen;

    uint8_t flashNum = data[1];
    char cchar = data[2];
    char DLEchar = data[3];
    uint32_t address = data[4] | (data[5] << 8);

    //! if file is still open by CAT_printFlashGraphics return WAIT!
    if( rd->fs != 0)
        return 0x06;            //! WAIT some time then send again

    sprintf(buffer, "%s%03d.%s", FRAME_FILENAME, flashNum, FRAME_EXTENSION);
    FRESULT fr = CAT_findFile(buffer);
    if( fr == FR_EXIST || fr != FR_NO_FILE)
        return 0x04;            //! ERROR, frame already taken...

    //! just create the file and get out... wait for graphics data
    if( fr == FR_NO_FILE)
    {
        fr = f_open_adj(rd, buffer, FA_CREATE_NEW | FA_WRITE);
        if( fr != FR_OK)
            return 0x04;            //! ERROR, disk error
    }
    f_lseek(rd, address);

    CAT_readCmdRingBuf(data, 7 + clen);

    uint32_t uclen = sizeof(_HeapMem);
    // decompress
    CAT_decompress(( char *)_HeapMem, &uclen, ( char *)&data[7], clen, DLEchar,
            cchar);
    // write to file
    fr = f_write(rd, _HeapMem, uclen, &written);
    f_close(rd);
    if( fr != FR_OK || written != uclen)
        return 0x04;
    else
        return 0x00;
}

/**
 * @brief Command 0x32
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_displayROMFrame( uint16_t *len, uint8_t shouldReply)
{
    static uint8_t flashNum;

    uint8_t data[256];
    char buffer[32];

    if( *len != 0)
    {
        CAT_readCmdRingBuf(data, 2);

        // return to remove from ring buffer
        *len = 2;
        if( catstate != -1 && nextCatState != -1)
            return 0x06;            // busy --- try later

        flashNum = data[1];

        //< send a reply and process request in main thread time
        data[1] = 0x00;
        if( shouldReply)
            CAT_prepareTxDataAndSend(data, 2);
#if 0
        display_removeWindow2();            // NS 24-07
#endif
        return IN_COMMAND_EXECUTION; // call me again please until I'm finished displaying
    }
    else
    {
        if( thread_getstatus(clip_pid) == STATUS_NOT_FOUND)
        {
            // 03-11-16 ????  display_showImageWindow();
            uint8_t ret = CAT_showROMframe(flashNum);
#if 0
            if( ret != 0x00)
            display_clear();
#endif
        }
        return FINISHED_EXECUTING_RAM;
    }
}

/***********************************************************
 *              K E Y P A D     A R E A
 ***********************************************************/
/*
 * @brief define the keypad functionality
 * @param len
 * @param shouldReply
 * @return
 */
#define KEYPAD_COLUMNS  4
#define KEYPAD_ROWS     4

typedef struct
{
    uint8_t keypadLocked;                   // keypad is currently locked
    uint8_t clearPressed;                   // clear key pressed
    uint8_t clearPressedOnEmptyBuf;         // clear key pressed on empty buffer
    uint8_t keyPressedSinceReturn; // was key pressed since last return key buffer request
    uint8_t keyPressedSinceStatus;  // was key pressed since last status request
    uint8_t clearEntireKeyBuffer;
    uint8_t clearUptoLock;
    uint8_t shouldBeep;
    uint8_t keyCountToLock;
    uint8_t characterEcho;                  // F0
    uint8_t clearKey;
    uint8_t lockKeys[KEYPAD_COLUMNS * KEYPAD_ROWS];
    uint8_t ramExecuteKeys[KEYPAD_COLUMNS * KEYPAD_ROWS];
    uint8_t illegalKeys[KEYPAD_COLUMNS * KEYPAD_ROWS];
    uint8_t ignoreKeys[KEYPAD_COLUMNS * KEYPAD_ROWS];
    uint8_t decimalPointLoc;                 // from command 0x03
    char initialText[64];                    // from command 0x03
    uint8_t initialTextAlignment;
} keypadConf_t;
keypadConf_t keypadConfiguration =
{ 0, 0, 0, 0, 0, 0, 0, 1, 9, 0, 15,
{ 0 },
{ 0 },
{ 0 },
{ 0 }, 2,
{ "0.00" } };

typedef struct
{
    uint8_t displayValue;
    uint8_t returnValue;

} keypadDef_t;
keypadDef_t keypadDefinition[KEYPAD_COLUMNS * KEYPAD_ROWS] =
{
{ '*', '*' },
{ '0', 0 },
{ '.', '.' },
{ 'E', '\n' },
{ '7', 7 },
{ '8', 8 },
{ '9', 9 },
{ 'X', '\t' },
{ '4', 4 },
{ '5', 5 },
{ '6', 6 },
{ 'N', '\r' },
{ '1', 1 },
{ '2', 2 },
{ '3', 3 },
{ 'C', 0x90 } };

/**
 * @brief command 0x1c clears the key buffer completely
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_clearKeyBuffer( uint16_t *len, uint8_t shouldReply)
{
    *len = 1;

    if( keypadConfiguration.clearUptoLock)
    {
        uint8_t tempBuffer[KEYPAD_BUF_SIZE];
        unsigned cnt = ringbuffer_peek(&keypad_ringbuf, ( char *)tempBuffer,
                KEYPAD_BUF_SIZE);
        int i;
        for( i = cnt - 1; i > 0; i--)
        {
            if( tempBuffer[i] == 0xFF)          // last lock
                break;
        } // endfor look for locks

        // mutex_lock
        ringbuffer_init(&keypad_ringbuf, ( char *)keypadBuffer,
                KEYPAD_BUF_SIZE);
        ringbuffer_add(&keypad_ringbuf, ( char *)tempBuffer, i);
        // mutex unlock
    } // endif clear up to lock
    else
    {
        // remove all bytes from key buffer
        ringbuffer_remove(&keypad_ringbuf, KEYPAD_BUF_SIZE);
    }
    keypadConfiguration.keypadLocked = 0;
    keypadConfiguration.clearPressed = 0;
    keypadConfiguration.clearPressedOnEmptyBuf = 0;

    return (0x00);
}

int CAT_returnKeyBuffer( uint16_t *len, uint8_t shouldReply)
{
    *len = 1;
    uint8_t tempBuffer[KEYPAD_BUF_SIZE], keybuffer[KEYPAD_BUF_SIZE + 3];

    keybuffer[0] = 0x0e;
    keybuffer[1] = 0x00; // all is OK

    // next, read everything to a local temp buffer
    unsigned cnt = ringbuffer_peek(&keypad_ringbuf, ( char *)tempBuffer,
    KEYPAD_BUF_SIZE);

    // convert all keys in ringbuffer to returnable bytes
    unsigned inx = 0;
    for( unsigned i = 0; i < cnt; i++)
    {
        uint8_t key = tempBuffer[i];
        if( key == '.')
        {
            tempBuffer[inx++] = key;
            continue;
        }
        /*
         if( keypadDefinition[key].returnValue == 0)
         continue;
         */
        tempBuffer[inx++] = keypadDefinition[key].returnValue;
    } // endfor

    inx = CAT_buildKeypadBuf(inx, ( char *)tempBuffer, ( char *)&keybuffer[3],
            keypadDefinition[1].returnValue/* 0 */, '.');
    if( keypadConfiguration.keypadLocked)
    {
        keybuffer[inx + 3] = 0xFF; // keypad was locked before the last character
        inx++;
    }
    keybuffer[2] = inx;
    CAT_prepareTxDataAndSend(keybuffer, inx + 3);

    keypadConfiguration.keyPressedSinceReturn = 0;

    //send it to master
    return I_WILL_DEAL_WITH_RETURN;
}

int CAT_lockKeypad( uint16_t *len, uint8_t shouldReply)
{
    *len = 1;
    keypadConfiguration.keypadLocked = 1;
    // notify someone that keypad is locked

    // stop EPP plaintext
    if( epp_getError() != EPP_OK)
        epp_abortPlaintext();
    return 0x00;
}

int CAT_waitForKeypadLock( uint16_t *len, uint8_t shouldReply)
{
    *len = 1;

    if( keypadConfiguration.keypadLocked)
        return 0x00;
    else
        return IN_COMMAND_EXECUTION;
}

/**
 * @brief shows a message box with text from RAM location
 *
 *        This function accepts title, message and number of seconds to
 *        keep on screen (or until a key is pressed).
 *
 *        [CMD][loc1][loc2][TO]
 *
 *        Where CMD is command # (0x81)
 *              loc1 is the location of title string in RAM
 *              loc2 is the location of the message text in RAM
 *              TO   is the timeout of message in seconds
 *
 * @param len
 * @param shouldReply
 * @return
 */
static kernel_pid_t mbthread = KERNEL_PID_UNDEF;
void CAT_fullMBdone( void)
{
    mbthread = KERNEL_PID_UNDEF;
}

int CAT_fullMessageBox( uint16_t *len, uint8_t shouldReply)
{
    static uint8_t loc1, loc2, timeout;
    static messageBoxStruct m;
    uint8_t data[4];

    //! FIRST READ DIRECTLY FROM ETHERNET THREAD!
    //! release it quickly
    if( *len != 0)
    {
        CAT_readCmdRingBuf(data, 4);

        // return how many bytes this message takes in the data stream
        *len = 4;

        loc1 = data[1];
        loc2 = data[2];
        timeout = data[3];

        // is user requesting to drop the message box?
        if( loc1 == 0 && loc2 == 0 && timeout == 0)
        {
            display_dropMessageBoxThread();
            return 0x00;
        }
        if( thread_getstatus( mbthread) != STATUS_NOT_FOUND ||
             thread_getstatus(clip_pid) != STATUS_NOT_FOUND)
            return 0x41;        // wait for thread to finish, then call mb again

        if( loc1 < 0x10 /*|| loc1 > 0xFF*/ || ramData[loc1].data == 0
                || ramData[loc1].length == 0)
            return 0x40;    //RAM reference number empty or illegal
        if( loc2 < 0x10 /*|| loc2 > 0xFF*/ || ramData[loc2].data == 0
                || ramData[loc2].length == 0)
            return 0x40;    //RAM reference number empty or illegal

#if 0
        if( timeout == 0)
            return 0x41;    // timeout parameter error
#endif

        data[1] = 0x00;     // OK!!
        if( shouldReply)
            CAT_prepareTxDataAndSend(data, 2);
#if 0
        return IN_COMMAND_EXECUTION;
    }
    else
    {
#if 0
        // waits until timeout is over.......
        display_setupMessageBox(( char *)ramData[loc1].data,
                ( char *)ramData[loc2].data, timeout);
#else

        if( thread_getstatus( mbthread) != STATUS_NOT_FOUND)
            return 0x41;        // wait for thread to finish, then call mb again


        m.title   = ( const char *)ramData[loc1].data;
        m.text    = ( const char *)ramData[loc2].data;
        m.timeout = timeout;
        m.mbDone  = CAT_fullMBdone;

        mbthread = thread_create(( char *)displaymb_stack, ( int)sizeof(displaymb_stack),
                         PRIORITY_MAIN - 2 /* was -1 */, CREATE_STACKTEST,
                          ( thread_task_func_t)display_mbThread, ( void *)&m, "mb_thread");
#endif
        return FINISHED_EXECUTING_RAM;
#else
        if( thread_getstatus( mbthread) != STATUS_NOT_FOUND)
            return 0x41;        // wait for thread to finish, then call mb again


        m.title   = ( const char *)ramData[loc1].data;
        m.text    = ( const char *)ramData[loc2].data;
        m.timeout = timeout;
        m.mbDone  = CAT_fullMBdone;

        mbthread = thread_create(( char *)displaymb_stack, ( int)sizeof(displaymb_stack),
                         PRIORITY_MAIN - 2 /* was -1 */, CREATE_STACKTEST,
                          ( thread_task_func_t)display_mbThread, ( void *)&m, "mb_thread");
#endif
    }
}

/**
 * @brief shows a message box with text from RAM location
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_messageBox( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2];
    static uint32_t mb_loc;


    //! FIRST READ DIRECTLY FROM ETHERNET THREAD!
    //! release it quickly
    if( *len != 0)
    {
        CAT_readCmdRingBuf(data, 2);

        // return how many bytes this message takes in the data stream
        *len = 2;

        uint32_t loc = ( uint32_t)data[1];
        if( loc < 0x10 || loc > 0xFF || ramData[loc].data == 0
                || ramData[loc].length == 0)
            return 0x40;    //RAM reference number empty or illegal


        mb_loc = loc;
        data[1] = 0x00;
        CAT_prepareTxDataAndSend(data, 2);

        //send it to master
        return IN_COMMAND_EXECUTION;
    }
    else
    {
        display_messageBox(( char *)ramData[mb_loc].data);
        mb_loc = 0;
        return FINISHED_EXECUTING_RAM;
    }
}

/**
 * @brief Activates keypad and Displays message, command 0x03
 * @param len
 * @param shouldReply
 * @return
 */
#if 0
int CAT_activateKeypadDisplay( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[256];

    CAT_readCmdRingBuf(data, 5);
    *len = 5 + data[4];
    CAT_readCmdRingBuf(data, *len);

    keypadConfiguration.decimalPointLoc = data[3]; // if 0 or FF -> no decimal point

    if( data[1] == 0x01)
    {
        // initial display data [when clear is pressed] = byte 5 to 5-n
        memcpy(keypadConfiguration.initialText, &data[5], data[4]);
        keypadConfiguration.initialTextAlignment = ALIGN_CENTER_RIGHT;

        CAT_showKeypad();

        //activate keypad
        display_setRightText( FIRST_LINE);
        display_setRightText( SECOND_LINE);
        display_showText( (char *)" ", 1, TXB_ID_1);// remove second line...

        CAT_scanForCommandsAndShow( keypadConfiguration.initialText, data[4], FIRST_LINE);
        //display_showText(keypadConfiguration.initialText, &data[4]);
        return 0x00;
    }
    else if( data[1] == 0x02)
    {   // read saved data from keypad....
        // !@#$!@#%
        CAT_showKeypad();
        return 0x00;
    }
    else
    // ERROR
    return 0x40;
} // endfunc
#else
int CAT_activateKeypadDisplay( uint16_t *len, uint8_t shouldReply)
{
    static int keypadMode = 0, __len;
    uint8_t data[256];

    if( *len > 0)
    {
        CAT_readCmdRingBuf(data, 5);
        *len = 5 + data[4];
        CAT_readCmdRingBuf(data, *len);

        keypadConfiguration.decimalPointLoc = data[3]; // if 0 or FF -> no decimal point
        if( keypadConfiguration.decimalPointLoc == 0xff)
            keypadConfiguration.decimalPointLoc = 0;

        if( data[1] == 0x01)
        {
            // initial display data [when clear is pressed] = byte 5 to 5-n
            memcpy(keypadConfiguration.initialText, &data[5], /*data[4]*/sizeof( keypadConfiguration.initialText));
            keypadConfiguration.initialText[data[4]] = NULL;    // NS 23-07-17
            keypadConfiguration.initialTextAlignment = ALIGN_CENTER_RIGHT;
            keypadMode = 1;
        }
        else if( data[1] == 0x02)
        {
            keypadMode = 2;
        }
        else
            return 0x40;

        // stop short clip in any case... (if PC forgot)
        CAT_stopShortClip();

        //! stops clip thread if active
        clip_StopAndKillThreads();

        data[1] = 0x00;     // OK!!
        __len = (int)data[4];
        if( shouldReply)
            CAT_prepareTxDataAndSend(data, 2);

        return IN_COMMAND_EXECUTION;
    }
    else
    {
        if( thread_get(clip_pid) != NULL)
            return FINISHED_EXECUTING_RAM;          // why not use IN COMMAND EXECUTION????

        CAT_showKeypad();

        if( keypadMode == 1)
        {
            //activate keypad
            display_setRightText( FIRST_LINE);
            display_setRightText( SECOND_LINE);
            display_showText(( char *)" ", 1, TXB_ID_1); // remove second line...

            CAT_scanForCommandsAndShow(keypadConfiguration.initialText, __len /*data[4]*/,
                    FIRST_LINE);
            //display_showText(keypadConfiguration.initialText, &data[4]);
            return FINISHED_EXECUTING_RAM;
        }
        else if( keypadMode == 2)
        {
            return FINISHED_EXECUTING_RAM;
        }
        else
            return FINISHED_EXECUTING_RAM;
    }
} // endfunc

#endif

/**
 * @brief Displays message, and in some cases, enables keypad entry
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_displayAndActivateKeypad( uint16_t *len, uint8_t shouldReply)
{
    return 0;
    // currently not implemented..

    uint8_t data[256];

    CAT_readCmdRingBuf(data, 2);
    // return how many bytes this message takes in the data stream
    *len = 2;
    //! display immediate data
    if( data[1] == 0x00)
    {
        CAT_showKeypad();
        CAT_readCmdRingBuf(data, 3);
        uint8_t cnt = data[2];
        CAT_readCmdRingBuf(data, cnt + 3);
        // show message &data[3], cnt
        memcpy(keypadConfiguration.initialText, &data[3],
                sizeof(keypadConfiguration.initialText));
        keypadConfiguration.initialTextAlignment = ALIGN_CENTER_LEFT;
        display_setLeftText( FIRST_LINE);
        CAT_scanForCommandsAndShow(keypadConfiguration.initialText, cnt,
                FIRST_LINE);
        //display_showText(keypadConfiguration.initialText, cnt);
        *len = cnt + 3;
        return 0x00;
    }
    //! display keybuffer activate keypad
    else if( data[1] == 0x01)
    {
        *len = 2;
        //activate (and show) the keypad and that's it for now
        return 0x00;
    }
    // display text in RAM
    else if( data[1] == 0x04)
    {
        *len = 3;

        CAT_readCmdRingBuf(data, 3);
        uint32_t loc = ( uint32_t)data[2];
        if( loc < 0x10 || loc > 0xFF || ramData[loc].data == 0
                || ramData[loc].length == 0)
            return 0x40;    //RAM reference number full or illegal

        CAT_showKeypad();
        memcpy(data, ramData[loc].data, ramData[loc].length);

        // now display data. Keypad left disabled
        memcpy(keypadConfiguration.initialText, ramData[loc].data,
                sizeof(keypadConfiguration.initialText));
        keypadConfiguration.initialTextAlignment = ALIGN_CENTER_LEFT;
        display_setLeftText( FIRST_LINE);
        CAT_scanForCommandsAndShow(keypadConfiguration.initialText,
                ramData[loc].length, FIRST_LINE);
        //display_showText(keypadConfiguration.initialText, ramData[loc].length);
    }
    return 0x00;
} // endfunc

/**
 * @brief Displays multiple messages, code 0x28
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_displayMultiple( uint16_t *len, uint8_t shouldReply)
{
    static uint8_t data[8], waitTime, messLoc;
    static uint32_t starttime;
    int ret;

    return 0;
    // not implemented because of showkeypad using _usleep

    if( *len)
    {
        memset(data, 0, 8);
        CAT_readCmdRingBuf(data, 8);
        // return how many bytes this message takes in the data stream
        *len = 2;
        //! display immediate data
        waitTime = data[1];
        if( data[3] == 0x00)
            *len = 4;
        else if( data[4] == 0x00)
            *len = 5;
        else if( data[5] == 0x00)
            *len = 6;
        else if( data[6] == 0x00)
            *len = 7;
        else
            *len = 8;
        starttime = 0;

        // check RAM locations for valid data
        for( int i = 0; i < 8; i++)
        {
            if( data[i] != 0x00)
            {
                if( data[i] < 0x10 || data[i] > 0xFF
                        || ramData[data[i]].data == 0
                        || ramData[data[i]].length == 0)
                    return (0x02);    //RAM reference number illegal
            }
        } // endfor check RAM locations
        if( shouldReply)
        {
            data[1] = 0x00;
            CAT_prepareTxDataAndSend(data, 2);
        }
        messLoc = 2;

        CAT_showKeypad();

        return IN_COMMAND_EXECUTION;
    } // got the data

    else
    {
        uint8_t loc = data[messLoc];
        if( loc == 0x00)
            return FINISHED_EXECUTING_RAM;

        if( /*hwtimer_now()*/xtimer_now() - starttime
                > ( uint32_t)waitTime * 100000)
        {
            // show message &data[3], cnt
            memcpy(keypadConfiguration.initialText, ramData[loc].data,
                    sizeof(keypadConfiguration.initialText));
            keypadConfiguration.initialTextAlignment = ALIGN_CENTER_LEFT;
            CAT_scanForCommandsAndShow(keypadConfiguration.initialText,
                    ramData[loc].length, FIRST_LINE);

            messLoc++;
#ifdef HWTIMER
            starttime = hwtimer_now();
#else
            starttime = xtimer_now();
#endif
        }
        return IN_COMMAND_EXECUTION;
    }
} // endfunc

/**
 * @brief Displays multiple ROM frames, code 0x82 (similar to 0x28)
 *
 * should prepare a RAM location with the data in 0x28
 *
 * @param len
 * @param shouldReply
 * @return
 */
uint8_t *screens = NULL;

int CAT_displayMultipleFrames( uint16_t *len, uint8_t shouldReply)
{
    static uint8_t data[8], waitTime, messLoc;
    static uint32_t starttime, __ramloc, *screens = NULL;
    int ret;

    if( *len)
    {
        memset(data, 0, 8);
        CAT_readCmdRingBuf(data, 8);
        // return how many bytes this message takes in the data stream
        *len = 3;
        starttime = 0;
        waitTime = data[2];

        if( data[1] < 0x10 || data[1] > 0xFF || ramData[data[1]].data == 0
                || ramData[data[1]].length == 0)
            return (0x02);    //RAM reference number illegal

        messLoc = 0;
        uint8_t tempramloc = data[1];

        display_removeWindow2();            // NS 24-07

        if( shouldReply)
        {
            data[1] = 0x00;
            CAT_prepareTxDataAndSend(data, 2);
        }

#if 0
        //! if already running this stunt... just change locations
        if( __ramloc != 0)
        {
            __ramloc = tempramloc;
            return I_WILL_DEAL_WITH_RETURN;
        }
#endif
        __ramloc = tempramloc;
#if 0
        char buffer[32];
        display_showImageWindow();
        int loclen = ramData[__ramloc].length;
        screens = display_malloc( 800*480*2*loclen);

        for( int i = 0; i < loclen; i++)
        {
            uint8_t loc = ramData[__ramloc].data[i];
            sprintf(buffer, "%s%03d.%s", ROM_FILENAME, loc, ROM_EXTENSION);
            uint32_t offset = (uint32_t )display_readBmpToMemory( buffer, NULL);
            BITMAP_FILEHEADER *fhead = (BITMAP_FILEHEADER *)offset;
            if( fhead->Signature != /*SIGNATURE_OF_BITMAP*/0x4D42) //< "BM" is the signature. If can't get that --- go out!
            {
                display_free( ( void *)offset);
                display_free( screens);
                screens = NULL;
                return 0x05;
            }
            uint8_t *pixelsStart = ( uint8_t *)( offset + fhead->BitsOffset);
            BITMAPINFOHEADER *bmpheader = (BITMAPINFOHEADER *)( offset + sizeof( BITMAP_FILEHEADER));
            display_setImageToDisplaySilently( bmpheader->biWidth, bmpheader->biHeight, fhead, screens);
            display_free(( void *)offset);
        }

        // ALSO prepare the room for the screens in memory
        uint mloc = 0;
        char buffer[32];
        while( screens != NULL)
        {
            uint8_t loc = ramData[__ramloc].data[mloc];
            if( loc == 0) break;
            sprintf(buffer, "%s%03d.%s", ROM_FILENAME, loc, ROM_EXTENSION);

        }
#endif
        return IN_COMMAND_EXECUTION;
    } // got the data

    else
    {   // if image window is not shown anymore --- abort display
        if( !display_isImageWindowShown() ||
        thread_getstatus( clip_pid) != STATUS_NOT_FOUND)
        {
            __ramloc = 0;
            waitTime = 0;
            messLoc = 0;
            CAT_showROMframe(0);
            return FINISHED_EXECUTING_RAM;
        }

        uint8_t loc = ramData[__ramloc].data[messLoc];
        if( /*hwtimer_now()*/xtimer_now() - starttime
                > ( uint32_t)waitTime * 100000)
        {
            uint32_t _goto = xtimer_now() - starttime;
            starttime = xtimer_now();
            CAT_showROMframe(loc);
            messLoc++;
            if( ramData[__ramloc].data[messLoc] == 0x00)
                messLoc = 0;
        }
        return IN_COMMAND_EXECUTION;
    }
} // endfunc

/**
 * @brief defines keypad buttons
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_defineKeypad( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2];

    CAT_readCmdRingBuf(data, 2);

    // return how many bytes this message takes in the data stream
    *len = 2;

    uint32_t loc = ( uint32_t)data[1];
    if( loc < 0x10 || loc > 0xFF || ramData[loc].data == 0
            || ramData[loc].length == 0)
        return 0x40;    //RAM reference number full or illegal

    int inx = 0;
    while( inx < ramData[loc].length)
    {
        uint8_t x = ramData[loc].data[inx++];
        uint8_t y = ramData[loc].data[inx++];
        if( x >= KEYPAD_COLUMNS || y >= KEYPAD_ROWS)
        {
            inx += 2;
            continue;
        }
        keypadDefinition[y * KEYPAD_COLUMNS + x].displayValue =
                ramData[loc].data[inx++];
        keypadDefinition[y * KEYPAD_COLUMNS + x].returnValue =
                ramData[loc].data[inx++];
    } // endwhile
    return 0x00;
} // endfunc

#if 0
/**
 * @brief returns the keypadDefinition structuer's display value to caller
 * @param x - number of column. Column 0 if left most.
 * @param y - number of row. Row 0 is bottom row.
 * @return char as the displayed value
 */
char CAT_getKeypadText( uint8_t x, uint8_t y)
{
    return( ( char)keypadDefinition[y * KEYPAD_COLUMNS + x]).displayValue);
}
#endif

int CAT_configureKeypad( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2];

    CAT_readCmdRingBuf(data, 2);

    // return how many bytes this message takes in the data stream
    *len = 2;

    uint32_t loc = ( uint32_t)data[1];
    if( loc < 0x10 || loc > 0xFF || ramData[loc].data == 0
            || ramData[loc].length == 0)
        return 0x40;    //RAM reference number full or illegal

    int inx = 0;
    uint8_t activationMode = ramData[loc].data[inx++];
    if( activationMode & 0x80)
        keypadConfiguration.clearEntireKeyBuffer = 1;
    else
        keypadConfiguration.clearEntireKeyBuffer = 0;
    if( activationMode & 0x40)
        keypadConfiguration.clearUptoLock = 1;
    else
        keypadConfiguration.clearUptoLock = 0;
    if( activationMode & 0x20)
        keypadConfiguration.shouldBeep = 1;
    else
        keypadConfiguration.shouldBeep = 0;

    keypadConfiguration.keyCountToLock = ramData[loc].data[inx++];
    keypadConfiguration.characterEcho = 0;

    while( inx < ramData[loc].length)
    {

        uint8_t cmd = ramData[loc].data[inx++];

        if( cmd == 0xF0)
        {     // character echo
            keypadConfiguration.characterEcho = ramData[loc].data[inx++];
        }
        else if( cmd == 0xF1)
        {     // lock keypad on keys
            for( int i = 0; i < KEYPAD_ROWS * KEYPAD_COLUMNS; i++)
                keypadConfiguration.lockKeys[i] = 0;
            while( ramData[loc].data[inx] < 0xF0)
            {
                int x = ramData[loc].data[inx++];
                int y = ramData[loc].data[inx++];
                if( x >= KEYPAD_COLUMNS || y > KEYPAD_ROWS)
                    continue;
                keypadConfiguration.lockKeys[y * KEYPAD_COLUMNS + x] = 1;
            } // endwhile
        } // endif lock keys
        else if( cmd == 0xF2)
        {     // execute command on key hit
            for( int i = 0; i < KEYPAD_ROWS * KEYPAD_COLUMNS; i++)
                keypadConfiguration.ramExecuteKeys[i] = 0;
            while( ramData[loc].data[inx] < 0xF0)
            {
                uint8_t x = ramData[loc].data[inx++];
                uint8_t y = ramData[loc].data[inx++];
                uint8_t e = ramData[loc].data[inx++];
                if( x >= KEYPAD_COLUMNS || y > KEYPAD_ROWS)
                    continue;
                keypadConfiguration.ramExecuteKeys[y * KEYPAD_COLUMNS + x] = e;
            } // endwhile
        } // endif RAM execute keys
        else if( cmd == 0xF3)
        {     // illegal keys
            for( int i = 0; i < KEYPAD_ROWS * KEYPAD_COLUMNS; i++)
                keypadConfiguration.illegalKeys[i] = 0;
            while( ramData[loc].data[inx] < 0xF0)
            {
                uint8_t x = ramData[loc].data[inx++];
                uint8_t y = ramData[loc].data[inx++];
                if( x >= KEYPAD_COLUMNS || y > KEYPAD_ROWS)
                    continue;
                keypadConfiguration.illegalKeys[y * KEYPAD_COLUMNS + x] = 1;
            } // endwhile
        } // endif illegal keys
        else if( cmd == 0xF4)
        {     // CLEAR KEY
            uint8_t x = ramData[loc].data[inx++];
            uint8_t y = ramData[loc].data[inx++];
            if( x >= KEYPAD_COLUMNS || y > KEYPAD_ROWS)
                continue;
            keypadConfiguration.clearKey = y * KEYPAD_COLUMNS + x;
        } // endif clear key
        else if( cmd == 0xF5)
        {     // dummy keys
            for( int i = 0; i < KEYPAD_ROWS * KEYPAD_COLUMNS; i++)
                keypadConfiguration.ignoreKeys[i] = 0;
            while( ramData[loc].data[inx] < 0xF0)
            {
                uint8_t x = ramData[loc].data[inx++];
                uint8_t y = ramData[loc].data[inx++];
                if( x >= KEYPAD_COLUMNS || y > KEYPAD_ROWS)
                    continue;
                keypadConfiguration.ignoreKeys[y * KEYPAD_COLUMNS + x] = 1;
            } // endwhile
        } // endif dummy (ignore) keys
    } // endwhile through all this RAM contents

    return 0x00;
} // endfunc

/**
 *
 * @brief gets track 1 data from magSwipe module and returns to master
 * @param length - length of received frame
 * @param shouldReply
 * @return I will deal with return...
 *
 */
int CAT_getTrack1( uint16_t *length, uint8_t shouldReply)
{
    uint8_t track1[MAX_BUFF_SZ1];
    int count = 0;
    uint8_t data[MAX_BUFF_SZ1 + 3];

    CAT_readCmdRingBuf(data, 1);
    *length = 1;

    if( card->getConfiguration() != CARD_TRACKS_1_2)
        data[1] = 0x01;
    else if( (count = card->getTrack1(( char *)track1)) == 0)
        data[1] = 0x01;
    else
    {
        data[1] = 0x00;
        memcpy(&data[3], track1, count);
    }
    data[2] = count;

    if( shouldReply)
        CAT_prepareTxDataAndSend(data, 3 + count);

    return I_WILL_DEAL_WITH_RETURN;
}

/**
 *
 * @brief gets track 2 data from magSwipe module and returns to master
 * @param data   - frame received
 * @param length - length of received frame
 * @param shouldReply
 * @return I will deal with return...
 *
 */
int CAT_getTrack2( uint16_t *length, uint8_t shouldReply)
{
    uint8_t track2[MAX_BUFF_SZ2];
    int count = 0;
    uint8_t data[MAX_BUFF_SZ2 + 3];

    CAT_readCmdRingBuf(data, 1);
    *length = 1;

    if( card->getConfiguration() != CARD_TRACKS_1_2
            && card->getConfiguration() != CARD_TRACKS_2
            && card->getConfiguration() != CARD_TRACKS_2)
        data[1] = 0x01;

    else if( (count = card->getTrack2(( char *)track2)) == 0 || count > 40)
    {
        data[1] = 0x01;
        count = 0;
    }
    else
    {
        data[1] = 0x00;
        memcpy(&data[3], track2, MAX_BUFF_SZ2);
    }
    data[2] = count;

    if( shouldReply)
        CAT_prepareTxDataAndSend(data, 3 + count);

    return I_WILL_DEAL_WITH_RETURN;
}

/**
 *
 * @brief gets track 3 data from magSwipe module and returns to master
 * @param data   - frame received
 * @param length - length of received frame
 * @param shouldReply
 * @return I will deal with return...
 *
 */
int CAT_getTrack3( uint16_t *length, uint8_t shouldReply)
{
#if 0
    uint8_t track3[MAX_BUFF_SZ3];
    int count;

    if( magStripe_getConfiguration() != CARD_TRACKS_1_2 ||
            magStripe_getConfiguration() != CARD_TRACKS_2 ||
            magStripe_getConfiguration() != CARD_TRACKS_2)
    return 0x01;

    if(( count = magStripe_getTrack2( track2)) == 0)
    return 0x01;

    data[1] = 0x00;
    data[2] = count;
    memcpy( &data[3], track2, count);

    CAT_prepareTxDataAndSend( data, 3 + count);
    return I_WILL_DEAL_WITH_RETURN;
#endif
    uint8_t data[3];

    CAT_readCmdRingBuf(data, 1);
    *length = 1;

    data[1] = 0x01;
    data[2] = 0x00;

    if( shouldReply)
        CAT_prepareTxDataAndSend(data, 3);

    return I_WILL_DEAL_WITH_RETURN;
}


/**
 * @brief This function sends back the 16 bytes read from the microRWD reader
 *        of the RFID tag close to the microRWD antenna.
 *
 * @param length
 * @param shouldReply
 * @return
 */
int CAT_getRFID( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[17];

    *length = 1;
    CAT_readCmdRingBuf(data, 1);

    if( microRWD_getTag( &data[1]) == RWD_OK)
    {
        if( shouldReply)
            CAT_prepareTxDataAndSend(data, 17);

        return I_WILL_DEAL_WITH_RETURN;
    }
    else
        return 0x00;
}


/**
 *
 * @brief returns EMV tag data
 * @param data   - frame received
 * @param length - length of received frame
 * @param shouldReply
 * @return  0x00 - all data returned OK
 *          0x01 - partial data returned, request again to get next batch
 *          0x02 - no data present
 *
 * Request:
 *          cmd         0x74
 *          tag         HH
 *          tag         LL
 *          start_from  HH
 *          start_from  LL
 *
 * Reply:
 *          00      cmd
 *          01      retcode     0x00, 0x01 or 0x02
 *          02      tag         HH
 *          03      tag         LL
 *          04      len
 *          05...   data....
 */
int CAT_emvTagData( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[5];
    uint8_t retcode[CAT_TX_BUFFER_SZ + 5];

    CAT_readCmdRingBuf(data, 4 + 1);          // command + 4 bytes data
    *length = 5;

    uint32_t _tag = (( uint32_t)data[1] << 8) | (( uint32_t)data[2] & 0xff);
    uint32_t startfrom = (( uint32_t)data[3] << 8)
            | (( uint32_t)data[4] & 0xff);

    const char *tagData = NULL;
    if( emv != NULL && emvReturnCode != CARD_IDLE)
        tagData = emv->getTagData(_tag);
    else
    {
        data[3] = data[2];
        data[2] = data[1];
        data[1] = 0x06;      // error - try again later
        data[4] = 0x00;      // no len
        if( shouldReply)
            CAT_prepareTxDataAndSend(data, 5);
        return I_WILL_DEAL_WITH_RETURN;
    }

    retcode[0] = data[0];
    if( tagData == NULL)
    {
        retcode[3] = data[2];   // tag #
        retcode[2] = data[1];
        retcode[1] = 0x02;
        retcode[4] = 0x00;      // no len
        if( shouldReply)
            CAT_prepareTxDataAndSend(retcode, 5);
    }
    else // if have tag data to return
    {
        retcode[3] = data[2];   // tag #
        retcode[2] = data[1];
        retcode[1] = 0x00;      // ret code OK with data
        int dataLen = strlen(tagData);
        if( startfrom > dataLen)
            startfrom = 0;
        if( dataLen - startfrom > CAT_TX_BUFFER_SZ - 3)
        {
            retcode[1] = 0x01;      // got more data than can currently send
            retcode[4] = CAT_TX_BUFFER_SZ - 3;
        }
        else
            retcode[4] = dataLen;

        memcpy(&retcode[5], &tagData[startfrom], retcode[4]);
        if( shouldReply)
            CAT_prepareTxDataAndSend(retcode, 5 + retcode[4]);
    }

    return I_WILL_DEAL_WITH_RETURN;
} // endfunc

/**
 *
 * @brief SETs EMV tag data
 * @param data   - frame received
 * @param length - length of received frame
 * @param shouldReply
 * @return  0x00 - all data returned OK
 *          0x01 - partial data returned, request again to get next batch
 *          0x02 - no data present
 *
 * @note    currently maximum length of data is 100 bytes
 *
 * Request:
 *          cmd         0x77
 *          tag         HH
 *          tag         LL
 *          len         LL
 *          data        0....len
 *
 * Reply:
 *          retcode     0x00 = OK, 0x01= error with emv object,
 *                      or 0x02 = len exceeds 100 bytes!
 *
 */
int CAT_setEmvTagData( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[5];
    uint8_t _value[CAT_TX_BUFFER_SZ + 4 + 1];

    CAT_readCmdRingBuf(data, 4);         // command + 3 bytes data - tag and len
    *length = 4;

    uint32_t _tag = (( uint32_t)data[1] << 8) | (( uint32_t)data[2] & 0xff);
    uint32_t _len = (( uint32_t)data[3]);
    *length = _len + 4;
    if( _len > 100)
    {
        return 0x02;                // length error
    }

    CAT_readCmdRingBuf(_value, _len + 4); // command + 3 bytes data - tag and len

    if( emv != NULL)
    {
        _value[_len + 4] = NULL;
        emv->setTagData(_tag, ( char *)&_value[4]);
        return 0x00;
    }
    return 0x01;                                    // error with emv object
} // endfunc

/**
 * @brief this is command 0x0d which returns the configuration of the CAT
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return error code to be returned to master or I_WILL_DEAL_WITH_RETURN
 */
int CAT_getConfiguration( uint16_t *length, uint8_t shouldReply)
{
#define CONFIG_STANDARD_CAT     0x01
    uint8_t data[21];

    CAT_readCmdRingBuf(data, 1);
    *length = 1;

    int count = 18;
    data[1] = 0x00;
    data[2] = count;                // currently
    data[3] = CONFIG_STANDARD_CAT;
    data[4] = (MAJOR_VERSION * 0x10) | MINOR_VERSION;     // from version.h
    data[5] = RELEASE_MONTH;                            // version month
    data[6] = RELEASE_DAY;                              // version day
    data[7] = RELEASE_YEAR;                             // version year
    data[8] = 0x02;                 // enhanced graphics display
    data[9] = data[10] = 0x00;      // graphic display has no column/rows
    data[11] = 0x05;                // printer 200dpi clamshell
    data[12] = 24;                  // number of columns printable

    uint8_t mtype = card->getConfiguration();
    if( mtype == CARD_TRACKS_1_2 || mtype == CARD_TRACKS_2)
        data[13] = 0x01;            // track 1/2
    else
        data[13] = 0x03;            // track 2/3

    data[14] = 0x54;                // keypad of 5 rows and 4 columns (!?!?!)
    data[15] = 0x00;                // firmware prerelease
    data[16] = 0x00;                // firmware prerelease
    data[17] = 0x01;                // PCB type??
    data[18] = data[19] = 0x00;     // PCB firmware

    if( shouldReply)
        CAT_prepareTxDataAndSend(data, count + 3);

    return I_WILL_DEAL_WITH_RETURN;
} // endfunc

/**
 * @brief this is command 0x0d which returns the configuration of the CAT
 * command 0x83
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return error code to be returned to master or I_WILL_DEAL_WITH_RETURN
 */
int CAT_getBoardConfiguration( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[21];

    CAT_readCmdRingBuf(data, 1);
    *length = 1;
#if 0
    uint32_t cpuid = *(uint32_t *)(0xE000ED00);

    uint8_t cpuid_[CPUID_ID_LEN];
    cpuid_get( (void *)cpuid_);
//    return cpuid[CPUID_ID_LEN - num - 1];
#endif
    int count = 18;

    data[1] = 0x00;
    data[2] = count;                // currently
    data[3] = CONFIG_STANDARD_CAT;
    data[4] = 0xFF;                 // version unreleased
    data[5] = 1;                    // version month
    data[6] = 1;                    // version day
    data[7] = 1;                    // version year
    data[8] = 0x02;                 // enhanced graphics display
    data[9] = data[10] = 0x00;      // graphic display has no column/rows
    data[11] = 0x05;                // printer 200dpi clamshell
    data[12] = 24;                  // number of columns printable

    uint8_t mtype = card->getConfiguration();
    if( mtype == CARD_TRACKS_1_2 || mtype == CARD_TRACKS_2)
        data[13] = 0x01;            // track 1/2
    else
        data[13] = 0x03;            // track 2/3

    data[14] = 0x54;                // keypad of 5 rows and 4 columns (!?!?!)
    data[15] = 0x00;                // firmware prerelease
    data[16] = 0x00;                // firmware prerelease
    data[17] = 0x01;                // PCB type??
    data[18] = data[19] = 0x00;     // PCB firmware

    if( shouldReply)
        CAT_prepareTxDataAndSend(data, count + 3);

    return I_WILL_DEAL_WITH_RETURN;
} // endfunc

/**
 * @brief clears the POWER fail flag. Command 0x1E
 * @param length
 * @param shouldReply
 * @return
 */
int CAT_clearPowerFlag( uint16_t *len, uint8_t shouldReply)
{
    powerFailFlag = 0;
    *len = 1;
    return 0x00;
}

/**
 * @brief this is command 0x0F which returns the 8 bit status of the CAT
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return error code to be returned to master or I_WILL_DEAL_WITH_RETURN
 */
int CAT_getEightBitStatus( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[2];

    CAT_readCmdRingBuf(data, 1);
    *length = 1;

    uint32_t stat = CAT_buildStatus();
    data[1] = GET_STATUS_LOW_8BIT(stat);

    if( shouldReply)
        CAT_prepareTxDataAndSend(data, 2);

    return I_WILL_DEAL_WITH_RETURN;
} // endfunc

/**
 * @brief this is command 0x21 which returns the 16 bit status of the CAT
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return error code to be returned to master or I_WILL_DEAL_WITH_RETURN
 */
int CAT_get16BitStatus( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[3];

    CAT_readCmdRingBuf(data, 1);
    *length = 1;
    data[1] = 0;
    data[2] = 0;

    uint32_t stat = CAT_buildStatus();
    data[1] = GET_STATUS_LOW_8BIT(stat);
    data[2] = GET_STATUS_MID_8BIT(stat);

    if( shouldReply)
        CAT_prepareTxDataAndSend(data, 3);

    return I_WILL_DEAL_WITH_RETURN;
} // endfunc

/**
 * @brief this is command 0x39 which returns the 24 bit status of the CAT
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return error code to be returned to master or I_WILL_DEAL_WITH_RETURN
 */
int CAT_get24BitStatus( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[4];

    CAT_readCmdRingBuf(data, 1);
    *length = 1;

    uint32_t stat = CAT_buildStatus();
    data[1] = GET_STATUS_LOW_8BIT(stat);
    data[2] = GET_STATUS_MID_8BIT(stat);
    data[3] = GET_STATUS_HIGH_8BIT(stat);

    if( shouldReply)
        CAT_prepareTxDataAndSend(data, 4);

    return I_WILL_DEAL_WITH_RETURN;
} // endfunc

int CAT_get40BitStatus( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[6];

    CAT_readCmdRingBuf(data, 1);
    *length = 1;

    uint32_t stat = CAT_buildStatus();
    data[1] = GET_STATUS_LOW_8BIT(stat);
    data[2] = GET_STATUS_MID_8BIT(stat);
    data[3] = GET_STATUS_HIGH_8BIT(stat);

    /*--------------------------------------*/
#define DEVICE_SD_ERROR     1
#define DEVICE_EPP_ERROR    2
#define DEVICE_ONGOING_LOAD 4
#define DEVICE_EMV_INSERTED 8

#define HAVE_TAG            1
#define MB_CANCELED         2
    /*--------------------------------------*/
    data[4] = 0;
    if( diskerror)
        data[4] |= DEVICE_SD_ERROR;
    if( epp_getError() != EPP_OK)
        data[4] |= DEVICE_EPP_ERROR;
    if( ethernet_isOngoingDownload() || !doneFileLoading)
        data[4] |= DEVICE_ONGOING_LOAD;
    if( card->isCpuCardInserted() && menuStatus == 0)
        data[4] |= DEVICE_EMV_INSERTED;


    data[5] = 0;
    if( microRWD_getTag( NULL) == RWD_OK && menuStatus == 0)
        data[5] |= HAVE_TAG;

    if( display_isMessageBoxCanceled())
        data[5] |= MB_CANCELED;


    /* already have this in byte 2 of the data
     if( thread_getstatus( clip_pid) != STATUS_NOT_FOUND)
     data[4] |= DEVICE_SHOWING_VID;
     */
    if( shouldReply)
        CAT_prepareTxDataAndSend(data, 6);

    return I_WILL_DEAL_WITH_RETURN;
}

/**
 * @brief this is command 0x10 which simply beeps for a specified duration
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return error code to be returned to master or I_WILL_DEAL_WITH_RETURN
 */
int CAT_beep( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[2];

    CAT_readCmdRingBuf(data, 2);
    *length = 2;
    CAT_soundBuzzer(10 * ( uint32_t)data[1]);
    return 0x00;
}

/**
 * @brief this is command 0x23 which sets beep duration. Default is 100mSec
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return error code to be returned to master or I_WILL_DEAL_WITH_RETURN
 */
int CAT_setBeepDuration( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[2];

    CAT_readCmdRingBuf(data, 2);
    *length = 2;
    beepDuration = data[1] * 10;
    return 0x00;
}

/**
 * @brief this is command 0x18 which stores data to 'RAM'
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return error code to be returned to master or I_WILL_DEAL_WITH_RETURN
 */
int CAT_storeToRAM( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[256];

    CAT_readCmdRingBuf(data, 3);

    uint8_t loc = ( uint32_t)data[1];
    uint8_t cnt = data[2];

    if( cnt > 240)
        return 0x80;

    // erase
    if( cnt == 0)
    {   // if already erased just return OK
        if( ramData[loc].length)
        {
            free(ramData[loc].data);
            ramData[loc].length = 0;
            ramData[loc].data = 0;
            ramData[loc].belongsToCmd = 0;
        }
        *length = 3;
        return 0x00;
    }
    else
    {
        *length = 3 + cnt;
        CAT_readCmdRingBuf(data, *length);

        if( ramData[loc].data != 0 || ramData[loc].length)
            return 0x80;        //! error - room taken in RAM

        ramData[loc].data = ( uint8_t *)malloc(cnt);
        if( ramData[loc].data == NULL)
            return 0x80;        //! sorry, no more room left in RAM!

        ramData[loc].length = cnt;
        memcpy(ramData[loc].data, &data[3], cnt); //! copy bytes to new allocation area
    }
    return 0x00;
} // endfunc store to ram

/**
 * @brief this is command 0x13 which turns mag stripe module off
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return error code to be returned to master or I_WILL_DEAL_WITH_RETURN
 */
int CAT_magStripeOff( uint16_t *length, uint8_t shouldReply)
{
    *length = 1;
    card->off();
    return 0x00;
}

/**
 * @brief this is command 0x14 which turns mag stripe module on
 *        it also updates some important parameters for card insertion and
 *        error
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return error code to be returned to master or I_WILL_DEAL_WITH_RETURN
 */
int CAT_magStripeOn( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[3];

    CAT_readCmdRingBuf(data, 3);

    *length = 3;
    cardMaxRetries = data[1] & 0x1F;
    cardIgnore1Err = (data[1] & 0x20) ? 1 : 0;
    cardIgnore2Err = (data[1] & 0x40) ? 1 : 0;
    cardIgnore3Err = (data[1] & 0x80) ? 1 : 0;
    cardInsertedMsg = data[2];
    cardRetryLimitReached = 0;

    card->on();
    return 0x00;
}

/**
 * @brief this is code 0x1f which returns the mag stripe reader errors & status
 * @param length
 * @param shouldReply
 * @return
 */
int CAT_returnCardStatus( uint16_t *length, uint8_t shouldReply)
{
    uint8_t data[3];
    *length = 1;

    data[0] = 0x1F;
    data[1] = 0x00;     // return OK
    data[2] = 0x00;

    uint8_t err = card->getError();
    if( false /*(err & MAG_LRC_ERROR) || (err & MAG_PARITY_ERROR)*/)
        data[2] |= (0x03 << 3);
    if( card->getTrack2( NULL) > 0)
        data[2] |= (1 << 1);
    if( card->getTrack1( NULL) > 0)
        data[2] |= (1 << 0);
    if( card->isON())
        data[2] |= (1 << 6);

    if( shouldReply)
        CAT_prepareTxDataAndSend(data, 3);
}

/**
 * @brief This is the first of complex functions which wait for a certain operation
 *        to be finished before continuing with the flow of commands.
 *        Command 0x17 waits for card until either card buffer is full or
 *        a timeout has been reached.
 *        If the card buffer is full teh CAT will go to the next RAM command
 *        but if an error occurred (timeout, max retries etc.) command execution
 *        is aborted.
 *        If the card inserted timeout is exceeded the CAT will beep and set the
 *        card inserted timeout bit.
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return during command execution - IN_COMMAND_EXECUTION
 *         when abort is specified  - NEXT_COMMANDS_ABORT
 *         when card was swiped     - I_WILL_DEAL_WITH_RETURN
 *
 */
int CAT_waitForCardBufferFull( uint16_t *length, uint8_t shouldReply)
{
    static uint8_t retryMsg, retryReachedCmd, state = 0;
    uint8_t data[3];

    // is first time around?
    if( *length > 0 && state == 0)
    {   // harvest all info from message
        CAT_readCmdRingBuf(data, 3);
        retryMsg = data[1];
        retryReachedCmd = data[2];
        cardRetries = 0;
        state = 1;
        *length = 3;
        return IN_COMMAND_EXECUTION;       // so that next time it returns to me
    }
    else if( state == 1)
    {
        // check if card was swiped. If yes, check if error occurred.
        // if not return card is swiped
        if( (card->getTrack2( NULL)) != 0)
        {
            // have swiped a card
            // continue with script execution
            state = 0;
            return I_WILL_DEAL_WITH_RETURN;
        }
        if( card->getError() != 0)
        {
            cardRetries++;
            if( cardRetries == cardMaxRetries)
            {
                //---> show retry error message at retryMsg
                //---> execute somehow the RAM code at retryLimitReached
                ramLoc256[0] = 0x05;
                ramLoc256[1] = retryReachedCmd;
                ramData[256].data = ramLoc256;
                ramData[256].length = 2;

                if( runningFromRAM > sizeof(saveRamLoc))
                    while( 1)
                        ;                      // STACK FAULT!!!
                saveRamLoc[runningFromRAM - 1] = ramLoc;
                ramData[ramLoc].currentInx += 3; // add extra 3 bytes for 17 xx xx
                *length = 0;
                runningFromRAM++;
                ramLoc = 256;
                state = 0;
                return IN_COMMAND_EXECUTION;
                //return I_WILL_DEAL_WITH_RETURN;
            }
            else
            {
                //---> show retry error message at retryMsg
                // show message &data[3], cnt
                CAT_scanForCommandsAndShow(( char *)ramData[retryMsg].data,
                        ramData[retryMsg].length, FIRST_LINE);
                /*
                 display_showText(ramData[retryMsg].data,
                 ramData[retryMsg].length);
                 */
            }
            card->on();     // reset mag stripe module and all errors
            return IN_COMMAND_EXECUTION;   // so that next time it returns to me
        }
    }
    state = 0;
    return I_WILL_DEAL_WITH_RETURN;

} // end special function

/**
 * @brief This is the first of complex functions which waits for a certain operation
 *        to be finished before continuing with the flow of commands.
 *        Command 0x29 waits for card until either card buffer is full or
 *        a timeout has been reached.
 *        If the card buffer is full teh CAT will go to the next RAM command
 *        but if an error occurred (timeout, max retries etc.) command execution
 *        is aborted.
 *        If the card inserted timeout is exceeded the CAT will beep and set the
 *        card inserted timeout bit.
 *
 * @param data
 * @param length
 * @param shouldReply
 * @return during command execution - IN_COMMAND_EXECUTION
 *         when abort is specified  - NEXT_COMMANDS_ABORT
 *         when card was swiped     - I_WILL_DEAL_WITH_RETURN
 *
 */
int CAT_waitForCardBufferFullPlus( uint16_t *length, uint8_t shouldReply)
{
    static uint8_t retryMsg, cardTimeoutMsg, retryLimitReached, state = 0;
    static uint32_t beepTime, cardInsertedTimeout, waitTimeBeforeStatus;
    static uint64_t starttime;
    static uint32_t shortst;
    uint8_t data[10];

    // is first time around?
    if( *length > 0 && state == 0)
    {   // harvest all info from message
        CAT_readCmdRingBuf(data, 10);
        retryMsg = data[1];
        cardTimeoutMsg = data[7];
        retryLimitReached = data[2];            // execute this when retry limit
        beepTime = ( uint32_t)data[4] << 8 | data[3];
        cardInsertedTimeout = ( uint32_t)data[6] << 8 | data[5];
        waitTimeBeforeStatus = ( uint32_t)data[9] << 8 | data[8];
        data[1] = 0x00;
        // get long long time
#ifdef HWTIMER
        timex_t ttt;
        vtimer_now(&ttt);
        starttime = timex_uint64(ttt);
#else
        starttime = xtimer_now64();
        shortst = xtimer_now();
#endif
        state = 1;

        if( shouldReply)
            CAT_prepareTxDataAndSend(data, 2);
        *length = 10;                   // take out 10 bytes from the RAM buffer
        return IN_COMMAND_EXECUTION;       // so that next time it returns to me
    }
    else if( state == 1)
    {
        // check if card was swiped. If yes, check if error occured.
        // if not return card is swiped
        if( (card->getTrack2( NULL)) != 0)
        {
            // have swiped a card
            // continue with script && state --  execution
            *length = 10;               // take out 10 bytes from the RAM buffer
            state = 0;
            return I_WILL_DEAL_WITH_RETURN;
        }
        if( card->getError() != 0)
        {
            cardRetries++;
            if( cardRetries == cardMaxRetries)
            {
                //---> show retry error message at retryMsg
                // show message &data[3], cnt
                CAT_scanForCommandsAndShow(( char *)ramData[retryMsg].data,
                        ramData[retryMsg].length, FIRST_LINE);

                //---> execute somehow the RAM code at retryLimitReached
                *length = 10;           // take out 10 bytes from the RAM buffer
                state = 0;
                //---> show retry error message at retryMsg
                //---> execute somehow the RAM code at retryLimitReached
                ramLoc256[0] = 0x05;
                ramLoc256[1] = retryLimitReached;
                ramData[256].data = ramLoc256;
                ramData[256].length = 2;

                if( runningFromRAM > sizeof(saveRamLoc))
                    while( 1)
                        ;                      // STACK FAULT!!!
                saveRamLoc[runningFromRAM - 1] = ramLoc;
                ramData[ramLoc].currentInx += *length; // add extra 10 bytes for 29 xx xx...
                *length = 0;
                runningFromRAM++;
                ramLoc = 256;
                state = 0;
                return IN_COMMAND_EXECUTION;

                //return I_WILL_DEAL_WITH_RETURN;
            }
            card->on();     // reset mag stripe module and all errors
            return IN_COMMAND_EXECUTION;   // so that next time it returns to me
        }

#ifdef HWTIMER
        timex_t nowx;
        vtimer_now(&nowx);
        if( timex_uint64(nowx) - starttime > cardInsertedTimeout * 1000)
#else
        if( xtimer_now() - shortst/*starttime*/> cardInsertedTimeout * 1000U)
#endif

        {
            CAT_soundBuzzer(beepTime);
            // show card inserted timeout message
            CAT_scanForCommandsAndShow(( char *)ramData[cardTimeoutMsg].data,
                    ramData[cardTimeoutMsg].length, FIRST_LINE);

#ifdef HWTIMER
            timex_t ttt;
            vtimer_now(&ttt);
            starttime = timex_uint64(ttt);
#else
            starttime = xtimer_now64();
            shortst = xtimer_now();
#endif
            state = 2;
        }
        return IN_COMMAND_EXECUTION;       // so that next time it returns to me
    } // endif state 1
    else if( state == 2)
    {
#ifdef HWTIMER
        timex_t nowx;
        vtimer_now(&nowx);
        if( timex_uint64(nowx) - starttime > waitTimeBeforeStatus * 1000)
#else
        if( xtimer_now() - shortst /*starttime*/> waitTimeBeforeStatus * 1000)
#endif
        {
            cardInsertedTimeoutReached = 1;
            state = 0;
            *length = 10;               // take out 10 bytes from the RAM buffer
            return NEXT_COMMANDS_ABORT;    // so that next time it returns to me
        }
        return IN_COMMAND_EXECUTION;       // so that next time it returns to me
    }
    else
        return IN_COMMAND_EXECUTION;       // so that next time it returns to me
} // end special function

/**
 * @brief start showing clip with filename
 * @param len
 * @param shouldReply
 * @return
 * @note
 */
int CAT_runClip( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[32];
    static char fname[32];

    CAT_readCmdRingBuf(data, 2);
    *len = data[1] + 2;
    // no data in command
    if( *len - 2 == 0)
    {
        clip_StopAndKillThreads();
        return 0x00;
    }
    CAT_readCmdRingBuf(data, *len);


    // drop the messagebox thread if still showing, wait until it is down
    if( thread_getstatus( mbthread) != STATUS_NOT_FOUND)
    {
        display_dropMessageBoxThread();
        return 0x06;                                // busy, try again later
    }

    memcpy(fname, &data[2], data[1]);
    fname[data[1]] = NULL;
    ////////////////////////////////////////
    CAT_runClipThread(fname);
    ////////////////////////////////////////
    return 0x00;
}

/**
 * @brief   starts the EMV session with a new card
 * @param len
 * @param shouldReply
 * @return CARD_OK (=0) if card was setup OK
 *         for other return codes see emvCard.hpp
 */

static int debugdebug = 0;
int CAT_emvStart( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2];

    // first (direct) call from Ethernet thread... just return OK and
    // wait for main thread to recall this function
    if( *len != 0)
    {
        CAT_readCmdRingBuf(data, 1);
        *len = 1;
        // if already running this --- return error 0x06
        if( catstate == data[0]/*emv != NULL || emvReturnCode == CARD_IDLE*/)
            return 0x06; // error, must finalize last EMV transaction before starting new one

        if( debugdebug != 0)
            return 0x00;


        // no more room in state buffer
        if( catstate != -1 && nextCatState != -1)
            return 0x06;                // no room right now, try again later

        data[1] = 0x00;
        emvReturnCode = CARD_IDLE;
        if( shouldReply)
            CAT_prepareTxDataAndSend(data, 2);

        return IN_COMMAND_EXECUTION;
    }
    else
    {
        debugdebug = 1;

        if( emv != NULL)
        {
            delete emv;
            //ashProtocol::purgeObject();
            display_removeMessageBox();
        }

        debugdebug = 2;

        const char *termType = NULL, *capability = NULL, *add = NULL;
        int inx = ashProtocol::indexByKey("EMV", NULL);
        if( inx >= 0)
        {
            ashProtocol::getParam(inx, "TerminalType", &termType);
            ashProtocol::getParam(inx, "TerminalCapabilities", &capability);
            ashProtocol::getParam(inx, "AddTerminalCapabilities", &add);
        }

        debugdebug = 3;

        emv = new emvCard();
        if( (uint32_t)emv > 0x2001d000)
        {
            inx++;
            inx--;
        }

        emv->card_cb = card;                            // store the card call backs in emvCard object

        int eret = emv->start(termType, capability, add);
        if( eret == CARD_MUST_RESET || eret == EMPTY_CANDIDATE_LIST)
        {
            ashProtocol::getParam(inx, "EnableBrandNotPresentFallback", &add);
            if( !strcmp( add, "true"))
            {
                emvReturnCode = CAN_TRY_MAGNETIC;
            }
            else
                emvReturnCode = eret;
        }
        else
        {
            if( eret < 0)
                eret = TERMINATE_SESSION;

            emvReturnCode = eret;
        }

        debugdebug = 0;
        return FINISHED_EXECUTING_RAM;
    }
}

/**
 * @brief   takes the response from the J command of SHVA
 *          and continued the process of EMV transaction
 *
 *          The one byte received with this command is
 *          the response from Ashrait, either
 *          00 - SHVA says continue
 *          01 - SHVA says continue with online only
 *          02 - SHVA says terminate transaction
 *
 * @param len
 * @param shouldReply
 * @return CARD_OK (=0) if card was setup OK
 *         for other return codes see emvCard.hpp
 */

int CAT_emvContinue( uint16_t *len, uint8_t shouldReply)
{
    static uint8_t forceOnline;
    uint8_t data[2];

    // first (direct) call from Ethernet thread... just return OK and
    // wait for main thread to recall this function
    if( *len != 0)
    {
        CAT_readCmdRingBuf(data, 2);
        *len = 2;

        // already removed emv object - RIP...
        if( emv == NULL)
            return 0x01;

        if( data[1] == 0x02)    // terminate
        {
            emv->terminate();
            delete emv;
            emv = NULL;
            //thread_print_all();
            display_removeMessageBox();
            return 0x02;
        }
        forceOnline = data[1];
        data[1] = 0x00;
        //
        // in case another call is made while
        // select has been called before or is in process
        // only CARD_OK (0) from the start function is allowed
        if( emvReturnCode != CARD_OK)
        {
            return 0x03;
        }
        emvReturnCode = CARD_IDLE;
        if( shouldReply)
            CAT_prepareTxDataAndSend(data, 2);

        return IN_COMMAND_EXECUTION;
    }
    else
    {
        int eret = emv->select( forceOnline);
        if( eret == TERMINATE_SESSION)
        {
            int inx = ashProtocol::indexByKey("EMV", NULL);
            if( inx >= 0)
            {
                const char *add;
                ashProtocol::getParam(inx, "EnableBrandNotPresentFallback", &add);
                if( !strcmp( add, "true"))
                {
                    emvReturnCode = CAN_TRY_MAGNETIC;
                }
                else
                    emvReturnCode = eret;
            }
        }
        else
        {
            if( eret < 0)
                eret = TERMINATE_SESSION;
            emvReturnCode = eret;
        }
        return FINISHED_EXECUTING_RAM;
    }
}

/**
 * @brief   takes the response from the J49 command of SHVA
 *          and finalizes the process of EMV transaction
 *
 *          The one byte received with this command is
 *          the response from Ashrait, either
 *          00 - SHVA says continue
 *          01 - SHVA says continue with online only
 *          02 - SHVA says terminate transaction
 *
 * @param len
 * @param shouldReply
 * @return CARD_OK (=0) if card was setup OK
 *         for other return codes see emvCard.hpp
 */

int CAT_emvFinalize( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[2];

    // first (direct) call from Ethernet thread... just return OK and
    // wait for main thread to recall this function
    if( *len != 0)
    {
        CAT_readCmdRingBuf(data, 2);
        *len = 2;
        if( data[1] == 0x02)    // terminate
        {
            // ??? not sure ??? emv->finishTransaction( false);
            emv->terminate();
            delete emv;
            emv = NULL;
            display_removeMessageBox();
            return 0x00;
        }
        data[1] = 0x00;
        if( emvReturnCode != EMV_MUST_GO_ONLINE
                && emvReturnCode != EMV_APPROVED_OFFLINE
                && emvReturnCode != EMV_CARD_DECLINED)
        {
            return 0x00;
        }
        emvReturnCode = CARD_IDLE;
        if( shouldReply)
            CAT_prepareTxDataAndSend(data, 2);

        return IN_COMMAND_EXECUTION;
    }
    else
    {
        int ret  = emv->finishTransaction( true);
        if( ret < 0)
            ret = TERMINATE_SESSION;
        emvReturnCode = ret;
        delete emv;
        emv = NULL;
        return FINISHED_EXECUTING_RAM;
    }
}

/**
 * @brief   returns the EMV session card return code
 * @param len
 * @param shouldReply
 * @return any of the code in emvCard.hpp:
 *
 * CARD_IDLE
 * CARD_MUST_RESET
 * CARD_OK
 * TERMINATE_SESSION
 * CARD_COULDNT_ATR
 * CARD_TRY_AGAIN
 *
 */

int CAT_getEmvReturnCode( uint16_t *len, uint8_t shouldReply)
{
//uint8_t data[2];

    //CAT_readCmdRingBuf(data, 1);
    *len = 1;
    if( emvReturnCode < 0)
        return 0;

    return emvReturnCode;
} // endfunc

void CAT_stopShortClip( void)
{
    stopShortClip = 1;
}

/**
 * @brief start showing clip with filename
 * @param len
 * @param shouldReply
 * @return
 * @note
 *
 * @returning to CAT host:
 *            0x00 - request was accepted and is being handled
 *            0x01 - stop was requested and going underway. Please WAIT
 *            0x04 - clip is currently showing or error while loading short clip to memory
 *
 *
 */
static uint32_t stiming;        // short clip timing, changed by no comm clip

int CAT_runShortClip( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[32];
    static char fname[32];
    static uint8_t frameNum;

    if( *len != 0)
    {
        CAT_readCmdRingBuf(data, 2);
        *len = data[1] + 2;
        CAT_readCmdRingBuf(data, *len);

        // if stop was requested wait until stopped
        if( stopShortClip == 1 && shortClip_offset != NULL)
            return 0x01;

        // if already displaying this short clip -- go out
        if( data[1] != 0 && !memcmp(fname, &data[2], data[1]) && shortClip_offset != NULL)
            return 0x00;

        if( data[1] != 0)
            memcpy(fname, &data[2], data[1]);
        fname[data[1]] = NULL;
        frameNum = 0;
        stiming = 0;
        stopShortClip = 0;

        // request to stop short clip
        if( data[1] == 0)
        {
            stopShortClip = 1;
            if( catstate != 0x72 && nextCatState != 0x72)
            {
                if( shortClip_offset != NULL)
                    display_free(( void *)shortClip_offset);
                if( shortClip_pInx != NULL)
                    free(shortClip_pInx);
                shortClip_pInx = NULL;
                shortClip_offset = NULL;
                display_endShortClip();
                fname[0] = NULL;
            }
            return 0x00;
        }

        ////////////////////////////////////////////////////
        // if clip is showing ---> hold your horses
        if( thread_getstatus(clip_pid) != STATUS_NOT_FOUND)
            return 0x04;

        if( CAT_showShortClip(fname) != 0x00)
        {
            fname[0] = NULL;
            return 0x04;    /// error!
        }
        else
        {
            data[1] = 0x00;
            if( shouldReply)
                CAT_prepareTxDataAndSend(data, 2);

            return IN_COMMAND_EXECUTION;
        }
    }
    ////////////////////////////////////////
    // COMMAND EXECUTION
    ////////////////////////////////////////
    else
    {   // if clip is showing ---> hold your horses
        if( thread_getstatus( clip_pid) != STATUS_NOT_FOUND)
        {
            free( shortClip_pInx);
            shortClip_pInx = NULL;
            shortClip_offset = NULL;
            fname[0] = NULL;
            return FINISHED_EXECUTING_RAM;
        }

        if( stopShortClip)
        {
            display_free( ( void *)shortClip_offset);
            free( shortClip_pInx);
            shortClip_pInx = NULL;
            shortClip_offset = NULL;
            stopShortClip = 0;
            display_endShortClip();
            fname[0] = NULL;
            return FINISHED_EXECUTING_RAM;
        }
        // just in case....
        if( fname[0] == NULL && shortClip_offset == NULL)
            return FINISHED_EXECUTING_RAM;
        // every call which gets here is 25mSec apart

        AVISTREAMHEADER *avihead = ( AVISTREAMHEADER *)riffParser_GetStreamHeader( 0);
        if( xtimer_now() - stiming < 1000000 / avihead->dwRate /*/ 25*/)    //! in millisecobnds
            return IN_COMMAND_EXECUTION;

#if 1
               display_shortClip( frameNum++, shortClip_pInx, shortClip_offset, (stiming==0));
#endif
        stiming = xtimer_now();

        //!get number of frames from avi header
        if( avihead->dwLength / avihead->dwScale <= frameNum)
            frameNum = 0;
        return IN_COMMAND_EXECUTION;
    } // end else
}



/**
 * @brief starts process of loading a clip with filename from internet server
 * @param len
 * @param shouldReply
 * @return
 * @note
 */
static char load_fname[32], load_destFN[32];
static uint load_checksum;
int CAT_loadClip( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[32];

    CAT_readCmdRingBuf(data, 2);
    *len = data[1] + 2;
    CAT_readCmdRingBuf(data, *len);

    if( data[1] > 32)
        return 0x01;
    memcpy( load_fname, &data[2], data[1] - 4);
    load_checksum = *((uint32_t *)(&data[2 + data[1] - 4]));

    load_fname[data[1] - 4] = NULL;

    if( ethernet_isOngoingDownload() && !doneFileLoading)
        return 0x06;                    // wait some time then try again

    strcpy( load_destFN, load_fname);
    //! if requested to download file exists, use temp extension
    //if( CAT_findFile( fname) == FR_EXIST)

    // always load to TMP, calculate checksum and if correct, copy to original
    replaceExtension( load_destFN, load_fname, (char *)"tmp");

    ////////////////////////////////////////
    ethdemo_getfile( load_fname, load_destFN, CAT_loadClipSuccess, NULL);
    ////////////////////////////////////////
    return 0x00;
}

/**
 * @brief       This is a callback from ETHERNET module, called after file
 *              has finished loading to SD card from web server
 *
 * @return
 */
err_t CAT_loadClipSuccess( void)
{
#define CALC_CHECKSUM
#ifdef CALC_CHECKSUM
    doneFileLoading = false;
    thread_create( (char *)checksum_stack, (int)sizeof(checksum_stack),
                   PRIORITY_IDLE - 1, CREATE_STACKTEST | CREATE_WOUT_YIELD,
                   (thread_task_func_t)CAT_calcFileChecksum, (void *)load_destFN, "checksum_thread");
#endif
    return 0;
}

FRESULT CAT_renameClip( void)
{
FRESULT ret;

    //if( f_unlink( load_fname) == FR_OK)
    f_unlink( load_fname);
    {
        ret = f_rename( load_destFN, load_fname);
        // also dont forget to delete the .inx extension
        char inxfile[32];
        replaceExtension( inxfile, load_destFN, "inx");
        f_unlink( inxfile);
        return ret;
    }
    //return( FR_INVALID_NAME);
}

/**
 * @brief   This is a worker function for a thread to calculate simple
 *          checksum for a file.
 *
 * @param arg - char * filename
 *
 */
void CAT_calcFileChecksum( void *arg)
{
#define BLOCK_SIZE  (512*10)

   char *filename = (char *)arg;
   FRESULT ret = FR_OK;
   uint red, checksum = 0;
   FIL *csrd = (FIL *)malloc( sizeof( FIL));
   uint32_t *data = (uint32_t *)malloc( BLOCK_SIZE);
   FRESULT fr = f_open_adj( csrd, filename, FA_READ);
   DWORD clmt[50];
   //! prepare the cluster table for FAST_SEEK
   /*! Using fast seek feature */
   rd->cltbl = clmt;                                /* Enable fast seek feature (cltbl != NULL) */
   clmt[0] = sizeof( clmt) / sizeof( DWORD);        /* Set table size */
   f_lseek( csrd, CREATE_LINKMAP);               /* Create CLMT */

   f_lseek( csrd, 0);               /* Create CLMT */
   while( true)
   {
       fr = f_fastread( csrd, data, BLOCK_SIZE, &red);
       if( fr == FR_OK)
       {
           // if gotten to end of file, make sure rest of data is zeros
           if( red < BLOCK_SIZE)
           {
               int missing = 4 - red % 4;
               uint8_t *data8 = (uint8_t *)data;
               if( missing == 4) missing = 0;
               for( int y = 0; y < missing; y++)
                   data8[red + y] = NULL;
           }
           for( int x = 0; x < red / sizeof( uint32_t); x++)
               checksum += data[x];
           if( red < BLOCK_SIZE)
               break;
       }
       else
       {
           ret = FR_INVALID_OBJECT;
           break;
       }
   } // endwhile true
   f_close( csrd);
   free( csrd);
   free( data);

   ///////////////////////////////////
   // C H E C K S U M    O  K   ??
   ///////////////////////////////////
   if( checksum == load_checksum)
   {
       // wait until clip is not showing and then rename
       while( thread_getstatus( clip_pid) != STATUS_NOT_FOUND)
           xtimer_usleep( 500000);

       // if checksum is correct....
       // only if both names are not the same (existing file was loaded)
       CAT_renameClip();
   }
   doneFileLoading = true;
   return;
} // endfunc

/**
 * @brief wait for a specified time in milliseconds
 * @param len
 * @param shouldReply
 * @return
 * @note this is a RAM command only!
 */
int CAT_waitTime( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[3];

    CAT_readCmdRingBuf(data, 3);
    *len = 3;
    uint32_t msecs = data[1] | (( uint32_t)data[2] << 8);
    //!!!! currently doesnt work....   xtimer_usleep( 1000*msecs);
    //hwtimer_wait(msecs);
    return 0x00;
}

/**
 * @brief sets display to graphics mode.... and clears display using black color
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_setGraphicsMode( uint16_t *len, uint8_t shouldReply)
{
    *len = 1;
    display_clear();
    return 0x00;
}

/**
 * @brief un-sets display to graphics mode.... CANT DO THAT HERE!@@#
 * @param len
 * @param shouldReply
 * @return
 */
int CAT_unsetGraphicsMode( uint16_t *len, uint8_t shouldReply)
{
    *len = 1;
    return 0x01;
}



/**
 * @brief injects a key sent from PC into keypad routines
 * @param len
 * @param shouldReply
 * @return
 * @note this is a RAM command only!
 */
int CAT_injectKey( uint16_t *len, uint8_t shouldReply)
{
    uint8_t data[3];

    CAT_readCmdRingBuf(data, 2);
    *len = 2;
    uint8_t kkk = data[1];
    //CAT_keyPressed( kkk);
    //
    // inject directly to the sw3501b... epp
    sw3501b_injectKey( kkk);
    return 0x00;
}



/*****************************************************************************/

/**
 * @brief This function prepares raw data for transmission over to the POS
 *        manager
 * @param buff   - buffer holding the data
 * @param length - byte length of data to be sent
 * @return
 */
static int CAT_prepareTxDataAndSend( uint8_t *buff, uint32_t length)
{
    uint16_t crc = 0xFFFF;
    crc = CAT_CRC16( SYNC, crc);
    crc = CAT_CRC16(myAddress, crc);
    ringbuffer_add_one(&tx_ringbuf, SYNC);

    if( myAddress == SF || myAddress == DLE || myAddress == SYNC)
        ringbuffer_add_one(&tx_ringbuf, DLE);
    ringbuffer_add_one(&tx_ringbuf, myAddress);

    if( length >= 256)
        while(1);
    for( uint32_t i = 0; i < length; i++)
    {
        crc = CAT_CRC16(buff[i], crc);
        // add DLE if have SF, DLE or SYNC in data
        if( buff[i] == SF || buff[i] == DLE || buff[i] == SYNC)
            ringbuffer_add_one(&tx_ringbuf, DLE);
        ringbuffer_add_one(&tx_ringbuf, buff[i]);
    } // endfor
    crc = CAT_CRC16( SF, crc);
    ringbuffer_add_one(&tx_ringbuf, SF);

    uint8_t crcb = crc & 0xFF;
    if( crcb == SF || crcb == DLE || crcb == SYNC)
        ringbuffer_add_one(&tx_ringbuf, DLE);
    ringbuffer_add_one(&tx_ringbuf, crcb);
    crcb = (crc & 0xFF00) >> 8;
    if( crcb == SF || crcb == DLE || crcb == SYNC)
        ringbuffer_add_one(&tx_ringbuf, DLE);
    ringbuffer_add_one(&tx_ringbuf, crcb);

    uint8_t testval[256];
    int rx = ringbuffer_get( &tx_ringbuf, (char *)testval, sizeof( testval));

#ifdef USE_ETHERNET
    ethernet_CAT_send( testval, rx);
#else
    // start the process rolling
    uart_tx_begin( CAT_COMM_UART);
#endif
    return 1;
}

//------------------------------------------------------------------------//
// Function: CRC16
//
// Scope: This functioncalculatestheCRCforasingle byte of data and
// combinesitwith thecumulativeCRC.
//
// Args: data. Data byteonwhichCRCisbeingcalculated.
// crc. CumulativeCRCof entireCATmessage.
//
// Globals: none
//
// Returns: crc. ResultofCRCof dataor'dwithcumulative CRC.
//
uint16_t CAT_CRC16( uint8_t data, uint16_t crc)
{
    int i;
    unsigned int feedback;
    for( i = 0; i < 8; i++)
    {
        if( (data ^ crc) & 1)
            feedback = 0xa001;
        else
            feedback = 0;
        data >>= 1;
        crc >>= 1;
        crc ^= feedback;
    }
    return (crc);
}


/**
 * @brief   Returns the terminal's address. There can be only one terminal
 *          using that address on the local network
 * @return  byte address
 */
extern "C" uint8_t CAT_getMyAddress(void)
{
    return myAddress;
}


/**
 * @brief This function initializes the CAT module
 */
static void CAT_initialize( void)
{
    powerFailFlag = 1;
    runningFromRAM = 0;
    cardRetryLimitReached = 0;
    beepDuration = 100;              // 100msec
    ramLoc = 0;
    lastReportedStatus = 0;
    doneFileLoading = true;
    rxstate = STATE_RESTART;
    DLEreceived = rxError = 0;
    myAddress = CAT_DEFAULT_ADDRESS;    // this should be assigned via user...
    cardInsertedTimeoutReached = 0;
    shortClip_offset = 0;               // holds the display malloc for short clips
    nextCatState = -1;
    noCommCounter = 90;
    nc_frameNum = NO_COMM_FIRST_TIME;   // no comm video is not showing
    menuStatus = 0;

    for( int i = 0; i < MAX_RAM_SLOTS; i++)
    {
        ramData[i].data = NULL;
        ramData[i].length = 0;
        ramData[i].currentInx = 0;
    } // endfor reset RAM data

    ringbuffer_init(&tx_ringbuf, txBuffer, CAT_TX_BUFFER_SZ);
    ringbuffer_init(&cmd_ringbuf, rxBuffer, CAT_RX_BUFSIZE);
    ringbuffer_init(&keypad_ringbuf, ( char *)keypadBuffer,
            sizeof(keypadBuffer));

    // debug test temp
    adc_init( ADC_0, ADC_RES_12BIT);
    ADC->CCR |= (uint32_t)ADC_CCR_TSVREFE;
} // endfunc

static void CAT_inject( char *data, int len)
{
    int i;
    uint8_t acrc[2];
    uint16_t _calcCRC = 0xFFFF;

    if( len == 0)
        len = strlen(data);

    for( i = 0; i < len - 2; i++)
    {
        _calcCRC = CAT_CRC16(( uint8_t)data[i], _calcCRC);
    }
    acrc[0] = _calcCRC & 0xFF;
    acrc[1] = (_calcCRC & 0xFF00) >> 8;

    for( int i = 0; i < len - 2; i++)
    {
        CAT_rxbyte( NULL, data[i]);
    }
    CAT_rxbyte( NULL, acrc[0]);
    CAT_rxbyte( NULL, acrc[1]);
}

/**
 * @brief This function decompresses a black & white bitmaped data received
 *        from the master
 * @param ucmsg     - uncompressed message
 * @param uclen     - [in]  sizeof ucmsg in bytes
 *                    [out] length of uncompressed message
 * @param cmsg      - compressed message
 * @param clen      - length of compressed message
 * @param DLEchar   - DLE character
 * @param cchar     - compression character
 * @return number of bytes taken from compressed cmsg
 *
 * @note  If ucmsg buffer is smaller than required the function would take
 *        as many characters as can fit into it and return the number of chars
 *        used from cmsg. Use this function again with &cmsg[return val] to
 *        continue decompression from where it stopped last time.
 */
static uint32_t CAT_decompress( char *ucmsg, uint32_t *uclen, char *cmsg,
        uint32_t clen, char DLEchar, char cchar)
{
    uint32_t inx = 0, leftlen = *uclen, p, llen = *uclen;

    for( p = 0; p < clen; p++)
    {   // if compressed char received?
        if( cmsg[p] == cchar)
        {   // if needs more bytes in uc memory than have...
            p++;
            if( (uint8_t)cmsg[p + 1] > leftlen)
                return (p - 1);
            for( int k = 0; k < ( int)cmsg[p + 1]; k++)
                ucmsg[inx++] = cmsg[p];
            p++;
        } // endif compress character
        else if( cmsg[p] == DLEchar)
        {
            p++;
            ucmsg[inx++] = cmsg[p];
        }
        else
            ucmsg[inx++] = cmsg[p];

        leftlen = llen - inx;
        *uclen = inx;
        if( leftlen <= 0)
            return p;
    } // endfor all compressed msg
    return p;
} // endfunc

/**********************************************/
/*   C A L L B A C K      F U N C T I O N S   */
/**********************************************/
void CAT_keyPressed( char buttonID)
{
    char keybuffer[KEYPAD_BUF_SIZE], tempBuffer[KEYPAD_BUF_SIZE];
    static uint32_t lastKey;
#define ANTI_BOUNCE_TIME    0               //was 100000
    /*
     * Note that currently if keypad is locked - no more keys are
     * allowed to enter this function
     */
    if( keypadConfiguration.keypadLocked == 1)
        return;

    //------- ANTI BOUNCE --------
    if( xtimer_now() - lastKey < ANTI_BOUNCE_TIME)
        return;
    lastKey = xtimer_now();
    //============================

    if( RESTART_DISPLAY_KEY == buttonID)
    {
        ringbuffer_remove( &keypad_ringbuf, KEYPAD_BUF_SIZE);

        if( keypadConfiguration.initialTextAlignment & ALIGN_H_RIGHT)
            display_setRightText( TXB_ID_1);
        else
            display_setLeftText( TXB_ID_1);

        CAT_scanForCommandsAndShow( keypadConfiguration.initialText, 0, FIRST_LINE);
        return;
    }


    //! mark that key is pressed after last return
    keypadConfiguration.keyPressedSinceReturn++;
    keypadConfiguration.keyPressedSinceStatus++;

    if( keypadConfiguration.clearKey == buttonID)
    {
        unsigned cnt = ringbuffer_peek(&keypad_ringbuf, tempBuffer,
                KEYPAD_BUF_SIZE);
        if( cnt == 0)
            keypadConfiguration.clearPressedOnEmptyBuf++;
        //! REMOVE ALL AFTER CLEAR!
        ringbuffer_remove( &keypad_ringbuf, KEYPAD_BUF_SIZE);
/*
        if( keypadConfiguration.initialTextAlignment & ALIGN_H_RIGHT)
            display_setRightText( TXB_ID_1);
        else
            display_setLeftText( TXB_ID_1);

        CAT_scanForCommandsAndShow( keypadConfiguration.initialText, 0, FIRST_LINE);
*/
        keypadConfiguration.clearPressed = 1;
        // beep
        // beep_on( beepDuration);
        CAT_soundBuzzer( beepDuration);
        //return;
    }
    if( keypadConfiguration.lockKeys[buttonID])
    {
        keypadConfiguration.keypadLocked = 1;

        // stop EPP plaintext
        if( epp_getError() == EPP_OK)
            epp_abortPlaintext();

        CAT_soundBuzzer( beepDuration);
        display_removeFocus();
        return;
    }
    if( keypadConfiguration.illegalKeys[buttonID])
    {
        // beep beep
        // beep_on( beepDuration);
        CAT_soundBuzzer( beepDuration);
        return;
    }
    //! ignore?
    if( keypadConfiguration.ignoreKeys[buttonID])
        return;
    if( keypadConfiguration.shouldBeep)
        CAT_soundBuzzer( beepDuration);

    // new NS as per request from amnon -- deal with clear as delete on char
    if( 11 == buttonID)
    {
        unsigned cnt = ringbuffer_peek(&keypad_ringbuf, tempBuffer,
                KEYPAD_BUF_SIZE);
        if( cnt == 0)
            keypadConfiguration.clearPressedOnEmptyBuf++;
        //! REMOVE ALL AFTER CLEAR!
        ringbuffer_remove_tail( &keypad_ringbuf, 1);
        // beep
        // beep_on( beepDuration);
        CAT_soundBuzzer( beepDuration);
        //return;
    }
    else
    {
        // convert buttonID to key value for keypad buffer
        if( buttonID != keypadConfiguration.clearKey)
            ringbuffer_add_one(&keypad_ringbuf, buttonID);
    }

    // next, read everything to a local temp buffer
    unsigned cnt = ringbuffer_peek(&keypad_ringbuf, tempBuffer,
            KEYPAD_BUF_SIZE);

    // convert all keys in ringbuffer to displayable chars
    unsigned inx = 0;
    for( unsigned i = 0; i < cnt; i++)
    {
        uint8_t key = tempBuffer[i];
        if( keypadDefinition[key].displayValue == 0)
            continue;
        tempBuffer[inx++] = keypadDefinition[key].displayValue;
    } // endfor

    inx = CAT_buildKeypadBuf(inx, tempBuffer, keybuffer, '0', '.');
    keybuffer[inx] = ( char)NULL;
    display_setRightText( TXB_ID_1);
    CAT_scanForCommandsAndShow( keybuffer, inx, SECOND_LINE);        //! non alarm, just scanning the key buffer for special commands

    //! Check if should LOCK?
    if( keypadConfiguration.keyCountToLock != 0
            && keypadConfiguration.keyCountToLock <= inx)
    {
        keypadConfiguration.keypadLocked = 1;

        // stop EPP plaintext
        if( epp_getError() != EPP_OK)
            epp_abortPlaintext();
    }
} // endfunc

/**
 * @brief builds the keybuffer from tempBuffer with key hits according to
 *        keypadConfiguration structure
 * @param inx       - number of characters in tempBuffer
 * @param tempBuffer- temporary key strokes buffer
 * @param keybuffer - return key buffer formatted
 * @param zero      - zero character
 * @param decimalpt - decimal point character
 * @return number of bytes in keybuffer (NOT including a terminating NULL)
 */
int CAT_buildKeypadBuf( unsigned inx, char *tempBuffer, char *keybuffer,
        char zero, char decimalpt)
{
    unsigned cnt = inx;

    inx = 0;
    // if have a decimal point somewhere inside
    if( keypadConfiguration.decimalPointLoc > 0)
    {   // if dealing with first character
        if( cnt <= keypadConfiguration.decimalPointLoc)
        {
            keybuffer[0] = zero /*'0'*/;
            keybuffer[1] = decimalpt /*'.'*/;
            inx = 2;
            for( uint i = 0; i < keypadConfiguration.decimalPointLoc - cnt; i++)
                keybuffer[inx++] = zero /*'0'*/;
            for( int i = 0; i < cnt; i++)
                keybuffer[inx++] = tempBuffer[i];

        } // endif <= decimal points
        else // > decimal points
        {
            for( int i = 0; i < cnt - keypadConfiguration.decimalPointLoc; i++)
            {
                keybuffer[inx++] = tempBuffer[i];
            } // endfor
            keybuffer[inx++] = decimalpt /*'.'*/;
            for( int i = cnt - keypadConfiguration.decimalPointLoc; i < cnt;
                    i++)
                keybuffer[inx++] = tempBuffer[i];
        } // endelse > decimal points
    }
    else
    {
        for( int i = 0; i < cnt; i++)
        {
            keybuffer[inx++] = tempBuffer[i];
        } // endfor all keys
    }
    return inx;
}

/**
 * @brief Starts the clip thread for showing a clip
 * @return pid of the new thread (major only - clip thread)
 *
 * @note In the future to transfer the filename to the clip
 */
kernel_pid_t CAT_runClipThread( char *filename)
{
static clipArguments_t clipargs;
    // debug:
    //LTDC_Cmd( DISABLE);

    if( thread_getstatus( clip_pid) == STATUS_NOT_FOUND)
    {
        clipargs.filename = filename;
        clipargs.CAT_notifyDoneCB = CAT_notifyClipThreadDone;

        /* Init Clip Thread */
        clip_pid = thread_create( (char *)clip_stack, sizeof( clip_stack),
                   PRIORITY_MAIN /* was MAIN - 2 NS 02-10 */ , CREATE_STACKTEST,
                   (thread_task_func_t)clip_Prepare, ( void *)&clipargs, "clip");
        puts("clip thread created");
        return clip_pid;
    }
    else
        return STATUS_NOT_FOUND;
}

/**
 * @brief   A callback function for clip thread to notify the it is done
 *
 */
void CAT_notifyClipThreadDone( void)
{
    clip_pid = KERNEL_PID_UNDEF;
}



/**
 * @brief makes sure that the keypad is displayed
 */
void CAT_showKeypad( void)
{
    // stop short clip in any case... (if PC forgot)
    CAT_stopShortClip();
#if 0
    //! stops clip thread if active
    clip_StopAndKillThreads();
    while( thread_get( clip_pid) != NULL)
#ifdef HWTIMER
        hwtimer_wait( 10000);
#else
    xtimer_usleep( 10000);
#endif
#endif
    // wait so that the short clip is canceled... ?!?!?!?
    //xtimer_usleep( 100000);

    //display_clear();
    display_window2();
    //display_periodic( NULL);                        // update the keypad window

    // at this time also try to start taking keys from the EPP
    if( epp_getPlaintext( 0) == EPP_OK)
    {
    }
}




/**
 *
 * @return 32 bit return value
 */
uint32_t CAT_buildStatus( void)
{
uint8_t data[3] = { 0, 0, 0};
static uint8_t prnstatus;
static uint32_t lastGetPrinterStatus = 0;
uint32_t tmptime;

#define CARD_IN_READER      (0x10)      // on byte #1
#define CARD_INSERTED       (0x80)
#define KEYPAD_LOCKED       (0x40)
#define HAVE_VALID_TRACK    (0x02)
#define POWER_FAIL          (0x20)
#define PAPER_LOW           (0x10)
#define PRINTER_OUT         (0x80)

    int t1 = card->getTrack1( NULL);
    int t2 = card->getTrack2( NULL);
    int tinserted = card->isCardInserted();

    // if showing menu--> stop returning card status to PC
    if( menuStatus == 1)
        t1 = t2 = tinserted = 0;

    uint8_t haveValidTrack = 0;

    if( /*t1 ||*/ t2)
        data[0] |= HAVE_VALID_TRACK;
    if( tinserted == 1)
        data[0] |= CARD_INSERTED;
    if( tinserted == 2)
        data[1] |= CARD_IN_READER;
    if( keypadConfiguration.keypadLocked)
        data[0] |= KEYPAD_LOCKED;
    if( powerFailFlag)
        data[0] |= POWER_FAIL;

    // read printer status only every 10 seconds
    if( (tmptime = xtimer_now()) - lastGetPrinterStatus > 10000000 ||
            lastGetPrinterStatus == 0)
    {
        prnstatus = printer_getStatus();
        lastGetPrinterStatus = tmptime;
    }
    /* @return 8 bit printer status (see .h file)
     *          STATUS_OFFLINE          - printer is offline
     *          STATUS_NOPAPER          - no paper
     *          STATUS_BAD_COMM         - could not get response from printer
     *          STATUS_COVER_OPEN       - printer cover is open
     *          STATUS_PRINTER_ERROR    - general error
     *          STATUS_MECHANISM_ERROR  - may be generated by opening cover
     *          STATUS_CUTTER_ERROR     - may be caused by paper jam
     */
    if( prnstatus & STATUS_NOPAPER)
        data[0] |= PAPER_LOW;
    if( (prnstatus & STATUS_OFFLINE) || (prnstatus & STATUS_PRINTER_ERROR)
            || (prnstatus & STATUS_MECHANISM_ERROR)
            || (prnstatus & STATUS_CUTTER_ERROR)
            || (prnstatus & STATUS_BAD_COMM))
        data[1] |= PRINTER_OUT;

    // is showing clip now?
    if( thread_getstatus( clip_pid) != STATUS_NOT_FOUND ||
         ( shortClip_offset != NULL && shortClip_pInx != NULL && nc_frameNum == NO_COMM_FIRST_TIME))
        data[1] |= 0x01;

    if( keypadConfiguration.clearPressed)
        data[0] |= 0x04;

    if( keypadConfiguration.keyPressedSinceReturn)
        data[0] |= 0x01;

    if( cardInsertedTimeoutReached)
        data[1] |= 0x20;
#if 0
    if( keypadConfiguration.keyPressedSinceStatus)
        data[1] |= 0x10;
    keypadConfiguration.keyPressedSinceStatus = 0;
//#else
#endif

    if( cardRetryLimitReached)
        data[0] |= 0x04;

    if( keypadConfiguration.clearPressedOnEmptyBuf)
        data[1] |= 0x40;

    // build the 3rd byte (for 24 bit status)
    if( prnstatus & STATUS_NOPAPER)
        data[2] = ( 1) << 5;
    else if( prnstatus & STATUS_COVER_OPEN)
        data[2] = ( 2) << 5;
    else if( prnstatus & STATUS_OFFLINE)
        data[2] = ( 3) << 5;
    else if( prnstatus & STATUS_MECHANISM_ERROR)
        data[2] = ( 4) << 5;
    else if( prnstatus & STATUS_CUTTER_ERROR)
        data[2] = ( 5) << 5;
    else // online and OK
        data[2] = ( 0) << 5;
    // is ram execution now?
    if( ramLoc != 0)
        data[2] |= ( 1 << 2);

    uint32_t ret = data[0] | ( data[1] << 8) | ( data[2] << 16);

    lastReportedStatus = ret;

    return ret;
}

/**
 * @brief This function scans and replaces commands from table at page
 *        4-4 and 4-5 of the display
 *
 * @param[in/out]   Text    string to be checked and then shown
 * @param[in/out]   cnt     count of string
 *
 */
#define SET_CURSOR_ABS          0x01
#define SET_CURSOR_ABS_PIXEL    0x02
#define SET_CURSOR_REL_PIXEL    0x03
#define DISPLAY_ON              0x04
#define DISPLAY_OFF             0x05
#define CURSOR_ON               0x06
#define CURSOR_OFF              0x07
#define LINE_FEED               0x0A
#define CARRIAGE_RETURN         0x0D
#define LF_CR                   0x0E
#define CLEAR_DISPLAY           0x0F
#define REVERSE_VIDEO           0x11
#define REVERSE_VIDEO_OFF       0x12
#define CURSOR_BLINK_ON         0x13
#define CURSOR_BLINK_OFF        0x14
#define SELECT_ROM_BASED_CHAR   0x15            // select ROM based character set
#define HOME_CURSOR             0x17
#define SELECT_16X2             0x1A            // select 16x2 LCD emulation
#define SET_CURSOR_X_Y          0x1B
#define SOUND_BEEPER            0x1D            // sound beeper
#define DISLPAY_FLASH_ON        0x1E
#define DISLPAY_FLASH_OFF       0x1F

void CAT_scanForCommandsAndShow( char *Text, uint8_t cnt, int id)
{
static char txt[256];
int inx = 0;

    if( cnt == 0)
        cnt = strlen( Text);

    for( int i = 0; i < cnt; i++)
    {
        switch( Text[i])
        {
            case SET_CURSOR_ABS:
            case SET_CURSOR_ABS_PIXEL:
            case SET_CURSOR_REL_PIXEL:
            case DISPLAY_ON:
            case DISPLAY_OFF:
            case CURSOR_ON:
            case CURSOR_OFF:
            case CLEAR_DISPLAY:
            case REVERSE_VIDEO:
            case REVERSE_VIDEO_OFF:
            case CURSOR_BLINK_ON:
            case CURSOR_BLINK_OFF:
            case HOME_CURSOR:
            case SELECT_16X2:
            case DISLPAY_FLASH_ON:
            case DISLPAY_FLASH_OFF:
                continue;           // just drop these chars for now

            case SET_CURSOR_X_Y:
                i += 2;             // remove the X and Y from the TEXT
                continue;

            case LINE_FEED:
            case CARRIAGE_RETURN:
            case LF_CR:
                display_showText( txt, inx, id);
                inx = 0;
                id++;
                //txt[inx++] = '\n';
                continue;

            case SELECT_ROM_BASED_CHAR:
                // change font here...
                continue;

            case SOUND_BEEPER:
                // get next char as duration and sound beeper
                i += 1;
                CAT_soundBuzzer( beepDuration);
                continue;

            default:
                txt[inx++] = Text[i];
                // if size is bigger than allocated truncate
                if( inx > sizeof( txt) - 4)
                    inx--;
                break;
        } // endswitch
    } // endfor all characters in Text

    display_showText( txt, inx, id);
    return;
}


#if 0
void CAT_playWav( void)
{

}

#include "stm32f4xx_tim.h"

void init_gpio(void) {

    GPIO_InitTypeDef gpio_init;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    gpio_init.GPIO_Mode = GPIO_Mode_AN;
    gpio_init.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_init.GPIO_Speed = GPIO_Low_Speed; //GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &gpio_init);

    //dac_init( DAC_0, DAC_RES_12BIT);
}
uint8_t *DAC_Buff;
void init_timer(void) {
    TIM_TimeBaseInitTypeDef tim_init;
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM6, ENABLE);
    tim_init.TIM_CounterMode = TIM_CounterMode_Up;
    tim_init.TIM_Period = 1705 - 1; //22*180 - 1; //1705 - 1;
    tim_init.TIM_Prescaler = 0;
    TIM_TimeBaseInit( TIM6, &tim_init);
    TIM_SelectOutputTrigger( TIM6, TIM_TRGOSource_Update);
    //TIM_Cmd(TIM6, ENABLE);
    TIM_Cmd( TIM6, DISABLE);
}
void init_dac(void) {
    DAC_InitTypeDef dac_init;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    DAC_StructInit(&dac_init);
    dac_init.DAC_Trigger = DAC_Trigger_T6_TRGO;
    dac_init.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
    dac_init.DAC_WaveGeneration = DAC_WaveGeneration_None;
    DAC_Init(DAC_Channel_1, &dac_init);
}

void init_dma(void) {
    DMA_InitTypeDef dma_init;
    RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_DMA1, ENABLE);
    DMA_DeInit( DMA1_Stream5);
    dma_init.DMA_Channel = DMA_Channel_7;
    dma_init.DMA_PeripheralBaseAddr = (uint32_t)(DAC_BASE + 0x10);// should be DHR12R1 ?
    dma_init.DMA_Memory0BaseAddr = (uint32_t)DAC_Buff;
    dma_init.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    dma_init.DMA_BufferSize = 512;
    dma_init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma_init.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    dma_init.DMA_MemoryDataSize = DMA_PeripheralDataSize_Word;
    dma_init.DMA_Mode = DMA_Mode_Circular;
    dma_init.DMA_Priority = DMA_Priority_High;
    dma_init.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma_init.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    dma_init.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma_init.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

    DMA_Init(DMA1_Stream5, &dma_init);
    DMA_Cmd(DMA1_Stream5, ENABLE);
    DAC_Cmd(DAC_Channel_1, ENABLE);
    DAC_DMACmd(DAC_Channel_1, ENABLE);
//  DAC_ITConfig(DAC_Channel_1, DMA_SxCR_HTIE,ENABLE);
//  DAC_ITConfig(DAC_Channel_1, DMA_SxCR_TCIE,ENABLE);
}



void *CAT_soundOff( void *arg)
{

    gpio_clear( GPIO_21);
    return NULL;



    TIM_Cmd(TIM6, DISABLE);
    DAC_Cmd(DAC_Channel_1, DISABLE);
    DAC_Cmd(DAC_Channel_2, DISABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
}

void CAT_soundBuzzer( uint32_t msec)
{

    // testing...
    return;


#if 0
    gpio_init_out( GPIO_27, GPIO_PULLUP);
    gpio_set( GPIO_27);

    test_startSound( msec);
    return;


    DAC_InitTypeDef            DAC_InitStructure;
    TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;



    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    /* DAC Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
    /* TIM2 Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    init_gpio();

    /* TIM2 Configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = 0x1;
  TIM_TimeBaseStructure.TIM_Prescaler = 0xA;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

  /* TIM2 TRGO selection */
  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

  /* DAC channel1 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_Triangle;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_2047;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  /* DAC channel2 Configuration */
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_1023;
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  /* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
   automatically connected to the DAC converter. */
  DAC_Cmd(DAC_Channel_1, ENABLE);

  /* Enable DAC Channel2: Once the DAC channel2 is enabled, PA.05 is
   automatically connected to the DAC converter. */
  DAC_Cmd(DAC_Channel_2, ENABLE);

  /* Set DAC dual channel DHR12RD register */
  DAC_SetDualChannelData(DAC_Align_12b_R, 0x100, 0x100);

  /* TIM2 enable counter */
  TIM_Cmd(TIM6, ENABLE);

  if( xtimer_callmeback( msec * 1000, CAT_soundOff) == -1)
  {   // if no available timers
      xtimer_usleep( msec * 1000);
      CAT_soundOff( 0);
  }
#endif
} // endfunc


/* Exported types ------------------------------------------------------------*/
typedef enum
{
  LittleEndian,
  BigEndian
}Endianness;

typedef struct
{
  uint32_t  RIFFchunksize;
  uint16_t  FormatTag;
  uint16_t  NumChannels;
  uint32_t  SampleRate;
  uint32_t  ByteRate;
  uint16_t  BlockAlign;
  uint16_t  BitsPerSample;
  uint32_t  DataSize;
}
WAVE_FormatTypeDef;

typedef enum
{
  Valid_WAVE_File = 0,
  Invalid_RIFF_ID,
  Invalid_WAVE_Format,
  Invalid_FormatChunk_ID,
  Unsupporetd_FormatTag,
  Unsupporetd_Number_Of_Channel,
  Unsupporetd_Sample_Rate,
  Unsupporetd_Bits_Per_Sample,
  Invalid_DataChunk_ID,
  Unsupporetd_ExtraFormatBytes,
  Invalid_FactChunk_ID
} ErrorCode;

static ErrorCode WavePlayer_WaveParsing(uint32_t *FileLen);
uint32_t ReadUnit(uint8_t *buffer, uint8_t idx, uint8_t NbrOfBytes, Endianness BytesFormat);

/* Exported constants --------------------------------------------------------*/
#define  CHUNK_ID                            0x52494646  /* correspond to the letters 'RIFF' */
#define  FILE_FORMAT                         0x57415645  /* correspond to the letters 'WAVE' */
#define  FORMAT_ID                           0x666D7420  /* correspond to the letters 'fmt ' */
#define  DATA_ID                             0x64617461  /* correspond to the letters 'data' */
#define  FACT_ID                             0x66616374  /* correspond to the letters 'fact' */
#define  LIST_ID                             0x4c495354  /* correspond to the letters 'LIST' */
#define  WAVE_FORMAT_PCM                     0x01
#define  FORMAT_CHNUK_SIZE                   0x10
#define  CHANNEL_MONO                        0x01
#define  CHANNEL_STEREO                      0x02
#define  SAMPLE_RATE_8000                    8000
#define  SAMPLE_RATE_11025                   11025
#define  SAMPLE_RATE_22050                   22050
#define  SAMPLE_RATE_44100                   44100
#define  BITS_PER_SAMPLE_8                   8
#define  BITS_PER_SAMPLE_16                  16


WAVE_FormatTypeDef WAVE_Format;
uint16_t buffer1[_MAX_SS] = { 0x00 };
static __IO uint32_t SpeechDataOffset = 0x00;
__IO uint32_t WaveCounter;




char wave_playback(const char *FileName)
{
  FRESULT res;                                //הכ גמחגנאשאולמדמ פףםךצטלט נוחףכעאעא
  UINT cnt;                                   //ךמכטקוסעגמ נואכםמ ןנמקטעאםםץ באיע
  uint32_t wavelen = 0;

  FIL *file = (FIL *)malloc( sizeof( FIL));
  DAC_Buff = (uint8_t *)malloc( 512*2);

  init_gpio();
  init_timer();
  init_dac();
  init_dma();



  /* Open the wave file to be played */
  if( f_open( file, FileName , FA_READ) != FR_OK)
  {
    return 0x00;
  }
  else
  {
    /* Read data(_MAX_SS byte) from the selected file */
    f_read( file, buffer1, _MAX_SS, &cnt);

    ErrorCode WaveFileStatus = WavePlayer_WaveParsing( &wavelen);
    #if 0
    if (WaveFileStatus == Valid_WAVE_File)  /* the .WAV file is valid */
    {
      /* Set WaveDataLenght to the Speech wave length */
      WaveDataLength = WAVE_Format.DataSize;
    }
    else /* Unvalid wave file */
    {
      /* Led Red Toggles in infinite loop */
      while(1)
      {
        STM_EVAL_LEDToggle(LED5);
        Delay(10);
      }
    }
    /* Play the wave */
    WavePlayBack(WAVE_Format.SampleRate);
    #endif
  }






  res = f_open( file, FileName, FA_READ );   //מעךנע פאיכ FileName הכ קעוםט
  if( res) return 1;
  res = f_lseek( file, WaveCounter /*0x2c*/);                  //ןונולוסעטע ףךאחאעוכ םא םאקאכמ ןמכוחםץ האםםץ
  if(res) return 2;
  f_read( file, &DAC_Buff[0], 512*2, &cnt);       // bitdepth 16
  if(res) return 3;
  TIM_Cmd(TIM6, ENABLE);
  //TIM6->CR1 |= TIM_CR1_CEN;                   //חאןףסעטע ןנומבנאחמגאםטו

  while(1)
  {
          int tt1 = xtimer_now();

         volatile ITStatus it_st;
         it_st = RESET;
         while(it_st == RESET)
         {   //זהול מסגמבמזהוםטו ןונגמי קאסעט בףפונא
             it_st = DMA_GetFlagStatus( DMA1_Stream5, DMA_FLAG_HTIF5);
         }
         f_read( file, &DAC_Buff[0], 256*2, &cnt);    //חאדנףחטע וו האםםלט
         DMA_ClearFlag( DMA1_Stream5, DMA_FLAG_HTIF5);//סבנמסטע פכאד
         if( cnt < 256) break;                        //וסכט ךמםוצ פאיכא

         it_st = RESET;
         while(it_st == RESET) {   //זהול מסגמבמזהוםטו ןונגמי קאסעט בףפונא
             it_st = DMA_GetFlagStatus(DMA1_Stream5, DMA_FLAG_TCIF5);
         }
         f_read( file, &DAC_Buff[512], 512, &cnt);  //חאדנףחטע וו האםםלט
         DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 );  //סבנמסטע פכאד
         if( cnt < 512) break;                        //וסכט ךמםוצ פאיכא

         int tt2 = xtimer_now() - tt1;
         tt2++;

     /*

     DMA1->IFCR |= DMA_ISR_HTIF3;             //סבנמסטע פכאד


     while(!(DMA1->ISR & DMA_ISR_TCIF3)) {}   //זהול מסגמבמזהוםטו געמנמי קאסעט בףפונא
     f_read (&file,&DAC_Buff[256],256,&cnt);  //חאדנףחטע וו האםםלט
     DMA1->IFCR |= DMA_ISR_TCIF3;             //סבנמסטע פכאד
     if(cnt<256)break;                        //וסכט ךמםוצ פאיכא
  */


  } // endwhile

 // TIM6->CR1 &= ~TIM_CR1_CEN;                  //מסעאםמגטע ןנומבנאחמגאםטו
  TIM_Cmd( TIM6, DISABLE);                       //מסעאםמגטע ןנומבנאחמגאםטו
  f_close( file);                             //חאךנע פאיכ
  free( file);
  free( DAC_Buff);

  return 0;                                   //ףסןורםמו חאגונרוםטו פ-טט
}




/**
  * @brief  Checks the format of the .WAV file and gets information about
  *   the audio format. This is done by reading the value of a
  *   number of parameters stored in the file header and comparing
  *   these to the values expected authenticates the format of a
  *   standard .WAV  file (44 bytes will be read). If  it is a valid
  *   .WAV file format, it continues reading the header to determine
  *   the  audio format such as the sample rate and the sampled data
  *   size. If the audio format is supported by this application,
  *   it retrieves the audio format in WAVE_Format structure and
  *   returns a zero value. Otherwise the function fails and the
  *   return value is nonzero.In this case, the return value specifies
  *   the cause of  the function fails. The error codes that can be
  *   returned by this function are declared in the header file.
  * @param  None
  * @retval Zero value if the function succeed, otherwise it return
  *         a nonzero value which specifies the error code.
  */
static ErrorCode WavePlayer_WaveParsing(uint32_t *FileLen)
{
  uint32_t temp = 0x00;
  uint32_t extraformatbytes = 0;

  /* Read chunkID, must be 'RIFF' */
  temp = ReadUnit((uint8_t*)buffer1, 0, 4, BigEndian);
  if (temp != CHUNK_ID)
  {
    return(Invalid_RIFF_ID);
  }

  /* Read the file length */
  WAVE_Format.RIFFchunksize = ReadUnit((uint8_t*)buffer1, 4, 4, LittleEndian);

  /* Read the file format, must be 'WAVE' */
  temp = ReadUnit((uint8_t*)buffer1, 8, 4, BigEndian);
  if (temp != FILE_FORMAT)
  {
    return(Invalid_WAVE_Format);
  }

  /* Read the format chunk, must be'fmt ' */
  temp = ReadUnit((uint8_t*)buffer1, 12, 4, BigEndian);
  if (temp != FORMAT_ID)
  {
    return(Invalid_FormatChunk_ID);
  }
  /* Read the length of the 'fmt' data, must be 0x10 -------------------------*/
  temp = ReadUnit((uint8_t*)buffer1, 16, 4, LittleEndian);
  if (temp != 0x10)
  {
    extraformatbytes = 1;
  }
  /* Read the audio format, must be 0x01 (PCM) */
  WAVE_Format.FormatTag = ReadUnit((uint8_t*)buffer1, 20, 2, LittleEndian);
  if (WAVE_Format.FormatTag != WAVE_FORMAT_PCM)
  {
    return(Unsupporetd_FormatTag);
  }

  /* Read the number of channels, must be 0x01 (Mono) or 0x02 (Stereo) */
  WAVE_Format.NumChannels = ReadUnit((uint8_t*)buffer1, 22, 2, LittleEndian);

  /* Read the Sample Rate */
  WAVE_Format.SampleRate = ReadUnit((uint8_t*)buffer1, 24, 4, LittleEndian);

  /* Read the Byte Rate */
  WAVE_Format.ByteRate = ReadUnit((uint8_t*)buffer1, 28, 4, LittleEndian);

  /* Read the block alignment */
  WAVE_Format.BlockAlign = ReadUnit((uint8_t*)buffer1, 32, 2, LittleEndian);

  /* Read the number of bits per sample */
  WAVE_Format.BitsPerSample = ReadUnit((uint8_t*)buffer1, 34, 2, LittleEndian);
  if (WAVE_Format.BitsPerSample != BITS_PER_SAMPLE_16)
  {
    return(Unsupporetd_Bits_Per_Sample);
  }
  SpeechDataOffset = 36;
  /* If there is Extra format bytes, these bytes will be defined in "Fact Chunk" */
  if (extraformatbytes == 1)
  {
    /* Read th Extra format bytes, must be 0x00 */
    temp = ReadUnit((uint8_t*)buffer1, 36, 2, LittleEndian);
    if (temp != 0x00)
    {
      return(Unsupporetd_ExtraFormatBytes);
    }
    /* Read the Fact chunk, must be 'fact' */
    temp = ReadUnit((uint8_t*)buffer1, 38, 4, BigEndian);
    if (temp != FACT_ID && temp != LIST_ID)
    {
      return(Invalid_FactChunk_ID);
    }
    /* Read Fact chunk data Size */
    temp = ReadUnit((uint8_t*)buffer1, 42, 4, LittleEndian);

    SpeechDataOffset += 10 + temp;
  }
  /* Read the Data chunk, must be 'data' */
  temp = ReadUnit((uint8_t*)buffer1, SpeechDataOffset, 4, BigEndian);
  SpeechDataOffset += 4;
  if (temp != DATA_ID)
  {
    return(Invalid_DataChunk_ID);
  }

  /* Read the number of sample data */
  WAVE_Format.DataSize = ReadUnit((uint8_t*)buffer1, SpeechDataOffset, 4, LittleEndian);
  SpeechDataOffset += 4;
  WaveCounter =  SpeechDataOffset;
  return(Valid_WAVE_File);
}

/**
* @brief  Reads a number of bytes from the SPI Flash and reorder them in Big
*         or little endian.
* @param  NbrOfBytes: number of bytes to read.
*         This parameter must be a number between 1 and 4.
* @param  ReadAddr: external memory address to read from.
* @param  Endians: specifies the bytes endianness.
*         This parameter can be one of the following values:
*             - LittleEndian
*             - BigEndian
* @retval Bytes read from the SPI Flash.
*/
uint32_t ReadUnit(uint8_t *buffer, uint8_t idx, uint8_t NbrOfBytes, Endianness BytesFormat)
{
    uint32_t index = 0;
    uint32_t temp = 0;

    for( index = 0; index < NbrOfBytes; index++)
    {
        temp |= buffer[idx + index] << (index * 8);
    }

    if( BytesFormat == BigEndian)
    {
        temp = __REV(temp);
    }
    return temp;
}
#endif

/**
 * @brief   This function reads to SDRAM and displays a short clip. The clip's
 *          total size should not be more than the available room in SDRAM, as it
 *          is loaded in it entirety to the SDRAM and played from there directly.
 *          !!* Currently only 8 bit bitmaps are supported *!!
 *
 * @param clipName - .avi file name on SD card
 * @return 0x00 if oK, otherwise ERROR and should abort
 */
uint8_t CAT_showShortClip( const char *clipName)
{
    static DWORD clmt[50];                                  /* Cluster link map table buffer */

    // free all memory involved if requesting to change a short clip
    if( shortClip_offset != NULL)
    {
        display_free( (void *)shortClip_offset);
        shortClip_offset = NULL;
        free( shortClip_pInx);
        shortClip_pInx = NULL;
        display_endShortClip();
    }
    // required????
    // NS TESTING display_clear();
    // NS out 10-7-17    display_removeWindow2();            // NS 24-07

    FRESULT fr = f_open_adj( rd, clipName, FA_READ);
    if( fr != FR_OK)
        return 0x04;                    //! ERROR, disk error
/*
    if( rd->fsize < 1500000)
        return 0x04;
*/
    //! prepare the cluster table for FAST_SEEK
    /*! Using fast seek feature */
    rd->cltbl = clmt;                                /* Enable fast seek feature (cltbl != NULL) */
    clmt[0] = sizeof( clmt) / sizeof( DWORD);        /* Set table size */
    fr = f_lseek( rd, CREATE_LINKMAP);               /* Create CLMT */
    if( fr != FR_OK)
    {
       puts( "could not create FAST SEEK  table\nforget it...\n");
       rd->cltbl = NULL;
    }
    shortClip_offset = (uint32_t)display_malloc( rd->fsize);
    if( shortClip_offset == (uint32_t)NULL) { f_close( rd); return 0x03; }    //! NOT ENOUGH MEMORY

    UINT red;
    for( int dx = 0; dx < 5; dx++)
    {
        fr = f_fastread( rd, (void *)shortClip_offset, rd->fsize, &red);
        if( fr == FR_DISK_ERR)
        {
            f_close( rd);
            f_open(rd, clipName, FA_READ);
        }
        else
            break;
    } // endfor
    f_close( rd);
    if( fr != FR_OK)
    {
        display_free( (void *)shortClip_offset);
        shortClip_offset = NULL;
        return 0x04;
    }

    char lfn_inx[/*_MAX_LFN*/32 + 1];
    uint rrr;
    int video_frames;

    replaceExtension( lfn_inx, (char *)clipName, "inx");

    fr = f_open( rd, lfn_inx, FA_READ);
    if( fr == FR_OK)
    {
        shortClip_pInx = (uint32_t *)malloc( rd->fsize);              // check if success....
        f_lseek( rd, 0);
        f_read( rd, (void *)shortClip_pInx, rd->fsize, &rrr);
        f_close( rd);
        if( f_open_adj( rd, clipName, FA_READ) == FR_OK)
        {
            video_frames = riffParser_Go( rd, ( DWORD **)&shortClip_pInx, /*ahProgBar*/0, 1);
            f_close( rd);
        }
        else
            riffParser_Go( rd, ( DWORD **)&shortClip_pInx, 0, true);
    }
    else
    {
        if( f_open( rd, clipName, FA_READ) == FR_OK)
        {
            video_frames = riffParser_Go( rd, ( DWORD **)&shortClip_pInx, /*ahProgBar*/0, 0);
            f_close( rd);

            if( video_frames > 0)
            {
                fr = f_open( rd, lfn_inx, FA_CREATE_ALWAYS | FA_WRITE);
                if( fr == FR_OK)
                {
                    // dump all indexes to file
                    fr = f_write( rd, (const void *)shortClip_pInx, video_frames * sizeof( int), &rrr);
                    f_close( rd);
                }
            }
            else
            {
                display_free( (void *)shortClip_offset);
                free( shortClip_pInx);
                shortClip_pInx = NULL;
                shortClip_offset = NULL;
                display_endShortClip();
                return FR_NO_FILE;
            }
        } // endif open avi file
    } // endelse
    // removed because it probably shows an image too early... display_prepareForShortClip();
    return 0x00;
} // endfunc


/**
 *
 * @param[in]   flashNum - number of ROM bitmap to be displayed
 *                         if 0 then clear screens
 * @return
 */
uint8_t CAT_showROMframe( uint8_t flashNum)
{
char buffer[32];


    //! needs a lot of fixing in _imageupdate - the
    // CLEAR SCREENS?
    if( flashNum == 0)
    {
        display_removeImageWindow();
        return 0;
    }
    sprintf(buffer, "%s%03d.%s", ROM_FILENAME, flashNum, ROM_EXTENSION);
    uint32_t offset = (uint32_t )display_readBmpToMemory( buffer, NULL);
#if 0
    FRESULT fr = f_open_adj(rd, buffer, FA_READ);
    if( fr != FR_OK)
        return 0x04;            //! ERROR, disk error

    f_lseek( rd, 0);
    //********************* read EVERYTHING fast to SDRAM
    uint32_t offset = ( SDRAM_BASE + SDRAM_SIZE - XSIZE_PHYS * YSIZE_PHYS * 4 - 100) & 0xFFFFFFF0;
    UINT red;
    for( int dx = 0; dx < 5; dx++)
    {
        FRESULT fr = f_fastread( rd, (void *)offset, rd->fsize, &red);
        if( fr == FR_DISK_ERR)
        {
            f_close( rd);
            f_open(rd, buffer, FA_READ);
        }
        else
            break;
    } // endfor
    f_close( rd);
#endif
    // take the bitmap and copy it to LTDC layer
    BITMAP_FILEHEADER *fhead = (BITMAP_FILEHEADER *)offset;
    if( fhead->Signature != /*SIGNATURE_OF_BITMAP*/ 0x4D42) //< "BM" is the signature. If can't get that --- go out!
    {
#ifdef DEBUG_CAT
       printf( "ERROR reading ROM file: %s\n", buffer);
#endif
       if( offset != NULL) display_free( ( void *)offset);
       display_removeImageWindow();
       return 0x05;
    }
    uint8_t *pixelsStart = ( uint8_t *)( offset + fhead->BitsOffset);
    BITMAPINFOHEADER *bmpheader = (BITMAPINFOHEADER *)( offset + sizeof( BITMAP_FILEHEADER));
    if( bmpheader->biBitCount != 24 && bmpheader->biBitCount != 8)        // 8 bits per color, 24 bits per pixel
    {
#ifdef DEBUG_CAT
        printf( "ERROR only 24 or 8 bits BMP are accepted at %s\n", buffer);
#endif
        display_free(( void *)offset);
        display_removeImageWindow();
        return 0x05;                        //! ERROR with file type
    }
    /// NS CAT_showKeypad();

    uint t1 = xtimer_now();
    display_setImageToDisplay( bmpheader->biWidth, bmpheader->biHeight, fhead);
    uint t2 = xtimer_now() - t1;
    display_free(( void *)offset);
    return 0x00;// all OK
} // endfunc


#if 1
int CAT_buildStatusString( void)
{
static uint32_t prevStatus = 0;
static uint32_t prevSeconds = 0;
static char statusString[128];

    // use lastReportedStatus....
    struct tm _tm;
    timex_get_localtime( &_tm);
    uint32_t secs = _tm.tm_sec + ( _tm.tm_hour << 16) + ( _tm.tm_min << 8);
    if( secs != prevSeconds)
    {
        int temperature = adc_sample( ADC_0, 0) * 31 - 26400;

        prevSeconds = secs;
        char buf[24];

        sprintf( buf, "v%02d.%02d", MAJOR_VERSION, MINOR_VERSION);

        if( ethernet_isOngoingDownload())
        {   // to move to KB and not just BYTES which overflows
            uint32_t uls = ethernet_getUploadSize() / 1000;
            uint32_t tus = ethernet_getTotalUploadSize() / 1000;
            sprintf( buf, "Ul:%d.%02d%%", uls * 100 / tus, (uls * 10000 / tus) % 100);
        }

        sprintf( statusString, "%02d-%02d %02d:%02d:%02d   %d.%02d degs   %s",
                _tm.tm_mday, _tm.tm_mon, _tm.tm_hour, _tm.tm_min, _tm.tm_sec,
                temperature / 100, temperature % 100, buf);
        display_updateStatusStrip( statusString);
        return 1;
    }
    return 0;
}
#endif





/**
 * @brief start showing no communications clip whenever there is no comm with host
 * @return
 * @note
 *
 */
int CAT_showNoCommClip( void)
{
    uint8_t data[32];
    const char fname[] = "nc8x4.avi";
    static uint32_t nc_stiming;

    if( nc_frameNum == NO_COMM_FIRST_TIME)
    {
       display_turnForegroundOff();

       if( CAT_showShortClip(fname) != 0x00)
           return 0x04;    /// error!
       nc_frameNum = 0;
       stopNCShortClip = 0;
       nc_stiming = 0;
    }


    ////////////////////////////////////////
    // COMMAND EXECUTION
    ////////////////////////////////////////
    // every call which gets here is 25mSec apart

    AVISTREAMHEADER *avihead = ( AVISTREAMHEADER *)riffParser_GetStreamHeader( 0);
    if( xtimer_now() - nc_stiming < 1000000 / avihead->dwRate /*/ 25*/)    //! in milliseconds
        return IN_COMMAND_EXECUTION;

   display_shortClip( nc_frameNum++, shortClip_pInx, shortClip_offset, (nc_stiming==0));

   nc_stiming = xtimer_now();

    //!get number of frames from avi header
    if( avihead->dwLength / avihead->dwScale <= nc_frameNum)
        nc_frameNum = 0;
    return IN_COMMAND_EXECUTION;
}


int CAT_restartNoCommClip( void)
{
    nc_frameNum = NO_COMM_FIRST_TIME;
    stopNCShortClip = 0;
}
int CAT_stopNoCommClip( void)
{
    if( nc_frameNum != NO_COMM_FIRST_TIME)
    {
        display_free( ( void *)shortClip_offset);
        free( shortClip_pInx);
        shortClip_pInx = NULL;
        shortClip_offset = NULL;
        display_endShortClip();
        nc_frameNum = NO_COMM_FIRST_TIME;
        display_turnForegroundOn();

        stiming = 0;                    // restart previous short clip (if present)
    }
    return FINISHED_EXECUTING_RAM;
}


















void CAT_soundBuzzer( uint32_t msec)
{
    return;
}



/*************************************************/
/*          S T A R T    S O U N D               */
/* taken from:                                   */
/* http://00xnor.blogspot.co.il/2014/01/6-stm32-f4-dac-dma-waveform-generator.html */
/*************************************************/
#define   OUT_FREQ          4000                                 // Output waveform frequency
#define   SINE_RES          128                                  // Waveform resolution
#define   DAC_DHR12R1_ADDR  0x40007408                           // DMA writes into this reg on every request
#define   DAC_DHR12R2_ADDR  0x40007414                           // DMA writes into this reg on every request
#define   CNT_FREQ          42000000                             // TIM6 counter clock (prescaled APB1)
#define   TIM_PERIOD        ((CNT_FREQ)/((SINE_RES)*(OUT_FREQ))) // Autoreload reg value

const uint16_t function[SINE_RES] = { 2048, 2145, 2242, 2339, 2435, 2530, 2624, 2717, 2808, 2897,
                                      2984, 3069, 3151, 3230, 3307, 3381, 3451, 3518, 3581, 3640,
                                      3696, 3748, 3795, 3838, 3877, 3911, 3941, 3966, 3986, 4002,
                                      4013, 4019, 4020, 4016, 4008, 3995, 3977, 3954, 3926, 3894,
                                      3858, 3817, 3772, 3722, 3669, 3611, 3550, 3485, 3416, 3344,
                                      3269, 3191, 3110, 3027, 2941, 2853, 2763, 2671, 2578, 2483,
                                      2387, 2291, 2194, 2096, 1999, 1901, 1804, 1708, 1612, 1517,
                                      1424, 1332, 1242, 1154, 1068, 985, 904, 826, 751, 679,
                                      610, 545, 484, 426, 373, 323, 278, 237, 201, 169,
                                      141, 118, 100, 87, 79, 75, 76, 82, 93, 109,
                                      129, 154, 184, 218, 257, 300, 347, 399, 455, 514,
                                      577, 644, 714, 788, 865, 944, 1026, 1111, 1198, 1287,
                                      1378, 1471, 1565, 1660, 1756, 1853, 1950, 2047 };

static void TIM6_Config(void);
static void DAC1_Config(void);

int test_startSound( uint32_t msec)
{
#if 0
  GPIO_InitTypeDef gpio_A;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  gpio_A.GPIO_Pin  = GPIO_Pin_5;
  gpio_A.GPIO_Mode = GPIO_Mode_AN;
  gpio_A.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &gpio_A);

  TIM6_Config();
  DAC2_Config();

  /**
   * sound is now enabled through PA5
   * it should be disable once beep is to be turned off
   */
    if( xtimer_callmeback( msec * 1000, test_stopSound) == -1)
    {   // if no available timers
        xtimer_usleep( msec * 1000);
        test_stopSound( 0);
    }
#endif
    return 0;
}


void *test_stopSound( void *arg)
{
    TIM_Cmd(TIM6, DISABLE);

    //DAC_Cmd(DAC_Channel_2, DISABLE);
    return NULL;
}

static void TIM6_Config(void)
{
  TIM_TimeBaseInitTypeDef TIM6_TimeBase;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

  // calculate timer6 period using actual APB1 frequency
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq( &RCC_Clocks);
  uint16_t tim_preiod = ( RCC_Clocks.PCLK2_Frequency / (SINE_RES * OUT_FREQ));

  TIM_TimeBaseStructInit(&TIM6_TimeBase);
  TIM6_TimeBase.TIM_Period        = (uint16_t)tim_preiod;
  TIM6_TimeBase.TIM_Prescaler     = 0;
  TIM6_TimeBase.TIM_ClockDivision = 0;
  TIM6_TimeBase.TIM_CounterMode   = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &TIM6_TimeBase);
  TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update);

  TIM_Cmd(TIM6, ENABLE);
}

// changed to use dac channel 2 (instead of 1 in original)
static void DAC2_Config(void)
{
  DAC_InitTypeDef DAC_INIT;
  DMA_InitTypeDef DMA_INIT;

  DAC_INIT.DAC_Trigger        = DAC_Trigger_T6_TRGO;
  DAC_INIT.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_INIT.DAC_OutputBuffer   = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_2, &DAC_INIT);

  DMA_DeInit(DMA1_Stream6);                 // was stream 5
  DMA_INIT.DMA_Channel            = DMA_Channel_7;
  DMA_INIT.DMA_PeripheralBaseAddr = (uint32_t)DAC_DHR12R2_ADDR;
  DMA_INIT.DMA_Memory0BaseAddr    = (uint32_t)&function;
  DMA_INIT.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
  DMA_INIT.DMA_BufferSize         = SINE_RES;
  DMA_INIT.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
  DMA_INIT.DMA_MemoryInc          = DMA_MemoryInc_Enable;
  DMA_INIT.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_INIT.DMA_MemoryDataSize     = DMA_MemoryDataSize_HalfWord;
  DMA_INIT.DMA_Mode               = DMA_Mode_Circular;
  DMA_INIT.DMA_Priority           = DMA_Priority_High;
  DMA_INIT.DMA_FIFOMode           = DMA_FIFOMode_Disable;
  DMA_INIT.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
  DMA_INIT.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
  DMA_INIT.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &DMA_INIT);            // was stream5 again

  DMA_Cmd(DMA1_Stream6, ENABLE);                // was stream5 again
  DAC_Cmd(DAC_Channel_2, ENABLE);
  DAC_DMACmd(DAC_Channel_2, ENABLE);
}

#if 0
/**
 * @brief this function checks the strobe line of the mag stripe read
 *        if the reader is connected, we should see high level there
 *        even though the line is pulled down.
 *
 * @return true if seeing magstripe reader, otherwise, false
 */
bool CAT_checkIfMagStripeConnected( void)
{
    /*
    gpio_init_in(  CARD_STROBE_T2, GPIO_PULLDOWN);
    if( gpio_read( CARD_STROBE_T2))
        return true;

    else
    */

    return false;

}
#endif


#if 0
#define MAX_SWUARTS     2
static uint8_t swuart_curEntity;

typedef enum
{
    SWU_IDLE,
    SWU_CHECK_START_BIT,
    SWU_GET_BYTE,
    SWU_CHECK_STOP_BIT
} SWUART_STATES;


typedef struct
{
    uint gpio_rx;
    uint gpio_tx;
    uint baud, halfBit, fullBit;
    uint8_t parity, data_bits, stop_bits;       // currently not used
    uint8_t rxByte;
    SWUART_STATES rxstate;

    typedef union flags
    {
        uint8_t allflags;
        typedef struct
        {
            uint8_t flag_rxc            : 1;
            uint8_t flag_error          : 1;
            uint8_t flag_txc            : 1;
        }
    };
} swuart_info_t;
static swuart_info_t suart_info[MAX_SWUARTS];

void swUart_Init()
{
    swuart_curEntity = 0;
    timer_init( TIMER_1, 1 /* get a count every 1/3 microsecond */, swUart_rxBit);
}

/**
 * @brief   This function is called from the timer module every time a bit count
 *          is elapsed.
 *          It works by a state machine to receive a byte (currently 8 bit byte)
 *          from a connected device.
 *
 * @param   arg - the entity of the receiving software uart
 *
 * @returns none.
 */
void swUart_rxCallback( void *arg)
{
    uint8_t entity = (uint8_t)arg;
    int bit = gpio_read( suart_info[entity].gpio_rx);

    switch( suart_info[entity].rxstate)
    {
        case SWU_IDLE:
            break;

        case SWU_CHECK_START_BIT:
            if( bit != 0)
                swUart_restartRx( entity);
            else
            {
                suart_info[entity].rxstate = SWU_GET_BYTE;
                suart_info[entity].rxByte = 0;

                timer_set( TIMER_1, entity, suart_info[entity].fullBit);
            }
            break;

        case SWU_GET_BYTE:
            bit = bit ? 0x80 : 0x00;
            suart_info[entity].rxByte = (suart_info[entity].rxByte >> 1) | bit;
            suart_info[entity].rxBits++;
            if( suart_info[entity].rxBits == suart_info[entity].data_bits)
                suart_info[entity].rxstate = SWU_CHECK_STOP_BIT;

            timer_set( TIMER_1, entity, suart_info[entity].fullBit);
            break;

        // parity??

        case SWU_CHECK_STOP_BIT:
            if( bit)
            {
                suart_info[entity].flags.flag_txc = 1;
                suart_info[entity].flags.flag_txerror = 0;
                // call a callback here ????
                swUart_restartRx( entity);
            }
            else
            {
                suart_info[entity].flags.flag_txc = 0;
                suart_info[entity].flags.flag_txerror = 1;
                // call an error callback here ????
                swUart_restartRx( entity);
            }
            break;

        default:
            break;

    } // endswitch
} // endfunc


/**
 * @brief   This function is called from the gpio EXTI interrupt every time
 *          a new frame is beginning on the Rx line
 *
 * @param   arg - the entity of the receiving software uart
 *
 * @returns none.
 */

void swUart_rxCallback( void *arg)
{
    uint8_t entity = (uint8_t)arg;

    gpio_irq_disable( suart_info[entity].gpio_rx);      // disable IRQ on Rx pin
    suart_info[entity].state = SWU_CHECK_START_BIT;
    timer_set( TIMER_1, entity, suart_info[entity].halfBit);
    timer_irq_enable( TIMER_1);
}
/**
 * @brief   Creates a new software uart entity
 *
 * @param
 *
 */
// currently constant 8 bit, no parity, 1 stop bit
uint8_t swUart_Create( uint baud_rate, gpio_t gpio_rx, gpio_t gpio_tx)
{
    uint8_t entity = swuart_curEntity;
    suart_info[entity].gpio_rx = gpio_rx;
    suart_info[entity].gpio_tx = gpio_tx;
    suart_info[entity].flags.allflags = 0;
    suart_info[entity].rxstate = SWU_IDLE;
    suart_info[entity].data_bits = 8;

    uint oneCountNano = 1000 * (TIMER_1_PRESCALER + 1) / ( CLOCK_CORECLOCK / 2000000U);
    suart_info[entity].fullBit = (1000000000U / baud_rate / (oneCountNano);
    suart_info[entity].halfBit = fullBit / 2;

    gpio_init_int( gpio_rx, GPIO_PULLUP, GPIO_FALLING, swUart_rxCallback, entity);
    return swuart_curEntity++;
}

void swUart_restartRx( uint8_t entity)
{
    gpio_irq_enable( suart_info[entity].gpio_rx);      // disable IRQ on Rx pin
    suart_info[entity].rxstate = SWU_IDLE;
}

#endif

