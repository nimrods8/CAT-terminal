/**
 * Display header
 *
 * @ingroup  app
 * @brief    This module is in charge of operating the display and
 *           touch screen.
 * @{
 * @file   display.h
 * @author Nimrod Stoler <nstoler@gmail.com>
 * @}
 */

#ifndef ___DISPLAY_H
#define ___DISPLAY_H

#include "stdlib.h"
#include "stdint.h"

#ifndef EMV_KERNEL
    #include "mutex.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifdef USE_TOUCH_PANEL
#include "FT5x06.H"
#endif

//#include "ugui.h"

#define RESTART_DISPLAY_KEY     44


#define KEY_ENTER               0x0d
#define KEY_CANCEL              0x1b
#define KEY_CORRECT             0x08
#define KEY_CLEAR               0x08

#define MENU_REQUEST            0x90

// THREAD INPUT DATA
typedef struct
{
    const char *title;
    const char *text;
    uint timeout;               // timeout is allowed to be zero, at which case
                                // the progressbar will just scroll
    void (*mbDone)( void);      // notify that the messagebox is done displaying
} messageBoxStruct;

//-----------------------------------------------------
// this is a small thread within display which displays
// and updates a messagebox on the foreground layer
//-----------------------------------------------------
void display_mbThread( void *arg);
void display_dropMessageBoxThread();
void display_showConsole( void);
void display_consoleWrite( const char *txt);
void display_consoleWriteColor( const char *txt, uint32_t forecolor);
/**
 * @brief    returns not true if message box is canceled
 *
 */
int display_isMessageBoxCanceled( void);


/**
 * @brief This is the main function of the ugui test
 * @return
 */
int display_main( void (*pressed)( char button), mutex_t *sync_dsp);

/**
 * @brief Being called from the touch thread every time a change happens
 *        in the status of the touch panel
 *
 * @param ctp_data - pointer to the CTP structure
 */
int display_periodic( /*_m_ctp_dev *ctp_data*/void);

void display_setPumpNumber( int pump);
/**
 * @brief sets the Host IP address to window_8, TXB_ID_0 (in the setup)
 * @param ip_str
 */
void display_setHostIP( const char *ip_str);


/**
 * @brief sets new image for window_2 image object to be displayed
 * @param width
 * @param height
 * @param pixelsStart
 */
void display_setImageToDisplay( int width, int height, void *pixelsStart);
void display_setBkImageToDisplay( char *bitmapName);

void display_messageBox( char *text);
void display_messageBoxPos( char *text, int x, int y);
void display_removeMessageBox( void);
void display_setupMessageBox( const char *title, const char *text, uint timeout);


void display_setRightText( int Id);
void display_setLeftText( int Id);


void display_showImageWindow();
void display_removeImageWindow();
bool display_isImageWindowShown( void);
int display_prepareForShortClip( void);
int display_shortClip( uint showFrame, uint32_t *pInx, uint32_t screens, bool newClip);
void display_endShortClip( void);
void display_updateTextW3( char *txt, int cnt);
void display_window3( void);
/**
 * @brief   returns the show status of the main menu
 * @return  true if main menu is currently displayed
 *          false if not
 */
bool display_isMainMenuShown( void);

/**
 * this function clears the display
 */
void display_clear( void);

void display_free( void *ptr);
void *display_malloc( size_t size);


/**
 * @brief       This function reads a file from SDcard to a memory address
 *              Memory address can only be within DMA reachable memory, i.e.
 *              no CCM.
 *
 * @param[in]   fName   file name
 * @param[in]   address pointer to byte address
 * @return      address of memory where the bitmap was read to if OK,
 *              otherwise NULL
 */
uint8_t *display_readBmpToMemory( const char *fName, uint8_t *address);


/**
 * @brief Setup LTDC to show keypad and messages window
 */
void display_window2(void);
void display_removeWindow2( void);
void display_turnForegroundOff( void);
void display_turnForegroundOn( void);

/**
 * @brief   fills a gui with color c
 * @param layer
 * @param c
 */
void display_fill( int layer, /*UG_COLOR*/uint32_t c);

/**
 * @brief   removes focus from ID_TXB_1 so that the cursor no longer shows
 *
 */
void display_removeFocus( void);


/**
 * @brief Displays new text on screen
 * @param txt
 * @param cnt
 */
void display_showText( char *txt, int cnt, int Id);
void display_dataGui();

int display_windowWithButtons( const char *title, int buttons, uint32_t *names);


/**
 * @brief       Sets up window_6 to show the get PIN interaction with user
 *
 * @param[in]   headerMsg - the text to show on the head of the get PIN window
 *
 * @return      currently none
 */
int display_windowGetPIN( const char *headerMsg);
int display_setTextGetPIN( const char *pinText);
/**
 * @brief   displays get PIN message and runs all the necessary EPP things
 *          to get the PIN from the PIN Pad
 *
 * @param[in] pinMaxLen - maximum length of requested PIN
 * @param[in] hdrMsg    - message to show at the top of the window
 * @param[in] errMsg    - message to show when error occurs
 * @param[in] cancelMsg - message to show when user cancels input
 * @param[in] timeoutMsg- message to show when input times out
 * @param[in] timeout   - timeout in seconds
 * @param[in] needsCard - does this PIN entry needs card inserted in creator reader
 * @return    < 0 if error, 0 if OK
 */
int display_getPIN( char *plaintext, const char *PAN,
                    uint8_t pinMaxLen,
                    const char *hdrMsg,
                    const char *errMsg,
                    const char *cancelMsg,
                    const char *timeoutMsg,
                    uint timeout, bool needsCard,
                    int (*getCardStatus)( void));

/**
 * @brief   takes enciphered PIN using EPP
 *
 * @param encryptedPinBlock - return PIN block from EPP
 *        maxlength - maximum len of PIN
 *        enterPIN  - string to be displayed
 *        PAN       - personal account number of cardholder
 *
 * @return  EPP_ERROR or EPP_OK and return PIN block is in
 *          encryptedPinBlock
 */
int display_getEncipheredPIN( char *encryptedPinBlock,
                              uint8_t maxlength,
                              const char *enterPIN,
                              const char *PAN);


void display_updateStatusStrip( const char *text);


#ifdef __cplusplus
}
#endif

#endif // __DISPLAY_H
