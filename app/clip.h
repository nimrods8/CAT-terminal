/**
 * Clip module header
 *
 * @ingroup  app
 * @brief    This module is in charge of showing the long clips on the LCD
 * @{
 * @file   clip.h
 * @author Nimrod Stoler <nstoler@gmail.com>
 * @}
 */

#ifndef ___CLIP_H
#define ___CLIP_H


#ifdef __cplusplus
extern "C" {
#endif




typedef struct
{
    char *filename;
    void (*CAT_notifyDoneCB)(void);
} clipArguments_t;


/**
 * @brief prepares a clip for display
 *
 * @param[in]   filename of the clip on SD card
 *
 * @return      none
 */
void clip_Prepare( void *arg);


/**
 * @brief opens the index file of the video
 * @return
 */
FRESULT clip_startSDcard( void);



/**
 * @brief this function kills both threads, one at a time
 *        and virtually stops showing the clip
 */
void clip_StopAndKillThreads( void);


/**
 * @brief replaces source filename string in the format xxxxx.aaa with
 *        a new string - dst- in the format xxxxx.www
 * @param dst - destination string
 * @param src = source filename
 * @param www - what to replace the extension with
 */
void replaceExtension( char *dst, char *src, char *www);


#ifdef __cplusplus
}
#endif

#endif // __DISPLAY_H
