/*
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
#include <stdio.h>
#include "board.h"
#include "mutex.h"
#include "kernel.h"
#ifdef HWTIMER
#include "hwtimer.h"
#else
#include "xtimer.h"
#endif
#include "thread.h"
#include "sched.h"
#include "msg.h"
//#include "sdram.h"
#include "stm32f4xx_ltdc.h"
#include "ugui.h"

//! FAT-fs files
#include "fatfs_sd_sdio.h"
#include "ff.h"
#include "aviriff.h"
#include "clip.h"

#define DEBUG_FRMS


/**
 * @ingroup     CLIP is a module to show short clips on LCD
 * @{
 *
 * @file
 * @brief  This is an implementation of display of short BMP L8 clips on LCD
 *         The bitmaps are loaded to SDRAM.
 *         The LTDC is working with background layer only.
 *         Bitmaps using 1 bit per pixel (i.e. white and black only) are
 *         expanded to address 0xD000000 (SDRAM start)
 *
 * @note   The clips should be prepared using special procedures.
 *         See below...
 *
 * @author Nimrod Stoler <nstoler@gmail.com>
 */

/**
 * module's static functions
 */
FRESULT clip_startSDcard( void);
FRESULT clip_readOrMakeIndex( char *filen);



/**
 *   DATA DEFINES
 */
#define FRAMES_BATCH_SIZE   6
#define START_SDRAM_VBUF1   SDRAM_BASE                      // 1st half
#define START_SDRAM_VBUF2   SDRAM_BASE + (SDRAM_SIZE / 2)   // 2nd half

/**
 * Static Variables
 */
char bitmap_stack[/*KERNEL_CONF_STACKSIZE_MAIN*/1024+256]       __attribute__ ((section(".ccm")));;
static volatile int busy, /*i, k,*/ startt, haveFrames, curFrame, video_frames;
static int file_inx, clip_debug, bmpErrorFrames;
static volatile uint32_t *pInx;                         /*< pointer to the index, to be malloc'ed by riffParser */
////// NS 09-09-15   volatile uint8_t *videobuf;        /*< video  buffer used by bitmap thread. Can be either  START_SDRAM_VBUF1 or 2 */
static DIR   *dj;                                       /* Directory search object */
static FIL   *rd;                                       /*< file handle */
static FILINFO fno;
static uint8_t stopExecution;
//static UG_GUI clipGui;
int set_numofframes, errorFrames, debug_num;
static uint32_t chunktime, chunkvol;
static DWORD clmt[50];                                  /* Cluster link map table buffer */

/**
 * Static Functions
 */
static void clip_releaseMemory( void);


/**********************************************/
/*        T H R E A D   E N T R Y             */
/**********************************************/
/**
 * @brief This thread is in charge of pushing the bitmap frames to
 *        the LCD, currently using STem GUI functions
 *        ****Movie is displayed on background layer (Layer1)
 * @param arg
 */
void *bitmap_thread(void *arg)
{
    (void) arg;

    BITMAPINFOHEADER *bmpheader;
    typedef struct
    {
       uint8_t colorA;
       uint8_t colorR;
       uint8_t colorG;
       uint8_t colorB;
    } bmpColors;
    bmpColors *bmpclut;

    /*! SET CLUT IN PLACE for 1 bit bitmaps */
    uint8_t twoColorCLUT[8];
    twoColorCLUT[0] = 0;
    twoColorCLUT[1] = 0;
    twoColorCLUT[2] = 0;
    twoColorCLUT[4] = 0xFF;
    twoColorCLUT[5] = 0xFF;
    twoColorCLUT[6] = 0xFF;



    msg_t msg;

    puts("--> bitmap_thread starting\n");

    AVISTREAMHEADER *avihead = riffParser_GetStreamHeader( 0);
    //LTDC_LayerPixelFormat( LTDC_Layer1, LTDC_Pixelformat_RGB888);
    //LTDC_ReloadConfig( LTDC_IMReload);
    //NS 09-09-15 GUIDRV_MovieInit( avihead->rcFrame.right - avihead->rcFrame.left, avihead->rcFrame.bottom - avihead->rcFrame.top, LTDC_Pixelformat_RGB888);

    int __curFrame = 0, clutEnabled = 0;
    uint8_t *___videobuf = (uint8_t *)START_SDRAM_VBUF1;                /*< ALWAYS starts with vbuf1 - ghost video buffer */
    int framesPerSecond = avihead->dwRate;                              /* avihead->dwScale ??? */;
    int timePerBatch = 1000000 * FRAMES_BATCH_SIZE / framesPerSecond;   /* time per batch of frames read */
    uint8_t *cLookupTable;

    //!get number of frames from avi header
#if 1
    int numofframes = avihead->dwLength / avihead->dwScale;
#else
    int numofframes = set_numofframes;
#endif
    framesPerSecond = FRAMES_BATCH_SIZE;
    if( framesPerSecond > avihead->dwRate) framesPerSecond = avihead->dwRate;


    cLookupTable = (uint8_t *)malloc( 4*256);

    while (busy && haveFrames != -1)
    {
        /*
#ifdef DEBUG_FRMS
        hwtimer_wait( 1000);
        continue;
#endif
*/
        if( haveFrames)
        {
           while( 1)
           {
              uint32_t start_addrs  = pInx[__curFrame] % _MAX_SS /* usually just 512 bytes for SDIO */;

              for( int i = __curFrame; i < __curFrame + framesPerSecond /*FRAMES_BATCH_SIZE*/; i++)
              {
#ifdef HWTIMER
                  startt = hwtimer_now();
#else
                  startt = xtimer_now();
#endif

                  clip_debug = 1;


                  uint32_t offset = pInx[i] - pInx[__curFrame] + ( uint32_t)___videobuf/*START_SDRAM_SDIO*/ + start_addrs;
                  ////printf( "i=%d, offset=%X", i, offset);
                  if( offset < SDRAM_BASE || offset > SDRAM_BASE + SDRAM_SIZE)
                  {
#ifdef DEBUG
                      printf( "ERROR > offset is out of bounds!");
#endif
                      continue;
                  }

                  clip_debug = 2;



                  BITMAP_FILEHEADER *fhead = (BITMAP_FILEHEADER *)offset;

                  if( fhead->Signature != SIGNATURE_OF_BITMAP /*0x4D42*/) //< "BM" is the signature. If can't get that --- go out!
                  {
#ifdef DEBUG
                     printf( "ERROR reading movie file at frame: %d\n", i);
                     bmpErrorFrames++;
#endif
                     continue;
                  }
                  uint32_t bitoffset;
                  memcpy( (uint8_t *)&bitoffset, (uint8_t *)&fhead->BitsOffset, 4);

                  //---------------------------------! corrupt?
                  if( bitoffset > 0xffff)
                      bitoffset = bitoffset & 0xffff;
                  //---------------------------------! corrupt?
                  uint8_t *pixelsStart = ( uint8_t *)( offset + /*fhead->BitsOffset*/bitoffset);
                  if( pixelsStart < SDRAM_BASE || pixelsStart > SDRAM_BASE + SDRAM_SIZE)
                  {
                      bmpErrorFrames++;
                      continue;
                  }

                  bmpheader = (BITMAPINFOHEADER *)( offset + sizeof( BITMAP_FILEHEADER));
#if 0
                  BITMAPINFOHEADER *localBmpHeader = (BITMAPINFOHEADER *)malloc( sizeof( BITMAPINFOHEADER));
                  uint8_t *dst = (uint8_t *)localBmpHeader;
                  uint8_t *src = (uint8_t *)bmpheader;
                  for( int pp = 0; pp < sizeof( BITMAPINFOHEADER); pp++)
                  {
                      *dst++ = *src++;
                  }
                  //memcpy( (uint8_t *)localBmpHeader, (uint8_t *)bmpheader, sizeof( BITMAPINFOHEADER));
#endif
                  /*************************
                   *   8 bits per pixel
                   *************************/
                  if( /*localB*/bmpheader->biBitCount == 8)   // if BITMAP is not full RGB888 this bitmap is using a CLUT
                  {
                     uint32_t ssize = ( bmpheader->biSize & 0x0000ffff);
                     bmpclut = ( bmpColors *)( (uint32_t)bmpheader + ssize);
                     if( !clutEnabled)
                     {
                        //! set pixel format of LTDC, stupid!
                        //LTDC_Layer1->PFCR = LTDC_Pixelformat_L8;
                        //LTDC_LayerPixelFormat( LTDC_Layer1, LTDC_Pixelformat_L8);
                        //LTDC_ReloadConfig( LTDC_IMReload);
                        GUIDRV_MovieInit( avihead->rcFrame.right - avihead->rcFrame.left,
                                          avihead->rcFrame.bottom - avihead->rcFrame.top,
                                          LTDC_Pixelformat_L8);
                     }
                     //! RLE 8 ???
                     if( /*localB*/bmpheader->biCompression == 1)
                     {
                         // **********************
                         // CURRENTLY UNHANDLED!!
                         // **********************

#ifdef DEBUG
                         printf( "RLE?!");
#endif
                     }
                     if( bmpclut > SDRAM_BASE + SDRAM_SIZE)
                         while(1);
                     memcpy( cLookupTable, bmpclut, 256*4);
                     GUIDRV_setPendingClut( ( uint8_t *)cLookupTable/*bmpclut*/, /*localB*/bmpheader->biClrUsed, pixelsStart); //< the clut will be written to LTDC during blanking time
                     //LTDC_Layer1->CR |= (uint32_t)LTDC_LxCR_CLUTEN;
                     LTDC_CLUTCmd( LTDC_Layer1, ENABLE);
                     clutEnabled = 1;
                  } // endif
                  /*************************
                   *   1 bit per pixel
                   *************************/
                  else if( /*localB*/bmpheader->biBitCount == 1)  // if BITMAP is not full RGB888 this bitmap is using a CLUT
                  {
                     if( !clutEnabled)
                     {
                        //! set pixel format of LTDC, stupid!
                        //LTDC_Layer1->PFCR = LTDC_Pixelformat_L8;
                        //LTDC_LayerPixelFormat( LTDC_Layer1, LTDC_Pixelformat_L8);
                        //LTDC_ReloadConfig( LTDC_IMReload);
                        GUIDRV_MovieInit( avihead->rcFrame.right - avihead->rcFrame.left, avihead->rcFrame.bottom - avihead->rcFrame.top, LTDC_Pixelformat_L8);
                     }

                     uint bottom = avihead->rcFrame.bottom;
                     uint right  = avihead->rcFrame.right / 8;


                     uint8_t *dstBuffer = (uint8_t *)( /*___videobuf*/START_SDRAM_VBUF1 + SDRAM_SIZE / 2 - 0x5Dff0);
                     int pixelcnt = 0;

                     cyclog_write( 0xe0, dstBuffer, 8);
                     //cyclog_write( 0xe1, avihead->rcFrame.bottom, 8);
                     //cyclog_write( 0xe2, avihead->rcFrame.right, 8);
                     cyclog_write( 0xe3, pixelsStart, 8);


                     for( int y = 0; y < bottom; y++)
                     {
                         for( int x = 0; x < right; x++)
                         {
                             uint8_t *ppixels = ( uint8_t *)pixelsStart;
                             uint8_t bte = ppixels[x + y * right];

                             if( bte == 0)
                             {
                                 uint32_t *dbf = ( uint32_t *)&dstBuffer[pixelcnt];
                                 *dbf = 0;
                                 dbf++;
                                 *dbf = 0;
                                 pixelcnt += 8;
                             }
                             else
                             {
                                 for( int bit = 0; bit < 8; bit++)
                                 {
                                    dstBuffer[pixelcnt++] = ( bte & (1 << bit));
                                 }
                             }
                         } // endfor all X pixels
                     } // endfor all Y pixels
                     pixelsStart = dstBuffer;

                     GUIDRV_setPendingClut( ( uint8_t *)twoColorCLUT, /*localB*/bmpheader->biClrUsed, pixelsStart);    //< the clut will be written to LTDC during blanking time

                     //LTDC_Layer1->CR |= (uint32_t)LTDC_LxCR_CLUTEN;
                     LTDC_CLUTCmd( LTDC_Layer1, ENABLE);

                     clutEnabled = 1;
                  }
                  /*************************
                   *   A L L    O T H E R
                   *   F O R M A T S
                   *************************/
                  else if( clutEnabled)
                  {
                      clutEnabled = 0;
                      LTDC_CLUTCmd( LTDC_Layer1, DISABLE);

                      //! set pixel format of LTDC, stupid!
                      // already done above, isn't it???????     LTDC_Layer1->CR &= (uint32_t)~LTDC_LxCR_CLUTEN;
                      //LTDC_Layer1->PFCR = LTDC_Pixelformat_RGB888;
                      //LTDC_LayerPixelFormat( LTDC_Layer1, LTDC_Pixelformat_RGB888);
                      //LTDC_ReloadConfig( LTDC_IMReload);
                      GUIDRV_MovieInit( avihead->rcFrame.right - avihead->rcFrame.left, avihead->rcFrame.bottom - avihead->rcFrame.top, LTDC_Pixelformat_RGB888);
                  }
#if 0
                  free( localBmpHeader);
#endif

                  clip_debug = 3;

                  //< draw the bitmap using STEM GUI functions for now
                  //GUI_DrawBitmap( &bmAvi, (240 - bmAvi.XSize) / 2 - 1, 0);
#if 0 // testing
                  LTDC_LayerAddress( LTDC_Layer1, (uint32_t)pixelsStart);
                  LTDC_ReloadConfig( LTDC_VBReload);
#endif
                  int waitus = ( 1000000 / avihead->dwRate) /* currently 40,000usec */ - ( xtimer_now() - startt);
                  if( waitus < 1000)
                     waitus = 1000;


                  clip_debug = 4;

                  xtimer_usleep( waitus);

                  clip_debug = 5;

                  // check if this frame was the last one...
                  // if so, go out
                  if( i+1 >= numofframes /*|| stopExecution*/ /*|| !busy*/)
                  {
                      haveFrames = -1;
                      break;
                  }


                  // move the data window up and down
                  display_manageW3Location();


              } // endfor batch number of frames
#if 0
              //! finished this round. go out!
              if( __curFrame + framesPerSecond + 1 >= numofframes ||
                   stopExecution || !busy)
              {
                 // OK to finish on main thread
                 haveFrames = -1;
                 break;
              }
#endif
#if 1
              if( numofframes > FRAMES_BATCH_SIZE && busy)
              {
                  //! Should sleep until main thread sends a message with
                  //! new address of next buffer

                  clip_debug = 6;

                  msg_receive( &msg);


                  clip_debug = 7;


                  ___videobuf = (uint8_t *)msg.content.ptr;
                  __curFrame = curFrame;            /*< ghost frames counter */
              }
              else break;

#else
              //! after 25 frames have been dumped to LTDC
              //! wait for the video buffer to change and then
              //! update curFrame and go display the next 25 frames
              while( __videobuf == videobuf)
                    hwtimer_wait( 1000);
              __videobuf = (uint8_t *)videobuf;         /*< ghost video buffer */
              __curFrame = curFrame;            /*< ghost frames counter */
#endif
#if 0
              if( __curFrame + FRAMES_BATCH_SIZE /*framesPerSecond*/ >= numofframes)
                 framesPerSecond = numofframes - __curFrame - 1;
#endif
           } // endwhile true

           //! if stopexecution was ordered
           if( stopExecution)
           {
               haveFrames = -1;
               break;
           }

        } // endif haveframes
    } // endwhile busy

    LTDC_LayerPixelFormat( LTDC_Layer1, LTDC_Pixelformat_RGB565);
    LTDC_LayerPixelFormat( LTDC_Layer2, LTDC_Pixelformat_RGB565);
    LTDC_ReloadConfig( LTDC_IMReload);
    LTDC_LayerPosition( LTDC_Layer2, 0, 0);
    LTDC_ReloadConfig( LTDC_IMReload);
    LTDC_LayerSize( LTDC_Layer2, XSIZE_PHYS, YSIZE_PHYS);
    LTDC_ReloadConfig( LTDC_IMReload);
    LTDC_LayerAlpha(LTDC_Layer2, 255); // was 55


#define SDRAM_BANK_ADDR     SDRAM_BASE
#define LAYER_1_OFFSET      0x000000        // 1024x600x2Bpp
#define LAYER_2_OFFSET      0x12C000        // 1024x600x2Bpp

    LTDC_LayerAddress( LTDC_Layer2, (uint32_t)( SDRAM_BANK_ADDR + LAYER_2_OFFSET));


    LTDC_LayerCmd( LTDC_Layer1, DISABLE);
    LTDC_LayerCmd( LTDC_Layer2, DISABLE);

    //! make sure layer1 >BACKGROUND LAYER< is back to RGB565 format for GUI
    LTDC_ReloadConfig( LTDC_VBReload);
    //display_window4();

    // in order to release clip thread
    haveFrames = -1;


    free( cLookupTable);

    printf( "\nFINISHED!\n");
    return NULL;
} // end thread




/**********************************************/
/*     P U B L I C     F U N C T I O N S      */
/**********************************************/
void clip_Prepare( void *arg)
{
uint8_t *videobuf;                      /*< video  buffer used by bitmap thread. Can be either  START_SDRAM_VBUF1 or 2 */

    debug_num = 0;

    clipArguments_t *clipargs = ( clipArguments_t *)arg;
    char *filename = clipargs->filename;
    stopExecution = 0;
    curFrame = 0;
    errorFrames = 0;
    bmpErrorFrames = 0;

    dj    = ( DIR *)malloc( sizeof( DIR));
    rd    = ( FIL *)malloc( sizeof( FIL));
    pInx  = NULL;

    if( filename == NULL) {
        if( clip_startSDcard() != FR_OK) {
            clip_releaseMemory();
            ///////////////////////////////////////////////////////
            // notify CAT that this thread is done
            clipargs->CAT_notifyDoneCB();
            return;
        }
    } // endif no filename given
    else
    {
        if( clip_readOrMakeIndex( filename) != FR_OK) {
            clip_releaseMemory();
            display_setupMessageBox( "Error", "Could not load video file", 5);
            ///////////////////////////////////////////////////////
            // notify CAT that this thread is done
            clipargs->CAT_notifyDoneCB();
            return;
        }
        memcpy( fno.fname, filename, sizeof( fno.fname));
    }
    display_clipGui();
    display_dataGui();


    //! read stream header of first stream which is the video one
    AVISTREAMHEADER *avihead = ( AVISTREAMHEADER *)riffParser_GetStreamHeader( 0);
    int readNext = /*was 25*/  avihead->dwRate /* avihead->dwScale ??? */;
    videobuf = (uint8_t *)START_SDRAM_VBUF1;
    uint8_t *__videobuf = videobuf;
    int rate = 1000000U / avihead->dwRate;              // currently 40mSec per image

    //! test this new frame batch size so that
    //! reading won't spill out of the small (!)
    //! SDRAM
    readNext = FRAMES_BATCH_SIZE;

    //!get number of frames from avi header
#if 1
    int numofframes = avihead->dwLength / avihead->dwScale;
#else
    int numofframes = set_numofframes;
#endif

#ifdef DEBUG
    if( video_frames != numofframes)
        printf( "Discrepancy in number of frames\n");
#endif

#if _USE_LFN
    char lfn[_MAX_LFN + 1];
//    fno2.lfname = lfn;
//    fno2.lfsize = sizeof lfn;
#endif
#if 0
    //! check if needs to write index file for the video RIFF parser
    char fnm[13];
    sprintf( fnm, "v%04d.inx", file_inx);
    FRESULT fr = f_findfirst( dj, &fno2, "", "*.inx");  /* search for index files */
    if( fr == FR_OK && !fno2.fname[0])
    {
        //!build index file
        uint32_t written;
        sprintf( fnm, "v%04d.inx", file_inx);
        fr = f_open( rd, fnm, FA_CREATE_NEW | FA_WRITE);
        if( fr == FR_OK)
        {
            // dump all indexes to file
            int tt1 = hwtimer_now();
            fr = f_write( rd, pInx, numofframes * sizeof( int), &written);
            f_close( rd);
            tt2 = hwtimer_now() - tt1;
        }
    } // endif write index to file
    else //! read index file to memory
    {
    } // endif read index file
#endif

    int retries;
    for( retries = 0; retries < 3; retries++)
    {
        FRESULT fr = f_open_adj( rd, fno.fname, FA_READ);
        if( fr != FR_OK)
        {
            clip_releaseMemory();
#ifdef DEBUG
            printf( "Clip - could not open %s", fno.fname);
#endif
            ///////////////////////////////////////////////////////
            // notify CAT that this thread is done
            clipargs->CAT_notifyDoneCB();
            return;
        }
       //! prepare the cluster table for FAST_SEEK
       /*! Using fast seek feature */
       rd->cltbl = clmt;                                /* Enable fast seek feature (cltbl != NULL) */
       clmt[0] = sizeof( clmt) / sizeof( DWORD);        /* Set table size */
       fr = f_lseek( rd, CREATE_LINKMAP);               /* Create CLMT */
       if( fr != FR_OK)
       {
          puts( "could not create FAST SEEK  table\retrying...\n");
          f_close( rd);
          continue;
       }
       else
          break;
    } // endfor retries
    if( retries == 3)
    {
        clip_releaseMemory();
        ///////////////////////////////////////////////////////
        // notify CAT that this thread is done
        clipargs->CAT_notifyDoneCB();
        return;
    }
    display_window3();

#if 0
    //LTDC_Cmd( DISABLE);
    //LTDC_LayerCmd( LTDC_Layer1, ENABLE);            // background - this is where the clip is displayed
    LTDC_LayerCmd( LTDC_Layer2, ENABLE);            // other data on screen
    LTDC_ReloadConfig( LTDC_IMReload);
#endif

    /////////////////////////////////////////////////////////////////
    //! read the 1st batch of frames to START_SDRAM_VBUF1
    uint32_t addrs   = pInx[curFrame] - pInx[curFrame] % _MAX_SS;
    uint32_t to_addrs = pInx[curFrame + readNext] + _MAX_SS - pInx[curFrame + readNext] % _MAX_SS;
#if 1
    if( clip_getNextChunk( __videobuf, addrs, to_addrs) == 0)
    {
        clip_releaseMemory();
        ///////////////////////////////////////////////////////
        // notify CAT that this thread is done
        clipargs->CAT_notifyDoneCB();
        return;
    }
#else
    f_lseek( rd, addrs);
    f_fastread( rd, (void *)__videobuf, to_addrs - addrs, &red);
#endif
    haveFrames = 1;      //! allow bitmap to start running over 1st second frames
    busy = 1;

    /********************************/
    /** start bitmap display thread */
    /********************************/
    kernel_pid_t bmp_id = thread_create(bitmap_stack, sizeof( bitmap_stack),
                               PRIORITY_MAIN - 5 /* wss -4 NS 02-10 */, CREATE_STACKTEST,
                               bitmap_thread, NULL, "bitmap_thread");
    puts("bitmap_thread created");


    while( 1)
    {
        debug_num = 1;

        //! change all parameters for next second
       curFrame += readNext;
       if( numofframes < curFrame + readNext)
           readNext = numofframes - curFrame - 1;

        //! video buffer for next second
        if( __videobuf == (uint8_t *)START_SDRAM_VBUF1)
            __videobuf = (uint8_t *)START_SDRAM_VBUF2;
        else
            __videobuf = (uint8_t *)START_SDRAM_VBUF1;

        debug_num = 2;


        //! measure start-time
#ifdef HWTIMER
        int startt = hwtimer_now();      //< take time at beginning
#else
        int startt = xtimer_now();       //< take time at beginning
#endif
        uint32_t last_wakeup = xtimer_now();


        //! calculate from and to addresses on SD card
        addrs    = pInx[curFrame] - pInx[curFrame] % _MAX_SS;
        to_addrs = pInx[curFrame + readNext] + _MAX_SS - pInx[curFrame + readNext] % _MAX_SS;

        debug_num = 3;

        if( clip_getNextChunk( __videobuf, addrs, to_addrs) == 0)
        {
            break;                  // bail out of this if reading doesn't work
        }

        debug_num = 4;


//#ifndef DEBUG_FRMS
        // tell the bitmap thread that videobuf had changed
        msg_t msg;
        msg.content.value = (uint32_t)__videobuf;
        msg_send( &msg, bmp_id);
//#endif


        debug_num = 5;


        //////////////////////////////////////////////////
        //! wait all the time needed to 1 full second,
        //! which is the next time we need to read a
        //! new bunch of frames, and also step aside
        //! to let the bitmap thread do its work
//#define HWTIMER_DEBUG
#ifdef HWTIMER_DEBUG
        int waitus = ( /*40000*/rate * readNext) - ( xtimer_now() - startt);
        for( int p = 0; p < waitus / 1000; p++)
        {
            xtimer_usleep( 1000);
            waitus -= 1000;
        }
#else
        int waitus = ( /*40000*/rate * readNext) - ( xtimer_now() - startt);
#endif


        //! NOTE: if numberofframes=1531 then the index of last frame is 1530
        if( curFrame + readNext + 1 >= numofframes || stopExecution)
        {
            break;
        }


/*
         GUI_SetTextMode( GUI_TM_XOR);
         GUI_DispStringAt( str1, 0, 0);
         GUI_DispStringAt( str2, 0, 305);

         GUI_SetTextMode( GUI_TM_TRANS);
         sprintf( str1, "f%d", curFrame);
         sprintf( str2, "%d mSec", waitus / 1000);
         GUI_DispStringAt( str1, 0, 0);
         GUI_DispStringAt( str2, 0, 305);
*/
        if( waitus < 1000) waitus = 1000;
        if( waitus > rate*readNext) waitus = rate*readNext;
        /*****************************************/
        //hwtimer_wait( HWTIMER_TICKS( waitus));
        /*****************************************/

        debug_num = waitus;


        xtimer_usleep( waitus);

        char txt[32];
#if 1
        int _temp = creator_tempGetTemp();

        sprintf( txt, "%02d/%03d[%03d] %d.%02d", errorFrames, curFrame, waitus / 1000 % 1000, _temp / 100, _temp % 100);
        display_updateTextW3( txt, 0);
#endif
        display_setProgressValue( 100 * curFrame / numofframes);

        // move the window up and down
        // MOVED TO BITMAP THREAD BECAUSE IT IS FASTER... display_manageW3Location();

        //UG_Update();


        debug_num = 6;

    } // endwhile all frames

    busy = 0;               //< to stop bitmap thread altogether
    debug_num = 7;

    // bitmap thread is waiting to hear from me?
    if( thread_getstatus( bmp_id) == STATUS_RECEIVE_BLOCKED)
    {
        msg_t msg;
        msg.content.value = (uint32_t)__videobuf;
        msg_send( &msg, bmp_id);
    }

     // wait until bitmap thread finishes showing last frames
     while( haveFrames != -1)
         xtimer_usleep( 40000U);

     f_close( rd);
     clip_releaseMemory();
     debug_num = 8;
     // reset & setup display all over again
     display_main( NULL, NULL);

     debug_num = 9;
     ///////////////////////////////////////////////////////
     // notify CAT that this thread is done
     clipargs->CAT_notifyDoneCB();
} // endfunc

/**
 * @brief opens the index file of the video
 * @return
 */
FRESULT clip_startSDcard( void)
{
UINT    written;
/**
 * @brief  Initialize the SD card
 */
#if 0   // already called in CAT.cpp
    //! Mount drive with adjusting clk speed
    FRESULT f = f_mount_adj( FatFs, "", 1);
    if( f != FR_OK)
        return f;
#endif
    /*! Search a directory for objects and display it */
    FRESULT fr;             /* Return value */
    FILINFO fno2;      /* File information */

    /** must prepare lfn prior to working with these
     *  functions... funny!
     */
    #if _USE_LFN
        char lfn[/*_MAX_LFN*/32 + 1];
        fno.lfname = lfn;
        fno.lfsize = sizeof lfn;

        char lfn2[/*_MAX_LFN*/32 + 1];
        fno2.lfname = lfn2;
        fno2.lfsize = sizeof lfn2;

        char lfn_inx[/*_MAX_LFN*/32 + 1];
    #else
    char lfn_inx[32 + 1];
    #endif


    uint rrr;
    //DWORD clmt[50];                                       /* Cluster link map table buffer */

    /**
     * look into directory and find a file with extension .avi
     */
    fr = f_findfirst( dj, &fno, "", "v*.avi");  /* Start to search for JPEG files with the name started by "dsc" */

    file_inx = 0;
    while (fr == FR_OK && fno.fname[0])
    {   /*< Repeat while an item is found */
        //< open avi file to get FIL handle
        file_inx++;

        replaceExtension( lfn_inx, fno.fname, "inx");

        fr = f_findfirst( dj, &fno2, "", lfn_inx);  /* Start to search for INX files with the name started by "dsc" */

        // found an index file, read and no need to parse riff file all over again
        if( fr == FR_OK && fno2.fname[0])
        {
            fr = f_open( rd, fno2.fname, FA_READ);
            if( fr != FR_OK)
                return fr;

            pInx = malloc( rd->fsize);              // check if success....
            f_lseek( rd, 0);
            f_fastread( rd, (void *)pInx, rd->fsize, &rrr);
            f_close( rd);
            if( f_open( rd, fno.fname, FA_READ) == FR_OK)
            {
                video_frames = riffParser_Go( rd, ( DWORD **)&pInx, /*ahProgBar*/0, 1);
                f_close( rd);
                break;
            }
            riffParser_Go( rd, ( DWORD **)&pInx, 0, true);
        }
        else
        {
#ifdef DEBUG
            printf("Parsing file: %s\n", fno.fname);
#endif
            if( f_open( rd, fno.fname, FA_READ) == FR_OK)
            {
                video_frames = riffParser_Go( rd, ( DWORD **)&pInx, /*ahProgBar*/0, 0);
                f_close( rd);

                if( video_frames > 0)
                {
                    fr = f_open( rd, lfn_inx, FA_CREATE_ALWAYS | FA_WRITE);
                    if( fr == FR_OK)
                    {
                        // dump all indexes to file
                        fr = f_write( rd, pInx, video_frames * sizeof( int), &written);
                        f_close( rd);
                    }
                }
                else
                {
#ifdef DEBUG
                    printf( "Error parsing AVI file!");
#endif
                }
                break;
            }
            break;
        }
    } // endwhile read all AVI files from SD card
    f_closedir( dj);

    return FR_OK;
} // endfunc


/**
 * @brief
 *
 * @param[in]   filen   file name of video file with extension .avi
 *
 * @return
 */
FRESULT clip_readOrMakeIndex( char *filen)
{
char lfn_inx[/*_MAX_LFN*/32 + 1];
uint32_t rrr;

    replaceExtension( lfn_inx, filen, "inx");
    FRESULT fr = f_open( rd, lfn_inx, FA_READ);
    if( fr == FR_OK && rd->fsize > 0)
    {
        pInx = malloc( rd->fsize);              // check if success....
        f_lseek( rd, 0);
        f_fastread( rd, (void *)pInx, rd->fsize, &rrr);
        f_close( rd);
        if( f_open( rd, filen, FA_READ) == FR_OK)
        {
            video_frames = riffParser_Go( rd, ( DWORD **)&pInx, /*ahProgBar*/0, 1);
            f_close( rd);
        }
        else
            riffParser_Go( rd, ( DWORD **)&pInx, 0, true);
    }
    else
    {
#ifdef DEBUG
        printf("Parsing file: %s\n", filen);
#endif
        if( f_open( rd, filen, FA_READ) == FR_OK)
        {
            video_frames = riffParser_Go( rd, ( DWORD **)&pInx, /*ahProgBar*/0, 0);
            f_close( rd);

            if( video_frames > 0)
            {
                fr = f_open( rd, lfn_inx, FA_CREATE_ALWAYS | FA_WRITE);
                if( fr == FR_OK)
                {
                    // dump all indexes to file
                    fr = f_write( rd, pInx, video_frames * sizeof( int), &rrr);
                    f_close( rd);
                }
            }
            else
            {
#ifdef DEBUG
                printf( "Error parsing AVI file!");
#endif
                return FR_NO_FILE;
            }
        } // endif open avi file
        else
            return FR_NO_FILE;
    } // endelse
    return FR_OK;
} // endfunc



/**
 * @brief this function kills both threads, one at a time
 *        and virtually stops showing the clip
 */
void clip_StopAndKillThreads( void)
{
    stopExecution = 1;
}


/**********************************************/
/*     P R I V A T E     F U N C T I O N S    */
/**********************************************/

static void clip_releaseMemory( void)
{
    free( rd);
    free( dj);
    free( pInx);
    rd = dj = pInx = NULL;
}


/**
 * @brief replaces source filename string in the format xxxxx.aaa with
 *        a new string - dst- in the format xxxxx.www
 * @param dst - destination string
 * @param src = source filename
 * @param www - what to replace the extension with
 */
void replaceExtension( char *dst, char *src, char *www)
{
    int i = 0;
    while( src[i] != (char)NULL)
    {
        i++;
    } // endwhile
    i -= 3;                     //< jump to extension
    memcpy( dst, src, i);       // build destination string
    memcpy( &dst[i], www, 3);   //< only 3 characters are allowed as extension
    dst[i+3] = (char)NULL;
}


/**
 * @brief   Reads the next chunk of images from SD card with retries...
 *
 * @param[in]   __videobuf  Address to read the data to
 * @param[in]   from_addrs  Start address to read from
 * @param[in]   to_addrs    End address to read from
 * @return 1 if OK, otherwise Error!
 */
int clip_getNextChunk( uint32_t xvideobuf, long from_addrs, long to_addrs)
{
int red;

    //! right now we need seek every time we read
    //! 2do - think of how to cancel this seek
    for( int x = 0; x < 5; x++)
    {
        if( from_addrs != f_tell( rd)) {
           f_lseek( rd, from_addrs);
        } // endif needs seek


        uint32_t tt1 = xtimer_now();
        FRESULT fa = f_fastread( rd, (void *)xvideobuf, to_addrs - from_addrs, &red);
        uint32_t tt2 = xtimer_now() - tt1;

        chunktime = tt2;
        chunkvol  = red;

        if( fa != FR_OK/*== FR_DISK_ERR*/)
        { //! try to close the file and reopen it...
           errorFrames++;

            printf( "Error reading: %d\n", errorFrames);
            f_close( rd);
            if( f_open_adj( rd, fno.fname, FA_READ) == FR_OK)
            {
                //! prepare the cluster table for FAST_SEEK
                /*! Using fast seek feature */
                rd->cltbl = clmt;                                /* Enable fast seek feature (cltbl != NULL) */

                //xtimer_halt();
                //while(1);
                continue;
            }
            else
                break;
        }
        else break;
    } // endif DISK ERROR

    if( rd->err != 0)
    { // unrecoverable error -- just stop showing clip and try again later
        //xtimer_halt();
        //while(1);
        return 0;
    }
    else
        return 1;
} // endfunc

void clip_getStatus(uint32_t *errors,
                    uint32_t *bmperrors,
                    uint32_t *lastChunkTime,
                    uint32_t *lastChunkVolume)
{
    *errors = errorFrames;
    *bmperrors = bmpErrorFrames;
    *lastChunkTime = chunktime;
    *lastChunkVolume = chunkvol;
}

