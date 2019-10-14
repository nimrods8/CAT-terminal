/*
Module: display.c
 *
 * Copyright (C) 2015 Nimrod Stoler
 *
 * This module is handling the display (TFT-LCD) via ugui and touch panel,
 * currently via SSD2533 module.
 *
 * See usage in .h file.
 *
 * @ingroup     apps
 * @file        display.c
 */
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include "board.h"
#include "thread.h"
#include "malloc.h"
#include "xtimer.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_ltdc.h"
#include "stm32f4xx_dma2d.h"
#include "stm32f4xx_dma.h"
//#include "ft5x06.h"
//#include "sdram.h"
#include "epp.h"
#include "creator.h"
#include "gt811.h"

//! FAT-fs files
#include "fatfs_sd_sdio.h"
#include "ff.h"

#include "aviriff.h"
#include "selftest.h"

#include "LCD.h"
#include "ugui.h"
#include "display.h"

// for version control
#include "../../app/version.h"



/********************************************************
 * Main idea:
 *
 * 1. STM32F... has two manageable LCD layers: background (layer 1) and
 *    foreground (layer 2). A third 'layer' allows to draw a background
 *    color, below both layers.
 * 2. Using both layers is hard for the CPU because of low SDRAM bandwidth!
 * 3. short clips are always displayed on background layer (using bgui)
 * 4. All message box are always displayed on foreground layer using either
 *    fgui or other GUIs
 * 5. rom frames are always displayed in foreground layer.
 *
 * 6. GENERAL GUIDELINES:
 *    (a) a new GUI should be created for every layer characteristics. For example:
 *        screen dimensions, bpp, key color, foreground/background ...
 *    (b) If all is same, use windows inside a GUI to manage what is shown.
 *        See for example fgui's window_4
 *    (c) For the basic dimensions and RGB565 use bgui for background layer and
 *        fgui for foreground layer, which are default layers.
 *        You can show and hide windows on these GUIs even if they are not
 *        currently selected using UG_ShowWindowGui/UG_HideWindowGui
 *    (d) Whenever you select a new GUI using UG_SelectGUI() you must release it
 *        when usage is done using UG_releaseGUI(). Not releasing the GUI is
 *        not allowed.
 *        Default GUIs should not be released (???)
 *
 */



/**********************************************
 * background image
 * currently taking more than 700KB on FLASH !
 **********************************************/
//#include "../../GFX/scr_complete.h"
//#include "../../GFX/scr_complete800x480.h"      // currently only 384KB

/**************************
 *      DATA DEFINES
 **************************/
#define SDRAM_BANK_ADDR     SDRAM_BASE
#define LAYER_1_OFFSET      0x000000        // 1024x600x2Bpp
#define LAYER_2_OFFSET      0x12C000        // 1024x600x2Bpp
#define END_OF_SDRAM        SDRAM_BANK_ADDR+SDRAM_SIZE

#define LAYER_1_FORMAT      LTDC_Pixelformat_RGB565
#define LAYER_2_FORMAT      LTDC_Pixelformat_RGB565

#define PTR_TO_RAM(ptr)         ptr // (((uint32_t)ptr & 0x00ffffff) | 0xd0000000)
#define RAM_TO_PTR(pid, ptr)    ptr // (((uint32_t)ptr & 0x00ffffff) | pid)


// display hardware sync 25-10-16
static mutex_t ltdc_sync = MUTEX_INIT;
static mutex_t dmalloc_mutex = MUTEX_INIT;


//====================================
//      SDRAM   MEMORY   MAP
//
// Chunks of XPHYS_SIZE * YPHYS_SIZE * 3 Bytes per pixel in RGB565 format
//------------------------------------
//address   Type        Description
//D0000000              Video Buffer BKGND
//------------------------------------
//D012C000              Video Buffer FOREGND
//------------------------------------
//D012C000              Video Buffer FOREGND
//------------------------------------
//...
//...
//...
//------------------------------------
//D07A0C90  STATIC      Background image (END_OF_SDRAM - 390000)
//------------------------------------
//D0800000              END OF SDRAM
//------------------------------------

//----- HARDWARE ACCELERATION FUNCTIONS -----
UG_RESULT _HW_DMA0_Fill( uint xs, uint ys, uint xe, uint ye, UG_COLOR c, UG_GUI *gui);
void _HW_layerPosition( UG_GUI *_gui, int x, int y);

//----- MEMORY ALLOCATION FUNCTIONS -----
void *display_malloc( size_t size);
void display_malloc_init( void);


/**************************
 *      Error Codes
 **************************/

/**************************
 *      STATIC DATA
 **************************/
/* GUI structure */
static UG_GUI fgui, bgui, guiSmall, guiClip, bGuiShort;

#ifdef USE_TOUCH_SCREEN
/* Touch structure */
static TP_STATE  TP_State;

/* STACK definition on .ccm area */
char touchpanel_stack[1256]                          __attribute__ ((section(".ccm")));;
#endif

/* Some defines */
#define MAX_OBJECTS        20

/* Window 1 */
UG_WINDOW  window_1;
UG_OBJECT  obj_buff_wnd_1[MAX_OBJECTS];
UG_TEXTBOX textbox1_1;
UG_BUTTON  button1;
UG_PROGRESS_BAR progress1_1;

/* Window 2 */
UG_WINDOW   window_2;
UG_OBJECT   obj_buff_wnd_2[MAX_OBJECTS];
UG_BUTTON   button2_1;
UG_TEXTBOX  textbox2_1;
UG_TEXTBOX  textbox2_2, textbox2_3, textbox2_4;
UG_IMAGE    image2_1;
UG_BUTTON   buttonKeypad[16];
static char buttonText[16][10] = { "*", "7", "4", "1", "0", "8", "5", "2", ".", "9",
                                    "6", "3", "ENTER", "NEXT", "NO", "CLEAR" };
static char Text1[64];           // holds the current message
static char Text2[64];           // holds the current message
static char TextPump[16];        // hold the pump #... message

/* Window 3 */
UG_WINDOW   window_3;
UG_OBJECT   obj_buff_wnd_3[MAX_OBJECTS];
UG_BUTTON   button3_1;
UG_TEXTBOX  textbox3_1;
UG_PROGRESS_BAR progress3_1;
UG_BMP      bmp, smallBMP;
static uint8_t w3_move_in, w3_move_out;

/* Window 4 */
/* used to show rom images while waiting for card */
UG_WINDOW   window_4;
UG_OBJECT   obj_buff_wnd_4[MAX_OBJECTS];
UG_IMAGE    image4_1;

/* Window 5 */
/* used to show EMV app selection buttons, with upto 5 buttons */
UG_WINDOW   window_5;
UG_OBJECT   obj_buff_wnd_5[MAX_OBJECTS];
UG_PROGRESS_BAR progress5_1;
int         window5_pressed;

/* Window 6 */
/* used to show GET PIN Window */
UG_WINDOW   window_6;
UG_OBJECT   obj_buff_wnd_6[MAX_OBJECTS];
UG_PROGRESS_BAR progress6_1;
UG_IMAGE    image6_1;
UG_TEXTBOX  textbox6_1, textbox6_2;

/* menu 1 window */
UG_MENU *menu, *menu_test;

/* Window 7 */
/* just a test... */
UG_WINDOW   window_8;
UG_OBJECT   obj_buff_wnd_8[MAX_OBJECTS];
UG_PROGRESS_BAR progress8_1;
UG_TEXTBOX  textbox8_0, textbox8_1, textbox8_2 /*, textbox8_3*/;
UG_BUTTON   button8_1, button8_2;

/* FSM */
#define STATE_MAIN_MENU                0
#define STATE_BENCHMARK_RUN            1
#define STATE_BENCHMARK_RESULT         2
volatile UG_U32 state;
volatile UG_U32 next_state;

/* virtual keypad callback */
// use MAX_COLUMNS * row + column
// as X is row and Y and columns, so the button
// value should be X * MAX_COLUMNS + Y
void (*buttonPressed)( char button);
static bool isMenuShown;                // reflects whether main menu is currently shown or not.
static int myAddress;                   // copied from CAT module

/* Benchmark */
volatile UG_U32 timer;
volatile UG_U32 hw_acc = 1;
char result_str[30];
UG_S16 xs,ys;
UG_S16 xe,ye;
UG_COLOR c;

/* background image's location on SDRAM */
static uint8_t *backgroundImageAddrs;

/* sync object used to sync display and CAT main loop during display init */
static mutex_t *sync_display;

/******************************************************************************
 * @brief This is part of a future display driver which uses the DMA2D to
 *        draw a line.
 *
 * @param x1 - from x
 * @param y1 - from y
 * @param x2 - to x
 * @param y2 - to y
 * @param c  - color
 * @return UG_RESULT_FAIL if error occured, otherwise UG_RESULT_OK
 */
/* Hardware accelerator for UG_DrawLine (Platform: STM32F4x9) */
UG_RESULT _HW_DrawLine( UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c, /*LTDC_Layer_TypeDef *LTDC_Layerx*/UG_GUI *g)
{
   DMA2D_InitTypeDef DMA2D_InitStruct;


   mutex_lock( &ltdc_sync);


   int bpp = GUIDRV_GetBytesPerPixel( /*FOREGROUND_LAYER*/ /*pixel format*/2);
   //uint32_t pixel_width = GUIDRV_GetLCDWidth();
#if 0
   uint32_t pixel_width = 1 + ((LTDC_Layerx->WHPCR & 0xffff0000) >> 16) - (LTDC_Layerx->WHPCR & 0x0000ffff);
   if( pixel_width == 0)
       pixel_width = GUIDRV_GetLCDWidth();
#else
   uint32_t pixel_width = g->x_dim;
#endif

#if 1
   RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_DMA2D, ENABLE);
   RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_DMA2D, DISABLE);
#endif
   DMA2D_InitStruct.DMA2D_Mode = DMA2D_R2M;
   DMA2D_InitStruct.DMA2D_CMode = DMA2D_RGB565;
   /* Convert UG_COLOR to RGB565 */
   DMA2D_InitStruct.DMA2D_OutputBlue = (c>>3) & 0x1F;
   DMA2D_InitStruct.DMA2D_OutputGreen = (c>>10) & 0x3F;
   DMA2D_InitStruct.DMA2D_OutputRed = (c>>19) & 0x1F;
   DMA2D_InitStruct.DMA2D_OutputAlpha = 0x0F;

   /* horizontal line */
   if ( y1 == y2 )
   {
      DMA2D_InitStruct.DMA2D_OutputOffset = 0;
      DMA2D_InitStruct.DMA2D_NumberOfLine = 1;
      DMA2D_InitStruct.DMA2D_PixelPerLine = x2-x1+1;
   }
   /* vertical line */
   else if ( x1 == x2 )
   {
      DMA2D_InitStruct.DMA2D_OutputOffset = pixel_width /*LCD_PIXEL_WIDTH*/ - 1;
      DMA2D_InitStruct.DMA2D_NumberOfLine = y2-y1+1;
      DMA2D_InitStruct.DMA2D_PixelPerLine = 1;
   }
   else
   {
      mutex_unlock( &ltdc_sync);
      return UG_RESULT_FAIL;
   }
#if 0
   if ( ltdc_work_layer == LAYER_1 )
   {
      DMA2D_InitStruct.DMA2D_OutputMemoryAdd = SDRAM_BANK_ADDR + LAYER_1_OFFSET + /*2*/ bpp * (pixel_width/*LCD_PIXEL_WIDTH*/ * y1 + x1);
   }
   else
   {
      DMA2D_InitStruct.DMA2D_OutputMemoryAdd = SDRAM_BANK_ADDR + LAYER_2_OFFSET + 2*(pixel_width/*LCD_PIXEL_WIDTH*/ * y1 + x1);
   }
#else
   // currently only layer 2 is active
   uint32_t Address = UG_GetCurVideoBuffer(g); // NS 31-07-17    g->hwlayer->CFBAR;
   DMA2D_InitStruct.DMA2D_OutputMemoryAdd = Address/*SDRAM_BANK_ADDR + LAYER_2_OFFSET*/ + bpp /*2*/ * (pixel_width/*LCD_PIXEL_WIDTH*/ * y1 + x1);
#endif
   DMA2D_Init(&DMA2D_InitStruct);
   DMA2D_StartTransfer();
   while(DMA2D_GetFlagStatus(DMA2D_FLAG_TC) == RESET) {};


   mutex_unlock( &ltdc_sync);

   return UG_RESULT_OK;
}

/* Hardware accelerator for UG_FillFrame (Platform: STM32F4x9) */
UG_RESULT _HW_FillFrame( UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c, /*LTDC_Layer_TypeDef* LTDC_Layerx*/UG_GUI *gui)
{
   DMA2D_InitTypeDef      DMA2D_InitStruct;

   mutex_lock( &ltdc_sync);


   int bpp = GUIDRV_GetBytesPerPixel( /*FOREGROUND_LAYER*//* pixel format...*/2);
#if 0
   uint32_t pixel_width = 1 + ((LTDC_Layerx->WHPCR & 0xffff0000) >> 16) - (LTDC_Layerx->WHPCR & 0x0000ffff);
   if( pixel_width == 0)
       pixel_width = GUIDRV_GetLCDWidth();
#endif
   uint32_t pixel_width = gui->x_dim;

   DMA2D_DeInit();
   DMA2D_InitStruct.DMA2D_Mode = DMA2D_R2M;
   DMA2D_InitStruct.DMA2D_CMode = DMA2D_RGB565;
   /* Convert UG_COLOR to RGB565 */
   DMA2D_InitStruct.DMA2D_OutputBlue = (c>>3) & 0x1F;
   DMA2D_InitStruct.DMA2D_OutputGreen = (c>>10) & 0x3F;
   DMA2D_InitStruct.DMA2D_OutputRed = (c>>19) & 0x1F;
   DMA2D_InitStruct.DMA2D_OutputAlpha = 0x0F;
   DMA2D_InitStruct.DMA2D_OutputOffset = (pixel_width/*LCD_PIXEL_WIDTH*/ - (x2-x1+1));
   DMA2D_InitStruct.DMA2D_NumberOfLine = y2-y1+1;
   DMA2D_InitStruct.DMA2D_PixelPerLine = x2-x1+1;
#if 0
   if ( ltdc_work_layer == LAYER_1 )
   {
      DMA2D_InitStruct.DMA2D_OutputMemoryAdd = SDRAM_BANK_ADDR + LAYER_1_OFFSET + 2*(LCD_PIXEL_WIDTH * y1 + x1);
   }
   else
   {
      DMA2D_InitStruct.DMA2D_OutputMemoryAdd = SDRAM_BANK_ADDR + LAYER_2_OFFSET + 2*(LCD_PIXEL_WIDTH * y1 + x1);
   }
#else
#if 1
   uint32_t Address = UG_GetCurVideoBuffer( gui); // NS 31-07-17    gui->hwlayer->CFBAR;           // should be gui->mainVideoBuffer
#else
   uint32_t Address = gui->mainVideoBuffer;
#endif
   DMA2D_InitStruct.DMA2D_OutputMemoryAdd = Address /*SDRAM_BANK_ADDR + LAYER_2_OFFSET*/ + bpp/*2*/ * (pixel_width/*LCD_PIXEL_WIDTH*/ * y1 + x1);
#endif

   DMA2D_Init(&DMA2D_InitStruct);
   DMA2D_StartTransfer();
   while(DMA2D_GetFlagStatus(DMA2D_FLAG_TC) == RESET){}

   mutex_unlock( &ltdc_sync);

   return UG_RESULT_OK;
}


/* Hardware accelerator for UG_FillFrame (Platform: STM32F4x9) */
UG_RESULT _HW_CopyRect( UG_RECT rect, UG_RECT imageRect, void *srcAdd,
                        int inputPixelFormat, /*LTDC_Layer_TypeDef* LTDC_Layerx*/UG_GUI *g)
{
    DMA2D_InitTypeDef      DMA2D_InitStruct;

    mutex_lock( &ltdc_sync);

    int bpp = GUIDRV_GetBytesPerPixel( /*FOREGROUND_LAYER*//* pixel format...*/2);
    //uint32_t pixel_width = GUIDRV_GetLCDWidth();
#if 0
    uint32_t pixel_width = 1 + ((LTDC_Layerx->WHPCR & 0xffff0000) >> 16) - (LTDC_Layerx->WHPCR & 0x0000ffff);
    if( pixel_width == 0)
        pixel_width = GUIDRV_GetLCDWidth();
#else
    uint32_t pixel_width = g->x_dim;
#endif

    // rect is the screen rectangle
    // imageRect is the image dimensions
    uint32_t Address = UG_GetCurVideoBuffer( g); // NS 31-07-2017     g->hwlayer->CFBAR;

    // testing NS 12-4-16!
    if( g->hwlayer == 0) Address = SDRAM_BANK_ADDR + 2 * LAYER_2_OFFSET;

    uint32_t pDst = Address + bpp * ( pixel_width * rect.ys + rect.xs);
    uint16_t scr_width  = UG_getRectWidth( &rect);
    uint16_t scr_height = UG_getRectHeight( &rect);
    uint16_t img_width  = UG_getRectWidth( &imageRect);
    uint16_t img_height = UG_getRectHeight( &imageRect);

    // add extra bytes to image width in case width does not divide by 4
    // then bitmap structure is automatically adjusted
    uint32_t fgor = img_width - scr_width;

    // copy one line at a time...
    DMA2D->CR      = 0x00000000UL | (1 << 9) | (1<<13) | (1<<14) | (1<<16);         // Control Register (Memory to memory and TCIE)
    DMA2D->FGMAR   = (U32)srcAdd;                     // Foreground Memory Address Register (Source address)
    DMA2D->OMAR    = (U32)pDst;                       // Output Memory Address Register (Destination address)
    DMA2D->FGOR    = fgor;                            // Foreground Offset Register (Source line offset)
    DMA2D->OOR     = XSIZE_PHYS - scr_width;          // Output Offset Register (Destination line offset)
    DMA2D->FGPFCCR = inputPixelFormat;                // Foreground PFC Control Register (Defines the input pixel format)
    DMA2D->OPFCCR  = LTDC_Pixelformat_RGB565;         // Output PFC Control Register (Defines the output pixel format)
    DMA2D->NLR     = (U32)(scr_width << 16) |
                     (U16)(scr_height + 1);           // Number of Line Register (Size configuration of area to be transfered)
    DMA2D->CR     |= 1;                               // Start operation
    //
    // Wait until transfer is done
    //
    while (DMA2D->CR & DMA2D_CR_START) {
        //__WFI();                                        // Sleep until next interrupt
    }

    mutex_unlock( &ltdc_sync);

    return UG_RESULT_OK;
}



/**
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * @param void_
 */
void main_ugui_test( void)
{
    LCD_X_DisplayDriver( FOREGROUND_LAYER, LCD_X_INITCONTROLLER, NULL);
    LCD_X_DisplayDriver( BACKGROUND_LAYER, LCD_X_INITCONTROLLER, NULL);

    DMA2D_DeadTimeConfig( 0xF0, DISABLE);
    /**
     * Currently, I can't run with both layers on. It blinks everytime
     * the software is changing the SDRAM contents...
     *
     * Change: I am running both layers because I lowered the scanning
     * frequency of the LCD....
     */
    //LTDC_LayerCmd( LTDC_Layer1, ENABLE/*DISABLE*/);
    //LTDC_LayerCmd( LTDC_Layer2, ENABLE);
    //LTDC_ReloadConfig( LTDC_IMReload);

    /* USE COLOR KEYING TO SHOW BACKGROUND LAYER TOO (Layer_1) */
    LTDC_ColorKeying_InitTypeDef   LTDC_colorkeying_InitStruct;

    /* configure the color Keying */
    LTDC_colorkeying_InitStruct.LTDC_ColorKeyBlue = 0x0000FF & C_BLACK;
    LTDC_colorkeying_InitStruct.LTDC_ColorKeyGreen = (0x00FF00 & C_BLACK) >> 8;
    LTDC_colorkeying_InitStruct.LTDC_ColorKeyRed = (0xFF0000 & C_BLACK) >> 16;

    /* Enable the color Keying for Layer2 */
    LTDC_ColorKeyingConfig(LTDC_Layer2, &LTDC_colorkeying_InitStruct, ENABLE);
    LTDC_ReloadConfig(LTDC_IMReload);
} // endof func main_ugui_test


/**
 * @brief       Reloads background bitmap from SD card
 *
 * @return      1 if load OK, 0 if error
 */
int display_reloadBackgoundImage()
{
    if( display_readBmpToMemory( "backgnd.bmp", backgroundImageAddrs) == NULL)
        return 0;

    // take the bitmap and copy it to LTDC layer
    BITMAP_FILEHEADER *fhead     = (BITMAP_FILEHEADER *)backgroundImageAddrs;
    //< "BM" is the signature. If can't get that --- go out!
    if( fhead->Signature == SIGNATURE_OF_BITMAP/* 0x4D42*/)
        return 1;
    else
        return 0;
}


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
uint8_t *display_readBmpToMemory( const char *fName, uint8_t *address)
{
    FIL *rd = malloc( sizeof(FIL));
    if( rd == NULL)
        return NULL;
    memset( rd, 0, sizeof( FIL));

    int tries;
    for( tries = 0; tries < 3; tries++)
    {
        FRESULT fr = f_open_adj( rd, fName, FA_READ);
        if( fr != FR_OK)
        {
            free( rd);
            return NULL;
        }
        UINT bread;

        if( address == NULL) {
            address = (uint8_t *)display_malloc( rd->fsize);
            if( address == NULL)
            {
                free( rd);
                return NULL;
            }
        } // endif NULL in address
        f_lseek( rd, 0);
        fr = f_fastread( rd, ( void *)address, rd->fsize, &bread);
        if( bread != rd->fsize || fr != FR_OK)
            continue;
        else
            break;
    } // endfor tries

    f_close(rd);
    free( rd);

    if( tries == 3)
    {
        display_free( address);
        address = NULL;
    }
    return address;
} // endfunc

/**
 * @brief       Draws background from SDRAM. If BMP signature is not present
 *              reload file from SD CARD
 *              This is how it is done:
 *              1. The background image is loaded to SDRAM memory. This is an
 *                 8 bit (L8) clut image.
 *              2. The clut is loaded to memory (using process in p.201 RM0385)
 *              3. A memory-to-memory with PFC is utilized in DMA2D, which takes
 *                 the 8-bit data and transforms it to the RGB565 pixel data
 *                 the LTDC is using, and copies it to display buffer memory.
 *
 *
 * @param from_x
 * @param from_y
 * @param to_x
 * @param to_y
 */
void display_drawBackground( uint16_t from_x, uint16_t from_y,
                             uint16_t to_x,   uint16_t to_y)
{
    if( to_y == 0) to_y = YSIZE_PHYS;
    if( to_x == 0) to_x = XSIZE_PHYS;
    if( to_x >= XSIZE_PHYS) to_x = XSIZE_PHYS - 1;
    if( to_y >= YSIZE_PHYS) to_y = YSIZE_PHYS - 1;

    //int pixel = GUIDRV_getPixelFormat( FOREGROUND_LAYER);
    int pixel = GUIDRV_getPixelFormat( BACKGROUND_LAYER);
/*
    if( pixel != LTDC_Pixelformat_RGB565)
        return;
*/
    uint32_t startAdd = ( uint32_t)UG_GetGUIVideoBuffer( BACKGROUND_LAYER);

    int bpp = GUIDRV_GetBytesPerPixel( pixel);
    uint32_t dstadd = (uint32_t)( /*SDRAM_BANK_ADDR + LAYER_2_OFFSET*/startAdd) +
                        bpp * XSIZE_PHYS * ( uint32_t)from_y;

#ifdef BKGND_IMAGE_IN_FLASH
    uint32_t srcadd = (uint32_t )_acscr_complete800x480 +
                        ( bmscr_complete800x480.BitsPerPixel / 8) *
                         XSIZE_PHYS * ( uint32_t)from_y;
#else
    BITMAP_FILEHEADER *fhead     = (BITMAP_FILEHEADER *)backgroundImageAddrs;
    BITMAPINFOHEADER  *bmpheader = (BITMAPINFOHEADER *)( backgroundImageAddrs + sizeof( BITMAP_FILEHEADER));
    uint32_t srcadd = 0;

    if( fhead->Signature == /*SIGNATURE_OF_BITMAP*/ 0x4D42) //< "BM" is the signature. If can't get that --- go out!
    {
        srcadd = (uint32_t)( (uint32_t)fhead->BitsOffset + (uint32_t)fhead) +
                from_y * 1 /*Bpp*/ * XSIZE_PHYS;
    }
    //! could not find bmp signature
    if( srcadd == 0)
    {
        return;
    }
    uint8_t *_clutPtr   = ( uint8_t *)( (uint32_t)bmpheader + (uint32_t)bmpheader->biSize);
#if 0
    uint32_t *tst = malloc( 256 * 4);
    memcpy( tst, _clutPtr, 256*4);
    _HW_DMA_LoadLUT( (LCD_COLOR *)tst, bmpheader->biClrUsed);
    free( tst);
    GUIDRV_CopyBuffer( dstadd, srcadd, XSIZE_PHYS, to_y - from_y);
#else
    if( to_y - from_y >= YSIZE_PHYS - 1)
    {

        mutex_lock( &ltdc_sync);
#if 1
        LTDC_LayerCmd( LTDC_Layer2, DISABLE);
        LTDC_ReloadConfig( LTDC_IMReload);
#else
        //LTDC_LayerCmd( LTDC_Layer1, DISABLE);
#endif


        uint32_t *tst = malloc( 256 * 4);
        memcpy( tst, _clutPtr, 256*4);
        // was 03-08-16... _HW_DMA_LoadLUT( (LCD_COLOR *)tst, bmpheader->biClrUsed);
#if 1
        GUIDRV_shipCLUT( (U32 *)tst, bmpheader->biClrUsed & 0xffff);
#else
        GUIDRV_setPendingClut( (U32 *)_clutPtr, bmpheader->biClrUsed & 0xffff);
#endif
        GUIDRV_CopyBuffer( dstadd, srcadd, XSIZE_PHYS, to_y - from_y);

        //LTDC_LayerCmd( LTDC_Layer1, ENABLE);
        //LTDC_ReloadConfig( LTDC_VBReload);

        free( tst);

#if 0 // DEBUG 23-03
        LTDC_LayerCmd( LTDC_Layer2, ENABLE);
#endif
        LTDC_ReloadConfig( LTDC_IMReload);

        mutex_unlock( &ltdc_sync);
    }
    else
        _HW_display_moveDirect( dstadd, srcadd, _clutPtr, XSIZE_PHYS, to_y - from_y);

#endif
#endif
} // endfunc


/*
 * @brief currently moves bitmap only from L8 to RGB565
 * @param destination - address of video buffer
 * @param source      - address of bitmap
 * @param clutPrt
 * @param xwidth      - width of bytes to copy
 * @param ylines      - number of y lines to copy
 */
#define UG_COLOR_TO_RGR565(x)  (uint16_t)(( (x & 0xf80000) >> 8) | ( (x & 0x00fc00) >> 5) | ((x & 0xf8) >> 3))

void _HW_display_moveDirect( uint16_t *destination,
                             void *source,
                             uint32_t *clutPtr,
                             uint xwidth, uint ylines)
{

    uint xdim = 800;
    uint8_t *srcL8 = (uint8_t *)source;
    uint inxx = 0;
    for( int y = 0; y < ylines; y++)
    {
        for( int x = 0; x < xwidth; x++)
        {
            //uint inx = x + y * xdim;
            uint8_t l8 = srcL8[inxx];
            uint color = clutPtr[l8];
            destination[inxx] = UG_COLOR_TO_RGR565( color);
            inxx++;
        } // endfor vertical pixels
        inxx += xdim - xwidth - 1;
    } // endfor horizontal lines
}


/**********************************************/
/*   C A L L B A C K      F U N C T I O N S   */
/**********************************************/

/**
 * @brief Callback function for the main menu
 * @param msg UG_MESSAGE pointer
 */
void window_1_callback( UG_MESSAGE* msg )
{
   if ( msg->type == MSG_TYPE_OBJECT )
   {
      if ( msg->id == OBJ_TYPE_BUTTON && msg->event == OBJ_EVENT_RELEASED)
      {
         switch( msg->sub_id )
         {
            case BTN_ID_0: /* Toggle green LED */
            {
               UG_invalidateBackground( BACKGROUND_LAYER, 0, YSIZE_PHYS);
               buttonPressed( RESTART_DISPLAY_KEY);      // emulate clear pressed
               UG_WindowShow( BACKGROUND_LAYER, &window_2 );
               break;
            }
#if 0
            case BTN_ID_1: /* Toggle red LED */
            {
               //TOGGLE_RED_LED;
               break;
            }
            case BTN_ID_2: /* Show µGUI info */
            {
               UG_WindowShow( &window_2 );
               break;
            }
            case BTN_ID_3: /* Toggle hardware acceleration */
            {
               if ( !hw_acc )
               {
                  UG_ButtonSetForeColor( &window_1, BTN_ID_3, C_RED );
                  UG_ButtonSetText( &window_1, BTN_ID_3, "HW_ACC OFF" );
                  UG_DriverEnable( DRIVER_DRAW_LINE );
                  UG_DriverEnable( DRIVER_FILL_FRAME );
               }
               else
               {
                  UG_ButtonSetForeColor( &window_1, BTN_ID_3, C_BLUE );
                  UG_ButtonSetText( &window_1, BTN_ID_3, "HW_ACC ON" );
                  UG_DriverDisable( DRIVER_DRAW_LINE );
                  UG_DriverDisable( DRIVER_FILL_FRAME );
               }
               hw_acc = !hw_acc;
               break;
            }
            case BTN_ID_4: /* Start benchmark */
            {
               next_state = STATE_BENCHMARK_RUN;
               break;
            }
            case BTN_ID_5: /* Resize window */
            {
               static UG_U32 tog;

               if ( !tog )
               {
                   UG_AREA area;
                   UG_WindowGetArea( &window_1, &area);
                   UG_WindowResize( &window_1, area.xs, area.ys + 40, area.xe /*239*/, area.ye - 40 /*319-40*/ );
               }
               else
               {
                  uint16_t x = UG_GetXDim();
                  uint16_t y = UG_GetYDim();
                  UG_WindowResize( &window_1, 0, 0, x-1/*239*/, y-1/*319*/ );
               }
               tog = ! tog;
               break;
            }
#endif
         }
      }
   }
}

/**
 * @brief Callback function for the main window
 * @param msg UG_MESSAGE pointer
 */
static uint8_t id0_pressed = 0, id1_pressed = 0, tog = 0;
void window_2_callback( UG_MESSAGE* msg )
{
   // new keypad pressed????
   if( msg->type == MSG_TYPE_WINDOW && msg->event == OBJ_EVENT_KEYPAD)
   { // dot = 46, star = 8, empty = 42
       int ret = (int)msg->src;
       if( ret != EPP_OK && ret != EPP_USER_CANCELED && ret != EPP_STILL_SCANNING &&
               ret != EPP_USER_CLEARED)
       {   // make the return value a row x column value according to the CAT protocol
           uint8_t row, column;
           epp_getRowColumn( (char)ret, &row, &column);
           buttonPressed( (char) row * epp_getColumns() + column);
       }
       else if( ret == EPP_USER_CANCELED)
       {   // if user pressed cancel --- do twice correct to __cancel__
           uint8_t row, column;
           epp_getRowColumn( (char)0x1b, &row, &column);
           buttonPressed( (char) row * epp_getColumns() + column);
           buttonPressed( (char) row * epp_getColumns() + column);
           epp_abortPlaintext();
       }
       else if( ret == /*EPP_USER_CLEARED*/KEY_CORRECT)
       {
           uint8_t row, column;
           epp_getRowColumn( (char)0x08, &row, &column);
           buttonPressed( (char) row * epp_getColumns() + column);
       }
   }


   if ( msg->type == MSG_TYPE_OBJECT )
   {
       if( msg->id == OBJ_TYPE_BUTTON && msg->event == OBJ_EVENT_PRESSED)
       {
           if( msg->sub_id == BTN_ID_0)
               id0_pressed = 1;
           if( msg->sub_id == BTN_ID_1)
               id1_pressed = 1;

           if( id1_pressed && id0_pressed)
           {
               if( !tog)
               {
                   UG_WindowHide( BACKGROUND_LAYER, &window_2);
                   char *buffer = malloc( 1024);
#if 0
                   thread_get_all( buffer, 1024);
#endif
                   //strcpy( buffer, "slafshdflasjdfhasd\nasdljfhadkjfah\nasjkdfhalskdjfh");
                   UG_ConsolePutString( BACKGROUND_LAYER, buffer, &FONT_10X16);
                   free( buffer);
               }
               else
               {
                   UG_WindowShow( BACKGROUND_LAYER, &window_2);
               }
           }
       }

      if ( msg->id == OBJ_TYPE_BUTTON && msg->event == OBJ_EVENT_RELEASED)
      {
          if( msg->sub_id == BTN_ID_0)
              id0_pressed = 0;
          if( msg->sub_id == BTN_ID_1)
              id1_pressed = 0;

         // UG_WindowMove( &window_2, 1, 0);
          if( msg->sub_id >= BTN_ID_0 && msg->sub_id <= BTN_ID_0 + 15)
              buttonPressed( msg->sub_id - BTN_ID_0);

/*
         switch( msg->sub_id )
         {
            case BTN_ID_0:
            {
               UG_WindowHide( &window_2 );
               break;
            }
         }
*/
      }
   }
}
/*******************************************/
/* Callback function for the small clip    */
/* window. Using touch to press the OK     */
/* button results in clip shutting down    */
/*******************************************/
void window_3_callback( UG_MESSAGE* msg )
{
   if ( msg->type == MSG_TYPE_OBJECT )
   {
      if ( msg->id == OBJ_TYPE_BUTTON && msg->event == OBJ_EVENT_RELEASED)
      {
         switch( msg->sub_id )
         {
            /* OK button */
            case BTN_ID_0:
            {
               //UG_WindowShow( BACKGROUND_LAYER, &window_1 );
               clip_StopAndKillThreads();
               break;
            }
         }
      }
   } // endif message type object
   /* if window has been touched (even if it is outside of the LCD bounds
    * it will issue this event... don't worry!
    */
   if(  msg->type == MSG_TYPE_WINDOW)
   {
       switch( msg->event)
       {
           case WND_EVENT_TOUCHED:
               w3_move_in = 1;
               break;
           case WND_EVENT_TIMER:
               break;
           case OBJ_EVENT_KEYPAD:
               if( msg->src == KEY_CANCEL)
                   clip_StopAndKillThreads();
               break;
       } // endswitch
   } // endif
}

///////////////////////////////////////////////////////////////
void window_menu_callback( UG_MESSAGE *msg)
{
static int timeout = 30;
char ttt[16];

    if ( msg->type == MSG_TYPE_OBJECT )
    {
       if ( msg->id == OBJ_TYPE_BUTTON && msg->event == OBJ_EVENT_RELEASED)
       {
          timeout = 30;
          switch( msg->sub_id )
          {
             case BTN_ID_10:
                UG_MenuShow( menu_test);
                break;

             case BTN_ID_11:
                 //UG_setGuiColorKey( FOREGROUND_LAYER, C_TRANSPARENT);
                 // make sure we have the correct CAT address to show
                 sprintf( ttt, "%d", myAddress);
                 UG_TextboxSetText( &window_8, TXB_ID_1, ttt);
                 UG_WindowShow( FOREGROUND_LAYER, &window_8);
                 UG_SetFocus( &window_8, TXB_ID_0);
                 break;

             /* OK button */
             case BTN_ID_13:
             {
                UG_MenuHide( menu);
                //UG_MenuDestroy(menu);
                isMenuShown = false;
                break;
             }
          } // endswitch
       } // endif
    } // endif
    //
    // every second while the window is active we will get this timer event
    if( msg->type == MSG_TYPE_WINDOW && msg->event == WND_EVENT_TIMER)
    {
        timeout--;
        if( timeout <= 0)
        {
            UG_MenuHide( menu);
            isMenuShown = false;
            timeout = 30;
        }
    }

} // endfunc


/**
 * @brief   returns the show status of the main menu
 * @return  true if main menu is currently displayed
 *          false if not
 */
bool display_isMainMenuShown( void)
{
    return isMenuShown;
}



/*******************************************************/
/*   S E L F    T E S T    M E N U    C A L L B A C K  */
/*******************************************************/
// move to a new module later on...
//
uint8_t tests_stack[2048] __attribute__ ((section(".ccm")));
//void display_TestSDram(void *arg);
//void display_TestPrinter( void *arg);

void menu_test_callback( UG_MESSAGE *msg)
{
static int timeout = 30;

    if ( msg->type == MSG_TYPE_OBJECT )
    {
       if ( msg->id == OBJ_TYPE_BUTTON && msg->event == OBJ_EVENT_RELEASED)
       {
          switch( msg->sub_id )
          {  // sdram
             case BTN_ID_10:
                thread_create(( char *)tests_stack, ( int)sizeof( tests_stack),
                PRIORITY_MAIN + 6, CREATE_STACKTEST, ( thread_task_func_t)selftest_TestSDram, ( void *)NULL, "sdram_test");
                break;

             // printer
             case BTN_ID_11:
                thread_create(( char *)tests_stack, ( int)sizeof( tests_stack),
                PRIORITY_MAIN + 6, CREATE_STACKTEST, ( thread_task_func_t)selftest_TestPrinter, ( void *)NULL, "printer_test");
                break;

            // keypad
            case BTN_ID_12:
               thread_create(( char *)tests_stack, ( int)sizeof( tests_stack),
               PRIORITY_MAIN + 6, CREATE_STACKTEST, ( thread_task_func_t)selftest_TestKeypad, ( void *)NULL, "keypad_test");
               break;

           // card reader
           case BTN_ID_13:
              //thread_create(( char *)tests_stack, ( int)sizeof( tests_stack),
              //PRIORITY_MAIN + 6, CREATE_STACKTEST, ( thread_task_func_t)selftest_TestCardRdr, ( void *)NULL, "cardrdr_test");
              break;

          // tag reader
          case BTN_ID_14:
             thread_create(( char *)tests_stack, ( int)sizeof( tests_stack),
             PRIORITY_MAIN + 6, CREATE_STACKTEST, ( thread_task_func_t)selftest_TestTag, ( void *)NULL, "tagrdr_test");
             break;

                   // sd card test
            case BTN_ID_15:
               thread_create(( char *)tests_stack, ( int)sizeof( tests_stack),
               PRIORITY_MAIN + 6, CREATE_STACKTEST, ( thread_task_func_t)selftest_TestSDcard, ( void *)NULL, "sdcrd_test");
               break;

           // gt811 touch screen test
            case BTN_ID_16:
               thread_create(( char *)tests_stack, ( int)sizeof( tests_stack),
               PRIORITY_MAIN + 6, CREATE_STACKTEST, ( thread_task_func_t)selftest_TestGT811, ( void *)NULL, "gt811_test");
               break;


             /* OK button */
             case BTN_ID_17:
             {
                UG_MenuHide( menu_test);
                break;
             }
          } // endswitch
       } // endif
    } // endif
    //
    // every second while the window is active we will get this timer event
    if( msg->type == MSG_TYPE_WINDOW && msg->event == WND_EVENT_TIMER)
    {
        timeout--;
        if( timeout <= 0)
        {
            UG_MenuHide( menu_test);
            timeout = 30;
        }
    }
} // endfunc

/* Callback function for the app selection window */
void window_5_callback( UG_MESSAGE* msg )
{
    // keypad pressed?????? transfer into window handler through this callback
    if( msg->type == MSG_TYPE_WINDOW && msg->event == OBJ_EVENT_KEYPAD)
    {
        window5_pressed = ( int)msg->src;
    }
    if ( msg->type == MSG_TYPE_OBJECT )
    {
       if ( msg->id == OBJ_TYPE_BUTTON && msg->event == OBJ_EVENT_RELEASED)
       {
          //switch( msg->sub_id )
          {
             {
                window5_pressed = msg->sub_id;
             }
          }
       }
    }
}


/* Callback function for the Message Box Thread window */
static int mb_keyadValue;
void window_mb_callback( UG_MESSAGE* msg )
{
   // keypad pressed?????? transfer into window handler through this callback
   if( msg->type == MSG_TYPE_WINDOW && msg->event == OBJ_EVENT_KEYPAD)
   {
      mb_keyadValue = ( int)msg->src;
   }
   else if ( msg->type == MSG_TYPE_OBJECT )
   {
      if ( msg->id == OBJ_TYPE_BUTTON && msg->event == OBJ_EVENT_RELEASED)
      {
         //switch( msg->sub_id )
         {
            {
               window5_pressed = msg->sub_id;
            }
         }
      }
   }
}


/* Callback function for the Message Box Thread window */
static int pin_keyadValue;
void window_pin_callback( UG_MESSAGE* msg )
{
   // keypad pressed?????? transfer into window handler through this callback
   if( msg->type == MSG_TYPE_WINDOW && msg->event == OBJ_EVENT_KEYPAD)
   {
       pin_keyadValue = ( int)msg->src;
   }
}



/**
 * @brief Callback function for a test window for TEXTBOX
 * @param msg UG_MESSAGE pointer
 */
static int window_8_status = 0, w8_inx = 0;
//static char textBoxBuf[16];

void window_8_callback( UG_MESSAGE* msg )
{

    if ( msg->type == MSG_TYPE_OBJECT )
    {
       if ( msg->id == OBJ_TYPE_BUTTON && msg->event == OBJ_EVENT_RELEASED)
       {
          switch( msg->sub_id )
          {
             /* OK */
             case BTN_ID_0:
                 // save data to file....
                 UG_WindowHide( FOREGROUND_LAYER, &window_8);
                 break;

             /* CANCEL */
             case BTN_ID_1:
                 UG_WindowHide( FOREGROUND_LAYER, &window_8);
                 break;
          } // endswitch
       } // endif
    } // endif

   //////////////////////////////////
   //     new keypad pressed????
   //////////////////////////////////
   /*
    *   KEY_ENTER               0x0d
        KEY_CANCEL              0x1b
        KEY_CORRECT             0x08
        KEY_CLEAR
    */
   if( msg->type == MSG_TYPE_WINDOW && msg->event == OBJ_EVENT_KEYPAD)
   {
       int key = (int)msg->src;
       int inx = msg->sub_id - TXB_ID_0;            // index of textbox
       if( key == KEY_CANCEL)
       {
           char *txt = UG_TextboxGetText( &window_8, msg->sub_id);
           txt[0] = NULL;
           UG_TextboxSetText( &window_8, msg->sub_id, txt);
           window_8_status = EPP_USER_CANCELED;
       }
       else if( key == KEY_ENTER)
       {
           switch( msg->sub_id)
           {
               case TXB_ID_0:       // IP
                   {   // Host default IP address
                       char *txt = UG_TextboxGetText( &window_8, msg->sub_id);
                       util_ChangeXmlFileValue( "ssetup.xml", "DEFAULT_HOST_IP", txt);
                   }
                   break;
               case TXB_ID_1:
                   {   // CAT address
                       char *txt = UG_TextboxGetText( &window_8, msg->sub_id);
                       util_ChangeXmlFileValue( "ssetup.xml", "DEVICE_ADDRESS", txt);
                   }
                   break;
               case TXB_ID_2:
                   break;
           } // endswitch
           // do something....
       }
       else if( key == KEY_CORRECT)
       {
           char *txt = UG_TextboxGetText( &window_8, msg->sub_id);
           int  txtlen = strlen( txt);
           if( txtlen > 0)
               txt[txtlen - 1] = NULL;

           UG_TextboxSetText( &window_8, msg->sub_id, txt);
       }
       else
       {   // make the return value a row x column value according to the CAT protocol
           /*
           uint8_t row, column;
           epp_getRowColumn( (char)ret, &row, &column);
           */
           char *txt = UG_TextboxGetText( &window_8, msg->sub_id);
           int  txtlen = strlen( txt);
           txt[txtlen] = key;
           txt[txtlen + 1] = NULL;

           UG_TextboxSetText( &window_8, msg->sub_id, txt);
       }
   }
} // endfunc



/* better rand() function */
UG_U32 randx( void )
{
   static UG_U32 z1 = 12345, z2 = 12345, z3 = 12345, z4 = 12345;
   UG_U32 b;
   b  = ((z1 << 6) ^ z1) >> 13;
   z1 = ((z1 & 4294967294U) << 18) ^ b;
   b  = ((z2 << 2) ^ z2) >> 27;
   z2 = ((z2 & 4294967288U) << 2) ^ b;
   b  = ((z3 << 13) ^ z3) >> 21;
   z3 = ((z3 & 4294967280U) << 7) ^ b;
   b  = ((z4 << 3) ^ z4) >> 12;
   z4 = ((z4 & 4294967168U) << 13) ^ b;
   return (z1 ^ z2 ^ z3 ^ z4);
}


void _HW_pset(UG_S16 x, UG_S16 y, UG_COLOR col, /*LTDC_Layer_TypeDef *LTDC_Layerx*/UG_GUI *gui)
{
   uint32_t addr;
   uint8_t r,g,b;

   if ( x<0 ) return;
   if ( y<0 ) return;


   // because of reading CFBAR below...
   mutex_lock( &ltdc_sync);


   int bpp = GUIDRV_GetBytesPerPixel( /*FOREGROUND_LAYER*/ /*pixel format*/2);
#if 0
   uint32_t pixel_width = 1 + ((LTDC_Layerx->WHPCR & 0xffff0000) >> 16) - (LTDC_Layerx->WHPCR & 0x0000ffff);
   if( pixel_width == 0)
       pixel_width = GUIDRV_GetLCDWidth();
#else
   uint32_t pixel_width = gui->x_dim;
#endif
   addr = bpp * ( x + y * /*LCD_PIXEL_WIDTH*/pixel_width/*GUIDRV_GetLCDWidth()*/);

   //addr<<=1; I am multiplying by bpp

   r = col>>16 & 0xFF;
   g = col>>8 & 0xFF;
   b = col & 0xFF;

   r >>= 3;
   g >>= 2;
   b >>= 3;

   col = b | (g<<5) | (r<<11);
#if 0
   if ( ltdc_work_layer == LAYER_1 )
   {
      sdram_write_16(LAYER_1_OFFSET+addr,col);
   }
   else
   {
      sdram_write_16(LAYER_2_OFFSET+addr,col);
   }
#else
   // can't read the address from the REGISTERS, use current address of gui instead?????
   uint32_t Address = /*gui->currentVideoBuffer*/UG_GetCurVideoBuffer( gui);

   *(uint16_t*) (Address /*SDRAM_BANK_ADDR + LAYER_2_OFFSET*/ + addr) = col;
#endif

   mutex_unlock( &ltdc_sync);

}

/**
 * @brief Being called from the touch thread every time a change happens
 *        in the status of the touch panel
 *
 * @param ctp_data - pointer to the CTP structure
 */
static int isTouch;
int display_periodic( /*_m_ctp_dev *ctp_data*/void)
{
int ret = 0;

#if 0
    if( state == STATE_MAIN_MENU && ctp_data != NULL)
   {    // have fingers on panel?
        if( ctp_data->tpsta)
        {   //! gestures only for first finger
            UG_TouchUpdate( ctp_data->x[0], ctp_data->y[0], TOUCH_STATE_PRESSED, FIRST_FINGER, ctp_data->gestures);
            //! have second finger?
            if( ctp_data->tpsta > 1)
                UG_TouchUpdate( ctp_data->x[1], ctp_data->y[1], TOUCH_STATE_PRESSED, SECOND_FINGER, TOUCH_NO_GESTURE);
        }
        else
        {
            UG_TouchUpdate( -1,-1,TOUCH_STATE_RELEASED, FIRST_FINGER, TOUCH_NO_GESTURE);
            UG_TouchUpdate( -1,-1,TOUCH_STATE_RELEASED, SECOND_FINGER, TOUCH_NO_GESTURE);
        }
   }
#endif
#if 0
   thread_print_all();
#endif

   TS_StateTypeDef ts;
   GT811_GetState( &ts);
   if( ts.touchDetected > 0)
   {
       UG_TouchUpdate( ts.touchX[0], ts.touchY[0], TOUCH_STATE_PRESSED, FIRST_FINGER /*, ctp_data->gestures*/);
       isTouch = 1;
       //! if have 4 fingers (or more) on the screen...
       if( ts.touchDetected > 3)
       {
           int left = 0, right = 0, up = 0, down = 0;
           for( int i = 0; i < 4; i++)
           {
               if( ts.touchX[i] < 70) left++;
               if( ts.touchX[i] > 730) right++;
               if( ts.touchY[i] < 70) up++;
               if( ts.touchY[i] > 400) down++;
           }
           if( left == 2 && right == 2 && up == 2 && down == 2)
           {
               //ret = MENU_REQUEST;
               UG_MenuShow( menu);
               isMenuShown = true;
           }
           isTouch = 0;
       }
   }
   else if( ts.touchDetected == 0 && isTouch == 1)
   {
       isTouch = 0;
       UG_TouchUpdate( 0, 0, TOUCH_STATE_RELEASED, FIRST_FINGER /*, ctp_data->gestures*/);
   }


   ////////////////////////////////////////////////////////
   // get the keys from the EPP if it is ON
   //int ret = epp_waitForPinEnd( NULL);
   int key = epp_getNextKey();
   UG_KeypadUpdate( key);
#if 0
   UG_WINDOW *wnd = UG_isGuiAcceptingKeys( FOREGROUND_LAYER);
   if( wnd != NULL)
   {
       wnd->keypressed = ret;
   }
   else
   {
        // if we have a key we should distribute it according to what's on screen right now
        wnd = UG_isGuiAcceptingKeys( BACKGROUND_LAYER);
        if( wnd != NULL)
        {
            wnd->keypressed = 0;

           // EPP_STILL_SCANNING - waiting for keys. nothing new
           // EPP_USER_CANCELED  - user pressed cancel key
           // EPP_OK - keying has ended as far as EPP is concerned...
           if( ret != EPP_OK && ret != EPP_USER_CANCELED && ret != EPP_STILL_SCANNING &&
                   ret != EPP_USER_CLEARED)
           {   // make the return value a row x column value according to the CAT protocol
               uint8_t row, column;
               epp_getRowColumn( (char)ret, &row, &column);
               buttonPressed( (char) row * epp_getColumns() + column);
           }
           else if( ret == EPP_USER_CANCELED)
           {   // if user pressed cancel --- do twice correct to __cancel__
               uint8_t row, column;
               epp_getRowColumn( (char)0x1b, &row, &column);
               buttonPressed( (char) row * epp_getColumns() + column);
               buttonPressed( (char) row * epp_getColumns() + column);
               epp_abortPlaintext();
           }
           else if( ret == EPP_USER_CLEARED)
           {
               uint8_t row, column;
               epp_getRowColumn( (char)0x08, &row, &column);
               buttonPressed( (char) row * epp_getColumns() + column);
           }
        }
   } // endif dealing with keys and windows
#endif
   UG_Update();

   return ret;
} // endfunc



/**********************************************/
/*  P U B L I C     F U N C T I O N S         */
/**********************************************/

/**
 * @brief This is the main function of the ugui test
 * @return
 */


int debugplc;

int display_main( void (*pressed)( char button), mutex_t *sync_dsp)
{
   // turn LCD off!
   //LTDC_Cmd( DISABLE);
   // init LCD

  if( sync_dsp != NULL)
      sync_display = sync_dsp;

  if( sync_display != NULL)
      mutex_lock( sync_display);

  LTDC_LayerCmd( LTDC_Layer1, DISABLE);
  LTDC_LayerCmd( LTDC_Layer2, DISABLE);

  UG_Create();

  main_ugui_test();
  if( pressed != NULL)
       buttonPressed = pressed;                 // copy call back function

  isMenuShown = false;

  // initialize display malloc of SDRAM
  display_malloc_init();

   //xtimer_usleep( 5000);

   int pixel = GUIDRV_getPixelFormat( BACKGROUND_LAYER);
   int bpp = GUIDRV_GetBytesPerPixel( pixel);
   uint32_t add1 = (uint32_t)display_malloc( XSIZE_PHYS * YSIZE_PHYS * bpp);
   LTDC_LayerAddress( LTDC_Layer1, (uint32_t)( add1));
   LTDC_CLUTCmd( LTDC_Layer1, DISABLE);                     // disable the clut
   LTDC_ReloadConfig( LTDC_IMReload);

   pixel = GUIDRV_getPixelFormat( FOREGROUND_LAYER);
   bpp = GUIDRV_GetBytesPerPixel( pixel);
   uint32_t add2 = (uint32_t)display_malloc( XSIZE_PHYS * YSIZE_PHYS * bpp);
   LTDC_LayerAddress( LTDC_Layer2, (uint32_t)( add2));
   LTDC_ReloadConfig( LTDC_IMReload);


   /* load background image from SD Card and place at end of SDRAM */
#define DDEBUG
#ifdef DDEBUG
   backgroundImageAddrs = display_readBmpToMemory( "backgnd.bmp", 0xd0790000);
#else
   backgroundImageAddrs = display_readBmpToMemory( "backgnd.bmp", NULL);
#endif


   debugplc = 0;

   /*----- Init Foreground µGUI -----*/
   UG_Init(&fgui, (void(*)(UG_S16,UG_S16,UG_S16,UG_S16))NULL,
           XSIZE_PHYS, YSIZE_PHYS, FOREGROUND_LAYER);
   UG_SelectGUI( FOREGROUND_LAYER, &fgui);
   UG_ClearScreens( FOREGROUND_LAYER, C_BLACK);

   //_HW_DMA0_Fill( 0, 0, XSIZE_PHYS - 1, YSIZE_PHYS - 1, C_BLACK, &fgui);

   debugplc = 1;


   /*----- Init Background µGUI -----*/
   UG_Init(&bgui, (void(*)(UG_S16,UG_S16,UG_S16,UG_S16))display_drawBackground,
           XSIZE_PHYS, YSIZE_PHYS, BACKGROUND_LAYER);
   UG_SelectGUI( BACKGROUND_LAYER, &bgui);
   _HW_DMA0_Fill( 0, 0, XSIZE_PHYS - 1, YSIZE_PHYS - 1, C_BLACK, &bgui);

   /* Register hardware acceleration  */
   display_regsiterDrivers( &fgui);
   display_regsiterDrivers( &bgui);

   //!! clear the entire display
   // no need....UG_FillScreen( C_BLACK);
   LTDC_LayerCmd( LTDC_Layer1, ENABLE/*DISABLE*/);

#if 0   // NS 23-03
   LTDC_LayerCmd( LTDC_Layer2, ENABLE);
#endif
   LTDC_ReloadConfig( LTDC_IMReload);

   /* -------------------------------------------------------------------------------- */
   /* Create Window 1 (4x16 LCD-like window)                                           */
   /* -------------------------------------------------------------------------------- */
    #define WINDOW_1_WIDTH      420
    #define WINDOW_1_HEIGHT     240
    #define W1_BUTTON_WIDTH     100
    #define W1_BUTTON_HEIGHT    45

   UG_WindowCreate( &window_1, obj_buff_wnd_1, MAX_OBJECTS, window_1_callback);
   UG_WindowResize( &window_1, (XSIZE_PHYS - WINDOW_1_WIDTH) / 2,
                               (YSIZE_PHYS - WINDOW_1_HEIGHT) / 2,
                               WINDOW_1_WIDTH + (XSIZE_PHYS - WINDOW_1_WIDTH) / 2,
                               WINDOW_1_HEIGHT + (YSIZE_PHYS - WINDOW_1_HEIGHT) / 2);
   UG_WindowSetTitleText( &window_1, "Error" );
   UG_WindowSetTitleTextFont( &window_1, &FONT_16X26);
   UG_WindowSetBackColor( & window_1, C_WHITE);

   UG_WindowSetBackColor( &window_1, C_WHITE);
   uint8_t style = UG_WindowGetStyle( &window_1);
   //UG_WindowSetStyle( &window_1, style & (~WND_STYLE_SHOW_TITLE));     // dont show title for this window
   UG_TextboxCreate( &window_1, &textbox1_1, TXB_ID_0, 0, 0,
                     UG_WindowGetInnerWidth( &window_1 ),
                     UG_WindowGetInnerHeight( &window_1) - W1_BUTTON_HEIGHT - 10);
   UG_TextboxSetFont( &window_1, TXB_ID_0, &FONT_GISHA_38);
   UG_TextboxSetText( &window_1, TXB_ID_0, "?" );
   UG_TextboxSetAlignment( &window_1, TXB_ID_0, ALIGN_CENTER);
   UG_TextboxSetForeColor( &window_1, TXB_ID_0, C_DARK_SLATE_GRAY);
   UG_TextboxSetBackColor( &window_1, TXB_ID_0, C_WHITE);
   UG_TextboxSetHSpace( &window_1, TXB_ID_0, 0 );

   UG_ButtonCreate( &window_1, &button1, BTN_ID_0, (WINDOW_1_WIDTH - W1_BUTTON_WIDTH) / 2,
                    WINDOW_1_HEIGHT - W1_BUTTON_HEIGHT * 2,
                    (WINDOW_1_WIDTH - W1_BUTTON_WIDTH) / 2 + W1_BUTTON_WIDTH,
                    WINDOW_1_HEIGHT - W1_BUTTON_HEIGHT * 2 + W1_BUTTON_HEIGHT);
   /* Configure Button 1 */
   UG_ButtonSetFont( &window_1, BTN_ID_0, &FONT_GISHA_38);
   UG_ButtonSetBackColor( &window_1, BTN_ID_0, C_LIME );
   UG_ButtonSetText( &window_1, BTN_ID_0, "OK" );
   UG_ButtonHide( &window_1, BTN_ID_0);

   UG_ProgressBarCreate( &window_1, &progress1_1, TXB_ID_1, 10,
                          UG_WindowGetInnerHeight( &window_1) - 25,
                          UG_WindowGetInnerWidth( &window_1 ) - 10,
                          UG_WindowGetInnerHeight( &window_1) - 5);
   UG_ProgressBarSetForeColor( &window_1, TXB_ID_1, C_GREEN, C_WHITE);




   /* -------------------------------------------------------------------------------- */
   /* Create Window 2 (µGUI Info)                                                      */
   /* -------------------------------------------------------------------------------- */
   UG_WindowCreate( &window_2, obj_buff_wnd_2, MAX_OBJECTS, window_2_callback );
   uint8_t style1 = UG_WindowGetStyle( &window_2);
   UG_WindowSetStyle( &window_2, style1 & (~WND_STYLE_SHOW_TITLE));     // dont show title for the buttons window
   UG_WindowSetTitleText( &window_2, "Keypad Entry" );
   UG_WindowSetTitleTextFont( &window_2, &FONT_12X20 );
   UG_WindowSetBackColor( &window_2, C_TRANSPARENT/*TO_UG_COLOR( 0, 40, 56, 69)*/);

   //------------------------***** build digit buttons *****---------------------------
   UG_RECT buttonRect = { 150, 140, 650, 410+25 };
   UG_RECT digitsRect = { buttonRect.xs, buttonRect.ys,
                          buttonRect.xe/* + 320*/, buttonRect.ye/* + 420*/};

   //UG_WindowResize( &window_2, buttonRect.xs, buttonRect.ys, buttonRect.xe + 5, buttonRect.ye + 5 /*30*/);
   UG_WindowResize( &window_2, 0, 0, XSIZE_PHYS, YSIZE_PHYS);

   uint32_t xbuttons = 4, ybuttons = 4;
   uint32_t YgapBetweenButtons = 4;
   uint32_t XgapBetweenButtons = 5;

   uint32_t singleButtonHeight = ( UG_getRectHeight( &buttonRect) -
                                   ( ybuttons - 1) * YgapBetweenButtons) / ybuttons;
   // I am dividing by digitx+1 because I want the last column to be double in width
   uint32_t digitButtonWidth   = ( UG_getRectWidth( &digitsRect) -
                                   ( xbuttons - 1) * XgapBetweenButtons) / ( xbuttons + 1);
   int binx = 0;

#ifdef USE_TOUCH_SCREEN_BUTTONS

   int _x = buttonRect.xs /* was 0 */; //digitsRect.x0;
   int _y = buttonRect.ys + ybuttons * singleButtonHeight + YgapBetweenButtons * (ybuttons - 1);//digitsRect.y1;
   for( uint32_t bx = 0; bx < xbuttons; bx++)
   {
       if( bx == xbuttons - 1) digitButtonWidth = digitButtonWidth * 2;

       for( uint32_t by = 0; by < ybuttons; by++)
       {
           int buttoninx = bx + by * xbuttons;

           UG_ButtonCreate( &window_2, &buttonKeypad[binx], BTN_ID_0 + buttoninx,
                            _x, _y - singleButtonHeight,
                            _x + digitButtonWidth, _y);

           _y -= singleButtonHeight + YgapBetweenButtons;

           UG_ButtonSetFont( &window_2, BTN_ID_0 + buttoninx, &FONT_TAHOMA_48);
           UG_ButtonSetText( &window_2, BTN_ID_0 + buttoninx, buttonText[binx]);
           UG_ButtonSetBackColor( &window_2, BTN_ID_0 + buttoninx, TO_UG_COLOR( 0, 93, 93, 93));     // #'s
           UG_ButtonSetForeColor( &window_2, BTN_ID_0 + buttoninx, C_WHITE);
           UG_ButtonSetAlternateForeColor( &window_2, BTN_ID_0 + buttoninx, C_SILVER);

           if( bx == xbuttons - 1)
           {
               char *buf;
               buf = UG_ButtonGetText( &window_2, BTN_ID_0 + buttoninx);
               if( !strcmp( buf, buttonText[12]))
                   UG_ButtonSetBackColor( &window_2, BTN_ID_0 + buttoninx, TO_UG_COLOR( 0, 46, 66, 81));     //DARK BLUE
               else if( !strcmp( buf, buttonText[13]))
                   UG_ButtonSetBackColor( &window_2, BTN_ID_0 + buttoninx, TO_UG_COLOR( 0, 51, 51, 51));     //VERY DARK GRAY
               else if( !strcmp( buf, buttonText[14]))
                   UG_ButtonSetBackColor( &window_2, BTN_ID_0 + buttoninx, TO_UG_COLOR( 0, 228, 131, 0));    //ORANGE
               else if( !strcmp( buf, buttonText[15]))
                   UG_ButtonSetBackColor( &window_2, BTN_ID_0 + buttoninx, TO_UG_COLOR( 0, 94, 0, 8));       //RED
           }
           binx++;
       } // endfor by
       _y = buttonRect.ys + ybuttons * singleButtonHeight + YgapBetweenButtons * (ybuttons - 1);//digitsRect.y1; //digitsRect.y1;
       _x += digitButtonWidth + XgapBetweenButtons;
   } // endfor b
#endif // USE_TOUCH_SCREEN_BUTTONS


   //! add the 1st text object for the messages
   UG_TextboxCreate( &window_2, &textbox2_1, TXB_ID_0, 32, 95, 740+32, 107+74);
   UG_TextboxSetFont( &window_2, TXB_ID_0, &/*FONT_32X53*/FONT_TAHOMA_52);
   UG_TextboxSetText( &window_2, TXB_ID_0, "-" );
   UG_TextboxSetAlignment( &window_2, TXB_ID_0, ALIGN_CENTER_LEFT/*ALIGN_BOTTOM_CENTER*/);
   UG_TextboxSetForeColor( &window_2, TXB_ID_0, C_YELLOW_GREEN/* was blue: TO_UG_COLOR( 0, 149, 199, 248)*/ /* was this: 27-7-17>  TO_UG_COLOR( 0, 60, 63, 136)*/);
   UG_TextboxSetBackColor( &window_2, TXB_ID_0, C_TRANSPARENT);
   UG_TextboxSetHSpace( &window_2, TXB_ID_0, 0 );

   //! add the 2nd text object for the messages
#define USE_CURSOR
#ifndef USE_CURSOR
   UG_TextboxCreate( &window_2, &textbox2_2, TXB_ID_1, 32, 194-7-5, 32+740, 194+74);
#else
   UG_TextboxCreate( &window_2, &textbox2_2, TXB_ID_1, 32, 194-7-5, 32+740-15 /*for cursor*/, 194+74);
#endif // USE_CURSOR
   UG_TextboxSetFont( &window_2, TXB_ID_1, &/*FONT_32X53*/FONT_TAHOMA_52);
   UG_TextboxSetText( &window_2, TXB_ID_1, "-" );
   UG_TextboxSetAlignment( &window_2, TXB_ID_1, ALIGN_CENTER_LEFT/*ALIGN_BOTTOM_CENTER*/);
   UG_TextboxSetForeColor( &window_2, TXB_ID_1, /* was blue: TO_UG_COLOR( 0, 149, 199, 248)*//*was this:>  TO_UG_COLOR( 0, 60, 63, 136)*/C_YELLOW_GREEN);
   UG_TextboxSetBackColor( &window_2, TXB_ID_1, C_TRANSPARENT);
   UG_TextboxSetHSpace( &window_2, TXB_ID_1, 0 );

#ifdef USE_CURSOR
   UG_U8 txstyle = UG_TextboxGetStyle( &window_2, TXB_ID_1);
   txstyle |= TXB_STYLE_CURSOR_ON;
   UG_TextboxSetStyle( &window_2, TXB_ID_1, txstyle);
#endif // USE_CURSOR



   //! this is for the pump number and other important information
   UG_TextboxCreate( &window_2, &textbox2_3, TXB_ID_2, 4, 4, 260+4, 4+35);
   UG_TextboxSetFont( &window_2, TXB_ID_2, &/*FONT_32X53*/FONT_16X26);
   UG_TextboxSetText( &window_2, TXB_ID_2, "" );
   UG_TextboxSetAlignment( &window_2, TXB_ID_2, ALIGN_CENTER_LEFT);
   UG_TextboxSetForeColor( &window_2, TXB_ID_2, TO_UG_COLOR( 0, 22, 96, 50));
   UG_TextboxSetBackColor( &window_2, TXB_ID_2, C_TRANSPARENT);
   UG_TextboxSetHSpace( &window_2, TXB_ID_2, 0 );


   //! this is for the pump number and other important information
   UG_TextboxCreate( &window_2, &textbox2_4, TXB_ID_3, 3, YSIZE_PHYS - 35 - 7, 630 + 3, YSIZE_PHYS - 7);
   UG_TextboxSetFont( &window_2, TXB_ID_3, &/*FONT_32X53*/FONT_16X26);
   UG_TextboxSetText( &window_2, TXB_ID_3, " " );
   UG_TextboxSetAlignment( &window_2, TXB_ID_3, ALIGN_CENTER_LEFT);
   UG_TextboxSetForeColor( &window_2, TXB_ID_3, TO_UG_COLOR( 0, 22, 96, 50));
   UG_TextboxSetBackColor( &window_2, TXB_ID_3, C_TRANSPARENT /*TO_UG_COLOR( 0, 50, 50, 50)*/);
   UG_TextboxSetHSpace( &window_2, TXB_ID_3, 0 );

#define SHOW_IMAGE_TEST
#ifdef SHOW_IMAGE_TEST
   // create an image object the size of the screen
   UG_ImageCreate( &window_2, &image2_1, IMG_ID_100, 730, 0, 730+64, 64);
   //! REMOVE RESIZE STYLE
   UG_U8 imgstyle2 = UG_ImageGetStyle( &image2_1);
   imgstyle2 &= ~IMG_STYLE_RESIZE;
   imgstyle2 |= IMG_STYLE_FRAME | IMG_STYLE_HAVE_TRANSPARENCY;
   UG_ImageSetStyle( &image2_1, imgstyle2);

   //uint32_t offset = ( SDRAM_BASE + SDRAM_SIZE - XSIZE_PHYS * YSIZE_PHYS * 4 - 100) & 0xFFFFFFFC;
   //uint32_t offset = (uint32_t)display_malloc( 32000);
   uint8_t *offset = display_readBmpToMemory( "fpump.bmp", NULL);
   if( offset != NULL)
   {
       UG_ImageSetTransparentColor( &image2_1, C_WHITE);

       BITMAP_FILEHEADER *fhead = (BITMAP_FILEHEADER *)offset;
       uint8_t *pStart = ( uint8_t *)( offset + fhead->BitsOffset);
       BITMAPINFOHEADER *bmpheader = (BITMAPINFOHEADER *)( offset + sizeof( BITMAP_FILEHEADER));
       if( bmpheader->biBitCount == 24)
       {
           smallBMP.bpp = bmpheader->biBitCount;
           smallBMP.height = bmpheader->biHeight;
           smallBMP.width  = bmpheader->biWidth;
           smallBMP.p = pStart;
       }
       UG_ImageSetBMP( &window_2, IMG_ID_100, &smallBMP);
       UG_ImageShow( &window_2, IMG_ID_100);
   } // endif
#endif

#if 0  // NS 23-03
   //! FOR DEBUGGING PURPOSES...
   LTDC_LayerCmd( LTDC_Layer2, ENABLE);
#endif
   UG_invalidateBackground( BACKGROUND_LAYER, 0, YSIZE_PHYS);
   UG_WindowShow( BACKGROUND_LAYER, &window_2 );              //! and the buttons window
   UG_WindowHide( BACKGROUND_LAYER, &window_2 );

   // 13-7-16     display_free( offset);                   //! release the display memory allocated for the image

   /* -------------------------------------------------------------------------------- */
   /* Create Window 4 (Frames Window)                                                      */
   /* -------------------------------------------------------------------------------- */
   UG_WindowCreate( &window_4, obj_buff_wnd_4, MAX_OBJECTS, window_2_callback );
   uint8_t style4 = UG_WindowGetStyle( &window_4);
   UG_WindowSetStyle( &window_4, style4 & (~WND_STYLE_SHOW_TITLE));     // dont show title for the buttons window
   UG_WindowSetTitleText( &window_4, "rom frames" );
   UG_WindowSetTitleTextFont( &window_4, &FONT_12X20 );
   UG_WindowSetBackColor( &window_4, C_TRANSPARENT);

   // create an image object the size of the screen
   UG_ImageCreate( &window_4, &image4_1, IMG_ID_100, 0, 0, 800, 480);
   //! REMOVE RESIZE STYLE
   UG_U8 imgstyle = UG_ImageGetStyle( &image4_1);
   imgstyle &= ~IMG_STYLE_RESIZE;
   imgstyle |= IMG_STYLE_FRAME;
   UG_ImageSetStyle( &image4_1, imgstyle);

   //UG_WindowHide( BACKGROUND_LAYER, &window_4);
   //UG_ImageHide( &window_4, IMG_ID_100);

   debugplc = 4;


   UG_Init( &bGuiShort,
            (void(*)(UG_S16,UG_S16,UG_S16,UG_S16))/*display_drawBackground*/NULL,
            XSIZE_PHYS, YSIZE_PHYS /*60*/, BACKGROUND_LAYER);
   UG_SelectGUI( BACKGROUND_LAYER, &bGuiShort);
   UG_ClearScreens( BACKGROUND_LAYER, C_BLACK);
   /* Register hardware acceleration  */
   /* 2do: add PFC to these functions */
   display_regsiterDrivers( &bGuiShort);
   UG_releaseGUI( BACKGROUND_LAYER, &bGuiShort);


   /* -------------------------------------------------------------------------------- */
   /* Create Window 3 (Benchmark Result)                                               */
   /* -------------------------------------------------------------------------------- */
   /* Init µGUI for the small clip information window */
   UG_Init( &guiSmall,
            (void(*)(UG_S16,UG_S16,UG_S16,UG_S16))/*display_drawBackground*/NULL,
            XSIZE_PHYS, YSIZE_PHYS /*60*/, FOREGROUND_LAYER);
   UG_SelectGUI( FOREGROUND_LAYER, &guiSmall);
   UG_ClearScreens( FOREGROUND_LAYER, C_BLACK);
   /* Register hardware acceleration  */
   /* 2do: add PFC to these functions */
   display_regsiterDrivers( &guiSmall);
   UG_releaseGUI( FOREGROUND_LAYER, &guiSmall);


   debugplc = 5;


   //!! clear the entire display
   UG_ClearScreens( FOREGROUND_LAYER, C_BLACK);

   UG_WindowCreate( &window_3, obj_buff_wnd_3, MAX_OBJECTS, window_3_callback );
   //UG_WindowSetTitleText( &window_3, "Benchmark Result" );
   //UG_WindowSetTitleTextFont( &window_3, &FONT_8X8 );
   style = UG_WindowGetStyle( &window_3);
   style = style & (~WND_STYLE_SHOW_TITLE);
   style = style & (~WND_STYLE_3D);
   style |= WND_STYLE_TIMER;
   UG_WindowSetStyle( &window_3, style);                // remove the frame around the window also
   UG_WindowSetTitleTextFont( &window_3, &FONT_8X8);
   UG_WindowSetBackColor( &window_3, /*C_RED*/C_BLACK);            // this is the color key
   //UG_WindowResize( &window_3, 0, 0, UG_GetXDim(FOREGROUND_LAYER), UG_GetYDim(FOREGROUND_LAYER));
   UG_WindowResize( &window_3, 0, 0, UG_GetXDim(FOREGROUND_LAYER), /*UG_GetYDim(FOREGROUND_LAYER)*/ 50);

   /* Create Button 1 */
   UG_ButtonCreate( &window_3, &button3_1, BTN_ID_0, 40, 1, 120, 49);
   UG_ButtonSetFont( &window_3, BTN_ID_0, &FONT_10X16);
   UG_ButtonSetText( &window_3, BTN_ID_0, "Exit" );
   UG_ButtonSetBackColor( &window_3, BTN_ID_0, TO_UG_COLOR( 0, 93, 93, 93));     // #'s
   UG_ButtonSetForeColor( &window_3, BTN_ID_0, C_WHITE);
   UG_ButtonSetAlternateForeColor( &window_3, BTN_ID_0, C_SILVER);

   UG_TextboxCreate( &window_3, &textbox3_1, TXB_ID_0, 650, 10, 780, 30);
   UG_TextboxSetFont( &window_3, TXB_ID_0, &/*FONT_10X16*/FONT_MSOFT_SS16B);
   UG_TextboxSetText( &window_3, TXB_ID_0, "0000/0000 \x91 \x92" );
   UG_TextboxSetAlignment( &window_3, TXB_ID_0, ALIGN_CENTER_RIGHT);
   UG_TextboxSetForeColor( &window_3, TXB_ID_0, TO_UG_COLOR( 0, 149, 199, 248));
   UG_TextboxSetBackColor( &window_3, TXB_ID_0, C_RED);

   UG_ProgressBarCreate( &window_3, &progress3_1, TXB_ID_0, 200, 15, 600, 35);
   UG_ProgressBarSetForeColor( &window_3, TXB_ID_0, C_GREEN, C_WHITE);

   debugplc = 6;

   /* ----------------------------- */
   /* Init µGUI for the clip window */
   /* ----------------------------- */
   UG_Init( &guiClip, NULL, XSIZE_PHYS, YSIZE_PHYS, BACKGROUND_LAYER);
   /* Register hardware acceleration  */
   display_regsiterDrivers( &guiClip);
   guiClip.active_window = NULL;


   debugplc = 7;
//#define DEBUG_MENU
#ifdef DEBUG_MENU
   {
       /////////////////////////////////////////////////////////////////////////////
       // define a debug menu
       //
       // LARGE FONT:  22x36
       // MEDIUM FONT: 16x26
       // SMALL FONT:  10x16

       UG_MENU *menu1 = (UG_MENU *)malloc( sizeof( UG_MENU));
       menu1->style = MENU_STYLE_MEDIUM_FONT;
       menu1->menu_title = "Settings...";
       UG_FONT *fnt = &FONT_22X36;
       int f_y_size = 36+10;
       if(( menu1->style & 3) == MENU_STYLE_MEDIUM_FONT)
       {
           fnt = &FONT_16X26;
           f_y_size = 26+10;
       }
       if( ( menu1->style & 3) == MENU_STYLE_SMALL_FONT)
       {
           fnt = &FONT_10X16;
           f_y_size = 16+10;
       }
       UG_WindowCreate( &menu1->menu_wnd, menu1->menu_wnd_objs, 20, window_3_callback );
       if( menu1->menu_title[0] != NULL)
       {
           UG_WindowSetTitleText( &menu1->menu_wnd, menu1->menu_title);
           UG_WindowSetTitleTextFont( &menu1->menu_wnd, fnt);
           style = UG_WindowGetStyle( &menu1->menu_wnd);
           style = style | (WND_STYLE_SHOW_TITLE);
       }
       else
           style = style & (~WND_STYLE_SHOW_TITLE);

       style = style | (WND_STYLE_3D);
       UG_WindowSetStyle( &menu1->menu_wnd, style);                // remove the frame around the window also
       UG_WindowSetBackColor( &menu1->menu_wnd, TO_UG_COLOR( 0, 240, 240, 240));          // this is the color key
       int h = UG_GetXDim(FOREGROUND_LAYER);
       int v = UG_GetYDim(FOREGROUND_LAYER);
       int xs, ys, xe, ye;
       xs = h / 2 - 400 / 2;
       xe = xs + 400;
       ys = v / 2 - f_y_size * 5 / 2;
       ye =ys + f_y_size * 5;
       int margin_up = 10, margin_left = 0;
       UG_WindowResize( &menu1->menu_wnd, xs, ys, xe, ye);

       char img1_name[] = "/images/boat.bmp";
       int hgt = (UG_WindowGetInnerHeight( &menu1->menu_wnd) - margin_up) / 4;
       int wth = UG_WindowGetInnerWidth( &menu1->menu_wnd) - 2 - margin_left;
       int btn1_xs, btn1_xe, btn1_ys, btn1_ye;

       btn1_xs = margin_left;
       btn1_ys = margin_up;
       btn1_xe = wth;
       btn1_ye = btn1_ys + hgt;
       UG_MENUITEM *menu_item1 = (UG_MENUITEM *)malloc( sizeof( UG_MENUITEM));
       UG_ButtonCreate( &menu1->menu_wnd, &menu_item1->button, BTN_ID_10, btn1_xs, btn1_ys, btn1_xe, btn1_ye);
       UG_ButtonSetFont( &menu1->menu_wnd, BTN_ID_10, &FONT_16X26);
       UG_ButtonSetText( &menu1->menu_wnd, BTN_ID_10, "Setup" );
       UG_ButtonSetBackColor( &menu1->menu_wnd, BTN_ID_10, C_TRANSPARENT /*TO_UG_COLOR( 0, 93, 93, 93)*/);     // #'s
       UG_ButtonSetForeColor( &menu1->menu_wnd, BTN_ID_10, C_BLACK);
       UG_ButtonSetAlternateForeColor( &menu1->menu_wnd, BTN_ID_10, C_BLUE);
       UG_ButtonSetStyle( &menu1->menu_wnd, BTN_ID_10, BTN_STYLE_NO_FILL | BTN_STYLE_2D);
       UG_U8 algn = ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT;
       UG_ButtonSetAlignment( &menu1->menu_wnd, BTN_ID_10, algn);

       int img_width = 32;
       int img_height = 32;
       // create an image object the size of the screen
       UG_ImageCreate( &menu1->menu_wnd, &menu_item1->img, IMG_ID_100, btn1_xs + 10,
                       btn1_ys + (btn1_ye-btn1_ys) / 2 - img_height/2, btn1_xs + 10 + img_width,
                       btn1_ys + (btn1_ye-btn1_ys) / 2 - img_height/2 + img_height);
       //! REMOVE RESIZE STYLE
       UG_U8 imgstyle2 = UG_ImageGetStyle( &menu_item1->img);
       imgstyle2 &= ~IMG_STYLE_RESIZE;
       imgstyle2 |= /*IMG_STYLE_FRAME |*/ IMG_STYLE_HAVE_TRANSPARENCY;
       UG_ImageSetStyle( &menu_item1->img, imgstyle2);

       //uint32_t offset = ( SDRAM_BASE + SDRAM_SIZE - XSIZE_PHYS * YSIZE_PHYS * 4 - 100) & 0xFFFFFFFC;
       //uint32_t offset = (uint32_t)display_malloc( 32000);
       uint8_t *offset = display_readBmpToMemory( img1_name, NULL);
       UG_ImageSetTransparentColor( &menu_item1->img, C_WHITE);
       BITMAP_FILEHEADER *fhead = (BITMAP_FILEHEADER *)offset;
       uint8_t *pStart = ( uint8_t *)( offset + fhead->BitsOffset);
       BITMAPINFOHEADER *bmpheader = (BITMAPINFOHEADER *)( offset + sizeof( BITMAP_FILEHEADER));
       if( bmpheader->biBitCount == 24)
       {
           menu_item1->bmp.bpp = bmpheader->biBitCount;
           menu_item1->bmp.height = bmpheader->biHeight;
           menu_item1->bmp.width  = bmpheader->biWidth;
           menu_item1->bmp.p = pStart;
       }
       UG_ImageSetBMP( &menu1->menu_wnd, IMG_ID_100, &menu_item1->bmp);
       UG_ImageShow( &menu1->menu_wnd, IMG_ID_100);

       UG_MENUITEM *menu_item2 = (UG_MENUITEM *)malloc( sizeof( UG_MENUITEM));
       UG_ButtonCreate( &menu1->menu_wnd, &menu_item2->button, BTN_ID_11, 0 + margin_left, margin_up + hgt /*+1*/, wth +  + margin_left, 2 * hgt + margin_up);
       UG_ButtonSetFont( &menu1->menu_wnd, BTN_ID_11, &FONT_16X26);
       UG_ButtonSetText( &menu1->menu_wnd, BTN_ID_11, "Jump to Loader" );
       UG_ButtonSetBackColor( &menu1->menu_wnd, BTN_ID_11, C_TRANSPARENT /*TO_UG_COLOR( 0, 93, 93, 93)*/);     // #'s
       UG_ButtonSetForeColor( &menu1->menu_wnd, BTN_ID_11, C_BLACK);
       UG_ButtonSetAlternateForeColor( &menu1->menu_wnd, BTN_ID_11, C_BLUE);
       UG_ButtonSetStyle( &menu1->menu_wnd, BTN_ID_11, BTN_STYLE_NO_FILL | BTN_STYLE_2D);
       algn = ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT;
       UG_ButtonSetAlignment( &menu1->menu_wnd, BTN_ID_11, algn);

       UG_MENUITEM *menu_item3 = (UG_MENUITEM *)malloc( sizeof( UG_MENUITEM));
       UG_ButtonCreate( &menu1->menu_wnd, &menu_item3->button, BTN_ID_12, 0 +  + margin_left, margin_up + 2*(hgt/*+1*/), wth + margin_left, margin_up + 2*(hgt/*+1*/)+hgt);
       UG_ButtonSetFont( &menu1->menu_wnd, BTN_ID_12, &FONT_16X26);
       UG_ButtonSetText( &menu1->menu_wnd, BTN_ID_12, "Exit" );
       UG_ButtonSetBackColor( &menu1->menu_wnd, BTN_ID_12, C_TRANSPARENT/*TO_UG_COLOR( 0, 93, 93, 93)*/);     // #'s
       UG_ButtonSetForeColor( &menu1->menu_wnd, BTN_ID_12, C_BLACK);
       UG_ButtonSetAlternateForeColor( &menu1->menu_wnd, BTN_ID_12, C_BLUE);
       UG_ButtonSetStyle( &menu1->menu_wnd, BTN_ID_12, BTN_STYLE_NO_FILL | BTN_STYLE_2D);
       algn = ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT;
       UG_ButtonSetAlignment( &menu1->menu_wnd, BTN_ID_12, algn);

       UG_WindowShow( FOREGROUND_LAYER, &menu1->menu_wnd);
   }
#else


   //UG_WindowShow( FOREGROUND_LAYER, &window_3);
   //UG_Update();


   menu = UG_MenuCreate(  MENU_STYLE_MEDIUM_FONT, "Settings", 400, window_menu_callback);
   UG_MenuItemAdd( menu, "/images/apple.bmp", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "Self Test");
   UG_MenuItemAdd( menu, "/images/process.bmp", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "Setup");
   UG_MenuItemAdd( menu, "/images/dwnld.bmp ", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "Run Loader");
   UG_MenuItemAdd( menu, "/images/umbrella.bmp", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "Exit");
   // done in the 4 finger press --->   UG_MenuShow( menu);

   menu_test = UG_MenuCreate(  MENU_STYLE_MEDIUM_FONT, "Self Test", 400, menu_test_callback);
   UG_MenuItemAdd( menu_test, "/images/apple.bmp", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "SD-Ram Test");
   UG_MenuItemAdd( menu_test, "/images/print.bmp", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "Printer Test");
   UG_MenuItemAdd( menu_test, "/images/calc.bmp ", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "Keypad Test");
   UG_MenuItemAdd( menu_test, "/images/id.bmp", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "Card Reader Test");
   UG_MenuItemAdd( menu_test, "/images/ipod.bmp", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "Tag Reader Test");
   UG_MenuItemAdd( menu_test, "/images/save.bmp", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "SD Card Test");
   UG_MenuItemAdd( menu_test, "/images/push-pin.bmp", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "Touch Screen Test");
   UG_MenuItemAdd( menu_test, "/images/umbrella.bmp", ALIGN_CENTER_LEFT | ALIGN_IMAGE_NEXT_TO_TEXT, "Exit");



   /****************** EXAMPLE *******************/
   /* CREATE A TEST INPUT BOX BY USING A TEXTBOX */
   /* ALL input data is handled in the window's  */
   /* callback function window_8_callback...     */
   /* Note the use of SetFocus function          */

   UG_WindowCreate( &window_8, obj_buff_wnd_8, MAX_OBJECTS, window_8_callback);
   UG_WindowSetTitleText( &window_8, "Setup" );
   UG_WindowSetTitleTextFont( &window_8, &FONT_16X26/*FONT_GISHA_38*/);
   style = UG_WindowGetStyle( &window_8);
   style |= WND_STYLE_3D | WND_STYLE_SHOW_TITLE;
   UG_WindowSetStyle( &window_8, style);                // remove the frame around the window also
   UG_WindowSetBackColor( &window_8, C_WHITE);            // this is the color key
   UG_WindowResize( &window_8, 300, 150, 500 + 250, 240 + 140 + 50);

   int margin_up = 6;
   int textbox_height = 50;
   int w8 = UG_WindowGetInnerWidth( &window_8);
   int h8 = UG_WindowGetInnerHeight( &window_8);
   int button_width = 100, button_height = 45, number_buttons = 2;
   int left_boxes = 195;


   UG_TextboxCreate( &window_8, &textbox8_0, TXB_ID_3, 1, margin_up, left_boxes, textbox_height + margin_up);
   UG_TextboxSetFont( &window_8, TXB_ID_3, &FONT_16X26);
   UG_TextboxSetText( &window_8, TXB_ID_3, "Host IP:" );
   UG_TextboxSetAlignment( &window_8, TXB_ID_3, ALIGN_CENTER_RIGHT);
   UG_TextboxSetForeColor( &window_8, TXB_ID_3, C_DARK_GRAY /*TO_UG_COLOR( 0, 149, 199, 248)*/);
   UG_TextboxSetBackColor( &window_8, TXB_ID_3, C_TRANSPARENT);

   ////////////////////////////////////////////////////////////
   // 1st textbox as inputbox
   ////////////////////////////////////////////////////////////
   // allocate room for data
   UG_CreateInputBox( &window_8, TXB_ID_0, "192.168.001.001", &FONT_DIGITAL_7X38,
                       ALIGN_CENTER_LEFT, C_DARK_GRAY, C_LIGHT_BLUE,
                       205, margin_up, 415, textbox_height + margin_up);
   UG_InputBoxSetMaxChars( &window_8, TXB_ID_0, 15);

   UG_TextboxCreate( &window_8, &textbox8_1, TXB_ID_4, 1, textbox_height + 2 * margin_up, left_boxes, 2 * textbox_height + 2 * margin_up);
   UG_TextboxSetFont( &window_8, TXB_ID_4, &FONT_16X26);
   UG_TextboxSetText( &window_8, TXB_ID_4, "CAT Address:" );
   UG_TextboxSetAlignment( &window_8, TXB_ID_4, ALIGN_CENTER_RIGHT);
   UG_TextboxSetForeColor( &window_8, TXB_ID_4, C_DARK_GRAY /*TO_UG_COLOR( 0, 149, 199, 248)*/);
   UG_TextboxSetBackColor( &window_8, TXB_ID_4, C_TRANSPARENT);
#if 0
   // 2nd textbox as inputbox
   // allocate room for data
   char *inputbox2 = (char *)malloc( 64);
   strcpy( inputbox2, "67734565");
   UG_TextboxCreate( &window_8, &textbox8_2, TXB_ID_1, 230, textbox_height + margin_up + 10, 430, 2 * textbox_height + margin_up + 10);
   UG_TextboxSetFont( &window_8, TXB_ID_1, &FONT_DIGITAL_7X38);
   UG_TextboxSetText( &window_8, TXB_ID_1, inputbox2);
   UG_TextboxSetAlignment( &window_8, TXB_ID_1, ALIGN_CENTER_LEFT);
   UG_TextboxSetForeColor( &window_8, TXB_ID_1, C_GREEN /*TO_UG_COLOR( 0, 149, 199, 248)*/);
   UG_TextboxSetBackColor( &window_8, TXB_ID_1, C_RED);

   styletxb = UG_TextboxGetStyle( &window_8, TXB_ID_1);
   styletxb |= WND_STYLE_FRAME | TXB_STYLE_CURSOR_ON | TXB_STYLE_INPUT_BOX;
   UG_TextboxSetStyle( &window_8, TXB_ID_1, styletxb);
#else
   char ttt[16];
   UG_CreateInputBox( &window_8, TXB_ID_1, ttt, &FONT_DIGITAL_7X38,
                       ALIGN_CENTER_LEFT, C_DARK_GRAY, C_LIGHT_BLUE,
                       left_boxes + 10, textbox_height + margin_up + margin_up, 405, 2 * textbox_height + margin_up + margin_up);
   UG_InputBoxSetMaxChars( &window_8, TXB_ID_1, 3);
#endif

   UG_CreateInputBox( &window_8, TXB_ID_2, "ANDROID-1234", &FONT_DIGITAL_7X38,
                       ALIGN_CENTER_LEFT, C_DARK_GRAY, C_LIGHT_BLUE,
                       left_boxes + 10, 2 * textbox_height + 3 * margin_up, 405, 3 * textbox_height + 3 * margin_up);
   UG_TextboxCreate( &window_8, &textbox8_2, TXB_ID_5, 1, 2 * textbox_height + 3 * margin_up, left_boxes, 3 * textbox_height + 3 * margin_up);
   UG_TextboxSetFont( &window_8, TXB_ID_5, &FONT_16X26);
   UG_TextboxSetText( &window_8, TXB_ID_5, "Host Name:" );
   UG_TextboxSetAlignment( &window_8, TXB_ID_5, ALIGN_CENTER_RIGHT);
   UG_TextboxSetForeColor( &window_8, TXB_ID_5, C_DARK_GRAY /*TO_UG_COLOR( 0, 149, 199, 248)*/);
   UG_TextboxSetBackColor( &window_8, TXB_ID_5, C_TRANSPARENT);

   int b_xs = w8 / (2 * number_buttons) - button_width / 2;
   int b_xe = b_xs + button_width;
   int b_ys = h8 - button_height - margin_up;
   int b_ye = b_ys + button_height;
   UG_ButtonCreate( &window_8, &button8_1, BTN_ID_0, b_xs, b_ys, b_xe, b_ye);

   /* Configure Button 1 */
   UG_ButtonSetFont( &window_8, BTN_ID_0, &FONT_TAHOMA_26B);
   UG_ButtonSetForeColor( &window_8, BTN_ID_0, C_DARK_GRAY);
   UG_ButtonSetBackColor( &window_8, BTN_ID_0, C_LIGHT_GRAY );
   UG_ButtonSetText( &window_8, BTN_ID_0, "OK" );
   UG_ButtonShow( &window_8, BTN_ID_0);
#if 1
   int b2_xs = w8 / (2 * number_buttons) + w8 / 2 - button_width / 2;
   int b2_xe = b2_xs + button_width;
   int b2_ys = h8 - button_height - margin_up;
   int b2_ye = b2_ys + button_height;
   UG_ButtonCreate( &window_8, &button8_2, BTN_ID_1, b2_xs, b_ys, b2_xe, b2_ye);

   /* Configure Button 1 */
   UG_ButtonSetFont( &window_8, BTN_ID_1, &FONT_TAHOMA_26B);
   UG_ButtonSetForeColor( &window_8, BTN_ID_1, C_DARK_GRAY);
   UG_ButtonSetBackColor( &window_8, BTN_ID_1, C_LIGHT_GRAY );
   UG_ButtonSetText( &window_8, BTN_ID_1, "Cancel" );
   UG_ButtonShow( &window_8, BTN_ID_1);
#endif
   window_8_status = 0;
   w8_inx = 0;
#endif
   /* -------------------------------------------------------------------------------- */
   /* Start demo application                                                           */
   /* -------------------------------------------------------------------------------- */
   //UG_WaitForUpdate();

   //UG_SelectGUI( BACKGROUND_LAYER, &bgui);


   debugplc = 8;

   UG_Update();


   debugplc = 9;


#if 1
   LTDC_Cmd( ENABLE/*DISABLE*/);
#endif



#ifdef DEBUG_MENU
   while(1) ;
#endif

   /**************************************
    *-------- LTDC IS NOW ENABLED --------
    **************************************/
   if( sync_display != NULL)
       mutex_unlock( sync_display);
   return 0;

#if 0
   /* Initialize FSM */
   next_state = STATE_MAIN_MENU;
   state = !STATE_MAIN_MENU;


   while(1)
   {
      static int frm_cnt;

      /* Do we change the state? */
      if ( next_state != state )
      {
         /* Initialize the next state */
         switch ( next_state )
         {
            case STATE_MAIN_MENU:
            {
               /* Nothing to do */
               break;
            }
            case STATE_BENCHMARK_RUN:
            {
               /* Clear layer 2 */
               //ltdc_draw_layer( LAYER_2 );
               UG_FillScreen( C_BLACK );

               /* Fade to layer 2 */
               //ltdc_fade_to_layer( LAYER_2 );

               /* Reset the frame counter */
               frm_cnt = 0;

               /* Run benchmark for 5 seconds */
               timer = 500;
               break;
            }
            case STATE_BENCHMARK_RESULT:
            {
               /* Nothing to do */
               break;
            }
         }
         state = next_state;
      }

      /* FSM */
      switch ( state )
      {
         /* Run the benchmark */
         case STATE_BENCHMARK_RUN:
         {
            xs = randx() % 1024;
            xe = randx() % 1024;
            ys = randx() % 600;
            ye = randx() % 600;
            c = randx() % 0xFFFFFF;
            UG_FillFrame( xs, ys, xe, ye, c );
            frm_cnt++;

            if ( !timer ) next_state = STATE_BENCHMARK_RESULT;
            continue;
         }
         /* Show benchmark result */
         case STATE_BENCHMARK_RESULT:
         {
            sprintf( result_str, "Result:\n%u frm/sec", frm_cnt/5 );
            UG_TextboxSetText( &window_3, TXB_ID_0, result_str );

            /* Fade to layer 1 */
            //ltdc_draw_layer( LAYER_1 );
            //ltdc_fade_to_layer( LAYER_1 );

            /* Show benchmark result */
            UG_WindowShow( &window_3 );

            next_state = STATE_MAIN_MENU;
            break;
         }
         case STATE_MAIN_MENU:
         {
            /* Let µGUI do the job! */
            break;
         }
      }
      vtimer_usleep( 100000);
      //hwtimer_wait( 100000);
   }
#endif
}

/**
 * @NOTE: CALL THIS FUNCTION ONLY AFTER UG_SelectGUI() was called
 */
void display_regsiterDrivers( UG_GUI *g)
{
    UG_DriverRegister( g, DRIVER_DRAW_LINE, (void*)/*_HW_DrawLine*/_HW_DMA0_Fill );
    UG_DriverRegister( g, DRIVER_FILL_FRAME, (void*)/*_HW_FillFrame*/_HW_DMA0_Fill );
    UG_DriverRegister( g, DRIVER_COPY_RECT, (void*)_HW_CopyRect );
    UG_DriverRegister( g, DRIVER_PSET, (void*)_HW_pset);
    UG_DriverRegister( g, DRIVER_LAYER_POS, (void *)_HW_layerPosition);
}


void display_setPumpNumber( int pump)
{
    sprintf( TextPump, "Pump #%d", pump);
    UG_TextboxSetText( &window_2, TXB_ID_2, TextPump);
    myAddress = pump;
}

/**
 * @brief sets the Host IP address to window_8, TXB_ID_0 (in the setup)
 * @param ip_str
 */
void display_setHostIP( const char *ip_str)
{
    UG_TextboxSetText( &window_8, TXB_ID_0, ip_str);
}

/**
 * @brief   fills a gui with color c
 * @param layer
 * @param c
 */
void display_fill( int layer, UG_COLOR c)
{
    UG_FillScreen( layer, c);
}


/* -------------------------------------------- */
/*             C L I P      G U I               */
/*             F U N C T I O N S                */
/* -------------------------------------------- */
void display_clipGui( void)
{
    // this selection sets up the background layer for the clip gui
    UG_GUI *prevgui = UG_SelectGUI( BACKGROUND_LAYER, &guiClip);
    UG_FillScreen( BACKGROUND_LAYER, C_BLACK);
    //UG_SelectGUI( BACKGROUND_LAYER, prevgui);
}

/**
 * @brief setup a data pane on top of the clip gui, using guiSmall
 *
 * @NOTE  after the clip is shown all the display memory and module are initialized
 *        so technically changing the addresses like that is acceptable
 */
void display_dataGui( void)
{
    guiSmall.x_dim = 800;
    guiSmall.y_dim = 51;
    guiSmall.mainVideoBuffer = guiSmall.currentVideoBuffer = 0xD07E7960;
    guiSmall.screenStartx = 0;
    guiSmall.screenStarty = 800;

    //UG_ResetGUI( FOREGROUND_LAYER, &guiSmall, 0, 429, guiSmall.x_dim, guiSmall.y_dim, LTDC_Pixelformat_RGB565);
    UG_SelectGUI( FOREGROUND_LAYER, &guiSmall);

    //UG_ClearScreens( FOREGROUND_LAYER, C_BLACK);

    guiSmall.active_window = NULL;
    guiSmall.next_window = NULL;
    guiSmall.last_window = NULL;
    LTDC_LayerAlpha( guiSmall.hwlayer, 75); // was 55
    w3_move_in = w3_move_out = 0;
}


/**
 * @brief   removes focus from ID_TXB_1 so that the cursor no longer shows
 *
 */
void display_removeFocus( void)
{
    UG_SetFocus( &window_2, TXB_ID_0);
}



/**
 * @brief Displays new text on screen
 * @param txt
 * @param cnt
 */
void display_showText( char *txt, int cnt, int Id)
{
    if( cnt >= sizeof( Text1))
        return;

   // UG_invalidateBackground( 0, 0);
    UG_invalidateObject( BACKGROUND_LAYER,  &window_2, OBJ_TYPE_TEXTBOX, Id /*TXB_ID_0*/);

    if( cnt == 0) { //cnt = strlen( txt);
        cnt = 1;
        txt[0] = ' ';
        txt[1] = NULL;
    }

    if( Id == TXB_ID_0)
    {
        memcpy( Text1, txt, cnt);
        Text1[cnt] = (char)NULL;
        UG_TextboxSetText( &window_2, Id /*TXB_ID_0*/, Text1);
    }
    else
    {
        memcpy( Text2, txt, cnt);
        Text2[cnt] = (char)NULL;
        UG_TextboxSetText( &window_2, Id /*TXB_ID_0*/, Text2);
        UG_SetFocus( &window_2, Id);                    // NS 23-07-17   set the focus on the second line
    }
}

void display_setRightText( int Id)
{
    UG_TextboxSetAlignment( &window_2, Id /*TXB_ID_0*/, ALIGN_CENTER_RIGHT);
}

void display_setLeftText( int Id)
{
    UG_TextboxSetAlignment( &window_2, Id /*TXB_ID_0*/, ALIGN_CENTER_LEFT);
}

void display_window3( void)
{
    //guiSmall.x_dim = 800;
    //guiSmall.y_dim = 50;

    //UG_SelectGUI( FOREGROUND_LAYER, &guiSmall);
    UG_WindowShow( FOREGROUND_LAYER, &window_3 );
    //UG_Update();
}
void display_window4( void)
{
    UG_SelectGUI( FOREGROUND_LAYER, &guiSmall);
    UG_FillScreen( FOREGROUND_LAYER, C_BLACK);
    //UG_WindowShow( &window_4 );
    //UG_Update();
}

// 2do: maybe in future change to show ProgressBar maybe in a thread
UG_GUI *prevmbGui;
static int inmessagebox = 0;
void display_messageBox( char *text)
{
    if( inmessagebox)
        return;
    inmessagebox = true;
    display_setupMessageBox( "Info", text, 3);
    inmessagebox = false;

#if 0
    guiSmall.screenStartx = 0;
    guiSmall.screenStarty = 0;
    prevmbGui = UG_SelectGUI( FOREGROUND_LAYER, &guiSmall);
    uint wd = UG_WindowGetOuterWidth( &window_1);
    UG_WindowSetXStart( &window_1, (guiSmall.x_dim - wd) / 2);
    UG_WindowSetXEnd( &window_1, (guiSmall.x_dim - wd) / 2 + wd);
    uint hg = UG_WindowGetOuterHeight( &window_1);
    UG_WindowSetYStart( &window_1, (guiSmall.y_dim - hg) / 2);
    UG_WindowSetYEnd( &window_1, (guiSmall.y_dim - hg) / 2 + hg);
    UG_TextboxSetText( &window_1, TXB_ID_0, text);
    UG_WindowShow( FOREGROUND_LAYER, &window_1);

    UG_ProgressBarSetValue( &window_1, TXB_ID_1, 50);

        UG_Update();            // let's try this again...
#endif
}
void display_removeMessageBox( void)
{
    UG_WindowHide( FOREGROUND_LAYER, &window_1);
    //
    // new release gui
    UG_releaseGUI( FOREGROUND_LAYER, &guiSmall);
    /*
    if( prevmbGui != &fgui)
        prevmbGui = &fgui;
    */
#if 0
    if( prevmbGui != NULL /*&& prevmbGui != &guiSmall*/)
        UG_SelectGUI( FOREGROUND_LAYER, prevmbGui);
    else
        UG_SelectGUI( FOREGROUND_LAYER, &fgui);
#endif
    UG_Update();                // trying this back
}

void display_messageBoxPos( char *text, int x, int y)
{
    //UG_SelectGUI( &gui);
    UG_TextboxSetText( &window_1, TXB_ID_0, text);
    UG_WindowShow( BACKGROUND_LAYER, &window_2);
    // NS 18-9 UG_Update();

    UG_WindowResize( &window_1, x, y, x + WINDOW_1_WIDTH, y + WINDOW_1_HEIGHT);
    UG_WindowShow( BACKGROUND_LAYER, &window_1);
    // NS 18-9 UG_Update();
}


/**
 * @brief Setup LTDC to show keypad and messages window
 */
void display_window2( void)
{
#if 1
    // NOT NECESSARY, THE FIRST GUI IS ALWAYS SELECTED.... UG_SelectGUI( BACKGROUND_LAYER, &bgui);
#else
    UG_ResetGUI( BACKGROUND_LAYER, &bgui, 0, 0, XSIZE_PHYS, YSIZE_PHYS, LTDC_Pixelformat_RGB565);
#endif
    //UG_ResetGUI( &gui, 0, 0, XSIZE_PHYS, YSIZE_PHYS, LTDC_Pixelformat_L8);

#if 0
    //! and shows the background layer only
    LTDC_LayerCmd( LTDC_Layer1, DISABLE);           // background - this is where the clip is displayed
    LTDC_LayerCmd( LTDC_Layer2, ENABLE);            // other data on screen
    LTDC_LayerSize( LTDC_Layer2, XSIZE_PHYS, YSIZE_PHYS);
    uint32_t add = SDRAM_BANK_ADDR + LAYER_2_OFFSET;
    LTDC_LayerAddress( LTDC_Layer2, add);

    // disable color keying, too
    LTDC_ColorKeying_InitTypeDef LTDC_colorkeying_InitStruct;
    LTDC_colorkeying_InitStruct.LTDC_ColorKeyBlue = 0;
    LTDC_colorkeying_InitStruct.LTDC_ColorKeyGreen = 0;
    LTDC_colorkeying_InitStruct.LTDC_ColorKeyRed = 0;
    LTDC_ColorKeyingConfig( LTDC_Layer2, &LTDC_colorkeying_InitStruct, DISABLE);
    LTDC_LayerAlpha(LTDC_Layer2, 255);

    LTDC_ReloadConfig( LTDC_IMReload);
#endif

    UG_invalidateBackground( BACKGROUND_LAYER, 0, YSIZE_PHYS);
    UG_WindowShow( BACKGROUND_LAYER, &window_2 );
    /*
    UG_Update();
    */
}


void display_removeWindow2( void)
{
    //UG_GUI *prev = UG_SelectGUI( BACKGROUND_LAYER, &gui);             // test 19-10
    UG_WindowHide( BACKGROUND_LAYER, &window_2 );
    //UG_releaseGUI( BACKGROUND_LAYER, &bgui);                            // test 19-10
}

void display_turnForegroundOff( void)
{
    UG_turnGuiOff( FOREGROUND_LAYER);
}
void display_turnForegroundOn( void)
{
    UG_turnGuiOn( FOREGROUND_LAYER);
}
/**
 * W I N D O W  _  3    F U N C T I O N S
 *
 * used by clip data gui
 */
void display_updateTextW3( char *txt, int cnt)
{
    if( cnt == 0) cnt = strlen( txt);
    memcpy( Text1, txt, cnt);
    Text1[cnt] = NULL;
    UG_TextboxSetText( &window_3, TXB_ID_0, Text1);
}

void display_setProgressValue( uint8_t val)
{
    UG_ProgressBarSetValue( &window_3, TXB_ID_0, val);
}

/**
 * @brief this function moves window_3 in and out of the screen
 *        when it is singalled by window3 call back function
 *
 */
void display_manageW3Location( void)
{
static uint32_t lasttime = 0, location = 480;

    if( w3_move_in)
    {
        uint32_t xnow = xtimer_now();
        if( xnow - lasttime > 25000)
        {
            lasttime  = xnow;
            _HW_layerPosition( &guiSmall, 0, location);
            location -= 2;
            if( location <= (uint32_t)(YSIZE_PHYS - UG_WindowGetOuterHeight( &window_3)))
            {
                w3_move_in = 0;         // done moving...
                w3_move_out = 1;        // set the stage for move out...
            }
        }
    } // endif move in
    else if( w3_move_out == 2)
    {
        uint32_t xnow = xtimer_now();
        if( xnow - lasttime > 25000)
        {
            lasttime  = xnow;
            location += 2;
            _HW_layerPosition( &guiSmall, 0, location);
            if( location > YSIZE_PHYS + 1)
            {
                w3_move_in = 0;         // done moving...
                w3_move_out = 0;        // set the stage for move out...
                lasttime = 0;
            }
        }
    }
    else if( w3_move_out == 1)
    {
        uint32_t xnow = xtimer_now();
        if( xnow - lasttime > 5000000)
        {
            w3_move_out = 2;
            lasttime = xnow;
        }
    }
} // endfunc


/**
 * @brief sets new image for window_4 image object to be displayed on
 *        foreground layer.
 *
 * @param width
 * @param height
 * @param offset - SDRAM address of bitmap to be displayed
 */
static int xcounter = 0, grn = 0;
void display_setImageToDisplay( int width, int height, void *offset)
{
    //UG_GUI *prev_gui = UG_SelectGUI( &guiSmall);
    //UG_WindowShow( &window_4 );

    BITMAP_FILEHEADER *fhead = (BITMAP_FILEHEADER *)offset;
    uint8_t *pStart = ( uint8_t *)( offset + fhead->BitsOffset);
    BITMAPINFOHEADER *bmpheader = (BITMAPINFOHEADER *)( offset + sizeof( BITMAP_FILEHEADER));

    if( bmpheader->biBitCount == 24)
    {
        bmp.bpp = 24;
        bmp.height = height;
        bmp.width  = width;
        bmp.p = pStart;
    }
    else if( bmpheader->biBitCount == 8)
    {
        bmp.bpp = 8;
        bmp.height = height;
        bmp.width  = width;
        bmp.p = offset;
    }
#if 0
    UG_SelectGUI( FOREGROUND_LAYER, &guiSmall);
    UG_ImageSetBMP( &window_4, IMG_ID_100, &bmp);
    UG_ImageShow( &window_4, IMG_ID_100);
    UG_WindowShow( FOREGROUND_LAYER, &window_4);
#else
    UG_ImageSetBMP( &window_4, IMG_ID_100, &bmp);
    UG_ImageShow( &window_4, IMG_ID_100);
    UG_WindowShowGui( &fgui, &window_4);
#endif
    // NS 03-04-17 UG_Update();

/*
    _HW_DMA0_Fill( 0, 0, 800, 480, TO_UG_COLOR( 0, 7, 194, 91), &gui);
    _HW_DMA0_Fill( xcounter, 0, xcounter+100, 480, TO_UG_COLOR( 0, 7, 194+grn, 91), &gui);
    xcounter++;
    grn++;
    if( grn > 255-194) grn = 0;
*/
    if( xcounter > 800-100) xcounter = 0;
    //UG_SelectGUI( prev_gui);
}

#if 0
/**
 * @brief sets new image L8 for window_4 image object to be displayed
 * @param bitmap - pointer to a L8 microsoft bitmap (-flip)?
 */
void display_setBkImageToDisplay( char *bitmapName)
{
    display_readBmpToMemory( bitmapName, backgroundImageAddrs);

    UG_GUI *prev_gui = UG_SelectGUI( &gui);
    UG_WindowShow( &window_4 );

    bmp.bpp = 8;
    bmp.height = 0;
    bmp.width  = 0;
    bmp.p = backgroundImageAddrs;

    UG_ImageSetBMP( &window_4, IMG_ID_100, &bmp);
    UG_ImageShow( &window_4, IMG_ID_100);
    UG_Update();
    UG_SelectGUI( prev_gui);
}
#endif

static uint32_t *screens = NULL;

/**
 * @brief   puts ROM frame window on GUI
 */
void display_showImageWindow()
{
    //!!!UG_FillScreen( C_BLUE);
    //guiSmall.x_dim = XSIZE_PHYS;
    //guiSmall.y_dim = YSIZE_PHYS;
    //guiSmall.screenStartx = guiSmall.screenStarty = 0;
    UG_ResetGUI( FOREGROUND_LAYER, &guiSmall, 0, 0, XSIZE_PHYS, YSIZE_PHYS, guiSmall.LTDC_Pixelformat);

    // 24-10   no need UG_SelectGUI( &guiSmall);
    //UG_ClearScreens( C_BLACK);
    UG_WindowShow( FOREGROUND_LAYER, &window_4 );
}

/*
 * @brief   removes ROM frame window from GUI
 */
void display_removeImageWindow()
{
    UG_WindowHideGui( &fgui, &window_4 );
    //display_clear();
}




#if 0
void display_storeMultipleFrames( )
{
    if( screens != NULL) display_free( screens);
    screens = display_malloc( guiSmall.x_dim * guiSmall.y_dim * 2 * framesNum);

}
#endif


bool display_isImageWindowShown( void)
{
    if( UG_GetCurrentWindow(FOREGROUND_LAYER) == &window_4)
        return true;
    else return false;
}


static uint8_t colorLookupTable[256*4];
//static UG_GUI *pgui;

void display_endShortClip( void)
{
    // restart the gui with RGB565 parameters
    // and select the underlying gui 19-10
    //UG_SelectGUI( UG_ResetGUI( &gui, 0, 0, XSIZE_PHYS, YSIZE_PHYS, LTDC_Pixelformat_RGB565));
    // 06-04-17 NS .... UG_ResetGUI( BACKGROUND_LAYER, &bgui, 0, 0, XSIZE_PHYS, YSIZE_PHYS, LTDC_Pixelformat_RGB565); // NS
#if 0
    if( pgui != NULL)
        UG_SelectGUI( BACKGROUND_LAYER, pgui);
    else
        UG_ResetGUI( BACKGROUND_LAYER, &bgui, 0, 0, XSIZE_PHYS, YSIZE_PHYS, LTDC_Pixelformat_RGB565); // NS
    pgui = NULL;
#endif

    //! go back to whatever was displayed before
    UG_releaseGUI( BACKGROUND_LAYER, &bGuiShort);
}


/**
 * @brief   This function changes LTDC layer parameters if required
 *          and returns the number of frames of short clip
 * @return  int number of frames in clip
 */
int display_prepareForShortClip( void)
{
    AVISTREAMHEADER *avihead = ( AVISTREAMHEADER *)riffParser_GetStreamHeader( 0);
#if 1
    // reset dimensions of screen
    bGuiShort.x_dim = XSIZE_PHYS;
    bGuiShort.y_dim = YSIZE_PHYS;

    uint32_t startx = ( bGuiShort/*bgui*/.x_dim - ( avihead->rcFrame.right  - avihead->rcFrame.left)) / 2;       // was fgui ???? 09-04
    uint32_t starty = ( bGuiShort/*bgui*/.y_dim - ( avihead->rcFrame.bottom - avihead->rcFrame.top )) / 2;       // was fgui ???? 09-04
/*
    pgui = UG_ResetGUI( BACKGROUND_LAYER, &bGuiShort, startx, starty, avihead->rcFrame.right,
                          avihead->rcFrame.bottom, LTDC_Pixelformat_L8);
*/

    //!!! THIS IS HOW IT SHOULD BE DONE !!!
    bGuiShort.screenStartx = startx;
    bGuiShort.screenStarty = starty;
    bGuiShort.x_dim = avihead->rcFrame.right;
    bGuiShort.y_dim = avihead->rcFrame.bottom;
    bGuiShort.LTDC_Pixelformat = LTDC_Pixelformat_L8;
    //! -------------------------
    //! select the short clip gui
    UG_SelectGUI( BACKGROUND_LAYER, &bGuiShort);

#else
    gui.screenStartx = 0;
    gui.screenStarty = 0;
    gui.x_dim = avihead->rcFrame.right;
    gui.y_dim = avihead->rcFrame.bottom;
    gui.LTDC_Pixelformat = LTDC_Pixelformat_L8;
    UG_ResetGUI( &gui, gui.screenStartx, gui.screenStarty,
                  gui.x_dim, gui.y_dim, gui.LTDC_Pixelformat);
#endif
    //!get number of frames from avi header
    int numofframes = avihead->dwLength / avihead->dwScale;
    return numofframes;
}

/**
 * @brief   displays an 8 bit bitmap image coming from an .avi file (BMP) stored
 *          in SDRAM
 *
 * @param currentFrame
 * @param pInx
 * @param screens
 * @return 0 on ERROR, 1 if OK
 *
 */
typedef struct
{
   uint8_t colorA;
   uint8_t colorR;
   uint8_t colorG;
   uint8_t colorB;
} bmpColors;

int display_shortClip( uint showFrame, uint32_t *pInx, uint32_t screens, bool newClip)
{
    if( pInx == NULL)
        return 0;
#if 0
    AVISTREAMHEADER *avihead = ( AVISTREAMHEADER *)riffParser_GetStreamHeader( 0);
    /*
    GUIDRV_MovieInit( avihead->rcFrame.right - avihead->rcFrame.left,
                      avihead->rcFrame.bottom - avihead->rcFrame.top,
                      LTDC_Pixelformat_L8);
     */
    //!get number of frames from avi header
    int numofframes = avihead->dwLength / avihead->dwScale;

    // setup background layer and leave foreground layer for gui stuff (buttons etc...)
    uint __curFrame = 0;
    //uint32_t start_addrs  = pInx[__curFrame] % _MAX_SS /* usually just 512 bytes for SDIO */;
    uint32_t offset = pInx[showFrame] /* - pInx[__curFrame]*/ + screens /*+ start_addrs*/;
    BITMAP_FILEHEADER *fhead = (BITMAP_FILEHEADER *)offset;
    if( (uint32_t)fhead < SDRAM_BASE || fhead->Signature != SIGNATURE_OF_BITMAP /*0x4D42*/) //< "BM" is the signature. If can't get that --- go out!
    {
        while( 1); //error in bitmap
    }
    void *pixelsStart = ( void *)( offset + fhead->BitsOffset);
    BITMAPINFOHEADER *bmpheader = (BITMAPINFOHEADER *)( offset + sizeof( BITMAP_FILEHEADER));

    /*************************
     *   8 bits per pixel
     *************************/
    if( bmpheader->biBitCount == 8)   // if BITMAP is not full RGB888 this bitmap is using a CLUT
    {
        bmpColors *bmpclut;

        display_prepareForShortClip();

        bmpclut = ( bmpColors *)( (uint32_t)bmpheader + (uint32_t)bmpheader->biSize);
        GUIDRV_setPendingClut( ( uint8_t *)bmpclut, bmpheader->biClrUsed); //< the clut will be written to LTDC during blanking time
        LTDC_CLUTCmd( LTDC_Layer1, ENABLE);

        uint32_t add__ = LTDC_Layer1->CFBAR;
#if 0
        if( add__ == 0xd0000008)
        {
            pixelsStart = 0xd0050000;
            memcpy( pixelsStart, 0xd0000008, 800000);

            //pixelsStart = 0xd0000009;
            LTDC_LayerAddress( LTDC_Layer1, (uint32_t)pixelsStart);
            gui.currentVideoBuffer = (uint32_t)pixelsStart;
        }
#else
        LTDC_LayerAddress( LTDC_Layer1, (uint32_t)pixelsStart);
        gui.currentVideoBuffer = (uint32_t)pixelsStart;
#endif

        LTDC_ReloadConfig( LTDC_VBReload);

/*
        LTDC_LayerCmd( LTDC_Layer1, DISABLE);
        LTDC_LayerCmd( LTDC_Layer1, ENABLE);
*/
    }
#else

    // my idea was to place the prepareforshortclip as close as possible to
    // actually showing the first frame, so there won't be any flickering
    if( newClip)
        display_prepareForShortClip();


    uint32_t offset = pInx[showFrame] + screens;
    BITMAP_FILEHEADER *fhead = (BITMAP_FILEHEADER *)offset;
    if( (uint32_t)fhead < SDRAM_BASE || fhead->Signature != SIGNATURE_OF_BITMAP /*0x4D42*/) //< "BM" is the signature. If can't get that --- go out!
        return( 0); //while( 1); //error in bitmap

    void *pixelsStart = ( void *)( offset + fhead->BitsOffset);
    BITMAPINFOHEADER *bmpheader = (BITMAPINFOHEADER *)( offset + sizeof( BITMAP_FILEHEADER));

    /****************************
     *   8 bits per pixel only!
     ****************************/
    if( bmpheader->biBitCount != 8)   // if BITMAP is not full RGB888 this bitmap is using a CLUT
        return 0;

    bmpColors *bmpclut;
    bmpclut = ( bmpColors *)( (uint32_t)bmpheader + (uint32_t)bmpheader->biSize);
    //
    //
    memcpy( colorLookupTable, bmpclut, 256*4);
    //
    //
    //< the clut will be written to LTDC during blanking time
    // I want the new image to show up right after the corresponding clut is
    // programmed into the CLUTWR, so this is what I'm doing now..
    GUIDRV_setPendingClut( ( uint8_t *)/*bmpclut*/colorLookupTable, bmpheader->biClrUsed, pixelsStart);

    // NS 18-09 no need?? LTDC_CLUTCmd( LTDC_Layer1, ENABLE);
#if 0
    LTDC_LayerAddress( LTDC_Layer1, (uint32_t)pixelsStart);
    LTDC_ReloadConfig( LTDC_VBReload);
#endif // testing 26=10

    /*bgui*/bGuiShort.currentVideoBuffer /*= gui.mainVideoBuffer*/ = (uint32_t)pixelsStart;      // was fgui ???? 09-04
#endif

    return 1;
} // endfunc


/**
 * this function clears the display
 */
void display_clear( void)
{
//    UG_GUI *prev = UG_SelectGUI( FOREGROUND_LAYER, &guiSmall);
    //!! clear the entire display
    UG_ClearScreens( FOREGROUND_LAYER, C_BLACK);
    // I can't do that now..... UG_ClearScreens( BACKGROUND_LAYER, C_BLACK);
//    UG_SelectGUI( prev);
    //UG_ClearScreens( C_BLACK); // NS try
//    UG_FillScreen( C_BLACK);
}

static float bounce_easeIn ( float t, float b, float c, float d);
static float bounce_easeOut( float t, float b, float c, float d);
static float bounce_easeInOut( float t, float b ,float c, float d);
static float bounce_In( float time, float begin, float end, float duration);

/**
 *
 * @param buttons
 *
 * @returns the ID of the button pressed
 */
int display_windowWithButtons( const char *title, int buttons, uint32_t *names)
{
UG_GUI *guiAS, *prevGui;

    guiAS = (UG_GUI *)malloc( sizeof( UG_GUI));
    if( guiAS == NULL)
        return 0;             // no memory left

    UG_Init( guiAS, NULL, 501, 320, FOREGROUND_LAYER);      // was 500 debug
    // NS 31-07-17 ???  guiAS->currentVideoBuffer = guiAS->mainVideoBuffer = guiAS->backupVideoBuffer;
    display_regsiterDrivers( guiAS);
    guiAS->colorKey = C_BLACK;

/*
    guiSmall.x_dim = 500;
    guiSmall.y_dim = 320;           // was 290
    UG_ResetGUI( FOREGROUND_LAYER, &guiSmall, -guiSmall.x_dim, 0, guiSmall.x_dim, guiSmall.y_dim, LTDC_Pixelformat_RGB565);
*/
    window5_pressed = 0;

    /* -------------------------------------------------------------------------------- */
    /* Create Window 2 (µGUI Info)                                                      */
    /* -------------------------------------------------------------------------------- */
    UG_WindowCreate( &window_5, obj_buff_wnd_5, MAX_OBJECTS, window_5_callback );
    uint8_t style1 = UG_WindowGetStyle( &window_5);
    //UG_WindowSetStyle( &window_5, style1 & (~WND_STYLE_SHOW_TITLE));     // dont show title for the buttons window
    UG_WindowSetTitleText( &window_5, title );
    UG_WindowSetTitleTextFont( &window_5, &FONT_12X20 );
    UG_WindowSetBackColor( &window_5, TO_UG_COLOR( 0, 255, 56, 69));

    //------------------------***** build digit buttons *****---------------------------
    UG_RECT buttonRect = { 0, 0, 500, 320 };
    UG_RECT digitsRect = { buttonRect.xs, buttonRect.ys,
                           buttonRect.xe, buttonRect.ye };

    //UG_WindowResize( &window_2, buttonRect.xs, buttonRect.ys, buttonRect.xe + 5, buttonRect.ye + 5 /*30*/);
    UG_WindowResize( &window_5, 0, 0, buttonRect.xe, buttonRect.ye);
    UG_WindowGetArea( &window_5, &buttonRect);

    uint32_t xbuttons = 1, ybuttons = buttons;
    uint32_t YgapBetweenButtons = 4;
    uint32_t XgapBetweenButtons = 5;

    // leave room for the progressbar at the bottom
    buttonRect.ye -= 25;

#define MAX_APP_BUTTONS     5

    uint32_t singleButtonHeight = ( UG_getRectHeight( &buttonRect) - YgapBetweenButtons) / MAX_APP_BUTTONS;
    uint32_t digitButtonWidth   = UG_getRectWidth( &buttonRect) - 75;

    int binx = 0;
    uint _x, _y;

    _y = /*buttonRect.ys +*/ UG_getRectHeight( &buttonRect) / 2 -
            singleButtonHeight * ybuttons / 2;
    _x = /*buttonRect.xs +*/ UG_getRectWidth( &buttonRect) / 2 - digitButtonWidth / 2;

    for( uint32_t by = 0; by < ybuttons; by++)
    {
        int buttoninx = by;
        UG_ButtonCreate( &window_5, &buttonKeypad[binx], BTN_ID_1 + buttoninx,
                         _x, _y,  _x + digitButtonWidth, _y + singleButtonHeight);

        _y += singleButtonHeight + YgapBetweenButtons;

        UG_ButtonSetFont( &window_5, BTN_ID_1 + buttoninx, &FONT_GISHA_38);
        UG_ButtonSetText( &window_5, BTN_ID_1 + buttoninx, (char *)names[binx]);
        UG_ButtonSetBackColor( &window_5, BTN_ID_1 + buttoninx, TO_UG_COLOR( 0, 93, 93, 93));     // #'s
        UG_ButtonSetForeColor( &window_5, BTN_ID_1 + buttoninx, C_WHITE);
        UG_ButtonSetAlternateForeColor( &window_5, BTN_ID_1 + buttoninx, C_SILVER);

        binx++;
    } // endfor by

    // add a progress bar at the bottom of the window to show how much time is left
    UG_ProgressBarCreate( &window_5, &progress5_1, TXB_ID_0,
            buttonRect.xs + 4, buttonRect.ye - 27, buttonRect.xe - 10, buttonRect.ye - 5);
    UG_ProgressBarSetForeColor( &window_5, TXB_ID_0, C_GREEN, C_WHITE);

    //()()()()()()()()()()()()()()()()()()()()()()()()
    guiAS->screenStartx = -guiAS->x_dim;
    guiAS->screenStarty = 0;
    guiAS->LTDC_Pixelformat = LTDC_Pixelformat_RGB565;
    UG_SelectGUI( FOREGROUND_LAYER, guiAS);

    //prevGui = UG_ResetGUI( FOREGROUND_LAYER, guiAS, -guiAS->x_dim, 0, guiAS->x_dim, guiAS->y_dim, LTDC_Pixelformat_RGB565);


    UG_FillScreen( FOREGROUND_LAYER, C_BLACK);
    UG_WindowShow( FOREGROUND_LAYER, &window_5);
    UG_Update();

#ifdef MESSAGEBOX_SLIDE
    static float time, beginning, change, duration;

    // Penner Easing Functions Test
    time        = 0;
    beginning   = -500.0;
    change      = (XSIZE_PHYS - guiAS->x_dim) / 2.0;
    duration    = 800.0;

    while( time < duration)
    {
        float a = bounce_In( time, beginning, change, duration);
        _HW_layerPosition( guiAS, (int)(a), (YSIZE_PHYS - guiAS->y_dim) / 2);
        xtimer_usleep( 1000);
        time++;
    }
#else
    _HW_layerPosition( guiAS, (int)((XSIZE_PHYS - guiAS->x_dim) / 2.0), (YSIZE_PHYS - guiAS->y_dim) / 2);
#endif

#if 0
    layerPosition( &guiSmall, (int)100, -100);
    layerPosition( &guiSmall, (int)200, 400);
#endif

    int val = 20; // wait 20 seconds for a click
    int val1 = 0, but_press;
    while( 1)
    {
        if( (but_press=getChar( 0)) != 0)
        {
            for( int i = 0; i < ybuttons; i++)
            {
                if( i != but_press)
                    UG_ButtonHide( &window_5, i + BTN_ID_1);
            }
            UG_Update();
            //return but_press;
            val = 0;                // go out now
        }
        else
        {
            xtimer_usleep( 250000);
            val1++;
            if( val1 == 4) { val1 = 0; val--; }

            UG_ProgressBarSetValue( &window_5, TXB_ID_0, 100 * val / 20);
            UG_Update();
        }
        if( val == 0)
            break;
    }

#ifdef MESSAGEBOX_SLIDE
    // Penner Easing Functions Test
    time        = 0;
    beginning   = (XSIZE_PHYS - guiAS->x_dim) / 2.0;
    change      = -500;
    duration    = 800.0;

    while( time < duration)
    {
        float a = bounce_In( time, beginning, change, duration);
        _HW_layerPosition( guiAS, (int)(a), (YSIZE_PHYS - guiAS->y_dim) / 2);
        xtimer_usleep( 1000);
        time++;
    }
#else
    _HW_layerPosition( guiAS, (int)(XSIZE_PHYS), (YSIZE_PHYS - guiAS->y_dim) / 2);
#endif
#if 0
    UG_SelectGUI( FOREGROUND_LAYER, prevGui);
#else
    UG_releaseGUI( FOREGROUND_LAYER, guiAS);
#endif

    // !@# 2do: fix this issue with all foreground guis taking another video buffer
    //display_free( guiAS->backupVideoBuffer);
    UG_deleteGUI( guiAS);

    free( guiAS);
    return but_press;
}

/**
 * @brief   This function displays the PIN entered by user. However, to protect
 *          user's privacy it can only show asterisks...
 *
 * @param pinText
 * @return
 */
int display_setTextGetPIN( const char *pinText)
{
char asterisks[16];

    // if using a non-encrypting EPP, show ***** but keep the pin text
    if( pinText[0] != '*')
    {
        memset( asterisks, '*', strlen( pinText));
        asterisks[strlen( pinText)] = NULL;
    }
    else
        strcpy( asterisks, pinText);

    UG_TextboxSetText( &window_6, TXB_ID_1, (char *)/*pinText*/asterisks);
    UG_Update();
    return 0;
}


void display_updateStatusStrip( const char *text)
{
    if( UG_GetCurrentWindow( BACKGROUND_LAYER) == &window_2)
    {
        UG_invalidateObject( BACKGROUND_LAYER, &window_2, OBJ_TYPE_TEXTBOX, TXB_ID_3);
        UG_TextboxSetText( &window_2, TXB_ID_3, (char *)text);
    }
    return;
}
/**
 * @brief   displays get PIN message and runs all the necessary EPP things
 *          to get the PIN from the PIN Pad
 *
 * @param[in] pinMaxLen - maximum length of requested PIN
 * @param[in] hdrMsg    - message to show at the top of the window
 * @param[in] errMsg    - message to show when error occurs
 * @param[in] cancelMsg - message to show when user cancels input
 * @param[in] timeoutMsg- message to show when input times out
 * @param[in] timeout   - timeout in seconds, 0 is forever
 * @param[in] needsCard - does this PIN entry needs card inserted in creator reader
 * @param[in] isCardInserted - pointer to function which checks if card is inserted
 * @return    < 0 if error, 0 if OK
 */

#define PIN_USER_CANCELED           -1
#define PIN_TIMEOUT                 -2
#define PIN_OK                      0

int display_getPIN( char *plaintext, const char *PAN,
                    uint8_t pinMaxLen,
                    const char *hdrMsg,
                    const char *errMsg,
                    const char *cancelMsg,
                    const char *timeoutMsg,
                    uint timeout, bool needsCard,
                    int (*isCardInserted)( void))
{
char displayString[32];
uint last_len = 0;
int ret = 0;

    pin_keyadValue = 0;

    // display enter PIN... ROM image?
    if( epp_getPIN( pinMaxLen) != EPP_OK)
    {
        if( errMsg != NULL)
        {// use setupMessageBox()....
            display_messageBox( errMsg);
#if 0
            //UG_Update();                    // added NS 02-11
            xtimer_usleep( 3000000);
            display_removeMessageBox();
#endif
            //UG_Update();                    // added NS 02-11
            return EPP_ERROR;
        }
    }
    else
    {
        display_windowGetPIN( hdrMsg);          // show the get PIN window
        displayString[0] = (char)NULL;          // nothing to display at first

        //! SCAN EPP and wait for PIN entry end
        uint uptoCount = 5;
        uint startto = timeout, val1 = 0;
        uint waittime = 1000000 / uptoCount;

        if( !needsCard) isCardInserted = NULL;
        ret = display_waitForPinEntry( displayString, timeout, isCardInserted);
        if( ret == PIN_TIMEOUT)
        {
            display_windowGetPinSlideOut();
            if( timeoutMsg != NULL)
            {
                display_messageBox( timeoutMsg);
                //UG_Update();
                //xtimer_usleep( 3000000);
                //display_removeMessageBox();
            }
            return EPP_ERROR;
        }
        else if( ret == PIN_USER_CANCELED)
        {
            display_windowGetPinSlideOut();
            if( cancelMsg != NULL)
            {
                display_messageBox( cancelMsg);
                //UG_Update();
                //xtimer_usleep( 3000000);
                //display_removeMessageBox();
                //UG_Update();
            }
            return EPP_ERROR;
        }
#if 0
        while( /*( ret = epp_waitForPinEnd( displayString)) != EPP_OK*/ window_pin_callback != EPP_OK)
        {
            ret = window_pin_callback;

            if( ret == EPP_USER_CANCELED)
                break;
            else if( ret == EPP_USER_CLEARED)
                displayString[0] = (char)NULL;          // nothing to display after clear is pressed
            else
            {
                displayString[last_len] = ret;
                displayString[last_len + 1] = NULL;
            }
            // display the displayString
            if( strlen( displayString) != last_len)
            {
                // display the string with ****
                display_setTextGetPIN( displayString);
                last_len = strlen( displayString);
                timeout = startto;
            }
            xtimer_usleep( waittime);
            val1++;
            if( val1 == uptoCount) { val1 = 0; timeout--; }

            UG_ProgressBarSetValue( &window_6, TXB_ID_0, 100 * timeout / startto);
            UG_Update();
            if( timeout == 0)
            {
                if( timeoutMsg != NULL)
                {
                    display_messageBox( timeoutMsg);
                    UG_Update();
                    xtimer_usleep( 3000000);
                    display_removeMessageBox();
                }
                ret = -1;
                break;
            }
            if( needsCard && /*CREATOR_CARD_INSERTED != creator_getStatus()*/ !isCardInserted())
            {
                ret = EPP_USER_CANCELED;
                break;
            }
        } // endwhile
#endif
    } // endif
    display_windowGetPinSlideOut();

    if( displayString[0] != '*')
    {
        strcpy( plaintext, displayString);
        ret = EPP_OK;
    }
    else
    {
        if( epp_getPinBlock( plaintext, PAN, strlen( PAN)) != EPP_OK)
            ret = EPP_ERROR;
        else
            ret = EPP_OK;
    }
    return ret;
} // endfunc

/**
 * @brief   this function takes keys from the keypad and displays it, manages
 *          the timeout and check if card is still in the reader (if should)
 *
 * @param displayString
 * @param timeout - in seconds, 0 is forever
 * @param isCardInserted - NULL is card inserted status should not be checked
 *
 * @return PIN_USER_CANCELED, PIN_TIMEOUT or PIN_OK. IF PIN_OK is returned, the
 *         pin code is in displayString
 */

int display_waitForPinEntry( char *displayString, uint timeout, int (*isCardInserted)( void))
{
int last_len = 0, ret;

    displayString[0] = (char)NULL;          // nothing to display at first

    //! SCAN EPP and wait for PIN entry end
    uint uptoCount = 5;
    uint startto = timeout, val1 = 0;
    uint waittime = 1000000 / uptoCount;
    while( true/*( ret = epp_waitForPinEnd( displayString)) != EPP_OK*/ /*pin_keyadValue != 0*/)
    {
        ret = pin_keyadValue;
        pin_keyadValue = 0;

        if( ret == KEY_CANCEL)
            break;
        else if( ret == KEY_CLEAR)
            displayString[0] = (char)NULL;          // nothing to display after clear is pressed
        else if( ret != 0)
        {
            displayString[last_len] = ret;
            displayString[last_len + 1] = NULL;
        }
        // display the displayString
        if( strlen( displayString) != last_len)
        {
            // display the string with ****
            display_setTextGetPIN( displayString);
            last_len = strlen( displayString);
            timeout = startto;
        }
        xtimer_usleep( waittime);
        val1++;
        if( val1 == uptoCount) { val1 = 0; timeout--; }

        UG_ProgressBarSetValue( &window_6, TXB_ID_0, 100 * timeout / startto);
        display_periodic(); // instead of UG_Update();
        if( timeout == 0)
            return PIN_TIMEOUT;
        if( isCardInserted != NULL && !isCardInserted())
            return( PIN_USER_CANCELED);
    } // endwhile
    return PIN_OK;      // entry ended OK
} // endfunc


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
                              const char *PAN)
{
char displayString[32];
uint last_len = 0;


    displayString[0] = (char)NULL;        // nothing to display at first
    // display enter PIN... ROM image?
    if( epp_getPIN( maxlength) != EPP_OK) return EPP_ERROR;

    display_windowGetPIN( enterPIN /*"ENTER YOUR PIN"*/);
#if 0
    while( epp_waitForPinEnd( displayString) != EPP_OK)
    {
        // display the displayString
        if( strlen( displayString) != last_len)
        {
            // display the string with ****
            display_setTextGetPIN( displayString);
            last_len = strlen( displayString);
        }
        xtimer_usleep( 100000);
    } // endwhile
#else
    int ret = display_waitForPinEntry( displayString, 30, NULL);
    if( ret != EPP_OK)
        return ret;
#endif

    if( epp_getPinBlock( encryptedPinBlock, PAN, strlen( PAN)) != EPP_OK)
    {
        memset( encryptedPinBlock, 0, sizeof( encryptedPinBlock));
        return EPP_ERROR;
    }
    else return EPP_OK;
}


/**
 * @brief       Sets up window_6 to show the get PIN interaction with user
 *
 * @param[in]   headerMsg - the text to show on the head of the get PIN window
 *
 * @return      currently none
 */
static UG_GUI *prevgui, *guiPIN;
int display_windowGetPIN( const char *headerMsg)
{
    guiPIN = (UG_GUI *)malloc( sizeof( UG_GUI));
    if( guiPIN == NULL)
        return 0;             // no memory left

    UG_Init( guiPIN, NULL, 500, 290, FOREGROUND_LAYER);
    // NS 31-07-17 ???  guiPIN->currentVideoBuffer = guiPIN->mainVideoBuffer = guiPIN->backupVideoBuffer;
    display_regsiterDrivers( guiPIN);

    guiPIN->colorKey = C_BLACK;

    /* -------------------------------------------------------------------------------- */
    /* Create Window 6 (µGUI)                                                           */
    /* -------------------------------------------------------------------------------- */
    UG_WindowCreate( &window_6, obj_buff_wnd_6, MAX_OBJECTS, window_pin_callback);
    uint8_t style1 = UG_WindowGetStyle( &window_6);
    UG_WindowSetStyle( &window_6, style1 & (~WND_STYLE_SHOW_TITLE));     // dont show title for the buttons window
    UG_WindowSetTitleText( &window_6, "Keypad Entry" );
    UG_WindowSetTitleTextFont( &window_6, &FONT_12X20 );
    UG_WindowSetBackColor( &window_6, TO_UG_COLOR( 0, 0, 56, 69));

    UG_RECT buttonRect = { 0, 0, guiPIN->x_dim, guiPIN->y_dim};
    UG_WindowResize( &window_6, 0, 0, buttonRect.xe, buttonRect.ye);

    // ---------- add a progress bar at the bottom of the window to show how much time is left
    UG_ProgressBarCreate( &window_6, &progress6_1, TXB_ID_0,
            buttonRect.xs + 5, buttonRect.ye - 27, buttonRect.xe - 5, buttonRect.ye - 5);
    UG_ProgressBarSetForeColor( &window_6, TXB_ID_0, C_POWDER_BLUE, C_WHITE);

    uint innerWidth  = UG_WindowGetInnerWidth( &window_6 );
    uint innerHeight = UG_WindowGetInnerHeight( &window_6);

    //! --------------- set up the header message text box ------------------
    UG_TextboxCreate( &window_6, &textbox6_2, TXB_ID_2, 10, 10, innerWidth - 10, 60);
    UG_TextboxSetFont( &window_6, TXB_ID_2, &FONT_TAHOMA_48);
    UG_TextboxSetText( &window_6, TXB_ID_2, (char *)headerMsg);
    UG_TextboxSetAlignment( &window_6, TXB_ID_2, ALIGN_CENTER);
    UG_TextboxSetForeColor( &window_6, TXB_ID_2, C_WHITE);
    UG_TextboxSetBackColor( &window_6, TXB_ID_2, C_TRANSPARENT);
    UG_TextboxSetHSpace( &window_6, TXB_ID_2, 0 );

    //! ----------------- set up the PIN '****' text box --------------------
    UG_TextboxCreate( &window_6, &textbox6_1, TXB_ID_1, 50, innerHeight / 2 - 40,
                        innerWidth - 50, innerHeight / 2 + 40);
    UG_TextboxSetFont( &window_6, TXB_ID_1, &FONT_32X53);
    UG_TextboxSetText( &window_6, TXB_ID_1, " " );
    UG_TextboxSetAlignment( &window_6, TXB_ID_1, ALIGN_CENTER);
    UG_TextboxSetForeColor( &window_6, TXB_ID_1, C_LIGHT_GRAY);
    UG_TextboxSetBackColor( &window_6, TXB_ID_1, TO_UG_COLOR( 0, 0, 56, 69));
    UG_TextboxSetHSpace( &window_6, TXB_ID_1, 2 );

    //mb_prevgui = UG_ResetGUI( FOREGROUND_LAYER, guiMB, -guiMB->x_dim, 0, guiMB->x_dim, guiMB->y_dim, LTDC_Pixelformat_RGB565);

    prevgui = UG_SelectGUI( FOREGROUND_LAYER, guiPIN); // need to do a UG_ResetGUI here?
    UG_FillScreen( FOREGROUND_LAYER, C_BLACK);
    UG_WindowShow( FOREGROUND_LAYER, &window_6);
    UG_Update();


#ifdef MESSAGEBOX_SLIDE
    static float time, beginning, end, duration;

    // Penner Easing Functions Test
    time        = 0;
    beginning   = -500.0;
    end         = (XSIZE_PHYS - guiPIN->x_dim) / 2.0;
    duration    = 800.0;

    while( time < duration)
    {
        float a = bounce_In( time, beginning, end, duration);
        _HW_layerPosition( guiPIN, (int)(a), (YSIZE_PHYS - guiPIN->y_dim) / 2);
        xtimer_usleep( 1000);
        time++;
    }
#else
    _HW_layerPosition( guiPIN, (int)(XSIZE_PHYS - guiPIN->x_dim) / 2.0, (YSIZE_PHYS - guiPIN->y_dim) / 2);
#endif
    return 0;
}


void display_windowGetPinSlideOut( void)
{
#ifdef MESSAGEBOX_SLIDE
    static float time, beginning, end, duration;

    // Penner Easing Functions Test
    time        = 0;
    beginning   = (XSIZE_PHYS - guiPIN->x_dim) / 2.0;
    end         = XSIZE_PHYS;
    duration    = 800.0;

    while( time < duration)
    {
        float a = bounce_In( time, beginning, end, duration);
        _HW_layerPosition( guiPIN, (int)(a), (YSIZE_PHYS - guiPIN->y_dim) / 2);
        xtimer_usleep( 1000);
        time++;
    }
#else
    _HW_layerPosition( guiPIN, (int)(XSIZE_PHYS), (YSIZE_PHYS - guiPIN->y_dim) / 2);
#endif
    //UG_SelectGUI( FOREGROUND_LAYER, prevgui);

    UG_releaseGUI( FOREGROUND_LAYER, guiPIN);

    //display_free( guiPIN->backupVideoBuffer);
    UG_deleteGUI( guiPIN);

    free( guiPIN);
    //LTDC_LayerCmd( LTDC_Layer2, DISABLE);   // TEST
    guiPIN = NULL;
}






/**
 * @brief       Displays a message box using FOREGROUND layer (guiSmall)
 *              and waits for an input from EPP (or touch screen) or for timeout
 *              to expire - - - ->  does not use a new thread
 *
 * @param title - string to show at title
 * @param text  - string to show at middle of box
 * @param timeout - time in seconds, or zero if indefinite (currently 1 hour!)
 *
 * @returns none
 */
void display_setupMessageBox( const char *title, const char *text, uint timeout)
{
#define WINDOW_1_WIDTH      420
#define WINDOW_1_HEIGHT     240
#define W1_BUTTON_WIDTH     100
#define W1_BUTTON_HEIGHT    45

    UG_GUI *guiMB = (UG_GUI *)malloc( sizeof( UG_GUI));
    if( guiMB == NULL)
        return;

    UG_Init( guiMB, NULL, 501, 290, FOREGROUND_LAYER);              // was 500 debug
    display_regsiterDrivers( guiMB);
    guiMB->screenStartx = -guiMB->x_dim;
    guiMB->screenStarty = 0;
    //UG_GUI *prevgui = UG_ResetGUI( FOREGROUND_LAYER, guiMB, -guiMB->x_dim, 0, guiMB->x_dim, guiMB->y_dim, LTDC_Pixelformat_RGB565);
    UG_SelectGUI( FOREGROUND_LAYER, guiMB);

    //UG_GUI *prevgui = UG_SelectGUI( guiMB);

    // test
    //....doesnt seem to add anything..... LTDC_LayerCmd( LTDC_Layer1, DISABLE);
    guiMB->colorKey = C_BLACK;
    window5_pressed = EPP_STILL_SCANNING;;
    UG_FillScreen( FOREGROUND_LAYER, C_BLACK);

    /* Window 7 */
    /* used to show MessageBox to user with OK button and progressbar */
    UG_WINDOW *window_7 = ( UG_WINDOW *)malloc( sizeof( UG_WINDOW));
    UG_OBJECT *obj_buff_wnd_7 = (UG_OBJECT *)malloc(sizeof( UG_OBJECT)*MAX_OBJECTS);
    UG_PROGRESS_BAR *progress7_1 = (UG_PROGRESS_BAR *)malloc( sizeof( UG_PROGRESS_BAR));
    UG_TEXTBOX *textbox7_1 = (UG_TEXTBOX *)malloc( sizeof( UG_TEXTBOX));
    UG_BUTTON *button7_1 = (UG_BUTTON *)malloc( sizeof( UG_BUTTON));

    UG_WindowCreate( window_7, obj_buff_wnd_7, MAX_OBJECTS, window_5_callback);
    UG_WindowResize( window_7, (guiMB->x_dim - WINDOW_1_WIDTH) / 2,
                                (guiMB->y_dim - WINDOW_1_HEIGHT) / 2,
                                WINDOW_1_WIDTH + (guiMB->x_dim - WINDOW_1_WIDTH) / 2,
                                WINDOW_1_HEIGHT + (guiMB->y_dim - WINDOW_1_HEIGHT) / 2);
    if( title != NULL)
    {
        UG_WindowSetTitleText( window_7, (const char *)title);
        UG_WindowSetTitleTextFont( window_7, &FONT_16X26);
        UG_WindowSetBackColor( window_7, C_WHITE);
    }
    else
    {
        uint8_t style = UG_WindowGetStyle( window_7);
        UG_WindowSetStyle( window_7, style & (~WND_STYLE_SHOW_TITLE));     // dont show title for this window
    }
    UG_WindowSetBackColor( window_7, C_WHITE_SMOKE);
    UG_TextboxCreate( window_7, textbox7_1, TXB_ID_0, 0, 0,
                      UG_WindowGetInnerWidth( window_7 ),
                      UG_WindowGetInnerHeight( window_7) - W1_BUTTON_HEIGHT * 2 - 10);
    UG_TextboxSetFont( window_7, TXB_ID_0, &FONT_GISHA_38);
    UG_TextboxSetText( window_7, TXB_ID_0, (const char *)text );
    UG_TextboxSetAlignment( window_7, TXB_ID_0, ALIGN_CENTER);
    UG_TextboxSetForeColor( window_7, TXB_ID_0, C_DARK_SLATE_GRAY);
    UG_TextboxSetBackColor( window_7, TXB_ID_0, C_WHITE_SMOKE);
    UG_TextboxSetHSpace( window_7, TXB_ID_0, 0 );

    UG_ButtonCreate( window_7, button7_1, BTN_ID_0, (WINDOW_1_WIDTH - W1_BUTTON_WIDTH) / 2,
                    WINDOW_1_HEIGHT - W1_BUTTON_HEIGHT * 2 - 38,
                    (WINDOW_1_WIDTH - W1_BUTTON_WIDTH) / 2 + W1_BUTTON_WIDTH,
                    WINDOW_1_HEIGHT - W1_BUTTON_HEIGHT * 2 + W1_BUTTON_HEIGHT - 38);
   /* Configure Button 1 */
    UG_ButtonSetFont( window_7, BTN_ID_0, &FONT_GISHA_38);
    UG_ButtonSetBackColor( window_7, BTN_ID_0, C_LIME );
    UG_ButtonSetText( window_7, BTN_ID_0, "Cancel" );
    //UG_ButtonHide( window_7, BTN_ID_0);

    UG_AREA area;
    UG_WindowGetArea( window_7, &area);

    // ---------- add a progress bar at the bottom of the window to show how much time is left
#if 0
    UG_ProgressBarCreate( window_7, progress7_1, TXB_ID_1,
                           4, (area.ye - area.ys) - 27,
                           (area.xe - area.xs) - 8, (area.ye - area.ys) - 5);
#else
    UG_ProgressBarCreate( window_7, progress7_1, TXB_ID_1,
                           4, (area.ye - area.ys) - 27,
                           (area.xe - area.xs) - 8, (area.ye - area.ys) - 5);

    // when timeout is zero, go for indeterminate time
    if( timeout == 0)
    {
        UG_ProgressBarSetStyle( window_7, TXB_ID_1, WND_STYLE_3D | PROGBAR_STYLE_NO_LIMIT);
        for( int p = 0; p < 100; p++)
        {
            UG_ProgressBarSetValue( window_7, TXB_ID_1, p);
            UG_Update();
            xtimer_usleep( 100000);
        }
    }
#endif
    UG_ProgressBarSetForeColor( window_7, TXB_ID_1, C_POWDER_BLUE, C_ORANGE_RED);

    //UG_WindowShow( FOREGROUND_LAYER, window_7);
    UG_WindowShowGui( guiMB, window_7);
    UG_Update();


#ifdef MESSAGEBOX_SLIDE
    static float time, beginning, end, duration;

    // Penner Easing Functions Test
    time        = 0;
    beginning   = -guiMB->x_dim;
    end         = (XSIZE_PHYS - guiMB->x_dim) / 2.0;
    duration    = 1000.0; // was 800

    while( time < duration)
    {
        float a = bounce_In( time, beginning, end, duration);
        _HW_layerPosition( guiMB, (int)(a), (YSIZE_PHYS - guiMB->y_dim) / 2);
        UG_Update();
        xtimer_usleep( 1000);
        time++;
    }
#else
    _HW_layerPosition( guiMB, (int)(XSIZE_PHYS - guiMB->x_dim) / 2.0, (YSIZE_PHYS - guiMB->y_dim) / 2);
#endif

    float val = 100; // wait 4 seconds for a click
    int val1 = 0;
    //UG_DriverDisable( DRIVER_FILL_FRAME);

    // testing 16-10-16
    //UG_WindowShow( FOREGROUND_LAYER, window_7);
    UG_WindowShowGui( guiMB, window_7);

    /////////////////////////
    // WAIT FOR __1__ KEY
#if 0
    if( epp_getPlaintext( 1) != EPP_OK)
    {
        // EPP error???!!!
    }
#endif
    int ret = 0;
    char displayString[12];
    if( timeout == 0) timeout = 3600;

    while( true/*( ret = epp_waitForPinEnd( displayString)) != EPP_OK*/)
    {
        ret = window5_pressed;
        if( ret == EPP_USER_CANCELED)
            break;
        // display the displayString

    //while( 1)
    //{
        xtimer_usleep( 100000);
        UG_U8 vbyte = (UG_U8)val;
        UG_ProgressBarSetValue( window_7, TXB_ID_1, vbyte);
        val -= 100.0 / (timeout * 10.0);
        UG_Update();
        if( val <= 0 || window5_pressed != EPP_STILL_SCANNING)
            break;
    }

#ifdef MESSAGEBOX_SLIDE
    // easing out
    time        = 0;
    beginning   = end;
    end         = XSIZE_PHYS;
    duration    = 1000.0; // was 800

    while( time < duration)
    {
        float a = bounce_In( time, beginning, end, duration);
        _HW_layerPosition( guiMB, (int)(a), (YSIZE_PHYS - guiMB->y_dim) / 2);
        xtimer_usleep( 1000);
        time++;
    }
#else
    _HW_layerPosition( guiMB, (int)(XSIZE_PHYS), (YSIZE_PHYS - guiMB->y_dim) / 2);
#endif
    // switch back to previous parameters of this gui and clear the current screen
    //UG_ResetGUI( guiMB, 0, 0, 800, 480, LTDC_Pixelformat_RGB565);
    UG_FillScreen( FOREGROUND_LAYER, C_BLACK);
    UG_WindowHide( FOREGROUND_LAYER, window_7);

    UG_releaseGUI( FOREGROUND_LAYER, guiMB);

    //UG_SelectGUI( FOREGROUND_LAYER, prevgui);

    free( window_7);
    free( obj_buff_wnd_7);
    free( progress7_1);
    free( textbox7_1);
    free( button7_1);
    // !@# 2do: fix this issue with all foreground guis taking another video buffer
    //display_free( guiMB->backupVideoBuffer);
    UG_deleteGUI( guiMB);

    free( guiMB);
    return;
}






/**
 * @brief   YES. This function would (in the future) wait for either a
 *          button click on the touch screen or button key from the EPP!!!
 *
 * @param timeout - in milliseconds. send with 0 timeout for nonblocking
 *
 * @return 0 if timeout, -1 if CANCEL, otherwise key
 */
int getChar( uint timeout)
{
    uint32_t starttime = xtimer_now();

    while( !window5_pressed && xtimer_now() - starttime < timeout)
        xtimer_usleep( 100000);
    if( window5_pressed) return window5_pressed;
    else return 0;
}


static float bounce_In( float time, float begin, float end, float duration)
{
    float v = 1.2 * (end - begin) / duration;
    if( time > duration * 0.9)
    {
        float backv = 0.8 * (end - begin) / duration;
        return v * (duration * 0.9) - backv * (time - duration * 0.9) + begin;
    }
    return( v * time + begin);
}


/**
 * @brief   sets the frame position relative to screen
 * @param layer
 * @param x     upper left corner of frame's location
 * @param y
 */
void _HW_layerPosition( UG_GUI *_gui, int x, int y)
{
    mutex_lock( &ltdc_sync);

    // first update gui with new start x and y
    _gui->screenStartx = x;
    _gui->screenStarty = y;

// from ugui.c
    //! deal with pixel format and CLUT
    if( _gui->LTDC_Pixelformat == LTDC_Pixelformat_L8 ||
         _gui->LTDC_Pixelformat == LTDC_Pixelformat_AL44 ||
          _gui->LTDC_Pixelformat == LTDC_Pixelformat_AL88)
        LTDC_CLUTCmd( _gui->hwlayer, ENABLE);
    else
        LTDC_CLUTCmd( _gui->hwlayer, DISABLE);

    //! change pixel format if applicable
    _gui->hwlayer->PFCR = _gui->LTDC_Pixelformat;

    // if foreground layer we should take care of color keying and blending
    if( _gui->hwlayer == LTDC_Layer2)
    {
        /* Specifies the blending factors */
        _gui->hwlayer->BFCR &= ~(LTDC_LxBFCR_BF2 | LTDC_LxBFCR_BF1);
        _gui->hwlayer->BFCR = ( LTDC_BlendingFactor1_PAxCA | /*LTDC_BlendingFactor2_CA*/
                                LTDC_BlendingFactor2_PAxCA);

        if( _gui->colorKey != C_TRANSPARENT)
        {
            //LTDC_LayerAlpha( LTDC_Layer2, 180);
            LTDC_ColorKeying_InitTypeDef LTDC_colorkeying_InitStruct;
            LTDC_colorkeying_InitStruct.LTDC_ColorKeyBlue = UG_COLOR_TO_BLUE( _gui->colorKey);
            LTDC_colorkeying_InitStruct.LTDC_ColorKeyGreen = UG_COLOR_TO_GREEN( _gui->colorKey);
            LTDC_colorkeying_InitStruct.LTDC_ColorKeyRed = UG_COLOR_TO_RED( _gui->colorKey);

            LTDC_ColorKeyingConfig( _gui->hwlayer, &LTDC_colorkeying_InitStruct, ENABLE);
        }
        else
        {
            LTDC_ColorKeying_InitTypeDef LTDC_colorkeying_InitStruct;
            LTDC_colorkeying_InitStruct.LTDC_ColorKeyBlue = 0;
            LTDC_colorkeying_InitStruct.LTDC_ColorKeyGreen = 0;
            LTDC_colorkeying_InitStruct.LTDC_ColorKeyRed = 0;

            LTDC_ColorKeyingConfig( _gui->hwlayer, &LTDC_colorkeying_InitStruct, DISABLE);
        }

    } // endif foreground layer

    //LTDC_ReloadConfig( /*LTDC_IMReload*/LTDC_VBReload);
/////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////
    // want entire frame out of the LCD? turn entire layer OFF
    //////////////////////////////////////////////////////////////////
    if( x <= -_gui->x_dim || x >= XSIZE_PHYS ||
             y <= -_gui->y_dim || y >= YSIZE_PHYS)
    {
        LTDC_LayerCmd( _gui->hwlayer, DISABLE);
        LTDC_ReloadConfig( LTDC_IMReload/*LTDC_VBReload*/);
        mutex_unlock( &ltdc_sync);
        return;
    }
    else
    {
#if 1           // NS 23-03
        LTDC_LayerCmd( _gui->hwlayer, ENABLE);
#endif
        LTDC_ReloadConfig( LTDC_IMReload);
    }

    int bpp = dislpay_getBytesPerPixel( _gui);
    //
    //! if x or y is out of the LCD
    if( x < 0 || x >= XSIZE_PHYS - _gui->x_dim ||
            y < 0 || y >= YSIZE_PHYS - _gui->y_dim)
    {
        int wx = x, hy = y;

        ///////////////////////////////////////////////
        uint32_t add = SDRAM_BANK_ADDR;
        if( _gui->hwlayer == LTDC_Layer2)
            add += LAYER_2_OFFSET;
        add = UG_GetCurVideoBuffer( _gui); //_gui->currentVideoBuffer;
        ///////////////////////////////////////////////

        uint showxBytes = 0;
        if( x < 0) { showxBytes = -x * bpp; x = 0; }
        else if( x >= XSIZE_PHYS - _gui->x_dim)
            wx = XSIZE_PHYS - wx - _gui->x_dim;
        else
            wx = 0;

        if( y < 0) { showxBytes += -y * bpp * _gui->x_dim/*XSIZE_PHYS*/; y = 0; }
        else if( y >= YSIZE_PHYS - _gui->y_dim)
            hy = YSIZE_PHYS - hy - _gui->y_dim; //hy - _gui->y_dim;
        else
            hy = 0;

        uint WidthPixels  = ( _gui->x_dim + wx);            // # x pixels shown
        uint HeightPixels = _gui->y_dim + hy;               // # y pixels shown

        if( (uint32_t)showxBytes + add != _gui->hwlayer->CFBAR)
            LTDC_LayerAddress( _gui->hwlayer, (uint32_t)showxBytes + add);
        {
            /* Reconfigures the horizontal and vertical start position */
            uint tempreg = LTDC->BPCR;
#if 1
            uint horizontal_start = (tempreg >> 16) + 1 + x;
            uint vertical_start = (tempreg & 0xFFFF) + 1 + y;
#else
            uint horizontal_start = 1 + x;
            uint vertical_start = 1 + y;
#endif
            uint horizontal_stop = ( WidthPixels - 1) + horizontal_start /* NS 1809 see p. 524 @ RM0385  - 1*/;
            uint vertical_stop   = ( HeightPixels - 1) + vertical_start  /* NS 1809  - 1*/;

            _gui->hwlayer->WHPCR = horizontal_start | (horizontal_stop << 16);
            _gui->hwlayer->WVPCR = vertical_start | (vertical_stop << 16);
        }

        // ------ set the layer size ------
        _gui->hwlayer->CFBLR  = ((/*XSIZE_PHYS*/_gui->x_dim * bpp) << 16) |
                                ((/*XSIZE_PHYS*/WidthPixels * bpp) + 3);

        /* Reconfigures the frame buffer line number */
        _gui->hwlayer->CFBLNR  = HeightPixels;

    }
    else
    {
        uint32_t add; /* = SDRAM_BANK_ADDR;
        if( _gui->hwlayer == LTDC_Layer2)
            add += LAYER_2_OFFSET;
        */
        add = UG_GetCurVideoBuffer( _gui); //_gui->currentVideoBuffer;

        uint WidthPixels  = _gui->x_dim;
        uint HeightPixels = _gui->y_dim;

        if( (uint32_t)add != _gui->hwlayer->CFBAR)
            LTDC_LayerAddress( _gui->hwlayer, (uint32_t)add);
        {
            /* Reconfigures the horizontal and vertical start position */
            uint tempreg = LTDC->BPCR;
            uint horizontal_start = (tempreg >> 16) + 1 + x;
            uint vertical_start = (tempreg & 0xFFFF) + 1 + y;

            uint horizontal_stop = (WidthPixels - 1) + horizontal_start /*  ?? see above - 1 */;
            uint vertical_stop   = (HeightPixels - 1) + vertical_start  /*  ?? see above - 1 */;

            _gui->hwlayer->WHPCR = horizontal_start | (horizontal_stop << 16);
            _gui->hwlayer->WVPCR = vertical_start | (vertical_stop << 16);
        }
        // ------ set the layer size ------
        _gui->hwlayer->CFBLR  = ((/*XSIZE_PHYS*/_gui->x_dim * bpp) << 16) | ((/*XSIZE_PHYS*/_gui->x_dim * bpp) + 3);
        /* Reconfigures the frame buffer line number */
        _gui->hwlayer->CFBLNR  = HeightPixels;
    }
    LTDC_ReloadConfig( LTDC_VBReload);

    mutex_unlock( &ltdc_sync);

} // endfunc layer position


/**
 * @brief   returns the number of bytes per each pixel
 * @param _gui
 * @return
 */
int dislpay_getBytesPerPixel( UG_GUI *_gui)
{
    int temp;
#if 0
    uint pxlformat = GUIDRV_getPixelformatByLayer( _gui->hwlayer);
#else
    uint pxlformat = _gui->LTDC_Pixelformat;
#endif
    if (pxlformat == LTDC_Pixelformat_ARGB8888)
    {
      temp = 4;
    }
    else if (pxlformat == LTDC_Pixelformat_RGB888)
    {
      temp = 3;
    }
    else if ((pxlformat == LTDC_Pixelformat_ARGB4444) ||
            (pxlformat == LTDC_Pixelformat_RGB565)    ||
            (pxlformat == LTDC_Pixelformat_ARGB1555)  ||
            (pxlformat == LTDC_Pixelformat_AL88))
    {
      temp = 2;
    }
    else
    {
      temp = 1;
    }
    return temp;
} // endfunc


/**
 * These are the Penner Bounce functions
 * taken from ....
 */
float bounce_easeIn ( float t, float b, float c, float d)
{
    return c - bounce_easeOut (d-t, 0, c, d) + b;
}

float bounce_easeOut( float t, float b, float c, float d)
{
    if ((t/=d) < (1/2.75f)) {
        return c*(7.5625f*t*t) + b;
    } else if (t < (2/2.75f)) {
        float postFix = t-=(1.5f/2.75f);
        return c*(7.5625f*(postFix)*t + .75f) + b;
    } else if (t < (2.5/2.75)) {
            float postFix = t-=(2.25f/2.75f);
        return c*(7.5625f*(postFix)*t + .9375f) + b;
    } else {
        float postFix = t-=(2.625f/2.75f);
        return c*(7.5625f*(postFix)*t + .984375f) + b;
    }
}

float bounce_easeInOut( float t, float b ,float c, float d)
{
    if (t < d/2) return bounce_easeIn (t*2, 0, c, d) * .5f + b;
    else return bounce_easeOut (t*2-d, 0, c, d) * .5f + c*.5f + b;
}



//-----------------------------------------------------------------
// S D R A M      M A L L O C
//-----------------------------------------------------------------
typedef struct node {
    struct node *next;
    void *ptr;
} NODE;


void display_free( void *ptr)
{
NODE *first_head;

    mutex_lock( &dmalloc_mutex);

    // noting to free?
    if( ptr == NULL)
        return;

    // search for the pointer
    first_head = (uint8_t *)ptr - sizeof( NODE);
    if( PTR_TO_RAM( first_head->ptr) != ptr)
    {
       mutex_unlock( &dmalloc_mutex);
       return;                   // error
    }
#ifdef DEBUG_MALLOC
    if( debug_counter > 1)
        printf( "Free at address %X\n", first_head);
#endif

    first_head->ptr = NULL;
    mutex_unlock( &dmalloc_mutex);
    return;
}

/**
 * @brief this function initializes the (n-) malloc mechanism
 */
void display_malloc_init( void)
{
    display_malloc( 0);
}

/**
 * @brief Call malloc with size=0 to initialize the entire memory!!!
 *
 * @param[in]   size    number of bytes to malloc
 *
 * @return      pointer to allocated memory
 */

void *display_malloc( size_t size)
{
NODE *first_head;
NODE *optimizerBlock;

    mutex_lock( &dmalloc_mutex);

    int found = 0;
    uint8_t *ptr = NULL;

    first_head = optimizerBlock = /*&_sheap*/SDRAM_BANK_ADDR;
    //!align to 4 bytes
    if( size % 4 != 0)
       size = size + ( 4 - (size % 4));

    // init?
    if( size == 0)
    {
        first_head->next = NULL;
        first_head->ptr = NULL;
        mutex_unlock( &dmalloc_mutex);
        return ptr;
    }
    if( size < 4) size = 4;


    #ifdef DEBUG_MALLOC
    if( debug_counter > 1)
        printf( "request for %d bytes\n", size);
    #endif

    // get the pid to add to the ptr of each NODE
    uint32_t pid = ( thread_getpid() & 0xFF) << 24;

    while( 1)
    {
        // debug
        if( first_head->next < first_head && first_head->next != 0)
        {
            if(( (uint32_t)first_head->next & 0xFFFFFF00) == 0)
            {
                first_head->next = 0;
            }
            else
                while( 1);
        }

        // this block is not occupied
        //if( first_head->ptr < &_sheap || first_head->ptr > &_eheap)
        if( PTR_TO_RAM( first_head->ptr) < /*&_sheap*/SDRAM_BANK_ADDR || PTR_TO_RAM( first_head->ptr) > /*&_eheap*/SDRAM_BANK_ADDR+SDRAM_SIZE)
        {
            /*******************************/
            if( first_head->next < 0x1000 && first_head->next != 0)
                first_head->next = NULL;
            if( first_head->ptr < 0x1000 && first_head->ptr != 0)
                first_head->ptr = NULL;
            /*******************************/
            // no more block allocated after this block - OK to alloc
            if( /*first_head->next == NULL*/(uint32_t)first_head->next < 0x1000)
            {
                // first_head points to the first head.
                // now we need to allocate a chunk of size 'size' and mark the next node on memory
                ptr = (uint8_t *)first_head + sizeof( NODE);

                //! SORRY, not enough room, free some...
                if( ptr + size >= SDRAM_BANK_ADDR+SDRAM_SIZE)
                {
                   mutex_unlock( &dmalloc_mutex);
                   return NULL;
                }
                NODE *next_node = ptr + size;

                next_node->next = NULL;
                next_node->ptr = NULL;

                first_head->next = next_node;
                first_head->ptr  = RAM_TO_PTR( pid, ptr);

                #ifdef DEBUG_MALLOC
                if( debug_counter > 1)
                    printf( "(A) granted at address %X\n", first_head);
                #endif

                break;
            }
            //! we have a free block but other blocks after it. check size of free
            //! block against requested size
            else
            {
                NODE *fptr = first_head;
                while( 1)
                {
                    ///!found
                    ///! testing -- need twice the sizeof NODE or else I will be missing
                    //   on the same length - 4 (20009548 case)
                    uint32_t tempDistance = ( uint32_t)( (uint8_t *)first_head->next - (uint8_t *)fptr);
                    if( /*first_head->next != NULL*/first_head->next > 0x1000 && tempDistance >= size + 1 /*was 2 */ * sizeof( NODE))
                    {
                        // first_head points to the first head.
                        // now we need to allocate a chunk of size 'size' and mark the next node on memory
                        ptr = (uint8_t *)( (uint32_t)fptr + sizeof( NODE));
                        NODE *next_node = ( NODE *)( (uint32_t)ptr + size);
                        if( first_head->next != next_node)
                        {   //! in case remaining space after malloc is less than
                            //! or equal one node - leave as is...
                            if( (uint)first_head->next - (uint)next_node <= sizeof( NODE))
                            {
                                fptr->next = first_head->next;
                                fptr->ptr = RAM_TO_PTR( pid, ptr) /*ptr*/;
                            }
                            else
                            {
                                next_node->next  = first_head->next;    //< this block points to the next block
                                next_node->ptr   = NULL;                //< empty smaller block is left behind
                                fptr->next = next_node;
                                fptr->ptr = RAM_TO_PTR( pid, ptr) /*ptr*/;
                            }
                        }
                        else
                        {
                            fptr->next = next_node;
                            fptr->ptr = RAM_TO_PTR( pid, ptr) /*ptr*/;
                        }
                        found = 1;
                        #ifdef DEBUG_MALLOC
                        if( debug_counter > 1)
                            printf( "(B) granted at address %X\n", fptr);
                        #endif
                        break;
                    } // endif found a large enough chunk
                    else if( /*first_head->next == NULL && first_head->ptr == NULL*/
                              first_head->next < 0x1000 && first_head->ptr < 0x1000)
                    { // last NODE found, see if have enough memory and allocate it
                        if( (uint32_t)fptr + size >= SDRAM_BANK_ADDR+SDRAM_SIZE)
                        {
                           mutex_unlock( &dmalloc_mutex);
                           return NULL;
                        }

                        NODE *next_node = ( NODE *)( (uint32_t)fptr + size + sizeof( NODE));
                        next_node->next = NULL;
                        next_node->ptr = NULL;

                        fptr->next = next_node;
                        fptr->ptr  = (uint8_t *)( (uint32_t)fptr + sizeof( NODE));
                        ptr = PTR_TO_RAM( fptr->ptr) /*fptr->ptr*/;
                        found = 1;
                        #ifdef DEBUG_MALLOC
                        if( debug_counter > 1)
                            printf( "(C) granted at address %X\n", fptr);
                        #endif
                        break;
                    }

                    // not enough room here, check if next chunk if free
                    // if chunk is free check to see if both are enough for 'size'
                    if( /*first_head->next->ptr == NULL*/first_head->next->ptr < 0x1000)
                    {
                        first_head = first_head->next;
#if 0
                        *((uint32_t *)optimizerBlock) = (uint32_t)first_head;
#endif
                        continue;
                    }
                    else
                    {
                        first_head = first_head->next;
                        break;
                    }
                } // endwhile 1
            }
            if( found)
               break;
        } // endwhile forever
        else
        {  // go to check next block if free
           optimizerBlock = first_head = first_head->next;
           continue;
        }
    } // endwhile

#ifdef DEBUG_MALLOC
    debug_counter++;
#endif

    mutex_unlock( &dmalloc_mutex);
    return ptr;
} // endfunc nimrod's malloc


/**
  * @brief  Configure the DMA controller according to the Stream parameters
  *         defined below for a DMA transfer of 2 Bytes (color) to memory
  *         address (usually) in the SDRAM region.
  * @param  None
  * @retval None
  *
  * @note   DMA Stream parameters definitions. You can modify these parameters
  *         to select a different DMA Stream and/or channel.
  *         But note that only DMA2 Streams are capable of Memory to Memory transfers.
  */
#define DMA_STREAM               DMA2_Stream0
#define DMA_CHANNEL              DMA_Channel_0
#define DMA_STREAM_CLOCK         RCC_AHB1Periph_DMA2
#define DMA_STREAM_IRQ           DMA2_Stream0_IRQn
#define DMA_IT_TCIF              DMA_IT_TCIF0
#define DMA_IT_HTIF              DMA_IT_HTIF0
#define DMA_IT_TEIF              DMA_IT_TEIF0
#define DAM_STREAM_IRQHANDLER    DMA2_Stream0_IRQHandler

#define BUFFER_SIZE              32
#define TIMEOUT_MAX              10000 /* Maximum timeout value */
static uint16_t color565;
UG_RESULT _HW_DMA0_Fill( uint xs, uint ys, uint xe, uint ye, UG_COLOR c, UG_GUI *gui)
{
    DMA_InitTypeDef  DMA_InitStructure;

    mutex_lock( &ltdc_sync);

    if( xs >= (uint)gui->x_dim) xs = (uint)gui->x_dim - 1;
    if( xe >= (uint)gui->x_dim) xe = (uint)gui->x_dim - 1;
    if( ys >= (uint)gui->y_dim) ys = (uint)gui->y_dim - 1;
    if( ye >= (uint)gui->y_dim) ye = (uint)gui->y_dim - 1;
    UG_AREA area = { xs, ys, xe, ye };
                      // red                green                   blue
    color565 = ((c & 0xF80000) >> 8) | ((c & 0x00FC00) >> 5) | ((c & 0x0000F8) >> 3);
    int Bpp = GUIDRV_GetBytesPerPixel( LTDC_Pixelformat_RGB565);
    uint32_t addr = gui->mainVideoBuffer + (area.xs + gui->x_dim * area.ys) * Bpp;

    /* Enable DMA clock */
    RCC_AHB1PeriphClockCmd(DMA_STREAM_CLOCK, ENABLE);

    /////////////////////////////////////////////////////////////////////////////
    // if caller wants to fill in a full line of display
    if( area.xs == 0 && area.xe >= gui->x_dim - 1)
    {
        uint left = (1 + area.ye - area.ys) * (1 + area.xe - area.xs);
        if( left == 0) left = 1;

        uint do64k = left / 0xffff;
        uint take = 0xffff;

        // de-init and setup DMA manager data
        DMA_DeInit(DMA_STREAM);
        /* Check if the DMA Stream is disabled before enabling it.
         Note that this step is useful when the same Stream is used multiple times:
         enabled, then disabled then re-enabled... In this case, the DMA Stream disable
         will be effective only at the end of the ongoing data transfer and it will
         not be possible to re-configure it before making sure that the Enable bit
         has been cleared by hardware. If the Stream is used only once, this step might
         be bypassed. */
        while (DMA_GetCmdStatus(DMA_STREAM) != DISABLE) { }

        /* Configure DMA Stream */
        DMA_InitStructure.DMA_Channel = DMA_CHANNEL;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&color565;
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)addr;
        DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory;
        DMA_InitStructure.DMA_BufferSize = (uint32_t)take;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //DMA_PeripheralInc_Enable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //DMA_PeripheralDataSize_Word;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord; //DMA_MemoryDataSize_Word;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = /*DMA_Priority_Low;//*/DMA_Priority_High; //DMA_Priority_Low; //DMA_Priority_High;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        //
        // if less than 64K transfers are requested - do in one sweep and return
        if( do64k == 0)
        {
            DMA_InitStructure.DMA_BufferSize = (uint32_t)left;
            DMA_Init(DMA_STREAM, &DMA_InitStructure);
            //---------------------------
            DMA_Cmd(DMA_STREAM, ENABLE);
            //---------------------------
            while( DMA_GetFlagStatus(DMA_STREAM, DMA_IT_TCIF) != SET) ;

            mutex_unlock( &ltdc_sync);

            return UG_RESULT_OK;
        }
        DMA_Init(DMA_STREAM, &DMA_InitStructure);

        while( true)
        {
            if( do64k == 0)
            {
                take = left % 0xffff;
                // de-init and setup DMA manager data
                DMA_DeInit(DMA_STREAM);
                /* Check if the DMA Stream is disabled before enabling it.
                 Note that this step is useful when the same Stream is used multiple times:
                 enabled, then disabled then re-enabled... In this case, the DMA Stream disable
                 will be effective only at the end of the ongoing data transfer and it will
                 not be possible to re-configure it before making sure that the Enable bit
                 has been cleared by hardware. If the Stream is used only once, this step might
                 be bypassed. */
                while (DMA_GetCmdStatus(DMA_STREAM) != DISABLE) { }

                /* Configure DMA Stream */
                DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)addr;
                DMA_InitStructure.DMA_BufferSize = (uint32_t)take;
                DMA_Init(DMA_STREAM, &DMA_InitStructure);
                //---------------------------
                DMA_Cmd(DMA_STREAM, ENABLE);
                //---------------------------
                while( DMA_GetFlagStatus(DMA_STREAM, DMA_IT_TCIF) != SET) ;

                mutex_unlock( &ltdc_sync);

                return UG_RESULT_OK;
            }
            /* DMA Stream enable */
            DMA_ClearFlag( DMA_STREAM, DMA_IT_TCIF);
            DMA_Cmd(DMA_STREAM, ENABLE);
            /*
            if( DMA_GetCmdStatus(DMA_STREAM)!=ENABLE)
                while(1);
            */
            /* Check if the DMA Stream has been effectively enabled.
             The DMA Stream Enable bit is cleared immediately by hardware if there is an
             error in the configuration parameters and the transfer is no started (ie. when
             wrong FIFO threshold is configured ...) */
            //Timeout = TIMEOUT_MAX;

            while( DMA_GetFlagStatus(DMA_STREAM, DMA_IT_TCIF) != SET) ;
            DMA_ClearFlag( DMA_STREAM, DMA_IT_TCIF);

            addr += take * Bpp;
            if( do64k == 0) break;

            DMA_STREAM->M0AR = addr;

            do64k--;
        } // endwhile forever
    } // endif all of area should be filled with color

    //! if only PART of area should be filled...
    else
    {
        uint left_lines = (1 + area.ye - area.ys);
        if( left_lines == 0) left_lines = 1;

        uint take = 1 + area.xe - area.xs;
        if( take == 0) take = 1;

        DMA_DeInit(DMA_STREAM);
        /* Check if the DMA Stream is disabled before enabling it.
         Note that this step is useful when the same Stream is used multiple times:
         enabled, then disabled then re-enabled... In this case, the DMA Stream disable
         will be effective only at the end of the ongoing data transfer and it will
         not be possible to re-configure it before making sure that the Enable bit
         has been cleared by hardware. If the Stream is used only once, this step might
         be bypassed. */
        while (DMA_GetCmdStatus(DMA_STREAM) != DISABLE) { }

        /* Configure DMA Stream */
        DMA_InitStructure.DMA_Channel = DMA_CHANNEL;
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&color565;
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)addr;
        DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory;
        DMA_InitStructure.DMA_BufferSize = (uint32_t)take;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //DMA_PeripheralInc_Enable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //DMA_PeripheralDataSize_Word;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_PeripheralDataSize_HalfWord; //DMA_MemoryDataSize_Word;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; //DMA_Priority_Low; //DMA_Priority_High;
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
        DMA_Init(DMA_STREAM, &DMA_InitStructure);

        // I won't allow more than 64K lines...
        if( left_lines > 0xffff) while(1);

        for( uint i = 0; i < left_lines; i++)
        {
            if( take < 5)
            {
                for( int x = 0; x < take * Bpp; x += Bpp)
                     *((uint16_t *)(addr + x)) = color565;
            }
            else
            {
                /* DMA Stream enable */
                DMA_ClearFlag( DMA_STREAM, DMA_IT_TCIF|DMA_IT_HTIF|DMA_IT_TEIF);
                DMA_Cmd(DMA_STREAM, ENABLE);
                /*
                if( DMA_GetCmdStatus(DMA_STREAM)!=ENABLE)
                    while(1);
                */
                while( DMA_GetFlagStatus(DMA_STREAM, DMA_IT_TCIF|DMA_IT_TEIF) != SET) ;
                DMA_ClearFlag( DMA_STREAM, DMA_IT_TCIF|DMA_IT_HTIF|DMA_IT_TEIF);
            }
            addr += gui->x_dim * Bpp;
            DMA_STREAM->M0AR = addr;
        } // endwhile forever
    } // endif PART of area should be filled with color

    mutex_unlock( &ltdc_sync);

    return UG_RESULT_OK;
} // endfunc



/**
 * @brief   When the CAT wants to drop the mbthread altogether
 *          call this function
 */
void display_dropMessageBoxThread( void)
{
    //window5_pressed = 1;
    mb_keyadValue = -1000;
}



/**
 * @brief    returns not true if message box is canceled
 *
 */
int display_isMessageBoxCanceled( void)
{
    return mb_keyadValue;
}





//-----------------------------------------------------
// this is a small thread within display which displays
// and updates a messagebox on the foreground layer
//-----------------------------------------------------
void display_mbThread( void *arg)
{
UG_GUI *guiMB, *mb_prevgui;

#define WINDOW_1_WIDTH      420
#define WINDOW_1_HEIGHT     240
#define W1_BUTTON_WIDTH     100
#define W1_BUTTON_HEIGHT    45

    messageBoxStruct *msgbox = (messageBoxStruct *)arg;

    guiMB = (UG_GUI *)malloc( sizeof( UG_GUI));
    if( guiMB == NULL)
        return;             // no memory left

    UG_Init( guiMB, NULL, 502, 290, FOREGROUND_LAYER);          // was 500
    // NS 31-07-17 ???  guiMB->currentVideoBuffer = guiMB->mainVideoBuffer = guiMB->backupVideoBuffer;
    display_regsiterDrivers( guiMB);

    guiMB->colorKey = C_BLACK;

    mb_keyadValue = 0;

    /* Window 7 */
    /* used to show MessageBox to user with OK button and progressbar */
    UG_WINDOW *window_7 = ( UG_WINDOW *)malloc( sizeof( UG_WINDOW));
    UG_OBJECT *obj_buff_wnd_7 = (UG_OBJECT *)malloc(sizeof( UG_OBJECT)*MAX_OBJECTS);
    UG_PROGRESS_BAR *progress7_1 = (UG_PROGRESS_BAR *)malloc( sizeof( UG_PROGRESS_BAR));
    UG_TEXTBOX *textbox7_1 = (UG_TEXTBOX *)malloc( sizeof( UG_TEXTBOX));
    UG_BUTTON *button7_1 = (UG_BUTTON *)malloc( sizeof( UG_BUTTON));


    UG_WindowCreate( window_7, obj_buff_wnd_7, MAX_OBJECTS, window_mb_callback);
    UG_WindowResize( window_7, (guiMB->x_dim - WINDOW_1_WIDTH) / 2,
                                (guiMB->y_dim - WINDOW_1_HEIGHT) / 2,
                                WINDOW_1_WIDTH + (guiMB->x_dim - WINDOW_1_WIDTH) / 2,
                                WINDOW_1_HEIGHT + (guiMB->y_dim - WINDOW_1_HEIGHT) / 2);
    if( msgbox->title != NULL)
    {
        UG_WindowSetTitleText( window_7, (const char *)msgbox->title);
        UG_WindowSetTitleTextFont( window_7, &FONT_16X26);
        UG_WindowSetBackColor( window_7, C_WHITE);
    }
    else
    {
        uint8_t style = UG_WindowGetStyle( window_7);
        UG_WindowSetStyle( window_7, style & (~WND_STYLE_SHOW_TITLE));     // dont show title for this window
    }
    UG_WindowSetBackColor( window_7, C_WHITE_SMOKE);
    UG_TextboxCreate( window_7, textbox7_1, TXB_ID_0, 0, 0,
                      UG_WindowGetInnerWidth( window_7 ),
                      UG_WindowGetInnerHeight( window_7) - W1_BUTTON_HEIGHT * 2 - 10);
    UG_TextboxSetFont( window_7, TXB_ID_0, &FONT_GISHA_38);
    UG_TextboxSetText( window_7, TXB_ID_0, (const char *)msgbox->text );
    UG_TextboxSetAlignment( window_7, TXB_ID_0, ALIGN_CENTER);
    UG_TextboxSetForeColor( window_7, TXB_ID_0, C_DARK_SLATE_GRAY);
    UG_TextboxSetBackColor( window_7, TXB_ID_0, C_WHITE_SMOKE);
    UG_TextboxSetHSpace( window_7, TXB_ID_0, 0 );

    UG_ButtonCreate( window_7, button7_1, BTN_ID_0, (WINDOW_1_WIDTH - W1_BUTTON_WIDTH) / 2,
                    WINDOW_1_HEIGHT - W1_BUTTON_HEIGHT * 2 - 18,
                    (WINDOW_1_WIDTH - W1_BUTTON_WIDTH) / 2 + W1_BUTTON_WIDTH,
                    WINDOW_1_HEIGHT - W1_BUTTON_HEIGHT * 2 + W1_BUTTON_HEIGHT - 18);
   /* Configure Button 1 */
    UG_ButtonSetFont( window_7, BTN_ID_0, &FONT_GISHA_38);
    UG_ButtonSetBackColor( window_7, BTN_ID_0, C_LIME );
    UG_ButtonSetText( window_7, BTN_ID_0, "OK" );
    //UG_ButtonHide( window_7, BTN_ID_0);

    UG_AREA area;
    UG_WindowGetArea( window_7, &area);

    // ---------- add a progress bar at the bottom of the window to show how much time is left
#if 0
    UG_ProgressBarCreate( window_7, progress7_1, TXB_ID_1,
                           4, (area.ye - area.ys) - 27,
                           (area.xe - area.xs) - 8, (area.ye - area.ys) - 5);
#else
    UG_ProgressBarCreate( window_7, progress7_1, TXB_ID_1,
                           4, (area.ye - area.ys) - 27,
                           (area.xe - area.xs) - 8, (area.ye - area.ys) - 5);

    // when timeout is zero, go for indeterminate time
    if( msgbox->timeout == 0)
    {
        UG_ProgressBarSetStyle( window_7, TXB_ID_1, WND_STYLE_3D | PROGBAR_STYLE_NO_LIMIT);
    }
#endif
    UG_ProgressBarSetForeColor( window_7, TXB_ID_1, C_POWDER_BLUE, C_ORANGE_RED);

    //()()()()()()()()()()()()()()()()()()()()()()()()
   // mb_prevgui = UG_ResetGUI( FOREGROUND_LAYER, guiMB, -guiMB->x_dim, 0, guiMB->x_dim, guiMB->y_dim, LTDC_Pixelformat_RGB565);
    guiMB->screenStartx = -guiMB->x_dim;
    guiMB->screenStarty = 0;
    UG_SelectGUI( FOREGROUND_LAYER, guiMB);

    UG_FillScreen( FOREGROUND_LAYER, C_BLACK);
    UG_WindowShow( FOREGROUND_LAYER, window_7);
    UG_Update();

#ifdef MESSAGEBOX_SLIDE
    static float time, beginning, end, duration;

    // Penner Easing Functions Test
    time        = 0;
    beginning   = -guiMB->x_dim;
    end         = (XSIZE_PHYS - guiMB->x_dim) / 2.0;
    duration    = 1000.0; // was 800

    while( time < duration)
    {
        float a = bounce_In( time, beginning, end, duration);
        _HW_layerPosition( guiMB, (int)(a), (YSIZE_PHYS - guiMB->y_dim) / 2);
        UG_Update();
        xtimer_usleep( 1000);
        time++;
    }
#else
    _HW_layerPosition( guiMB, (int)(XSIZE_PHYS - guiMB->x_dim) / 2.0, (YSIZE_PHYS - guiMB->y_dim) / 2);
#endif


    float val = 100; // wait 4 seconds for a click
    int val1 = 0;

    // testing 16-10-16
    UG_WindowShow( FOREGROUND_LAYER, window_7);

    /////////////////////////
    // WAIT FOR __1__ KEY
    if( /*epp_getPlaintext( 1) != EPP_OK*/mb_keyadValue != 0)
    {
        // EPP error???!!!
    }
    int ret;
    char displayString[12];
    //if( msgbox->timeout == 0) msgbox->timeout = 3600;

    while( true /*( ret = epp_waitForPinEnd( displayString)) != EPP_OK*/)
    {
        /*
        if( ret == EPP_USER_CANCELED)
            break;
        */
        // display the displayString

        if( msgbox->timeout != 0)
        {
            int ret = /*epp_waitForPinEnd( displayString)*/mb_keyadValue;
            if( ret == KEY_ENTER || ret == KEY_CANCEL)
                break;
            xtimer_usleep( 100000);
            UG_U8 vbyte = (UG_U8)val;
            UG_ProgressBarSetValue( window_7, TXB_ID_1, vbyte);
            val -= 100.0 / (msgbox->timeout * 10.0);
            UG_Update();
            if( val <= 0 /*|| window5_pressed != 0*/)
                break;
        }
        else
        {   // wait until a key is pressed
            if( mb_keyadValue != 0 /*&& mb_keyadValue != EPP_STILL_SCANNING*/) break;
            xtimer_usleep( 200000);
            UG_ProgressBarSetValue( window_7, TXB_ID_1, 0);
            UG_Update();
        }
    } // endwhile

#ifdef MESSAGEBOX_SLIDE
    // easing out
    time        = 0;
    beginning   = end;
    end         = XSIZE_PHYS;
    duration    = 1000.0; // was 800

    while( time < duration)
    {
        float a = bounce_In( time, beginning, end, duration);
        _HW_layerPosition( guiMB, (int)(a), (YSIZE_PHYS - guiMB->y_dim) / 2);
        xtimer_usleep( 1000);
        time++;
    }
#else
    _HW_layerPosition( guiMB, (int)(XSIZE_PHYS), (YSIZE_PHYS - guiMB->y_dim) / 2);
#endif
    // switch back to previous parameters of this gui and clear the current screen
    //UG_ResetGUI( guiMB, 0, 0, 800, 480, LTDC_Pixelformat_RGB565);
    // 25-10-16 ??? do I need this>??? UG_FillScreen( FOREGROUND_LAYER, C_BLACK);
    // 25-10-16 ??? do I need this>??? UG_WindowHide( FOREGROUND_LAYER, window_7);
/*
    if( mb_prevgui != &fgui && mb_prevgui != &guiSmall)
        mb_prevgui = &fgui;
*/
    //UG_SelectGUI( FOREGROUND_LAYER, mb_prevgui);
    UG_releaseGUI(  FOREGROUND_LAYER, guiMB);

    UG_Update();

    free( window_7);
    free( obj_buff_wnd_7);
    free( progress7_1);
    free( textbox7_1);
    free( button7_1);
    // !@# 2do: fix this issue with all foreground guis taking another video buffer
    //display_free( guiMB->backupVideoBuffer);
    // deletes the display memory allocated for this gui
    UG_deleteGUI( guiMB);

    free( guiMB);
    //LTDC_LayerCmd( LTDC_Layer2, DISABLE);   // TEST

    // notify caller that the messagebox is done with LTDC
    msgbox->mbDone();
    return;
} // end display mb thread



#if 0
static UG_WINDOW *wnd_sdram;
//////////////////////////////////////////
// SELF TESTS AREA....
//////////////////////////////////////////
void window_tests_callback( UG_MESSAGE* msg )
{
   if ( msg->type == MSG_TYPE_OBJECT )
   {
      if ( msg->id == OBJ_TYPE_BUTTON && msg->event == OBJ_EVENT_RELEASED)
      {
         switch( msg->sub_id )
         {
            case BTN_ID_0: /* Toggle green LED */
            {
               UG_WindowHide( FOREGROUND_LAYER, wnd_sdram);
               break;
            }
         } // endswitch
      } // endif
   } // endif

}


void display_TestSDram(void *arg)
{
    int w_width = 400, w_height = 160;
    wnd_sdram = (UG_WINDOW *)malloc( sizeof( UG_WINDOW));
    UG_OBJECT *obj_buff_wnd = (UG_OBJECT *)malloc( sizeof( UG_OBJECT) * MAX_OBJECTS);
    UG_BUTTON *button_cancel = (UG_BUTTON *)malloc( sizeof( UG_BUTTON));
    UG_TEXTBOX *textbox_sdram = (UG_TEXTBOX *)malloc( sizeof( UG_TEXTBOX));
    UG_PROGRESS_BAR *progress_bar = (UG_PROGRESS_BAR *)malloc( sizeof( UG_PROGRESS_BAR));

    UG_WindowCreate( wnd_sdram, obj_buff_wnd, MAX_OBJECTS, window_tests_callback );
    UG_U8 style = UG_WindowGetStyle( wnd_sdram);
    UG_WindowSetStyle( wnd_sdram, style);                // remove the frame around the window also
    UG_WindowSetTitleTextFont( wnd_sdram, &FONT_10X16);
    UG_WindowSetTitleText( wnd_sdram, "SDRAM Test");
    UG_WindowSetBackColor( wnd_sdram, C_WHITE_SMOKE);
    int xs = UG_GetXDim(FOREGROUND_LAYER) / 2 - w_width / 2;
    int ys = UG_GetYDim(FOREGROUND_LAYER) / 2 - w_height / 2;
    int xe = xs + w_width;
    int ye = ys + w_height;
    UG_WindowResize( wnd_sdram, xs, ys, xe, ye);

    int inner_w = UG_WindowGetInnerWidth( wnd_sdram);
    int inner_h = UG_WindowGetInnerHeight( wnd_sdram);

    /* Create Button 1 */
    UG_ButtonCreate( wnd_sdram, button_cancel, BTN_ID_0, inner_w / 2 - 45, inner_h - 42,
                      inner_w / 2 + 45, inner_h - 10);
    UG_ButtonSetFont( wnd_sdram, BTN_ID_0, &FONT_10X16);
    UG_ButtonSetText( wnd_sdram, BTN_ID_0, "Cancel" );
    UG_ButtonSetBackColor( wnd_sdram, BTN_ID_0, TO_UG_COLOR( 0, 93, 93, 93));     // #'s
    UG_ButtonSetForeColor( wnd_sdram, BTN_ID_0, C_WHITE);
    UG_ButtonSetAlternateForeColor( wnd_sdram, BTN_ID_0, C_SILVER);

    UG_ProgressBarCreate( wnd_sdram, progress_bar, TXB_ID_1, inner_w / 2 - 140, 55,
                           inner_w / 2 + 140, 80);
    UG_ProgressBarSetForeColor( wnd_sdram, TXB_ID_1, C_ROYAL_BLUE, C_DARK_GRAY);

    UG_TextboxCreate( wnd_sdram, textbox_sdram, TXB_ID_0, 0, 10, inner_w, 50);
    UG_TextboxSetFont( wnd_sdram, TXB_ID_0, &FONT_16X26);
    UG_TextboxSetText( wnd_sdram, TXB_ID_0, "Test in Progress..." );
    UG_TextboxSetAlignment( wnd_sdram, TXB_ID_0, ALIGN_CENTER);
    UG_TextboxSetForeColor( wnd_sdram, TXB_ID_0, C_BLUE/*TO_UG_COLOR( 0, 149, 199, 248)*/);
    UG_TextboxSetBackColor( wnd_sdram, TXB_ID_0, C_WHITE_SMOKE);

    UG_WindowShow( FOREGROUND_LAYER, wnd_sdram);

    uint32_t add = ( uint32_t)SDRAM_BANK_ADDR;
    uint32_t buffer[4] = { 0xaa55aa55, 0x22662266, 0x99aa99aa, 0xccffccff };
    uint32_t readbuf[4], savebuf[4];
    int fail = 0;
    // to jump over dislp[ay area
    for( int i = 0x0000000; i < SDRAM_SIZE; i += sizeof( buffer))
    {
        SDRAM_ReadBuffer( savebuf, i, 4);
        SDRAM_WriteBuffer(buffer, i, 4);
        SDRAM_ReadBuffer( readbuf, i, 4);
        SDRAM_WriteBuffer(savebuf, i, 4);
        for( uint k = 0; k < sizeof( readbuf) / sizeof( uint32_t); k++)
        {
            if( readbuf[k] != buffer[k])
            {
                fail = 1;
                break;
            }
        }
        if( fail) break;
        UG_ProgressBarSetValue( wnd_sdram, TXB_ID_1, 100 * i / SDRAM_SIZE);
    }
    if( fail == 1)
    {
        UG_TextboxSetForeColor( wnd_sdram, TXB_ID_0, C_RED/*TO_UG_COLOR( 0, 149, 199, 248)*/);
        UG_TextboxSetText( wnd_sdram, TXB_ID_0, "SDRam test FAIL!" );
    }
    else
    {
        UG_TextboxSetForeColor( wnd_sdram, TXB_ID_0, C_DARK_GREEN/*TO_UG_COLOR( 0, 149, 199, 248)*/);
        UG_TextboxSetText( wnd_sdram, TXB_ID_0, "SDRam test OK" );
    }
    UG_ButtonSetText( wnd_sdram, BTN_ID_0, "Exit" );
    xtimer_usleep( 5000000);

    UG_WindowHide( FOREGROUND_LAYER, wnd_sdram);
    UG_Update();
    free( progress_bar);
    free( textbox_sdram);
    free( button_cancel);
    free( obj_buff_wnd);
    free( wnd_sdram);
}


void display_TestPrinter(void *arg)
{
    int w_width = 400, w_height = 160;
    wnd_sdram = (UG_WINDOW *)malloc( sizeof( UG_WINDOW));
    UG_OBJECT *obj_buff_wnd = (UG_OBJECT *)malloc( sizeof( UG_OBJECT) * MAX_OBJECTS);
    UG_BUTTON *button_cancel = (UG_BUTTON *)malloc( sizeof( UG_BUTTON));
    UG_TEXTBOX *textbox_sdram = (UG_TEXTBOX *)malloc( sizeof( UG_TEXTBOX));
    UG_PROGRESS_BAR *progress_bar = (UG_PROGRESS_BAR *)malloc( sizeof( UG_PROGRESS_BAR));

    UG_WindowCreate( wnd_sdram, obj_buff_wnd, MAX_OBJECTS, window_tests_callback );
    UG_U8 style = UG_WindowGetStyle( wnd_sdram);
    UG_WindowSetStyle( wnd_sdram, style);                // remove the frame around the window also
    UG_WindowSetTitleTextFont( wnd_sdram, &FONT_10X16);
    UG_WindowSetTitleText( wnd_sdram, "Printer Test");
    UG_WindowSetBackColor( wnd_sdram, C_WHITE_SMOKE);
    int xs = UG_GetXDim(FOREGROUND_LAYER) / 2 - w_width / 2;
    int ys = UG_GetYDim(FOREGROUND_LAYER) / 2 - w_height / 2;
    int xe = xs + w_width;
    int ye = ys + w_height;
    UG_WindowResize( wnd_sdram, xs, ys, xe, ye);

    int inner_w = UG_WindowGetInnerWidth( wnd_sdram);
    int inner_h = UG_WindowGetInnerHeight( wnd_sdram);

    /* Create Button 1 */
    UG_ButtonCreate( wnd_sdram, button_cancel, BTN_ID_0, inner_w / 2 - 45, inner_h - 42,
                      inner_w / 2 + 45, inner_h - 10);
    UG_ButtonSetFont( wnd_sdram, BTN_ID_0, &FONT_10X16);
    UG_ButtonSetText( wnd_sdram, BTN_ID_0, "Cancel" );
    UG_ButtonSetBackColor( wnd_sdram, BTN_ID_0, TO_UG_COLOR( 0, 93, 93, 93));     // #'s
    UG_ButtonSetForeColor( wnd_sdram, BTN_ID_0, C_WHITE);
    UG_ButtonSetAlternateForeColor( wnd_sdram, BTN_ID_0, C_SILVER);

    UG_ProgressBarCreate( wnd_sdram, progress_bar, TXB_ID_1, inner_w / 2 - 140, 55,
                           inner_w / 2 + 140, 80);
    UG_ProgressBarSetForeColor( wnd_sdram, TXB_ID_1, C_ROYAL_BLUE, C_DARK_GRAY);

    UG_TextboxCreate( wnd_sdram, textbox_sdram, TXB_ID_0, 0, 10, inner_w, 50);
    UG_TextboxSetFont( wnd_sdram, TXB_ID_0, &FONT_16X26);
    UG_TextboxSetText( wnd_sdram, TXB_ID_0, "Test in Progress..." );
    UG_TextboxSetAlignment( wnd_sdram, TXB_ID_0, ALIGN_CENTER);
    UG_TextboxSetForeColor( wnd_sdram, TXB_ID_0, C_BLUE/*TO_UG_COLOR( 0, 149, 199, 248)*/);
    UG_TextboxSetBackColor( wnd_sdram, TXB_ID_0, C_WHITE_SMOKE);

    UG_WindowShow( FOREGROUND_LAYER, wnd_sdram);

    int fail = 0;



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

    Printer_print( "\n\n\n")
    Printer_print( "TEST TELAK TERMINAL\n");
    Printer_print( "===================\n");
    Printer_print( "OS Running on %s board.\n", "Waveshare");
    Printer_print( "This board features %s MCU.\n", "STM32f746");
    Printer_print( "CPU id: %s %s rev %Xp%X\n", cpuname, part, var, pat);
    Printer_print( "Software version %02d.%02d\n", MAJOR_VERSION, MINOR_VERSION)
    Printer_print( "My IP: %d.%d.%d.%d\n", 0, 0, 0, 0);
    Printer_print( "\n\n\n");
    Printer_print( "\n\n\n");
    printer_paperCut();

    UG_TextboxSetForeColor( wnd_sdram, TXB_ID_0, C_DARK_GREEN/*TO_UG_COLOR( 0, 149, 199, 248)*/);
    UG_TextboxSetText( wnd_sdram, TXB_ID_0, "Printer test done" );

    UG_ButtonSetText( wnd_sdram, BTN_ID_0, "Exit" );
    xtimer_usleep( 5000000);

    UG_WindowHide( FOREGROUND_LAYER, wnd_sdram);
    UG_Update();
    free( progress_bar);
    free( textbox_sdram);
    free( button_cancel);
    free( obj_buff_wnd);
    free( wnd_sdram);
} // endfunc

#endif



void display_showConsole( void)
{
    // for the console
    UG_FontSelect( &FONT_12X20);
    UG_ConsoleSetArea( BACKGROUND_LAYER, 20, 40, 780, 480);
    UG_ConsoleSetForecolor( BACKGROUND_LAYER, C_LIGHT_GRAY);
    UG_ConsoleSetBackcolor( BACKGROUND_LAYER, C_DARK_GRAY);
}

/**
 * @brief   append with a CR if you want a new line
 * @param txt
 */
void display_consoleWrite( const char *txt)
{
    UG_ConsolePutString( BACKGROUND_LAYER, txt, &FONT_12X20);
}

/**
 * @brief   append with a CR if you want a new line
 * @param txt
 */
void display_consoleWriteColor( const char *txt, uint32_t forecolor)
{
    UG_COLOR c = UG_ConsoleGetForecolor( BACKGROUND_LAYER);
    UG_ConsoleSetForecolor( BACKGROUND_LAYER, forecolor);
    UG_ConsolePutString( BACKGROUND_LAYER, txt, &FONT_12X20);
    UG_ConsoleSetForecolor( BACKGROUND_LAYER, c);
}




/*********************************************************/
// THIS SHOULD EVENTUALLY BE IN A UTILS/Other MODULE
/*********************************************************/
/**
 * @brief   This function changes the contents of a
 *          ashProtocol type file.
 *          The format of the file is:
 *          [indentifier1=valueA]
 *          [indentifier2=valueB]
 *          [indentifier3=valueC]
 *          [indentifier4=valueD]
 *          ......
 *
 *          This function will look into the file until it find the
 *          requested identifier, change its value and save the file
 *          in a .tmp file and then rename it to the original filename.
 *
 * @param fname
 * @param identifier
 * @param newVal
 * @return  -1 error, 0 OK
 *
 */
int util_ChangeXmlFileValue( const char *fname, const char *identifier, const char *newVal)
{
char *name_id, *strinx, fout[16];
uint statem = 0, iii, stx = 0;

    // allocate 256 bytes for each bulk read
    char *ttext = (char *)malloc( 256);
    char *otext = (char *)malloc( 256);
    FIL *rd = (FIL *)malloc(sizeof(FIL));
    FIL *outrd = (FIL *)malloc(sizeof(FIL));

    FRESULT fr = f_open(rd, fname, FA_READ);
    if( fr != FR_OK)
    {
        free( rd);
        free( outrd);
        free( ttext);
        free( otext);
        return -1;
    }
    replaceExtension( fout, fname, "tmp");
    fr = f_open(outrd, fout, FA_CREATE_ALWAYS | FA_WRITE);
    if( fr != FR_OK)
    {
        free( rd);
        free( outrd);
        free( ttext);
        free( otext);
        return -1;
    }
    int newkey = 0;

    ///////////////////////////////////////////
    // read line by line...
    ///////////////////////////////////////////
    while( f_gets( ttext, 256, rd) != 0)
    {
        char *text = ttext;
        uint length = strlen( text);
        for( uint i = 0; i < length; i++)
        {
            //! look carefully for the beginning of the xml tag
            if( statem == 0 && text[i] == '<')
            {
                stx = i + 1;
                statem++;
            }
            // start of a master key tag????
            if( statem == 0 && text[i] == '[')
            {
                newkey = 1;
                stx = i+1;
                statem++;
            }
            if( statem == 1 && text[i] == '=' && newkey == 1)
            {
                iii = i + 1;
                name_id = &text[stx];
                text[i] = (char)NULL;
                statem++;
            }
            if( statem == 1 && text[i] == ']' && newkey == 1)
            {
                iii = i + 1;
                name_id = &text[stx];
                text[i] = (char)NULL;
                strinx = "-";
                newkey = 2;
            }
            if( statem == 2 && text[i] == ']' && newkey == 1)   // end of MASTER KEY definition
            {
                newkey = 2;
                strinx = &text[iii];
                text[i] = (char)NULL;
                statem = 0;
            }
            if( newkey == 2)
            {
                // if same as identifier...
                if( !strcmp( name_id, identifier))
                {
                    sprintf( otext, "[%s=%s]\r\n", identifier, newVal);
                    f_puts( otext, outrd);
                }
                else
                {
                    sprintf( ttext, "[%s=%s]\r\n", name_id, strinx);
                    f_puts( ttext, outrd);
                }
                break;
            }
            if( statem == 1 && text[i] == '>')   // end of key definition
            {
                if( text[i-1] == '/')            // empty definition??? -- drop!
                {
                    newkey = 0;
                    statem = 0;
                    break;
                }
                strinx = &text[i+1];
                text[i] = (char)NULL;
                name_id = &text[stx];
                statem++;
            }
            if( statem == 2 && text[i] == '<')   // end of key definition
            {
                text[i] = (char)NULL;
                name_id = &text[stx];
                statem = 0;

                // insert new key-value pair
                // if same as identifier...
                if( !strcmp( name_id, identifier))
                {
                    sprintf( ttext, "<%s>%s</%s>\r\n", identifier, newVal, identifier);
                    f_puts( otext, outrd);
                }
                else
                {
                    sprintf( ttext, "<%s>%s</%s>\r\n", name_id, strinx, name_id);
                    f_puts( ttext, outrd);
                }
                break;
            }

        } // endfor i
    } // endwhile all string lines in file

    f_close( rd);
    f_close( outrd);
    free( rd);
    free( outrd);
    free( ttext);
    free( otext);

    // now rename and delete the .tmp file
    f_unlink( fname);
    fr = f_rename( fout, fname);
    f_unlink( fout);

    return 0;
} // endof func












/**
 * @notes
 *
 * From tests I've done, it seems that for running with two full layers
 * we need at least 200MHz CPU clock and up to about 290 in the SAI pll
 * to prevent (major) flickering with the above DMA0 function.
 * Unfortunately, flickering cannot be avoided when using DMA2D
 */
