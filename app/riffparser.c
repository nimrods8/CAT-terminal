/**
 * @brief	This is the riff parser handling AVI files
 *		The module reads the AVI file and sets
 *		and index for all the images.
 *		Currently we are using only bitmaps, but other type of images
 *		should be handled the same.
 *
 * @note	sound handling is currently missing
 *
 */


/****************************************/
/* 	        INCLUDES 		*/
/****************************************/
#include "malloc.h"
#include "stdlib.h"
#include "string.h"
#include "ff.h"
#include "aviriff.h"
#include "progbar.h"
#include "hwtimer.h"

/****************************************/
/* 	   PRIVATE FUNCTIONS		*/
/****************************************/
static BYTE ReadElement( FIL *rd, UINT *bytesleft);
static void invokeChunk( FIL *rd, int FourCC, UINT length, UINT paddedLength);
static void invokeList( FIL *rd, int FourCC, UINT length);
static void FromFourCC( int FourCC, char *chars);
static int ReadData( FIL *rd, BYTE *data, UINT offset, UINT length);
static void SkipData( FIL *rd, UINT skipBytes);
static int ReadOneInt( FIL *rd, int *FourCC);
static int ReadTwoInts( FIL *rd, int *FourCC, UINT *size);


#define FCC_FROM_FILE(fcc)       ( ((fcc & 0x000000FF) << 24) | ((fcc & 0x0000FF00) << 8) | ((fcc & 0x00FF0000) >> 8) | ((fcc & 0xFF000000) >> 24))

/****************************************/
/* 	   PRIVATE   DEFINITIONS	*/
/****************************************/
#define DEBUG_AVI

#define __FourCC (x)       ( (int)x[0] << 24 | (int)x[1] << 16 | (int)x[2] << 8 | (int)x[3])
#define byte    unsigned char
#define FOURCC_STRING_LENGTH	(4+1)

#define DWORDSIZE 4
#define TWODWORDSSIZE 8
#define RIFF4CC "RIFF"
#define RIFX4CC "RIFX"
#define LIST4CC "LIST"

/****************************************/
/* 	   PRIVATE   TYPES		*/
/****************************************/
AVIMAINHEADER aviMain;
AVISTREAMHEADER aviStream[2];
AVIOLDINDEX aviOldInx;
BITMAPINFOHEADER bmpHeader;
WAVEFORMATEX wavHeader;
AVISUPERINDEX superIndex;
AVIFIELDINDEX fieldIndex;

DWORD *audioIndexTable, *videoIndexTable;       //< pointer to index table which includes all frames of this movie
int videoStreamNumber, audioStreamNumber;       //< -1 if no stream present
int streamNum, vinx, ainx, headerinx;
int frame, lastprogbar;
int offset_of_movi_data_block;
int bailout, breakBeforeIndex;
PROGBAR_Handle hProgBar;


/**
 * @brief returns the avi stream header structure pointer of the requested stream number
 * @param num is the stream number
 * @return a pointer to AVISTREAMHEADER structure
 * @note   aviStream[num].dwLength / aviStream[num].dwScale is the number of video frames on file
 */
AVISTREAMHEADER *riffParser_GetStreamHeader( int num)
{
   return &aviStream[num];
}

/**
 * @brief releases the memory used for image index.
 * must be called after using the parser.
 * @param inx [in] pointer to the malloc'ed memory
 * @return always 0
 */
int riffParser_Release( DWORD *inx)
{
  free( inx);
  return 0;
}


/**
 * @brief	This is the main entry point for the .AVI
 * 		RIFF parser.
 *
 * @param [IN]  rd - the opened file handle
 *        [IN]  videoIndexTable is the file position of each
 *              image to be displayed
 *        [IN]  ahProgBar - handle to the progress bar
 *        [IN]  breakIndex - if 1 breaks out right after reading the AVI stream
 *              header. Use riffParser_GetStreamHeader() to get the pointer
 *              to that structure.
 *              NOTE: In this case the function returns NULL
 *
 * @result	int - how many video frames have been found and indexed
 *                -1 if error occured
 */
int riffParser_Go( FIL *rd, DWORD **vInxTable, PROGBAR_Handle ahProgBar, int breakIndex)
{
int FourCC, datasize, fileType;

    bailout = 0;
    breakBeforeIndex = breakIndex;
    headerinx = 0;

// just debugging
    offset_of_movi_data_block = frame = lastprogbar = 0;
    hProgBar = ahProgBar;

    // continue to read chunks of RIFF until you finish reading all the file
    while( 1)
    {
        // if finished reading ENTIRE file just break out of this loop
        if( rd->fptr == rd->fsize)
            break;

        if( !ReadTwoInts( rd, &FourCC, &datasize))
        {
            return -1;  // error!
        }

        if( !ReadOneInt( rd, &fileType))
            return -1;   // error!

        // Check for a valid RIFF header
        //int m_fileriff = FourCC;
        //int m_filetype = fileType;
        UINT m_datasize;
        UINT m_filesize = rd->fsize;

        char riff[FOURCC_STRING_LENGTH];

        // Check for a valid RIFF header
        FromFourCC( FourCC, riff);
        if( !strcmp( riff, RIFF4CC) || !strcmp(riff, RIFX4CC))
        {
            // Good header. Check size
            m_datasize = datasize;
            if (m_filesize >= m_datasize + TWODWORDSSIZE)
            {
                //Console.WriteLine(ShortName + " has a valid size");
				    // If we got here - the file is valid. Output information about the file
                //FromFourCC( m_fileriff, riff);
                //char ftype[FOURCC_STRING_LENGTH];
                //FromFourCC( m_fileriff, ftype);

                //printf("File is a \"" + riff + "\" with a specific type of \"" + ftype + "\"");

                    // Store the size to loop on the elements
                UINT size = m_datasize;

                // Read all top level elements and chunks
                while (size > 0)
                {
                   // Prefix the line with the current top level type
                   //printf( FromFourCC(m_filetype) + " (" + size.ToString() + "): ");
                   // Get the next element (if there is one)
                   if ( !ReadElement( rd, &size)) break;
                   if( bailout)
                      return NULL;
                }
                printf("\n\rDone.");
            }
            else
            {
            }
        } // endif have RIFF block
        else break;
    } // endwhile forever
    printf(L"Done with AVI File. Don't forget to close it");


    *vInxTable = videoIndexTable;			//< return the video index table pointer
    return vinx;
} // end of riff parser




/// <summary>
/// Read the next RIFF element invoking the correct delegate.
/// Returns true if an element can be read
/// </summary>
/// <param name="bytesleft">Reference to number of bytes left in the current list</param>
/// <param name="chunk">Method to invoke if a chunk is found</param>
/// <param name="list">Method to invoke if a list is found</param>
/// <returns>true if the next element can be read, 0 in case of error</returns>
static BYTE ReadElement( FIL *rd, UINT *bytesleft)
{
    // Are we done?
    if( TWODWORDSSIZE > *bytesleft)
        return 0;

    // We have enough bytes, read
    int FourCC;
    UINT size;

    if( !ReadTwoInts( rd, &FourCC, &size))
        return 0;

    // Reduce bytes left
    *bytesleft -= TWODWORDSSIZE;

    // Do we have enough bytes?
    if( *bytesleft < size)
    {
        // Skip the bad data and throw an exception
        SkipData( rd, *bytesleft);
        *bytesleft = 0;
    }

    // Examine the element, is it a list or a chunk
    char type[FOURCC_STRING_LENGTH];
    FromFourCC( FourCC, type);
    if( !strcmp( type, LIST4CC))
    {
        // We have a list
        if( !ReadOneInt( rd, &FourCC))
            return 0;

        invokeList( rd, FourCC, size - 4);

        // Adjust size
        *bytesleft -= size;
    }
    else
    {
        // Calculated padded size - padded to WORD boundary
        int paddedSize = size;
        if (0 != (size & 1)) ++paddedSize;

        invokeChunk( rd, FourCC, size, paddedSize);
        // Adjust size
        *bytesleft -= paddedSize;
    }

    return 1;
} // endfunc ReadElement








/**
 * <summary>
 * Non-thread-safe method to read two ints from the stream
 * </summary>
 * <param name="FourCC">Output FourCC int</param>
 * <param name="size">Output chunk/list size</param>
 *
 * @return  0 - if error, 1 if read OK
 *
 */
static int ReadTwoInts( FIL *rd, int *FourCC, UINT *size)
{
    *FourCC = 0;
    *size = 0;

    BYTE m_eightBytes[TWODWORDSSIZE];
    UINT readsize;
    FRESULT frs = f_read( rd, m_eightBytes, TWODWORDSSIZE, &readsize);

    if( frs != FR_OK || TWODWORDSSIZE != readsize)
       return 0;

    *FourCC = ((int)m_eightBytes[0] << 24) | ((int)m_eightBytes[1] << 16)
                    | ((int)m_eightBytes[2] << 8) | ((int)m_eightBytes[3]);
    *size = ((int)m_eightBytes[7] << 24) | ((int)m_eightBytes[6] << 16)
                    | ((int)m_eightBytes[5] << 8) | ((int)m_eightBytes[4]);

    return 1;
}

/**
/// <summary>
/// Non-thread-safe read a single int from the stream
 * and format it as a FourCC
/// </summary>
/// <param name="FourCC">Output int</param>
 *
 * @return  if error - 0, else if OK returns 1
 *
 */
static int ReadOneInt( FIL *rd, int *FourCC)
{
    BYTE m_fourBytes[DWORDSIZE];
    UINT readsize;
    FRESULT frs = f_read( rd, m_fourBytes, DWORDSIZE, &readsize);

    if( frs != FR_OK || DWORDSIZE != readsize)
       return 0;

    *FourCC = ((int)m_fourBytes[0] << 24) | ((int)m_fourBytes[1] << 16)
                    | ((int)m_fourBytes[2] << 8) | ((int)m_fourBytes[3]);

    return 1;
}

/// <summary>
/// Skip the specified number of bytes
/// </summary>
/// <param name="skipBytes">Number of bytes to skip</param>
static void SkipData( FIL *rd, UINT skipBytes)
{
  // should have been a function to do seek from current position
  // for now I am trying with this solution. See if it works
  f_lseek( rd, f_tell( rd) + skipBytes);
}

/// <summary>
/// Read the specified length into the byte array at the specified
/// offset in the array
/// </summary>
/// <param name="data">Array of bytes to read into</param>
/// <param name="offset">Offset in the array to start from</param>
/// <param name="length">Number of bytes to read</param>
/// <returns>Number of bytes actually read</returns>
static int ReadData( FIL *rd, BYTE *data, UINT offset, UINT length)
{
    UINT read;
    FRESULT frs = f_read( rd, &data[offset], length, &read);
    if( frs != FR_OK || read != (UINT)length)
      return 0;
    else
      return read;
}




/**
 * @brief convert integer FourCC into string
 * @param FourCC
 * @param chars
 */
static void FromFourCC( int FourCC, char *chars)
{
    chars[3] = (char)(FourCC & 0xFF);
    chars[2] = (char)((FourCC >> 8) & 0xFF);
    chars[1] = (char)((FourCC >> 16) & 0xFF);
    chars[0] = (char)((FourCC >> 24) & 0xFF);
    chars[4] = 0;

    return;
}

/**
 * @brief converts 4 characters string to FourCC format
 * @param charFourCC is the string representing the FourCC
 * @return FourCC format
 */
static int ToFourCC( char *charFourCC)
{
  if( charFourCC[4] != 0)
     return 0;
    int result = ((int)charFourCC[3]) << 24
                | ((int)charFourCC[2]) << 16
                | ((int)charFourCC[1]) << 8
                | ((int)charFourCC[0]);

    return result;
}


/**
 * @brief converts 4 characters string to FourCC format
 * @param charFourCC is the string representing the FourCC
 * @return FourCC format
 */
static int ToFourCC2(char c0, char c1, char c2, char c3)
{
    int result = ((int)c3) << 24
                | ((int)c2) << 16
                | ((int)c1) << 8
                | ((int)c0);

    return result;
}


/**
 * @brief Process a RIFF list element (list sub elements)
 * @param rd 		File pointer (FIL *)
 * @param FourCC	list's FourCC designator
 * @param length	length of list
 */
static void invokeList( FIL *rd, int FourCC, UINT length)
{
  char type[FOURCC_STRING_LENGTH];

#ifdef DEBUG_AVI
  FromFourCC( FourCC, type);
#endif

  // Define the processing delegates
  //RiffParser.ProcessChunkElement pc = new RiffParser.ProcessChunkElement(ProcessChunk);
  //RiffParser.ProcessListElement pl = new RiffParser.ProcessListElement(ProcessList);

  // Read all the elements in the current list
  while (length > 0) {
			// Prefix each line with the type of the current list
      if (!ReadElement( rd, &length)) break;
  } // endwhile
} // endfunc invokeList


/**
 * @brief Process a RIFF chunk element (skip the data)
 * @param rd
 * @param FourCC
 * @param length
 * @param paddedLength
 */
static void invokeChunk( FIL *rd, int FourCC, UINT length, UINT paddedLength)
{
    char type[FOURCC_STRING_LENGTH];

#ifdef DEBUG_AVI
    FromFourCC( FourCC, type);
#endif

    AVIMAINHEADER *avimain;
    AVISTREAMHEADER *avistream;
    //AVIOLDINDEX *avioldinx;

    /**********************************/
    /* M A I N   A V I   H E A D E R  */
    /**********************************/
    if( FourCC == ckidMAINAVIHEADER)
    {
        BYTE buff[sizeof( aviMain)];
        UINT red;
        if( f_read( rd, buff, length, &red) != FR_OK)
          return;

        avimain = (AVIMAINHEADER *)&buff[0];
        avimain->fcc = ckidMAINAVIHEADER;
        avimain->cb = length;
        memcpy( &aviMain, avimain, sizeof( aviMain)); 

        if( aviMain.dwFlags & AVIF_HASINDEX) // Index at end of file?
           printf( "This avi has index at end of file\n");
        if( aviMain.dwFlags & AVIF_MUSTUSEINDEX) // must use Index at end of file?
           printf( "This avi must use the index at end of file\n");
        if( aviMain.dwFlags & AVIF_ISINTERLEAVED) 
           printf( "This avi is interleaved\n");
        if( aviMain.dwFlags & AVIF_TRUSTCKTYPE) // Use CKType to find key frames
           printf( "Use CKType to find key frames\n");
        if( aviMain.dwFlags & AVIF_WASCAPTUREFILE) 
           printf( "This avi originates from a capture device\n");
        if( aviMain.dwFlags & AVIF_COPYRIGHTED) 
           printf( "This avi is copyrighted!\n");

        //< reset stream numbers and other parameters used in the future
        videoStreamNumber = audioStreamNumber = -1;
        streamNum = 0;
        vinx = ainx = 0;
    }
    /**********************************/
    /*    S T R E A M   H E A D E R   */
    /**********************************/
    else if( FourCC == ckidSTREAMHEADER)
    {
        BYTE buff[sizeof( aviStream)];
        UINT red;
        if( f_read( rd, buff, length, &red) != FR_OK)
          return;

        avistream = (AVISTREAMHEADER *)&buff[0];
        memcpy( &aviStream[headerinx], avistream, sizeof( aviStream)); 
        aviStream[headerinx].offset_of_first_bmp = 0;

        /** MALLOC
         * @brief allocate enough room to hold dwLength / dwScale frame indexes in RAM memory
         */
        // I know I read it from the file and directly copied it to the structure
        // and because of Intel's funny endian, I need to reverse it...
        aviStream[headerinx].fccType = FCC_FROM_FILE( aviStream[headerinx].fccType);


        if( breakBeforeIndex)
        {
           bailout = 1;
           return;
        }


        if( aviStream[headerinx].fccType == streamtypeVIDEO)
           videoIndexTable = malloc( sizeof( DWORD) * (int)aviStream[headerinx].dwLength / (int)aviStream[headerinx].dwScale);
    }
#if 0
    /**********************************/
    /*     S U P E R    I N D E X     */
    /**********************************/
    else if( FourCC == ckidAVISUPERINDEX)
    {
        AVISUPERINDEX *superinx;
        BYTE buff[sizeof( AVISUPERINDEX)];
        UINT red;
        if( f_read( rd, buff, sizeof( AVISUPERINDEX), &red) != FR_OK)
          return;

        superinx = (AVISUPERINDEX *)&buff[0];
        memcpy( &superIndex, superinx, sizeof( superIndex));
    }
#endif
    /**********************************/
    /*       N E W   I N D E X        */
    /**********************************/
    else if( type[0] == 'i' && type[1] == 'x')
    {
        AVIFIELDINDEX *finx;
        BYTE buff[sizeof( AVIFIELDINDEX)];
        UINT red;
        if( f_read( rd, buff, sizeof( AVIFIELDINDEX), &red) != FR_OK)
          return;

        // make sure the offset is not calculated from the beginning of the movi list
        finx = (AVIFIELDINDEX *)&buff[0];
        memcpy( &fieldIndex, finx, sizeof( fieldIndex)); 

        DWORD aq = fieldIndex.dwChunkId;
        aq = ((aq & 0x000000FF) << 24 | (aq & 0x0000FF00) << 8 | (aq & 0x00FF0000) >> 8 | (aq & 0xFF000000) >> 24);

        char chunkStr[FOURCC_STRING_LENGTH];
        FromFourCC( aq, chunkStr);
        chunkStr[2] = 0;
        int num = atoi( chunkStr);
        FromFourCC( aq, chunkStr);
        _aIndex *field_inx;
/*
        if( chunkStr[2] == 'd' && chunkStr[3] == 'c' && videoStreamNumber == num)
        {
           videoIndexTable[vinx++] = (DWORD)fieldIndex.aIndex[0].dwOffset + fieldIndex.qwBaseOffsetL;
        } // this is a video frame index
*/
        int entries = fieldIndex.nEntriesInUse;
        for( UINT iii = 0; entries > 0; iii += sizeof( DWORD) * fieldIndex.wLongsPerEntry, entries--)
        {
            if( f_read( rd, buff, sizeof( DWORD) * fieldIndex.wLongsPerEntry, &red) != FR_OK)
               return;
            field_inx = (_aIndex *)buff;

            if( chunkStr[2] == 'd' && chunkStr[3] == 'c' && videoStreamNumber == num)
            {   
               videoIndexTable[vinx++] = (DWORD)field_inx->dwOffset + fieldIndex.qwBaseOffsetL;
            } // this is a video frame index
        } // endfor for each one index entry
        if( vinx != aviMain.dwTotalFrames - 1)
           printf( "Error - avimain total frames mismatch!\n");
    }

    /**********************************/
    /*       O L D   I N D E X        */
    /**********************************/
    /*! this code should run only if have no new index around the file */
    else if( FourCC == ckidAVIOLDINDEX && vinx == 0) // idx1
    {
        BYTE buff[sizeof( AVIOLDINDEX)];
        UINT red, num;
        char chunkStr[FOURCC_STRING_LENGTH];

        if( f_read( rd, buff, sizeof( AVIOLDINDEX), &red) != FR_OK)
          return;

        AVIOLDINDEX *avioldinx = (AVIOLDINDEX *)buff;
        int off_from_movi_list = 0;
        if( avioldinx->aIndex->dwOffset < sizeof( aviMain))
           off_from_movi_list = 1;

        for( UINT iii = 0 /*sizeof( AVIOLDINDEX)*/; iii < length; iii += sizeof( AVIOLDINDEX))
        {
            avioldinx = (AVIOLDINDEX*)buff;
            DWORD aq = avioldinx->aIndex->dwChunkId;
            aq = ((aq & 0x000000FF) << 24 | (aq & 0x0000FF00) << 8 | (aq & 0x00FF0000) >> 8 | (aq & 0xFF000000) >> 24);
            FromFourCC( aq, chunkStr);
            chunkStr[2] = 0;
            num = atoi( chunkStr);
            FromFourCC( aq, chunkStr);

            //! take only movie frames with data
            if( chunkStr[2] == 'd' && chunkStr[3] == 'c' && videoStreamNumber == num)
            {
                //! This could be an inserted frame with zero size
                //! Just point to the previous frame
                if( avioldinx->aIndex->dwSize == 0)
                {
                   printf( "Found zero size frame at %d\n", vinx);
                   if( vinx > 0)
                   {
                      videoIndexTable[vinx] = videoIndexTable[vinx - 1];
                   }
                   else
                   {
                      videoIndexTable[vinx] = 0;
                   }
                   vinx++;
                }
                else
                {
                   //! Note: offset_of_movi_data_block is the offset of the beginning AVI's RIFF movi chunk
                   //!       to this we add the offset from the idx1. This will point to the first byte of the
                   //!	 '00dc' string of the frame. We are adding an additional 8 bytes to point beyond the
                   //!	 '00dc' + length which are total 8 bytes. This will point to the signature of the
                   //!	 frame itself.
                   if( off_from_movi_list)
                      videoIndexTable[vinx++] = (DWORD)avioldinx->aIndex->dwOffset + offset_of_movi_data_block + 8;
                   else
                      videoIndexTable[vinx++] = (DWORD)avioldinx->aIndex->dwOffset;
                }
            } // this is a video frame index
            else if( chunkStr[2] == 'w' && chunkStr[3] == 'b' && audioStreamNumber == num)
            {
                    // 2 D O  !!!
            }
            if( f_read( rd, buff, sizeof( AVIOLDINDEX), &red) != FR_OK)
              return;

        } // endfor for each one index entry
        if( vinx != aviMain.dwTotalFrames)
           printf( "Error - avimain total frames mismatch!\n");
        else
           printf( "Old Index Read OK. Frames #%d\n", vinx);
    }


    /**********************************/
    /*    S T R E A M   F O R M A T   */
    /**********************************/
    else if( FourCC == ckidSTREAMFORMAT)
    {
        BYTE buff[length];
        UINT red;
        if( f_read( rd, buff, length, &red) != FR_OK)
          return;

        streamNum++;

        /*******************************/
        /* B I T M A P    S T R E A M  */
        /*******************************/
        if( aviStream[headerinx].fccType == streamtypeVIDEO)
        { // pickup video stream, allocate index table room for ALL frames on avi
            if( videoStreamNumber == -1)
               videoStreamNumber = streamNum - 1;
            else printf( "too many video streams on file. Dropping!\n");

            BITMAPINFOHEADER *bmp = (BITMAPINFOHEADER *)buff;
            memcpy( &bmpHeader, bmp, sizeof( bmpHeader)); 
        }
        else 
        /*******************************/
        /*    W A V     S T R E A M    */
        /*******************************/
        { // pickup audio stream
            if( audioStreamNumber == -1)
               audioStreamNumber = streamNum - 1;
            else printf( "too many audio streams on file. Dropping all in excess!\n");

            WAVEFORMATEX *wav = (WAVEFORMATEX *)buff;
            memcpy( &wavHeader, wav, sizeof( wavHeader)); 
        }
        headerinx++;
    }
    /**********************************/
    /*    V I D E O    F R A M E      */
    /**********************************/
    else if( type[2] == 'd' && type[3] == 'c')
    {   // if don't have an index, pick each frame's offset from xxdc (actual bitmaps) position in file
	char gtype[FOURCC_STRING_LENGTH - 2];
	memcpy( gtype, &type[0], sizeof( gtype));
	gtype[2] = 0;
	int streamn = atoi( gtype);
	if( streamn == videoStreamNumber)
	{
	    if( !( aviMain.dwFlags & AVIF_HASINDEX || aviMain.dwFlags & AVIF_MUSTUSEINDEX))
	    {
	       videoIndexTable[vinx++] = (DWORD)rd->fptr;
	    }
	    // right now do nothing, just advance pointer to next block
	    if( offset_of_movi_data_block == 0)
	       offset_of_movi_data_block = (DWORD)f_tell( rd) - 12;	//!< to bring pointer to beginning of movi chunk (see avi pdf);
	    SkipData( rd, paddedLength);
	}
	frame++;

	int prg = 100 * frame / ((int)aviStream[streamn].dwLength / (int)aviStream[streamn].dwScale);
	if( prg != lastprogbar)
	{
	    char str[35];
	    sprintf( str, "Progress Frames %d %%", prg);
	    //PROGBAR_SetValue( hProgBar, prg);
	    //>> NS 03-08-16     GUI_DispStringAt( str, 50, 80);

	    lastprogbar = prg;
	}
    } // endif video frame
    /**********************************/
    /*    A U D I O    F R A M E      */
    /**********************************/
    else if( type[2] == 'w' && type[3] == 'b')
    {
	char gtype[FOURCC_STRING_LENGTH - 2];
	memcpy( gtype, &type[0], sizeof( gtype));
	gtype[2] = 0;
	int streamn = atoi( gtype);

	if( streamn == audioStreamNumber)
	{
	    if( offset_of_movi_data_block == 0)
	       offset_of_movi_data_block = (DWORD)f_tell( rd) - 12;	//!< to bring pointer to beginning of movi chunk (see avi pdf)
	    SkipData( rd, paddedLength);
	}
    }
    else 
        SkipData( rd, paddedLength);

    // Skip data and update bytesleft
    // now I am reading this data---- so this is a bug......  SkipData( rd, paddedLength);
} // endfunc invokeChunk




/**
 * END OF MODULE
 */
