/**
 * @brief	riff parser module is designed to analyze and parse an
 * 		AVI file based on bitmaps
 * 		Currently no sound is analyzed
 * 		There is only one function exposed to the application.
 * 		This function builds an index of file seek positions
 * 		for each bitmap on the AVI file. The index is built into
 * 		vInxTable
 */
#include "stdint.h"
#include "ff.h"
#include "aviriff.h"
#include "PROGBAR.h"

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief	This is the main entry point for the .AVI
 * 		RIFF parser.
 *
 * @param	[IN] videoIndexTable is the file position of each
 * 		image to be displayed
 * 		[IN] rd - the opened file handle
 *
 * @result	int
 */
int riffParser_Go( FIL *rd, DWORD **vInxTable, PROGBAR_Handle hProgBar, int breakBeforeIndex);

/**
 * @brief releases the memory used for image index.
 * must be called after using the parser.
 * @param inx [in] pointer to the malloc'ed memory
 * @return always 0
 */
int riffParser_Release( DWORD *inx);

/**
 * @brief returns the avi stream header structure pointer of the requested stream number
 * @param num is the stream number
 * @return a pointer to AVISTREAMHEADER structure
 */
AVISTREAMHEADER *riffParser_GetStreamHeader( int num);

#ifdef __cplusplus
}
#endif
