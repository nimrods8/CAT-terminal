/***********************************************************************
 * @ingroup     ashrait emv protocol implementation
 * @file        ashProtocol.cpp
 * @brief       implementation of ASHRAIT EMV v?????
 *
 ************************************************************************/
#include <stdio.h>
#include <string.h>
#include <malloc.h>

#include "ff.h"
#include "../../app/display.h"
#include "ashProtocol.hpp"


//------------- Some static Definitions -------------------------

/**
 * @here come the STATIC definitions
 */
#ifdef STATIC_ASH
binarySearchST **ashProtocol::ashEntries;
int ashProtocol::N, ashProtocol::capacity;
#endif
#define INIT_CAPACITY       2



/**
 * @brief   CREATE THE AshProtocol module
 *
 *          Call ::init to add new xml file
 */
void ashProtocol::create( void)
{
    capacity = INIT_CAPACITY;
    ashEntries = new binarySearchST *[capacity];
    N = 0;
}

/**
 * @brief   This function searches for a given value in a set of symbol tables
 *          previous loaded to this module from files...
 *
 * @param key       char string search for key
 * @param value     char string value to look for in key (may be NULL)
 * @return index of found symbol table in ashEntries, or ASH_PROTOCOL_ERROR
 */
int ashProtocol::indexByKey( const char *key, const char *value)
{
    Value *v1 = NULL;

    if( value != NULL)
        v1 = new Value( value, strlen( value));

    Value *k = new Value( key, strlen( key));
    for( int i = 0; i < N; i++)
    {
        Value *v2 = ashEntries[i]->get( k);
        if( v2 != NULL && (v1 == NULL || ashEntries[i]->comparer( v2, v1) == 0))
        {
            delete k;                           // release the memory used by Value class
            if( v1 != NULL)
                delete v1;
            return i;
        }
    } // endfor all loaded entries
    delete k;                           // release the memory used by Value class
    if( v1 != NULL) delete v1;
    return ASH_PROTOCOL_ERROR;
} // endfunc


/**
 * @brief   returns a parameter from the terminal xml-like file downloaded
 *          from ashrait emv (PinPad data xml).
 *
 * @param index - index of ashEntries[]
 * @param key   - requested key to return
 * @param ppVal - pointer to pointer of const char
 *
 * @return  ASH_PROTOCOL_ERROR if error, ASH_PROTOCOL_OK if OK
 */
int ashProtocol::getParam( uint index, const char *key, const char **ppVal)
{
    if( index >= (uint)N) return ASH_PROTOCOL_ERROR;

    Value *k = new Value( key, strlen(key));
    *ppVal = ashEntries[index]->get( k)->toString();
    delete k;

    return ASH_PROTOCOL_OK;
} // endfunc



//----- resize the underlying arrays with new capacity -----
void ashProtocol::resize(int _capacity) {
    if( _capacity < N) return;

    binarySearchST **tempst = new binarySearchST*[_capacity];
/*
    for (int i = 0; i < _capacity; i++) {
        if( i < capacity)
            tempst[i] = ashEntries[i];
        else
            tempst[i] = new binarySearchST();
*/
    for( int i = 0; i < N; i++) {
        tempst[i] = ashEntries[i];
    }
    delete[] ashEntries;

    // can't do that because it erases the Symbol Tables...... >>> delete ashEntries;
    ashEntries = tempst;
    capacity = _capacity;
}




/**
 * @brief reads terminal file sent from ashrait (through CAT superviser) and
 *        stores it in different ashEntries, loosely corresponding to XML
 *        structures
 *
 * @param ashTerminalFilename
 *
 *
 * XML-like file structure:
 *
 * [mkey1=string1]
 * <aaa>bbb</aaa>
 * <ccc>ddd</ccc>
 * <eee>1234</eee>
 *
 * [mkey2=string2]
 * <www>eee</www>
 * <rrr>ttt</rrr>
 * <kkk>ppp</kkk>
 *
 * [merchant]
 * <nnn>mmm</nnn>
 * ....
 *
 * ....
 *
 * returns      AS
 *
 */
int ashProtocol::init( const char *fname)
{
char *name_id, *strinx;
uint statem = 0, iii, stx = 0;
/*
    ashEntries = new binarySearchST[INIT_CAPACITY];
    capacity = INIT_CAPACITY;
    N = 0;
*/
    // allocate 1024 bytes for each bulk read
    // based on the assumption that 1024 is the maximum number of bytes
    // per one public key entry.....
    char *ttext = (char *)malloc( 256);
    FIL *rd = (FIL *)malloc(sizeof(FIL));


    FRESULT fr = f_open(rd, fname, FA_READ);
    if( fr != FR_OK)
    {
        free( rd);
        free( ttext);
        return ASH_PROTOCOL_ERROR;
    }
#if 0
    capacity = INIT_CAPACITY;
    ashEntries = new binarySearchST *[capacity];
#endif
    /*
    for( int i = 0; i < capacity; i++)
        ashEntries[i] = new binarySearchST();
    */
    //N = 0;
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
                // insert new key-value pair
                newkey = 0;
                if (N == capacity) resize( 2 * capacity);
                N++;
                ashEntries[N-1] = new binarySearchST();
                ashEntries[N-1]->put( name_id, strinx);
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
                ashEntries[N-1]->put( name_id, strinx);
                break;
            }

        } // endfor i
    } // endwhile all string lines in file

#ifdef STATIC_ASH // debug
    binarySearchST **debugst = ashEntries;
#endif
    int debug_N = N;
    int debug_cap = capacity;
    int inx = indexByKey( "MERCHANT", NULL);

    f_close(rd);
    free( rd);
    free( ttext);
    return ASH_PROTOCOL_OK;
} // endof constructor


void ashProtocol::purgeObject( void)
{
    for( int i = 0; i < /*capacity*/ N; i++)
        delete ashEntries[i];
    delete[] ashEntries;
    N = 0;
    capacity = 0;
}

#if 0
void ashProtocol::init(const char *ashTerminalFilename)
{
    ashEntries = new binarySearchST[INIT_CAPACITY];
    capacity = INIT_CAPACITY;
    N = 0;
    /*
    for( int i = 0; i < capacity; i++)
        ashEntries[i] = new binarySearchST();
    */
    // here, we read the terminal files sent from ashrait and store them in
    // the separate symbol tables


    ashEntries[N].put( "Threshold", "100");
    ashEntries[N].put( "AID", "A0003456030");
    ashEntries[N].put( "CardType", "CREDIT");
    N++;

    ashEntries[N].put( "Threshold", "200");
    ashEntries[N].put( "AID", "A0002345561");
    ashEntries[N].put( "CardType", "DEBIT");
    N++;
}
#endif
