/***********************************************************************
 * @ingroup     ashrait emv protocol implementation
 * @file        ashProtocol.cpp
 * @brief       implementation of ASHRAIT EMV v?????
 *
 *              This module reads the xml file generated by Ashrait
 *              and stores it for emvCard's use inside a binarySearchTree
 *
 ************************************************************************/

#ifndef __ASH_PROTOCOL
#define __ASH_PROTOCOL

#include "binarySearchST.hpp"


#define STATIC_ASH
#define ASH_PROTOCOL_OK         0
#define ASH_PROTOCOL_ERROR      -1

class ashProtocol
{
    // variables:
    static binarySearchST **ashEntries;
    static int N, capacity;

    /**
     * @brief   CREATE THE AshProtocol module
     *
     *          Call ::init to add new xml file
     */
public:
    static void create( void);

    /**
     * @brief   This function searches for a given value in a set of symbol tables
     *          previous loaded to this module from files...
     *
     * @param key       char string search for key
     * @param value     char string value to look for in key (may be NULL)
     * @return index of found symboltable in ashEntries, or ASH_PROTOCOL_ERROR
     */
    static int indexByKey( const char *key, const char *value);

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
    static int getParam( uint index, const char *key, const char **ppVal);

    //----- resize the underlying arrays with new capacity -----
    static void resize(int _capacity);

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
    static int init( const char *fname);
    static void purgeObject( void);


};
#endif
