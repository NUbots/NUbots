/* -*- C++ -*- vim: set syntax=cpp:
 * PURPOSE: File containing definition of token-identifier and
 *          a function that maps token identifiers to a string
 *          name.
 *
 * NOTE: This file has been created automatically by Quex.
 *       Visit quex.org for further info.
 *
 * DATE: Thu Apr 10 14:30:00 2014
 *
 * (C) 2005-2010 Frank-Rene Schaefer
 * ABSOLUTELY NO WARRANTY                                           */
#ifndef __QUEX_INCLUDE_GUARD__AUTO_TOKEN_IDS_QUEX_REPLACEMENT__QUEX_TOKEN__
#define __QUEX_INCLUDE_GUARD__AUTO_TOKEN_IDS_QUEX_REPLACEMENT__QUEX_TOKEN__

#ifndef __QUEX_OPTION_PLAIN_C
#   include<cstdio> 
#else
#   include<stdio.h> 
#endif

/* The token class definition file can only be included after 
 * the definition on TERMINATION and UNINITIALIZED.          
 * (fschaef 12y03m24d: "I do not rememember why I wrote this.")    */
#include "Replacement-token.hpp"

const QUEX_TYPE_TOKEN_ID QUEX_TKN_BALL          = ((QUEX_TYPE_TOKEN_ID)10004);
const QUEX_TYPE_TOKEN_ID QUEX_TKN_CYAN_TEAM     = ((QUEX_TYPE_TOKEN_ID)10005);
const QUEX_TYPE_TOKEN_ID QUEX_TKN_DEDENT        = ((QUEX_TYPE_TOKEN_ID)10000);
const QUEX_TYPE_TOKEN_ID QUEX_TKN_FIELD         = ((QUEX_TYPE_TOKEN_ID)10006);
const QUEX_TYPE_TOKEN_ID QUEX_TKN_GOAL          = ((QUEX_TYPE_TOKEN_ID)10007);
const QUEX_TYPE_TOKEN_ID QUEX_TKN_INDENT        = ((QUEX_TYPE_TOKEN_ID)10001);
const QUEX_TYPE_TOKEN_ID QUEX_TKN_LINE          = ((QUEX_TYPE_TOKEN_ID)10008);
const QUEX_TYPE_TOKEN_ID QUEX_TKN_MAGENTA_TEAM  = ((QUEX_TYPE_TOKEN_ID)10009);
const QUEX_TYPE_TOKEN_ID QUEX_TKN_NODENT        = ((QUEX_TYPE_TOKEN_ID)10002);
const QUEX_TYPE_TOKEN_ID QUEX_TKN_TERMINATION   = ((QUEX_TYPE_TOKEN_ID)0);
const QUEX_TYPE_TOKEN_ID QUEX_TKN_UNCLASSIFIED  = ((QUEX_TYPE_TOKEN_ID)10010);
const QUEX_TYPE_TOKEN_ID QUEX_TKN_UNINITIALIZED = ((QUEX_TYPE_TOKEN_ID)10003);


QUEX_NAMESPACE_TOKEN_OPEN
extern const char* QUEX_NAME_TOKEN(map_id_to_name)(const QUEX_TYPE_TOKEN_ID TokenID);
QUEX_NAMESPACE_TOKEN_CLOSE

#endif /* __QUEX_INCLUDE_GUARD__AUTO_TOKEN_IDS_QUEX_REPLACEMENT__QUEX_TOKEN__ */
