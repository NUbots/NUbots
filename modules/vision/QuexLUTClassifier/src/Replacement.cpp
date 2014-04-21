#include "Replacement.hpp"
#include <quex/code_base/analyzer/C-adaptions.h>
QUEX_NAMESPACE_MAIN_OPEN
/* Global */QUEX_NAME(Mode)  QUEX_NAME(ReplacementRules);
#ifndef __QUEX_INDICATOR_DUMPED_TOKEN_ID_DEFINED
    static QUEX_TYPE_TOKEN_ID    QUEX_NAME_TOKEN(DumpedTokenIdObject);
#endif
#define self  (*(QUEX_TYPE_DERIVED_ANALYZER*)me)
#define __self_result_token_id    QUEX_NAME_TOKEN(DumpedTokenIdObject)

void
QUEX_NAME(ReplacementRules_on_entry)(QUEX_TYPE_ANALYZER* me, const QUEX_NAME(Mode)* FromMode) {
    (void)me;
    (void)FromMode;
#   ifdef QUEX_OPTION_RUNTIME_MODE_TRANSITION_CHECK
    QUEX_NAME(ReplacementRules).has_entry_from(FromMode);
#   endif

}

void
QUEX_NAME(ReplacementRules_on_exit)(QUEX_TYPE_ANALYZER* me, const QUEX_NAME(Mode)* ToMode)  {
    (void)me;
    (void)ToMode;
#   ifdef QUEX_OPTION_RUNTIME_MODE_TRANSITION_CHECK
    QUEX_NAME(ReplacementRules).has_exit_to(ToMode);
#   endif

}

#if defined(QUEX_OPTION_INDENTATION_TRIGGER) 
void
QUEX_NAME(ReplacementRules_on_indentation)(QUEX_TYPE_ANALYZER*    me, 
                                        QUEX_TYPE_INDENTATION  Indentation, 
                                        QUEX_TYPE_CHARACTER*   Begin) {
    (void)me;
    (void)Indentation;
    (void)Begin;
    return;
}
#endif

#ifdef QUEX_OPTION_RUNTIME_MODE_TRANSITION_CHECK
bool
QUEX_NAME(ReplacementRules_has_base)(const QUEX_NAME(Mode)* Mode) {
    (void)Mode;
    return false;
}
bool
QUEX_NAME(ReplacementRules_has_entry_from)(const QUEX_NAME(Mode)* Mode) {
    (void)Mode;
    return true; /* default */
}
bool
QUEX_NAME(ReplacementRules_has_exit_to)(const QUEX_NAME(Mode)* Mode) {
    (void)Mode;
    return false;
}
#endif    
#undef self
#undef __self_result_token_id
QUEX_NAMESPACE_MAIN_CLOSE

/* #include "Replacement.hpp"*/
QUEX_NAMESPACE_MAIN_OPEN
QUEX_TYPE_CHARACTER  QUEX_LEXEME_NULL_IN_ITS_NAMESPACE = (QUEX_TYPE_CHARACTER)0;
#ifdef      __QUEX_COUNT_VOID
#   undef   __QUEX_COUNT_VOID
#endif
#ifdef      __QUEX_OPTION_COUNTER
#    define __QUEX_COUNT_VOID(ME, BEGIN, END) \
            do {                              \
                QUEX_NAME(ReplacementRules_counter)((ME), (BEGIN), (END));     \
                __quex_debug_counter();       \
            } while(0)
#else
#    define __QUEX_COUNT_VOID(ME, BEGIN, END) /* empty */
#endif
#ifdef __QUEX_OPTION_COUNTER
static void
QUEX_NAME(ReplacementRules_counter)(QUEX_TYPE_ANALYZER* me, QUEX_TYPE_CHARACTER* LexemeBegin, QUEX_TYPE_CHARACTER* LexemeEnd)
{
#   define self (*me)
    QUEX_TYPE_CHARACTER* iterator    = LexemeBegin;
#   if defined(QUEX_OPTION_COLUMN_NUMBER_COUNTING)
    const QUEX_TYPE_CHARACTER* reference_p = LexemeBegin;
#   endif
__QUEX_IF_COUNT_COLUMNS(reference_p = iterator);
    __QUEX_IF_COUNT_SHIFT_VALUES();

    __quex_assert(LexemeBegin <= LexemeEnd);
    for(iterator=LexemeBegin; iterator < LexemeEnd; ) {
        if( (*(iterator)) >= 0xB ) {
                            ++(((iterator)));
            continue;
        } else if( (*(iterator)) == 0xA ) {
            __QUEX_IF_COUNT_LINES_ADD((size_t)1);
        __QUEX_IF_COUNT_COLUMNS_SET((size_t)1);
        __QUEX_IF_COUNT_COLUMNS(reference_p = (iterator) + 1);
                ++(((iterator)));
            continue;
        } else if( (*(iterator)) == 0x9 ) {
                    __QUEX_IF_COUNT_COLUMNS_ADD((size_t)(((iterator) - reference_p)));
        __QUEX_IF_COUNT_COLUMNS(self.counter._column_number_at_end &= ~ ((size_t)0x3));
        __QUEX_IF_COUNT_COLUMNS(self.counter._column_number_at_end += 4);
        __QUEX_IF_COUNT_COLUMNS(reference_p = (iterator) + 1);
                ++(((iterator)));
            continue;
        } else {
                            ++(((iterator)));
            continue;
        }

    }
    __quex_assert(iterator == LexemeEnd); /* Otherwise, lexeme violates codec character boundaries. */
__QUEX_IF_COUNT_COLUMNS_ADD((size_t)((iterator - reference_p)));
   return;
#  undef self
}
#endif /* __QUEX_OPTION_COUNTER */
#include <quex/code_base/analyzer/member/basic>
#include <quex/code_base/buffer/Buffer>
#ifdef QUEX_OPTION_TOKEN_POLICY_QUEUE
#   include <quex/code_base/token/TokenQueue>
#endif

#ifdef    CONTINUE
#   undef CONTINUE
#endif
#define   CONTINUE goto __REENTRY_PREPARATION; 

#ifdef    RETURN
#   undef RETURN
#endif

#define RETURN    __QUEX_PURE_RETURN;
#include <quex/code_base/temporary_macros_on>

__QUEX_TYPE_ANALYZER_RETURN_VALUE  
QUEX_NAME(ReplacementRules_analyzer_function)(QUEX_TYPE_ANALYZER* me) 
{
    /* NOTE: Different modes correspond to different analyzer functions. The 
     *       analyzer functions are all located inside the main class as static
     *       functions. That means, they are something like 'globals'. They 
     *       receive a pointer to the lexical analyzer, since static members do
     *       not have access to the 'this' pointer.                          */
#   if defined(QUEX_OPTION_TOKEN_POLICY_SINGLE)
    register QUEX_TYPE_TOKEN_ID __self_result_token_id 
           = (QUEX_TYPE_TOKEN_ID)__QUEX_SETTING_TOKEN_ID_UNINITIALIZED;
#   endif
#   ifdef     self
#       undef self
#   endif
#   define self (*((QUEX_TYPE_ANALYZER*)me))
    void*                          position                       = (void*)0x0;
    QUEX_TYPE_GOTO_LABEL           target_state_else_index        = QUEX_GOTO_LABEL_VOID;
    const size_t                   PositionRegisterN              = (size_t)0;
    QUEX_TYPE_CHARACTER            input                          = (QUEX_TYPE_CHARACTER)(0x00);
    QUEX_TYPE_GOTO_LABEL           target_state_index             = QUEX_GOTO_LABEL_VOID;
#   ifndef QUEX_OPTION_COMPUTED_GOTOS
#   endif /* QUEX_OPTION_COMPUTED_GOTOS */
#   define ReplacementRules    (QUEX_NAME(ReplacementRules))

    /* Post context positions do not have to be reset or initialized. If a state
     * is reached which is associated with 'end of post context' it is clear what
     * post context is meant. This results from the ways the state machine is 
     * constructed. Post context position's live cycle:
     *
     * (1)   unitialized (don't care)
     * (1.b) on buffer reload it may, or may not be adapted (don't care)
     * (2)   when a post context begin state is passed, then it is **SET** (now: take care)
     * (2.b) on buffer reload it **is adapted**.
     * (3)   when a terminal state of the post context is reached (which can only be reached
     *       for that particular post context), then the post context position is used
     *       to reset the input position.                                              */
#   if    defined(QUEX_OPTION_AUTOMATIC_ANALYSIS_CONTINUATION_ON_MODE_CHANGE) \
       || defined(QUEX_OPTION_ASSERTS)
    me->DEBUG_analyzer_function_at_entry = me->current_analyzer_function;
#   endif
__REENTRY:
    me->buffer._lexeme_start_p = me->buffer._input_p;
    QUEX_LEXEME_TERMINATING_ZERO_UNDO(&me->buffer);
_1646: /* INIT_STATE_TRANSITION_BLOCK */
    input = *(me->buffer._input_p);
    __quex_debug("Init State\n");
    __quex_debug_state(1413);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1413, 1647);
        case 0x28: goto _1419;
        case 0x63: goto _1417;
        case 0x67: goto _1420;
        case 0x6F: goto _1414;
        case 0x70: goto _1415;
        case 0x75: goto _1416;
        case 0x77: goto _1418;
        case 0x79: goto _1421;

    }
    __quex_debug_drop_out(1413);

goto _1649; /* TERMINAL_FAILURE */

_1413:


    ++(me->buffer._input_p);
    goto _1646;


    __quex_assert_no_passage();
_1536: /* (1536 from 1535) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1536);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1536, 1650);
        case 0x67: goto _1537;

    }
_1650:
    __quex_debug_drop_out(1536);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1537: /* (1537 from 1536) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1537);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1537, 1651);
        case 0x7B: goto _1538;

    }
_1651:
    __quex_debug_drop_out(1537);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1538: /* (1538 from 1537) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1538);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1538, 1652);
        case 0x33: goto _1539;

    }
_1652:
    __quex_debug_drop_out(1538);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1539: /* (1539 from 1538) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1539);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1539, 1653);
        case 0x2C: goto _1540;

    }
_1653:
    __quex_debug_drop_out(1539);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1540: /* (1540 from 1539) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1540);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1540, 1654);
        case 0x7D: goto _1541;

    }
_1654:
    __quex_debug_drop_out(1540);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1541: /* (1541 from 1540) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1541);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1541, 1655);
        case 0x29: goto _1542;

    }
_1655:
    __quex_debug_drop_out(1541);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1542: /* (1542 from 1541) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1542);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1542, 1656);
        case 0x2B: goto _1543;

    }
_1656:
    __quex_debug_drop_out(1542);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1543: /* (1543 from 1542) */

    ++(me->buffer._input_p);
    __quex_debug_state(1543);
    __quex_debug_drop_out(1543);
goto TERMINAL_690;

    __quex_assert_no_passage();
_1544: /* (1544 from 1464) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1544);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1544, 1658);
        case 0x7D: goto _1545;

    }
_1658:
    __quex_debug_drop_out(1544);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1545: /* (1545 from 1544) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1545);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1545, 1659);
        case 0x75: goto _1546;

    }
_1659:
    __quex_debug_drop_out(1545);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1546: /* (1546 from 1545) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1546);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1546, 1660);
        case 0x7B: goto _1547;

    }
_1660:
    __quex_debug_drop_out(1546);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1547: /* (1547 from 1546) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1547);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1547, 1661);
        case 0x31: goto _1548;

    }
_1661:
    __quex_debug_drop_out(1547);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1548: /* (1548 from 1547) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1548);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1548, 1662);
        case 0x2C: goto _1549;

    }
_1662:
    __quex_debug_drop_out(1548);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1549: /* (1549 from 1548) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1549);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1549, 1663);
        case 0x32: goto _1550;

    }
_1663:
    __quex_debug_drop_out(1549);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1550: /* (1550 from 1549) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1550);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1550, 1664);
        case 0x7D: goto _1551;

    }
_1664:
    __quex_debug_drop_out(1550);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1551: /* (1551 from 1550) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1551);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1551, 1665);
        case 0x67: goto _1552;

    }
_1665:
    __quex_debug_drop_out(1551);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1552: /* (1552 from 1551) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1552);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1552, 1666);
        case 0x7B: goto _1553;

    }
_1666:
    __quex_debug_drop_out(1552);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1553: /* (1553 from 1552) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1553);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1553, 1667);
        case 0x36: goto _1554;

    }
_1667:
    __quex_debug_drop_out(1553);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1554: /* (1554 from 1553) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1554);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1554, 1668);
        case 0x2C: goto _1555;

    }
_1668:
    __quex_debug_drop_out(1554);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1555: /* (1555 from 1554) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1555);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1555, 1669);
        case 0x7D: goto _1556;

    }
_1669:
    __quex_debug_drop_out(1555);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1556: /* (1556 from 1555) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1556);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1556, 1670);
        case 0x29: goto _1557;

    }
_1670:
    __quex_debug_drop_out(1556);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1557: /* (1557 from 1556) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1557);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1557, 1671);
        case 0x2B: goto _1558;

    }
_1671:
    __quex_debug_drop_out(1557);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1558: /* (1558 from 1557) */

    ++(me->buffer._input_p);
    __quex_debug_state(1558);
    __quex_debug_drop_out(1558);
goto TERMINAL_685;

    __quex_assert_no_passage();
_1559: /* (1559 from 1427) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1559);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1559, 1673);
        case 0x31: goto _1561;
        case 0x35: goto _1560;

    }
_1673:
    __quex_debug_drop_out(1559);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1560: /* (1560 from 1559) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1560);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1560, 1674);
        case 0x2C: goto _1577;

    }
_1674:
    __quex_debug_drop_out(1560);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1561: /* (1561 from 1559) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1561);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1561, 1675);
        case 0x2C: goto _1562;

    }
_1675:
    __quex_debug_drop_out(1561);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1562: /* (1562 from 1561) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1562);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1562, 1676);
        case 0x7D: goto _1563;

    }
_1676:
    __quex_debug_drop_out(1562);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1563: /* (1563 from 1562) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1563);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1563, 1677);
        case 0x75: goto _1564;

    }
_1677:
    __quex_debug_drop_out(1563);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1564: /* (1564 from 1563) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1564);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1564, 1678);
        case 0x7B: goto _1565;

    }
_1678:
    __quex_debug_drop_out(1564);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1565: /* (1565 from 1564) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1565);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1565, 1679);
        case 0x31: goto _1566;

    }
_1679:
    __quex_debug_drop_out(1565);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1566: /* (1566 from 1565) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1566);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1566, 1680);
        case 0x2C: goto _1567;

    }
_1680:
    __quex_debug_drop_out(1566);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1567: /* (1567 from 1566) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1567);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1567, 1681);
        case 0x33: goto _1568;

    }
_1681:
    __quex_debug_drop_out(1567);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1568: /* (1568 from 1567) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1568);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1568, 1682);
        case 0x7D: goto _1569;

    }
_1682:
    __quex_debug_drop_out(1568);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1569: /* (1569 from 1568) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1569);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1569, 1683);
        case 0x6F: goto _1570;

    }
_1683:
    __quex_debug_drop_out(1569);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1570: /* (1570 from 1569) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1570);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1570, 1684);
        case 0x7B: goto _1571;

    }
_1684:
    __quex_debug_drop_out(1570);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1571: /* (1571 from 1570) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1571);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1571, 1685);
        case 0x35: goto _1572;

    }
_1685:
    __quex_debug_drop_out(1571);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1572: /* (1572 from 1571) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1572);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1572, 1686);
        case 0x2C: goto _1573;

    }
_1686:
    __quex_debug_drop_out(1572);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1573: /* (1573 from 1572) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1573);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1573, 1687);
        case 0x7D: goto _1574;

    }
_1687:
    __quex_debug_drop_out(1573);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1574: /* (1574 from 1573) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1574);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1574, 1688);
        case 0x29: goto _1575;

    }
_1688:
    __quex_debug_drop_out(1574);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1575: /* (1575 from 1574) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1575);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1575, 1689);
        case 0x2B: goto _1576;

    }
_1689:
    __quex_debug_drop_out(1575);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1576: /* (1576 from 1575) */

    ++(me->buffer._input_p);
    __quex_debug_state(1576);
    __quex_debug_drop_out(1576);
goto TERMINAL_696;

    __quex_assert_no_passage();
_1577: /* (1577 from 1560) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1577);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1577, 1691);
        case 0x7D: goto _1578;

    }
_1691:
    __quex_debug_drop_out(1577);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1578: /* (1578 from 1577) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1578);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1578, 1692);
        case 0x75: goto _1579;

    }
_1692:
    __quex_debug_drop_out(1578);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1579: /* (1579 from 1578) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1579);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1579, 1693);
        case 0x7B: goto _1580;

    }
_1693:
    __quex_debug_drop_out(1579);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1580: /* (1580 from 1579) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1580);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1580, 1694);
        case 0x31: goto _1581;

    }
_1694:
    __quex_debug_drop_out(1580);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1581: /* (1581 from 1580) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1581);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1581, 1695);
        case 0x2C: goto _1582;

    }
_1695:
    __quex_debug_drop_out(1581);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1582: /* (1582 from 1581) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1582);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1582, 1696);
        case 0x33: goto _1583;

    }
_1696:
    __quex_debug_drop_out(1582);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1583: /* (1583 from 1582) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1583);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1583, 1697);
        case 0x7D: goto _1584;

    }
_1697:
    __quex_debug_drop_out(1583);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1584: /* (1584 from 1583) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1584);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1584, 1698);
        case 0x6F: goto _1585;

    }
_1698:
    __quex_debug_drop_out(1584);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1585: /* (1585 from 1584) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1585);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1585, 1699);
        case 0x7B: goto _1586;

    }
_1699:
    __quex_debug_drop_out(1585);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1586: /* (1586 from 1585) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1586);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1586, 1700);
        case 0x31: goto _1587;

    }
_1700:
    __quex_debug_drop_out(1586);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1587: /* (1587 from 1586) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1587);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1587, 1701);
        case 0x2C: goto _1588;

    }
_1701:
    __quex_debug_drop_out(1587);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1588: /* (1588 from 1587) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1588);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1588, 1702);
        case 0x7D: goto _1589;

    }
_1702:
    __quex_debug_drop_out(1588);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1589: /* (1589 from 1588) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1589);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1589, 1703);
        case 0x29: goto _1590;

    }
_1703:
    __quex_debug_drop_out(1589);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1590: /* (1590 from 1589) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1590);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1590, 1704);
        case 0x2B: goto _1591;

    }
_1704:
    __quex_debug_drop_out(1590);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1591: /* (1591 from 1590) */

    ++(me->buffer._input_p);
    __quex_debug_state(1591);
    __quex_debug_drop_out(1591);
goto TERMINAL_695;

    __quex_assert_no_passage();
_1592: /* (1592 from 1426) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1592);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1592, 1706);
        case 0x31: goto _1594;
        case 0x35: goto _1593;

    }
_1706:
    __quex_debug_drop_out(1592);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1593: /* (1593 from 1592) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1593);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1593, 1707);
        case 0x2C: goto _1618;

    }
_1707:
    __quex_debug_drop_out(1593);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1594: /* (1594 from 1592) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1594);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1594, 1708);
        case 0x2C: goto _1595;

    }
_1708:
    __quex_debug_drop_out(1594);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1595: /* (1595 from 1594) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1595);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1595, 1709);
        case 0x7D: goto _1596;

    }
_1709:
    __quex_debug_drop_out(1595);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1596: /* (1596 from 1595) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1596);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1596, 1710);
        case 0x6F: goto _1597;
        case 0x75: goto _1598;

    }
_1710:
    __quex_debug_drop_out(1596);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1597: /* (1597 from 1596) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1597);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1597, 1711);
        case 0x79: goto _1611;

    }
_1711:
    __quex_debug_drop_out(1597);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1598: /* (1598 from 1596) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1598);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1598, 1712);
        case 0x7B: goto _1599;

    }
_1712:
    __quex_debug_drop_out(1598);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1599: /* (1599 from 1598) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1599);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1599, 1713);
        case 0x31: goto _1600;

    }
_1713:
    __quex_debug_drop_out(1599);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1600: /* (1600 from 1599) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1600);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1600, 1714);
        case 0x2C: goto _1601;

    }
_1714:
    __quex_debug_drop_out(1600);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1601: /* (1601 from 1600) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1601);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1601, 1715);
        case 0x33: goto _1602;

    }
_1715:
    __quex_debug_drop_out(1601);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1602: /* (1602 from 1601) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1602);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1602, 1716);
        case 0x7D: goto _1603;

    }
_1716:
    __quex_debug_drop_out(1602);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1603: /* (1603 from 1602) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1603);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1603, 1717);
        case 0x79: goto _1604;

    }
_1717:
    __quex_debug_drop_out(1603);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1604: /* (1604 from 1603) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1604);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1604, 1718);
        case 0x7B: goto _1605;

    }
_1718:
    __quex_debug_drop_out(1604);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1605: /* (1605 from 1604) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1605);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1605, 1719);
        case 0x35: goto _1606;

    }
_1719:
    __quex_debug_drop_out(1605);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1606: /* (1606 from 1605) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1606);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1606, 1720);
        case 0x2C: goto _1607;

    }
_1720:
    __quex_debug_drop_out(1606);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1607: /* (1607 from 1606) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1607);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1607, 1721);
        case 0x7D: goto _1608;

    }
_1721:
    __quex_debug_drop_out(1607);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1608: /* (1608 from 1607) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1608);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1608, 1722);
        case 0x29: goto _1609;

    }
_1722:
    __quex_debug_drop_out(1608);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1609: /* (1609 from 1608) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1609);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1609, 1723);
        case 0x2B: goto _1610;

    }
_1723:
    __quex_debug_drop_out(1609);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1610: /* (1610 from 1609) */

    ++(me->buffer._input_p);
    __quex_debug_state(1610);
    __quex_debug_drop_out(1610);
goto TERMINAL_694;

    __quex_assert_no_passage();
_1611: /* (1611 from 1597) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1611);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1611, 1725);
        case 0x7B: goto _1612;

    }
_1725:
    __quex_debug_drop_out(1611);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1612: /* (1612 from 1611) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1612);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1612, 1726);
        case 0x35: goto _1613;

    }
_1726:
    __quex_debug_drop_out(1612);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1613: /* (1613 from 1612) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1613);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1613, 1727);
        case 0x2C: goto _1614;

    }
_1727:
    __quex_debug_drop_out(1613);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1614: /* (1614 from 1613) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1614);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1614, 1728);
        case 0x7D: goto _1615;

    }
_1728:
    __quex_debug_drop_out(1614);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1615: /* (1615 from 1614) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1615);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1615, 1729);
        case 0x29: goto _1616;

    }
_1729:
    __quex_debug_drop_out(1615);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1616: /* (1616 from 1615) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1616);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1616, 1730);
        case 0x2B: goto _1617;

    }
_1730:
    __quex_debug_drop_out(1616);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1617: /* (1617 from 1616) */

    ++(me->buffer._input_p);
    __quex_debug_state(1617);
    __quex_debug_drop_out(1617);
goto TERMINAL_691;

    __quex_assert_no_passage();
_1618: /* (1618 from 1593) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1618);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1618, 1732);
        case 0x7D: goto _1619;

    }
_1732:
    __quex_debug_drop_out(1618);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1619: /* (1619 from 1618) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1619);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1619, 1733);
        case 0x6F: goto _1620;
        case 0x75: goto _1621;

    }
_1733:
    __quex_debug_drop_out(1619);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1620: /* (1620 from 1619) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1620);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1620, 1734);
        case 0x79: goto _1634;

    }
_1734:
    __quex_debug_drop_out(1620);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1621: /* (1621 from 1619) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1621);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1621, 1735);
        case 0x7B: goto _1622;

    }
_1735:
    __quex_debug_drop_out(1621);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1622: /* (1622 from 1621) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1622);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1622, 1736);
        case 0x31: goto _1623;

    }
_1736:
    __quex_debug_drop_out(1622);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1623: /* (1623 from 1622) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1623);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1623, 1737);
        case 0x2C: goto _1624;

    }
_1737:
    __quex_debug_drop_out(1623);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1624: /* (1624 from 1623) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1624);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1624, 1738);
        case 0x33: goto _1625;

    }
_1738:
    __quex_debug_drop_out(1624);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1625: /* (1625 from 1624) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1625);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1625, 1739);
        case 0x7D: goto _1626;

    }
_1739:
    __quex_debug_drop_out(1625);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1626: /* (1626 from 1625) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1626);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1626, 1740);
        case 0x79: goto _1627;

    }
_1740:
    __quex_debug_drop_out(1626);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1627: /* (1627 from 1626) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1627);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1627, 1741);
        case 0x7B: goto _1628;

    }
_1741:
    __quex_debug_drop_out(1627);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1628: /* (1628 from 1627) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1628);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1628, 1742);
        case 0x31: goto _1629;

    }
_1742:
    __quex_debug_drop_out(1628);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1629: /* (1629 from 1628) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1629);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1629, 1743);
        case 0x2C: goto _1630;

    }
_1743:
    __quex_debug_drop_out(1629);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1630: /* (1630 from 1629) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1630);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1630, 1744);
        case 0x7D: goto _1631;

    }
_1744:
    __quex_debug_drop_out(1630);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1631: /* (1631 from 1630) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1631);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1631, 1745);
        case 0x29: goto _1632;

    }
_1745:
    __quex_debug_drop_out(1631);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1632: /* (1632 from 1631) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1632);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1632, 1746);
        case 0x2B: goto _1633;

    }
_1746:
    __quex_debug_drop_out(1632);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1633: /* (1633 from 1632) */

    ++(me->buffer._input_p);
    __quex_debug_state(1633);
    __quex_debug_drop_out(1633);
goto TERMINAL_693;

    __quex_assert_no_passage();
_1634: /* (1634 from 1620) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1634);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1634, 1748);
        case 0x7B: goto _1635;

    }
_1748:
    __quex_debug_drop_out(1634);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1635: /* (1635 from 1634) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1635);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1635, 1749);
        case 0x31: goto _1636;

    }
_1749:
    __quex_debug_drop_out(1635);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1636: /* (1636 from 1635) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1636);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1636, 1750);
        case 0x2C: goto _1637;

    }
_1750:
    __quex_debug_drop_out(1636);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1637: /* (1637 from 1636) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1637);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1637, 1751);
        case 0x7D: goto _1638;

    }
_1751:
    __quex_debug_drop_out(1637);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1638: /* (1638 from 1637) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1638);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1638, 1752);
        case 0x29: goto _1639;

    }
_1752:
    __quex_debug_drop_out(1638);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1639: /* (1639 from 1638) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1639);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1639, 1753);
        case 0x2B: goto _1640;

    }
_1753:
    __quex_debug_drop_out(1639);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1640: /* (1640 from 1639) */

    ++(me->buffer._input_p);
    __quex_debug_state(1640);
    __quex_debug_drop_out(1640);
goto TERMINAL_692;

    __quex_assert_no_passage();
_1641: /* (1641 from 1418) */

    ++(me->buffer._input_p);
    __quex_debug_state(1641);
    __quex_debug_drop_out(1641);
goto TERMINAL_699;

    __quex_assert_no_passage();
_1642: /* (1642 from 1417) */

    ++(me->buffer._input_p);
    __quex_debug_state(1642);
    __quex_debug_drop_out(1642);
goto TERMINAL_704;

    __quex_assert_no_passage();
_1643: /* (1643 from 1416) */

    ++(me->buffer._input_p);
    __quex_debug_state(1643);
    __quex_debug_drop_out(1643);
goto TERMINAL_705;

    __quex_assert_no_passage();
_1644: /* (1644 from 1415) */

    ++(me->buffer._input_p);
    __quex_debug_state(1644);
    __quex_debug_drop_out(1644);
goto TERMINAL_703;

    __quex_assert_no_passage();
_1645: /* (1645 from 1414) */

    ++(me->buffer._input_p);
    __quex_debug_state(1645);
    __quex_debug_drop_out(1645);
goto TERMINAL_701;

    __quex_assert_no_passage();
_1422: /* (1422 from 1421) */

    ++(me->buffer._input_p);
    __quex_debug_state(1422);
    __quex_debug_drop_out(1422);
goto TERMINAL_702;

    __quex_assert_no_passage();
_1423: /* (1423 from 1420) */

    ++(me->buffer._input_p);
    __quex_debug_state(1423);
    __quex_debug_drop_out(1423);
goto TERMINAL_700;

    __quex_assert_no_passage();
_1424: /* (1424 from 1419) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1424);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1424, 1762);
        case 0x3A: goto _1425;

    }
_1762:
    __quex_debug_drop_out(1424);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1425: /* (1425 from 1424) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1425);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1425, 1763);
        case 0x67: goto _1428;
        case 0x6F: goto _1427;
        case 0x77: goto _1429;
        case 0x79: goto _1426;

    }
_1763:
    __quex_debug_drop_out(1425);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1426: /* (1426 from 1425) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1426);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1426, 1764);
        case 0x7B: goto _1592;

    }
_1764:
    __quex_debug_drop_out(1426);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1427: /* (1427 from 1425) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1427);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1427, 1765);
        case 0x7B: goto _1559;

    }
_1765:
    __quex_debug_drop_out(1427);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1428: /* (1428 from 1425) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1428);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1428, 1766);
        case 0x7B: goto _1463;

    }
_1766:
    __quex_debug_drop_out(1428);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1429: /* (1429 from 1425) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1429);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1429, 1767);
        case 0x7B: goto _1430;

    }
_1767:
    __quex_debug_drop_out(1429);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1430: /* (1430 from 1429) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1430);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1430, 1768);
        case 0x31: goto _1432;
        case 0x35: goto _1431;

    }
_1768:
    __quex_debug_drop_out(1430);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1431: /* (1431 from 1430) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1431);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1431, 1769);
        case 0x2C: goto _1448;

    }
_1769:
    __quex_debug_drop_out(1431);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1432: /* (1432 from 1430) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1432);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1432, 1770);
        case 0x2C: goto _1433;

    }
_1770:
    __quex_debug_drop_out(1432);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1433: /* (1433 from 1432) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1433);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1433, 1771);
        case 0x7D: goto _1434;

    }
_1771:
    __quex_debug_drop_out(1433);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1434: /* (1434 from 1433) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1434);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1434, 1772);
        case 0x75: goto _1435;

    }
_1772:
    __quex_debug_drop_out(1434);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1435: /* (1435 from 1434) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1435);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1435, 1773);
        case 0x7B: goto _1436;

    }
_1773:
    __quex_debug_drop_out(1435);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1436: /* (1436 from 1435) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1436);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1436, 1774);
        case 0x31: goto _1437;

    }
_1774:
    __quex_debug_drop_out(1436);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1437: /* (1437 from 1436) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1437);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1437, 1775);
        case 0x2C: goto _1438;

    }
_1775:
    __quex_debug_drop_out(1437);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1438: /* (1438 from 1437) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1438);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1438, 1776);
        case 0x33: goto _1439;

    }
_1776:
    __quex_debug_drop_out(1438);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1439: /* (1439 from 1438) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1439);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1439, 1777);
        case 0x7D: goto _1440;

    }
_1777:
    __quex_debug_drop_out(1439);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1440: /* (1440 from 1439) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1440);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1440, 1778);
        case 0x77: goto _1441;

    }
_1778:
    __quex_debug_drop_out(1440);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1441: /* (1441 from 1440) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1441);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1441, 1779);
        case 0x7B: goto _1442;

    }
_1779:
    __quex_debug_drop_out(1441);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1442: /* (1442 from 1441) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1442);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1442, 1780);
        case 0x35: goto _1443;

    }
_1780:
    __quex_debug_drop_out(1442);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1443: /* (1443 from 1442) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1443);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1443, 1781);
        case 0x2C: goto _1444;

    }
_1781:
    __quex_debug_drop_out(1443);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1444: /* (1444 from 1443) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1444);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1444, 1782);
        case 0x7D: goto _1445;

    }
_1782:
    __quex_debug_drop_out(1444);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1445: /* (1445 from 1444) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1445);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1445, 1783);
        case 0x29: goto _1446;

    }
_1783:
    __quex_debug_drop_out(1445);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1446: /* (1446 from 1445) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1446);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1446, 1784);
        case 0x2B: goto _1447;

    }
_1784:
    __quex_debug_drop_out(1446);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1447: /* (1447 from 1446) */

    ++(me->buffer._input_p);
    __quex_debug_state(1447);
    __quex_debug_drop_out(1447);
goto TERMINAL_697;

    __quex_assert_no_passage();
_1448: /* (1448 from 1431) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1448);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1448, 1786);
        case 0x7D: goto _1449;

    }
_1786:
    __quex_debug_drop_out(1448);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1449: /* (1449 from 1448) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1449);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1449, 1787);
        case 0x75: goto _1450;

    }
_1787:
    __quex_debug_drop_out(1449);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1450: /* (1450 from 1449) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1450);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1450, 1788);
        case 0x7B: goto _1451;

    }
_1788:
    __quex_debug_drop_out(1450);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1451: /* (1451 from 1450) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1451);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1451, 1789);
        case 0x31: goto _1452;

    }
_1789:
    __quex_debug_drop_out(1451);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1452: /* (1452 from 1451) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1452);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1452, 1790);
        case 0x2C: goto _1453;

    }
_1790:
    __quex_debug_drop_out(1452);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1453: /* (1453 from 1452) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1453);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1453, 1791);
        case 0x33: goto _1454;

    }
_1791:
    __quex_debug_drop_out(1453);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1454: /* (1454 from 1453) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1454);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1454, 1792);
        case 0x7D: goto _1455;

    }
_1792:
    __quex_debug_drop_out(1454);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1455: /* (1455 from 1454) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1455);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1455, 1793);
        case 0x77: goto _1456;

    }
_1793:
    __quex_debug_drop_out(1455);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1456: /* (1456 from 1455) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1456);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1456, 1794);
        case 0x7B: goto _1457;

    }
_1794:
    __quex_debug_drop_out(1456);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1457: /* (1457 from 1456) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1457);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1457, 1795);
        case 0x31: goto _1458;

    }
_1795:
    __quex_debug_drop_out(1457);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1458: /* (1458 from 1457) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1458);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1458, 1796);
        case 0x2C: goto _1459;

    }
_1796:
    __quex_debug_drop_out(1458);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1459: /* (1459 from 1458) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1459);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1459, 1797);
        case 0x7D: goto _1460;

    }
_1797:
    __quex_debug_drop_out(1459);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1460: /* (1460 from 1459) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1460);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1460, 1798);
        case 0x29: goto _1461;

    }
_1798:
    __quex_debug_drop_out(1460);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1461: /* (1461 from 1460) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1461);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1461, 1799);
        case 0x2B: goto _1462;

    }
_1799:
    __quex_debug_drop_out(1461);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1462: /* (1462 from 1461) */

    ++(me->buffer._input_p);
    __quex_debug_state(1462);
    __quex_debug_drop_out(1462);
goto TERMINAL_698;

    __quex_assert_no_passage();
_1463: /* (1463 from 1428) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1463);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1463, 1801);
        case 0x31: goto _1467;
        case 0x32: goto _1465;
        case 0x33: goto _1466;
        case 0x36: goto _1464;

    }
_1801:
    __quex_debug_drop_out(1463);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1464: /* (1464 from 1463) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1464);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1464, 1802);
        case 0x2C: goto _1544;

    }
_1802:
    __quex_debug_drop_out(1464);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1465: /* (1465 from 1463) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1465);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1465, 1803);
        case 0x30: goto _1528;

    }
_1803:
    __quex_debug_drop_out(1465);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1466: /* (1466 from 1463) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1466);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1466, 1804);
        case 0x2C: goto _1505;

    }
_1804:
    __quex_debug_drop_out(1466);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1467: /* (1467 from 1463) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1467);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1467, 1805);
        case 0x2C: goto _1469;
        case 0x32: goto _1468;

    }
_1805:
    __quex_debug_drop_out(1467);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1468: /* (1468 from 1467) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1468);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1468, 1806);
        case 0x2C: goto _1485;

    }
_1806:
    __quex_debug_drop_out(1468);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1469: /* (1469 from 1467) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1469);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1469, 1807);
        case 0x7D: goto _1470;

    }
_1807:
    __quex_debug_drop_out(1469);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1470: /* (1470 from 1469) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1470);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1470, 1808);
        case 0x75: goto _1471;

    }
_1808:
    __quex_debug_drop_out(1470);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1471: /* (1471 from 1470) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1471);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1471, 1809);
        case 0x7B: goto _1472;

    }
_1809:
    __quex_debug_drop_out(1471);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1472: /* (1472 from 1471) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1472);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1472, 1810);
        case 0x31: goto _1473;

    }
_1810:
    __quex_debug_drop_out(1472);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1473: /* (1473 from 1472) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1473);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1473, 1811);
        case 0x2C: goto _1474;

    }
_1811:
    __quex_debug_drop_out(1473);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1474: /* (1474 from 1473) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1474);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1474, 1812);
        case 0x34: goto _1475;

    }
_1812:
    __quex_debug_drop_out(1474);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1475: /* (1475 from 1474) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1475);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1475, 1813);
        case 0x7D: goto _1476;

    }
_1813:
    __quex_debug_drop_out(1475);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1476: /* (1476 from 1475) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1476);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1476, 1814);
        case 0x67: goto _1477;

    }
_1814:
    __quex_debug_drop_out(1476);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1477: /* (1477 from 1476) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1477);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1477, 1815);
        case 0x7B: goto _1478;

    }
_1815:
    __quex_debug_drop_out(1477);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1478: /* (1478 from 1477) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1478);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1478, 1816);
        case 0x31: goto _1479;

    }
_1816:
    __quex_debug_drop_out(1478);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1479: /* (1479 from 1478) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1479);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1479, 1817);
        case 0x32: goto _1480;

    }
_1817:
    __quex_debug_drop_out(1479);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1480: /* (1480 from 1479) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1480);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1480, 1818);
        case 0x2C: goto _1481;

    }
_1818:
    __quex_debug_drop_out(1480);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1481: /* (1481 from 1480) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1481);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1481, 1819);
        case 0x7D: goto _1482;

    }
_1819:
    __quex_debug_drop_out(1481);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1482: /* (1482 from 1481) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1482);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1482, 1820);
        case 0x29: goto _1483;

    }
_1820:
    __quex_debug_drop_out(1482);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1483: /* (1483 from 1482) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1483);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1483, 1821);
        case 0x2B: goto _1484;

    }
_1821:
    __quex_debug_drop_out(1483);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1484: /* (1484 from 1483) */

    ++(me->buffer._input_p);
    __quex_debug_state(1484);
    __quex_debug_drop_out(1484);
goto TERMINAL_687;

    __quex_assert_no_passage();
_1485: /* (1485 from 1468) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1485);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1485, 1823);
        case 0x7D: goto _1486;

    }
_1823:
    __quex_debug_drop_out(1485);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1486: /* (1486 from 1485) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1486);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1486, 1824);
        case 0x75: goto _1487;

    }
_1824:
    __quex_debug_drop_out(1486);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1487: /* (1487 from 1486) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1487);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1487, 1825);
        case 0x7B: goto _1488;

    }
_1825:
    __quex_debug_drop_out(1487);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1488: /* (1488 from 1487) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1488);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1488, 1826);
        case 0x31: goto _1489;

    }
_1826:
    __quex_debug_drop_out(1488);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1489: /* (1489 from 1488) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1489);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1489, 1827);
        case 0x2C: goto _1490;

    }
_1827:
    __quex_debug_drop_out(1489);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1490: /* (1490 from 1489) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1490);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1490, 1828);
        case 0x34: goto _1491;

    }
_1828:
    __quex_debug_drop_out(1490);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1491: /* (1491 from 1490) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1491);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1491, 1829);
        case 0x7D: goto _1492;

    }
_1829:
    __quex_debug_drop_out(1491);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1492: /* (1492 from 1491) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1492);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1492, 1830);
        case 0x67: goto _1493;

    }
_1830:
    __quex_debug_drop_out(1492);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1493: /* (1493 from 1492) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1493);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1493, 1831);
        case 0x7B: goto _1494;

    }
_1831:
    __quex_debug_drop_out(1493);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1494: /* (1494 from 1493) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1494);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1494, 1832);
        case 0x31: goto _1495;

    }
_1832:
    __quex_debug_drop_out(1494);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1495: /* (1495 from 1494) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1495);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1495, 1833);
        case 0x2C: goto _1496;
        case 0x32: goto _1497;

    }
_1833:
    __quex_debug_drop_out(1495);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1496: /* (1496 from 1495) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1496);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1496, 1834);
        case 0x7D: goto _1502;

    }
_1834:
    __quex_debug_drop_out(1496);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1497: /* (1497 from 1495) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1497);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1497, 1835);
        case 0x2C: goto _1498;

    }
_1835:
    __quex_debug_drop_out(1497);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1498: /* (1498 from 1497) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1498);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1498, 1836);
        case 0x7D: goto _1499;

    }
_1836:
    __quex_debug_drop_out(1498);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1499: /* (1499 from 1498) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1499);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1499, 1837);
        case 0x29: goto _1500;

    }
_1837:
    __quex_debug_drop_out(1499);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1500: /* (1500 from 1499) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1500);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1500, 1838);
        case 0x2B: goto _1501;

    }
_1838:
    __quex_debug_drop_out(1500);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1501: /* (1501 from 1500) */

    ++(me->buffer._input_p);
    __quex_debug_state(1501);
    __quex_debug_drop_out(1501);
goto TERMINAL_686;

    __quex_assert_no_passage();
_1502: /* (1502 from 1496) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1502);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1502, 1840);
        case 0x29: goto _1503;

    }
_1840:
    __quex_debug_drop_out(1502);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1503: /* (1503 from 1502) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1503);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1503, 1841);
        case 0x2B: goto _1504;

    }
_1841:
    __quex_debug_drop_out(1503);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1504: /* (1504 from 1503) */

    ++(me->buffer._input_p);
    __quex_debug_state(1504);
    __quex_debug_drop_out(1504);
goto TERMINAL_688;

    __quex_assert_no_passage();
_1505: /* (1505 from 1466) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1505);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1505, 1843);
        case 0x7D: goto _1506;

    }
_1843:
    __quex_debug_drop_out(1505);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1506: /* (1506 from 1505) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1506);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1506, 1844);
        case 0x75: goto _1507;

    }
_1844:
    __quex_debug_drop_out(1506);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1507: /* (1507 from 1506) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1507);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1507, 1845);
        case 0x67: goto _1509;
        case 0x7B: goto _1508;

    }
_1845:
    __quex_debug_drop_out(1507);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1508: /* (1508 from 1507) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1508);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1508, 1846);
        case 0x31: goto _1516;

    }
_1846:
    __quex_debug_drop_out(1508);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1509: /* (1509 from 1507) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1509);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1509, 1847);
        case 0x7B: goto _1510;

    }
_1847:
    __quex_debug_drop_out(1509);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1510: /* (1510 from 1509) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1510);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1510, 1848);
        case 0x33: goto _1511;

    }
_1848:
    __quex_debug_drop_out(1510);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1511: /* (1511 from 1510) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1511);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1511, 1849);
        case 0x2C: goto _1512;

    }
_1849:
    __quex_debug_drop_out(1511);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1512: /* (1512 from 1511) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1512);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1512, 1850);
        case 0x7D: goto _1513;

    }
_1850:
    __quex_debug_drop_out(1512);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1513: /* (1513 from 1512) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1513);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1513, 1851);
        case 0x29: goto _1514;

    }
_1851:
    __quex_debug_drop_out(1513);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1514: /* (1514 from 1513) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1514);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1514, 1852);
        case 0x2B: goto _1515;

    }
_1852:
    __quex_debug_drop_out(1514);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1515: /* (1515 from 1514) */

    ++(me->buffer._input_p);
    __quex_debug_state(1515);
    __quex_debug_drop_out(1515);
goto TERMINAL_684;

    __quex_assert_no_passage();
_1516: /* (1516 from 1508) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1516);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1516, 1854);
        case 0x2C: goto _1517;

    }
_1854:
    __quex_debug_drop_out(1516);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1517: /* (1517 from 1516) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1517);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1517, 1855);
        case 0x37: goto _1518;

    }
_1855:
    __quex_debug_drop_out(1517);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1518: /* (1518 from 1517) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1518);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1518, 1856);
        case 0x7D: goto _1519;

    }
_1856:
    __quex_debug_drop_out(1518);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1519: /* (1519 from 1518) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1519);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1519, 1857);
        case 0x67: goto _1520;

    }
_1857:
    __quex_debug_drop_out(1519);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1520: /* (1520 from 1519) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1520);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1520, 1858);
        case 0x7B: goto _1521;

    }
_1858:
    __quex_debug_drop_out(1520);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1521: /* (1521 from 1520) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1521);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1521, 1859);
        case 0x32: goto _1522;

    }
_1859:
    __quex_debug_drop_out(1521);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1522: /* (1522 from 1521) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1522);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1522, 1860);
        case 0x30: goto _1523;

    }
_1860:
    __quex_debug_drop_out(1522);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1523: /* (1523 from 1522) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1523);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1523, 1861);
        case 0x2C: goto _1524;

    }
_1861:
    __quex_debug_drop_out(1523);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1524: /* (1524 from 1523) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1524);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1524, 1862);
        case 0x7D: goto _1525;

    }
_1862:
    __quex_debug_drop_out(1524);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1525: /* (1525 from 1524) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1525);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1525, 1863);
        case 0x29: goto _1526;

    }
_1863:
    __quex_debug_drop_out(1525);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1526: /* (1526 from 1525) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1526);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1526, 1864);
        case 0x2B: goto _1527;

    }
_1864:
    __quex_debug_drop_out(1526);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1527: /* (1527 from 1526) */

    ++(me->buffer._input_p);
    __quex_debug_state(1527);
    __quex_debug_drop_out(1527);
goto TERMINAL_689;

    __quex_assert_no_passage();
_1528: /* (1528 from 1465) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1528);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1528, 1866);
        case 0x2C: goto _1529;

    }
_1866:
    __quex_debug_drop_out(1528);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1529: /* (1529 from 1528) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1529);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1529, 1867);
        case 0x7D: goto _1530;

    }
_1867:
    __quex_debug_drop_out(1529);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1530: /* (1530 from 1529) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1530);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1530, 1868);
        case 0x75: goto _1531;

    }
_1868:
    __quex_debug_drop_out(1530);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1531: /* (1531 from 1530) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1531);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1531, 1869);
        case 0x7B: goto _1532;

    }
_1869:
    __quex_debug_drop_out(1531);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1532: /* (1532 from 1531) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1532);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1532, 1870);
        case 0x31: goto _1533;

    }
_1870:
    __quex_debug_drop_out(1532);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1533: /* (1533 from 1532) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1533);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1533, 1871);
        case 0x2C: goto _1534;

    }
_1871:
    __quex_debug_drop_out(1533);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1534: /* (1534 from 1533) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1534);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1534, 1872);
        case 0x37: goto _1535;

    }
_1872:
    __quex_debug_drop_out(1534);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1535: /* (1535 from 1534) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1535);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1535, 1873);
        case 0x7D: goto _1536;

    }
_1873:
    __quex_debug_drop_out(1535);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1414: /* (1414 from 1413) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1414);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1414, 1874);
        case 0x2B: goto _1645;

    }
_1874:
    __quex_debug_drop_out(1414);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1415: /* (1415 from 1413) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1415);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1415, 1875);
        case 0x2B: goto _1644;

    }
_1875:
    __quex_debug_drop_out(1415);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1416: /* (1416 from 1413) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1416);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1416, 1876);
        case 0x2B: goto _1643;

    }
_1876:
    __quex_debug_drop_out(1416);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1417: /* (1417 from 1413) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1417);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1417, 1877);
        case 0x2B: goto _1642;

    }
_1877:
    __quex_debug_drop_out(1417);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1418: /* (1418 from 1413) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1418);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1418, 1878);
        case 0x2B: goto _1641;

    }
_1878:
    __quex_debug_drop_out(1418);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1419: /* (1419 from 1413) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1419);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1419, 1879);
        case 0x3F: goto _1424;

    }
_1879:
    __quex_debug_drop_out(1419);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1420: /* (1420 from 1413) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1420);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1420, 1880);
        case 0x2B: goto _1423;

    }
_1880:
    __quex_debug_drop_out(1420);

goto _1649; /* TERMINAL_FAILURE */

    __quex_assert_no_passage();
_1421: /* (1421 from 1413) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_state(1421);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 1421, 1881);
        case 0x2B: goto _1422;

    }
_1881:
    __quex_debug_drop_out(1421);

goto _1649; /* TERMINAL_FAILURE */
    /* (*) Terminal states _______________________________________________________
     *
     * States that implement actions of the 'winner patterns.                     */

    /* Lexeme setup: 
     *
     * There is a temporary zero stored at the end of each lexeme, if the action 
     * references to the 'Lexeme'. 'LexemeNull' provides a reference to an empty
     * zero terminated string.                                                    */
#if defined(QUEX_OPTION_ASSERTS)
#   define Lexeme       QUEX_NAME(access_Lexeme)((const char*)__FILE__, (size_t)__LINE__, &me->buffer)
#   define LexemeBegin  QUEX_NAME(access_LexemeBegin)((const char*)__FILE__, (size_t)__LINE__, &me->buffer)
#   define LexemeL      QUEX_NAME(access_LexemeL)((const char*)__FILE__, (size_t)__LINE__, &me->buffer)
#   define LexemeEnd    QUEX_NAME(access_LexemeEnd)((const char*)__FILE__, (size_t)__LINE__, &me->buffer)
#else
#   define Lexeme       (me->buffer._lexeme_start_p)
#   define LexemeBegin  Lexeme
#   define LexemeL      ((size_t)(me->buffer._input_p - me->buffer._lexeme_start_p))
#   define LexemeEnd    me->buffer._input_p
#endif

#define LexemeNull      (&QUEX_LEXEME_NULL)

TERMINAL_684:
    __quex_debug("* terminal 684:   \"(?:g{3,}ug{3,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(16);
    {
#   line 20 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3793 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_685:
    __quex_debug("* terminal 685:   \"(?:g{6,}u{1,2}g{6,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(21);
    {
#   line 23 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3808 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_686:
    __quex_debug("* terminal 686:   \"(?:g{12,}u{1,4}g{12,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(23);
    {
#   line 26 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3823 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_687:
    __quex_debug("* terminal 687:   \"(?:g{1,}u{1,4}g{12,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(22);
    {
#   line 29 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3838 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_688:
    __quex_debug("* terminal 688:   \"(?:g{12,}u{1,4}g{1,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(22);
    {
#   line 30 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3853 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_689:
    __quex_debug("* terminal 689:   \"(?:g{3,}u{1,7}g{20,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(22);
    {
#   line 33 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3868 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_690:
    __quex_debug("* terminal 690:   \"(?:g{20,}u{1,7}g{3,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(22);
    {
#   line 34 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3883 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_691:
    __quex_debug("* terminal 691:   \"(?:y{1,}oy{5,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(16);
    {
#   line 37 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_GOAL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3898 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_692:
    __quex_debug("* terminal 692:   \"(?:y{5,}oy{1,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(16);
    {
#   line 38 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_GOAL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3913 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_693:
    __quex_debug("* terminal 693:   \"(?:y{5,}u{1,3}y{1,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(21);
    {
#   line 41 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_GOAL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3928 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_694:
    __quex_debug("* terminal 694:   \"(?:y{1,}u{1,3}y{5,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(21);
    {
#   line 42 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_GOAL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3943 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_695:
    __quex_debug("* terminal 695:   \"(?:o{5,}u{1,3}o{1,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(21);
    {
#   line 45 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_BALL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3958 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_696:
    __quex_debug("* terminal 696:   \"(?:o{1,}u{1,3}o{5,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(21);
    {
#   line 46 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_BALL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3973 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_697:
    __quex_debug("* terminal 697:   \"(?:w{1,}u{1,3}w{5,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(21);
    {
#   line 49 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_LINE);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3988 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_698:
    __quex_debug("* terminal 698:   \"(?:w{5,}u{1,3}w{1,})+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(21);
    {
#   line 50 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_LINE);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4003 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_699:
    __quex_debug("* terminal 699:   \"w+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(2);
    {
#   line 53 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_LINE);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4018 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_700:
    __quex_debug("* terminal 700:   \"g+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(2);
    {
#   line 54 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4033 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_701:
    __quex_debug("* terminal 701:   \"o+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(2);
    {
#   line 55 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_BALL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4048 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_702:
    __quex_debug("* terminal 702:   \"y+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(2);
    {
#   line 56 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_GOAL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4063 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_703:
    __quex_debug("* terminal 703:   \"p+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(2);
    {
#   line 57 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_MAGENTA_TEAM);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4078 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_704:
    __quex_debug("* terminal 704:   \"c+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(2);
    {
#   line 58 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_CYAN_TEAM);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4093 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_705:
    __quex_debug("* terminal 705:   \"u+\"\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(2);
    {
#   line 59 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_UNCLASSIFIED);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4108 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;


_1649: /* TERMINAL: FAILURE */
    if(QUEX_NAME(Buffer_is_end_of_file)(&me->buffer)) {
        /* Init state is going to detect 'input == buffer limit code', and
         * enter the reload procedure, which will decide about 'end of stream'. */
    } else {
        /* In init state 'input = *input_p' and we need to increment
         * in order to avoid getting stalled. Else, input = *(input_p - 1),
         * so 'input_p' points already to the next character.                   */
        if( me->buffer._input_p == me->buffer._lexeme_start_p ) {
            /* Step over non-matching character */
            ++(me->buffer._input_p);
        }
    }
    __QUEX_COUNT_VOID(&self, LexemeBegin, LexemeEnd);
    QUEX_LEXEME_TERMINATING_ZERO_SET(&me->buffer);
    {
QUEX_ERROR_EXIT("\n    Match failure in mode 'ReplacementRules'.\n"
    "    No 'on_failure' section provided for this mode.\n"
    "    Proposal: Define 'on_failure' and analyze 'Lexeme'.\n");

    }
    goto __REENTRY_PREPARATION_2;


/* TERMINAL: END_OF_STREAM */
_1647:
__QUEX_IF_COUNT_SHIFT_VALUES();
    {
self_send(__QUEX_SETTING_TOKEN_ID_TERMINATION);
RETURN;

    }
    /* End of Stream causes a return from the lexical analyzer, so that no
     * tokens can be filled after the termination token.                    */
    RETURN;

__REENTRY_PREPARATION:
    /* (*) Common point for **restarting** lexical analysis.
     *     at each time when CONTINUE is called at the end of a pattern.     */
 

    /* FAILURE needs not to run through 'on_after_match'. It enters here.    */
__REENTRY_PREPARATION_2:

#   undef Lexeme
#   undef LexemeBegin
#   undef LexemeEnd
#   undef LexemeNull
#   undef LexemeL

#   ifndef __QUEX_OPTION_PLAIN_ANALYZER_OBJECT
#   ifdef  QUEX_OPTION_TOKEN_POLICY_QUEUE
    if( QUEX_NAME(TokenQueue_is_full)(&self._token_queue) ) {
        return;
    }
#   else
    if( self_token_get_id() != __QUEX_SETTING_TOKEN_ID_UNINITIALIZED) {
        return __self_result_token_id;
    }
#   endif
#   endif


    /* Post context positions do not have to be reset or initialized. If a state
     * is reached which is associated with 'end of post context' it is clear what
     * post context is meant. This results from the ways the state machine is 
     * constructed. Post context position's live cycle:
     *
     * (1)   unitialized (don't care)
     * (1.b) on buffer reload it may, or may not be adapted (don't care)
     * (2)   when a post context begin state is passed, then it is **SET** (now: take care)
     * (2.b) on buffer reload it **is adapted**.
     * (3)   when a terminal state of the post context is reached (which can only be reached
     *       for that particular post context), then the post context position is used
     *       to reset the input position.                                              */

    /*  If a mode change happened, then the function must first return and
     *  indicate that another mode function is to be called. At this point, 
     *  we to force a 'return' on a mode change. 
     *
     *  Pseudo Code: if( previous_mode != current_mode ) {
     *                   return 0;
     *               }
     *
     *  When the analyzer returns, the caller function has to watch if a mode 
     *  change occurred. If not it can call this function again.             */
#   if    defined(QUEX_OPTION_AUTOMATIC_ANALYSIS_CONTINUATION_ON_MODE_CHANGE)        || defined(QUEX_OPTION_ASSERTS)
    if( me->DEBUG_analyzer_function_at_entry != me->current_analyzer_function ) 
#   endif
    { 
#       if defined(QUEX_OPTION_AUTOMATIC_ANALYSIS_CONTINUATION_ON_MODE_CHANGE)
        self_token_set_id(__QUEX_SETTING_TOKEN_ID_UNINITIALIZED);
        __QUEX_PURE_RETURN;
#       elif defined(QUEX_OPTION_ASSERTS)
        QUEX_ERROR_EXIT("Mode change without immediate return from the lexical analyzer.");
#       endif
    }

    goto __REENTRY;

    __quex_assert_no_passage();
__RELOAD_FORWARD:

    __quex_debug1("__RELOAD_FORWARD");
    __quex_assert(*(me->buffer._input_p) == QUEX_SETTING_BUFFER_LIMIT_CODE);
    if( me->buffer._memory._end_of_file_p == 0x0 ) {

        __quex_debug_reload_before(); /* Leave macro here to report source position. */
        QUEX_NAME(buffer_reload_forward)(&me->buffer, (QUEX_TYPE_CHARACTER_POSITION*)position, PositionRegisterN);

        __quex_debug_reload_after();
        QUEX_GOTO_STATE(target_state_index);
    }
    __quex_debug("reload impossible\n");
    QUEX_GOTO_STATE(target_state_else_index);
#   ifndef QUEX_OPTION_COMPUTED_GOTOS
    __quex_assert_no_passage();
__STATE_ROUTER:
    switch( target_state_index ) {
        case 1413: { goto _1413; }
        case 1414: { goto _1414; }
        case 1415: { goto _1415; }
        case 1416: { goto _1416; }
        case 1417: { goto _1417; }
        case 1418: { goto _1418; }
        case 1419: { goto _1419; }
        case 1420: { goto _1420; }
        case 1421: { goto _1421; }
        case 1422: { goto _1422; }
        case 1423: { goto _1423; }
        case 1424: { goto _1424; }
        case 1425: { goto _1425; }
        case 1426: { goto _1426; }
        case 1427: { goto _1427; }
        case 1428: { goto _1428; }
        case 1429: { goto _1429; }
        case 1430: { goto _1430; }
        case 1431: { goto _1431; }
        case 1432: { goto _1432; }
        case 1433: { goto _1433; }
        case 1434: { goto _1434; }
        case 1435: { goto _1435; }
        case 1436: { goto _1436; }
        case 1437: { goto _1437; }
        case 1438: { goto _1438; }
        case 1439: { goto _1439; }
        case 1440: { goto _1440; }
        case 1441: { goto _1441; }
        case 1442: { goto _1442; }
        case 1443: { goto _1443; }
        case 1444: { goto _1444; }
        case 1445: { goto _1445; }
        case 1446: { goto _1446; }
        case 1447: { goto _1447; }
        case 1448: { goto _1448; }
        case 1449: { goto _1449; }
        case 1450: { goto _1450; }
        case 1451: { goto _1451; }
        case 1452: { goto _1452; }
        case 1453: { goto _1453; }
        case 1454: { goto _1454; }
        case 1455: { goto _1455; }
        case 1456: { goto _1456; }
        case 1457: { goto _1457; }
        case 1458: { goto _1458; }
        case 1459: { goto _1459; }
        case 1460: { goto _1460; }
        case 1461: { goto _1461; }
        case 1462: { goto _1462; }
        case 1463: { goto _1463; }
        case 1464: { goto _1464; }
        case 1465: { goto _1465; }
        case 1466: { goto _1466; }
        case 1467: { goto _1467; }
        case 1468: { goto _1468; }
        case 1469: { goto _1469; }
        case 1470: { goto _1470; }
        case 1471: { goto _1471; }
        case 1472: { goto _1472; }
        case 1473: { goto _1473; }
        case 1474: { goto _1474; }
        case 1475: { goto _1475; }
        case 1476: { goto _1476; }
        case 1477: { goto _1477; }
        case 1478: { goto _1478; }
        case 1479: { goto _1479; }
        case 1480: { goto _1480; }
        case 1481: { goto _1481; }
        case 1482: { goto _1482; }
        case 1483: { goto _1483; }
        case 1484: { goto _1484; }
        case 1485: { goto _1485; }
        case 1486: { goto _1486; }
        case 1487: { goto _1487; }
        case 1488: { goto _1488; }
        case 1489: { goto _1489; }
        case 1490: { goto _1490; }
        case 1491: { goto _1491; }
        case 1492: { goto _1492; }
        case 1493: { goto _1493; }
        case 1494: { goto _1494; }
        case 1495: { goto _1495; }
        case 1496: { goto _1496; }
        case 1497: { goto _1497; }
        case 1498: { goto _1498; }
        case 1499: { goto _1499; }
        case 1500: { goto _1500; }
        case 1501: { goto _1501; }
        case 1502: { goto _1502; }
        case 1503: { goto _1503; }
        case 1504: { goto _1504; }
        case 1505: { goto _1505; }
        case 1506: { goto _1506; }
        case 1507: { goto _1507; }
        case 1508: { goto _1508; }
        case 1509: { goto _1509; }
        case 1510: { goto _1510; }
        case 1511: { goto _1511; }
        case 1512: { goto _1512; }
        case 1513: { goto _1513; }
        case 1514: { goto _1514; }
        case 1515: { goto _1515; }
        case 1516: { goto _1516; }
        case 1517: { goto _1517; }
        case 1518: { goto _1518; }
        case 1519: { goto _1519; }
        case 1520: { goto _1520; }
        case 1521: { goto _1521; }
        case 1522: { goto _1522; }
        case 1523: { goto _1523; }
        case 1524: { goto _1524; }
        case 1525: { goto _1525; }
        case 1526: { goto _1526; }
        case 1527: { goto _1527; }
        case 1528: { goto _1528; }
        case 1529: { goto _1529; }
        case 1530: { goto _1530; }
        case 1531: { goto _1531; }
        case 1532: { goto _1532; }
        case 1533: { goto _1533; }
        case 1534: { goto _1534; }
        case 1535: { goto _1535; }
        case 1536: { goto _1536; }
        case 1537: { goto _1537; }
        case 1538: { goto _1538; }
        case 1539: { goto _1539; }
        case 1540: { goto _1540; }
        case 1541: { goto _1541; }
        case 1542: { goto _1542; }
        case 1543: { goto _1543; }
        case 1544: { goto _1544; }
        case 1545: { goto _1545; }
        case 1546: { goto _1546; }
        case 1547: { goto _1547; }
        case 1548: { goto _1548; }
        case 1549: { goto _1549; }
        case 1550: { goto _1550; }
        case 1551: { goto _1551; }
        case 1552: { goto _1552; }
        case 1553: { goto _1553; }
        case 1554: { goto _1554; }
        case 1555: { goto _1555; }
        case 1556: { goto _1556; }
        case 1557: { goto _1557; }
        case 1558: { goto _1558; }
        case 1559: { goto _1559; }
        case 1560: { goto _1560; }
        case 1561: { goto _1561; }
        case 1562: { goto _1562; }
        case 1563: { goto _1563; }
        case 1564: { goto _1564; }
        case 1565: { goto _1565; }
        case 1566: { goto _1566; }
        case 1567: { goto _1567; }
        case 1568: { goto _1568; }
        case 1569: { goto _1569; }
        case 1570: { goto _1570; }
        case 1571: { goto _1571; }
        case 1572: { goto _1572; }
        case 1573: { goto _1573; }
        case 1574: { goto _1574; }
        case 1575: { goto _1575; }
        case 1576: { goto _1576; }
        case 1577: { goto _1577; }
        case 1578: { goto _1578; }
        case 1579: { goto _1579; }
        case 1580: { goto _1580; }
        case 1581: { goto _1581; }
        case 1582: { goto _1582; }
        case 1583: { goto _1583; }
        case 1584: { goto _1584; }
        case 1585: { goto _1585; }
        case 1586: { goto _1586; }
        case 1587: { goto _1587; }
        case 1588: { goto _1588; }
        case 1589: { goto _1589; }
        case 1590: { goto _1590; }
        case 1591: { goto _1591; }
        case 1592: { goto _1592; }
        case 1593: { goto _1593; }
        case 1594: { goto _1594; }
        case 1595: { goto _1595; }
        case 1596: { goto _1596; }
        case 1597: { goto _1597; }
        case 1598: { goto _1598; }
        case 1599: { goto _1599; }
        case 1600: { goto _1600; }
        case 1601: { goto _1601; }
        case 1602: { goto _1602; }
        case 1603: { goto _1603; }
        case 1604: { goto _1604; }
        case 1605: { goto _1605; }
        case 1606: { goto _1606; }
        case 1607: { goto _1607; }
        case 1608: { goto _1608; }
        case 1609: { goto _1609; }
        case 1610: { goto _1610; }
        case 1611: { goto _1611; }
        case 1612: { goto _1612; }
        case 1613: { goto _1613; }
        case 1614: { goto _1614; }
        case 1615: { goto _1615; }
        case 1616: { goto _1616; }
        case 1617: { goto _1617; }
        case 1618: { goto _1618; }
        case 1619: { goto _1619; }
        case 1620: { goto _1620; }
        case 1621: { goto _1621; }
        case 1622: { goto _1622; }
        case 1623: { goto _1623; }
        case 1624: { goto _1624; }
        case 1625: { goto _1625; }
        case 1626: { goto _1626; }
        case 1627: { goto _1627; }
        case 1628: { goto _1628; }
        case 1629: { goto _1629; }
        case 1630: { goto _1630; }
        case 1631: { goto _1631; }
        case 1632: { goto _1632; }
        case 1633: { goto _1633; }
        case 1634: { goto _1634; }
        case 1635: { goto _1635; }
        case 1636: { goto _1636; }
        case 1637: { goto _1637; }
        case 1638: { goto _1638; }
        case 1639: { goto _1639; }
        case 1640: { goto _1640; }
        case 1641: { goto _1641; }
        case 1642: { goto _1642; }
        case 1643: { goto _1643; }
        case 1644: { goto _1644; }
        case 1645: { goto _1645; }
        case 1647: { goto _1647; }
        case 1650: { goto _1650; }
        case 1651: { goto _1651; }
        case 1652: { goto _1652; }
        case 1653: { goto _1653; }
        case 1654: { goto _1654; }
        case 1655: { goto _1655; }
        case 1656: { goto _1656; }
        case 1658: { goto _1658; }
        case 1659: { goto _1659; }
        case 1660: { goto _1660; }
        case 1661: { goto _1661; }
        case 1662: { goto _1662; }
        case 1663: { goto _1663; }
        case 1664: { goto _1664; }
        case 1665: { goto _1665; }
        case 1666: { goto _1666; }
        case 1667: { goto _1667; }
        case 1668: { goto _1668; }
        case 1669: { goto _1669; }
        case 1670: { goto _1670; }
        case 1671: { goto _1671; }
        case 1673: { goto _1673; }
        case 1674: { goto _1674; }
        case 1675: { goto _1675; }
        case 1676: { goto _1676; }
        case 1677: { goto _1677; }
        case 1678: { goto _1678; }
        case 1679: { goto _1679; }
        case 1680: { goto _1680; }
        case 1681: { goto _1681; }
        case 1682: { goto _1682; }
        case 1683: { goto _1683; }
        case 1684: { goto _1684; }
        case 1685: { goto _1685; }
        case 1686: { goto _1686; }
        case 1687: { goto _1687; }
        case 1688: { goto _1688; }
        case 1689: { goto _1689; }
        case 1691: { goto _1691; }
        case 1692: { goto _1692; }
        case 1693: { goto _1693; }
        case 1694: { goto _1694; }
        case 1695: { goto _1695; }
        case 1696: { goto _1696; }
        case 1697: { goto _1697; }
        case 1698: { goto _1698; }
        case 1699: { goto _1699; }
        case 1700: { goto _1700; }
        case 1701: { goto _1701; }
        case 1702: { goto _1702; }
        case 1703: { goto _1703; }
        case 1704: { goto _1704; }
        case 1706: { goto _1706; }
        case 1707: { goto _1707; }
        case 1708: { goto _1708; }
        case 1709: { goto _1709; }
        case 1710: { goto _1710; }
        case 1711: { goto _1711; }
        case 1712: { goto _1712; }
        case 1713: { goto _1713; }
        case 1714: { goto _1714; }
        case 1715: { goto _1715; }
        case 1716: { goto _1716; }
        case 1717: { goto _1717; }
        case 1718: { goto _1718; }
        case 1719: { goto _1719; }
        case 1720: { goto _1720; }
        case 1721: { goto _1721; }
        case 1722: { goto _1722; }
        case 1723: { goto _1723; }
        case 1725: { goto _1725; }
        case 1726: { goto _1726; }
        case 1727: { goto _1727; }
        case 1728: { goto _1728; }
        case 1729: { goto _1729; }
        case 1730: { goto _1730; }
        case 1732: { goto _1732; }
        case 1733: { goto _1733; }
        case 1734: { goto _1734; }
        case 1735: { goto _1735; }
        case 1736: { goto _1736; }
        case 1737: { goto _1737; }
        case 1738: { goto _1738; }
        case 1739: { goto _1739; }
        case 1740: { goto _1740; }
        case 1741: { goto _1741; }
        case 1742: { goto _1742; }
        case 1743: { goto _1743; }
        case 1744: { goto _1744; }
        case 1745: { goto _1745; }
        case 1746: { goto _1746; }
        case 1748: { goto _1748; }
        case 1749: { goto _1749; }
        case 1750: { goto _1750; }
        case 1751: { goto _1751; }
        case 1752: { goto _1752; }
        case 1753: { goto _1753; }
        case 1762: { goto _1762; }
        case 1763: { goto _1763; }
        case 1764: { goto _1764; }
        case 1765: { goto _1765; }
        case 1766: { goto _1766; }
        case 1767: { goto _1767; }
        case 1768: { goto _1768; }
        case 1769: { goto _1769; }
        case 1770: { goto _1770; }
        case 1771: { goto _1771; }
        case 1772: { goto _1772; }
        case 1773: { goto _1773; }
        case 1774: { goto _1774; }
        case 1775: { goto _1775; }
        case 1776: { goto _1776; }
        case 1777: { goto _1777; }
        case 1778: { goto _1778; }
        case 1779: { goto _1779; }
        case 1780: { goto _1780; }
        case 1781: { goto _1781; }
        case 1782: { goto _1782; }
        case 1783: { goto _1783; }
        case 1784: { goto _1784; }
        case 1786: { goto _1786; }
        case 1787: { goto _1787; }
        case 1788: { goto _1788; }
        case 1789: { goto _1789; }
        case 1790: { goto _1790; }
        case 1791: { goto _1791; }
        case 1792: { goto _1792; }
        case 1793: { goto _1793; }
        case 1794: { goto _1794; }
        case 1795: { goto _1795; }
        case 1796: { goto _1796; }
        case 1797: { goto _1797; }
        case 1798: { goto _1798; }
        case 1799: { goto _1799; }
        case 1801: { goto _1801; }
        case 1802: { goto _1802; }
        case 1803: { goto _1803; }
        case 1804: { goto _1804; }
        case 1805: { goto _1805; }
        case 1806: { goto _1806; }
        case 1807: { goto _1807; }
        case 1808: { goto _1808; }
        case 1809: { goto _1809; }
        case 1810: { goto _1810; }
        case 1811: { goto _1811; }
        case 1812: { goto _1812; }
        case 1813: { goto _1813; }
        case 1814: { goto _1814; }
        case 1815: { goto _1815; }
        case 1816: { goto _1816; }
        case 1817: { goto _1817; }
        case 1818: { goto _1818; }
        case 1819: { goto _1819; }
        case 1820: { goto _1820; }
        case 1821: { goto _1821; }
        case 1823: { goto _1823; }
        case 1824: { goto _1824; }
        case 1825: { goto _1825; }
        case 1826: { goto _1826; }
        case 1827: { goto _1827; }
        case 1828: { goto _1828; }
        case 1829: { goto _1829; }
        case 1830: { goto _1830; }
        case 1831: { goto _1831; }
        case 1832: { goto _1832; }
        case 1833: { goto _1833; }
        case 1834: { goto _1834; }
        case 1835: { goto _1835; }
        case 1836: { goto _1836; }
        case 1837: { goto _1837; }
        case 1838: { goto _1838; }
        case 1840: { goto _1840; }
        case 1841: { goto _1841; }
        case 1843: { goto _1843; }
        case 1844: { goto _1844; }
        case 1845: { goto _1845; }
        case 1846: { goto _1846; }
        case 1847: { goto _1847; }
        case 1848: { goto _1848; }
        case 1849: { goto _1849; }
        case 1850: { goto _1850; }
        case 1851: { goto _1851; }
        case 1852: { goto _1852; }
        case 1854: { goto _1854; }
        case 1855: { goto _1855; }
        case 1856: { goto _1856; }
        case 1857: { goto _1857; }
        case 1858: { goto _1858; }
        case 1859: { goto _1859; }
        case 1860: { goto _1860; }
        case 1861: { goto _1861; }
        case 1862: { goto _1862; }
        case 1863: { goto _1863; }
        case 1864: { goto _1864; }
        case 1866: { goto _1866; }
        case 1867: { goto _1867; }
        case 1868: { goto _1868; }
        case 1869: { goto _1869; }
        case 1870: { goto _1870; }
        case 1871: { goto _1871; }
        case 1872: { goto _1872; }
        case 1873: { goto _1873; }
        case 1874: { goto _1874; }
        case 1875: { goto _1875; }
        case 1876: { goto _1876; }
        case 1877: { goto _1877; }
        case 1878: { goto _1878; }
        case 1879: { goto _1879; }
        case 1880: { goto _1880; }
        case 1881: { goto _1881; }

        default:
            __QUEX_STD_fprintf(stderr, "State router: index = %i\n", (int)target_state_index);
            QUEX_ERROR_EXIT("State router: unknown index.\n");
    }
#   endif /* QUEX_OPTION_COMPUTED_GOTOS */

    /* Prevent compiler warning 'unused variable': use variables once in a part of the code*/
    /* that is never reached (and deleted by the compiler anyway).*/
    (void)QUEX_LEXEME_NULL;
    (void)QUEX_NAME_TOKEN(DumpedTokenIdObject);
    QUEX_ERROR_EXIT("Unreachable code has been reached.\n");
#   undef ReplacementRules
#   undef self
}
#include <quex/code_base/temporary_macros_off>
QUEX_NAMESPACE_MAIN_CLOSE


QUEX_NAMESPACE_TOKEN_OPEN

const char*
QUEX_NAME_TOKEN(map_id_to_name)(const QUEX_TYPE_TOKEN_ID TokenID)
{
   static char  error_string[64];
   static const char  uninitialized_string[] = "<UNINITIALIZED>";
   static const char  termination_string[]   = "<TERMINATION>";
#  if defined(QUEX_OPTION_INDENTATION_TRIGGER)
   static const char  indent_string[]        = "<INDENT>";
   static const char  dedent_string[]        = "<DEDENT>";
   static const char  nodent_string[]        = "<NODENT>";
#  endif
   static const char  token_id_str_BALL[]          = "BALL";
   static const char  token_id_str_CYAN_TEAM[]     = "CYAN_TEAM";
   static const char  token_id_str_FIELD[]         = "FIELD";
   static const char  token_id_str_GOAL[]          = "GOAL";
   static const char  token_id_str_LINE[]          = "LINE";
   static const char  token_id_str_MAGENTA_TEAM[]  = "MAGENTA_TEAM";
   static const char  token_id_str_UNCLASSIFIED[]  = "UNCLASSIFIED";
       

   /* NOTE: This implementation works only for token id types that are 
    *       some type of integer or enum. In case an alien type is to
    *       used, this function needs to be redefined.                  */
   switch( TokenID ) {
   default: {
       __QUEX_STD_sprintf(error_string, "<UNKNOWN TOKEN-ID: %i>", (int)TokenID);
       return error_string;
   }
   case QUEX_TKN_TERMINATION:    return termination_string;
   case QUEX_TKN_UNINITIALIZED:  return uninitialized_string;
#  if defined(QUEX_OPTION_INDENTATION_TRIGGER)
   case QUEX_TKN_INDENT:         return indent_string;
   case QUEX_TKN_DEDENT:         return dedent_string;
   case QUEX_TKN_NODENT:         return nodent_string;
#  endif
   case QUEX_TKN_BALL:          return token_id_str_BALL;
   case QUEX_TKN_CYAN_TEAM:     return token_id_str_CYAN_TEAM;
   case QUEX_TKN_FIELD:         return token_id_str_FIELD;
   case QUEX_TKN_GOAL:          return token_id_str_GOAL;
   case QUEX_TKN_LINE:          return token_id_str_LINE;
   case QUEX_TKN_MAGENTA_TEAM:  return token_id_str_MAGENTA_TEAM;
   case QUEX_TKN_UNCLASSIFIED:  return token_id_str_UNCLASSIFIED;

   }
}

QUEX_NAMESPACE_TOKEN_CLOSE

