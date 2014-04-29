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
            continue;/* ['\v', 'ÿ'] */

        } else if( (*(iterator)) == 0xA ) {
            __QUEX_IF_COUNT_LINES_ADD((size_t)1);
        __QUEX_IF_COUNT_COLUMNS_SET((size_t)1);
        __QUEX_IF_COUNT_COLUMNS(reference_p = (iterator) + 1);
                ++(((iterator)));
            continue;/* '\n' */

        } else if( (*(iterator)) == 0x9 ) {
                    __QUEX_IF_COUNT_COLUMNS_ADD((size_t)(((iterator) - reference_p)));
        __QUEX_IF_COUNT_COLUMNS(self.counter._column_number_at_end &= ~ ((size_t)0x3));
        __QUEX_IF_COUNT_COLUMNS(self.counter._column_number_at_end += 4);
        __QUEX_IF_COUNT_COLUMNS(reference_p = (iterator) + 1);
                ++(((iterator)));
            continue;/* '\t' */

        } else {
                            ++(((iterator)));
            continue;/* [\0, '\b'] */

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
    const QUEX_TYPE_GOTO_LABEL     (template_3078_target_1[26])   = { QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3219), QUEX_LABEL(3227), QUEX_LABEL(3221), QUEX_LABEL(3228), QUEX_LABEL(3208), QUEX_LABEL(3208), QUEX_LABEL(3208), QUEX_LABEL(3208), QUEX_LABEL(3226), QUEX_LABEL(3179), QUEX_LABEL(3179), QUEX_LABEL(3179), QUEX_LABEL(3179), QUEX_LABEL(3185)  };
    QUEX_TYPE_CHARACTER_POSITION   position[1]                    = { 0};
    const size_t                   PositionRegisterN              = (size_t)1;
    QUEX_TYPE_CHARACTER            input                          = (QUEX_TYPE_CHARACTER)(0x00);
    QUEX_TYPE_GOTO_LABEL           target_state_index             = QUEX_GOTO_LABEL_VOID;
    const QUEX_TYPE_GOTO_LABEL     (template_3067_target_0[246])  = { QUEX_LABEL(3412), QUEX_LABEL(3413), QUEX_LABEL(3290), QUEX_LABEL(3289), QUEX_LABEL(3288), QUEX_LABEL(3287), QUEX_LABEL(3286), QUEX_LABEL(3285), QUEX_LABEL(3284), QUEX_LABEL(3283), QUEX_LABEL(3282), QUEX_LABEL(3281), QUEX_LABEL(3280), QUEX_LABEL(3279), QUEX_LABEL(3278), QUEX_LABEL(3277), QUEX_LABEL(3276), QUEX_LABEL(3275), QUEX_LABEL(3274), QUEX_LABEL(3273), QUEX_LABEL(3272), QUEX_LABEL(3271), QUEX_LABEL(3270), QUEX_LABEL(3269), QUEX_LABEL(3268), QUEX_LABEL(3267), QUEX_LABEL(3266), QUEX_LABEL(3265), QUEX_LABEL(3264), QUEX_LABEL(3263), QUEX_LABEL(3262), QUEX_LABEL(3261), QUEX_LABEL(3245), QUEX_LABEL(3244), QUEX_LABEL(3243), QUEX_LABEL(3242), QUEX_LABEL(3241), QUEX_LABEL(3240), QUEX_LABEL(3239), QUEX_LABEL(3238), QUEX_LABEL(3237), QUEX_LABEL(3236), QUEX_LABEL(3235), QUEX_LABEL(3429), QUEX_LABEL(3430), QUEX_LABEL(3428), QUEX_LABEL(3427), QUEX_LABEL(3426), QUEX_LABEL(3425), QUEX_LABEL(3424), QUEX_LABEL(3423), QUEX_LABEL(3422), QUEX_LABEL(3421), QUEX_LABEL(3420), QUEX_LABEL(3419), QUEX_LABEL(3418), QUEX_LABEL(3417), QUEX_LABEL(3416), QUEX_LABEL(3415), QUEX_LABEL(3414), QUEX_LABEL(3433), QUEX_LABEL(3434), QUEX_LABEL(3432), QUEX_LABEL(3431), QUEX_LABEL(3403), QUEX_LABEL(3402), QUEX_LABEL(3401), QUEX_LABEL(3400), QUEX_LABEL(3399), QUEX_LABEL(3398), QUEX_LABEL(3397), QUEX_LABEL(3396), QUEX_LABEL(3395), QUEX_LABEL(3394), QUEX_LABEL(3393), QUEX_LABEL(3392), QUEX_LABEL(3374), QUEX_LABEL(3373), QUEX_LABEL(3364), QUEX_LABEL(3363), QUEX_LABEL(3362), QUEX_LABEL(3361), QUEX_LABEL(3360), QUEX_LABEL(3359), QUEX_LABEL(3358), QUEX_LABEL(3357), QUEX_LABEL(3356), QUEX_LABEL(3355), QUEX_LABEL(3354), QUEX_LABEL(3339), QUEX_LABEL(3338), QUEX_LABEL(3337), QUEX_LABEL(3336), QUEX_LABEL(3335), QUEX_LABEL(3317), QUEX_LABEL(3316), QUEX_LABEL(3298), QUEX_LABEL(3067), QUEX_LABEL(3297), QUEX_LABEL(3296), QUEX_LABEL(3295), QUEX_LABEL(3294), QUEX_LABEL(3293), QUEX_LABEL(3292), QUEX_LABEL(3291), QUEX_LABEL(3067), QUEX_LABEL(3411), QUEX_LABEL(3067), QUEX_LABEL(3410), QUEX_LABEL(3409), QUEX_LABEL(3408), QUEX_LABEL(3407), QUEX_LABEL(3406), QUEX_LABEL(3405), QUEX_LABEL(3404), QUEX_LABEL(3067), QUEX_LABEL(3372), QUEX_LABEL(3371), QUEX_LABEL(3370), QUEX_LABEL(3369), QUEX_LABEL(3368), QUEX_LABEL(3367), QUEX_LABEL(3366), QUEX_LABEL(3365), QUEX_LABEL(3448), QUEX_LABEL(3067), QUEX_LABEL(3447), QUEX_LABEL(3446), QUEX_LABEL(3445), QUEX_LABEL(3444), QUEX_LABEL(3443), QUEX_LABEL(3442), QUEX_LABEL(3441), QUEX_LABEL(3440), QUEX_LABEL(3439), QUEX_LABEL(3438), QUEX_LABEL(3437), QUEX_LABEL(3436), QUEX_LABEL(3435), QUEX_LABEL(3067), QUEX_LABEL(3353), QUEX_LABEL(3352), QUEX_LABEL(3351), QUEX_LABEL(3350), QUEX_LABEL(3349), QUEX_LABEL(3348), QUEX_LABEL(3347), QUEX_LABEL(3346), QUEX_LABEL(3345), QUEX_LABEL(3344), QUEX_LABEL(3343), QUEX_LABEL(3342), QUEX_LABEL(3341), QUEX_LABEL(3340), QUEX_LABEL(3067), QUEX_LABEL(3260), QUEX_LABEL(3259), QUEX_LABEL(3258), QUEX_LABEL(3257), QUEX_LABEL(3256), QUEX_LABEL(3255), QUEX_LABEL(3254), QUEX_LABEL(3253), QUEX_LABEL(3252), QUEX_LABEL(3251), QUEX_LABEL(3250), QUEX_LABEL(3249), QUEX_LABEL(3248), QUEX_LABEL(3247), QUEX_LABEL(3465), QUEX_LABEL(3067), QUEX_LABEL(3464), QUEX_LABEL(3463), QUEX_LABEL(3462), QUEX_LABEL(3461), QUEX_LABEL(3460), QUEX_LABEL(3459), QUEX_LABEL(3458), QUEX_LABEL(3457), QUEX_LABEL(3456), QUEX_LABEL(3455), QUEX_LABEL(3454), QUEX_LABEL(3453), QUEX_LABEL(3452), QUEX_LABEL(3451), QUEX_LABEL(3450), QUEX_LABEL(3449), QUEX_LABEL(3067), QUEX_LABEL(3391), QUEX_LABEL(3390), QUEX_LABEL(3389), QUEX_LABEL(3388), QUEX_LABEL(3387), QUEX_LABEL(3386), QUEX_LABEL(3385), QUEX_LABEL(3384), QUEX_LABEL(3383), QUEX_LABEL(3382), QUEX_LABEL(3381), QUEX_LABEL(3380), QUEX_LABEL(3379), QUEX_LABEL(3378), QUEX_LABEL(3377), QUEX_LABEL(3376), QUEX_LABEL(3375), QUEX_LABEL(3067), QUEX_LABEL(3334), QUEX_LABEL(3333), QUEX_LABEL(3332), QUEX_LABEL(3331), QUEX_LABEL(3330), QUEX_LABEL(3329), QUEX_LABEL(3328), QUEX_LABEL(3327), QUEX_LABEL(3326), QUEX_LABEL(3325), QUEX_LABEL(3324), QUEX_LABEL(3323), QUEX_LABEL(3322), QUEX_LABEL(3321), QUEX_LABEL(3320), QUEX_LABEL(3319), QUEX_LABEL(3318), QUEX_LABEL(3067), QUEX_LABEL(3315), QUEX_LABEL(3314), QUEX_LABEL(3313), QUEX_LABEL(3312), QUEX_LABEL(3311), QUEX_LABEL(3310), QUEX_LABEL(3309), QUEX_LABEL(3308), QUEX_LABEL(3307), QUEX_LABEL(3306), QUEX_LABEL(3305), QUEX_LABEL(3304), QUEX_LABEL(3303), QUEX_LABEL(3302), QUEX_LABEL(3301), QUEX_LABEL(3300), QUEX_LABEL(3299), QUEX_LABEL(3067), QUEX_LABEL(3067), QUEX_LABEL(3142), QUEX_LABEL(3143), QUEX_LABEL(3246)  };
    ptrdiff_t                      state_key                      = (ptrdiff_t)0;
    const QUEX_TYPE_GOTO_LABEL     (template_3078_target_0[26])   = { QUEX_LABEL(3223), QUEX_LABEL(3224), QUEX_LABEL(3222), QUEX_LABEL(3220), QUEX_LABEL(3232), QUEX_LABEL(3078), QUEX_LABEL(3231), QUEX_LABEL(3230), QUEX_LABEL(3229), QUEX_LABEL(3216), QUEX_LABEL(3217), QUEX_LABEL(3078), QUEX_LABEL(3216), QUEX_LABEL(3217), QUEX_LABEL(3216), QUEX_LABEL(3217), QUEX_LABEL(3215), QUEX_LABEL(3225), QUEX_LABEL(3214), QUEX_LABEL(3210), QUEX_LABEL(3078), QUEX_LABEL(3180), QUEX_LABEL(3180), QUEX_LABEL(3180), QUEX_LABEL(3180), QUEX_LABEL(3186)  };
    const QUEX_TYPE_GOTO_LABEL     (template_3079_target_1[2])    = { QUEX_LABEL(3079), QUEX_LABEL(3234)  };
    const QUEX_TYPE_GOTO_LABEL     (template_3079_target_0[2])    = { QUEX_LABEL(3234), QUEX_LABEL(3079)  };
    QUEX_TYPE_GOTO_LABEL           target_state_else_index        = QUEX_GOTO_LABEL_VOID;
    const QUEX_TYPE_GOTO_LABEL     (template_3076_target_0[21])   = { QUEX_LABEL(3178), QUEX_LABEL(3178), QUEX_LABEL(3178), QUEX_LABEL(3178), QUEX_LABEL(3178), QUEX_LABEL(3178), QUEX_LABEL(3178), QUEX_LABEL(3178), QUEX_LABEL(3178), QUEX_LABEL(3178), QUEX_LABEL(3178), QUEX_LABEL(3178), QUEX_LABEL(3164), QUEX_LABEL(3167), QUEX_LABEL(3168), QUEX_LABEL(3173), QUEX_LABEL(3157), QUEX_LABEL(3157), QUEX_LABEL(3157), QUEX_LABEL(3157), QUEX_LABEL(3166)  };
    const QUEX_TYPE_GOTO_LABEL     (template_3076_target_1[21])   = { QUEX_LABEL(3176), QUEX_LABEL(3177), QUEX_LABEL(3175), QUEX_LABEL(3174), QUEX_LABEL(3171), QUEX_LABEL(3172), QUEX_LABEL(3170), QUEX_LABEL(3169), QUEX_LABEL(3076), QUEX_LABEL(3076), QUEX_LABEL(3163), QUEX_LABEL(3162), QUEX_LABEL(3162), QUEX_LABEL(3163), QUEX_LABEL(3163), QUEX_LABEL(3162), QUEX_LABEL(3161), QUEX_LABEL(3165), QUEX_LABEL(3160), QUEX_LABEL(3158), QUEX_LABEL(3076)  };
    const QUEX_TYPE_GOTO_LABEL     (template_3077_target_1[28])   = { QUEX_LABEL(3205), QUEX_LABEL(3206), QUEX_LABEL(3204), QUEX_LABEL(3202), QUEX_LABEL(3191), QUEX_LABEL(3192), QUEX_LABEL(3190), QUEX_LABEL(3189), QUEX_LABEL(3197), QUEX_LABEL(3077), QUEX_LABEL(3196), QUEX_LABEL(3195), QUEX_LABEL(3193), QUEX_LABEL(3077), QUEX_LABEL(3077), QUEX_LABEL(3199), QUEX_LABEL(3181), QUEX_LABEL(3182), QUEX_LABEL(3077), QUEX_LABEL(3201), QUEX_LABEL(3200), QUEX_LABEL(3181), QUEX_LABEL(3182), QUEX_LABEL(3188), QUEX_LABEL(3184), QUEX_LABEL(3182), QUEX_LABEL(3181), QUEX_LABEL(3198)  };
    const QUEX_TYPE_GOTO_LABEL     (template_3077_target_0[28])   = { QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3183), QUEX_LABEL(3187), QUEX_LABEL(3207), QUEX_LABEL(3207), QUEX_LABEL(3194), QUEX_LABEL(3203), QUEX_LABEL(3207)  };
    const QUEX_TYPE_GOTO_LABEL     (template_3078_target_2[26])   = { QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3233), QUEX_LABEL(3213), QUEX_LABEL(3218), QUEX_LABEL(3212), QUEX_LABEL(3211), QUEX_LABEL(3078)  };
    const QUEX_TYPE_GOTO_LABEL     (template_3075_target_1[51])   = { QUEX_LABEL(3130), QUEX_LABEL(3139), QUEX_LABEL(3141), QUEX_LABEL(3155), QUEX_LABEL(3156), QUEX_LABEL(3141), QUEX_LABEL(3114), QUEX_LABEL(3114), QUEX_LABEL(3136), QUEX_LABEL(3132), QUEX_LABEL(3118), QUEX_LABEL(3141), QUEX_LABEL(3141), QUEX_LABEL(3133), QUEX_LABEL(3137), QUEX_LABEL(3126), QUEX_LABEL(3119), QUEX_LABEL(3141), QUEX_LABEL(3141), QUEX_LABEL(3141), QUEX_LABEL(3115), QUEX_LABEL(3115), QUEX_LABEL(3138), QUEX_LABEL(3141), QUEX_LABEL(3141), QUEX_LABEL(3123), QUEX_LABEL(3135), QUEX_LABEL(3131), QUEX_LABEL(3120), QUEX_LABEL(3117), QUEX_LABEL(3091), QUEX_LABEL(3091), QUEX_LABEL(3091), QUEX_LABEL(3091), QUEX_LABEL(3091), QUEX_LABEL(3091), QUEX_LABEL(3091), QUEX_LABEL(3091), QUEX_LABEL(3113), QUEX_LABEL(3094), QUEX_LABEL(3095), QUEX_LABEL(3093), QUEX_LABEL(3093), QUEX_LABEL(3093), QUEX_LABEL(3093), QUEX_LABEL(3093), QUEX_LABEL(3093), QUEX_LABEL(3094), QUEX_LABEL(3094), QUEX_LABEL(3095), QUEX_LABEL(3075)  };
    const QUEX_TYPE_GOTO_LABEL     (template_3075_target_0[51])   = { QUEX_LABEL(3142), QUEX_LABEL(3143), QUEX_LABEL(3129), QUEX_LABEL(3142), QUEX_LABEL(3143), QUEX_LABEL(3144), QUEX_LABEL(3125), QUEX_LABEL(3145), QUEX_LABEL(3146), QUEX_LABEL(3147), QUEX_LABEL(3148), QUEX_LABEL(3140), QUEX_LABEL(3124), QUEX_LABEL(3149), QUEX_LABEL(3150), QUEX_LABEL(3145), QUEX_LABEL(3148), QUEX_LABEL(3151), QUEX_LABEL(3152), QUEX_LABEL(3127), QUEX_LABEL(3145), QUEX_LABEL(3149), QUEX_LABEL(3150), QUEX_LABEL(3128), QUEX_LABEL(3148), QUEX_LABEL(3122), QUEX_LABEL(3153), QUEX_LABEL(3154), QUEX_LABEL(3121), QUEX_LABEL(3148), QUEX_LABEL(3112), QUEX_LABEL(3134), QUEX_LABEL(3111), QUEX_LABEL(3110), QUEX_LABEL(3109), QUEX_LABEL(3108), QUEX_LABEL(3107), QUEX_LABEL(3106), QUEX_LABEL(3075), QUEX_LABEL(3097), QUEX_LABEL(3096), QUEX_LABEL(3105), QUEX_LABEL(3104), QUEX_LABEL(3103), QUEX_LABEL(3102), QUEX_LABEL(3101), QUEX_LABEL(3100), QUEX_LABEL(3099), QUEX_LABEL(3098), QUEX_LABEL(3116), QUEX_LABEL(3141)  };
#   ifndef QUEX_OPTION_COMPUTED_GOTOS
#   endif /* QUEX_OPTION_COMPUTED_GOTOS */

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
    /* BEGIN: STATE MACHINE
     * init-state = 2337L
     * 02337 (1812, 63), (1813, 173), (1814, 364), (1815, 512), (1816, 638), (1817, 847), (1818, 1052), (1819, 1146), (1820, 1213), (1821, 1290), (1822, 1373), (1823, 1452), (1824, 1535), (1825, 1618), (1826, 1697), (1827, 1719), (1828, 1734), (1829, 1749), (1830, 1764), (1831, 1779), (1832, 1794), (1833, 1809)
     *      == 'c' ==> 02341
     *      == 'g' ==> 02340
     *      == 'o' ==> 02339
     *      == 'p' ==> 02338
     *      == 'u' ==> 02342
     *      == 'w' ==> 02343
     *      == 'y' ==> 02344
     *     
     * 02341 (1832, 1795, A)
     *      == 'c' ==> 02341
     *     
     * 02340 (1812, 64), (1813, 174), (1814, 365), (1815, 513), (1816, 639), (1817, 848), (1818, 1053), (1828, 1736, A)
     *      == 'g' ==> 02397
     *      == 'u' ==> 02398
     *     
     * 02397 (1812, 65), (1813, 175), (1814, 366), (1815, 513), (1816, 640), (1817, 849), (1818, 1054), (1828, 1736, A)
     *      == 'g' ==> 02414
     *      == 'u' ==> 02398
     *     
     * 02414 (1812, 66), (1813, 176), (1814, 367), (1815, 513), (1816, 641), (1817, 850), (1818, 1055), (1828, 1736, A)
     *      == 'g' ==> 02416
     *      == 'u' ==> 02415
     *     
     * 02416 (1812, 66), (1813, 177), (1814, 368), (1815, 513), (1816, 642), (1817, 850), (1818, 1056), (1828, 1736, A)
     *      == 'g' ==> 02417
     *      == 'u' ==> 02415
     *     
     * 02417 (1812, 66), (1813, 178), (1814, 369), (1815, 513), (1816, 643), (1817, 850), (1818, 1057), (1828, 1736, A)
     *      == 'g' ==> 02418
     *      == 'u' ==> 02415
     *     
     * 02418 (1812, 66), (1813, 179), (1814, 370), (1815, 513), (1816, 644), (1817, 850), (1818, 1058), (1828, 1736, A)
     *      == 'g' ==> 02419
     *      == 'u' ==> 02420
     *     
     * 02419 (1812, 66), (1813, 179), (1814, 371), (1815, 513), (1816, 645), (1817, 850), (1818, 1059), (1828, 1736, A)
     *      == 'g' ==> 02507
     *      == 'u' ==> 02420
     *     
     * 02507 (1812, 66), (1813, 179), (1814, 372), (1815, 513), (1816, 646), (1817, 850), (1818, 1060), (1828, 1736, A)
     *      == 'g' ==> 02508
     *      == 'u' ==> 02420
     *     
     * 02508 (1812, 66), (1813, 179), (1814, 373), (1815, 513), (1816, 647), (1817, 850), (1818, 1061), (1828, 1736, A)
     *      == 'g' ==> 02509
     *      == 'u' ==> 02420
     *     
     * 02509 (1812, 66), (1813, 179), (1814, 374), (1815, 513), (1816, 648), (1817, 850), (1818, 1062), (1828, 1736, A)
     *      == 'g' ==> 02510
     *      == 'u' ==> 02420
     *     
     * 02510 (1812, 66), (1813, 179), (1814, 375), (1815, 513), (1816, 649), (1817, 850), (1818, 1063), (1828, 1736, A)
     *      == 'g' ==> 02511
     *      == 'u' ==> 02420
     *     
     * 02511 (1812, 66), (1813, 179), (1814, 376), (1815, 513), (1816, 650), (1817, 850), (1818, 1064), (1828, 1736, A)
     *      == 'g' ==> 02512
     *      == 'u' ==> 02513
     *     
     * 02512 (1812, 66), (1813, 179), (1814, 376), (1815, 513), (1816, 650), (1817, 850), (1818, 1065), (1828, 1736, A)
     *      == 'g' ==> 02577
     *      == 'u' ==> 02513
     *     
     * 02577 (1812, 66), (1813, 179), (1814, 376), (1815, 513), (1816, 650), (1817, 850), (1818, 1066), (1828, 1736, A)
     *      == 'g' ==> 02578
     *      == 'u' ==> 02513
     *     
     * 02578 (1812, 66), (1813, 179), (1814, 376), (1815, 513), (1816, 650), (1817, 850), (1818, 1067), (1828, 1736, A)
     *      == 'g' ==> 02579
     *      == 'u' ==> 02513
     *     
     * 02579 (1812, 66), (1813, 179), (1814, 376), (1815, 513), (1816, 650), (1817, 850), (1818, 1068), (1828, 1736, A)
     *      == 'g' ==> 02580
     *      == 'u' ==> 02513
     *     
     * 02580 (1812, 66), (1813, 179), (1814, 376), (1815, 513), (1816, 650), (1817, 850), (1818, 1069), (1828, 1736, A)
     *      == 'g' ==> 02581
     *      == 'u' ==> 02513
     *     
     * 02581 (1812, 66), (1813, 179), (1814, 376), (1815, 513), (1816, 650), (1817, 850), (1818, 1070), (1828, 1736, A)
     *      == 'g' ==> 02582
     *      == 'u' ==> 02513
     *     
     * 02582 (1812, 66), (1813, 179), (1814, 376), (1815, 513), (1816, 650), (1817, 850), (1818, 1071), (1828, 1736, A)
     *      == 'g' ==> 02583
     *      == 'u' ==> 02513
     *     
     * 02583 (1812, 66), (1813, 179), (1814, 376), (1815, 513), (1816, 650), (1817, 850), (1818, 1072), (1828, 1736, A)
     *      == 'g' ==> 02583
     *      == 'u' ==> 02584
     *     
     * 02584 (1812, 67), (1813, 180), (1814, 377), (1815, 514), (1816, 651), (1817, 851), (1818, 1073)
     *      == 'g' ==> 02586
     *      == 'u' ==> 02585
     *     
     * 02586 (1812, 68), (1813, 182), (1814, 379), (1815, 516), (1816, 652, A), (1817, 852), (1818, 1074)
     *      == 'g' ==> 02587
     *     
     * 02587 (1812, 69), (1813, 183), (1814, 380), (1815, 517), (1816, 652, A), (1817, 859), (1818, 1081)
     *      == 'g' ==> 02588
     *     
     * 02588 (1812, 70, A), (1813, 184), (1814, 381), (1815, 518), (1816, 652, A), (1817, 860), (1818, 1082, A)
     *      == 'g' ==> 02589
     *     
     * 02589 (1812, 70, A), (1813, 185), (1814, 382), (1815, 519), (1816, 652, A), (1817, 861), (1818, 1082, A)
     *      == 'g' ==> 02590
     *     
     * 02590 (1812, 70, A), (1813, 186), (1814, 383), (1815, 520), (1816, 652, A), (1817, 862), (1818, 1082, A)
     *      == 'g' ==> 02591
     *     
     * 02591 (1812, 70, A), (1813, 187, A), (1814, 384), (1815, 521), (1816, 652, A), (1817, 863), (1818, 1082, A)
     *      == 'g' ==> 02592
     *     
     * 02592 (1812, 70, A), (1813, 187, A), (1814, 385), (1815, 522), (1816, 652, A), (1817, 864), (1818, 1082, A)
     *      == 'g' ==> 02593
     *     
     * 02593 (1812, 70, A), (1813, 187, A), (1814, 386), (1815, 523), (1816, 652, A), (1817, 865), (1818, 1082, A)
     *      == 'g' ==> 02594
     *     
     * 02594 (1812, 70, A), (1813, 187, A), (1814, 387), (1815, 524), (1816, 652, A), (1817, 866), (1818, 1082, A)
     *      == 'g' ==> 02595
     *     
     * 02595 (1812, 70, A), (1813, 187, A), (1814, 388), (1815, 525), (1816, 652, A), (1817, 867), (1818, 1082, A)
     *      == 'g' ==> 02596
     *     
     * 02596 (1812, 70, A), (1813, 187, A), (1814, 389), (1815, 526), (1816, 652, A), (1817, 868), (1818, 1082, A)
     *      == 'g' ==> 02597
     *     
     * 02597 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 869), (1818, 1082, A)
     *      == 'g' ==> 02598
     *     
     * 02598 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 870), (1818, 1082, A)
     *      == 'g' ==> 02599
     *     
     * 02599 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 871), (1818, 1082, A)
     *      == 'g' ==> 02600
     *     
     * 02600 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 872), (1818, 1082, A)
     *      == 'g' ==> 02601
     *     
     * 02601 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 873), (1818, 1082, A)
     *      == 'g' ==> 02602
     *     
     * 02602 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 874), (1818, 1082, A)
     *      == 'g' ==> 02603
     *     
     * 02603 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 875), (1818, 1082, A)
     *      == 'g' ==> 02604
     *     
     * 02604 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 876), (1818, 1082, A)
     *      == 'g' ==> 02605
     *     
     * 02605 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 877, A), (1818, 1082, A)
     *      == 'g' ==> 02605
     *     
     * 02585 (1813, 181), (1814, 378), (1815, 515), (1816, 653), (1817, 853), (1818, 1075)
     *      == 'g' ==> 02606
     *      == 'u' ==> 02607
     *     
     * 02606 (1813, 182), (1814, 379), (1815, 516), (1816, 652, A), (1817, 852), (1818, 1074)
     *      == 'g' ==> 02652
     *     
     * 02652 (1813, 183), (1814, 380), (1815, 517), (1816, 652, A), (1817, 859), (1818, 1081)
     *      == 'g' ==> 02653
     *     
     * 02653 (1813, 184), (1814, 381), (1815, 518), (1816, 652, A), (1817, 860), (1818, 1082, A)
     *      == 'g' ==> 02654
     *     
     * 02654 (1813, 185), (1814, 382), (1815, 519), (1816, 652, A), (1817, 861), (1818, 1082, A)
     *      == 'g' ==> 02655
     *     
     * 02655 (1813, 186), (1814, 383), (1815, 520), (1816, 652, A), (1817, 862), (1818, 1082, A)
     *      == 'g' ==> 02656
     *     
     * 02656 (1813, 187, A), (1814, 384), (1815, 521), (1816, 652, A), (1817, 863), (1818, 1082, A)
     *      == 'g' ==> 02657
     *     
     * 02657 (1813, 187, A), (1814, 385), (1815, 522), (1816, 652, A), (1817, 864), (1818, 1082, A)
     *      == 'g' ==> 02658
     *     
     * 02658 (1813, 187, A), (1814, 386), (1815, 523), (1816, 652, A), (1817, 865), (1818, 1082, A)
     *      == 'g' ==> 02659
     *     
     * 02659 (1813, 187, A), (1814, 387), (1815, 524), (1816, 652, A), (1817, 866), (1818, 1082, A)
     *      == 'g' ==> 02660
     *     
     * 02660 (1813, 187, A), (1814, 388), (1815, 525), (1816, 652, A), (1817, 867), (1818, 1082, A)
     *      == 'g' ==> 02661
     *     
     * 02661 (1813, 187, A), (1814, 389), (1815, 526), (1816, 652, A), (1817, 868), (1818, 1082, A)
     *      == 'g' ==> 02662
     *     
     * 02662 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 869), (1818, 1082, A)
     *      == 'g' ==> 02663
     *     
     * 02663 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 870), (1818, 1082, A)
     *      == 'g' ==> 02664
     *     
     * 02664 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 871), (1818, 1082, A)
     *      == 'g' ==> 02665
     *     
     * 02665 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 872), (1818, 1082, A)
     *      == 'g' ==> 02666
     *     
     * 02666 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 873), (1818, 1082, A)
     *      == 'g' ==> 02667
     *     
     * 02667 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 874), (1818, 1082, A)
     *      == 'g' ==> 02668
     *     
     * 02668 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 875), (1818, 1082, A)
     *      == 'g' ==> 02669
     *     
     * 02669 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 876), (1818, 1082, A)
     *      == 'g' ==> 02670
     *     
     * 02670 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 877, A), (1818, 1082, A)
     *      == 'g' ==> 02670
     *     
     * 02607 (1814, 391), (1815, 528), (1816, 654), (1817, 854), (1818, 1076)
     *      == 'g' ==> 02609
     *      == 'u' ==> 02608
     *     
     * 02609 (1814, 379), (1815, 516), (1816, 652, A), (1817, 852), (1818, 1074)
     *      == 'g' ==> 02610
     *     
     * 02610 (1814, 380), (1815, 517), (1816, 652, A), (1817, 859), (1818, 1081)
     *      == 'g' ==> 02611
     *     
     * 02611 (1814, 381), (1815, 518), (1816, 652, A), (1817, 860), (1818, 1082, A)
     *      == 'g' ==> 02612
     *     
     * 02612 (1814, 382), (1815, 519), (1816, 652, A), (1817, 861), (1818, 1082, A)
     *      == 'g' ==> 02613
     *     
     * 02613 (1814, 383), (1815, 520), (1816, 652, A), (1817, 862), (1818, 1082, A)
     *      == 'g' ==> 02614
     *     
     * 02614 (1814, 384), (1815, 521), (1816, 652, A), (1817, 863), (1818, 1082, A)
     *      == 'g' ==> 02615
     *     
     * 02615 (1814, 385), (1815, 522), (1816, 652, A), (1817, 864), (1818, 1082, A)
     *      == 'g' ==> 02616
     *     
     * 02616 (1814, 386), (1815, 523), (1816, 652, A), (1817, 865), (1818, 1082, A)
     *      == 'g' ==> 02617
     *     
     * 02617 (1814, 387), (1815, 524), (1816, 652, A), (1817, 866), (1818, 1082, A)
     *      == 'g' ==> 02618
     *     
     * 02618 (1814, 388), (1815, 525), (1816, 652, A), (1817, 867), (1818, 1082, A)
     *      == 'g' ==> 02619
     *     
     * 02619 (1814, 389), (1815, 526), (1816, 652, A), (1817, 868), (1818, 1082, A)
     *      == 'g' ==> 02620
     *     
     * 02620 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 869), (1818, 1082, A)
     *      == 'g' ==> 02621
     *     
     * 02621 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 870), (1818, 1082, A)
     *      == 'g' ==> 02622
     *     
     * 02622 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 871), (1818, 1082, A)
     *      == 'g' ==> 02623
     *     
     * 02623 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 872), (1818, 1082, A)
     *      == 'g' ==> 02624
     *     
     * 02624 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 873), (1818, 1082, A)
     *      == 'g' ==> 02625
     *     
     * 02625 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 874), (1818, 1082, A)
     *      == 'g' ==> 02626
     *     
     * 02626 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 875), (1818, 1082, A)
     *      == 'g' ==> 02627
     *     
     * 02627 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 876), (1818, 1082, A)
     *      == 'g' ==> 02628
     *     
     * 02628 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 877, A), (1818, 1082, A)
     *      == 'g' ==> 02628
     *     
     * 02608 (1814, 392), (1815, 529), (1816, 655), (1817, 855), (1818, 1077)
     *      == 'g' ==> 02609
     *      == 'u' ==> 02629
     *     
     * 02629 (1817, 856), (1818, 1078)
     *      == 'g' ==> 02630
     *      == 'u' ==> 02631
     *     
     * 02630 (1817, 852), (1818, 1074)
     *      == 'g' ==> 02633
     *     
     * 02633 (1817, 859), (1818, 1081)
     *      == 'g' ==> 02634
     *     
     * 02634 (1817, 860), (1818, 1082, A)
     *      == 'g' ==> 02635
     *     
     * 02635 (1817, 861), (1818, 1082, A)
     *      == 'g' ==> 02636
     *     
     * 02636 (1817, 862), (1818, 1082, A)
     *      == 'g' ==> 02637
     *     
     * 02637 (1817, 863), (1818, 1082, A)
     *      == 'g' ==> 02638
     *     
     * 02638 (1817, 864), (1818, 1082, A)
     *      == 'g' ==> 02639
     *     
     * 02639 (1817, 865), (1818, 1082, A)
     *      == 'g' ==> 02640
     *     
     * 02640 (1817, 866), (1818, 1082, A)
     *      == 'g' ==> 02641
     *     
     * 02641 (1817, 867), (1818, 1082, A)
     *      == 'g' ==> 02642
     *     
     * 02642 (1817, 868), (1818, 1082, A)
     *      == 'g' ==> 02643
     *     
     * 02643 (1817, 869), (1818, 1082, A)
     *      == 'g' ==> 02644
     *     
     * 02644 (1817, 870), (1818, 1082, A)
     *      == 'g' ==> 02645
     *     
     * 02645 (1817, 871), (1818, 1082, A)
     *      == 'g' ==> 02646
     *     
     * 02646 (1817, 872), (1818, 1082, A)
     *      == 'g' ==> 02647
     *     
     * 02647 (1817, 873), (1818, 1082, A)
     *      == 'g' ==> 02648
     *     
     * 02648 (1817, 874), (1818, 1082, A)
     *      == 'g' ==> 02649
     *     
     * 02649 (1817, 875), (1818, 1082, A)
     *      == 'g' ==> 02650
     *     
     * 02650 (1817, 876), (1818, 1082, A)
     *      == 'g' ==> 02651
     *     
     * 02651 (1817, 877, A), (1818, 1082, A)
     *      == 'g' ==> 02651
     *     
     * 02631 (1817, 857), (1818, 1079)
     *      == 'g' ==> 02630
     *      == 'u' ==> 02632
     *     
     * 02632 (1817, 858), (1818, 1080)
     *      == 'g' ==> 02630
     *     
     * 02513 (1812, 67), (1813, 180), (1814, 377), (1815, 514), (1816, 651), (1817, 851)
     *      == 'g' ==> 02515
     *      == 'u' ==> 02514
     *     
     * 02515 (1812, 68), (1813, 182), (1814, 379), (1815, 516), (1816, 652, A), (1817, 852)
     *      == 'g' ==> 02516
     *     
     * 02516 (1812, 69), (1813, 183), (1814, 380), (1815, 517), (1816, 652, A), (1817, 859)
     *      == 'g' ==> 02517
     *     
     * 02517 (1812, 70, A), (1813, 184), (1814, 381), (1815, 518), (1816, 652, A), (1817, 860)
     *      == 'g' ==> 02518
     *     
     * 02518 (1812, 70, A), (1813, 185), (1814, 382), (1815, 519), (1816, 652, A), (1817, 861)
     *      == 'g' ==> 02519
     *     
     * 02519 (1812, 70, A), (1813, 186), (1814, 383), (1815, 520), (1816, 652, A), (1817, 862)
     *      == 'g' ==> 02520
     *     
     * 02520 (1812, 70, A), (1813, 187, A), (1814, 384), (1815, 521), (1816, 652, A), (1817, 863)
     *      == 'g' ==> 02521
     *     
     * 02521 (1812, 70, A), (1813, 187, A), (1814, 385), (1815, 522), (1816, 652, A), (1817, 864)
     *      == 'g' ==> 02522
     *     
     * 02522 (1812, 70, A), (1813, 187, A), (1814, 386), (1815, 523), (1816, 652, A), (1817, 865)
     *      == 'g' ==> 02523
     *     
     * 02523 (1812, 70, A), (1813, 187, A), (1814, 387), (1815, 524), (1816, 652, A), (1817, 866)
     *      == 'g' ==> 02524
     *     
     * 02524 (1812, 70, A), (1813, 187, A), (1814, 388), (1815, 525), (1816, 652, A), (1817, 867)
     *      == 'g' ==> 02525
     *     
     * 02525 (1812, 70, A), (1813, 187, A), (1814, 389), (1815, 526), (1816, 652, A), (1817, 868)
     *      == 'g' ==> 02526
     *     
     * 02526 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 869)
     *      == 'g' ==> 02527
     *     
     * 02527 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 870)
     *      == 'g' ==> 02528
     *     
     * 02528 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 871)
     *      == 'g' ==> 02529
     *     
     * 02529 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 872)
     *      == 'g' ==> 02530
     *     
     * 02530 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 873)
     *      == 'g' ==> 02531
     *     
     * 02531 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 874)
     *      == 'g' ==> 02532
     *     
     * 02532 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 875)
     *      == 'g' ==> 02533
     *     
     * 02533 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 876)
     *      == 'g' ==> 02534
     *     
     * 02534 (1812, 70, A), (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 877, A)
     *      == 'g' ==> 02534
     *     
     * 02514 (1813, 181), (1814, 378), (1815, 515), (1816, 653), (1817, 853)
     *      == 'g' ==> 02536
     *      == 'u' ==> 02535
     *     
     * 02536 (1813, 182), (1814, 379), (1815, 516), (1816, 652, A), (1817, 852)
     *      == 'g' ==> 02537
     *     
     * 02537 (1813, 183), (1814, 380), (1815, 517), (1816, 652, A), (1817, 859)
     *      == 'g' ==> 02538
     *     
     * 02538 (1813, 184), (1814, 381), (1815, 518), (1816, 652, A), (1817, 860)
     *      == 'g' ==> 02539
     *     
     * 02539 (1813, 185), (1814, 382), (1815, 519), (1816, 652, A), (1817, 861)
     *      == 'g' ==> 02540
     *     
     * 02540 (1813, 186), (1814, 383), (1815, 520), (1816, 652, A), (1817, 862)
     *      == 'g' ==> 02541
     *     
     * 02541 (1813, 187, A), (1814, 384), (1815, 521), (1816, 652, A), (1817, 863)
     *      == 'g' ==> 02542
     *     
     * 02542 (1813, 187, A), (1814, 385), (1815, 522), (1816, 652, A), (1817, 864)
     *      == 'g' ==> 02543
     *     
     * 02543 (1813, 187, A), (1814, 386), (1815, 523), (1816, 652, A), (1817, 865)
     *      == 'g' ==> 02544
     *     
     * 02544 (1813, 187, A), (1814, 387), (1815, 524), (1816, 652, A), (1817, 866)
     *      == 'g' ==> 02545
     *     
     * 02545 (1813, 187, A), (1814, 388), (1815, 525), (1816, 652, A), (1817, 867)
     *      == 'g' ==> 02546
     *     
     * 02546 (1813, 187, A), (1814, 389), (1815, 526), (1816, 652, A), (1817, 868)
     *      == 'g' ==> 02547
     *     
     * 02547 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 869)
     *      == 'g' ==> 02548
     *     
     * 02548 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 870)
     *      == 'g' ==> 02549
     *     
     * 02549 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 871)
     *      == 'g' ==> 02550
     *     
     * 02550 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 872)
     *      == 'g' ==> 02551
     *     
     * 02551 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 873)
     *      == 'g' ==> 02552
     *     
     * 02552 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 874)
     *      == 'g' ==> 02553
     *     
     * 02553 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 875)
     *      == 'g' ==> 02554
     *     
     * 02554 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 876)
     *      == 'g' ==> 02555
     *     
     * 02555 (1813, 187, A), (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 877, A)
     *      == 'g' ==> 02555
     *     
     * 02535 (1814, 391), (1815, 528), (1816, 654), (1817, 854)
     *      == 'g' ==> 02556
     *      == 'u' ==> 02557
     *     
     * 02556 (1814, 379), (1815, 516), (1816, 652, A), (1817, 852)
     *      == 'g' ==> 02558
     *     
     * 02558 (1814, 380), (1815, 517), (1816, 652, A), (1817, 859)
     *      == 'g' ==> 02559
     *     
     * 02559 (1814, 381), (1815, 518), (1816, 652, A), (1817, 860)
     *      == 'g' ==> 02560
     *     
     * 02560 (1814, 382), (1815, 519), (1816, 652, A), (1817, 861)
     *      == 'g' ==> 02561
     *     
     * 02561 (1814, 383), (1815, 520), (1816, 652, A), (1817, 862)
     *      == 'g' ==> 02562
     *     
     * 02562 (1814, 384), (1815, 521), (1816, 652, A), (1817, 863)
     *      == 'g' ==> 02563
     *     
     * 02563 (1814, 385), (1815, 522), (1816, 652, A), (1817, 864)
     *      == 'g' ==> 02564
     *     
     * 02564 (1814, 386), (1815, 523), (1816, 652, A), (1817, 865)
     *      == 'g' ==> 02565
     *     
     * 02565 (1814, 387), (1815, 524), (1816, 652, A), (1817, 866)
     *      == 'g' ==> 02566
     *     
     * 02566 (1814, 388), (1815, 525), (1816, 652, A), (1817, 867)
     *      == 'g' ==> 02567
     *     
     * 02567 (1814, 389), (1815, 526), (1816, 652, A), (1817, 868)
     *      == 'g' ==> 02568
     *     
     * 02568 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 869)
     *      == 'g' ==> 02569
     *     
     * 02569 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 870)
     *      == 'g' ==> 02570
     *     
     * 02570 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 871)
     *      == 'g' ==> 02571
     *     
     * 02571 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 872)
     *      == 'g' ==> 02572
     *     
     * 02572 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 873)
     *      == 'g' ==> 02573
     *     
     * 02573 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 874)
     *      == 'g' ==> 02574
     *     
     * 02574 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 875)
     *      == 'g' ==> 02575
     *     
     * 02575 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 876)
     *      == 'g' ==> 02576
     *     
     * 02576 (1814, 390, A), (1815, 527, A), (1816, 652, A), (1817, 877, A)
     *      == 'g' ==> 02576
     *     
     * 02557 (1814, 392), (1815, 529), (1816, 655), (1817, 855)
     *      == 'g' ==> 02556
     *      == 'u' ==> 02446
     *     
     * 02446 (1817, 856)
     *      == 'g' ==> 02448
     *      == 'u' ==> 02447
     *     
     * 02448 (1817, 852)
     *      == 'g' ==> 02449
     *     
     * 02449 (1817, 859)
     *      == 'g' ==> 02450
     *     
     * 02450 (1817, 860)
     *      == 'g' ==> 02451
     *     
     * 02451 (1817, 861)
     *      == 'g' ==> 02452
     *     
     * 02452 (1817, 862)
     *      == 'g' ==> 02453
     *     
     * 02453 (1817, 863)
     *      == 'g' ==> 02454
     *     
     * 02454 (1817, 864)
     *      == 'g' ==> 02455
     *     
     * 02455 (1817, 865)
     *      == 'g' ==> 02456
     *     
     * 02456 (1817, 866)
     *      == 'g' ==> 02457
     *     
     * 02457 (1817, 867)
     *      == 'g' ==> 02458
     *     
     * 02458 (1817, 868)
     *      == 'g' ==> 02459
     *     
     * 02459 (1817, 869)
     *      == 'g' ==> 02460
     *     
     * 02460 (1817, 870)
     *      == 'g' ==> 02461
     *     
     * 02461 (1817, 871)
     *      == 'g' ==> 02462
     *     
     * 02462 (1817, 872)
     *      == 'g' ==> 02463
     *     
     * 02463 (1817, 873)
     *      == 'g' ==> 02464
     *     
     * 02464 (1817, 874)
     *      == 'g' ==> 02465
     *     
     * 02465 (1817, 875)
     *      == 'g' ==> 02466
     *     
     * 02466 (1817, 876)
     *      == 'g' ==> 02467
     *     
     * 02467 (1817, 877, A)
     *      == 'g' ==> 02467
     *     
     * 02447 (1817, 857)
     *      == 'g' ==> 02448
     *      == 'u' ==> 02468
     *     
     * 02468 (1817, 858)
     *      == 'g' ==> 02448
     *     
     * 02420 (1812, 67), (1813, 180), (1815, 514), (1817, 851)
     *      == 'g' ==> 02421
     *      == 'u' ==> 02422
     *     
     * 02421 (1812, 68), (1813, 182), (1815, 516), (1817, 852)
     *      == 'g' ==> 02488
     *     
     * 02488 (1812, 69), (1813, 183), (1815, 517), (1817, 859)
     *      == 'g' ==> 02489
     *     
     * 02489 (1812, 70, A), (1813, 184), (1815, 518), (1817, 860)
     *      == 'g' ==> 02490
     *     
     * 02490 (1812, 70, A), (1813, 185), (1815, 519), (1817, 861)
     *      == 'g' ==> 02491
     *     
     * 02491 (1812, 70, A), (1813, 186), (1815, 520), (1817, 862)
     *      == 'g' ==> 02492
     *     
     * 02492 (1812, 70, A), (1813, 187, A), (1815, 521), (1817, 863)
     *      == 'g' ==> 02493
     *     
     * 02493 (1812, 70, A), (1813, 187, A), (1815, 522), (1817, 864)
     *      == 'g' ==> 02494
     *     
     * 02494 (1812, 70, A), (1813, 187, A), (1815, 523), (1817, 865)
     *      == 'g' ==> 02495
     *     
     * 02495 (1812, 70, A), (1813, 187, A), (1815, 524), (1817, 866)
     *      == 'g' ==> 02496
     *     
     * 02496 (1812, 70, A), (1813, 187, A), (1815, 525), (1817, 867)
     *      == 'g' ==> 02497
     *     
     * 02497 (1812, 70, A), (1813, 187, A), (1815, 526), (1817, 868)
     *      == 'g' ==> 02498
     *     
     * 02498 (1812, 70, A), (1813, 187, A), (1815, 527, A), (1817, 869)
     *      == 'g' ==> 02499
     *     
     * 02499 (1812, 70, A), (1813, 187, A), (1815, 527, A), (1817, 870)
     *      == 'g' ==> 02500
     *     
     * 02500 (1812, 70, A), (1813, 187, A), (1815, 527, A), (1817, 871)
     *      == 'g' ==> 02501
     *     
     * 02501 (1812, 70, A), (1813, 187, A), (1815, 527, A), (1817, 872)
     *      == 'g' ==> 02502
     *     
     * 02502 (1812, 70, A), (1813, 187, A), (1815, 527, A), (1817, 873)
     *      == 'g' ==> 02503
     *     
     * 02503 (1812, 70, A), (1813, 187, A), (1815, 527, A), (1817, 874)
     *      == 'g' ==> 02504
     *     
     * 02504 (1812, 70, A), (1813, 187, A), (1815, 527, A), (1817, 875)
     *      == 'g' ==> 02505
     *     
     * 02505 (1812, 70, A), (1813, 187, A), (1815, 527, A), (1817, 876)
     *      == 'g' ==> 02506
     *     
     * 02506 (1812, 70, A), (1813, 187, A), (1815, 527, A), (1817, 877, A)
     *      == 'g' ==> 02506
     *     
     * 02422 (1813, 181), (1815, 515), (1817, 853)
     *      == 'g' ==> 02424
     *      == 'u' ==> 02423
     *     
     * 02424 (1813, 182), (1815, 516), (1817, 852)
     *      == 'g' ==> 02425
     *     
     * 02425 (1813, 183), (1815, 517), (1817, 859)
     *      == 'g' ==> 02426
     *     
     * 02426 (1813, 184), (1815, 518), (1817, 860)
     *      == 'g' ==> 02427
     *     
     * 02427 (1813, 185), (1815, 519), (1817, 861)
     *      == 'g' ==> 02428
     *     
     * 02428 (1813, 186), (1815, 520), (1817, 862)
     *      == 'g' ==> 02429
     *     
     * 02429 (1813, 187, A), (1815, 521), (1817, 863)
     *      == 'g' ==> 02430
     *     
     * 02430 (1813, 187, A), (1815, 522), (1817, 864)
     *      == 'g' ==> 02431
     *     
     * 02431 (1813, 187, A), (1815, 523), (1817, 865)
     *      == 'g' ==> 02432
     *     
     * 02432 (1813, 187, A), (1815, 524), (1817, 866)
     *      == 'g' ==> 02433
     *     
     * 02433 (1813, 187, A), (1815, 525), (1817, 867)
     *      == 'g' ==> 02434
     *     
     * 02434 (1813, 187, A), (1815, 526), (1817, 868)
     *      == 'g' ==> 02435
     *     
     * 02435 (1813, 187, A), (1815, 527, A), (1817, 869)
     *      == 'g' ==> 02436
     *     
     * 02436 (1813, 187, A), (1815, 527, A), (1817, 870)
     *      == 'g' ==> 02437
     *     
     * 02437 (1813, 187, A), (1815, 527, A), (1817, 871)
     *      == 'g' ==> 02438
     *     
     * 02438 (1813, 187, A), (1815, 527, A), (1817, 872)
     *      == 'g' ==> 02439
     *     
     * 02439 (1813, 187, A), (1815, 527, A), (1817, 873)
     *      == 'g' ==> 02440
     *     
     * 02440 (1813, 187, A), (1815, 527, A), (1817, 874)
     *      == 'g' ==> 02441
     *     
     * 02441 (1813, 187, A), (1815, 527, A), (1817, 875)
     *      == 'g' ==> 02442
     *     
     * 02442 (1813, 187, A), (1815, 527, A), (1817, 876)
     *      == 'g' ==> 02443
     *     
     * 02443 (1813, 187, A), (1815, 527, A), (1817, 877, A)
     *      == 'g' ==> 02443
     *     
     * 02423 (1815, 528), (1817, 854)
     *      == 'g' ==> 02444
     *      == 'u' ==> 02445
     *     
     * 02444 (1815, 516), (1817, 852)
     *      == 'g' ==> 02469
     *     
     * 02469 (1815, 517), (1817, 859)
     *      == 'g' ==> 02470
     *     
     * 02470 (1815, 518), (1817, 860)
     *      == 'g' ==> 02471
     *     
     * 02471 (1815, 519), (1817, 861)
     *      == 'g' ==> 02472
     *     
     * 02472 (1815, 520), (1817, 862)
     *      == 'g' ==> 02473
     *     
     * 02473 (1815, 521), (1817, 863)
     *      == 'g' ==> 02474
     *     
     * 02474 (1815, 522), (1817, 864)
     *      == 'g' ==> 02475
     *     
     * 02475 (1815, 523), (1817, 865)
     *      == 'g' ==> 02476
     *     
     * 02476 (1815, 524), (1817, 866)
     *      == 'g' ==> 02477
     *     
     * 02477 (1815, 525), (1817, 867)
     *      == 'g' ==> 02478
     *     
     * 02478 (1815, 526), (1817, 868)
     *      == 'g' ==> 02479
     *     
     * 02479 (1815, 527, A), (1817, 869)
     *      == 'g' ==> 02480
     *     
     * 02480 (1815, 527, A), (1817, 870)
     *      == 'g' ==> 02481
     *     
     * 02481 (1815, 527, A), (1817, 871)
     *      == 'g' ==> 02482
     *     
     * 02482 (1815, 527, A), (1817, 872)
     *      == 'g' ==> 02483
     *     
     * 02483 (1815, 527, A), (1817, 873)
     *      == 'g' ==> 02484
     *     
     * 02484 (1815, 527, A), (1817, 874)
     *      == 'g' ==> 02485
     *     
     * 02485 (1815, 527, A), (1817, 875)
     *      == 'g' ==> 02486
     *     
     * 02486 (1815, 527, A), (1817, 876)
     *      == 'g' ==> 02487
     *     
     * 02487 (1815, 527, A), (1817, 877, A)
     *      == 'g' ==> 02487
     *     
     * 02445 (1815, 529), (1817, 855)
     *      == 'g' ==> 02444
     *      == 'u' ==> 02446
     *     
     * 02415 (1812, 67), (1815, 514), (1817, 851)
     *      == 'g' ==> 02672
     *      == 'u' ==> 02671
     *     
     * 02672 (1812, 68), (1815, 516), (1817, 852)
     *      == 'g' ==> 02673
     *     
     * 02673 (1812, 69), (1815, 517), (1817, 859)
     *      == 'g' ==> 02674
     *     
     * 02674 (1812, 70, A), (1815, 518), (1817, 860)
     *      == 'g' ==> 02675
     *     
     * 02675 (1812, 70, A), (1815, 519), (1817, 861)
     *      == 'g' ==> 02676
     *     
     * 02676 (1812, 70, A), (1815, 520), (1817, 862)
     *      == 'g' ==> 02677
     *     
     * 02677 (1812, 70, A), (1815, 521), (1817, 863)
     *      == 'g' ==> 02678
     *     
     * 02678 (1812, 70, A), (1815, 522), (1817, 864)
     *      == 'g' ==> 02679
     *     
     * 02679 (1812, 70, A), (1815, 523), (1817, 865)
     *      == 'g' ==> 02680
     *     
     * 02680 (1812, 70, A), (1815, 524), (1817, 866)
     *      == 'g' ==> 02681
     *     
     * 02681 (1812, 70, A), (1815, 525), (1817, 867)
     *      == 'g' ==> 02682
     *     
     * 02682 (1812, 70, A), (1815, 526), (1817, 868)
     *      == 'g' ==> 02683
     *     
     * 02683 (1812, 70, A), (1815, 527, A), (1817, 869)
     *      == 'g' ==> 02684
     *     
     * 02684 (1812, 70, A), (1815, 527, A), (1817, 870)
     *      == 'g' ==> 02685
     *     
     * 02685 (1812, 70, A), (1815, 527, A), (1817, 871)
     *      == 'g' ==> 02686
     *     
     * 02686 (1812, 70, A), (1815, 527, A), (1817, 872)
     *      == 'g' ==> 02687
     *     
     * 02687 (1812, 70, A), (1815, 527, A), (1817, 873)
     *      == 'g' ==> 02688
     *     
     * 02688 (1812, 70, A), (1815, 527, A), (1817, 874)
     *      == 'g' ==> 02689
     *     
     * 02689 (1812, 70, A), (1815, 527, A), (1817, 875)
     *      == 'g' ==> 02690
     *     
     * 02690 (1812, 70, A), (1815, 527, A), (1817, 876)
     *      == 'g' ==> 02691
     *     
     * 02691 (1812, 70, A), (1815, 527, A), (1817, 877, A)
     *      == 'g' ==> 02691
     *     
     * 02671 (1815, 515), (1817, 853)
     *      == 'g' ==> 02444
     *      == 'u' ==> 02423
     *     
     * 02398 (1815, 514)
     *      == 'g' ==> 02400
     *      == 'u' ==> 02399
     *     
     * 02400 (1815, 516)
     *      == 'g' ==> 02401
     *     
     * 02401 (1815, 517)
     *      == 'g' ==> 02402
     *     
     * 02402 (1815, 518)
     *      == 'g' ==> 02403
     *     
     * 02403 (1815, 519)
     *      == 'g' ==> 02404
     *     
     * 02404 (1815, 520)
     *      == 'g' ==> 02405
     *     
     * 02405 (1815, 521)
     *      == 'g' ==> 02406
     *     
     * 02406 (1815, 522)
     *      == 'g' ==> 02407
     *     
     * 02407 (1815, 523)
     *      == 'g' ==> 02408
     *     
     * 02408 (1815, 524)
     *      == 'g' ==> 02409
     *     
     * 02409 (1815, 525)
     *      == 'g' ==> 02410
     *     
     * 02410 (1815, 526)
     *      == 'g' ==> 02411
     *     
     * 02411 (1815, 527, A)
     *      == 'g' ==> 02411
     *     
     * 02399 (1815, 515)
     *      == 'g' ==> 02400
     *      == 'u' ==> 02412
     *     
     * 02412 (1815, 528)
     *      == 'g' ==> 02400
     *      == 'u' ==> 02413
     *     
     * 02413 (1815, 529)
     *      == 'g' ==> 02400
     *     
     * 02339 (1823, 1453), (1824, 1536), (1829, 1750, A)
     *      == 'o' ==> 02692
     *      == 'u' ==> 02693
     *     
     * 02692 (1823, 1454), (1824, 1536), (1829, 1750, A)
     *      == 'o' ==> 02701
     *      == 'u' ==> 02693
     *     
     * 02701 (1823, 1455), (1824, 1536), (1829, 1750, A)
     *      == 'o' ==> 02702
     *      == 'u' ==> 02693
     *     
     * 02702 (1823, 1456), (1824, 1536), (1829, 1750, A)
     *      == 'o' ==> 02703
     *      == 'u' ==> 02693
     *     
     * 02703 (1823, 1457), (1824, 1536), (1829, 1750, A)
     *      == 'o' ==> 02703
     *      == 'u' ==> 02704
     *     
     * 02704 (1823, 1458), (1824, 1537)
     *      == 'o' ==> 02706
     *      == 'u' ==> 02705
     *     
     * 02706 (1823, 1459, A), (1824, 1538)
     *      == 'o' ==> 02707
     *     
     * 02707 (1823, 1459, A), (1824, 1541)
     *      == 'o' ==> 02708
     *     
     * 02708 (1823, 1459, A), (1824, 1542)
     *      == 'o' ==> 02709
     *     
     * 02709 (1823, 1459, A), (1824, 1543)
     *      == 'o' ==> 02710
     *     
     * 02710 (1823, 1459, A), (1824, 1544, A)
     *      == 'o' ==> 02710
     *     
     * 02705 (1823, 1460), (1824, 1539)
     *      == 'o' ==> 02706
     *      == 'u' ==> 02711
     *     
     * 02711 (1823, 1461), (1824, 1540)
     *      == 'o' ==> 02706
     *     
     * 02693 (1824, 1537)
     *      == 'o' ==> 02694
     *      == 'u' ==> 02695
     *     
     * 02694 (1824, 1538)
     *      == 'o' ==> 02697
     *     
     * 02697 (1824, 1541)
     *      == 'o' ==> 02698
     *     
     * 02698 (1824, 1542)
     *      == 'o' ==> 02699
     *     
     * 02699 (1824, 1543)
     *      == 'o' ==> 02700
     *     
     * 02700 (1824, 1544, A)
     *      == 'o' ==> 02700
     *     
     * 02695 (1824, 1539)
     *      == 'o' ==> 02694
     *      == 'u' ==> 02696
     *     
     * 02696 (1824, 1540)
     *      == 'o' ==> 02694
     *     
     * 02338 (1831, 1780, A)
     *      == 'p' ==> 02338
     *     
     * 02342 (1833, 1810, A)
     *      == 'u' ==> 02342
     *     
     * 02343 (1825, 1619), (1826, 1698), (1827, 1720, A)
     *      == 'u' ==> 02377
     *      == 'w' ==> 02378
     *     
     * 02377 (1825, 1620)
     *      == 'u' ==> 02390
     *      == 'w' ==> 02391
     *     
     * 02390 (1825, 1622)
     *      == 'u' ==> 02396
     *      == 'w' ==> 02391
     *     
     * 02396 (1825, 1623)
     *      == 'w' ==> 02391
     *     
     * 02391 (1825, 1621)
     *      == 'w' ==> 02392
     *     
     * 02392 (1825, 1624)
     *      == 'w' ==> 02393
     *     
     * 02393 (1825, 1625)
     *      == 'w' ==> 02394
     *     
     * 02394 (1825, 1626)
     *      == 'w' ==> 02395
     *     
     * 02395 (1825, 1627, A)
     *      == 'w' ==> 02395
     *     
     * 02378 (1825, 1619), (1826, 1699), (1827, 1720, A)
     *      == 'u' ==> 02377
     *      == 'w' ==> 02379
     *     
     * 02379 (1825, 1619), (1826, 1700), (1827, 1720, A)
     *      == 'u' ==> 02377
     *      == 'w' ==> 02380
     *     
     * 02380 (1825, 1619), (1826, 1701), (1827, 1720, A)
     *      == 'u' ==> 02377
     *      == 'w' ==> 02381
     *     
     * 02381 (1825, 1619), (1826, 1702), (1827, 1720, A)
     *      == 'u' ==> 02382
     *      == 'w' ==> 02381
     *     
     * 02382 (1825, 1620), (1826, 1703)
     *      == 'u' ==> 02383
     *      == 'w' ==> 02384
     *     
     * 02383 (1825, 1622), (1826, 1704)
     *      == 'u' ==> 02389
     *      == 'w' ==> 02384
     *     
     * 02389 (1825, 1623), (1826, 1706)
     *      == 'w' ==> 02384
     *     
     * 02384 (1825, 1621), (1826, 1705, A)
     *      == 'w' ==> 02385
     *     
     * 02385 (1825, 1624), (1826, 1705, A)
     *      == 'w' ==> 02386
     *     
     * 02386 (1825, 1625), (1826, 1705, A)
     *      == 'w' ==> 02387
     *     
     * 02387 (1825, 1626), (1826, 1705, A)
     *      == 'w' ==> 02388
     *     
     * 02388 (1825, 1627, A), (1826, 1705, A)
     *      == 'w' ==> 02388
     *     
     * 02344 (1819, 1147), (1820, 1214), (1821, 1291), (1822, 1374), (1830, 1765, A)
     *      == 'o' ==> 02346
     *      == 'u' ==> 02345
     *      == 'y' ==> 02347
     *     
     * 02346 (1819, 1148)
     *      == 'y' ==> 02365
     *     
     * 02365 (1819, 1149)
     *      == 'y' ==> 02366
     *     
     * 02366 (1819, 1150)
     *      == 'y' ==> 02367
     *     
     * 02367 (1819, 1151)
     *      == 'y' ==> 02368
     *     
     * 02368 (1819, 1152)
     *      == 'y' ==> 02369
     *     
     * 02369 (1819, 1153, A)
     *      == 'y' ==> 02369
     *     
     * 02345 (1822, 1375)
     *      == 'u' ==> 02371
     *      == 'y' ==> 02370
     *     
     * 02371 (1822, 1377)
     *      == 'u' ==> 02372
     *      == 'y' ==> 02370
     *     
     * 02372 (1822, 1378)
     *      == 'y' ==> 02370
     *     
     * 02370 (1822, 1376)
     *      == 'y' ==> 02373
     *     
     * 02373 (1822, 1379)
     *      == 'y' ==> 02374
     *     
     * 02374 (1822, 1380)
     *      == 'y' ==> 02375
     *     
     * 02375 (1822, 1381)
     *      == 'y' ==> 02376
     *     
     * 02376 (1822, 1382, A)
     *      == 'y' ==> 02376
     *     
     * 02347 (1819, 1147), (1820, 1215), (1821, 1292), (1822, 1374), (1830, 1765, A)
     *      == 'o' ==> 02346
     *      == 'u' ==> 02345
     *      == 'y' ==> 02348
     *     
     * 02348 (1819, 1147), (1820, 1216), (1821, 1293), (1822, 1374), (1830, 1765, A)
     *      == 'o' ==> 02346
     *      == 'u' ==> 02345
     *      == 'y' ==> 02349
     *     
     * 02349 (1819, 1147), (1820, 1217), (1821, 1294), (1822, 1374), (1830, 1765, A)
     *      == 'o' ==> 02346
     *      == 'u' ==> 02345
     *      == 'y' ==> 02350
     *     
     * 02350 (1819, 1147), (1820, 1218), (1821, 1295), (1822, 1374), (1830, 1765, A)
     *      == 'o' ==> 02352
     *      == 'u' ==> 02351
     *      == 'y' ==> 02350
     *     
     * 02352 (1819, 1148), (1820, 1219)
     *      == 'y' ==> 02353
     *     
     * 02353 (1819, 1149), (1820, 1220, A)
     *      == 'y' ==> 02354
     *     
     * 02354 (1819, 1150), (1820, 1220, A)
     *      == 'y' ==> 02355
     *     
     * 02355 (1819, 1151), (1820, 1220, A)
     *      == 'y' ==> 02356
     *     
     * 02356 (1819, 1152), (1820, 1220, A)
     *      == 'y' ==> 02357
     *     
     * 02357 (1819, 1153, A), (1820, 1220, A)
     *      == 'y' ==> 02357
     *     
     * 02351 (1821, 1296), (1822, 1375)
     *      == 'u' ==> 02359
     *      == 'y' ==> 02358
     *     
     * 02359 (1821, 1297), (1822, 1377)
     *      == 'u' ==> 02360
     *      == 'y' ==> 02358
     *     
     * 02360 (1821, 1299), (1822, 1378)
     *      == 'y' ==> 02358
     *     
     * 02358 (1821, 1298, A), (1822, 1376)
     *      == 'y' ==> 02361
     *     
     * 02361 (1821, 1298, A), (1822, 1379)
     *      == 'y' ==> 02362
     *     
     * 02362 (1821, 1298, A), (1822, 1380)
     *      == 'y' ==> 02363
     *     
     * 02363 (1821, 1298, A), (1822, 1381)
     *      == 'y' ==> 02364
     *     
     * 02364 (1821, 1298, A), (1822, 1382, A)
     *      == 'y' ==> 02364
     *     
     * END: STATE MACHINE
     */
_3080: /* INIT_STATE_TRANSITION_BLOCK */
    input = *(me->buffer._input_p);
    __quex_debug("Init State\n");
    __quex_debug_state(2337);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 2337, 3081);/* \0 */

        case 0x63: goto _3082;/* 'c' */

        case 0x67: goto _3083;/* 'g' */

        case 0x6F: goto _3084;/* 'o' */

        case 0x70: goto _3085;/* 'p' */

        case 0x75: goto _3086;/* 'u' */

        case 0x77: goto _3087;/* 'w' */

        case 0x79: goto _3088;/* 'y' */


    }
    __quex_debug_drop_out(2337);

goto _3090; /* TERMINAL_FAILURE */

_2337:


    ++(me->buffer._input_p);
    goto _3080;
#   define __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE(X) ( \
             (X) == 0 ? 2446 :    \
             (X) == 1 ? 2629 :    \
             (X) == 2 ? 2426 :    \
             (X) == 3 ? 2447 :    \
             (X) == 4 ? 2631 :    \
             (X) == 5 ? 2427 :    \
             (X) == 6 ? 2422 :    \
             (X) == 7 ? 2671 :    \
             (X) == 8 ? 2585 :    \
             (X) == 9 ? 2514 :    \
             (X) == 10 ? 2399 :    \
             (X) == 11 ? 2672 :    \
             (X) == 12 ? 2421 :    \
             (X) == 13 ? 2535 :    \
             (X) == 14 ? 2607 :    \
             (X) == 15 ? 2423 :    \
             (X) == 16 ? 2412 :    \
             (X) == 17 ? 2673 :    \
             (X) == 18 ? 2488 :    \
             (X) == 19 ? 2424 :    \
             (X) == 20 ? 2445 :    \
             (X) == 21 ? 2557 :    \
             (X) == 22 ? 2608 :    \
             (X) == 23 ? 2425 :    \
             (X) == 24 ? 2413 :    \
             (X) == 25 ? 2420 :    \
             (X) == 26 ? 2584 :    \
             (X) == 27 ? 2513 :    \
             (X) == 28 ? 2415 :    \
             (X) == 29 ? 2398 :    \
             (X) == 30 ? 2581 :    \
             (X) == 31 ? 2582 :    \
             (X) == 32 ? 2580 :    \
             (X) == 33 ? 2579 :    \
             (X) == 34 ? 2578 :    \
             (X) == 35 ? 2577 :    \
             (X) == 36 ? 2512 :    \
             (X) == 37 ? 2511 :    \
             (X) == 38 ? 2583 :    \
             (X) == 39 ? 2414 :    \
             (X) == 40 ? 2340 :    \
             (X) == 41 ? 2510 :    \
             (X) == 42 ? 2509 :    \
             (X) == 43 ? 2508 :    \
             (X) == 44 ? 2507 :    \
             (X) == 45 ? 2419 :    \
             (X) == 46 ? 2418 :    \
             (X) == 47 ? 2417 :    \
             (X) == 48 ? 2416 :    \
             (X) == 49 ? 2397 :    \
             (X) == 50 ? 2342 : 0)

    __quex_assert_no_passage();
_3091: /* (2513 from 2577) (2513 from 2578) (2513 from 2511) (2513 from 2512) (2513 from 2581) (2513 from 2582) (2513 from 2579) (2513 from 2580) */
    state_key = 27;
    __quex_debug("state_key = 27\n");
    goto _3092;
_3093: /* (2420 from 2419) (2420 from 2418) (2420 from 2508) (2420 from 2507) (2420 from 2510) (2420 from 2509) */
    state_key = 25;
    __quex_debug("state_key = 25\n");
    goto _3092;
_3094: /* (2415 from 2416) (2415 from 2417) (2415 from 2414) */
    state_key = 28;
    __quex_debug("state_key = 28\n");
    goto _3092;
_3095: /* (2398 from 2397) (2398 from 2340) */
    state_key = 29;
    __quex_debug("state_key = 29\n");
    goto _3092;
_3096: /* (2397 from 2340) */
    state_key = 49;
    __quex_debug("state_key = 49\n");
    goto _3092;
_3097: /* (2416 from 2414) */
    state_key = 48;
    __quex_debug("state_key = 48\n");
    goto _3092;
_3098: /* (2417 from 2416) */
    state_key = 47;
    __quex_debug("state_key = 47\n");
    goto _3092;
_3099: /* (2418 from 2417) */
    state_key = 46;
    __quex_debug("state_key = 46\n");
    goto _3092;
_3100: /* (2419 from 2418) */
    state_key = 45;
    __quex_debug("state_key = 45\n");
    goto _3092;
_3101: /* (2507 from 2419) */
    state_key = 44;
    __quex_debug("state_key = 44\n");
    goto _3092;
_3102: /* (2508 from 2507) */
    state_key = 43;
    __quex_debug("state_key = 43\n");
    goto _3092;
_3103: /* (2509 from 2508) */
    state_key = 42;
    __quex_debug("state_key = 42\n");
    goto _3092;
_3104: /* (2510 from 2509) */
    state_key = 41;
    __quex_debug("state_key = 41\n");
    goto _3092;
_3105: /* (2511 from 2510) */
    state_key = 37;
    __quex_debug("state_key = 37\n");
    goto _3092;
_3106: /* (2512 from 2511) */
    state_key = 36;
    __quex_debug("state_key = 36\n");
    goto _3092;
_3107: /* (2577 from 2512) */
    state_key = 35;
    __quex_debug("state_key = 35\n");
    goto _3092;
_3108: /* (2578 from 2577) */
    state_key = 34;
    __quex_debug("state_key = 34\n");
    goto _3092;
_3109: /* (2579 from 2578) */
    state_key = 33;
    __quex_debug("state_key = 33\n");
    goto _3092;
_3110: /* (2580 from 2579) */
    state_key = 32;
    __quex_debug("state_key = 32\n");
    goto _3092;
_3111: /* (2581 from 2580) */
    state_key = 30;
    __quex_debug("state_key = 30\n");
    goto _3092;
_3112: /* (2582 from 2581) */
    state_key = 31;
    __quex_debug("state_key = 31\n");
    goto _3092;
_3113: /* (2584 from 2583) */
    state_key = 26;
    __quex_debug("state_key = 26\n");

_3092:
    position[0] = me->buffer._input_p; __quex_debug("position[0] = input_p;\n");
    goto _3075;
_3114: /* (2423 from 2671) (2423 from 2422) */
    state_key = 15;
    __quex_debug("state_key = 15\n");
    goto _3075;
_3115: /* (2446 from 2445) (2446 from 2557) */
    state_key = 0;
    __quex_debug("state_key = 0\n");
    goto _3075;
_3083: /* (2340 from 2337) */
    state_key = 40;
    __quex_debug("state_key = 40\n");
    goto _3075;
_3086: /* (2342 from 2337) */
    state_key = 50;
    __quex_debug("state_key = 50\n");
    goto _3075;
_3116: /* (2414 from 2397) */
    state_key = 39;
    __quex_debug("state_key = 39\n");
    goto _3075;
_3117: /* (2399 from 2398) */
    state_key = 10;
    __quex_debug("state_key = 10\n");
    goto _3075;
_3118: /* (2412 from 2399) */
    state_key = 16;
    __quex_debug("state_key = 16\n");
    goto _3075;
_3119: /* (2413 from 2412) */
    state_key = 24;
    __quex_debug("state_key = 24\n");
    goto _3075;
_3120: /* (2671 from 2415) */
    state_key = 7;
    __quex_debug("state_key = 7\n");
    goto _3075;
_3121: /* (2672 from 2415) */
    state_key = 11;
    __quex_debug("state_key = 11\n");
    goto _3075;
_3122: /* (2421 from 2420) */
    state_key = 12;
    __quex_debug("state_key = 12\n");
    goto _3075;
_3123: /* (2422 from 2420) */
    state_key = 6;
    __quex_debug("state_key = 6\n");
    goto _3075;
_3124: /* (2488 from 2421) */
    state_key = 18;
    __quex_debug("state_key = 18\n");
    goto _3075;
_3125: /* (2424 from 2422) */
    state_key = 19;
    __quex_debug("state_key = 19\n");
    goto _3075;
_3126: /* (2445 from 2423) */
    state_key = 20;
    __quex_debug("state_key = 20\n");
    goto _3075;
_3127: /* (2425 from 2424) */
    state_key = 23;
    __quex_debug("state_key = 23\n");
    goto _3075;
_3128: /* (2426 from 2425) */
    state_key = 2;
    __quex_debug("state_key = 2\n");
    goto _3075;
_3129: /* (2427 from 2426) */
    state_key = 5;
    __quex_debug("state_key = 5\n");
    goto _3075;
_3130: /* (2447 from 2446) */
    state_key = 3;
    __quex_debug("state_key = 3\n");
    goto _3075;
_3131: /* (2514 from 2513) */
    state_key = 9;
    __quex_debug("state_key = 9\n");
    goto _3075;
_3132: /* (2535 from 2514) */
    state_key = 13;
    __quex_debug("state_key = 13\n");
    goto _3075;
_3133: /* (2557 from 2535) */
    state_key = 21;
    __quex_debug("state_key = 21\n");
    goto _3075;
_3134: /* (2583 from 2582) */
    state_key = 38;
    __quex_debug("state_key = 38\n");
    goto _3075;
_3135: /* (2585 from 2584) */
    state_key = 8;
    __quex_debug("state_key = 8\n");
    goto _3075;
_3136: /* (2607 from 2585) */
    state_key = 14;
    __quex_debug("state_key = 14\n");
    goto _3075;
_3137: /* (2608 from 2607) */
    state_key = 22;
    __quex_debug("state_key = 22\n");
    goto _3075;
_3138: /* (2629 from 2608) */
    state_key = 1;
    __quex_debug("state_key = 1\n");
    goto _3075;
_3139: /* (2631 from 2629) */
    state_key = 4;
    __quex_debug("state_key = 4\n");
    goto _3075;
_3140: /* (2673 from 2672) */
    state_key = 17;
    __quex_debug("state_key = 17\n");

_3075: /* (2583 from 2583) (2342 from 2342) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_template_state(3075, state_key);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 3075, 3141);/* \0 */

        case 0x67: QUEX_GOTO_STATE(template_3075_target_0[state_key]);/* 'g' */

        case 0x75: QUEX_GOTO_STATE(template_3075_target_1[state_key]);/* 'u' */


    }
_3141:
    __quex_debug_template_drop_out(3075, state_key);

    switch( state_key ) {
        case 0xB: 
        case 0x9: 
        case 0xC: 
        case 0x6: 
        case 0x8: 
        case 0xA: 
        case 0x7:         me->buffer._input_p -= 2; 
        goto TERMINAL_1828;
        case 0x11: 
        case 0xD: 
        case 0x12: 
        case 0x13: 
        case 0xF: 
        case 0x10: 
        case 0xE:         me->buffer._input_p -= 3; 
        goto TERMINAL_1828;
        case 0x2C: 
        case 0x2B: 
        case 0x2A: 
        case 0x29: 
        case 0x25: 
        case 0x24: 
        case 0x23: 
        case 0x22: 
        case 0x21: 
        case 0x20: 
        case 0x1E: 
        case 0x1F: 
        case 0x26: 
        case 0x31: 
        case 0x28: 
        case 0x27: 
        case 0x30: 
        case 0x2F: 
        case 0x2E: 
        case 0x2D:         goto TERMINAL_1828;
        case 0x1A: 
        case 0x1B: 
        case 0x19: 
        case 0x1D: 
        case 0x1C:         me->buffer._input_p -= 1; 
        goto TERMINAL_1828;
        case 0x5: 
        case 0x3: 
        case 0x4:         me->buffer._input_p -= 6; 
        goto TERMINAL_1828;
        case 0x32:         goto TERMINAL_1833;
        case 0x16: 
        case 0x17: 
        case 0x14: 
        case 0x18: 
        case 0x15:         me->buffer._input_p -= 4; 
        goto TERMINAL_1828;
        case 0x2: 
        case 0x1: 
        case 0x0:         me->buffer._input_p -= 5; 
        goto TERMINAL_1828;

    }
#   undef __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE
#   define __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE(X) ( \
             (X) == 0 ? 2393 :    \
             (X) == 1 ? 2394 :    \
             (X) == 2 ? 2392 :    \
             (X) == 3 ? 2391 :    \
             (X) == 4 ? 2386 :    \
             (X) == 5 ? 2387 :    \
             (X) == 6 ? 2385 :    \
             (X) == 7 ? 2384 :    \
             (X) == 8 ? 2388 :    \
             (X) == 9 ? 2395 :    \
             (X) == 10 ? 2389 :    \
             (X) == 11 ? 2396 :    \
             (X) == 12 ? 2377 :    \
             (X) == 13 ? 2382 :    \
             (X) == 14 ? 2383 :    \
             (X) == 15 ? 2390 :    \
             (X) == 16 ? 2379 :    \
             (X) == 17 ? 2380 :    \
             (X) == 18 ? 2378 :    \
             (X) == 19 ? 2343 :    \
             (X) == 20 ? 2381 : 0)

    __quex_assert_no_passage();
_3157: /* (2377 from 2378) (2377 from 2343) (2377 from 2379) (2377 from 2380) */
    position[0] = me->buffer._input_p; __quex_debug("position[0] = input_p;\n");
    state_key = 12;
    __quex_debug("state_key = 12\n");
    goto _3076;
_3158: /* (2378 from 2343) */
    state_key = 18;
    __quex_debug("state_key = 18\n");
    goto _3159;
_3160: /* (2379 from 2378) */
    state_key = 16;
    __quex_debug("state_key = 16\n");
    goto _3159;
_3161: /* (2380 from 2379) */
    state_key = 17;
    __quex_debug("state_key = 17\n");

_3159:
    position[0] = me->buffer._input_p; __quex_debug("position[0] = input_p;\n");
    goto _3076;
_3162: /* (2391 from 2377) (2391 from 2396) (2391 from 2390) */
    state_key = 3;
    __quex_debug("state_key = 3\n");
    goto _3076;
_3163: /* (2384 from 2383) (2384 from 2382) (2384 from 2389) */
    state_key = 7;
    __quex_debug("state_key = 7\n");
    goto _3076;
_3087: /* (2343 from 2337) */
    state_key = 19;
    __quex_debug("state_key = 19\n");
    goto _3076;
_3164: /* (2390 from 2377) */
    state_key = 15;
    __quex_debug("state_key = 15\n");
    goto _3076;
_3165: /* (2381 from 2380) */
    state_key = 20;
    __quex_debug("state_key = 20\n");
    goto _3076;
_3166: /* (2382 from 2381) */
    state_key = 13;
    __quex_debug("state_key = 13\n");
    goto _3076;
_3167: /* (2383 from 2382) */
    state_key = 14;
    __quex_debug("state_key = 14\n");
    goto _3076;
_3168: /* (2389 from 2383) */
    state_key = 10;
    __quex_debug("state_key = 10\n");
    goto _3076;
_3169: /* (2385 from 2384) */
    state_key = 6;
    __quex_debug("state_key = 6\n");
    goto _3076;
_3170: /* (2386 from 2385) */
    state_key = 4;
    __quex_debug("state_key = 4\n");
    goto _3076;
_3171: /* (2387 from 2386) */
    state_key = 5;
    __quex_debug("state_key = 5\n");
    goto _3076;
_3172: /* (2388 from 2387) */
    state_key = 8;
    __quex_debug("state_key = 8\n");
    goto _3076;
_3173: /* (2396 from 2390) */
    state_key = 11;
    __quex_debug("state_key = 11\n");
    goto _3076;
_3174: /* (2392 from 2391) */
    state_key = 2;
    __quex_debug("state_key = 2\n");
    goto _3076;
_3175: /* (2393 from 2392) */
    state_key = 0;
    __quex_debug("state_key = 0\n");
    goto _3076;
_3176: /* (2394 from 2393) */
    state_key = 1;
    __quex_debug("state_key = 1\n");
    goto _3076;
_3177: /* (2395 from 2394) */
    state_key = 9;
    __quex_debug("state_key = 9\n");

_3076: /* (2388 from 2388) (2395 from 2395) (2381 from 2381) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_template_state(3076, state_key);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 3076, 3178);/* \0 */

        case 0x75: QUEX_GOTO_STATE(template_3076_target_0[state_key]);/* 'u' */

        case 0x77: QUEX_GOTO_STATE(template_3076_target_1[state_key]);/* 'w' */


    }
_3178:
    __quex_debug_template_drop_out(3076, state_key);

    switch( state_key ) {
        case 0x9: 
        case 0x8:         goto TERMINAL_1825;
        case 0x7: 
        case 0x6: 
        case 0x4: 
        case 0x5:         goto TERMINAL_1826;
        case 0xF: 
        case 0xE:         me->buffer._input_p -= 2; 
        goto TERMINAL_1827;
        case 0xB: 
        case 0xA:         me->buffer._input_p -= 3; 
        goto TERMINAL_1827;
        case 0x2: 
        case 0x0: 
        case 0x1: 
        case 0x3:         __quex_assert(position[0] != 0x0);
me->buffer._input_p = position[0];

        goto TERMINAL_1827;
        case 0xC: 
        case 0xD:         me->buffer._input_p -= 1; 
        goto TERMINAL_1827;
        case 0x12: 
        case 0x10: 
        case 0x11: 
        case 0x14: 
        case 0x13:         goto TERMINAL_1827;

    }
#   undef __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE
#   define __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE(X) ( \
             (X) == 0 ? 2374 :    \
             (X) == 1 ? 2375 :    \
             (X) == 2 ? 2373 :    \
             (X) == 3 ? 2370 :    \
             (X) == 4 ? 2355 :    \
             (X) == 5 ? 2356 :    \
             (X) == 6 ? 2354 :    \
             (X) == 7 ? 2353 :    \
             (X) == 8 ? 2363 :    \
             (X) == 9 ? 2364 :    \
             (X) == 10 ? 2362 :    \
             (X) == 11 ? 2361 :    \
             (X) == 12 ? 2358 :    \
             (X) == 13 ? 2357 :    \
             (X) == 14 ? 2369 :    \
             (X) == 15 ? 2366 :    \
             (X) == 16 ? 2372 :    \
             (X) == 17 ? 2360 :    \
             (X) == 18 ? 2376 :    \
             (X) == 19 ? 2368 :    \
             (X) == 20 ? 2367 :    \
             (X) == 21 ? 2345 :    \
             (X) == 22 ? 2351 :    \
             (X) == 23 ? 2352 :    \
             (X) == 24 ? 2346 :    \
             (X) == 25 ? 2359 :    \
             (X) == 26 ? 2371 :    \
             (X) == 27 ? 2365 : 0)

    __quex_assert_no_passage();
_3179: /* (2345 from 2344) (2345 from 2349) (2345 from 2347) (2345 from 2348) */
    position[0] = me->buffer._input_p; __quex_debug("position[0] = input_p;\n");
    state_key = 21;
    __quex_debug("state_key = 21\n");
    goto _3077;
_3180: /* (2346 from 2344) (2346 from 2349) (2346 from 2348) (2346 from 2347) */
    state_key = 24;
    __quex_debug("state_key = 24\n");
    goto _3077;
_3181: /* (2370 from 2345) (2370 from 2372) (2370 from 2371) */
    state_key = 3;
    __quex_debug("state_key = 3\n");
    goto _3077;
_3182: /* (2358 from 2351) (2358 from 2360) (2358 from 2359) */
    state_key = 12;
    __quex_debug("state_key = 12\n");
    goto _3077;
_3183: /* (2371 from 2345) */
    state_key = 26;
    __quex_debug("state_key = 26\n");
    goto _3077;
_3184: /* (2365 from 2346) */
    state_key = 27;
    __quex_debug("state_key = 27\n");
    goto _3077;
_3185: /* (2351 from 2350) */
    state_key = 22;
    __quex_debug("state_key = 22\n");
    goto _3077;
_3186: /* (2352 from 2350) */
    state_key = 23;
    __quex_debug("state_key = 23\n");
    goto _3077;
_3187: /* (2359 from 2351) */
    state_key = 25;
    __quex_debug("state_key = 25\n");
    goto _3077;
_3188: /* (2353 from 2352) */
    state_key = 7;
    __quex_debug("state_key = 7\n");
    goto _3077;
_3189: /* (2354 from 2353) */
    state_key = 6;
    __quex_debug("state_key = 6\n");
    goto _3077;
_3190: /* (2355 from 2354) */
    state_key = 4;
    __quex_debug("state_key = 4\n");
    goto _3077;
_3191: /* (2356 from 2355) */
    state_key = 5;
    __quex_debug("state_key = 5\n");
    goto _3077;
_3192: /* (2357 from 2356) */
    state_key = 13;
    __quex_debug("state_key = 13\n");
    goto _3077;
_3193: /* (2361 from 2358) */
    state_key = 11;
    __quex_debug("state_key = 11\n");
    goto _3077;
_3194: /* (2360 from 2359) */
    state_key = 17;
    __quex_debug("state_key = 17\n");
    goto _3077;
_3195: /* (2362 from 2361) */
    state_key = 10;
    __quex_debug("state_key = 10\n");
    goto _3077;
_3196: /* (2363 from 2362) */
    state_key = 8;
    __quex_debug("state_key = 8\n");
    goto _3077;
_3197: /* (2364 from 2363) */
    state_key = 9;
    __quex_debug("state_key = 9\n");
    goto _3077;
_3198: /* (2366 from 2365) */
    state_key = 15;
    __quex_debug("state_key = 15\n");
    goto _3077;
_3199: /* (2367 from 2366) */
    state_key = 20;
    __quex_debug("state_key = 20\n");
    goto _3077;
_3200: /* (2368 from 2367) */
    state_key = 19;
    __quex_debug("state_key = 19\n");
    goto _3077;
_3201: /* (2369 from 2368) */
    state_key = 14;
    __quex_debug("state_key = 14\n");
    goto _3077;
_3202: /* (2373 from 2370) */
    state_key = 2;
    __quex_debug("state_key = 2\n");
    goto _3077;
_3203: /* (2372 from 2371) */
    state_key = 16;
    __quex_debug("state_key = 16\n");
    goto _3077;
_3204: /* (2374 from 2373) */
    state_key = 0;
    __quex_debug("state_key = 0\n");
    goto _3077;
_3205: /* (2375 from 2374) */
    state_key = 1;
    __quex_debug("state_key = 1\n");
    goto _3077;
_3206: /* (2376 from 2375) */
    state_key = 18;
    __quex_debug("state_key = 18\n");

_3077: /* (2364 from 2364) (2357 from 2357) (2376 from 2376) (2369 from 2369) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_template_state(3077, state_key);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 3077, 3207);/* \0 */

        case 0x75: QUEX_GOTO_STATE(template_3077_target_0[state_key]);/* 'u' */

        case 0x79: QUEX_GOTO_STATE(template_3077_target_1[state_key]);/* 'y' */


    }
_3207:
    __quex_debug_template_drop_out(3077, state_key);

    switch( state_key ) {
        case 0xE: 
        case 0xD:         goto TERMINAL_1819;
        case 0xB: 
        case 0xA: 
        case 0x8: 
        case 0x9: 
        case 0xC:         goto TERMINAL_1821;
        case 0x7: 
        case 0x6: 
        case 0x4: 
        case 0x5:         goto TERMINAL_1820;
        case 0x1A: 
        case 0x1B: 
        case 0x19:         me->buffer._input_p -= 2; 
        goto TERMINAL_1830;
        case 0x11: 
        case 0x10: 
        case 0xF:         me->buffer._input_p -= 3; 
        goto TERMINAL_1830;
        case 0x12:         goto TERMINAL_1822;
        case 0x17: 
        case 0x15: 
        case 0x18: 
        case 0x16:         me->buffer._input_p -= 1; 
        goto TERMINAL_1830;
        case 0x3: 
        case 0x2: 
        case 0x0: 
        case 0x1:         __quex_assert(position[0] != 0x0);
me->buffer._input_p = position[0];

        goto TERMINAL_1830;
        case 0x14:         me->buffer._input_p -= 4; 
        goto TERMINAL_1830;
        case 0x13:         me->buffer._input_p -= 5; 
        goto TERMINAL_1830;

    }
#   undef __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE
#   define __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE(X) ( \
             (X) == 0 ? 2698 :    \
             (X) == 1 ? 2699 :    \
             (X) == 2 ? 2697 :    \
             (X) == 3 ? 2694 :    \
             (X) == 4 ? 2709 :    \
             (X) == 5 ? 2710 :    \
             (X) == 6 ? 2708 :    \
             (X) == 7 ? 2707 :    \
             (X) == 8 ? 2706 :    \
             (X) == 9 ? 2696 :    \
             (X) == 10 ? 2711 :    \
             (X) == 11 ? 2700 :    \
             (X) == 12 ? 2693 :    \
             (X) == 13 ? 2704 :    \
             (X) == 14 ? 2695 :    \
             (X) == 15 ? 2705 :    \
             (X) == 16 ? 2701 :    \
             (X) == 17 ? 2702 :    \
             (X) == 18 ? 2692 :    \
             (X) == 19 ? 2339 :    \
             (X) == 20 ? 2703 :    \
             (X) == 21 ? 2348 :    \
             (X) == 22 ? 2349 :    \
             (X) == 23 ? 2347 :    \
             (X) == 24 ? 2344 :    \
             (X) == 25 ? 2350 : 0)

    __quex_assert_no_passage();
_3208: /* (2693 from 2701) (2693 from 2702) (2693 from 2339) (2693 from 2692) */
    state_key = 12;
    __quex_debug("state_key = 12\n");
    goto _3209;
_3210: /* (2692 from 2339) */
    state_key = 18;
    __quex_debug("state_key = 18\n");
    goto _3209;
_3211: /* (2347 from 2344) */
    state_key = 23;
    __quex_debug("state_key = 23\n");
    goto _3209;
_3212: /* (2348 from 2347) */
    state_key = 21;
    __quex_debug("state_key = 21\n");
    goto _3209;
_3213: /* (2349 from 2348) */
    state_key = 22;
    __quex_debug("state_key = 22\n");
    goto _3209;
_3214: /* (2701 from 2692) */
    state_key = 16;
    __quex_debug("state_key = 16\n");
    goto _3209;
_3215: /* (2702 from 2701) */
    state_key = 17;
    __quex_debug("state_key = 17\n");

_3209:
    position[0] = me->buffer._input_p; __quex_debug("position[0] = input_p;\n");
    goto _3078;
_3216: /* (2694 from 2693) (2694 from 2696) (2694 from 2695) */
    state_key = 3;
    __quex_debug("state_key = 3\n");
    goto _3078;
_3217: /* (2706 from 2705) (2706 from 2704) (2706 from 2711) */
    state_key = 8;
    __quex_debug("state_key = 8\n");
    goto _3078;
_3084: /* (2339 from 2337) */
    state_key = 19;
    __quex_debug("state_key = 19\n");
    goto _3078;
_3088: /* (2344 from 2337) */
    state_key = 24;
    __quex_debug("state_key = 24\n");
    goto _3078;
_3218: /* (2350 from 2349) */
    state_key = 25;
    __quex_debug("state_key = 25\n");
    goto _3078;
_3219: /* (2695 from 2693) */
    state_key = 14;
    __quex_debug("state_key = 14\n");
    goto _3078;
_3220: /* (2697 from 2694) */
    state_key = 2;
    __quex_debug("state_key = 2\n");
    goto _3078;
_3221: /* (2696 from 2695) */
    state_key = 9;
    __quex_debug("state_key = 9\n");
    goto _3078;
_3222: /* (2698 from 2697) */
    state_key = 0;
    __quex_debug("state_key = 0\n");
    goto _3078;
_3223: /* (2699 from 2698) */
    state_key = 1;
    __quex_debug("state_key = 1\n");
    goto _3078;
_3224: /* (2700 from 2699) */
    state_key = 11;
    __quex_debug("state_key = 11\n");
    goto _3078;
_3225: /* (2703 from 2702) */
    state_key = 20;
    __quex_debug("state_key = 20\n");
    goto _3078;
_3226: /* (2704 from 2703) */
    state_key = 13;
    __quex_debug("state_key = 13\n");
    goto _3078;
_3227: /* (2705 from 2704) */
    state_key = 15;
    __quex_debug("state_key = 15\n");
    goto _3078;
_3228: /* (2711 from 2705) */
    state_key = 10;
    __quex_debug("state_key = 10\n");
    goto _3078;
_3229: /* (2707 from 2706) */
    state_key = 7;
    __quex_debug("state_key = 7\n");
    goto _3078;
_3230: /* (2708 from 2707) */
    state_key = 6;
    __quex_debug("state_key = 6\n");
    goto _3078;
_3231: /* (2709 from 2708) */
    state_key = 4;
    __quex_debug("state_key = 4\n");
    goto _3078;
_3232: /* (2710 from 2709) */
    state_key = 5;
    __quex_debug("state_key = 5\n");

_3078: /* (2710 from 2710) (2700 from 2700) (2703 from 2703) (2350 from 2350) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_template_state(3078, state_key);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 3078, 3233);/* \0 */

        case 0x6F: QUEX_GOTO_STATE(template_3078_target_0[state_key]);/* 'o' */

        case 0x75: QUEX_GOTO_STATE(template_3078_target_1[state_key]);/* 'u' */

        case 0x79: QUEX_GOTO_STATE(template_3078_target_2[state_key]);/* 'y' */


    }
_3233:
    __quex_debug_template_drop_out(3078, state_key);

    switch( state_key ) {
        case 0x8: 
        case 0x7: 
        case 0x6: 
        case 0x4: 
        case 0x5:         goto TERMINAL_1823;
        case 0xB:         goto TERMINAL_1824;
        case 0x18: 
        case 0x17: 
        case 0x15: 
        case 0x16: 
        case 0x19:         goto TERMINAL_1830;
        case 0xF: 
        case 0xE:         me->buffer._input_p -= 2; 
        goto TERMINAL_1829;
        case 0x9: 
        case 0xA:         me->buffer._input_p -= 3; 
        goto TERMINAL_1829;
        case 0x2: 
        case 0x0: 
        case 0x1: 
        case 0x3:         __quex_assert(position[0] != 0x0);
me->buffer._input_p = position[0];

        goto TERMINAL_1829;
        case 0xD: 
        case 0xC:         me->buffer._input_p -= 1; 
        goto TERMINAL_1829;
        case 0x13: 
        case 0x12: 
        case 0x10: 
        case 0x11: 
        case 0x14:         goto TERMINAL_1829;

    }
#   undef __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE
#   define __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE(X) ( \
             (X) == 0 ? 2338 :    \
             (X) == 1 ? 2341 : 0)

    __quex_assert_no_passage();
_3085: /* (2338 from 2337) */
    state_key = 0;
    __quex_debug("state_key = 0\n");
    goto _3079;
_3082: /* (2341 from 2337) */
    state_key = 1;
    __quex_debug("state_key = 1\n");

_3079: /* (2338 from 2338) (2341 from 2341) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_template_state(3079, state_key);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 3079, 3234);/* \0 */

        case 0x63: QUEX_GOTO_STATE(template_3079_target_0[state_key]);/* 'c' */

        case 0x70: QUEX_GOTO_STATE(template_3079_target_1[state_key]);/* 'p' */


    }
_3234:
    __quex_debug_template_drop_out(3079, state_key);

    switch( state_key ) {
        case 0x0:         goto TERMINAL_1831;
        case 0x1:         goto TERMINAL_1832;

    }
#   undef __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE
#   define __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE(X) ( \
             (X) == 0 ? 2630 :    \
             (X) == 1 ? 2633 :    \
             (X) == 2 ? 2478 :    \
             (X) == 3 ? 2477 :    \
             (X) == 4 ? 2476 :    \
             (X) == 5 ? 2475 :    \
             (X) == 6 ? 2474 :    \
             (X) == 7 ? 2473 :    \
             (X) == 8 ? 2472 :    \
             (X) == 9 ? 2471 :    \
             (X) == 10 ? 2470 :    \
             (X) == 11 ? 2469 :    \
             (X) == 12 ? 2466 :    \
             (X) == 13 ? 2465 :    \
             (X) == 14 ? 2464 :    \
             (X) == 15 ? 2463 :    \
             (X) == 16 ? 2462 :    \
             (X) == 17 ? 2461 :    \
             (X) == 18 ? 2460 :    \
             (X) == 19 ? 2459 :    \
             (X) == 20 ? 2458 :    \
             (X) == 21 ? 2457 :    \
             (X) == 22 ? 2456 :    \
             (X) == 23 ? 2455 :    \
             (X) == 24 ? 2454 :    \
             (X) == 25 ? 2453 :    \
             (X) == 26 ? 2452 :    \
             (X) == 27 ? 2451 :    \
             (X) == 28 ? 2450 :    \
             (X) == 29 ? 2449 :    \
             (X) == 30 ? 2448 :    \
             (X) == 31 ? 2444 :    \
             (X) == 32 ? 2410 :    \
             (X) == 33 ? 2409 :    \
             (X) == 34 ? 2408 :    \
             (X) == 35 ? 2407 :    \
             (X) == 36 ? 2406 :    \
             (X) == 37 ? 2405 :    \
             (X) == 38 ? 2404 :    \
             (X) == 39 ? 2403 :    \
             (X) == 40 ? 2402 :    \
             (X) == 41 ? 2401 :    \
             (X) == 42 ? 2400 :    \
             (X) == 43 ? 2649 :    \
             (X) == 44 ? 2650 :    \
             (X) == 45 ? 2648 :    \
             (X) == 46 ? 2647 :    \
             (X) == 47 ? 2646 :    \
             (X) == 48 ? 2645 :    \
             (X) == 49 ? 2644 :    \
             (X) == 50 ? 2643 :    \
             (X) == 51 ? 2642 :    \
             (X) == 52 ? 2641 :    \
             (X) == 53 ? 2640 :    \
             (X) == 54 ? 2639 :    \
             (X) == 55 ? 2638 :    \
             (X) == 56 ? 2637 :    \
             (X) == 57 ? 2636 :    \
             (X) == 58 ? 2635 :    \
             (X) == 59 ? 2634 :    \
             (X) == 60 ? 2654 :    \
             (X) == 61 ? 2655 :    \
             (X) == 62 ? 2653 :    \
             (X) == 63 ? 2652 :    \
             (X) == 64 ? 2619 :    \
             (X) == 65 ? 2618 :    \
             (X) == 66 ? 2617 :    \
             (X) == 67 ? 2616 :    \
             (X) == 68 ? 2615 :    \
             (X) == 69 ? 2614 :    \
             (X) == 70 ? 2613 :    \
             (X) == 71 ? 2612 :    \
             (X) == 72 ? 2611 :    \
             (X) == 73 ? 2610 :    \
             (X) == 74 ? 2609 :    \
             (X) == 75 ? 2606 :    \
             (X) == 76 ? 2587 :    \
             (X) == 77 ? 2586 :    \
             (X) == 78 ? 2567 :    \
             (X) == 79 ? 2566 :    \
             (X) == 80 ? 2565 :    \
             (X) == 81 ? 2564 :    \
             (X) == 82 ? 2563 :    \
             (X) == 83 ? 2562 :    \
             (X) == 84 ? 2561 :    \
             (X) == 85 ? 2560 :    \
             (X) == 86 ? 2559 :    \
             (X) == 87 ? 2558 :    \
             (X) == 88 ? 2556 :    \
             (X) == 89 ? 2540 :    \
             (X) == 90 ? 2539 :    \
             (X) == 91 ? 2538 :    \
             (X) == 92 ? 2537 :    \
             (X) == 93 ? 2536 :    \
             (X) == 94 ? 2516 :    \
             (X) == 95 ? 2515 :    \
             (X) == 96 ? 2486 :    \
             (X) == 97 ? 2487 :    \
             (X) == 98 ? 2485 :    \
             (X) == 99 ? 2484 :    \
             (X) == 100 ? 2483 :    \
             (X) == 101 ? 2482 :    \
             (X) == 102 ? 2481 :    \
             (X) == 103 ? 2480 :    \
             (X) == 104 ? 2479 :    \
             (X) == 105 ? 2411 :    \
             (X) == 106 ? 2627 :    \
             (X) == 107 ? 2628 :    \
             (X) == 108 ? 2626 :    \
             (X) == 109 ? 2625 :    \
             (X) == 110 ? 2624 :    \
             (X) == 111 ? 2623 :    \
             (X) == 112 ? 2622 :    \
             (X) == 113 ? 2621 :    \
             (X) == 114 ? 2620 :    \
             (X) == 115 ? 2576 :    \
             (X) == 116 ? 2575 :    \
             (X) == 117 ? 2574 :    \
             (X) == 118 ? 2573 :    \
             (X) == 119 ? 2572 :    \
             (X) == 120 ? 2571 :    \
             (X) == 121 ? 2570 :    \
             (X) == 122 ? 2569 :    \
             (X) == 123 ? 2568 :    \
             (X) == 124 ? 2669 :    \
             (X) == 125 ? 2670 :    \
             (X) == 126 ? 2668 :    \
             (X) == 127 ? 2667 :    \
             (X) == 128 ? 2666 :    \
             (X) == 129 ? 2665 :    \
             (X) == 130 ? 2664 :    \
             (X) == 131 ? 2663 :    \
             (X) == 132 ? 2662 :    \
             (X) == 133 ? 2661 :    \
             (X) == 134 ? 2660 :    \
             (X) == 135 ? 2659 :    \
             (X) == 136 ? 2658 :    \
             (X) == 137 ? 2657 :    \
             (X) == 138 ? 2656 :    \
             (X) == 139 ? 2555 :    \
             (X) == 140 ? 2554 :    \
             (X) == 141 ? 2553 :    \
             (X) == 142 ? 2552 :    \
             (X) == 143 ? 2551 :    \
             (X) == 144 ? 2550 :    \
             (X) == 145 ? 2549 :    \
             (X) == 146 ? 2548 :    \
             (X) == 147 ? 2547 :    \
             (X) == 148 ? 2546 :    \
             (X) == 149 ? 2545 :    \
             (X) == 150 ? 2544 :    \
             (X) == 151 ? 2543 :    \
             (X) == 152 ? 2542 :    \
             (X) == 153 ? 2541 :    \
             (X) == 154 ? 2443 :    \
             (X) == 155 ? 2442 :    \
             (X) == 156 ? 2441 :    \
             (X) == 157 ? 2440 :    \
             (X) == 158 ? 2439 :    \
             (X) == 159 ? 2438 :    \
             (X) == 160 ? 2437 :    \
             (X) == 161 ? 2436 :    \
             (X) == 162 ? 2435 :    \
             (X) == 163 ? 2434 :    \
             (X) == 164 ? 2433 :    \
             (X) == 165 ? 2432 :    \
             (X) == 166 ? 2431 :    \
             (X) == 167 ? 2430 :    \
             (X) == 168 ? 2429 :    \
             (X) == 169 ? 2690 :    \
             (X) == 170 ? 2691 :    \
             (X) == 171 ? 2689 :    \
             (X) == 172 ? 2688 :    \
             (X) == 173 ? 2687 :    \
             (X) == 174 ? 2686 :    \
             (X) == 175 ? 2685 :    \
             (X) == 176 ? 2684 :    \
             (X) == 177 ? 2683 :    \
             (X) == 178 ? 2682 :    \
             (X) == 179 ? 2681 :    \
             (X) == 180 ? 2680 :    \
             (X) == 181 ? 2679 :    \
             (X) == 182 ? 2678 :    \
             (X) == 183 ? 2677 :    \
             (X) == 184 ? 2676 :    \
             (X) == 185 ? 2675 :    \
             (X) == 186 ? 2674 :    \
             (X) == 187 ? 2605 :    \
             (X) == 188 ? 2604 :    \
             (X) == 189 ? 2603 :    \
             (X) == 190 ? 2602 :    \
             (X) == 191 ? 2601 :    \
             (X) == 192 ? 2600 :    \
             (X) == 193 ? 2599 :    \
             (X) == 194 ? 2598 :    \
             (X) == 195 ? 2597 :    \
             (X) == 196 ? 2596 :    \
             (X) == 197 ? 2595 :    \
             (X) == 198 ? 2594 :    \
             (X) == 199 ? 2593 :    \
             (X) == 200 ? 2592 :    \
             (X) == 201 ? 2591 :    \
             (X) == 202 ? 2590 :    \
             (X) == 203 ? 2589 :    \
             (X) == 204 ? 2588 :    \
             (X) == 205 ? 2534 :    \
             (X) == 206 ? 2533 :    \
             (X) == 207 ? 2532 :    \
             (X) == 208 ? 2531 :    \
             (X) == 209 ? 2530 :    \
             (X) == 210 ? 2529 :    \
             (X) == 211 ? 2528 :    \
             (X) == 212 ? 2527 :    \
             (X) == 213 ? 2526 :    \
             (X) == 214 ? 2525 :    \
             (X) == 215 ? 2524 :    \
             (X) == 216 ? 2523 :    \
             (X) == 217 ? 2522 :    \
             (X) == 218 ? 2521 :    \
             (X) == 219 ? 2520 :    \
             (X) == 220 ? 2519 :    \
             (X) == 221 ? 2518 :    \
             (X) == 222 ? 2517 :    \
             (X) == 223 ? 2506 :    \
             (X) == 224 ? 2505 :    \
             (X) == 225 ? 2504 :    \
             (X) == 226 ? 2503 :    \
             (X) == 227 ? 2502 :    \
             (X) == 228 ? 2501 :    \
             (X) == 229 ? 2500 :    \
             (X) == 230 ? 2499 :    \
             (X) == 231 ? 2498 :    \
             (X) == 232 ? 2497 :    \
             (X) == 233 ? 2496 :    \
             (X) == 234 ? 2495 :    \
             (X) == 235 ? 2494 :    \
             (X) == 236 ? 2493 :    \
             (X) == 237 ? 2492 :    \
             (X) == 238 ? 2491 :    \
             (X) == 239 ? 2490 :    \
             (X) == 240 ? 2489 :    \
             (X) == 241 ? 2467 :    \
             (X) == 242 ? 2651 :    \
             (X) == 243 ? 2468 :    \
             (X) == 244 ? 2632 :    \
             (X) == 245 ? 2428 : 0)

    __quex_assert_no_passage();
_3148: /* (2400 from 2399) (2400 from 2412) (2400 from 2398) (2400 from 2413) */
    state_key = 42;
    __quex_debug("state_key = 42\n");
    goto _3067;
_3145: /* (2444 from 2445) (2444 from 2423) (2444 from 2671) */
    state_key = 31;
    __quex_debug("state_key = 31\n");
    goto _3067;
_3142: /* (2448 from 2447) (2448 from 2468) (2448 from 2446) */
    state_key = 30;
    __quex_debug("state_key = 30\n");
    goto _3067;
_3143: /* (2630 from 2629) (2630 from 2632) (2630 from 2631) */
    state_key = 0;
    __quex_debug("state_key = 0\n");
    goto _3067;
_3149: /* (2556 from 2557) (2556 from 2535) */
    state_key = 88;
    __quex_debug("state_key = 88\n");
    goto _3067;
_3150: /* (2609 from 2607) (2609 from 2608) */
    state_key = 74;
    __quex_debug("state_key = 74\n");
    goto _3067;
_3235: /* (2401 from 2400) */
    state_key = 41;
    __quex_debug("state_key = 41\n");
    goto _3067;
_3236: /* (2402 from 2401) */
    state_key = 40;
    __quex_debug("state_key = 40\n");
    goto _3067;
_3237: /* (2403 from 2402) */
    state_key = 39;
    __quex_debug("state_key = 39\n");
    goto _3067;
_3238: /* (2404 from 2403) */
    state_key = 38;
    __quex_debug("state_key = 38\n");
    goto _3067;
_3239: /* (2405 from 2404) */
    state_key = 37;
    __quex_debug("state_key = 37\n");
    goto _3067;
_3240: /* (2406 from 2405) */
    state_key = 36;
    __quex_debug("state_key = 36\n");
    goto _3067;
_3241: /* (2407 from 2406) */
    state_key = 35;
    __quex_debug("state_key = 35\n");
    goto _3067;
_3242: /* (2408 from 2407) */
    state_key = 34;
    __quex_debug("state_key = 34\n");
    goto _3067;
_3243: /* (2409 from 2408) */
    state_key = 33;
    __quex_debug("state_key = 33\n");
    goto _3067;
_3244: /* (2410 from 2409) */
    state_key = 32;
    __quex_debug("state_key = 32\n");
    goto _3067;
_3245: /* (2411 from 2410) */
    state_key = 105;
    __quex_debug("state_key = 105\n");
    goto _3067;
_3144: /* (2428 from 2427) */
    state_key = 245;
    __quex_debug("state_key = 245\n");
    goto _3067;
_3246: /* (2429 from 2428) */
    state_key = 168;
    __quex_debug("state_key = 168\n");
    goto _3067;
_3247: /* (2430 from 2429) */
    state_key = 167;
    __quex_debug("state_key = 167\n");
    goto _3067;
_3248: /* (2431 from 2430) */
    state_key = 166;
    __quex_debug("state_key = 166\n");
    goto _3067;
_3249: /* (2432 from 2431) */
    state_key = 165;
    __quex_debug("state_key = 165\n");
    goto _3067;
_3250: /* (2433 from 2432) */
    state_key = 164;
    __quex_debug("state_key = 164\n");
    goto _3067;
_3251: /* (2434 from 2433) */
    state_key = 163;
    __quex_debug("state_key = 163\n");
    goto _3067;
_3252: /* (2435 from 2434) */
    state_key = 162;
    __quex_debug("state_key = 162\n");
    goto _3067;
_3253: /* (2436 from 2435) */
    state_key = 161;
    __quex_debug("state_key = 161\n");
    goto _3067;
_3254: /* (2437 from 2436) */
    state_key = 160;
    __quex_debug("state_key = 160\n");
    goto _3067;
_3255: /* (2438 from 2437) */
    state_key = 159;
    __quex_debug("state_key = 159\n");
    goto _3067;
_3256: /* (2439 from 2438) */
    state_key = 158;
    __quex_debug("state_key = 158\n");
    goto _3067;
_3257: /* (2440 from 2439) */
    state_key = 157;
    __quex_debug("state_key = 157\n");
    goto _3067;
_3258: /* (2441 from 2440) */
    state_key = 156;
    __quex_debug("state_key = 156\n");
    goto _3067;
_3259: /* (2442 from 2441) */
    state_key = 155;
    __quex_debug("state_key = 155\n");
    goto _3067;
_3260: /* (2443 from 2442) */
    state_key = 154;
    __quex_debug("state_key = 154\n");
    goto _3067;
_3261: /* (2469 from 2444) */
    state_key = 11;
    __quex_debug("state_key = 11\n");
    goto _3067;
_3155: /* (2468 from 2447) */
    state_key = 243;
    __quex_debug("state_key = 243\n");
    goto _3067;
_3262: /* (2449 from 2448) */
    state_key = 29;
    __quex_debug("state_key = 29\n");
    goto _3067;
_3263: /* (2450 from 2449) */
    state_key = 28;
    __quex_debug("state_key = 28\n");
    goto _3067;
_3264: /* (2451 from 2450) */
    state_key = 27;
    __quex_debug("state_key = 27\n");
    goto _3067;
_3265: /* (2452 from 2451) */
    state_key = 26;
    __quex_debug("state_key = 26\n");
    goto _3067;
_3266: /* (2453 from 2452) */
    state_key = 25;
    __quex_debug("state_key = 25\n");
    goto _3067;
_3267: /* (2454 from 2453) */
    state_key = 24;
    __quex_debug("state_key = 24\n");
    goto _3067;
_3268: /* (2455 from 2454) */
    state_key = 23;
    __quex_debug("state_key = 23\n");
    goto _3067;
_3269: /* (2456 from 2455) */
    state_key = 22;
    __quex_debug("state_key = 22\n");
    goto _3067;
_3270: /* (2457 from 2456) */
    state_key = 21;
    __quex_debug("state_key = 21\n");
    goto _3067;
_3271: /* (2458 from 2457) */
    state_key = 20;
    __quex_debug("state_key = 20\n");
    goto _3067;
_3272: /* (2459 from 2458) */
    state_key = 19;
    __quex_debug("state_key = 19\n");
    goto _3067;
_3273: /* (2460 from 2459) */
    state_key = 18;
    __quex_debug("state_key = 18\n");
    goto _3067;
_3274: /* (2461 from 2460) */
    state_key = 17;
    __quex_debug("state_key = 17\n");
    goto _3067;
_3275: /* (2462 from 2461) */
    state_key = 16;
    __quex_debug("state_key = 16\n");
    goto _3067;
_3276: /* (2463 from 2462) */
    state_key = 15;
    __quex_debug("state_key = 15\n");
    goto _3067;
_3277: /* (2464 from 2463) */
    state_key = 14;
    __quex_debug("state_key = 14\n");
    goto _3067;
_3278: /* (2465 from 2464) */
    state_key = 13;
    __quex_debug("state_key = 13\n");
    goto _3067;
_3279: /* (2466 from 2465) */
    state_key = 12;
    __quex_debug("state_key = 12\n");
    goto _3067;
_3280: /* (2467 from 2466) */
    state_key = 241;
    __quex_debug("state_key = 241\n");
    goto _3067;
_3281: /* (2470 from 2469) */
    state_key = 10;
    __quex_debug("state_key = 10\n");
    goto _3067;
_3282: /* (2471 from 2470) */
    state_key = 9;
    __quex_debug("state_key = 9\n");
    goto _3067;
_3283: /* (2472 from 2471) */
    state_key = 8;
    __quex_debug("state_key = 8\n");
    goto _3067;
_3284: /* (2473 from 2472) */
    state_key = 7;
    __quex_debug("state_key = 7\n");
    goto _3067;
_3285: /* (2474 from 2473) */
    state_key = 6;
    __quex_debug("state_key = 6\n");
    goto _3067;
_3286: /* (2475 from 2474) */
    state_key = 5;
    __quex_debug("state_key = 5\n");
    goto _3067;
_3287: /* (2476 from 2475) */
    state_key = 4;
    __quex_debug("state_key = 4\n");
    goto _3067;
_3288: /* (2477 from 2476) */
    state_key = 3;
    __quex_debug("state_key = 3\n");
    goto _3067;
_3289: /* (2478 from 2477) */
    state_key = 2;
    __quex_debug("state_key = 2\n");
    goto _3067;
_3290: /* (2479 from 2478) */
    state_key = 104;
    __quex_debug("state_key = 104\n");
    goto _3067;
_3291: /* (2480 from 2479) */
    state_key = 103;
    __quex_debug("state_key = 103\n");
    goto _3067;
_3292: /* (2481 from 2480) */
    state_key = 102;
    __quex_debug("state_key = 102\n");
    goto _3067;
_3293: /* (2482 from 2481) */
    state_key = 101;
    __quex_debug("state_key = 101\n");
    goto _3067;
_3294: /* (2483 from 2482) */
    state_key = 100;
    __quex_debug("state_key = 100\n");
    goto _3067;
_3295: /* (2484 from 2483) */
    state_key = 99;
    __quex_debug("state_key = 99\n");
    goto _3067;
_3296: /* (2485 from 2484) */
    state_key = 98;
    __quex_debug("state_key = 98\n");
    goto _3067;
_3297: /* (2486 from 2485) */
    state_key = 96;
    __quex_debug("state_key = 96\n");
    goto _3067;
_3298: /* (2487 from 2486) */
    state_key = 97;
    __quex_debug("state_key = 97\n");
    goto _3067;
_3152: /* (2489 from 2488) */
    state_key = 240;
    __quex_debug("state_key = 240\n");
    goto _3067;
_3299: /* (2490 from 2489) */
    state_key = 239;
    __quex_debug("state_key = 239\n");
    goto _3067;
_3300: /* (2491 from 2490) */
    state_key = 238;
    __quex_debug("state_key = 238\n");
    goto _3067;
_3301: /* (2492 from 2491) */
    state_key = 237;
    __quex_debug("state_key = 237\n");
    goto _3067;
_3302: /* (2493 from 2492) */
    state_key = 236;
    __quex_debug("state_key = 236\n");
    goto _3067;
_3303: /* (2494 from 2493) */
    state_key = 235;
    __quex_debug("state_key = 235\n");
    goto _3067;
_3304: /* (2495 from 2494) */
    state_key = 234;
    __quex_debug("state_key = 234\n");
    goto _3067;
_3305: /* (2496 from 2495) */
    state_key = 233;
    __quex_debug("state_key = 233\n");
    goto _3067;
_3306: /* (2497 from 2496) */
    state_key = 232;
    __quex_debug("state_key = 232\n");
    goto _3067;
_3307: /* (2498 from 2497) */
    state_key = 231;
    __quex_debug("state_key = 231\n");
    goto _3067;
_3308: /* (2499 from 2498) */
    state_key = 230;
    __quex_debug("state_key = 230\n");
    goto _3067;
_3309: /* (2500 from 2499) */
    state_key = 229;
    __quex_debug("state_key = 229\n");
    goto _3067;
_3310: /* (2501 from 2500) */
    state_key = 228;
    __quex_debug("state_key = 228\n");
    goto _3067;
_3311: /* (2502 from 2501) */
    state_key = 227;
    __quex_debug("state_key = 227\n");
    goto _3067;
_3312: /* (2503 from 2502) */
    state_key = 226;
    __quex_debug("state_key = 226\n");
    goto _3067;
_3313: /* (2504 from 2503) */
    state_key = 225;
    __quex_debug("state_key = 225\n");
    goto _3067;
_3314: /* (2505 from 2504) */
    state_key = 224;
    __quex_debug("state_key = 224\n");
    goto _3067;
_3315: /* (2506 from 2505) */
    state_key = 223;
    __quex_debug("state_key = 223\n");
    goto _3067;
_3154: /* (2515 from 2513) */
    state_key = 95;
    __quex_debug("state_key = 95\n");
    goto _3067;
_3147: /* (2536 from 2514) */
    state_key = 93;
    __quex_debug("state_key = 93\n");
    goto _3067;
_3316: /* (2516 from 2515) */
    state_key = 94;
    __quex_debug("state_key = 94\n");
    goto _3067;
_3317: /* (2517 from 2516) */
    state_key = 222;
    __quex_debug("state_key = 222\n");
    goto _3067;
_3318: /* (2518 from 2517) */
    state_key = 221;
    __quex_debug("state_key = 221\n");
    goto _3067;
_3319: /* (2519 from 2518) */
    state_key = 220;
    __quex_debug("state_key = 220\n");
    goto _3067;
_3320: /* (2520 from 2519) */
    state_key = 219;
    __quex_debug("state_key = 219\n");
    goto _3067;
_3321: /* (2521 from 2520) */
    state_key = 218;
    __quex_debug("state_key = 218\n");
    goto _3067;
_3322: /* (2522 from 2521) */
    state_key = 217;
    __quex_debug("state_key = 217\n");
    goto _3067;
_3323: /* (2523 from 2522) */
    state_key = 216;
    __quex_debug("state_key = 216\n");
    goto _3067;
_3324: /* (2524 from 2523) */
    state_key = 215;
    __quex_debug("state_key = 215\n");
    goto _3067;
_3325: /* (2525 from 2524) */
    state_key = 214;
    __quex_debug("state_key = 214\n");
    goto _3067;
_3326: /* (2526 from 2525) */
    state_key = 213;
    __quex_debug("state_key = 213\n");
    goto _3067;
_3327: /* (2527 from 2526) */
    state_key = 212;
    __quex_debug("state_key = 212\n");
    goto _3067;
_3328: /* (2528 from 2527) */
    state_key = 211;
    __quex_debug("state_key = 211\n");
    goto _3067;
_3329: /* (2529 from 2528) */
    state_key = 210;
    __quex_debug("state_key = 210\n");
    goto _3067;
_3330: /* (2530 from 2529) */
    state_key = 209;
    __quex_debug("state_key = 209\n");
    goto _3067;
_3331: /* (2531 from 2530) */
    state_key = 208;
    __quex_debug("state_key = 208\n");
    goto _3067;
_3332: /* (2532 from 2531) */
    state_key = 207;
    __quex_debug("state_key = 207\n");
    goto _3067;
_3333: /* (2533 from 2532) */
    state_key = 206;
    __quex_debug("state_key = 206\n");
    goto _3067;
_3334: /* (2534 from 2533) */
    state_key = 205;
    __quex_debug("state_key = 205\n");
    goto _3067;
_3335: /* (2537 from 2536) */
    state_key = 92;
    __quex_debug("state_key = 92\n");
    goto _3067;
_3336: /* (2538 from 2537) */
    state_key = 91;
    __quex_debug("state_key = 91\n");
    goto _3067;
_3337: /* (2539 from 2538) */
    state_key = 90;
    __quex_debug("state_key = 90\n");
    goto _3067;
_3338: /* (2540 from 2539) */
    state_key = 89;
    __quex_debug("state_key = 89\n");
    goto _3067;
_3339: /* (2541 from 2540) */
    state_key = 153;
    __quex_debug("state_key = 153\n");
    goto _3067;
_3340: /* (2542 from 2541) */
    state_key = 152;
    __quex_debug("state_key = 152\n");
    goto _3067;
_3341: /* (2543 from 2542) */
    state_key = 151;
    __quex_debug("state_key = 151\n");
    goto _3067;
_3342: /* (2544 from 2543) */
    state_key = 150;
    __quex_debug("state_key = 150\n");
    goto _3067;
_3343: /* (2545 from 2544) */
    state_key = 149;
    __quex_debug("state_key = 149\n");
    goto _3067;
_3344: /* (2546 from 2545) */
    state_key = 148;
    __quex_debug("state_key = 148\n");
    goto _3067;
_3345: /* (2547 from 2546) */
    state_key = 147;
    __quex_debug("state_key = 147\n");
    goto _3067;
_3346: /* (2548 from 2547) */
    state_key = 146;
    __quex_debug("state_key = 146\n");
    goto _3067;
_3347: /* (2549 from 2548) */
    state_key = 145;
    __quex_debug("state_key = 145\n");
    goto _3067;
_3348: /* (2550 from 2549) */
    state_key = 144;
    __quex_debug("state_key = 144\n");
    goto _3067;
_3349: /* (2551 from 2550) */
    state_key = 143;
    __quex_debug("state_key = 143\n");
    goto _3067;
_3350: /* (2552 from 2551) */
    state_key = 142;
    __quex_debug("state_key = 142\n");
    goto _3067;
_3351: /* (2553 from 2552) */
    state_key = 141;
    __quex_debug("state_key = 141\n");
    goto _3067;
_3352: /* (2554 from 2553) */
    state_key = 140;
    __quex_debug("state_key = 140\n");
    goto _3067;
_3353: /* (2555 from 2554) */
    state_key = 139;
    __quex_debug("state_key = 139\n");
    goto _3067;
_3354: /* (2558 from 2556) */
    state_key = 87;
    __quex_debug("state_key = 87\n");
    goto _3067;
_3355: /* (2559 from 2558) */
    state_key = 86;
    __quex_debug("state_key = 86\n");
    goto _3067;
_3356: /* (2560 from 2559) */
    state_key = 85;
    __quex_debug("state_key = 85\n");
    goto _3067;
_3357: /* (2561 from 2560) */
    state_key = 84;
    __quex_debug("state_key = 84\n");
    goto _3067;
_3358: /* (2562 from 2561) */
    state_key = 83;
    __quex_debug("state_key = 83\n");
    goto _3067;
_3359: /* (2563 from 2562) */
    state_key = 82;
    __quex_debug("state_key = 82\n");
    goto _3067;
_3360: /* (2564 from 2563) */
    state_key = 81;
    __quex_debug("state_key = 81\n");
    goto _3067;
_3361: /* (2565 from 2564) */
    state_key = 80;
    __quex_debug("state_key = 80\n");
    goto _3067;
_3362: /* (2566 from 2565) */
    state_key = 79;
    __quex_debug("state_key = 79\n");
    goto _3067;
_3363: /* (2567 from 2566) */
    state_key = 78;
    __quex_debug("state_key = 78\n");
    goto _3067;
_3364: /* (2568 from 2567) */
    state_key = 123;
    __quex_debug("state_key = 123\n");
    goto _3067;
_3365: /* (2569 from 2568) */
    state_key = 122;
    __quex_debug("state_key = 122\n");
    goto _3067;
_3366: /* (2570 from 2569) */
    state_key = 121;
    __quex_debug("state_key = 121\n");
    goto _3067;
_3367: /* (2571 from 2570) */
    state_key = 120;
    __quex_debug("state_key = 120\n");
    goto _3067;
_3368: /* (2572 from 2571) */
    state_key = 119;
    __quex_debug("state_key = 119\n");
    goto _3067;
_3369: /* (2573 from 2572) */
    state_key = 118;
    __quex_debug("state_key = 118\n");
    goto _3067;
_3370: /* (2574 from 2573) */
    state_key = 117;
    __quex_debug("state_key = 117\n");
    goto _3067;
_3371: /* (2575 from 2574) */
    state_key = 116;
    __quex_debug("state_key = 116\n");
    goto _3067;
_3372: /* (2576 from 2575) */
    state_key = 115;
    __quex_debug("state_key = 115\n");
    goto _3067;
_3153: /* (2586 from 2584) */
    state_key = 77;
    __quex_debug("state_key = 77\n");
    goto _3067;
_3146: /* (2606 from 2585) */
    state_key = 75;
    __quex_debug("state_key = 75\n");
    goto _3067;
_3373: /* (2587 from 2586) */
    state_key = 76;
    __quex_debug("state_key = 76\n");
    goto _3067;
_3374: /* (2588 from 2587) */
    state_key = 204;
    __quex_debug("state_key = 204\n");
    goto _3067;
_3375: /* (2589 from 2588) */
    state_key = 203;
    __quex_debug("state_key = 203\n");
    goto _3067;
_3376: /* (2590 from 2589) */
    state_key = 202;
    __quex_debug("state_key = 202\n");
    goto _3067;
_3377: /* (2591 from 2590) */
    state_key = 201;
    __quex_debug("state_key = 201\n");
    goto _3067;
_3378: /* (2592 from 2591) */
    state_key = 200;
    __quex_debug("state_key = 200\n");
    goto _3067;
_3379: /* (2593 from 2592) */
    state_key = 199;
    __quex_debug("state_key = 199\n");
    goto _3067;
_3380: /* (2594 from 2593) */
    state_key = 198;
    __quex_debug("state_key = 198\n");
    goto _3067;
_3381: /* (2595 from 2594) */
    state_key = 197;
    __quex_debug("state_key = 197\n");
    goto _3067;
_3382: /* (2596 from 2595) */
    state_key = 196;
    __quex_debug("state_key = 196\n");
    goto _3067;
_3383: /* (2597 from 2596) */
    state_key = 195;
    __quex_debug("state_key = 195\n");
    goto _3067;
_3384: /* (2598 from 2597) */
    state_key = 194;
    __quex_debug("state_key = 194\n");
    goto _3067;
_3385: /* (2599 from 2598) */
    state_key = 193;
    __quex_debug("state_key = 193\n");
    goto _3067;
_3386: /* (2600 from 2599) */
    state_key = 192;
    __quex_debug("state_key = 192\n");
    goto _3067;
_3387: /* (2601 from 2600) */
    state_key = 191;
    __quex_debug("state_key = 191\n");
    goto _3067;
_3388: /* (2602 from 2601) */
    state_key = 190;
    __quex_debug("state_key = 190\n");
    goto _3067;
_3389: /* (2603 from 2602) */
    state_key = 189;
    __quex_debug("state_key = 189\n");
    goto _3067;
_3390: /* (2604 from 2603) */
    state_key = 188;
    __quex_debug("state_key = 188\n");
    goto _3067;
_3391: /* (2605 from 2604) */
    state_key = 187;
    __quex_debug("state_key = 187\n");
    goto _3067;
_3392: /* (2652 from 2606) */
    state_key = 63;
    __quex_debug("state_key = 63\n");
    goto _3067;
_3393: /* (2610 from 2609) */
    state_key = 73;
    __quex_debug("state_key = 73\n");
    goto _3067;
_3394: /* (2611 from 2610) */
    state_key = 72;
    __quex_debug("state_key = 72\n");
    goto _3067;
_3395: /* (2612 from 2611) */
    state_key = 71;
    __quex_debug("state_key = 71\n");
    goto _3067;
_3396: /* (2613 from 2612) */
    state_key = 70;
    __quex_debug("state_key = 70\n");
    goto _3067;
_3397: /* (2614 from 2613) */
    state_key = 69;
    __quex_debug("state_key = 69\n");
    goto _3067;
_3398: /* (2615 from 2614) */
    state_key = 68;
    __quex_debug("state_key = 68\n");
    goto _3067;
_3399: /* (2616 from 2615) */
    state_key = 67;
    __quex_debug("state_key = 67\n");
    goto _3067;
_3400: /* (2617 from 2616) */
    state_key = 66;
    __quex_debug("state_key = 66\n");
    goto _3067;
_3401: /* (2618 from 2617) */
    state_key = 65;
    __quex_debug("state_key = 65\n");
    goto _3067;
_3402: /* (2619 from 2618) */
    state_key = 64;
    __quex_debug("state_key = 64\n");
    goto _3067;
_3403: /* (2620 from 2619) */
    state_key = 114;
    __quex_debug("state_key = 114\n");
    goto _3067;
_3404: /* (2621 from 2620) */
    state_key = 113;
    __quex_debug("state_key = 113\n");
    goto _3067;
_3405: /* (2622 from 2621) */
    state_key = 112;
    __quex_debug("state_key = 112\n");
    goto _3067;
_3406: /* (2623 from 2622) */
    state_key = 111;
    __quex_debug("state_key = 111\n");
    goto _3067;
_3407: /* (2624 from 2623) */
    state_key = 110;
    __quex_debug("state_key = 110\n");
    goto _3067;
_3408: /* (2625 from 2624) */
    state_key = 109;
    __quex_debug("state_key = 109\n");
    goto _3067;
_3409: /* (2626 from 2625) */
    state_key = 108;
    __quex_debug("state_key = 108\n");
    goto _3067;
_3410: /* (2627 from 2626) */
    state_key = 106;
    __quex_debug("state_key = 106\n");
    goto _3067;
_3411: /* (2628 from 2627) */
    state_key = 107;
    __quex_debug("state_key = 107\n");
    goto _3067;
_3412: /* (2633 from 2630) */
    state_key = 1;
    __quex_debug("state_key = 1\n");
    goto _3067;
_3156: /* (2632 from 2631) */
    state_key = 244;
    __quex_debug("state_key = 244\n");
    goto _3067;
_3413: /* (2634 from 2633) */
    state_key = 59;
    __quex_debug("state_key = 59\n");
    goto _3067;
_3414: /* (2635 from 2634) */
    state_key = 58;
    __quex_debug("state_key = 58\n");
    goto _3067;
_3415: /* (2636 from 2635) */
    state_key = 57;
    __quex_debug("state_key = 57\n");
    goto _3067;
_3416: /* (2637 from 2636) */
    state_key = 56;
    __quex_debug("state_key = 56\n");
    goto _3067;
_3417: /* (2638 from 2637) */
    state_key = 55;
    __quex_debug("state_key = 55\n");
    goto _3067;
_3418: /* (2639 from 2638) */
    state_key = 54;
    __quex_debug("state_key = 54\n");
    goto _3067;
_3419: /* (2640 from 2639) */
    state_key = 53;
    __quex_debug("state_key = 53\n");
    goto _3067;
_3420: /* (2641 from 2640) */
    state_key = 52;
    __quex_debug("state_key = 52\n");
    goto _3067;
_3421: /* (2642 from 2641) */
    state_key = 51;
    __quex_debug("state_key = 51\n");
    goto _3067;
_3422: /* (2643 from 2642) */
    state_key = 50;
    __quex_debug("state_key = 50\n");
    goto _3067;
_3423: /* (2644 from 2643) */
    state_key = 49;
    __quex_debug("state_key = 49\n");
    goto _3067;
_3424: /* (2645 from 2644) */
    state_key = 48;
    __quex_debug("state_key = 48\n");
    goto _3067;
_3425: /* (2646 from 2645) */
    state_key = 47;
    __quex_debug("state_key = 47\n");
    goto _3067;
_3426: /* (2647 from 2646) */
    state_key = 46;
    __quex_debug("state_key = 46\n");
    goto _3067;
_3427: /* (2648 from 2647) */
    state_key = 45;
    __quex_debug("state_key = 45\n");
    goto _3067;
_3428: /* (2649 from 2648) */
    state_key = 43;
    __quex_debug("state_key = 43\n");
    goto _3067;
_3429: /* (2650 from 2649) */
    state_key = 44;
    __quex_debug("state_key = 44\n");
    goto _3067;
_3430: /* (2651 from 2650) */
    state_key = 242;
    __quex_debug("state_key = 242\n");
    goto _3067;
_3431: /* (2653 from 2652) */
    state_key = 62;
    __quex_debug("state_key = 62\n");
    goto _3067;
_3432: /* (2654 from 2653) */
    state_key = 60;
    __quex_debug("state_key = 60\n");
    goto _3067;
_3433: /* (2655 from 2654) */
    state_key = 61;
    __quex_debug("state_key = 61\n");
    goto _3067;
_3434: /* (2656 from 2655) */
    state_key = 138;
    __quex_debug("state_key = 138\n");
    goto _3067;
_3435: /* (2657 from 2656) */
    state_key = 137;
    __quex_debug("state_key = 137\n");
    goto _3067;
_3436: /* (2658 from 2657) */
    state_key = 136;
    __quex_debug("state_key = 136\n");
    goto _3067;
_3437: /* (2659 from 2658) */
    state_key = 135;
    __quex_debug("state_key = 135\n");
    goto _3067;
_3438: /* (2660 from 2659) */
    state_key = 134;
    __quex_debug("state_key = 134\n");
    goto _3067;
_3439: /* (2661 from 2660) */
    state_key = 133;
    __quex_debug("state_key = 133\n");
    goto _3067;
_3440: /* (2662 from 2661) */
    state_key = 132;
    __quex_debug("state_key = 132\n");
    goto _3067;
_3441: /* (2663 from 2662) */
    state_key = 131;
    __quex_debug("state_key = 131\n");
    goto _3067;
_3442: /* (2664 from 2663) */
    state_key = 130;
    __quex_debug("state_key = 130\n");
    goto _3067;
_3443: /* (2665 from 2664) */
    state_key = 129;
    __quex_debug("state_key = 129\n");
    goto _3067;
_3444: /* (2666 from 2665) */
    state_key = 128;
    __quex_debug("state_key = 128\n");
    goto _3067;
_3445: /* (2667 from 2666) */
    state_key = 127;
    __quex_debug("state_key = 127\n");
    goto _3067;
_3446: /* (2668 from 2667) */
    state_key = 126;
    __quex_debug("state_key = 126\n");
    goto _3067;
_3447: /* (2669 from 2668) */
    state_key = 124;
    __quex_debug("state_key = 124\n");
    goto _3067;
_3448: /* (2670 from 2669) */
    state_key = 125;
    __quex_debug("state_key = 125\n");
    goto _3067;
_3151: /* (2674 from 2673) */
    state_key = 186;
    __quex_debug("state_key = 186\n");
    goto _3067;
_3449: /* (2675 from 2674) */
    state_key = 185;
    __quex_debug("state_key = 185\n");
    goto _3067;
_3450: /* (2676 from 2675) */
    state_key = 184;
    __quex_debug("state_key = 184\n");
    goto _3067;
_3451: /* (2677 from 2676) */
    state_key = 183;
    __quex_debug("state_key = 183\n");
    goto _3067;
_3452: /* (2678 from 2677) */
    state_key = 182;
    __quex_debug("state_key = 182\n");
    goto _3067;
_3453: /* (2679 from 2678) */
    state_key = 181;
    __quex_debug("state_key = 181\n");
    goto _3067;
_3454: /* (2680 from 2679) */
    state_key = 180;
    __quex_debug("state_key = 180\n");
    goto _3067;
_3455: /* (2681 from 2680) */
    state_key = 179;
    __quex_debug("state_key = 179\n");
    goto _3067;
_3456: /* (2682 from 2681) */
    state_key = 178;
    __quex_debug("state_key = 178\n");
    goto _3067;
_3457: /* (2683 from 2682) */
    state_key = 177;
    __quex_debug("state_key = 177\n");
    goto _3067;
_3458: /* (2684 from 2683) */
    state_key = 176;
    __quex_debug("state_key = 176\n");
    goto _3067;
_3459: /* (2685 from 2684) */
    state_key = 175;
    __quex_debug("state_key = 175\n");
    goto _3067;
_3460: /* (2686 from 2685) */
    state_key = 174;
    __quex_debug("state_key = 174\n");
    goto _3067;
_3461: /* (2687 from 2686) */
    state_key = 173;
    __quex_debug("state_key = 173\n");
    goto _3067;
_3462: /* (2688 from 2687) */
    state_key = 172;
    __quex_debug("state_key = 172\n");
    goto _3067;
_3463: /* (2689 from 2688) */
    state_key = 171;
    __quex_debug("state_key = 171\n");
    goto _3067;
_3464: /* (2690 from 2689) */
    state_key = 169;
    __quex_debug("state_key = 169\n");
    goto _3067;
_3465: /* (2691 from 2690) */
    state_key = 170;
    __quex_debug("state_key = 170\n");

_3067: /* (2487 from 2487) (2411 from 2411) (2576 from 2576) (2670 from 2670) (2628 from 2628) (2555 from 2555) (2506 from 2506) (2534 from 2534) (2467 from 2467) (2443 from 2443) (2691 from 2691) (2605 from 2605) (2651 from 2651) */

    ++(me->buffer._input_p);
    input = *(me->buffer._input_p);
    __quex_debug_template_state(3067, state_key);
    switch( input ) {
        case 0x0: QUEX_GOTO_RELOAD(__RELOAD_FORWARD, 3067, 3466);/* \0 */

        case 0x67: QUEX_GOTO_STATE(template_3067_target_0[state_key]);/* 'g' */


    }
_3466:
    __quex_debug_template_drop_out(3067, state_key);

    switch( state_key ) {
        case 0xF2: 
        case 0xF1:         goto TERMINAL_1817;
        case 0x55: 
        case 0x54: 
        case 0x53: 
        case 0x52: 
        case 0x51: 
        case 0x50: 
        case 0x4F: 
        case 0x4E: 
        case 0x4D: 
        case 0x4C: 
        case 0x4B: 
        case 0x4A: 
        case 0x49: 
        case 0x48: 
        case 0x47: 
        case 0x46: 
        case 0x45: 
        case 0x44: 
        case 0x43: 
        case 0x42: 
        case 0x41: 
        case 0x40: 
        case 0x5F: 
        case 0x5E: 
        case 0x3F: 
        case 0x3E: 
        case 0x3C: 
        case 0x3D: 
        case 0x5D: 
        case 0x5C: 
        case 0x5B: 
        case 0x5A: 
        case 0x59: 
        case 0x58: 
        case 0x57: 
        case 0x56:         goto TERMINAL_1816;
        case 0x1F: 
        case 0x1E: 
        case 0x1D: 
        case 0x1C: 
        case 0x1B: 
        case 0x1A: 
        case 0x19: 
        case 0x18: 
        case 0x17: 
        case 0x16: 
        case 0x15: 
        case 0x14: 
        case 0x13: 
        case 0x12: 
        case 0x11: 
        case 0x10: 
        case 0xF: 
        case 0xE: 
        case 0xD: 
        case 0xC: 
        case 0xB: 
        case 0xA: 
        case 0x9: 
        case 0x8: 
        case 0x7: 
        case 0x6: 
        case 0x5: 
        case 0x4: 
        case 0x3: 
        case 0x2: 
        case 0x0: 
        case 0x1: 
        case 0x2A: 
        case 0x29: 
        case 0x28: 
        case 0x27: 
        case 0x26: 
        case 0x25: 
        case 0x24: 
        case 0x23: 
        case 0x22: 
        case 0x21: 
        case 0x20:         __quex_assert(position[0] != 0x0);
me->buffer._input_p = position[0];

        goto TERMINAL_1828;
        case 0x3B: 
        case 0x3A: 
        case 0x39: 
        case 0x38: 
        case 0x37: 
        case 0x36: 
        case 0x35: 
        case 0x34: 
        case 0x33: 
        case 0x32: 
        case 0x31: 
        case 0x30: 
        case 0x2F: 
        case 0x2E: 
        case 0x2D: 
        case 0x2B: 
        case 0x2C:         goto TERMINAL_1818;
        case 0x6E: 
        case 0x6D: 
        case 0x6C: 
        case 0x6A: 
        case 0x6B: 
        case 0x7B: 
        case 0x7A: 
        case 0x79: 
        case 0x78: 
        case 0x77: 
        case 0x76: 
        case 0x75: 
        case 0x74: 
        case 0x73: 
        case 0x72: 
        case 0x71: 
        case 0x70: 
        case 0x6F:         goto TERMINAL_1814;
        case 0xF4: 
        case 0xF3: 
        case 0xF5:         me->buffer._input_p -= 7; 
        goto TERMINAL_1828;
        case 0x69: 
        case 0x68: 
        case 0x67: 
        case 0x66: 
        case 0x65: 
        case 0x64: 
        case 0x63: 
        case 0x62: 
        case 0x60: 
        case 0x61:         goto TERMINAL_1815;
        case 0xCC: 
        case 0xCB: 
        case 0xCA: 
        case 0xC9: 
        case 0xC8: 
        case 0xC7: 
        case 0xC6: 
        case 0xC5: 
        case 0xC4: 
        case 0xC3: 
        case 0xC2: 
        case 0xC1: 
        case 0xC0: 
        case 0xBF: 
        case 0xBE: 
        case 0xBD: 
        case 0xBC: 
        case 0xBB: 
        case 0xBA: 
        case 0xB9: 
        case 0xB8: 
        case 0xB7: 
        case 0xB6: 
        case 0xB5: 
        case 0xB4: 
        case 0xB3: 
        case 0xB2: 
        case 0xB1: 
        case 0xB0: 
        case 0xAF: 
        case 0xAE: 
        case 0xAD: 
        case 0xAC: 
        case 0xAB: 
        case 0xA9: 
        case 0xAA: 
        case 0xF0: 
        case 0xEF: 
        case 0xEE: 
        case 0xED: 
        case 0xEC: 
        case 0xEB: 
        case 0xEA: 
        case 0xE9: 
        case 0xE8: 
        case 0xE7: 
        case 0xE6: 
        case 0xE5: 
        case 0xE4: 
        case 0xE3: 
        case 0xE2: 
        case 0xE1: 
        case 0xE0: 
        case 0xDF: 
        case 0xDE: 
        case 0xDD: 
        case 0xDC: 
        case 0xDB: 
        case 0xDA: 
        case 0xD9: 
        case 0xD8: 
        case 0xD7: 
        case 0xD6: 
        case 0xD5: 
        case 0xD4: 
        case 0xD3: 
        case 0xD2: 
        case 0xD1: 
        case 0xD0: 
        case 0xCF: 
        case 0xCE: 
        case 0xCD:         goto TERMINAL_1812;
        case 0xA5: 
        case 0xA4: 
        case 0xA3: 
        case 0xA2: 
        case 0xA1: 
        case 0xA0: 
        case 0x9F: 
        case 0x9E: 
        case 0x9D: 
        case 0x9C: 
        case 0x9B: 
        case 0x9A: 
        case 0x7C: 
        case 0x98: 
        case 0x8A: 
        case 0x89: 
        case 0x88: 
        case 0x87: 
        case 0x86: 
        case 0x85: 
        case 0x84: 
        case 0x83: 
        case 0x82: 
        case 0x81: 
        case 0x80: 
        case 0x7F: 
        case 0x7E: 
        case 0x99: 
        case 0x7D: 
        case 0x97: 
        case 0x96: 
        case 0x95: 
        case 0x94: 
        case 0x93: 
        case 0x92: 
        case 0x91: 
        case 0x90: 
        case 0x8F: 
        case 0x8E: 
        case 0x8D: 
        case 0x8C: 
        case 0x8B: 
        case 0xA8: 
        case 0xA7: 
        case 0xA6:         goto TERMINAL_1813;

    }
#   undef __QUEX_DEBUG_MAP_STATE_KEY_TO_STATE

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

TERMINAL_1812:
    __quex_debug("* terminal 1812:   g{3,}ug{3,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 20 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3903 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1813:
    __quex_debug("* terminal 1813:   g{6,}u{1,2}g{6,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 23 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3918 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1814:
    __quex_debug("* terminal 1814:   g{12,}u{1,4}g{12,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 26 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3933 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1815:
    __quex_debug("* terminal 1815:   g{1,}u{1,4}g{12,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 29 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3948 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1816:
    __quex_debug("* terminal 1816:   g{12,}u{1,4}g{1,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 30 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3963 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1817:
    __quex_debug("* terminal 1817:   g{3,}u{1,7}g{20,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 33 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3978 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1818:
    __quex_debug("* terminal 1818:   g{20,}u{1,7}g{3,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 34 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 3993 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1819:
    __quex_debug("* terminal 1819:   y{1,}oy{5,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 37 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_GOAL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4008 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1820:
    __quex_debug("* terminal 1820:   y{5,}oy{1,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 38 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_GOAL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4023 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1821:
    __quex_debug("* terminal 1821:   y{5,}u{1,3}y{1,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 41 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_GOAL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4038 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1822:
    __quex_debug("* terminal 1822:   y{1,}u{1,3}y{5,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 42 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_GOAL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4053 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1823:
    __quex_debug("* terminal 1823:   o{5,}u{1,3}o{1,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 45 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_BALL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4068 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1824:
    __quex_debug("* terminal 1824:   o{1,}u{1,3}o{5,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 46 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_BALL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4083 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1825:
    __quex_debug("* terminal 1825:   w{1,}u{1,3}w{5,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 49 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_LINE);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4098 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1826:
    __quex_debug("* terminal 1826:   w{5,}u{1,3}w{1,}\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 50 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_LINE);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4113 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1827:
    __quex_debug("* terminal 1827:   w+\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 53 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_LINE);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4128 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1828:
    __quex_debug("* terminal 1828:   g+\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 54 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_FIELD);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4143 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1829:
    __quex_debug("* terminal 1829:   o+\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 55 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_BALL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4158 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1830:
    __quex_debug("* terminal 1830:   y+\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 56 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_GOAL);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4173 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1831:
    __quex_debug("* terminal 1831:   p+\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 57 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_MAGENTA_TEAM);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4188 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1832:
    __quex_debug("* terminal 1832:   c+\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 58 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_CYAN_TEAM);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4203 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;

TERMINAL_1833:
    __quex_debug("* terminal 1833:   u+\n");
    __QUEX_IF_COUNT_SHIFT_VALUES();
__QUEX_IF_COUNT_COLUMNS_ADD(LexemeL);
    {
#   line 59 "Replacement.qx"
    self_write_token_p()->number = (LexemeL);
    self_send(QUEX_TKN_UNCLASSIFIED);
    QUEX_SETTING_AFTER_SEND_CONTINUE_OR_RETURN();
    
#   line 4218 "Replacement.cpp"

    }
    goto __REENTRY_PREPARATION;


_3090: /* TERMINAL: FAILURE */
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
_3081:
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
        case 2337: { goto _2337; }
        case 3067: { goto _3067; }
        case 3075: { goto _3075; }
        case 3076: { goto _3076; }
        case 3077: { goto _3077; }
        case 3078: { goto _3078; }
        case 3079: { goto _3079; }
        case 3081: { goto _3081; }
        case 3082: { goto _3082; }
        case 3083: { goto _3083; }
        case 3084: { goto _3084; }
        case 3085: { goto _3085; }
        case 3086: { goto _3086; }
        case 3087: { goto _3087; }
        case 3088: { goto _3088; }
        case 3091: { goto _3091; }
        case 3092: { goto _3092; }
        case 3093: { goto _3093; }
        case 3094: { goto _3094; }
        case 3095: { goto _3095; }
        case 3096: { goto _3096; }
        case 3097: { goto _3097; }
        case 3098: { goto _3098; }
        case 3099: { goto _3099; }
        case 3100: { goto _3100; }
        case 3101: { goto _3101; }
        case 3102: { goto _3102; }
        case 3103: { goto _3103; }
        case 3104: { goto _3104; }
        case 3105: { goto _3105; }
        case 3106: { goto _3106; }
        case 3107: { goto _3107; }
        case 3108: { goto _3108; }
        case 3109: { goto _3109; }
        case 3110: { goto _3110; }
        case 3111: { goto _3111; }
        case 3112: { goto _3112; }
        case 3113: { goto _3113; }
        case 3114: { goto _3114; }
        case 3115: { goto _3115; }
        case 3116: { goto _3116; }
        case 3117: { goto _3117; }
        case 3118: { goto _3118; }
        case 3119: { goto _3119; }
        case 3120: { goto _3120; }
        case 3121: { goto _3121; }
        case 3122: { goto _3122; }
        case 3123: { goto _3123; }
        case 3124: { goto _3124; }
        case 3125: { goto _3125; }
        case 3126: { goto _3126; }
        case 3127: { goto _3127; }
        case 3128: { goto _3128; }
        case 3129: { goto _3129; }
        case 3130: { goto _3130; }
        case 3131: { goto _3131; }
        case 3132: { goto _3132; }
        case 3133: { goto _3133; }
        case 3134: { goto _3134; }
        case 3135: { goto _3135; }
        case 3136: { goto _3136; }
        case 3137: { goto _3137; }
        case 3138: { goto _3138; }
        case 3139: { goto _3139; }
        case 3140: { goto _3140; }
        case 3141: { goto _3141; }
        case 3142: { goto _3142; }
        case 3143: { goto _3143; }
        case 3144: { goto _3144; }
        case 3145: { goto _3145; }
        case 3146: { goto _3146; }
        case 3147: { goto _3147; }
        case 3148: { goto _3148; }
        case 3149: { goto _3149; }
        case 3150: { goto _3150; }
        case 3151: { goto _3151; }
        case 3152: { goto _3152; }
        case 3153: { goto _3153; }
        case 3154: { goto _3154; }
        case 3155: { goto _3155; }
        case 3156: { goto _3156; }
        case 3157: { goto _3157; }
        case 3158: { goto _3158; }
        case 3159: { goto _3159; }
        case 3160: { goto _3160; }
        case 3161: { goto _3161; }
        case 3162: { goto _3162; }
        case 3163: { goto _3163; }
        case 3164: { goto _3164; }
        case 3165: { goto _3165; }
        case 3166: { goto _3166; }
        case 3167: { goto _3167; }
        case 3168: { goto _3168; }
        case 3169: { goto _3169; }
        case 3170: { goto _3170; }
        case 3171: { goto _3171; }
        case 3172: { goto _3172; }
        case 3173: { goto _3173; }
        case 3174: { goto _3174; }
        case 3175: { goto _3175; }
        case 3176: { goto _3176; }
        case 3177: { goto _3177; }
        case 3178: { goto _3178; }
        case 3179: { goto _3179; }
        case 3180: { goto _3180; }
        case 3181: { goto _3181; }
        case 3182: { goto _3182; }
        case 3183: { goto _3183; }
        case 3184: { goto _3184; }
        case 3185: { goto _3185; }
        case 3186: { goto _3186; }
        case 3187: { goto _3187; }
        case 3188: { goto _3188; }
        case 3189: { goto _3189; }
        case 3190: { goto _3190; }
        case 3191: { goto _3191; }
        case 3192: { goto _3192; }
        case 3193: { goto _3193; }
        case 3194: { goto _3194; }
        case 3195: { goto _3195; }
        case 3196: { goto _3196; }
        case 3197: { goto _3197; }
        case 3198: { goto _3198; }
        case 3199: { goto _3199; }
        case 3200: { goto _3200; }
        case 3201: { goto _3201; }
        case 3202: { goto _3202; }
        case 3203: { goto _3203; }
        case 3204: { goto _3204; }
        case 3205: { goto _3205; }
        case 3206: { goto _3206; }
        case 3207: { goto _3207; }
        case 3208: { goto _3208; }
        case 3209: { goto _3209; }
        case 3210: { goto _3210; }
        case 3211: { goto _3211; }
        case 3212: { goto _3212; }
        case 3213: { goto _3213; }
        case 3214: { goto _3214; }
        case 3215: { goto _3215; }
        case 3216: { goto _3216; }
        case 3217: { goto _3217; }
        case 3218: { goto _3218; }
        case 3219: { goto _3219; }
        case 3220: { goto _3220; }
        case 3221: { goto _3221; }
        case 3222: { goto _3222; }
        case 3223: { goto _3223; }
        case 3224: { goto _3224; }
        case 3225: { goto _3225; }
        case 3226: { goto _3226; }
        case 3227: { goto _3227; }
        case 3228: { goto _3228; }
        case 3229: { goto _3229; }
        case 3230: { goto _3230; }
        case 3231: { goto _3231; }
        case 3232: { goto _3232; }
        case 3233: { goto _3233; }
        case 3234: { goto _3234; }
        case 3235: { goto _3235; }
        case 3236: { goto _3236; }
        case 3237: { goto _3237; }
        case 3238: { goto _3238; }
        case 3239: { goto _3239; }
        case 3240: { goto _3240; }
        case 3241: { goto _3241; }
        case 3242: { goto _3242; }
        case 3243: { goto _3243; }
        case 3244: { goto _3244; }
        case 3245: { goto _3245; }
        case 3246: { goto _3246; }
        case 3247: { goto _3247; }
        case 3248: { goto _3248; }
        case 3249: { goto _3249; }
        case 3250: { goto _3250; }
        case 3251: { goto _3251; }
        case 3252: { goto _3252; }
        case 3253: { goto _3253; }
        case 3254: { goto _3254; }
        case 3255: { goto _3255; }
        case 3256: { goto _3256; }
        case 3257: { goto _3257; }
        case 3258: { goto _3258; }
        case 3259: { goto _3259; }
        case 3260: { goto _3260; }
        case 3261: { goto _3261; }
        case 3262: { goto _3262; }
        case 3263: { goto _3263; }
        case 3264: { goto _3264; }
        case 3265: { goto _3265; }
        case 3266: { goto _3266; }
        case 3267: { goto _3267; }
        case 3268: { goto _3268; }
        case 3269: { goto _3269; }
        case 3270: { goto _3270; }
        case 3271: { goto _3271; }
        case 3272: { goto _3272; }
        case 3273: { goto _3273; }
        case 3274: { goto _3274; }
        case 3275: { goto _3275; }
        case 3276: { goto _3276; }
        case 3277: { goto _3277; }
        case 3278: { goto _3278; }
        case 3279: { goto _3279; }
        case 3280: { goto _3280; }
        case 3281: { goto _3281; }
        case 3282: { goto _3282; }
        case 3283: { goto _3283; }
        case 3284: { goto _3284; }
        case 3285: { goto _3285; }
        case 3286: { goto _3286; }
        case 3287: { goto _3287; }
        case 3288: { goto _3288; }
        case 3289: { goto _3289; }
        case 3290: { goto _3290; }
        case 3291: { goto _3291; }
        case 3292: { goto _3292; }
        case 3293: { goto _3293; }
        case 3294: { goto _3294; }
        case 3295: { goto _3295; }
        case 3296: { goto _3296; }
        case 3297: { goto _3297; }
        case 3298: { goto _3298; }
        case 3299: { goto _3299; }
        case 3300: { goto _3300; }
        case 3301: { goto _3301; }
        case 3302: { goto _3302; }
        case 3303: { goto _3303; }
        case 3304: { goto _3304; }
        case 3305: { goto _3305; }
        case 3306: { goto _3306; }
        case 3307: { goto _3307; }
        case 3308: { goto _3308; }
        case 3309: { goto _3309; }
        case 3310: { goto _3310; }
        case 3311: { goto _3311; }
        case 3312: { goto _3312; }
        case 3313: { goto _3313; }
        case 3314: { goto _3314; }
        case 3315: { goto _3315; }
        case 3316: { goto _3316; }
        case 3317: { goto _3317; }
        case 3318: { goto _3318; }
        case 3319: { goto _3319; }
        case 3320: { goto _3320; }
        case 3321: { goto _3321; }
        case 3322: { goto _3322; }
        case 3323: { goto _3323; }
        case 3324: { goto _3324; }
        case 3325: { goto _3325; }
        case 3326: { goto _3326; }
        case 3327: { goto _3327; }
        case 3328: { goto _3328; }
        case 3329: { goto _3329; }
        case 3330: { goto _3330; }
        case 3331: { goto _3331; }
        case 3332: { goto _3332; }
        case 3333: { goto _3333; }
        case 3334: { goto _3334; }
        case 3335: { goto _3335; }
        case 3336: { goto _3336; }
        case 3337: { goto _3337; }
        case 3338: { goto _3338; }
        case 3339: { goto _3339; }
        case 3340: { goto _3340; }
        case 3341: { goto _3341; }
        case 3342: { goto _3342; }
        case 3343: { goto _3343; }
        case 3344: { goto _3344; }
        case 3345: { goto _3345; }
        case 3346: { goto _3346; }
        case 3347: { goto _3347; }
        case 3348: { goto _3348; }
        case 3349: { goto _3349; }
        case 3350: { goto _3350; }
        case 3351: { goto _3351; }
        case 3352: { goto _3352; }
        case 3353: { goto _3353; }
        case 3354: { goto _3354; }
        case 3355: { goto _3355; }
        case 3356: { goto _3356; }
        case 3357: { goto _3357; }
        case 3358: { goto _3358; }
        case 3359: { goto _3359; }
        case 3360: { goto _3360; }
        case 3361: { goto _3361; }
        case 3362: { goto _3362; }
        case 3363: { goto _3363; }
        case 3364: { goto _3364; }
        case 3365: { goto _3365; }
        case 3366: { goto _3366; }
        case 3367: { goto _3367; }
        case 3368: { goto _3368; }
        case 3369: { goto _3369; }
        case 3370: { goto _3370; }
        case 3371: { goto _3371; }
        case 3372: { goto _3372; }
        case 3373: { goto _3373; }
        case 3374: { goto _3374; }
        case 3375: { goto _3375; }
        case 3376: { goto _3376; }
        case 3377: { goto _3377; }
        case 3378: { goto _3378; }
        case 3379: { goto _3379; }
        case 3380: { goto _3380; }
        case 3381: { goto _3381; }
        case 3382: { goto _3382; }
        case 3383: { goto _3383; }
        case 3384: { goto _3384; }
        case 3385: { goto _3385; }
        case 3386: { goto _3386; }
        case 3387: { goto _3387; }
        case 3388: { goto _3388; }
        case 3389: { goto _3389; }
        case 3390: { goto _3390; }
        case 3391: { goto _3391; }
        case 3392: { goto _3392; }
        case 3393: { goto _3393; }
        case 3394: { goto _3394; }
        case 3395: { goto _3395; }
        case 3396: { goto _3396; }
        case 3397: { goto _3397; }
        case 3398: { goto _3398; }
        case 3399: { goto _3399; }
        case 3400: { goto _3400; }
        case 3401: { goto _3401; }
        case 3402: { goto _3402; }
        case 3403: { goto _3403; }
        case 3404: { goto _3404; }
        case 3405: { goto _3405; }
        case 3406: { goto _3406; }
        case 3407: { goto _3407; }
        case 3408: { goto _3408; }
        case 3409: { goto _3409; }
        case 3410: { goto _3410; }
        case 3411: { goto _3411; }
        case 3412: { goto _3412; }
        case 3413: { goto _3413; }
        case 3414: { goto _3414; }
        case 3415: { goto _3415; }
        case 3416: { goto _3416; }
        case 3417: { goto _3417; }
        case 3418: { goto _3418; }
        case 3419: { goto _3419; }
        case 3420: { goto _3420; }
        case 3421: { goto _3421; }
        case 3422: { goto _3422; }
        case 3423: { goto _3423; }
        case 3424: { goto _3424; }
        case 3425: { goto _3425; }
        case 3426: { goto _3426; }
        case 3427: { goto _3427; }
        case 3428: { goto _3428; }
        case 3429: { goto _3429; }
        case 3430: { goto _3430; }
        case 3431: { goto _3431; }
        case 3432: { goto _3432; }
        case 3433: { goto _3433; }
        case 3434: { goto _3434; }
        case 3435: { goto _3435; }
        case 3436: { goto _3436; }
        case 3437: { goto _3437; }
        case 3438: { goto _3438; }
        case 3439: { goto _3439; }
        case 3440: { goto _3440; }
        case 3441: { goto _3441; }
        case 3442: { goto _3442; }
        case 3443: { goto _3443; }
        case 3444: { goto _3444; }
        case 3445: { goto _3445; }
        case 3446: { goto _3446; }
        case 3447: { goto _3447; }
        case 3448: { goto _3448; }
        case 3449: { goto _3449; }
        case 3450: { goto _3450; }
        case 3451: { goto _3451; }
        case 3452: { goto _3452; }
        case 3453: { goto _3453; }
        case 3454: { goto _3454; }
        case 3455: { goto _3455; }
        case 3456: { goto _3456; }
        case 3457: { goto _3457; }
        case 3458: { goto _3458; }
        case 3459: { goto _3459; }
        case 3460: { goto _3460; }
        case 3461: { goto _3461; }
        case 3462: { goto _3462; }
        case 3463: { goto _3463; }
        case 3464: { goto _3464; }
        case 3465: { goto _3465; }
        case 3466: { goto _3466; }

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
#   undef self
}
#include <quex/code_base/temporary_macros_off>
    /* BEGIN: MODE PATTERNS
     * 
     * MODE: ReplacementRules
     * 
     *     PATTERN-ACTION PAIRS:
     *       ON_END_OF_STREAM
     *       ON_FAILURE
     *       (1812) ReplacementRules: g{3,}ug{3,}
     *       (1813) ReplacementRules: g{6,}u{1,2}g{6,}
     *       (1814) ReplacementRules: g{12,}u{1,4}g{12,}
     *       (1815) ReplacementRules: g{1,}u{1,4}g{12,}
     *       (1816) ReplacementRules: g{12,}u{1,4}g{1,}
     *       (1817) ReplacementRules: g{3,}u{1,7}g{20,}
     *       (1818) ReplacementRules: g{20,}u{1,7}g{3,}
     *       (1819) ReplacementRules: y{1,}oy{5,}
     *       (1820) ReplacementRules: y{5,}oy{1,}
     *       (1821) ReplacementRules: y{5,}u{1,3}y{1,}
     *       (1822) ReplacementRules: y{1,}u{1,3}y{5,}
     *       (1823) ReplacementRules: o{5,}u{1,3}o{1,}
     *       (1824) ReplacementRules: o{1,}u{1,3}o{5,}
     *       (1825) ReplacementRules: w{1,}u{1,3}w{5,}
     *       (1826) ReplacementRules: w{5,}u{1,3}w{1,}
     *       (1827) ReplacementRules: w+
     *       (1828) ReplacementRules: g+
     *       (1829) ReplacementRules: o+
     *       (1830) ReplacementRules: y+
     *       (1831) ReplacementRules: p+
     *       (1832) ReplacementRules: c+
     *       (1833) ReplacementRules: u+
     * 
     * 
     * END: MODE PATTERNS
     */
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

