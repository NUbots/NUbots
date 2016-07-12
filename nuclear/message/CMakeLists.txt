# Switch between enhanced messages and normal messages
IF(NUCLEAR_ENHANCED_MESSAGES)
    INCLUDE(EnhancedMessages.cmake)
ELSE()
    INCLUDE(StandardMessages.cmake)
ENDIF(NUCLEAR_ENHANCED_MESSAGES)
