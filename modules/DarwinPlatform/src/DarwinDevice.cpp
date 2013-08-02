#include "DarwinDevice.h"

Darwin::DarwinDevice::DarwinDevice(UART& coms, int id) : m_coms(coms), m_id(id) {}

bool Darwin::DarwinDevice::ping() {
    
    CommandResult result = m_coms.execute(PingCommand(m_id));
    
    return result.header.errorcode == ErrorCode::NONE;
}

bool Darwin::DarwinDevice::reset() {
    
    CommandResult result = m_coms.execute(ResetCommand(m_id));
    
    return result.header.errorcode == ErrorCode::NONE;
}