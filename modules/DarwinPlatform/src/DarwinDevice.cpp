#include "DarwinDevice.h"

Darwin::DarwinDevice::DarwinDevice(UART& coms, int id) : m_coms(coms), m_id(id) {}

bool Darwin::DarwinDevice::ping() {
    
    PingCommand cmd(m_id);
    CommandResult result = m_coms.execute(cmd);
    
    return result.header.errorcode == ErrorCode::NONE;
}