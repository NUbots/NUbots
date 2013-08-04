#include "DarwinDevice.h"

Darwin::DarwinDevice::DarwinDevice(UART& coms, int id) : m_coms(coms), m_id(id) {}

bool Darwin::DarwinDevice::ping() {
    
    // Ping and get the result
    CommandResult result = m_coms.execute(PingCommand(m_id));
    
    // Check if there was an error code
    return result.header.errorcode == ErrorCode::NONE;
}