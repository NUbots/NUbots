#include "LogRecorder.h"
#include "debug.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUImage/NUImage.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

LogFileWriter::LogFileWriter(std::string data_type)
{
    m_data_name = data_type;
    m_status = lf_UNKNOWN;
}


LogFileWriter::~LogFileWriter()
{
    m_file_stream.close();
}

std::string LogFileWriter::toString()
{
    std::stringstream result;
    return result.str();
}

bool LogFileWriter::Open(std::string file_path)
{
    debug << "LW:Opening log file: " << file_path << " - ";
    m_file_stream.open(file_path.c_str(),std::ios_base::out | std::ios_base::trunc);
    if(m_file_stream.is_open() && m_file_stream.good())
    {
        m_file_name = file_path;
        m_status = lf_OPEN;
        debug << "SUCCESS" << std::endl;
        return true;
    }
    else
    {
        m_status = lf_UNAVAILABLE;
        debug << "FAILED" << std::endl;
        return false;
    }
}

bool LogFileWriter::Close()
{
    m_file_stream.close();
    if(!m_file_stream.is_open())
    {
        m_status = lf_CLOSED;
        return true;
    }
    else
    {
        m_status = lf_UNKNOWN;
        return false;
    }
}

LogRecorder::LogRecorder(int playerNumber)
{
    m_player_number = playerNumber;
    m_log_writers.push_back(new LogFileWriter("sensor"));
    m_log_writers.push_back(new LogFileWriter("image"));
    m_log_writers.push_back(new LogFileWriter("object"));
    m_log_writers.push_back(new LogFileWriter("teaminfo"));
    m_log_writers.push_back(new LogFileWriter("gameinfo"));
}

LogRecorder::~LogRecorder()
{
    std::vector<LogFileWriter*>::iterator it;
    for(it = m_log_writers.begin(); it != m_log_writers.end(); ++it)
    {
        delete (*it);
    }
    m_log_writers.clear();
    return;
}

bool LogRecorder::SetLogging(std::string dataType, bool enabled)
{
    LogFileWriter* target = GetDataWriter(dataType);
    if(target == NULL) return false;
    bool success = false;
    bool log_reset_required = false;
    switch(target->GetFileStatus())
    {
    case lf_CLOSED:
        if(enabled)
        {
            target->Open(GetLogPath(m_player_number, dataType));
            log_reset_required = true;
            success = (target->GetFileStatus() == lf_OPEN);
        }
        else
        {
            success = true;
        }
        break;
    case lf_OPEN:
        if(enabled)
        {
            success = true;
        }
        else
        {
            target->Close();
            log_reset_required = true;
            success = (target->GetFileStatus() == lf_CLOSED);
        }
        break;
    case lf_UNAVAILABLE:
        success = false;
        break;
    case lf_UNKNOWN:
        if(enabled)
        {
            target->Open(GetLogPath(m_player_number,dataType));
            log_reset_required = true;
            success = (target->GetFileStatus() == lf_OPEN);
        }
        else
        {
            target->Close();
            log_reset_required = true;
            success = (target->GetFileStatus() == lf_CLOSED);
        }
        break;
    default:
        success = false;
    }
    return success;
}

bool LogRecorder::HasDataType(std::string dataType)
{
    std::vector<LogFileWriter*>::iterator it;
    for(it = m_log_writers.begin(); it != m_log_writers.end(); ++it)
    {
        if((*it)->GetDataType() == dataType) return true;
    }
    return false;
}

LogFileWriter* LogRecorder::GetDataWriter(std::string dataType)
{
    std::vector<LogFileWriter*>::iterator it;
    for(it = m_log_writers.begin(); it != m_log_writers.end(); it++)
    {
        if((*it)->GetDataType() == dataType) return *it;
    }
    return NULL;
}

bool LogRecorder::WriteData(NUBlackboard* theBlackboard)
{
    std::vector<LogFileWriter*>::iterator it;
    for(it = m_log_writers.begin(); it != m_log_writers.end(); ++it)
    {
        if((*it)->GetFileStatus() == lf_OPEN)
        {
            std::string data_type = (*it)->GetDataType();
            if(data_type == "sensor")
                (*it)->GetFile() << *(theBlackboard->Sensors) << std::flush;
            else if(data_type == "image")
                (*it)->GetFile() << *(theBlackboard->Image) << std::flush;
            else if(data_type == "object")
                (*it)->GetFile() << *(theBlackboard->Objects) << std::flush;
            else if(data_type == "gameinfo")
                (*it)->GetFile() << *(theBlackboard->GameInfo) << std::flush;
            else if(data_type == "teaminfo")
                (*it)->GetFile() << *(theBlackboard->TeamInfo) << std::flush;
        }
    }
    return true;
}
