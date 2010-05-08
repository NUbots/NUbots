#ifndef STREAMFILEREADER_H
#define STREAMFILEREADER_H
#include <string>
#include <fstream>
#include <map>
#include "Tools/FileFormats/TimestampedData.h"
#include <QDebug>

template<class C>
class StreamFileReader
{
    typedef std::map<double,std::fstream::pos_type> FileIndex;
    typedef FileIndex::value_type IndexEntry;
public:
    StreamFileReader(): m_fileEndLocation(0)
    {
        m_dataBuffer = new C();
    }

    StreamFileReader(const std::string& filename): m_fileEndLocation(0)
    {
        m_dataBuffer = new C();
        OpenFile(filename);
    }

    ~StreamFileReader()
    {
        delete m_dataBuffer;
    }

    bool OpenFile(const std::string& filename)
    {
        m_file.open(filename.c_str(),std::ios_base::in | std::ios_base::binary);
        if(m_file.good())
        {
            m_file.seekg(0,std::ios_base::end);
            m_fileEndLocation = m_file.tellg();
            GenerateIndex();
        }
        return m_file.good();
    }

    void CloseFile()
    {
        m_file.close();
    }

    void DisplayIndex()
    {
        qDebug() << m_index.size();
        FileIndex::iterator indexEntry = m_index.begin();
        while(indexEntry != m_index.end())
        {
            qDebug() << "Time: " << (*indexEntry).first << "\tIndex: " << (*indexEntry).second;
            ++indexEntry;
        }
    }

    bool HasTime(double time)
    {
        FileIndex::iterator pos = m_index.find(floor(time));
        return (pos != m_index.end());
    }

    C* ReadFrameAtTime(double time)
    {
        std::fstream::pos_type position = GetPositionFromTime(time);
        qDebug() << "Getting Frame at: " << position;
        if(position != -1)
        {
            return ReadFrame(position);
        }
    }

    C* ReadFrameNumber(int frameNumber)
    {
        if((frameNumber < m_index.size()) && (frameNumber >= 0))
        {
            return ReadFrame(m_index[frameNumber]);
        }
        else
        {
            return NULL;
        }
    }

    double FindClosestTime(double time)
    {
        FileIndex::iterator pos = m_index.lower_bound(floor(time));
        if(pos != m_index.end())
        {
            if(((*pos).first > time) && (pos != m_index.begin()))
            {
                --pos;
            }
            return (*pos).first;
        }
        else
        {
            return 0;
        }
    }

private:
    C* ReadFrame(std::fstream::pos_type startingLocation)
    {
        if(m_file.is_open() && ValidStartingLocation(startingLocation))
        {
            m_file.seekg(startingLocation,std::ios_base::beg);
            try{
                m_file >> (*m_dataBuffer);
            }   catch(...){return NULL;}
        }
        return m_dataBuffer;
    }

    bool ValidStartingLocation(std::fstream::pos_type startingLocation)
    {
        return (startingLocation < m_fileEndLocation);
    }

    std::fstream::pos_type GetPositionFromTime(double time)
    {
        FileIndex::iterator pos = m_index.lower_bound(floor(time));
        if(pos != m_index.end())
        {
            if(((*pos).first > time) && (pos != m_index.begin()))
            {
                --pos;
            }
            return (*pos).second;
        }
        else
        {
            return -1;
        }
    }

    void GenerateIndex()
    {
        if (m_file.is_open())
        {
            double timestamp = 0.0;
            std::fstream::pos_type origPos = m_file.tellg();
            m_file.seekg(0,std::ios_base::beg);
            m_index.clear();
            std::fstream::pos_type filePosition;
            while (m_file.good())
            {
                filePosition = m_file.tellg();
                try{
                    m_file >> (*m_dataBuffer);
                }   catch(...){break;}
                timestamp = (static_cast<TimestampedData*>(m_dataBuffer))->GetTimestamp();
                m_index.insert(IndexEntry(floor(timestamp),filePosition));
            }
            m_file.clear();
            m_file.seekg(origPos,std::ios_base::beg);
        }
    }

    // Member variables
    FileIndex m_index;
    C* m_dataBuffer;
    std::fstream m_file;
    std::fstream::pos_type m_fileEndLocation;

};

#endif // STREAMFILEREADER_H
