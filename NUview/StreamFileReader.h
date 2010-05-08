#ifndef STREAMFILEREADER_H
#define STREAMFILEREADER_H
#include <string>
#include <fstream>
#include <map>
#include <vector>
#include "Tools/FileFormats/TimestampedData.h"
#include <QDebug>

template<class C>
class StreamFileReader
{
    typedef std::map<double,std::fstream::pos_type> FileIndex;
    typedef std::vector<double> TimeIndex;
    typedef FileIndex::iterator IndexIterator;
    typedef FileIndex::value_type IndexEntry;
    typedef std::fstream::pos_type Position;
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
        IndexIterator indexEntry = m_index.begin();
        while(indexEntry != m_index.end())
        {
            qDebug() << "Time: " << (*indexEntry).first << "\tIndex: " << (*indexEntry).second;
            ++indexEntry;
        }
    }

    bool HasTime(double time)
    {
        IndexIterator pos = m_index.find(floor(time));
        return (pos != m_index.end());
    }

    C* ReadFrameAtTime(double time)
    {
        IndexIterator entry = GetIndexFromTime(time);
        qDebug() << "Getting Frame at: " << (*entry).second;
        if(entry != m_index.end())
        {
            return ReadFrame(entry);
        }
        else
        {
            return NULL;
        }
    }

    C* ReadFrameNumber(int frameNumber)
    {

            float time = TimeAtPosition(frameNumber-1);
            if(time)
                return ReadFrame(m_index[time]);
            else
                return NULL;
    }

    C* ReadFirstFrame()
    {
        IndexIterator entry = m_index.begin();
        if(entry != m_index.end())
        {
            return ReadFrame(entry);
        }
        else
        {
            return NULL;
        }
    }

    C* ReadNextFrame()
    {
        IndexIterator entry = m_selectedFrame;
        ++entry;
        if(entry != m_index.end())
        {
            return ReadFrame(entry);
        }
        else
        {
            return NULL;
        }
    }

    C* ReadPrevFrame()
    {
        IndexIterator entry = m_selectedFrame;
        if(entry != m_index.begin())
        {
            --entry;
            return ReadFrame(entry);
        }
        else
        {
            return NULL;
        }
    }

    C* ReadLastFrame()
    {
        IndexIterator entry = m_index.end();
        if(entry != m_index.begin())
        {
            --entry;
            return ReadFrame(entry);
        }
        else
        {
            return NULL;
        }
    }

    double FindClosestTime(double time)
    {
        IndexIterator pos = GetIndexFromTime(time);
        if(pos != m_index.end())
        {
            return (*pos).first;
        }
        else
        {
            return 0;
        }
    }

    int GetTotalFrames()
    {
        if(m_file.is_open())
        {
            return m_index.size();
        }
        else
        {
            return 0;
        }
    }

    double TimeAtPosition(unsigned int position)
    {
        if((position <= m_index.size()) && (position > 0))
        {
            return m_timeIndex[position-1];
        }
        else
        {
            return 0.0;
        }
    }

private:
    C* ReadFrame(IndexIterator entry)
    {
        Position startingLocation = (*entry).second;
        if(m_file.is_open() && ValidStartingLocation(startingLocation))
        {
            m_file.seekg(startingLocation,std::ios_base::beg);
            try{
                m_file >> (*m_dataBuffer);
                m_selectedFrame = entry;
                return m_dataBuffer;
            }   catch(...){}

        }
        return NULL;
    }

    bool ValidStartingLocation(std::fstream::pos_type startingLocation)
    {
        return (startingLocation < m_fileEndLocation);
    }

    IndexIterator GetIndexFromTime(double time)
    {
        IndexIterator pos = m_index.lower_bound(floor(time));
        if(pos != m_index.end())
        {
            if(((*pos).first > time) && (pos != m_index.begin()))
            {
                --pos;
            }
        }
        return pos;
    }

    void GenerateIndex()
    {
        if (m_file.is_open())
        {
            double timestamp = 0.0;
            Position origPos = m_file.tellg();
            m_file.seekg(0,std::ios_base::beg);
            m_index.clear();
            m_timeIndex.clear();
            Position filePosition;
            while (m_file.good())
            {
                filePosition = m_file.tellg();
                try{
                    m_file >> (*m_dataBuffer);
                }   catch(...){break;}
                timestamp = (static_cast<TimestampedData*>(m_dataBuffer))->GetTimestamp();
                timestamp = floor(timestamp);
                m_index.insert(IndexEntry(timestamp,filePosition));
                m_timeIndex.push_back(timestamp);
            }
            m_file.clear();
            m_file.seekg(origPos,std::ios_base::beg);
        }
    }

    // Member variables
    FileIndex m_index;
    TimeIndex m_timeIndex;
    C* m_dataBuffer;
    std::fstream m_file;
    Position m_fileEndLocation;
    IndexIterator m_selectedFrame;

};

#endif // STREAMFILEREADER_H
