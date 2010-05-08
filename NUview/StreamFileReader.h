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
    typedef std::fstream::pos_type Position;
    struct FrameEntry
    {
        unsigned int frameSequenceNumber;
        Position position;
    };
    typedef std::map<double,FrameEntry> FileIndex;
    typedef std::vector<double> TimeIndex;
    typedef typename FileIndex::iterator IndexIterator;
    typedef typename FileIndex::value_type IndexEntry;
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
        IndexIterator indexEntry = m_index.begin();
        while(indexEntry != m_index.end())
        {
            qDebug() << "Time: " << (*indexEntry).first << "\tIndex: " << (*indexEntry).second.position;
            ++indexEntry;
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

    double CurrentFrameTime()
    {
        if(m_selectedFrame != m_index.end())
        {
            return (*m_selectedFrame).first;
        }
        return 0.0;
    }

    double CurrentFrameSequenceNumber()
    {
        if(m_selectedFrame != m_index.end())
        {
            return (*m_selectedFrame).second.frameSequenceNumber;
        }
        return 0.0;
    }

    C* ReadFirstFrame()
    {
        IndexIterator entry = m_index.begin();
        return ReadFrame(entry);
    }

    C* ReadNextFrame()
    {
        IndexIterator entry = m_selectedFrame;
        ++entry;
        return ReadFrame(entry);
    }

    C* ReadPrevFrame()
    {
        IndexIterator entry = m_selectedFrame;
        --entry;
        return ReadFrame(entry);
    }

    C* ReadLastFrame()
    {
        IndexIterator entry = m_index.end();
        --entry;
        return ReadFrame(entry);
    }

    int TotalFrames()
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

    double StartTime()
    {
        return TimeAtPosition(1);
    }

    double EndTime()
    {
        return TimeAtPosition(TotalFrames());
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

    bool HasTime(double time)
    {
        IndexIterator pos = m_index.find(floor(time));
        return (pos != m_index.end());
    }

    C* ReadFrameAtTime(double time)
    {
        IndexIterator entry = GetIndexFromTime(time);
        qDebug() << "Getting Frame at: " << (*entry).second.position;
        return ReadFrame(entry);
    }

private:
    C* ReadFrame(IndexIterator entry)
    {
        if(ValidEntry(entry))
        {
            Position startingLocation = (*entry).second.position;
            if(m_file.is_open() && ValidStartingLocation(startingLocation))
            {
                m_file.seekg(startingLocation,std::ios_base::beg);
                try{
                    m_file >> (*m_dataBuffer);
                    m_selectedFrame = entry;
                    return m_dataBuffer;
                }   catch(...){}

            }
        }
        return NULL;
    }

    bool ValidStartingLocation(Position startingLocation)
    {
        return (startingLocation < m_fileEndLocation);
    }

    bool ValidEntry(IndexIterator entry)
    {
        return ( (entry != m_index.end()) && (entry != m_index.rend()) );
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
            FrameEntry temp;
            double timestamp = 0.0;
            Position origPos = m_file.tellg();
            m_file.seekg(0,std::ios_base::beg);
            m_index.clear();
            m_timeIndex.clear();
            temp.frameSequenceNumber = 0;
            while (m_file.good())
            {
                temp.position = m_file.tellg();
                temp.frameSequenceNumber++;
                try{
                    m_file >> (*m_dataBuffer);
                }   catch(...){break;}
                timestamp = (static_cast<TimestampedData*>(m_dataBuffer))->GetTimestamp();
                timestamp = floor(timestamp);
                m_index.insert(IndexEntry(timestamp,temp));
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
