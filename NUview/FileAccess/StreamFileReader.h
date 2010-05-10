/*! @file StreamFileReader.h
    @brief Declaration and definition of the StreamFileReader template class

    @class StreamFileReader
    @brief Class used to read timestamped data from a stream file.

    The stream file reader is used to access time-stamped data stored within a stream file.
    Because of the nature of a stream the file is parsed and each timestamped data objects
    timestamp and location within the file is indexed allowing fast, random access of the
    stream file.
    When reading in the data it is locally buffered and a pointer to the object type read
    is returned.
    Because it is a templated class, any timestamped data can be read from a stream file
    containing it. However any class used in the template must implement the abstract class
    TimestampedData found in the /Tools/FileFormats/ directory so as to access timestamps.

    @author Steven Nicklin

  Copyright (c) 2010 Steven Nicklin

    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STREAMFILEREADER_H
#define STREAMFILEREADER_H
#include <string>
#include <fstream>
#include <map>
#include <vector>
#include "Tools/FileFormats/TimestampedData.h"
#include "NavigableFileReader.h"
#include <cmath>
#include <QDebug>

template<class C>
class StreamFileReader: public NavigableFileReader
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
    /**
      *     Default constructor. Initialises the StreamFileReader.
      */
    StreamFileReader(QObject *parent = 0): NavigableFileReader(parent), m_fileEndLocation(0)
    {
        m_dataBuffer = new C();
        m_selectedFrame = m_index.end();
    }

    /**
      *     Open constructor. Initialises the StreamFileReader and opens and indexes the file described by the filename.
      *     @param filename name of the file to be opened and accessed by the StreamFileReader.
      */
    StreamFileReader(const std::string& filename, QObject *parent = 0): NavigableFileReader(parent), m_fileEndLocation(0)
    {
        m_dataBuffer = new C();
        OpenFile(filename);
        m_selectedFrame = m_index.end();
    }
    /**
      *     Destructor. Removes dynamically allocated memory and closes any open files.
      */
    ~StreamFileReader()
    {
        delete m_dataBuffer;
        CloseFile();
    }

    /**
      *     Open constructor. Initialises the StreamFileReader and opens and indexes the file described by the filename.
      *     Indexing of the file is then performed allowing fast random access to occur.
      *     @param filename The file path and name used to open the file.
      *     @return True when the file is opened and indexed correctly. False when the file is unable to be correctly opened and accessed.
      */
    bool OpenFile(const std::string& filename)
    {
        m_file.open(filename.c_str(),std::ios_base::in | std::ios_base::binary);
        if(m_file.good())
        {
            m_file.seekg(0,std::ios_base::end);
            m_fileEndLocation = m_file.tellg();
            GenerateIndex();
        }
        return IsValid();
    }

    /**
      *     Determine if the current log reader is valid. A log reader is valid if it has correctly opened
      *     a file and created an index of the objects within the file.
      *     @return True if the the log reader is valid. False if it is not.
      */
    bool IsValid()
    {
        return ((m_file.good()) && (m_index.size() > 0));
    }

    /**
      *     Closes any currently opened files.
      */
    void CloseFile()
    {
        m_file.close();
    }

    /**
      *     Prints the current index.
      */
    void DisplayIndex()
    {
        IndexIterator indexEntry = m_index.begin();
        while(indexEntry != m_index.end())
        {
            qDebug() << "Sequence Number: "<< (*indexEntry).second.frameSequenceNumber << "\tTime: " << (*indexEntry).first << "\tIndex: " << (*indexEntry).second.position;
            ++indexEntry;
        }
    }

    /**
      *     Read in the element in the stream with the sequence number give. The sequence number is given by the elements position in the file.
      *     The first element is at sequence number 1, the next at sequence number 2 and so forth up to the total number of elements in the file.
      *     @param frameNumber The sequence number of the desired data.
      *     @return A pointer to the object read from the file. NULL is returned if an error occurs.
      */
    C* ReadFrameNumber(int frameSequenceNumber)
    {

            float time = TimeAtSequenceNumber(frameSequenceNumber);
            if(time)
                return ReadFrame(GetIndexFromTime(time));
            else
                return NULL;
    }

    /**
      *     Gets the timestamp of the currently buffered data. This function should only be called after data has been requested.
      *     @return The timestamp in milliseconds.
      */
    double CurrentFrameTime()
    {
        if(m_selectedFrame != m_index.end())
        {
            return (*m_selectedFrame).first;
        }
        return 0.0;
    }

    /**
      *     Gets the sequence number of the currently buffered data. This function should only be called after data has been requested.
      *     @return The sequence number.
      */
    unsigned int CurrentFrameSequenceNumber()
    {
        if(m_selectedFrame != m_index.end())
        {
            return (*m_selectedFrame).second.frameSequenceNumber;
        }
        return 0.0;
    }

    /**
      *     Reads in the first object within the file to a buffer.
      *     @return Pointer to the buffer containing the new object. NULL if the data could not be read.
      */
    C* ReadFirstFrame()
    {
        IndexIterator entry = m_index.begin();
        return ReadFrame(entry);
    }

    /**
      *     Reads in the next object relative to the currently buffered data from the file into the buffer.
      *     @return Pointer to the buffer containing the new object. NULL if the data could not be read.
      */
    C* ReadNextFrame()
    {
        IndexIterator entry = m_selectedFrame;
        ++entry;
        return ReadFrame(entry);
    }

    /**
      *     Reads in the previous object relative to the currently buffered data from the file into the buffer.
      *     @return Pointer to the buffer containing the new object. NULL if the data could not be read.
      */
    C* ReadPrevFrame()
    {
        IndexIterator entry = m_selectedFrame;
        if(entry != m_index.begin())
        {
            --entry;
            return ReadFrame(entry);
        }
        return NULL;
    }

    /**
      *     Reads the last object in the file into the buffer.
      *     @return Pointer to the buffer containing the new object. NULL if the data could not be read.
      */
    C* ReadLastFrame()
    {
        IndexIterator entry = m_index.end();
        if(entry != m_index.begin())
        {
            --entry;
            return ReadFrame(entry);
        }
        return NULL;
    }

    /**
      *     Return the total number of objects found within the current file.
      *     @return The total number of objects in the file. This is the maximium sequence number that can be accessed.
      */
    unsigned int TotalFrames()
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

    /**
      *     Finds the closest indexed time for the data to be valid at the given point of time. Meaning that if the exact
      *     time is not available, the time of the most recent data before this point of time is given.
      *     @param time The time in milliseconds.
      *     @return The time of the data that best fits the given time.
      */
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

    /**
      *     Gives the time of the first entry in the file.
      *     @return The time at the start of the file.
      */
    double StartTime()
    {
        return TimeAtSequenceNumber(1);
    }

    /**
      *     Gives the time of the last entry in the file.
      *     @return The time at the end of the file.
      */
    double EndTime()
    {
        return TimeAtSequenceNumber(TotalFrames());
    }

    /**
      *     Gives the time of the data given by the requested sequence number (N).
      *     @return The time in milliseconds of the Nth data entry.
      */
    double TimeAtSequenceNumber(unsigned int sequenceNumber)
    {
        if((sequenceNumber <= m_index.size()) && (sequenceNumber > 0))
        {
            return m_timeIndex[sequenceNumber-1];
        }
        else
        {
            return 0.0;
        }
    }

    /**
      *     Determine if the file contains a value at the exact time given.
      *     @param The time in milliseconds.
      *     @return True if a value is present with the given timestamp. False if not.
      */
    bool HasTime(double time)
    {
        IndexIterator pos = m_index.find(floor(time));
        return (pos != m_index.end());
    }

    /**
      *     Read the data that is most valid at the given point of time from the file into the data buffer.
      *     @param The time in milliseconds.
      *     @return Pointer to the buffer containing the new object. NULL if the data could not be read.
      */
    C* ReadFrameAtTime(double time)
    {
        IndexIterator entry = GetIndexFromTime(time);
        qDebug() << "Getting Frame at: " << (*entry).second.position;
        return ReadFrame(entry);
    }

    void EmitControlAvailability()
    {
        if(IsValid() == false)
        {
            emit nextFrameAvailable(false);
            emit previousFrameAvailable(false);
            emit firstFrameAvailable(false);
            emit lastFrameAvailable(false);
            emit setFrameAvailable(false);
        }
        else
        {
            emit nextFrameAvailable(CurrentFrameSequenceNumber() < TotalFrames());
            emit previousFrameAvailable(CurrentFrameSequenceNumber() > 0);
            emit firstFrameAvailable(true);
            emit lastFrameAvailable(true);
            emit setFrameAvailable(true);
        }
    }

private:
    /**
      *     Read the data described by the given entry into the data buffer.
      *     @param entry Iterator pointing to the desired entry.
      *     @return Pointer to the buffer containing the new object. NULL if the data could not be read.
      */
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

    /**
      *     Determine if a starting location for an object within the file is valid.
      *     @param startingLocation Position within the file at which the start of an object is proposed.
      *     @return True if this value is within the file. False if it is not.
      */
    bool ValidStartingLocation(Position startingLocation)
    {
        return ( (startingLocation < m_fileEndLocation) && (startingLocation >= 0) );
    }

    /**
      *     Determine if an iterator is pointing to a valid entry.
      *     @param entry Iterator pointing to the desired entry.
      *     @return True if this Iterator is valid. False if it is not.
      */
    bool ValidEntry(IndexIterator entry)
    {
        return (entry != m_index.end());
    }

    /**
      *     Retrieve the index of the describing the data given by the time.
      *     @param time in milliseconds.
      *     @return Iterator pointing to the desired entry.
      */
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

    /**
      *     Scan the file and index the location and timestamp of each object within the file.
      *     This allows random access of the stream file.
      */
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
    FileIndex m_index;                  //!< Index mapping timestamp to Frame entries.
    TimeIndex m_timeIndex;              //!< Index mapping sequence number to timestamp.
    C* m_dataBuffer;                    //!< Pointer to data buffer used to store the objects read.
    std::fstream m_file;                //!< The file.
    Position m_fileEndLocation;         //!< The end position of the file.
    IndexIterator m_selectedFrame;      //!< Reference to the currently selected frame in the FileIndex.

};

#endif // STREAMFILEREADER_H
