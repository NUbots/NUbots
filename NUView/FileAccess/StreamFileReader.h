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
#include "Tools/FileFormats/TimestampedData.h"
#include <QDebug>
#include <cmath>
#include "IndexedFileReader.h"
#include "Tools/FileFormats/FileFormatException.h"

template<class C>
class StreamFileReader: public IndexedFileReader
{
public:
    /**
      *     Default constructor. Initialises the StreamFileReader.
      */
    StreamFileReader(): IndexedFileReader()
    {
        m_dataBuffer = new C();
    }

    /**
      *     Open constructor. Initialises the StreamFileReader and opens and indexes the file described by the filename.
      *     @param filename name of the file to be opened and accessed by the StreamFileReader.
      */
    StreamFileReader(const std::string& filename): IndexedFileReader()
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
            if(time>=0.0)
                return ReadFrame(GetIndexFromTime(time));
            else
                return NULL;
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
      *     Read the data that is most valid at the given point of time from the file into the data buffer.
      *     @param The time in milliseconds.
      *     @return Pointer to the buffer containing the new object. NULL if the data could not be read.
      */
    C* ReadFrameAtTime(double time)
    {
        IndexIterator entry = GetIndexFromTime(time);
        return ReadFrame(entry);
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
      *     Scan the file and index the location and timestamp of each object within the file.
      *     This allows random access of the stream file.
      */
    void IndexFile()
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
            bool eofReached = false;
            const unsigned int min_length = 12;
            while (m_file.good() && ((m_fileEndLocation - m_file.tellg()) > min_length))
            {
                int pos = m_file.tellg();
                //qDebug("Indexing Frame %d at %d", temp.frameSequenceNumber, pos);
                temp.position = m_file.tellg();
                try{
                    m_file >> (*m_dataBuffer);
                }
                catch(FileFormatException& e){
                    qDebug("Bad frame found: %s", e.getMessage().c_str());
                    eofReached = true;
                }
                catch(...){
                    qDebug("Bad frame found");
                    eofReached = true;
                }
                // File Cursor Has Not Moved
                if(pos == m_file.tellg())
                {
                    qDebug("ERROR Reading Frame (%d). Check File Format.", temp.frameSequenceNumber);
                    CloseFile();
                    return;
                }
                if(eofReached) break;
                timestamp = (static_cast<TimestampedData*>(m_dataBuffer))->GetTimestamp();
                timestamp = floor(timestamp);
                if(HasTime(timestamp))
                {
                    qDebug("File: %s - Found duplicate frame time: %d", m_filename.c_str(),timestamp);
                    continue;
                }
                temp.frameSequenceNumber++;
                m_index.insert(IndexEntry(timestamp,temp));
                m_timeIndex.push_back(timestamp);
            }
            m_file.clear();
            m_file.seekg(origPos,std::ios_base::beg);
        }
    }

    // Member variables
    C* m_dataBuffer;                    //!< Pointer to data buffer used to store the objects read.
};

#endif // STREAMFILEREADER_H
