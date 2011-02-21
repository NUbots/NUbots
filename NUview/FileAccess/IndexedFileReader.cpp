#include "IndexedFileReader.h"
#include <cmath>
IndexedFileReader::IndexedFileReader(): m_fileEndLocation(0)
{
    m_selectedFrame = m_index.end();
}

IndexedFileReader::~IndexedFileReader()
{
    CloseFile();
}

/**
  *     Determine if the current log reader is valid. A log reader is valid if it has correctly opened
  *     a file and created an index of the objects within the file.
  *     @return True if the the log reader is valid. False if it is not.
  */
bool IndexedFileReader::IsValid()
{
    return ((m_file.good()) && (m_index.size() > 0));
}

/**
  *     Open constructor. Initialises the StreamFileReader and opens and indexes the file described by the filename.
  *     Indexing of the file is then performed allowing fast random access to occur.
  *     @param filename The file path and name used to open the file.
  *     @return True when the file is opened and indexed correctly. False when the file is unable to be correctly opened and accessed.
  */
bool IndexedFileReader::OpenFile(const std::string& filename)
{
    CloseFile();
    m_file.open(filename.c_str(),std::ios_base::in | std::ios_base::binary);
    if(m_file.good())
    {
        m_file.seekg(0,std::ios_base::end);
        m_fileEndLocation = m_file.tellg();
        IndexFile();
    }
    return IsValid();
}

/**
  *     Closes any currently opened files.
  */
void IndexedFileReader::CloseFile()
{
    m_file.close();
    m_fileEndLocation = 0;
    ClearIndex();
}

/**
  *     Gets the timestamp of the currently buffered data. This function should only be called after data has been requested.
  *     @return The timestamp in milliseconds.
  */
double IndexedFileReader::CurrentFrameTime()
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
unsigned int IndexedFileReader::CurrentFrameSequenceNumber()
{
    if(m_selectedFrame != m_index.end())
    {
        return (*m_selectedFrame).second.frameSequenceNumber;
    }
    return 0.0;
}


/**
  *     Return the total number of objects found within the current file.
  *     @return The total number of objects in the file. This is the maximium sequence number that can be accessed.
  */
unsigned int IndexedFileReader::TotalFrames()
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
double IndexedFileReader::FindClosestTime(double time)
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
double IndexedFileReader::StartTime()
{
    return TimeAtSequenceNumber(1);
}

/**
  *     Gives the time of the last entry in the file.
  *     @return The time at the end of the file.
  */
double IndexedFileReader::EndTime()
{
    return TimeAtSequenceNumber(TotalFrames());
}

/**
  *     Gives the time of the data given by the requested sequence number (N).
  *     @return The time in milliseconds of the Nth data entry.
  */
double IndexedFileReader::TimeAtSequenceNumber(unsigned int sequenceNumber)
{
    if((sequenceNumber <= m_index.size()) && (sequenceNumber > 0))
    {
        return m_timeIndex[sequenceNumber-1];
    }
    else
    {
        return -1.0;
    }
}

/**
  *     Determine if the file contains a value at the exact time given.
  *     @param The time in milliseconds.
  *     @return True if a value is present with the given timestamp. False if not.
  */
bool IndexedFileReader::HasTime(double time)
{
    IndexIterator pos = m_index.find(floor(time));
    return (pos != m_index.end());
}


/**
  *     Determine if a starting location for an object within the file is valid.
  *     @param startingLocation Position within the file at which the start of an object is proposed.
  *     @return True if this value is within the file. False if it is not.
  */
bool IndexedFileReader::ValidStartingLocation(Position startingLocation)
{
    return ( (startingLocation < m_fileEndLocation) && (startingLocation >= 0) );
}

/**
  *     Determine if an iterator is pointing to a valid entry.
  *     @param entry Iterator pointing to the desired entry.
  *     @return True if this Iterator is valid. False if it is not.
  */
bool IndexedFileReader::ValidEntry(IndexedFileReader::IndexIterator entry)
{
    return (entry != m_index.end());
}

/**
  *     Retrieve the index of the entry describing the data given by the time.
  *     @param time in milliseconds.
  *     @return Iterator pointing to the desired entry.
  */
IndexedFileReader::IndexIterator IndexedFileReader::GetIndexFromTime(double time)
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

void IndexedFileReader::ClearIndex()
{
    m_index.clear();
    m_timeIndex.clear();
}
