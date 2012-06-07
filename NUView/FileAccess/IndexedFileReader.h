/*! @file IndexedFileReader.h
    @brief Declaration and definition of the IndexedFileReader template class

    @class IndexedFileReader
    @brief Virtual class used to index timestamped data from a file.

    The indexed file reader defines and interface for indexing and accessing
    data within a file. Classes which inherit from this class must define the
    indexing method as well as any data reading methods required.

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

#ifndef INDEXEDFILEREADER_H
#define INDEXEDFILEREADER_H
#include <string>
#include <fstream>
#include <map>
#include <vector>
class IndexedFileReader
{

public:

    // Declare types and structures used in class.
    typedef std::fstream::pos_type Position;
    struct FrameEntry
    {
        unsigned int frameSequenceNumber;
        Position position;
    };
    typedef std::map<double,FrameEntry> FileIndex;
    typedef std::vector<double> TimeIndex;
    typedef FileIndex::iterator IndexIterator;
    typedef FileIndex::value_type IndexEntry;


    IndexedFileReader();
    virtual ~IndexedFileReader();

    // Public File Access Functions
    bool IsValid();
    bool OpenFile(const std::string& filename);
    void CloseFile();

    // Public Index Access Functions.
    double StartTime();
    double EndTime();
    double CurrentFrameTime();
    bool HasTime(double time);
    double FindClosestTime(double time);
    double TimeAtSequenceNumber(unsigned int sequenceNumber);

    unsigned int CurrentFrameSequenceNumber();
    unsigned int TotalFrames();

    // Unimplemented virtual functions.
    virtual void IndexFile() = 0;

protected:
    // Protected helper functions
    bool ValidStartingLocation(Position startingLocation);
    bool ValidEntry(IndexIterator entry);
    IndexIterator GetIndexFromTime(double time);
    void ClearIndex();

    // Protected member variables
    FileIndex m_index;                  //!< Index mapping timestamp to Frame entries.
    TimeIndex m_timeIndex;              //!< Index mapping sequence number to timestamp.
    std::fstream m_file;                //!< The file.
    Position m_fileEndLocation;         //!< The end position of the file.
    IndexIterator m_selectedFrame;      //!< Reference to the currently selected frame in the FileIndex.
    std::string m_filename;             //!< The name of the open file.
};

#endif // FILEREADER_H
