#ifndef STREAMFILEREADER_H
#define STREAMFILEREADER_H
#include <string>
#include <fstream>
#include <map>

template<class C>
class StreamFileReader
{
    typedef std::map<double,unsigned int> FileIndex;
    typedef FileIndex::value_type IndexEntry;
public:
    StreamFileReader();
    StreamFileReader(const std::string& filename);
    ~StreamFileReader();
    void OpenFile(const std::string& filename);
    void CloseFile();

private:
    void GenerateIndex();
    C* m_dataBuffer;
    std::fstream m_file;
    FileIndex m_index;
};

#endif // STREAMFILEREADER_H
