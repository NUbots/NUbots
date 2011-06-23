#ifndef FILEFORMATEXCEPTION_H
#define FILEFORMATEXCEPTION_H

#include <iostream>
#include <string>

class FileFormatException {
public:
    FileFormatException(const std::string& msg) : msg_(msg) {}
    ~FileFormatException( ) {}

  std::string getMessage( ) const {return(msg_);}
private:
   std::string msg_;
};


#endif // FILEFORMATEXCEPTION_H
