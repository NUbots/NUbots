#ifndef PARSE_H
#define PARSE_H

#include <vector>
#include <stdlib.h>
#include <stdio.h>

// these define the maximum number of characters in a key, and in a value
#define MAX_KEYLENGTH 64
#define MAX_VALLENGTH 64

using namespace std;

struct Pair {
  char* key; // note max lengths above
  char* val;
};

class Parse {
  public:
    Parse();
    void WriteFile(const char* filename, const char* buffer, int buffersize);
    bool ParseFile(const char* filename);

    // probably should never call this externally, but there's no technical reason why you can't ...
    void RegisterPair(char* k, char* v);

    int GetAsInt(const char* k);
    bool GetAsBool(const char* k);
    char* GetAsString(const char* k);
    double GetAsDouble(const char* k);
    bool HasKey(const char* k);
  private:
    // don't mess with this.
    Pair* FindPair(const char* k);
    Pair* FindPairFailFast(const char* k);

    bool IsTerminator(const char c);
    bool IsLF(const char c);
    bool IsSpace(const char c);

    vector<Pair> keyValuePairs;
};

#endif
