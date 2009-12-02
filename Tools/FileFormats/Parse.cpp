#include "Parse.h"

#include <iostream>
#include <string.h>
using namespace std;

#define strcmpi strcmp 

Parse::Parse() {
  // hmm !
}

// Detects both CR and LF characters
bool Parse::IsTerminator(const char c) {
  if (c == char(13) || c == (char)10)
    return true;
  return false;
}

// Only DOS has these.
bool Parse::IsLF(const char c) {
  if (c == (char)13)
    return true;
  return false;
}

// Detects spaces AND tab characters.
bool Parse::IsSpace(const char c) {
  if (c == ' ' || c == '\t')
    return true;
  return false;
}

void Parse::WriteFile(const char* filename, const char* buffer, int buffersize) {
  printf("Config: Writing %s.\n",filename);
  FILE* f = fopen("/ms/open-r/mw/test.t","wb");
  if (f != NULL) { 
    fwrite(buffer,1,buffersize,f);
    fclose(f);
  } else {
    cout << "Parse: unable to open file for writing" << endl << flush;
  }
}

// This code is dirty, but it seems to be indestructible. I haven't found anything
//   that can crash it outright.
// It parses the specified file as a config file, looking for key,value pairs. These
//   are stored in a vector using RegisterPair().
bool Parse::ParseFile(const char* filename) {
  //printf("Config: Loading %s.\n", filename);
  FILE* f = fopen(filename,"rb");
  if (f == NULL) {
    printf("Config: Unable to open file %s.\n",filename);
    return false;
  }
  fseek(f,0,SEEK_END);
  long filesize = ftell(f);
  rewind(f);
  char* filebuffer = (char*) malloc(filesize);
  if (filebuffer == NULL) {
    printf("Config: Unable to malloc %d bytes required to parse %s.\n",(int)filesize,filename);
    return false;
  }

  fread(filebuffer,1,filesize,f);
  fclose(f);

  int i = 0;
  int linenumber=0;
  while (i < filesize) {
    char* keystring = (char*)malloc(MAX_KEYLENGTH);
    char* valstring = (char*)malloc(MAX_VALLENGTH);

    linenumber++;
    int k = 0;
    // ignore opening spaces
    while (i < filesize && IsSpace(filebuffer[i])) i++;
    // read up to =
    while (filebuffer[i] != '=' && IsTerminator(filebuffer[i]) == false && i < filesize && k < MAX_KEYLENGTH) {
      keystring[k] = filebuffer[i];
      k++;
      i++;
    }

    // if keystring is empty or starts with a [, # or //, we ignore this line.
    if (k == 0 || keystring[0] == '[' || keystring[0] == '#' || (k > 1 && keystring[0] == '/' && keystring[1] == '/')) {
      // ignore line !
      while (i < filesize && IsTerminator(filebuffer[i]) == false) i++;
      i++;
      if (IsLF(filebuffer[i-1])) i++;
      free(keystring);
      free(valstring);
      continue;
    }

    // check for non-empty key, with NO equals. if this wasn't caught above, then we have an invalid line !
    if (filebuffer[i] != '=' || k >= MAX_KEYLENGTH) {
      while (i < filesize && IsTerminator(filebuffer[i]) == false) i++;
      i++;
      // did anyone ever understand this?
      if (IsLF(filebuffer[i-1])) i++;
      printf("Config: Error on line %d of file %s - ",linenumber,filename);
      if (k >= MAX_KEYLENGTH) {
        printf("Key too long.\n");
      } else printf("Missing equals.\n");
      free(keystring);
      free(valstring);
      continue;
    }

    // remove trailing spaces
    while (k > 0 && IsSpace(keystring[k-1])) k--;

    keystring[k]='\0';
    
    k=0;
    // ignore =
    i++;
    // ignore whitespace after =
    while (i < filesize && IsSpace(filebuffer[i])) i++;

    // read till end of line.
    while (i < filesize && IsTerminator(filebuffer[i]) == false && k < MAX_VALLENGTH) {
      valstring[k] = filebuffer[i];
      k++;
      i++;
    }
    if (k >= MAX_VALLENGTH) {
      while (i < filesize && IsTerminator(filebuffer[i]) == false) i++;
      i++;
      if (IsLF(filebuffer[i-1])) i++;
      printf("Config: Error on line %d of file %s - ",linenumber,filename);
      printf("Value was too long.\n");
      continue;
    }
    i++;
    // make sure we really did hit the end of the line... need to be in the right place for next read. this is dos's fault !
    if (IsLF(filebuffer[i-1])) i++;

    // remove trailing spaces
    while (k > 0 && IsSpace(valstring[k-1])) k--;
    valstring[k]='\0';

    RegisterPair(keystring,valstring);
  }
  free(filebuffer);
  return true;
}

// store the specified key,value pair in our vector.
// if the key already exists, it is overwritten (with a warning)
void Parse::RegisterPair(char* k,char* v) {
  Pair* p = FindPairFailFast(k);
  // key, value pair already exists
  if (p != NULL) {
    // value changed when we reloaded it ! warn user.
    if (strcmpi(p->val,v)!=0) {
      printf("Config: WARNING! Pair (%s,%s) is now (%s,%s).\n",p->key,p->val, k, v);
    }
    free(p->val);
    p->val = v;
    return;
  }
  Pair newp;
  newp.key = k;
  newp.val = v;
  keyValuePairs.push_back(newp);
  if (strlen(v) != 0) {
    //printf("Config: Registered (%s,%s).\n",newp.key,newp.val);
  } else {
    //printf("Config: Registered (%s,%s) (Value field empty!).\n",newp.key,newp.val);
  }

}

Pair* Parse::FindPair(const char* k) {
  for (unsigned int i = 0; i < keyValuePairs.size(); i++) {
    if (strcmpi(keyValuePairs[i].key,k)==0) {
      return &keyValuePairs[i];
    }
  }
  printf("Config: Unable to find key %s.\n",k);
  return NULL;
}

Pair* Parse::FindPairFailFast(const char* k) {
  for (unsigned int i = 0; i < keyValuePairs.size(); i++) {
    if (strcmpi(keyValuePairs[i].key,k)==0) {
      return &keyValuePairs[i];
    }
  }
  return NULL;
}

bool Parse::HasKey(const char* k){
  for (unsigned int i = 0; i < keyValuePairs.size(); i++) {
    if (strcmpi(keyValuePairs[i].key,k)==0) {
      return true;
    }
  }
  return false;
}

int Parse::GetAsInt(const char* k) {
  Pair* p = FindPair(k);
  if (p != NULL) {
    return atoi(p->val);
  }
  return -1;
}

double Parse::GetAsDouble(const char* k) {
  Pair* p = FindPair(k);
  if (p != NULL) {
    return atof(p->val);
  }
  return -1.0;
}

// Use with care !
char* Parse::GetAsString(const char* k) {
  Pair* p = FindPair(k);
  if (p != NULL) {
    return p->val;
  }
  return NULL;
}

bool Parse::GetAsBool(const char* k) {
  Pair* p = FindPair(k);
  if (p != NULL) {
    // nb strcmpi is not ANSI C !
    if (strcmpi(p->val,"on")==0) {
      return true;
    } else if (strcmpi(p->val,"true") == 0) {
      return true;
    }
  }
  return false;
}

//int main() {
//  Parse p;
//  p.ParseFile("test.cfg");
//  char* test = p.GetAsString("test");
//  p.RegisterPair("test","hello");
//  test = p.GetAsString("test");
//}
