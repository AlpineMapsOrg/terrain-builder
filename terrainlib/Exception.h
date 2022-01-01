#ifndef EXCEPTION_H
#define EXCEPTION_H

#include <stdexcept>

/// This represents a CTB runtime error
class Exception : public std::runtime_error
{
public:
  Exception(const char *message):
      std::runtime_error(message)
  {
    while(false) {}
  }
};


#endif // EXCEPTION_H
