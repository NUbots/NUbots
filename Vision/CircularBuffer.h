/*!
  @file CircularBuffer.h
  @brief Declaration of CircularBuffer class.
  @author <a href="mailto:joshua.wilson@newcastle.edu.au">Josh Wilson</a>
*/

#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

template <class V> class CircularBuffer
{
    public:



        CircularBuffer<V>():buff_size(INIT_BUFFER_SIZE)
        {
            buff = V[buff_size];
        }

        CircularBuffer<V>(int bufferSize):buff_size(bufferSize)
        {
            buff = V[buff_size];
        }


    private:
        const int INIT_BUFFER_SIZE = 4;
        int buff_size;
        V buff[];
        int head;
        int tail;
};

#endif
