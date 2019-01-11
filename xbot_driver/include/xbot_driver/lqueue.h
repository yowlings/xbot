#ifndef LQUEUE_H
#define LQUEUE_H
#define MAX_L 100
#include "math.h"
template<typename T>

class lqueue
{
public:

    lqueue(int l):size(0)
    {
        if(l<MAX_L)
        {
            L=l;
        }
        else
        {
            L=MAX_L;
        }

    }
    ~lqueue(){}

    T data[MAX_L];
    int L;

    int size;
    void lpush(T node){
        if(size<L)
        {
            *(data+size)=node;
            size++;
        }
        else
        {
            for(int i=0;i<L;i++)
            {
                *(data+i)=*(data+i+1);
            }
            *(data+L-1)=node;

        }


    }
    bool isImpulse(T node){
      if(size == 0){
        return false;
      }
      else{

        return (abs(node - this->mean()) > 0.12*this->mean());
      }

    }
    T medium(){
        T tdata[MAX_L];
        for(int i=0;i<size;i++)
        {
            for(int j=i+1;j<size;j++)
            {
                tdata[i]=(data[j]>data[i])?data[j]:data[i];
            }
        }
        return tdata[(size-1)/2];
    }
    T mean(){
        T total=0;
        for(int i=0;i<size;i++)
        {
            total+=data[i];
        }
        return total/size;
    }



};

#endif
