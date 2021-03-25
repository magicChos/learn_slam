#ifndef ACE_RING_QUEQUE_H
#define ACE_RING_QUEQUE_H

namespace ace{
namespace common{

#define ELMENT_SIZE (1920*1080) //1080P
#define QUEQUE_LENGTH 60        //2 seconds

class Ring_Queue{
public:
    Ring_Queue(char *rgb_pool, char *depth_pool, int size); //allocate memory outside
    Ring_Queue(int wd, int ht, int size); //allocate memory inside,image pool
    Ring_Queue(int size); //allocate memory inside,non-image pool
    ~Ring_Queue();
    int queue_push(char *pRgb, char *pDepth, int length);
    int queue_pop(int step);
    int queue_pop_addr(int step, long& rgb_addr, long& depth_addr);
    int queue_length();
    
    char* rgb_data[QUEQUE_LENGTH];
    char* depth_data[QUEQUE_LENGTH];
    
private:
    
    char* rgb_ptr;
    char* depth_ptr;
    int idx;
    int front;
    int rear;
    int max_size;
    int width;
    int height;
};

}  // namespace common
}  // namespace ace

#endif//ACE_RING_QUEQUE_H