#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include "ace/common/ring_queue.h"
#include "ace/common/log.h"
namespace ace{
namespace common{

Ring_Queue::Ring_Queue(char *rgb_pool, char *depth_pool, int size)
{
    for(int i = 0; i < size; i++){
        rgb_data[i] = rgb_pool + i*ELMENT_SIZE;
        depth_data[i] = depth_pool + i*ELMENT_SIZE;
    }
    
    max_size = (size > QUEQUE_LENGTH) ? QUEQUE_LENGTH : size;
    idx = 0; front = 0; rear = 0;
    width = 0; height = 0;
    rgb_ptr = 0; depth_ptr = 0;
}

Ring_Queue::Ring_Queue(int wd, int ht, int size)
{
    char *rgb_ptr = (char*)malloc(sizeof(char)*ELMENT_SIZE*QUEQUE_LENGTH);
    if(rgb_ptr == 0){
        LogError("malloc rgb pool fail\n");
    }
    char *depth_ptr = (char*)malloc(sizeof(char)*ELMENT_SIZE*QUEQUE_LENGTH);
    if(depth_ptr == 0){
        LogError("malloc depth pool fail\n");
    }
    for(int i = 0; i < size; i++){
        rgb_data[i] = rgb_ptr + i*ELMENT_SIZE;
        depth_data[i] = depth_ptr + i*ELMENT_SIZE;
    }
    
    max_size = (size > QUEQUE_LENGTH) ? QUEQUE_LENGTH : size;
    idx = 0; front = 0; rear = 0;
    width = wd;
    height = ht;
}

Ring_Queue::Ring_Queue(int size)
{
    char *rgb_ptr = (char*)malloc(sizeof(char)*ELMENT_SIZE*QUEQUE_LENGTH);
    if(rgb_ptr == 0){
        LogError("malloc rgb pool fail\n");
    }
    char *depth_ptr = (char*)malloc(sizeof(char)*ELMENT_SIZE*QUEQUE_LENGTH);
    if(depth_ptr == 0){
        LogError("malloc depth pool fail\n");
    }
    for(int i = 0; i < size; i++){
        rgb_data[i] = rgb_ptr + i*ELMENT_SIZE;
        depth_data[i] = depth_ptr + i*ELMENT_SIZE;
    }
    
    max_size = (size > QUEQUE_LENGTH) ? QUEQUE_LENGTH : size;
    idx = 0; front = 0; rear = 0;
    width = 0; height = 0;
}

Ring_Queue::~Ring_Queue()
{
   if(rgb_ptr){
       free(rgb_ptr);
   }
   
   if(depth_ptr){
       free(depth_ptr);
   }
}

int Ring_Queue::queue_push(char *pRgb, char *pDepth, int length)
{
    memcpy(rgb_data[rear], pRgb, length);
    memcpy(depth_data[rear], pDepth, length);
    //LogTrace("get number %d\n",*(int*)rgb_data[rear]);
    rear = (rear + 1) % max_size;
    return 0;
}

int Ring_Queue::queue_pop(int step)
{
    int current_idx = idx;
    idx = (idx + step) % max_size;
    front = (front+1) % max_size;
    return current_idx;
}

int Ring_Queue::queue_pop_addr(int step, long& rgb_addr, long& depth_addr)
{
    int current_idx = idx;
    idx = (idx + step) % max_size;
    front = (front+1) % max_size;
    rgb_addr = (long)(rgb_data[current_idx]);
    depth_addr = (long)(depth_data[current_idx]);
    return current_idx;
}

int Ring_Queue::queue_length()
{
    return (rear - front + max_size) % max_size;
}

}  // namespace common
}  // namespace ace
