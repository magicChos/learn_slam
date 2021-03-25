#include <stdio.h>   
#include <stdlib.h>
#include <iostream>
#include <utility>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include "gtest/gtest.h"
#include "ace/common/ring_queue.h"
#include "ace/common/ace_ipc.h"
#include "ace/common/log.h"

namespace ace{
namespace common{
namespace{

char image_msg[] = "/image_msg";

void image_pool_thread(void)
{
    /*
    char *rgb_pool = (char*)malloc(sizeof(char)*ELMENT_SIZE*QUEQUE_LENGTH);
    if(rgb_pool == 0){
        std::cout << "malloc rgb pool fail" << std::endl;
    }
    char *depth_pool = (char*)malloc(sizeof(char)*ELMENT_SIZE*QUEQUE_LENGTH);
    if(rgb_pool == 0){
        std::cout << "malloc depth pool fail" <<std::endl;
    }
    int n = 0;
    static Ring_Queue images_pool(rgb_pool, depth_pool, QUEQUE_LENGTH);
    */
    int n = 0;
    static Ring_Queue images_pool(60);
    mqd_t mqdes = mq_rw_create(image_msg, 64);
    LogInfo("create msg queue %d\n", mqdes);
    
    char buf[8];
    
    while(1)
    {
        n++;
        if(n == 100){
            break;
        }
        images_pool.queue_push((char*)&n, (char*)&n, sizeof(int));
        *(long*)buf = (long)(&images_pool);
        mq_send_msg(mqdes, buf, sizeof(long));
        std::cout << "camera_pool_thread "<< std::this_thread::get_id() << " push " << n \
        << " ,and send msq " << &images_pool << std::endl;
        
        std::this_thread::sleep_for(std::chrono::microseconds(40));
    }
    
    //free(rgb_pool);
    //free(depth_pool);
}

void image_process_thread(void)
{
    mqd_t mqdes = mq_rd_open(image_msg);
    LogInfo("open msg queue %d\n", mqdes);
    int len = 0;
    char buf[64];
    Ring_Queue* handle = 0;
    long rgb_addr, depth_addr;
    int n;
    while(1)
    {
        len = mq_recv_msg(mqdes, buf, 64);
        handle = (Ring_Queue*)(*(long*)buf);
        handle->queue_pop_addr(1, rgb_addr, depth_addr);
        n = *(int*)rgb_addr;
        printf("recv %d bytes msgs 0x%lx 0x%lx %d\n", len, (*(long*)buf), rgb_addr, n);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        if(n == 99){
            break;
        }
    }
}

TEST(image_pool_thread, image_process_thread)
{
    std::thread img_pool_t(&image_pool_thread);
    std::thread img_process_t(&image_process_thread);
    img_pool_t.join();
    img_process_t.join();
    LogInfo("All threads joined.\n");
    char  c = getchar();
}

}
}  // namespace common
}  // namespace ace
