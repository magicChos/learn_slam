#ifndef ACE_IPC_H
#define ACE_IPC_H
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <stdlib.h>
#include <stdint.h>

namespace ace{
namespace common{

#define MSG_QUEUE_NAME_SIZE 64
#define MSG_QUEUE_MAX_SIZE 8
#define MSG_CONTENT_SIZE 64

mqd_t mq_rw_create(char *name, int maxsize);
mqd_t mq_rd_open(char *name);
int mq_recv_msg(mqd_t mq, char *buf, int maxsize);
int mq_recv_latest_msg(mqd_t mq, char *buf, int maxsize);
int mq_send_msg(mqd_t mq, char *buf, int length);
int mq_mq_getattr(mqd_t mq, mq_attr *attr);
int mq_mq_close(mqd_t mq);
int mq_mq_unlink(const char *name);

}  // namespace common
}  // namespace ace
#endif//ACE_IPC_H