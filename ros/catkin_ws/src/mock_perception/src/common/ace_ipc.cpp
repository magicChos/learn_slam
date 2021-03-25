#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include "ace/common/ace_ipc.h"
#include "ace/common/log.h"

namespace ace{
namespace common{

mqd_t mq_rw_create(char *name, int maxsize)
{
	mqd_t mq;
	struct mq_attr attr;

	attr.mq_flags = 0;       /* BLOCK */
	attr.mq_maxmsg =  MSG_QUEUE_MAX_SIZE;     /* msg count */
	attr.mq_msgsize = maxsize;  /* msg queue size in bytes */
	attr.mq_curmsgs = 0; /* current msg count in queue */

	mq = mq_open(name, O_CREAT | O_RDWR, 0644, &attr);
	if (mq == (mqd_t)(-1)) {
		LogError("mq_open failed, %s\n", strerror(errno));
	}
	LogInfo("msg queue mqd:%d\n", mq);
	return mq;
}

mqd_t mq_rd_open(char *name)
{
	mqd_t mq;

	mq = mq_open(name, O_RDONLY);
	if (mq == (mqd_t)(-1)) {
		LogError("mq_open failed, %s\n", strerror(errno));
	}
	return mq;
}

int mq_recv_msg(mqd_t mq, char *buf, int maxsize)
{
	int bytes_read;

again:
	bytes_read = mq_receive(mq, buf, maxsize, NULL);
    if (bytes_read < 0) {
		if (errno == EINTR)
			goto again;
		else {
			LogError("mq_receive failed, %s\n", strerror(errno));
			return -1;
		}
	}
	return bytes_read;
}

int mq_recv_latest_msg(mqd_t mq, char *buf, int maxsize)
{
   int bytes_read;
    mq_attr attr;
    do{
        bytes_read = mq_recv_msg(mq, buf, maxsize);
        mq_mq_getattr(mq, &attr);
    }while(attr.mq_curmsgs);
    return bytes_read;
}

int mq_send_msg(mqd_t mq, char *buf, int length)
{
	int bytes_read;

again:
	bytes_read = mq_send(mq, buf, length, 0);
	if (bytes_read < 0) {
		if (errno == EINTR)
			goto again;
		else {
			LogError("mq_send failed, %s\n", strerror(errno));
			return -1;
		}
	}
	return bytes_read;
}

int mq_mq_getattr(mqd_t mq, mq_attr *attr)
{
    return mq_getattr(mq, attr);
}

int mq_mq_close(mqd_t mq)
{
	return mq_close(mq);
}

int mq_mq_unlink(const char *name)
{
    mq_unlink(name);
}

}  // namespace common
}  // namespace ace