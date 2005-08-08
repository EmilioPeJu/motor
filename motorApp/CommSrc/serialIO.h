#ifdef __cplusplus
extern "C"
{
class serialIO;
void *serialIOInit(int card, char *task);
int serialIOSend(serialIO *serialIO, char const *buffer,
		 int buffer_len, int timeout);
int serialIORecv(serialIO *serialIO, char *buffer, int buffer_len,
                 int terminator, int timeout);
int serialIOSendRecv(serialIO *serialIO, const char *outbuff, int outbuff_len,
                     char *inbuff, int inbuff_len, 
                     int terminator, int timeout);
}
#else  /* For C just define serialInfo as a dummy structure since it can't
          understand the include files which define what it really is */

void *serialIOInit(int card, char *task);
int serialIOSend(void *serialIO, char const *buffer,
		 int buffer_len, int timeout);
int serialIORecv(void *serialIO, char *buffer, int buffer_len,
                 int terminator, int timeout);
int serialIOSendRecv(void *serialIO, const char *outbuff, int outbuff_len,
                     char *inbuff, int inbuff_len, 
                     int terminator, int timeout);
#endif
