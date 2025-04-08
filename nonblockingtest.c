#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>

// I could not work out how to include them so for now
// they're hard-coded.
#define DM510_IOCRESET       18944
#define DM510_IOCTMAXREADERS 18945
#define DM510_IOCTBUFFERSIZE 18946
#define DM510_IOCQMAXREADERS 18947
#define DM510_IOCQBUFFERSIZE 18948

int main(int argc, char *argv[]) {
	char msg[] = "Hello this is a rather long message that should fill the buffer!";
	int buffersize = 8;
	int res = 0;
	// Open non-blocking
	int fd = open("/dev/dm510-0", O_WRONLY | O_NONBLOCK);

	printf("Resetting device...\n");
	res = ioctl(fd, DM510_IOCRESET, 0);
	if (res < 0) {
		printf("ERROR. Reset failed with: %d\n", res);
		return -1;
	}

	printf("\nReducing the buffer size to %d...\n", buffersize);
	res = ioctl(fd, DM510_IOCTBUFFERSIZE, buffersize);
	if (res < 0) {
		printf("ERROR. Failed to resize buffer, err: %d\n", res);
		return -2;
	}

	printf("\nWriting to the buffer...\n");
	printf("This should fill it up.\n");
	res = write(fd, msg, strlen(msg));
	if (res < 0) {
		printf("ERROR. Failed to fill the buffer, got: %d\n", res);
		return -3;
	}

	printf("\nWriting to the buffer again...\n");
	printf("This should fail with EAGAIN.\n");
	res = write(fd, msg, strlen(msg));
	if (res != -1 && errno != EAGAIN) {
		printf("ERROR. Writing to the buffer did fail with EAGAIN: %d\n", errno);
		return -4;
	}

	printf("\nSUCCESS. Test completed without error.\n");

	return 0;
}
