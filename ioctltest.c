#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

// I could not work out how to include them so for now
// they're hard-coded.
#define DM510_IOCRESET       18944
#define DM510_IOCTMAXREADERS 18945
#define DM510_IOCTBUFFERSIZE 18946
#define DM510_IOCQMAXREADERS 18947
#define DM510_IOCQBUFFERSIZE 18948

int main(int argc, char *argv[]) {
	int res = 0;
	int fd = open("/dev/dm510-0", O_RDONLY);

	printf("Resetting device...\n");
	res = ioctl(fd, DM510_IOCRESET, 0);
	printf("Got result: %d\n", res);

	printf("\nChecking defaults...\n");
	res = ioctl(fd, DM510_IOCQMAXREADERS, 0);
	printf("Max readers: %d\n", res);
	res = ioctl(fd, DM510_IOCQBUFFERSIZE, 0);
	printf("Buffer size: %d\n", res);

	printf("\nSetting max readers to 1...\n");
	res = ioctl(fd, DM510_IOCTMAXREADERS, 1);
	printf("Max readers set. Got result: %d\n", res);
	res = ioctl(fd, DM510_IOCQMAXREADERS, 0);
	printf("Max readers: %d\n", res);
	res = ioctl(fd, DM510_IOCQBUFFERSIZE, 0);
	printf("Buffer size: %d\n", res);

	printf("\nSetting buffer size to 1000...\n");
	res = ioctl(fd, DM510_IOCTBUFFERSIZE, 1000);
	printf("Buffer size set. Got result: %d\n", res);
	res = ioctl(fd, DM510_IOCQMAXREADERS, 0);
	printf("Max readers: %d\n", res);
	res = ioctl(fd, DM510_IOCQBUFFERSIZE, 0);
	printf("Buffer size: %d\n", res);

	printf("\nTrying to open the file again (this should fail)...\n");
	int fd2 = open("/dev/dm510-0", O_RDONLY);
	if (fd2 < 0) {
		printf("It failed! Successfully\n");
	} else {
		printf("It opened... Wrongly\n");
	}
	res = ioctl(fd, DM510_IOCQMAXREADERS, 0);
	printf("Max readers: %d\n", res);
	res = ioctl(fd, DM510_IOCQBUFFERSIZE, 0);
	printf("Buffer size: %d\n", res);

	printf("\nTrying an invalid command...\n");
	res = ioctl(fd, 123, 1000);
	printf("Got result: %d\n", res);
	res = ioctl(fd, DM510_IOCQMAXREADERS, 0);
	printf("Max readers: %d\n", res);
	res = ioctl(fd, DM510_IOCQBUFFERSIZE, 0);
	printf("Buffer size: %d\n", res);

	printf("\nResetting device, again...\n");
	res = ioctl(fd, DM510_IOCRESET, 0);
	printf("Got result: %d\n", res);

	printf("\nChecking defaults, again...\n");
	res = ioctl(fd, DM510_IOCQMAXREADERS, 0);
	printf("Max readers: %d\n", res);
	res = ioctl(fd, DM510_IOCQBUFFERSIZE, 0);
	printf("Buffer size: %d\n", res);
}
