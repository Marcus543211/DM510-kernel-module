/* Prototype module for second mandatory DM510 assignment */
#ifndef __KERNEL__
#  define __KERNEL__
#endif
#ifndef MODULE
#  define MODULE
#endif

#include <linux/module.h>
#include <linux/init.h> 
#include <linux/cdev.h>
#include <linux/slab.h>	
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/wait.h>
/* #include <asm/uaccess.h> */
#include <linux/uaccess.h>
#include <linux/semaphore.h>
/* #include <asm/system.h> */
#include <asm/switch_to.h>

/* Prototypes - this would normally go in a .h file */
static int dm510_open(struct inode*, struct file*);
static int dm510_release(struct inode*, struct file*);
static ssize_t dm510_read(struct file*, char*, size_t, loff_t*);
static ssize_t dm510_write(struct file*, const char*, size_t, loff_t*);
long dm510_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

#define DEVICE_NAME "dm510_dev" /* Dev name as it appears in /proc/devices */
#define MAJOR_NUMBER 255
#define MIN_MINOR_NUMBER 0
#define MAX_MINOR_NUMBER 1

#define DEVICE_COUNT 2

/* Ioctl definitions */
#define DM510_IOC_MAGIC 'J'

#define DM510_IOCRESET       _IO(DM510_IOC_MAGIC, 0)
#define DM510_IOCTMAXREADERS _IO(DM510_IOC_MAGIC, 1)
#define DM510_IOCTBUFFERSIZE _IO(DM510_IOC_MAGIC, 2)
#define DM510_IOCQMAXREADERS _IO(DM510_IOC_MAGIC, 3)
#define DM510_IOCQBUFFERSIZE _IO(DM510_IOC_MAGIC, 4)

#define DM510_IOC_MAXNR 4
/* end of what really should have been in a .h file */

#define BUFFER_SIZE 4000
#define MAX_READERS 10000

struct buffer {
	wait_queue_head_t readq /*outq*/, writeq /*inq*/;
	char *start, *end;
	int buffersize;
	char *rp, *wp;
	struct mutex mutex;
};

struct dm510 {
	struct buffer *readbuf, writebuf;
	int maxreaders, nreaders, nwriters;
	struct mutex mutex;
	struct cdev cdev;
};

struct dm510 dm510_devices[DEVICE_COUNT];

/* file operations struct */
static struct file_operations dm510_fops = {
	.owner   = THIS_MODULE,
	.read    = dm510_read,
	.write   = dm510_write,
	.open    = dm510_open,
	.release = dm510_release,
        .unlocked_ioctl   = dm510_ioctl
};

/* allocates a buffer, assumes mutex is held or similar */
int alloc_buffer(struct buffer *buf, size_t size) {
	buf->start = kmalloc(size, GFP_KERNEL);
	if (!buf->start) {
		return -ENOMEM;
	}
	buf->rp = buf->wp = buf->start;
	buf->end = buf->start + size;
	buf->buffersize = size;
	return 0;
}

/* frees buffer */
void free_buffer(struct buffer *buf) {
	kfree(buf->start);
}

/* allocates and sets up a buffer */
int setup_buffer(struct buffer *buf) {
	init_waitqueue_head(&buf->readq);
	init_waitqueue_head(&buf->writeq);
	mutex_init(&buf->mutex);
	return alloc_buffer(buf, BUFFER_SIZE);
}

/* setup of a struct dm510, allocates writebuf, sets readbuf to NULL */
int setup_device(struct dm510 *dev, int index) {
	int err, devno = MKDEV(MAJOR_NUMBER, index);

	err = setup_buffer(&dev->writebuf);
	if (err) {
		return err;
	}
	dev->readbuf = NULL;
	dev->maxreaders = MAX_READERS;
	dev->nreaders = dev->nwriters = 0;

	mutex_init(&dev->mutex);

	cdev_init(&dev->cdev, &dm510_fops);
	dev->cdev.owner = THIS_MODULE;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err) {
		printk(KERN_NOTICE "Error %d adding dm510-%d", err, index);
	}
	return 0;
}

/* free resources from a struct dm510 */
void cleanup_device(struct dm510 *dev) {
	cdev_del(&dev->cdev);
	free_buffer(&dev->writebuf);
}

/* Called when module is unloaded */
void dm510_cleanup_module(void) {
	/* clean up code belongs here */
	dev_t devno = MKDEV(MAJOR_NUMBER, MIN_MINOR_NUMBER);
	unregister_chrdev_region(devno, DEVICE_COUNT);

	cleanup_device(&dm510_devices[0]);
	cleanup_device(&dm510_devices[1]);

	printk(KERN_INFO "DM510: Module unloaded.\n");
}

/* called when module is loaded */
int dm510_init_module(void) {
	/* initialization code belongs here */
	printk(KERN_INFO "DM510: Hello from your device!\n");

	dev_t dev = MKDEV(MAJOR_NUMBER, MIN_MINOR_NUMBER);
	int result = register_chrdev_region(dev, DEVICE_COUNT, DEVICE_NAME);
	if (result < 0) {
		printk(KERN_WARNING "DM510: can't get major %d\n", MAJOR_NUMBER);
		return result;
	}

	int result0 = setup_device(&dm510_devices[0], 0);
	int result1 = setup_device(&dm510_devices[1], 1);
	if (result0 || result1) {
		dm510_cleanup_module();
		return result0 || result1;
	}
	dm510_devices[0].readbuf = &dm510_devices[1].writebuf;
	dm510_devices[1].readbuf = &dm510_devices[0].writebuf;

	return 0;
}

/* Called when a process tries to open the device file */
static int dm510_open(struct inode *inode, struct file *filp) {
	/* device claiming code belongs here */
	struct dm510 *dev;

	dev = container_of(inode->i_cdev, struct dm510, cdev);
	filp->private_data = dev;

	if (mutex_lock_interruptible(&dev->mutex)) {
		return -ERESTARTSYS;
	}

	if (filp->f_mode & FMODE_READ) {
		if (dev->nreaders >= dev->maxreaders) {
			mutex_unlock(&dev->mutex);
			return -EBUSY; /* already at max readers */
		}
		dev->nreaders++;
	}
	if (filp->f_mode & FMODE_WRITE) {
		if (dev->nwriters > 0) {
			mutex_unlock(&dev->mutex);
			return -EBUSY; /* only one writer at a time */
		}
		dev->nwriters++;

		/* clear the buffer */
		if (mutex_lock_interruptible(&dev->writebuf.mutex))
			return -ERESTARTSYS;
		dev->writebuf.rp = dev->writebuf.wp = dev->writebuf.start;
		mutex_unlock(&dev->writebuf.mutex);
	}

	mutex_unlock(&dev->mutex);

	return nonseekable_open(inode, filp);
}


/* Called when a process closes the device file. */
static int dm510_release(struct inode *inode, struct file *filp) {
	/* device release code belongs here */
	struct dm510 *dev = filp->private_data;

	mutex_lock(&dev->mutex);
	if (filp->f_mode & FMODE_READ) {
		dev->nreaders--;
	}
	if (filp->f_mode & FMODE_WRITE) {
		dev->nwriters--;
	}
	mutex_unlock(&dev->mutex);
	return 0;
}


/* Called when a process, which already opened the dev file, attempts to read from it. */
static ssize_t dm510_read(
    struct file *filp,
    char *buf,      /* The buffer to fill with data     */
    size_t count,   /* The max number of bytes to read  */
    loff_t *f_pos)  /* The offset in the file           */
{
	
	struct dm510 *dev = filp->private_data;
	struct buffer *rbuf = dev->readbuf;
		
	if (mutex_lock_interruptible(&rbuf->mutex)) {
		return -ERESTARTSYS;
	}

	while (rbuf->rp == rbuf->wp) {
		mutex_unlock(&dev->mutex);
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		printk("\"%s\" reading: going to sleep\n", current->comm);
		if (wait_event_interruptible(rbuf->writeq, (rbuf->rp != rbuf->wp))) {
			return -ERESTARTSYS;
		}
	}

	if (rbuf->wp > rbuf->rp) {
		count = min (count, (size_t)(rbuf->wp - rbuf->rp));
	}
	else {
		count = min(count, (size_t)(rbuf->end - rbuf->rp));
	}
	if (copy_to_user(buf, rbuf->rp, count)) {
		mutex_unlock (&dev->mutex);
		return -EFAULT;
	}
	rbuf->rp += count;
	if (rbuf->rp == rbuf->end) {
		rbuf->rp = rbuf->start;
	}
	mutex_unlock (&rbuf->mutex);

	wake_up_interruptible(&rbuf->readq);
	printk("\"%s\" did read %li bytes\n",current->comm, (long)count);
	return count; //return number of bytes read
}

int spacefree(struct buffer *dev) {
	if (dev->rp == dev->wp) {
		return dev->buffersize - 1;
	}
	return ((dev->rp + dev->buffersize - dev->wp) % dev->buffersize) -1;
}

int getwritespace(struct buffer *dev, struct file *filp) {
	while (spacefree(dev) == 0) {
		DEFINE_WAIT(wait);

		mutex_unlock(&dev->mutex);
		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}
		printk("\"%s\" writing; going to sleep\n", current->comm);
		prepare_to_wait(&dev->readq, &wait, TASK_INTERRUPTIBLE);
		if (spacefree(dev) == 0)
			schedule();
		finish_wait(&dev->readq, &wait);
		if (signal_pending(current))
			return -ERESTARTSYS;
		if (mutex_lock_interruptible(&dev->mutex))
			return -ERESTARTSYS;
	}
	return 0;
}

/* Called when a process writes to dev file */
static ssize_t dm510_write(
    struct file *filp,
    const char *buf,/* The buffer to get data from      */
    size_t count,   /* The max number of bytes to write */
    loff_t *f_pos)  /* The offset in the file           */
{
	struct dm510 *dev = filp->private_data;
	struct buffer *wbuf = &dev->writebuf;
	int result;

	if (mutex_lock_interruptible(&wbuf->mutex)) {
		return -ERESTARTSYS;
	}
	
	result = getwritespace(wbuf, filp);
	if (result) {
		return result;
	}
	
	count = min(count, (size_t)spacefree(wbuf));
	if (wbuf->wp >= wbuf->rp) {
		count = min(count, (size_t)(wbuf->end - wbuf->wp));
	}
	else {
		count = min(count, (size_t)(wbuf->end - wbuf->wp));
	}
	printk("Going to accept %li bytes to %p from %p\n", (long)count, wbuf->wp, buf);
	if (copy_from_user(wbuf->wp, buf, count)) {
		mutex_unlock(&wbuf->mutex);
		return -EFAULT;
	}
	wbuf->wp += count;
	if (wbuf->wp == wbuf->end) {
		wbuf->wp = wbuf->start;
	}
	mutex_unlock(&wbuf->mutex);

	wake_up_interruptible(&wbuf->writeq);

	printk("\"%s\" did write %li bytes\n", current->comm, (long)count);
	return count; //return number of bytes written
}

/* called by system call ioctl */ 
long dm510_ioctl( 
    struct file *filp, 
    unsigned int cmd,   /* command passed from the user */
    unsigned long arg) /* argument of the command */
{
	/* ioctl code belongs here */
	printk(KERN_INFO "DM510: ioctl called.\n");

	struct dm510 *dev = filp->private_data;
	int retval = 0;

	/* ensure command is valid */
	if (_IOC_TYPE(cmd) != DM510_IOC_MAGIC) { return -ENOTTY; }
	if (_IOC_NR(cmd) > DM510_IOC_MAXNR) { return -ENOTTY; }

	if (mutex_lock_interruptible(&dev->mutex)) {
		return -ERESTARTSYS;
	}
	if (mutex_lock_interruptible(&dev->writebuf.mutex)) {
		return -ERESTARTSYS;
	}

	switch (cmd) {
		case DM510_IOCRESET:
			dev->maxreaders = MAX_READERS;
			free_buffer(&dev->writebuf);
			retval = alloc_buffer(&dev->writebuf, BUFFER_SIZE);
			wake_up_interruptible(&dev->writebuf.writeq);
			break;
		case DM510_IOCTMAXREADERS:
			dev->maxreaders = arg;
			break;
		case DM510_IOCTBUFFERSIZE:
			free_buffer(&dev->writebuf);
			retval = alloc_buffer(&dev->writebuf, arg);
			wake_up_interruptible(&dev->writebuf.writeq);
			break;
		case DM510_IOCQMAXREADERS:
			retval = dev->maxreaders;
			break;
		case DM510_IOCQBUFFERSIZE:
			retval = dev->writebuf.buffersize;
			break;
		default: /* should never happen, as we check at the start */
			retval = -ENOTTY;
			break;
	}

	mutex_unlock(&dev->writebuf.mutex);
	mutex_unlock(&dev->mutex);

	return retval;
}

module_init(dm510_init_module);
module_exit(dm510_cleanup_module);

MODULE_AUTHOR("...Jonathan Kilhof, Marcus Søndergaard, and Olivia Jespersen.");
MODULE_LICENSE("GPL");
