/* serialsnoop.c - Monitor a serial connection, using two serial ports. */


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <regex.h>
#include <string.h>
#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <time.h>
#include <ctype.h>

#include "version.h"

#define SS_TIME_FORMAT "%Y-%m-%dT%H:%M:%S"

/* Maximum number of write errors we will accept on one port. */
#define MAX_WRITE_ERRORS 10

/* Size of write buffers. */
#define BUFFER_SIZE 64

struct port {
	int number;
	char *name;
	int fd;
	unsigned int write_errors;
	struct termios t1;
	struct termios t2;
	unsigned int bytesin;
	unsigned int bytesout;
	char pre_sentinel[4];
	unsigned char buffer[BUFFER_SIZE];
	char post_sentinel[4];
};

static struct port port0;
static struct port port1;

static char *myname;
static int port_baud = B1200;
static char port_baud_string[8] = "1200";
static char port_parity = 'E';
static int port_bits = 7;
static int port_stopbits = 1;

enum outputformat { outTEXT, outXML };
static enum outputformat outputformat = outTEXT;


enum snoop_mode {
	SNOOPMODE_MONITOR,
	SNOOPMODE_PASSTHROUGH,
};

static enum snoop_mode snoop_mode = SNOOPMODE_MONITOR;

static int signal_pipe[2];

static struct timeval starttime;

static int flush_stdout = 0;
static int ssdebug = 0;


static void
usage(void)
{
	fprintf(stderr, "Usage: %s [options] port0 port1\n", myname);
	fprintf(stderr, "   -p|--port portparams\n");
	fprintf(stderr, "   -f|--format text|xml\n");
	fprintf(stderr, "   -M|--monitor (default: do not pass data between ports)\n");
	fprintf(stderr, "   -P|--passthrough (pass data between ports)\n");
	fprintf(stderr, "   -V|--version\n");
	fprintf(stderr, "   portparams defaults to 1200E71\n");
	exit(1);
}


static void
check_sentinels(struct port *p)
{
	if (p->pre_sentinel[0] != 0
	    || p->pre_sentinel[1] != 0
	    || p->pre_sentinel[2] != 0
	    || p->pre_sentinel[3] != 0
	    || p->post_sentinel[0] != 0
	    || p->post_sentinel[1] != 0
	    || p->post_sentinel[2] != 0
	    || p->post_sentinel[3] != 0) {
		fprintf(stderr, "SENTINEL ERROR on %s\n", p->name);
		fprintf(stderr, "  PRE : %02x %02x %02x %02x\n",
			p->pre_sentinel[0], p->pre_sentinel[1],
			p->pre_sentinel[2], p->pre_sentinel[3]);
		fprintf(stderr, "  POST: %02x %02x %02x %02x\n",
			p->post_sentinel[0], p->post_sentinel[1],
			p->post_sentinel[2], p->post_sentinel[3]);
		fprintf(stderr, "  BUFFER_SIZE=%d\n", BUFFER_SIZE);
		fprintf(stderr, "  bytesin  = %u (mod: %u)\n",
			p->bytesin, p->bytesin  % BUFFER_SIZE);
		fprintf(stderr, "  bytesout = %u (mod: %u)\n",
			p->bytesout, p->bytesout % BUFFER_SIZE);
		exit(99);
	}
}


static void
add_buffer_byte(struct port *p, unsigned char c)
{
	if ((p->bytesin - p->bytesout) >= BUFFER_SIZE) {
		fprintf(stderr, "%s: buffer overrun writing to %s\n", myname, p->name);
		exit(3);
	}
	p->buffer[p->bytesin % BUFFER_SIZE] = c;
	check_sentinels(p);
	p->bytesin ++;
}


static int
buffer_has_bytes(struct port *p)
{
	return p->bytesout < p->bytesin;
}


static unsigned char
get_buffer_byte(struct port *p)
{
	unsigned char c;

	if (p->bytesin <= p->bytesout) {
		fprintf(stderr, "%s: buffer underrun on %s\n", myname, p->name);
		exit(3);
	}
	c = p->buffer[p->bytesout % BUFFER_SIZE];
	p->bytesout ++;
	return c;
}


static unsigned char
peek_buffer_byte(struct port *p)
{
	if (!buffer_has_bytes(p)) {
		fprintf(stderr, "Trying to peek into empty buffer\n");
		exit(3);
	}
	return p->buffer[p->bytesout % BUFFER_SIZE];
}


static void
write_data(struct port *p)
{
	unsigned char c;
	int ret;

	if (! buffer_has_bytes(p)) {
		fprintf(stderr, "Trying to write from empty buffer\n");
		return;
	}

	c = peek_buffer_byte(p);
	ret = write(p->fd, &c, 1);
	switch (ret) {
	case -1:
		if (errno != EAGAIN) {
			fprintf(stderr, "%s: cannot write to %s: %s\n", myname, p->name,
				strerror(errno));
			p->write_errors ++;
			if (p->write_errors > MAX_WRITE_ERRORS) {
				fprintf(stderr, "%s: too many write errors on %s: goodbye\n", myname,
					p->name);
				exit(2);
			}
		}
		break;
	case 0:
		fprintf(stderr, "%s: 0 write to %s.  Should this happen?\n",
			myname, p->name);
		break;
	case 1:
		/* Successful write, remove the byte we just wrote. */
		get_buffer_byte(p);
		break;
	default:
		fprintf(stderr, "%s: %d returned from write to %s.  Why?\n",
			myname, ret, p->name);
		break;
	}
	fprintf(stderr, "   -V|--version\n");
	fprintf(stderr, "   portparams defaults to 1200E71\n");
	exit(1);
}


static void
port_params(char *params)
{
	static char *port_par_regex = "^([1-9][0-9]*)(N|E|O)?(7|8)?(1|2)?$";
	regex_t port_reg;
	int ret;

	ret = regcomp(&port_reg, port_par_regex, REG_ICASE|REG_EXTENDED);
	if (0 != ret) {
		static char buf[1024];
		regerror(ret, &port_reg, buf, 1024);
		fprintf(stderr, "%s: Cannot compile regex \"%s\": %s\n",
			myname, port_par_regex, buf);
		exit(2);
	}

	regmatch_t matches[5];
	ret = regexec(&port_reg, params, 5, matches, 0);
	if (0 != ret) {
		fprintf(stderr, "%s: \"%s\" is not a port params string.\n",
			myname, params);
		exit(2);
	}

	/* matches[0] will be filled in because the whole regex has matched.
	   So search for the other matches. */

	int start = matches[1].rm_so;
	int end   = matches[1].rm_eo;
	int length = end - start;
	//printf("speed start:%d end:%d length:%d\n", start, end, length);
	if (-1 == start || -1 == end) {
		fprintf(stderr, "%s: no speed specified\n", myname);
		exit(2);
	}
	if (3 == length  &&  0 == strncmp(params+start, "300", 3)) {
		port_baud = B300;
		strncpy(port_baud_string, "300", 8);
	} else if (4 == length  &&  0 == strncmp(params+start, "1200", 4)) {
		port_baud = B1200;
		strncpy(port_baud_string, "1200", 8);
	} else if (4 == length  &&  0 == strncmp(params+start, "2400", 4)) {
		port_baud = B2400;
		strncpy(port_baud_string, "2400", 8);
	} else if (4 == length  &&  0 == strncmp(params+start, "4800", 4)) {
		port_baud = B4800;
		strncpy(port_baud_string, "4800", 8);
	} else if (4 == length  &&  0 == strncmp(params+start, "9600", 4)) {
		port_baud = B9600;
		strncpy(port_baud_string, "9600", 8);
	} else if (5 == length  &&  0 == strncmp(params+start, "19200", 4)) {
		port_baud = B19200;
		strncpy(port_baud_string, "19200", 8);
	} else if (5 == length  &&  0 == strncmp(params+start, "38400", 4)) {
		port_baud = B38400;
		strncpy(port_baud_string, "38400", 8);
	} else {
		char buf[length+1];
		strncpy(buf, params+start, length);
		buf[length] = '\0';
		fprintf(stderr, "%s: unsupported baud rate: %s\n", myname, buf);
		exit(2);
	}

	/* Hmm, we might never see these error checks, because they won't get
	   matched by the regex above... */

	if (-1 == matches[2].rm_so) {
		port_parity = 'E';
	} else {
		switch (params[matches[2].rm_so]) {
		case 'n': case 'N':
			port_parity = 'N';
			break;
		case 'e': case 'E':
			port_parity = 'E';
			break;
		case 'o': case 'O':
			port_parity = 'O';
			break;
		default:
			fprintf(stderr,
				"%s: Strange parity character %c (0x%02x)\n",
				myname, params[matches[2].rm_so],
				params[matches[2].rm_so]);
			exit(2);
		}
	}

	if (-1 == matches[3].rm_so) {
		port_bits = 7;
	} else if ('7' == params[matches[3].rm_so]) {
		port_bits = 7;
	} else if ('8' == params[matches[3].rm_so]) {
		port_bits = 8;
	} else {
		fprintf(stderr, "%s: Strange data bits character %c (0x%02x)\n",
			myname, params[matches[3].rm_so],
			params[matches[3].rm_so]);
		exit(2);
	}

	if (-1 == matches[4].rm_so) {
		port_stopbits = 1;
	} else if ('1' == params[matches[4].rm_so]) {
		port_stopbits = 1;
	} else if ('2' == params[matches[4].rm_so]) {
		port_stopbits = 2;
	} else {
		fprintf(stderr, "%s: Strange stop bits character %c (0x%02x)\n",
			myname, params[matches[4].rm_so],
			params[matches[4].rm_so]);
		exit(2);
	}
}


static void
open_ports(void)
{
	int open_mode;

	if (snoop_mode == SNOOPMODE_PASSTHROUGH)
		open_mode = O_RDWR;
	else if (snoop_mode == SNOOPMODE_MONITOR)
		open_mode = O_RDONLY;
	else {
		fprintf(stderr, "Unknown snoop_mode: %d\n", snoop_mode);
		exit(1);
	}
	if (ssdebug) fprintf(stderr, "Opening %s\n", port0.name);
	port0.fd = open(port0.name, open_mode|O_NONBLOCK|O_NOCTTY);
	if (-1 == port0.fd) {
		fprintf(stderr, "%s: cannot open %s: %s\n", myname, port0.name,
			strerror(errno));
		exit(2);
	}
	if (ssdebug) fprintf(stderr, "Opening %s\n", port1.name);
	port1.fd = open(port1.name, open_mode|O_NONBLOCK|O_NOCTTY);
	if (-1 == port1.fd) {
		fprintf(stderr, "%s: cannot open %s: %s\n", myname, port1.name,
			strerror(errno));
		exit(2);
	}
}


static void
setup_port(struct port *p)
{
	int ret;

	cfmakeraw(&p->t1);
	ret = cfsetispeed(&p->t1, port_baud);
	if (-1 == ret) {
		fprintf(stderr, "%s: cannot set %s speed to %s: %s\n",
			myname, p->name, port_baud_string, strerror(errno));
		exit(2);
	}

	if ('E'==port_parity) {
		p->t1.c_cflag |= PARENB;
		p->t1.c_cflag &= ~PARODD;
	}
	else if ('O'==port_parity) {
		p->t1.c_cflag |= PARENB;
		p->t1.c_cflag |= PARODD;
	}
	else {
		p->t1.c_cflag &= ~PARENB;
	}

	p->t1.c_cflag &= ~CSIZE;
	if (7==port_bits) {
		p->t1.c_cflag |= CS7;
	} else if (8==port_bits) {
		p->t1.c_cflag |= CS8;
	}

	/* no flow control */
	p->t1.c_cflag &= ~CRTSCTS;

	ret = tcsetattr(p->fd, TCSANOW, &p->t1);
	if (-1 == ret) {
		fprintf(stderr, "%s: cannot set port parameters for %s: %s\n",
			myname, p->name, strerror(errno));
		exit(2);
	}

	p->bytesin  = 0;
	p->bytesout = 0;

	p->pre_sentinel[0] = p->pre_sentinel[1]
		= p->pre_sentinel[2] = p->pre_sentinel[3] = 0;
	p->post_sentinel[0] = p->post_sentinel[1]
		= p->post_sentinel[2] = p->post_sentinel[3] = 0;

	p->write_errors = 0;
}


static void
setup_ports(void)
{
	setup_port(&port0);
	setup_port(&port1);
}


static void
print_trailer(void)
{
	switch (outputformat) {
	case outTEXT:
		break;
	case outXML:
		printf(" </data>\n</capture>\n");
	}
}


static void
print_byte(struct port *p, unsigned char c)
{
	struct timeval t;
	struct tm *tm;
	struct timeval tdiff;
	char timestring[200];

	gettimeofday(&t, 0);
	timersub(&t, &starttime, &tdiff);
	tm = gmtime(&(t.tv_sec));
	strftime(timestring, 199, SS_TIME_FORMAT, tm);

	/* The output here should be done in a single printf, to try to make it
	   a single write. */
	switch (outputformat) {
	case outTEXT:
		if (isprint(c))
			printf("%d %s %d.%06d 0x%02x %c\n", p->number,
			       timestring, (int)tdiff.tv_sec,
			       (int)tdiff.tv_usec, c, c);
		else
			printf("%d %s %d.%06d 0x%02x\n", p->number,
			       timestring, (int)tdiff.tv_sec,
			       (int)tdiff.tv_usec, c);
		break;
	case outXML:
		if (isprint(c))
			printf("  <byte line='%d' time='%d.%06d' "
			       "value='0x%02x' ascii='%c' />\n",
			       p->number, (int)tdiff.tv_sec,
			       (int)tdiff.tv_usec, c, c);
		else
			printf("  <byte line='%d' time='%d.%06d' "
			       "value='0x%02x' />\n",
			       p->number, (int)tdiff.tv_sec,
			       (int)tdiff.tv_usec, c);
		break;
	}
	fflush(stdout);
}


static void
read_data(struct port *p)
{
	unsigned char c;
	int nread;

	//printf("Reading from %s\n", p->name);
	while (1) {
		nread = read(p->fd, &c, 1);
		if (0 == nread) {
			fprintf(stderr, "EOF on %s\n", p->name);
			exit(0);
		} else if (1 == nread) {
			if (snoop_mode == SNOOPMODE_PASSTHROUGH)
				add_buffer_byte(p, c);
			print_byte(p, c);
		} else if (-1 == nread) {
			if (EWOULDBLOCK == errno) {
				break;
			} else {
				fprintf(stderr, "Error reading %s: %s\n",
					p->name, strerror(errno));
				exit(2);
			}
		} else {
			fprintf(stderr, "Strange return from reading %s: %d\n",
				p->name, nread);
		}
	}
}


static void
print_header(void)
{
	char timestring[200];
	struct tm *tm;

	tm = gmtime(&(starttime.tv_sec));
	strftime(timestring, 199, SS_TIME_FORMAT, tm);

	switch (outputformat) {
	case outTEXT:
		printf("port 0: %s\n", port0.name);
		printf("port 1: %s\n", port1.name);
		printf("start time: %s.%06d\n", timestring,
		       (int)starttime.tv_usec);
		printf("Port parameters: %s%c%d%d\n", port_baud_string, port_parity, port_bits,
		       port_stopbits);
		break;
	case outXML:
		printf("<?xml version='1.0' encoding='UTF-8' ?>\n"
		       "<capture>\n"
		       " <header>\n"
		       "  <starttime>%d.%06d</starttime>\n"
		       "  <port><id>0</id><name>%s</name></port>\n"
		       "  <port><id>1</id><name>%s</name></port>\n"
		       "  <parameters>%s%c%d%d</parameters>\n"
		       " </header>\n"
		       " <data>\n",
		       (int)starttime.tv_sec, (int)starttime.tv_usec,
		       port0.name, port1.name,
		       port_baud_string, port_parity, port_bits, port_stopbits);
		break;
	}
	fflush(stdout);
}


static void
mainloop(void)
{
	fd_set rfds;
	fd_set wfds;
	fd_set xfds;
	struct timeval timeout;
	int ret;
	int selecterr;
	int maxfdp1;

	while (1) {

		FD_ZERO(&rfds);
		FD_ZERO(&wfds);
		FD_ZERO(&xfds);
		maxfdp1 = 0;
		FD_SET(port0.fd, &rfds);
		if (port0.fd >= maxfdp1)
			maxfdp1 = port0.fd + 1;
		FD_SET(port1.fd, &rfds);
		if (port1.fd >= maxfdp1)
			maxfdp1 = port1.fd + 1;
		if (snoop_mode == SNOOPMODE_PASSTHROUGH) {
			if (port0.bytesout < port0.bytesin) {
				FD_SET(port0.fd, &wfds);
			}
			if (port1.bytesout < port1.bytesin) {
				FD_SET(port1.fd, &wfds);
			}
		}
		FD_SET(signal_pipe[0], &rfds);
		if (signal_pipe[0] >= maxfdp1)
			maxfdp1 = signal_pipe[0] + 1;
		timeout.tv_sec = 0;
		timeout.tv_usec = 10000;

		ret = select(maxfdp1, &rfds, &wfds, &xfds, &timeout);
		selecterr = errno;
		switch (ret) {
		case -1:
			switch (selecterr) {
			case EINTR:
				/* A question here: after the signal handler
				   writes to the pipe, is the pipe always ready
				   for reading when we return EINTR from
				   select()?  It seems to work in a couple of
				   tests, but is this guaranteed?  */
				if (FD_ISSET(signal_pipe[0], &rfds)) {
					print_trailer();
					exit(0);
				}
			default:
				fprintf(stderr, "%s: error in select(): %s\n",
					myname, strerror(selecterr));
				exit(2);
				break;
			}
		default:
			/* Handle file descriptors. */
			if (FD_ISSET(port0.fd, &rfds)) {
				read_data(&port0);
			}
			if (FD_ISSET(port1.fd, &rfds)) {
				read_data(&port1);
			}
			if (snoop_mode == SNOOPMODE_PASSTHROUGH) {
				if (FD_ISSET(port0.fd, &wfds)) {
					write_data(&port0);
				}
				if (FD_ISSET(port1.fd, &wfds)) {
					write_data(&port1);
				}
			}
			/*FALLTHROUGH*/
		case 0:
			/* Read serial port control lines. */
			break;
		}
	}
}


static void
sigint_handler(int signum)
{
	write(signal_pipe[1], "i", 1);
}

static void
setup_signal_handler(void)
{
	int ret;

	ret = pipe(signal_pipe);
	if (-1 ==ret) {
		fprintf(stderr, "%s: Cannot create pipe: %s\n",
			myname, strerror(errno));
		exit(2);
	}
	signal(SIGINT, sigint_handler);
}


static const char *opts = "dp:f:MPV";
static const struct option longopts[] = {
	{ "debug",  no_argument,       NULL, 'd' },
	{ "flush",  no_argument,       NULL, 'F' },
	{ "format", required_argument, NULL, 'f' },
	{ "param",  required_argument, NULL, 'p' },
	{ "monitor", no_argument,      NULL, 'M' },
	{ "passthrough", no_argument,  NULL, 'P' },
	{ "version",no_argument,       NULL, 'V' },
	{ 0, 0, 0, 0}
};


int
main(int argc, char **argv)
{

	myname = argv[0];

	while (1) {
		int c = getopt_long(argc, argv, opts, longopts, 0);
		if (-1==c)
			break;
		switch (c) {
		case 'd':
			ssdebug = 1;
			break;
		case 'f':
			if (!strcmp(optarg, "xml"))
				outputformat = outXML;
			else if (!strcmp(optarg, "text"))
				outputformat = outTEXT;
			else
				usage();
			break;
		case 'F':
			flush_stdout = 1;
			break;
		case 'M':
			snoop_mode = SNOOPMODE_MONITOR;
			break;
		case 'p':
			port_params(optarg);
			break;
		case 'P':
			snoop_mode = SNOOPMODE_PASSTHROUGH;
			break;
		case 'V':
			printf("serialsnoop " SERIALSNOOP_VERSION_STR "\n");
			exit(0);
		default:
		case '?':
			usage();
		break;
		}
	}

	if (argc-optind != 2) {
		usage();
	}
	port0.number = 0;
	port0.name = argv[optind];
	port1.number = 1;
	port1.name = argv[optind+1];

	open_ports();
	setup_ports();
	gettimeofday(&starttime, 0);
	print_header();
	setup_signal_handler();
	mainloop();
	return 0;
}
