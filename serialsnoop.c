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


struct port {
	int number;
	char *name;
	int fd;
	struct termios t1;
	struct termios t2;
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
	if (ssdebug) fprintf(stderr, "Opening %s\n", port0.name);
	port0.fd = open(port0.name, O_RDONLY|O_NONBLOCK|O_NOCTTY);
	if (-1 == port0.fd) {
		fprintf(stderr, "%s: cannot open %s: %s\n", myname, port0.name,
			strerror(errno));
		exit(2);
	}
	if (ssdebug) fprintf(stderr, "Opening %s\n", port1.name);
	port1.fd = open(port1.name, O_RDONLY|O_NONBLOCK|O_NOCTTY);
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
}


static void
setup_ports(void)
{
	setup_port(&port0);
	setup_port(&port1);
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
			printf("  <byte line='%d' time='%d.%06d' value='0x%02x' ascii='%c' />\n",
			       p->number, (int)tdiff.tv_sec,
			       (int)tdiff.tv_usec, c, c);
		else
			printf("  <byte line='%d' time='%d.%06d' value='0x%02x' />\n", p->number,
			       (int)tdiff.tv_sec, (int)tdiff.tv_usec, c);
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
		printf("Port parameters: %s%c%d%d\n",
		       port_baud_string, port_parity, port_bits, port_stopbits);
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
print_trailer(void)
{
	switch (outputformat) {
	case outTEXT:
		break;
	case outXML:
		printf(" </data>\n</capture>\n");
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
		if (port0.fd >= maxfdp1) maxfdp1 = port0.fd + 1;
		FD_SET(port1.fd, &rfds);
		if (port1.fd >= maxfdp1) maxfdp1 = port1.fd + 1;
		FD_SET(signal_pipe[0], &rfds);
		if (signal_pipe[0] >= maxfdp1) maxfdp1 = signal_pipe[0] + 1;
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


static const char *opts = "dp:f:V";
static const struct option longopts[] = {
	{ "debug",  no_argument,       NULL, 'd' },
	{ "flush",  no_argument,       NULL, 'F' },
	{ "format", required_argument, NULL, 'f' },
	{ "param",  required_argument, NULL, 'p' },
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
		case 'p':
			port_params(optarg);
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
