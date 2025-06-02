#include <cstdio>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

bool setup_tty(int serial);
bool write_filename(int serial, char *fname);
bool print_to_file(int serial, int local_fd, char *fname);

int main(int argc, char **argv) {
    if (argc == 1) {
        printf("Please supply a serial port to communicate with\n");
        return -1;
    }

    int serial = open(argv[1], O_RDWR);
    if (serial < 0) {
        printf("Failed to open serial port: %d (%s)\n", errno, strerror(errno));
        return -1;
    }

    if (!setup_tty(serial)) {
        return -1;
    }

    printf("SD Interface Program\nSuccessfully opened flight computer serial port: %s\nType help or ? for help\n", argv[1]);

    char cmd[256];
    char fname[256];
    char local_fname[256];
    char res[4096];

    while (true) {
        (void)scanf("%s", cmd);

        if (strcmp(cmd, "ls") == 0) {
            if (write(serial, "L", 1) == -1) {
                printf("Failed to write to serial port: %d (%s)\n", errno,
                       strerror(errno));
                return -1;
            }

            ssize_t n;

            while ((n = read(serial, res, sizeof(res))) != 0) {
                for (size_t i = 0; i < n; i++) {
                    if (res[i] == '\0') {
                        printf("\n");
                    } else {
                        printf("%c", res[i]);
                    }
                }
            }
            printf("\n");
        } else if (strcmp(cmd, "cat") == 0) {
            (void)scanf("%s", fname);

            if (!print_to_file(serial, STDOUT_FILENO, fname)) {
                return -1;
            }
            printf("\n");
        } else if (strcmp(cmd, "save") == 0) {
            size_t n = scanf("%s%*[ \t]%s", fname, local_fname);
            int fd;
            if (n == 2) {
                fd = open(local_fname, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
            } else {
                fd = open(fname, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
            }

            if (!print_to_file(serial, fd, fname)) {
                return -1;
            }

            close(fd);

            printf("DONE\n");
        } else if (strcmp(cmd, "rm") == 0) {
            (void)scanf("%s", fname);

            if (write(serial, "R", 1) == -1) {
                return -1;
            }

            if (!write_filename(serial, fname)) {
                return -1;
            }
        } else if (strcmp(cmd, "rmall") == 0) {
            if (write(serial, "C", 1) == -1) {
                return -1;
            }
        } else if (strcmp(cmd, "format") == 0) {
            if (write(serial, "F", 1) == -1) {
                return -1;
            }

            while (read(serial, res, 1) == 0) {
                printf(".");
                usleep(500 * 1000);
            }
            printf("DONE\n");
        } else if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
            printf("Available commands: \n"
                   "ls \t List all files\n"
                   "cat <file> \t Print the contents of <file>\n"
                   "save <remote_file> [<local_file>] \t Save the contents of "
                   "<remote_file> in <local_file>, which has the same name as "
                   "<remote_file> by default\n"
                   "rm <file> \t Delete <file> from the SD card\n"
                   "rmall \t Delete all files on the SD card\n"
                   "format \t Reformat the SD card. All data will be lost\n");
        } else {
            printf("Invalid command: `%s` (type help or ? for help)\n", cmd);
        }
    }
}

bool setup_tty(int serial) {
    struct termios tty;

    if (tcgetattr(serial, &tty) != 0) {
        printf("Failed to read serial port settings: %d (%s)\n", errno,
               strerror(errno));
        return false;
    }

    tty.c_cflag &= ~PARENB; // No parity bit
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8 Data bits
    tty.c_cflag &= ~CRTSCTS; // No hardware flow control
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON; // Disable line-by-line mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                     ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes
                           // (e.g. newline chars)
    tty.c_oflag &=
        ~ONLCR; // Prevent conversion of newline to carriage return/line feed

    tty.c_cc[VTIME] = 1; // Wait for up to 100ms
    tty.c_cc[VMIN] = 0;

    cfsetspeed(&tty, B230400);

    if (tcsetattr(serial, TCSANOW, &tty) != 0) {
        printf("Failed to set serial port settings: %d (%s)\n", errno,
               strerror(errno));
        return false;
    }

    return true;
}

bool write_filename(int serial, char *fname) {
    char b[1] = {(char)strlen(fname)};
    if (write(serial, b, 1) == -1) {
        return false;
    }

    if (write(serial, fname, strlen(fname)) == -1) {
        return false;
    }

    return true;
}

bool print_to_file(int serial, int local_fd, char *fname) {
    char contents[4096];

    if (write(serial, "P", 1) == -1) {
        return false;
    }

    if (!write_filename(serial, fname)) {
        return false;
    }

    ssize_t n;

    while ((n = read(serial, contents, sizeof(contents))) != 0) {
        contents[n] = '\0';
        if (write(local_fd, contents, n) == -1) {
            return false;
        }
    }

    return true;
}
